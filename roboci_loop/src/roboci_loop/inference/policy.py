"""OpenPI 推理服务与 roboci_loop 消息之间的适配。"""

from dataclasses import dataclass
import time
from typing import Literal, Mapping, Sequence

import numpy as np

from ..common import ARM_DATA, CAM_MSG, END_EFFECTOR_DATA, JOINT_DATA, ROBOT_DATA


@dataclass(frozen=True)
class JointGroup:
    """模型状态/动作向量中的一个连续设备分组。"""

    kind: Literal["arm", "end_effector"]
    name: str
    size: int

    def __post_init__(self) -> None:
        if self.kind not in ("arm", "end_effector"):
            raise ValueError("kind 必须为 arm 或 end_effector")
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("name 必须为非空字符串")
        if isinstance(self.size, bool) or not isinstance(self.size, int) or self.size <= 0:
            raise ValueError("size 必须为正整数")


class InferenceLayout:
    """显式定义模型关节顺序和图像 key，避免依赖字典枚举顺序。"""

    def __init__(self, groups: Sequence[JointGroup], cameras: Mapping[str, str]) -> None:
        self.groups = tuple(groups)
        self.cameras = dict(cameras)  # 模型输入 key -> camera_name
        if not self.groups:
            raise ValueError("groups 不能为空")
        identities = [(group.kind, group.name) for group in self.groups]
        if len(set(identities)) != len(identities):
            raise ValueError("groups 包含重复设备")
        if any(not isinstance(key, str) or not key.strip() or
               not isinstance(name, str) or not name.strip()
               for key, name in self.cameras.items()):
            raise ValueError("相机模型 key 和 camera_name 必须为非空字符串")

    @property
    def action_dim(self) -> int:
        return sum(group.size for group in self.groups)

    @staticmethod
    def _items(data: ROBOT_DATA) -> dict[tuple[str, str], object]:
        items = {("arm", item.arm_name): item for item in data.arms}
        items.update({
            ("end_effector", item.end_effector_name): item
            for item in data.end_effectors
        })
        return items

    def state(self, data: ROBOT_DATA) -> np.ndarray:
        """将完整机器人反馈编码为模型的一维 float32 状态。"""
        items, parts = self._items(data), []
        for group in self.groups:
            item = items.get((group.kind, group.name))
            if item is None:
                raise ValueError(f"机器人反馈缺少 {group.kind}/{group.name}")
            values = np.asarray(item.joints.position)
            if values.shape != (group.size,) or not np.isfinite(values).all():
                raise ValueError(
                    f"{group.kind}/{group.name} 反馈应为 {group.size} 个有限关节角"
                )
            parts.append(values)
        return np.concatenate(parts).astype(np.float32, copy=False)

    def observation(self, robot: ROBOT_DATA, cameras: CAM_MSG, prompt: str) -> dict:
        """构造 OpenPI observation；图像为 RGB uint8 HxWx3。"""
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("prompt 必须为非空字符串")
        result = {"observation/state": self.state(robot), "prompt": prompt}
        for key, camera_name in self.cameras.items():
            frame = cameras.getFrame(camera_name)
            if frame is None or not frame.hasRGB():
                raise ValueError(f"相机 {camera_name} 缺少 RGB 图像")
            image = np.asarray(frame.rgb)
            if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
                raise ValueError(f"相机 {camera_name} RGB 必须为 uint8 HxWx3")
            result[key] = np.ascontiguousarray(image)
        return result

    def target(self, action, *, time_stamp: float | None = None) -> ROBOT_DATA:
        """将模型绝对关节动作解码为 RobotController 目标。"""
        values = np.asarray(action, dtype=np.float64)
        if values.shape != (self.action_dim,) or not np.isfinite(values).all():
            raise ValueError(f"action 必须为 {self.action_dim} 个有限值")
        stamp = time.time() if time_stamp is None else float(time_stamp)
        if not np.isfinite(stamp) or stamp <= 0:
            raise ValueError("time_stamp 必须为有限正数")
        result, offset = ROBOT_DATA(time_stamp=stamp), 0
        for group in self.groups:
            joints = JOINT_DATA(position=values[offset:offset + group.size])
            if group.kind == "arm":
                result.arms.append(
                    ARM_DATA(group.name, time_stamp=stamp, joints=joints)
                )
            else:
                result.end_effectors.append(
                    END_EFFECTOR_DATA(group.name, time_stamp=stamp, joints=joints)
                )
            offset += group.size
        return result


@dataclass(frozen=True)
class OpenPIConfig:
    url: str = "ws://127.0.0.1:8000"
    action_horizon: int = 30
    broker: Literal["serial", "rtg"] = "serial"
    control_hz: float = 10.0
    trigger_fraction: float = 0.5
    guidance_steps: int = 6
    api_key: str | None = None

    def __post_init__(self) -> None:
        if not self.url.startswith(("ws://", "wss://")):
            raise ValueError("url 必须以 ws:// 或 wss:// 开头")
        if (isinstance(self.action_horizon, bool) or
                not isinstance(self.action_horizon, int) or self.action_horizon <= 0):
            raise ValueError("action_horizon 必须为正整数")
        if self.broker not in ("serial", "rtg"):
            raise ValueError("broker 必须为 serial 或 rtg")
        if not np.isfinite(self.control_hz) or self.control_hz <= 0:
            raise ValueError("control_hz 必须为有限正数")
        if not np.isfinite(self.trigger_fraction) or not 0 < self.trigger_fraction < 1:
            raise ValueError("trigger_fraction 必须在 (0, 1) 内")
        if (isinstance(self.guidance_steps, bool) or
                not isinstance(self.guidance_steps, int) or self.guidance_steps < 0):
            raise ValueError("guidance_steps 必须为非负整数")


class _ValidatedPolicy:
    """在官方 broker 切片前校验模型返回的完整动作块。"""

    def __init__(self, policy, horizon: int, action_dim: int) -> None:
        self._policy = policy
        self._shape = (horizon, action_dim)

    def infer(self, observation: dict) -> dict:
        result = self._policy.infer(observation)
        if not isinstance(result, dict) or "actions" not in result:
            raise ValueError("模型响应必须为包含 actions 的字典")
        actions = np.asarray(result["actions"], dtype=np.float32)
        if actions.shape != self._shape or not np.isfinite(actions).all():
            raise ValueError(
                f"完整 actions 必须为有限值数组 {self._shape}，实际为 {actions.shape}"
            )
        validated = dict(result)
        validated["actions"] = actions
        return validated

    def reset(self) -> None:
        self._policy.reset()


class OpenPIPolicy:
    """组合官方 WebSocket policy 与 action broker，并校验单步动作。"""

    def __init__(self, config: OpenPIConfig, action_dim: int) -> None:
        from openpi_client.action_chunk_broker import ActionChunkBroker
        from openpi_client.rtg_action_broker import RTGActionBroker
        from openpi_client.websocket_client_policy import WebsocketClientPolicy

        if isinstance(action_dim, bool) or not isinstance(action_dim, int) or action_dim <= 0:
            raise ValueError("action_dim 必须为正整数")
        self.config, self.action_dim = config, action_dim
        client = WebsocketClientPolicy(host=config.url, api_key=config.api_key)
        self.metadata = client.get_server_metadata()
        self._client = client
        validated = _ValidatedPolicy(client, config.action_horizon, action_dim)
        if config.broker == "serial":
            self._broker = ActionChunkBroker(validated, config.action_horizon)
        else:
            self._broker = RTGActionBroker(
                validated, config.action_horizon, control_hz=config.control_hz,
                trigger_fraction=config.trigger_fraction,
                guidance_steps=config.guidance_steps,
            )

    def infer(self, observation: dict) -> np.ndarray:
        result = self._broker.infer(observation)
        if "actions" not in result:
            raise ValueError("模型响应缺少 actions")
        action = np.asarray(result["actions"], dtype=np.float64)
        if action.shape != (self.action_dim,) or not np.isfinite(action).all():
            raise ValueError(
                f"单步 actions 必须为 {self.action_dim} 个有限值，实际为 {action.shape}"
            )
        return action

    def reset(self) -> None:
        self._broker.reset()

    def close(self) -> None:
        close = getattr(self._broker, "close", None)
        if close is not None:
            close()
        socket = getattr(self._client, "_ws", None)
        if socket is not None:
            socket.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
