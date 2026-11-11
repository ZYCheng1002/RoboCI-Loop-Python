import importlib
import importlib.util
import math
import os
from pathlib import Path
from typing import Optional, Sequence
import time
import threading

import numpy as np

from ...common.msg_robot import END_EFFECTOR_DATA, JOINT_DATA
from .end_effector_base import EndEffectorBase, EndEffectorConfig


def _load_sdk(sdk_path):
    if sdk_path is None:
        try:
            return importlib.import_module("linked_hand")
        except ModuleNotFoundError as exc:
            if exc.name != "linked_hand":
                raise
        sdk_path = os.environ.get("ROBOCI_LINKED_HAND_SDK")
        if sdk_path is None:
            sdk_path = Path(__file__).resolve().parents[5] / "third_party" / "linked_hand_python"
    file = Path(sdk_path).expanduser().resolve() / "linked_hand.py"
    if not file.is_file():
        raise ImportError(
            "找不到 linked_hand SDK。请安装 linked_hand 模块，或通过 sdk_path/"
            "ROBOCI_LINKED_HAND_SDK 指定完整 linked_hand_python 目录；"
            f"当前检查路径：{file.parent}"
        )
    spec = importlib.util.spec_from_file_location("_roboci_linked_hand", file)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法从 {file} 创建 linked_hand 模块")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class HandLG603(EndEffectorBase):
    """LG603，默认 GCAN / ECanVci，六关节顺序见 JOINT_NAMES。

    六关节逐个请求，不是同时采样或原子控制。超时后应停止使用并关闭连接。
    start 不使能、不校准，自动启动后台读取；read 非阻塞消费最新完整反馈。
    控制同步发送，与后台读取并行；enable 暂停读取后执行。
    start/stop 不应与上层读写或 enable 并发，关闭前应结束上层调用线程。
    """

    JOINT_NAMES = ("thumb_rotation", "index", "middle", "ring", "little", "thumb_flex")

    def __init__(
        self, config: Optional[EndEffectorConfig] = None, *,
        device_index: int = 0, channel: int = 0, device_can_id: int = 0x65,
        bitrate: int = 1000000, timeout_ms: int = 2000,
        sdk_path: Optional[str] = None,
    ) -> None:
        super().__init__(config)
        for name, value, low, high in (
            ("device_index", device_index, 0, 2**32 - 1),
            ("channel", channel, 0, 1), ("device_can_id", device_can_id, 1, 255),
            ("bitrate", bitrate, 1, 2**32 - 1), ("timeout_ms", timeout_ms, 1, 2**31 - 1),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
                raise ValueError(f"{name} 必须为 {low}~{high} 的整数")
        self.device_index, self.channel = device_index, channel
        self.device_can_id, self.bitrate = device_can_id, bitrate
        self.timeout_ms, self.sdk_path = timeout_ms, sdk_path
        self._hand = None
        self._sdk = None
        self._fingers = ()
        self._limits = None
        self._state_lock = threading.Lock()
        self._read_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._stop_reading = threading.Event()
        self._reader = None
        self._latest = None
        self._error = None

    @property
    def is_running(self) -> bool:
        return self._hand is not None

    def start(self, *, background_read: bool = True) -> None:
        """打开连接；多通道共用设备时可先全部打开，再调用 start_feedback()。"""
        if self.is_running:
            return
        sdk = _load_sdk(self.sdk_path)
        fingers = tuple(getattr(sdk.Finger, name.upper()) for name in self.JOINT_NAMES)     # 
        limits = np.array([sdk.limits(finger) for finger in fingers])                       # 查询限位值
        hand = sdk.Hand(can_config=sdk.CanConfig.gcan(
            device_index=self.device_index, channel=self.channel,
            device_can_id=self.device_can_id, bitrate=self.bitrate, driver="ecanvci",
        ))
        try:
            hand.set_timeout_ms(self.timeout_ms)
        except Exception:
            hand.close()
            raise
        self._sdk, self._fingers, self._limits = sdk, fingers, limits
        self._hand = hand
        with self._state_lock:
            self._latest = None
            self._error = None
        self._stop_reading.clear()
        if background_read:
            self.start_feedback()

    def start_feedback(self) -> None:
        """启动后台反馈；应在共用 GCAN 设备的所有通道打开后调用。"""
        self._require_hand()
        if self._reader is not None:
            return
        # 状态读取线程(单独启动)
        self._reader = threading.Thread(
            target=self._read_loop, name=f"{self.config.end_effector_name}-feedback",
        )
        try:
            self._reader.start()
        except Exception:
            self._reader = None
            self.stop()
            raise

    def _require_hand(self):
        if self._hand is None:
            raise RuntimeError("灵巧手尚未连接，请先调用 start()")
        with self._state_lock:
            if self._error is not None:
                raise RuntimeError("灵巧手通信失败，请 stop() 后重新连接") from self._error
        return self._hand

    @property
    def joint_limits(self) -> np.ndarray:
        """六关节 [下限, 上限]，单位 rad，返回副本。"""
        self._require_hand()
        return np.deg2rad(self._limits.copy())

    def _record_error(self, exc):
        with self._state_lock:
            if self._error is None:
                self._error = exc
            self._latest = None
        self._stop_reading.set()

    def _raise_communication_error(self):
        with self._state_lock:
            error = self._error
        raise RuntimeError(
            f"{self.config.end_effector_name} CAN 通信失败 "
            f"(device={self.device_index}, channel={self.channel}, id={self.device_can_id:#x})；"
            "请 stop() 后重新连接"
        ) from error

    def _read_loop(self):
        try:
            while not self._stop_reading.is_set():
                with self._read_lock:
                    positions = []
                    for finger in self._fingers:
                        if self._stop_reading.is_set():
                            return
                        positions.append(math.radians(self._hand.get_position(finger)))
                    state = END_EFFECTOR_DATA(
                        end_effector_name=self.config.end_effector_name,
                        time_stamp=time.time(),
                        joints=JOINT_DATA(names=list(self.JOINT_NAMES), position=positions),
                    )
                with self._state_lock:
                    if not self._stop_reading.is_set():
                        self._latest = state
        except Exception as exc:
            self._record_error(exc)

    def read(self) -> Optional[END_EFFECTOR_DATA]:
        """消费最新一帧，无新反馈返回 None；时间戳为后台完成整帧读取的时间。

        未消费的旧帧会被覆盖；通信异常优先抛出，不返回故障前的缓存。
        """
        self._require_hand()
        with self._state_lock:
            if self._error is not None:
                raise RuntimeError("灵巧手通信失败，请 stop() 后重新连接") from self._error
            state, self._latest = self._latest, None
            return state

    def send_joint_positions(self, positions: Sequence[float], *, slow: bool = False) -> None:
        """发送位置；SDK 未提供独立慢速接口，slow 时仍需调用方限速。"""
        hand = self._require_hand()
        angles = np.asarray(positions, dtype=np.float64)
        if angles.shape != (6,) or not np.isfinite(angles).all():
            raise ValueError("positions 必须为 6 个有限关节角度（rad）")
        degrees = np.rad2deg(angles)
        # 启动/HOLD 会把实测角度作为保持目标；编码器在机械边界附近允许有
        # ±0.5° 残差（与 SDK benchmark 一致），随后裁剪到精确命令范围。
        tolerance_deg = 0.5
        if (np.any(degrees < self._limits[:, 0] - tolerance_deg)
                or np.any(degrees > self._limits[:, 1] + tolerance_deg)):
            raise ValueError(
                f"关节角度超出 SDK 范围：target_deg={degrees.tolist()}, "
                f"limits_deg={self._limits.tolist()}"
            )
        degrees = np.clip(degrees, self._limits[:, 0], self._limits[:, 1])
        with self._command_lock:
            try:
                for finger, value in zip(self._fingers, degrees):
                    self._require_hand()
                    hand.set_position(finger, float(value))
            except Exception as exc:
                self._record_error(exc)
                self._raise_communication_error()

    def enable(self, on: bool = True) -> None:
        """显式设置所有关节使能；六次调用依次执行，失败时可能部分完成。"""
        hand = self._require_hand()
        if not isinstance(on, bool):
            raise ValueError("on 必须为 bool")
        with self._command_lock, self._read_lock:
            try:
                for finger in self._fingers:
                    self._require_hand()
                    hand.enable(finger, on)
            except Exception as exc:
                self._record_error(exc)
                raise

    def stop_feedback(self) -> None:
        """停止并等待反馈线程；用于多设备按依赖顺序关闭，不关闭 CAN 句柄。"""
        self._stop_reading.set()
        if self._reader is not None:
            self._reader.join()
            self._reader = None

    def stop(self) -> None:
        self.stop_feedback()
        if self._hand is not None:
            self._hand.close()
            self._hand = None
        with self._state_lock:
            self._latest = None
