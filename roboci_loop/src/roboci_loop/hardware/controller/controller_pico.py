import importlib
from typing import Optional

import numpy as np

from ...common.msg_controller import CONTROLLER_STATE, HEAD_STATE, TRACKER_STATE
from ...common.msg_robot import POSE_DATA
from .controller_base import ControllerBase, ControllerConfig


class ControllerPico(ControllerBase):
    """XRoboToolkit PICO 输入，需先运行 PC Service 并连接头显。

    SDK 使用全局缓存，仅允许一个实例。不要在外部调用 SDK 的 init/close。
    保留 SDK 坐标轴，位置 m，四元数 xyzw；不执行机器人坐标系转换。
    各 getter 独立访问缓存，不保证头显、双手及按钮来自同一帧。
    无新时间戳时返回 None；SDK 不提供单侧在线状态，断开一侧后其缓存可能保留。
    """

    _owner = None

    def __init__(self, config: Optional[ControllerConfig] = None) -> None:
        super().__init__(config)
        self._sdk = None
        self._last_timestamp = 0

    @property
    def is_running(self) -> bool:
        return self._sdk is not None

    def start(self) -> None:
        if self.is_running:
            return
        if ControllerPico._owner is not None:
            raise RuntimeError("PICO SDK 已被另一个 ControllerPico 实例使用")
        try:
            sdk = importlib.import_module("xrobotoolkit_sdk")
        except ModuleNotFoundError as exc:
            if exc.name != "xrobotoolkit_sdk":
                raise
            raise ImportError("请安装 PICO SDK：python -m pip install xrobotoolkit-sdk") from exc
        sdk.init()
        try:
            # 忽略 SDK 上次会话留下的缓存，等待本次会话的时间戳更新。
            self._last_timestamp = sdk.get_time_stamp_ns()
        except Exception:
            sdk.close()
            raise
        self._sdk = sdk
        ControllerPico._owner = self

    @staticmethod
    def _pose(values, child: str) -> Optional[POSE_DATA]:
        values = np.asarray(values, dtype=np.float64)
        if values.shape != (7,) or not np.isfinite(values).all():
            return None
        norm = np.linalg.norm(values[3:])
        if norm < 1e-9:
            return None
        return POSE_DATA(
            frame_id="pico_world", child_frame_id=child,
            position=values[:3].copy(), orientation=values[3:].copy() / norm,
        )

    def _controller(self, side: str, stamp: float) -> CONTROLLER_STATE:
        sdk = self._sdk
        def get(field):
            return getattr(sdk, f"get_{side}_{field}")()
        axis = get("axis")
        primary, secondary = ("X", "Y") if side == "left" else ("A", "B")
        return CONTROLLER_STATE(
            pose=self._pose(get("controller_pose"), f"{side}_controller"),
            axis_x=axis[0], axis_y=axis[1], trigger=get("trigger"), grip=get("grip"),
            axis_click=get("axis_click"), menu_button=get("menu_button"),
            primary_button=getattr(sdk, f"get_{primary}_button")(),
            secondary_button=getattr(sdk, f"get_{secondary}_button")(), time_stamp=stamp,
        )

    def read(self) -> Optional[TRACKER_STATE]:
        if not self.is_running:
            raise RuntimeError("PICO 尚未启动，请先调用 start()")
        timestamp = self._sdk.get_time_stamp_ns()
        if timestamp <= 0 or timestamp == self._last_timestamp:
            return None
        stamp = timestamp / 1_000_000_000
        state = TRACKER_STATE(
            tracker_name=self.config.controller_name, time_stamp=stamp,
            head=HEAD_STATE(pose=self._pose(self._sdk.get_headset_pose(), "head"), time_stamp=stamp),
            left=self._controller("left", stamp), right=self._controller("right", stamp),
        )
        self._last_timestamp = timestamp
        return state

    def stop(self) -> None:
        if self.is_running:
            self._sdk.close()
            self._sdk = None
            ControllerPico._owner = None
            self._last_timestamp = 0
