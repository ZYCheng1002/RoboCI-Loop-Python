from dataclasses import dataclass
from typing import Optional

from .msg_robot import POSE_DATA


@dataclass
class CONTROLLER_STATE:
    """单侧手柄状态"""

    pose: Optional[POSE_DATA] = None  # 手柄位姿, 位置单位 m, 四元数 (x,y,z,w)
    axis_x: float = 0.0  # 摇杆 X, 归一化范围 [-1, 1], 无量纲
    axis_y: float = 0.0  # 摇杆 Y, 归一化范围 [-1, 1], 无量纲
    trigger: float = 0.0  # 食指扳机, 归一化范围 [0, 1], 无量纲
    grip: float = 0.0  # 握持输入, 归一化范围 [0, 1], 数字按键用 0/1
    axis_click: bool = False  # 摇杆按钮
    primary_button: bool = False  # 主按键, 左 X / 右 A
    secondary_button: bool = False  # 次按键, 左 Y / 右 B
    menu_button: bool = False  # 菜单按键, 左菜单 / 右截图或录屏
    time_stamp: float = 0.0  # SDK 采样时间, 单位 s, 保留 SDK 时间基准


@dataclass
class HEAD_STATE:
    """头部状态"""

    pose: Optional[POSE_DATA] = None  # 头部位姿, 位置单位 m, 四元数 (x,y,z,w)
    status: int = 0  # SDK 原始跟踪状态, 保留原值
    hand_mode: int = 0  # 手部输入模式, 0 未启用 / 1 手柄 / 2 手势
    time_stamp: float = 0.0  # SDK 采样时间, 单位 s, 保留 SDK 时间基准


@dataclass
class TRACKER_STATE:
    """跟踪设备状态, 汇总头部及左右手柄输入"""

    tracker_name: str = ""  # 跟踪设备名称
    device_id: str = ""  # 设备标识
    time_stamp: float = 0.0  # SDK 时间戳, 单位 s, 保留 SDK 时间基准
    head: Optional[HEAD_STATE] = None  # 头部状态, 未提供时为空
    left: Optional[CONTROLLER_STATE] = None  # 左手柄状态, 未提供时为空
    right: Optional[CONTROLLER_STATE] = None  # 右手柄状态, 未提供时为空
