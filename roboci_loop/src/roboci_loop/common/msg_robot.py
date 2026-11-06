"""机器人通用数据：反馈与绝对目标复用，语义由调用接口决定。"""
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class JOINT_DATA:
    """关节数据；反馈为实测值，目标为绝对设定值，顺序与硬件接口一致。"""
    names: List[str] = field(default_factory=list)                                       # 关节名称列表
    position: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float64))  # rad
    velocity: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float64))  # rad/s
    effort: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float64))    # N·m

    def __post_init__(self) -> None:
        # 接受列表或数组输入，统一为独立的一维 float64 数组。
        for name in ("position", "velocity", "effort"):
            values = np.array(getattr(self, name), dtype=np.float64, copy=True)
            if values.ndim != 1:
                raise ValueError(f"{name} 必须为一维数组")
            setattr(self, name, values)


@dataclass
class POSE_DATA:
    """child_frame_id 在 frame_id 坐标系中的位姿, 末端位姿"""
    frame_id: str = ""
    child_frame_id: str = ""
    # 末端位置
    position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    # 末端姿态 (x,y,z,w)
    orientation: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    )


@dataclass
class ARM_DATA:
    """机械臂反馈或目标数据, 反馈可同时包含关节与末端位姿, 作为目标时, 由执行接口明确选择控制方式"""
    arm_name: str = ""                                      # 机械臂名称
    time_stamp: float = 0.0                                 # 反馈采样或目标生成时间, 单位s, 0 表示未提供
    joints: JOINT_DATA = field(default_factory=JOINT_DATA)  # 关节角度
    end_pose: Optional[POSE_DATA] = None                    # 末端位姿


@dataclass
class END_EFFECTOR_DATA:
    """末端执行器反馈或绝对关节目标；适用于任意关节数的手和夹爪。"""
    end_effector_name: str = ""
    time_stamp: float = 0.0
    joints: JOINT_DATA = field(default_factory=JOINT_DATA)


@dataclass
class ROBOT_DATA:
    """整机反馈或目标。

    time_stamp 使用主机 Unix 时间（s）：反馈为汇总时间，目标为生成时间；
    各设备的时间戳可保留独立采样时间。0 表示未提供。
    目标只包含需要更新的设备；缺失设备不更新，空消息不表示停止或保持。
    动作来源、控制权和目标有效性由控制模块管理，超时策略由调用方决定。
    控制器反馈为增量消息：列表为空表示该设备没有新的未消费反馈。
    """
    time_stamp: float = 0.0
    arms: List[ARM_DATA] = field(default_factory=list)
    end_effectors: List[END_EFFECTOR_DATA] = field(default_factory=list)
