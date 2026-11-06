"""输入位姿相对运动到机器人末端目标的映射，不处理按键或控制权。"""
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
import pinocchio as pin

from ..common import POSE_DATA
from ..kinematics import PlacoIK
from ..kinematics.geometry import matrix_to_pose, pose_to_matrix


@dataclass(frozen=True)
class PoseTeleopConfig:
    position_scale: float = 1.
    cartesian_filter_alpha: float = .1
    control_dt: float = .02
    control_orientation: bool = True
    input_to_world: np.ndarray = field(default_factory=lambda: np.eye(3))

    def __post_init__(self):
        if not np.isfinite(self.position_scale) or self.position_scale <= 0:
            raise ValueError("位移比例必须为有限正数")
        if not np.isfinite(self.control_dt) or self.control_dt <= 0:
            raise ValueError("控制周期必须为有限正数")
        if not 0 < self.cartesian_filter_alpha <= 1:
            raise ValueError("滤波系数必须在 (0, 1] 内")
        rotation = np.asarray(self.input_to_world, dtype=float)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3))
                or not np.isclose(np.linalg.det(rotation), 1)):
            raise ValueError("input_to_world 必须为三维旋转矩阵")


@dataclass
class _Reference:
    input_pose: np.ndarray
    robot_pose: np.ndarray
    delta: np.ndarray


class PoseTeleop:
    """单个末端的相对位姿跟踪；调用方决定何时 start、compute、stop。

    每次 start 重新捕获参考并初始化 IK；compute 每次推进一次 QP。
    调用方负责新帧筛选、超时、按键及硬件控制，同一 IK 实例不应并发使用。
    """

    def __init__(self, ik: PlacoIK, *, reference_joints: Optional[Sequence[float]] = None,
                 config: Optional[PoseTeleopConfig] = None) -> None:
        self.ik = ik
        self.config = config if config is not None else PoseTeleopConfig()
        self._rotation = np.array(self.config.input_to_world, dtype=float, copy=True)
        self._rest = None if reference_joints is None else np.array(reference_joints, dtype=float, copy=True)
        self._reference: Optional[_Reference] = None

    @property
    def is_running(self) -> bool:
        return self._reference is not None

    def start(self, joints: Sequence[float], input_pose: POSE_DATA) -> None:
        """从当前实测关节角和输入位姿建立参考；无效输入抛出异常。"""
        self.stop()
        input_matrix = pose_to_matrix(input_pose)
        robot_matrix = pose_to_matrix(self.ik.forward(joints))
        self.ik.begin_tracking(joints, dt=self.config.control_dt, reference_joints=self._rest)
        self._reference = _Reference(input_matrix, robot_matrix, np.zeros(6))

    def compute(self, input_pose: POSE_DATA) -> Optional[np.ndarray]:
        """将输入位姿映射成一轮关节目标；未启动或 QP 失败返回 None。"""
        if self._reference is None:
            return None
        current = pose_to_matrix(input_pose)
        reference = self._reference
        delta = self.config.position_scale * self._rotation @ (current[:3, 3] - reference.input_pose[:3, 3])
        relative = self._rotation @ current[:3, :3] @ reference.input_pose[:3, :3].T @ self._rotation.T
        rotation_delta = pin.log3(relative) if self.config.control_orientation else np.zeros(3)
        reference.delta += self.config.cartesian_filter_alpha * (np.r_[delta, rotation_delta] - reference.delta)
        target_matrix = reference.robot_pose.copy()
        target_matrix[:3, 3] += reference.delta[:3]
        target_matrix[:3, :3] = pin.exp3(reference.delta[3:]) @ reference.robot_pose[:3, :3]
        target = matrix_to_pose(target_matrix, self.ik.config.world_frame, self.ik.config.end_frame)
        joints = self.ik.track(target)
        return None if joints is None else np.array(joints, dtype=float, copy=True)

    def stop(self) -> None:
        self._reference = None
