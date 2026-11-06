"""基于 Placo 的固定基座机械臂 FK/IK，不执行硬件操作。"""
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import pinocchio as pin
import placo

from ..common import POSE_DATA
from .geometry import matrix_to_pose, pose_to_matrix


@dataclass(frozen=True)
class PlacoIKConfig:
    urdf_path: str                                              # 模型 URDF 路径，由调用方指定
    base_frame: str                                             # URDF 中固定在根部的机械臂基座坐标系
    end_frame: str                                              # 求解末端坐标系，不等同于实际工具 TCP
    joint_names: tuple[str, ...]                                # 与硬件一致的关节顺序，不限定关节数量或名称
    world_frame: str = "world"                                  # FK 输出和 IK 目标统一使用的参考系
    base_position: tuple[float, float, float] = (0., 0., 0.)    # 基座在世界系中的位置，m
    base_rpy_deg: tuple[float, float, float] = (0., 0., 0.)     # 基座安装角，度，Rz @ Ry @ Rx

    def __post_init__(self):
        for vector in (self.base_position, self.base_rpy_deg):
            if np.asarray(vector).shape != (3,) or not np.isfinite(vector).all():
                raise ValueError("基座位置和安装角必须为三个有限数值")


class PlacoIK:
    """固定基座 FK/IK：关节 rad、位置 m、四元数 xyzw。

    每次 track 推进一次 QP，失败返回 None；不保证单步到位，不检查碰撞。
    每臂独立实例，同一实例串行调用。
    """

    def __init__(self, config: PlacoIKConfig) -> None:
        self.config = config  # 模型与求解参数，不包含任何特定机器人的默认值
        path = Path(self.config.urdf_path).expanduser()
        self._robot = placo.RobotWrapper(str(path.resolve()), placo.Flags.ignore_collisions)  # 独立模型状态，不加载碰撞几何
        self.joint_names = self.config.joint_names  # 关节输入输出顺序
        if set(self.joint_names) != set(self._robot.joint_names()) or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names 必须完整列出模型的活动关节，且不能重复")
        self._robot.set_T_world_fbase(np.eye(4))
        self._robot.update_kinematics()
        base = self._robot.model.frames[self._robot.model.getFrameId(self.config.base_frame)]
        if base.parentJoint > 1:
            raise ValueError("base_frame 必须固定在模型根部，不能随活动关节运动")
        # 若 URDF 根部到 base_frame 存在固定偏移，也保证配置描述的是 base_frame 的安装位姿。
        model_base = self._robot.get_T_world_frame(self.config.base_frame)
        rotation = pin.rpy.rpyToMatrix(np.deg2rad(self.config.base_rpy_deg))
        world_base = pin.SE3(rotation, np.asarray(self.config.base_position, dtype=float)).homogeneous
        self._robot.set_T_world_fbase(world_base @ np.linalg.inv(model_base))
        self._robot.update_kinematics()
        self._limits = np.array([self._robot.get_joint_limits(n) for n in self.joint_names])  # URDF 关节限位，rad
        self._tracking_solver = None  # 遥操作单步 QP 求解器，开始跟随时建立
        self._tracking_task = None  # 遥操作单步 QP 的末端任务

    def _make_solver(self, name):
        """共用固定基座、关节限位和末端任务配置。"""
        solver = placo.KinematicsSolver(self._robot)
        solver.mask_fbase(True)
        solver.enable_joint_limits(True)
        task = solver.add_frame_task(self.config.end_frame, self._robot.get_T_world_frame(self.config.end_frame))
        task.configure(name, "soft", 1., 1.)
        return solver, task

    @property
    def joint_limits(self) -> np.ndarray:
        """按 joint_names 排列的 [下限, 上限] 副本，rad。"""
        return self._limits.copy()

    def _set_joints(self, joints):
        """设置模型关节角并更新运动学，返回独立的角度数组。"""
        angles = np.asarray(joints, dtype=float)
        if angles.shape != (len(self.joint_names),) or not np.isfinite(angles).all():
            raise ValueError("关节角必须为与 joint_names 等长的有限一维数组")
        for name, angle in zip(self.joint_names, angles):
            self._robot.set_joint(name, float(angle))
        self._robot.update_kinematics()
        return angles.copy()

    def forward(self, joints: Sequence[float]) -> POSE_DATA:
        """正向运动学计算, 已增加安装基座位姿"""
        self._set_joints(joints)
        matrix = self._robot.get_T_world_frame(self.config.end_frame)
        return matrix_to_pose(matrix, self.config.world_frame, self.config.end_frame)

    def _target_matrix(self, target_pose):
        if target_pose.frame_id and target_pose.frame_id != self.config.world_frame:
            raise ValueError("目标必须在配置的 world_frame 中表达")
        if target_pose.child_frame_id and target_pose.child_frame_id != self.config.end_frame:
            raise ValueError("目标末端必须为配置的 end_frame，请先转换工具 TCP")
        return pose_to_matrix(target_pose)

    def begin_tracking(self, joints: Sequence[float], *, dt: float = .02,
                       reference_joints: Optional[Sequence[float]] = None) -> None:
        """从实测角开始单步跟踪，加入可操作度、参考姿态和速度约束。"""
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt 必须为有限正数")
        current = self._set_joints(joints)
        reference = current if reference_joints is None else np.asarray(reference_joints, dtype=float)
        if reference.shape != current.shape or not np.isfinite(reference).all():
            raise ValueError("参考关节角维度不匹配或含无效值")
        self._tracking_solver, self._tracking_task = self._make_solver("tracking")
        self._tracking_solver.dt = dt
        self._tracking_solver.enable_velocity_limits(True)
        manipulability = self._tracking_solver.add_manipulability_task(self.config.end_frame, "both", 1.)
        manipulability.configure("manipulability", "soft", 1e-2)
        regularization = self._tracking_solver.add_joints_task()
        regularization.set_joints(dict(zip(self.joint_names, reference)))
        regularization.configure("joints_regularization", "soft", 1e-4)

    def track(self, target_pose: POSE_DATA) -> Optional[list[float]]:
        """每周期只推进一次 QP，不要求本周期到达目标；跟踪过程中勿调用 forward 改写同一实例的模型状态。"""
        if self._tracking_solver is None:
            raise RuntimeError("请先调用 begin_tracking")
        self._tracking_task.T_world_frame = self._target_matrix(target_pose)
        try:
            self._robot.update_kinematics()
            self._tracking_solver.solve(True)
        except RuntimeError:
            return None
        joints = np.array([self._robot.get_joint(name) for name in self.joint_names])
        if not np.isfinite(joints).all():
            return None
        joints = np.clip(joints, self._limits[:, 0], self._limits[:, 1])
        self._set_joints(joints)
        return joints.tolist()
