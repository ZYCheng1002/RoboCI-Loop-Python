"""位姿消息与齐次矩阵转换；统一使用 m 和 xyzw 四元数。"""
import numpy as np
import pinocchio as pin

from ..common import POSE_DATA


def pose_to_matrix(pose: POSE_DATA) -> np.ndarray:
    if pose is None:
        raise ValueError("位姿缺失")
    position = np.asarray(pose.position, dtype=float)
    quaternion = np.asarray(pose.orientation, dtype=float)
    if position.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(position).all() or not np.isfinite(quaternion).all():
        raise ValueError("位姿数据无效")
    norm = np.linalg.norm(quaternion)
    if norm < 1e-9:
        raise ValueError("四元数无效")
    return pin.SE3(pin.Quaternion(quaternion / norm).matrix(), position).homogeneous.copy()


def matrix_to_pose(matrix: np.ndarray, frame: str, child: str) -> POSE_DATA:
    return POSE_DATA(frame_id=frame, child_frame_id=child,
                     position=matrix[:3, 3].copy(),
                     orientation=pin.Quaternion(matrix[:3, :3]).coeffs().copy())
