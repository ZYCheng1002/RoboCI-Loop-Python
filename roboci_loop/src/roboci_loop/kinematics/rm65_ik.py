"""RM65-B 的模型配置；正逆运动学算法复用通用 PlacoIK。"""
from dataclasses import dataclass, field
from importlib.resources import files
from typing import Optional

from .placo_ik import PlacoIK, PlacoIKConfig


@dataclass(frozen=True)
class RM65IKConfig(PlacoIKConfig):
    urdf_path: str = field(default_factory=lambda: str(
        files("roboci_loop").joinpath("models", "rm65_b", "RM65-B.urdf")
    ))  # 随 roboci_loop 安装的 RM65-B 模型，也可传入其他 URDF 绝对路径
    base_frame: str = "base_link"  # RM65-B 基座
    end_frame: str = "link_6"  # URDF 末端，不包含额外工具 TCP
    joint_names: tuple[str, ...] = tuple(f"joint_{i}" for i in range(1, 7))  # 与 RealMan 硬件一致的关节顺序


class RM65IK(PlacoIK):
    """RM65-B 模型默认配置；安装位姿和世界系 FK/IK 由 PlacoIK 统一处理。"""

    def __init__(self, config: Optional[RM65IKConfig] = None) -> None:
        super().__init__(config if config is not None else RM65IKConfig())
