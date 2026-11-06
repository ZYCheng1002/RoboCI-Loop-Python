"""手与夹爪共用的末端执行器接口。"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from ...common.msg_robot import END_EFFECTOR_DATA


@dataclass(frozen=True)
class EndEffectorConfig:
    """手、夹爪等末端执行器的公共配置。"""

    end_effector_name: str = "end_effector"

    def __post_init__(self) -> None:
        if not isinstance(self.end_effector_name, str) or not self.end_effector_name.strip():
            raise ValueError("end_effector_name 必须为非空字符串")


class EndEffectorBase(ABC):
    def __init__(self, config: Optional[EndEffectorConfig] = None) -> None:
        self.config = config if config is not None else EndEffectorConfig()

    @property
    @abstractmethod
    def is_running(self) -> bool:
        """是否已连接。"""

    @property
    @abstractmethod
    def joint_limits(self) -> np.ndarray:
        """连接后返回关节限位副本，形状 (N, 2)，每行为 [下限, 上限]，单位 rad。

        顺序与 read() 的关节反馈及 send_joint_positions() 的目标一致。
        """

    @abstractmethod
    def start(self) -> None:
        """连接设备。"""

    @abstractmethod
    def read(self) -> Optional[END_EFFECTOR_DATA]:
        """返回新反馈；无新数据返回 None，通信失败抛异常。"""

    @abstractmethod
    def send_joint_positions(self, positions: Sequence[float], *, slow: bool = False) -> None:
        """发送绝对关节位置目标。

        手和夹爪均按驱动关节顺序传入角度列表（rad）。
        slow=True 请求慢速运动；驱动无此能力时需调用方限速。
        驱动负责校验数量、有效值和硬件限位；上层调用与 read 串行执行，
        驱动可在内部独立采集反馈。
        """

    @abstractmethod
    def stop(self) -> None:
        """关闭连接。"""

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
