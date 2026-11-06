from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Sequence

from ...common.msg_robot import ARM_DATA


@dataclass(frozen=True)
class ArmConfig:
    """机械臂公共配置；通信与控制模式由具体驱动配置。"""

    arm_name: str = "arm"

    def __post_init__(self) -> None:
        if not isinstance(self.arm_name, str) or not self.arm_name.strip():
            raise ValueError("arm_name 必须为非空字符串")


class ArmBase(ABC):
    """机械臂接口。角度使用 rad，距离使用 m；调用方串行调用控制接口。"""

    def __init__(self, config: Optional[ArmConfig] = None) -> None:
        self.config = config if config is not None else ArmConfig()

    @property
    @abstractmethod
    def is_running(self) -> bool:
        """是否已连接。"""

    @abstractmethod
    def start(self) -> None:
        """连接设备；重复调用无操作，不自动运动。"""

    @abstractmethod
    def read(self, timeout_ms: int = 1000) -> Optional[ARM_DATA]:
        """读取状态；等待状态超时返回 None。"""

    @abstractmethod
    def send_joint_positions(self, positions: Sequence[float], *, slow: bool = False) -> None:
        """发送关节目标角度（rad）；slow=True 请求驱动执行慢速运动。"""

    @abstractmethod
    def stop_motion(self) -> None:
        """停止机械臂运动。"""

    @abstractmethod
    def stop(self) -> None:
        """断开连接；重复调用无操作，不等同于停止运动。"""

    def __enter__(self) -> "ArmBase":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()
