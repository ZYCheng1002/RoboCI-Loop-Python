"""控制器模式"""
from enum import Enum, auto


class ControlMode(Enum):
    HOLD = auto()
    INIT = auto()
    AUTO = auto()
    TELEOP = auto()
