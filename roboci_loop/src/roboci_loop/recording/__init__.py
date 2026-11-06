"""图像、机器人反馈与目标的后台记录。"""

from .recorder import Recorder, RecorderConfig
from .robot_events import RobotEvents
from .mcap_record import McapRecord

__all__ = ["Recorder", "RecorderConfig", "RobotEvents", "McapRecord"]
