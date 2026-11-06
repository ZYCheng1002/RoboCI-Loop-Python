"""相机配置与硬件适配。"""

from .camera_base import CameraBase, CameraConfig
from .camera_orbbec import CameraOrbbec

__all__ = ["CameraBase", "CameraConfig", "CameraOrbbec"]
