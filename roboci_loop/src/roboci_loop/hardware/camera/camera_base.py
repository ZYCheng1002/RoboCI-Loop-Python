from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from ...common.msg_camera import CAM_FRAME


@dataclass(frozen=True)
class CameraConfig:
    """相机公共配置"""
    camera_name: str = "camera"  # 相机名称
    fps: int = 60  # 帧率, Hz
    width: int = 640  # 图像宽度, 像素
    height: int = 480  # 图像高度, 像素

    def __post_init__(self) -> None:
        if not isinstance(self.camera_name, str) or not self.camera_name.strip():
            raise ValueError("camera_name 必须为非空字符串")
        for name in ("fps", "width", "height"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} 必须为正整数")


class CameraBase(ABC):
    """相机接口；由调用方串行调用，不在内部创建采集线程。"""

    def __init__(self, config: Optional[CameraConfig] = None) -> None:
        self.config = config if config is not None else CameraConfig()

    @property
    @abstractmethod
    def is_running(self) -> bool:
        """是否已启动采集。"""

    @abstractmethod
    def start(self) -> None:
        """启动采集；已启动时重复调用无操作。"""

    @abstractmethod
    def read(self, timeout_ms: int = 1000) -> Optional[CAM_FRAME]:
        """读取一帧；超时或缺少启用的图像流时返回 None，未启动时抛出异常。

        timeout_ms 是 SDK 等待超时配置，单位 ms；消息物理量仍采用 SI。
        """

    @abstractmethod
    def stop(self) -> None:
        """停止采集；未启动时重复调用无操作。"""

    def __enter__(self) -> "CameraBase":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()
