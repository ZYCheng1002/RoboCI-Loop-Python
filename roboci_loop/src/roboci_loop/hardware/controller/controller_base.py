from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from ...common.msg_controller import TRACKER_STATE


@dataclass(frozen=True)
class ControllerConfig:
    controller_name: str = "controller"

    def __post_init__(self) -> None:
        if not isinstance(self.controller_name, str) or not self.controller_name.strip():
            raise ValueError("controller_name 必须为非空字符串")


class ControllerBase(ABC):
    """头显与双手柄输入；由上层串行调用，不创建轮询线程。"""

    def __init__(self, config: Optional[ControllerConfig] = None) -> None:
        self.config = config if config is not None else ControllerConfig()

    @property
    @abstractmethod
    def is_running(self) -> bool:
        """是否已启动 SDK，不代表设备在线。"""

    @abstractmethod
    def start(self) -> None:
        """启动输入；重复调用无操作。"""

    @abstractmethod
    def read(self) -> Optional[TRACKER_STATE]:
        """非阻塞读取最新状态；没有新数据返回 None。"""

    @abstractmethod
    def stop(self) -> None:
        """关闭输入；重复调用无操作。"""

    def __enter__(self) -> "ControllerBase":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()
