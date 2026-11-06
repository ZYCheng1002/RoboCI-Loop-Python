"""内部自定义单相机帧和多相机消息。"""

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class CAM_FRAME:
    """相机帧；图像使用 NumPy 数组, None 或空数组表示无图像。"""

    camera_name: str = ""               # 相机名称
    rgb: Optional[np.ndarray] = None    # RGB 图像，uint8，形状 (H, W, 3)
    depth: Optional[np.ndarray] = None  # 深度图，float32，形状 (H, W)，单位 m
    time_stamp: float = 0.0             # 主机系统时间戳, 秒

    def hasRGB(self) -> bool:
        """判断 RGB 图像是否非空。"""
        return self.rgb is not None and self.rgb.size > 0

    def hasDepth(self) -> bool:
        """判断深度图是否非空。"""
        return self.depth is not None and self.depth.size > 0


@dataclass
class CAM_MSG:
    """多视角相机消息"""

    time_stamp: float = 0.0                                 # 消息时间戳, 时间同步后
    frames: List[CAM_FRAME] = field(default_factory=list)   # 帧列表

    def size(self) -> int:
        """获取相机数量。"""
        return len(self.frames)

    def empty(self) -> bool:
        """判断消息是否为空。"""
        return not self.frames

    def addFrame(self, frame: CAM_FRAME) -> None:
        """添加相机帧（保存对象引用）。"""
        self.frames.append(frame)

    def getFrame(self, name: str) -> Optional[CAM_FRAME]:
        """返回首个名称匹配的相机帧，未找到时返回 None。"""
        for frame in self.frames:
            if frame.camera_name == name:
                return frame
        return None

    def clear(self) -> None:
        """重置时间戳并清空相机帧。"""
        self.time_stamp = 0.0
        self.frames.clear()
