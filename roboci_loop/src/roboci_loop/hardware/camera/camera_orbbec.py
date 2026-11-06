import importlib
import time
from typing import Optional

import numpy as np

from ...common.msg_camera import CAM_FRAME
from .camera_base import CameraBase, CameraConfig


class CameraOrbbec(CameraBase):
    """Orbbec SDK v2 相机；RGB uint8，深度 float32（m）。

    请求 RGB 和 Y16 原始图像流，不执行深度到彩色的空间对齐。
    序列号为空时使用 SDK 默认设备；多相机应显式指定序列号。
    """

    def __init__(
        self,
        config: Optional[CameraConfig] = None,
        *,
        serial_number: Optional[str] = None,
        enable_rgb: bool = True,
        enable_depth: bool = False,
    ) -> None:
        super().__init__(config)
        if not enable_rgb and not enable_depth:
            raise ValueError("RGB 和深度至少需要启用一个")
        self.serial_number = serial_number
        self.enable_rgb = enable_rgb
        self.enable_depth = enable_depth
        self._pipeline = None
        self._context = None
        self._sdk = None

    @property
    def is_running(self) -> bool:
        return self._pipeline is not None

    def start(self) -> None:
        if self.is_running:
            return
        try:
            sdk = importlib.import_module("pyorbbecsdk")
        except ModuleNotFoundError as exc:
            if exc.name != "pyorbbecsdk":
                raise
            raise ImportError("请安装 Orbbec SDK：python -m pip install pyorbbecsdk2") from exc

        context = None
        if self.serial_number:
            context = sdk.Context()
            device = context.query_devices().get_device_by_serial_number(self.serial_number)
            pipeline = sdk.Pipeline(device)
        else:
            pipeline = sdk.Pipeline()
        config = sdk.Config()
        streams = []
        if self.enable_rgb:
            streams.append((sdk.OBSensorType.COLOR_SENSOR, sdk.OBFormat.RGB))
        if self.enable_depth:
            streams.append((sdk.OBSensorType.DEPTH_SENSOR, sdk.OBFormat.Y16))
        for sensor, image_format in streams:
            try:
                profiles = pipeline.get_stream_profile_list(sensor)
                profile = profiles.get_video_stream_profile(
                    self.config.width, self.config.height, image_format, self.config.fps
                )
                config.enable_stream(profile)
            except sdk.OBError as exc:
                raise RuntimeError(
                    f"相机 {self.config.camera_name} 不支持请求的图像流："
                    f"{sensor}, {image_format}, {self.config.width}x{self.config.height}"
                    f" @ {self.config.fps} FPS"
                ) from exc
        # 启动失败时释放可能已经部分启动的流，保留原始异常。
        try:
            pipeline.start(config)
        except Exception:
            try:
                pipeline.stop()
            except Exception:
                pass
            raise
        self._sdk = sdk
        self._context = context
        self._pipeline = pipeline

    def read(self, timeout_ms: int = 1000) -> Optional[CAM_FRAME]:
        if not self.is_running:
            raise RuntimeError("相机尚未启动，请先调用 start()")
        if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms <= 0:
            raise ValueError("timeout_ms 必须为正整数")
        frames = self._pipeline.wait_for_frames(timeout_ms)
        if frames is None:
            return None
        time_stamp = time.time()  # 主机收到帧组的 Unix 时间, s, 不等同于曝光时间
        color = frames.get_color_frame() if self.enable_rgb else None
        depth = frames.get_depth_frame() if self.enable_depth else None
        if (self.enable_rgb and color is None) or (self.enable_depth and depth is None):
            return None

        rgb_image = None
        depth_image = None
        if color is not None:
            if color.get_format() != self._sdk.OBFormat.RGB:
                raise RuntimeError("彩色帧格式不是请求的 RGB")
            # 复制数据，避免下一次 SDK 采集复用缓冲区后修改已有消息。
            rgb_image = np.frombuffer(color.get_data(), dtype=np.uint8).reshape(
                color.get_height(), color.get_width(), 3
            ).copy()
        if depth is not None:
            if depth.get_format() != self._sdk.OBFormat.Y16:
                raise RuntimeError("深度帧格式不是请求的 Y16")
            depth_image = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(
                depth.get_height(), depth.get_width()
            ).astype(np.float32)
            depth_image *= np.float32(depth.get_depth_scale() / 1000.0)
        return CAM_FRAME(
            camera_name=self.config.camera_name,
            rgb=rgb_image,
            depth=depth_image,
            time_stamp=time_stamp,
        )

    def stop(self) -> None:
        if not self.is_running:
            return
        self._pipeline.stop()
        self._pipeline = None
        self._context = None
        self._sdk = None
