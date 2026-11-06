"""外部传入图像、反馈或目标，后台编码保存；不采集硬件、不决定采样频率。"""
from contextlib import ExitStack
import csv
from dataclasses import asdict, dataclass, field
from datetime import datetime
from fractions import Fraction
import json
from pathlib import Path
import queue
import threading
import time

import av
import numpy as np

from ..common import CAM_FRAME, ROBOT_DATA


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"不支持的记录类型：{type(value).__name__}")


def _depth_mm(depth):
    values = np.rint(depth * 1000)
    valid = np.isfinite(values) & (values >= 1) & (values <= 65535)
    result = np.zeros(depth.shape, dtype=np.uint16)
    result[valid] = values[valid].astype(np.uint16)
    return result


@dataclass(frozen=True)
class RecorderConfig:
    output_dir: str = field(default_factory=lambda: f"data/{datetime.now():%Y%m%d_%H%M%S_%f}")  # 每次记录的新目录
    record_depth: bool = False  # 默认仅保存 RGB
    fps: int = 30  # 视频编码播放帧率，实际采集时间另存 CSV
    crf: int = 18  # H.264 质量参数，越小质量越高
    queue_frames: int = 32  # 每台相机的最大待编码帧数
    queue_robot: int = 1024  # 反馈/目标共用的待写入条数

    def __post_init__(self):
        for value in (self.fps, self.queue_frames, self.queue_robot):
            if not isinstance(value, int) or value <= 0:
                raise ValueError("帧率和队列长度必须为正整数")
        if not 0 <= self.crf <= 51:
            raise ValueError("crf 必须为 0~51")


class Recorder:
    """record_camera / record_robot 线程安全，队列满返回 False，不阻塞控制。

    入队时复制快照；关闭时排空已接收数据并刷新编码器。
    不做相机同步、重采样或状态插值；调用方决定什么时候记录哪些数据。
    start/stop 由外部串行调用，同一实例只使用一次。
    """

    def __init__(self, config=None):
        self.config = config if config is not None else RecorderConfig()  # 保存配置
        self.output_dir = Path(self.config.output_dir).expanduser().resolve()  # 保存目录
        self._lock = threading.RLock()  # 保护线程、统计和后台异常
        self._done = threading.Event()  # 不再接受数据，写入线程排空队列后退出
        self._streams = {}  # 流名称 → (队列, 工作线程)
        self._stats = {}  # 每条流的接收、写入、丢弃统计
        self._error = None  # 首个后台写入异常
        self._started = None  # 开始记录的 Unix 时间，None 表示未启动
        self._finished = False  # 是否已经完成关闭

    def start(self):
        if self._started is not None:
            raise RuntimeError("Recorder 已使用，请创建新实例")
        av.codec.Codec("libx264", "w")
        if self.config.record_depth:
            av.codec.Codec("ffv1", "w")
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self._started = time.time()
        with self._lock:
            self._spawn("robot", self._write_robot, self.config.queue_robot)

    def check_error(self):
        with self._lock:
            if self._error is not None:
                raise RuntimeError("记录后台线程失败") from self._error

    def _spawn(self, name, handler, capacity):
        pending = queue.Queue(capacity)
        self._stats[name] = dict(accepted=0, written=0, dropped=0)
        thread = threading.Thread(target=self._run, args=(name, handler, pending), name=f"record-{name}")
        self._streams[name] = pending, thread
        thread.start()

    def _run(self, name, handler, pending):
        try:
            handler(name, pending)
        except Exception as exc:
            with self._lock:
                if self._error is None:
                    self._error = exc
                self._done.set()

    def _offer(self, name, item, handler=None):
        with self._lock:
            self.check_error()
            if self._started is None or self._done.is_set():
                raise RuntimeError("Recorder 未启动或已停止")
            if name not in self._streams:
                self._spawn(name, handler, self.config.queue_frames)
            try:
                self._streams[name][0].put_nowait(item)
                self._stats[name]["accepted"] += 1
                return True
            except queue.Full:
                self._stats[name]["dropped"] += 1
                return False

    def record_robot(self, data: ROBOT_DATA, *, kind="state", metadata=None) -> bool:
        """反馈和目标共用接口，kind 为 state 或 target；metadata 可记录模式、来源等。"""
        if kind not in ("state", "target"):
            raise ValueError("kind 必须为 state 或 target")
        payload = asdict(data)
        for arm in payload['arms']:
            pose = arm['end_pose']
            if pose is not None:
                pose['position'] = dict(zip(('x', 'y', 'z'), pose['position']))
                pose['orientation'] = dict(zip(('qx', 'qy', 'qz', 'qw'), pose['orientation']))
        line = json.dumps(dict(schema_version=2, kind=kind, recorded_at=time.time(), data=payload,
                               metadata=metadata or {}), default=_json_value, allow_nan=False)
        return self._offer("robot", line)

    def record_camera(self, frame: CAM_FRAME) -> bool:
        """保存 RGB 和可选深度，时间戳保留原值；每个 camera_name 单独编码。"""
        name = frame.camera_name
        if not name or name in (".", "..") or "/" in name or "\\" in name:
            raise ValueError("camera_name 必须为非空单级目录名称")
        if not frame.hasRGB() or frame.rgb.dtype != np.uint8 or frame.rgb.ndim != 3 or frame.rgb.shape[2] != 3:
            raise ValueError("RGB 必须为 H×W×3 uint8 数组")
        if frame.rgb.shape[0] % 2 or frame.rgb.shape[1] % 2:
            raise ValueError("yuv420p 编码要求图像宽高为偶数")
        depth = None
        if self.config.record_depth and frame.hasDepth():
            if frame.depth.ndim != 2:
                raise ValueError("深度图必须为二维米制数组")
            depth = frame.depth.copy()
        snapshot = CAM_FRAME(name, frame.rgb.copy(), depth, frame.time_stamp)
        return self._offer(f"cameras/{name}", (snapshot, time.time()), self._write_camera)

    def _items(self, pending):
        while not self._done.is_set() or not pending.empty():
            try:
                yield pending.get(timeout=.1)
            except queue.Empty:
                continue

    def _written(self, name):
        with self._lock:
            self._stats[name]["written"] += 1

    def _write_robot(self, name, pending):
        with (self.output_dir / "robot.jsonl").open("w", encoding="utf-8") as file:
            for line in self._items(pending):
                file.write(line + "\n")
                self._written(name)

    def _write_camera(self, name, pending):
        folder = self.output_dir / name
        folder.mkdir(parents=True)
        stream = depth_stream = video = depth_video = None
        count = depth_count = 0
        with ExitStack() as stack:
            csv_file = stack.enter_context((folder / "timestamps.csv").open("w", newline=""))
            writer = csv.writer(csv_file)
            writer.writerow(["frame_index", "time_stamp_s", "recorded_at_s", "depth_frame_index"])
            for frame, recorded_at in self._items(pending):
                height, width = frame.rgb.shape[:2]
                if stream is None:
                    video = stack.enter_context(av.open(str(folder / "rgb.mp4"), "w"))
                    stream = video.add_stream("libx264", rate=self.config.fps)
                    stream.width, stream.height, stream.pix_fmt = width, height, "yuv420p"
                    stream.codec_context.thread_count = 2
                    stream.options = dict(crf=str(self.config.crf), preset="ultrafast", tune="zerolatency")
                if (height, width) != (stream.height, stream.width):
                    raise ValueError("记录期间 RGB 尺寸发生变化")
                image = av.VideoFrame.from_ndarray(frame.rgb, format="rgb24")
                image.pts = count
                image.time_base = Fraction(1, self.config.fps)
                for packet in stream.encode(image):
                    video.mux(packet)
                depth_index = ""
                if frame.hasDepth():
                    millimeters = _depth_mm(frame.depth)
                    if depth_stream is None:
                        depth_video = stack.enter_context(av.open(str(folder / "depth.mkv"), "w"))
                        depth_stream = depth_video.add_stream("ffv1", rate=self.config.fps)
                        depth_stream.height, depth_stream.width = millimeters.shape
                        depth_stream.pix_fmt = "gray16le"
                        depth_stream.codec_context.thread_count = 2
                    if millimeters.shape != (depth_stream.height, depth_stream.width):
                        raise ValueError("记录期间深度尺寸发生变化")
                    image = av.VideoFrame.from_ndarray(millimeters, format="gray16le")
                    image.pts, image.time_base = count, Fraction(1, self.config.fps)
                    for packet in depth_stream.encode(image):
                        depth_video.mux(packet)
                    depth_index = depth_count
                    depth_count += 1
                writer.writerow([count, frame.time_stamp, recorded_at, depth_index])
                count += 1
                self._written(name)
            for encoding, container in ((stream, video), (depth_stream, depth_video)):
                if encoding is not None:
                    for packet in encoding.encode():
                        container.mux(packet)
        with self._lock:
            self._stats[name]["depth_written"] = depth_count

    def stop(self):
        if self._started is None or self._finished:
            return
        with self._lock:
            self._done.set()
            threads = [thread for _, thread in self._streams.values()]
        for thread in threads:
            thread.join()
        self._finished = True
        complete = self._error is None and all(s["dropped"] == 0 and s["accepted"] == s["written"]
                                               for s in self._stats.values())
        info = dict(config=asdict(self.config), started_at=self._started, ended_at=time.time(),
                    complete=complete, error=str(self._error) if self._error else None,
                    streams=self._stats, depth_unit="mm", depth_invalid=0)
        info['robot_schema'] = dict(version=2, joint_position_unit='rad', position_unit='m',
                                   orientation='quaternion_xyzw', timestamp_unit='unix_seconds',
                                   sampling='device_feedback_events_and_submitted_targets; no_state_fill_or_resampling',
                                   sample_timestamp='data.time_stamp: state=feedback event; target=submission or startup',
                                   device_timestamp='arms/end_effectors[].time_stamp: state=feedback sample; target=submission or startup')
        (self.output_dir / "info.json").write_text(json.dumps(info, default=_json_value, indent=2), encoding="utf-8")
        self.check_error()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
