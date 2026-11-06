"""ROS2 CDR 消息后台写入 MCAP；无需 ROS2 package 或 colcon。"""
from datetime import datetime
from pathlib import Path
import queue
import threading
import time


class McapRecord:
    """可重复 start/stop；每次新建 bag。队列溢出显式报错，不静默丢数据。"""

    def __init__(self, output_dir='data/mcap', queue_size=256):
        if queue_size <= 0:
            raise ValueError('queue_size 必须为正数')
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.queue_size = queue_size
        self._lock = threading.RLock()
        self._thread = None
        self._pending = None
        self._error = None
        self.path = None

    @property
    def is_recording(self):
        with self._lock:
            return self._thread is not None

    def check_error(self):
        if self._error is not None:
            raise RuntimeError('MCAP 后台记录失败') from self._error

    def start(self, topics, subdirectory=None):
        """topics: {topic_name: type_name}；subdirectory 用于按 Task 分组。"""
        import rosbag2_py
        with self._lock:
            if self._thread is not None:
                self.check_error()
                return self.path
            if subdirectory is not None:
                if (not isinstance(subdirectory, str) or not subdirectory
                        or Path(subdirectory).name != subdirectory
                        or subdirectory in ('.', '..')):
                    raise ValueError('录制子目录必须是单层安全目录名')
            parent = self.output_dir if subdirectory is None else self.output_dir / subdirectory
            path = parent / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            parent.mkdir(parents=True, exist_ok=True)
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('cdr', 'cdr'))
            try:
                for index, (name, type_name) in enumerate(topics.items(), 1):
                    writer.create_topic(rosbag2_py.TopicMetadata(
                        id=index, name=name, type=type_name, serialization_format='cdr'))
            except BaseException:
                writer.close()
                raise
            self.path = path
            self._topics = dict(topics)
            self._error = None
            self._pending = queue.Queue(self.queue_size)
            self._thread = threading.Thread(target=self._run, args=(writer, self._pending), name='mcap-writer')
            try:
                self._thread.start()
            except BaseException:
                self._thread = None
                writer.close()
                raise
            return path

    def record(self, topic, message, timestamp_ns=None):
        """未录制时返回 False；bag 时间为接收时间，设备采样时间保留在 header。"""
        from rclpy.serialization import serialize_message
        with self._lock:
            if self._thread is None:
                return False
            self.check_error()
            if topic not in self._topics:
                raise ValueError(f'未注册 topic: {topic}')
            payload = serialize_message(message)
            try:
                self._pending.put_nowait((topic, payload, time.time_ns() if timestamp_ns is None else timestamp_ns))
            except queue.Full as exc:
                self._error = RuntimeError('MCAP 队列已满，请降低图像带宽或提高磁盘吞吐')
                raise self._error from exc
            return True

    def _run(self, writer, pending):
        try:
            while True:
                item = pending.get()
                if item is None:
                    break
                if self._error is None:
                    try:
                        writer.write(*item)
                    except Exception as exc:
                        self._error = exc
        finally:
            try:
                writer.close()
            except Exception as exc:
                self._error = self._error or exc

    def stop(self):
        """等待已接收消息排空并生成 rosbag2 metadata.yaml。"""
        with self._lock:
            if self._thread is None:
                return self.path
            self._pending.put(None)
            self._thread.join()
            self._thread = None
            self.check_error()
            return self.path
