"""ROS Dashboard service、录制、状态统计与相机发布节点。"""

import base64
from collections import deque
from datetime import datetime
import json
import threading
import time

import av
import numpy as np
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, JointState

from record_history import delete_history_record, discard_history_record, read_history, validate_metadata
from roboci_loop.common import CAM_MSG
from roboci_loop.recording.mcap_record import McapRecord

CAMERA_KEYS = {
    'observation/image': 'base',
    'observation/left_wrist_image': 'left_wrist',
    'observation/right_wrist_image': 'right_wrist',
}


def transform_camera_frame(frame, rotate_180=False):
    """按相机安装方向原地变换一帧，并返回同一对象。"""
    if rotate_180:
        for channel in ('rgb', 'depth'):
            values = getattr(frame, channel)
            if values is not None and values.size:
                setattr(frame, channel, np.ascontiguousarray(values[::-1, ::-1]))
    return frame


def stamp(header, seconds, frame_id=''):
    ns = round(seconds * 1_000_000_000)
    header.stamp.sec, header.stamp.nanosec = divmod(ns, 1_000_000_000)
    header.frame_id = frame_id


class PicoRos2(Node):
    def __init__(self, config, output_dir, prefix='/human_tele_correct'):
        super().__init__('human_tele_correct')
        self.prefix = '/' + prefix.strip('/')
        self.recorder = McapRecord(output_dir)
        self.lock = threading.RLock()
        self.ready = False
        self.preview = {}                           # 相机预览图像
        self.preview_times = {}
        self.samples_input = deque(maxlen=180)
        self.samples = {}
        self.started_at = None
        self.completed = None
        self.review = None
        self.targets = {}
        self.publishers_by_topic = {}               # 存储所有的publisher
        self.topic_types = {}                       # 存储所有的topic数据类型
        self.config = config
        self.camera_frames = {}
        self.inference_state = 'idle'
        self.inference_outcome = None
        self.intervened = False
        self.active_prompt = ''
        self.active_task = ''
        self.trajectory = []
        self.chunk_start_index = 0
        self.chunk_start_target = None
        self.inference_worker = None
        self.start_inference_callback = None
        self.stop_inference_callback = None
        self.rollback_callback = None
        self.abort_inference_callback = None
        self.review_inference_callback = None
        self.home_callback = None
        self.rollback_active = False
        self.rollback_requires_release = False
        recording = config.get('recording', {})
        self.rgb_jpeg_quality = int(recording.get('rgb_jpeg_quality', 85))
        if not 1 <= self.rgb_jpeg_quality <= 100:
            raise ValueError('recording.rgb_jpeg_quality 必须在 1–100 之间')
        preview = config.get('preview', {})
        self.preview_main_camera = str(preview.get('main_camera', 'base'))          # 预览的主相机设备
        self.preview_main_width = int(preview.get('main_width', 640))               # 相机画面预览主相机尺寸
        self.preview_secondary_width = int(preview.get('secondary_width', 320))     # 相机画面预览其余相机尺寸
        self.preview_jpeg_quality = int(preview.get('jpeg_quality', 90))
        if self.preview_main_camera not in config['cameras']:
            raise ValueError('preview.main_camera 必须是已配置的相机')
        if self.preview_main_width <= 0 or self.preview_secondary_width <= 0:
            raise ValueError('preview 宽度必须为正整数')
        if not 1 <= self.preview_jpeg_quality <= 100:
            raise ValueError('preview.jpeg_quality 必须在 1–100 之间')
        self.target_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 初始化topic
        for kind in ('state', 'target'):
            for group, specs in (('arms', config['arms']), ('hands', config['end_effectors'])):
                for name in specs:
                    self.add_topic(f'{kind}/{group}/{name}/joint_states', JointState, 'sensor_msgs/msg/JointState', kind)
                    if group == 'arms':
                        self.add_topic(f'{kind}/{group}/{name}/end_pose', PoseStamped, 'geometry_msgs/msg/PoseStamped', kind)
        # 初始化相机相关的topic
        for name, spec in config['cameras'].items():
            self.add_topic(f'cameras/{name}/rgb/image_compressed', CompressedImage,
                           'sensor_msgs/msg/CompressedImage')
            if spec.get('config', {}).get('enable_depth', False):
                self.add_topic(f'cameras/{name}/depth/image_raw', Image, 'sensor_msgs/msg/Image')
        # 初始化service
        self.create_service(SetParametersAtomically, f'{self.prefix}/dashboard', self.dashboard)

    def add_topic(self, suffix, cls, type_name, kind='state'):
        topic = f'{self.prefix}/{suffix}'
        self.samples[topic] = deque(maxlen=180)
        self.topic_types[topic] = type_name
        self.publishers_by_topic[topic] = self.create_publisher(cls, topic, self.target_qos if kind == 'target' else 10)

    def emit(self, suffix, message, target=False):
        topic = f'{self.prefix}/{suffix}'
        with self.lock:
            self.samples[topic].append(time.monotonic())
            self.publishers_by_topic[topic].publish(message)
            if target:
                self.targets[topic] = message
            self.recorder.record(topic, message)

    def start_recording(self, metadata):
        if not self.ready:
            raise RuntimeError('设备尚未就绪')
        if self.recorder.is_recording:
            raise RuntimeError('正在录制')
        if self.completed is not None and self.review is None:
            raise RuntimeError('请先提交上一条数据的质检')
        metadata = validate_metadata(metadata)
        path = self.recorder.start(self.topic_types, metadata['task'])
        try:
            (path / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
            (path / 'config.py').write_text('CONFIG = ' + repr(self.config) + '\n')
            for topic, message in self.targets.items():
                self.recorder.record(topic, message)
        except Exception:
            self.recorder.stop()
            raise
        self.started_at = time.monotonic()
        self.review = None
        return {'ok': True, 'message': '开始录制', 'bag_directory': str(path)}

    def stop_recording(self):
        if not self.recorder.is_recording:
            raise RuntimeError('当前未录制')
        self.completed = self.recorder.stop()
        self.started_at = None
        return {'ok': True, 'message': '录制已保存，请提交数据质检', 'bag_directory': str(self.completed)}

    def save_inference_result(self, success):
        result = {
            'outcome': 'success' if success else 'failure',
            'success': success,
            'intervened': self.intervened,
            'prompt': self.active_prompt,
            'trajectory_steps': len(self.trajectory),
            'finished_at': datetime.now().astimezone().isoformat(),
        }
        task_dir = self.recorder.output_dir / self.active_task
        task_dir.mkdir(parents=True, exist_ok=True)
        stats_path = task_dir / 'stats.json'
        stats = {
            'attempts': 0, 'successes': 0, 'failures': 0,
            'interventions': 0, 'trajectory_steps_total': 0,
            'average_trajectory_steps': 0., 'success_rate': 0.,
            'prompts': {},
        }
        if stats_path.exists():
            loaded = json.loads(stats_path.read_text(encoding='utf-8'))
            if isinstance(loaded, dict):
                stats.update(loaded)
        stats['attempts'] += 1
        stats['successes' if result['success'] else 'failures'] += 1
        stats['interventions'] += int(result['intervened'])
        stats['trajectory_steps_total'] += result['trajectory_steps']
        stats['success_rate'] = stats['successes'] / stats['attempts']
        stats['average_trajectory_steps'] = (
            stats['trajectory_steps_total'] / stats['attempts']
        )
        prompt_stats = stats['prompts'].setdefault(
            result['prompt'], {'attempts': 0, 'successes': 0, 'failures': 0}
        )
        prompt_stats['attempts'] += 1
        prompt_stats['successes' if result['success'] else 'failures'] += 1
        stats['updated_at'] = result['finished_at']
        stats_path.write_text(
            json.dumps(stats, ensure_ascii=False, indent=2), encoding='utf-8'
        )

    def camera_snapshot(self):
        inference = self.config['inference']
        now = time.time()
        with self.lock:
            frames = [self.camera_frames.get(name) for name in CAMERA_KEYS.values()]
        missing = [
            name for name, frame in zip(CAMERA_KEYS.values(), frames) if frame is None
        ]
        if missing:
            raise TimeoutError(f"相机尚未就绪：{', '.join(missing)}")
        stamps = [frame.time_stamp for frame in frames]
        if now - min(stamps) > inference['max_observation_age']:
            raise TimeoutError('相机图像已过期')
        if max(stamps) - min(stamps) > inference['max_camera_skew']:
            raise TimeoutError('相机图像时间差过大')
        return CAM_MSG(time_stamp=max(stamps), frames=frames)

    def status(self):
        # 系统状态检查
        self.recorder.check_error()
        now = time.monotonic()
        devices = {}
        # 计算相关fps
        for topic, samples in {**self.samples, 'PICO': self.samples_input}.items():
            if '/target/' in topic or '/end_pose' in topic:
                continue
            recent = [value for value in samples if now - value <= 2.]
            devices[topic] = {'online': bool(recent) and now - recent[-1] < 1.,
                             'fps': (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0.}
        stats = {
            'attempts': 0, 'successes': 0, 'failures': 0,
            'interventions': 0, 'success_rate': 0.,
        }
        if self.active_task:
            stats_path = self.recorder.output_dir / self.active_task / 'stats.json'
            try:
                if stats_path.exists():
                    stats.update(json.loads(stats_path.read_text(encoding='utf-8')))
            except (OSError, ValueError, UnicodeError):
                pass
        return {'ok': True, 'ready': self.ready,
                'overall': 'online' if self.ready and devices and all(d['online'] for d in devices.values()) else 'offline',
                'devices': devices, 'recording': self.recorder.is_recording,
                'recording_duration_sec': now-self.started_at if self.started_at is not None else 0.,
                'review_pending': self.completed is not None and self.review is None and not self.recorder.is_recording,
                'bag_directory': str(self.recorder.path or ''),
                'inference_state': self.inference_state,
                'inference_outcome': self.inference_outcome,
                'intervened': self.intervened,
                'active_prompt': self.active_prompt,
                'trajectory_steps': len(self.trajectory),
                'chunk_steps': len(self.trajectory) - self.chunk_start_index,
                'rollback_active': self.rollback_active,
                'task_stats': stats,
                'prompts': self.config['inference']['prompts']}

    def dashboard(self, request, response):
        """web交互"""
        # A single atomic request carries the action and JSON payload, without a custom ROS package.
        with self.lock:
            try:
                if len(request.parameters) != 1:
                    raise ValueError('每次请求传入一个字符串参数')
                parameter = request.parameters[0]
                if parameter.value.type != 4:
                    raise ValueError('请求参数必须为字符串 JSON')
                action, payload = parameter.name, json.loads(parameter.value.string_value or '{}')
                active = self.recorder.path if self.recorder.is_recording else None
                if action == 'status':
                    result = self.status()
                elif action == 'start':
                    result = self.start_recording(payload)
                elif action == 'stop':
                    result = self.stop_recording()
                elif action == 'inference_start':
                    result = self.start_inference_callback(payload.get('prompt'), payload.get('task'))
                elif action == 'inference_stop':
                    result = self.stop_inference_callback()
                elif action == 'inference_abort':
                    result = self.abort_inference_callback()
                elif action == 'inference_review':
                    result = self.review_inference_callback(payload.get('success'))
                elif action == 'home':
                    result = self.home_callback()
                elif action == 'rollback':
                    result = self.rollback_callback()
                elif action == 'history':
                    result = {'ok': True, **read_history(self.recorder.output_dir, active)}
                elif action == 'review':
                    if self.completed is None or self.recorder.is_recording:
                        raise ValueError('没有可标注的已完成录制')
                    if payload.get('status') not in ('accepted', 'rejected', 'needs_review'):
                        raise ValueError('质检状态无效')
                    review = {key: str(payload.get(key, '')) for key in ('status', 'reason', 'note')}
                    review['reviewed_at'] = datetime.now().astimezone().isoformat()
                    (self.completed / 'review.json').write_text(
                        json.dumps(review, ensure_ascii=False, indent=2), encoding='utf-8'
                    )
                    if review['status'] == 'rejected':
                        self.completed = discard_history_record(
                            self.recorder.output_dir, self.completed, active
                        )
                        message = '数据已移入当前 Task 的 .trash 目录'
                    else:
                        message = '质检已保存'
                    self.review = review
                    result = {'ok': True, 'message': message,
                              'bag_directory': str(self.completed)}
                elif action == 'delete':
                    deleted = delete_history_record(self.recorder.output_dir, payload.get('directory'), active)
                    if deleted == self.completed:
                        self.completed = None
                    result = {'ok': True, 'message': '记录已删除'}
                elif action == 'image':
                    image = self.preview.get(payload.get('name'))
                    result = {'ok': image is not None, 'image': base64.b64encode(image).decode() if image else ''}
                else:
                    raise ValueError(f'未知操作：{action}')
                response.result.successful = result.get('ok', True)
                response.result.reason = json.dumps(result, ensure_ascii=False)
            except Exception as exc:
                response.result.successful = False
                response.result.reason = json.dumps({'ok': False, 'message': str(exc)}, ensure_ascii=False)
        return response

    def robot(self, data, kind, events, ik):
        # service 开始录制不能切在同一次目标的 joint_states 与 end_pose 之间。
        with self.lock:
            self._robot(data, kind, events, ik)

    def _robot(self, data, kind, events, ik):
        data = events.prepare(data, kind=kind)
        if data is None:
            return
        for group, items, attr in (('arms', data.arms, 'arm_name'), ('hands', data.end_effectors, 'end_effector_name')):
            for item in items:
                name = getattr(item, attr)
                msg = JointState()
                stamp(msg.header, item.time_stamp)
                msg.name = list(item.joints.names)
                msg.position = item.joints.position.tolist()
                msg.velocity = item.joints.velocity.tolist()
                msg.effort = item.joints.effort.tolist()
                self.emit(f'{kind}/{group}/{name}/joint_states', msg, kind == 'target')
                if group == 'arms':
                    pose = ik.forward(item.joints.position)
                    msg = PoseStamped()
                    stamp(msg.header, item.time_stamp, pose.frame_id)
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pose.position)
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, pose.orientation)
                    self.emit(f'{kind}/{group}/{name}/end_pose', msg, kind == 'target')

    def camera(self, frame):
        """统一变换相机帧，再用于推理、预览和 ROS/MCAP 发布。"""
        transform = self.config.get('camera_transforms', {}).get(frame.camera_name, {})
        frame = transform_camera_frame(frame, transform.get('rotate_180', False))
        with self.lock:
            self.camera_frames[frame.camera_name] = frame
        # 先完成相机预览图像处理
        if frame.rgb is not None and frame.rgb.size:
            now = time.monotonic()
            key = f'preview/{frame.camera_name}'
            if now - self.preview_times.get(key, 0.) >= .2:
                source_height, source_width = frame.rgb.shape[:2]
                requested_width = (self.preview_main_width
                    if frame.camera_name == self.preview_main_camera
                    else self.preview_secondary_width)
                width = min(source_width, requested_width)
                # yuv420p 要求偶数尺寸，同时保持原始宽高比。
                width = max(2, width // 2 * 2)
                height = max(2, round(source_height * width / source_width) // 2 * 2)
                image = av.VideoFrame.from_ndarray(frame.rgb, format='rgb24')
                image = image.reformat(width=width, height=height, format='yuvj420p')
                codec = av.CodecContext.create('mjpeg', 'w')
                codec.width, codec.height, codec.pix_fmt = width, height, 'yuvj420p'
                codec.options = {'qscale': str(
                    2 + round((100 - self.preview_jpeg_quality) * 29 / 99)
                )}
                encoded = b''.join(bytes(packet) for packet in [*codec.encode(image), *codec.encode(None)])
                with self.lock:
                    self.preview[frame.camera_name] = encoded
                self.preview_times[key] = now
        # 相机落盘数据处理
        for channel, values, encoding in (
                ('rgb', frame.rgb, 'rgb8'), ('depth', frame.depth, '32FC1')):
            if values is None or not values.size:
                continue
            if channel == 'rgb':
                image = av.VideoFrame.from_ndarray(
                    np.ascontiguousarray(values, dtype=np.uint8), format='rgb24')
                codec = av.CodecContext.create('mjpeg', 'w')
                codec.width, codec.height, codec.pix_fmt = image.width, image.height, 'yuvj420p'
                # FFmpeg 的 MJPEG qscale 越小质量越高；对外仍使用直观的 1–100。
                codec.options = {'qscale': str(2 + round((100 - self.rgb_jpeg_quality) * 29 / 99))}
                image = image.reformat(format='yuvj420p')
                msg = CompressedImage()
                stamp(msg.header, frame.time_stamp, f'{frame.camera_name}_optical_frame')
                msg.format = 'jpeg'
                msg.data = b''.join(bytes(packet) for packet in
                                    [*codec.encode(image), *codec.encode(None)])
                # 记录数据
                self.emit(f'cameras/{frame.camera_name}/rgb/image_compressed', msg)
                continue
            # 深度图
            values = np.ascontiguousarray(values, dtype=np.dtype('<f4'))
            msg = Image()
            stamp(msg.header, frame.time_stamp, f'{frame.camera_name}_optical_frame')
            msg.height, msg.width = values.shape[:2]
            msg.encoding, msg.is_bigendian = encoding, 0
            msg.step = values.strides[0]
            msg.data = values.tobytes()
            self.emit(f'cameras/{frame.camera_name}/{channel}/image_raw', msg)


