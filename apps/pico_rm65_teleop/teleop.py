"""PICO 双臂 RM65 遥操作；独立 ROS2 service 接收 Web 采集请求。"""
import argparse
import base64
from collections import deque
from datetime import datetime
import json
import av
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path
import queue
import runpy
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState, Image, CompressedImage
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParametersAtomically
from record_history import (
    delete_history_record, discard_history_record, read_history, validate_metadata,
)

from roboci_loop.common import ARM_DATA, END_EFFECTOR_DATA, JOINT_DATA, ROBOT_DATA
from roboci_loop.controller import ControlMode, DeviceControlConfig, RobotController, RobotControllerConfig
from roboci_loop.factory import build_arm, build_camera, build_controller, build_end_effector, build_ik
from roboci_loop.hardware.end_effector import HandLG603
from roboci_loop.recording import RobotEvents
from roboci_loop.recording.mcap_record import McapRecord
from roboci_loop.teleop import PoseTeleop, PoseTeleopConfig


@dataclass
class SideControl:
    name: str
    controller: RobotController
    teleop: PoseTeleop
    hand: object
    home: np.ndarray
    settings: dict
    last_feedback: float = field(default_factory=time.monotonic)
    home_started: float = 0.
    reset_down: bool = False
    first_down: bool = False
    faulted: bool = False
    hand_target: np.ndarray | None = None
    trigger_anchor: np.ndarray | None = None
    trigger_reference: float | None = None
    last_hand_update: float = field(default_factory=time.monotonic)

    def __post_init__(self):
        limits = self.hand.joint_limits
        for key in ('hand_open_deg', 'hand_closed_deg', 'hand_first_deg'):
            values = np.deg2rad(self.settings[key])
            if (values.shape != (len(limits),) or not np.isfinite(values).all()
                    or np.any(values < limits[:, 0]) or np.any(values > limits[:, 1])):
                raise ValueError(f'{self.name}: {key} 超出关节限位或数量不匹配')
        self.hand_limits = limits.copy()
        speeds = np.asarray(self.settings['hand_thumb_speed_deg_s'], dtype=float)
        if speeds.shape != (2,) or not np.isfinite(speeds).all() or np.any(speeds <= 0):
            raise ValueError('hand_thumb_speed_deg_s 必须是两个有限正数')
        deadzone = self.settings['hand_axis_deadzone']
        if not np.isfinite(deadzone) or not 0 <= deadzone < 1:
            raise ValueError('hand_axis_deadzone 必须在 [0, 1) 范围内')

    def hold(self):
        self.teleop.stop()
        self.controller.set_mode(ControlMode.HOLD)

    def request_home(self):
        self.teleop.stop()
        self.hand_target = np.zeros(6)
        self.trigger_anchor = self.hand_target.copy()
        self.trigger_reference = None
        target = ROBOT_DATA(time_stamp=time.time(),
            arms=[ARM_DATA(self.name, joints=JOINT_DATA(position=self.home))],
            end_effectors=[END_EFFECTOR_DATA(self.name, joints=JOINT_DATA(position=self.hand_target.copy()))])
        if self.controller.submit_init(target, end_effector_tolerance=np.deg2rad(
                self.settings.get('hand_home_tolerance_deg', 2.))):
            self.home_started = time.monotonic()
            return target

    def request_first_pose(self, trigger):
        """Primary/First 只移动灵巧手，并以该姿态建立后续控制基准。"""
        self.teleop.stop()
        self.hand_target = np.deg2rad(self.settings['hand_first_deg'])
        self.trigger_anchor = self.hand_target.copy()
        self.trigger_reference = float(np.clip(trigger, 0, 1))
        target = ROBOT_DATA(time_stamp=time.time(), end_effectors=[
            END_EFFECTOR_DATA(self.name, joints=JOINT_DATA(position=self.hand_target.copy()))
        ])
        if self.controller.submit_init(
                target, end_effector_tolerance=np.deg2rad(
                    self.settings.get('hand_home_tolerance_deg', 2.))):
            self.home_started = time.monotonic()
            return target

    @staticmethod
    def _axis(value, deadzone):
        if abs(value) <= deadzone:
            return 0.0
        return np.sign(value) * (abs(value) - deadzone) / (1.0 - deadzone)

    def hand_positions(self, control, feedback, now):
        if self.hand_target is None:
            if feedback.end_effectors:
                self.hand_target = feedback.end_effectors[0].joints.position.copy()
            else:
                self.hand_target = np.deg2rad(self.settings['hand_open_deg'])
        if self.trigger_anchor is None or self.trigger_reference is None:
            self.trigger_anchor = self.hand_target.copy()
            self.trigger_reference = float(np.clip(control.trigger, 0, 1))
        dt = min(max(now - self.last_hand_update, 0.0), 0.1)
        self.last_hand_update = now
        deadzone = self.settings['hand_axis_deadzone']
        axis_x = self._axis(float(control.axis_x), deadzone)
        axis_y = self._axis(float(control.axis_y), deadzone)
        rotation_speed, flex_speed = np.deg2rad(self.settings['hand_thumb_speed_deg_s'])
        # 左右手旋转镜像：右手摇杆 +X 让 thumb_rotation 朝 0° 走。
        rotation_sign = 1.0 if self.name == 'left' else -1.0
        self.hand_target[0] += rotation_sign * axis_x * rotation_speed * dt
        self.hand_target[5] += axis_y * flex_speed * dt
        opened, closed = (np.deg2rad(self.settings[key])
                          for key in ('hand_open_deg', 'hand_closed_deg'))
        trigger_delta = float(np.clip(control.trigger, 0, 1)) - self.trigger_reference
        self.hand_target[1:5] = (self.trigger_anchor[1:5]
            + trigger_delta * (closed[1:5] - opened[1:5]))
        self.hand_target = np.clip(
            self.hand_target, self.hand_limits[:, 0], self.hand_limits[:, 1]
        )
        return self.hand_target.copy()

    def update(self, tracker, now, last_input, feedback):
        if feedback.arms:
            self.last_feedback = now
        if self.controller.mode == ControlMode.INIT:
            if now - self.home_started > 60.:
                raise TimeoutError(f'{self.name} 初始化超时: {self.controller.initialization_status()}')
            return
        if (now - self.last_feedback > self.settings['feedback_timeout']
                or now - last_input > self.settings['input_timeout']):
            self.hold()
            return
        if tracker is None:
            return
        control = getattr(tracker, self.name)
        reset = control is not None and control.secondary_button
        first = control is not None and control.primary_button
        reset_pressed, self.reset_down = reset and not self.reset_down, reset
        first_pressed, self.first_down = first and not self.first_down, first
        if reset_pressed:
            return self.request_home()
        if first_pressed:
            return self.request_first_pose(control.trigger)
        if control is None or not np.isfinite(control.grip):
            self.hold()
            return
        if control.grip <= self.settings['grip_threshold']:
            self.faulted = False
            self.hold()
            return
        if self.faulted or not feedback.arms:
            return
        try:
            if (control.pose is None or not np.isfinite(control.trigger)
                    or not np.isfinite(control.axis_x) or not np.isfinite(control.axis_y)):
                raise ValueError('手柄位姿、trigger 或摇杆输入无效')
            if not self.teleop.is_running:
                self.teleop.start(feedback.arms[0].joints.position, control.pose)
            joints = self.teleop.compute(control.pose)
            if joints is None:
                raise RuntimeError('IK 单步求解失败')
        except (ValueError, RuntimeError) as exc:
            self.faulted = True
            self.hold()
            print(f'[{self.name}] 跟随停止: {exc}；松开 Grip 后重试', flush=True)
            return
        target = ROBOT_DATA(time_stamp=time.time(),
            arms=[ARM_DATA(self.name, joints=JOINT_DATA(position=joints))],
            end_effectors=[END_EFFECTOR_DATA(
                self.name, joints=JOINT_DATA(position=self.hand_positions(control, feedback, now)))])
        if self.controller.set_mode(ControlMode.TELEOP) and self.controller.submit_teleop(target):
            return target
        self.hold()


def stamp(header, seconds, frame_id=''):
    ns = round(seconds * 1_000_000_000)
    header.stamp.sec, header.stamp.nanosec = divmod(ns, 1_000_000_000)
    header.frame_id = frame_id


class PicoRos2(Node):
    def __init__(self, config, output_dir, prefix='/vr_teleop'):
        super().__init__('pico_rm65_teleop')
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
        return {'ok': True, 'message': '录制已保存，请提交质检', 'bag_directory': str(self.completed)}

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
        return {'ok': True, 'ready': self.ready,
                'overall': 'online' if self.ready and devices and all(d['online'] for d in devices.values()) else 'offline',
                'devices': devices, 'recording': self.recorder.is_recording,
                'recording_duration_sec': now-self.started_at if self.started_at is not None else 0.,
                'review_pending': self.completed is not None and self.review is None and not self.recorder.is_recording,
                'bag_directory': str(self.recorder.path or '')}

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
        """相机图像处理和发布"""
        # 先完成相机预览图像红处理
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


def run(config, output_dir, prefix):
    stop, errors = threading.Event(), queue.Queue()
    with ExitStack() as stack:
        node = PicoRos2(config, output_dir, prefix)
        stack.callback(node.destroy_node)
        stack.callback(node.recorder.stop)
        arms, hands, sides, threads = {}, {}, {}, []
        # 初始化控制器
        for builder, specs, devices in ((build_end_effector, config['end_effectors'], hands),
                                        (build_arm, config['arms'], arms)):
            for name, spec in specs.items():
                device = builder(name, spec)
                stack.callback(device.stop)
                if isinstance(device, HandLG603):
                    # LG603 两只灵巧手共用一个CAN, 需要先解决通信问题
                    device.start(background_read=False)
                else:
                    device.start()
                devices[name] = device
            if devices is hands:
                time.sleep(.8)
                for hand in hands.values():
                    if isinstance(hand, HandLG603):
                        hand.start_feedback()
        # 初始化PICO
        pico = build_controller('pico', config['controllers']['pico'])
        stack.callback(pico.stop)
        pico.start()
        settings = config['teleop']
        hz = settings['control_hz']
        for name, home_deg in settings['home_deg'].items():
            home = np.deg2rad(home_deg)
            control = RobotController({name: arms[name]}, {name: hands[name]}, config=RobotControllerConfig(
                arm=DeviceControlConfig(control_hz=hz),
                end_effector=DeviceControlConfig(control_hz=settings['hand_control_hz'])))
            stack.callback(control.stop)
            control.start()
            # 主要用于求逆解
            teleop = PoseTeleop(build_ik(config['kinematics'][name]), reference_joints=home,
                config=PoseTeleopConfig(control_dt=1/hz, **settings['pose']))
            # 单边控制
            side = SideControl(name, control, teleop, hands[name], home, settings)
            sides[name] = side
            events = RobotEvents(control.latest_state())
            state_ik = build_ik(config['kinematics'][name])
            target_ik = build_ik(config['kinematics'][name])
            side.publish_target = lambda data, events=events, ik=target_ik: node.robot(data, 'target', events, ik)
            def feedback(data, events=events, ik=state_ik):
                try:
                    node.robot(data, 'state', events, ik)
                except Exception as exc:
                    errors.put(exc)
                    stop.set()
            control.set_feedback_callback(feedback)
            startup = control.latest_state()
            startup.time_stamp = time.time()
            side.publish_target(startup)
        def capture(camera):
            try:
                while not stop.is_set():
                    frame = camera.read(timeout_ms=100)
                    if frame is not None:
                        node.camera(frame)
            except Exception as exc:
                errors.put(exc)
                stop.set()
        def join_workers():
            node.ready = False
            stop.set()
            for thread in threads:
                thread.join()
        camera_stack = stack.enter_context(ExitStack())
        stack.callback(join_workers)
        # 打开所有的相机线程
        for name, spec in config['cameras'].items():
            camera = build_camera(name, spec)
            camera_stack.callback(camera.stop)
            camera.start()
            thread = threading.Thread(target=capture, args=(camera,), name=f'ros-camera-{name}')
            threads.append(thread)
            thread.start()
        if settings['home_on_start']:
            for side in sides.values():
                side.publish_target(side.request_home())
        node.ready = True
        node.get_logger().info(f'就绪，Web 通过 {node.prefix}/dashboard 控制 MCAP 录制；Grip 跟随，Y/B 回初始姿态')
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name='teleop-services', daemon=True)
        spin_thread.start()
        def stop_services():
            executor.shutdown()
            spin_thread.join()
        stack.callback(stop_services)
        for side in sides.values():
            stack.callback(side.controller.stop)
        last_input = time.monotonic()
        while rclpy.ok():
            started = time.monotonic()
            if not errors.empty():
                raise RuntimeError('设备发布失败') from errors.get_nowait()
            node.recorder.check_error()
            tracker = pico.read()
            if tracker is not None:
                with node.lock:
                    node.samples_input.append(started)
                last_input = started
            for side in sides.values():
                side.controller.check_error()
                target = side.update(tracker, started, last_input, side.controller.read())
                side.publish_target(target)
            stop.wait(max(0., 1/hz - (time.monotonic() - started)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[2] / 'configs/record_pico.py')
    parser.add_argument('--output-dir', default='~/datasets')
    parser.add_argument('--prefix', default='/vr_teleop')
    args, ros_args = parser.parse_known_args()
    config = runpy.run_path(str(args.config.expanduser().resolve()))['CONFIG']
    rclpy.init(args=ros_args)
    try:
        run(config, args.output_dir, args.prefix)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
