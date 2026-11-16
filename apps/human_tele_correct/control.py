"""OpenPI 推理、PICO 人工纠正、轨迹回退与数据采集后端。"""
import argparse
import copy
from contextlib import ExitStack
from pathlib import Path
import queue
import runpy
import threading
import time

import numpy as np
import rclpy
from inference_runtime import InferenceWorker, split_side, takeover_requested
from dashboard_node import CAMERA_KEYS, PicoRos2
from side_control import SideControl

from roboci_loop.common import ROBOT_DATA
from roboci_loop.controller import ControlMode, DeviceControlConfig, RobotController, RobotControllerConfig
from roboci_loop.factory import build_arm, build_camera, build_controller, build_end_effector, build_ik
from roboci_loop.hardware.end_effector import HandLG603
from roboci_loop.inference import InferenceLayout, JointGroup, OpenPIConfig, OpenPIPolicy
from roboci_loop.recording import RobotEvents
from roboci_loop.teleop import PoseTeleop, PoseTeleopConfig


def run(config, output_dir, prefix):
    stop, errors = threading.Event(), queue.Queue()
    with ExitStack() as stack:
        node = PicoRos2(config, output_dir, prefix)
        stack.callback(node.destroy_node)
        stack.callback(node.recorder.stop)
        layout = InferenceLayout((
            JointGroup('arm', 'left', 6),
            JointGroup('end_effector', 'left', 6),
            JointGroup('arm', 'right', 6),
            JointGroup('end_effector', 'right', 6),
        ), CAMERA_KEYS)
        inference = config['inference']
        if (not isinstance(inference.get('prompts'), list)
                or not inference['prompts']
                or any(not isinstance(prompt, str) or not prompt.strip()
                       for prompt in inference['prompts'])):
            raise ValueError('inference.prompts 必须是非空字符串列表')
        if (not np.isfinite(inference.get('rollback_hz', 0))
                or inference['rollback_hz'] <= 0
                or inference['rollback_hz'] > inference['control_hz']):
            raise ValueError('rollback_hz 必须为不高于 control_hz 的有限正数')
        policy = OpenPIPolicy(OpenPIConfig(
            url=inference['server'],
            action_horizon=inference['action_horizon'],
            broker=inference['broker'],
            control_hz=inference['control_hz'],
            trigger_fraction=inference['trigger_fraction'],
            guidance_steps=inference['guidance_steps'],
        ), layout.action_dim)
        def close_inference():
            if node.inference_worker is not None:
                node.inference_worker.close()
            policy.close()
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
                        stack.callback(hand.stop_feedback)
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

        controllers = {name: side.controller for name, side in sides.items()}
        hand_limits = {name: hand.joint_limits for name, hand in hands.items()}

        def start_inference(prompt, task=None):
            with node.lock:
                if node.inference_state != 'idle':
                    raise RuntimeError('请先结束并评价上一轮推理')
                if prompt not in inference['prompts']:
                    raise ValueError('请选择配置中的 prompt')
                task = str(task or '').strip()
                invalid = '/\\<>:"|?*'
                if task and (len(task) > 64 or task in ('.', '..') or task.endswith('.')
                        or any(c in invalid or ord(c) < 32 for c in task)):
                    raise ValueError('Task 最多 64 个字符，且不能包含路径特殊字符')
                stats_task = task or '_inference'
                if node.inference_worker is not None and node.inference_worker.thread.is_alive():
                    raise RuntimeError('上一轮推理请求仍在退出，请稍后重试')
                for name, controller in controllers.items():
                    if not controller.set_mode(ControlMode.AUTO):
                        raise RuntimeError(f'{name} 无法切换到 AUTO 模式')
                node.inference_worker = InferenceWorker(
                    policy, layout, controllers, node.camera_snapshot, prompt, inference,
                )
                node.active_prompt = prompt
                node.active_task = stats_task
                node.inference_outcome = None
                node.intervened = False
                node.rollback_requires_release = False
                node.trajectory.clear()
                node.chunk_start_index = 0
                node.chunk_start_target = None
                node.inference_state = 'running'
                node.inference_worker.start()
                return {'ok': True, 'message': '推理已开始'}

        def stop_inference():
            with node.lock:
                if node.inference_state != 'running':
                    raise RuntimeError('当前没有可结束的推理')
                if node.inference_worker is not None:
                    node.inference_worker.stop()
                for controller in controllers.values():
                    controller.set_mode(ControlMode.HOLD)
                node.inference_state = 'awaiting_review'
                return {'ok': True, 'message': '推理已结束，请人工提交成功或失败'}

        def abort_inference():
            with node.lock:
                if node.inference_state != 'running':
                    raise RuntimeError('当前没有可中断的推理')
                if node.inference_worker is not None:
                    node.inference_worker.stop()
                for controller in controllers.values():
                    controller.set_mode(ControlMode.HOLD)
                # 网页中断与 Grip 接管使用相同的实测状态基准：清除推理目标、
                # PICO 位姿参考和 Trigger 锚点，避免首次接管时手指跳到默认张开位。
                for side in sides.values():
                    side.reset_teleop_session()
                node.inference_state = 'awaiting_review'
                node.inference_outcome = None
                node.intervened = True
                node.rollback_requires_release = False
                return {
                    'ok': True,
                    'message': '推理已中断，请人工选择推理成功或失败；也可人工接管或回退当前 chunk',
                }

        def review_inference(success):
            if type(success) is not bool:
                raise ValueError('推理结果必须为成功或失败')
            with node.lock:
                if node.inference_state != 'awaiting_review':
                    raise RuntimeError('当前没有待评价的推理')
                node.inference_outcome = 'success' if success else 'failure'
                node.save_inference_result(success)
                node.inference_state = 'idle'
                return {'ok': True, 'message': '推理结果已记为' + ('成功' if success else '失败')}

        def request_home():
            with node.lock:
                if node.inference_state == 'running' or node.rollback_active:
                    raise RuntimeError('请先中断推理或等待回退完成')
                targets = []
                for side in sides.values():
                    target = side.request_home()
                    if target is None:
                        raise RuntimeError(f'{side.name} 无法进入初始化')
                    targets.append((side, target))
                node.intervened = False
                node.rollback_requires_release = False
                for side, target in targets:
                    side.publish_target(target)
                return {'ok': True, 'message': '正在回到初始化位置'}

        def rollback():
            with node.lock:
                if not node.intervened:
                    raise RuntimeError('只有本轮人工接管后才能回退')
                if node.rollback_active:
                    raise RuntimeError('轨迹正在回退')
                if not node.trajectory:
                    raise RuntimeError('本轮没有可回退的推理轨迹')
                trajectory = [
                    copy.deepcopy(target)
                    for target in node.trajectory[node.chunk_start_index:][::-1]
                ]
                if node.chunk_start_target is not None:
                    trajectory.append(copy.deepcopy(node.chunk_start_target))
                node.rollback_active = True
                node.rollback_requires_release = True

            def execute():
                try:
                    for side in sides.values():
                        side.teleop.stop()
                    for controller in controllers.values():
                        if not controller.set_mode(ControlMode.AUTO):
                            raise RuntimeError('无法进入轨迹回退模式')
                    period = 1 / inference['rollback_hz']
                    for target in trajectory:
                        if stop.is_set():
                            break
                        for name, controller in controllers.items():
                            side_target = split_side(target, name)
                            if not controller.submit_inference(side_target):
                                raise RuntimeError(f'{name} 回退目标提交失败')
                            sides[name].publish_target(side_target)
                        stop.wait(period)
                    for controller in controllers.values():
                        controller.set_mode(ControlMode.HOLD)
                    # 给控制线程一个周期消费 AUTO -> HOLD，随后从最新实测状态
                    # 重建遥操作和灵巧手参考，避免恢复旧 TELEOP 目标。
                    stop.wait(period)
                    for side in sides.values():
                        side.reset_teleop_session()
                except Exception as exc:
                    errors.put(exc)
                    stop.set()
                finally:
                    with node.lock:
                        node.rollback_active = False

            thread = threading.Thread(target=execute, name='trajectory-rollback')
            threads.append(thread)
            thread.start()
            return {
                'ok': True,
                'message': f'开始回退当前 chunk 的 {len(trajectory)} 个轨迹点；完成后请先松开 Grip',
            }

        node.start_inference_callback = start_inference
        node.stop_inference_callback = stop_inference
        node.abort_inference_callback = abort_inference
        node.review_inference_callback = review_inference
        node.home_callback = request_home
        node.rollback_callback = rollback
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
        node.get_logger().info(
            f'就绪，Web 通过 {node.prefix}/dashboard 控制录制、推理和轨迹回退'
        )
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
        # 必须先等待推理线程退出，再关闭 controller、相机和设备。
        stack.callback(close_inference)
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
            with node.lock:
                state = node.inference_state
                intervened = node.intervened
                rollback_active = node.rollback_active
                worker = node.inference_worker
            if (state == 'running'
                    and takeover_requested(tracker, settings['grip_threshold'])):
                with node.lock:
                    node.intervened = True
                    node.inference_state = 'awaiting_review'
                    node.inference_worker.stop()
                for controller in controllers.values():
                    controller.set_mode(ControlMode.HOLD)
                for side in sides.values():
                    side.reset_teleop_session()
                state, intervened = 'awaiting_review', True
                node.get_logger().warning('Grip 触发人工接管：推理已中断，等待人工评价')
            if state == 'running' and worker is not None:
                if not worker.errors.empty():
                    raise RuntimeError('OpenPI 推理失败') from worker.errors.get_nowait()
                try:
                    action = worker.results.get_nowait()
                except queue.Empty:
                    action = None
                if action is not None:
                    target = layout.target(action)
                    for item in target.end_effectors:
                        limits = hand_limits[item.end_effector_name]
                        item.joints.position = np.clip(
                            item.joints.position, limits[:, 0], limits[:, 1]
                        )
                    with node.lock:
                        if len(node.trajectory) % inference['action_horizon'] == 0:
                            node.chunk_start_index = len(node.trajectory)
                            if node.trajectory:
                                node.chunk_start_target = copy.deepcopy(node.trajectory[-1])
                            else:
                                start_target = ROBOT_DATA(time_stamp=time.time())
                                for controller in controllers.values():
                                    state = controller.latest_state()
                                    start_target.arms.extend(state.arms)
                                    start_target.end_effectors.extend(state.end_effectors)
                                node.chunk_start_target = start_target
                        node.trajectory.append(copy.deepcopy(target))
                    for name, controller in controllers.items():
                        side_target = split_side(target, name)
                        if not controller.submit_inference(side_target):
                            raise RuntimeError(f'{name} 推理目标提交失败')
                        sides[name].publish_target(side_target)
            elif intervened and not rollback_active:
                if node.rollback_requires_release:
                    if tracker is not None and not takeover_requested(
                            tracker, settings['grip_threshold']):
                        with node.lock:
                            node.rollback_requires_release = False
                    stop.wait(max(0., 1/hz - (time.monotonic() - started)))
                    continue
                for side in sides.values():
                    target = side.update(
                        tracker, started, last_input, side.controller.read()
                    )
                    side.publish_target(target)
            stop.wait(max(0., 1/hz - (time.monotonic() - started)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[2] / 'configs/human_tele_correct.py')
    parser.add_argument('--output-dir', default='~/datasets')
    parser.add_argument('--prefix', default='/human_tele_correct')
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
