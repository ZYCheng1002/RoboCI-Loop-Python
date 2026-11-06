"""独立组织 PICO 遥操作与记录，保存图像、反馈和提交给 controller 的目标。"""
from contextlib import ExitStack
import time
import threading

from loguru import logger
import numpy as np

from roboci_loop.controller import DeviceControlConfig, RobotController, RobotControllerConfig
from roboci_loop.hardware.arm import ArmConfig, ArmRM65
from roboci_loop.hardware.camera import CameraConfig, CameraOrbbec
from roboci_loop.recording import Recorder, RecorderConfig
from roboci_loop.hardware.controller import ControllerConfig, ControllerPico
from roboci_loop.hardware.end_effector import EndEffectorConfig, HandLG603
from roboci_loop.kinematics import RM65IK, RM65IKConfig
from roboci_loop.teleop import PoseTeleop, PoseTeleopConfig
from teleop_pico_example import SideControl


RECORD_DEPTH = False  # 默认关闭深度采集和保存
FPS = 30
CAMERAS = {
    "base": "CV27864000L2",
    "left_wrist": "CV2C861000N1",
    "right_wrist": "CV278640008T",
}
LOCAL_IP = "192.168.1.17"
ENABLE_HANDS = True  # 显式使能，不校准或归零
HOME_ON_START = True  # 与 ROS 节点一致，启动后先用普通 movej 回初始姿态
CONTROL_HZ = 50
CONTROL_ORIENTATION = True
PICO_TO_WORLD = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
ARMS = {
    "left": ("192.168.1.18", (0., 90., 90.), [90., 40., 55., 0., 50., -15.]),
    "right": ("192.168.1.19", (90., 90., 0.), [90., -40., -55., 0., -50., 15.]),
}


def main():
    stop = threading.Event()
    camera_errors = []
    with Recorder(RecorderConfig(record_depth=RECORD_DEPTH, fps=FPS)) as recorder, ExitStack() as stack:
        logger.info("记录目录：{}", recorder.output_dir)
        arms, hands, sides, pose_iks = {}, {}, {}, {}

        def capture(name, serial):
            try:
                with CameraOrbbec(CameraConfig(name, FPS, 1280, 720),
                                  serial_number=serial, enable_depth=RECORD_DEPTH) as camera:
                    while not stop.is_set():
                        frame = camera.read()
                        if frame is not None:
                            recorder.record_camera(frame)
            except Exception as exc:
                camera_errors.append(exc)
                stop.set()

        threads = []
        def stop_cameras():
            stop.set()
            for thread in threads:
                if thread.ident is not None:
                    thread.join()
        stack.callback(stop_cameras)
        for name, serial in CAMERAS.items():
            thread = threading.Thread(target=capture, args=(name, serial), name=f"capture-{name}")
            threads.append(thread)
            thread.start()

        def record_target(side, target):
            if target is None:
                return
            # 使用独立 FK，不改写遥操作跟踪中的模型。
            for item in (*target.arms, *target.end_effectors):
                item.time_stamp = target.time_stamp
            for arm in target.arms:
                arm.end_pose = pose_iks[side.name].forward(arm.joints.position)
            recorder.record_robot(target, kind="target", metadata=dict(
                side=side.name, mode=side.controller.mode.name, stage="submitted"))

        for side, channel, can_id in (("left", 0, 0x65), ("right", 1, 0x64)):
            hand = HandLG603(EndEffectorConfig(side), channel=channel, device_can_id=can_id)
            stack.callback(hand.stop)
            hand.start()
            hands[side] = hand
        time.sleep(3)
        for side, (ip, installation, home) in ARMS.items():
            arm = ArmRM65(ArmConfig(side), ip=ip, udp_ip=LOCAL_IP, can_fd=True)
            stack.callback(arm.stop)
            arm.start()
            arms[side] = arm
        pico = stack.enter_context(ControllerPico(ControllerConfig("pico")))
        if ENABLE_HANDS:
            for hand in hands.values():
                hand.enable(True)
        # 每侧一个 controller，使 Y/B 的 INIT 不占用另一侧的控制模式。
        for side, (_, installation, home) in ARMS.items():
            ik = RM65IK(RM65IKConfig(base_rpy_deg=installation))
            pose_iks[side] = RM65IK(RM65IKConfig(base_rpy_deg=installation))
            teleop = PoseTeleop(
                ik, reference_joints=np.deg2rad(home),
                config=PoseTeleopConfig(control_dt=1 / CONTROL_HZ,
                                       control_orientation=CONTROL_ORIENTATION,
                                       input_to_world=PICO_TO_WORLD),
            )
            controller = RobotController(
                {side: arms[side]}, {side: hands[side]}, config=RobotControllerConfig(
                    arm=DeviceControlConfig(
                        control_hz=CONTROL_HZ),
                ),
            )
            stack.callback(controller.stop)
            controller.start()
            sides[side] = SideControl(side, controller, teleop, hands[side], np.deg2rad(home))
        if HOME_ON_START:
            for side in sides.values():
                record_target(side, side.request_home())
        logger.info("同侧 Grip > 0.9 按住跟随，松开保持；左 Y/右 B 回初始姿态；trigger 控制手指")
        last_input = last_print = time.monotonic()
        while True:
            started = time.monotonic()
            recorder.check_error()
            if camera_errors:
                raise RuntimeError("相机采集失败") from camera_errors[0]
            tracker = pico.read()
            if tracker is not None:
                if not np.isfinite(tracker.time_stamp) or tracker.time_stamp <= 0:
                    tracker = None
                else:
                    last_input = started
            for side in sides.values():
                if side.faulted:
                    continue
                try:
                    # 反馈只消费一次，同一份用于记录和遥操作；INIT 时也持续记录。
                    feedback = side.controller.read()
                    recorder.record_robot(feedback, kind="state", metadata=dict(
                        side=side.name, mode=side.controller.mode.name))
                    target = side.update(tracker, started, last_input, feedback=feedback)
                except Exception:
                    logger.exception("{} 侧停止；另一侧继续运行", side.name)
                    side.teleop.stop()
                    side.controller.stop()
                    side.faulted = True
                    continue
                record_target(side, target)
            if started - last_print >= 1:
                logger.info("模式：{}", {name: ('FAULT' if side.faulted else side.controller.mode.name)
                                         for name, side in sides.items()})
                last_print = started
            time.sleep(max(0, 1 / CONTROL_HZ - (time.monotonic() - started)))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logger.info("遥操作已退出，记录已刷新保存")
