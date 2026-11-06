"""按 ROS 节点操作方式：同侧 Grip 跟随，左 Y/右 B 回初始姿态。"""
from contextlib import ExitStack
from dataclasses import dataclass, field
import time

from loguru import logger
import numpy as np

from roboci_loop.common import ARM_DATA, END_EFFECTOR_DATA, JOINT_DATA, ROBOT_DATA
from roboci_loop.controller import ControlMode, DeviceControlConfig, RobotController, RobotControllerConfig
from roboci_loop.hardware.arm import ArmConfig, ArmRM65
from roboci_loop.hardware.controller import ControllerConfig, ControllerPico
from roboci_loop.hardware.end_effector import EndEffectorConfig, HandLG603
from roboci_loop.kinematics import RM65IK, RM65IKConfig
from roboci_loop.teleop import PoseTeleop, PoseTeleopConfig


LOCAL_IP = "192.168.1.17"
ENABLE_HANDS = True  # 显式使能，不校准或归零
HOME_ON_START = True  # 启动后先用普通 movej 回初始姿态
CONTROL_HZ = 50
FEEDBACK_TIMEOUT = .5  # 外部判断机械臂反馈中断，s
INPUT_TIMEOUT = .2  # PICO 输入超时，s
CONTROL_ORIENTATION = True
GRIP_THRESHOLD = .9
PICO_TO_WORLD = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
HAND_OPEN = np.deg2rad(np.full(6, 10.))
HAND_CLOSED = np.deg2rad(np.full(6, 60.))
ARMS = {
    "left": ("192.168.1.18", (0., 90., 90.), [90., 40., 55., 0., 50., -15.]),
    "right": ("192.168.1.19", (90., 90., 0.), [90., -40., -55., 0., -50., 15.]),
}


@dataclass
class SideControl:
    """单侧的设备和操作状态，左右两侧互不影响。"""
    name: str
    controller: RobotController
    teleop: PoseTeleop
    hand: HandLG603
    home: np.ndarray
    last_feedback: float = field(default_factory=time.monotonic)
    home_started: float = 0.
    reset_down: bool = False
    tracking_faulted: bool = False
    faulted: bool = False

    def hold(self):
        self.teleop.stop()
        self.controller.set_mode(ControlMode.HOLD)

    def request_home(self):
        self.teleop.stop()
        target = ROBOT_DATA(
            time_stamp=time.time(), 
            arms=[ARM_DATA(self.name, joints=JOINT_DATA(position=self.home))],
            end_effectors=[END_EFFECTOR_DATA(self.name, joints=JOINT_DATA(position=[0,0,0,0,0,0]))])
        if self.controller.submit_init(target):
            self.home_started = time.monotonic()
            return target

    def update(self, tracker, now, last_input, *, feedback=None):
        """更新单侧操作；返回本轮成功提交的目标，供录制示例保存。"""
        if self.controller.mode == ControlMode.INIT:
            if now - self.home_started > 60.:
                raise TimeoutError("初始化运动超时")
            return
        if feedback is None:
            feedback = self.controller.read()
        if feedback.arms:
            self.last_feedback = now
        if now - self.last_feedback > FEEDBACK_TIMEOUT or now - last_input > INPUT_TIMEOUT:
            self.hold()
            return
        if tracker is None:
            return
        hand_input = getattr(tracker, self.name)
        reset = hand_input is not None and hand_input.secondary_button
        pressed, self.reset_down = reset and not self.reset_down, reset
        if pressed:
            return self.request_home()
        if hand_input is None or not np.isfinite(hand_input.grip):
            self.hold()
            return
        if hand_input.grip <= GRIP_THRESHOLD:
            self.tracking_faulted = False
            self.hold()
            return
        if self.tracking_faulted or not feedback.arms:
            return
        try:
            if hand_input.pose is None or not np.isfinite(hand_input.trigger):
                raise ValueError("手柄位姿或 trigger 无效")
            if not self.teleop.is_running:
                self.teleop.start(feedback.arms[0].joints.position, hand_input.pose)
            joints = self.teleop.compute(hand_input.pose)
            if joints is None:
                raise RuntimeError("单步 QP 求解失败")
        except (ValueError, RuntimeError) as exc:
            logger.warning("{} 跟随停止：{}；松开 Grip 后重试", self.name, exc)
            self.tracking_faulted = True
            self.hold()
            return
        fraction = float(np.clip(hand_input.trigger, 0, 1))
        limits = self.hand.joint_limits
        hand_target = np.clip(HAND_OPEN + fraction * (HAND_CLOSED - HAND_OPEN),
                              limits[:, 0], limits[:, 1])
        target = ROBOT_DATA(time_stamp=time.time(),
            arms=[ARM_DATA(self.name, joints=JOINT_DATA(position=joints))],
            end_effectors=[END_EFFECTOR_DATA(self.name, joints=JOINT_DATA(position=hand_target))])
        self.controller.set_mode(ControlMode.TELEOP)
        if self.controller.submit_teleop(target):
            return target
        self.hold()


def main():
    with ExitStack() as stack:
        arms, hands, sides = {}, {}, {}
        # 连接灵巧手
        for side, channel, can_id in (("left", 0, 0x65), ("right", 1, 0x64)):
            hand = HandLG603(EndEffectorConfig(side), channel=channel, device_can_id=can_id)
            stack.callback(hand.stop)
            hand.start()
            hands[side] = hand
        time.sleep(1)
        # 连接机械臂
        for side, (ip, installation, home) in ARMS.items():
            arm = ArmRM65(ArmConfig(side), ip=ip, udp_ip=LOCAL_IP, can_fd=True)
            stack.callback(arm.stop)
            arm.start()
            arms[side] = arm
        # 连接PICO
        pico = stack.enter_context(ControllerPico(ControllerConfig("pico")))
        if ENABLE_HANDS:
            for hand in hands.values():
                hand.enable(True)
        # 每侧一个 controller，使 Y/B 的 INIT 不占用另一侧的控制模式。
        for side, (_, installation, home) in ARMS.items():
            ik = RM65IK(RM65IKConfig(base_rpy_deg=installation))
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
                side.request_home()
        logger.info("同侧 Grip > 0.9 按住跟随，松开保持；左 Y/右 B 回初始姿态；trigger 控制手指")
        last_input = last_print = time.monotonic()
        while True:
            started = time.monotonic()
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
                    side.update(tracker, started, last_input)
                except Exception:
                    logger.exception("{} 侧停止；另一侧继续运行", side.name)
                    side.teleop.stop()
                    side.controller.stop()
                    side.faulted = True
            if started - last_print >= 1:
                logger.info("模式：{}", {name: ('FAULT' if side.faulted else side.controller.mode.name)
                                         for name, side in sides.items()})
                last_print = started
            time.sleep(max(0, 1 / CONTROL_HZ - (time.monotonic() - started)))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logger.info("遥操作已退出")
