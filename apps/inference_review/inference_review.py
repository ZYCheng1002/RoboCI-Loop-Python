"""OpenPI 双臂推理与 PICO 人工接管。

推理期间持续监听 PICO；任一侧 Grip 达到阈值后永久停止推理并切换为遥操作。
人工接管一旦发生，本次运行不会恢复自动推理。
"""

import argparse
from contextlib import ExitStack
from dataclasses import dataclass, field
import faulthandler
from pathlib import Path
import queue
import runpy
import threading
import time

from loguru import logger
import numpy as np

from roboci_loop.common import ARM_DATA, CAM_MSG, END_EFFECTOR_DATA, JOINT_DATA, ROBOT_DATA
from roboci_loop.controller import (
    ControlMode, DeviceControlConfig, RobotController, RobotControllerConfig,
)
from roboci_loop.factory import (
    build_arm, build_camera, build_controller, build_end_effector, build_ik,
)
from roboci_loop.hardware.end_effector import HandLG603
from roboci_loop.inference import InferenceLayout, JointGroup, OpenPIConfig, OpenPIPolicy
from roboci_loop.teleop import PoseTeleop, PoseTeleopConfig


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[2] / "configs" / "inference_review_config.py"
)
CAMERA_KEYS = {
    "observation/image": "base",
    "observation/left_wrist_image": "left_wrist",
    "observation/right_wrist_image": "right_wrist",
}


class CameraCollector:
    """每台相机一个读取线程，向推理线程提供各相机最新帧。"""

    def __init__(self, cameras) -> None:
        self.cameras = dict(cameras)
        self._frames = {}
        self._error = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []

    def start(self) -> None:
        for name, camera in self.cameras.items():
            thread = threading.Thread(
                target=self._capture, args=(name, camera), name=f"review-camera-{name}"
            )
            thread.start()
            self._threads.append(thread)

    def _capture(self, name, camera) -> None:
        try:
            while not self._stop.is_set():
                frame = camera.read(timeout_ms=100)
                if frame is not None:
                    with self._lock:
                        self._frames[name] = frame
        except Exception as exc:
            with self._lock:
                if self._error is None:
                    self._error = exc
            self._stop.set()

    def snapshot(self, *, max_age: float, max_skew: float) -> CAM_MSG:
        now = time.time()
        with self._lock:
            if self._error is not None:
                raise RuntimeError("相机采集线程失败") from self._error
            frames = [self._frames.get(name) for name in self.cameras]
        missing = [name for name, frame in zip(self.cameras, frames) if frame is None]
        if missing:
            raise TimeoutError(f"相机尚未就绪：{', '.join(missing)}")
        stamps = [frame.time_stamp for frame in frames]
        if now - min(stamps) > max_age:
            raise TimeoutError("相机图像已过期")
        if max(stamps) - min(stamps) > max_skew:
            raise TimeoutError("相机图像时间差过大")
        return CAM_MSG(time_stamp=max(stamps), frames=frames)

    def close(self) -> None:
        self._stop.set()
        for thread in self._threads:
            if thread.ident is not None:
                thread.join()


@dataclass
class SideControl:
    """单侧 PICO 遥操作状态；实现与 pico_rm65_teleop 保持一致。"""

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

    def __post_init__(self) -> None:
        limits = self.hand.joint_limits
        for key in ("hand_open_deg", "hand_closed_deg", "hand_first_deg"):
            values = np.deg2rad(self.settings[key])
            if (values.shape != (len(limits),) or not np.isfinite(values).all()
                    or np.any(values < limits[:, 0]) or np.any(values > limits[:, 1])):
                raise ValueError(f"{self.name}: {key} 超出关节限位或数量不匹配")
        self.hand_limits = limits.copy()
        speeds = np.asarray(self.settings["hand_thumb_speed_deg_s"], dtype=float)
        if speeds.shape != (2,) or not np.isfinite(speeds).all() or np.any(speeds <= 0):
            raise ValueError("hand_thumb_speed_deg_s 必须是两个有限正数")
        deadzone = self.settings["hand_axis_deadzone"]
        if not np.isfinite(deadzone) or not 0 <= deadzone < 1:
            raise ValueError("hand_axis_deadzone 必须在 [0, 1) 范围内")

    def hold(self) -> None:
        self.teleop.stop()
        self.controller.set_mode(ControlMode.HOLD)

    def request_home(self):
        self.teleop.stop()
        self.hand_target = np.zeros(6)
        self.trigger_anchor = self.hand_target.copy()
        self.trigger_reference = None
        target = ROBOT_DATA(
            time_stamp=time.time(),
            arms=[ARM_DATA(self.name, joints=JOINT_DATA(position=self.home))],
            end_effectors=[END_EFFECTOR_DATA(
                self.name, joints=JOINT_DATA(position=self.hand_target.copy())
            )],
        )
        if self.controller.submit_init(
                target, end_effector_tolerance=np.deg2rad(
                    self.settings.get("hand_home_tolerance_deg", 2.))):
            self.home_started = time.monotonic()
            return target

    def request_first_pose(self, trigger):
        self.teleop.stop()
        self.hand_target = np.deg2rad(self.settings["hand_first_deg"])
        self.trigger_anchor = self.hand_target.copy()
        self.trigger_reference = float(np.clip(trigger, 0, 1))
        target = ROBOT_DATA(time_stamp=time.time(), end_effectors=[
            END_EFFECTOR_DATA(
                self.name, joints=JOINT_DATA(position=self.hand_target.copy())
            )
        ])
        if self.controller.submit_init(
                target, end_effector_tolerance=np.deg2rad(
                    self.settings.get("hand_home_tolerance_deg", 2.))):
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
                self.hand_target = np.deg2rad(self.settings["hand_open_deg"])
        if self.trigger_anchor is None or self.trigger_reference is None:
            self.trigger_anchor = self.hand_target.copy()
            self.trigger_reference = float(np.clip(control.trigger, 0, 1))
        dt = min(max(now - self.last_hand_update, 0.0), 0.1)
        self.last_hand_update = now
        deadzone = self.settings["hand_axis_deadzone"]
        axis_x = self._axis(float(control.axis_x), deadzone)
        axis_y = self._axis(float(control.axis_y), deadzone)
        rotation_speed, flex_speed = np.deg2rad(
            self.settings["hand_thumb_speed_deg_s"]
        )
        rotation_sign = 1.0 if self.name == "left" else -1.0
        self.hand_target[0] += rotation_sign * axis_x * rotation_speed * dt
        self.hand_target[5] += axis_y * flex_speed * dt
        opened, closed = (
            np.deg2rad(self.settings[key])
            for key in ("hand_open_deg", "hand_closed_deg")
        )
        trigger_delta = float(np.clip(control.trigger, 0, 1)) - self.trigger_reference
        self.hand_target[1:5] = (
            self.trigger_anchor[1:5]
            + trigger_delta * (closed[1:5] - opened[1:5])
        )
        self.hand_target = np.clip(
            self.hand_target, self.hand_limits[:, 0], self.hand_limits[:, 1]
        )
        return self.hand_target.copy()

    def update(self, tracker, now, last_input, feedback):
        if feedback.arms:
            self.last_feedback = now
        if self.controller.mode == ControlMode.INIT:
            if now - self.home_started > 60.:
                raise TimeoutError(
                    f"{self.name} 初始化超时: {self.controller.initialization_status()}"
                )
            return
        if (now - self.last_feedback > self.settings["feedback_timeout"]
                or now - last_input > self.settings["input_timeout"]):
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
        if control.grip <= self.settings["grip_threshold"]:
            self.faulted = False
            self.hold()
            return
        if self.faulted or not feedback.arms:
            return
        try:
            if (control.pose is None or not np.isfinite(control.trigger)
                    or not np.isfinite(control.axis_x) or not np.isfinite(control.axis_y)):
                raise ValueError("手柄位姿、trigger 或摇杆输入无效")
            if not self.teleop.is_running:
                self.teleop.start(feedback.arms[0].joints.position, control.pose)
            joints = self.teleop.compute(control.pose)
            if joints is None:
                raise RuntimeError("IK 单步求解失败")
        except (ValueError, RuntimeError) as exc:
            self.faulted = True
            self.hold()
            logger.warning("[{}] 跟随停止: {}；松开 Grip 后重试", self.name, exc)
            return
        target = ROBOT_DATA(
            time_stamp=time.time(),
            arms=[ARM_DATA(self.name, joints=JOINT_DATA(position=joints))],
            end_effectors=[END_EFFECTOR_DATA(
                self.name,
                joints=JOINT_DATA(position=self.hand_positions(control, feedback, now)),
            )],
        )
        if (self.controller.set_mode(ControlMode.TELEOP)
                and self.controller.submit_teleop(target)):
            return target
        self.hold()


class InferenceWorker:
    """后台执行可能阻塞的模型请求；人工接管后丢弃未完成结果。"""

    def __init__(self, policy, layout, controllers, collector, settings) -> None:
        self.policy = policy
        self.layout = layout
        self.controllers = controllers
        self.collector = collector
        self.settings = settings
        self.results = queue.Queue(maxsize=1)
        self.errors = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="openpi-inference")

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        period = 1.0 / self.settings["control_hz"]
        try:
            while not self.stop_event.is_set():
                started = time.monotonic()
                robot = ROBOT_DATA(time_stamp=time.time())
                for controller in self.controllers.values():
                    controller.check_error()
                    state = controller.latest_state()
                    robot.arms.extend(state.arms)
                    robot.end_effectors.extend(state.end_effectors)
                cameras = self.collector.snapshot(
                    max_age=self.settings["max_observation_age"],
                    max_skew=self.settings["max_camera_skew"],
                )
                action = self.policy.infer(
                    self.layout.observation(robot, cameras, self.settings["prompt"])
                )
                if self.stop_event.is_set():
                    break
                try:
                    self.results.put_nowait(action)
                except queue.Full:
                    try:
                        self.results.get_nowait()
                    except queue.Empty:
                        pass
                    self.results.put_nowait(action)
                self.stop_event.wait(max(0., period - (time.monotonic() - started)))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.errors.put(exc)

    def check_error(self) -> None:
        if not self.errors.empty():
            raise RuntimeError("OpenPI 推理线程失败") from self.errors.get_nowait()

    def stop(self) -> None:
        self.stop_event.set()

    def close(self) -> None:
        self.stop()
        if self.thread.ident is not None:
            self.thread.join()


def split_side(data: ROBOT_DATA, side: str) -> ROBOT_DATA:
    return ROBOT_DATA(
        time_stamp=data.time_stamp,
        arms=[item for item in data.arms if item.arm_name == side],
        end_effectors=[
            item for item in data.end_effectors if item.end_effector_name == side
        ],
    )


def takeover_requested(tracker, threshold: float) -> bool:
    if tracker is None:
        return False
    return any(
        control is not None and np.isfinite(control.grip) and control.grip > threshold
        for control in (tracker.left, tracker.right)
    )


def run(config) -> None:
    settings = config["teleop"]
    inference = config["inference"]
    layout = InferenceLayout((
        JointGroup("arm", "left", 6),
        JointGroup("end_effector", "left", 6),
        JointGroup("arm", "right", 6),
        JointGroup("end_effector", "right", 6),
    ), CAMERA_KEYS)
    policy_config = OpenPIConfig(
        url=inference["server"], action_horizon=inference["action_horizon"],
        broker=inference["broker"], control_hz=inference["control_hz"],
        trigger_fraction=inference["trigger_fraction"],
        guidance_steps=inference["guidance_steps"],
    )

    with ExitStack() as stack:
        policy = OpenPIPolicy(policy_config, layout.action_dim)
        stack.callback(policy.close)
        logger.info("模型服务 metadata：{}", policy.metadata)

        hands = {}
        for name, spec in config["end_effectors"].items():
            hand = build_end_effector(name, spec)
            stack.callback(hand.stop)
            if isinstance(hand, HandLG603):
                hand.start(background_read=False)
            else:
                hand.start()
            hands[name] = hand
        time.sleep(.8)
        for hand in hands.values():
            if isinstance(hand, HandLG603):
                hand.start_feedback()
                stack.callback(hand.stop_feedback)

        arms = {}
        for name, spec in config["arms"].items():
            arm = build_arm(name, spec)
            stack.callback(arm.stop)
            arm.start()
            arms[name] = arm

        pico = build_controller("pico", config["controllers"]["pico"])
        stack.callback(pico.stop)
        pico.start()

        cameras = {}
        for name in CAMERA_KEYS.values():
            camera = build_camera(name, config["cameras"][name])
            stack.callback(camera.stop)
            camera.start()
            cameras[name] = camera
        collector = CameraCollector(cameras)
        collector.start()
        stack.callback(collector.close)

        controllers, sides = {}, {}
        for name in ("left", "right"):
            controller = RobotController(
                {name: arms[name]}, {name: hands[name]},
                config=RobotControllerConfig(
                    arm=DeviceControlConfig(control_hz=settings["control_hz"]),
                    end_effector=DeviceControlConfig(
                        control_hz=settings["hand_control_hz"]
                    ),
                ),
            )
            stack.callback(controller.stop)
            controller.start()
            controllers[name] = controller
            home = np.deg2rad(settings["home_deg"][name])
            teleop = PoseTeleop(
                build_ik(config["kinematics"][name]), reference_joints=home,
                config=PoseTeleopConfig(
                    control_dt=1 / settings["control_hz"], **settings["pose"]
                ),
            )
            sides[name] = SideControl(
                name, controller, teleop, hands[name], home, settings
            )

        if not inference["skip_home"]:
            for side in sides.values():
                if side.request_home() is None:
                    raise RuntimeError(f"无法提交 {side.name} 初始姿态")
            deadline = time.monotonic() + inference["home_timeout"]
            while any(control.mode == ControlMode.INIT for control in controllers.values()):
                for control in controllers.values():
                    control.check_error()
                if time.monotonic() >= deadline:
                    status = {
                        name: control.initialization_status()
                        for name, control in controllers.items()
                    }
                    raise TimeoutError(f"初始化超时：{status}")
                time.sleep(.05)
            logger.info("机器人已到达初始姿态")
        for name, controller in controllers.items():
            if not controller.set_mode(ControlMode.AUTO):
                raise RuntimeError(f"{name} 无法切换到 AUTO 模式")

        camera_deadline = time.monotonic() + inference["camera_startup_timeout"]
        while True:
            try:
                collector.snapshot(
                    max_age=inference["max_observation_age"],
                    max_skew=inference["max_camera_skew"],
                )
                break
            except TimeoutError:
                if time.monotonic() >= camera_deadline:
                    raise
                time.sleep(.02)

        worker = InferenceWorker(policy, layout, controllers, collector, inference)
        worker.start()
        stack.callback(worker.close)
        hand_limits = {name: hand.joint_limits for name, hand in hands.items()}
        last_limit_warning = {}
        last_input = time.monotonic()
        takeover = False
        period = 1 / settings["control_hz"]
        logger.info("推理开始；按住任一 Grip 将永久切换至人工遥操作")

        while True:
            started = time.monotonic()
            tracker = pico.read()
            if tracker is not None:
                last_input = started
            if not takeover and takeover_requested(tracker, settings["grip_threshold"]):
                takeover = True
                worker.stop()
                for controller in controllers.values():
                    controller.set_mode(ControlMode.HOLD)
                logger.warning("检测到 Grip，人工接管已生效；本次运行不再恢复推理")

            for controller in controllers.values():
                controller.check_error()
            if takeover:
                for side in sides.values():
                    side.update(
                        tracker, started, last_input, side.controller.read()
                    )
            else:
                worker.check_error()
                try:
                    action = worker.results.get_nowait()
                except queue.Empty:
                    action = None
                if action is not None:
                    target = layout.target(action)
                    for item in target.end_effectors:
                        limits = hand_limits[item.end_effector_name]
                        original = item.joints.position.copy()
                        item.joints.position = np.clip(
                            original, limits[:, 0], limits[:, 1]
                        )
                        if not np.array_equal(original, item.joints.position):
                            now = time.monotonic()
                            name = item.end_effector_name
                            if now - last_limit_warning.get(name, float("-inf")) >= 5.:
                                logger.warning("{} 手模型目标已裁剪到硬件限位", name)
                                last_limit_warning[name] = now
                    for name, controller in controllers.items():
                        if not controller.submit_inference(split_side(target, name)):
                            raise RuntimeError(f"{name} 推理目标提交失败")
            time.sleep(max(0., period - (time.monotonic() - started)))


def main() -> None:
    faulthandler.enable(all_threads=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = runpy.run_path(str(args.config.expanduser().resolve()))["CONFIG"]
    try:
        run(config)
    except KeyboardInterrupt:
        logger.info("系统已停止，机器人进入 HOLD 并关闭设备")


if __name__ == "__main__":
    main()
