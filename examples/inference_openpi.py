"""使用 OpenPI 服务、三路相机和 RobotController 执行双臂推理。

模型状态/动作顺序固定为：左臂、左手、右臂、右手，共 24 维弧度。
启动前请确认配置中的 IP、CAN 通道、相机序列号及模型动作定义一致。

运行示例
串行模式:
python examples/inference_openpi.py \
  --server ws://127.0.0.1:8000 \
  --prompt "Grasp the black capacitor" \
  --broker serial

RTG模式:
python examples/inference_openpi.py \
  --server ws://127.0.0.1:8000 \
  --prompt "Grasp the black capacitor" \
  --broker rtg \
  --control-hz 10 \
  --action-horizon 30

基于当前状态:
python examples/inference_openpi.py \
  --prompt "Grasp the black capacitor" \
  --skip-home

"""

import argparse
from contextlib import ExitStack
import faulthandler
from pathlib import Path
import runpy
import threading
import time

from loguru import logger
import numpy as np

from roboci_loop.common import CAM_MSG, ROBOT_DATA
from roboci_loop.controller import (
    ControlMode, DeviceControlConfig, RobotController, RobotControllerConfig,
)
from roboci_loop.factory import build_arm, build_camera, build_end_effector
from roboci_loop.hardware.end_effector import HandLG603
from roboci_loop.inference import (
    InferenceLayout, JointGroup, OpenPIConfig, OpenPIPolicy,
)


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "configs" / "record_pico.py"
CAMERA_KEYS = {
    "observation/image": "base",
    "observation/left_wrist_image": "left_wrist",
    "observation/right_wrist_image": "right_wrist",
}


class CameraCollector:
    """每台相机一个读取线程，向控制循环提供各相机最新帧。"""

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
                target=self._capture, args=(name, camera), name=f"inference-camera-{name}"
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


def run(args) -> None:
    config = runpy.run_path(str(args.config.expanduser().resolve()))["CONFIG"]
    layout = InferenceLayout(
        (
            JointGroup("arm", "left", 6),
            JointGroup("end_effector", "left", 6),
            JointGroup("arm", "right", 6),
            JointGroup("end_effector", "right", 6),
        ),
        CAMERA_KEYS,
    )
    policy_config = OpenPIConfig(
        url=args.server, action_horizon=args.action_horizon,
        broker=args.broker, control_hz=args.control_hz,
        trigger_fraction=args.trigger_fraction, guidance_steps=args.guidance_steps,
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
        time.sleep(0.8)
        for hand in hands.values():
            if isinstance(hand, HandLG603):
                hand.start_feedback()
        hand_limits = {name: hand.joint_limits for name, hand in hands.items()}

        arms = {}
        for name, spec in config["arms"].items():
            arm = build_arm(name, spec)
            stack.callback(arm.stop)
            arm.start()
            arms[name] = arm

        cameras = {}
        for name in CAMERA_KEYS.values():
            camera = build_camera(name, config["cameras"][name])
            stack.callback(camera.stop)
            camera.start()
            cameras[name] = camera
        collector = CameraCollector(cameras)
        collector.start()
        stack.callback(collector.close)

        # controller 退出后先 join 手部反馈线程；句柄仍由原 stop callback 最终关闭。
        for hand in hands.values():
            if isinstance(hand, HandLG603):
                stack.callback(hand.stop_feedback)

        # 与现有遥操作案例一致：每侧一个 controller，故障和 INIT 不互相占用模式。
        controllers = {}
        for side in ("left", "right"):
            controller = RobotController(
                {side: arms[side]}, {side: hands[side]},
                config=RobotControllerConfig(
                    arm=DeviceControlConfig(control_hz=args.device_control_hz),
                    end_effector=DeviceControlConfig(control_hz=args.device_control_hz),
                ),
            )
            stack.callback(controller.stop)
            controller.start()
            controllers[side] = controller

        def split_side(data: ROBOT_DATA, side: str) -> ROBOT_DATA:
            return ROBOT_DATA(
                time_stamp=data.time_stamp,
                arms=[item for item in data.arms if item.arm_name == side],
                end_effectors=[item for item in data.end_effectors
                               if item.end_effector_name == side],
            )

        if not args.skip_home:
            homes = config["teleop"]["home_deg"]
            initial = layout.target(np.concatenate((
                np.deg2rad(homes["left"]), np.zeros(6),
                np.deg2rad(homes["right"]), np.zeros(6),
            )))
            for side, controller in controllers.items():
                if not controller.submit_init(split_side(initial, side)):
                    raise RuntimeError(f"无法提交 {side} 初始姿态")
            deadline = time.monotonic() + args.home_timeout
            while any(controller.mode == ControlMode.INIT
                      for controller in controllers.values()):
                for controller in controllers.values():
                    controller.check_error()
                if time.monotonic() >= deadline:
                    status = {side: controller.initialization_status()
                              for side, controller in controllers.items()}
                    raise TimeoutError(f"初始化超时：{status}")
                time.sleep(0.05)
            logger.info("机器人已到达初始姿态")
        for side, controller in controllers.items():
            if not controller.set_mode(ControlMode.AUTO):
                raise RuntimeError(f"{side} 无法切换到 AUTO 模式")

        # 相机线程与硬件初始化并行运行；进入推理前确认三路首帧均已就绪。
        camera_deadline = time.monotonic() + args.camera_startup_timeout
        while True:
            try:
                collector.snapshot(
                    max_age=args.max_observation_age,
                    max_skew=args.max_camera_skew,
                )
                break
            except TimeoutError:
                if time.monotonic() >= camera_deadline:
                    raise
                time.sleep(0.02)

        logger.info("推理开始：{} Hz，broker={}；Ctrl-C 停止", args.control_hz, args.broker)
        period = 1.0 / args.control_hz
        step = 0
        last_limit_warning = {}
        while args.max_steps <= 0 or step < args.max_steps:
            started = time.monotonic()
            if args.diagnostic and step == 0:
                logger.info("诊断：开始读取 controller 和相机快照")
            for controller in controllers.values():
                controller.check_error()
            cameras_msg = collector.snapshot(
                max_age=args.max_observation_age, max_skew=args.max_camera_skew,
            )
            robot = ROBOT_DATA(time_stamp=time.time())
            for controller in controllers.values():
                state = controller.latest_state()
                robot.arms.extend(state.arms)
                robot.end_effectors.extend(state.end_effectors)
            observation = layout.observation(robot, cameras_msg, args.prompt)
            if args.diagnostic and step == 0:
                logger.info(
                    "诊断：观测就绪，state={}，images={}",
                    observation["observation/state"].shape,
                    {key: value.shape for key, value in observation.items()
                     if isinstance(value, np.ndarray) and value.ndim == 3},
                )
                logger.info("诊断：开始调用 OpenPI")
            action = policy.infer(observation)
            if args.diagnostic and step == 0:
                logger.info(
                    "诊断：OpenPI 返回 action，范围=[{:.4f}, {:.4f}]",
                    float(action.min()), float(action.max()),
                )
            if not args.no_submit:
                target = layout.target(action)
                for item in target.end_effectors:
                    limits = hand_limits[item.end_effector_name]
                    original = item.joints.position.copy()
                    item.joints.position = np.clip(original, limits[:, 0], limits[:, 1])
                    if not np.array_equal(original, item.joints.position):
                        now = time.monotonic()
                        name = item.end_effector_name
                        if now - last_limit_warning.get(name, float("-inf")) >= 5.0:
                            logger.warning(
                                "{} 手模型目标已裁剪到硬件限位：raw_deg={} clipped_deg={}",
                                name, np.rad2deg(original).round(3).tolist(),
                                np.rad2deg(item.joints.position).round(3).tolist(),
                            )
                            last_limit_warning[name] = now
                for side, controller in controllers.items():
                    if not controller.submit_inference(split_side(target, side)):
                        raise RuntimeError(
                            f"{side} 推理目标提交失败，controller 已停止或不在 AUTO 模式"
                        )
            if args.diagnostic and step == 0:
                logger.info("诊断：{}", "已跳过硬件提交" if args.no_submit else "目标已提交")
            step += 1
            elapsed = time.monotonic() - started
            if elapsed > period:
                logger.warning("推理周期超时：{:.1f} ms > {:.1f} ms", elapsed * 1000, period * 1000)
            time.sleep(max(0.0, period - elapsed))


def main() -> None:
    # SIGSEGV 等原生扩展崩溃时打印所有 Python 线程的最后调用位置。
    faulthandler.enable(all_threads=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--server", default="ws://192.168.2.15:8000")
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--broker", choices=("serial", "rtg"), default="serial")
    parser.add_argument("--control-hz", type=float, default=3.0)
    parser.add_argument("--device-control-hz", type=float, default=50.0)
    parser.add_argument("--action-horizon", type=int, default=30)
    parser.add_argument("--trigger-fraction", type=float, default=0.5)
    parser.add_argument("--guidance-steps", type=int, default=6)
    parser.add_argument("--max-observation-age", type=float, default=0.5)
    parser.add_argument("--camera-startup-timeout", type=float, default=10.0)
    parser.add_argument("--max-camera-skew", type=float, default=0.1)
    parser.add_argument("--max-steps", type=int, default=0, help="0 表示持续运行")
    parser.add_argument("--home-timeout", type=float, default=60.0)
    parser.add_argument("--skip-home", action="store_true", help="保持当前位置开始推理")
    parser.add_argument("--no-submit", action="store_true", help="执行推理但不向硬件提交动作")
    parser.add_argument("--diagnostic", action="store_true", help="打印首轮推理的分阶段诊断")
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        logger.info("推理已停止，机器人进入 HOLD 并关闭设备")


if __name__ == "__main__":
    main()
