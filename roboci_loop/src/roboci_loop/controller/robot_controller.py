"""每个执行设备一个线程，负责读取反馈和定频发送关节目标。"""

import copy
from dataclasses import dataclass, field
import threading
import time
from typing import Mapping, Optional

import numpy as np
from loguru import logger

from .control_mode import ControlMode
from ..common import ROBOT_DATA
from ..hardware.arm import ArmBase
from ..hardware.end_effector import EndEffectorBase


@dataclass(frozen=True)
class DeviceControlConfig:
    control_hz: float = 100.0       # 单设备循环频率上限，单位 Hz


@dataclass(frozen=True)
class RobotControllerConfig:
    arm: DeviceControlConfig = field(
        default_factory=DeviceControlConfig
    )  # 臂默认控制配置
    end_effector: DeviceControlConfig = field(
        default_factory=lambda: DeviceControlConfig(control_hz=50.0)
    )
    arms: dict[str, DeviceControlConfig] = field(default_factory=dict)
    end_effectors: dict[str, DeviceControlConfig] = field(default_factory=dict)


@dataclass
class _DeviceLoop:
    """单个设备线程的局部运行数据，不与其他线程共享。"""

    measured: Optional[np.ndarray] = None                       # 最新实测关节角，rad
    hold: Optional[np.ndarray] = None                           # 当前模式的保持关节角，rad
    planned: bool = False                                       # 是否有尚未停止的 movej


class RobotController:
    """上层先连接设备, controller.start() 启动每设备的读写线程."""

    def __init__(
        self,
        arms: Mapping[str, ArmBase],                    # 机械臂设备
        end_effectors: Mapping[str, EndEffectorBase],   # 手、夹爪等末端执行器
        *,
        config: Optional[RobotControllerConfig] = None, # 设备配置
    ) -> None:
        self.config = (
            copy.deepcopy(config) if config is not None else RobotControllerConfig()
        )  # 独立配置副本
        self.arms = dict(arms)      # 已连接的机械臂，键为 arm_name
        self.end_effectors = dict(end_effectors)
        self._devices = {
            **{("arms", n): d for n, d in self.arms.items()},
            **{("end_effectors", n): d for n, d in self.end_effectors.items()},
        }
        if any(set(getattr(self.config, k)) - set(getattr(self, k))
               for k in ("arms", "end_effectors")):
            raise ValueError("控制配置包含未知设备名称")
        self._condition = threading.Condition(threading.RLock())    # 保护共享状态及启动通知
        self._stop = threading.Event()                              # 通知所有设备结束工作
        self._ready = threading.Event()                             # 所有设备反馈就绪后才允许发送
        self._threads = []                                          # 每个设备的工作线程
        self._started = False                                       # 是否启动过；停止后不复用此实例
        self._mode = ControlMode.HOLD                               # HOLD / INIT / AUTO / TELEOP
        self._reset_devices = set(self._devices)                    # 模式切换后待重置的设备
        self._targets = {}                                          # 当前来源最新目标，键为 (设备类型, 名称)，单位 rad
        self._pending = {}                                          # 每设备容量 1，read 后清空
        self._feedback = {}                                         # 各设备最新反馈消息
        self._init_done = set()                                     # 当前初始化中已到位的设备索引
        self._feedback_callback = None

    def set_feedback_callback(self, callback) -> None:
        """注册新反馈回调 (ROBOT_DATA)，在设备线程执行；回调不得阻塞或调用 stop。"""
        with self._condition:
            self._feedback_callback = callback

    @property
    def mode(self) -> ControlMode:
        with self._condition:
            return self._mode

    def start(self, timeout: float = 30.0) -> None:
        """等待全部设备首次有效反馈，启动 HOLD；初始化过程不发送运动目标。"""
        self._validate_positive(timeout, "timeout")
        with self._condition:
            if self._stop.is_set():
                raise RuntimeError("controller 已停止，请重新创建实例")
            # 启动过检查是否报错, 无需二次启动
            if self._started:
                return
            self._started = True
            # 为多个设备分配线程
            for key, device in self._devices.items():
                thread = threading.Thread(
                    target=self._run, args=(key, device), name=f"{key[0]}-{key[1]}"
                )
                self._threads.append(thread)
        try:
            for thread in self._threads:
                thread.start()
            with self._condition:
                # 阻塞, 等待至(收到所有设备反馈 or 退出指令 or 超时)
                self._condition.wait_for(
                    lambda: len(self._feedback) == len(self._devices)
                    or self._stop.is_set(),
                    timeout,
                )
                if self._stop.is_set() or len(self._feedback) != len(self._devices):
                    raise RuntimeError("等待设备首次反馈失败或超时")
                # 设置启动
                self._ready.set()
        except BaseException:
            self.stop()
            raise

    def set_mode(self, mode: ControlMode) -> bool:
        """由外部切换控制模式；切换时清空目标并重新建立保持位置。"""
        if not isinstance(mode, ControlMode):
            logger.error("mode 必须为 ControlMode 枚举")
            return False
        with self._condition:
            if not self._ready.is_set() or self._stop.is_set():
                logger.error("controller 尚未启动或已经停止")
                return False
            if self._mode == ControlMode.INIT:
                logger.warning("初始化中, 禁止切换")
                return False
            if mode != self._mode:  # 切换模式前, 先完成相关重置
                self._mode = mode
                self._reset_devices.update(self._devices)
                self._targets = {}
                self._init_done = set()
                self._condition.notify_all()
            return True

    def _submit(self, data, mode):
        with self._condition:
            if (not self._ready.is_set() or self._stop.is_set()  or self._mode != mode):
                return False
            targets = self._validate_targets(data)
            self._targets.update(targets)
            return True

    def _validate_targets(self, data):
        targets = {}
        for kind, attr in (("arms", "arm_name"), ("end_effectors", "end_effector_name")):
            for item in getattr(data, kind):
                key = (kind, getattr(item, attr))
                if key not in self._devices or key in targets:
                    raise ValueError(f"未知或重复设备：{key}")
                target = item.joints.position
                if target.shape != self._feedback[key].joints.position.shape or not np.isfinite(target).all():
                    raise ValueError(f"{key} 目标无效或数量不匹配")
                targets[key] = target.copy()
        return targets

    @staticmethod
    def _validate_positive(value, name):
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} 必须为有限正数")

    def submit_init(self, data: ROBOT_DATA, *, tolerance: float = 0.02,
                    end_effector_tolerance: Optional[float] = None) -> bool:
        """非阻塞提交初始化目标；INIT 中拒绝其他提交和模式切换。

        全部指定设备的新反馈到位后自动 HOLD；超时由外部判断并 stop。
        """
        self._validate_positive(tolerance, "tolerance")
        if end_effector_tolerance is not None:
            self._validate_positive(end_effector_tolerance, "end_effector_tolerance")
        with self._condition:
            if not self._ready.is_set() or self._stop.is_set() or self._mode == ControlMode.INIT:
                return False
            targets = self._validate_targets(data)
            if not targets:
                raise ValueError("初始化目标不能为空")
            self._mode = ControlMode.INIT               # 内部模式切换为初始化
            self._reset_devices.update(self._devices)   # 重置设备
            self._targets = targets                     # 清空目标
            self._init_done = set()
            self._init_tolerance = tolerance
            self._init_end_effector_tolerance = tolerance if end_effector_tolerance is None else end_effector_tolerance
            return True

    def _initialization_tolerance(self, key):
        return self._init_tolerance if key[0] == 'arms' else self._init_end_effector_tolerance

    def submit_inference(self, data: ROBOT_DATA) -> bool:
        """在 AUTO 模式更新目标；推理任务的调度和取消由外部负责。"""
        return self._submit(data, ControlMode.AUTO)

    def initialization_status(self) -> dict:
        """初始化目标的到位状态快照；角度及误差为度，仅用于诊断。"""
        with self._condition:
            if self._mode != ControlMode.INIT:
                return {}
            status = {}
            for key, target in self._targets.items():
                state = self._feedback.get(key)
                measured = None if state is None else state.joints.position
                status['/'.join(key)] = {
                    'done': key in self._init_done,
                    'target_deg': np.rad2deg(target).round(2).tolist(),
                    'measured_deg': None if measured is None else np.rad2deg(measured).round(2).tolist(),
                    'error_deg': None if measured is None else np.rad2deg(target - measured).round(2).tolist(),
                    'tolerance_deg': round(float(np.rad2deg(self._initialization_tolerance(key))), 2),
                    'feedback_age_s': None if state is None else round(time.time() - state.time_stamp, 2),
                }
            return status

    def submit_teleop(self, data: ROBOT_DATA) -> bool:
        """在 TELEOP 模式更新目标。"""
        return self._submit(data, ControlMode.TELEOP)

    def read(self) -> ROBOT_DATA:
        """非阻塞消费每设备最新一条反馈；空列表表示该设备尚无新消息。

        多次 read 竞争同一组队列；各设备分别采样，不保证同步。
        """
        with self._condition:
            data = ROBOT_DATA(time_stamp=time.time())
            for (kind, _), state in self._pending.items():
                getattr(data, kind).append(state)
            self._pending.clear()
            return data

    def latest_state(self) -> ROBOT_DATA:
        """完整的最新反馈快照，不消费 read() 的增量队列；保留设备采样时间。"""
        with self._condition:
            data = ROBOT_DATA(time_stamp=time.time())
            for (kind, _), state in self._feedback.items():
                getattr(data, kind).append(copy.deepcopy(state))
            return data

    def _read_device(self, key, device, loop):
        """读取、验证反馈并更新缓存；返回本轮是否收到新反馈。"""
        state = device.read(timeout_ms=1) if key[0] == "arms" else device.read()    # 获取最新状态
        if self._stop.is_set():
            return False
        if state is not None:
            measured = state.joints.position
            if (measured.ndim != 1 or not measured.size or not np.isfinite(measured).all()):
                logger.error(f"{key} 关节反馈无效")
                return False
            if loop.measured is not None and measured.shape != loop.measured.shape:
                logger.error(f"{key} 反馈关节数量变化")
                return False
            loop.measured = measured.copy()
            with self._condition:
                self._feedback[key] = copy.deepcopy(state)
                self._pending[key] = copy.deepcopy(state)
                self._condition.notify_all()
                callback = self._feedback_callback
            if callback is not None:
                data = ROBOT_DATA(time_stamp=state.time_stamp)
                getattr(data, key[0]).append(copy.deepcopy(state))
                callback(data)
        return state is not None

    def _select_target(self, key, loop):
        """处理模式切换，返回模式、目标及是否参与初始化。"""
        with self._condition:
            reset = key in self._reset_devices
            if reset:
                loop.hold = loop.measured.copy()
                self._reset_devices.discard(key)
            return (
                self._mode,
                self._targets.get(key, loop.hold).copy(),
                self._mode == ControlMode.INIT and key in self._targets,
                reset,
            )

    def _run_init(self, key, device, loop, target, fresh):
        """初始化专用执行：臂规划一次，末端直接发送目标；仅新反馈用于到位判断。"""
        if key[0] == "arms" and not loop.planned:
            device.send_joint_positions(target.tolist(), slow=True)
            loop.planned = True
        elif key[0] != "arms":
            device.send_joint_positions(target.tolist(), slow=True)
        if fresh:
            with self._condition:
                if self._mode != ControlMode.INIT or key in self._reset_devices:
                    return
                if np.max(np.abs(loop.measured - target)) <= self._initialization_tolerance(key):
                    self._init_done.add(key)
                else:
                    self._init_done.discard(key)
                if self._init_done == set(self._targets):
                    self._mode = ControlMode.HOLD
                    self._targets = {}
                    self._reset_devices.update(self._devices)
                self._condition.notify_all()

    def _send_target(self, key, device, loop, selection, fresh):
        """INIT 的臂只发一次 movej；手及其他模式直接下发目标。"""
        mode, target, initializing, reset = selection
        if loop.planned and (mode != ControlMode.INIT or reset):
            device.stop_motion()  # 退出旧规划后，才能继续透传
            loop.planned = False
        if mode == ControlMode.INIT:
            if initializing:
                self._run_init(key, device, loop, target, fresh)
            return
        device.send_joint_positions(target.tolist())

    def _run(self, key, device):
        """每设备一个线程：读取反馈 → 选择目标 → 执行目标 → 等待下一周期。"""
        kind, name = key
        default = self.config.arm if kind == "arms" else self.config.end_effector
        settings = getattr(self.config, kind).get(name, default)
        loop = _DeviceLoop()                # 当前设备的反馈
        period = 1 / settings.control_hz    # 计算控制时延
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                fresh = self._read_device(key, device, loop)  # 读取最新状态
                if self._stop.is_set():
                    break
                if self._ready.is_set():
                    selection = self._select_target(key, loop)
                    if not self._stop.is_set():
                        self._send_target(
                            key, device, loop, selection, fresh
                        )
                deadline = max(deadline + period, time.monotonic())
                self._stop.wait(max(0, deadline - time.monotonic()))
        except Exception:
            logger.exception("{} 设备控制线程失败", key)
            with self._condition:
                self._mode = ControlMode.HOLD
                self._reset_devices.update(self._devices)
                self._targets = {}
                self._stop.set()
                self._condition.notify_all()
        finally:
            if kind == "arms":
                try:
                    device.stop_motion()
                except Exception:
                    logger.exception("{} 停止运动失败", name)

    def stop(self) -> None:
        """通知并等待所有设备线程退出，再由上层关闭设备；手保持最后目标。

        无法打断已进入 SDK 的阻塞调用，退出耗时受 SDK 超时影响。
        """
        with self._condition:
            self._stop.set()
            self._mode = ControlMode.HOLD
            self._targets = {}
            self._condition.notify_all()
        for thread in self._threads:
            if thread.ident is not None and thread is not threading.current_thread():
                thread.join()
