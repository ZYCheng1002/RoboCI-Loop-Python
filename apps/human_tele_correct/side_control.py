"""单侧机械臂与灵巧手的 PICO 遥操作控制。"""

from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np

from roboci_loop.common import ARM_DATA, END_EFFECTOR_DATA, JOINT_DATA, ROBOT_DATA
from roboci_loop.controller import ControlMode


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
    trigger_closing: bool | None = None
    last_trigger: float | None = None
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

    def reset_teleop_session(self):
        """外部运动后丢弃旧 PICO/手部锚点，并以最新反馈作为新手部目标。"""
        self.teleop.stop()
        feedback = self.controller.latest_state()
        self.hand_target = (feedback.end_effectors[0].joints.position.copy()
                            if feedback.end_effectors else None)
        self.trigger_anchor = None
        self.trigger_reference = None
        self.trigger_closing = None
        self.last_trigger = None
        self.last_hand_update = time.monotonic()
        self.faulted = False

    def request_home(self):
        self.teleop.stop()
        self.hand_target = np.zeros(6)
        self.trigger_anchor = self.hand_target.copy()
        self.trigger_reference = None
        self.trigger_closing = None
        self.last_trigger = None
        target = ROBOT_DATA(time_stamp=time.time(),
            arms=[ARM_DATA(self.name, joints=JOINT_DATA(position=self.home))],
            end_effectors=[END_EFFECTOR_DATA(self.name, joints=JOINT_DATA(position=self.hand_target.copy()))])
        if self.controller.submit_init(target, end_effector_tolerance=np.deg2rad(
                self.settings.get('hand_home_tolerance_deg', 2.))):
            self.home_started = time.monotonic()
            return target

    def request_first_pose(self, trigger):
        """Primary/First 直接下发灵巧手目标，不等待到位。"""
        self.teleop.stop()
        self.hand_target = np.deg2rad(self.settings['hand_first_deg'])
        self.trigger_anchor = self.hand_target.copy()
        self.trigger_reference = None
        self.trigger_closing = None
        self.last_trigger = float(np.clip(trigger, 0, 1))
        self.last_hand_update = time.monotonic()
        target = ROBOT_DATA(time_stamp=time.time(), end_effectors=[
            END_EFFECTOR_DATA(self.name, joints=JOINT_DATA(position=self.hand_target.copy()))
        ])
        if (self.controller.set_mode(ControlMode.TELEOP)
                and self.controller.submit_teleop(target)):
            return target

    @staticmethod
    def _axis(value, deadzone):
        if abs(value) <= deadzone:
            return 0.0
        return np.sign(value) * (abs(value) - deadzone) / (1.0 - deadzone)

    def hand_positions(self, control, feedback, now):
        if self.hand_target is None:
            # read() 是各设备独立的增量反馈，可能只有机械臂；接管首帧必须
            # 使用完整最新状态，不能缺少手反馈时退回默认张开位置。
            state = feedback if feedback.end_effectors else self.controller.latest_state()
            if not state.end_effectors:
                return None
            self.hand_target = state.end_effectors[0].joints.position.copy()
        trigger = float(np.clip(control.trigger, 0, 1))
        if self.trigger_anchor is None:
            self.trigger_anchor = self.hand_target.copy()
        if self.last_trigger is None:
            self.last_trigger = trigger
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
        trigger_delta = trigger - self.last_trigger
        if abs(trigger_delta) > 1e-4:
            closing = trigger_delta > 0
            if closing != self.trigger_closing:
                self.trigger_anchor = self.hand_target.copy()
                self.trigger_reference = self.last_trigger
                self.trigger_closing = closing
        if self.trigger_closing is True:
            span = 1.0 - self.trigger_reference
            progress = 1.0 if span <= 1e-6 else np.clip(
                (trigger - self.trigger_reference) / span, 0.0, 1.0)
            self.hand_target[1:5] = (self.trigger_anchor[1:5]
                + progress * (closed[1:5] - self.trigger_anchor[1:5]))
        elif self.trigger_closing is False:
            span = self.trigger_reference
            progress = 0.0 if span <= 1e-6 else np.clip(trigger / span, 0.0, 1.0)
            self.hand_target[1:5] = (opened[1:5]
                + progress * (self.trigger_anchor[1:5] - opened[1:5]))
        self.last_trigger = trigger
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
            arms=[ARM_DATA(self.name, joints=JOINT_DATA(position=joints))])
        hand_positions = self.hand_positions(control, feedback, now)
        if hand_positions is not None:
            target.end_effectors.append(END_EFFECTOR_DATA(
                self.name, joints=JOINT_DATA(position=hand_positions)))
        if self.controller.set_mode(ControlMode.TELEOP) and self.controller.submit_teleop(target):
            return target
        self.hold()


