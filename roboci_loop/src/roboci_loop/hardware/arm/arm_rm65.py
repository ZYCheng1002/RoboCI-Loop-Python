"""RealMan RM65 机械臂驱动（API2）。SDK 的线程模式及 UDP 回调是进程全局资源。"""
import copy
import importlib
import ipaddress
import math
import threading
import time
from typing import Optional, Sequence

import numpy as np

from ...common.msg_robot import ARM_DATA, JOINT_DATA, POSE_DATA
from .arm_base import ArmBase, ArmConfig


class ArmRM65(ArmBase):
    """IP 连接；可选 UDP 状态和 CAN FD 关节透传。

    udp_ip 是本机接收网卡 IPv4；为空时通过 TCP 查询状态。
    udp_cycle 是 SDK 周期参数（实际周期为 udp_cycle * 5 ms）。
    can_fd=True 时每次调用仅发送一个目标，不创建控制循环或插值。
    高跟随要求调用方稳定地以不超过 10 ms 的周期发送平滑轨迹。
    read() 在 UDP 模式消费最新一帧，丢弃中间帧；时间戳为主机接收时间。
    SDK 初始化后不要在进程内另行切换线程模式、覆盖回调或调用 rm_destroy。
    """

    _sdk_lock = threading.RLock()
    _sdk = None
    _callback = None
    _instances = {}

    def __init__(
        self, config: Optional[ArmConfig] = None, *, ip: str,
        port: int = 8080, udp_ip: Optional[str] = None,
        udp_port: int = 8089, udp_cycle: int = 1,
        can_fd: bool = False, follow: bool = False,
        trajectory_mode: int = 0, radio: int = 0, speed: int = 80,
    ) -> None:
        super().__init__(config)
        self.ip = str(ipaddress.IPv4Address(ip))
        self.udp_ip = None if udp_ip is None else str(ipaddress.IPv4Address(udp_ip))
        for name, value, low, high in (
            ("port", port, 1, 65535), ("udp_port", udp_port, 1, 65535),
            ("udp_cycle", udp_cycle, 1, 65535), ("speed", speed, 1, 100),
            ("trajectory_mode", trajectory_mode, 0, 2),
            ("radio", radio, 0, 999 if trajectory_mode == 2 else 100),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
                raise ValueError(f"{name} 必须为 {low}~{high} 的整数")
        if not isinstance(can_fd, bool) or not isinstance(follow, bool):
            raise ValueError("can_fd 和 follow 必须为 bool")
        self.port, self.udp_port, self.udp_cycle = port, udp_port, udp_cycle
        self.can_fd, self.follow = can_fd, follow
        self.trajectory_mode, self.radio, self.speed = trajectory_mode, radio, speed
        self._robot = None
        self._dof = 0
        self._condition = threading.Condition()
        self._state = None

    @property
    def is_running(self) -> bool:
        return self._robot is not None

    @staticmethod
    def _check(code: int, operation: str) -> None:
        if code != 0:
            raise RuntimeError(f"RM65 {operation} 失败，SDK 返回码 {code}")

    @classmethod
    def _dispatch(cls, data) -> None:
        # ctypes 回调内不允许异常逃逸；坏包丢弃，read 会超时。
        try:
            with cls._sdk_lock:
                arm = cls._instances.get(data.arm_ip.decode())
            if arm is None or not arm.is_running or data.errCode != 0:
                return
            joint = data.joint_status
            pose = data.waypoint
            state = arm._make_state(
                joint.joint_position[:arm._dof],
                [pose.position.x, pose.position.y, pose.position.z,
                 pose.euler.rx, pose.euler.ry, pose.euler.rz],
                joint.joint_speed[:arm._dof],
            )
            with arm._condition:
                arm._state = state
                arm._condition.notify_all()
        except Exception:
            return

    def _push_config(self, enable: bool):
        return self._sdk.rm_realtime_push_config_t(
            cycle=self.udp_cycle, enable=enable, port=self.udp_port,
            force_coordinate=-1, ip=self.udp_ip,
            custom_config=self._sdk.rm_udp_custom_config_t(joint_speed=1),
        )

    def start(self) -> None:
        if self.is_running:
            return
        cls = ArmRM65
        with cls._sdk_lock:
            if self.ip in cls._instances:
                raise RuntimeError(f"{self.ip} 已被另一个 ArmRM65 实例连接")
            if cls._sdk is None:
                try:
                    sdk = importlib.import_module("Robotic_Arm.rm_robot_interface")
                except ModuleNotFoundError as exc:
                    if exc.name != "Robotic_Arm":
                        raise
                    raise ImportError("请安装 RealMan SDK：python -m pip install Robotic_Arm") from exc
                robot = sdk.RoboticArm(sdk.rm_thread_mode_e.RM_TRIPLE_MODE_E)
                cls._sdk = sdk
            else:
                robot = cls._sdk.RoboticArm()
            handle = robot.rm_create_robot_arm(self.ip, self.port)
            if handle.id < 0:
                raise ConnectionError(f"无法连接 RM65 {self.ip}:{self.port}")
            self._robot = robot
            cls._instances[self.ip] = self
        try:
            self._dof = robot.arm_dof
            if self._dof not in (6, 7):
                raise RuntimeError(f"无法获取有效机械臂自由度：{self._dof}")
            if self.udp_ip is not None:
                with cls._sdk_lock:
                    if cls._callback is None:
                        cls._callback = self._sdk.rm_realtime_arm_state_callback_ptr(cls._dispatch)
                        robot.rm_realtime_arm_state_call_back(cls._callback)
                self._check(robot.rm_set_realtime_push(self._push_config(True)), "配置 UDP")
        except Exception:
            try:
                self.stop()
            except Exception:
                pass
            raise

    def _require_robot(self):
        if self._robot is None:
            raise RuntimeError("机械臂尚未连接，请先调用 start()")
        return self._robot

    def _make_state(self, positions, pose, velocities=()) -> ARM_DATA:
        # SDK 欧拉角为固定轴 XYZ（roll/pitch/yaw），转换为消息的 xyzw。
        rx, ry, rz = (value / 2 for value in pose[3:])
        sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
        cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
        return ARM_DATA(
            arm_name=self.config.arm_name, time_stamp=time.time(),
            joints=JOINT_DATA(
                names=[f"joint_{i + 1}" for i in range(self._dof)],
                position=[math.radians(x) for x in positions],
                velocity=[math.radians(x) for x in velocities],
            ),
            end_pose=POSE_DATA(
                position=np.array(pose[:3], dtype=np.float64),
                orientation=np.array([sx*cy*cz-cx*sy*sz, cx*sy*cz+sx*cy*sz,
                                      cx*cy*sz-sx*sy*cz, cx*cy*cz+sx*sy*sz]),
            ),
        )

    def read(self, timeout_ms: int = 1000) -> Optional[ARM_DATA]:
        """TCP 模式同步查询，timeout_ms 仅控制 UDP 等待，TCP 超时由 SDK 管理。"""
        robot = self._require_robot()
        if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms <= 0:
            raise ValueError("timeout_ms 必须为正整数")
        if self.udp_ip is None:
            code, data = robot.rm_get_current_arm_state()
            self._check(code, "读取状态")
            return self._make_state(data["joint"], data["pose"])
        with self._condition:
            self._condition.wait_for(
                lambda: self._state is not None or not self.is_running, timeout_ms / 1000
            )
            state, self._state = self._state, None
            return copy.deepcopy(state)

    def send_joint_positions(self, positions: Sequence[float], *, slow: bool = False) -> None:
        """slow 使用普通 movej（速度不超过 10%）；否则沿用透传配置。"""
        robot = self._require_robot()
        angles = np.asarray(positions, dtype=np.float64)
        if angles.shape != (self._dof,) or not np.isfinite(angles).all():
            raise ValueError(f"positions 必须为 {self._dof} 个有限关节角度（rad）")
        degrees = np.rad2deg(angles).tolist()
        if not isinstance(slow, bool):
            raise ValueError("slow 必须为 bool")
        if slow:
            code = robot.rm_movej(degrees, min(self.speed, 50), 0, 0, 0)
        elif self.can_fd:
            code = robot.rm_movej_canfd(
                degrees, self.follow, trajectory_mode=self.trajectory_mode, radio=self.radio
            )
        else:
            code = robot.rm_movej(degrees, self.speed, 0, 0, 0)
        self._check(code, "发送关节目标")

    def stop_motion(self) -> None:
        self._check(self._require_robot().rm_set_arm_stop(), "停止运动")

    def stop(self) -> None:
        robot = self._robot
        if robot is None:
            return
        try:
            if self.udp_ip is not None:
                self._check(robot.rm_set_realtime_push(self._push_config(False)), "关闭 UDP")
        finally:
            # 不调用全局 rm_destroy，以免断开其他机械臂。
            code = robot.rm_delete_robot_arm()
            with self._sdk_lock:
                self._instances.pop(self.ip, None)
                self._robot = None
            with self._condition:
                self._state = None
                self._condition.notify_all()
            self._check(code, "断开连接")
