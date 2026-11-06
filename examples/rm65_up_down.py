"""双臂沿世界 Z 轴上移 0.1 m 再回到起点，共往复三次，保持末端姿态。"""
from contextlib import ExitStack
import time

import numpy as np

from roboci_loop.hardware.arm import ArmConfig, ArmRM65
from roboci_loop.kinematics import RM65IK, RM65IKConfig


LOCAL_IP = "192.168.1.17"
HEIGHT = -0.1  # m，从起点上移的距离（总行程 0.1 m，不是正负各 0.1 m）
CYCLES = 3  # 完整的上去、回来计一次
PERIOD = 10.  # s，一次往复时间
CONTROL_HZ = 100
MAX_JOINT_SPEED = 1.  # rad/s，预检查相邻轨迹点，拒绝突变的逆解
ARMS = {
    "left": ("192.168.1.18", (0., 90., 90.)),
    "right": ("192.168.1.19", (90., 90., 0.)),
}


def make_path(ik, state):
    """预先求出一整次往复的关节轨迹，失败时不执行运动。"""
    target = ik.forward(state.joints.position)
    start_z = float(target.position[2])
    seed = state.joints.position
    ik.begin_tracking(seed, dt=1 / CONTROL_HZ)
    steps = round(PERIOD * CONTROL_HZ)
    path = []
    for i in range(steps + 1):
        # 余弦轨迹：从零速度开始，上移后以零速度返回起点。
        target.position[2] = start_z + HEIGHT * (1 - np.cos(2 * np.pi * i / steps)) / 2
        joints = ik.track(target)
        if joints is None:
            raise RuntimeError(f"轨迹第 {i} 点 单步 QP 求解失败，不执行运动")
        joints = np.asarray(joints)
        allowed = MAX_JOINT_SPEED / CONTROL_HZ
        if np.max(np.abs(joints - seed)) > allowed:
            raise RuntimeError(f"轨迹第 {i} 点关节变化过快，不执行运动")
        path.append(joints.tolist())
        seed = joints
    if np.max(np.abs(np.asarray(path[-1]) - path[1])) > MAX_JOINT_SPEED / CONTROL_HZ:
        raise RuntimeError("往复衔接处关节变化过快，不执行运动")
    return path


def main():
    with ExitStack() as stack:
        arms, paths = {}, {}
        for name, (ip, installation) in ARMS.items():
            arm = ArmRM65(ArmConfig(name), ip=ip, udp_ip=LOCAL_IP, can_fd=True)
            stack.callback(arm.stop)
            arm.start()
            arms[name] = arm
        # 先结束运动，再关闭全部连接；一个臂清理失败也会继续清理其他臂。
        for arm in arms.values():
            stack.callback(arm.stop_motion)
        for name, arm in arms.items():
            state = arm.read(timeout_ms=3000)
            if state is None:
                raise RuntimeError(f"{name} 没有初始状态")
            ik = RM65IK(RM65IKConfig(base_rpy_deg=ARMS[name][1]))
            paths[name] = make_path(ik, state)
            print(f"{name}: 一次往复轨迹 IK 检查完成", flush=True)

        # 规划期间不应移动机械臂，执行前再次确认当前角度仍接近起点。
        for name, arm in arms.items():
            state = arm.read(timeout_ms=1000)
            if state is None or np.max(np.abs(state.joints.position - paths[name][0])) > .02:
                raise RuntimeError(f"{name} 起点发生变化，请重新运行")
        print(f"开始运动：上移 {HEIGHT} m 后返回，往复 {CYCLES} 次", flush=True)
        deadline = time.perf_counter()
        for cycle in range(CYCLES):
            for i in range(1, len(paths['left'])):
                deadline += 1 / CONTROL_HZ
                time.sleep(max(0, deadline - time.perf_counter()))
                for name, arm in arms.items():
                    arm.send_joint_positions(paths[name][i])
                # 调度落后时延长轨迹时间，不连续补发。
                deadline = max(deadline, time.perf_counter())
            print(f"已完成 {cycle + 1}/{CYCLES} 次", flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("已停止往复运动")
