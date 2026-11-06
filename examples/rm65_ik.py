"""RM65IK 使用示例：读取双臂当前位置并执行一次跟踪求解，不发送运动指令。"""
import time

import numpy as np

from roboci_loop.hardware.arm import ArmConfig, ArmRM65
from roboci_loop.kinematics import RM65IK, RM65IKConfig


BASE_RPY_DEG = {
    "left": (0., 90., 90.),
    "right": (90., 90., 0.),
}


def main():
    for name, ip in (("left", "192.168.1.18"), ("right", "192.168.1.19")):
        with ArmRM65(ArmConfig(name), ip=ip) as arm:
            state = arm.read()  # TCP 查询当前关节角和末端位姿
        if state is None or state.end_pose is None:
            raise RuntimeError(f"{name} 未获取到末端位姿")
        # 平移默认零：各臂以自身基座原点为原点，安装后的轴方向与世界系一致。
        ik = RM65IK(RM65IKConfig(base_rpy_deg=BASE_RPY_DEG[name]))
        current = state.joints.position
        start = time.perf_counter()
        target = ik.forward(current)  # 模型世界系目标，不直接混用 SDK 基座系位姿
        ik.begin_tracking(current, dt=.02)
        result = ik.track(target)
        elapsed = (time.perf_counter() - start) * 1000
        print(f"{name}: 当前关节角 (deg): {np.rad2deg(current).round(3).tolist()}")
        print(f"  当前末端位置 (m): {state.end_pose.position.round(4).tolist()}")
        print(f"  当前末端姿态 (xyzw): {state.end_pose.orientation.round(4).tolist()}")
        if result is None:
            print("  单步 QP 求解失败。")
            continue
        print(f"  单步跟踪关节角 (deg): {np.rad2deg(result).round(3).tolist()}")
        difference = np.rad2deg(np.asarray(result) - np.asarray(current))
        print(f"  差值 跟踪 - 实测 (deg): {difference.round(3).tolist()}")
        print(f"  最大绝对差值 (deg): {np.max(np.abs(difference)):.3f}")
        print(f"  初始化及单步求解耗时: {elapsed:.2f} ms")


if __name__ == '__main__':
    main()
