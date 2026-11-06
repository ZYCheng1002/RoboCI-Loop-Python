"""配置驱动的设备创建示例。

运行：python examples/factory_example.py
默认只构造对象，不连接设备；IK 会加载模型，因此单独按需创建。
"""
from roboci_loop.factory import (
    build_arm,
    build_camera,
    build_controller,
    build_end_effector,
    build_ik,
)


# 同样的字典结构也可以来自 JSON / YAML 文件。
CONFIG = {
    "arms": {
        "left": {
            "type": "rm65",
            "config": {"ip": "192.168.1.18", "udp_ip": "192.168.1.17", "can_fd": True},
        },
        "right": {
            "type": "rm65",
            "config": {"ip": "192.168.1.19", "udp_ip": "192.168.1.17", "can_fd": True},
        },
    },
    "end_effectors": {
        "left": {"type": "lg603", "config": {"channel": 0, "device_can_id": 0x65}},
        "right": {"type": "lg603", "config": {"channel": 1, "device_can_id": 0x64}},
    },
    "cameras": {
        "base": {
            "type": "orbbec",
            "config": {
                "serial_number": "CV27864000L2",
                "fps": 30,
                "width": 640,
                "height": 480,
                "enable_depth": False,
            },
        },
    },
    "controllers": {"pico": {"type": "pico"}},
    "kinematics": {
        "left": {"type": "rm65", "config": {"base_rpy_deg": [0, 90, 90]}},
        "right": {"type": "rm65", "config": {"base_rpy_deg": [90, 90, 0]}},
    },
}


def main():
    arms = {name: build_arm(name, spec) for name, spec in CONFIG["arms"].items()}
    hands = {
        name: build_end_effector(name, spec)
        for name, spec in CONFIG["end_effectors"].items()
    }
    cameras = {
        name: build_camera(name, spec) for name, spec in CONFIG["cameras"].items()
    }
    inputs = {
        name: build_controller(name, spec) for name, spec in CONFIG["controllers"].items()
    }

    for group, devices in (
        ("arms", arms), ("end_effectors", hands), ("cameras", cameras), ("controllers", inputs),
    ):
        for name, device in devices.items():
            print(f"{group}.{name}: {type(device).__name__}, running={device.is_running}")

    # 安装运动学依赖后，可按同一份配置创建 IK（加载模型，不连接硬件）：
    # ik = {name: build_ik(spec) for name, spec in CONFIG["kinematics"].items()}

    # 在实际应用中，由应用层管理连接与关闭，例如：
    # from contextlib import ExitStack
    # with ExitStack() as stack:
    #     for device in arms.values():
    #         stack.callback(device.stop)  # 先注册清理，启动失败也可释放资源
    #         device.start()
    #     # 在这里读取状态，或把 arms / hands 传给 RobotController。


if __name__ == "__main__":
    main()
