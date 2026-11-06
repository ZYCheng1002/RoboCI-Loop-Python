"""PICO 遥操作采集配置；运行前修改设备 IP、相机序列号和初始关节角。

启动后回初始姿态；关闭 home_on_start 可保持当前位置开始。
深度采集需同时开启相机 enable_depth 和 recording.record_depth。
recording 可设置 output_dir；省略时每次在 data/ 下新建时间戳目录。
"""

CONFIG = {
    "arms": {
        "left": {
            "type": "rm65",
            "config": {
                "ip": "192.168.1.18",
                "udp_ip": "192.168.1.17",
                "can_fd": True,
            },
        },
        "right": {
            "type": "rm65",
            "config": {
                "ip": "192.168.1.19",
                "udp_ip": "192.168.1.17",
                "can_fd": True,
            },
        },
    },

    "end_effectors": {
        "left": {
            "type": "lg603",
            "config": {
                "channel": 0,
                "device_can_id": 0x65,
            },
        },
        "right": {
            "type": "lg603",
            "config": {
                "channel": 1,
                "device_can_id": 0x64,
            },
        },
    },

    "cameras": {
        "base": {
            "type": "orbbec",
            "config": {
                "serial_number": "CV27864000L2",
                "fps": 60,
                "width": 640,
                "height": 480,
                "enable_rgb": True,
                "enable_depth": False,
            },
        },
        "left_wrist": {
            "type": "orbbec",
            "config": {
                "serial_number": "CV2C861000N1",
                "fps": 60,
                "width": 640,
                "height": 480,
                "enable_rgb": True,
                "enable_depth": False,
            },
        },
        "right_wrist": {
            "type": "orbbec",
            "config": {
                "serial_number": "CV278640008T",
                "fps": 60,
                "width": 640,
                "height": 480,
                "enable_rgb": True,
                "enable_depth": False,
            },
        },
    },

    "controllers": {
        "pico": {
            "type": "pico",
        },
    },

    "kinematics": {
        "left": {
            "type": "rm65",
            "config": {
                "base_rpy_deg": [0, 90, 90],
            },
        },
        "right": {
            "type": "rm65",
            "config": {
                "base_rpy_deg": [90, 90, 0],
            },
        },
    },

    "teleop": {
        "control_hz": 50,
        "hand_control_hz": 50,
        "home_on_start": True,
        "hand_home_tolerance_deg": 2.0,  # 手回零允许的小幅残差；机械臂仍使用默认 0.02 rad
        "feedback_timeout": 0.5,
        "input_timeout": 0.2,
        "grip_threshold": 0.9,
        "hand_open_deg": [10, 10, 10, 10, 10, 10],
        "hand_closed_deg": [100, 175, 175, 175, 175, 45],
        "hand_first_deg": [85, 70, 75, 175, 175, 25],
        "hand_axis_deadzone": 0.12,
        # [拇指旋转, 拇指屈曲] 的摇杆增量速度。
        "hand_thumb_speed_deg_s": [90, 90],
        "home_deg": {
            "left": [90, 40, 55, 0, 50, -15],
            "right": [90, -40, -55, 0, -50, 15],
        },
        "pose": {
            "control_orientation": True,
            "position_scale": 1.0,
            "cartesian_filter_alpha": 0.1,
            "input_to_world": [
                [0.0, 0.0, -1.0],
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
        },
    },

    "preview": {
        "main_camera": "base",
        "main_width": 640,
        "secondary_width": 320,
        "jpeg_quality": 90,
    },

    "recording": {
        "rgb_jpeg_quality": 85,
        "record_depth": False,
    },
}
