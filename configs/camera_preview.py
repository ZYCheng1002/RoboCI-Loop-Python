"""三视角相机预览配置；按实际设备修改序列号。"""

CONFIG = {
    "cameras": {
        "base": {
            "type": "orbbec",
            "config": {
                "serial_number": "CV27864000L2",
                "fps": 30,
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
                "fps": 30,
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
                "fps": 30,
                "width": 640,
                "height": 480,
                "enable_rgb": True,
                "enable_depth": False,
            },
        },
    },
    "preview": {
        "rotate_180": ["left_wrist"],  # 预览画面旋转 180° 的相机
        "read_timeout_ms": 100,
        "depth_max_m": 2.0,  # 深度颜色显示范围：0～2 m，不改变原始数据
    },
}
