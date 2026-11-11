"""OpenPI 推理人工介入配置。

硬件、PICO 和遥操作设置沿用 record_pico.py；本文件只维护推理专用参数。
"""

from copy import deepcopy
from pathlib import Path
import runpy


_BASE_CONFIG = Path(__file__).with_name("record_pico.py")
CONFIG = deepcopy(runpy.run_path(str(_BASE_CONFIG))["CONFIG"])

CONFIG["inference"] = {
    "server": "ws://192.168.2.15:8000",
    "prompt": "Place the wire harness into the white tray, avoiding obstacles.",
    "broker": "rtg",  # serial / rtg
    "control_hz": 15.,
    "action_horizon": 30,
    "trigger_fraction": .5,
    "guidance_steps": 6,
    "max_observation_age": .5,
    "camera_startup_timeout": 10.,
    "max_camera_skew": .1,
    "home_timeout": 60.,
    "skip_home": False,
}
