"""推理、人工纠正、轨迹回退与数据采集配置。"""

from copy import deepcopy
from pathlib import Path
import runpy


_BASE_CONFIG = Path(__file__).with_name('record_pico.py')
CONFIG = deepcopy(runpy.run_path(str(_BASE_CONFIG))['CONFIG'])

# 左腕相机安装方向倒置；RGB、深度、推理输入、预览和录制统一旋转。
CONFIG['camera_transforms'] = {'left_wrist': {'rotate_180': True}}

CONFIG['inference'] = {
    'server': 'ws://192.168.2.15:8000',
    'prompts': [
        'Place the wire harness into the white tray, avoiding obstacles.',
        # 在此继续添加可选 prompt。
    ],
    'broker': 'rtg',
    'control_hz': 15.,
    'action_horizon': 30,
    'trigger_fraction': .5,
    'guidance_steps': 6,
    'max_observation_age': .5,
    'max_camera_skew': .1,
    # 回退按历史目标重放；应不高于推理控制频率。
    'rollback_hz': 15.,
}
