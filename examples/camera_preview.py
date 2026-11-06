"""从 Python 配置开启全部相机，分别预览三视角；Q / Esc / 关闭窗口 / Ctrl+C 退出。

python examples/camera_preview.py
python examples/camera_preview.py --config /path/to/camera_preview.py
需要带 GUI 的 OpenCV：pip install opencv-python
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import runpy

import numpy as np

from roboci_loop.factory import build_camera


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "configs/camera_preview.py"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Python 配置文件路径")
    args = parser.parse_args()

    import cv2

    settings = runpy.run_path(str(args.config.expanduser().resolve()))["CONFIG"]
    preview = settings["preview"]
    timeout = preview["read_timeout_ms"]
    depth_max = preview["depth_max_m"]
    rotate_180 = set(preview.get("rotate_180", []))
    if isinstance(timeout, bool) or not isinstance(timeout, int) or timeout <= 0:
        raise ValueError("read_timeout_ms 必须为正整数")
    if not np.isfinite(depth_max) or depth_max <= 0:
        raise ValueError("depth_max_m 必须为有限正数")

    cameras = {name: build_camera(name, spec) for name, spec in settings["cameras"].items()}
    if not cameras:
        raise ValueError("cameras 至少需要配置一台相机")
    streams = []
    for name, camera in cameras.items():
        if camera.enable_rgb:
            streams.append(f"{name} / RGB")
        if camera.enable_depth:
            streams.append(f"{name} / Depth (0-{depth_max:g} m)")

    with ExitStack() as stack:
        stack.callback(cv2.destroyAllWindows)
        for name, camera in cameras.items():
            stack.callback(camera.stop)
            camera.start()
            print(f"相机 {name} 已启动")
        for title in streams:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        print("Q / Esc / 关闭任一窗口退出全部预览")
        while True:
            for name, camera in cameras.items():
                frame = camera.read(timeout_ms=timeout)
                if frame is None:
                    continue
                if frame.hasRGB():
                    rgb = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
                    if name in rotate_180:
                        rgb = cv2.rotate(rgb, cv2.ROTATE_180)
                    cv2.imshow(f"{name} / RGB", rgb)
                if frame.hasDepth():
                    valid = np.isfinite(frame.depth) & (frame.depth > 0)
                    depth = np.where(valid, frame.depth, 0)
                    scaled = (np.clip(depth / depth_max, 0, 1) * 255).astype(np.uint8)
                    colored = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
                    colored[~valid] = 0
                    if name in rotate_180:
                        colored = cv2.rotate(colored, cv2.ROTATE_180)
                    cv2.imshow(f"{name} / Depth (0-{depth_max:g} m)", colored)
            # 即使读取超时也处理窗口事件，保持退出响应。
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                break
            if any(cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1 for title in streams):
                break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
