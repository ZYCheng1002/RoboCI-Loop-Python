"""OpenPI 后台推理线程及推理控制辅助函数。"""

import queue
import threading
import time

import numpy as np

from roboci_loop.common import ROBOT_DATA


class InferenceWorker:
    """后台调用 OpenPI，人工接管后丢弃尚未完成的结果。"""

    def __init__(self, policy, layout, controllers, camera_snapshot, prompt, settings):
        self.policy = policy
        self.layout = layout
        self.controllers = controllers
        self.camera_snapshot = camera_snapshot
        self.prompt = prompt
        self.settings = settings
        self.results = queue.Queue(maxsize=1)
        self.errors = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name='openpi-inference')

    def start(self):
        self.policy.reset()
        self.thread.start()

    def _run(self):
        period = 1 / self.settings['control_hz']
        try:
            while not self.stop_event.is_set():
                started = time.monotonic()
                robot = ROBOT_DATA(time_stamp=time.time())
                for controller in self.controllers.values():
                    controller.check_error()
                    state = controller.latest_state()
                    robot.arms.extend(state.arms)
                    robot.end_effectors.extend(state.end_effectors)
                observation = self.layout.observation(
                    robot, self.camera_snapshot(), self.prompt
                )
                action = self.policy.infer(observation)
                if self.stop_event.is_set():
                    break
                if self.results.full():
                    self.results.get_nowait()
                self.results.put_nowait(action)
                self.stop_event.wait(max(0., period - (time.monotonic() - started)))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.errors.put(exc)

    def stop(self):
        self.stop_event.set()

    def close(self):
        self.stop()
        if self.thread.ident is not None:
            self.thread.join()


def split_side(data, side):
    return ROBOT_DATA(
        time_stamp=data.time_stamp,
        arms=[item for item in data.arms if item.arm_name == side],
        end_effectors=[
            item for item in data.end_effectors if item.end_effector_name == side
        ],
    )


def takeover_requested(tracker, threshold):
    if tracker is None:
        return False
    return any(
        control is not None and np.isfinite(control.grip) and control.grip > threshold
        for control in (tracker.left, tracker.right)
    )


