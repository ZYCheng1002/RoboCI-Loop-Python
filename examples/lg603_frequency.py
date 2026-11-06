"""每只手一个上层循环；LG603 驱动内部独立采集反馈。"""
import queue
import threading
import time

import numpy as np

from roboci_loop.hardware.end_effector import EndEffectorConfig, HandLG603


INIT_POSITION = [0., 0., 0., 0., 0., 0.]


def worker(hand, stop, errors):
    """控制同步发送，读取只消费驱动缓存。"""
    rng = np.random.default_rng()
    limits = hand.joint_limits
    control_count = read_count = 0
    started = time.perf_counter()
    try:
        while not stop.is_set():
            hand.send_joint_positions(rng.uniform(limits[:, 0], limits[:, 1]))
            control_count += 1
            state = hand.read()
            if state is not None:
                position = state.joints.position
                if position.shape != (6,) or not np.isfinite(position).all():
                    raise RuntimeError("灵巧手反馈无效")
                read_count += 1
            now = time.perf_counter()
            elapsed = now - started
            if elapsed >= 1.0:
                print(
                    f"[{hand.config.end_effector_name}] "
                    f"控制频率: {control_count / elapsed:.1f} Hz，"
                    f"反馈消费频率: {read_count / elapsed:.1f} Hz",
                    flush=True,
                )
                started, control_count, read_count = now, 0, 0
    except Exception as exc:
        errors.put((hand.config.end_effector_name, "worker", exc))
        stop.set()


def main():
    stop = threading.Event()
    errors = queue.Queue()
    hands = []
    threads = []
    try:
        for name, channel, can_id in (("left_hand", 0, 0x65), ("right_hand", 1, 0x64)):
            hand = HandLG603(EndEffectorConfig(name), channel=channel, device_can_id=can_id)
            hands.append(hand)
            hand.start(background_read=False)
        # 打开第二通道时，不让第一通道存在进行中的请求。
        # 与厂商 benchmark 一致，给双通道初始化留出稳定时间。
        time.sleep(0.8)
        for hand in hands:
            hand.start_feedback()
        for hand in hands:
            name = hand.config.end_effector_name
            # 启动工作线程前完成预热与初始目标发送；通信异常直接退出。
            deadline = time.monotonic() + 6 * hand.timeout_ms / 1000 + 1
            state = hand.read()
            while state is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{name} 等待首次反馈超时")
                time.sleep(0.001)
                state = hand.read()
            position = state.joints.position
            if position.shape != (6,) or not np.isfinite(position).all():
                raise RuntimeError(f"{name} 预热反馈无效")
            hand.send_joint_positions(INIT_POSITION)
            print(f"[{name}] 预热成功", flush=True)

        for hand in hands:
            thread = threading.Thread(
                target=worker, args=(hand, stop, errors),
                name=f"{hand.config.end_effector_name}-control",
            )
            threads.append(thread)
            thread.start()
        while not stop.wait(0.1):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        # SDK 请求返回后线程才能退出；必须 join 后再销毁 SDK 连接。
        for thread in threads:
            if thread.ident is not None:
                thread.join()
        try:
            for hand in reversed(hands):
                try:
                    hand.stop()
                except Exception as exc:
                    errors.put((hand.config.end_effector_name, "stop", exc))
        finally:
            if not errors.empty():
                name, operation, exc = errors.get_nowait()
                raise RuntimeError(f"{name} {operation} 失败") from exc


if __name__ == "__main__":
    main()
