#!/usr/bin/env python3
"""双手整掌读写压测；--control 启用四线程，否则仅两个读取线程。"""
import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from linked_hand import CanConfig, Hand, limits, version


@dataclass
class Stats:
    rounds: int = 0
    commands: int = 0
    errors: int = 0
    elapsed: float = 0.0
    error: str = ""


def worker(hand, target, write, stats, ready, stop, timing):
    ready.wait()
    try:
        while not stop.is_set() and time.perf_counter() < timing["deadline"]:
            for finger in range(1, 7):
                if stop.is_set() or time.perf_counter() >= timing["deadline"]:
                    return  # 未完成六关节不计为完整一轮
                if write:
                    hand.set_position(finger, target[finger - 1])
                else:
                    value = hand.get_position(finger)
                    if not math.isfinite(value):
                        raise ValueError(f"finger {finger}: non-finite position")
                stats.commands += 1
            stats.rounds += 1
    except Exception as error:
        stats.errors += 1
        stats.error = str(error)
        stop.set()
    finally:
        stats.elapsed = time.perf_counter() - timing["start"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control", action="store_true", help="四线程：重复发送初始位置，同时读取；不自动使能")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--left-id", type=lambda s: int(s, 0), default=0x65)
    parser.add_argument("--right-id", type=lambda s: int(s, 0), default=0x64)
    parser.add_argument("--timeout-ms", type=int, default=2000)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or args.timeout_ms <= 0:
        parser.error("seconds 和 timeout-ms 必须为正数")

    ready, stop = threading.Event(), threading.Event()
    timing, workers, results = {}, [], []
    with ExitStack() as stack:
        hands = []
        for name, channel, can_id in [("left", 0, args.left_id), ("right", 1, args.right_id)]:
            hand = Hand(can_config=CanConfig.gcan(device_index=args.device, channel=channel,
                        bitrate=1000000, device_can_id=can_id, driver="ecanvci"))
            stack.callback(hand.close)
            hand.set_timeout_ms(args.timeout_ms)
            hands.append((name, hand))
        # 先打开两通道，再等待初始化并读取目标；这些时间不计入测速。
        time.sleep(0.8)
        for name, hand in hands:
            target = [hand.get_position(finger) for finger in range(1, 7)]
            if args.control:
                for i, value in enumerate(target):
                    low, high = limits(i + 1)
                    if not math.isfinite(value) or value < low - 0.5 or value > high + 0.5:
                        raise ValueError(f"{name} finger {i+1}: 初始角度 {value} 超出范围，停止测试")
                    target[i] = max(low, min(high, value))  # 仅容忍零点附近的测量误差
            for write in ([False, True] if args.control else [False]):
                stats = Stats()
                results.append((name, "WRITE" if write else "READ", stats))
                workers.append(threading.Thread(target=worker,
                    args=(hand, target, write, stats, ready, stop, timing),
                    name=f"{name}-{'write' if write else 'read'}"))
        print(f"SDK {version()}, {len(workers)} threads, {args.seconds:g}s; angles in degrees", flush=True)
        try:
            for thread in workers:
                thread.start()
            timing["start"] = time.perf_counter()
            timing["deadline"] = timing["start"] + args.seconds
            ready.set()
            for thread in workers:
                thread.join()
        finally:
            stop.set()
            ready.set()
            for thread in workers:
                if thread.ident is not None:
                    thread.join()  # 必须在关闭 hand 前停止所有工作线程
        for name, operation, stats in results:
            hz = stats.rounds / stats.elapsed if stats.elapsed else 0
            print(f"{name:5s} {operation:5s}: {hz:.2f} Hz, rounds={stats.rounds}, "
                  f"commands={stats.commands}, errors={stats.errors}, elapsed={stats.elapsed:.3f}s")
            if stats.error:
                print(f"  {stats.error}")
        return int(any(stats.errors for _, _, stats in results))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as error:
        print(f"测试失败: {error}", file=sys.stderr)
        sys.exit(1)
