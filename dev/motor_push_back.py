#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""倒车上台测试：逐档尝试大力倒车冲上台面，记录每档是否成功。

场景：车掉到台下，需要倒车上台；本程序每档速度执行
「前进（到准备位）→ 停车 → 大力倒车冲台 → 停车」，询问是否上台，
参数与结果存 CSV，用于确定需要多大的速度（力）才能倒车上台。

纯电机测试：直接打开 vendor 库裸控 CDS，不读任何传感器，不经 up_controller.py。

真机用法：
    python3 dev/motor_push_back.py                       # 默认档位 500~900，每档倒车 2s
    python3 dev/motor_push_back.py --speeds 600          # 只测单档
    python3 dev/motor_push_back.py --speeds 500,700 --reverse-seconds 3
"""

import argparse
import csv
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT in sys.path:
    sys.path.remove(ROOT)
sys.path.insert(0, ROOT)

from config import (  # noqa: E402
    CHASSIS_MOTOR_INVERT,
    CHASSIS_MOTOR_SWAP,
    PATROL_MIN_ACTIVE_SPEED,
)


DEFAULT_SPEEDS = (500, 600, 700, 800, 900)
DEFAULT_FORWARD_SPEED = PATROL_MIN_ACTIVE_SPEED
DEFAULT_FORWARD_SECONDS = 1.0
DEFAULT_REVERSE_SECONDS = 2.0
STOP_PAUSE = 0.5   # 档间停车停顿（秒）


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(ROOT, "data", "motor_push_back_%s.csv" % stamp)


def _open_up():
    """直接打开 vendor 库（同 dev/rpm_tool.py 模式，纯电机不经驱动层）。"""
    try:
        import uptech
    except ImportError as exc:
        raise RuntimeError("电机测试需要 uptech 库") from exc
    hardware = uptech.UpTech()
    hardware.CDS_Open()
    return hardware


def _drive(hardware, left, right, invert, swap):
    """裸 CDS 差速指令（同 up_controller.move_cmd 的修正语义）。"""
    if swap:
        left, right = right, left
    if invert:
        left, right = -left, -right
    hardware.CDS_SetSpeed(7, left)
    hardware.CDS_SetSpeed(8, -right)


def _stop(hardware):
    hardware.CDS_SetSpeed(7, 0)
    hardware.CDS_SetSpeed(8, 0)


def run(args):
    hardware = _open_up()
    print("电机修正：invert=%s swap=%s" % (args.motor_invert, args.motor_swap))
    print("档位：%s；每档：前进%d×%gs → 停车 → 倒车%gs 冲台" % (
        args.speeds, args.forward_speed, args.forward_seconds,
        args.reverse_seconds))
    print("注意：确认前进/倒车方向无障碍，人与线缆远离车轮！Ctrl+C 可急停。")
    rows = []
    try:
        for index, speed in enumerate(args.speeds, start=1):
            print("\n[%d/%d] 倒车档位 speed=%d" % (index, len(args.speeds), speed))
            print("  前进 %d×%gs" % (args.forward_speed, args.forward_seconds))
            _drive(hardware, args.forward_speed, args.forward_speed,
                   args.motor_invert, args.motor_swap)
            time.sleep(args.forward_seconds)
            _stop(hardware)
            time.sleep(STOP_PAUSE)
            print("  大力倒车 %d×%gs 冲台……" % (speed, args.reverse_seconds))
            _drive(hardware, -speed, -speed, args.motor_invert, args.motor_swap)
            time.sleep(args.reverse_seconds)
            _stop(hardware)
            time.sleep(STOP_PAUSE)
            while True:
                choice = input("  上台了吗？y=上去了 n=没上去 ?=不确定：").strip().lower()
                result = {"y": "up", "n": "not_up"}.get(choice)
                if result:
                    break
                print("  输入 y / n / ?")
            rows.append({
                "speed": speed,
                "forward_speed": args.forward_speed,
                "forward_seconds": args.forward_seconds,
                "reverse_seconds": args.reverse_seconds,
                "result": result,
            })
            print("  → speed=%d : %s" % (speed, result))
    except KeyboardInterrupt:
        print("\n已中断")
    finally:
        _stop(hardware)
        hardware.CDS_Close()

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    with open(args.log, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=(
                "speed", "forward_speed", "forward_seconds",
                "reverse_seconds", "result"))
        writer.writeheader()
        writer.writerows(rows)
    print("\n测试完成，数据已存：%s" % args.log)
    for row in rows:
        print("  speed=%d → %s" % (row["speed"], row["result"]))


def main():
    parser = argparse.ArgumentParser(description="倒车上台测试：逐档测大力倒车冲台")
    parser.add_argument("--speeds", default=",".join(map(str, DEFAULT_SPEEDS)),
                        help="倒车速度档位，逗号分隔（默认 500,600,700,800,900）")
    parser.add_argument("--forward-speed", type=int, default=DEFAULT_FORWARD_SPEED,
                        help="前进准备速度（默认 400）")
    parser.add_argument("--forward-seconds", type=float, default=DEFAULT_FORWARD_SECONDS,
                        help="前进准备时长（秒，默认 1.0）")
    parser.add_argument("--reverse-seconds", type=float, default=DEFAULT_REVERSE_SECONDS,
                        help="每档倒车冲台时长（秒，默认 2.0）")
    parser.add_argument("--log", default=None, help="CSV 路径；默认 data/motor_push_back_时间.csv")
    parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
    )
    parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    args = parser.parse_args()
    try:
        args.speeds = tuple(int(part) for part in args.speeds.split(",") if part.strip())
    except ValueError:
        parser.error("--speeds 需要逗号分隔的数字，如 500,600,700")
    if not args.speeds:
        parser.error("--speeds 不能为空")
    for speed in args.speeds:
        if not 1 <= speed <= 1023:
            parser.error("速度必须在 1..1023：%d" % speed)
    if args.forward_seconds <= 0 or args.reverse_seconds <= 0:
        parser.error("时长必须为正数")
    if not args.log:
        args.log = _default_log_path()
    run(args)


if __name__ == "__main__":
    main()
