#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""转角标定交互采集工具：逐目标角实测「直接转到 N 度」的速度/时长组合。

转角与时长非线性（起步、摩擦、电池都有影响），每个目标角独立实测、不做插值。
每档流程：输入「速度 时长」→ 车原地转 → 输入实测角度（回车=刚好到位定稿，
数字=记录后继续调参）；左右转各定稿一组 (speed, duration)。

产出：
    data/motor_turn_calibration.csv   单测格式 direction,angle,speed,duration（定稿即写）
    data/motor_turn_trials_时间.csv   每次试验明细，便于复盘调参过程
定稿完成后把整表同步进 config.py 的 MOTOR_TURN_CALIBRATION（左右两张表，按转向方向查）。

真机用法：
    python3 dev/turn_tool.py                 # 按默认 6 档角度集，从缺档开始
    python3 dev/turn_tool.py --angle 90      # 只补测 90°
    python3 dev/turn_tool.py --angles 10,90,180
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

from config import CHASSIS_MOTOR_INVERT, CHASSIS_MOTOR_SWAP  # noqa: E402

# 目标角集合：直接转到 N 度（新车 2026-08-14 已标 6 档；补测用 --angle/--angles）。
ANGLES = (10.0, 22.5, 45.0, 90.0, 135.0, 180.0)
DIRECTIONS = ("left", "right")
TABLE_FIELDS = ("direction", "angle", "speed", "duration")
TRIAL_FIELDS = ("t", "angle", "direction", "speed", "duration", "measured", "result")
DEFAULT_CANDIDATE = (500, 1.0)


def _default_paths():
    out = os.path.join(ROOT, "data", "motor_turn_calibration.csv")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    trials = os.path.join(ROOT, "data", "motor_turn_trials_%s.csv" % stamp)
    return out, trials


def _load_table(path):
    """读已有定稿表（缺档续测用）；文件不存在返回空表。"""
    table = {}
    if not os.path.isfile(path):
        return table
    with open(path, newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            table[(row["direction"], float(row["angle"]))] = (
                int(row["speed"]), float(row["duration"]))
    return table


def _write_table(path, table):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    rows = sorted(
        ({"direction": direction, "angle": angle, "speed": speed,
          "duration": duration}
         for (direction, angle), (speed, duration) in table.items()),
        key=lambda row: (row["angle"], row["direction"]),
    )
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=TABLE_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def _turn(hardware, direction, speed, duration):
    """原地转 duration 秒：left=左转、right=右转（同 motor_tool build_cases 语义）。"""
    if direction == "left":
        command = (-speed, speed)
    else:
        command = (speed, -speed)
    print("  转动中：%s转 speed=%d duration=%.2fs（Ctrl+C 急停）" % (
        "左" if direction == "left" else "右", speed, duration))
    try:
        hardware.move_cmd(*command)
        time.sleep(duration)
    finally:
        hardware.move_cmd(0, 0)


def run(args):
    from up_controller import UpController

    out, trials = _default_paths()
    table = _load_table(out)
    hardware = UpController(
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    print("角度集：%s；左/右各定稿一组（回车=到位定稿，数字=实测角度，"
          "r=重跑，s=跳过，q=保存退出）" % ", ".join(str(a) for a in args.angles))
    print("注意：车放地面平整处，人与线缆远离车轮；每档前把车摆回起点方向。")
    pending = [
        (angle, direction)
        for angle in args.angles
        for direction in DIRECTIONS
        if (direction, angle) not in table
    ]
    done_count = len(args.angles) * 2 - len(pending)
    if done_count:
        print("已完成 %d/%d 组，续测剩余 %d 组。" % (
            done_count, len(args.angles) * 2, len(pending)))
    try:
        with open(trials, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=TRIAL_FIELDS)
            writer.writeheader()
            for angle, direction in pending:
                print("\n目标：%s转 %.1f°" % ("左" if direction == "left" else "右",
                                             angle))
                speed, duration = DEFAULT_CANDIDATE
                while True:
                    choice = input("  速度 时长（回车重跑 %d %.2f）：" % (
                        speed, duration)).strip()
                    if choice:
                        if choice == "q":
                            print("已保存，退出。")
                            return
                        if choice == "s":
                            print("  跳过 %s转 %.1f°" % (
                                "左" if direction == "left" else "右", angle))
                            break
                        parts = choice.split()
                        if len(parts) == 2:
                            try:
                                speed, duration = int(parts[0]), float(parts[1])
                            except ValueError:
                                print("  格式：速度(整数) 时长(秒)，如 500 1")
                                continue
                        else:
                            print("  格式：速度(整数) 时长(秒)，如 500 1")
                            continue
                    _turn(hardware, direction, speed, duration)
                    measured = input("  实测角度（回车=正好 %.1f° 定稿；"
                                     "数字=实际角度继续调）：" % angle).strip()
                    if measured == "":
                        table[(direction, angle)] = (speed, duration)
                        _write_table(out, table)
                        writer.writerow({
                            "t": round(time.monotonic(), 3),
                            "angle": angle, "direction": direction,
                            "speed": speed, "duration": duration,
                            "measured": "", "result": "fixed",
                        })
                        handle.flush()
                        print("  → %s转 %.1f° 定稿：speed=%d duration=%.2f" % (
                            "左" if direction == "left" else "右",
                            angle, speed, duration))
                        break
                    writer.writerow({
                        "t": round(time.monotonic(), 3),
                        "angle": angle, "direction": direction,
                        "speed": speed, "duration": duration,
                        "measured": measured, "result": "retry",
                    })
                    handle.flush()
                    print("  记录：实测 %s°（目标 %.1f°），继续调参。" % (
                        measured, angle))
    except KeyboardInterrupt:
        print("\n已中断")
    finally:
        hardware.close()
    missing = [
        (angle, direction)
        for angle in args.angles
        for direction in DIRECTIONS
        if (direction, angle) not in table
    ]
    if missing:
        print("缺档：%s" % ", ".join(
            "%s转%.1f°" % ("左" if direction == "left" else "右", angle)
            for angle, direction in missing))
    else:
        print("全部定稿：%s" % out)
    print("试验明细：%s" % trials)


def main():
    parser = argparse.ArgumentParser(description="转角标定交互采集")
    parser.add_argument("--angle", type=float, default=None,
                        help="只测单个目标角")
    parser.add_argument("--angles", default=None,
                        help="目标角列表，逗号分隔（默认 %s）" % (
                            ",".join(str(a) for a in ANGLES)))
    parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
    )
    parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    args = parser.parse_args()
    if args.angle is not None:
        args.angles = (args.angle,)
    elif args.angles:
        args.angles = tuple(float(part) for part in args.angles.split(",")
                            if part.strip())
    else:
        args.angles = ANGLES
    run(args)


if __name__ == "__main__":
    main()
