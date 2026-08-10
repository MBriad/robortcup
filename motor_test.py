#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车体前进、后退、原地左转、原地右转的真机电机测试。"""

import argparse
import csv
import os
import time

from config import CHASSIS_MOTOR_INVERT, CHASSIS_MOTOR_SWAP


DEV_MODE = True
DEFAULT_SPEED = 400
DEFAULT_DURATION = 0.5
DEFAULT_PAUSE = 0.8
MIN_SPEED = 400
MAX_SPEED = 800


def build_cases(speed):
    """返回四种车体动作对应的左右轮命令。"""
    return (
        ("forward", "车体前进", speed, speed),
        ("backward", "车体后退", -speed, -speed),
        ("turn-left", "车体原地左转", -speed, speed),
        ("turn-right", "车体原地右转", speed, -speed),
    )


def default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join("data", "motor_test_%s.csv" % stamp)


def run_command(controller, left, right, duration):
    """执行单次命令；无论是否异常都立即停车。"""
    try:
        controller.move_cmd(left, right)
        time.sleep(duration)
    finally:
        controller.move_cmd(0, 0)


def run_dev(args):
    from up_controller import UpController

    if args.ground:
        print("实车模式：把车放在平整空地中央，人与线缆远离车轮。")
        confirm = "GROUND"
    else:
        print("悬空模式：抬起车轮，确认四周无人和障碍物。")
        confirm = "LIFTED"
    if input("确认安全后输入 %s：" % confirm).strip() != confirm:
        print("未完成安全确认，测试取消。")
        return

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    controller = UpController(
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    print("电机修正：invert=%s swap=%s" % (args.motor_invert, args.motor_swap))
    fields = ("step", "mode", "action", "description", "left_cmd", "right_cmd",
              "duration", "distance_cm", "motor_invert", "motor_swap", "observed")
    cases = build_cases(args.speed)
    if args.action != "all":
        cases = tuple(case for case in cases if case[0] == args.action)
    try:
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for step, (action, description, left, right) in enumerate(cases, start=1):
                choice = input(
                    "[%d/%d] %s，命令=(%d, %d)。回车准备，s 跳过，q 退出：" %
                    (step, len(cases), description, left, right)
                ).strip().lower()
                if choice == "q":
                    break
                if choice == "s":
                    observed = "skipped"
                    distance_cm = ""
                else:
                    if args.ground:
                        for remaining in range(args.countdown, 0, -1):
                            print("%d 秒后执行，Ctrl+C 可急停" % remaining)
                            time.sleep(1.0)
                    run_command(controller, left, right, args.duration)
                    observed = input("观察是否正确？y=正确 n=错误 ?=不确定：").strip().lower()
                    observed = {"y": "correct", "n": "wrong"}.get(observed, "uncertain")
                    distance_cm = ""
                    if action in ("forward", "backward"):
                        measured = input("输入实际移动距离 cm（未测量直接回车）：").strip()
                        if measured:
                            try:
                                distance_cm = float(measured)
                            except ValueError:
                                print("距离格式无效，本次留空。")
                    time.sleep(args.pause)
                writer.writerow({
                    "step": step,
                    "mode": "ground" if args.ground else "lifted",
                    "action": action,
                    "description": description,
                    "left_cmd": left,
                    "right_cmd": right,
                    "duration": args.duration,
                    "distance_cm": distance_cm,
                    "motor_invert": int(args.motor_invert),
                    "motor_swap": int(args.motor_swap),
                    "observed": observed,
                })
                handle.flush()
    finally:
        controller.close()
    print("测试日志：%s" % args.log)


def main():
    parser = argparse.ArgumentParser(description="车体四种基本动作电机测试")
    parser.add_argument(
        "action", choices=("forward", "backward", "turn-left", "turn-right", "all"),
        help="要测试的车体动作",
    )
    parser.add_argument("--ground", action="store_true", help="车轮着地进行实车动作")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument("--pause", type=float, default=DEFAULT_PAUSE)
    parser.add_argument("--countdown", type=int, default=3)
    parser.add_argument("--log", default=default_log_path())
    parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
    )
    parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    args = parser.parse_args()

    if not MIN_SPEED <= args.speed <= MAX_SPEED:
        parser.error("--speed 必须在 %d..%d" % (MIN_SPEED, MAX_SPEED))
    if args.duration <= 0.0 or args.pause < 0.0 or args.countdown < 0:
        parser.error("--duration 必须为正数，--pause/--countdown 不能为负数")
    if not DEV_MODE:
        print("DEV_MODE=False：真机电机测试已禁用")
        return
    run_dev(args)


if __name__ == "__main__":
    main()
