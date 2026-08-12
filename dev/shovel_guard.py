#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""铲子防掉落独立真机测试与 CSV 回放；推东西模式用 --active 模拟（生产由视觉/数字红外给出）。"""

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
    IR_ADC_MAX,
    PATROL_STALE_SECONDS,
    SHOVEL_ADC_MAX,
    SHOVEL_IR_CHANNELS,
)
from ir import IrSensor  # noqa: E402
from shovel_guard import ShovelGuard  # noqa: E402


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(ROOT, "data", "shovel_%s.csv" % stamp)


def run(args):
    from up_controller import UpController

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    hardware = UpController(
        poll_hz=args.hz,
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    sensor = IrSensor(
        adc_reader=lambda: hardware.adc_data,
        channels=SHOVEL_IR_CHANNELS,
        adc_max=SHOVEL_ADC_MAX,
    )
    guard = ShovelGuard()
    if args.active:
        print("推东西模式已开启（模拟 active=True）：铲子悬空将触发停车+倒车收回")
    fields = (
        "t", "left", "right", "valid", "active", "hang", "state",
        "reason", "left_cmd", "right_cmd", "healthy",
    )
    start = time.monotonic()
    period = 1.0 / args.hz
    try:
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while not args.seconds or time.monotonic() - start < args.seconds:
                loop_start = time.monotonic()
                raw = sensor.read_raw()
                healthy = hardware.healthy and not hardware.stale(PATROL_STALE_SECONDS)
                result = guard.update(raw, args.active, now=loop_start, healthy=healthy)
                hardware.move_cmd(result["left"], result["right"])
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    "left": raw["left"],
                    "right": raw["right"],
                    "valid": int(raw["valid"]),
                    "active": int(args.active),
                    "hang": int(result["hang"]),
                    "state": result["state"],
                    "reason": result["reason"],
                    "left_cmd": result["left"],
                    "right_cmd": result["right"],
                    "healthy": int(healthy),
                })
                handle.flush()
                line = "state=%s hang=%d cmd=(%d,%d) L=%d R=%d" % (
                    result["state"], int(result["hang"]),
                    result["left"], result["right"],
                    raw["left"], raw["right"],
                )
                sys.stdout.write("\r" + line)
                sys.stdout.flush()
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print()
    finally:
        hardware.close()
    print("\n铲子防掉落测试日志：%s" % args.log)


def run_replay(path, active):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                rows.append({
                    "left": float(row["left"]),
                    "right": float(row["right"]),
                    "valid": bool(int(row.get("valid", "1"))),
                    "t": float(row["t"]),
                })
            except (KeyError, ValueError):
                raise SystemExit("CSV 列应为 t,left,right[,valid]")
    guard = ShovelGuard()
    states = []
    for row in rows:
        result = guard.update(row, active, now=row["t"], healthy=True)
        states.append(result["state"])
        if result["state"] != "IDLE":
            print("t=%.3fs state=%s hang=%d cmd=(%d,%d) %s" % (
                row["t"], result["state"], int(result["hang"]),
                result["left"], result["right"], result["reason"]))
    print("回放结束：状态序列 %s" % (" → ".join(states) if states else "空"))


def main():
    parser = argparse.ArgumentParser(description="铲子防掉落状态机独立测试")
    parser.add_argument("--replay", metavar="CSV", help="PC 回放铲子采集 CSV")
    parser.add_argument("--active", action="store_true",
                        help="推东西模式开启（真机测试模拟 active；回放时默认开启）")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--log", default=_default_log_path())
    parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
    )
    parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    args = parser.parse_args()
    if args.replay:
        run_replay(args.replay, active=args.active or True)
        return
    run(args)


if __name__ == "__main__":
    main()
