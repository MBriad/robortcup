#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""掉台回归状态机独立真机测试与 CSV 回放；生产比赛请运行根目录 main.py。"""

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
    DIGI_IR_ACTIVE_LEVEL,
    DIGI_IR_BITS,
    DIGI_IR_PINS,
    GRAY_ADC_MAX,
    GRAY_CHANNELS,
    GRAY_WHITE_ENTER,
    IR_ADC_MAX,
    IR_CHANNELS,
    PATROL_STALE_SECONDS,
)
from digi_ir import DigiIR  # noqa: E402
from gray import GrayRiskModel, GraySensor  # noqa: E402
from ir import IrSensor  # noqa: E402
from reentry import ReentryController  # noqa: E402


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(ROOT, "data", "reentry_%s.csv" % stamp)


def run(args):
    from up_controller import UpController

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    hardware = UpController(
        poll_hz=args.hz,
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    gray_sensor = GraySensor(
        adc_reader=lambda: hardware.adc_data,
        channels=GRAY_CHANNELS,
        adc_max=GRAY_ADC_MAX,
        white_enter=GRAY_WHITE_ENTER,
    )
    digi = DigiIR(
        io_reader=lambda: hardware.io_data,
        bits=DIGI_IR_BITS,
        active_level=DIGI_IR_ACTIVE_LEVEL,
    )
    analog_sensor = IrSensor(
        adc_reader=lambda: hardware.adc_data,
        channels=IR_CHANNELS,
        adc_max=IR_ADC_MAX,
    )
    reentry = ReentryController(force_fall=args.force_trigger)
    if args.force_trigger:
        print("强制触发模式：启动后自动触发一次状态机（台架测试；真实掉台不要传此参数）")
    fields = (
        "t", "front", "rear", "left", "right", "zone_front", "zone_rear",
        "zone_left", "zone_right", "fall", "fall_edge", "ir_front", "ir_rear",
        "ir_left_front", "ir_left_rear", "ir_right_front", "ir_right_rear",
        "ir_valid", "adc_left", "adc_right", "adc_signal_raw", "diff",
        "state", "reason", "left_cmd", "right_cmd", "healthy",
    )
    start = time.monotonic()
    period = 1.0 / args.hz
    try:
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while not args.seconds or time.monotonic() - start < args.seconds:
                loop_start = time.monotonic()
                gray_raw = gray_sensor.read_raw()
                ir_states = digi.read_states()
                analog_raw = analog_sensor.read_raw()
                healthy = hardware.healthy and not hardware.stale(PATROL_STALE_SECONDS)
                result = reentry.update(
                    gray_raw, ir_states, analog_raw,
                    now=loop_start, healthy=healthy,
                )
                hardware.move_cmd(result["left"], result["right"])
                observation = result["observation"]
                diff = (
                    analog_raw["left"] - analog_raw["right"]
                    if analog_raw["valid"] else float("nan")
                )
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    **{name: gray_raw[name] for name in GrayRiskModel.NAMES},
                    **{"zone_" + name: round(observation["zone"][name], 4)
                       for name in GrayRiskModel.NAMES},
                    "fall": int(result["fall"]),
                    "fall_edge": int(result["fall_edge"]),
                    **{"ir_" + name: int(ir_states[name]) for name in DIGI_IR_PINS},
                    "ir_valid": int(ir_states["valid"]),
                    "adc_left": analog_raw["left"],
                    "adc_right": analog_raw["right"],
                    "adc_signal_raw": max(analog_raw["left"], analog_raw["right"]),
                    "diff": round(diff, 1),
                    "state": result["state"],
                    "reason": result["reason"],
                    "left_cmd": result["left"],
                    "right_cmd": result["right"],
                    "healthy": int(healthy),
                })
                handle.flush()
                line = "state=%s fall=%d cmd=(%d,%d) diff=%s z=%.2f" % (
                    result["state"], int(result["fall"]),
                    result["left"], result["right"],
                    "nan" if diff != diff else "%.0f" % diff,
                    observation["zone_score"],
                )
                sys.stdout.write("\r" + line)
                sys.stdout.flush()
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print()
    finally:
        hardware.close()
    print("\n掉台回归测试日志：%s" % args.log)


def run_replay(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                rows.append({
                    name: float(row[name])
                    for name in ("t", "front", "rear", "left", "right")
                })
            except (KeyError, ValueError):
                raise SystemExit("CSV 列应为 t,front,rear,left,right")
    reentry = ReentryController()
    fake_ir = {name: False for name in DIGI_IR_PINS} | {"valid": True}
    fake_analog = {"left": 0.0, "right": 0.0, "valid": False}
    triggered_at = None
    trigger_result = None
    end_state = ""
    trace = []
    for row in rows:
        result = reentry.update(
            {name: row[name] for name in GrayRiskModel.NAMES},
            fake_ir, fake_analog, now=row["t"], healthy=True,
        )
        if result["observation"]["ready"]:
            trace.append(result["observation"]["zone_score"])
        if result["fall_edge"] and triggered_at is None:
            triggered_at = row["t"]
            trigger_result = result
        end_state = result["state"]
    if triggered_at is None:
        print("未触发掉台（%d 行，结束状态 %s）" % (len(rows), end_state))
    else:
        print("掉台触发：t=%.3fs（共 %d 行）→ %s" % (
            triggered_at, len(rows), trigger_result["reason"]))
    if trace:
        print("zone_score 范围：[%.3f, %.3f]（中位 %.3f）" % (
            min(trace), max(trace), sorted(trace)[len(trace) // 2]))


def main():
    parser = argparse.ArgumentParser(description="掉台回归状态机独立测试")
    parser.add_argument("--replay", metavar="CSV", help="PC 回放灰度 CSV")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--log", default=_default_log_path())
    parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
        force_trigger=False,
    )
    parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    parser.add_argument("--force-trigger", action="store_true",
                        help="启动即强制触发一次（台架测试）")
    args = parser.parse_args()
    if args.replay:
        run_replay(args.replay)
        return
    run(args)


if __name__ == "__main__":
    main()
