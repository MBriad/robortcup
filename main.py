#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RoboCup 生产入口：巡台为常态，掉台回归状态机可抢占电机控制。"""

import argparse
import csv
import os
import sys
import time

from config import (
    CHASSIS_MOTOR_INVERT,
    CHASSIS_MOTOR_SWAP,
    PATROL_STALE_SECONDS,
)
from digi_ir import DIGI_IR_PINS, DigiIR
from gray import GrayRiskModel, GraySensor
from ir import IrSensor
from reentry import ReentryController
from ring_patrol import RingPatrolController


class RobotController:
    """统一调度巡台与掉台回归；输入传感器数据，输出唯一电机命令。"""

    def __init__(self):
        self.patrol = RingPatrolController()
        self.reentry = ReentryController()
        self._reentry_active = False

    def update(self, gray_raw, ir, analog, now=None, healthy=True):
        now = time.monotonic() if now is None else float(now)
        reentry_result = self.reentry.update(
            gray_raw, ir, analog, now=now, healthy=healthy
        )

        if reentry_result["state"] != "WAIT":
            self._reentry_active = True
            return self._result("reentry", reentry_result)

        if self._reentry_active:
            # 回台期间巡台计时已失效，重新预热灰度模型后再恢复运动。
            self.patrol = RingPatrolController()
            self._reentry_active = False

        patrol_result = self.patrol.update(gray_raw, now=now, healthy=healthy)
        return self._result("patrol", patrol_result)

    def _result(self, mode, selected):
        result = dict(selected)
        result.update({
            "mode": mode,
            "patrol_state": self.patrol.state,
            "reentry_state": self.reentry.state,
        })
        return result


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join("data", "main_%s.csv" % stamp)


def run(args):
    from up_controller import UpController

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    hardware = UpController(
        poll_hz=args.hz,
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    gray_sensor = GraySensor(adc_reader=lambda: hardware.adc_data)
    digi = DigiIR(io_reader=lambda: hardware.io_data)
    analog_sensor = IrSensor(adc_reader=lambda: hardware.adc_data)
    robot = RobotController()
    fields = (
        "t", "front", "rear", "left", "right",
        "zone_front", "zone_rear", "zone_left", "zone_right", "zone_score",
        "ir_front", "ir_rear", "ir_left_front", "ir_left_rear",
        "ir_right_front", "ir_right_rear", "ir_valid", "analog_diff",
        "mode", "state", "patrol_state", "reentry_state", "reason",
        "left_cmd", "right_cmd", "healthy",
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
                result = robot.update(
                    gray_raw, ir_states, analog_raw,
                    now=loop_start, healthy=healthy,
                )
                hardware.move_cmd(result["left"], result["right"])
                observation = result["observation"]
                analog_diff = (
                    analog_raw["left"] - analog_raw["right"]
                    if analog_raw["valid"] else float("nan")
                )
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    **{name: gray_raw[name] for name in GrayRiskModel.NAMES},
                    **{"zone_" + name: round(observation["zone"][name], 4)
                       for name in GrayRiskModel.NAMES},
                    "zone_score": round(observation["zone_score"], 4),
                    **{"ir_" + name: int(ir_states[name]) for name in DIGI_IR_PINS},
                    "ir_valid": int(ir_states["valid"]),
                    "analog_diff": round(analog_diff, 1),
                    "mode": result["mode"],
                    "state": result["state"],
                    "patrol_state": result["patrol_state"],
                    "reentry_state": result["reentry_state"],
                    "reason": result["reason"],
                    "left_cmd": result["left"],
                    "right_cmd": result["right"],
                    "healthy": int(healthy),
                })
                handle.flush()
                line = "mode=%s state=%s cmd=(%d,%d) z=%.2f" % (
                    result["mode"], result["state"],
                    result["left"], result["right"],
                    observation["zone_score"],
                )
                sys.stdout.write("\r" + line)
                sys.stdout.flush()
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print()
    finally:
        hardware.close()
    print("\n生产运行日志：%s" % args.log)


def main():
    parser = argparse.ArgumentParser(description="RoboCup 巡台与掉台回归生产程序")
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
    run(parser.parse_args())


if __name__ == "__main__":
    main()
