#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡台状态机独立真机测试；生产比赛请运行根目录 main.py。"""

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
    PATROL_STALE_SECONDS,
)
from gray import GrayRiskModel, GraySensor  # noqa: E402
from ring_patrol import RingPatrolController  # noqa: E402


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(ROOT, "data", "patrol_%s.csv" % stamp)


def run(args):
    from up_controller import UpController

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    hardware = UpController(
        poll_hz=args.hz,
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    sensor = GraySensor(adc_reader=lambda: hardware.adc_data)
    patrol = RingPatrolController()
    fields = (
        "t", "front", "rear", "left", "right", "zone_front", "zone_rear",
        "zone_left", "zone_right", "zone_score", "white_hits", "state",
        "reason", "risk_sensor", "linear_risk", "turn_risk", "near_edge",
        "speed_level", "turn_angle", "turn_direction", "turn_duration",
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
                raw = sensor.read_raw()
                healthy = hardware.healthy and not hardware.stale(PATROL_STALE_SECONDS)
                result = patrol.update(raw, now=loop_start, healthy=healthy)
                hardware.move_cmd(result["left"], result["right"])
                observation = result["observation"]
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    **{name: raw[name] for name in GrayRiskModel.NAMES},
                    **{"zone_" + name: round(observation["zone"][name], 4)
                       for name in GrayRiskModel.NAMES},
                    "zone_score": round(observation["zone_score"], 4),
                    "white_hits": "/".join(observation["white_hits"]),
                    "state": result["state"],
                    "reason": result["reason"],
                    "risk_sensor": result["risk_sensor"],
                    "linear_risk": round(result["linear_risk"], 4),
                    "turn_risk": round(result["turn_risk"], 4),
                    "near_edge": int(observation["near_edge"]),
                    "speed_level": result["speed_level"],
                    "turn_angle": result["turn_angle"],
                    "turn_direction": result["turn_direction"],
                    "turn_duration": result["turn_duration"],
                    "left_cmd": result["left"],
                    "right_cmd": result["right"],
                    "healthy": int(healthy),
                })
                handle.flush()
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        pass
    finally:
        hardware.close()
    print("巡台测试日志：%s" % args.log)


def main():
    parser = argparse.ArgumentParser(description="巡台状态机独立真机测试")
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
