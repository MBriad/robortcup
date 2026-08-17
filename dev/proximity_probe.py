#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""近物候选转向与视觉确认独立真机测试。"""

import argparse
import csv
import os
import sys
import time

DEV_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(DEV_DIR)
YOLO_DIR = os.path.join(ROOT, "rpi-yolo-pi4-int8-lto-8fps")
sys.path[:] = [
    path for path in sys.path
    if os.path.abspath(path or os.getcwd()) not in (DEV_DIR, ROOT)
]
sys.path.insert(0, ROOT)

from config import (  # noqa: E402
    CHASSIS_MOTOR_INVERT,
    CHASSIS_MOTOR_SWAP,
    DIGI_IR_ACTIVE_LEVEL,
    DIGI_IR_BITS,
    GRAY_ADC_MAX,
    GRAY_CENTER_REFERENCE,
    GRAY_CHANNELS,
    GRAY_EDGE_REFERENCE,
    GRAY_FILTER_WINDOW,
    GRAY_NEAR_EDGE_CLEAR,
    GRAY_NEAR_EDGE_ENTER,
    GRAY_WHITE_CLEAR,
    GRAY_WHITE_ENTER,
    GRAY_WHITE_REFERENCE,
    SHOVEL_ADC_MAX,
    SHOVEL_IR_CHANNELS,
    PROBE_DEV_HZ,
    PROBE_DEV_LABEL,
    PROBE_DEV_SECONDS,
    PROBE_LOG_DIR,
    PROBE_STALE_SECONDS,
    VISION_CAMERA_DEVICE,
    VISION_MAX_AGE_MS,
)
from digi_ir import DigiIR  # noqa: E402
from proximity_probe import ProximityProbeController  # noqa: E402
from gray import GrayRiskModel, GraySensor  # noqa: E402
from ir import IrSensor  # noqa: E402
from shovel_guard import ShovelGuard  # noqa: E402


FIELDS = (
    "t", "label", "mode", "state", "reason", "left_cmd", "right_cmd",
    "motor_enabled", "healthy", "vision_sequence", "vision_status",
    "vision_has_good", "vision_has_bad",
    "probe_state", "probe_source_direction",
    "probe_turn_direction", "enemy_slow", "enemy_confirmed",
    "probe_vision_count", "probe_vision_verdict",
    "shovel_state", "shovel_active",
    "ir_front", "ir_left_front", "ir_right_front", "ir_left_rear",
    "ir_right_rear", "ir_rear", "gray_front", "gray_rear", "gray_left",
    "gray_right", "shovel_left", "shovel_right",
)


def default_log_path(label):
    safe = "".join(
        char if char.isalnum() or char in "-_" else "_"
        for char in label.strip()
    ).strip("_") or "proximity_probe"
    return os.path.join(
        ROOT, PROBE_LOG_DIR,
        "%s_%s.csv" % (safe, time.strftime("%Y%m%d_%H%M%S"))
    )


def run(args):
    if args.drive:
        print("近物候选真机测试：架起车轮确认转向，再放到不会掉台的场地。")
        if input("确认安全并启用电机请输入 DRIVE：").strip() != "DRIVE":
            print("未完成安全确认，测试取消。")
            return

    if YOLO_DIR not in sys.path:
        sys.path.insert(0, YOLO_DIR)
    from rpi_yolo_api import VisionClient
    from up_controller import UpController

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
    gray_model = GrayRiskModel(
        window=GRAY_FILTER_WINDOW,
        edge_reference=GRAY_EDGE_REFERENCE,
        center_reference=GRAY_CENTER_REFERENCE,
        white_reference=GRAY_WHITE_REFERENCE,
        white_enter=GRAY_WHITE_ENTER,
        white_clear=GRAY_WHITE_CLEAR,
        near_edge_enter=GRAY_NEAR_EDGE_ENTER,
        near_edge_clear=GRAY_NEAR_EDGE_CLEAR,
        adc_max=GRAY_ADC_MAX,
    )
    shovel_sensor = IrSensor(
        adc_reader=lambda: hardware.adc_data,
        channels=SHOVEL_IR_CHANNELS,
        adc_max=SHOVEL_ADC_MAX,
    )
    probe = ProximityProbeController()
    shovel_guard = ShovelGuard()
    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    start = time.monotonic()
    period = 1.0 / args.hz
    vision = None
    try:
        vision = VisionClient(command=(
            "rpi-yolo", "--report-every", "0",
            "--camera", VISION_CAMERA_DEVICE,
        )).start()
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            while not args.seconds or time.monotonic() - start < args.seconds:
                loop_start = time.monotonic()
                gray = gray_sensor.read_raw()
                observation = gray_model.update(gray)
                ir = digi.read_states()
                shovel = shovel_sensor.read_raw()
                vision_raw = (
                    vision.get_raw(max_age_ms=VISION_MAX_AGE_MS)
                    or {"status": "stale"}
                )
                detections = list(vision_raw.get("detections") or [])
                target = vision_raw.get("target")
                if isinstance(target, dict):
                    detections.append(target)
                vision_has_good = any(
                    item.get("type") == "good" for item in detections
                )
                vision_has_bad = any(
                    item.get("type") == "bad" for item in detections
                )
                probe_vision = {
                    "sequence": vision_raw.get("sequence"),
                    "status": vision_raw.get("status", "stale"),
                    "has_good": vision_has_good,
                    "has_bad": vision_has_bad,
                }
                healthy = hardware.healthy and not hardware.stale(PROBE_STALE_SECONDS)
                result = probe.update(
                    ir, observation,
                    vision=probe_vision,
                    now=loop_start, healthy=healthy, allow_start=True,
                )
                guard_result = shovel_guard.update(
                    shovel,
                    active=(
                        result["state"] == "ENEMY_PUSH"
                        or shovel_guard.state != "IDLE"
                    ),
                    now=loop_start,
                    healthy=healthy,
                )
                if guard_result["state"] != "IDLE":
                    if result["state"] == "ENEMY_PUSH":
                        probe.finish_push()
                    result = {
                        **result,
                        "left": guard_result["left"],
                        "right": guard_result["right"],
                        "owns_control": True,
                        "state": guard_result["state"],
                        "reason": guard_result["reason"],
                    }
                mode = (
                    "enemy_push" if result["state"] == "ENEMY_PUSH"
                    else "proximity_probe" if result["owns_control"]
                    else "probe_idle"
                )
                left = result["left"] if result["owns_control"] else 0
                right = result["right"] if result["owns_control"] else 0
                if args.drive:
                    hardware.move_cmd(left, right)
                else:
                    hardware.move_cmd(0, 0)
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    "label": args.label,
                    "mode": mode,
                    "state": result["state"],
                    "reason": result["reason"],
                    "left_cmd": left,
                    "right_cmd": right,
                    "motor_enabled": int(args.drive),
                    "healthy": int(healthy),
                    "vision_sequence": vision_raw.get("sequence") if vision_raw else None,
                    "vision_status": vision_raw.get("status") if vision_raw else "stale",
                    "vision_has_good": int(vision_has_good),
                    "vision_has_bad": int(vision_has_bad),
                    "probe_state": probe.state,
                    "probe_source_direction": result.get("source_direction"),
                    "probe_turn_direction": result.get("turn_direction"),
                    "enemy_slow": int(bool(result.get("slow", False))),
                    "enemy_confirmed": int(bool(result.get("confirmed", False))),
                    "probe_vision_count": result.get("vision_count", 0),
                    "probe_vision_verdict": result.get("vision_verdict"),
                    "shovel_state": guard_result["state"],
                    "shovel_active": int(guard_result["state"] != "IDLE"),
                    **{"ir_" + name: int(bool(ir[name])) for name in (
                        "front", "left_front", "right_front", "left_rear",
                        "right_rear", "rear",
                    )},
                    **{"gray_" + name: gray[name] for name in (
                        "front", "rear", "left", "right",
                    )},
                    "shovel_left": shovel["left"],
                    "shovel_right": shovel["right"],
                })
                handle.flush()
                print("\rmode=%s state=%s cmd=(%d,%d) vision=%s" % (
                    mode, result["state"], left, right,
                    vision_raw.get("status") if vision_raw else "stale",
                ), end="", flush=True)
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print()
    finally:
        if vision is not None:
            vision.close()
        hardware.close()
    print("近物候选日志：%s" % args.log)


def main():
    parser = argparse.ArgumentParser(description="近物候选转向与视觉确认真机测试")
    parser.add_argument("--label", default=PROBE_DEV_LABEL)
    parser.add_argument("--seconds", type=float, default=PROBE_DEV_SECONDS,
                        help="0 表示运行到 Ctrl+C")
    parser.add_argument("--hz", type=float, default=PROBE_DEV_HZ)
    parser.add_argument("--log", default=None)
    parser.add_argument("--drive", action="store_true")
    parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
    )
    parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    args = parser.parse_args()
    if args.seconds < 0.0 or args.hz <= 0.0:
        parser.error("--hz 必须为正，--seconds 不能为负")
    if not args.log:
        args.log = default_log_path(args.label)
    run(args)


if __name__ == "__main__":
    main()
