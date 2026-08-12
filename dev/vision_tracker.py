#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO good 能量块对准真机测试；运行时同步保存每个新视觉结果。"""

import argparse
import csv
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YOLO_DIR = os.path.join(ROOT, "rpi-yolo-pi4-int8-lto-8fps")
if ROOT in sys.path:
    sys.path.remove(ROOT)
sys.path.insert(0, ROOT)

from config import (  # noqa: E402
    CHASSIS_MOTOR_INVERT,
    CHASSIS_MOTOR_SWAP,
    VISION_LOOP_HZ,
    VISION_MAX_AGE_MS,
)
from vision_tracker import VisionTracker  # noqa: E402


FIELDS = (
    "t", "sequence", "vision_timestamp_ms", "received_age_ms", "valid",
    "reason", "frame_width", "frame_height", "action", "target_type",
    "confidence", "center_x", "center_y",
    "offset_x", "offset_y", "distance_cm", "dt", "error_x_normalized",
    "filtered_error", "turn_command", "state", "left_cmd", "right_cmd",
    "motor_enabled",
)


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(ROOT, "data", "vision_tracker_%s.csv" % stamp)


def _result_key(control):
    sequence = control.get("sequence")
    if sequence is not None:
        return "sequence", sequence
    return "status", control.get("valid"), control.get("reason"), control.get("action")


def _rounded(value, digits=6):
    return "" if value is None else round(float(value), digits)


def run(args):
    if YOLO_DIR not in sys.path:
        sys.path.insert(0, YOLO_DIR)
    from rpi_yolo_api import VisionClient

    hardware = None
    if args.drive:
        print("实车转向模式：把车放在平整空地，确保周围没有人和障碍物。")
        if input("确认安全并启用电机请输入 DRIVE：").strip() != "DRIVE":
            print("未完成安全确认，测试取消。")
            return
        from up_controller import UpController
        hardware = UpController(
            motor_invert=args.motor_invert,
            motor_swap=args.motor_swap,
        )

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    tracker = VisionTracker()
    start = time.monotonic()
    last_key = object()
    last_frame_at = None
    period = 1.0 / args.hz
    try:
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            with VisionClient() as vision:
                while not args.seconds or time.monotonic() - start < args.seconds:
                    loop_started = time.monotonic()
                    control = vision.get_control(max_age_ms=args.max_age_ms)
                    key = _result_key(control)
                    if key != last_key:
                        dt = None if last_frame_at is None else loop_started - last_frame_at
                        result = tracker.update(control)
                        if hardware is not None:
                            hardware.move_cmd(result["left"], result["right"])
                        writer.writerow({
                            "t": round(loop_started - start, 6),
                            "sequence": control.get("sequence"),
                            "vision_timestamp_ms": control.get("timestamp_ms"),
                            "received_age_ms": _rounded(control.get("age_ms"), 3),
                            "valid": int(bool(control.get("valid"))),
                            "reason": control.get("reason"),
                            "frame_width": control.get("frame_width"),
                            "frame_height": control.get("frame_height"),
                            "action": control.get("action"),
                            "target_type": control.get("target_type"),
                            "confidence": control.get("confidence"),
                            "center_x": control.get("center_x"),
                            "center_y": control.get("center_y"),
                            "offset_x": control.get("offset_x"),
                            "offset_y": control.get("offset_y"),
                            "distance_cm": control.get("distance_cm"),
                            "dt": _rounded(dt),
                            "error_x_normalized": _rounded(result["error_x"]),
                            "filtered_error": _rounded(result["filtered_error"]),
                            "turn_command": result["turn_command"],
                            "state": result["state"],
                            "left_cmd": result["left"],
                            "right_cmd": result["right"],
                            "motor_enabled": int(hardware is not None),
                        })
                        handle.flush()
                        print(
                            "seq=%s action=%s offset=%s state=%s cmd=(%d,%d)" % (
                                control.get("sequence"), control.get("action"),
                                control.get("offset_x"), result["state"],
                                result["left"], result["right"],
                            )
                        )
                        last_key = key
                        last_frame_at = loop_started
                    time.sleep(max(0.0, period - (time.monotonic() - loop_started)))
    except KeyboardInterrupt:
        pass
    finally:
        if hardware is not None:
            hardware.close()
    print("视觉追踪日志：%s" % args.log)


def main():
    parser = argparse.ArgumentParser(
        description="YOLO good 能量块对准与同步 CSV 采集（默认不启用电机）"
    )
    parser.add_argument("--drive", action="store_true", help="安全确认后启用 move_cmd")
    parser.add_argument("--hz", type=float, default=VISION_LOOP_HZ)
    parser.add_argument("--max-age-ms", type=float, default=VISION_MAX_AGE_MS)
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
    if args.hz <= 0.0 or args.max_age_ms <= 0.0 or args.seconds < 0.0:
        parser.error("--hz/--max-age-ms 必须为正，--seconds 不能为负")
    run(args)


if __name__ == "__main__":
    main()
