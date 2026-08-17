#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hunt 视觉、红外与电机命令同步 CSV 采集；--drive 才启用电机。"""

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
    DIGI_IR_ACTIVE_LEVEL,
    DIGI_IR_BITS,
    HUNT_COLLECT_LOG_DIR,
    HUNT_COLLECT_SECONDS,
    SHOVEL_ADC_MAX,
    SHOVEL_IR_CHANNELS,
    VISION_CAMERA_DEVICE,
    VISION_LOOP_HZ,
    VISION_MAX_AGE_MS,
)
from digi_ir import DigiIR  # noqa: E402
from hunt import HuntController  # noqa: E402
from ir import IrSensor  # noqa: E402
from shovel_guard import ShovelGuard  # noqa: E402


ADC_FIELDS = tuple("adc%d" % index for index in range(10))
IO_FIELDS = tuple("io%d" % index for index in range(8))
FIELDS = (
    "t", "label", "measured_distance_cm", "sequence", "vision_timestamp_ms",
    "received_age_ms", "vision_valid", "vision_status", "vision_error",
    "frame_width", "frame_height", "action", "fps", "inference_ms",
    "detection_count", "detection_index", "selected_target", "class_id",
    "target_type", "confidence", "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2",
    "bbox_width", "bbox_height", "bbox_area", "bbox_area_ratio", "center_x",
    "center_y", "offset_x", "offset_y", "model_distance_cm", "sensor_valid",
    "sensor_error", "io_mask", "hunt_mode", "hunt_state", "hunt_reason",
    "hunt_owns_control", "near_direction", "good_offset_x", "bad_offset_x",
    "good_confidence", "good_acquire_count", "good_miss_count", "good_locked",
    "shovel_left", "shovel_right", "shovel_state", "shovel_active",
    "shovel_hang",
    "left_cmd", "right_cmd", "motor_enabled",
) + ADC_FIELDS + IO_FIELDS


def _safe_label(label):
    safe = "".join(
        char if char.isalnum() or char in "-_" else "_"
        for char in label.strip()
    ).strip("_")
    return safe or "hunt"


def default_output_path(label):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    filename = "%s_%s.csv" % (_safe_label(label), stamp)
    return os.path.join(ROOT, HUNT_COLLECT_LOG_DIR, filename)


def _to_bits(mask):
    if not isinstance(mask, int) or mask < 0:
        return [""] * 8
    return [(mask >> index) & 1 for index in range(8)]


def read_sensors(hardware):
    try:
        adc = list(hardware.ADC_Get_All_Channle())
        mask = hardware.ADC_IO_GetAllInputLevel()
        if len(adc) < 10 or not isinstance(mask, int):
            raise ValueError("ADC/IO 数据形状无效")
        return {
            "valid": 1,
            "error": "",
            "adc": adc[:10],
            "mask": mask,
            "io": _to_bits(mask),
        }
    except Exception as error:
        return {
            "valid": 0,
            "error": str(error),
            "adc": [""] * 10,
            "mask": "",
            "io": [""] * 8,
        }


def read_controller_sensors(controller):
    try:
        if controller.stale():
            raise RuntimeError("UpController 传感器数据过期")
        adc = list(controller.adc_data)
        io = list(controller.io_data)
        if len(adc) < 10 or len(io) < 8:
            raise ValueError("ADC/IO 数据形状无效")
        mask = sum((int(bit) & 1) << index for index, bit in enumerate(io[:8]))
        return {
            "valid": 1,
            "error": "",
            "adc": adc[:10],
            "mask": mask,
            "io": io[:8],
        }
    except Exception as error:
        return {
            "valid": 0,
            "error": str(error),
            "adc": [""] * 10,
            "mask": "",
            "io": [""] * 8,
        }


def _bbox_values(detection, frame_width, frame_height):
    bbox = detection.get("bbox") if detection else None
    if not isinstance(bbox, list) or len(bbox) != 4:
        return ("",) * 8
    x1, y1, x2, y2 = bbox
    width = max(0.0, float(x2) - float(x1))
    height = max(0.0, float(y2) - float(y1))
    area = width * height
    frame_area = float(frame_width or 0) * float(frame_height or 0)
    ratio = area / frame_area if frame_area > 0.0 else ""
    return x1, y1, x2, y2, width, height, area, ratio


def rows_for_result(raw, sensors, label, measured_distance_cm, elapsed,
                    hunt_result=None, motor_enabled=False):
    if raw is None:
        raw = {
            "status": "no_data_or_stale",
            "action": "search",
            "detections": [],
        }
    detections = raw.get("detections") or []
    hunt_result = hunt_result or {
        "mode": "collect_only",
        "state": "COLLECT_ONLY",
        "reason": "未启用 hunt 控制",
        "owns_control": False,
        "target": None,
        "near_direction": None,
        "good_offset_x": None,
        "bad_offset_x": None,
        "left": 0,
        "right": 0,
    }
    target = hunt_result.get("target")
    vision_valid = int(raw.get("status") != "error" and raw.get("sequence") is not None)
    common = {
        "t": round(float(elapsed), 6),
        "label": label,
        "measured_distance_cm": "" if measured_distance_cm is None else measured_distance_cm,
        "sequence": raw.get("sequence"),
        "vision_timestamp_ms": raw.get("timestamp_ms"),
        "received_age_ms": raw.get("age_ms"),
        "vision_valid": vision_valid,
        "vision_status": raw.get("status"),
        "vision_error": raw.get("error", ""),
        "frame_width": raw.get("frame_width"),
        "frame_height": raw.get("frame_height"),
        "action": raw.get("action"),
        "fps": raw.get("fps"),
        "inference_ms": raw.get("inference_ms"),
        "detection_count": len(detections),
        "sensor_valid": sensors["valid"],
        "sensor_error": sensors["error"],
        "io_mask": sensors["mask"],
        "hunt_mode": hunt_result.get("mode"),
        "hunt_state": hunt_result.get("state"),
        "hunt_reason": hunt_result.get("reason"),
        "hunt_owns_control": int(bool(hunt_result.get("owns_control"))),
        "near_direction": hunt_result.get("near_direction"),
        "good_offset_x": hunt_result.get("good_offset_x"),
        "bad_offset_x": hunt_result.get("bad_offset_x"),
        "good_confidence": hunt_result.get("good_confidence"),
        "good_acquire_count": hunt_result.get("good_acquire_count"),
        "good_miss_count": hunt_result.get("good_miss_count"),
        "good_locked": int(bool(hunt_result.get("good_locked", False))),
        "shovel_left": hunt_result.get("shovel_left"),
        "shovel_right": hunt_result.get("shovel_right"),
        "shovel_state": hunt_result.get("shovel_state"),
        "shovel_active": int(bool(hunt_result.get("shovel_active", False))),
        "shovel_hang": int(bool(hunt_result.get("shovel_hang", False))),
        "left_cmd": hunt_result.get("left", 0),
        "right_cmd": hunt_result.get("right", 0),
        "motor_enabled": int(bool(motor_enabled)),
        **{name: sensors["adc"][index] for index, name in enumerate(ADC_FIELDS)},
        **{name: sensors["io"][index] for index, name in enumerate(IO_FIELDS)},
    }
    if not detections:
        return [{**common, **{name: "" for name in FIELDS if name not in common}}]

    rows = []
    for index, detection in enumerate(detections):
        bbox = _bbox_values(
            detection, raw.get("frame_width"), raw.get("frame_height")
        )
        rows.append({
            **common,
            "detection_index": index,
            "selected_target": int(detection == target),
            "class_id": detection.get("class_id"),
            "target_type": detection.get("type"),
            "confidence": detection.get("confidence"),
            "bbox_x1": bbox[0],
            "bbox_y1": bbox[1],
            "bbox_x2": bbox[2],
            "bbox_y2": bbox[3],
            "bbox_width": bbox[4],
            "bbox_height": bbox[5],
            "bbox_area": bbox[6],
            "bbox_area_ratio": bbox[7],
            "center_x": detection.get("center_x"),
            "center_y": detection.get("center_y"),
            "offset_x": detection.get("offset_x"),
            "offset_y": detection.get("offset_y"),
            "model_distance_cm": detection.get("distance_cm"),
        })
    return rows


def _result_key(raw):
    if raw is None:
        return "status", "no_data_or_stale"
    sequence = raw.get("sequence")
    if sequence is not None:
        return "sequence", sequence
    return "status", raw.get("status"), raw.get("error")


def _open_hardware():
    try:
        import uptech
    except ImportError as error:
        raise RuntimeError("bad 采集工具需要树莓派 uptech 库") from error
    hardware = uptech.UpTech()
    hardware.ADC_IO_Open()
    return hardware


def collect(args):
    if YOLO_DIR not in sys.path:
        sys.path.insert(0, YOLO_DIR)
    from rpi_yolo_api import VisionClient

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    direct_hardware = None
    controller = None
    if args.drive:
        print("实车 hunt 模式：把车放在平整空地，确保四周无人且不会掉台。")
        if input("确认安全并启用电机请输入 DRIVE：").strip() != "DRIVE":
            print("未完成安全确认，测试取消。")
            return
        from up_controller import UpController
        controller = UpController(
            poll_hz=args.hz,
            motor_invert=args.motor_invert,
            motor_swap=args.motor_swap,
        )
    else:
        direct_hardware = _open_hardware()

    hunt = HuntController()
    digi = DigiIR(bits=DIGI_IR_BITS, active_level=DIGI_IR_ACTIVE_LEVEL)
    shovel_sensor = IrSensor(
        channels=SHOVEL_IR_CHANNELS,
        adc_max=SHOVEL_ADC_MAX,
    )
    shovel_guard = ShovelGuard()
    start = time.monotonic()
    last_log_key = object()
    period = 1.0 / args.hz
    row_count = 0
    print("hunt 追踪与近距 bad 避让同步采集：%s（电机=%s）" % (
        args.log, "启用" if controller is not None else "关闭",
    ))
    try:
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            command = (
                "rpi-yolo", "--report-every", "0",
                "--camera", VISION_CAMERA_DEVICE,
            )
            with VisionClient(command=command) as vision:
                while not args.seconds or time.monotonic() - start < args.seconds:
                    loop_started = time.monotonic()
                    raw = vision.get_raw(max_age_ms=args.max_age_ms)
                    if controller is not None:
                        sensors = read_controller_sensors(controller)
                    else:
                        sensors = read_sensors(direct_hardware)
                    if sensors["valid"]:
                        ir_states = digi.read_states(sensors["io"])
                    else:
                        ir_states = {
                            name: False for name in DIGI_IR_BITS
                        } | {"valid": False}
                    result = hunt.update(
                        raw, ir_states, now=loop_started,
                        healthy=bool(sensors["valid"]),
                    )
                    if sensors["valid"]:
                        shovel_raw = shovel_sensor.read_raw(sensors["adc"])
                    else:
                        shovel_raw = {"left": 0.0, "right": 0.0, "valid": False}
                    shovel_active = (
                        result.get("state") == "GOOD_PUSH"
                        or shovel_guard.state != "IDLE"
                    )
                    guard_result = shovel_guard.update(
                        shovel_raw, active=shovel_active,
                        now=loop_started, healthy=bool(sensors["valid"]),
                    )
                    result = dict(result)
                    result.update({
                        "shovel_left": shovel_raw["left"],
                        "shovel_right": shovel_raw["right"],
                        "shovel_state": guard_result["state"],
                        "shovel_active": shovel_active,
                        "shovel_hang": guard_result["hang"],
                    })
                    if guard_result["state"] != "IDLE":
                        if (guard_result["state"] in ("REVERSE", "SAFE_STOP")
                                and result.get("state") == "GOOD_PUSH"):
                            hunt.finish_push()
                        result.update({
                            "left": guard_result["left"],
                            "right": guard_result["right"],
                            "owns_control": True,
                            "mode": "shovel_guard",
                            "state": guard_result["state"],
                            "reason": guard_result["reason"],
                        })
                    if controller is not None:
                        controller.move_cmd(result["left"], result["right"])
                    log_key = (
                        _result_key(raw), result["state"],
                        result["left"], result["right"],
                        result.get("near_direction"),
                    )
                    if log_key != last_log_key:
                        rows = rows_for_result(
                            raw, sensors, args.label, args.distance_cm,
                            loop_started - start,
                            hunt_result=result,
                            motor_enabled=controller is not None,
                        )
                        writer.writerows(rows)
                        handle.flush()
                        row_count += len(rows)
                        detection_count = 0 if raw is None else len(raw.get("detections") or [])
                        sequence = None if raw is None else raw.get("sequence")
                        action = "search" if raw is None else raw.get("action")
                        print("seq=%s action=%s detections=%d state=%s cmd=(%d,%d) sensor_valid=%d" % (
                            sequence, action, detection_count, result["state"],
                            result["left"], result["right"], sensors["valid"],
                        ))
                        last_log_key = log_key
                    time.sleep(max(0.0, period - (time.monotonic() - loop_started)))
    except KeyboardInterrupt:
        pass
    finally:
        if controller is not None:
            controller.close()
        if direct_hardware is not None:
            direct_hardware.ADC_IO_Close()
    print("采集完成：%s，共 %d 行" % (args.log, row_count))


def main():
    parser = argparse.ArgumentParser(description="hunt 视觉、红外与电机命令同步采集")
    sub = parser.add_subparsers(dest="command", required=True)
    collect_parser = sub.add_parser("collect", help="每个新视觉帧写入全部 detections 和传感器")
    collect_parser.add_argument("--label", required=True, help="场景标签，如 hunt_near_bad")
    collect_parser.add_argument("--distance-cm", type=float, default=None,
                                help="人工测距记录；不参与 hunt 控制")
    collect_parser.add_argument("--seconds", type=float, default=HUNT_COLLECT_SECONDS)
    collect_parser.add_argument("--hz", type=float, default=VISION_LOOP_HZ)
    collect_parser.add_argument("--max-age-ms", type=float, default=VISION_MAX_AGE_MS)
    collect_parser.add_argument("--log", default=None)
    collect_parser.add_argument(
        "--drive", action="store_true",
        help="现场输入 DRIVE 后通过 move_cmd 执行 hunt",
    )
    collect_parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
    )
    collect_parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    collect_parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    collect_parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    collect_parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    args = parser.parse_args()
    if args.command == "collect":
        if args.seconds < 0.0 or args.hz <= 0.0 or args.max_age_ms <= 0.0:
            parser.error("--hz/--max-age-ms 必须为正，--seconds 不能为负")
        if args.distance_cm is not None and args.distance_cm < 0.0:
            parser.error("--distance-cm 不能为负数")
        if not args.log:
            args.log = default_output_path(args.label)
        collect(args)


if __name__ == "__main__":
    main()
