#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""六路数字红外真机扫描与 CSV 采集工具。"""

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
    DIGI_IR_ACTIVE_LEVEL,
    DIGI_IR_BITS,
    DIGI_IR_PINS,
)
from digi_ir import DigiIR  # noqa: E402


POLL_INTERVAL = 0.5
SAMPLE_RATES = (10, 50)
DEFAULT_HZ = 50


def _open_up():
    try:
        import uptech
    except ImportError as exc:
        raise RuntimeError("数字红外真机工具需要 uptech 库") from exc
    hardware = uptech.UpTech()
    hardware.ADC_IO_Open()
    return hardware


def _to_bits(mask):
    if isinstance(mask, int) and mask >= 0:
        return [(mask >> index) & 1 for index in range(8)]
    return None


def scan(hardware):
    pins = ", ".join("%s=%d" % item for item in DIGI_IR_PINS.items())
    print("扫描 8 位 IO（%s；ACTIVE_LEVEL=%d）（Ctrl+C 退出）" % (
        pins, DIGI_IR_ACTIVE_LEVEL))
    try:
        while True:
            bits = _to_bits(hardware.ADC_IO_GetAllInputLevel())
            line = "io 掩码异常" if bits is None else " ".join(
                "io%d=%d" % (index, value)
                for index, value in enumerate(bits)
            )
            sys.stdout.write("\r" + line)
            sys.stdout.flush()
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print()


def collect(hardware, out, hz, duration):
    sensor = DigiIR(bits=DIGI_IR_BITS, active_level=DIGI_IR_ACTIVE_LEVEL)
    period = 1.0 / hz
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fields = (
        "t", "mask", "io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7",
        "left_rear", "left_front", "right_rear", "right_front", "rear", "front",
        "valid",
    )
    start = time.monotonic()
    count = 0
    stopped = False
    print("采集中：%s @%dHz%s（Ctrl+C 停止）" % (
        out, hz, "" if duration else "，手动停止"))
    try:
        with open(out, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while not duration or time.monotonic() - start < duration:
                loop_start = time.monotonic()
                mask = hardware.ADC_IO_GetAllInputLevel()
                bits = _to_bits(mask)
                states = sensor.read_states(mask)
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    "mask": mask if isinstance(mask, int) else "",
                    **{"io%d" % index: bits[index] if bits is not None else ""
                       for index in range(8)},
                    **{name: int(states[name]) for name in DIGI_IR_PINS},
                    "valid": int(states["valid"]),
                })
                count += 1
                if count % hz == 0:
                    handle.flush()
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        stopped = True
        print()
    print("%s：%s，%d 行" % ("手动停止" if stopped else "完成", out, count))


def main():
    parser = argparse.ArgumentParser(description="六路数字红外真机校准工具")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("scan", help="持续显示原始 8 位 IO")
    collect_parser = sub.add_parser("collect", help="采集原始 IO 和六路状态到 CSV")
    collect_parser.add_argument("--out", required=True)
    collect_parser.add_argument("--hz", type=int, default=DEFAULT_HZ,
                                choices=SAMPLE_RATES)
    collect_parser.add_argument("--dur", type=float, default=0.0)
    args = parser.parse_args()

    hardware = _open_up()
    try:
        if args.command == "scan":
            scan(hardware)
        else:
            collect(hardware, args.out, args.hz, args.dur)
    finally:
        hardware.ADC_IO_Close()


if __name__ == "__main__":
    main()
