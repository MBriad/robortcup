#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""前头两路模拟红外 ADC 的真机扫描与 CSV 采集工具。"""

import argparse
import csv
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT in sys.path:
    sys.path.remove(ROOT)
sys.path.insert(0, ROOT)

from config import IR_ADC_MAX, IR_CHANNELS  # noqa: E402
from ir import IrSensor  # noqa: E402


POLL_INTERVAL = 0.5
SAMPLE_RATES = (10, 50)
DEFAULT_HZ = 50


def _default_output_path():
    filename = time.strftime("front_adc_%Y%m%d_%H%M%S.csv")
    return os.path.join(ROOT, "data", filename)


def _open_up():
    try:
        import uptech
    except ImportError as exc:
        raise RuntimeError("模拟红外真机工具需要 uptech 库") from exc
    hardware = uptech.UpTech()
    hardware.ADC_IO_Open()
    return hardware


def scan(hardware, mapped):
    sensor = IrSensor(channels=IR_CHANNELS, adc_max=IR_ADC_MAX)
    marks = {IR_CHANNELS["left"]: "L", IR_CHANNELS["right"]: "R"}
    if mapped:
        print("扫描红外通道（left=adc%d, right=adc%d）（Ctrl+C 退出）" % (
            IR_CHANNELS["left"], IR_CHANNELS["right"]))
    else:
        print("扫描 10 路 ADC（L=前左红外，R=前右红外）（Ctrl+C 退出）")
    try:
        while True:
            adc = hardware.ADC_Get_All_Channle()
            if mapped:
                raw = sensor.read_raw(adc)
                diff = sensor.diff(raw)
                line = "left=%5d right=%5d diff=%6d valid=%d" % (
                    raw["left"], raw["right"], diff or 0,
                    int(raw["valid"]))
            else:
                line = " ".join(
                    "adc%d%s=%5d" % (
                        index, marks.get(index, ""),
                        adc[index] if 0 <= index < len(adc) else -1,
                    )
                    for index in range(10)
                )
            sys.stdout.write("\r" + line)
            sys.stdout.flush()
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print()


def collect(hardware, out, hz, duration):
    sensor = IrSensor(channels=IR_CHANNELS, adc_max=IR_ADC_MAX)
    period = 1.0 / hz
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    start = time.monotonic()
    count = 0
    stopped = False
    print("采集中：%s @%dHz%s（Ctrl+C 停止）" % (
        out, hz, "" if duration else "，手动停止"))
    try:
        with open(out, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle, fieldnames=("t", "left", "right", "diff", "valid")
            )
            writer.writeheader()
            while not duration or time.monotonic() - start < duration:
                loop_start = time.monotonic()
                raw = sensor.read_raw(hardware.ADC_Get_All_Channle())
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    "left": raw["left"],
                    "right": raw["right"],
                    "diff": sensor.diff(raw),
                    "valid": int(raw["valid"]),
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
    parser = argparse.ArgumentParser(description="前头两路模拟红外 ADC 采集工具")
    sub = parser.add_subparsers(dest="command", required=True)
    scan_parser = sub.add_parser("scan", help="持续显示 ADC 或映射后的红外值")
    scan_parser.add_argument("--mapped", action="store_true")
    collect_parser = sub.add_parser("collect", help="采集前头 ADC 到 CSV")
    collect_parser.add_argument("--out", default=None,
                                help="输出路径；默认写入 data/front_adc_时间.csv")
    collect_parser.add_argument("--hz", type=int, default=DEFAULT_HZ,
                                choices=SAMPLE_RATES)
    collect_parser.add_argument("--dur", type=float, default=0.0)
    args = parser.parse_args()

    hardware = _open_up()
    try:
        if args.command == "scan":
            scan(hardware, args.mapped)
        else:
            collect(hardware, args.out or _default_output_path(),
                    args.hz, args.dur)
    finally:
        hardware.ADC_IO_Close()


if __name__ == "__main__":
    main()
