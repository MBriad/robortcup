#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四路灰度真机扫描与 CSV 采集工具。"""

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
    GRAY_ADC_MAX,
    GRAY_CHANNELS,
    GRAY_WHITE_ENTER,
)
from dev import prompt_output_path  # noqa: E402
from gray import GraySensor  # noqa: E402


POLL_INTERVAL = 0.5
SAMPLE_RATES = (10, 50)
DEFAULT_HZ = 50


def _default_output_path():
    filename = time.strftime("gray_%Y%m%d_%H%M%S.csv")
    return os.path.join(ROOT, "data", filename)


def _open_up():
    try:
        import uptech
    except ImportError as exc:
        raise RuntimeError("灰度真机工具需要 uptech 库") from exc
    hardware = uptech.UpTech()
    hardware.ADC_IO_Open()
    return hardware


def scan(hardware):
    sensor = GraySensor(
        channels=GRAY_CHANNELS,
        adc_max=GRAY_ADC_MAX,
        white_enter=GRAY_WHITE_ENTER,
    )
    channels = ", ".join(
        "%s=adc%d" % (name, GRAY_CHANNELS[name])
        for name in ("front", "rear", "left", "right")
    )
    print("扫描灰度通道（%s）（Ctrl+C 退出）" % channels)
    try:
        while True:
            raw = sensor.read_raw(hardware.ADC_Get_All_Channle())
            line = " ".join(
                "%s=%5d" % (name, raw[name])
                for name in ("front", "rear", "left", "right")
            )
            sys.stdout.write("\r" + line)
            sys.stdout.flush()
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print()


def collect(hardware, out, hz, duration):
    sensor = GraySensor(
        channels=GRAY_CHANNELS,
        adc_max=GRAY_ADC_MAX,
        white_enter=GRAY_WHITE_ENTER,
    )
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
                handle, fieldnames=("t", "front", "rear", "left", "right")
            )
            writer.writeheader()
            while not duration or time.monotonic() - start < duration:
                loop_start = time.monotonic()
                raw = sensor.read_raw(hardware.ADC_Get_All_Channle())
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    **raw,
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
    parser = argparse.ArgumentParser(description="四路灰度真机校准工具")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("scan", help="持续显示映射后的四路灰度")
    collect_parser = sub.add_parser("collect", help="采集四路灰度到 CSV")
    collect_parser.add_argument("--out", default=None,
                                help="输出路径；不给则交互输入文件名")
    collect_parser.add_argument("--hz", type=int, default=DEFAULT_HZ,
                                choices=SAMPLE_RATES)
    collect_parser.add_argument("--dur", type=float, default=0.0)
    args = parser.parse_args()
    if args.command == "collect" and not args.out:
        args.out = prompt_output_path(_default_output_path())

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
