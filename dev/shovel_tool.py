#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""铲子底下两路模拟红外的真机扫描与 CSV 采集工具（定悬空/收回阈值用）。"""

import argparse
import csv
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT in sys.path:
    sys.path.remove(ROOT)
sys.path.insert(0, ROOT)

from config import IR_ADC_MAX, SHOVEL_ADC_MAX, SHOVEL_IR_CHANNELS  # noqa: E402
from ir import IrSensor  # noqa: E402


POLL_INTERVAL = 0.5
SAMPLE_RATES = (10, 50)
DEFAULT_HZ = 50


def _default_output_path():
    filename = time.strftime("shovel_%Y%m%d_%H%M%S.csv")
    return os.path.join(ROOT, "data", filename)


def _ask_group():
    """交互选择采集分组：1=悬空 2=台内 3=自定义。返回文件名前缀或 None。"""
    while True:
        choice = input(
            "这组数据是：1=悬空(铲子出台) 2=台内(铲子在台上) 3=自定义 ？ "
        ).strip()
        if choice == "1":
            return "shovel_hang"
        if choice == "2":
            return "shovel_stage"
        if choice == "3":
            return None
        print("请输入 1 / 2 / 3")


def _ask_output_path(prefix):
    """交互询问输出路径；prefix=None 为自定义（不加自动前缀）。

    文件名关键词供 dev/calibrate_shovel.py 分类：
    hang → 悬空组；stage → 台内组；不含关键词的（自定义）文件不参与标定。
    """
    default = _default_output_path()
    if prefix:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        default = os.path.join(ROOT, "data", "%s_%s.csv" % (prefix, stamp))
    else:
        print("提示：文件名含 hang（悬空）或 stage（台内）会被 calibrate 识别；"
              "不含关键词的文件不参与标定。")
    name = input("输出文件名（回车用默认 %s）：" % os.path.basename(default)).strip()
    if not name:
        return default
    if prefix and not os.path.isabs(name):
        name = "%s_%s" % (prefix, name)
    if not name.lower().endswith(".csv"):
        name += ".csv"
    return os.path.join(ROOT, "data", name)


def _open_up():
    try:
        import uptech
    except ImportError as exc:
        raise RuntimeError("铲子红外真机工具需要 uptech 库") from exc
    hardware = uptech.UpTech()
    hardware.ADC_IO_Open()
    return hardware


def scan(hardware):
    sensor = IrSensor(channels=SHOVEL_IR_CHANNELS, adc_max=SHOVEL_ADC_MAX)
    marks = {SHOVEL_IR_CHANNELS["left"]: "L", SHOVEL_IR_CHANNELS["right"]: "R"}
    print("扫描 10 路 ADC（L/R=铲子底下两路；先看全 10 路确认接线，Ctrl+C 退出）")
    try:
        while True:
            adc = hardware.ADC_Get_All_Channle()
            raw = sensor.read_raw(adc)
            line = " ".join(
                "adc%d%s=%5d" % (
                    index, marks.get(index, ""),
                    adc[index] if 0 <= index < len(adc) else -1,
                )
                for index in range(10)
            )
            line += "  shovel(left=%5d right=%5d valid=%d)" % (
                raw["left"], raw["right"], int(raw["valid"]))
            sys.stdout.write("\r" + line)
            sys.stdout.flush()
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print()


def collect(hardware, out, hz, duration):
    sensor = IrSensor(channels=SHOVEL_IR_CHANNELS, adc_max=SHOVEL_ADC_MAX)
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
                handle, fieldnames=("t", "left", "right", "valid")
            )
            writer.writeheader()
            while not duration or time.monotonic() - start < duration:
                loop_start = time.monotonic()
                raw = sensor.read_raw(hardware.ADC_Get_All_Channle())
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    "left": raw["left"],
                    "right": raw["right"],
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
    parser = argparse.ArgumentParser(description="铲子底下模拟红外 ADC 采集工具")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("scan", help="扫描全 10 路 ADC（L/R 标记铲子通道）")
    collect_parser = sub.add_parser("collect", help="采集铲子两路到 CSV")
    collect_parser.add_argument("--out", default=None,
                                help="输出路径；不给则交互输入文件名")
    collect_parser.add_argument("--hz", type=int, default=DEFAULT_HZ,
                                choices=SAMPLE_RATES)
    collect_parser.add_argument("--dur", type=float, default=0.0)
    args = parser.parse_args()
    if args.command == "collect" and not args.out:
        prefix = _ask_group()
        args.out = _ask_output_path(prefix)

    hardware = _open_up()
    try:
        if args.command == "scan":
            scan(hardware)
        else:
            collect(hardware, args.out or _default_output_path(),
                    args.hz, args.dur)
    finally:
        hardware.ADC_IO_Close()


if __name__ == "__main__":
    main()
