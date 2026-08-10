#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灰度传感器模块 (gray.py) —— 4 路模拟 ADC 灰度：前/后/左/右

模块规范（本项目约定，见 CLAUDE.md）：
- 独立模块：本模块不 import 其他项目模块，独立可运行（dev 模式）
- define 切换：DEV_MODE 同文件切换 dev / 生产
    True  = dev 自测：直接 import uptech 读硬件，支持 scan / collect 子命令
    False = 生产：不持有硬件，adc 数据外部注入（GraySensor(adc_reader=...)），PC 可单测
- 数据采集保存到文件（CSV），供标定/算法分析，不凭旧值写死
- 底层驱动（uptech.py / up_controller.py）不可修改

用法（dev 模式，真机）：
    python3 gray.py scan                     # 打印映射内 4 路灰度（0.5s 刷新）
    python3 gray.py collect --out data/xxx.csv --hz 50 [--dur 10]
        --out 必填（文件名自己定）；--hz 10|50（默认 50）；--dur 秒数（默认一直
        采集，Ctrl+C 停止；静态场景可 --dur 10 限时）；CSV 列：t,front,rear,left,right
"""

import argparse
import csv
import os
import sys
import time

from config import GRAY_CHANNELS   # 通道映射（scan 实测确认，2026-08-09）

# ---------- define 开关：同文件切换模式 ----------
DEV_MODE = True   # True=dev 自测（真机直读）；False=生产（外部注入数据）

# ---------- 参数（归纳进 config.py） ----------
ADC_MAX = 10000.0      # ADC 合法上限：超限=坏值→按 0 处理（fail-safe）
POLL_INTERVAL = 0.5    # scan 模式刷新间隔（秒）
SAMPLE_RATES = (10, 50)  # collect 允许的采样频率（两档，方便切换）
DEFAULT_HZ = 50        # 默认采样率（压边/运动变化快；静态显式 --hz 10）
DEFAULT_DUR = 0.0      # 默认不传 --dur = 一直采集，Ctrl+C 手动停止


def _open_up():
    """dev 模式打开硬件（仅树莓派真机）。"""
    try:
        import uptech
    except ImportError as e:
        raise RuntimeError(
            "dev 模式需要 uptech 库（仅树莓派真机；PC 请用 DEV_MODE=False + 注入数据）") from e
    up = uptech.UpTech()
    up.ADC_IO_Open()
    return up


def read_gray(up, adc=None):
    """读四路灰度，返回 {front,rear,left,right} 原始值（坏值→0，fail-safe）。"""
    if adc is None:
        adc = up.ADC_Get_All_Channle()
    out = {}
    for name, ch in GRAY_CHANNELS.items():
        v = adc[ch] if 0 <= ch < len(adc) else 0
        try:
            v = float(v)
        except (TypeError, ValueError):
            v = 0.0
        if not (0.0 <= v <= ADC_MAX):   # NaN/Inf/越界 → 坏值 → 0（0=暗=告警侧）
            v = 0.0
        out[name] = v
    return out


def cmd_scan(up):
    """打印映射内 4 路灰度（0.5s 刷新）：front/rear/left/right + 各自 ADC 通道号。"""
    chs = ", ".join("%s=adc%d" % (n, GRAY_CHANNELS[n]) for n in ("front", "rear", "left", "right"))
    print("扫描灰度通道（%s）（Ctrl+C 退出）" % chs)
    try:
        while True:
            adc = up.ADC_Get_All_Channle()
            vals = " ".join("%s=%5d" % (n, read_gray(up, adc)[n])
                            for n in ("front", "rear", "left", "right"))
            sys.stdout.write("\r" + vals)
            sys.stdout.flush()
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print()
    finally:
        up.ADC_IO_Close()


def cmd_collect(up, out, hz, dur):
    """采集灰度数据到 CSV（列：t,front,rear,left,right）。"""
    if hz not in SAMPLE_RATES:
        raise SystemExit("--hz 只接受 %s" % "/".join(map(str, SAMPLE_RATES)))
    dt = 1.0 / hz
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    t0 = time.monotonic()
    n = 0
    print("采集中：%s @%dHz%s（Ctrl+C 停止）" % (
        out, hz, "" if dur else "，手动停止"))
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "front", "rear", "left", "right"])
        while True:
            raw = read_gray(up)
            w.writerow([round(time.monotonic() - t0, 3),
                        raw["front"], raw["rear"], raw["left"], raw["right"]])
            n += 1
            if n % hz == 0:
                print("  已采集 %d 点 (%.1fs)" % (n, time.monotonic() - t0))
                f.flush()
            if dur and (time.monotonic() - t0) >= dur:
                break
            time.sleep(dt)
    print("完成：%s，%d 行" % (out, n))
    up.ADC_IO_Close()


class GraySensor:
    """生产模式：四路灰度（前/后/左/右，模拟 ADC），数据外部注入，不持有硬件。

    adc_reader：返回 10 路 ADC 列表的可调用对象（如 lambda: ctrl.adc_data）。
    """

    def __init__(self, adc_reader=None):
        self._reader = adc_reader

    def read_raw(self, adc=None):
        if adc is None:
            if self._reader is None:
                raise RuntimeError("生产模式需注入 adc_reader（或显式传 adc）")
            adc = self._reader()
        out = {}
        for name, ch in GRAY_CHANNELS.items():
            v = adc[ch] if 0 <= ch < len(adc) else 0
            try:
                v = float(v)
            except (TypeError, ValueError):
                v = 0.0
            if not (0.0 <= v <= ADC_MAX):
                v = 0.0
            out[name] = v
        return out

    def edge(self, threshold, raw=None):
        """接近边缘：返回暗的方向名列表（灰度 < threshold；空=安全）。"""
        raw = self.read_raw() if raw is None else raw
        return [name for name, v in raw.items() if v < threshold]

    def fall(self, threshold, raw=None):
        """掉台告警：任一灰度 < threshold。"""
        raw = self.read_raw() if raw is None else raw
        return min(raw.values()) < threshold

    def on_stage(self, threshold, raw=None):
        """在台上：任一灰度 > threshold。"""
        raw = self.read_raw() if raw is None else raw
        return max(raw.values()) > threshold


def main():
    parser = argparse.ArgumentParser(description="灰度传感器 dev 工具")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan", help="打印 10 路 ADC（实测定接线）")
    p = sub.add_parser("collect", help="采集灰度到 CSV")
    p.add_argument("--out", required=True, help="CSV 输出路径（必填，自己命名）")
    p.add_argument("--hz", type=int, default=DEFAULT_HZ, choices=SAMPLE_RATES,
                   help="采样频率 10|50（默认 %d）" % DEFAULT_HZ)
    p.add_argument("--dur", type=float, default=DEFAULT_DUR,
                   help="采集时长秒（默认一直采集，Ctrl+C 停止；需要限时传正数）")
    args = parser.parse_args()

    if not DEV_MODE:
        print("DEV_MODE=False（生产模式）：无 dev 子命令，注入 adc_reader 集成使用")
        sys.exit(0)
    up = _open_up()
    try:
        if args.cmd == "scan":
            cmd_scan(up)
        elif args.cmd == "collect":
            cmd_collect(up, args.out, args.hz, args.dur)
    finally:
        pass  # 各子命令内部已 close


if __name__ == "__main__":
    main()
