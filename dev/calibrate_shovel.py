#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从铲子采集 CSV 重算悬空/收回阈值（SHOVEL_HANG_ENTER / SHOVEL_HANG_CLEAR），输出可审计结果。

文件名约定（采集时用 dev/shovel_tool.py collect，交互命名）：
    工具交互选 1/2 会自动加前缀：shovel_hang_*（悬空）/ shovel_stage_*（台内）
    中文或英文关键词同样识别：
        含「悬空/出台」或 hang → 悬空组（铲子伸出台面，信号接近底噪）
        含「台内/台上」或 stage → 台内组（铲子在台内正常位，信号高）
    其余文件忽略（自定义分组数据不会被计入）。

阈值逻辑（新车 2026-08-15 起极性：铲子悬空=信号高、台内=信号低）：
    先按 SHOVEL_FILTER_WINDOW 对每帧的 max/min(两路) 序列做滚动中值滤波（与 guard 判据一致），
    再用滤波后分位定阈值：
    ENTER = 台内组 signal_min p99 与 悬空组 signal_min p01 的中点（min(两路) 高于它判悬空）
    CLEAR = 台内组 signal_max p99 与 悬空组 signal_max p01 的中点（max(两路) 低于它判收回；
            CLEAR > ENTER 形成滞回）

真机流程：
    python3 dev/shovel_tool.py collect   # 采「悬空」：铲子伸出台面，5~10 秒
    python3 dev/shovel_tool.py collect   # 采「台内」：铲子在台内正常位，5~10 秒
    python3 dev/calibrate_shovel.py      # 重算阈值 → data/shovel_model.csv
"""

import argparse
import csv
import os
import statistics
import sys
from collections import deque

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT in sys.path:
    sys.path.remove(ROOT)
sys.path.insert(0, ROOT)

from config import SHOVEL_FILTER_WINDOW  # noqa: E402


def _percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _classify(filename):
    lowered = filename.lower()
    if "悬空" in filename or "出台" in filename or "hang" in lowered:
        return "hang"
    if "台内" in filename or "台上" in filename or "stage" in lowered:
        return "stage"
    return None


def _read_rows(path):
    """读取采集 CSV，过滤无效行；返回 [(signal_max, signal_min), ...]。"""
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                left = float(row["left"])
                right = float(row["right"])
            except (KeyError, ValueError):
                continue
            if row.get("valid", "1").strip() != "1":
                continue
            rows.append((max(left, right), min(left, right)))
    return rows


def _median_series(values, window):
    """对序列做 window 帧滚动中值滤波（丢弃不满窗口的头部，与 guard 一致）。"""
    samples = deque(maxlen=window)
    filtered = []
    for value in values:
        samples.append(value)
        if len(samples) == window:
            filtered.append(statistics.median(samples))
    return filtered


def analyze(data_dir):
    """逐文件滤波后摘要 + 分组汇总，返回 (摘要行, hang_min, stage_min, hang_max, stage_max)。"""
    summary = []
    hang_min = []
    stage_min = []
    hang_max = []
    stage_max = []
    for filename in sorted(os.listdir(data_dir)):
        if not filename.lower().endswith(".csv"):
            continue
        role = _classify(filename)
        if role is None:
            continue
        rows = _read_rows(os.path.join(data_dir, filename))
        if not rows:
            continue
        maxima = _median_series(
            [signal_max for signal_max, _ in rows], SHOVEL_FILTER_WINDOW)
        minima = _median_series(
            [signal_min for _, signal_min in rows], SHOVEL_FILTER_WINDOW)
        if not maxima:
            continue
        summary.append({
            "source": filename,
            "role": role,
            "rows": len(rows),
            "signal_max_p50": round(_percentile(maxima, 0.5), 1),
            "signal_max_p99": round(_percentile(maxima, 0.99), 1),
            "signal_min_p01": round(_percentile(minima, 0.01), 1),
            "signal_min_p50": round(_percentile(minima, 0.5), 1),
        })
        if role == "hang":
            hang_max.extend(maxima)
            hang_min.extend(minima)
        else:
            stage_max.extend(maxima)
            stage_min.extend(minima)
    return summary, hang_min, stage_min, hang_max, stage_max


def derive_model(hang_min, stage_min, hang_max, stage_max, model_path):
    """由悬空/台内的 min、max 滤波分布定 ENTER/CLEAR，要求两态区间不重叠。"""
    stage_min_p99 = _percentile(stage_min, 0.99)
    hang_min_p01 = _percentile(hang_min, 0.01)
    stage_max_p99 = _percentile(stage_max, 0.99)
    hang_max_p01 = _percentile(hang_max, 0.01)
    if stage_min_p99 >= hang_min_p01 or stage_max_p99 >= hang_max_p01:
        raise RuntimeError(
            "悬空与台内信号区间重叠：台内 min p99=%.0f vs 悬空 min p01=%.0f，"
            "台内 max p99=%.0f vs 悬空 max p01=%.0f；"
            "请重采（悬空要真正伸出，台内要正常贴台面）" % (
                stage_min_p99, hang_min_p01, stage_max_p99, hang_max_p01)
        )
    enter = (stage_min_p99 + hang_min_p01) / 2.0
    clear = (stage_max_p99 + hang_max_p01) / 2.0
    model = [
        {
            "parameter": "hang_enter",
            "value": round(enter, 1),
            "source": "台内 min p99=%.0f / 悬空 min p01=%.0f" % (
                stage_min_p99, hang_min_p01),
            "note": "滤波后 min(两路) 高于此值 = 铲子悬空（shovel_guard SHOVEL_HANG_ENTER）",
        },
        {
            "parameter": "hang_clear",
            "value": round(clear, 1),
            "source": "台内 max p99=%.0f / 悬空 max p01=%.0f" % (
                stage_max_p99, hang_max_p01),
            "note": "倒车后滤波后 max(两路) 低于此值 = 已收回台内（shovel_guard SHOVEL_HANG_CLEAR）",
        },
    ]
    os.makedirs(os.path.dirname(os.path.abspath(model_path)), exist_ok=True)
    with open(model_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=("parameter", "value", "source", "note"))
        writer.writeheader()
        writer.writerows(model)
    return model


def main():
    data_dir = os.path.join(ROOT, "data")
    parser = argparse.ArgumentParser(description="铲子悬空/收回阈值重算")
    parser.add_argument("--data-dir", default=data_dir)
    parser.add_argument("--out", default=os.path.join(data_dir, "shovel_model.csv"))
    args = parser.parse_args()

    summary, hang_min, stage_min, hang_max, stage_max = analyze(args.data_dir)
    if not hang_min or not stage_min:
        raise SystemExit(
            "缺少分组数据：需要文件名含关键词的采集 CSV\n"
            "  hang / 悬空 / 出台 → 悬空组；stage / 台内 / 台上 → 台内组")
    model = derive_model(hang_min, stage_min, hang_max, stage_max, args.out)
    ignored = [
        name for name in sorted(os.listdir(args.data_dir))
        if name.lower().endswith(".csv") and _classify(name) is None
    ]
    if ignored:
        print("\n忽略的文件（文件名不含 hang/stage/悬空/台内 关键词，未参与标定）：")
        for name in ignored:
            print("  " + name)

    print("逐文件摘要：")
    print("  %-28s %-6s %5s %10s %10s %10s %10s" % (
        "source", "role", "rows", "max_p50", "max_p99", "min_p01", "min_p50"))
    for row in summary:
        print("  %-28s %-6s %5d %10.1f %10.1f %10.1f %10.1f" % (
            row["source"], row["role"], row["rows"],
            row["signal_max_p50"], row["signal_max_p99"],
            row["signal_min_p01"], row["signal_min_p50"]))
    print("\n建议阈值（写入 config.py 的 SHOVEL_HANG_ENTER / SHOVEL_HANG_CLEAR）：")
    for row in model:
        print("  %-14s = %s" % (row["parameter"], row["value"]))
    print("\n模型：%s" % args.out)


if __name__ == "__main__":
    main()
