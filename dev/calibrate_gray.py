#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从已采集 CSV 重算灰度模型，并输出可审计的校准结果。"""

import argparse
import csv
import os
import statistics
from collections import deque

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

NAMES = ("front", "rear", "left", "right")
FILES = {
    "black": ("四个同黑.csv",),
    "white": ("四个同白.csv",),
    "edge": ("竖向边缘.csv", "竖向边缘2.csv", "边缘.csv", "边缘2.csv"),
    "center": (
        "武字中间旋转2圈.csv", "武字中间逆时针旋转2圈.csv",
        "武字数据.csv", "武字数据逆时针.csv",
    ),
    "traverse": ("中轴.csv", "中轴2.csv", "对角轴.csv", "对角轴2.csv"),
}
INNER_FILES = (
    "更内环侧向移动车头朝武字反方向.csv",
    "更内环侧向移动车头朝武字方向.csv",
    "更内环平行移动车头朝内.csv",
    "更内环平行移动车头朝外.csv",
)
INNER_EDGE_FRACTION = 0.20
FILTER_WINDOW = 3
# 与 config.py GRAY_NEAR_EDGE_ENTER/CLEAR 保持一致（2026-08-14 新车调到 0.35）。
NEAR_EDGE_ENTER = 0.35
NEAR_EDGE_CLEAR = 0.65


def quantile(values, ratio):
    values = sorted(values)
    pos = (len(values) - 1) * ratio
    low = int(pos)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (pos - low)


def read_filtered(path):
    queues = {name: deque(maxlen=FILTER_WINDOW) for name in NAMES}
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            for name in NAMES:
                queues[name].append(float(row[name]))
            if len(queues["front"]) == FILTER_WINDOW:
                rows.append({
                    "t": float(row["t"]),
                    **{name: float(statistics.median(queues[name])) for name in NAMES},
                })
    return rows


def load_groups(data_dir):
    groups = {}
    for role, filenames in FILES.items():
        groups[role] = []
        for filename in filenames:
            path = os.path.join(data_dir, filename)
            if not os.path.isfile(path):
                raise FileNotFoundError("缺少校准文件：%s" % path)
            groups[role].extend(read_filtered(path))
    groups["inner"] = []
    groups["inner_edge"] = []
    for filename in INNER_FILES:
        path = os.path.join(data_dir, filename)
        if not os.path.isfile(path):
            raise FileNotFoundError("缺少内环校准文件：%s" % path)
        rows = read_filtered(path)
        edge_count = max(1, int(len(rows) * INNER_EDGE_FRACTION))
        groups["inner"].extend(rows)
        groups["inner_edge"].extend(rows[:edge_count])
        groups["inner_edge"].extend(rows[-edge_count:])
    return groups


def build_model(groups):
    model = {}
    safe_rows = groups["edge"] + groups["center"]
    for name in NAMES:
        edge_ref = quantile([row[name] for row in groups["edge"]], 0.5)
        center_ref = quantile([row[name] for row in groups["inner_edge"]], 0.5)
        white_ref = quantile([row[name] for row in groups["white"]], 0.5)
        safe_upper = quantile([row[name] for row in safe_rows], 0.999)
        white_lower = quantile([row[name] for row in groups["white"]], 0.001)
        model[name] = {
            "edge_reference": edge_ref,
            "center_reference": center_ref,
            "white_reference": white_ref,
            "safe_upper": safe_upper,
            "white_lower": white_lower,
            "white_enter": round((safe_upper + white_lower) / 2.0),
            "white_clear": round(safe_upper + 0.2 * (white_lower - safe_upper)),
        }
    return model


def zone_score(row, model):
    scores = [
        ((row[name] - model[name]["edge_reference"]) /
         (model[name]["center_reference"] - model[name]["edge_reference"]))
        for name in NAMES
    ]
    return float(statistics.median(scores))


def write_model(path, model):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = ("sensor", "filter_window", "near_edge_enter", "near_edge_clear",
              "edge_reference", "center_reference", "white_reference", "safe_upper",
              "white_lower", "white_enter", "white_clear")
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for name in NAMES:
            writer.writerow({
                "sensor": name,
                "filter_window": FILTER_WINDOW,
                "near_edge_enter": NEAR_EDGE_ENTER,
                "near_edge_clear": NEAR_EDGE_CLEAR,
                **model[name],
            })


def write_summary(path, groups, model):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = ("group", "samples", "zone_p001", "zone_p50", "zone_p999",
              "near_edge_rate", "white_hit_rate")
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for role, rows in groups.items():
            zones = [zone_score(row, model) for row in rows]
            white_hits = sum(any(
                row[name] >= model[name]["white_enter"] for name in NAMES
            ) for row in rows)
            writer.writerow({
                "group": role,
                "samples": len(rows),
                "zone_p001": round(quantile(zones, 0.001), 4),
                "zone_p50": round(quantile(zones, 0.5), 4),
                "zone_p999": round(quantile(zones, 0.999), 4),
                "near_edge_rate": round(
                    sum(value < NEAR_EDGE_ENTER for value in zones) / len(zones), 6
                ),
                "white_hit_rate": round(white_hits / len(rows), 6),
            })


def main():
    parser = argparse.ArgumentParser(description="重算四路灰度巡台模型")
    parser.add_argument("--data-dir", default=os.path.join(ROOT, "data"))
    parser.add_argument("--out", default=os.path.join(ROOT, "data", "gray_model.csv"))
    parser.add_argument(
        "--summary", default=os.path.join(ROOT, "data", "gray_model_summary.csv")
    )
    args = parser.parse_args()

    groups = load_groups(args.data_dir)
    model = build_model(groups)
    write_model(args.out, model)
    write_summary(args.summary, groups, model)
    print("模型：%s" % args.out)
    print("回放摘要：%s" % args.summary)


if __name__ == "__main__":
    main()
