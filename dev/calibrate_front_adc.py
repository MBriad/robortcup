#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""处理前头两路 ADC 采样，保留原始文件并输出滤波数据与分组摘要。"""

import argparse
import csv
import os
import statistics
from collections import deque


EXPECTED_FIELDS = ("t", "left", "right", "diff", "valid")
DEFAULT_FILTER_WINDOW = 9
DEFAULT_ACTIVE_MIN = 50.0
DEFAULT_CENTER_CONFIRM = 3


def _percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _classify(filename):
    if filename.startswith(("正对墙", "正对着墙")):
        if "居中位置" in filename:
            distance = "middle"
        elif "最远" in filename:
            distance = "far"
        else:
            distance = "near"
        return "center_" + distance, distance, False
    if "左偏" in filename:
        distance = "middle" if "居中位置" in filename else "far"
        return "left_bias", distance, "从90度" in filename
    if "右偏" in filename:
        distance = "middle" if "居中位置" in filename else "far"
        return "right_bias", distance, "从90度" in filename
    return None


def _read_source(path):
    with open(path, newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != EXPECTED_FIELDS:
            return []
        return list(reader)


def _phase(index, count, dynamic):
    if not dynamic:
        return "steady"
    fraction = index / max(count - 1, 1)
    if fraction < 0.2:
        return "start"
    if fraction >= 0.8:
        return "end"
    return "transition"


def _process_source(path, window, active_min):
    classification = _classify(os.path.basename(path))
    rows = _read_source(path)
    if classification is None or not rows:
        return []

    label, distance, dynamic = classification
    valid_count = sum(row["valid"] == "1" for row in rows)
    left_window = deque(maxlen=window)
    right_window = deque(maxlen=window)
    valid_index = 0
    processed = []

    for source_row, row in enumerate(rows, start=2):
        if row["valid"] != "1":
            continue
        left = float(row["left"])
        right = float(row["right"])
        left_window.append(left)
        right_window.append(right)
        left_filtered = float(statistics.median(left_window))
        right_filtered = float(statistics.median(right_window))
        diff_filtered = left_filtered - right_filtered
        processed.append({
            "source": os.path.basename(path),
            "source_row": source_row,
            "label": label,
            "distance": distance,
            "phase": _phase(valid_index, valid_count, dynamic),
            "t": float(row["t"]),
            "left": left,
            "right": right,
            "diff": float(row["diff"]),
            "left_filtered": left_filtered,
            "right_filtered": right_filtered,
            "diff_filtered": diff_filtered,
            "both_active": int(
                left_filtered >= active_min and right_filtered >= active_min
            ),
            "valid": 1,
        })
        valid_index += 1
    return processed


def _summarize(rows, active_min):
    left = [row["left_filtered"] for row in rows]
    right = [row["right_filtered"] for row in rows]
    diff = [row["diff_filtered"] for row in rows]
    signal = [max(left_value, right_value) for left_value, right_value
              in zip(left, right)]
    both_active_ratio = sum(row["both_active"] for row in rows) / len(rows)
    right_floor_ratio = sum(value < active_min for value in right) / len(rows)
    usable = both_active_ratio >= 0.8
    return {
        "source": rows[0]["source"],
        "label": rows[0]["label"],
        "distance": rows[0]["distance"],
        "phase": rows[0]["phase"],
        "rows": len(rows),
        "left_median": round(statistics.median(left), 3),
        "right_median": round(statistics.median(right), 3),
        "diff_median": round(statistics.median(diff), 3),
        "diff_p05": round(_percentile(diff, 0.05), 3),
        "diff_p95": round(_percentile(diff, 0.95), 3),
        "signal_p01": round(_percentile(signal, 0.01), 3),
        "signal_p99": round(_percentile(signal, 0.99), 3),
        "both_active_ratio": round(both_active_ratio, 4),
        "right_floor_ratio": round(right_floor_ratio, 4),
        "alignment_usable": int(usable),
        "note": (
            "左右两路均进入有效量程，可用于对齐建模"
            if usable else
            "至少一路长期处于底噪，不能用于左右方向建模"
        ),
    }


def analyze(data_dir, processed_path, summary_path,
            window=DEFAULT_FILTER_WINDOW, active_min=DEFAULT_ACTIVE_MIN):
    if window < 1 or window % 2 == 0:
        raise ValueError("滤波窗口必须为正奇数")

    processed = []
    for filename in sorted(os.listdir(data_dir)):
        if not filename.lower().endswith(".csv"):
            continue
        processed.extend(_process_source(
            os.path.join(data_dir, filename), window, active_min
        ))
    if not processed:
        raise RuntimeError("未找到前头 ADC 原始 CSV")

    processed_fields = (
        "source", "source_row", "label", "distance", "phase", "t",
        "left", "right", "diff", "left_filtered", "right_filtered",
        "diff_filtered", "both_active", "valid",
    )
    os.makedirs(os.path.dirname(os.path.abspath(processed_path)), exist_ok=True)
    with open(processed_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=processed_fields)
        writer.writeheader()
        writer.writerows(processed)

    groups = {}
    for row in processed:
        key = (row["source"], row["phase"])
        groups.setdefault(key, []).append(row)
    summary = [
        _summarize(rows, active_min)
        for _, rows in sorted(groups.items())
    ]
    summary_fields = (
        "source", "label", "distance", "phase", "rows", "left_median",
        "right_median", "diff_median", "diff_p05", "diff_p95",
        "signal_p01", "signal_p99",
        "both_active_ratio", "right_floor_ratio", "alignment_usable", "note",
    )
    with open(summary_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summary)
    return processed, summary


def derive_model(summary, model_path, window=DEFAULT_FILTER_WINDOW):
    steady = {
        row["label"]: row
        for row in summary
        if row["distance"] == "middle" and row["phase"] == "steady"
    }
    required = ("left_bias", "center_middle", "right_bias")
    missing = [label for label in required if label not in steady]
    if missing:
        raise RuntimeError("缺少固定姿态数据：%s" % ", ".join(missing))

    left = steady["left_bias"]
    center = steady["center_middle"]
    right = steady["right_bias"]
    low_gap = center["diff_p05"] - left["diff_p95"]
    high_gap = right["diff_p05"] - center["diff_p95"]
    if low_gap <= 0.0 or high_gap <= 0.0:
        raise RuntimeError("左偏、正对、右偏的滤波区间仍有重叠")

    diff_low = (left["diff_p95"] + center["diff_p05"]) / 2.0
    diff_high = (center["diff_p95"] + right["diff_p05"]) / 2.0
    far_center = next((
        row for row in summary
        if row["label"] == "center_far" and row["phase"] == "steady"
    ), None)
    if far_center is None:
        raise RuntimeError("缺少远距离正对墙数据，无法生成信号强度阈值")
    middle_signal_min = min(
        left["signal_p01"], center["signal_p01"], right["signal_p01"]
    )
    signal_min = (far_center["signal_p99"] + middle_signal_min) / 2.0
    model = [
        {
            "parameter": "filter_window",
            "value": window,
            "source": "三组固定姿态 CSV",
            "note": "滚动中值滤波窗口",
        },
        {
            "parameter": "diff_low",
            "value": round(diff_low, 3),
            "source": "%s / %s" % (left["source"], center["source"]),
            "note": "低于此值表示车头左偏，需要右转",
        },
        {
            "parameter": "diff_high",
            "value": round(diff_high, 3),
            "source": "%s / %s" % (center["source"], right["source"]),
            "note": "高于此值表示车头右偏，需要左转",
        },
        {
            "parameter": "center_confirm",
            "value": DEFAULT_CENTER_CONFIRM,
            "source": "控制防抖",
            "note": "连续正对帧数",
        },
        {
            "parameter": "signal_min",
            "value": round(signal_min, 0),
            "source": "%s / 三组固定姿态 CSV" % far_center["source"],
            "note": "低于此峰值表示离墙过远，需要靠墙脉冲",
        },
        {
            "parameter": "low_gap",
            "value": round(low_gap, 3),
            "source": "左偏 p95 到正对 p05",
            "note": "低侧分类安全间隔",
        },
        {
            "parameter": "high_gap",
            "value": round(high_gap, 3),
            "source": "正对 p95 到右偏 p05",
            "note": "高侧分类安全间隔",
        },
    ]
    os.makedirs(os.path.dirname(os.path.abspath(model_path)), exist_ok=True)
    with open(model_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=("parameter", "value", "source", "note")
        )
        writer.writeheader()
        writer.writerows(model)
    return model


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(root, "data")
    parser = argparse.ArgumentParser(description="前头 ADC 采样滤波与质量分析")
    parser.add_argument("--data-dir", default=data_dir)
    parser.add_argument("--window", type=int, default=DEFAULT_FILTER_WINDOW)
    parser.add_argument("--active-min", type=float, default=DEFAULT_ACTIVE_MIN)
    parser.add_argument(
        "--processed", default=os.path.join(data_dir, "front_adc_processed.csv")
    )
    parser.add_argument(
        "--summary", default=os.path.join(data_dir, "front_adc_summary.csv")
    )
    parser.add_argument(
        "--model", default=os.path.join(data_dir, "front_adc_model.csv")
    )
    args = parser.parse_args()
    processed, summary = analyze(
        args.data_dir, args.processed, args.summary,
        window=args.window, active_min=args.active_min,
    )
    model = derive_model(summary, args.model, window=args.window)
    usable = sum(row["alignment_usable"] for row in summary)
    print("完成：%d 条滤波数据，%d 个分组，%d 个分组可用于对齐建模" % (
        len(processed), len(summary), usable,
    ))
    print("滤波数据：%s" % args.processed)
    print("统计摘要：%s" % args.summary)
    print("对齐模型：%s" % args.model)
    print("模型参数：%s" % ", ".join(
        "%s=%s" % (row["parameter"], row["value"]) for row in model[:5]
    ))


if __name__ == "__main__":
    main()
