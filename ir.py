#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两路模拟红外的数据注入、清洗与前墙对齐模型，不持有硬件。"""

import math
import statistics
from collections import deque

# ---------- 参数（config.py 仅做汇总；本模块不依赖其他项目模块） ----------
# 通道映射按真机接线确认（换车时用 `scan` 看全 10 路再改这里）。
IR_CHANNELS = {
    "left": 5,     # 前左红外
    "right": 4,    # 前右红外
}
ADC_MAX = 10000.0      # ADC 合法上限：超限=坏值→0 + valid=False（fail-safe）
# 来源：data/front_adc_model.csv，2026-08-11 三组固定姿态数据。
IR_ALIGNMENT_FILTER_WINDOW = 9
IR_ALIGNMENT_DIFF_LOW = 331.0
IR_ALIGNMENT_DIFF_HIGH = 497.0
IR_ALIGNMENT_CONFIRM = 3
IR_ALIGNMENT_SIGNAL_MIN = 377.0


class IrSensor:
    """两路模拟红外数据注入接口（前左/前右），不持有硬件。

    adc_reader：返回 10 路 ADC 列表的可调用对象（如 lambda: ctrl.adc_data）。
    read_raw() 返回 {left, right, valid}；坏值→0 且 valid=False（断线/越界，
    策略层收到无效应停车，不当「没墙」继续开）。
    """

    def __init__(self, adc_reader=None):
        self._reader = adc_reader

    def read_raw(self, adc=None):
        if adc is None:
            if self._reader is None:
                raise RuntimeError("需注入 adc_reader（或显式传 adc）")
            adc = self._reader()
        out = {}
        valid = True
        for name, ch in IR_CHANNELS.items():
            if not (0 <= ch < len(adc)):
                out[name] = 0.0
                valid = False
                continue
            try:
                v = float(adc[ch])
            except (TypeError, ValueError):
                v = 0.0
            if not (0.0 <= v <= ADC_MAX):
                v = 0.0
                valid = False
            out[name] = v
        out["valid"] = valid
        return out

    def diff(self, raw=None):
        """两路差（left - right）：对墙对齐判断用；数据无效返回 None。"""
        raw = self.read_raw() if raw is None else raw
        if not raw["valid"]:
            return None
        return raw["left"] - raw["right"]


class IrAlignmentModel:
    """前墙对齐分类：左偏需右转，正对保持，右偏需左转。"""

    def __init__(self, window=IR_ALIGNMENT_FILTER_WINDOW):
        if window < 1 or window % 2 == 0:
            raise ValueError("滤波窗口必须为正奇数")
        self.window = window
        self.reset()

    def reset(self):
        self._left = deque(maxlen=self.window)
        self._right = deque(maxlen=self.window)

    def update(self, raw):
        valid = bool(raw.get("valid", False))
        try:
            left = float(raw["left"])
            right = float(raw["right"])
        except (KeyError, TypeError, ValueError):
            left = right = 0.0
            valid = False
        if not all(math.isfinite(value) and 0.0 <= value <= ADC_MAX
                   for value in (left, right)):
            left = right = 0.0
            valid = False
        if not valid:
            self.reset()
            return {
                "valid": False, "ready": False, "left": left,
                "right": right, "diff": 0.0, "signal": 0.0,
                "strong": False, "position": "invalid",
                "correction": "stop",
            }

        self._left.append(left)
        self._right.append(right)
        filtered_left = float(statistics.median(self._left))
        filtered_right = float(statistics.median(self._right))
        diff = filtered_left - filtered_right
        signal = max(filtered_left, filtered_right)
        ready = len(self._left) == self.window
        if not ready:
            position, correction = "warming", "stop"
        elif diff < IR_ALIGNMENT_DIFF_LOW:
            position, correction = "left_bias", "right"
        elif diff > IR_ALIGNMENT_DIFF_HIGH:
            position, correction = "right_bias", "left"
        else:
            position, correction = "center", "stop"
        return {
            "valid": True,
            "ready": ready,
            "left": filtered_left,
            "right": filtered_right,
            "diff": diff,
            "signal": signal,
            "strong": ready and signal >= IR_ALIGNMENT_SIGNAL_MIN,
            "position": position,
            "correction": correction,
        }
