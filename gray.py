#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四路灰度传感器的数据注入、清洗与风险模型，不持有硬件。"""

import math
import statistics
from collections import deque

# ---------- 独立默认值；生产/dev 运行时由 config.py 注入 ----------
GRAY_CHANNELS = {
    "front": 2,    # 前灰度
    "rear": 3,     # 后灰度
    "left": 0,     # 左灰度
    "right": 1,    # 右灰度
}
ADC_MAX = 10000.0      # ADC 合法上限：超限=坏值→按 0 处理（fail-safe）

# 由 data/*.csv 经 dev/calibrate_gray.py 重算，2026-08-10。
# edge/center 用于黑色渐变外圈定位；white 用于真正压到白边时的紧急告警。
GRAY_FILTER_WINDOW = 3
# 来源：data/gray_model.csv；真实边缘中位数 -> 内环较外侧中位数。
GRAY_EDGE_REFERENCE = {"front": 666.0, "rear": 798.0, "left": 458.0, "right": 1143.0}
GRAY_CENTER_REFERENCE = {"front": 1033.5, "rear": 1257.0, "left": 817.0, "right": 1622.0}
GRAY_WHITE_REFERENCE = {"front": 1924.0, "rear": 2283.0, "left": 1625.0, "right": 2507.0}
GRAY_WHITE_ENTER = {"front": 1798.0, "rear": 2154.0, "left": 1508.0, "right": 2395.0}
GRAY_WHITE_CLEAR = {"front": 1731.0, "rear": 2085.0, "left": 1445.0, "right": 2337.0}
# 大转阈值：提高会更早 180 度，降低会更靠近真实边缘才 180 度。
GRAY_NEAR_EDGE_ENTER = 0.50
GRAY_NEAR_EDGE_CLEAR = 0.65


class GraySensor:
    """四路灰度数据注入接口（前/后/左/右，模拟 ADC），不持有硬件。

    adc_reader：返回 10 路 ADC 列表的可调用对象（如 lambda: ctrl.adc_data）。
    """

    def __init__(self, adc_reader=None, channels=None, adc_max=ADC_MAX,
                 white_enter=None):
        self._reader = adc_reader
        self.channels = dict(GRAY_CHANNELS if channels is None else channels)
        self.adc_max = float(adc_max)
        self.white_enter = dict(
            GRAY_WHITE_ENTER if white_enter is None else white_enter
        )

    def read_raw(self, adc=None):
        if adc is None:
            if self._reader is None:
                raise RuntimeError("需注入 adc_reader（或显式传 adc）")
            adc = self._reader()
        out = {}
        for name, ch in self.channels.items():
            v = adc[ch] if 0 <= ch < len(adc) else 0
            try:
                v = float(v)
            except (TypeError, ValueError):
                v = 0.0
            if not (0.0 <= v <= self.adc_max):
                v = 0.0
            out[name] = v
        return out

    def edge(self, threshold=None, raw=None):
        """压到白边：返回超过逐路阈值的方向名列表（空=未压白边）。"""
        raw = self.read_raw() if raw is None else raw
        threshold = self.white_enter if threshold is None else threshold
        thresholds = threshold if isinstance(threshold, dict) else {
            name: threshold for name in self.channels
        }
        return [name for name, v in raw.items() if v >= thresholds[name]]

    def fall(self, threshold=None, raw=None):
        """兼容旧接口：任一路压到白边即视为掉台风险。"""
        return bool(self.edge(threshold, raw))

    def on_stage(self, threshold=None, raw=None):
        """兼容旧接口：四路均未压到白边。"""
        return not self.fall(threshold, raw)


class GrayRiskModel:
    """三点中值滤波后的双层风险模型。

    zone_score 越小越靠近暗外圈；white_hits 表示已经压到白边。
    模型只处理注入的四路原始值，不持有硬件，PC 可直接回放 CSV。
    """

    NAMES = ("front", "rear", "left", "right")

    def __init__(
            self, window=GRAY_FILTER_WINDOW, edge_reference=None,
            center_reference=None, white_reference=None, white_enter=None,
            white_clear=None, near_edge_enter=GRAY_NEAR_EDGE_ENTER,
            near_edge_clear=GRAY_NEAR_EDGE_CLEAR, adc_max=ADC_MAX):
        if window < 1 or window % 2 == 0:
            raise ValueError("滤波窗口必须为正奇数")
        self.window = window
        self.edge_reference = dict(
            GRAY_EDGE_REFERENCE if edge_reference is None else edge_reference
        )
        self.center_reference = dict(
            GRAY_CENTER_REFERENCE if center_reference is None else center_reference
        )
        self.white_reference = dict(
            GRAY_WHITE_REFERENCE if white_reference is None else white_reference
        )
        self.white_enter = dict(
            GRAY_WHITE_ENTER if white_enter is None else white_enter
        )
        self.white_clear = dict(
            GRAY_WHITE_CLEAR if white_clear is None else white_clear
        )
        self.near_edge_enter = float(near_edge_enter)
        self.near_edge_clear = float(near_edge_clear)
        self.adc_max = float(adc_max)
        self._samples = {name: deque(maxlen=window) for name in self.NAMES}

    def _clean(self, raw):
        cleaned = {}
        valid = True
        for name in GrayRiskModel.NAMES:
            try:
                value = float(raw[name])
            except (KeyError, TypeError, ValueError):
                value = 0.0
                valid = False
            if not math.isfinite(value) or not 0.0 <= value <= self.adc_max:
                value = 0.0
                valid = False
            cleaned[name] = value
        return cleaned, valid

    def update(self, raw):
        cleaned, valid = self._clean(raw)
        for name in self.NAMES:
            self._samples[name].append(cleaned[name])

        filtered = {
            name: float(statistics.median(self._samples[name]))
            for name in self.NAMES
        }
        zone = {
            name: ((filtered[name] - self.edge_reference[name]) /
                   (self.center_reference[name] - self.edge_reference[name]))
            for name in self.NAMES
        }
        white = {
            name: ((filtered[name] - self.center_reference[name]) /
                   (self.white_reference[name] - self.center_reference[name]))
            for name in self.NAMES
        }
        zone_score = float(statistics.median(zone.values()))
        white_hits = tuple(
            name for name in self.NAMES
            if filtered[name] >= self.white_enter[name]
        )
        return {
            "ready": len(self._samples["front"]) == self.window,
            "valid": valid,
            "raw": cleaned,
            "filtered": filtered,
            "zone": zone,
            "white": white,
            "zone_score": zone_score,
            "near_edge": zone_score < self.near_edge_enter,
            "near_clear": zone_score > self.near_edge_clear,
            "white_hits": white_hits,
            "white_clear": all(
                filtered[name] < self.white_clear[name] for name in self.NAMES
            ),
        }
