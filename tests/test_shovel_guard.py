#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import (
    SHOVEL_HANG_CONFIRM,
    SHOVEL_REVERSE_MIN_SECONDS,
    SHOVEL_REVERSE_SPEED,
    SHOVEL_REVERSE_TIMEOUT,
)
from shovel_guard import ShovelGuard


HANGING = {"left": 20.0, "right": 20.0, "valid": True}    # 悬空：远低于 ENTER=1400
ON_STAGE = {"left": 1600.0, "right": 1600.0, "valid": True}  # 台内：高于 CLEAR=1450


class ShovelGuardTest(unittest.TestCase):
    def update(self, guard, raw, active=True, now=0.0, healthy=True):
        return guard.update(raw, active, now=now, healthy=healthy)

    def test_inactive_never_triggers(self):
        guard = ShovelGuard(window=1)
        for index in range(SHOVEL_HANG_CONFIRM + 2):
            result = self.update(guard, HANGING, active=False, now=index * 0.02)
            self.assertEqual("IDLE", result["state"])
            self.assertEqual((0, 0), (result["left"], result["right"]))
            self.assertFalse(result["hang"])

    def test_hang_stops_immediately_then_reverses_after_confirm(self):
        guard = ShovelGuard(window=1)
        # 第一帧悬空即停车（防掉落优先），不等待确认
        result = self.update(guard, HANGING, now=0.00)
        self.assertEqual("HANGED", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))
        self.assertTrue(result["hang"])

        for index in range(SHOVEL_HANG_CONFIRM - 1):
            result = self.update(guard, HANGING, now=0.02 + index * 0.02)
        # 连续 SHOVEL_HANG_CONFIRM 帧悬空后开始倒车
        self.assertEqual("REVERSE", result["state"])
        self.assertEqual(
            (-SHOVEL_REVERSE_SPEED, -SHOVEL_REVERSE_SPEED),
            (result["left"], result["right"]),
        )

    def test_signal_jitter_returns_to_idle(self):
        guard = ShovelGuard(window=1)
        result = self.update(guard, HANGING, now=0.00)
        self.assertEqual("HANGED", result["state"])
        # 下一帧信号恢复（ADC 抖动）→ 回 IDLE，不倒车
        result = self.update(guard, ON_STAGE, now=0.02)
        self.assertEqual("IDLE", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_reverse_ends_when_back_on_stage(self):
        guard = ShovelGuard(window=1)
        self.update(guard, HANGING, now=0.00)
        for index in range(SHOVEL_HANG_CONFIRM - 1):
            self.update(guard, HANGING, now=0.02 + index * 0.02)
        result = self.update(guard, HANGING, now=0.10)
        self.assertEqual("REVERSE", result["state"])

        # 倒车中信号恢复但未到最短时长：继续倒
        result = self.update(
            guard, ON_STAGE, now=0.10 + SHOVEL_REVERSE_MIN_SECONDS / 2)
        self.assertEqual("REVERSE", result["state"])

        # 信号恢复且已到最短时长：收回完成
        result = self.update(
            guard, ON_STAGE, now=0.10 + SHOVEL_REVERSE_MIN_SECONDS + 0.001)
        self.assertEqual("IDLE", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_reverse_timeout_stops_and_recovers(self):
        guard = ShovelGuard(window=1)
        self.update(guard, HANGING, now=0.00)
        for index in range(SHOVEL_HANG_CONFIRM - 1):
            self.update(guard, HANGING, now=0.02 + index * 0.02)
        reverse_start = 0.04  # 第 3 帧确认后进入 REVERSE
        result = self.update(guard, HANGING, now=reverse_start)
        self.assertEqual("REVERSE", result["state"])

        # 超时从 REVERSE 开始时刻算起
        result = self.update(
            guard, HANGING, now=reverse_start + SHOVEL_REVERSE_TIMEOUT + 0.001)
        self.assertEqual("SAFE_STOP", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

        result = self.update(
            guard, ON_STAGE, now=reverse_start + SHOVEL_REVERSE_TIMEOUT + 0.1)
        self.assertEqual("IDLE", result["state"])

    def test_invalid_data_does_not_trigger(self):
        guard = ShovelGuard(window=1)
        invalid = {"left": 0.0, "right": 0.0, "valid": False}
        for index in range(SHOVEL_HANG_CONFIRM + 1):
            result = self.update(guard, invalid, now=index * 0.02)
            self.assertEqual("IDLE", result["state"])

    def test_median_filter_suppresses_single_spike(self):
        # 3 帧中值滤波：悬空中出现一帧台内尖峰，中位数仍低 → 触发
        guard = ShovelGuard(window=3)
        result = None
        for index, raw in enumerate((HANGING, HANGING, ON_STAGE)):
            result = self.update(guard, raw, now=index * 0.02)
        self.assertEqual("HANGED", result["state"])
        self.assertTrue(result["hang"])

        # 反向：台内中出现一帧悬空尖峰，中位数仍高 → 不触发
        guard = ShovelGuard(window=3)
        result = None
        for index, raw in enumerate((ON_STAGE, ON_STAGE, HANGING)):
            result = self.update(guard, raw, now=index * 0.02)
        self.assertEqual("IDLE", result["state"])
        self.assertFalse(result["hang"])


if __name__ == "__main__":
    unittest.main()
