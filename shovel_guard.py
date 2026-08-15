#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""铲子防掉落保护：推东西模式下铲子悬空检测 → 停车 → 倒车收回，不持有硬件。

输入全部外部注入：
    shovel_raw : IrSensor(channels=SHOVEL_IR_CHANNELS).read_raw() 的 dict（left/right + valid）
    active     : 推东西模式开关（将来由视觉/数字红外检测给出；dev 阶段用命令行模拟）
    healthy    : 驱动层健康标志
输出：电机命令 / 状态。上层以 state != "IDLE" 判定接管电机。

流程（新车 2026-08-15 实测极性：铲子悬空=信号高，台内=信号低）：
    IDLE（active=False 或铲子在台内）
      → 两路信号均 > SHOVEL_HANG_ENTER → HANGED（第一帧即停车，防掉落优先）
      → HANGED：连续 SHOVEL_HANG_CONFIRM 帧仍悬空 → REVERSE；信号恢复 → 回 IDLE
      → REVERSE：倒车收回；两路均 < SHOVEL_HANG_CLEAR 且已倒 ≥ SHOVEL_REVERSE_MIN_SECONDS → IDLE；
        超时 SHOVEL_REVERSE_TIMEOUT → SAFE_STOP（信号恢复后回 IDLE）
"""

import statistics
import time
from collections import deque

from config import (
    SHOVEL_FILTER_WINDOW,
    SHOVEL_HANG_CLEAR,
    SHOVEL_HANG_CONFIRM,
    SHOVEL_HANG_ENTER,
    SHOVEL_REVERSE_MIN_SECONDS,
    SHOVEL_REVERSE_SPEED,
    SHOVEL_REVERSE_TIMEOUT,
)


class ShovelGuard:
    """推东西模式下铲子悬空防掉落状态机。

    对两路信号各做 window 帧滚动中值滤波后再判定（阈值按滤波后数据标定）。
    """

    def __init__(self, window=SHOVEL_FILTER_WINDOW):
        if window < 1 or window % 2 == 0:
            raise ValueError("滤波窗口必须为正奇数")
        self.window = window
        self._max_samples = deque(maxlen=window)
        self._min_samples = deque(maxlen=window)
        self.state = "IDLE"
        self.reason = "待机"
        self.command = (0, 0)
        self.hang = False          # 铲子悬空电平（滤波后两路最小信号高于进入阈值）
        self._hang_count = 0
        self._state_started = 0.0

    def _enter(self, state, now, command, reason):
        self.state = state
        self._state_started = now
        self.command = command
        self.reason = reason

    def update(self, shovel_raw, active, now=None, healthy=True):
        now = time.monotonic() if now is None else float(now)
        valid = bool(shovel_raw.get("valid", False))
        if not active or not healthy or not valid:
            # 模式关 / 数据无效：不接管，让出控制权（清空滤波样本）
            self._max_samples.clear()
            self._min_samples.clear()
            self._hang_count = 0
            self.hang = False
            self._enter("IDLE", now, (0, 0), "待机")
            return self._result()

        left = float(shovel_raw["left"])
        right = float(shovel_raw["right"])
        self._max_samples.append(max(left, right))
        self._min_samples.append(min(left, right))
        if len(self._max_samples) < self.window:
            # 中值滤波窗口未满（启动前 window-1 帧）：视为安全，不触发
            self.hang = False
            return self._result()
        filtered_max = statistics.median(self._max_samples)
        filtered_min = statistics.median(self._min_samples)
        self.hang = filtered_min > SHOVEL_HANG_ENTER
        elapsed = now - self._state_started

        if self.state == "IDLE":
            if self.hang:
                self._hang_count = 1
                self._enter("HANGED", now, (0, 0), "铲子悬空，停车")
            return self._result()

        if self.state == "HANGED":
            if not self.hang:
                self._hang_count = 0
                self._enter("IDLE", now, (0, 0), "信号恢复，继续待机")
            else:
                self._hang_count += 1
                if self._hang_count >= SHOVEL_HANG_CONFIRM:
                    self._enter("REVERSE", now,
                                (-SHOVEL_REVERSE_SPEED, -SHOVEL_REVERSE_SPEED),
                                "铲子悬空确认，倒车收回")
            return self._result()

        if self.state == "REVERSE":
            if filtered_max < SHOVEL_HANG_CLEAR and elapsed >= SHOVEL_REVERSE_MIN_SECONDS:
                self._enter("IDLE", now, (0, 0), "铲子已收回台内")
            elif elapsed >= SHOVEL_REVERSE_TIMEOUT:
                self._enter("SAFE_STOP", now, (0, 0), "倒车超时未收回，停车待命")
            return self._result()

        if self.state == "SAFE_STOP":
            if filtered_max < SHOVEL_HANG_CLEAR:
                self._enter("IDLE", now, (0, 0), "信号恢复，解除保护停车")
            return self._result()

        # 理论不可达；防御兜底
        self._enter("IDLE", now, (0, 0), "未知状态")
        return self._result()

    def _result(self):
        return {
            "left": self.command[0],
            "right": self.command[1],
            "state": self.state,
            "reason": self.reason,
            "hang": self.hang,
        }
