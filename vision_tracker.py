#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO good 能量块横向对准控制；仅处理注入数据，不持有相机或电机。"""

import math

from config import (
    VISION_DEAD_ZONE,
    VISION_ERROR_FILTER_ALPHA,
    VISION_IMAGE_WIDTH,
    VISION_TURN_KP,
    VISION_TURN_MAX_SPEED,
    VISION_TURN_MIN_SPEED,
)


class VisionTracker:
    """把 YOLO 横向像素偏差转换为左右轮原地转向命令。"""

    def __init__(
            self, image_width=VISION_IMAGE_WIDTH,
            filter_alpha=VISION_ERROR_FILTER_ALPHA,
            dead_zone=VISION_DEAD_ZONE, kp=VISION_TURN_KP,
            min_speed=VISION_TURN_MIN_SPEED,
            max_speed=VISION_TURN_MAX_SPEED):
        if image_width <= 0 or not 0.0 < filter_alpha <= 1.0:
            raise ValueError("image_width 必须为正，filter_alpha 必须在 (0, 1] 内")
        if not 0.0 <= dead_zone < 1.0 or not 0 < min_speed <= max_speed:
            raise ValueError("死区或转速范围无效")
        self.default_half_width = float(image_width) / 2.0
        self.filter_alpha = float(filter_alpha)
        self.dead_zone = float(dead_zone)
        self.kp = float(kp)
        self.min_speed = int(min_speed)
        self.max_speed = int(max_speed)
        self.filtered_error = None

    def _stop(self, state, reason, raw_error=None):
        self.filtered_error = None
        return {
            "left": 0,
            "right": 0,
            "state": state,
            "reason": reason,
            "error_x": raw_error,
            "filtered_error": None,
            "turn_command": 0,
        }

    def update(self, control):
        """处理一帧 VisionClient.get_control() 结果。"""
        if not control.get("valid"):
            return self._stop("VISION_STOP", control.get("reason", "vision_invalid"))
        if control.get("action") != "push":
            return self._stop("SEARCH", "当前没有 good 能量块")
        if control.get("target_type") not in (None, "good"):
            return self._stop("SEARCH", "当前目标不是 good 能量块")

        offset = control.get("offset_x")
        if not isinstance(offset, (int, float)) or not math.isfinite(offset):
            return self._stop("VISION_STOP", "good 目标缺少有效 offset_x")

        frame_width = control.get("frame_width")
        if (isinstance(frame_width, (int, float)) and
                math.isfinite(frame_width) and frame_width > 0):
            half_width = float(frame_width) / 2.0
        else:
            half_width = self.default_half_width
        error = float(offset) / half_width
        if self.filtered_error is None:
            self.filtered_error = error
        else:
            alpha = self.filter_alpha
            self.filtered_error = alpha * error + (1.0 - alpha) * self.filtered_error

        if abs(self.filtered_error) <= self.dead_zone:
            turn = 0
            state = "ALIGNED"
            reason = "good 能量块已进入中心死区"
        else:
            magnitude = int(round(abs(self.kp * self.filtered_error)))
            magnitude = max(self.min_speed, min(self.max_speed, magnitude))
            turn = magnitude if self.filtered_error > 0.0 else -magnitude
            state = "ALIGN_RIGHT" if turn > 0 else "ALIGN_LEFT"
            reason = "按横向误差原地对准 good 能量块"

        return {
            "left": turn,
            "right": -turn,
            "state": state,
            "reason": reason,
            "error_x": error,
            "filtered_error": self.filtered_error,
            "turn_command": turn,
        }
