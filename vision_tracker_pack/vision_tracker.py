#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO good 能量块追踪控制；仅处理注入数据，不持有相机或电机。"""

import math

from config import (
    VISION_APPROACH_SPEED,
    VISION_ARC_INNER_SPEED,
    VISION_ARC_OUTER_SPEED,
    VISION_BIG_TURN_CLEAR,
    VISION_BIG_TURN_ENTER,
    VISION_BIG_TURN_SPEED,
    VISION_DEAD_ZONE,
)


class VisionTracker:
    """按归一化横向误差选择大转、小弧线或直线接近。"""

    def __init__(
            self, dead_zone=VISION_DEAD_ZONE,
            big_turn_enter=VISION_BIG_TURN_ENTER,
            big_turn_clear=VISION_BIG_TURN_CLEAR,
            big_turn_speed=VISION_BIG_TURN_SPEED,
            arc_inner_speed=VISION_ARC_INNER_SPEED,
            arc_outer_speed=VISION_ARC_OUTER_SPEED,
            approach_speed=VISION_APPROACH_SPEED):
        if not 0.0 <= dead_zone < big_turn_clear < big_turn_enter <= 1.0:
            raise ValueError("视觉死区和大小转阈值顺序无效")
        if not 0 < big_turn_speed <= 1023:
            raise ValueError("大转速度无效")
        if not 0 < arc_inner_speed <= arc_outer_speed <= 1023:
            raise ValueError("小转差速无效")
        if not 0 < approach_speed <= 1023:
            raise ValueError("接近速度无效")
        self.dead_zone = float(dead_zone)
        self.big_turn_enter = float(big_turn_enter)
        self.big_turn_clear = float(big_turn_clear)
        self.big_turn_speed = int(big_turn_speed)
        self.arc_inner_speed = int(arc_inner_speed)
        self.arc_outer_speed = int(arc_outer_speed)
        self.approach_speed = int(approach_speed)
        self.big_turn_direction = None

    def _result(self, command, state, reason, error=None, turn=0):
        return {
            "left": command[0],
            "right": command[1],
            "state": state,
            "reason": reason,
            "error_x": error,
            "turn_command": turn,
        }

    def _stop(self, state, reason):
        self.big_turn_direction = None
        return self._result((0, 0), state, reason)

    def _big_turn(self, direction, error):
        speed = self.big_turn_speed
        if direction == "right":
            command = (speed, -speed)
            turn = speed
        else:
            command = (-speed, speed)
            turn = -speed
        self.big_turn_direction = direction
        return self._result(
            command,
            "BIG_TURN_RIGHT" if direction == "right" else "BIG_TURN_LEFT",
            "目标偏差较大，原地大转",
            error,
            turn,
        )

    def _arc(self, direction, error):
        inner = self.arc_inner_speed
        outer = self.arc_outer_speed
        if direction == "right":
            command = (outer, inner)
            turn = outer - inner
        else:
            command = (inner, outer)
            turn = inner - outer
        return self._result(
            command,
            "ARC_RIGHT" if direction == "right" else "ARC_LEFT",
            "持续差速前进并小幅对准",
            error,
            turn,
        )

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

        error = float(offset)
        magnitude = abs(error)
        if magnitude <= self.dead_zone:
            self.big_turn_direction = None
            speed = self.approach_speed
            return self._result(
                (speed, speed),
                "APPROACH",
                "good 能量块已居中，直线接近",
                error,
            )

        direction = "right" if error > 0.0 else "left"
        if (self.big_turn_direction == direction and
                magnitude > self.big_turn_clear):
            return self._big_turn(direction, error)

        self.big_turn_direction = None
        if magnitude >= self.big_turn_enter:
            return self._big_turn(direction, error)
        return self._arc(direction, error)
