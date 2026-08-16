#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""能量块追踪与近距 bad 避让；仅处理注入数据并返回电机命令。"""

import math
import time

from config import (
    HUNT_BAD_CENTER_ZONE,
    HUNT_BAD_CONFIRM_FRAMES,
    MOTOR_TURN_CALIBRATION,
)
from vision_tracker import VisionTracker


class HuntController:
    """追踪任意距离 good；只在红外近距确认时避开 bad。"""

    def __init__(
            self, tracker=None, center_zone=HUNT_BAD_CENTER_ZONE,
            confirm_frames=HUNT_BAD_CONFIRM_FRAMES,
            turn_calibration=MOTOR_TURN_CALIBRATION):
        if not 0.0 <= center_zone < 1.0:
            raise ValueError("bad 居中区间无效")
        if int(confirm_frames) < 1:
            raise ValueError("bad 确认帧数必须为正")
        self.tracker = tracker or VisionTracker()
        self.center_zone = float(center_zone)
        self.confirm_frames = int(confirm_frames)
        self.turn_calibration = turn_calibration
        self.state = "IDLE"
        self.command = (0, 0)
        self.turn_direction = None
        self.turn_until = 0.0
        self._near_bad_count = 0
        self._last_bad_sequence = None
        self._avoid_armed = True
        self._fallback_direction = "right"

    @staticmethod
    def _bbox_area(detection):
        bbox = detection.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return 0.0
        try:
            x1, y1, x2, y2 = (float(value) for value in bbox)
        except (TypeError, ValueError):
            return 0.0
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
            return 0.0
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    def _select(self, raw, target_type):
        if not isinstance(raw, dict):
            return None
        preferred = raw.get("target")
        if isinstance(preferred, dict) and preferred.get("type") == target_type:
            return preferred
        matches = [
            detection for detection in (raw.get("detections") or [])
            if detection.get("type") == target_type
        ]
        return max(matches, key=self._bbox_area) if matches else None

    @staticmethod
    def _offset(target):
        if target is None:
            return None
        value = target.get("offset_x")
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            return None
        return float(value)

    @staticmethod
    def _near_direction(ir):
        if not isinstance(ir, dict) or not ir.get("valid"):
            return None
        left_front = bool(ir.get("left_front"))
        right_front = bool(ir.get("right_front"))
        left_rear = bool(ir.get("left_rear"))
        right_rear = bool(ir.get("right_rear"))
        if ir.get("front") or (left_front and right_front):
            return "front"
        if ir.get("rear") or (left_rear and right_rear):
            return "rear"
        if left_front or left_rear:
            return "left"
        if right_front or right_rear:
            return "right"
        return None

    def _near_target(self, raw, near_direction):
        if near_direction in (None, "rear") or not isinstance(raw, dict):
            return None
        candidates = []
        detections = list(raw.get("detections") or [])
        preferred = raw.get("target")
        if isinstance(preferred, dict) and preferred not in detections:
            detections.append(preferred)
        for detection in detections:
            offset = self._offset(detection)
            if offset is None:
                continue
            if near_direction == "left" and offset <= self.center_zone:
                candidates.append((offset, detection))
            elif near_direction == "right" and offset >= -self.center_zone:
                candidates.append((-offset, detection))
            elif near_direction == "front":
                candidates.append((abs(offset), detection))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    def _result(self, owns_control, mode, state, reason, target=None,
                good_offset=None, bad_offset=None, near_direction=None):
        return {
            "left": self.command[0],
            "right": self.command[1],
            "owns_control": bool(owns_control),
            "mode": mode,
            "state": state,
            "reason": reason,
            "target": target,
            "target_type": target.get("type") if target else None,
            "good_offset_x": good_offset,
            "bad_offset_x": bad_offset,
            "near_direction": near_direction,
            "turn_direction": self.turn_direction,
        }

    def cancel(self):
        self.state = "IDLE"
        self.command = (0, 0)
        self.turn_direction = None
        self.turn_until = 0.0
        self._near_bad_count = 0
        self._last_bad_sequence = None
        self._avoid_armed = True
        self.tracker.big_turn_direction = None

    def _choose_turn(self, bad_offset, good_offset, near_direction):
        if bad_offset < -self.center_zone:
            return "right"
        if bad_offset > self.center_zone:
            return "left"
        if good_offset is not None and abs(good_offset) > self.tracker.dead_zone:
            return "right" if good_offset > 0.0 else "left"
        if near_direction == "left":
            return "right"
        if near_direction == "right":
            return "left"
        direction = self._fallback_direction
        self._fallback_direction = "left" if direction == "right" else "right"
        return direction

    def _start_turn(self, direction, now):
        speed, duration = self.turn_calibration[direction][90.0]
        speed = int(speed)
        self.command = (speed, -speed) if direction == "right" else (-speed, speed)
        self.state = "AVOID_TURN"
        self.turn_direction = direction
        self.turn_until = now + float(duration)
        self._near_bad_count = 0
        self._last_bad_sequence = None

    def update(self, raw, ir, now=None, healthy=True):
        now = time.monotonic() if now is None else float(now)
        near_direction = self._near_direction(ir)

        if not healthy:
            self.cancel()
            return self._result(
                True, "safety_stop", "SENSOR_STOP", "传感器无效或数据过期",
                near_direction=near_direction,
            )

        if self.state == "AVOID_TURN":
            if now < self.turn_until:
                return self._result(
                    True, "avoid_bad", "AVOID_TURN",
                    "锁定方向完成 90 度原地转向",
                    near_direction=near_direction,
                )
            self.state = "IDLE"
            self.command = (0, 0)
            self.turn_direction = None
            self._avoid_armed = False
            return self._result(
                True, "release", "AVOID_RELEASE",
                "bad 避让转向完成，停车一帧后释放控制权",
                near_direction=near_direction,
            )

        self.command = (0, 0)
        vision_valid = (
            isinstance(raw, dict)
            and raw.get("sequence") is not None
            and raw.get("status") != "error"
        )
        good = self._select(raw, "good") if vision_valid else None
        bad = self._select(raw, "bad") if vision_valid else None
        near_target = self._near_target(raw, near_direction) if vision_valid else None
        if near_target is not None and near_target.get("type") == "good":
            good = near_target
        good_offset = self._offset(good)
        bad_offset = self._offset(bad)
        near_bad = near_target is not None and near_target.get("type") == "bad"
        if near_bad:
            bad = near_target
            bad_offset = self._offset(bad)

        if not near_bad:
            self._near_bad_count = 0
            self._last_bad_sequence = None
            self._avoid_armed = True

        if near_bad and self._avoid_armed:
            sequence = raw.get("sequence")
            if sequence != self._last_bad_sequence:
                self._near_bad_count += 1
                self._last_bad_sequence = sequence
            if self._near_bad_count < self.confirm_frames:
                self.state = "BAD_CONFIRM"
                return self._result(
                    True, "avoid_bad", "BAD_CONFIRM",
                    "红外近距 bad 等待不同视觉帧确认",
                    bad, good_offset, bad_offset, near_direction,
                )
            direction = self._choose_turn(
                bad_offset, good_offset, near_direction
            )
            self._start_turn(direction, now)
            return self._result(
                True, "avoid_bad", "AVOID_TURN",
                "近距 bad 已确认，锁定方向原地转 90 度",
                bad, good_offset, bad_offset, near_direction,
            )

        if near_bad and not self._avoid_armed:
            self.state = "IDLE"
            return self._result(
                False, "release", "AVOID_DONE",
                "本次近距 bad 已避让，等待目标离开后重新布防",
                bad, good_offset, bad_offset, near_direction,
            )

        if good is not None and good_offset is not None:
            tracked = self.tracker.update({
                "valid": True,
                "reason": "ok",
                "action": "push",
                "target_type": "good",
                "offset_x": good_offset,
            })
            self.command = (tracked["left"], tracked["right"])
            self.state = tracked["state"]
            return self._result(
                True, "track_good", tracked["state"], tracked["reason"],
                good, good_offset, bad_offset, near_direction,
            )

        self.tracker.big_turn_direction = None
        if near_direction is not None:
            self.state = "IR_UNCLASSIFIED"
            return self._result(
                False, "release", "IR_UNCLASSIFIED",
                "红外近物体未被识别为能量块，交给敌人模块",
                bad, good_offset, bad_offset, near_direction,
            )
        self.state = "IDLE"
        return self._result(
            False, "release", "NO_TARGET",
            "没有可追踪 good；远处 bad 不触发避让",
            bad, good_offset, bad_offset, near_direction,
        )
