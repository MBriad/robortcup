#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""能量块追踪与近距 bad 避让；仅处理注入数据并返回电机命令。"""

import math
import time

from config import (
    HUNT_BAD_CENTER_ZONE,
    HUNT_BAD_CONFIRM_FRAMES,
    HUNT_GOOD_ACQUIRE_FRAMES,
    HUNT_GOOD_CONFIRM_FRAMES,
    HUNT_GOOD_HIGH_CONFIDENCE,
    HUNT_GOOD_LOST_HOLD_FRAMES,
    HUNT_GOOD_LOST_HOLD_SECONDS,
    HUNT_GOOD_MIN_CONFIDENCE,
    HUNT_GOOD_PUSH_SPEED,
    MOTOR_TURN_CALIBRATION,
)
from vision_tracker import VisionTracker


class HuntController:
    """追踪任意距离 good；只在红外近距确认时避开 bad。"""

    def __init__(
            self, tracker=None, center_zone=HUNT_BAD_CENTER_ZONE,
            confirm_frames=HUNT_BAD_CONFIRM_FRAMES,
            good_min_confidence=HUNT_GOOD_MIN_CONFIDENCE,
            good_high_confidence=HUNT_GOOD_HIGH_CONFIDENCE,
            good_acquire_frames=HUNT_GOOD_ACQUIRE_FRAMES,
            good_lost_hold_frames=HUNT_GOOD_LOST_HOLD_FRAMES,
            good_lost_hold_seconds=HUNT_GOOD_LOST_HOLD_SECONDS,
            good_confirm_frames=HUNT_GOOD_CONFIRM_FRAMES,
            good_push_speed=HUNT_GOOD_PUSH_SPEED,
            turn_calibration=MOTOR_TURN_CALIBRATION):
        if not 0.0 <= center_zone < 1.0:
            raise ValueError("bad 居中区间无效")
        if int(confirm_frames) < 1:
            raise ValueError("bad 确认帧数必须为正")
        if not 0.0 <= good_min_confidence <= good_high_confidence <= 1.0:
            raise ValueError("good 置信度阈值顺序无效")
        if int(good_acquire_frames) < 1:
            raise ValueError("good 获取确认帧数必须为正")
        if int(good_lost_hold_frames) < 0:
            raise ValueError("good 丢帧保留帧数不能为负")
        if float(good_lost_hold_seconds) <= 0.0:
            raise ValueError("good 丢帧保留时间必须为正")
        if int(good_confirm_frames) < 1:
            raise ValueError("good 确认帧数必须为正")
        self.tracker = tracker or VisionTracker()
        self.center_zone = float(center_zone)
        self.confirm_frames = int(confirm_frames)
        self.good_min_confidence = float(good_min_confidence)
        self.good_high_confidence = float(good_high_confidence)
        self.good_acquire_frames = int(good_acquire_frames)
        self.good_lost_hold_frames = int(good_lost_hold_frames)
        self.good_lost_hold_seconds = float(good_lost_hold_seconds)
        self.good_confirm_frames = int(good_confirm_frames)
        self.good_push_speed = int(good_push_speed)
        self.turn_calibration = turn_calibration
        self.state = "IDLE"
        self.command = (0, 0)
        self.turn_direction = None
        self.turn_until = 0.0
        self._near_bad_count = 0
        self._last_bad_sequence = None
        self._avoid_armed = True
        self._fallback_direction = "right"
        self._good_confirm_count = 0
        self._last_good_sequence = None
        self._good_armed = True
        self._good_acquire_count = 0
        self._last_good_acquire_sequence = None
        self._good_locked = False
        self._good_miss_count = 0
        self._last_good_miss_sequence = None
        self._good_last_seen_at = None
        self._last_good_seen_sequence = None

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
    def _confidence(target):
        if target is None:
            return None
        value = target.get("confidence")
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
            if detection.get("type") == "good":
                confidence = self._confidence(detection)
                if (confidence is None
                        or confidence < self.good_min_confidence):
                    continue
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
        good_confidence = self._confidence(target) if (
            target and target.get("type") == "good"
        ) else None
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
            "good_confidence": good_confidence,
            "good_acquire_count": self._good_acquire_count,
            "good_miss_count": self._good_miss_count,
            "good_locked": self._good_locked,
        }

    def cancel(self):
        self.state = "IDLE"
        self.command = (0, 0)
        self.turn_direction = None
        self.turn_until = 0.0
        self._near_bad_count = 0
        self._last_bad_sequence = None
        self._avoid_armed = True
        self._good_confirm_count = 0
        self._last_good_sequence = None
        self._good_acquire_count = 0
        self._last_good_acquire_sequence = None
        self._good_locked = False
        self._good_miss_count = 0
        self._last_good_miss_sequence = None
        self._good_last_seen_at = None
        self._last_good_seen_sequence = None
        self.tracker.big_turn_direction = None

    def finish_push(self):
        """结束本次推动；当前 good 消失前禁止立即重新推动。"""
        self.cancel()
        self._good_armed = False

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

        if self.state == "GOOD_PUSH":
            speed = self.good_push_speed
            self.command = (speed, speed)
            return self._result(
                True, "push_good", "GOOD_PUSH",
                "good 已居中，持续前推直到铲子悬空保护触发",
                near_direction=near_direction,
            )

        self.command = (0, 0)
        vision_valid = (
            isinstance(raw, dict)
            and raw.get("sequence") is not None
            and raw.get("status") in ("target", "no_target")
        )
        good = self._select(raw, "good") if vision_valid else None
        bad = self._select(raw, "bad") if vision_valid else None
        near_target = self._near_target(raw, near_direction) if vision_valid else None
        if near_target is not None and near_target.get("type") == "good":
            good = near_target
        raw_good = good
        good_confidence = self._confidence(good)
        if (good_confidence is None
                or good_confidence < self.good_min_confidence):
            good = None
            good_confidence = None
        good_offset = self._offset(good)
        bad_offset = self._offset(bad)
        near_bad = near_target is not None and near_target.get("type") == "bad"
        if near_bad:
            self._good_confirm_count = 0
            self._last_good_sequence = None
            self._good_acquire_count = 0
            self._last_good_acquire_sequence = None
            self._good_locked = False
            self._good_miss_count = 0
            self._last_good_miss_sequence = None
            self._good_last_seen_at = None
            self._last_good_seen_sequence = None
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

        if not self._good_armed:
            if vision_valid and raw_good is None:
                self._good_armed = True
            else:
                self.state = "GOOD_REARM_WAIT"
                self.command = (0, 0)
                return self._result(
                    False, "release", "GOOD_REARM_WAIT",
                    "等待当前 good 消失后重新允许推动",
                    good, good_offset, bad_offset, near_direction,
                )

        good_sequence = raw.get("sequence") if isinstance(raw, dict) else None
        if good is not None:
            if good_sequence != self._last_good_seen_sequence:
                if (self._good_last_seen_at is not None
                        and now - self._good_last_seen_at
                        > self.good_lost_hold_seconds):
                    self._good_locked = False
                    self._good_acquire_count = 0
                    self._last_good_acquire_sequence = None
                    self._good_confirm_count = 0
                    self._last_good_sequence = None
                self._last_good_seen_sequence = good_sequence
                self._good_last_seen_at = now
            elif (self._good_last_seen_at is not None
                  and now - self._good_last_seen_at
                  > self.good_lost_hold_seconds):
                good = None
                good_confidence = None
                good_offset = None

        if good is not None and good_offset is not None:
            self._good_miss_count = 0
            self._last_good_miss_sequence = None
            if not self._good_locked:
                sequence = raw.get("sequence")
                if good_confidence >= self.good_high_confidence:
                    self._good_locked = True
                else:
                    if sequence != self._last_good_acquire_sequence:
                        self._good_acquire_count += 1
                        self._last_good_acquire_sequence = sequence
                    if self._good_acquire_count < self.good_acquire_frames:
                        self.state = "GOOD_ACQUIRE"
                        return self._result(
                            True, "track_good", "GOOD_ACQUIRE",
                            "中置信度 good 停车等待不同视觉帧确认",
                            good, good_offset, bad_offset, near_direction,
                        )
                    self._good_locked = True
                self._good_acquire_count = 0
                self._last_good_acquire_sequence = None
            tracked = self.tracker.update({
                "valid": True,
                "reason": "ok",
                "action": "push",
                "target_type": "good",
                "offset_x": good_offset,
            })
            self.command = (tracked["left"], tracked["right"])
            if tracked["state"] == "APPROACH":
                sequence = raw.get("sequence")
                if sequence != self._last_good_sequence:
                    self._good_confirm_count += 1
                    self._last_good_sequence = sequence
                if self._good_confirm_count < self.good_confirm_frames:
                    self.command = (0, 0)
                    self.state = "GOOD_CONFIRM"
                    return self._result(
                        True, "track_good", "GOOD_CONFIRM",
                        "good 已居中，等待不同视觉帧确认",
                        good, good_offset, bad_offset, near_direction,
                    )
                speed = self.good_push_speed
                self.command = (speed, speed)
                self.state = "GOOD_PUSH"
                return self._result(
                    True, "push_good", "GOOD_PUSH",
                    "good 已居中，锁定持续前推直到铲子保护触发",
                    good, good_offset, bad_offset, near_direction,
                )
            self._good_confirm_count = 0
            self._last_good_sequence = None
            self.state = tracked["state"]
            return self._result(
                True, "track_good", tracked["state"], tracked["reason"],
                good, good_offset, bad_offset, near_direction,
            )

        self._good_confirm_count = 0
        self._last_good_sequence = None

        if self._good_locked:
            sequence = raw.get("sequence") if isinstance(raw, dict) else None
            if (sequence is not None
                    and sequence != self._last_good_miss_sequence):
                self._good_miss_count += 1
                self._last_good_miss_sequence = sequence
            elapsed = (
                now - self._good_last_seen_at
                if self._good_last_seen_at is not None else float("inf")
            )
            if (self._good_miss_count <= self.good_lost_hold_frames
                    and elapsed <= self.good_lost_hold_seconds):
                self.state = "GOOD_LOST_HOLD"
                return self._result(
                    True, "track_good", "GOOD_LOST_HOLD",
                    "已锁定 good 临时丢帧，停车保留目标身份",
                    good_offset=good_offset, bad_offset=bad_offset,
                    near_direction=near_direction,
                )
            self._good_locked = False
            self._good_miss_count = 0
            self._last_good_miss_sequence = None
            self._good_last_seen_at = None
            self._last_good_seen_sequence = None

        self._good_acquire_count = 0
        self._last_good_acquire_sequence = None

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
