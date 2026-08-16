#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import unittest

from config import (
    VISION_APPROACH_SPEED,
    VISION_ARC_INNER_SPEED,
    VISION_ARC_OUTER_SPEED,
    VISION_BIG_TURN_CLEAR,
    VISION_BIG_TURN_ENTER,
    VISION_BIG_TURN_SPEED,
    VISION_DEAD_ZONE,
)
from vision_tracker import VisionTracker

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YOLO_DIR = os.path.join(ROOT, "rpi-yolo-pi4-int8-lto-8fps")
if YOLO_DIR not in sys.path:
    sys.path.insert(0, YOLO_DIR)

from rpi_yolo_api import VisionClient


def frame(offset_x=0.0, action="push", target_type="good", valid=True,
          frame_width=320):
    return {
        "valid": valid,
        "reason": "ok" if valid else "stale",
        "action": action,
        "target_type": target_type,
        "offset_x": offset_x,
        "frame_width": frame_width,
    }


class VisionTrackerTest(unittest.TestCase):
    def test_centered_target_drives_straight(self):
        result = VisionTracker().update(frame(offset_x=VISION_DEAD_ZONE / 2.0))
        self.assertEqual("APPROACH", result["state"])
        self.assertEqual(
            (VISION_APPROACH_SPEED, VISION_APPROACH_SPEED),
            (result["left"], result["right"]),
        )

    def test_small_right_error_uses_forward_arc(self):
        result = VisionTracker().update(frame(offset_x=0.25))
        self.assertEqual("ARC_RIGHT", result["state"])
        self.assertEqual(
            (VISION_ARC_OUTER_SPEED, VISION_ARC_INNER_SPEED),
            (result["left"], result["right"]),
        )

    def test_small_left_error_uses_forward_arc(self):
        result = VisionTracker().update(frame(offset_x=-0.25))
        self.assertEqual("ARC_LEFT", result["state"])
        self.assertEqual(
            (VISION_ARC_INNER_SPEED, VISION_ARC_OUTER_SPEED),
            (result["left"], result["right"]),
        )

    def test_large_errors_use_in_place_turn(self):
        cases = (
            (VISION_BIG_TURN_ENTER, "BIG_TURN_RIGHT",
             (VISION_BIG_TURN_SPEED, -VISION_BIG_TURN_SPEED)),
            (-VISION_BIG_TURN_ENTER, "BIG_TURN_LEFT",
             (-VISION_BIG_TURN_SPEED, VISION_BIG_TURN_SPEED)),
        )
        for error, state, command in cases:
            result = VisionTracker().update(frame(offset_x=error))
            self.assertEqual(state, result["state"])
            self.assertEqual(command, (result["left"], result["right"]))

    def test_big_turn_uses_hysteresis_before_returning_to_arc(self):
        tracker = VisionTracker()
        result = tracker.update(frame(offset_x=VISION_BIG_TURN_ENTER + 0.05))
        self.assertEqual("BIG_TURN_RIGHT", result["state"])
        result = tracker.update(frame(offset_x=VISION_BIG_TURN_CLEAR + 0.05))
        self.assertEqual("BIG_TURN_RIGHT", result["state"])
        result = tracker.update(frame(offset_x=VISION_BIG_TURN_CLEAR - 0.05))
        self.assertEqual("ARC_RIGHT", result["state"])
        result = tracker.update(frame(offset_x=VISION_BIG_TURN_ENTER - 0.05))
        self.assertEqual("ARC_RIGHT", result["state"])

    def test_direction_uses_raw_error_without_filter_lag(self):
        tracker = VisionTracker()
        tracker.update(frame(offset_x=-0.25))
        result = tracker.update(frame(offset_x=0.25))
        self.assertEqual("ARC_RIGHT", result["state"])
        self.assertEqual(
            (VISION_ARC_OUTER_SPEED, VISION_ARC_INNER_SPEED),
            (result["left"], result["right"]),
        )

    def test_bad_missing_or_invalid_target_stops(self):
        tracker = VisionTracker()
        controls = (
            frame(action="scan_with_ir", target_type="bad"),
            frame(action="search", target_type=None),
            frame(valid=False),
        )
        for control in controls:
            result = tracker.update(control)
            self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_all_nonzero_wheel_commands_clear_motor_deadband(self):
        tracker = VisionTracker()
        for error in (-0.8, -0.25, 0.0, 0.25, 0.8):
            result = tracker.update(frame(offset_x=error))
            for command in (result["left"], result["right"]):
                if command:
                    self.assertGreaterEqual(abs(command), 400)


class VisionClientTest(unittest.TestCase):
    def test_control_result_keeps_runtime_frame_size(self):
        client = VisionClient()
        client._latest = {
            "sequence": 7,
            "timestamp_ms": 100,
            "frame_width": 640,
            "frame_height": 480,
            "status": "target",
            "action": "push",
            "target": {"type": "good", "offset_x": 0.25},
        }
        client._received_at = time.monotonic()
        result = client.get_control(max_age_ms=700)
        self.assertEqual(640, result["frame_width"])
        self.assertEqual(480, result["frame_height"])


if __name__ == "__main__":
    unittest.main()
