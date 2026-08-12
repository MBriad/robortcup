#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import unittest

from config import VISION_TURN_MIN_SPEED
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
    def test_centered_good_target_stops(self):
        result = VisionTracker().update(frame(offset_x=0.02))
        self.assertEqual("ALIGNED", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))
        self.assertAlmostEqual(0.02, result["error_x"])

    def test_target_on_right_turns_right_above_motor_deadband(self):
        result = VisionTracker().update(frame(offset_x=0.25))
        self.assertEqual("ALIGN_RIGHT", result["state"])
        self.assertGreaterEqual(result["left"], VISION_TURN_MIN_SPEED)
        self.assertEqual(-result["left"], result["right"])

    def test_target_on_left_turns_left(self):
        result = VisionTracker().update(frame(offset_x=-0.25))
        self.assertEqual("ALIGN_LEFT", result["state"])
        self.assertLessEqual(result["left"], -VISION_TURN_MIN_SPEED)
        self.assertEqual(-result["left"], result["right"])

    def test_yolo_offset_is_not_normalized_twice(self):
        result = VisionTracker().update(frame(offset_x=0.075, frame_width=640))
        self.assertAlmostEqual(0.075, result["error_x"])
        self.assertEqual("ALIGN_RIGHT", result["state"])

    def test_bad_or_missing_target_does_not_move(self):
        tracker = VisionTracker()
        for control in (frame(action="scan_with_ir", target_type="bad"),
                        frame(action="search", target_type=None)):
            result = tracker.update(control)
            self.assertEqual("SEARCH", result["state"])
            self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_invalid_vision_stops_and_resets_filter(self):
        tracker = VisionTracker(filter_alpha=0.5)
        tracker.update(frame(offset_x=0.5))
        result = tracker.update(frame(valid=False))
        self.assertEqual("VISION_STOP", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))
        result = tracker.update(frame(offset_x=0.0))
        self.assertEqual("ALIGNED", result["state"])

    def test_filter_is_updated_only_when_update_is_called(self):
        tracker = VisionTracker(filter_alpha=0.5)
        tracker.update(frame(offset_x=1.0))
        result = tracker.update(frame(offset_x=0.0))
        self.assertAlmostEqual(0.5, result["filtered_error"])


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
            "target": {"type": "good", "offset_x": 80.0},
        }
        client._received_at = time.monotonic()
        result = client.get_control(max_age_ms=700)
        self.assertEqual(640, result["frame_width"])
        self.assertEqual(480, result["frame_height"])


if __name__ == "__main__":
    unittest.main()
