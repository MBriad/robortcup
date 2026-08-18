#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import unittest

from config import MOTOR_TURN_CALIBRATION
from hunt import HuntController
from dev.hunt import default_output_path, rows_for_result


SENSORS = {
    "valid": 1,
    "error": "",
    "adc": list(range(10)),
    "mask": 5,
    "io": [1, 0, 1, 0, 0, 0, 0, 0],
}


class HuntToolTest(unittest.TestCase):
    def test_all_detections_are_expanded_with_bbox_metrics(self):
        bad = {
            "class_id": 1,
            "type": "bad",
            "confidence": 0.9,
            "bbox": [100, 120, 200, 170],
            "center_x": 150,
            "center_y": 145,
            "offset_x": -0.53125,
            "offset_y": -0.395833,
            "distance_cm": None,
        }
        good = {
            "class_id": 0,
            "type": "good",
            "confidence": 0.8,
            "bbox": [300, 200, 340, 240],
            "center_x": 320,
            "center_y": 220,
            "offset_x": 0.0,
            "offset_y": -0.083333,
            "distance_cm": None,
        }
        raw = {
            "sequence": 3,
            "timestamp_ms": 1000,
            "age_ms": 8.0,
            "frame_width": 640,
            "frame_height": 480,
            "status": "target",
            "action": "push",
            "fps": 8.0,
            "inference_ms": 110.0,
            "target": good,
            "detections": [bad, good],
        }
        hunt_result = HuntController(confirm_frames=1).update(
            raw,
            {
                "left_rear": False,
                "left_front": True,
                "right_rear": False,
                "right_front": False,
                "rear": False,
                "front": False,
                "valid": True,
            },
            now=0.0,
        )
        rows = rows_for_result(
            raw, SENSORS, "mixed", 50.0, 1.25,
            hunt_result=hunt_result, motor_enabled=True,
        )
        self.assertEqual(2, len(rows))
        self.assertEqual("bad", rows[0]["target_type"])
        self.assertEqual(1, rows[0]["selected_target"])
        self.assertEqual(100.0, rows[0]["bbox_width"])
        self.assertEqual(50.0, rows[0]["bbox_height"])
        self.assertAlmostEqual(5000.0 / (640.0 * 480.0), rows[0]["bbox_area_ratio"])
        self.assertEqual(0, rows[1]["selected_target"])
        self.assertEqual(7, rows[1]["adc7"])
        self.assertEqual(1, rows[1]["io2"])
        self.assertEqual("avoid_bad", rows[0]["hunt_mode"])
        self.assertEqual("AVOID_TURN", rows[0]["hunt_state"])
        speed = MOTOR_TURN_CALIBRATION["right"][90.0][0]
        self.assertEqual(speed, rows[0]["left_cmd"])
        self.assertEqual(-speed, rows[0]["right_cmd"])
        self.assertEqual(1, rows[0]["motor_enabled"])

    def test_empty_frame_is_kept_for_baseline(self):
        raw = {
            "sequence": 4,
            "status": "no_target",
            "action": "search",
            "frame_width": 640,
            "frame_height": 480,
            "detections": [],
        }
        rows = rows_for_result(raw, SENSORS, "empty_scene", None, 2.0)
        self.assertEqual(1, len(rows))
        self.assertEqual(0, rows[0]["detection_count"])
        self.assertEqual("", rows[0]["target_type"])
        self.assertEqual("empty_scene", rows[0]["label"])
        self.assertEqual("COLLECT_ONLY", rows[0]["hunt_state"])
        self.assertEqual(0, rows[0]["motor_enabled"])

    def test_stale_result_is_logged_as_invalid(self):
        rows = rows_for_result(None, SENSORS, "bad_left_far", 80.0, 0.0)
        self.assertEqual(0, rows[0]["vision_valid"])
        self.assertEqual("no_data_or_stale", rows[0]["vision_status"])
        self.assertEqual(80.0, rows[0]["measured_distance_cm"])

    def test_default_filename_uses_safe_label(self):
        path = default_output_path("bad left/far")
        self.assertEqual("data", os.path.basename(os.path.dirname(path)))
        self.assertTrue(os.path.basename(path).startswith("bad_left_far_"))


if __name__ == "__main__":
    unittest.main()
