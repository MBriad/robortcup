#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import MOTOR_TURN_CALIBRATION, PATROL_RECOVER_STEP_CM
from dev.motor_tool import DEFAULT_SPEED, MIN_SPEED, build_cases

class MotorDirectionTest(unittest.TestCase):
    def test_four_direction_signs(self):
        cases = {action: (left, right)
                 for action, _, left, right in build_cases(120)}
        self.assertEqual((120, 120), cases["forward"])
        self.assertEqual((-120, -120), cases["backward"])
        self.assertEqual((-120, 120), cases["turn-left"])
        self.assertEqual((120, -120), cases["turn-right"])

    def test_speed_is_applied_to_every_case(self):
        for _, _, left, right in build_cases(MIN_SPEED):
            self.assertEqual(MIN_SPEED, abs(left))
            self.assertEqual(MIN_SPEED, abs(right))

    def test_default_speed_clears_motor_deadband(self):
        self.assertGreaterEqual(DEFAULT_SPEED, MIN_SPEED)

    def test_turn_calibration_csv_matches_config(self):
        path = os.path.join(ROOT, "data", "motor_turn_calibration.csv")
        by_direction = {"left": {}, "right": {}}
        with open(path, newline="", encoding="utf-8-sig") as handle:
            for row in csv.DictReader(handle):
                by_direction[row["direction"]][float(row["angle"])] = (
                    int(row["speed"]), float(row["duration"])
                )
        self.assertEqual(MOTOR_TURN_CALIBRATION["left"], by_direction["left"])
        self.assertEqual(MOTOR_TURN_CALIBRATION["right"], by_direction["right"])

    def test_linear_calibration_csv_matches_recover_step(self):
        path = os.path.join(ROOT, "data", "motor_linear_calibration.csv")
        with open(path, newline="", encoding="utf-8-sig") as handle:
            rows = list(csv.DictReader(handle))

        self.assertEqual({"forward", "backward"}, {row["direction"] for row in rows})
        for row in rows:
            self.assertEqual(400, int(row["speed"]))
            self.assertEqual(1.0, float(row["duration"]))
            self.assertEqual(PATROL_RECOVER_STEP_CM, float(row["distance_cm"]))
            self.assertEqual("distance_measured", row["result"])


if __name__ == "__main__":
    unittest.main()
