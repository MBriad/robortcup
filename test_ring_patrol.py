#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import unittest

from gray import GRAY_CENTER_REFERENCE, GRAY_EDGE_REFERENCE, GRAY_WHITE_REFERENCE
from ring_patrol import RingPatrolController


DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


def replay(filename):
    controller = RingPatrolController()
    states = []
    with open(os.path.join(DATA_DIR, filename), newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            raw = {name: float(row[name]) for name in ("front", "rear", "left", "right")}
            states.append(controller.update(raw, now=float(row["t"])))
    return states


class RingPatrolTest(unittest.TestCase):
    def test_center_data_never_triggers_escape(self):
        files = ("武字中间旋转2圈.csv", "武字中间逆时针旋转2圈.csv",
                 "武字数据.csv", "武字数据逆时针.csv")
        for filename in files:
            states = replay(filename)
            unsafe = {item["state"] for item in states} & {"WHITE_ESCAPE", "RECENTER"}
            self.assertEqual(set(), unsafe, filename)

    def test_edge_data_enters_recenter_without_white_escape(self):
        states = replay("边缘.csv")
        names = {item["state"] for item in states}
        self.assertIn("RECENTER", names)
        self.assertNotIn("WHITE_ESCAPE", names)

    def test_axis_replay_contains_edge_and_center_sections(self):
        states = replay("中轴.csv")
        names = [item["state"] for item in states]
        self.assertIn("RECENTER", names[:300])
        self.assertIn("CRUISE", names)
        self.assertIn("RECENTER", names[-400:])

    def test_single_sample_jump_is_filtered(self):
        controller = RingPatrolController()
        raw = dict(GRAY_CENTER_REFERENCE)
        for index in range(10):
            sample = dict(raw)
            if index == 5:
                sample["front"] = 6000.0
            result = controller.update(sample, now=index * 0.02)
            self.assertNotEqual("WHITE_ESCAPE", result["state"])

    def white_command(self, sensor):
        controller = RingPatrolController()
        sample = dict(GRAY_CENTER_REFERENCE)
        sample[sensor] = GRAY_WHITE_REFERENCE[sensor]
        result = None
        for index in range(6):
            result = controller.update(sample, now=index * 0.02)
        self.assertEqual("WHITE_ESCAPE", result["state"])
        return result["left"], result["right"]

    def test_front_white_commands_reverse(self):
        left, right = self.white_command("front")
        self.assertLess(left, 0)
        self.assertLess(right, 0)

    def test_rear_white_commands_forward(self):
        left, right = self.white_command("rear")
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)

    def test_left_white_commands_turn_right(self):
        left, right = self.white_command("left")
        self.assertGreater(left, 0)
        self.assertLess(right, 0)

    def test_right_white_commands_turn_left(self):
        left, right = self.white_command("right")
        self.assertLess(left, 0)
        self.assertGreater(right, 0)

    def test_sensor_fault_stops(self):
        controller = RingPatrolController()
        result = controller.update(dict(GRAY_EDGE_REFERENCE), healthy=False)
        self.assertEqual((0, 0), (result["left"], result["right"]))
        self.assertEqual("SENSOR_STOP", result["state"])


if __name__ == "__main__":
    unittest.main()
