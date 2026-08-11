#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from digi_ir import DIGI_IR_PINS
from gray import GRAY_CENTER_REFERENCE, GRAY_EDGE_REFERENCE
from main import RobotController


def ir_states(**active):
    states = {name: False for name in DIGI_IR_PINS}
    states.update(active)
    states["valid"] = True
    return states


ANALOG_VALID = {"left": 500.0, "right": 500.0, "valid": True}
FALLEN_GRAY = {
    name: max(0.0, value - 100.0)
    for name, value in GRAY_EDGE_REFERENCE.items()
}


class RobotControllerTest(unittest.TestCase):
    def update(self, robot, gray, ir=None, now=0.0):
        return robot.update(
            gray,
            ir_states() if ir is None else ir,
            ANALOG_VALID,
            now=now,
            healthy=True,
        )

    def warm_patrol(self, robot):
        result = None
        for index in range(5):
            result = self.update(
                robot, dict(GRAY_CENTER_REFERENCE), now=index * 0.02
            )
        return result

    def test_safe_ground_uses_patrol_command(self):
        robot = RobotController()
        result = self.warm_patrol(robot)
        self.assertEqual("patrol", result["mode"])
        self.assertEqual("CRUISE", result["state"])
        self.assertGreater(result["left"], 0)
        self.assertGreater(result["right"], 0)

    def test_reentry_preempts_patrol_after_fall_trigger(self):
        robot = RobotController()
        self.warm_patrol(robot)
        result = None
        for index in range(5, 11):
            result = self.update(
                robot,
                FALLEN_GRAY,
                ir_states(right_front=True),
                now=index * 0.02,
            )
            if result["mode"] == "reentry":
                break
        self.assertEqual("reentry", result["mode"])
        self.assertEqual("TURN_RIGHT_90", result["state"])
        self.assertEqual((500, -500), (result["left"], result["right"]))

    def test_patrol_is_recreated_after_reentry_finishes(self):
        robot = RobotController()
        self.warm_patrol(robot)
        previous_patrol = robot.patrol
        robot._reentry_active = True
        robot.reentry.state = "SAFE_STOP"

        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE), now=1.0
        )

        self.assertEqual("patrol", result["mode"])
        self.assertIsNot(previous_patrol, robot.patrol)
        self.assertEqual("WARMUP", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))


if __name__ == "__main__":
    unittest.main()
