#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import (
    DIGI_IR_PINS,
    GRAY_CENTER_REFERENCE,
    GRAY_EDGE_REFERENCE,
    GRAY_FILTER_WINDOW,
    IR_ALIGNMENT_DIFF_HIGH,
    IR_ALIGNMENT_DIFF_LOW,
    IR_ALIGNMENT_FILTER_WINDOW,
    IR_ALIGNMENT_SIGNAL_MIN,
    MOTOR_TURN_CALIBRATION,
    PATROL_SHOVEL_PREHEAT_FRONT_ZONE,
    SHOVEL_FILTER_WINDOW,
    SHOVEL_HANG_CONFIRM,
    SHOVEL_REVERSE_MIN_SECONDS,
    SHOVEL_REVERSE_SPEED,
)
from main import RobotController


def ir_states(**active):
    states = {name: False for name in DIGI_IR_PINS}
    states.update(active)
    states["valid"] = True
    return states


ANALOG_VALID = {"left": 500.0, "right": 500.0, "valid": True}
SHOVEL_ON_STAGE = {"left": 20.0, "right": 20.0, "valid": True}
SHOVEL_HANGING = {"left": 1600.0, "right": 1600.0, "valid": True}
FALLEN_GRAY = {
    name: max(0.0, value - 100.0)
    for name, value in GRAY_EDGE_REFERENCE.items()
}


def gray_at_zone(**scores):
    return {
        name: (
            GRAY_EDGE_REFERENCE[name]
            + scores[name] * (GRAY_CENTER_REFERENCE[name] - GRAY_EDGE_REFERENCE[name])
        )
        for name in GRAY_CENTER_REFERENCE
    }


class RobotControllerTest(unittest.TestCase):
    def test_runtime_controllers_use_config_sensor_parameters(self):
        robot = RobotController()
        self.assertEqual(GRAY_FILTER_WINDOW, robot.patrol.model.window)
        self.assertEqual(GRAY_FILTER_WINDOW, robot.reentry.model.window)
        self.assertEqual(SHOVEL_FILTER_WINDOW, robot.shovel_guard.window)
        self.assertEqual(
            IR_ALIGNMENT_FILTER_WINDOW, robot.reentry._alignment.window,
        )
        self.assertEqual(IR_ALIGNMENT_DIFF_LOW, robot.reentry._alignment.diff_low)
        self.assertEqual(IR_ALIGNMENT_DIFF_HIGH, robot.reentry._alignment.diff_high)
        self.assertEqual(
            IR_ALIGNMENT_SIGNAL_MIN, robot.reentry._alignment.signal_min,
        )

    def update(self, robot, gray, ir=None, shovel=None, now=0.0):
        return robot.update(
            gray,
            ir_states() if ir is None else ir,
            ANALOG_VALID,
            SHOVEL_ON_STAGE if shovel is None else shovel,
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
        self.assertEqual("MEDIUM_CRUISE", result["state"])
        self.assertGreater(result["left"], 0)
        self.assertGreater(result["right"], 0)
        self.assertFalse(result["shovel_preheat"])

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
        speed = MOTOR_TURN_CALIBRATION["right"][90.0][0]
        self.assertEqual((speed, -speed), (result["left"], result["right"]))

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

    def test_preheated_shovel_guard_stops_then_reverses(self):
        robot = RobotController()
        edge = gray_at_zone(
            front=PATROL_SHOVEL_PREHEAT_FRONT_ZONE - 0.05,
            rear=1.0,
            left=1.0,
            right=1.0,
        )
        now = 0.0
        result = None
        for _ in range(SHOVEL_FILTER_WINDOW + 2):
            result = self.update(
                robot, edge, shovel=SHOVEL_ON_STAGE, now=now,
            )
            now += 0.02
        self.assertEqual("patrol", result["mode"])
        self.assertTrue(result["shovel_preheat"])
        self.assertEqual("IDLE", result["shovel_state"])

        for _ in range(SHOVEL_FILTER_WINDOW // 2 + 1):
            result = self.update(
                robot, edge, shovel=SHOVEL_HANGING, now=now,
            )
            now += 0.02
        self.assertEqual("shovel_guard", result["mode"])
        self.assertEqual("HANGED", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

        for _ in range(SHOVEL_HANG_CONFIRM - 1):
            result = self.update(
                robot, edge, shovel=SHOVEL_HANGING, now=now,
            )
            now += 0.02
        self.assertEqual("REVERSE", result["state"])
        self.assertEqual(
            (-SHOVEL_REVERSE_SPEED, -SHOVEL_REVERSE_SPEED),
            (result["left"], result["right"]),
        )

        previous_patrol = robot.patrol
        now += SHOVEL_REVERSE_MIN_SECONDS
        for _ in range(SHOVEL_FILTER_WINDOW // 2 + 1):
            result = self.update(
                robot, edge, shovel=SHOVEL_ON_STAGE, now=now,
            )
            now += 0.02
        self.assertEqual("patrol", result["mode"])
        self.assertIsNot(previous_patrol, robot.patrol)
        self.assertEqual("WARMUP", result["state"])


if __name__ == "__main__":
    unittest.main()
