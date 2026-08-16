#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from config import (
    ENEMY_PUSH_SPEED,
    ENEMY_SLOW_CONFIRM,
    ENEMY_SLOW_SPEED,
    MOTOR_TURN_CALIBRATION,
    SHOVEL_FILTER_WINDOW,
)
from enemy_push import EnemyPushController


def ir(**active):
    states = {
        "left_rear": False,
        "left_front": False,
        "right_rear": False,
        "right_front": False,
        "rear": False,
        "front": False,
        "valid": True,
    }
    states.update(active)
    return states


def observation(front=1.0, rear=1.0, left=1.0, right=1.0):
    return {"zone": {
        "front": front,
        "rear": rear,
        "left": left,
        "right": right,
    }}


ON_STAGE = {"left": 20.0, "right": 20.0, "valid": True}
HANGING = {"left": 1600.0, "right": 1600.0, "valid": True}


class EnemyPushControllerTest(unittest.TestCase):
    def test_camera_busy_blocks_new_enemy_action(self):
        result = EnemyPushController().update(
            ir(front=True), observation(), ON_STAGE,
            now=0.0, allow_start=False,
        )
        self.assertFalse(result["owns_control"])
        self.assertEqual("IDLE", result["state"])

    def test_front_target_starts_push(self):
        controller = EnemyPushController()
        started = controller.update(
            ir(front=True), observation(), ON_STAGE, now=0.0,
        )
        pushing = controller.update(
            ir(front=True), observation(), ON_STAGE, now=0.02,
        )
        self.assertEqual("PUSH", started["state"])
        self.assertEqual((0, 0), (started["left"], started["right"]))
        self.assertEqual(
            (ENEMY_PUSH_SPEED, ENEMY_PUSH_SPEED),
            (pushing["left"], pushing["right"]),
        )

    def test_left_front_turns_left_45_degrees(self):
        controller = EnemyPushController()
        controller.update(
            ir(left_front=True), observation(), ON_STAGE, now=1.0,
        )
        result = controller.update(
            ir(left_front=True), observation(), ON_STAGE, now=1.02,
        )
        speed = MOTOR_TURN_CALIBRATION["left"][45.0][0]
        self.assertEqual("SEEK_TURN", result["state"])
        self.assertEqual("left_front", result["source_direction"])
        self.assertEqual((-speed, speed), (result["left"], result["right"]))

    def test_left_rear_turns_left_135_degrees(self):
        controller = EnemyPushController()
        controller.update(
            ir(left_rear=True), observation(), ON_STAGE, now=1.0,
        )
        result = controller.update(
            ir(left_rear=True), observation(), ON_STAGE, now=1.02,
        )
        speed = MOTOR_TURN_CALIBRATION["left"][135.0][0]
        self.assertEqual((-speed, speed), (result["left"], result["right"]))

    def test_turn_plan_can_be_injected_from_config(self):
        controller = EnemyPushController(
            turn_plan={"left_front": ("right", 90.0)},
        )
        controller.update(
            ir(left_front=True), observation(), ON_STAGE, now=1.0,
        )
        result = controller.update(
            ir(left_front=True), observation(), ON_STAGE, now=1.02,
        )
        speed = MOTOR_TURN_CALIBRATION["right"][90.0][0]
        self.assertEqual((speed, -speed), (result["left"], result["right"]))

    def test_slow_zone_latches_350_speed(self):
        controller = EnemyPushController()
        controller.update(ir(front=True), observation(), ON_STAGE, now=0.0)
        result = None
        for index in range(ENEMY_SLOW_CONFIRM):
            result = controller.update(
                ir(front=True), observation(front=1.2), ON_STAGE,
                now=0.02 * (index + 1),
            )
        self.assertTrue(result["slow"])
        self.assertEqual(
            (ENEMY_SLOW_SPEED, ENEMY_SLOW_SPEED),
            (result["left"], result["right"]),
        )

    def test_shovel_hang_stops_push_and_enters_retreat(self):
        controller = EnemyPushController()
        controller.update(ir(front=True), observation(), ON_STAGE, now=0.0)
        result = None
        for index in range(SHOVEL_FILTER_WINDOW):
            result = controller.update(
                ir(front=True), observation(), HANGING,
                now=0.02 * (index + 1),
            )
        self.assertEqual("RETREAT", result["state"])
        self.assertTrue(result["confirmed"])
        self.assertEqual((0, 0), (result["left"], result["right"]))


if __name__ == "__main__":
    unittest.main()
