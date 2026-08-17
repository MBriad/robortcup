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
    PROBE_VISION_CONFIRM_FRAMES,
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


def vision_detection(target_type, offset_x, confidence=0.9):
    return {
        "type": target_type,
        "offset_x": offset_x,
        "bbox": [100, 100, 180, 180],
        "confidence": confidence,
    }


def vision_frame(sequence, *detections, target=None):
    return {
        "sequence": sequence,
        "status": "target" if detections else "no_target",
        "frame_width": 640,
        "frame_height": 480,
        "target": target,
        "detections": list(detections),
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

    def update(self, robot, gray, ir=None, shovel=None, vision=None, now=0.0):
        return robot.update(
            gray,
            ir_states() if ir is None else ir,
            ANALOG_VALID,
            SHOVEL_ON_STAGE if shovel is None else shovel,
            vision=vision,
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

    def confirm_enemy_push(self, robot, start=0.2, trigger_sequence=1,
                           gray=None, shovel=None):
        gray = dict(GRAY_CENTER_REFERENCE) if gray is None else gray
        result = self.update(
            robot, gray, ir=ir_states(front=True), shovel=shovel,
            vision=vision_frame(trigger_sequence), now=start,
        )
        self.assertEqual("PROBE_VISION_WAIT", result["state"])
        for index in range(PROBE_VISION_CONFIRM_FRAMES):
            result = self.update(
                robot, gray, ir=ir_states(front=True), shovel=shovel,
                vision=vision_frame(trigger_sequence + index + 1),
                now=start + 0.02 * (index + 1),
            )
        self.assertEqual("ENEMY_PUSH", result["state"])
        return result

    def test_safe_ground_uses_patrol_command(self):
        robot = RobotController()
        result = self.warm_patrol(robot)
        self.assertEqual("patrol", result["mode"])
        self.assertEqual("MEDIUM_CRUISE", result["state"])
        self.assertGreater(result["left"], 0)
        self.assertGreater(result["right"], 0)
        self.assertFalse(result["shovel_preheat"])

    def test_far_good_preempts_safe_patrol(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.25)
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        self.assertEqual("hunt", result["mode"])
        self.assertEqual("track_good", result["hunt_mode"])
        self.assertEqual("ARC_RIGHT", result["state"])

    def test_medium_confidence_good_stops_then_preempts_on_second_frame(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.25, confidence=0.7)
        first = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        second = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(2, good, target=good), now=0.35,
        )
        self.assertEqual("hunt", first["mode"])
        self.assertEqual("GOOD_ACQUIRE", first["state"])
        self.assertEqual((0, 0), (first["left"], first["right"]))
        self.assertEqual("ARC_RIGHT", second["state"])

    def test_far_bad_does_not_preempt_patrol(self):
        robot = RobotController()
        self.warm_patrol(robot)
        bad = vision_detection("bad", -0.4)
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, bad, target=bad), now=0.2,
        )
        self.assertEqual("patrol", result["mode"])

    def test_front_ir_without_energy_block_starts_vision_wait(self):
        robot = RobotController()
        self.warm_patrol(robot)
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(front=True), vision=vision_frame(1), now=0.2,
        )
        self.assertEqual("proximity_probe", result["mode"])
        self.assertEqual("PROBE_VISION_WAIT", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))
        self.assertEqual(0, result["probe_vision_count"])

    def test_front_ir_stops_while_vision_is_unavailable(self):
        robot = RobotController()
        self.warm_patrol(robot)
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(front=True), vision=None, now=0.2,
        )
        self.assertEqual("proximity_probe", result["mode"])
        self.assertEqual("PROBE_VISION_WAIT", result["probe_state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_stale_vision_does_not_preempt_front_ir_candidate(self):
        robot = RobotController()
        self.warm_patrol(robot)
        stale_good = vision_detection("good", 0.25)
        stale = vision_frame(1, stale_good, target=stale_good)
        stale["status"] = "stale"
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(front=True), vision=stale, now=0.2,
        )
        self.assertEqual("proximity_probe", result["mode"])
        self.assertEqual("PROBE_VISION_WAIT", result["state"])
        self.assertFalse(result["vision_has_good"])

    def test_enemy_push_requires_three_new_no_target_frames(self):
        robot = RobotController()
        self.warm_patrol(robot)
        result = self.confirm_enemy_push(robot)
        self.assertEqual("enemy_push", result["mode"])
        self.assertEqual("enemy_confirmed", result["probe_vision_verdict"])

    def test_proximity_probe_is_not_named_enemy_before_confirmation(self):
        robot = RobotController()
        self.warm_patrol(robot)
        candidate = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(front=True), vision=vision_frame(1), now=0.2,
        )
        self.assertEqual("proximity_probe", candidate["mode"])
        self.assertEqual("PROBE_VISION_WAIT", candidate["state"])

        confirmed = candidate
        for sequence in range(2, 2 + PROBE_VISION_CONFIRM_FRAMES):
            confirmed = self.update(
                robot, dict(GRAY_CENTER_REFERENCE),
                ir=ir_states(front=True), vision=vision_frame(sequence),
                now=0.2 + sequence * 0.02,
            )
        self.assertEqual("enemy_push", confirmed["mode"])
        self.assertEqual("ENEMY_PUSH", confirmed["state"])

    def test_visual_target_preempts_active_probe_wait(self):
        for target_type, expected_state in (
                ("good", "ARC_RIGHT"), ("bad", "BAD_CONFIRM")):
            with self.subTest(target_type=target_type):
                robot = RobotController()
                self.warm_patrol(robot)
                self.update(
                    robot, dict(GRAY_CENTER_REFERENCE),
                    ir=ir_states(front=True), vision=vision_frame(1), now=0.2,
                )
                target = vision_detection(target_type, 0.25)
                result = self.update(
                    robot, dict(GRAY_CENTER_REFERENCE),
                    ir=ir_states(front=True),
                    vision=vision_frame(2, target, target=target), now=0.22,
                )
                self.assertEqual("hunt", result["mode"])
                self.assertEqual(expected_state, result["state"])
                self.assertEqual("IDLE", result["probe_state"])

    def test_good_preempts_active_probe_turn(self):
        robot = RobotController()
        self.warm_patrol(robot)
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(left_rear=True), vision=vision_frame(1), now=0.2,
        )
        good = vision_detection("good", -0.25)
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(left_rear=True),
            vision=vision_frame(2, good, target=good), now=0.22,
        )
        self.assertEqual("hunt", result["mode"])
        self.assertEqual("ARC_LEFT", result["state"])
        self.assertEqual("IDLE", result["probe_state"])

    def test_good_tracking_preempts_probe_front_ir(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.25)
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(front=True),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        self.assertEqual("hunt", result["mode"])
        self.assertEqual("track_good", result["hunt_mode"])
        self.assertEqual("IDLE", result["probe_state"])

    def test_good_tracking_does_not_activate_shovel_guard(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.25)
        now = 0.2
        result = None
        for index in range(SHOVEL_FILTER_WINDOW):
            result = self.update(
                robot, dict(GRAY_CENTER_REFERENCE),
                shovel=SHOVEL_HANGING,
                vision=vision_frame(index + 1, good, target=good),
                now=now,
            )
            now += 0.02
        self.assertEqual("hunt", result["mode"])
        self.assertEqual("ARC_RIGHT", result["state"])
        self.assertEqual("IDLE", result["shovel_state"])
        self.assertFalse(result["shovel_active"])

    def test_good_push_ignores_front_side_edge_until_shovel_hangs(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.0)
        confirming = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        started = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(2, good, target=good), now=0.22,
        )
        self.assertEqual("GOOD_CONFIRM", confirming["state"])
        self.assertEqual("GOOD_PUSH", started["state"])

        result = None
        for index in range(10):
            result = self.update(
                robot, FALLEN_GRAY, shovel=SHOVEL_ON_STAGE,
                vision=vision_frame(index + 3),
                now=0.24 + index * 0.02,
            )
        self.assertEqual("hunt", result["mode"])
        self.assertEqual("GOOD_PUSH", result["state"])
        self.assertGreater(result["left"], 0)
        self.assertGreater(result["right"], 0)

    def test_good_push_shovel_hang_is_owned_by_main(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.0)
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(2, good, target=good), now=0.22,
        )

        result = None
        for index in range(SHOVEL_FILTER_WINDOW):
            result = self.update(
                robot, dict(GRAY_CENTER_REFERENCE), shovel=SHOVEL_HANGING,
                vision=vision_frame(index + 3), now=0.24 + index * 0.02,
            )

        self.assertEqual("shovel_guard", result["mode"])
        self.assertEqual("HANGED", result["state"])
        self.assertEqual("GOOD_PUSH", result["hunt_state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_transient_shovel_hang_resumes_good_push(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.0)
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(2, good, target=good), now=0.22,
        )

        samples = [SHOVEL_HANGING] * 5 + [SHOVEL_ON_STAGE] * 4
        result = None
        for index, shovel in enumerate(samples):
            result = self.update(
                robot, dict(GRAY_CENTER_REFERENCE), shovel=shovel,
                vision=vision_frame(index + 3, good, target=good),
                now=0.24 + index * 0.02,
            )
        self.assertEqual("HANGED", result["state"])

        resumed = self.update(
            robot, dict(GRAY_CENTER_REFERENCE), shovel=SHOVEL_ON_STAGE,
            vision=vision_frame(12, good, target=good), now=0.42,
        )
        self.assertEqual("hunt", resumed["mode"])
        self.assertEqual("GOOD_PUSH", resumed["state"])
        self.assertGreater(resumed["left"], 0)
        self.assertGreater(resumed["right"], 0)

    def test_confirmed_shovel_hang_finishes_good_push_on_reverse(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.0)
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(2, good, target=good), now=0.22,
        )

        result = None
        total = SHOVEL_FILTER_WINDOW + SHOVEL_HANG_CONFIRM - 1
        for index in range(total):
            result = self.update(
                robot, dict(GRAY_CENTER_REFERENCE), shovel=SHOVEL_HANGING,
                vision=vision_frame(index + 3, good, target=good),
                now=0.24 + index * 0.02,
            )
        self.assertEqual("REVERSE", result["state"])
        self.assertEqual("IDLE", result["hunt_state"])

    def test_invalid_shovel_during_good_push_cancels_to_patrol(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.0)
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(2, good, target=good), now=0.22,
        )
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            shovel={"left": 0.0, "right": 0.0, "valid": False},
            vision=vision_frame(3, good, target=good), now=0.24,
        )
        self.assertEqual("patrol", result["mode"])
        self.assertEqual("WARMUP", result["state"])
        self.assertEqual("IDLE", result["hunt_state"])
        self.assertFalse(result["shovel_active"])

    def test_left_rear_ir_starts_probe_turn_when_camera_idle(self):
        robot = RobotController()
        self.warm_patrol(robot)
        started = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(left_rear=True), vision=vision_frame(1), now=0.2,
        )
        turning = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(left_rear=True), vision=vision_frame(1), now=0.22,
        )
        speed = MOTOR_TURN_CALIBRATION["left"][135.0][0]
        self.assertEqual("proximity_probe", started["mode"])
        self.assertEqual("PROBE_TURN", started["state"])
        self.assertEqual((-speed, speed), (turning["left"], turning["right"]))

    def test_far_bad_does_not_block_unrelated_ir_probe_turn(self):
        robot = RobotController()
        self.warm_patrol(robot)
        far_bad_on_left = vision_detection("bad", -0.7)
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(right_front=True),
            vision=vision_frame(1, far_bad_on_left, target=far_bad_on_left),
            now=0.2,
        )
        self.assertEqual("proximity_probe", result["mode"])
        self.assertEqual("PROBE_TURN", result["state"])
        self.assertEqual("right_front", result["source_direction"])

    def test_enemy_push_ignores_front_side_edge_until_shovel_hangs(self):
        robot = RobotController()
        self.warm_patrol(robot)
        self.confirm_enemy_push(robot)

        edge = gray_at_zone(
            front=0.0,
            rear=1.0,
            left=0.0,
            right=1.0,
        )
        result = None
        for index in range(10):
            result = self.update(
                robot, edge, ir=ir_states(front=True),
                shovel=SHOVEL_ON_STAGE, vision=vision_frame(index + 5),
                now=0.28 + index * 0.02,
            )
        self.assertEqual("enemy_push", result["mode"])
        self.assertEqual("ENEMY_PUSH", result["state"])
        self.assertGreater(result["left"], 0)
        self.assertGreater(result["right"], 0)

    def test_near_bad_preempts_after_two_new_frames(self):
        robot = RobotController()
        self.warm_patrol(robot)
        bad = vision_detection("bad", -0.4)
        near_left = ir_states(left_front=True)
        first = self.update(
            robot, dict(GRAY_CENTER_REFERENCE), ir=near_left,
            vision=vision_frame(1, bad), now=0.2,
        )
        second = self.update(
            robot, dict(GRAY_CENTER_REFERENCE), ir=near_left,
            vision=vision_frame(2, bad), now=0.35,
        )
        self.assertEqual("hunt", first["mode"])
        self.assertEqual("BAD_CONFIRM", first["state"])
        self.assertEqual("hunt", second["mode"])
        self.assertEqual("AVOID_TURN", second["state"])

    def test_gray_edge_response_preempts_good_tracking(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.25)
        edge = gray_at_zone(
            front=0.0,
            rear=1.0,
            left=0.0,
            right=1.0,
        )
        result = None
        for index in range(10):
            result = self.update(
                robot, edge,
                vision=vision_frame(index + 1, good, target=good),
                now=0.2 + index * 0.02,
            )
        self.assertEqual("patrol", result["mode"])
        self.assertEqual("EDGE_AVOID", result["state"])

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

    def test_patrol_preheat_does_not_activate_shovel_guard(self):
        robot = RobotController()
        edge = gray_at_zone(
            front=PATROL_SHOVEL_PREHEAT_FRONT_ZONE - 0.05,
            rear=1.0,
            left=1.0,
            right=1.0,
        )
        now = 0.0
        result = None
        for _ in range(SHOVEL_FILTER_WINDOW + SHOVEL_HANG_CONFIRM + 2):
            result = self.update(
                robot, edge, shovel=SHOVEL_HANGING, now=now,
            )
            now += 0.02
        self.assertEqual("patrol", result["mode"])
        self.assertTrue(result["shovel_preheat"])
        self.assertEqual("IDLE", result["shovel_state"])
        self.assertFalse(result["shovel_active"])

    def test_probe_turn_does_not_activate_shovel_guard(self):
        robot = RobotController()
        self.warm_patrol(robot)
        result = None
        for index in range(SHOVEL_FILTER_WINDOW):
            result = self.update(
                robot, dict(GRAY_CENTER_REFERENCE),
                ir=ir_states(left_front=True), shovel=SHOVEL_HANGING,
                vision=vision_frame(index + 1), now=0.2 + index * 0.02,
            )
        self.assertEqual("proximity_probe", result["mode"])
        self.assertEqual("PROBE_TURN", result["state"])
        self.assertEqual("IDLE", result["shovel_state"])
        self.assertFalse(result["shovel_active"])

    def test_enemy_push_shovel_hang_is_owned_by_main(self):
        robot = RobotController()
        self.warm_patrol(robot)
        self.confirm_enemy_push(robot)
        result = None
        for index in range(SHOVEL_FILTER_WINDOW):
            result = self.update(
                robot, dict(GRAY_CENTER_REFERENCE), ir=ir_states(front=True),
                shovel=SHOVEL_HANGING, vision=vision_frame(index + 5),
                now=0.28 + index * 0.02,
            )
        self.assertEqual("shovel_guard", result["mode"])
        self.assertEqual("HANGED", result["state"])
        self.assertEqual("ENEMY_PUSH", result["probe_state"])

    def test_visual_target_preempts_confirmed_enemy_push(self):
        for target_type, expected_state in (
                ("good", "ARC_RIGHT"), ("bad", "BAD_CONFIRM")):
            with self.subTest(target_type=target_type):
                robot = RobotController()
                self.warm_patrol(robot)
                self.confirm_enemy_push(robot)
                target = vision_detection(target_type, 0.25)
                result = self.update(
                    robot, dict(GRAY_CENTER_REFERENCE),
                    ir=ir_states(front=True),
                    shovel=SHOVEL_ON_STAGE,
                    vision=vision_frame(10, target, target=target),
                    now=0.4,
                )
                self.assertEqual("hunt", result["mode"])
                self.assertEqual(expected_state, result["state"])
                self.assertEqual("IDLE", result["probe_state"])

    def test_hunt_release_recreates_patrol(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.25)
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        previous_patrol = robot.patrol
        first_missing = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(2), now=0.22,
        )
        second_missing = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(3), now=0.24,
        )
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(4), now=0.26,
        )
        self.assertEqual("GOOD_LOST_HOLD", first_missing["state"])
        self.assertEqual("GOOD_LOST_HOLD", second_missing["state"])
        self.assertEqual("patrol", result["mode"])
        self.assertIsNot(previous_patrol, robot.patrol)
        self.assertEqual("WARMUP", result["state"])

    def test_good_loss_with_front_ir_waits_before_probe_confirmation(self):
        robot = RobotController()
        self.warm_patrol(robot)
        good = vision_detection("good", 0.25)
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            vision=vision_frame(1, good, target=good), now=0.2,
        )
        first = self.update(
            robot, dict(GRAY_CENTER_REFERENCE), ir=ir_states(front=True),
            vision=vision_frame(2), now=0.3,
        )
        second = self.update(
            robot, dict(GRAY_CENTER_REFERENCE), ir=ir_states(front=True),
            vision=vision_frame(3), now=0.4,
        )
        released = self.update(
            robot, dict(GRAY_CENTER_REFERENCE), ir=ir_states(front=True),
            vision=vision_frame(4), now=0.5,
        )
        self.assertEqual("GOOD_LOST_HOLD", first["state"])
        self.assertEqual("GOOD_LOST_HOLD", second["state"])
        self.assertEqual("IDLE", second["probe_state"])
        self.assertEqual("proximity_probe", released["mode"])
        self.assertEqual("PROBE_VISION_WAIT", released["state"])

    def test_probe_turn_completion_waits_for_vision(self):
        robot = RobotController()
        self.warm_patrol(robot)
        self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(left_front=True), vision=vision_frame(1), now=1.0,
        )
        duration = MOTOR_TURN_CALIBRATION["left"][45.0][1]
        result = self.update(
            robot, dict(GRAY_CENTER_REFERENCE),
            ir=ir_states(), vision=vision_frame(2),
            now=1.0 + duration,
        )
        self.assertEqual("proximity_probe", result["mode"])
        self.assertEqual("PROBE_VISION_WAIT", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))


if __name__ == "__main__":
    unittest.main()
