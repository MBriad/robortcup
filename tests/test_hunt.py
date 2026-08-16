#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from config import MOTOR_TURN_CALIBRATION
from hunt import HuntController


def detection(target_type, offset_x, area=1600.0):
    side = area ** 0.5
    return {
        "type": target_type,
        "offset_x": offset_x,
        "bbox": [0.0, 0.0, side, side],
        "confidence": 0.9,
    }


def frame(sequence, *detections, target=None):
    return {
        "sequence": sequence,
        "status": "target" if detections else "no_target",
        "frame_width": 640,
        "frame_height": 480,
        "target": target,
        "detections": list(detections),
    }


def ir(**active):
    result = {
        "left_rear": False,
        "left_front": False,
        "right_rear": False,
        "right_front": False,
        "rear": False,
        "front": False,
        "valid": True,
    }
    result.update(active)
    return result


class HuntControllerTest(unittest.TestCase):
    def test_far_good_is_tracked(self):
        good = detection("good", 0.25)
        result = HuntController().update(frame(1, good, target=good), ir(), now=0.0)
        self.assertTrue(result["owns_control"])
        self.assertEqual("track_good", result["mode"])
        self.assertEqual("ARC_RIGHT", result["state"])
        self.assertEqual((500, 400), (result["left"], result["right"]))

    def test_far_bad_is_ignored(self):
        bad = detection("bad", -0.4)
        result = HuntController().update(frame(1, bad, target=bad), ir(), now=0.0)
        self.assertFalse(result["owns_control"])
        self.assertEqual("NO_TARGET", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_near_good_is_tracked_while_far_bad_is_ignored(self):
        good = detection("good", -0.45)
        bad = detection("bad", -0.2, area=6400.0)
        result = HuntController().update(
            frame(1, good, bad, target=bad), ir(left_front=True), now=0.0,
        )
        self.assertEqual("track_good", result["mode"])
        self.assertIs(good, result["target"])
        self.assertEqual("left", result["near_direction"])

    def test_near_bad_requires_two_distinct_frames(self):
        controller = HuntController()
        bad = detection("bad", -0.4)
        first = controller.update(
            frame(1, bad), ir(left_front=True), now=0.0,
        )
        repeated = controller.update(
            frame(1, bad), ir(left_front=True), now=0.05,
        )
        second = controller.update(
            frame(2, bad), ir(left_front=True), now=0.15,
        )
        self.assertEqual("BAD_CONFIRM", first["state"])
        self.assertEqual("BAD_CONFIRM", repeated["state"])
        self.assertEqual((0, 0), (first["left"], first["right"]))
        self.assertEqual("AVOID_TURN", second["state"])
        speed = MOTOR_TURN_CALIBRATION["right"][90.0][0]
        self.assertEqual((speed, -speed), (second["left"], second["right"]))

    def test_turn_direction_is_locked_until_calibrated_duration_finishes(self):
        controller = HuntController(confirm_frames=1)
        left_bad = detection("bad", -0.5)
        started = controller.update(
            frame(1, left_bad), ir(left_front=True), now=1.0,
        )
        speed, duration = MOTOR_TURN_CALIBRATION["right"][90.0]
        right_bad = detection("bad", 0.5)
        turning = controller.update(
            frame(2, right_bad), ir(right_front=True),
            now=1.0 + duration - 0.001,
        )
        released = controller.update(
            frame(3, right_bad), ir(right_front=True),
            now=1.0 + duration,
        )
        self.assertEqual((speed, -speed), (started["left"], started["right"]))
        self.assertEqual((speed, -speed), (turning["left"], turning["right"]))
        self.assertEqual("AVOID_RELEASE", released["state"])
        self.assertEqual((0, 0), (released["left"], released["right"]))

    def test_completed_avoidance_releases_without_forward_motion(self):
        controller = HuntController(confirm_frames=1)
        bad = detection("bad", -0.4)
        controller.update(frame(1, bad), ir(left_front=True), now=0.0)
        duration = MOTOR_TURN_CALIBRATION["right"][90.0][1]
        controller.update(frame(2, bad), ir(left_front=True), now=duration)
        result = controller.update(
            frame(3, bad), ir(left_front=True), now=duration + 0.02,
        )
        self.assertFalse(result["owns_control"])
        self.assertEqual("AVOID_DONE", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_centered_bad_turns_toward_visible_good(self):
        controller = HuntController(confirm_frames=1)
        bad = detection("bad", 0.02)
        good = detection("good", 0.6)
        result = controller.update(
            frame(1, bad, good), ir(front=True), now=0.0,
        )
        speed = MOTOR_TURN_CALIBRATION["right"][90.0][0]
        self.assertEqual("right", result["turn_direction"])
        self.assertEqual((speed, -speed), (result["left"], result["right"]))

    def test_near_unknown_releases_to_enemy_module(self):
        result = HuntController().update(frame(1), ir(front=True), now=0.0)
        self.assertFalse(result["owns_control"])
        self.assertEqual("IR_UNCLASSIFIED", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))


if __name__ == "__main__":
    unittest.main()
