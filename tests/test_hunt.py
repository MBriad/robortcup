#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from config import (
    HUNT_GOOD_ACQUIRE_FRAMES,
    HUNT_GOOD_HIGH_CONFIDENCE,
    HUNT_GOOD_LOST_HOLD_FRAMES,
    HUNT_GOOD_LOST_HOLD_SECONDS,
    HUNT_GOOD_MIN_CONFIDENCE,
    HUNT_GOOD_PUSH_SPEED,
    MOTOR_TURN_CALIBRATION,
)
from hunt import HuntController


def detection(target_type, offset_x, area=1600.0, confidence=0.9):
    side = area ** 0.5
    return {
        "type": target_type,
        "offset_x": offset_x,
        "bbox": [0.0, 0.0, side, side],
        "confidence": confidence,
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
    def test_good_acquisition_uses_config_parameters(self):
        controller = HuntController()
        self.assertEqual(HUNT_GOOD_MIN_CONFIDENCE,
                         controller.good_min_confidence)
        self.assertEqual(HUNT_GOOD_HIGH_CONFIDENCE,
                         controller.good_high_confidence)
        self.assertEqual(HUNT_GOOD_ACQUIRE_FRAMES,
                         controller.good_acquire_frames)
        self.assertEqual(HUNT_GOOD_LOST_HOLD_FRAMES,
                         controller.good_lost_hold_frames)
        self.assertEqual(HUNT_GOOD_LOST_HOLD_SECONDS,
                         controller.good_lost_hold_seconds)

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

    def test_medium_confidence_good_stops_until_second_distinct_frame(self):
        controller = HuntController()
        good = detection("good", 0.25, confidence=0.7)
        first = controller.update(frame(1, good), ir(), now=0.0)
        repeated = controller.update(frame(1, good), ir(), now=0.05)
        second = controller.update(frame(2, good), ir(), now=0.15)
        self.assertEqual("GOOD_ACQUIRE", first["state"])
        self.assertEqual("GOOD_ACQUIRE", repeated["state"])
        self.assertEqual((0, 0), (first["left"], first["right"]))
        self.assertEqual("ARC_RIGHT", second["state"])

    def test_low_confidence_good_does_not_interrupt_patrol(self):
        good = detection("good", 0.25, confidence=0.4)
        result = HuntController().update(frame(1, good), ir(), now=0.0)
        self.assertFalse(result["owns_control"])
        self.assertEqual("NO_TARGET", result["state"])

    def test_locked_good_holds_two_missing_frames_then_releases(self):
        controller = HuntController()
        good = detection("good", 0.25)
        controller.update(frame(1, good), ir(), now=0.0)
        first = controller.update(frame(2), ir(front=True), now=0.1)
        second = controller.update(frame(3), ir(front=True), now=0.2)
        released = controller.update(frame(4), ir(front=True), now=0.3)
        self.assertEqual("GOOD_LOST_HOLD", first["state"])
        self.assertEqual("GOOD_LOST_HOLD", second["state"])
        self.assertEqual((0, 0), (second["left"], second["right"]))
        self.assertFalse(released["owns_control"])
        self.assertEqual("IR_UNCLASSIFIED", released["state"])

    def test_locked_good_hold_expires_without_new_vision_frame(self):
        controller = HuntController()
        good = detection("good", 0.25)
        controller.update(frame(1, good), ir(), now=0.0)
        holding = controller.update(None, ir(front=True), now=0.2)
        released = controller.update(None, ir(front=True), now=0.36)
        self.assertEqual("GOOD_LOST_HOLD", holding["state"])
        self.assertFalse(released["owns_control"])
        self.assertEqual("IR_UNCLASSIFIED", released["state"])

    def test_repeated_good_sequence_cannot_refresh_lock_forever(self):
        controller = HuntController()
        good = detection("good", 0.25)
        controller.update(frame(1, good), ir(), now=0.0)
        repeated = controller.update(frame(1, good), ir(), now=0.2)
        expired = controller.update(frame(1, good), ir(), now=0.36)
        self.assertEqual("ARC_RIGHT", repeated["state"])
        self.assertFalse(expired["owns_control"])
        self.assertEqual("NO_TARGET", expired["state"])

    def test_low_confidence_good_does_not_hide_near_bad(self):
        controller = HuntController()
        low_good = detection("good", 0.0, confidence=0.4)
        bad = detection("bad", 0.1)
        result = controller.update(
            frame(1, low_good, bad, target=low_good),
            ir(front=True), now=0.0,
        )
        self.assertEqual("BAD_CONFIRM", result["state"])

    def test_near_bad_overrides_locked_good(self):
        controller = HuntController()
        good = detection("good", 0.25)
        bad = detection("bad", 0.0)
        controller.update(frame(1, good), ir(), now=0.0)
        result = controller.update(frame(2, bad), ir(front=True), now=0.1)
        self.assertEqual("BAD_CONFIRM", result["state"])
        self.assertFalse(result["good_locked"])

    def test_centered_good_latches_push_after_vision_is_lost(self):
        controller = HuntController()
        good = detection("good", 0.0)
        first = controller.update(
            frame(1, good, target=good), ir(), now=0.0,
        )
        repeated = controller.update(
            frame(1, good, target=good), ir(), now=0.05,
        )
        started = controller.update(
            frame(2, good, target=good), ir(), now=0.1,
        )
        continued = controller.update(frame(3), ir(), now=0.2)
        self.assertEqual("GOOD_CONFIRM", first["state"])
        self.assertEqual("GOOD_CONFIRM", repeated["state"])
        self.assertEqual((0, 0), (first["left"], first["right"]))
        self.assertEqual("GOOD_PUSH", started["state"])
        self.assertEqual("push_good", started["mode"])
        self.assertEqual("GOOD_PUSH", continued["state"])
        self.assertEqual(
            (HUNT_GOOD_PUSH_SPEED, HUNT_GOOD_PUSH_SPEED),
            (continued["left"], continued["right"]),
        )

    def test_finished_good_push_rearms_only_after_target_disappears(self):
        controller = HuntController()
        good = detection("good", 0.0)
        controller.update(frame(1, good, target=good), ir(), now=0.0)
        controller.update(frame(2, good, target=good), ir(), now=0.1)
        controller.finish_push()

        blocked = controller.update(
            frame(3, good, target=good), ir(), now=0.2,
        )
        disappeared = controller.update(frame(4), ir(), now=0.3)
        confirming = controller.update(
            frame(5, good, target=good), ir(), now=0.4,
        )

        self.assertFalse(blocked["owns_control"])
        self.assertEqual("GOOD_REARM_WAIT", blocked["state"])
        self.assertEqual("NO_TARGET", disappeared["state"])
        self.assertEqual("GOOD_CONFIRM", confirming["state"])

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
