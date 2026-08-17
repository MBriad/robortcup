#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from config import (
    PROBE_IR_REARM_CLEAR_FRAMES,
    ENEMY_PUSH_SPEED,
    ENEMY_SLOW_CONFIRM,
    ENEMY_SLOW_SPEED,
    PROBE_VISION_CONFIRM_FRAMES,
    PROBE_VISION_WAIT_TIMEOUT,
    MOTOR_TURN_CALIBRATION,
)
from proximity_probe import ProximityProbeController


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


def vision(sequence, status="no_target", good=False, bad=False):
    return {
        "sequence": sequence,
        "status": status,
        "has_good": good,
        "has_bad": bad,
    }


class ProximityProbeControllerTest(unittest.TestCase):
    def confirm_push(self, controller, start=0.0, trigger_sequence=1):
        result = controller.update(
            ir(front=True), observation(),
            vision=vision(trigger_sequence), now=start,
        )
        self.assertEqual("PROBE_VISION_WAIT", result["state"])
        for index in range(PROBE_VISION_CONFIRM_FRAMES):
            result = controller.update(
                ir(front=True), observation(),
                vision=vision(trigger_sequence + index + 1),
                now=start + 0.02 * (index + 1),
            )
        self.assertEqual("ENEMY_PUSH", result["state"])
        return result

    def test_camera_busy_blocks_new_probe_action(self):
        result = ProximityProbeController().update(
            ir(front=True), observation(),
            now=0.0, allow_start=False,
        )
        self.assertFalse(result["owns_control"])
        self.assertEqual("IDLE", result["state"])

    def test_front_target_stops_and_trigger_frame_does_not_count(self):
        controller = ProximityProbeController()
        started = controller.update(
            ir(front=True), observation(), vision=vision(7), now=0.0,
        )
        repeated = controller.update(
            ir(front=True), observation(), vision=vision(7), now=0.02,
        )
        self.assertEqual("PROBE_VISION_WAIT", started["state"])
        self.assertEqual((0, 0), (started["left"], started["right"]))
        self.assertEqual(0, started["vision_count"])
        self.assertEqual(0, repeated["vision_count"])

    def test_three_distinct_no_target_frames_start_push(self):
        controller = ProximityProbeController()
        started = self.confirm_push(controller)
        pushing = controller.update(
            ir(front=True), observation(), vision=vision(5), now=0.08,
        )
        self.assertEqual("enemy_confirmed", started["vision_verdict"])
        self.assertEqual(
            (ENEMY_PUSH_SPEED, ENEMY_PUSH_SPEED),
            (pushing["left"], pushing["right"]),
        )

    def test_repeated_sequence_is_not_counted_twice(self):
        controller = ProximityProbeController()
        controller.update(
            ir(front=True), observation(), vision=vision(1), now=0.0,
        )
        first = controller.update(
            ir(front=True), observation(), vision=vision(2), now=0.02,
        )
        repeated = controller.update(
            ir(front=True), observation(), vision=vision(2), now=0.04,
        )
        self.assertEqual(1, first["vision_count"])
        self.assertEqual(1, repeated["vision_count"])
        self.assertEqual("PROBE_VISION_WAIT", repeated["state"])

    def test_front_ir_loss_cancels_candidate(self):
        controller = ProximityProbeController()
        controller.update(
            ir(front=True), observation(), vision=vision(1), now=0.0,
        )
        result = controller.update(
            ir(), observation(), vision=vision(2), now=0.02,
        )
        self.assertFalse(result["owns_control"])
        self.assertEqual("IDLE", result["state"])
        self.assertEqual("target_lost", result["vision_verdict"])

    def test_good_or_bad_vetoes_enemy_attack(self):
        for target_type in ("good", "bad"):
            with self.subTest(target_type=target_type):
                controller = ProximityProbeController()
                controller.update(
                    ir(front=True), observation(), vision=vision(1), now=0.0,
                )
                result = controller.update(
                    ir(front=True), observation(),
                    vision=vision(2, good=target_type == "good",
                                  bad=target_type == "bad"),
                    now=0.02,
                )
                self.assertFalse(result["owns_control"])
                self.assertEqual("IDLE", result["state"])
                self.assertEqual(target_type, result["vision_verdict"])

    def test_stale_vision_times_out_and_requires_front_clear(self):
        controller = ProximityProbeController()
        controller.update(
            ir(front=True), observation(), vision=vision(1), now=0.0,
        )
        timed_out = controller.update(
            ir(front=True), observation(),
            vision=vision(2, status="stale"),
            now=PROBE_VISION_WAIT_TIMEOUT,
        )
        blocked = controller.update(
            ir(front=True), observation(), vision=vision(3), now=0.7,
        )
        for index in range(PROBE_IR_REARM_CLEAR_FRAMES):
            controller.update(
                ir(), observation(), vision=vision(4 + index),
                now=0.72 + 0.02 * index,
            )
        restarted = controller.update(
            ir(front=True), observation(), vision=vision(8), now=0.8,
        )
        self.assertEqual("timeout", timed_out["vision_verdict"])
        self.assertFalse(blocked["owns_control"])
        self.assertEqual("PROBE_VISION_WAIT", restarted["state"])

    def test_left_front_turns_left_45_degrees(self):
        controller = ProximityProbeController()
        controller.update(
            ir(left_front=True), observation(), now=1.0,
        )
        result = controller.update(
            ir(left_front=True), observation(), now=1.02,
        )
        speed = MOTOR_TURN_CALIBRATION["left"][45.0][0]
        self.assertEqual("PROBE_TURN", result["state"])
        self.assertEqual("left_front", result["source_direction"])
        self.assertEqual((-speed, speed), (result["left"], result["right"]))

    def test_left_rear_turns_left_135_degrees(self):
        controller = ProximityProbeController()
        controller.update(
            ir(left_rear=True), observation(), now=1.0,
        )
        result = controller.update(
            ir(left_rear=True), observation(), now=1.02,
        )
        speed = MOTOR_TURN_CALIBRATION["left"][135.0][0]
        self.assertEqual((-speed, speed), (result["left"], result["right"]))

    def test_far_bad_does_not_cancel_side_probe_turn(self):
        controller = ProximityProbeController()
        controller.update(
            ir(right_front=True), observation(),
            vision=vision(1, status="target", bad=True), now=1.0,
        )
        result = controller.update(
            ir(right_front=True), observation(),
            vision=vision(2, status="target", bad=True), now=1.02,
        )
        speed = MOTOR_TURN_CALIBRATION["right"][45.0][0]
        self.assertTrue(result["owns_control"])
        self.assertEqual("PROBE_TURN", result["state"])
        self.assertEqual((speed, -speed), (result["left"], result["right"]))

    def test_turn_completion_stops_for_post_turn_vision(self):
        controller = ProximityProbeController()
        controller.update(
            ir(left_front=True), observation(), vision=vision(1), now=1.0,
        )
        duration = MOTOR_TURN_CALIBRATION["left"][45.0][1]
        result = controller.update(
            ir(), observation(), vision=vision(1),
            now=1.0 + duration,
        )
        self.assertEqual("PROBE_VISION_WAIT", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_post_turn_no_front_releases_on_new_no_target_frame(self):
        controller = ProximityProbeController()
        controller.update(
            ir(left_front=True), observation(), vision=vision(1), now=1.0,
        )
        duration = MOTOR_TURN_CALIBRATION["left"][45.0][1]
        controller.update(
            ir(), observation(), vision=vision(1),
            now=1.0 + duration,
        )
        result = controller.update(
            ir(), observation(), vision=vision(2),
            now=1.02 + duration,
        )
        self.assertFalse(result["owns_control"])
        self.assertEqual("post_turn_no_front", result["vision_verdict"])

    def test_turn_plan_can_be_injected_from_config(self):
        controller = ProximityProbeController(
            turn_plan={"left_front": ("right", 90.0)},
        )
        controller.update(
            ir(left_front=True), observation(), now=1.0,
        )
        result = controller.update(
            ir(left_front=True), observation(), now=1.02,
        )
        speed = MOTOR_TURN_CALIBRATION["right"][90.0][0]
        self.assertEqual((speed, -speed), (result["left"], result["right"]))

    def test_slow_zone_latches_350_speed(self):
        controller = ProximityProbeController()
        self.confirm_push(controller)
        result = None
        for index in range(ENEMY_SLOW_CONFIRM):
            result = controller.update(
                ir(front=True), observation(front=1.2),
                vision=vision(10 + index), now=0.1 + 0.02 * index,
            )
        self.assertTrue(result["slow"])
        self.assertEqual(
            (ENEMY_SLOW_SPEED, ENEMY_SLOW_SPEED),
            (result["left"], result["right"]),
        )

    def test_front_and_side_edge_do_not_end_push_before_shovel(self):
        controller = ProximityProbeController()
        self.confirm_push(controller)
        result = None
        for index in range(10):
            result = controller.update(
                ir(front=True),
                observation(front=1.5, left=1.5, right=1.5),
                vision=vision(10 + index), now=0.1 + 0.02 * index,
            )
        self.assertEqual("ENEMY_PUSH", result["state"])
        self.assertTrue(result["owns_control"])

    def test_good_immediately_vetoes_confirmed_push(self):
        controller = ProximityProbeController()
        self.confirm_push(controller)
        result = controller.update(
            ir(front=True), observation(),
            vision=vision(5, good=True), now=0.1,
        )
        self.assertFalse(result["owns_control"])
        self.assertEqual("IDLE", result["state"])
        self.assertEqual("good", result["vision_verdict"])

    def test_two_distinct_bad_frames_veto_confirmed_push(self):
        controller = ProximityProbeController()
        self.confirm_push(controller)
        first = controller.update(
            ir(front=True), observation(), vision=vision(5, bad=True), now=0.1,
        )
        repeated = controller.update(
            ir(front=True), observation(), vision=vision(5, bad=True), now=0.12,
        )
        confirmed = controller.update(
            ir(front=True), observation(), vision=vision(6, bad=True), now=0.14,
        )
        self.assertEqual("ENEMY_PUSH", first["state"])
        self.assertEqual("ENEMY_PUSH", repeated["state"])
        self.assertEqual(1, first["bad_interrupt_count"])
        self.assertEqual(1, repeated["bad_interrupt_count"])
        self.assertFalse(confirmed["owns_control"])
        self.assertEqual("IDLE", confirmed["state"])
        self.assertEqual(2, confirmed["bad_interrupt_count"])
        self.assertEqual("bad_confirmed", confirmed["vision_verdict"])

    def test_finished_push_waits_for_front_target_to_leave(self):
        controller = ProximityProbeController()
        self.confirm_push(controller)
        controller.finish_push()

        blocked = controller.update(
            ir(front=True), observation(), vision=vision(5), now=0.1,
        )
        cleared = None
        for index in range(PROBE_IR_REARM_CLEAR_FRAMES):
            cleared = controller.update(
                ir(), observation(), vision=vision(6 + index),
                now=0.2 + 0.02 * index,
            )
        restarted = controller.update(
            ir(front=True), observation(), vision=vision(10), now=0.3,
        )

        self.assertFalse(blocked["owns_control"])
        self.assertEqual("IDLE", blocked["state"])
        self.assertFalse(cleared["owns_control"])
        self.assertEqual("PROBE_VISION_WAIT", restarted["state"])


if __name__ == "__main__":
    unittest.main()
