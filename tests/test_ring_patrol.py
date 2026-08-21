#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import glob
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import (
    GRAY_CENTER_REFERENCE,
    GRAY_CHANNELS,
    GRAY_EDGE_REFERENCE,
    GRAY_WHITE_REFERENCE,
    MOTOR_TURN_CALIBRATION,
    PATROL_COMMAND_LIMIT,
    PATROL_CRUISE_LINEAR,
    PATROL_DIAGONAL_CONFIRM,
    PATROL_DIAGONAL_SIDE_ZONE,
    PATROL_DIAGONAL_TURN_DELTA,
    PATROL_EDGE_RETREAT_SECONDS,
    PATROL_EDGE_TURN_ANGLE,
    PATROL_MEDIUM_LINEAR,
    PATROL_MIN_ACTIVE_SPEED,
    PATROL_RECOVER_BACKWARD_SPEED,
    PATROL_RECOVER_FORWARD_SPEED,
    PATROL_RECOVER_SECONDS,
    PATROL_SHOVEL_PREHEAT_FRONT_ZONE,
)
from gray import GrayRiskModel
from ring_patrol import RingPatrolController


DATA_DIR = os.path.join(ROOT, "data")
DIAGONAL_WHITE_EDGE_LOG = "patrol_20260816_091726.csv"
MANUAL_DIAGONAL_TRIAL_LOG = "patrol_20260816_093316.csv"
SHOVEL_FRONT_EDGE_LOG = "边缘容易掉台.csv"
SHOVEL_PREHEAT_EDGE_LOG = "边缘希望激活铲子.csv"


def raw_at_zone(score):
    return {
        name: (
            GRAY_EDGE_REFERENCE[name]
            + score * (GRAY_CENTER_REFERENCE[name] - GRAY_EDGE_REFERENCE[name])
        )
        for name in GRAY_CENTER_REFERENCE
    }


def raw_at_zone_components(**scores):
    return {
        name: (
            GRAY_EDGE_REFERENCE[name]
            + scores[name]
            * (GRAY_CENTER_REFERENCE[name] - GRAY_EDGE_REFERENCE[name])
        )
        for name in GRAY_CENTER_REFERENCE
    }


def feed(controller, raw, count, start=0.0, step=0.02, healthy=True):
    result = None
    for index in range(count):
        result = controller.update(
            raw, now=start + index * step, healthy=healthy,
        )
    return result


def replay(path):
    controller = RingPatrolController()
    results = []
    with open(path, newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                raw = {
                    name: float(row[name])
                    for name in GrayRiskModel.NAMES
                }
                now = float(row["t"])
            except (KeyError, TypeError, ValueError):
                continue
            healthy = float(row.get("healthy", 1) or 0) > 0.0
            results.append(controller.update(raw, now=now, healthy=healthy))
    return results


class RingPatrolTest(unittest.TestCase):
    def test_sensor_channel_mapping_matches_new_car_collection(self):
        self.assertEqual(
            {"front": 2, "rear": 3, "left": 1, "right": 0},
            GRAY_CHANNELS,
        )

    def test_cruise_is_straight_and_graded_by_zone(self):
        cases = (
            (1.20, "CRUISE", PATROL_CRUISE_LINEAR),
            (0.80, "MEDIUM_CRUISE", PATROL_MEDIUM_LINEAR),
        )
        for zone, state, speed in cases:
            result = feed(RingPatrolController(), raw_at_zone(zone), 5)
            self.assertEqual(state, result["state"])
            self.assertEqual((speed, speed), (result["left"], result["right"]))

    def test_front_edge_retreats_before_135_degree_turn(self):
        raw = raw_at_zone_components(
            front=0.0, rear=0.4, left=0.2, right=0.2,
        )
        controller = RingPatrolController()
        result = feed(controller, raw, 5)
        self.assertEqual("EDGE_AVOID", result["state"])
        self.assertEqual(
            (-PATROL_RECOVER_BACKWARD_SPEED, -PATROL_RECOVER_BACKWARD_SPEED),
            (result["left"], result["right"]),
        )

        result = controller.update(
            raw,
            now=controller.state_started + PATROL_EDGE_RETREAT_SECONDS + 0.001,
        )
        self.assertEqual("EDGE_TURN", result["state"])
        self.assertEqual(PATROL_EDGE_TURN_ANGLE, result["turn_angle"])
        self.assertLess(result["left"] * result["right"], 0)

    def test_side_risk_turns_away_even_when_the_zone_is_shallow(self):
        cases = (
            ("left", 0.70, 0.95, (1, -1)),
            ("right", 0.95, 0.70, (-1, 1)),
        )
        for side, left, right, expected_sign in cases:
            with self.subTest(side=side):
                raw = raw_at_zone_components(
                    front=0.70, rear=0.90, left=left, right=right,
                )
                controller = RingPatrolController()
                feed(controller, raw, 5)
                result = controller.update(
                    raw,
                    now=(controller.state_started
                         + PATROL_EDGE_RETREAT_SECONDS + 0.001),
                )
                self.assertEqual("EDGE_TURN", result["state"])
                self.assertEqual(expected_sign, (
                    1 if result["left"] > 0 else -1,
                    1 if result["right"] > 0 else -1,
                ))

    def test_clear_rear_edge_moves_forward_before_turn(self):
        raw = raw_at_zone_components(
            front=0.4, rear=0.0, left=0.2, right=0.2,
        )
        result = feed(RingPatrolController(), raw, 5)
        self.assertEqual("EDGE_AVOID", result["state"])
        self.assertEqual(
            (PATROL_RECOVER_FORWARD_SPEED, PATROL_RECOVER_FORWARD_SPEED),
            (result["left"], result["right"]),
        )

    def test_front_dark_trend_triggers_early_retreat(self):
        raw = raw_at_zone_components(
            front=0.45, rear=1.00, left=0.80, right=0.80,
        )
        result = feed(RingPatrolController(), raw, 7)
        self.assertEqual("EDGE_AVOID", result["state"])
        self.assertIn("提前离边", result["reason"])

    def test_turn_completes_into_forward_recovery(self):
        edge = raw_at_zone_components(
            front=0.0, rear=0.4, left=0.2, right=0.2,
        )
        controller = RingPatrolController()
        feed(controller, edge, 5)
        turn = feed(
            controller,
            raw_at_zone(0.8),
            3,
            start=controller.state_started + PATROL_EDGE_RETREAT_SECONDS + 0.001,
        )
        duration = MOTOR_TURN_CALIBRATION[turn["turn_direction"]][
            PATROL_EDGE_TURN_ANGLE
        ][1]
        result = controller.update(
            raw_at_zone(0.8),
            now=controller.state_started + duration + 0.001,
        )
        self.assertEqual("RECOVER_FORWARD", result["state"])
        self.assertEqual(
            (PATROL_RECOVER_FORWARD_SPEED, PATROL_RECOVER_FORWARD_SPEED),
            (result["left"], result["right"]),
        )

        result = feed(
            controller,
            raw_at_zone(0.8),
            3,
            start=controller.state_started + PATROL_RECOVER_SECONDS + 0.001,
        )
        self.assertIn(result["state"], ("MEDIUM_CRUISE", "CRUISE"))

    def test_forward_recovery_interrupts_when_front_darkens_again(self):
        edge = raw_at_zone_components(
            front=0.0, rear=0.4, left=0.2, right=0.2,
        )
        controller = RingPatrolController()
        feed(controller, edge, 5)
        turn = feed(
            controller,
            raw_at_zone(0.8),
            3,
            start=controller.state_started + PATROL_EDGE_RETREAT_SECONDS + 0.001,
        )
        duration = MOTOR_TURN_CALIBRATION[turn["turn_direction"]][
            PATROL_EDGE_TURN_ANGLE
        ][1]
        controller.update(
            raw_at_zone(0.8), now=controller.state_started + duration + 0.001,
        )

        risky = raw_at_zone_components(
            front=0.70, rear=0.80, left=0.80, right=0.80,
        )
        result = feed(
            controller, risky, 3,
            start=controller.state_started + 0.02,
        )
        self.assertEqual("EDGE_AVOID", result["state"])
        self.assertIn("提前离边", result["reason"])

    def test_deep_dark_during_turn_restarts_retreat(self):
        edge = raw_at_zone_components(
            front=0.0, rear=0.4, left=0.2, right=0.2,
        )
        controller = RingPatrolController()
        feed(controller, edge, 5)
        feed(
            controller,
            raw_at_zone(0.8),
            3,
            start=controller.state_started + PATROL_EDGE_RETREAT_SECONDS + 0.001,
        )
        deep = raw_at_zone_components(
            front=-0.8, rear=0.2, left=0.2, right=0.2,
        )
        result = feed(controller, deep, 2, start=controller.state_started + 0.02)
        self.assertEqual("EDGE_AVOID", result["state"])
        self.assertIn("深暗", result["reason"])

    def test_front_white_at_edge_commands_reverse(self):
        sample = raw_at_zone(0.0)
        sample["front"] = GRAY_WHITE_REFERENCE["front"]
        result = feed(RingPatrolController(), sample, 6)
        self.assertEqual("WHITE_ESCAPE", result["state"])
        self.assertEqual(
            (-PATROL_MIN_ACTIVE_SPEED, -PATROL_MIN_ACTIVE_SPEED),
            (result["left"], result["right"]),
        )

    def test_side_white_edge_turns_away_from_the_white_side(self):
        sample = raw_at_zone(0.0)
        sample["left"] = GRAY_WHITE_REFERENCE["left"]
        result = feed(RingPatrolController(), sample, 6)
        self.assertEqual("EDGE_TURN", result["state"])
        self.assertGreater(result["left"], 0)
        self.assertLess(result["right"], 0)

    def test_center_white_mark_does_not_trigger_escape(self):
        sample = raw_at_zone(1.2)
        sample["front"] = GRAY_WHITE_REFERENCE["front"]
        result = feed(RingPatrolController(), sample, 10)
        self.assertNotEqual("WHITE_ESCAPE", result["state"])

    def test_sensor_fault_stops(self):
        result = RingPatrolController().update(
            dict(GRAY_CENTER_REFERENCE), now=0.0, healthy=False,
        )
        self.assertEqual("SENSOR_STOP", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_rearm_restarts_retreat_timer(self):
        controller = RingPatrolController()
        feed(controller, raw_at_zone(1.0), 3)
        controller.rearm(now=10.0)
        self.assertEqual("EDGE_AVOID", controller.state)
        self.assertEqual(10.0, controller.state_started)
        result = controller.update(raw_at_zone(1.0), now=10.1)
        self.assertEqual("EDGE_AVOID", result["state"])

    def test_diagonal_white_edge_restarts_retreat_before_forward_recovery(self):
        path = os.path.join(DATA_DIR, DIAGONAL_WHITE_EDGE_LOG)
        results = replay(path)
        reasons = [item["reason"] for item in results]
        self.assertTrue(
            any("斜压白边" in reason for reason in reasons),
            DIAGONAL_WHITE_EDGE_LOG,
        )
        self.assertNotIn(
            "RECOVER_FORWARD",
            {item["state"] for item in results},
            DIAGONAL_WHITE_EDGE_LOG,
        )
        self.assertEqual("EDGE_AVOID", results[-1]["state"])

    def test_manual_diagonal_trials_interrupt_forward_recovery(self):
        results = replay(os.path.join(DATA_DIR, MANUAL_DIAGONAL_TRIAL_LOG))
        protected = [
            item for item in results
            if "车身斜压白边" in item["reason"]
        ]
        self.assertTrue(protected, MANUAL_DIAGONAL_TRIAL_LOG)
        self.assertTrue(
            all(item["state"] == "EDGE_AVOID" for item in protected),
            MANUAL_DIAGONAL_TRIAL_LOG,
        )

        run = 0
        max_run = 0
        for item in results:
            zone = item["observation"]["zone"]
            risky_forward = (
                item["state"] == "RECOVER_FORWARD"
                and min(zone["left"], zone["right"])
                < PATROL_DIAGONAL_SIDE_ZONE
                and abs(zone["left"] - zone["right"])
                >= PATROL_DIAGONAL_TURN_DELTA
            )
            run = run + 1 if risky_forward else 0
            max_run = max(max_run, run)
        self.assertLess(max_run, PATROL_DIAGONAL_CONFIRM)

    def test_shovel_edge_logs_preheat_and_retreat_on_first_ready_frame(self):
        for filename in (SHOVEL_FRONT_EDGE_LOG, SHOVEL_PREHEAT_EDGE_LOG):
            results = replay(os.path.join(DATA_DIR, filename))
            ready = [item for item in results if item["observation"]["ready"]]
            self.assertTrue(ready, filename)
            self.assertTrue(all(item["shovel_preheat"] for item in ready), filename)
            self.assertEqual("EDGE_AVOID", ready[0]["state"], filename)
            self.assertLess(ready[0]["left"], 0, filename)
            self.assertLess(ready[0]["right"], 0, filename)

    def test_safe_inner_does_not_preheat_shovel(self):
        result = feed(
            RingPatrolController(),
            raw_at_zone(PATROL_SHOVEL_PREHEAT_FRONT_ZONE + 0.10),
            5,
        )
        self.assertFalse(result["shovel_preheat"])

    def test_recorded_patrol_logs_complete_cycle_with_valid_commands(self):
        paths = [
            path
            for path in sorted(glob.glob(os.path.join(DATA_DIR, "patrol_*.csv")))
            if os.path.basename(path) != DIAGONAL_WHITE_EDGE_LOG
        ]
        self.assertTrue(paths)
        required = {"EDGE_AVOID", "EDGE_TURN", "RECOVER_FORWARD"}
        for path in paths:
            results = replay(path)
            states = {item["state"] for item in results}
            self.assertTrue(required <= states, os.path.basename(path))
            self.assertTrue(
                {"CRUISE", "MEDIUM_CRUISE"} & states,
                os.path.basename(path),
            )
            self.assertNotIn("SAFE_STOP", states, os.path.basename(path))
            for item in results:
                if item["state"] == "EDGE_TURN":
                    self.assertEqual(PATROL_EDGE_TURN_ANGLE, item["turn_angle"])
                for command in (item["left"], item["right"]):
                    self.assertLessEqual(abs(command), PATROL_COMMAND_LIMIT)
                    if command:
                        self.assertGreaterEqual(
                            abs(command), PATROL_MIN_ACTIVE_SPEED,
                        )

    def test_gray_model_csv_matches_runtime_references(self):
        path = os.path.join(DATA_DIR, "gray_model.csv")
        with open(path, newline="", encoding="utf-8-sig") as handle:
            rows = {row["sensor"]: row for row in csv.DictReader(handle)}
        for name in GRAY_CENTER_REFERENCE:
            self.assertAlmostEqual(
                GRAY_EDGE_REFERENCE[name],
                float(rows[name]["edge_reference"]),
                places=6,
            )
            self.assertAlmostEqual(
                GRAY_CENTER_REFERENCE[name],
                float(rows[name]["center_reference"]),
                places=6,
            )


if __name__ == "__main__":
    unittest.main()
