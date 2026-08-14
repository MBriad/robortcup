#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import (
    GRAY_ADC_MAX,
    GRAY_CENTER_REFERENCE,
    GRAY_CHANNELS,
    GRAY_EDGE_REFERENCE,
    GRAY_FILTER_WINDOW,
    GRAY_NEAR_EDGE_CLEAR,
    GRAY_NEAR_EDGE_ENTER,
    GRAY_WHITE_CLEAR,
    GRAY_WHITE_ENTER,
    GRAY_WHITE_REFERENCE,
    MOTOR_TURN_CALIBRATION,
    PATROL_CRUISE_LINEAR,
    PATROL_CRUISE_TURN,
    PATROL_EDGE_AVOID_LINEAR,
    PATROL_EDGE_AVOID_TURN,
    PATROL_EDGE_TURN_ANGLE,
    PATROL_MEDIUM_LINEAR,
    PATROL_MEDIUM_TURN,
    PATROL_MIN_ACTIVE_SPEED,
    PATROL_RECOVER_SPEED,
)
from gray import GrayRiskModel
from ring_patrol import RingPatrolController


DATA_DIR = os.path.join(ROOT, "data")


def configured_gray_model():
    return GrayRiskModel(
        window=GRAY_FILTER_WINDOW,
        edge_reference=GRAY_EDGE_REFERENCE,
        center_reference=GRAY_CENTER_REFERENCE,
        white_reference=GRAY_WHITE_REFERENCE,
        white_enter=GRAY_WHITE_ENTER,
        white_clear=GRAY_WHITE_CLEAR,
        near_edge_enter=GRAY_NEAR_EDGE_ENTER,
        near_edge_clear=GRAY_NEAR_EDGE_CLEAR,
        adc_max=GRAY_ADC_MAX,
    )


def replay(filename):
    controller = RingPatrolController()
    states = []
    with open(os.path.join(DATA_DIR, filename), newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            raw = {name: float(row[name]) for name in ("front", "rear", "left", "right")}
            states.append(controller.update(raw, now=float(row["t"])))
    return states


def raw_at_zone(score):
    return {
        name: (GRAY_EDGE_REFERENCE[name] +
               score * (GRAY_CENTER_REFERENCE[name] - GRAY_EDGE_REFERENCE[name]))
        for name in GRAY_CENTER_REFERENCE
    }


def raw_at_zone_components(**scores):
    return {
        name: (GRAY_EDGE_REFERENCE[name] +
               scores[name] * (GRAY_CENTER_REFERENCE[name] - GRAY_EDGE_REFERENCE[name]))
        for name in GRAY_CENTER_REFERENCE
    }


class RingPatrolTest(unittest.TestCase):
    def test_center_data_never_triggers_white_edge_escape(self):
        files = ("武字中间旋转2圈.csv", "武字中间逆时针旋转2圈.csv",
                 "武字数据.csv", "武字数据逆时针.csv")
        for filename in files:
            states = replay(filename)
            unsafe = {item["state"] for item in states} & {"WHITE_ESCAPE"}
            self.assertEqual(set(), unsafe, filename)

    def test_edge_data_enters_moving_avoidance_without_white_escape(self):
        states = replay("边缘.csv")
        names = {item["state"] for item in states}
        self.assertIn("EDGE_AVOID", names)
        self.assertNotIn("WHITE_ESCAPE", names)

    def test_axis_replay_contains_edge_and_center_sections(self):
        states = replay("中轴.csv")
        names = [item["state"] for item in states]
        self.assertIn("EDGE_AVOID", names[:300])
        self.assertIn("CRUISE", names)
        protective = {"EDGE_AVOID", "RECOVER_FORWARD", "RECOVER_BACKWARD", "SAFE_STOP"}
        self.assertTrue(protective & set(names[-400:]))

    def test_updated_inner_outer_data_is_safe_reference(self):
        files = (
            "更内环侧向移动车头朝武字反方向.csv",
            "更内环侧向移动车头朝武字方向.csv",
            "更内环平行移动车头朝内.csv",
            "更内环平行移动车头朝外.csv",
        )
        for filename in files:
            states = replay(filename)
            names = {item["state"] for item in states}
            self.assertFalse(
                {"EDGE_TURN", "WHITE_ESCAPE", "SAFE_STOP"} & names,
                filename,
            )

    def test_sensor_channel_mapping_matches_collection_labels(self):
        # 新车接线左右插反（2026-08-14 scan 确认），left/right 通道对调。
        self.assertEqual(
            {"front": 2, "rear": 3, "left": 1, "right": 0},
            GRAY_CHANNELS,
        )

    def test_single_sample_jump_is_filtered(self):
        controller = RingPatrolController()
        raw = dict(GRAY_CENTER_REFERENCE)
        for index in range(10):
            sample = dict(raw)
            if index == 5:
                sample["front"] = 6000.0
            result = controller.update(sample, now=index * 0.02)
            self.assertNotEqual("WHITE_ESCAPE", result["state"])

    def white_result(self, *sensors):
        controller = RingPatrolController()
        # 其余路按边缘暗值取值，模拟边界白（zone 低，通过白边门槛）。
        sample = dict(GRAY_EDGE_REFERENCE)
        for sensor in sensors:
            sample[sensor] = GRAY_WHITE_REFERENCE[sensor]
        result = None
        for index in range(6):
            result = controller.update(sample, now=index * 0.02)
        self.assertIn(result["state"], ("WHITE_ESCAPE", "EDGE_TURN"))
        return result

    def white_command(self, sensor):
        result = self.white_result(sensor)
        return result["left"], result["right"]

    def test_front_white_commands_reverse(self):
        left, right = self.white_command("front")
        self.assertLess(left, 0)
        self.assertLess(right, 0)

    def test_rear_white_commands_forward(self):
        left, right = self.white_command("rear")
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)

    def test_left_white_turns_180_degrees(self):
        result = self.white_result("left")
        self.assertEqual("EDGE_TURN", result["state"])
        self.assertEqual(PATROL_EDGE_TURN_ANGLE, result["turn_angle"])
        self.assertLess(result["left"] * result["right"], 0)

    def test_right_white_turns_180_degrees(self):
        result = self.white_result("right")
        self.assertEqual("EDGE_TURN", result["state"])
        self.assertEqual(PATROL_EDGE_TURN_ANGLE, result["turn_angle"])
        self.assertLess(result["left"] * result["right"], 0)

    def test_big_turn_completes_even_if_white_is_still_visible(self):
        sample = dict(GRAY_EDGE_REFERENCE)
        sample["left"] = GRAY_WHITE_REFERENCE["left"]
        controller = RingPatrolController()
        result = None
        for index in range(5):
            result = controller.update(sample, now=index * 0.02)
        self.assertEqual("EDGE_TURN", result["state"])
        duration = MOTOR_TURN_CALIBRATION[controller.turn_direction][
            PATROL_EDGE_TURN_ANGLE][1]
        result = controller.update(
            sample, now=controller.state_started + duration + 0.001,
        )
        self.assertEqual("RECOVER_FORWARD", result["state"])
        self.assertEqual((PATROL_RECOVER_SPEED, PATROL_RECOVER_SPEED),
                         (result["left"], result["right"]))

    def test_side_gray_uses_forward_arc_before_danger(self):
        # 弧线背离暗侧：左端变暗→右转离开，右端变暗→左转离开。
        cases = (
            ("left", (640, 400), "right"),
            ("right", (400, 640), "left"),
        )
        for sensor, command, direction in cases:
            scores = {name: 0.55 for name in GRAY_CENTER_REFERENCE}
            scores[sensor] = 0.10
            controller = RingPatrolController()
            result = None
            for index in range(5):
                result = controller.update(raw_at_zone_components(**scores), now=index * 0.02)
            self.assertEqual("EDGE_AVOID", result["state"])
            self.assertEqual(command, (result["left"], result["right"]))
            self.assertEqual(direction, result["turn_direction"])
            self.assertGreater(result["left"] * result["right"], 0)

    def test_side_arc_uses_big_turn_if_gray_does_not_improve(self):
        scores = {name: 0.20 for name in GRAY_CENTER_REFERENCE}
        scores["left"] = 0.00
        controller = RingPatrolController()
        result = None
        for index in range(40):
            result = controller.update(raw_at_zone_components(**scores), now=index * 0.02)
        self.assertEqual("EDGE_TURN", result["state"])
        self.assertEqual(PATROL_EDGE_TURN_ANGLE, result["turn_angle"])
        self.assertLess(result["left"] * result["right"], 0)

    def test_side_arc_switches_directly_to_180_turn(self):
        scores = {name: 0.20 for name in GRAY_CENTER_REFERENCE}
        scores["left"] = 0.00
        controller = RingPatrolController()
        switched = None
        for index in range(15):
            result = controller.update(
                raw_at_zone_components(**scores), now=index * 0.02
            )
            if result["state"] == "EDGE_TURN":
                switched = result
                break
        self.assertIsNotNone(switched)
        self.assertEqual(PATROL_EDGE_TURN_ANGLE, switched["turn_angle"])
        self.assertLess(switched["left"] * switched["right"], 0)

    def test_latest_patrol_start_does_not_trigger_big_turn(self):
        path = os.path.join(DATA_DIR, "patrol_small_big_turn1.csv")
        with open(path, newline="", encoding="utf-8-sig") as handle:
            rows = [
                row for row in csv.DictReader(handle)
                if float(row["front"]) > 0.0
            ][:10]
        controller = RingPatrolController()
        states = []
        for row in rows:
            raw = {
                name: float(row[name])
                for name in ("front", "rear", "left", "right")
            }
            states.append(controller.update(raw, now=float(row["t"])))
        self.assertNotIn("EDGE_TURN", {item["state"] for item in states})
        self.assertGreater(states[-1]["observation"]["zone_score"], 0.75)

    def test_sensor_fault_stops(self):
        controller = RingPatrolController()
        result = controller.update(dict(GRAY_EDGE_REFERENCE), healthy=False)
        self.assertEqual((0, 0), (result["left"], result["right"]))
        self.assertEqual("SENSOR_STOP", result["state"])

    def test_all_nonzero_commands_clear_motor_deadband(self):
        states = replay("中轴.csv") + replay("边缘.csv")
        for item in states:
            for command in (item["left"], item["right"]):
                if command:
                    self.assertGreaterEqual(abs(command), PATROL_MIN_ACTIVE_SPEED)

    def test_zone_score_applies_graded_speed_limits(self):
        cases = (
            (1.00, "fast", RingPatrolController._mix(
                PATROL_CRUISE_LINEAR, PATROL_CRUISE_TURN)),
            (0.88, "medium", RingPatrolController._mix(
                PATROL_MEDIUM_LINEAR, PATROL_MEDIUM_TURN)),
        )
        for zone_score, level, command in cases:
            controller = RingPatrolController()
            result = None
            for index in range(5):
                result = controller.update(raw_at_zone(zone_score), now=index * 0.02)
            self.assertEqual(level, result["speed_level"])
            self.assertEqual(command, (result["left"], result["right"]))

    def test_low_zone_starts_avoidance_before_hard_edge_confirmation(self):
        scores = {name: 0.55 for name in GRAY_CENTER_REFERENCE}
        scores["left"] = 0.10
        controller = RingPatrolController()
        result = None
        for index in range(3):
            result = controller.update(raw_at_zone_components(**scores), now=index * 0.02)
        self.assertEqual("EDGE_AVOID", result["state"])
        # 左端变暗 → 右转离开（背离暗侧）。
        self.assertEqual(
            RingPatrolController._mix(PATROL_EDGE_AVOID_LINEAR, PATROL_EDGE_AVOID_TURN),
            (result["left"], result["right"]),
        )

    def test_white_escape_uses_minimum_active_speed(self):
        for sensor in ("front", "rear"):
            result = self.white_result(sensor)
            self.assertEqual("danger", result["speed_level"])
            self.assertEqual(
                (PATROL_MIN_ACTIVE_SPEED, PATROL_MIN_ACTIVE_SPEED),
                (abs(result["left"]), abs(result["right"])),
            )

    def test_per_sensor_values_are_normalized_before_fusion(self):
        model = configured_gray_model()
        observation = None
        for _ in range(3):
            observation = model.update(raw_at_zone(0.40))
        for value in observation["zone"].values():
            self.assertAlmostEqual(0.40, value, places=6)
        self.assertAlmostEqual(0.40, observation["zone_score"], places=6)

    def test_gray_model_csv_matches_runtime_references(self):
        path = os.path.join(DATA_DIR, "gray_model.csv")
        with open(path, newline="", encoding="utf-8-sig") as handle:
            rows = {row["sensor"]: row for row in csv.DictReader(handle)}
        for name in GRAY_CENTER_REFERENCE:
            self.assertAlmostEqual(
                GRAY_EDGE_REFERENCE[name], float(rows[name]["edge_reference"]), places=6
            )
            self.assertAlmostEqual(
                GRAY_CENTER_REFERENCE[name], float(rows[name]["center_reference"]), places=6
            )

    def test_rear_edge_uses_big_turn_after_confirmation(self):
        raw = raw_at_zone_components(front=0.20, rear=0.00, left=0.20, right=0.20)
        controller = RingPatrolController()
        result = None
        for index in range(5):
            result = controller.update(raw, now=index * 0.02)
        self.assertEqual("EDGE_TURN", result["state"])
        self.assertEqual(PATROL_EDGE_TURN_ANGLE, result["turn_angle"])

    def test_front_edge_uses_big_turn_after_confirmation(self):
        raw = raw_at_zone_components(front=0.00, rear=0.20, left=0.20, right=0.20)
        controller = RingPatrolController()
        result = None
        for index in range(5):
            result = controller.update(raw, now=index * 0.02)
        self.assertEqual("EDGE_TURN", result["state"])
        self.assertEqual(PATROL_EDGE_TURN_ANGLE, result["turn_angle"])

    def test_real_edge_log_uses_only_calibrated_big_turns(self):
        states = replay("patrol_speed_test4.csv")
        turns = []
        for item in states:
            if item["left"] * item["right"] < 0:
                turns.append(item)
                self.assertEqual("EDGE_TURN", item["state"])
                self.assertEqual(PATROL_EDGE_TURN_ANGLE, item["turn_angle"])
        self.assertTrue(turns)

    def test_failed_direction_search_keeps_alternating_recover(self):
        raw = raw_at_zone_components(front=0.20, rear=0.00, left=0.20, right=0.20)
        controller = RingPatrolController()
        result = None
        for index in range(350):
            result = controller.update(raw, now=index * 0.02)
        self.assertIn(result["state"], ("RECOVER_FORWARD", "RECOVER_BACKWARD"))
        self.assertNotEqual((0, 0), (result["left"], result["right"]))
        result = controller.update(raw, now=7.5)
        self.assertIn(result["state"], ("RECOVER_FORWARD", "RECOVER_BACKWARD"))


if __name__ == "__main__":
    unittest.main()
