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
    DIGI_IR_PINS,
    GRAY_ADC_MAX,
    GRAY_CENTER_REFERENCE,
    GRAY_EDGE_REFERENCE,
    GRAY_FILTER_WINDOW,
    GRAY_NEAR_EDGE_CLEAR,
    GRAY_NEAR_EDGE_ENTER,
    GRAY_WHITE_CLEAR,
    GRAY_WHITE_ENTER,
    GRAY_WHITE_REFERENCE,
    IR_ADC_MAX,
    IR_ALIGNMENT_CONFIRM,
    IR_ALIGNMENT_DIFF_HIGH,
    IR_ALIGNMENT_DIFF_LOW,
    IR_ALIGNMENT_FILTER_WINDOW,
    IR_ALIGNMENT_SIGNAL_MIN,
    MOTOR_TURN_CALIBRATION,
    PATROL_MIN_ACTIVE_SPEED,
)
from gray import GrayRiskModel
from ir import IrAlignmentModel
from reentry import (
    APPROACH_SPEED,
    APPROACH_TIMEOUT,
    FALL_CONFIRM,
    ReentryController,
)


def fallen_gray():
    return {
        name: max(0.0, value - 100.0)
        for name, value in GRAY_EDGE_REFERENCE.items()
    }


def safe_gray():
    return dict(GRAY_CENTER_REFERENCE)


def ir_states(**active):
    states = {name: False for name in DIGI_IR_PINS}
    states.update(active)
    states["valid"] = True
    return states


ANALOG_CENTERED = {"left": 500.0, "right": 500.0, "valid": True}
ANALOG_LEFT_BIAS = {"left": 550.0, "right": 305.0, "valid": True}
ANALOG_ALIGNED = {"left": 661.0, "right": 273.0, "valid": True}
ANALOG_RIGHT_BIAS = {"left": 610.0, "right": 20.0, "valid": True}
ANALOG_WEAK = {"left": 184.0, "right": 20.0, "valid": True}


def configured_gray_model(window=GRAY_FILTER_WINDOW):
    return GrayRiskModel(
        window=window,
        edge_reference=GRAY_EDGE_REFERENCE,
        center_reference=GRAY_CENTER_REFERENCE,
        white_reference=GRAY_WHITE_REFERENCE,
        white_enter=GRAY_WHITE_ENTER,
        white_clear=GRAY_WHITE_CLEAR,
        near_edge_enter=GRAY_NEAR_EDGE_ENTER,
        near_edge_clear=GRAY_NEAR_EDGE_CLEAR,
        adc_max=GRAY_ADC_MAX,
    )


def configured_alignment_model(window=IR_ALIGNMENT_FILTER_WINDOW):
    return IrAlignmentModel(
        window=window,
        diff_low=IR_ALIGNMENT_DIFF_LOW,
        diff_high=IR_ALIGNMENT_DIFF_HIGH,
        signal_min=IR_ALIGNMENT_SIGNAL_MIN,
        adc_max=IR_ADC_MAX,
    )


class ReentryControllerTest(unittest.TestCase):
    @staticmethod
    def controller_without_filter_delay():
        controller = ReentryController()
        controller.model = configured_gray_model(window=1)
        return controller

    def replay_csv(self, filename):
        controller = ReentryController()
        trigger_times = []
        path = os.path.join(ROOT, "data", filename)
        with open(path, newline="", encoding="utf-8-sig") as csv_file:
            for row in csv.DictReader(csv_file):
                result = controller.update(
                    {
                        name: float(row[name])
                        for name in ("front", "rear", "left", "right")
                    },
                    ir_states(),
                    ANALOG_CENTERED,
                    now=float(row["t"]),
                    healthy=bool(int(row.get("healthy", "1"))),
                )
                if result["fall_edge"]:
                    trigger_times.append(float(row["t"]))
        return trigger_times

    def trigger_fall(self, controller, ir, start=0.0):
        result = None
        for index in range(5):
            result = controller.update(
                fallen_gray(), ir, ANALOG_CENTERED,
                now=start + index * 0.02, healthy=True,
            )
        self.assertTrue(result["fall_edge"])
        return result

    def test_fall_requires_exact_consecutive_confirmation(self):
        controller = self.controller_without_filter_delay()

        for index in range(FALL_CONFIRM - 1):
            result = controller.update(
                fallen_gray(), ir_states(), ANALOG_CENTERED,
                now=index * 0.02, healthy=True,
            )
            self.assertFalse(result["fall_edge"])
            self.assertEqual("WAIT", result["state"])

        result = controller.update(
            fallen_gray(), ir_states(), ANALOG_CENTERED,
            now=(FALL_CONFIRM - 1) * 0.02, healthy=True,
        )
        self.assertTrue(result["fall_edge"])
        self.assertEqual("IR_WAIT", result["state"])

        result = controller.update(
            fallen_gray(), ir_states(), ANALOG_CENTERED,
            now=FALL_CONFIRM * 0.02, healthy=True,
        )
        self.assertFalse(result["fall_edge"])

    def test_safe_frame_resets_fall_confirmation(self):
        controller = self.controller_without_filter_delay()
        now = 0.0

        for _ in range(FALL_CONFIRM - 1):
            controller.update(
                fallen_gray(), ir_states(), ANALOG_CENTERED,
                now=now, healthy=True,
            )
            now += 0.02

        result = controller.update(
            safe_gray(), ir_states(), ANALOG_CENTERED,
            now=now, healthy=True,
        )
        self.assertFalse(result["fall"])
        self.assertFalse(result["fall_edge"])

        for _ in range(FALL_CONFIRM - 1):
            now += 0.02
            result = controller.update(
                fallen_gray(), ir_states(), ANALOG_CENTERED,
                now=now, healthy=True,
            )
            self.assertFalse(result["fall_edge"])

        now += 0.02
        result = controller.update(
            fallen_gray(), ir_states(), ANALOG_CENTERED,
            now=now, healthy=True,
        )
        self.assertTrue(result["fall_edge"])

    def test_one_safe_sensor_blocks_four_sensor_fall_policy(self):
        controller = self.controller_without_filter_delay()
        three_dark = fallen_gray()
        three_dark["rear"] = GRAY_CENTER_REFERENCE["rear"]

        for index in range(FALL_CONFIRM + 2):
            result = controller.update(
                three_dark, ir_states(), ANALOG_CENTERED,
                now=index * 0.02, healthy=True,
            )

        self.assertFalse(result["fall"])
        self.assertFalse(result["fall_edge"])
        self.assertEqual("WAIT", result["state"])

    def test_fall_can_trigger_again_after_gray_recovers(self):
        controller = self.controller_without_filter_delay()
        first_edges = 0
        second_edges = 0

        for index in range(FALL_CONFIRM):
            result = controller.update(
                fallen_gray(), ir_states(), ANALOG_CENTERED,
                now=index * 0.02, healthy=True,
            )
            first_edges += int(result["fall_edge"])

        result = controller.update(
            safe_gray(), ir_states(), ANALOG_CENTERED,
            now=0.10, healthy=True,
        )
        self.assertEqual("WAIT", result["state"])

        for index in range(FALL_CONFIRM):
            result = controller.update(
                fallen_gray(), ir_states(), ANALOG_CENTERED,
                now=0.12 + index * 0.02, healthy=True,
            )
            second_edges += int(result["fall_edge"])

        self.assertEqual(1, first_edges)
        self.assertEqual(1, second_edges)

    def test_real_fall_csvs_trigger_detector(self):
        for filename in ("掉台四个灰度.csv", "掉台四个灰度比较脏.csv"):
            with self.subTest(filename=filename):
                self.assertTrue(self.replay_csv(filename))

    def test_latest_reentry_csv_does_not_match_four_sensor_policy(self):
        self.assertEqual([], self.replay_csv("reentry_20260811_100641.csv"))

    def test_adc_correction_uses_calibrated_turn_directions(self):
        controller = ReentryController()
        controller._alignment = configured_alignment_model(window=1)
        result = self.trigger_fall(controller, ir_states(front=True))
        self.assertEqual("ADC_CORRECT", result["state"])

        result = controller.update(
            fallen_gray(), ir_states(front=True),
            ANALOG_LEFT_BIAS,
            now=0.10, healthy=True,
        )
        self.assertEqual(
            (PATROL_MIN_ACTIVE_SPEED, -PATROL_MIN_ACTIVE_SPEED),
            (result["left"], result["right"]),
        )

        controller = ReentryController()
        controller._alignment = configured_alignment_model(window=1)
        self.trigger_fall(controller, ir_states(front=True))
        result = controller.update(
            fallen_gray(), ir_states(front=True), ANALOG_RIGHT_BIAS,
            now=0.10, healthy=True,
        )
        self.assertEqual(
            (-PATROL_MIN_ACTIVE_SPEED, PATROL_MIN_ACTIVE_SPEED),
            (result["left"], result["right"]),
        )

    def test_adc_center_requires_consecutive_confirmation(self):
        controller = ReentryController()
        controller._alignment = configured_alignment_model(window=1)
        self.trigger_fall(controller, ir_states(front=True))

        for index in range(IR_ALIGNMENT_CONFIRM - 1):
            result = controller.update(
                fallen_gray(), ir_states(front=True), ANALOG_ALIGNED,
                now=0.10 + index * 0.02, healthy=True,
            )
            self.assertEqual("ADC_CORRECT", result["state"])
            self.assertEqual((0, 0), (result["left"], result["right"]))

        result = controller.update(
            fallen_gray(), ir_states(front=True), ANALOG_ALIGNED,
            now=0.10 + (IR_ALIGNMENT_CONFIRM - 1) * 0.02, healthy=True,
        )
        self.assertEqual("REVERSE", result["state"])
        self.assertEqual((-400, -400), (result["left"], result["right"]))

    def test_adc_weak_signal_rushes_once_then_stops_on_timeout(self):
        controller = ReentryController()
        controller._alignment = configured_alignment_model(window=1)
        self.trigger_fall(controller, ir_states(front=True))

        # 信号弱：一次性大力前冲（不循环重试）
        result = controller.update(
            fallen_gray(), ir_states(front=True), ANALOG_WEAK,
            now=0.10, healthy=True,
        )
        self.assertEqual("ADC_APPROACH", result["state"])
        self.assertEqual((APPROACH_SPEED, APPROACH_SPEED),
                         (result["left"], result["right"]))

        # 超时未贴墙：直接停车，不再回矫正重试
        result = controller.update(
            fallen_gray(), ir_states(front=True), ANALOG_WEAK,
            now=0.10 + APPROACH_TIMEOUT + 0.001, healthy=True,
        )
        self.assertEqual("SAFE_STOP", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

    def test_adc_approach_stops_when_wall_touched(self):
        controller = ReentryController()
        controller._alignment = configured_alignment_model(window=1)
        self.trigger_fall(controller, ir_states(front=True))

        result = controller.update(
            fallen_gray(), ir_states(front=True), ANALOG_WEAK,
            now=0.10, healthy=True,
        )
        self.assertEqual("ADC_APPROACH", result["state"])
        self.assertEqual((APPROACH_SPEED, APPROACH_SPEED),
                         (result["left"], result["right"]))

        touched = {"left": 1200.0, "right": 1200.0, "valid": True}
        result = controller.update(
            fallen_gray(), ir_states(front=True), touched,
            now=0.11, healthy=True,
        )
        self.assertEqual("ADC_CORRECT", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))
        self.assertIn("贴墙", result["reason"])

    def test_real_front_adc_csvs_match_calibrated_positions(self):
        cases = (
            ("左偏20度且离墙居中位置.csv", "left_bias"),
            ("正对墙且离墙居中位置.csv", "center"),
            ("右偏20度且离墙居中位置.csv", "right_bias"),
        )
        for filename, expected in cases:
            with self.subTest(filename=filename):
                model = configured_alignment_model()
                ready_positions = []
                path = os.path.join(ROOT, "data", filename)
                with open(path, newline="", encoding="utf-8-sig") as handle:
                    for row in csv.DictReader(handle):
                        result = model.update({
                            "left": float(row["left"]),
                            "right": float(row["right"]),
                            "valid": bool(int(row["valid"])),
                        })
                        if result["ready"]:
                            ready_positions.append(result["position"])
                matched = sum(
                    position == expected for position in ready_positions
                ) / len(ready_positions)
                self.assertGreaterEqual(matched, 0.95)

    def test_front_adc_model_csv_matches_runtime_parameters(self):
        path = os.path.join(ROOT, "data", "front_adc_model.csv")
        with open(path, newline="", encoding="utf-8") as handle:
            model = {
                row["parameter"]: float(row["value"])
                for row in csv.DictReader(handle)
            }
        self.assertEqual(IR_ALIGNMENT_FILTER_WINDOW, int(model["filter_window"]))
        self.assertEqual(IR_ALIGNMENT_DIFF_LOW, model["diff_low"])
        self.assertEqual(IR_ALIGNMENT_DIFF_HIGH, model["diff_high"])
        self.assertEqual(IR_ALIGNMENT_CONFIRM, int(model["center_confirm"]))
        self.assertEqual(IR_ALIGNMENT_SIGNAL_MIN, model["signal_min"])

    def test_ir_appearing_after_fall_starts_turn(self):
        controller = ReentryController()
        result = self.trigger_fall(controller, ir_states())
        self.assertEqual("IR_WAIT", result["state"])
        self.assertEqual((0, 0), (result["left"], result["right"]))

        result = controller.update(
            fallen_gray(), ir_states(right_front=True), ANALOG_CENTERED,
            now=0.10, healthy=True,
        )
        self.assertEqual("TURN_RIGHT_90", result["state"])
        self.assertEqual((500, -500), (result["left"], result["right"]))

    def test_turn_uses_full_calibrated_duration_before_adc(self):
        controller = ReentryController()
        result = self.trigger_fall(controller, ir_states(right_front=True))
        self.assertEqual("TURN_RIGHT_90", result["state"])
        started = controller._state_started
        duration = MOTOR_TURN_CALIBRATION[90.0][1]

        result = controller.update(
            fallen_gray(), ir_states(front=True), ANALOG_CENTERED,
            now=started + duration - 0.001, healthy=True,
        )
        self.assertEqual("TURN_RIGHT_90", result["state"])
        self.assertEqual((500, -500), (result["left"], result["right"]))

        result = controller.update(
            fallen_gray(), ir_states(front=True), ANALOG_CENTERED,
            now=started + duration + 0.001, healthy=True,
        )
        self.assertEqual("ADC_APPROACH", result["state"])
        self.assertEqual((APPROACH_SPEED, APPROACH_SPEED),
                         (result["left"], result["right"]))

    def test_turn_completion_starts_approach_without_front_ir(self):
        controller = ReentryController()
        self.trigger_fall(controller, ir_states(right_front=True))
        duration = MOTOR_TURN_CALIBRATION[90.0][1]

        result = controller.update(
            fallen_gray(), ir_states(), ANALOG_CENTERED,
            now=controller._state_started + duration + 0.001,
            healthy=True,
        )
        self.assertEqual("ADC_APPROACH", result["state"])
        self.assertEqual((APPROACH_SPEED, APPROACH_SPEED),
                         (result["left"], result["right"]))


if __name__ == "__main__":
    unittest.main()
