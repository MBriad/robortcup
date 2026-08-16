#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import inspect
import io
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import digi_ir
import gray
import ir
from dev import calibrate_front_adc, digi_ir_tool, ir_tool


class SensorModuleBoundaryTest(unittest.TestCase):
    def test_sensor_modules_accept_runtime_config_injection(self):
        gray_sensor = gray.GraySensor(channels={
            "front": 0, "rear": 1, "left": 2, "right": 3,
        })
        self.assertEqual(
            {"front": 10.0, "rear": 20.0, "left": 30.0, "right": 40.0},
            gray_sensor.read_raw([10, 20, 30, 40]),
        )

        ir_sensor = ir.IrSensor(channels={"left": 1, "right": 0})
        self.assertEqual(
            {"left": 20.0, "right": 10.0, "valid": True},
            ir_sensor.read_raw([10, 20]),
        )

        digital = digi_ir.DigiIR(bits={"front": 0}, active_level=1)
        self.assertEqual(
            {"front": True, "valid": True}, digital.read_states(1),
        )

    def test_production_sensor_modules_do_not_contain_dev_runtime(self):
        for module in (gray, ir, digi_ir):
            source = inspect.getsource(module)
            for forbidden in ("DEV_MODE", "argparse", "import uptech", "def main("):
                self.assertNotIn(forbidden, source, module.__name__)

    def test_dev_entrypoints_prefer_root_modules_when_pythonpath_contains_root(self):
        environment = dict(os.environ)
        environment["PYTHONPATH"] = ROOT
        for filename in ("ring_patrol.py", "reentry.py", "enemy_push.py"):
            result = subprocess.run(
                [sys.executable, "-B", os.path.join(ROOT, "dev", filename), "--help"],
                cwd=ROOT,
                env=environment,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(0, result.returncode, result.stderr)

    def test_enemy_dev_runner_does_not_create_other_strategies(self):
        path = os.path.join(ROOT, "dev", "enemy_push.py")
        with open(path, encoding="utf-8") as source_file:
            source = source_file.read()
        for forbidden in (
                "RobotController", "RingPatrolController",
                "ReentryController", "HuntController"):
            self.assertNotIn(forbidden, source)

    def test_digital_ir_collection_writes_raw_and_mapped_values(self):
        class FakeHardware:
            @staticmethod
            def ADC_IO_GetAllInputLevel():
                return 0

        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "digi_ir.csv")
            with mock.patch.object(
                digi_ir_tool.time,
                "monotonic",
                side_effect=(0.0, 0.0, 0.0, 0.0, 0.02),
            ), mock.patch.object(digi_ir_tool.time, "sleep"):
                with redirect_stdout(io.StringIO()):
                    digi_ir_tool.collect(FakeHardware(), path, hz=50, duration=0.01)

            with open(path, newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(1, len(rows))
        self.assertEqual("0", rows[0]["mask"])
        for index in range(8):
            self.assertEqual("0", rows[0]["io%d" % index])
        for name in digi_ir.DIGI_IR_PINS:
            self.assertEqual("1", rows[0][name])
        self.assertEqual("1", rows[0]["valid"])

    def test_front_adc_collection_writes_raw_values_and_diff(self):
        class FakeHardware:
            @staticmethod
            def ADC_Get_All_Channle():
                adc = [0] * 10
                adc[ir.IR_CHANNELS["left"]] = 700
                adc[ir.IR_CHANNELS["right"]] = 300
                return adc

        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "front_adc.csv")
            with mock.patch.object(
                ir_tool.time,
                "monotonic",
                side_effect=(0.0, 0.0, 0.0, 0.0, 0.02),
            ), mock.patch.object(ir_tool.time, "sleep"):
                with redirect_stdout(io.StringIO()):
                    ir_tool.collect(FakeHardware(), path, hz=50, duration=0.01)

            with open(path, newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(1, len(rows))
        self.assertEqual("700.0", rows[0]["left"])
        self.assertEqual("300.0", rows[0]["right"])
        self.assertEqual("400.0", rows[0]["diff"])
        self.assertEqual("1", rows[0]["valid"])

    def test_front_adc_calibration_keeps_raw_and_flags_floor_data(self):
        self.assertEqual(
            ("left_bias", "middle", False),
            calibrate_front_adc._classify("左偏20度且离墙居中位置.csv"),
        )
        with tempfile.TemporaryDirectory() as directory:
            center_path = os.path.join(directory, "正对着墙.csv")
            bias_path = os.path.join(
                directory, "车子左偏且从90度往正中间转且离墙居中位置转.csv"
            )
            for path, samples in (
                (center_path, ((1300, 1050), (1310, 1060), (1320, 1070))),
                (bias_path, ((20, 20), (200, 20), (400, 20))),
            ):
                with open(path, "w", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(
                        handle, fieldnames=("t", "left", "right", "diff", "valid")
                    )
                    writer.writeheader()
                    for index, (left, right) in enumerate(samples):
                        writer.writerow({
                            "t": index * 0.02,
                            "left": left,
                            "right": right,
                            "diff": left - right,
                            "valid": 1,
                        })

            processed_path = os.path.join(directory, "processed.csv")
            summary_path = os.path.join(directory, "summary.csv")
            processed, summary = calibrate_front_adc.analyze(
                directory, processed_path, summary_path,
                window=1, active_min=50.0,
            )

            with open(center_path, newline="", encoding="utf-8") as handle:
                original_rows = list(csv.DictReader(handle))

        self.assertEqual(3, len(original_rows))
        self.assertEqual(6, len(processed))
        center = next(row for row in summary if row["label"] == "center_near")
        bias_start = next(
            row for row in summary
            if row["label"] == "left_bias" and row["phase"] == "start"
        )
        self.assertEqual(1, center["alignment_usable"])
        self.assertEqual(0, bias_start["alignment_usable"])

    def test_front_adc_model_uses_non_overlapping_steady_ranges(self):
        summary = [
            {
                "source": "left.csv", "label": "left_bias",
                "distance": "middle", "phase": "steady",
                "diff_p05": 220.0, "diff_p95": 290.0,
                "signal_p01": 530.0, "signal_p99": 600.0,
            },
            {
                "source": "center.csv", "label": "center_middle",
                "distance": "middle", "phase": "steady",
                "diff_p05": 370.0, "diff_p95": 440.0,
                "signal_p01": 640.0, "signal_p99": 710.0,
            },
            {
                "source": "right.csv", "label": "right_bias",
                "distance": "middle", "phase": "steady",
                "diff_p05": 560.0, "diff_p95": 620.0,
                "signal_p01": 580.0, "signal_p99": 650.0,
            },
            {
                "source": "far.csv", "label": "center_far",
                "distance": "far", "phase": "steady",
                "diff_p05": 150.0, "diff_p95": 200.0,
                "signal_p01": 160.0, "signal_p99": 224.0,
            },
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "model.csv")
            rows = calibrate_front_adc.derive_model(summary, path, window=9)
        values = {row["parameter"]: row["value"] for row in rows}
        self.assertEqual(330.0, values["diff_low"])
        self.assertEqual(500.0, values["diff_high"])
        self.assertEqual(377.0, values["signal_min"])


if __name__ == "__main__":
    unittest.main()
