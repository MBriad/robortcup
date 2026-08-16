#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RoboCup 生产入口：安全、能量块、敌人推动与巡台统一仲裁。"""

import argparse
import csv
import os
import sys
import time

from config import (
    CHASSIS_MOTOR_INVERT,
    CHASSIS_MOTOR_SWAP,
    DIGI_IR_ACTIVE_LEVEL,
    DIGI_IR_BITS,
    DIGI_IR_PINS,
    GRAY_ADC_MAX,
    GRAY_CHANNELS,
    GRAY_WHITE_ENTER,
    IR_ADC_MAX,
    IR_CHANNELS,
    PATROL_STALE_SECONDS,
    SHOVEL_ADC_MAX,
    SHOVEL_IR_CHANNELS,
    VISION_CAMERA_DEVICE,
    VISION_MAX_AGE_MS,
)
from digi_ir import DigiIR
from enemy_push import EnemyPushController
from gray import GrayRiskModel, GraySensor
from hunt import HuntController
from ir import IrSensor
from reentry import ReentryController
from ring_patrol import RingPatrolController
from shovel_guard import ShovelGuard


ROOT = os.path.dirname(os.path.abspath(__file__))
YOLO_DIR = os.path.join(ROOT, "rpi-yolo-pi4-int8-lto-8fps")
HUNT_ALLOWED_PATROL_STATES = ("CRUISE", "MEDIUM_CRUISE")


class RobotController:
    """统一调度巡台与掉台回归；输入传感器数据，输出唯一电机命令。"""

    def __init__(self):
        self.patrol = RingPatrolController()
        self.reentry = ReentryController()
        self.shovel_guard = ShovelGuard()
        self.hunt = HuntController()
        self.enemy = EnemyPushController(guard=self.shovel_guard)
        self._reentry_active = False
        self._shovel_guard_active = False

    def update(self, gray_raw, ir, analog, shovel=None, vision=None,
               now=None, healthy=True):
        now = time.monotonic() if now is None else float(now)
        vision_available = (
            isinstance(vision, dict)
            and vision.get("sequence") is not None
            and vision.get("status") != "error"
        )
        reentry_result = self.reentry.update(
            gray_raw, ir, analog, now=now, healthy=healthy
        )

        if reentry_result["state"] != "WAIT":
            self.hunt.cancel()
            self.enemy.cancel()
            self._reentry_active = True
            self.shovel_guard = ShovelGuard()
            self.enemy.guard = self.shovel_guard
            self._shovel_guard_active = False
            return self._result("reentry", reentry_result)

        if self._reentry_active:
            # 回台期间巡台计时已失效，重新预热灰度模型后再恢复运动。
            self.patrol = RingPatrolController()
            self._reentry_active = False

        patrol_result = self.patrol.update(gray_raw, now=now, healthy=healthy)
        shovel = shovel or {"left": 0.0, "right": 0.0, "valid": False}
        shovel_preheat = bool(patrol_result["shovel_preheat"])

        if self.enemy.active:
            if patrol_result["state"] not in HUNT_ALLOWED_PATROL_STATES:
                self.enemy.cancel()
                self.hunt.cancel()
                return self._result("patrol", patrol_result)
            self.hunt.cancel()
            enemy_result = self.enemy.update(
                ir, patrol_result["observation"], shovel,
                now=now, healthy=healthy, allow_start=False,
            )
            if enemy_result["owns_control"]:
                selected = dict(enemy_result)
                selected["observation"] = patrol_result["observation"]
                selected["shovel_preheat"] = True
                return self._result("enemy_push", selected)
            return self._result("patrol", patrol_result)

        shovel_result = self.shovel_guard.update(
            shovel,
            active=shovel_preheat or self.shovel_guard.state != "IDLE",
            now=now,
            healthy=healthy,
        )
        if shovel_result["state"] != "IDLE":
            self.hunt.cancel()
            self.enemy.cancel()
            self._shovel_guard_active = True
            selected = dict(shovel_result)
            selected["observation"] = patrol_result["observation"]
            selected["shovel_preheat"] = shovel_preheat
            return self._result("shovel_guard", selected)

        if self._shovel_guard_active:
            self.patrol = RingPatrolController()
            self._shovel_guard_active = False
            patrol_result = self.patrol.update(
                gray_raw, now=now, healthy=healthy
            )
        if patrol_result["state"] not in HUNT_ALLOWED_PATROL_STATES:
            self.hunt.cancel()
            self.enemy.cancel()
            return self._result("patrol", patrol_result)

        hunt_result = self.hunt.update(
            vision, ir, now=now, healthy=healthy
        )
        if hunt_result["owns_control"]:
            selected = dict(hunt_result)
            selected["hunt_mode"] = hunt_result["mode"]
            selected["observation"] = patrol_result["observation"]
            selected["shovel_preheat"] = shovel_preheat
            return self._result("hunt", selected)

        enemy_result = self.enemy.update(
            ir, patrol_result["observation"], shovel,
            now=now, healthy=healthy, allow_start=vision_available,
        )
        if enemy_result["owns_control"]:
            selected = dict(enemy_result)
            selected["observation"] = patrol_result["observation"]
            selected["shovel_preheat"] = True
            return self._result("enemy_push", selected)
        return self._result("patrol", patrol_result)

    def _result(self, mode, selected):
        result = dict(selected)
        result.update({
            "mode": mode,
            "patrol_state": self.patrol.state,
            "reentry_state": self.reentry.state,
            "shovel_state": self.shovel_guard.state,
            "hunt_state": self.hunt.state,
            "enemy_state": self.enemy.state,
            "shovel_preheat": bool(result.get("shovel_preheat", False)),
        })
        return result


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join("data", "main_%s.csv" % stamp)


def run(args):
    from up_controller import UpController

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    hardware = UpController(
        poll_hz=args.hz,
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    gray_sensor = GraySensor(
        adc_reader=lambda: hardware.adc_data,
        channels=GRAY_CHANNELS,
        adc_max=GRAY_ADC_MAX,
        white_enter=GRAY_WHITE_ENTER,
    )
    digi = DigiIR(
        io_reader=lambda: hardware.io_data,
        bits=DIGI_IR_BITS,
        active_level=DIGI_IR_ACTIVE_LEVEL,
    )
    analog_sensor = IrSensor(
        adc_reader=lambda: hardware.adc_data,
        channels=IR_CHANNELS,
        adc_max=IR_ADC_MAX,
    )
    shovel_sensor = IrSensor(
        adc_reader=lambda: hardware.adc_data,
        channels=SHOVEL_IR_CHANNELS,
        adc_max=SHOVEL_ADC_MAX,
    )
    robot = RobotController()
    fields = (
        "t", "front", "rear", "left", "right",
        "zone_front", "zone_rear", "zone_left", "zone_right", "zone_score",
        "ir_front", "ir_rear", "ir_left_front", "ir_left_rear",
        "ir_right_front", "ir_right_rear", "ir_valid", "analog_diff",
        "shovel_left", "shovel_right", "shovel_valid", "shovel_preheat",
        "mode", "state", "patrol_state", "reentry_state", "shovel_state",
        "hunt_mode", "hunt_state", "hunt_target_type", "good_offset_x", "bad_offset_x",
        "near_direction", "enemy_state", "enemy_source_direction",
        "enemy_turn_direction", "enemy_slow", "enemy_confirmed",
        "vision_sequence", "vision_status",
        "reason", "left_cmd", "right_cmd", "healthy",
    )
    start = time.monotonic()
    period = 1.0 / args.hz
    vision = None
    try:
        if YOLO_DIR not in sys.path:
            sys.path.insert(0, YOLO_DIR)
        from rpi_yolo_api import VisionClient
        vision = VisionClient(command=(
            "rpi-yolo", "--report-every", "0",
            "--camera", VISION_CAMERA_DEVICE,
        )).start()
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while not args.seconds or time.monotonic() - start < args.seconds:
                loop_start = time.monotonic()
                gray_raw = gray_sensor.read_raw()
                ir_states = digi.read_states()
                analog_raw = analog_sensor.read_raw()
                shovel_raw = shovel_sensor.read_raw()
                vision_raw = vision.get_raw(max_age_ms=VISION_MAX_AGE_MS)
                healthy = hardware.healthy and not hardware.stale(PATROL_STALE_SECONDS)
                result = robot.update(
                    gray_raw, ir_states, analog_raw, shovel_raw,
                    vision=vision_raw,
                    now=loop_start, healthy=healthy,
                )
                hardware.move_cmd(result["left"], result["right"])
                observation = result["observation"]
                analog_diff = (
                    analog_raw["left"] - analog_raw["right"]
                    if analog_raw["valid"] else float("nan")
                )
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    **{name: gray_raw[name] for name in GrayRiskModel.NAMES},
                    **{"zone_" + name: round(observation["zone"][name], 4)
                       for name in GrayRiskModel.NAMES},
                    "zone_score": round(observation["zone_score"], 4),
                    **{"ir_" + name: int(ir_states[name]) for name in DIGI_IR_PINS},
                    "ir_valid": int(ir_states["valid"]),
                    "analog_diff": round(analog_diff, 1),
                    "shovel_left": shovel_raw["left"],
                    "shovel_right": shovel_raw["right"],
                    "shovel_valid": int(shovel_raw["valid"]),
                    "shovel_preheat": int(result.get("shovel_preheat", False)),
                    "mode": result["mode"],
                    "state": result["state"],
                    "patrol_state": result["patrol_state"],
                    "reentry_state": result["reentry_state"],
                    "shovel_state": result["shovel_state"],
                    "hunt_mode": result.get("hunt_mode"),
                    "hunt_state": result["hunt_state"],
                    "hunt_target_type": result.get("target_type"),
                    "good_offset_x": result.get("good_offset_x"),
                    "bad_offset_x": result.get("bad_offset_x"),
                    "near_direction": result.get("near_direction"),
                    "enemy_state": result["enemy_state"],
                    "enemy_source_direction": result.get("source_direction"),
                    "enemy_turn_direction": result.get("turn_direction"),
                    "enemy_slow": int(bool(result.get("slow", False))),
                    "enemy_confirmed": int(bool(result.get("confirmed", False))),
                    "vision_sequence": (
                        vision_raw.get("sequence") if vision_raw else None
                    ),
                    "vision_status": (
                        vision_raw.get("status") if vision_raw else "stale"
                    ),
                    "reason": result["reason"],
                    "left_cmd": result["left"],
                    "right_cmd": result["right"],
                    "healthy": int(healthy),
                })
                handle.flush()
                line = "mode=%s state=%s cmd=(%d,%d) z=%.2f" % (
                    result["mode"], result["state"],
                    result["left"], result["right"],
                    observation["zone_score"],
                )
                sys.stdout.write("\r" + line)
                sys.stdout.flush()
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print()
    finally:
        if vision is not None:
            vision.close()
        hardware.close()
    print("\n生产运行日志：%s" % args.log)


def main():
    parser = argparse.ArgumentParser(description="RoboCup 巡台与掉台回归生产程序")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--log", default=_default_log_path())
    parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
    )
    parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
