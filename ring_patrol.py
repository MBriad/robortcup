#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""擂台巡台策略：灰度变暗时移动避让，危险边缘只直线撤离或停车。"""

import argparse
import csv
import os
import time

from config import (
    CHASSIS_MOTOR_INVERT,
    CHASSIS_MOTOR_SWAP,
    MOTOR_TURN_CALIBRATION,
    PATROL_COMMAND_LIMIT,
    PATROL_CRUISE_LINEAR,
    PATROL_CRUISE_TURN,
    PATROL_EDGE_AVOID_LINEAR,
    PATROL_EDGE_AVOID_TURN,
    PATROL_EDGE_ARC_CHECK_SECONDS,
    PATROL_EDGE_TURN_ANGLE,
    PATROL_FAST_ZONE_SCORE,
    PATROL_MEDIUM_LINEAR,
    PATROL_MEDIUM_TURN,
    PATROL_SMALL_TURN_ZONE_SCORE,
    PATROL_NEAR_CONFIRM,
    PATROL_RECOVER_MIN_IMPROVEMENT,
    PATROL_RECOVER_SECONDS,
    PATROL_RECOVER_SPEED,
    PATROL_STALE_SECONDS,
    PATROL_WHITE_CONFIRM,
    PATROL_WHITE_ESCAPE_SECONDS,
    PATROL_WHITE_ESCAPE_SPEED,
)
from gray import GrayRiskModel, GraySensor


DEV_MODE = True


class RingPatrolController:
    """输入四路灰度，输出左右轮速度；不持有硬件。"""

    def __init__(self):
        self.model = GrayRiskModel()
        self.state = "WARMUP"
        self.state_started = 0.0
        self.command = (0, 0)
        self.reason = "等待滤波窗口"
        self._white_count = 0
        self._near_count = 0
        self.turn_angle = 0.0
        self.turn_direction = ""
        self.turn_duration = 0.0
        self.risk_sensor = ""
        self._avoid_start_zone = 0.0
        self._avoid_turn_sign = 0.0
        self._turn_sign = 1.0
        self._recover_command = (0, 0)
        self._recover_start_zone = 0.0

    @staticmethod
    def _mix(linear, turn):
        left = float(linear + turn)
        right = float(linear - turn)
        peak = max(abs(left), abs(right), 1.0)
        if peak > PATROL_COMMAND_LIMIT:
            scale = PATROL_COMMAND_LIMIT / peak
            left *= scale
            right *= scale
        return int(round(left)), int(round(right))

    @staticmethod
    def _dominant_risk(observation):
        return min(GrayRiskModel.NAMES, key=lambda name: observation["zone"][name])

    @staticmethod
    def _risk_signals(observation):
        zone = observation["zone"]
        return (
            zone["front"] - zone["rear"],
            zone["left"] - zone["right"],
        )

    def _start_edge_avoid(self, observation, now, reason):
        linear_signal, turn_signal = self._risk_signals(observation)
        if abs(linear_signal) >= abs(turn_signal):
            dominant = "rear" if linear_signal > 0.0 else "front"
        else:
            dominant = "right" if turn_signal > 0.0 else "left"
        continuing = self.state == "EDGE_AVOID" and self.risk_sensor == dominant
        self.risk_sensor = dominant
        if dominant == "front":
            command = (-PATROL_RECOVER_SPEED, -PATROL_RECOVER_SPEED)
        elif dominant == "rear":
            command = (PATROL_RECOVER_SPEED, PATROL_RECOVER_SPEED)
        else:
            if not continuing:
                self._avoid_turn_sign = -1.0 if turn_signal < 0.0 else 1.0
            turn = self._avoid_turn_sign * PATROL_EDGE_AVOID_TURN
            command = self._mix(PATROL_EDGE_AVOID_LINEAR, turn)
        if not continuing:
            self._avoid_start_zone = observation["zone_score"]
            self._enter("EDGE_AVOID", now, command, reason)
        else:
            self.command = command
            self.reason = reason
        if dominant in ("left", "right"):
            self.turn_direction = "right" if self._avoid_turn_sign > 0.0 else "left"

    def _start_edge_turn(self, observation, now, reason):
        _, turn_signal = self._risk_signals(observation)
        if abs(turn_signal) > 0.05:
            sign = -1.0 if turn_signal > 0.0 else 1.0
        else:
            sign = self._turn_sign
            self._turn_sign *= -1.0
        speed, duration = MOTOR_TURN_CALIBRATION[PATROL_EDGE_TURN_ANGLE]
        self.risk_sensor = self._dominant_risk(observation)
        self._enter("EDGE_TURN", now, self._mix(0, sign * speed), reason)
        self.turn_angle = PATROL_EDGE_TURN_ANGLE
        self.turn_direction = "right" if sign > 0.0 else "left"
        self.turn_duration = duration

    def _start_white_response(self, observation, now):
        hits = set(observation["white_hits"])
        self.risk_sensor = self._dominant_risk(observation)
        if "front" in hits and "rear" not in hits:
            command = (-PATROL_WHITE_ESCAPE_SPEED, -PATROL_WHITE_ESCAPE_SPEED)
            self._enter("WHITE_ESCAPE", now, command, "前方白边，先后退")
        elif "rear" in hits and "front" not in hits:
            command = (PATROL_WHITE_ESCAPE_SPEED, PATROL_WHITE_ESCAPE_SPEED)
            self._enter("WHITE_ESCAPE", now, command, "后方白边，先前进")
        else:
            self._start_edge_turn(observation, now, "侧向或多方向白边，直接转 180 度")

    def _start_recover(self, observation, now, forward, reason):
        self._recover_start_zone = observation["zone_score"]
        speed = PATROL_RECOVER_SPEED if forward else -PATROL_RECOVER_SPEED
        self._recover_command = (speed, speed)
        state = "RECOVER_FORWARD" if forward else "RECOVER_BACKWARD"
        self._enter(state, now, self._recover_command, reason)

    def _start_cruise_for_zone(self, observation, now, reason):
        if observation["zone_score"] < PATROL_SMALL_TURN_ZONE_SCORE:
            self._start_edge_avoid(observation, now, "灰度趋势变暗，提前避让")
        elif observation["zone_score"] < PATROL_FAST_ZONE_SCORE:
            self.risk_sensor = ""
            self._enter("MEDIUM_CRUISE", now,
                        self._mix(PATROL_MEDIUM_LINEAR, PATROL_MEDIUM_TURN), reason)
        else:
            self.risk_sensor = ""
            self._enter("CRUISE", now,
                        self._mix(PATROL_CRUISE_LINEAR, PATROL_CRUISE_TURN), reason)

    def _enter(self, state, now, command, reason):
        self.state = state
        self.state_started = now
        self.command = command
        self.reason = reason
        self.turn_angle = 0.0
        self.turn_direction = ""
        self.turn_duration = 0.0

    def update(self, raw, now=None, healthy=True):
        now = time.monotonic() if now is None else float(now)
        observation = self.model.update(raw)

        if not healthy or not observation["valid"]:
            self._enter("SENSOR_STOP", now, (0, 0), "传感器无效或数据过期")
            return self._result(observation)
        if not observation["ready"]:
            self._enter("WARMUP", now, (0, 0), "等待滤波窗口")
            return self._result(observation)

        self._white_count = self._white_count + 1 if observation["white_hits"] else 0
        self._near_count = self._near_count + 1 if observation["near_edge"] else 0

        if self.state == "SAFE_STOP":
            if observation["white_clear"] and observation["near_clear"]:
                self._start_cruise_for_zone(observation, now, "人工移回安全区，恢复巡航")
            return self._result(observation)

        if (self._white_count >= PATROL_WHITE_CONFIRM and
                self.state not in (
                    "WHITE_ESCAPE", "EDGE_TURN",
                    "RECOVER_FORWARD", "RECOVER_BACKWARD",
                )):
            self._start_white_response(observation, now)
            if self.state == "SAFE_STOP":
                return self._result(observation)

        elapsed = now - self.state_started
        if self.state == "WHITE_ESCAPE":
            if elapsed >= PATROL_WHITE_ESCAPE_SECONDS:
                if observation["white_clear"] and observation["near_clear"]:
                    self._start_cruise_for_zone(observation, now, "白边直线脱离完成")
                else:
                    self._start_recover(
                        observation,
                        now,
                        self.command[0] > 0,
                        "白边后继续直线脱离，不在边缘转向",
                    )
            return self._result(observation)

        if self.state == "EDGE_TURN":
            if elapsed >= self.turn_duration:
                self._start_recover(
                    observation, now, True, "180 度转向完成，向前离开边缘"
                )
            return self._result(observation)

        if self.state in ("RECOVER_FORWARD", "RECOVER_BACKWARD"):
            if elapsed >= PATROL_RECOVER_SECONDS:
                improvement = observation["zone_score"] - self._recover_start_zone
                if observation["near_clear"]:
                    self._start_cruise_for_zone(observation, now, "已脱离暗外圈")
                elif improvement >= PATROL_RECOVER_MIN_IMPROVEMENT:
                    self._start_recover(
                        observation,
                        now,
                        self.state == "RECOVER_FORWARD",
                        "方向有效，继续低速脱离",
                    )
                else:
                        self._enter(
                            "SAFE_STOP",
                            now,
                            (0, 0),
                            "大转后直线脱离仍未改善，停车等待",
                        )
            return self._result(observation)

        avoid_check_seconds = (
            PATROL_EDGE_ARC_CHECK_SECONDS
            if self.risk_sensor in ("left", "right")
            else PATROL_RECOVER_SECONDS
        )
        if self.state == "EDGE_AVOID" and elapsed >= avoid_check_seconds:
            improvement = observation["zone_score"] - self._avoid_start_zone
            if improvement < PATROL_RECOVER_MIN_IMPROVEMENT:
                self._start_edge_turn(
                    observation, now, "小转未改善，直接大转 180 度"
                )
                return self._result(observation)
            self._avoid_start_zone = observation["zone_score"]
            self.state_started = now

        if self._near_count >= PATROL_NEAR_CONFIRM:
            self._start_edge_turn(observation, now, "进入危险区，直接大转 180 度")
        else:
            self._start_cruise_for_zone(observation, now, "按区域分级巡航")
        return self._result(observation)

    def _result(self, observation):
        linear_risk, turn_risk = self._risk_signals(observation)
        speed_level = {
            "CRUISE": "fast",
            "MEDIUM_CRUISE": "medium",
            "EDGE_AVOID": "slow",
            "EDGE_TURN": "danger_turn",
            "WHITE_ESCAPE": "danger",
            "RECOVER_FORWARD": "danger",
            "RECOVER_BACKWARD": "danger",
            "SAFE_STOP": "stop",
            "SENSOR_STOP": "stop",
            "WARMUP": "stop",
        }.get(self.state, "stop")
        return {
            "left": self.command[0],
            "right": self.command[1],
            "state": self.state,
            "reason": self.reason,
            "speed_level": speed_level,
            "turn_angle": self.turn_angle,
            "turn_direction": self.turn_direction,
            "turn_duration": self.turn_duration,
            "risk_sensor": self.risk_sensor,
            "linear_risk": linear_risk,
            "turn_risk": turn_risk,
            "observation": observation,
        }


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join("data", "patrol_%s.csv" % stamp)


def run_dev(args):
    from up_controller import UpController

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    controller = UpController(
        poll_hz=args.hz,
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    sensor = GraySensor(adc_reader=lambda: controller.adc_data)
    patrol = RingPatrolController()
    fields = ("t", "front", "rear", "left", "right", "zone_front", "zone_rear",
              "zone_left", "zone_right", "zone_score", "white_hits", "state",
              "reason", "risk_sensor", "linear_risk", "turn_risk", "near_edge",
              "speed_level", "turn_angle", "turn_direction",
              "turn_duration", "left_cmd", "right_cmd", "healthy")
    start = time.monotonic()
    period = 1.0 / args.hz
    try:
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while not args.seconds or time.monotonic() - start < args.seconds:
                loop_start = time.monotonic()
                raw = sensor.read_raw()
                healthy = controller.healthy and not controller.stale(PATROL_STALE_SECONDS)
                result = patrol.update(raw, now=loop_start, healthy=healthy)
                controller.move_cmd(result["left"], result["right"])
                observation = result["observation"]
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    **{name: raw[name] for name in GrayRiskModel.NAMES},
                    **{"zone_" + name: round(observation["zone"][name], 4)
                       for name in GrayRiskModel.NAMES},
                    "zone_score": round(observation["zone_score"], 4),
                    "white_hits": "/".join(observation["white_hits"]),
                    "state": result["state"],
                    "reason": result["reason"],
                    "risk_sensor": result["risk_sensor"],
                    "linear_risk": round(result["linear_risk"], 4),
                    "turn_risk": round(result["turn_risk"], 4),
                    "near_edge": int(observation["near_edge"]),
                    "speed_level": result["speed_level"],
                    "turn_angle": result["turn_angle"],
                    "turn_direction": result["turn_direction"],
                    "turn_duration": result["turn_duration"],
                    "left_cmd": result["left"],
                    "right_cmd": result["right"],
                    "healthy": int(healthy),
                })
                handle.flush()
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
    print("巡台日志：%s" % args.log)


def main():
    parser = argparse.ArgumentParser(description="灰度闭环擂台巡台")
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
    args = parser.parse_args()
    if not DEV_MODE:
        print("DEV_MODE=False：请在生产程序中注入灰度值并调用 RingPatrolController.update")
        return
    run_dev(args)


if __name__ == "__main__":
    main()
