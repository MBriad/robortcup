#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""擂台巡台策略：暗外圈提前回中，白边触发锁存式紧急脱离。"""

import argparse
import csv
import os
import time

from config import (
    PATROL_COMMAND_LIMIT,
    PATROL_CRUISE_LINEAR,
    PATROL_CRUISE_TURN,
    PATROL_NEAR_CONFIRM,
    PATROL_RECENTER_LINEAR,
    PATROL_RECENTER_MAX_SECONDS,
    PATROL_RECENTER_MIN_SECONDS,
    PATROL_RECENTER_TURN,
    PATROL_SLOW_LINEAR,
    PATROL_SLOW_TURN,
    PATROL_STALE_SECONDS,
    PATROL_WHITE_CONFIRM,
    PATROL_WHITE_LINEAR,
    PATROL_WHITE_MAX_SECONDS,
    PATROL_WHITE_MIN_SECONDS,
    PATROL_WHITE_TURN,
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
        self._turn_sign = 1.0

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

    def _risk_vector(self, observation, white):
        if white:
            levels = {
                name: max(0.0, observation["white"][name])
                for name in GrayRiskModel.NAMES
            }
            for name in levels:
                levels[name] = levels[name] if name in observation["white_hits"] else 0.0
        else:
            levels = {
                name: max(0.0, 0.50 - observation["zone"][name])
                for name in GrayRiskModel.NAMES
            }
        return levels

    def _escape_command(self, observation, white):
        levels = self._risk_vector(observation, white)
        linear_signal = levels["rear"] - levels["front"]
        turn_signal = levels["left"] - levels["right"]
        linear_speed = PATROL_WHITE_LINEAR if white else PATROL_RECENTER_LINEAR
        turn_speed = PATROL_WHITE_TURN if white else PATROL_RECENTER_TURN

        linear = 0.0
        if abs(linear_signal) > 0.05:
            linear = linear_speed if linear_signal > 0.0 else -linear_speed
        if abs(turn_signal) > 0.05:
            turn = turn_speed if turn_signal > 0.0 else -turn_speed
        else:
            turn = turn_speed * self._turn_sign
            self._turn_sign *= -1.0
        return self._mix(linear, turn)

    def _enter(self, state, now, command, reason):
        self.state = state
        self.state_started = now
        self.command = command
        self.reason = reason

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

        if self._white_count >= PATROL_WHITE_CONFIRM and self.state != "WHITE_ESCAPE":
            hits = "/".join(observation["white_hits"])
            self._enter("WHITE_ESCAPE", now, self._escape_command(observation, True),
                        "压到白边：%s" % hits)

        elapsed = now - self.state_started
        if self.state == "WHITE_ESCAPE":
            if elapsed >= PATROL_WHITE_MIN_SECONDS and observation["white_clear"]:
                self._enter("RECENTER", now, self._escape_command(observation, False),
                            "离开白边后继续回中")
            elif elapsed >= PATROL_WHITE_MAX_SECONDS:
                self._enter("RECENTER", now, self._mix(0, PATROL_RECENTER_TURN * self._turn_sign),
                            "白边脱离超时，原地换向搜索")
                self._turn_sign *= -1.0
            return self._result(observation)

        if self.state == "RECENTER":
            if elapsed >= PATROL_RECENTER_MIN_SECONDS and observation["near_clear"]:
                self._enter("CRUISE", now, self._mix(PATROL_CRUISE_LINEAR, PATROL_CRUISE_TURN),
                            "已回到安全内圈")
            elif elapsed >= PATROL_RECENTER_MAX_SECONDS:
                self._enter("RECENTER", now, self._escape_command(observation, False),
                            "仍在暗外圈，重新选择回中方向")
            return self._result(observation)

        if self._near_count >= PATROL_NEAR_CONFIRM:
            self._enter("RECENTER", now, self._escape_command(observation, False),
                        "进入暗外圈")
        elif observation["zone_score"] < 0.60:
            self._enter("SLOW_CRUISE", now, self._mix(PATROL_SLOW_LINEAR, PATROL_SLOW_TURN),
                        "接近外圈，降速巡航")
        else:
            self._enter("CRUISE", now, self._mix(PATROL_CRUISE_LINEAR, PATROL_CRUISE_TURN),
                        "安全内圈顺时针巡航")
        return self._result(observation)

    def _result(self, observation):
        return {
            "left": self.command[0],
            "right": self.command[1],
            "state": self.state,
            "reason": self.reason,
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
    fields = ("t", "front", "rear", "left", "right", "zone_score", "white_hits",
              "state", "reason", "left_cmd", "right_cmd", "healthy")
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
                    "zone_score": round(observation["zone_score"], 4),
                    "white_hits": "/".join(observation["white_hits"]),
                    "state": result["state"],
                    "reason": result["reason"],
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
    parser.add_argument("--motor-invert", action="store_true")
    parser.add_argument("--motor-swap", action="store_true")
    args = parser.parse_args()
    if not DEV_MODE:
        print("DEV_MODE=False：请在生产程序中注入灰度值并调用 RingPatrolController.update")
        return
    run_dev(args)


if __name__ == "__main__":
    main()
