#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡台状态机独立真机测试；生产比赛请运行根目录 main.py。"""

import argparse
from collections import deque
import csv
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT in sys.path:
    sys.path.remove(ROOT)
sys.path.insert(0, ROOT)

from config import (  # noqa: E402
    CHASSIS_MOTOR_INVERT,
    CHASSIS_MOTOR_SWAP,
    GRAY_ADC_MAX,
    GRAY_CHANNELS,
    GRAY_WHITE_ENTER,
    PATROL_COMMAND_LIMIT,
    PATROL_EDGE_TURN_ANGLE,
    PATROL_MIN_ACTIVE_SPEED,
    PATROL_STALE_SECONDS,
)
from gray import GrayRiskModel, GraySensor  # noqa: E402
from ring_patrol import RingPatrolController  # noqa: E402


def _default_log_path():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(ROOT, "data", "patrol_%s.csv" % stamp)


class TrialMarker:
    """保存人工试验标签，供采集循环无阻塞地写入 CSV。"""

    def __init__(self, trial_id=1, pose=""):
        self._lock = threading.Lock()
        self._events = deque(["trial_start"])
        self.trial_id = int(trial_id)
        self.pose = pose
        self.fallen = False

    def next_trial(self, pose=""):
        with self._lock:
            self.trial_id += 1
            if pose:
                self.pose = pose
            self.fallen = False
            self._events.append("trial_start")

    def set_pose(self, pose):
        with self._lock:
            self.pose = pose
            self._events.append("pose_changed")

    def mark(self, event, fallen=False):
        with self._lock:
            self.fallen = self.fallen or fallen
            self._events.append(event)

    def snapshot(self):
        with self._lock:
            event = self._events.popleft() if self._events else ""
            return {
                "trial_id": self.trial_id,
                "pose": self.pose,
                "event": event,
                "fallen": int(self.fallen),
            }


def _read_markers(marker):
    print("CSV 标记：n [姿态]=下一次试验，p <姿态>=改姿态，e <事件>=事件，f=掉台")
    while True:
        try:
            line = input().strip()
        except (EOFError, KeyboardInterrupt):
            return
        if not line:
            continue
        command, _, value = line.partition(" ")
        command = command.lower()
        value = value.strip()
        if command == "n":
            marker.next_trial(value)
        elif command == "p" and value:
            marker.set_pose(value)
        elif command == "e" and value:
            marker.mark(value)
        elif command == "f":
            marker.mark("fallen", fallen=True)
        else:
            print("未知标记；使用 n [姿态] / p <姿态> / e <事件> / f")


def replay(path):
    """在 PC 上回放巡台 CSV，不访问硬件。"""
    patrol = RingPatrolController()
    counts = {}
    violations = []
    previous_state = None
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

            healthy = True
            if row.get("healthy") is not None:
                try:
                    healthy = float(row["healthy"]) > 0.0
                except (TypeError, ValueError):
                    healthy = False

            result = patrol.update(raw, now=now, healthy=healthy)
            state = result["state"]
            counts[state] = counts.get(state, 0) + 1
            if state != previous_state:
                print("  t=%.2f %s（%s）" % (now, state, result["reason"]))
                previous_state = state

            for command in (result["left"], result["right"]):
                if command and abs(command) < PATROL_MIN_ACTIVE_SPEED:
                    violations.append(
                        "低速指令 %d（<%d 电机死区）"
                        % (command, PATROL_MIN_ACTIVE_SPEED)
                    )
                if abs(command) > PATROL_COMMAND_LIMIT:
                    violations.append("越限指令 %d" % command)
            if (
                state == "EDGE_TURN"
                and result["turn_angle"] != PATROL_EDGE_TURN_ANGLE
            ):
                violations.append(
                    "转向角 %.0f 非 %.0f"
                    % (result["turn_angle"], PATROL_EDGE_TURN_ANGLE)
                )

    cycle = {
        "CRUISE", "EDGE_AVOID", "EDGE_TURN", "RECOVER_FORWARD",
    } <= set(counts)
    print("状态计数：%s" % counts)
    print(
        "完整循环（巡航→退离→转向→回中）：%s"
        % ("通过" if cycle else "未出现完整循环")
    )
    print(
        "指令/角度检查：%s"
        % ("通过" if not violations else "；".join(violations[:5]))
    )
    return cycle and not violations


def run(args):
    from up_controller import UpController

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    hardware = UpController(
        poll_hz=args.hz,
        motor_invert=args.motor_invert,
        motor_swap=args.motor_swap,
    )
    sensor = GraySensor(
        adc_reader=lambda: hardware.adc_data,
        channels=GRAY_CHANNELS,
        adc_max=GRAY_ADC_MAX,
        white_enter=GRAY_WHITE_ENTER,
    )
    patrol = RingPatrolController()
    marker = TrialMarker(args.trial_id, args.pose)
    if sys.stdin.isatty():
        threading.Thread(target=_read_markers, args=(marker,), daemon=True).start()
    fields = (
        "t", "front", "rear", "left", "right", "zone_front", "zone_rear",
        "zone_left", "zone_right", "zone_score", "white_hits", "state",
        "reason", "risk_sensor", "linear_risk", "turn_risk", "near_edge",
        "speed_level", "turn_angle", "turn_direction", "turn_duration",
        "left_cmd", "right_cmd", "healthy", "trial_id", "pose", "event",
        "fallen", "control_mode", "shovel_preheat",
    )
    start = time.monotonic()
    period = 1.0 / args.hz
    try:
        with open(args.log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while not args.seconds or time.monotonic() - start < args.seconds:
                loop_start = time.monotonic()
                raw = sensor.read_raw()
                healthy = hardware.healthy and not hardware.stale(PATROL_STALE_SECONDS)
                result = patrol.update(raw, now=loop_start, healthy=healthy)
                hardware.move_cmd(result["left"], result["right"])
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
                    **marker.snapshot(),
                    "control_mode": args.control_mode,
                    "shovel_preheat": int(result["shovel_preheat"]),
                })
                handle.flush()
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        pass
    finally:
        hardware.close()
    print("巡台测试日志：%s" % args.log)


def main():
    parser = argparse.ArgumentParser(description="巡台状态机独立真机测试")
    parser.add_argument("--replay", metavar="CSV", help="PC 回放巡台 CSV，不动电机")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--log", default=_default_log_path())
    parser.add_argument("--trial-id", type=int, default=1)
    parser.add_argument("--pose", default="", help="本次试验的初始车身姿态")
    parser.add_argument(
        "--control-mode",
        default="patrol",
        help="控制方式，例如 patrol、manual_placement 或 manual_motor",
    )
    parser.add_argument("--yes", action="store_true", help="跳过真机安全确认")
    parser.set_defaults(
        motor_invert=CHASSIS_MOTOR_INVERT,
        motor_swap=CHASSIS_MOTOR_SWAP,
    )
    parser.add_argument("--motor-invert", dest="motor_invert", action="store_true")
    parser.add_argument("--no-motor-invert", dest="motor_invert", action="store_false")
    parser.add_argument("--motor-swap", dest="motor_swap", action="store_true")
    parser.add_argument("--no-motor-swap", dest="motor_swap", action="store_false")
    args = parser.parse_args()
    if args.replay:
        raise SystemExit(0 if replay(args.replay) else 1)
    if not args.yes:
        confirmed = input("车已放台上或架起，确认安全后输入 PATROL：").strip()
        if confirmed != "PATROL":
            print("未完成安全确认，启动取消。")
            return
    run(args)


if __name__ == "__main__":
    main()
