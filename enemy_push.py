#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""敌人搜索与推动状态机；只处理注入数据并返回电机命令。"""

import time

from config import (
    ENEMY_ATTACK_PAUSE_SECONDS,
    ENEMY_COOLDOWN_SECONDS,
    ENEMY_PUSH_SPEED,
    ENEMY_REAR_ABORT_ZONE,
    ENEMY_REAR_FIRST_TURN,
    ENEMY_RETREAT_SECONDS,
    ENEMY_RETREAT_SPEED,
    ENEMY_SLOW_CONFIRM,
    ENEMY_SLOW_SPEED,
    ENEMY_SLOW_ZONE,
    ENEMY_TURN_PLAN,
    ENEMY_WHITE_CONFIRM,
    ENEMY_WHITE_ZONE,
    MOTOR_TURN_CALIBRATION,
)
from shovel_guard import ShovelGuard


class EnemyPushController:
    """摄像头空闲时按数字红外找敌，正对后推动并安全收回。"""

    def __init__(
            self, guard=None, turn_calibration=MOTOR_TURN_CALIBRATION,
            turn_plan=ENEMY_TURN_PLAN,
            rear_first_turn=ENEMY_REAR_FIRST_TURN,
            push_speed=ENEMY_PUSH_SPEED, slow_speed=ENEMY_SLOW_SPEED):
        self.guard = guard or ShovelGuard()
        self.turn_calibration = turn_calibration
        self.turn_plan = dict(turn_plan)
        self.push_speed = int(push_speed)
        self.slow_speed = int(slow_speed)
        self.state = "IDLE"
        self.command = (0, 0)
        self.reason = "等待摄像头和红外空闲"
        self.source_direction = None
        self.turn_direction = None
        self.turn_until = 0.0
        self.confirmed = False
        self.slow = False
        self._slow_count = 0
        self._white_count = 0
        self._state_started = 0.0
        self._side_armed = True
        self._fallback_turn = rear_first_turn

    @property
    def active(self):
        return self.state != "IDLE"

    @staticmethod
    def _front_detected(ir):
        return bool(
            ir.get("front")
            or (ir.get("left_front") and ir.get("right_front"))
        )

    @staticmethod
    def _side_direction(ir):
        if ir.get("left_front"):
            return "left_front"
        if ir.get("right_front"):
            return "right_front"
        if ir.get("rear") or (
                ir.get("left_rear") and ir.get("right_rear")):
            return "rear"
        if ir.get("left_rear"):
            return "left_rear"
        if ir.get("right_rear"):
            return "right_rear"
        return None

    def _turn_for_source(self, source):
        direction, angle = self.turn_plan[source]
        return (self._fallback_turn if direction is None else direction), angle

    def _enter(self, state, now, reason):
        self.state = state
        self._state_started = now
        self.command = (0, 0)
        self.reason = reason
        if state == "PUSH":
            self.confirmed = False
            self.slow = False
            self._slow_count = 0
            self._white_count = 0

    def _start_turn(self, source, now):
        direction, angle = self._turn_for_source(source)
        if source == "rear":
            self._fallback_turn = (
                "left" if self._fallback_turn == "right" else "right"
            )
        speed, duration = self.turn_calibration[direction][angle]
        self.source_direction = source
        self.turn_direction = direction
        self.turn_until = now + float(duration)
        self._turn_speed = int(speed)
        self._side_armed = False
        self._enter("SEEK_TURN", now, "%s 发现敌人，原地转向" % source)

    def _start_push(self, now, reason):
        self.source_direction = "front"
        self.turn_direction = None
        self._enter("PUSH", now, reason)

    def cancel(self):
        self.state = "IDLE"
        self.command = (0, 0)
        self.reason = "控制权被更高优先级状态收回"
        self.source_direction = None
        self.turn_direction = None
        self.turn_until = 0.0
        self.confirmed = False
        self.slow = False
        self._slow_count = 0
        self._white_count = 0

    def _abort(self, now, reason):
        self._enter("COOLDOWN", now, reason)

    def _result(self, owns_control=None):
        if owns_control is None:
            owns_control = self.active
        return {
            "left": self.command[0],
            "right": self.command[1],
            "owns_control": bool(owns_control),
            "state": self.state,
            "reason": self.reason,
            "source_direction": self.source_direction,
            "turn_direction": self.turn_direction,
            "confirmed": self.confirmed,
            "slow": self.slow,
            "guard": self.guard.state,
            "hang": self.guard.hang,
        }

    def update(self, ir, observation, shovel_raw, now=None, healthy=True,
               allow_start=True):
        now = time.monotonic() if now is None else float(now)
        ir_valid = isinstance(ir, dict) and bool(ir.get("valid"))
        if not healthy or not ir_valid:
            if self.active:
                self._abort(now, "传感器无效或数据过期，停止敌人动作")
                return self._result()
            return self._result(False)

        front = self._front_detected(ir)
        side = self._side_direction(ir)
        if not front and side is None:
            self._side_armed = True

        if self.state == "IDLE":
            if not allow_start:
                self.reason = "摄像头正在处理能量块"
                return self._result(False)
            if front:
                self._start_push(now, "正前红外发现敌人，开始推动")
                return self._result()
            if side is not None and self._side_armed:
                self._start_turn(side, now)
                return self._result()
            self.reason = "没有敌人目标"
            return self._result(False)

        guard_result = self.guard.update(
            shovel_raw, active=True, now=now, healthy=healthy
        )

        if self.state == "SEEK_TURN":
            if guard_result["state"] != "IDLE":
                self._enter("RETREAT", now, "转向时铲子悬空，倒车收回")
                return self._result()
            if front:
                self._start_push(now, "敌人已进入正前方，开始推动")
                return self._result()
            if now >= self.turn_until:
                self._enter("IDLE", now, "定角转向完成，释放控制权")
                self.source_direction = None
                self.turn_direction = None
                return self._result(False)
            speed = self._turn_speed
            self.command = (
                (-speed, speed) if self.turn_direction == "left"
                else (speed, -speed)
            )
            return self._result()

        zone = (observation or {}).get("zone") or {}
        zone_valid = all(name in zone for name in ("front", "rear", "left", "right"))

        if self.state == "PUSH":
            if not zone_valid:
                self._abort(now, "灰度观测无效，中断推动")
                return self._result()
            if zone["rear"] <= ENEMY_REAR_ABORT_ZONE:
                self._abort(now, "后路接近台外，中断推动")
                return self._result()
            if guard_result["state"] in ("HANGED", "REVERSE"):
                self.confirmed = True
                self._enter("RETREAT", now, "铲子悬空，敌人已推出并倒车收回")
                return self._result()
            if min(zone.values()) >= ENEMY_WHITE_ZONE:
                self._white_count += 1
            else:
                self._white_count = 0
            if self._white_count >= ENEMY_WHITE_CONFIRM:
                self._enter("RETREAT", now, "白边保护触发，停止推动并倒车")
                return self._result()
            if zone["front"] < ENEMY_SLOW_ZONE:
                self._slow_count += 1
            else:
                self._slow_count = 0
            if self._slow_count >= ENEMY_SLOW_CONFIRM:
                self.slow = True
            speed = self.slow_speed if self.slow else self.push_speed
            self.command = (speed, speed)
            return self._result()

        if self.state == "RETREAT":
            if not zone_valid or zone.get("rear", -1.0) <= ENEMY_REAR_ABORT_ZONE:
                self._abort(now, "倒车方向不安全，停车冷却")
                return self._result()
            if guard_result["state"] == "SAFE_STOP":
                self._abort(now, "铲子倒车收回超时")
                return self._result()
            if (guard_result["state"] == "IDLE"
                    and now - self._state_started >= ENEMY_RETREAT_SECONDS):
                self._enter("PAUSE", now, "倒车收回完成，攻击间隔停车")
                return self._result()
            self.command = (-ENEMY_RETREAT_SPEED, -ENEMY_RETREAT_SPEED)
            return self._result()

        if self.state == "PAUSE":
            if now - self._state_started >= ENEMY_ATTACK_PAUSE_SECONDS:
                self._enter("IDLE", now, "攻击间隔结束，释放控制权")
                return self._result(False)
            return self._result()

        if self.state == "COOLDOWN":
            if now - self._state_started >= ENEMY_COOLDOWN_SECONDS:
                self._enter("IDLE", now, "安全冷却结束，释放控制权")
                return self._result(False)
            return self._result()

        self.cancel()
        return self._result(False)
