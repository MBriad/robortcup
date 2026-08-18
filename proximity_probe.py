#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""敌人搜索与推动状态机；只处理注入数据并返回电机命令。"""

import time

from config import (
    ENEMY_BAD_INTERRUPT_FRAMES,
    ENEMY_COOLDOWN_SECONDS,
    ENEMY_PUSH_SPEED,
    ENEMY_REAR_ABORT_ZONE,
    ENEMY_SLOW_CONFIRM,
    ENEMY_SLOW_SPEED,
    ENEMY_SLOW_ZONE,
    MOTOR_TURN_CALIBRATION,
    PROBE_BRAKE_SECONDS,
    PROBE_IR_REARM_CLEAR_FRAMES,
    PROBE_REAR_FIRST_TURN,
    PROBE_TURN_PLAN,
    PROBE_VISION_CONFIRM_FRAMES,
    PROBE_VISION_WAIT_TIMEOUT,
)


class ProximityProbeController:
    """数字红外发现近物后转向并交给视觉分类；确认无能量块后才推动敌人。"""

    def __init__(
            self, turn_calibration=MOTOR_TURN_CALIBRATION,
            turn_plan=PROBE_TURN_PLAN,
            rear_first_turn=PROBE_REAR_FIRST_TURN,
            push_speed=ENEMY_PUSH_SPEED, slow_speed=ENEMY_SLOW_SPEED,
            bad_interrupt_frames=ENEMY_BAD_INTERRUPT_FRAMES,
            vision_confirm_frames=PROBE_VISION_CONFIRM_FRAMES,
            vision_wait_timeout=PROBE_VISION_WAIT_TIMEOUT,
            ir_rearm_clear_frames=PROBE_IR_REARM_CLEAR_FRAMES,
            brake_seconds=PROBE_BRAKE_SECONDS):
        if int(vision_confirm_frames) < 1:
            raise ValueError("敌人视觉确认帧数必须为正")
        if int(bad_interrupt_frames) < 1:
            raise ValueError("bad 打断敌人推进帧数必须为正")
        if float(vision_wait_timeout) <= 0.0:
            raise ValueError("敌人视觉等待超时必须为正")
        if int(ir_rearm_clear_frames) < 1:
            raise ValueError("红外重新布防清除帧数必须为正")
        if float(brake_seconds) < 0.0:
            raise ValueError("刹车归0时长不能为负")
        self.turn_calibration = turn_calibration
        self.turn_plan = dict(turn_plan)
        self.push_speed = int(push_speed)
        self.slow_speed = int(slow_speed)
        self.bad_interrupt_frames = int(bad_interrupt_frames)
        self.vision_confirm_frames = int(vision_confirm_frames)
        self.vision_wait_timeout = float(vision_wait_timeout)
        self.ir_rearm_clear_frames = int(ir_rearm_clear_frames)
        self.brake_seconds = float(brake_seconds)
        self.state = "IDLE"
        self.command = (0, 0)
        self.reason = "等待摄像头和红外空闲"
        self.source_direction = None
        self.turn_direction = None
        self.turn_until = 0.0
        self.confirmed = False
        self.slow = False
        self._slow_count = 0
        self._bad_interrupt_count = 0
        self._last_bad_interrupt_sequence = None
        self._state_started = 0.0
        self._side_armed = True
        self._front_armed = True
        self._side_clear_count = 0
        self._front_clear_count = 0
        self._fallback_turn = rear_first_turn
        self._brake_source = None
        self._brake_until = 0.0
        self.vision_count = 0
        self.vision_verdict = "idle"
        self._last_vision_sequence = None
        self._vision_wait_requires_front = True

    @property
    def active(self):
        return self.state != "IDLE"

    @property
    def bad_interrupt_count(self):
        return self._bad_interrupt_count

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
        if state == "ENEMY_PUSH":
            self.confirmed = False
            self.slow = False
            self._slow_count = 0
            self._bad_interrupt_count = 0
            self._last_bad_interrupt_sequence = None

    def _start_brake(self, source, now):
        """检测到侧向近物后先刹车归0，停稳再按原地标定转向。"""
        self._brake_source = source
        self.source_direction = source
        self._brake_until = now + self.brake_seconds
        self._side_armed = False
        self._side_clear_count = 0
        self._enter("PROBE_BRAKE", now, "%s 发现近物候选，先刹车归0再转向" % source)

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
        self._enter("PROBE_TURN", now, "%s 发现近物候选，原地转向" % source)
        self.command = (
            (-self._turn_speed, self._turn_speed)
            if self.turn_direction == "left"
            else (self._turn_speed, -self._turn_speed)
        )

    def _start_push(self, now, reason):
        self.source_direction = "front"
        self.turn_direction = None
        self.vision_verdict = "enemy_confirmed"
        self._enter("ENEMY_PUSH", now, reason)

    def _start_vision_wait(self, now, vision_sequence, require_front=True):
        self.source_direction = "front"
        self.turn_direction = None
        self.vision_count = 0
        self.vision_verdict = "waiting"
        self._last_vision_sequence = vision_sequence
        self._vision_wait_requires_front = bool(require_front)
        self._enter(
            "PROBE_VISION_WAIT", now,
            "近物体已停车，等待新视觉帧排除 good/bad",
        )

    def cancel(self):
        self.state = "IDLE"
        self.command = (0, 0)
        self.reason = "控制权被更高优先级状态收回"
        self.source_direction = None
        self.turn_direction = None
        self.turn_until = 0.0
        self._brake_source = None
        self._brake_until = 0.0
        self.confirmed = False
        self.slow = False
        self._slow_count = 0
        self._bad_interrupt_count = 0
        self._last_bad_interrupt_sequence = None
        self.vision_count = 0
        self.vision_verdict = "idle"
        self._last_vision_sequence = None
        self._vision_wait_requires_front = True

    def finish_push(self):
        """结束本次推动；前方红外解除前禁止再次推动同一目标。"""
        was_push = self.state == "ENEMY_PUSH"
        self.cancel()
        self._front_armed = False
        self._front_clear_count = 0
        self.confirmed = was_push
        self.vision_verdict = "push_finished"
        self.reason = "推动结束，等待前方目标离开后重新布防"

    def _abort(self, now, reason):
        self._enter("COOLDOWN", now, reason)

    def _update_rearm(self, front, side):
        if front:
            self._front_clear_count = 0
        elif not self._front_armed:
            self._front_clear_count += 1
            if self._front_clear_count >= self.ir_rearm_clear_frames:
                self._front_armed = True
                self._front_clear_count = 0

        if side is not None:
            self._side_clear_count = 0
        elif not self._side_armed:
            self._side_clear_count += 1
            if self._side_clear_count >= self.ir_rearm_clear_frames:
                self._side_armed = True
                self._side_clear_count = 0

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
            "vision_count": self.vision_count,
            "vision_verdict": self.vision_verdict,
            "bad_interrupt_count": self.bad_interrupt_count,
        }

    def update(self, ir, observation, vision=None, now=None, healthy=True,
               allow_start=True):
        now = time.monotonic() if now is None else float(now)
        vision = vision if isinstance(vision, dict) else {}
        vision_sequence = vision.get("sequence")
        vision_status = vision.get("status")
        vision_has_good = bool(vision.get("has_good", False))
        vision_has_bad = bool(vision.get("has_bad", False))
        ir_valid = isinstance(ir, dict) and bool(ir.get("valid"))
        if not healthy or not ir_valid:
            if self.active:
                self._abort(now, "传感器无效或数据过期，停止敌人动作")
                return self._result()
            return self._result(False)

        front = self._front_detected(ir)
        side = self._side_direction(ir)
        self._update_rearm(front, side)

        if self.state == "IDLE":
            if not allow_start:
                self.reason = "摄像头正在处理能量块"
                return self._result(False)
            if front and self._front_armed:
                self._start_vision_wait(now, vision_sequence)
                return self._result()
            if front:
                self.reason = "等待前方目标离开后重新允许推动"
                return self._result(False)
            if side is not None and self._side_armed:
                self._start_brake(side, now)
                return self._result()
            self.reason = "没有近物候选"
            return self._result(False)

        if self.state == "PROBE_BRAKE":
            if vision_has_good:
                self.cancel()
                self._side_armed = False
                self._side_clear_count = 0
                self.reason = "刹车期间视觉识别为 good，取消近物候选"
                self.vision_verdict = "good"
                return self._result(False)
            if front:
                self._start_vision_wait(now, vision_sequence)
                return self._result()
            if now >= self._brake_until:
                self._start_turn(self._brake_source, now)
                return self._result()
            return self._result()

        if self.state == "PROBE_TURN":
            if vision_has_good:
                self.cancel()
                self._side_armed = False
                self._side_clear_count = 0
                self.reason = "转向期间视觉识别为 good，取消近物候选"
                self.vision_verdict = "good"
                return self._result(False)
            if front:
                self._start_vision_wait(now, vision_sequence)
                return self._result()
            if now >= self.turn_until:
                self._start_vision_wait(
                    now, vision_sequence, require_front=False,
                )
                self.reason = "定角转向完成，停车等待转后视觉结果"
                return self._result()
            speed = self._turn_speed
            self.command = (
                (-speed, speed) if self.turn_direction == "left"
                else (speed, -speed)
            )
            return self._result()

        if self.state == "PROBE_VISION_WAIT":
            if vision_has_good or vision_has_bad:
                verdict = "good" if vision_has_good else "bad"
                self.cancel()
                self._front_armed = False
                self._front_clear_count = 0
                self._side_armed = False
                self._side_clear_count = 0
                self.reason = "视觉识别为 %s，取消近物候选" % verdict
                self.vision_verdict = verdict
                return self._result(False)
            if now - self._state_started >= self.vision_wait_timeout:
                self.cancel()
                self._front_armed = False
                self._front_clear_count = 0
                self._side_armed = False
                self._side_clear_count = 0
                self.reason = "等待新视觉结果超时，取消近物候选"
                self.vision_verdict = "timeout"
                return self._result(False)
            if self._vision_wait_requires_front and not front:
                self.cancel()
                self._front_armed = False
                self._front_clear_count = 0
                self.reason = "等待视觉期间前方红外消失，取消敌人候选"
                self.vision_verdict = "target_lost"
                return self._result(False)
            if (vision_sequence is None
                    or vision_status in (None, "stale", "error")):
                self.reason = "视觉无新结果，停车等待"
                self.vision_verdict = "waiting"
                return self._result()
            if vision_sequence == self._last_vision_sequence:
                self.reason = "等待下一个不同视觉帧"
                return self._result()

            self._last_vision_sequence = vision_sequence
            if vision_status != "no_target":
                self.cancel()
                self._front_armed = False
                self._front_clear_count = 0
                self._side_armed = False
                self._side_clear_count = 0
                self.reason = "视觉存在未分类目标，取消近物候选"
                self.vision_verdict = "visual_target"
                return self._result(False)

            if not self._vision_wait_requires_front and not front:
                self.cancel()
                self._side_armed = False
                self._side_clear_count = 0
                self.reason = "转向后正前红外未发现目标，恢复巡台"
                self.vision_verdict = "post_turn_no_front"
                return self._result(False)

            self._vision_wait_requires_front = True

            self.vision_count += 1
            self.vision_verdict = "no_target_%d" % self.vision_count
            if self.vision_count >= self.vision_confirm_frames:
                self._start_push(
                    now, "连续新视觉帧均无 good/bad，确认近物体为敌人并开始推动",
                )
            else:
                self.reason = "视觉无 good/bad，确认 %d/%d" % (
                    self.vision_count, self.vision_confirm_frames,
                )
            return self._result()

        zone = (observation or {}).get("zone") or {}
        zone_valid = all(name in zone for name in ("front", "rear", "left", "right"))

        if self.state == "ENEMY_PUSH":
            if vision_has_good:
                self.cancel()
                self._front_armed = False
                self._front_clear_count = 0
                self.reason = "推动期间视觉识别为 good，取消敌人攻击"
                self.vision_verdict = "good"
                return self._result(False)
            if vision_has_bad:
                if vision_sequence != self._last_bad_interrupt_sequence:
                    self._bad_interrupt_count += 1
                    self._last_bad_interrupt_sequence = vision_sequence
                if self._bad_interrupt_count >= self.bad_interrupt_frames:
                    bad_interrupt_count = self._bad_interrupt_count
                    self.cancel()
                    self._front_armed = False
                    self._front_clear_count = 0
                    self.reason = "推动期间连续 %d 帧识别为 bad，取消敌人攻击" % (
                        self.bad_interrupt_frames,
                    )
                    self.vision_verdict = "bad_confirmed"
                    result = self._result(False)
                    result["bad_interrupt_count"] = bad_interrupt_count
                    return result
            else:
                self._bad_interrupt_count = 0
                self._last_bad_interrupt_sequence = None
            if not zone_valid:
                self._abort(now, "灰度观测无效，中断推动")
                return self._result()
            if zone["rear"] <= ENEMY_REAR_ABORT_ZONE:
                self._abort(now, "后路接近台外，中断推动")
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

        if self.state == "COOLDOWN":
            if now - self._state_started >= ENEMY_COOLDOWN_SECONDS:
                self._enter("IDLE", now, "安全冷却结束，释放控制权")
                return self._result(False)
            return self._result()

        self.cancel()
        return self._result(False)
