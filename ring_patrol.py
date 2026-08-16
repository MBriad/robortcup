#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""擂台巡台策略：输入注入的灰度数据，输出左右轮速度，不持有硬件。"""

import time

import config as cfg
from gray import GrayRiskModel


class RingPatrolController:
    """灰度闭环巡台：巡航、退离、转向并回到台面中央。"""

    def __init__(self):
        self.model = GrayRiskModel(
            window=cfg.GRAY_FILTER_WINDOW,
            edge_reference=cfg.GRAY_EDGE_REFERENCE,
            center_reference=cfg.GRAY_CENTER_REFERENCE,
            white_reference=cfg.GRAY_WHITE_REFERENCE,
            white_enter=cfg.GRAY_WHITE_ENTER,
            white_clear=cfg.GRAY_WHITE_CLEAR,
            near_edge_enter=cfg.GRAY_NEAR_EDGE_ENTER,
            near_edge_clear=cfg.GRAY_NEAR_EDGE_CLEAR,
            adc_max=cfg.GRAY_ADC_MAX,
        )
        self.state = "WARMUP"
        self.state_started = 0.0
        self.command = (0, 0)
        self.reason = "等待滤波窗口"
        self._white_count = 0
        self._near_count = 0
        self._early_front_count = 0
        self._diagonal_count = 0
        self._turn_sign = 1.0
        self._last_turn_sign = 1.0
        self.turn_angle = 0.0
        self.turn_direction = ""
        self.turn_duration = 0.0
        self.risk_sensor = ""

    @staticmethod
    def _mix(linear, turn):
        left = float(linear + turn)
        right = float(linear - turn)
        peak = max(abs(left), abs(right), 1.0)
        if peak > cfg.PATROL_COMMAND_LIMIT:
            scale = cfg.PATROL_COMMAND_LIMIT / peak
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

    @staticmethod
    def _turn_table(sign):
        return cfg.MOTOR_TURN_CALIBRATION["left" if sign < 0.0 else "right"]

    def _start_edge_avoid(self, observation, now, reason):
        linear_signal, turn_signal = self._risk_signals(observation)
        if abs(linear_signal) >= abs(turn_signal):
            self.risk_sensor = "rear" if linear_signal > 0.0 else "front"
        else:
            self.risk_sensor = "right" if turn_signal > 0.0 else "left"

        # 只有明确的车尾贴边信号才前进，其余情况统一先后退制造转向空间。
        if linear_signal > cfg.PATROL_REAR_RETREAT_DELTA:
            command = (cfg.PATROL_RECOVER_SPEED, cfg.PATROL_RECOVER_SPEED)
        else:
            command = (-cfg.PATROL_RECOVER_SPEED, -cfg.PATROL_RECOVER_SPEED)
        self._enter("EDGE_AVOID", now, command, reason)

    @staticmethod
    def _early_front_risk(observation):
        """检测车头先进入暗区的趋势，避免浅色边缘漏判。"""
        zone = observation.get("zone", {})
        try:
            return (
                observation["zone_score"] < cfg.PATROL_EARLY_FRONT_ZONE
                and zone["front"] < cfg.PATROL_EARLY_FRONT_ABS
            )
        except (KeyError, TypeError):
            return False

    def _diagonal_turn_risk(self, observation):
        """检测转向侧斜压边缘，而整体中位数仍看似安全的姿态。"""
        zone = observation["zone"]
        if self.turn_direction == "left":
            toward, away = zone["left"], zone["right"]
        elif self.turn_direction == "right":
            toward, away = zone["right"], zone["left"]
        else:
            return False
        return (
            toward < cfg.PATROL_DIAGONAL_SIDE_ZONE
            and away - toward >= cfg.PATROL_DIAGONAL_TURN_DELTA
        )

    @staticmethod
    def _diagonal_recover_risk(observation):
        """检测回中移动时任一侧斜压边缘的姿态。"""
        zone = observation["zone"]
        return (
            min(zone["left"], zone["right"]) < cfg.PATROL_DIAGONAL_SIDE_ZONE
            and abs(zone["left"] - zone["right"])
            >= cfg.PATROL_DIAGONAL_TURN_DELTA
        )

    def _start_edge_turn(self, observation, now, reason, reuse_direction=False):
        linear_signal, turn_signal = self._risk_signals(observation)
        shallow = observation["zone_score"] >= cfg.PATROL_EDGE_SHALLOW_ZONE
        if reuse_direction:
            sign = self._last_turn_sign
        elif shallow and abs(turn_signal) <= cfg.PATROL_ALTERNATE_TURN_SIGNAL:
            sign = self._turn_sign
            self._turn_sign *= -1.0
        elif abs(turn_signal) > 0.05:
            sign = -1.0 if turn_signal > 0.0 else 1.0
        else:
            sign = self._turn_sign
            self._turn_sign *= -1.0

        if abs(linear_signal) >= abs(turn_signal):
            self.risk_sensor = "rear" if linear_signal > 0.0 else "front"
        else:
            self.risk_sensor = "right" if turn_signal > 0.0 else "left"

        turn_angle = cfg.PATROL_EDGE_TURN_ANGLE
        speed, duration = self._turn_table(sign)[turn_angle]
        self._last_turn_sign = sign
        self._enter("EDGE_TURN", now, self._mix(0, sign * speed), reason)
        self.turn_angle = turn_angle
        self.turn_direction = "right" if sign > 0.0 else "left"
        self.turn_duration = duration

    def _start_white_response(self, observation, now):
        hits = set(observation["white_hits"])
        self.risk_sensor = self._dominant_risk(observation)
        if "front" in hits and "rear" not in hits:
            command = (-cfg.PATROL_WHITE_ESCAPE_SPEED,) * 2
            self._enter("WHITE_ESCAPE", now, command, "前方白边，先后退")
        elif "rear" in hits and "front" not in hits:
            command = (cfg.PATROL_WHITE_ESCAPE_SPEED,) * 2
            self._enter("WHITE_ESCAPE", now, command, "后方白边，先前进")
        else:
            self._start_edge_turn(observation, now, "侧向或多方向白边，直接转 135 度")

    @staticmethod
    def _white_edge_risk(observation):
        """白边命中且已进入暗外圈时才判为白边事件。"""
        hits = set(observation.get("white_hits") or ())
        return bool(hits & {"front", "rear"}) and bool(
            observation.get("near_edge")
        )

    def _start_recover(self, observation, now, forward, reason):
        speed = cfg.PATROL_RECOVER_SPEED if forward else -cfg.PATROL_RECOVER_SPEED
        state = "RECOVER_FORWARD" if forward else "RECOVER_BACKWARD"
        self._enter(state, now, (speed, speed), reason)

    def _start_cruise_for_zone(self, observation, now, reason):
        if observation["zone_score"] < cfg.PATROL_FAST_ZONE_SCORE:
            self.risk_sensor = ""
            self._enter(
                "MEDIUM_CRUISE",
                now,
                self._mix(cfg.PATROL_MEDIUM_LINEAR, cfg.PATROL_MEDIUM_TURN),
                reason,
            )
        else:
            self.risk_sensor = ""
            self._enter(
                "CRUISE",
                now,
                self._mix(cfg.PATROL_CRUISE_LINEAR, cfg.PATROL_CRUISE_TURN),
                reason,
            )

    def _enter(self, state, now, command, reason):
        self.state = state
        self.state_started = now
        self.command = command
        self.reason = reason
        self.turn_angle = 0.0
        self.turn_direction = ""
        self.turn_duration = 0.0

    def rearm(self, now=None):
        """外部自救结束后重置计时，并重新执行退离动作。"""
        now = time.monotonic() if now is None else float(now)
        self._white_count = 0
        self._near_count = 0
        self._early_front_count = 0
        self._diagonal_count = 0
        self._enter(
            "EDGE_AVOID",
            now,
            (-cfg.PATROL_RECOVER_SPEED, -cfg.PATROL_RECOVER_SPEED),
            "外部自救后重新退离",
        )

    def update(self, raw, now=None, healthy=True):
        now = time.monotonic() if now is None else float(now)
        observation = self.model.update(raw)

        if not healthy or not observation["valid"]:
            self._enter("SENSOR_STOP", now, (0, 0), "传感器无效或数据过期")
            return self._result(observation)
        if not observation["ready"]:
            self._enter("WARMUP", now, (0, 0), "等待滤波窗口")
            return self._result(observation)

        self._white_count = (
            self._white_count + 1 if self._white_edge_risk(observation) else 0
        )
        self._near_count = self._near_count + 1 if observation["near_edge"] else 0
        self._early_front_count = (
            self._early_front_count + 1 if self._early_front_risk(observation) else 0
        )

        if (
            self._white_count >= cfg.PATROL_WHITE_CONFIRM
            and self.state
            not in (
                "WHITE_ESCAPE",
                "EDGE_TURN",
                "RECOVER_FORWARD",
                "RECOVER_BACKWARD",
            )
        ):
            self._start_white_response(observation, now)

        if (
            self._near_count >= cfg.PATROL_NEAR_CONFIRM
            and self.state
            not in (
                "WHITE_ESCAPE",
                "EDGE_AVOID",
                "EDGE_TURN",
                "RECOVER_FORWARD",
                "RECOVER_BACKWARD",
            )
        ):
            self._start_edge_avoid(
                observation, now, "进入危险区，先直线退离再转向"
            )
            return self._result(observation)

        if (
            self._early_front_count >= cfg.PATROL_EARLY_CONFIRM
            and observation["zone_score"] < cfg.PATROL_EARLY_FRONT_ZONE
            and self.state
            not in (
                "WHITE_ESCAPE",
                "EDGE_AVOID",
                "EDGE_TURN",
                "RECOVER_FORWARD",
                "RECOVER_BACKWARD",
            )
        ):
            self._start_edge_avoid(observation, now, "前向灰度趋势变暗，提前离边")
            return self._result(observation)

        diagonal_risk = (
            self.state == "EDGE_TURN" and self._diagonal_turn_risk(observation)
        ) or (
            self.state == "RECOVER_FORWARD"
            and self._diagonal_recover_risk(observation)
        )
        if diagonal_risk:
            self._diagonal_count += 1
        else:
            self._diagonal_count = 0
        if self._diagonal_count >= cfg.PATROL_DIAGONAL_CONFIRM:
            reason = (
                "转向侧斜压白边，重新退离后再转"
                if self.state == "EDGE_TURN"
                else "回中时车身斜压白边，重新退离后再转"
            )
            self._start_edge_avoid(
                observation, now, reason
            )
            return self._result(observation)

        if self.state in ("EDGE_TURN", "RECOVER_FORWARD", "RECOVER_BACKWARD"):
            deep_threshold = (
                cfg.PATROL_DEEP_ZONE
                if self.state == "EDGE_TURN"
                else cfg.PATROL_RECOVER_DEEP_ZONE
            )
            if min(observation["zone"].values()) < deep_threshold:
                reason = (
                    "转中单路深暗，重新退离后再转"
                    if self.state == "EDGE_TURN"
                    else "回中移动时单路深暗，重新退离后再转"
                )
                self._start_edge_avoid(observation, now, reason)
                return self._result(observation)

        elapsed = now - self.state_started
        if self.state == "WHITE_ESCAPE":
            if elapsed >= cfg.PATROL_WHITE_ESCAPE_SECONDS:
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
                if self._diagonal_recover_risk(observation):
                    self._start_edge_avoid(
                        observation, now, "转向结束时车身斜压白边，重新退离后再转"
                    )
                else:
                    self._start_recover(
                        observation, now, True, "转向完成，向前离开边缘"
                    )
            return self._result(observation)

        if self.state in ("RECOVER_FORWARD", "RECOVER_BACKWARD"):
            if elapsed >= cfg.PATROL_RECOVER_SECONDS:
                if observation["zone_score"] >= cfg.PATROL_RECOVER_RELEASE_ZONE:
                    self._start_cruise_for_zone(
                        observation, now, "已达到避边释放线，恢复中速穿越渐变区"
                    )
                else:
                    self._start_edge_avoid(
                        observation, now, "转后前进仍在暗外圈，重新退离后再转"
                    )
            return self._result(observation)

        if (
            self.state == "EDGE_AVOID"
            and elapsed >= cfg.PATROL_EDGE_RETREAT_SECONDS
        ):
            self._start_edge_turn(
                observation,
                now,
                "直线退离完成，开始转向",
                reuse_direction=self.reason.startswith("转后前进"),
            )
            return self._result(observation)

        if self.state == "EDGE_AVOID":
            return self._result(observation)

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
            "shovel_preheat": bool(
                observation["ready"]
                and observation["zone"]["front"]
                < cfg.PATROL_SHOVEL_PREHEAT_FRONT_ZONE
            ),
            "observation": observation,
        }
