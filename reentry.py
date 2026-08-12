#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
掉台回归状态机 (reentry.py)

场景：巡台掉出台面后（四路灰度全部暗于「真实边缘」参考值），用 6 路数字红外
搜索台墙并完成标定转向；停车后按模拟红外 left-right 差值矫正朝向；
矫正完毕倒车脱离。当前流程到倒车脱离为止，主动上台动作尚未实现。

流程：
    WAIT（掉台触发：四路 zone 全 <0 连续 FALL_CONFIRM 帧）
      → 前头有值：ADC_CORRECT；正后：TURN_180；右侧任一：TURN_RIGHT_90；
        左侧任一：TURN_LEFT_90；全无值：IR_WAIT（有值后重新分派）
      → 转向按标定时长完整执行；完成后直接大力前冲撞墙（ADC_APPROACH，
        信号弱时也会冲，不循环重试）
      → ADC_APPROACH：一次性大力前冲（APPROACH_SPEED），前头模拟红外
        signal≥APPROACH_TOUCH_SIGNAL 判定贴墙 → 立即停车进 ADC_CORRECT；
        超时 APPROACH_TIMEOUT 未贴墙 → SAFE_STOP
      → ADC_CORRECT：9 帧中值滤波后按校准区间原地转，连续确认正对
        （超时 CORRECT_TIMEOUT → SAFE_STOP）
      → REVERSE：倒车直到前头红外无值（超时 REVERSE_TIMEOUT → SAFE_STOP）
      → SAFE_STOP：灰度恢复（人工/后续上台）后回 WAIT

输入全部外部注入（不持有硬件，PC 可单测）；真机独立测试见 dev/reentry.py。
"""

import time

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
    PATROL_COMMAND_LIMIT,
    REENTRY_APPROACH_SPEED as APPROACH_SPEED,
    REENTRY_APPROACH_TIMEOUT as APPROACH_TIMEOUT,
    REENTRY_APPROACH_TOUCH_SIGNAL as APPROACH_TOUCH_SIGNAL,
    REENTRY_CORRECT_TIMEOUT as CORRECT_TIMEOUT,
    REENTRY_CORRECT_TURN_SPEED as CORRECT_TURN_SPEED,
    REENTRY_FALL_CONFIRM as FALL_CONFIRM,
    REENTRY_REVERSE_SPEED as REVERSE_SPEED,
    REENTRY_REVERSE_TIMEOUT as REVERSE_TIMEOUT,
)
from gray import GrayRiskModel
from ir import IrAlignmentModel


class ReentryController:
    """掉台回归状态机：输入 gray 四路原始值 + 6 路数字红外 + 模拟红外左右值，输出电机指令。

    update(gray_raw, ir, analog, now=None, healthy=True)：
      gray_raw : GraySensor.read_raw() 的 dict（front/rear/left/right）
      ir       : DigiIR.read_states() 的 dict（6 路布尔 + valid）
      analog   : IrSensor.read_raw() 的 dict（left/right + valid）
    返回 dict：left/right/state/reason/fall/fall_edge/observation。
    """

    def __init__(self, force_fall=False):
        self.model = GrayRiskModel(
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
        self._alignment = IrAlignmentModel(
            window=IR_ALIGNMENT_FILTER_WINDOW,
            diff_low=IR_ALIGNMENT_DIFF_LOW,
            diff_high=IR_ALIGNMENT_DIFF_HIGH,
            signal_min=IR_ALIGNMENT_SIGNAL_MIN,
            adc_max=IR_ADC_MAX,
        )
        self.state = "WAIT"
        self.reason = "等待掉台触发"
        self.command = (0, 0)
        self.fall = False         # 掉台电平（持续）
        self.fall_edge = False    # 掉台触发边沿（连续帧数刚达到阈值那帧为 True）
        self._force_fall = force_fall   # 强制触发一次（dev 台架测试用，不影响 fall 电平）
        self._fall_count = 0
        self._state_started = 0.0
        self._turn_angle = 0.0
        self._turn_duration = 0.0
        self._correct_count = 0

    # ---------- 状态迁移 ----------
    def _enter(self, state, now, command, reason):
        self.state = state
        self._state_started = now
        self.command = command
        self.reason = reason

    def _start_from_trigger(self, ir, now):
        """掉台触发后的去向：按当前 6 路红外优先级定转向/矫正/停车。"""
        if ir["front"]:
            self._start_correct(now, "掉台触发且前头红外亮，直接矫正")
        elif ir["rear"]:
            self._start_turn(now, 180.0, "正后红外亮，转 180 度")
        elif ir["right_front"] or ir["right_rear"]:
            self._start_turn(now, 90.0, "右侧红外亮，右转 90 度")
        elif ir["left_front"] or ir["left_rear"]:
            self._start_turn(now, -90.0, "左侧红外亮，左转 90 度")
        else:
            self._enter("IR_WAIT", now, (0, 0), "掉台但六路红外暂时无值，停车等待")

    def _start_turn(self, now, angle, reason):
        """原地转向：angle 正=右转、负=左转；速度/时长取 MOTOR_TURN_CALIBRATION。"""
        sign = 1.0 if angle > 0 else -1.0
        speed, duration = MOTOR_TURN_CALIBRATION[abs(angle)]
        if angle > 0:
            state = "TURN_RIGHT_90" if abs(angle) == 90.0 else "TURN_180"
        else:
            state = "TURN_LEFT_90"
        self._turn_angle = abs(angle)
        self._turn_duration = duration
        self._enter(state, now, self._mix(0, sign * speed), reason)

    def _start_correct(self, now, reason):
        self._alignment.reset()
        self._correct_count = 0
        self._enter("ADC_CORRECT", now, (0, 0), reason)

    def _start_approach(self, now, reason):
        """一次性大力前冲撞墙；贴墙（signal 满值）或超时即结束，不循环重试。"""
        self._enter(
            "ADC_APPROACH", now, (APPROACH_SPEED, APPROACH_SPEED), reason,
        )

    def _finish_approach(self, now, reason):
        self._alignment.reset()
        self._correct_count = 0
        self._enter("ADC_CORRECT", now, (0, 0), reason)

    def _start_reverse(self, now, reason):
        self._enter("REVERSE", now,
                    (-REVERSE_SPEED, -REVERSE_SPEED), reason)

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

    # ---------- 主循环 ----------
    def update(self, gray_raw, ir, analog, now=None, healthy=True):
        now = time.monotonic() if now is None else float(now)
        obs = self.model.update(gray_raw)
        ir_valid = bool(ir.get("valid", False))
        if not healthy or not obs["valid"] or not obs["ready"] or not ir_valid:
            self._fall_count = 0
            self.fall = False
            self._enter("SENSOR_STOP", now, (0, 0), "传感器无效或数据过期")
            return self._result(obs)

        all_dark = all(z < 0.0 for z in obs["zone"].values())
        self._fall_count = self._fall_count + 1 if all_dark else 0
        if self._force_fall and self.state == "WAIT":
            self._fall_count = FALL_CONFIRM   # 强制触发一次（台架测试），fall 电平仍按真实灰度
            self._force_fall = False
        self.fall = all_dark
        self.fall_edge = self._fall_count == FALL_CONFIRM

        elapsed = now - self._state_started

        if self.state == "IR_WAIT":
            if not self.fall:
                self._enter("WAIT", now, (0, 0), "等待掉台触发")
            elif any(ir[name] for name in DIGI_IR_PINS):
                self._start_from_trigger(ir, now)
            return self._result(obs)

        if self.state in ("WAIT", "SAFE_STOP", "SENSOR_STOP"):
            if self.state != "SAFE_STOP" and self.fall_edge:
                # WAIT 首次触发 / SENSOR_STOP 恢复后重新达到连续帧数：重走流程
                self._start_from_trigger(ir, now)
            elif not self.fall:
                self._enter("WAIT", now, (0, 0), "等待掉台触发")
            # SAFE_STOP 且仍掉台：保持停车（不重复触发）
            return self._result(obs)

        if self.state in ("TURN_180", "TURN_LEFT_90", "TURN_RIGHT_90"):
            if elapsed >= self._turn_duration:
                self._start_approach(now, "定时转向完成，大力前冲撞墙")
            return self._result(obs)

        if self.state == "ADC_CORRECT":
            alignment = self._alignment.update(analog)
            if not alignment["valid"]:
                self._enter("SAFE_STOP", now, (0, 0), "模拟红外无效，无法矫正")
                return self._result(obs)
            if elapsed >= CORRECT_TIMEOUT:
                self._enter("SAFE_STOP", now, (0, 0), "矫正超时未正对")
                return self._result(obs)
            if not alignment["ready"]:
                self.command = (0, 0)
                self.reason = "ADC 中值滤波准备中"
                return self._result(obs)

            if not alignment["strong"]:
                # 信号弱（前头数字红外亮但模拟信号不足）：一次性大力冲撞，不循环
                self._start_approach(
                    now, "ADC 信号弱 %.0f，大力前冲撞墙" % alignment["signal"])
                return self._result(obs)

            d = alignment["diff"]
            if alignment["position"] == "center":
                self._correct_count += 1
                self.command = (0, 0)
                self.reason = "ADC 正对确认 %d/%d，diff=%.0f" % (
                    self._correct_count, IR_ALIGNMENT_CONFIRM, d,
                )
                if self._correct_count >= IR_ALIGNMENT_CONFIRM:
                    self._start_reverse(now, "ADC 矫正完成，倒车")
            else:
                self._correct_count = 0
                sign = 1.0 if alignment["correction"] == "right" else -1.0
                self.command = self._mix(0, sign * CORRECT_TURN_SPEED)
                self.reason = "ADC %s，向%s矫正，diff=%.0f" % (
                    "左偏" if alignment["position"] == "left_bias" else "右偏",
                    "右" if alignment["correction"] == "right" else "左",
                    d,
                )
            return self._result(obs)

        if self.state == "ADC_APPROACH":
            if analog.get("valid") and max(
                    analog["left"], analog["right"]) >= APPROACH_TOUCH_SIGNAL:
                # 前头红外 signal 已达贴墙阈值：立即停车防堵转，回矫正
                self._finish_approach(now, "大力冲撞贴墙，停车矫正")
            elif elapsed >= APPROACH_TIMEOUT:
                self._enter("SAFE_STOP", now, (0, 0), "大力冲撞超时未贴墙，停车")
            return self._result(obs)

        if self.state == "REVERSE":
            if not ir["front"]:
                self._enter("SAFE_STOP", now, (0, 0), "倒车完成（前头红外无值）")
            elif elapsed >= REVERSE_TIMEOUT:
                self._enter("SAFE_STOP", now, (0, 0), "倒车超时")
            return self._result(obs)

        # 理论不可达；防御兜底
        self._enter("SAFE_STOP", now, (0, 0), "未知状态")
        return self._result(obs)

    def _result(self, obs):
        return {
            "left": self.command[0],
            "right": self.command[1],
            "state": self.state,
            "reason": self.reason,
            "fall": self.fall,
            "fall_edge": self.fall_edge,
            "observation": obs,
        }
