#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
推能量块 (push_pack/push_block.py) —— 移植包

从原项目 hardware/actuator.py 的 push_and_retreat(block=True) 移植到新车
（detail/），按 hunt/patrol 模式交付。推对手（block=False）不包含。

移植要点：
- 内嵌 ShovelGuard：铲下红外悬空状态机（2026-08-15 实测极性：悬空=信号高；
  min(两路)>ENTER 触发、max(两路)<CLEAR 收回，AD4/5）
- 前冲两档：快 500 → 慢 325（爬行档已删除，2026-08-15）
- 停线 = 铲下双路悬空（**推下确认**）；front 单路悬空/侧向悬空 → 停+退不确认；
  后路悬空/断流 → 立即打断（安全门）
- 推完反馈式倒车收回（倒到铲子收回信号 + 固定时长）
- 起步预检：铲下 9 帧预热，起步前已悬空（车在沿上）→ 取消推块直接倒车收回
- 视觉差速纠偏（可选：vision 提供 read() → TargetSample 契约）

状态机为 **step 驱动**（每 tick 注入数据，不 sleep）：真机 20ms 循环调用，
回放逐行调用——同一份逻辑两边共用。

用法：
  python3 push_pack/push_block.py --replay <推块CSV>
      回放推块 CSV（不动电机）：逐行喂状态机，打印轨迹 + 推下判定
  python3 push_pack/push_block.py [--yes] [--log 路径]
      真机单次推块：车头正对能量块 → 输入 PUSH 确认 → 执行一次推块
      → 落盘 detail/data/push_<时间戳>.csv

推块 CSV 列（回放输入必需：t, front, rear, left, right, shovel_left,
shovel_right, healthy；真机输出额外含 state/stage/left_cmd/right_cmd/
guard_state/hang/reason/confirmed）：
    t, front, rear, left, right, shovel_left, shovel_right, healthy
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time
from collections import deque

import push_block_config as cfg


# =====================================================================
# 灰度 zone（detail 参照，3 帧中值）
# =====================================================================
class GrayZones:
    """四路灰度 3 帧中值滤波 + zone 归一化（台心=1、边缘=0）。坏值按 0 处理
    （0=暗=告警侧，fail-safe）。"""

    NAMES = ("front", "rear", "left", "right")

    def __init__(self, window=cfg.GRAY_FILTER_WINDOW):
        self._buf = {n: deque(maxlen=window) for n in self.NAMES}

    def update(self, raw):
        cleaned = {}
        for n in self.NAMES:
            try:
                v = float(raw[n])
                if not math.isfinite(v) or not 0.0 <= v <= cfg.GRAY_ADC_MAX:
                    v = 0.0
            except (KeyError, TypeError, ValueError):
                v = 0.0
            cleaned[n] = v
            self._buf[n].append(v)
        filt = {n: float(statistics.median(self._buf[n])) for n in self.NAMES}
        return {n: (filt[n] - cfg.GRAY_EDGE_REFERENCE[n]) /
                   (cfg.GRAY_CENTER_REFERENCE[n] - cfg.GRAY_EDGE_REFERENCE[n])
                for n in self.NAMES}


# =====================================================================
# 铲下红外悬空状态机（shovel_pack 2026-08-15：悬空=信号高）
# =====================================================================
class ShovelGuard:
    """铲下两路模拟红外悬空检测。

    IDLE →(min(两路)>ENTER) HANGED →(连续 CONFIRM 帧) REVERSE(倒车命令)
         →(max(两路)<CLEAR 且倒够 min_seconds) IDLE / 超时 SAFE_STOP
    窗口未满视为安全；非 active/断流/无效 → 清样本回 IDLE（让出控制权）。
    """

    def __init__(self):
        self.window = cfg.SHOVEL_FILTER_WINDOW
        self._max_samples = deque(maxlen=self.window)
        self._min_samples = deque(maxlen=self.window)
        self.state = "IDLE"
        self.reason = "待机"
        self.hang = False          # 滤波后 min(两路) 高于进入阈值 = 铲子悬空
        self._hang_count = 0
        self._state_started = 0.0

    def _enter(self, state, now, reason):
        self.state = state
        self._state_started = now
        self.reason = reason

    def update(self, shovel_raw, active, now=None, healthy=True):
        now = time.monotonic() if now is None else float(now)
        valid = bool(shovel_raw.get("valid", False))
        if not active or not healthy or not valid:
            # 模式关/数据无效：不接管，清空滤波样本
            self._max_samples.clear()
            self._min_samples.clear()
            self._hang_count = 0
            self.hang = False
            self._enter("IDLE", now, "待机")
            return
        try:
            left = float(shovel_raw["left"])
            right = float(shovel_raw["right"])
        except (KeyError, TypeError, ValueError):
            return
        self._max_samples.append(max(left, right))
        self._min_samples.append(min(left, right))
        if len(self._max_samples) < self.window:
            # 中值滤波窗口未满：视为安全，不触发
            self.hang = False
            return
        filtered_max = statistics.median(self._max_samples)
        filtered_min = statistics.median(self._min_samples)
        self.hang = filtered_min > cfg.SHOVEL_HANG_ENTER
        elapsed = now - self._state_started

        if self.state == "IDLE":
            if self.hang:
                self._hang_count = 1
                self._enter("HANGED", now, "铲子悬空，停车")
            return

        if self.state == "HANGED":
            if not self.hang:
                self._hang_count = 0
                self._enter("IDLE", now, "信号恢复，继续待机")
            else:
                self._hang_count += 1
                if self._hang_count >= cfg.SHOVEL_HANG_CONFIRM:
                    self._enter("REVERSE", now, "铲子悬空确认，倒车收回")
            return

        if self.state == "REVERSE":
            if filtered_max < cfg.SHOVEL_HANG_CLEAR \
                    and elapsed >= cfg.SHOVEL_REVERSE_MIN_SECONDS:
                self._enter("IDLE", now, "铲子已收回台内")
            elif elapsed >= cfg.SHOVEL_REVERSE_TIMEOUT:
                self._enter("SAFE_STOP", now, "倒车超时未收回，停车待命")
            return

        if self.state == "SAFE_STOP":
            if filtered_max < cfg.SHOVEL_HANG_CLEAR:
                self._enter("IDLE", now, "信号恢复，解除保护停车")
            return

        # 理论不可达；防御兜底
        self._enter("IDLE", now, "未知状态")


# =====================================================================
# 单次推块状态机（step 驱动）
# =====================================================================
class BlockPusher:
    """单次推块：WARMUP（铲下预热）→ PUSH（三档前冲）→ RETREAT（倒车收回）
    → DONE。纯逻辑：每 tick 注入灰度/铲下红外/健康，返回电机命令；
    不持有硬件、不 sleep——真机 20ms 循环与回放逐行共用。"""

    def __init__(self, vision=None):
        self.vision = vision          # 可选：read() → TargetSample 契约
        self.guard = ShovelGuard()
        self.zones = GrayZones()
        self.state = "WARMUP"
        self.stage = 1                # 1=快推 2=慢推 3=爬行
        self.reason = "预热铲子"
        self.done = False
        self.confirmed = False        # 铲下双路悬空确认（推下判定）
        self.command = (0, 0)
        self._state_started = 0.0
        self._warm_frames = 0
        self._confirm_enabled = False  # 只有 PUSH 阶段的悬空事件才算推下确认
        self._slow_debounce = 0
        self._stage1_started = 0.0
        self._stage2_deadline = 0.0
        self._push_started = 0.0
        self._retreat_started = 0.0
        self._vision_started_at = 0.0
        self._last_frame_at = None

    # ---------- 内部 ----------
    def _enter(self, state, now, reason):
        self.state = state
        self._state_started = now
        self.reason = reason
        self.command = (0, 0)     # 切换瞬间先停车，防上一阶段旧指令再跑一拍
        if state == "RETREAT":
            self._retreat_started = now

    def _finish(self, now, reason):
        self.done = True
        self.command = (0, 0)
        self.reason = reason
        return self._result(now)

    def _result(self, now):
        return {
            "left": self.command[0],
            "right": self.command[1],
            "state": self.state,
            "stage": self.stage,
            "reason": self.reason,
            "done": self.done,
            "confirmed": self.confirmed,
            "guard": self.guard.state,
            "hang": self.guard.hang,
        }

    def _vision_turn(self, now, vision_sample):
        """视觉差速纠偏：|误差|>死区 → 转向量（限幅）；无新鲜帧/丢目标 → 0。"""
        if self.vision is None and vision_sample is None:
            return 0.0
        sample = vision_sample
        if sample is None:
            try:
                sample = self.vision.read()
            except Exception:
                return 0.0
        try:
            frame_at = float(sample.timestamp)
            cx = float(sample.center_x)
            valid = (sample.found and math.isfinite(frame_at) and math.isfinite(cx)
                     and frame_at >= self._vision_started_at
                     and (self._last_frame_at is None
                          or frame_at > self._last_frame_at))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return 0.0
        if not valid:
            return 0.0
        self._last_frame_at = frame_at
        error = cfg.VISION["image_center"] - cx
        if abs(error) <= cfg.VISION["deadband"]:
            return 0.0
        turn = cfg.VISION["gain"] * error
        return max(-cfg.VISION["turn_max"],
                   min(cfg.VISION["turn_max"], turn))

    # ---------- 生命周期 ----------
    def begin(self, now=None):
        now = time.monotonic() if now is None else float(now)
        self._state_started = now
        self._vision_started_at = now
        # 清 guard 残留样本（喂一帧无效），旧样本混入中值会让悬空判定延迟
        self.guard.update({"left": 0.0, "right": 0.0, "valid": False},
                          active=False, now=now)

    def step(self, gray_raw, shovel_raw, healthy=True, now=None,
             vision_sample=None):
        now = time.monotonic() if now is None else float(now)
        p = cfg.PUSH
        zone = self.zones.update(gray_raw)
        shovel = {"left": shovel_raw.get("left", 0.0),
                  "right": shovel_raw.get("right", 0.0),
                  "valid": healthy and bool(shovel_raw.get("valid", True))}

        # ---------- WARMUP：铲下预热（起步前已悬空 → 取消推块） ----------
        if self.state == "WARMUP":
            self.command = (0, 0)
            self.guard.update(shovel, active=True, now=now, healthy=healthy)
            self._warm_frames += 1
            if self.guard.state == "HANGED":
                # 车已在沿上（铲子已悬空）：不该推，直接倒车收回
                self._enter("RETREAT", now,
                            "起步前铲子已悬空（ShovelGuard）→ 取消推块，倒车收回")
            elif self._warm_frames >= self.guard.window:
                self._enter("PUSH", now, "预热完成，开始前冲")
                self._confirm_enabled = True
                self._stage1_started = now
                self._push_started = now
            return self._result(now)

        # ---------- PUSH：三档前冲 ----------
        if self.state == "PUSH":
            # 安全门：断流/后路悬空 → 立即打断（front/左/右压沿是推块正常过程）
            if not healthy:
                return self._finish(now, "数据断流，中断推块")
            if zone["rear"] <= p["rear_abort_zone"]:
                return self._finish(now, "后路悬空，中断推块")
            # 铲下双路悬空停线 = 推下确认（用户判据：铲子悬空才算推下去）
            self.guard.update(shovel, active=True, now=now, healthy=healthy)
            if self.guard.state in ("HANGED", "REVERSE"):
                self.confirmed = True
                self._enter("RETREAT", now, "铲子悬空停（ShovelGuard）→ 已推下")
                return self._result(now)
            # front 单路悬空兜底：停+退，**不确认推下**（无双路悬空证据）
            if zone["front"] <= p["block_front_suspend"]:
                self._enter("RETREAT", now, "车头单路悬空兜底（不确认推下）")
                return self._result(now)
            # 侧向悬空兜底：斜推时侧路先悬空 → 正常结束前冲进倒车（不确认）
            if min(zone["left"], zone["right"]) <= p["side_abort_zone"]:
                self._enter("RETREAT", now, "侧向悬空兜底（不确认推下）")
                return self._result(now)
            # 档位切换去抖（武字抗干扰）
            if zone["front"] <= p["block_slow_threshold"]:
                self._slow_debounce += 1
            else:
                self._slow_debounce = 0
            # 阶段推进
            if self.stage == 1:
                if (self._slow_debounce >= p["block_slow_debounce"]
                        or now - self._stage1_started >= p["block_stage1_timeout"]):
                    self.stage = 2
                    self._stage2_deadline = now + p["block_stage2_timeout"]
                    self.reason = "进入边缘带，慢推"
            else:
                if now >= self._stage2_deadline:
                    self._enter("RETREAT", now, "慢推超时（未悬空，不计推下）")
                    return self._result(now)
            if now - self._push_started >= p["block_forward_dur"]:
                self._enter("RETREAT", now, "前冲总时长上限（不计推下）")
                return self._result(now)
            # 速度 + 视觉纠偏
            speed = (p["block_fast_speed"] if self.stage == 1
                     else p["block_slow_speed"])
            turn = self._vision_turn(now, vision_sample)
            self.command = (int(round(speed - turn)), int(round(speed + turn)))
            return self._result(now)

        # ---------- RETREAT：反馈式倒车收回 ----------
        if self.state == "RETREAT":
            if not healthy:
                return self._finish(now, "数据断流，中断倒车")
            if zone["rear"] <= p["rear_abort_zone"]:
                return self._finish(now, "后路悬空，中断倒车")
            self.guard.update(shovel, active=True, now=now, healthy=healthy)
            if self._confirm_enabled and self.guard.state in ("HANGED", "REVERSE"):
                self.confirmed = True   # 倒车中仍悬空（停在沿上）也保持确认
            self.command = (-cfg.SHOVEL_REVERSE_SPEED, -cfg.SHOVEL_REVERSE_SPEED)
            elapsed = now - self._retreat_started
            if self.guard.state == "SAFE_STOP":
                return self._finish(now, "倒车超时未收回（SAFE_STOP）")
            if self.guard.state == "IDLE" and elapsed >= p["block_back_dur"]:
                return self._finish(now, "推块完成，已收回台内")
            if elapsed >= p["block_back_dur"] + cfg.SHOVEL_REVERSE_TIMEOUT:
                return self._finish(now, "倒车总超时")
            return self._result(now)

        # ---------- DONE ----------
        self.command = (0, 0)
        return self._result(now)


# =====================================================================
# PC 回放模式（不动电机）：逐行喂状态机，打印轨迹 + 推下判定
# =====================================================================
def cmd_replay(path):
    pusher = BlockPusher()
    first = True
    prev_state = None
    prev_stage = None
    with open(path, newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                t = float(row["t"])
                gray = {n: float(row[n])
                        for n in ("front", "rear", "left", "right")}
                shovel = {"left": float(row["shovel_left"]),
                          "right": float(row["shovel_right"]),
                          "valid": True}
                healthy = True
                if row.get("healthy") is not None:
                    try:
                        healthy = float(row["healthy"]) > 0.0
                    except (TypeError, ValueError):
                        healthy = False
            except (TypeError, ValueError, KeyError):
                continue          # 截断/坏行：跳过
            if first:
                pusher.begin(now=t)
                first = False
            result = pusher.step(gray, shovel, healthy=healthy, now=t)
            if (result["state"] != prev_state or result["stage"] != prev_stage):
                print("  t=%.2f %-7s 档位%d guard=%s（%s）cmd=(%d,%d)"
                      % (t, result["state"], result["stage"], result["guard"],
                         result["reason"], result["left"], result["right"]))
                prev_state = result["state"]
                prev_stage = result["stage"]
            if result["done"]:
                break
    print("推下判定: confirmed=%s（%s）" % (pusher.confirmed, pusher.reason))
    return pusher.confirmed


# =====================================================================
# 真机模式（树莓派）：确认后执行一次推块 + CSV 落盘
# =====================================================================
def _adc(data, ch):
    try:
        v = float(data[ch])
    except (IndexError, TypeError, ValueError):
        return 0.0
    if not math.isfinite(v) or not 0.0 <= v <= cfg.GRAY_ADC_MAX:
        return 0.0
    return v


def cmd_push(args):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for p in (root, os.path.join(root, "detail")):
        if p not in sys.path:
            sys.path.insert(0, p)
    try:
        from up_controller import UpController
        controller = UpController(motor_invert=cfg.CHASSIS_MOTOR_INVERT,
                                  motor_swap=cfg.CHASSIS_MOTOR_SWAP)
    except RuntimeError as e:
        print("!! %s" % e)
        print("!! push_block 真机模式只能在树莓派上跑；PC 请用 --replay 回放")
        return
    pusher = BlockPusher()
    log = args.log or os.path.join(
        root, "detail", "data", "push_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(log)), exist_ok=True)
    if not args.yes:
        if input("车头正对能量块、车放台上，确认安全后输入 PUSH：").strip() != "PUSH":
            print("未完成安全确认，启动取消。")
            controller.close()
            return
    fields = ("t", "front", "rear", "left", "right", "shovel_left", "shovel_right",
              "state", "stage", "left_cmd", "right_cmd", "guard_state", "hang",
              "reason", "confirmed", "healthy")
    start = time.monotonic()
    pusher.begin(start)
    try:
        with open(log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while not pusher.done:
                loop_start = time.monotonic()
                healthy = controller.healthy and not controller.stale(0.2)
                gray = {n: _adc(controller.adc_data, ch)
                        for n, ch in cfg.GRAY_CHANNELS.items()}
                shovel = {"left": _adc(controller.adc_data,
                                       cfg.SHOVEL_IR_CHANNELS["left"]),
                          "right": _adc(controller.adc_data,
                                        cfg.SHOVEL_IR_CHANNELS["right"]),
                          "valid": healthy}
                result = pusher.step(gray, shovel, healthy=healthy, now=loop_start)
                controller.move_cmd(result["left"], result["right"])
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    **{n: gray[n] for n in ("front", "rear", "left", "right")},
                    "shovel_left": shovel["left"],
                    "shovel_right": shovel["right"],
                    "state": result["state"],
                    "stage": result["stage"],
                    "left_cmd": result["left"],
                    "right_cmd": result["right"],
                    "guard_state": result["guard"],
                    "hang": int(result["hang"]),
                    "reason": result["reason"],
                    "confirmed": int(result["confirmed"]),
                    "healthy": int(healthy),
                })
                handle.flush()
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            controller.move_cmd(0, 0)
        except Exception:
            pass
        controller.close()
    print("推块结束：confirmed=%s（%s）" % (pusher.confirmed, pusher.reason))
    print("推块日志：%s" % log)


def main():
    parser = argparse.ArgumentParser(description="推能量块（单独可运行）")
    parser.add_argument("--replay", metavar="CSV",
                        help="回放推块 CSV（不动电机）")
    parser.add_argument("--yes", action="store_true", help="跳过安全确认")
    parser.add_argument("--log", default=None,
                        help="真机 CSV 落盘路径（默认 detail/data/push_<时间戳>.csv）")
    args = parser.parse_args()
    if args.replay:
        cmd_replay(args.replay)
        return
    cmd_push(args)


if __name__ == "__main__":
    main()
