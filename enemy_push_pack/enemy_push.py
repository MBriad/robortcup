#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
敌人推动模块 (enemy_push_pack/enemy_push.py) —— 移植包

识别敌人（前头红外 io0）→ 分级推 700/350 → 铲子防掉停 → 倒车收回 → 短停
继续打循环。纯红外触发，**不含视觉**（buff/debuff 分流以后接）。

自包含：内嵌 DiagIR（6 方位敌侦测，= hunt 的 IO 部分）、ShovelGuard
（铲下悬空，shovel_pack 08-15 极性）、GrayZones（新车 zone 灰度）；
真机只 import 你的 detail/up_controller.py 硬件层，无其他模块依赖。

流程（step 驱动状态机，不 sleep）：
    IDLE 等前头红外（io0）→ 亮 → PUSH 一直推（用户拍板：不设时长上限，
    其他红外不管）：**分级速度** front zone ≥1.3 快 700、<1.3 慢 350
    （白边之前降速，给铲子防掉留反应时间；档位只降不升）
      → 停线：铲子双路悬空（ShovelGuard 防掉，确认推下）/ 白边四路全亮
      （white_bright_threshold 保护，不确认）→ RETREAT 倒车 1.0s 收回
      → 回 IDLE，**attack_pause 0.5s 后 io0 还亮就继续打**
      （多次攻击；边沿触发版已按用户否决）
    安全打断（断流 / 后路悬空 / 倒车超时）→ COOLDOWN 3s → IDLE

⚠️ io0 锁存（如推墙）会一直推——后路悬空/断流兜底，其余靠铲子防掉。

用法：
  python3 enemy_push_pack/enemy_push.py [--log 路径]
      真机：**直接启动即开始**（无确认输入）——前头红外亮就推，Ctrl+C 退出；
      落盘 detail/data/attack_<时间戳>.csv
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time
from collections import deque
from typing import Callable, Dict, Optional

import enemy_push_config as cfg


# =====================================================================
# 数字红外敌侦测（原封移植 hunt_pack DiagIR；通道=新车映射）
# =====================================================================
class DiagIR:
    """4 路对角 + 正前/后向数字红外 → 6 方位编码（0前 1左前 2右前 3左后
    4右后 5后，无=-1）。IO 多数表决滤波；正前 IO 单独亮不算敌人（需
    on_stage+under_shovel 确认，本模块不传 → 正前确认门恒关，d==0 只靠
    左前+右前同时）。"""

    def __init__(self, controller=None,
                 diag_channels: Optional[Dict[str, int]] = None,
                 front_ch: int = 4, rear_ch: int = 5,
                 active_low: bool = True, window: int = 5,
                 on_stage: Optional[Callable[[], bool]] = None,
                 under_shovel: Optional[Callable[[], bool]] = None):
        self._controller = controller
        self.diag_channels = (dict(diag_channels) if diag_channels else {
            "left_front": 0, "right_front": 1, "left_rear": 2, "right_rear": 3})
        self.front_ch = front_ch
        self.rear_ch = rear_ch
        self.active_low = active_low
        self.window = max(1, int(window))
        self._on_stage = on_stage or (lambda: False)
        self._under_shovel = under_shovel or (lambda: False)
        self._io_buf: dict = {}
        self._frame: Optional[dict] = None

    def begin_frame(self) -> None:
        chans = set(self.diag_channels.values())
        for ch in (self.front_ch, self.rear_ch):
            if ch is not None:
                chans.add(ch)
        self._frame = {"i%d" % ch: self._live_io(ch) for ch in chans}

    def _live_io(self, ch: int) -> int:
        if self._controller is None:
            raw = 1
        else:
            try:
                raw = self._controller.io_data[ch]
            except (AttributeError, IndexError, KeyError, TypeError):
                raw = 0
            if raw not in (0, 1):
                raw = 0
        q = self._io_buf.setdefault(ch, deque(maxlen=self.window))
        q.append(raw)
        n = len(q)
        ones = sum(q)
        if ones * 2 > n:
            return 1
        if ones * 2 < n:
            return 0
        return 0 if self.active_low else 1

    def _ir_detected_live(self, ch: int) -> bool:
        val = self._live_io(ch)
        return (val == 0) if self.active_low else (val == 1)

    def _diag_detected_live(self, name: str) -> bool:
        ch = self.diag_channels.get(name)
        return ch is not None and self._ir_detected_live(ch)

    def front_target_detected_live(self) -> bool:
        if self.front_ch is None:
            return False
        return self._ir_detected_live(self.front_ch)

    def enemy_direction_live(self) -> int:
        lf = self._diag_detected_live("left_front")
        rf = self._diag_detected_live("right_front")
        lr = self._diag_detected_live("left_rear")
        rr = self._diag_detected_live("right_rear")
        if lf and rf:
            return 0
        if lr and rr:
            return 5
        if lf:
            return 1
        if rf:
            return 2
        if lr:
            return 3
        if rr:
            return 4
        return -1


# =====================================================================
# 铲下红外悬空状态机（shovel_pack 08-15：悬空=信号高）
# =====================================================================
class ShovelGuard:
    """IDLE →(min(两路)>ENTER) HANGED →(确认) REVERSE →(max<CLEAR) IDLE /
    超时 SAFE_STOP。窗口未满视为安全；非 active/断流 → 清样本回 IDLE。"""

    def __init__(self):
        self.window = cfg.SHOVEL_FILTER_WINDOW
        self._max_samples = deque(maxlen=self.window)
        self._min_samples = deque(maxlen=self.window)
        self.state = "IDLE"
        self.reason = "待机"
        self.hang = False
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
        self._enter("IDLE", now, "未知状态")


# =====================================================================
# 灰度 zone（detail 参照，3 帧中值）
# =====================================================================
class GrayZones:
    """灰度 zone（detail 参照，3 帧中值）。四路全算——分级速度用 front、
    后路安全门用 rear、白边保护用四路最小值。"""

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
# 敌人推动状态机（step 驱动）
# =====================================================================
class EnemyPusher:
    """IDLE → TURN → PUSH → RETREAT → TURN_AWAY → COOLDOWN → IDLE。
    纯逻辑：每 tick 注入灰度/铲下红外/健康，返回电机命令；不 sleep——
    真机 20ms 循环与测试共用。"""

    def __init__(self, controller=None):
        self.ir = DiagIR(controller=controller,
                         diag_channels=cfg.DIAG_IR_CHANNELS,
                         front_ch=cfg.FRONT_TARGET_IO_CH,
                         rear_ch=cfg.REAR_IR_CHANNELS[0],
                         active_low=cfg.IR_ACTIVE_LOW,
                         window=cfg.IO_FILTER_WINDOW)
        self.guard = ShovelGuard()
        self.zones = GrayZones()
        self.state = "IDLE"
        self.reason = "等待红外"
        self.confirmed = False
        self.command = (0, 0)
        self._slow = False          # 慢档锁存（进 PUSH 时清零）
        self._slow_debounce = 0
        self._white_ticks = 0       # 白边保护连续帧计数
        self._state_started = 0.0

    def _enter(self, state, now, reason):
        self.state = state
        self._state_started = now
        self.reason = reason
        self.command = (0, 0)     # 切换瞬间先停车
        if state == "PUSH":
            self.confirmed = False
            self._slow = False
            self._slow_debounce = 0
            self._white_ticks = 0

    def _result(self, now):
        return {
            "left": self.command[0],
            "right": self.command[1],
            "state": self.state,
            "reason": self.reason,
            "confirmed": self.confirmed,
            "guard": self.guard.state,
            "hang": self.guard.hang,
        }

    def step(self, gray_raw, shovel_raw, healthy=True, now=None):
        now = time.monotonic() if now is None else float(now)
        a = cfg.ATTACK
        zone = self.zones.update(gray_raw)
        shovel = {"left": shovel_raw.get("left", 0.0),
                  "right": shovel_raw.get("right", 0.0),
                  "valid": healthy and bool(shovel_raw.get("valid", True))}

        if self.state == "IDLE":
            # 保持 guard 滤波窗口温热（推块模块同款预热）：进入 PUSH 时窗口
            # 已满，铲子悬空第一帧就能停，不用 9 帧（0.18s）白冲
            self.guard.update(shovel, active=True, now=now, healthy=healthy)
            # 只认前头红外（io0）：亮 → 直接推；对角/后向红外不管（用户拍板）。
            # attack_pause：两次攻击之间短停（多次攻击节奏，2026-08-16 用户
            # 定：退完立刻再推会突刺双退，等红外熄灭又太被动）
            if (self.ir.front_target_detected_live()
                    and now - self._state_started >= a["attack_pause"]):
                self._enter("PUSH", now, "前头红外亮，直接推")
            return self._result(now)

        if self.state == "PUSH":
            if not healthy:
                return self._abort(now, "数据断流，中断推")
            if zone["rear"] <= a["rear_abort_zone"]:
                self._enter("RETREAT", now, "后路悬空，中断推→倒车收回")
                return self._result(now)
            self.guard.update(shovel, active=True, now=now, healthy=healthy)
            if self.guard.state in ("HANGED", "REVERSE"):
                self.confirmed = True
                self._enter("RETREAT", now, "铲子悬空停 → 对手已推下")
                return self._result(now)
            # 白边灰度保护（2026-08-16）：四路 zone 全部 ≥ 阈值连续 N 帧 =
            # 车整体开上白边（此场景铲下红外不触发，曾 4 次推掉台）→ 中断推、
            # 倒车退回（不确认推下）
            if min(zone.values()) >= a["white_bright_threshold"]:
                self._white_ticks += 1
            else:
                self._white_ticks = 0
            if self._white_ticks >= a["white_confirm_ticks"]:
                self._enter("RETREAT", now, "白边保护（四路全亮），中断推")
                return self._result(now)
            # 分级速度（2026-08-16）：front zone 跌破 slow_threshold（白边之前）
            # → 降速到慢档，给铲子防掉（9 帧滤波 ~0.24s）留反应时间；
            # 去抖 6 帧防武字，且档位只降不升（进了边缘带不回快档）
            if zone["front"] < a["slow_threshold"]:
                self._slow_debounce += 1
            else:
                self._slow_debounce = 0
            if self._slow_debounce >= a["slow_debounce"]:
                self._slow = True
            speed = a["slow_speed"] if self._slow else a["push_speed"]
            # 前头红外亮就一直推（用户拍板）：不设时长上限；铲子防掉是唯一
            # 停线。⚠️ io0 锁存（如推墙）会一直推，后路悬空/断流兜底
            self.command = (speed, speed)
            return self._result(now)

        if self.state == "RETREAT":
            if not healthy:
                return self._abort(now, "数据断流，中断倒车")
            if zone["rear"] <= a["rear_abort_zone"]:
                return self._abort(now, "后路悬空，中断倒车")
            self.guard.update(shovel, active=True, now=now, healthy=healthy)
            if self.guard.state in ("HANGED", "REVERSE"):
                self.confirmed = True
            if self.guard.state == "SAFE_STOP":
                return self._abort(now, "倒车超时未收回（SAFE_STOP）")
            if (self.guard.state == "IDLE"
                    and now - self._state_started >= a["retreat_dur"]):
                # 收回完成 → 回 IDLE（多次攻击，2026-08-16 用户定：等红外
                # 熄灭太被动；attack_pause 短停控制攻击节奏）
                self._enter("IDLE", now, "倒车收回完成")
                return self._result(now)
            if now - self._state_started >= a["retreat_dur"] + cfg.SHOVEL_REVERSE_TIMEOUT:
                return self._abort(now, "倒车总超时")
            self.command = (-a["retreat_speed"], -a["retreat_speed"])
            return self._result(now)

        if self.state == "COOLDOWN":
            if now - self._state_started >= a["cooldown"]:
                self._enter("IDLE", now, "冷却结束，继续等待")
            return self._result(now)

        self.command = (0, 0)
        return self._result(now)

    def _abort(self, now, reason):
        self.command = (0, 0)
        self._enter("COOLDOWN", now, reason)
        return self._result(now)


# =====================================================================
# 真机模式（树莓派）：确认后静止等待红外 → 对准+推+收回+离开 循环 + CSV
# =====================================================================
def _adc(data, ch):
    try:
        v = float(data[ch])
    except (IndexError, TypeError, ValueError):
        return 0.0
    if not math.isfinite(v) or not 0.0 <= v <= cfg.GRAY_ADC_MAX:
        return 0.0
    return v


def cmd_run(args):
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
        print("!! enemy_push 真机模式只能在树莓派上跑")
        return
    pusher = EnemyPusher(controller=controller)
    log = args.log or os.path.join(
        root, "detail", "data", "attack_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(log)), exist_ok=True)
    # 直接启动（用户要求，2026-08-16）：前头红外亮就推，无确认输入
    fields = ("t", "front", "rear", "left", "right", "shovel_left", "shovel_right",
              "state", "left_cmd", "right_cmd", "guard_state", "hang",
              "reason", "confirmed", "healthy")
    start = time.monotonic()
    last_push = None
    try:
        with open(log, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            while True:
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
                if result["state"] != last_push:
                    print("%s %s（%s）confirmed=%s"
                          % (time.strftime("%H:%M:%S"), result["state"],
                             result["reason"], result["confirmed"]))
                last_push = result["state"]
                writer.writerow({
                    "t": round(loop_start - start, 3),
                    **{n: gray[n] for n in ("front", "rear", "left", "right")},
                    "shovel_left": shovel["left"],
                    "shovel_right": shovel["right"],
                    "state": result["state"],
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
    print("敌人推动日志：%s" % log)


def main():
    parser = argparse.ArgumentParser(description="敌人推动模块（单独可运行）")
    parser.add_argument("--log", default=None,
                        help="真机 CSV 落盘路径（默认 detail/data/attack_<时间戳>.csv）")
    args = parser.parse_args()
    cmd_run(args)


if __name__ == "__main__":
    main()
