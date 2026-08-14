#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
猎杀模式 单文件独立模块 (hunt_pack/hunt.py) —— 源: tools/calibrate.py cmd_hunt

仅依赖 Python 标准库 + 你提供的 gray.py（灰度模型，注入实例、不 import）。
猎杀模式流程（**不巡台、不扫描**）：

    车静止等待 → 对角红外亮 → 固定角度粗转（45°/135°/180°，早期停在
    正前 IO4 连续触发）→ IO4 未确认则同向慢搜（_fine_align）→ 摄像头
    多数表决分类：
        buff   → 视觉 center_x 连续微调居中后**锁定目标保持静止**
                 （原项目用户拍板"别前进后退了"——推块动作未接入）
        debuff → 转离 90° + 横移 + 回正绕行（_evade_debuff）
        unknown→ 同方位冷却后继续静止等待
    后向红外 IO5 单独亮 → 按方位 5（正后）掉头兜底。

数据落盘（你的约定：采集数据丢 data/）：单 CSV data/hunt_<时间戳>.csv，
type 列区分 decision（每轮决策一行）/align（对准事件）。

接入你的项目需要提供的接口：
  1. 电机 : ctrl.move_cmd(left, right)
            ⚠️ 转向约定 move_cmd(s,-s)=右转/顺时针是**原车**实测；换车必须验证
               （见 hunt_config.TURN_SGN；08-14 车朝目标反方向转的根因是旧
               DIGI_IR_BITS 映射，重 scan 修正后 TURN_SGN 保持 +1）
  2. 数字红外 : controller.io_data（通道 → 0/1），通道号/极性见 hunt_config.py，
               重测后传入 DiagIR
  3. 灰度安全钩子 : 由 GraySafety 适配层提供（消费你的 GrayRiskModel）——
       data_fresh()   = controller.stale()/healthy（断流判定）
       fall_risk(exclude) = white_hits 非空（压白边=掉台风险；exclude 跳过方向）
       edge_risk()    = near_edge（模型预热期返回 False 不判定）
       min_zone()     = 四路 zone 最小值（debuff 绕行查边：< GRAY["evade_zone"] 停）
       front/rear/left/right_gray_live() = 滤波后四路原始值
     阈值分工：white/zone 参考值与滤波窗口在**你的 gray.py** 里调；
     evade_zone 在 hunt_config.py 的 GRAY 里调
  4. 视觉 : 你的后台程序 + 你的 Python 接口。适配器 = VisionAdapter（后台线程
     轮询你的 client.read_result()，read() 永不阻塞）；to_sample() 把你的一帧
     输出转成 TargetSample（found/kind/center_x/timestamp，timestamp 必须真实
     单调——分类"只认不同帧"依赖它）。接口字段不同就只改 to_sample
  5. actuator —— 只做 debuff 绕行的开关（源逻辑：actuator 为 None 时不绕行）
  6. on_locked(kind) —— 可选回调，识别 buff 且视觉居中锁定瞬间触发一次

最小使用示例：
    import hunt_config as cfg
    from hunt import DiagIR, GraySafety, VisionAdapter, hunt_run
    from gray import GraySensor, GrayRiskModel   # 你的 gray.py

    ir = DiagIR(controller=my_controller,
                diag_channels=cfg.DIAG_IR_CHANNELS,
                front_ch=cfg.FRONT_TARGET_IO_CH,
                rear_ch=cfg.REAR_IR_CHANNELS[0],
                active_low=cfg.IR_ACTIVE_LOW,
                window=cfg.IO_FILTER_WINDOW,
                on_stage=..., under_shovel=...)
    safety = GraySafety(
        my_controller,
        read_raw=lambda: GraySensor(
            adc_reader=lambda: my_controller.adc_data).read_raw(),
        model=GrayRiskModel())
    vision = VisionAdapter(client=my_vision_client)  # client.read_result() -> dict
    vision.start()

    class MySensors:
        def enemy_direction(self): return ir.enemy_direction()
        def front_target_detected_live(self): return ir.front_target_detected_live()
        def rear_obstacle(self): return ir.rear_obstacle()
        def data_fresh(self): return safety.data_fresh()
        def fall_risk(self, exclude=()): return safety.fall_risk(exclude)
        def edge_risk(self): return safety.edge_risk()
        def front_gray_live(self): return safety.front_gray_live()
        # rear_gray_live / left_gray_live / right_gray_live 同 front
        def min_zone(self): return safety.min_zone()

    hunt_run(my_ctrl, MySensors(), vision, cfg.HUNT,
             on_locked=lambda kind: print("锁定", kind))

⚠️ 所有参数（hunt_config.HUNT）是原车标定值，换车必须重标。
"""

import csv
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Callable, Dict, Optional

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

from hunt_config import GRAY, HUNT, TURN_SGN

# =====================================================================
# 视觉样本契约（源: perception/target.py）
# =====================================================================
@dataclass(frozen=True)
class TargetSample:
    """
    视觉输出的单帧目标样本（不可变快照）。

    - kind     : "buff" / "debuff" / "enemy" / "unknown"
    - found    : 本帧是否检测到目标
    - center_x : 目标包围框中心 X（原项目视觉输出 640 宽，中心 320；
                 你的分辨率不同时改 hunt_config.HUNT["image_center"]）
    - distance : 距离代理（包围框高度像素，越大越近；猎杀模式未使用）
    - timestamp: 采样时刻 time.monotonic()——**必须真实单调**，
                 分类"只认不同 producer 帧"依赖它
    """
    kind: str = "unknown"
    found: bool = False
    center_x: float = 0.0
    distance: float = 0.0
    timestamp: float = 0.0


# =====================================================================
# 视觉适配器（源: perception/rpi_yolo_vision.py 的线程适配器模式）
# =====================================================================
def to_sample(data, now=None):
    """把你的视觉接口一帧输出转成 TargetSample。

    **你的接口字段不同就只改这一个函数。** data 期望字段：
        {"found": bool, "kind": "buff"/"debuff", "center_x": float,
         "timestamp": float}   # timestamp 用单调时钟（time.monotonic 基准）
    缺字段/非法值按"无目标"处理（fail-safe）。
    """
    if not isinstance(data, dict):
        return TargetSample(timestamp=time.monotonic())
    try:
        if not bool(data.get("found", False)):
            return TargetSample(timestamp=time.monotonic())
        kind = str(data.get("kind", "unknown"))
        center_x = float(data.get("center_x", math.nan))
        ts = data.get("timestamp")
        ts = float(ts if ts is not None else
                   (now if now is not None else time.monotonic()))
    except (TypeError, ValueError, OverflowError):
        return TargetSample(timestamp=time.monotonic())
    if not math.isfinite(center_x) or not math.isfinite(ts):
        return TargetSample(timestamp=time.monotonic())
    return TargetSample(kind=kind, found=True, center_x=center_x,
                        timestamp=ts)


class VisionAdapter:
    """视觉后台程序适配器：后台线程轮询你的 client，read() 永不阻塞。

    client：你的 Python 接口对象，需提供 read_result() -> dict（字段见
    to_sample；阻塞发生在适配器线程里，不影响 20ms 决策循环）。
    None=视觉不可用（read() 恒 not found，分类必然 unknown → 不盲推）。

    start()/stop() 幂等。status().available：client 存在且最近一次
    轮询成功（_visual_available 用它快速放弃）。
    """

    def __init__(self, client=None, poll_interval=0.02):
        self._client = client
        self._poll = max(float(poll_interval), 0.005)
        self._lock = threading.Lock()
        self._sample = TargetSample()
        self._available = client is not None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._started = False

    def start(self):
        if self._started:
            return
        self._started = True
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._started and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)

    def read(self):
        with self._lock:
            return self._sample

    def status(self):
        return SimpleNamespace(
            state="running" if self._available else "no_frame",
            available=self._available)

    def _loop(self):
        while not self._stop.is_set():
            if self._client is None:
                self._stop.wait(0.1)
                continue
            try:
                sample = to_sample(self._client.read_result())
                self._available = True
            except Exception:
                sample = TargetSample(timestamp=time.monotonic())
                self._available = False
            with self._lock:
                self._sample = sample
            self._stop.wait(self._poll)


# =====================================================================
# 数字红外敌侦测（源: hardware/sensors.py 的 IO 部分）
# =====================================================================
class DiagIR:
    """
    4 路对角数字红外（左前/右前/左后/右后）+ 正前/后向数字红外 → 6 方位编码：
        0=前 1=左前 2=右前 3=左后 4=右后 5=后，无=-1

    关键设计（原项目实车验证）：
    - IO 读数做 N 帧多数表决滑动滤波（滤单帧毛刺，平票按极性判）；
    - 极性 active_low=True：反射式红外 0=触发（0=有反射/目标进入探测范围）；
    - **正前 IO 单独亮不算敌人**（可能是台壁/围栏）——必须叠加 on_stage
      （在台上）+ 铲下确认（目标已贴正前）才判 0；
    - 对角单路 IO 承载方位，必须优先于正前宽信号；
    - 后向 IO 单独亮不在 6 方位编码内（见 rear_obstacle，调用方兜底按 5）。

    controller   : 提供 io_data（通道 → 0/1）的对象；None=桩（恒无触发）
    diag_channels: 对角红外通道映射 {"left_front": 0, ...}
    front_ch     : 正前方数字红外通道（None=未配置）
    rear_ch      : 车后数字红外通道（None=未配置）
    active_low   : True=0 表示触发（反射式红外常见极性）
    window       : 多数表决滤波窗口（帧）
    on_stage     : 是否在台上的判定函数（正前确认门用，缺省恒 False）
    under_shovel : 铲下是否有目标（正前确认门用，缺省恒 False）
    """

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
        self._frame: Optional[dict] = None   # begin_frame() 的帧缓存

    # ---------- 帧快照：一轮决策内数据一致（可选，每轮决策开头调用） ----------
    def begin_frame(self) -> None:
        """一次性读取所有用到的 IO 通道并缓存（本帧内 enemy_direction 复用）。"""
        chans = set(self.diag_channels.values())
        for ch in (self.front_ch, self.rear_ch):
            if ch is not None:
                chans.add(ch)
        self._frame = {"i%d" % ch: self._live_io(ch) for ch in chans}

    # ---------- 原始读数（多数表决滑动滤波） ----------
    def _live_io(self, ch: int) -> int:
        """实时 IO 读数（总是推进滤波）。桩模式恒 1（不触发）。

        坏值（非 0/1）按 0 处理——active_low 下 0=触发，坏值 fail-safe
        偏向"有目标"（宁可多转一下，不可漏敌）。
        """
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
        return 0 if self.active_low else 1   # 平票按极性默认值

    def io(self, ch: int) -> int:
        """决策路径：帧缓存优先；无帧时实时。"""
        if self._frame is not None and "i%d" % ch in self._frame:
            return self._frame["i%d" % ch]
        return self._live_io(ch)

    # ---------- 触发判定 ----------
    def _ir_detected(self, ch: int) -> bool:
        """决策路径：按极性判定触发（active_low：0=触发）。"""
        val = self.io(ch)
        return (val == 0) if self.active_low else (val == 1)

    def _ir_detected_live(self, ch: int) -> bool:
        """实时版：转向对准等长循环内不拿旧帧（否则会过冲）。"""
        val = self._live_io(ch)
        return (val == 0) if self.active_low else (val == 1)

    def _diag_detected(self, name: str) -> bool:
        ch = self.diag_channels.get(name)
        return ch is not None and self._ir_detected(ch)

    def _diag_detected_live(self, name: str) -> bool:
        ch = self.diag_channels.get(name)
        return ch is not None and self._ir_detected_live(ch)

    def front_target_detected(self) -> bool:
        """正前方数字红外触发 → 正前有东西（目标或墙）。未配置恒 False。"""
        if self.front_ch is None:
            return False
        return self._ir_detected(self.front_ch)

    def front_target_detected_live(self) -> bool:
        """正前方数字红外触发（实时版：转向对准长循环内用）。"""
        if self.front_ch is None:
            return False
        return self._ir_detected_live(self.front_ch)

    def rear_obstacle(self) -> bool:
        """车后数字红外触发 → 后方有东西。未配置恒 False。

        猎杀模式用它兜底：块在正后方时对角后向可能不触发，只有后向亮
        → 调用方按方位 5（正后）掉头处理。
        """
        if self.rear_ch is None:
            return False
        return self._ir_detected(self.rear_ch)

    # ---------- 6 方位敌侦测 ----------
    def enemy_direction(self) -> int:
        """6 方位敌侦测（对角数字红外优先，正前 IO 仅确认）。

        0=前（左前+右前同时；或正前 IO 触发且**在台上**且**铲下确认**——
        单靠正前 IO 可能是台壁/围栏，不当敌人）
        1=左前 2=右前 3=左后 4=右后 5=后（左后+右后同时），无=-1。
        """
        lf = self._diag_detected("left_front")
        rf = self._diag_detected("right_front")
        lr = self._diag_detected("left_rear")
        rr = self._diag_detected("right_rear")
        # 单路对角 IO 承载方位，必须优先于正前宽信号
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
        if (self.front_target_detected() and self._on_stage()
                and self._under_shovel()):
            return 0
        return -1

    def enemy_direction_live(self) -> int:
        """实时版（转向对准等长循环内用：目标转到正前后必须立即看到变化）。"""
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
        if (self.front_target_detected_live() and self._on_stage()
                and self._under_shovel()):
            return 0
        return -1


# =====================================================================
# 运动/分类辅助（源: strategy/motion_helpers.py）
# =====================================================================
def _turn_ok(sensors):
    """旋转专用安全：数据新鲜且无**真掉台**（fall_risk）→ 安全。

    旋转时前向传感器扫过台沿（悬空）是正常现象——台沿类信号不打断转向；
    真掉台/断流立即停。sensors 缺接口时按不安全处理（fail-safe）。
    """
    try:
        if not sensors.data_fresh():
            return False
        if sensors.fall_risk():
            return False
        return True
    except Exception:
        return False


def _turn_sgn(d):
    """方位 → 转向符号（乘 hunt_config.TURN_SGN 车旋向修正）。

    原车约定：`tcmd=(s,-s)`=右转/顺时针；左前/左后/后(1/3/5) 需左转、
    右前/右后(2/4) 需右转。TURN_SGN 做车旋向修正（新车架起车轮验证前
    暂按原约定 +1）；车再变只改 hunt_config.TURN_SGN。
    """
    return -TURN_SGN if d in (1, 3, 5) else TURN_SGN


def _drive_sliced(ctrl, sensors, left, right, duration, check_fn):
    """分片行驶：每 20ms 查 check_fn（安全），危机/断流立即停车。

    返回 True=走完；False=被安全打断（已停车）。
    """
    deadline = time.monotonic() + max(duration, 0.0)
    ctrl.move_cmd(left, right)
    while time.monotonic() < deadline:
        if not check_fn(sensors):
            ctrl.move_cmd(0, 0)
            return False
        time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
    ctrl.move_cmd(0, 0)
    return True


def _classify(vision, tries=3, min_votes=2, timeout=1.5):
    """只用不同 producer 帧做稳定分类；重复旧帧不能凑票。

    至少 min_votes 张**不同时间戳**新帧一致且期间没有相反类别才返回
    buff/debuff；帧不足、类别冲突、时间戳异常或超时均返回 unknown。
    这样红外只负责发现物体，绝不代替视觉完成 buff/debuff 分类。
    """
    if vision is None:
        return "none"
    try:
        tries = int(tries)
        min_votes = int(min_votes)
        timeout = float(timeout)
    except (TypeError, ValueError, OverflowError):
        return "unknown"
    if (tries < min_votes or min_votes < 1 or not math.isfinite(timeout)
            or timeout <= 0):
        return "unknown"
    counts = {"buff": 0, "debuff": 0}
    seen_timestamps = set()
    deadline = time.monotonic() + timeout
    while len(seen_timestamps) < tries and time.monotonic() < deadline:
        s = vision.read()
        try:
            frame_at = float(s.timestamp)
            valid = (s is not None and s.found and s.kind in counts
                     and math.isfinite(frame_at))
        except (AttributeError, TypeError, ValueError, OverflowError):
            valid = False
        if valid and frame_at not in seen_timestamps:
            seen_timestamps.add(frame_at)
            counts[s.kind] += 1
            # 一旦出现相反类别，本轮身份不稳定，不放行攻击
            if counts["buff"] >= min_votes and counts["debuff"] == 0:
                print("  [分类] buff 新帧确认 %d/%d" % (counts["buff"], min_votes))
                return "buff"
            if counts["debuff"] >= min_votes and counts["buff"] == 0:
                print("  [分类] debuff 新帧确认 %d/%d" % (counts["debuff"], min_votes))
                return "debuff"
        time.sleep(0.02)
    print("  [分类] 新帧票数 buff=%d debuff=%d（需同类 %d 帧且零冲突）→ unknown"
          % (counts["buff"], counts["debuff"], min_votes))
    if counts["buff"] >= min_votes and counts["debuff"] == 0:
        return "buff"
    if counts["debuff"] >= min_votes and counts["buff"] == 0:
        return "debuff"
    return "unknown"


# =====================================================================
# 猎杀模式辅助（源: tools/calibrate.py）
# =====================================================================
def _front_target_active(sensors):
    """正前 IO 实时触发（未配置/异常 → None）。"""
    reader = getattr(sensors, "front_target_detected_live", None)
    if not callable(reader):
        return None
    try:
        return bool(reader())
    except Exception:
        return None


def _align_event(logger, phase, direction, command, active, ticks, elapsed, result,
                 center_x=None, frame_at=None):
    """对准事件写 CSV（保持高频轮询不进 CSV，只记决策点）。"""
    if callable(logger):
        logger({"phase": phase, "direction": direction, "left": command[0],
                "right": command[1], "io4_active": active, "confirm_ticks": ticks,
                "center_x": center_x, "frame_at": frame_at,
                "elapsed": elapsed, "result": result})


def _turn_to_target(ctrl, sensors, cfg, direction=None, align_log=None):
    """固定角度粗转，早期只停在正前 IO4 连续触发。

    返回 (ok, io4_confirmed)。铲前模拟红外（AD6/7 向下照地）刻意不参与
    目标对准——它向前下安装，对平地恒有反射，无法区分地面与目标。
    """
    d = sensors.enemy_direction() if direction is None else direction
    if d < 0:
        return False, False
    if d == 0:
        # 目标已在正前（红外报 0）→ 视为已确认：旧实现返回 (True, False)
        # 让 _fine_align 走同向慢搜，车已正对却转离目标；IO4 不亮只是
        # 目标超出正前红外触发距离（30cm+），不是没对准 → 跳过 sweep 直接分类
        _align_event(align_log, "coarse", d, (0, 0), True, 2, 0.0, "already_front")
        return True, True
    if d in (1, 2):
        speed, dur = cfg["turn_45_speed"], cfg["turn_45_dur"]
        ang = dur / 0.6 * 45.0
    elif d == 3:
        speed = cfg["turn_135_speed"]
        dur = cfg.get("turn_135_left_rear_dur", cfg["turn_135_dur"])
        ang = dur / cfg["turn_135_dur"] * 135.0
    elif d == 4:
        speed = cfg["turn_135_speed"]
        dur = cfg.get("turn_135_right_rear_dur", cfg["turn_135_dur"])
        ang = dur / cfg["turn_135_dur"] * 135.0
    else:
        speed, dur, ang = cfg["turn_speed"], cfg["turn_big_dur"], 180.0
    sgn = _turn_sgn(d)
    tcmd = (int(speed * sgn), int(-speed * sgn))
    confirm_need = max(1, int(cfg.get("front_target_align_confirm_ticks", 2)))
    confirm_ticks = 0
    t0 = time.monotonic()
    print("  红外发现目标（方位 %d）→ 固定转 %.0f°" % (d, ang))
    _align_event(align_log, "coarse", d, tcmd, None, 0, 0.0, "start")
    try:
        if not _turn_ok(sensors):
            _align_event(align_log, "coarse", d, tcmd, None, 0, 0.0, "abort")
            print("  !! 转向前安全预检失败（掉台/断流）")
            return False, False
        ctrl.move_cmd(tcmd[0], tcmd[1])
        while time.monotonic() < t0 + dur:
            if not _turn_ok(sensors):
                _align_event(align_log, "coarse", d, tcmd, None, confirm_ticks,
                             time.monotonic() - t0, "abort")
                print("  !! 转向被打断（掉台/断流）——车没转够 %.0f°" % ang)
                return False, False
            active = _front_target_active(sensors)
            if active is None:
                _align_event(align_log, "coarse", d, tcmd, None, confirm_ticks,
                             time.monotonic() - t0, "io4_unavailable")
                print("  !! 正前 IO4 不可用 → 放弃对准")
                return True, False
            confirm_ticks = confirm_ticks + 1 if active else 0
            if confirm_ticks >= confirm_need:
                _align_event(align_log, "coarse", d, tcmd, True, confirm_ticks,
                             time.monotonic() - t0, "found")
                print("  [对准] 正前 IO4 连续触发 → 提前停转")
                return True, True
            time.sleep(0.02)
        _align_event(align_log, "coarse", d, tcmd, False, confirm_ticks,
                     time.monotonic() - t0, "finished")
        return True, False
    finally:
        ctrl.move_cmd(0, 0)


def _fine_align(ctrl, sensors, cfg, direction, already_aligned=False, align_log=None):
    """粗转后未确认时，同向连续慢转搜正前 IO4。

    IO4 只确认目标进入正前波束，不证明左右视觉对准——真正的视觉对准由
    分类后的 _visual_fine_align 负责。返回 "found"/"timeout"/"abort"/
    "unavailable"；"timeout"（目标 30cm+ IO4 不亮）不阻塞，照常视觉分类。
    """
    t0 = time.monotonic()
    direction = int(direction)
    try:
        if already_aligned:
            _align_event(align_log, "sweep", direction, (0, 0), True, 2, 0.0, "found")
            return "found"
        speed = int(cfg.get("front_target_align_speed", 400))
        timeout = float(cfg.get("front_target_align_timeout", 2.0))
        confirm_need = max(1, int(cfg.get("front_target_align_confirm_ticks", 2)))
        if speed <= 0 or timeout <= 0:
            _align_event(align_log, "sweep", direction, (0, 0), None, 0, 0.0,
                         "invalid_config")
            return "unavailable"
        sign = _turn_sgn(direction)
        command = (speed * sign, -speed * sign)
        confirm_ticks = 0
        if not _turn_ok(sensors):
            _align_event(align_log, "sweep", direction, command, None, 0, 0.0, "abort")
            return "abort"
        if _front_target_active(sensors) is None:
            _align_event(align_log, "sweep", direction, command, None, 0, 0.0,
                         "io4_unavailable")
            print("  !! 正前 IO4 不可用 → 不执行视觉分类")
            return "unavailable"
        print("  [正前搜寻] IO4 未确认 → 同向连续转动（最多 %.1fs）" % timeout)
        _align_event(align_log, "sweep", direction, command, False, 0, 0.0, "start")
        ctrl.move_cmd(command[0], command[1])
        deadline = t0 + timeout
        while time.monotonic() < deadline:
            if not _turn_ok(sensors):
                _align_event(align_log, "sweep", direction, command, None,
                             confirm_ticks, time.monotonic() - t0, "abort")
                return "abort"
            active = _front_target_active(sensors)
            if active is None:
                _align_event(align_log, "sweep", direction, command, None,
                             confirm_ticks, time.monotonic() - t0, "io4_unavailable")
                return "unavailable"
            confirm_ticks = confirm_ticks + 1 if active else 0
            if confirm_ticks >= confirm_need:
                _align_event(align_log, "sweep", direction, command, True,
                             confirm_ticks, time.monotonic() - t0, "found")
                print("  [正前搜寻] IO4 连续触发 → 目标已到车头范围，停转")
                return "found"
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        _align_event(align_log, "sweep", direction, command, False, confirm_ticks,
                     time.monotonic() - t0, "timeout")
        print("  [正前搜寻] %.1fs 内未确认正前目标（IO4 距离近，目标可能 30cm+）"
              "→ 照常视觉分类" % timeout)
        return "timeout"
    finally:
        ctrl.move_cmd(0, 0)


def _visual_available(vision):
    """视觉后端显式不健康 → 立即视为不可用。"""
    status = getattr(vision, "status", None)
    if not callable(status):
        return True
    try:
        return bool(status().available)
    except Exception:
        return False


def _visual_frame_at(sample):
    """取样本时间戳（非数值/非有限 → None）。"""
    try:
        frame_at = float(sample.timestamp)
    except (AttributeError, TypeError, ValueError):
        return None
    return frame_at if math.isfinite(frame_at) else None


def _visual_align_ok(sensors):
    """视觉微调安全门：数据新鲜 + 无真掉台（排除 front）+ 无 edge。

    - **front_edge_alert 不参与**：铲前红外台面上可低于阈值误触，微调是
      原地小转不出沿；
    - **fall 排除 front**：车头正对台边目标时 front 压黑带/过渡带，单帧
      抖动跌破 fall 阈值会误报"真掉台"拦死微调；后/左/右悬空（真掉台）仍拦。
    """
    try:
        if not sensors.data_fresh():
            return False
        fall = getattr(sensors, "fall_risk", None)
        if callable(fall):
            try:
                if fall(exclude=("front",)):
                    return False
            except TypeError:                # 桩/旧接口无 exclude 参数
                if fall():
                    return False
        edge = getattr(sensors, "edge_risk", None)
        if callable(edge) and edge():
            return False
        return True
    except Exception:
        return False


def _visual_fine_align(ctrl, sensors, vision, cfg, direction, kind="buff",
                       align_log=None):
    """分类确认后，把目标连续转进摄像头死区（center_x 闭环）。

    旧脉冲/停稳循环把 2 秒预算大多耗在停车上——这里转向命令在新帧之间
    持续保持，每 20ms 查安全门；超时宽窗让接近阶段接管（相机稳定但光心
    偏移时）。返回 "found"（连续新帧居中）/"best_effort"（超时但进入宽窗）/
    "timeout"/"abort"/"unavailable"。
    """
    t0 = time.monotonic()
    direction = int(direction)
    last_frame_at = None
    confirm_frames = 0
    try:
        timeout = float(cfg.get("vision_align_timeout", 2.0))
        deadband = float(cfg.get("vision_align_deadband", 60.0))
        large_error = float(cfg.get("vision_align_large_error", 120.0))
        small_speed = int(cfg.get("vision_align_small_speed", 400))
        large_speed = int(cfg.get("vision_align_large_speed", 400))
        confirm_need = int(cfg.get("vision_align_confirm_frames", 2))
        timeout_deadband = float(cfg.get(
            "vision_align_timeout_deadband", deadband * 2.0))
        image_center = float(cfg.get("image_center", 320.0))
        numeric = (timeout, deadband, large_error, timeout_deadband, image_center)
        if (vision is None or not all(math.isfinite(v) for v in numeric)
                or timeout <= 0 or deadband <= 0 or large_error <= deadband
                or small_speed <= 0 or large_speed <= 0
                or timeout_deadband < deadband
                or confirm_need < 1):
            _align_event(align_log, "visual", direction, (0, 0), None, 0,
                         0.0, "invalid_config")
            return "unavailable"

        deadline = t0 + timeout
        last_center_x = None
        command = (0, 0)
        turn_started = False
        saw_frame_after_turn = False
        _align_event(align_log, "visual", direction, (0, 0), None, 0,
                     0.0, "start")
        print("  [视觉微调] center_x 连续转向（总上限 %.1fs）" % timeout)
        while time.monotonic() < deadline:
            if not _visual_align_ok(sensors):
                _align_event(align_log, "visual", direction, (0, 0), None,
                             confirm_frames, time.monotonic() - t0, "abort")
                print("  !! 视觉微调被安全门/断流打断")
                return "abort"

            sample = vision.read()
            if not _visual_available(vision):
                _align_event(align_log, "visual", direction, (0, 0), None,
                             confirm_frames, time.monotonic() - t0, "unavailable")
                print("  !! 视觉微调时相机不可用 → 放弃本轮")
                return "unavailable"
            frame_at = _visual_frame_at(sample)
            is_new = (frame_at is not None
                      and (last_frame_at is None or frame_at > last_frame_at))
            if (not is_new or sample is None or not sample.found
                    or sample.kind != kind):
                # 无新帧/非目标类别：保持当前转向命令到下一张新帧
                time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
                continue
            try:
                center_x = float(sample.center_x)
            except (TypeError, ValueError, OverflowError):
                center_x = math.nan
            if not math.isfinite(center_x):
                last_frame_at = frame_at
                confirm_frames = 0
                continue

            if turn_started:
                saw_frame_after_turn = True
            last_center_x = center_x
            last_frame_at = frame_at
            error = image_center - center_x
            if abs(error) <= deadband:
                confirm_frames += 1
                _align_event(align_log, "visual", direction, (0, 0), None,
                             confirm_frames, time.monotonic() - t0, "centered_frame",
                             center_x=center_x, frame_at=frame_at)
                if confirm_frames >= confirm_need:
                    print("  [视觉微调] 连续 %d 张新帧居中 → 停转" % confirm_need)
                    _align_event(align_log, "visual", direction, (0, 0), None,
                                 confirm_frames, time.monotonic() - t0, "found",
                                 center_x=center_x, frame_at=frame_at)
                    return "found"
                if command != (0, 0):
                    ctrl.move_cmd(0, 0)
                    command = (0, 0)
                continue

            confirm_frames = 0
            if abs(error) > large_error:
                speed = large_speed
                band = "large_turn"
            else:
                speed = small_speed
                band = "small_turn"
            # 朝目标转（原车实车日志方向实锤）：目标在左（error>0）→ 左转、
            # 目标在右（error<0）→ 右转；TURN_SGN 做车旋向修正
            next_command = ((-speed * TURN_SGN, speed * TURN_SGN) if error > 0
                            else (speed * TURN_SGN, -speed * TURN_SGN))
            _align_event(align_log, "visual", direction, next_command, None, 0,
                         time.monotonic() - t0, band, center_x=center_x,
                         frame_at=frame_at)
            if next_command != command:
                ctrl.move_cmd(next_command[0], next_command[1])
                command = next_command
            turn_started = True
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))

        if (saw_frame_after_turn and last_center_x is not None
                and abs(image_center - last_center_x) <= timeout_deadband):
            _align_event(align_log, "visual", direction, (0, 0), None,
                         confirm_frames, time.monotonic() - t0, "best_effort",
                         center_x=last_center_x, frame_at=last_frame_at)
            print("  [视觉微调] 超时但目标已进入宽对准窗（%.0fpx）→ 交给接近闭环"
                  % abs(image_center - last_center_x))
            return "best_effort"
        _align_event(align_log, "visual", direction, (0, 0), None,
                     confirm_frames, time.monotonic() - t0, "timeout")
        print("  !! 视觉微调 %.1fs 超时 → 放弃本轮" % timeout)
        return "timeout"
    finally:
        ctrl.move_cmd(0, 0)


# =====================================================================
# 灰度安全适配层（消费你提供的 gray.py GrayRiskModel）
# =====================================================================
class GraySafety:
    """把你的灰度模型接成猎杀模式需要的安全钩子。

    controller : 提供 stale()/healthy（断流判定）；None=桩（恒新鲜）
    read_raw   : 返回 {"front"/"rear"/"left"/"right": 原始值} 的可调用
                 （如 GraySensor(adc_reader=...).read_raw，见模块示例）
    model      : GrayRiskModel 实例（white/zone 参考值在你的 gray.py 里调）

    钩子语义（对你 gray.py 输出的映射）：
    - data_fresh()           : 轮询数据新鲜（stale()/healthy）
    - fall_risk(exclude)     : white_hits 非空 = 压白边 = 掉台风险
                               （exclude 跳过方向；读不到 → True，fail-safe）
    - edge_risk()            : near_edge（模型预热期返回 False 不判定）
    - front/rear/left/right_gray_live() : 滤波后四路原始值（None=读不到）
    - min_zone()             : 四路 zone 最小值（绕行查边用）
    """

    def __init__(self, controller=None, read_raw=None, model=None):
        self._controller = controller
        self._read_raw = read_raw or (lambda: None)
        self._model = model
        self._obs = None

    def _observe(self):
        if self._model is None:
            return None
        try:
            raw = self._read_raw()
            self._obs = self._model.update(raw) if raw is not None else None
        except Exception:
            self._obs = None
        return self._obs

    def data_fresh(self):
        if self._controller is None:
            return True
        try:
            stale = getattr(self._controller, "stale", None)
            if callable(stale) and stale():
                return False
            healthy = getattr(self._controller, "healthy", None)
            if healthy is not None and not healthy:
                return False
            return True
        except Exception:
            return False

    def fall_risk(self, exclude=()):
        obs = self._observe()
        if obs is None or not obs.get("valid", False):
            return True                  # 读不到/坏值 → 按危机（fail-safe）
        hits = obs.get("white_hits") or ()
        return any(name not in exclude for name in hits)

    def edge_risk(self):
        obs = self._observe()
        if obs is None or not obs.get("valid", False):
            return True                  # fail-safe
        if not obs.get("ready", False):
            return False                 # 预热期不判定（与 zone 模型语义一致）
        return bool(obs.get("near_edge", False))

    def _gray_live(self, name):
        obs = self._observe()
        if obs is None:
            return None
        return (obs.get("filtered") or {}).get(name)

    def front_gray_live(self):
        return self._gray_live("front")

    def rear_gray_live(self):
        return self._gray_live("rear")

    def left_gray_live(self):
        return self._gray_live("left")

    def right_gray_live(self):
        return self._gray_live("right")

    def min_zone(self):
        obs = self._observe()
        if obs is None or not obs.get("valid", False):
            return None
        zone = obs.get("zone") or {}
        return min(zone.values()) if zone else None


def _drive_ok(sensors):
    """直行安全（debuff 绕行横移用）：数据新鲜、无真掉台、未到暗外圈。

    zone 判定（消费你的灰度模型）：min(zone) < GRAY["evade_zone"] → 到边停
    （原项目巡台同款判定；white/zone 参考值在你的 gray.py 里调）。
    读不到 zone → 按不安全处理（拦，fail-safe）。
    """
    try:
        if not sensors.data_fresh():
            return False
        if sensors.fall_risk():
            return False
        min_zone = getattr(sensors, "min_zone", None)
        if not callable(min_zone):
            return False
        z = min_zone()
        if z is None or z < float(GRAY["evade_zone"]):
            return False
        return True
    except Exception:
        return False


def _evade_debuff(ctrl, sensors, cfg, d):
    """识别到 debuff → 转离 90°、完整横移、回正后继续等待。

    对角红外在车转离后本来就会丢失，不能把"丢失"解释成已绕开——那会使
    横移只持续一个 20ms 分片，随后又回到 debuff 前方。这里固定完成几何
    动作：朝远离侧转 90°，横移 evade_forward_dur，再反向转 90° 回原朝向。
    每个分片只受数据新鲜/掉台/贴边安全门打断。返回 False=安全打断。
    """
    c = cfg
    s = int(c["turn_90_speed"])
    if d in (2, 4):        # 目标在右 → 左转离（TURN_SGN 修正车旋向）
        label, tcmd = "左转", (-s * TURN_SGN, s * TURN_SGN)
    elif d == 5:           # 正后 → 正前方无目标，不转直接前进远离
        label, tcmd = "不转", None
    else:                  # 0/1/3 或 d<0（方向未知——debuff 可能在正前远处，
                           # 前进=直冲）→ 一律右转离
        label, tcmd = "右转", (s * TURN_SGN, -s * TURN_SGN)
    print("  == 识别 debuff → 绕行（%s 90° + 横移 %ss + 回正）==" % (
        label, c.get("evade_forward_dur", 0.5)))
    try:
        if tcmd is not None and not _drive_sliced(
                ctrl, sensors, tcmd[0], tcmd[1], c["turn_90_dur"], _turn_ok):
            return False
        # 不能因目标红外进入侧向盲区而缩短横移；只让真实安全门打断
        if not _drive_sliced(ctrl, sensors, c["drive_speed"], c["drive_speed"],
                             c.get("evade_forward_dur", 0.5),
                             lambda s: _drive_ok(s)):
            return False
        if tcmd is not None and not _drive_sliced(
                ctrl, sensors, -tcmd[0], -tcmd[1], c["turn_90_dur"], _turn_ok):
            return False
        return True
    finally:
        ctrl.move_cmd(0, 0)


def _cfg(cfg, key, default):
    """cfg 取值：缺字段/None 用默认（可传 dict 或对象）。"""
    try:
        value = cfg.get(key, default)
    except AttributeError:
        return default
    return default if value is None else value


# =====================================================================
# 猎杀模式主循环（源: tools/calibrate.py cmd_hunt）
# =====================================================================
def hunt_run(ctrl, sensors, vision, cfg=None, actuator=None, on_locked=None,
             log_dir="data", stop_event=None):
    """猎杀模式（**不巡台、不扫描**）：车静止等待 → 红外亮才转向。

    流程：对角红外亮 → 固定角度粗转（早期停在正前 IO4 连续触发）→
    IO4 未确认同向慢搜 → 摄像头分类：
      buff   → 视觉 center_x 微调居中 → **锁定目标保持静止**，回调
               on_locked("buff")（锁定瞬间一次，推块动作以后由你注入）
      debuff → _evade_debuff 绕行（actuator 为 None 时不绕行，与源一致）
      unknown→ 同方位冷却后继续静止等待。
    后向红外单独亮 → 按方位 5（正后）掉头兜底。

    数据落盘：单 CSV log_dir/hunt_<时间戳>.csv（type 列区分 decision/align，
    默认 log_dir="data"）。stop_event（可选）：置位后主循环退出并兜底停车。

    ⚠️ 静止等待意味着目标必须在某个对角红外探测范围内才会被发现——
    放块时对准车的任意方向即可（对角红外覆盖前后左右 6 方位）。
    """
    c = cfg if cfg is not None else HUNT
    print("== 猎杀模式（车静止等红外→转向→IO4 确认→视觉分类/微调）==")
    print("  车静止不动；对角红外亮才转向；Ctrl+C 退出")
    logf = None
    logw = None
    align_log = None
    log_t0 = time.monotonic()
    try:
        os.makedirs(log_dir, exist_ok=True)
        base = os.path.join(log_dir, "hunt_%s" % time.strftime("%Y%m%d_%H%M%S"))
        logf = open(base + ".csv", "w", newline="", encoding="utf-8")
        logw = csv.writer(logf)
        logw.writerow(["t", "type", "dir", "ir1", "ir2", "kind", "action",
                       "phase", "direction", "left_cmd", "right_cmd",
                       "io4_active", "confirm_ticks", "center_x", "frame_at",
                       "elapsed", "result"])

        def _write_align(event):
            logw.writerow([round(time.monotonic() - log_t0, 3), "align",
                           "", "", "", "", "",
                           event["phase"], event["direction"], event["left"],
                           event["right"], event["io4_active"],
                           event["confirm_ticks"], event["center_x"],
                           event["frame_at"], round(event["elapsed"], 3),
                           event["result"]])
            logf.flush()

        align_log = _write_align
    except Exception:
        logf = None
        logw = None
    try:
        cooldown_dir = -1      # 同方位冷却（识别失败后防原地反复转同一角度）
        cooldown_t = 0.0
        while not (stop_event is not None and stop_event.is_set()):
            d = sensors.enemy_direction()
            # 后向兜底：enemy_direction 只用对角红外 IO0-3（左后/右后），
            # 后向红外不在判定里——目标在车正后方时对角后向可能不触发，
            # 只有后向亮 → 按方位 5（后）处理掉头
            if d < 0 and callable(getattr(sensors, "rear_obstacle", None)) \
                    and sensors.rear_obstacle():
                d = 5
                print("  [后向红外触发，按方位 5 处理]")
            if d >= 0 and not (d == cooldown_dir
                               and time.monotonic() - cooldown_t < c["engage_cooldown"]):
                # direction=d：用兜底后的方位（后向兜底时 enemy_direction()
                # 内部重读会得 -1 导致不转）
                ok_turn, io4_confirmed = _turn_to_target(
                    ctrl, sensors, c, direction=d, align_log=align_log)
                if not ok_turn:
                    # 转向被安全打断（掉台/断流）→ 不再识别/绕行，回循环顶
                    # 静止等待；同方位进冷却，防危机未解除原地再转
                    cooldown_dir = d
                    cooldown_t = time.monotonic()
                    continue
                cooldown_dir = d
                cooldown_t = time.monotonic()
                align_result = _fine_align(ctrl, sensors, c, d,
                                           already_aligned=io4_confirmed,
                                           align_log=align_log)
                if align_result in ("abort", "unavailable"):
                    # 安全打断/IO4 不可用 → 不分类，直接冷却
                    continue
                # "timeout"（IO4 未确认，目标 30cm+）→ 照常视觉分类
                # 转完等摄像头稳定再分类（转中画面糊）
                time.sleep(_cfg(c, "vision_settle_delay", 2.0))
                kind = _classify(vision)
                action = "识别=" + kind
                if kind == "buff":
                    visual_result = _visual_fine_align(
                        ctrl, sensors, vision, c, d, kind="buff",
                        align_log=align_log)
                    if visual_result not in ("found", "best_effort"):
                        continue
                    action = "视觉对准buff"
                    print("  == 识别 buff → 视觉已居中，锁定目标（保持静止）==")
                    # 测试阶段识别到 buff 后**只锁定对准**（车保持当前朝向
                    # 静止，等下一轮红外），不执行推块动作（源项目决定）
                    if callable(on_locked):
                        try:
                            on_locked("buff")
                        except Exception:
                            pass
                elif kind == "debuff" and actuator is not None:
                    action = "绕行"
                    if not _evade_debuff(ctrl, sensors, c, d):
                        continue        # 绕行中安全门打断 → 回循环顶静止等待
                    if sensors.enemy_direction() >= 0:
                        cooldown_dir = d
                        cooldown_t = time.monotonic() + (
                            _cfg(c, "evade_cooldown", 5.0) - c["engage_cooldown"])
                else:
                    print("  ?? 未识别到目标 → 冷却 %.1fs 后继续扫描"
                          % c["engage_cooldown"])
                ir = sensors.front_ir_values() if callable(
                    getattr(sensors, "front_ir_values", None)) else None
                ir1 = ir[0] if ir else None
                ir2 = ir[1] if ir else None
                if logw:
                    logw.writerow([round(time.monotonic() - log_t0, 3),
                                   "decision", d, ir1, ir2, kind, action,
                                   "", "", "", "", "", "", "", "", "", ""])
            else:
                # 无目标：**静止等待**（不扫描不旋转，等对角红外亮才转向）
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n猎杀结束")
    finally:
        try:
            ctrl.move_cmd(0, 0)
        except Exception:
            pass
        if logf is not None:
            try:
                logf.close()
            except Exception:
                pass
            print("  数据已存：%s" % logf.name)


if __name__ == "__main__":
    print(__doc__)
