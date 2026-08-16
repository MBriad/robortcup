#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
参数集中管理 (push_pack/push_block_config.py) ——「调参数只改这一个文件」

与 push_block.py 成对使用。来源：
- 灰度参照/通道 = detail/config.py 新车实测（zone 语义：台心=1、边缘=0）
- 铲下红外 = shovel_pack 2026-08-15 实测（⚠️ 新车极性：悬空=信号高、台内=低，
  通道 AD4/5；detail/config.py 的 SHOVEL 段还是 08-12 旧标定（AD6/7、悬空=低），
  以本文件为准，确认后需同步更新 detail/config.py）
- 推块档位速度 = 新车电机可靠下限 400 约束（原项目 300/250 不转，不适用）
"""

# ---------- 灰度（detail/config.py 新车实测，zone 模型） ----------
GRAY_CHANNELS = {
    "front": 2,    # 新车左右插反已对调（2026-08-14 scan）
    "rear": 3,
    "left": 1,
    "right": 0,
}
GRAY_ADC_MAX = 10000.0
GRAY_FILTER_WINDOW = 3
GRAY_EDGE_REFERENCE = {
    "front": 494.0, "rear": 632.0, "left": 747.0, "right": 675.0,
}
GRAY_CENTER_REFERENCE = {
    "front": 817.0, "rear": 1136.0, "left": 1159.0, "right": 973.0,
}

# ---------- 铲下红外（shovel_pack 2026-08-15 实测，悬空=信号高） ----------
SHOVEL_IR_CHANNELS = {"left": 4, "right": 5}   # 铲子底下 2 路模拟红外 AD4/5
# 阈值来源（9 帧中值滤波后，shovel_pack 注释）：悬空 min(两路) p01=1291、
# 台内 min(两路) p99=46 → ENTER 取中点 670；悬空 max(两路) p01=1452、
# 台内 max(两路) p99=1265 → CLEAR 取中点 1360。
# 判据：min(两路)>ENTER = 悬空触发、max(两路)<CLEAR = 已收回（滞回）。
SHOVEL_HANG_ENTER = 670.0
SHOVEL_HANG_CLEAR = 1360.0
SHOVEL_FILTER_WINDOW = 9
SHOVEL_HANG_CONFIRM = 3        # 悬空确认帧数（防抖）
SHOVEL_REVERSE_SPEED = 400     # 倒车收回速度（≥ 电机死区下限）
SHOVEL_REVERSE_MIN_SECONDS = 0.3
SHOVEL_REVERSE_TIMEOUT = 3.0

# ---------- 推块档位（原项目 actuator.push_and_retreat(block=True) 语义） ----------
PUSH = {
    # 两档速度：快 500 冲深 → 慢 325（2026-08-15 用户拍板改 325；⚠️ 此前实测
    # 记录新车 400 以下不可靠（300 曾实测不转）——上车验证，不转改回 400）
    "block_fast_speed": 500,
    "block_slow_speed": 325,
    # 灰度分界（zone 语义，front 单路；依据 detail/data 实测分位 2026-08-15）：
    #   台面-内环 front p5=0.47 / 台面-更内环 p5=0.85
    #   浅色边缘 front p5=0.11 p50=0.30 / 边缘姿态 p50=-0.07
    #   黑带 p50=-0.49 / 悬空(掉台) p50=-0.48（front 单路分不开黑带与悬空）
    "block_slow_threshold": 1.3,    # 快→慢（删爬行档后唯一减速线）：front zone
                                    # ≤ 1.3 进慢档（2026-08-15 白边掉台修复：
                                    # push_20260815_211837 实测白边段 front 变亮
                                    # 1.2→1.8、旧 0.45 减速线永不触发 → 500 全速
                                    # 压白边；1.3 让起步即慢推，快档仅更内环亮区
                                    # （front≥1.3）使用）
    "block_front_suspend": -0.45,   # front 单路悬空兜底：≤ -0.45 停+退、**不确认
                                    # 推下**（黑带 -0.49、悬空 -0.48 分不开，语义同
                                    # 原项目"比黑带还暗"：到黑带仍无双路悬空 =
                                    # 没推下去）
    "rear_abort_zone": -0.45,       # 前冲/倒车安全门：后路 zone ≤ -0.45（车尾出沿）
                                    # 或断流 → 立即打断；front/left/right 压沿是推块
                                    # 正常过程不打断（原项目 exclude 同款精神）
    "side_abort_zone": -0.45,       # 侧向悬空兜底：左/右 zone ≤ -0.45 → 正常结束
                                    # 前冲进倒车（斜推时侧路先悬空，不确认推下）
    "block_slow_debounce": 6,       # 档位切换去抖帧数（武字抗干扰）
    "block_stage1_timeout": 2.0,    # 快推最长时长（超时强制进慢推）
    "block_stage2_timeout": 6.0,    # 慢推最长时长（超时未悬空 → 后退不计推下）
    "block_forward_dur": 8.0,       # 前冲总时长上限（= 快推 2s + 慢推 6s）
    "block_back_dur": 1.0,          # 推完后退时长（guard 收回 + 固定时长）
}

# ---------- 视觉差速纠偏（可选；vision_backend 就绪后注入） ----------
VISION = {
    "image_center": 320.0,      # 640 宽画面中心
    "gain": 0.6,                # 误差→转向量（px → 轮速差）
    "turn_max": 150,            # 转向量上限（≈快推 500 的 30%，保证仍是前进）
    "deadband": 30.0,           # 中心死区（px）
}

# ---------- 车轮方向（detail/config.py 新车实测） ----------
CHASSIS_MOTOR_INVERT = False
CHASSIS_MOTOR_SWAP = False
