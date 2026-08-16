#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
敌人推动模块参数集中管理 (enemy_push_pack/enemy_push_config.py) ——「调参数只改这一个文件」

与 enemy_push.py 成对使用。来源：
- 数字红外通道/极性 = 你的 detail/config.py DIGI_IR_BITS（新车 08-14 scan）
- 铲下红外 = shovel_pack 08-15 标定（悬空=高，AD4/5）
- 灰度参照 = detail/config.py 新车实测（zone 语义）
- 推速 700 = 你的 REENTRY_APPROACH_SPEED 实测撞墙档（原定 800，用户拍板 700）
"""

# ---------- 数字红外（= detail/config.py DIGI_IR_BITS，io0=前 io3=左前
# io2=右前 io1=左后 io4=右后 io6=后，0=触发） ----------
DIAG_IR_CHANNELS = {
    "left_front": 3,
    "right_front": 2,
    "left_rear": 1,
    "right_rear": 4,
}
FRONT_TARGET_IO_CH = 0
REAR_IR_CHANNELS = [6]
IR_ACTIVE_LOW = True
IO_FILTER_WINDOW = 5

# ---------- 灰度（detail 新车实测，zone 模型：台心=1、边缘=0） ----------
GRAY_CHANNELS = {"front": 2, "rear": 3, "left": 1, "right": 0}
GRAY_ADC_MAX = 10000.0
GRAY_FILTER_WINDOW = 3
GRAY_EDGE_REFERENCE = {
    "front": 494.0,
    "rear": 632.0,
    "left": 747.0,
    "right": 675.0,
}
GRAY_CENTER_REFERENCE = {
    "front": 817.0,
    "rear": 1136.0,
    "left": 1159.0,
    "right": 973.0,
}

# ---------- 铲下红外（shovel_pack 08-15：悬空=高，AD4/5） ----------
SHOVEL_IR_CHANNELS = {"left": 4, "right": 5}
SHOVEL_HANG_ENTER = 670.0  # min(两路)>此值 = 铲子悬空
SHOVEL_HANG_CLEAR = 1360.0  # max(两路)<此值 = 已收回台内
SHOVEL_FILTER_WINDOW = 9
SHOVEL_HANG_CONFIRM = 3
SHOVEL_REVERSE_SPEED = 400
SHOVEL_REVERSE_MIN_SECONDS = 0.3
SHOVEL_REVERSE_TIMEOUT = 3.0

# ---------- 敌人推动时序 ----------
ATTACK = {
    # 只认前头红外 io0（用户拍板 2026-08-15 晚）：亮就一直推，其他红外不管；
    # 停线 = 铲子防掉（ShovelGuard 悬空），无时长上限、不转离
    "push_speed": 700,
    # 分级速度（2026-08-16）：front zone < slow_threshold（白边之前）→ 慢档，
    # 给铲子防掉（9 帧滤波 ~0.24s）留反应时间；去抖防武字
    "slow_speed": 350,
    "slow_threshold": 1.3,
    "slow_debounce": 6,
    # 倒车收回（ShovelGuard 反馈式：收回信号 + 固定时长）
    "retreat_speed": 400,
    "retreat_dur": 1.0,
    # 冷却：仅安全打断后使用（断流/后路悬空/SAFE_STOP）；正常推完直接回等待
    "cooldown": 3.0,
    # 两次攻击之间短停（秒，2026-08-16）：退完立刻再推会突刺双退，等红外
    # 熄灭又太被动——短停 0.5s 后 io0 还亮就继续打（多次攻击）
    "attack_pause": 0.5,
    "rear_abort_zone": -0.45,  # 后路悬空：立即打断（车尾出沿）
    # 白边灰度保护（2026-08-16）：四路 zone 全部 ≥ white_bright_threshold 连续
    # white_confirm_ticks 帧 = 车整体开上白边（此场景铲下红外不触发）→ 中断推、
    # 倒车退回不确认。attack_20260816_092127 验证：4 次掉台前 2.5s 内全部命中，
    # 亮区推敌误触 7 段（推→退→再推多转几轮）
    "white_bright_threshold": 1.4,
    "white_confirm_ticks": 6,
}

# ---------- 车轮方向（detail 新车实测） ----------
CHASSIS_MOTOR_INVERT = False
CHASSIS_MOTOR_SWAP = False
