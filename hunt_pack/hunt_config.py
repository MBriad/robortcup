#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
猎杀模式参数集中管理 (dev/hunt_config.py) ——「调参数只改这一个文件」

与 dev/hunt.py 成对使用，仅依赖标准库。数值取自你的 detail/config.py
新车标定（2026-08-14 实测）——那边改了什么，这里同步。
"""

# ---------- 数字红外接线/极性（= 你的 detail/config.py DIGI_IR_BITS，io_data 位） ----------
# 新车 2026-08-14 scan 实测：io0=前 io3=左前 io1=左后 io2=右前 io4=右后 io6=后
#（0=触发，见 IR_ACTIVE_LOW）
DIAG_IR_CHANNELS = {
    "left_front": 3,    # 左前 = io3
    "right_front": 2,   # 右前 = io2
    "left_rear": 1,     # 左后 = io1
    "right_rear": 4,    # 右后 = io4
}

# 正前方数字红外（IO）：正前目标判定走此通道（0=触发，IR_ACTIVE_LOW）
FRONT_TARGET_IO_CH = 0

# 车后数字红外（IO）：后向兜底（块在正后方时对角后向可能不触发，只有后向亮）
REAR_IR_CHANNELS = [6]

# 数字红外（对角/正前/后向）反射式：0=触发
#（= 你的 detail/config.py DIGI_IR_ACTIVE_LEVEL=0，新车 2026-08-14）
IR_ACTIVE_LOW = True

# IO 多数表决滤波窗口（帧）
IO_FILTER_WINDOW = 5

# ---------- 灰度安全（模型在你的 gray.py；这里只有猎杀用到的判定参数） ----------
# 白色/zone 参考值与滤波窗口在**你的 gray.py** 里调；GraySafety 直接消费
# GrayRiskModel 的输出（white_hits/near_edge/zone）。
GRAY = {
    # debuff 绕行横移查边：min(zone) < 此值=到边停
    #（= 你的 detail/config.py GRAY_NEAR_EDGE_ENTER，新车 2026-08-14 调参
    #   0.50→0.35：内环巡行 zone≈0.5~0.9，0.50 会误触）
    "evade_zone": 0.35,
}

# ---------- 猎杀模式参数（取自你的 detail/config.py 新车标定 2026-08-14） ----------
# 电机修正 invert/swap 由你的 up_controller 处理（CHASSIS_MOTOR_INVERT/SWAP=False）；
# 本文件所有速度 ≥400（新车可靠驱动下限实测）。

# 车旋向修正：hunt.py 原约定 move_cmd(s,-s)=右转/顺时针（原车实测，与你
# motor_tool 的 (speed,-speed)=原地右转 一致）。
# 2026-08-14 曾观察到"车朝目标反方向转"——根因是 DIGI_IR_BITS 旧映射
#（08-10），重 scan 修正映射后旋向约定无需翻 → TURN_SGN=+1（原约定）。
# 所有转向（粗转/慢搜/视觉微调/debuff 绕行）统一乘这个符号；
# 上车架起车轮验证后若真的反了，只把这里改成 -1。
TURN_SGN = 1

HUNT = {
    # 粗转标定：固定角度转（早期停在正前 IO4 连续触发）
    # 你的 MOTOR_TURN_CALIBRATION：45°(500,0.6) / 135°(550,0.8) /
    # 180°(600,0.95~0.975)——左右转分开查表，此处取通用值
    "turn_45_speed": 500, "turn_45_dur": 0.6,    # 方位1/2：45°
    "turn_135_speed": 550, "turn_135_dur": 0.8,  # 方位3/4：135°
    "turn_135_left_rear_dur": 0.8,               # 左后单独标定（实测过头时调小）
    "turn_135_right_rear_dur": 0.8,              # 右后单独标定
    "turn_speed": 600, "turn_big_dur": 0.95,     # 方位5（正后）：180° 掉头
    "front_target_align_confirm_ticks": 2,       # 正前 IO4 连续帧数确认（防毛刺）
    # IO4 未确认时的同向慢搜（_fine_align）
    "front_target_align_speed": 400,             # 搜速（原车可靠起转下限）
    "front_target_align_timeout": 1.5,           # 搜转上限（秒）：超时照常视觉分类
                                                 #（IO4 探测距离近 ≤30cm，目标远不亮）
    # 冷却
    "engage_cooldown": 2.5,      # 同方位识别/对准失败后冷却（秒），防原地反复转
    "evade_cooldown": 4.0,       # debuff 绕行后全方位忽略目标红外的时长（秒）
    # 视觉分类 / 微调
    "vision_settle_delay": 0.15, # 转完等画面稳再分类（转中画面糊）
    "vision_align_timeout": 2.0,
    "vision_align_deadband": 30.0,      # |误差| ≤ 30px 算居中
    "vision_align_large_error": 120.0,  # |误差| > 120px 用大档速度
    "vision_align_small_speed": 400,
    "vision_align_large_speed": 400,
    "vision_align_confirm_frames": 2,   # 连续 2 张新帧居中才确认
    "vision_align_timeout_deadband": 45.0,  # 超时但最后误差 ≤ 45px 也算成功
    "image_center": 320.0,       # 图像中心 x（原项目视觉输出 640 宽）
    # debuff 绕行
    "turn_90_speed": 550, "turn_90_dur": 0.75,  # 90° 转（你的标定 90°=(550, 0.75/0.725)）
    "drive_speed": 400,          # 横移速度
    "evade_forward_dur": 0.8,    # 横移时长（秒，≈32cm@400）——必须走满，
                                 # 不能因红外进盲区提前截短
}
