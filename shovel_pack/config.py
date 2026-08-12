"""真机运行参数统一入口；生产与 dev 程序只从这里取可调参数。

每组参数注明来源（对应模块文件或数据 CSV）；模块内保留独立默认值，
生产/dev 运行时一律以本文件值为准。
"""

# ---------- 传感器接线（gray.py / ir.py / digi_ir.py 通道映射） ----------
GRAY_CHANNELS = {          # gray.py GraySensor.channels
    "front": 2,
    "rear": 3,
    "left": 0,
    "right": 1,
}
IR_CHANNELS = {            # ir.py IrSensor.channels
    "left": 5,
    "right": 4,
}
DIGI_IR_PINS = {           # digi_ir.py DIGI_IR_PINS
    "left_rear": 100,
    "left_front": 101,
    "right_rear": 102,
    "right_front": 103,
    "rear": 104,
    "front": 105,
}
DIGI_IR_BITS = {           # digi_ir.py DigiIR.bits
    "left_rear": 2,
    "left_front": 0,
    "right_rear": 3,
    "right_front": 1,
    "rear": 5,
    "front": 4,
}
DIGI_IR_ACTIVE_LEVEL = 0   # digi_ir.py DigiIR.active_level
GRAY_ADC_MAX = 10000.0     # gray.py
IR_ADC_MAX = 10000.0       # ir.py

# ---------- 灰度模型（gray.py GrayRiskModel 构造参数；来源：data/gray_model.csv） ----------
GRAY_FILTER_WINDOW = 3
GRAY_EDGE_REFERENCE = {
    "front": 666.0, "rear": 798.0, "left": 458.0, "right": 1143.0,
}
GRAY_CENTER_REFERENCE = {
    "front": 1033.5, "rear": 1257.0, "left": 817.0, "right": 1622.0,
}
GRAY_WHITE_REFERENCE = {
    "front": 1924.0, "rear": 2283.0, "left": 1625.0, "right": 2507.0,
}
GRAY_WHITE_ENTER = {
    "front": 1798.0, "rear": 2154.0, "left": 1508.0, "right": 2395.0,
}
GRAY_WHITE_CLEAR = {
    "front": 1731.0, "rear": 2085.0, "left": 1445.0, "right": 2337.0,
}
GRAY_NEAR_EDGE_ENTER = 0.50
GRAY_NEAR_EDGE_CLEAR = 0.65

# ---------- 前头 ADC 对齐（ir.py IrAlignmentModel 构造参数；来源：data/front_adc_model.csv） ----------
IR_ALIGNMENT_FILTER_WINDOW = 9
IR_ALIGNMENT_DIFF_LOW = 331.0
IR_ALIGNMENT_DIFF_HIGH = 497.0
IR_ALIGNMENT_CONFIRM = 3
IR_ALIGNMENT_SIGNAL_MIN = 377.0

# 电机修正（up_controller.py UpController motor_invert/motor_swap 参数；
# 来源：data/motor_linear_calibration.csv）。真机确认整体反向，400 以下不能可靠驱动。
CHASSIS_MOTOR_INVERT = True
CHASSIS_MOTOR_SWAP = False

# ---------- 巡台（ring_patrol.py RingPatrolController 参数） ----------
PATROL_MIN_ACTIVE_SPEED = 400
PATROL_CRUISE_LINEAR = 450
PATROL_CRUISE_TURN = 50       # 输出：左 500、右 400，顺时针缓弧
PATROL_MEDIUM_LINEAR = 425
PATROL_MEDIUM_TURN = 25       # 输出：左 450、右 400
# 小转调参：当前输出左/右 400/500；增大转向量会更急，但要保持 linear-turn >= 400。
PATROL_EDGE_AVOID_LINEAR = 450
PATROL_EDGE_AVOID_TURN = 50
PATROL_EDGE_ARC_CHECK_SECONDS = 0.20
PATROL_EDGE_TURN_ANGLE = 180.0
PATROL_FAST_ZONE_SCORE = 0.90
# 小转阈值：提高会更早小转，降低会更晚小转；必须高于大转阈值。
PATROL_SMALL_TURN_ZONE_SCORE = 0.75
PATROL_COMMAND_LIMIT = 1023

# 转向速度/时长标定（ring_patrol.py EDGE_TURN 与 reentry.py TURN_* 共用；
# 来源：data/motor_turn_calibration.csv，左右转实测结果一致）。
MOTOR_TURN_CALIBRATION = {
    22.5: (400, 0.6),
    45.0: (500, 0.6),
    90.0: (500, 1),
    112.5: (600, 0.8),
    135.0: (600, 0.9),
    165.0: (600, 1.0),
    180.0: (625, 1.2),
    225.0: (700, 1.0),
    360.0: (725, 1.4),
}
PATROL_WHITE_ESCAPE_SPEED = 400
PATROL_WHITE_ESCAPE_SECONDS = 0.6
PATROL_RECOVER_SPEED = 400
PATROL_RECOVER_SECONDS = 0.6
PATROL_RECOVER_STEP_CM = 21.5  # 400 速度运行 0.6 秒的前进/后退实测距离
PATROL_RECOVER_MIN_IMPROVEMENT = 0.03

PATROL_WHITE_CONFIRM = 2
PATROL_NEAR_CONFIRM = 3
PATROL_STALE_SECONDS = 0.20

# ---------- 掉台回归（reentry.py ReentryController 参数） ----------
REENTRY_FALL_CONFIRM = 3
REENTRY_CORRECT_TURN_SPEED = PATROL_MIN_ACTIVE_SPEED
REENTRY_CORRECT_TIMEOUT = 3.0
REENTRY_APPROACH_SPEED = 700               # 大力前冲撞墙速度（一次冲到底，撞上立即停车防堵转）
REENTRY_APPROACH_TIMEOUT = 2.0             # 大力冲撞兜底时长：超时未贴墙则停车（真机调）
REENTRY_APPROACH_TOUCH_SIGNAL = 1000.0     # 前头模拟红外 signal≥此值=已贴墙；来源 front_adc_summary.csv（正对着墙 p99≈1315、居中 p99≈710）
REENTRY_REVERSE_SPEED = PATROL_RECOVER_SPEED   # 掉台矫正完毕倒车速度（与巡台恢复分开调）
REENTRY_REVERSE_TIMEOUT = 3.0

# ---------- 铲子防掉落（shovel_guard.py ShovelGuard 参数） ----------
# 阈值来源 2026-08-12 固定姿态采集（data/shovel_hang_chunhei.csv / shovel_stage_*.csv，
# 9 帧中值滤波后）：出台纯黑主区 signal_max≤1354，台内中部≥1442。
SHOVEL_IR_CHANNELS = {"left": 6, "right": 7}   # 铲子底下 2 路模拟红外；接线用 dev/shovel_tool.py scan 确认
SHOVEL_ADC_MAX = IR_ADC_MAX
SHOVEL_FILTER_WINDOW = 9       # 信号中值滤波窗口（同 ir.py 前头红外对齐模型）
SHOVEL_HANG_ENTER = 1400.0     # 滤波后 signal_max<此值 = 铲子悬空（出台纯黑 ≤1354，台内 ≥1442）
SHOVEL_HANG_CLEAR = 1450.0     # 倒车后滤波后 signal_max>此值 = 已收回台内（滞回，> ENTER）
SHOVEL_HANG_CONFIRM = 3        # 悬空确认帧数（防抖）
SHOVEL_REVERSE_SPEED = PATROL_MIN_ACTIVE_SPEED   # 倒车收回速度（≥ 电机死区下限）
SHOVEL_REVERSE_MIN_SECONDS = 0.3   # 最短倒车时长，防信号抖动提前停
SHOVEL_REVERSE_TIMEOUT = 3.0       # 倒车超时兜底 → 停车待命

# ---------- YOLO 能量块对准（vision_tracker.py；初值待 dev/vision_tracker.py CSV 标定） ----------
VISION_IMAGE_WIDTH = 320            # YOLO 输出横坐标基准；先用部署模型输入宽度
VISION_LOOP_HZ = 50.0               # 电机安全轮询频率；YOLO 实际约 8 FPS
VISION_MAX_AGE_MS = 450             # 超过约 3 帧周期仍无新结果，立即停车
VISION_ERROR_FILTER_ALPHA = 0.45    # 横向归一化误差 EMA，新数据权重
VISION_DEAD_ZONE = 0.10             # 中心左右各 16 px 内停止转向
VISION_TURN_KP = 600.0              # 归一化横向误差到原地转向速度
VISION_TURN_MIN_SPEED = 400         # 实测电机可靠动作下限
VISION_TURN_MAX_SPEED = 600         # 首轮地面测试限速，避免 8 FPS 下转过头
