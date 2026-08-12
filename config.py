"""真机运行参数统一入口；生产与 dev 程序只从这里取可调参数。"""

# ---------- 传感器接线 ----------
GRAY_CHANNELS = {
    "front": 2,
    "rear": 3,
    "left": 0,
    "right": 1,
}
IR_CHANNELS = {
    "left": 5,
    "right": 4,
}
DIGI_IR_PINS = {
    "left_rear": 100,
    "left_front": 101,
    "right_rear": 102,
    "right_front": 103,
    "rear": 104,
    "front": 105,
}
DIGI_IR_BITS = {
    "left_rear": 2,
    "left_front": 0,
    "right_rear": 3,
    "right_front": 1,
    "rear": 5,
    "front": 4,
}
DIGI_IR_ACTIVE_LEVEL = 0
GRAY_ADC_MAX = 10000.0
IR_ADC_MAX = 10000.0

# ---------- 灰度模型（来源：data/gray_model.csv） ----------
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

# ---------- 前头 ADC 对齐（来源：data/front_adc_model.csv） ----------
IR_ALIGNMENT_FILTER_WINDOW = 9
IR_ALIGNMENT_DIFF_LOW = 331.0
IR_ALIGNMENT_DIFF_HIGH = 497.0
IR_ALIGNMENT_CONFIRM = 3
IR_ALIGNMENT_SIGNAL_MIN = 377.0

# 来源：data/motor_linear_calibration.csv。真机确认整体反向，400 以下不能可靠驱动。
CHASSIS_MOTOR_INVERT = True
CHASSIS_MOTOR_SWAP = False
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

# 来源：data/motor_turn_calibration.csv，左右转实测结果一致。
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

# ---------- 掉台回归 ----------
REENTRY_FALL_CONFIRM = 3
REENTRY_CORRECT_TURN_SPEED = PATROL_MIN_ACTIVE_SPEED
REENTRY_CORRECT_TIMEOUT = 3.0
REENTRY_APPROACH_SPEED = 700               # 大力前冲撞墙速度（一次冲到底，撞上立即停车防堵转）
REENTRY_APPROACH_TIMEOUT = 2.0             # 大力冲撞兜底时长：超时未贴墙则停车（真机调）
REENTRY_APPROACH_TOUCH_SIGNAL = 1000.0     # 前头模拟红外 signal≥此值=已贴墙；来源 front_adc_summary.csv（正对着墙 p99≈1315、居中 p99≈710）
REENTRY_REVERSE_SPEED = PATROL_RECOVER_SPEED   # 掉台矫正完毕倒车速度（与巡台恢复分开调）
REENTRY_REVERSE_TIMEOUT = 3.0
