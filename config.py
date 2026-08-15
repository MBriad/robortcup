"""真机运行参数统一入口；生产与 dev 程序只从这里取可调参数。

每组参数注明来源（对应模块文件或数据 CSV）；模块内保留独立默认值，
生产/dev 运行时一律以本文件值为准。
"""

# ---------- 传感器接线（gray.py / ir.py / digi_ir.py 通道映射） ----------
GRAY_CHANNELS = {  # gray.py GraySensor.channels
    # 新车接线左右插反（2026-08-14 scan 确认）：left/right 通道对调。
    "front": 2,
    "rear": 3,
    "left": 1,
    "right": 0,
}
IR_CHANNELS = {  # ir.py IrSensor.channels
    # 新车 2026-08-15 scan 实测：前方红外左=7、右=6。
    "left": 7,
    "right": 6,
}
DIGI_IR_PINS = {  # digi_ir.py DIGI_IR_PINS
    "left_rear": 100,
    "left_front": 101,
    "right_rear": 102,
    "right_front": 103,
    "rear": 104,
    "front": 105,
}
DIGI_IR_BITS = {  # digi_ir.py DigiIR.bits
    # 新车 2026-08-14 scan 实测：io0=前 io3=左前 io1=左后 io2=右前 io4=右后 io6=后
    "left_rear": 1,
    "left_front": 3,
    "right_rear": 4,
    "right_front": 2,
    "rear": 6,
    "front": 0,
}
DIGI_IR_ACTIVE_LEVEL = 0  # digi_ir.py DigiIR.active_level
GRAY_ADC_MAX = 10000.0  # gray.py
IR_ADC_MAX = 10000.0  # ir.py

# ---------- 灰度模型（gray.py GrayRiskModel 构造参数；来源：data/gray_model.csv，
# 新车 2026-08-14 重采重算） ----------
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
GRAY_WHITE_REFERENCE = {
    "front": 1704.0,
    "rear": 2144.0,
    "left": 1920.0,
    "right": 1858.0,
}
GRAY_WHITE_ENTER = {
    "front": 1560.0,
    "rear": 1946.0,
    "left": 1790.0,
    "right": 1720.0,
}
GRAY_WHITE_CLEAR = {
    "front": 1479.0,
    "rear": 1835.0,
    "left": 1718.0,
    "right": 1645.0,
}
# 新车 2026-08-14 调参：内环巡行 zone≈0.5~0.9，0.50 会误触大转，降到 0.35。
GRAY_NEAR_EDGE_ENTER = 0.35
GRAY_NEAR_EDGE_CLEAR = 0.65

# ---------- 前头 ADC git对齐（ir.py IrAlignmentModel 构造参数；来源：data/front_adc_model.csv） ----------
IR_ALIGNMENT_FILTER_WINDOW = 9
IR_ALIGNMENT_DIFF_LOW = 331.0
IR_ALIGNMENT_DIFF_HIGH = 497.0
IR_ALIGNMENT_CONFIRM = 3
IR_ALIGNMENT_SIGNAL_MIN = 377.0

# 电机修正（up_controller.py UpController motor_invert/motor_swap 参数）。
# 新车 2026-08-13 实测：invert=False（前进后退正常）；400 以下不能可靠驱动。
CHASSIS_MOTOR_INVERT = False
CHASSIS_MOTOR_SWAP = False

# ---------- 巡台（ring_patrol.py RingPatrolController 参数） ----------
PATROL_MIN_ACTIVE_SPEED = 400
PATROL_CRUISE_LINEAR = 450
PATROL_CRUISE_TURN = 50  # 输出：左 500、右 400，顺时针缓弧
PATROL_MEDIUM_LINEAR = 425
PATROL_MEDIUM_TURN = 25  # 输出：左 450、右 400
# 小转调参（2026-08-14 新车两轮实测：50、80 都偏小）：输出左/右 640/400；
# 增大转向量会更急，但要保持 linear-turn >= 400。
PATROL_EDGE_AVOID_LINEAR = 520
PATROL_EDGE_AVOID_TURN = 120
PATROL_EDGE_ARC_CHECK_SECONDS = 0.50
PATROL_EDGE_TURN_ANGLE = 180.0
PATROL_FAST_ZONE_SCORE = 0.90
# 小转阈值：提高会更早小转，降低会更晚小转；必须高于大转阈值。
# 新车 2026-08-14：更内环 zone≈0.83~1.7，0.75 进入浅灰区才小转太晚，提到 0.85。
PATROL_SMALL_TURN_ZONE_SCORE = 0.85
# 避让退出滞回：zone 恢复到该值以上才退出 EDGE_AVOID，防 0.85 边界来回振荡。
PATROL_EDGE_AVOID_CLEAR = 0.95
# 小转无改善时，只有 zone 低于此值才升级 180° 大转；更内环小转不改善只继续弧线。
PATROL_EDGE_TURN_ZONE_MAX = 0.60
PATROL_COMMAND_LIMIT = 1023

# 转向速度/时长标定（ring_patrol.py EDGE_TURN 与 reentry.py TURN_* 共用；
# 来源：data/motor_turn_calibration.csv，新车 2026-08-14 实测，左右转分开查表）。
MOTOR_TURN_CALIBRATION = {
    "left": {
        10.0: (400, 0.5),
        22.5: (400, 1.0),
        45.0: (500, 0.6),
        90.0: (550, 0.75),
        135.0: (550, 0.8),
        180.0: (600, 0.95),
    },
    "right": {
        10.0: (400, 0.5),
        22.5: (400, 1.0),
        45.0: (500, 0.6),
        90.0: (550, 0.725),
        135.0: (550, 0.8),
        180.0: (600, 0.975),
    },
}
PATROL_WHITE_ESCAPE_SPEED = 400
PATROL_WHITE_ESCAPE_SECONDS = 0.6
PATROL_RECOVER_SPEED = 550
# 新车 2026-08-14 实测：400 速度后退 1.5 秒更合理（1.0 仍偏短）。
PATROL_RECOVER_SECONDS = 1.5
PATROL_RECOVER_STEP_CM = (
    21.5  # 旧车 400×0.6s 距离；新车 550×1.5s 待重标（直线标定暂缓）
)
PATROL_RECOVER_MIN_IMPROVEMENT = 0.03

PATROL_WHITE_CONFIRM = 2
# 白边判定 zone 门槛：武字白实测 zone>=0.7，边界白 <0.3，取 0.5 区分。
PATROL_WHITE_ZONE_MAX = 0.5
PATROL_NEAR_CONFIRM = 3
PATROL_STALE_SECONDS = 0.20

# ---------- 掉台回归（reentry.py ReentryController 参数） ----------
REENTRY_FALL_CONFIRM = 3
REENTRY_CORRECT_TURN_SPEED = PATROL_MIN_ACTIVE_SPEED
REENTRY_CORRECT_TIMEOUT = 3.0
REENTRY_APPROACH_SPEED = 700  # 大力前冲撞墙速度（一次冲到底，撞上立即停车防堵转）
REENTRY_APPROACH_TIMEOUT = 2.0  # 大力冲撞兜底时长：超时未贴墙则停车（真机调）
REENTRY_APPROACH_TOUCH_SIGNAL = 1000.0  # 前头模拟红外 signal≥此值=已贴墙；来源 front_adc_summary.csv（正对着墙 p99≈1315、居中 p99≈710）
REENTRY_REVERSE_SPEED = PATROL_RECOVER_SPEED  # 掉台矫正完毕倒车速度（与巡台恢复分开调）
REENTRY_REVERSE_TIMEOUT = 3.0

# ---------- 铲子防掉落（shovel_guard.py ShovelGuard 参数） ----------
# 阈值来源 2026-08-15 新车采集（data/shovel_hang_OutOfStage.csv / shovel_stage_OnStage.csv，
# 9 帧中值滤波后）：悬空 min(两路) p01=1291、台内 min(两路) p99=46 → ENTER 取中点 670；
# 悬空 max(两路) p01=1452、台内 max(两路) p99=1265 → CLEAR 取中点 1360。
# 新车极性与旧车相反：悬空=信号高、台内=信号低（判据：两路均 >ENTER 触发、均 <CLEAR 收回）。
SHOVEL_IR_CHANNELS = {
    # 新车 2026-08-15 scan 实测：铲子底下红外左=4、右=5。
    "left": 4,
    "right": 5,
}  # 铲子底下 2 路模拟红外；接线用 dev/shovel_tool.py scan 确认
SHOVEL_ADC_MAX = IR_ADC_MAX
SHOVEL_FILTER_WINDOW = 9  # 信号中值滤波窗口（同 ir.py 前头红外对齐模型）
SHOVEL_HANG_ENTER = 670.0  # 滤波后 min(两路)>此值 = 铲子悬空
SHOVEL_HANG_CLEAR = 1360.0  # 倒车后滤波后 max(两路)<此值 = 已收回台内（滞回，CLEAR>ENTER）
SHOVEL_HANG_CONFIRM = 3  # 悬空确认帧数（防抖）
SHOVEL_REVERSE_SPEED = PATROL_MIN_ACTIVE_SPEED  # 倒车收回速度（≥ 电机死区下限）
SHOVEL_REVERSE_MIN_SECONDS = 0.3  # 最短倒车时长，防信号抖动提前停
SHOVEL_REVERSE_TIMEOUT = 3.0  # 倒车超时兜底 → 停车待命

# ---------- YOLO 能量块对准（vision_tracker.py；初值待 dev/vision_tracker.py CSV 标定） ----------
VISION_LOOP_HZ = 50.0  # 电机安全轮询频率；YOLO 实际约 8 FPS
VISION_MAX_AGE_MS = 450  # 超过约 3 帧周期仍无新结果，立即停车
VISION_ERROR_FILTER_ALPHA = 0.45  # 横向归一化误差 EMA，新数据权重
VISION_DEAD_ZONE = 0.04  # 归一化死区；640 宽画面约为中心左右各 13 px
VISION_TURN_KP = 600.0  # 归一化横向误差到原地转向速度
VISION_TURN_MIN_SPEED = 400  # 实测电机可靠动作下限
VISION_TURN_MAX_SPEED = 600  # 首轮地面测试限速，避免 8 FPS 下转过头
