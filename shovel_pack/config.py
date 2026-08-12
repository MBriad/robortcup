"""铲子防掉落模块参数（独立包精简版，只含 SHOVEL_*；完整参数见主项目 config.py）。

阈值来源 2026-08-12 固定姿态采集（9 帧中值滤波后）：
    出台纯黑地面 signal_max ≤ 1354，台内 ≥ 1442 → ENTER=1400 / CLEAR=1450。
换车/换场地必须重新标定（标定方法见 README.md）。
"""

# ---------- 铲子防掉落（shovel_guard.py ShovelGuard 参数） ----------
SHOVEL_IR_CHANNELS = {"left": 6, "right": 7}   # 铲子底下 2 路模拟红外通道（换车用 scan 确认）
SHOVEL_ADC_MAX = 10000.0                       # ADC 合法上限（原引用 IR_ADC_MAX）
SHOVEL_FILTER_WINDOW = 9                       # 信号中值滤波窗口（同 ir.py 前头红外对齐模型）
SHOVEL_HANG_ENTER = 1400.0                     # 滤波后 signal_max<此值 = 铲子悬空
SHOVEL_HANG_CLEAR = 1450.0                     # 倒车后滤波后 signal_max>此值 = 已收回台内（滞回，> ENTER）
SHOVEL_HANG_CONFIRM = 3                        # 悬空确认帧数（防抖）
SHOVEL_REVERSE_SPEED = 400                     # 倒车收回速度（电机死区下限；原引用 PATROL_MIN_ACTIVE_SPEED）
SHOVEL_REVERSE_MIN_SECONDS = 0.3               # 最短倒车时长，防信号抖动提前停
SHOVEL_REVERSE_TIMEOUT = 3.0                   # 倒车超时兜底 → 停车待命
