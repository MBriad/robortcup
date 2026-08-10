#!/usr/bin/env python3
"""
控制器封装 (hardware/up_controller.py) —— 由 test/RpiYolo/up_controller.py 移植

对外 API（与封装前保持一致）：
- move_cmd(left, right)   : 底盘差速（CDS 7=左轮、CDS 8=右轮取反）
- set_cds_mode(ids, mode) : 舵机模式设置
- set_chassis_mode(mode)  : 底盘模式记录
- get_ad_data()           : 返回 ADC 读数列表（10 路）
- adc_data / io_data      : 属性，由后台线程轮询更新
- stale(max_age)          : 传感器数据是否过期（无首帧/超时未更新）
- healthy/poll_errors     : 轮询健康标志与失败计数（异常不再静默）
- close()                 : 安全停机（先停车 → 停线程 → 关 CDS → 关 ADC/IO）

"""

import threading
import time
from math import isfinite

try:
    import uptech  # 树莓派系统库（如 /home/pi/uptench_star）
except (ImportError, OSError):
    try:
        from .vendor import uptech  # 项目自带副本（hardware/vendor/uptech.py）
    except (ImportError, OSError):
        uptech = None  # PC 等无硬件环境


class UpController:
    """底盘 + 传感器控制器：move_cmd 驱动，adc_data/io_data 由轮询线程填充。"""

    # cmd（保留封装 API）
    NO_CONTROLLER = 0
    MOVE_UP = 1
    MOVE_LEFT = 2
    MOVE_RIGHT = 3
    MOVE_YAW_LEFT = 4
    MOVE_YAW_RIGHT = 5
    MOVE_STOP = 6
    PICK_UP_BALL = 7

    SPEED = 256
    YAW_SPEED = 210

    # chassis_mode 1 for CDS5516 servo, 2 for BMDC controller
    CHASSIS_MODE_SERVO = 1
    CHASSIS_MODE_CONTROLLER = 2

    def __init__(self, poll_hz: float = 50.0,
                 motor_invert: bool = False, motor_swap: bool = False):
        if uptech is None:
            raise RuntimeError(
                "uptech 库不可用（仅树莓派真机；PC 上请用 Sensors/Actuator 桩 + fake 对象）")
        self.up = uptech.UpTech()
        open_flag = self.up.ADC_IO_Open()
        print(f"ad_io_open = {open_flag}")
        self.up.CDS_Open()
        self.cmd = 0

        self.poll_hz = poll_hz
        self.motor_invert = motor_invert   # 电机方向修正（软件，不用拆线）
        self.motor_swap = motor_swap
        self.adc_data = [0] * 10   # ADC 10 路（厂商 ADC_DATA=[0]*10）
        self.io_data = [0] * 8     # 数字 IO 8 路，io_data[i] = 掩码第 i 位

        # 轮询健康（P0-3）：数据过期检测用，异常不再静默吞掉
        self.last_update = 0.0     # 最近一次成功读取的时间戳
        self.first_frame = False   # 是否已收到首帧有效数据
        self.healthy = False       # 最近一次读取是否成功
        self.poll_errors = 0       # 累计读取失败次数
        self.last_error = None     # 最近一次异常对象

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    # ---------- 传感器轮询（原 edge_detect_thread，修复后启用） ----------
    def _poll_loop(self) -> None:
        dt = 1.0 / max(self.poll_hz, 1.0)
        while not self._stop.is_set():
            try:
                adc = list(self.up.ADC_Get_All_Channle())
                mask = self.up.ADC_IO_GetAllInputLevel()
                # 首帧校验（P1）：短/畸形数据不标健康，走异常路径置 unhealthy
                if (len(adc) < 10 or any(not (isinstance(v, (int, float)) and isfinite(v))
                                         for v in adc[:10])):
                    raise ValueError("ADC 数据畸形（长度/数值非法）")
                if not isinstance(mask, int):
                    raise ValueError("IO 掩码畸形")
                self.adc_data = list(adc[:10])
                self.io_data = [(mask >> i) & 1 for i in range(8)]
                self.last_update = time.monotonic()
                self.first_frame = True
                self.healthy = True
                self.poll_errors = 0
                self.last_error = None
            except Exception as e:
                # 单次读失败不致命，但必须留痕：健康标志置 False，Sensors 据此急刹
                self.poll_errors += 1
                self.last_error = e
                self.healthy = False
            time.sleep(dt)

    def stale(self, max_age: float = 0.5) -> bool:
        """传感器数据是否过期：从未收到首帧，或超过 max_age 秒未更新。

        Sensors 检测到过期会按危机处理（fall_risk/edge_risk 返回 True 急刹）。
        """
        return (not self.first_frame) or (time.monotonic() - self.last_update > max_age)

    def close(self) -> None:
        """安全停机（P0-2）：先停车 → 停轮询线程 → 关 CDS → 关 ADC/IO。

        线程 join 超时仍存活（底层读取卡死）时也**尽力尝试**关闭硬件
        （进程退出场景；每个调用都 try 包裹，残留竞态风险可接受）。
        """
        try:
            self.move_cmd(0, 0)
        except Exception:
            pass
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self.up.CDS_Close()
        except Exception:
            pass
        try:
            self.up.ADC_IO_Close()
        except Exception:
            pass

    # ---------- 底盘 ----------
    # 速度指令，自由控制-开环控制器
    def move_cmd(self, left_speed, right_speed):
        if self.motor_swap:
            left_speed, right_speed = right_speed, left_speed
        if self.motor_invert:
            left_speed, right_speed = -left_speed, -right_speed
        self.up.CDS_SetSpeed(7, left_speed)
        self.up.CDS_SetSpeed(8, -right_speed)

    def set_chassis_mode(self, mode):
        self.chassis_mode = mode

    def set_cds_mode(self, ids, mode):
        for id in ids:
            self.up.CDS_SetMode(id, mode)

    # ---------- 传感器 ----------
    def get_ad_data(self):
        return self.adc_data

    def lcd_display(self, content):
        """LCD 显示（真机库支持时生效，否则静默跳过）。"""
        try:
            self.up.LCD_PutString(30, 0, content)
            self.up.LCD_Refresh()
            self.up.LCD_SetFont(self.up.FONT_8X14)
        except Exception:
            pass


if __name__ == '__main__':
    ctrl = UpController()
    try:
        while True:
            print("adc0-3:", ctrl.adc_data[:4], " io0-7:", ctrl.io_data)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.close()
