#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""六路数字红外的位映射与数据注入，不持有硬件。"""

# ---------- 独立默认值；生产/dev 运行时由 config.py 注入 ----------
# 板子引脚号（用户已确认接线；位号 DIGI_IR_BITS 需 scan 实测确认）。
DIGI_IR_PINS = {
    "left_rear": 100,     # 左后
    "left_front": 101,    # 左前
    "right_rear": 102,    # 右后
    "right_front": 103,   # 右前
    "rear": 104,          # 正后
    "front": 105,         # 正前
}
# 传感器 → io_data 位号（io_data = [(mask >> i) & 1 for i in range(8)]）。
# 新车 2026-08-14 scan 实测：IO0=前、IO3=左前、IO1=左后、IO2=右前、IO4=右后、IO6=后。
DIGI_IR_BITS = {
    "left_rear": 1,
    "left_front": 3,
    "right_rear": 4,
    "right_front": 2,
    "rear": 6,
    "front": 0,
}
ACTIVE_LEVEL = 0      # 检测到时 io 位为 0（低有效，真机确认）


def _to_bits(io):
    """把 io 输入归一成 8 位 0/1 列表：接受掩码 int（非负）或 up_controller.io_data 的 0/1 列表。"""
    if isinstance(io, int) and io >= 0:
        return [(io >> i) & 1 for i in range(8)]
    if isinstance(io, (list, tuple)) and len(io) >= 8:
        return [1 if v else 0 for v in io[:8]]
    return None


class DigiIR:
    """6 路数字红外数据注入接口，io 掩码外部注入，不持有硬件。

    io_reader：返回 8 位掩码或 0/1 列表的可调用对象（如 lambda: ctrl.io_data）。
    read_states() 返回 {left_rear..front: bool, valid: bool}；无效掩码 → valid=False。
    """

    def __init__(self, io_reader=None, bits=None, active_level=ACTIVE_LEVEL):
        self._reader = io_reader
        self.bits = dict(DIGI_IR_BITS if bits is None else bits)
        self.active_level = int(active_level)

    def read_states(self, io=None):
        if io is None:
            if self._reader is None:
                raise RuntimeError("需注入 io_reader（或显式传 io 掩码）")
            io = self._reader()
        bits = _to_bits(io)
        if bits is None:
            return {name: False for name in self.bits} | {"valid": False}
        out = {}
        valid = True
        for name, bit in self.bits.items():
            if not 0 <= bit < 8:
                out[name] = False
                valid = False
            else:
                out[name] = bits[bit] == self.active_level
        out["valid"] = valid
        return out
