# vision_tracker_pack —— YOLO good 能量块追踪逻辑包

纯逻辑状态机：注入视觉结果 → 返回左右轮命令。不持有相机、不持有电机、
不 sleep——你自己的主循环（建议 20ms）每帧调用 `update()`。

## 文件

- `vision_tracker.py` —— `VisionTracker` 类，仅依赖标准库 + 本目录 `config.py`
- `config.py` —— 7 个 `VISION_*` 参数（精简版，每参数带来源注释；完整参数见主项目）

## 输入契约

你的视觉后端每帧提供一个 dict（我们项目里由 `rpi_yolo_api.VisionClient.get_control()`
产出，你们换成自己的实现即可）。本模块只用以下键，其余键忽略：

| 键 | 类型 | 语义 |
|---|---|---|
| `valid` | bool | `False`（无目标/断流/结果过期）→ 立即停车 `VISION_STOP` |
| `action` | str | 只有 `"push"` 才追踪；其他（`search`/`scan_with_ir`…）→ 停车等待 `SEARCH` |
| `target_type` | `None`/`"good"` | 不是 good 能量块 → 停车等待 `SEARCH` |
| `offset_x` | float | 归一化横向误差，见下；缺失/非有限值 → 停车 |

**offset_x 归一化**：`(center_x − 画面中心) / (画面半宽)`，值域约 `[-1, 1]`；
**正 = 目标在画面右侧**（车需右转）。640 宽画面下 0.08 ≈ 中心左右各 26 px。

## 输出契约

`update(control)` 返回 dict：

- `left` / `right`：电机命令（0 ~ 1023 前进方向，负为倒转；0 = 停车）
- `state`：`APPROACH` / `ARC_LEFT` / `ARC_RIGHT` / `BIG_TURN_LEFT` /
  `BIG_TURN_RIGHT` / `SEARCH` / `VISION_STOP`
- `reason`：中文原因（日志用）
- `error_x`：当帧归一化误差；`turn_command`：大转转向量（非大转状态为 0）

## 状态机语义

| 条件（`error = offset_x`） | 状态 | 命令 |
|---|---|---|
| `|error| ≤ 0.08` | APPROACH | `(400, 400)` 直线接近 |
| `0.08 < |error| < 0.55` | ARC_L/R | 差速弧线：外轮 500、内轮 400 |
| `|error| ≥ 0.55` | BIG_TURN_L/R | 原地大转 `(±400, ∓400)` |
| 大转中同方向 `|error| > 0.35` | 保持大转 | 滞回：避免大小转反复切换 |
| 大转中 `|error| ≤ 0.35` | 回弧线 | |

## 接入

```python
import time
from vision_tracker import VisionTracker
# from 你的驱动层 import move_cmd

tracker = VisionTracker()
while True:
    control = vision.get_control()      # 你的视觉后端，契约见上
    result = tracker.update(control)
    move_cmd(result["left"], result["right"])
    time.sleep(0.02)                    # 电机安全轮询 50Hz；视觉实际约 8 FPS
```

拷包后可在 PC 自检（纯逻辑不依赖树莓派）：

```python
import sys
sys.path.insert(0, "vision_tracker_pack")
from vision_tracker import VisionTracker
print(VisionTracker().update(
    {"valid": True, "action": "push", "target_type": "good", "offset_x": 0.25}))
# 应输出 state='ARC_RIGHT'、命令 (500, 400)
```

## 参数重标

阈值（`dead_zone` / `big_turn_enter` / `big_turn_clear`）是归一化值，与车无关，
可直接用。速度值按我们车标定（2026-08-16 实测）：

- `400` = 该车 CDS 伺服死区下限，低于它电机不可靠转动
- `500/400` = 已验证的弧线差速

同款车直接用；不同款车先重标电机死区下限（`BIG_TURN_SPEED` /
`ARC_INNER_SPEED` / `APPROACH_SPEED` 至少取该值），差速比例按自己车实测调整。
