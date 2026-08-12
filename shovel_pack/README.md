# 铲子防掉落模块（shovel_pack）

推东西模式下，铲子底下 2 路模拟红外检测铲子悬空（伸出台面）→ **停车 → 倒车收回台内**。
纯逻辑模块，无硬件依赖，PC 可直接跑；本目录**复制即用**，主项目版本独立保留。

## 文件

| 文件 | 说明 |
|---|---|
| `shovel_guard.py` | 状态机模块（`ShovelGuard`）——**要分享的核心算法** |
| `config.py` | 参数文件（`SHOVEL_*` 为铲子部分，其他参数与本模块无关可忽略） |
| `__init__.py` | 包导出（可选，方便 `from shovel_pack import ShovelGuard`） |

**依赖**：`shovel_guard.py` → `config.py`（纯参数，无 import）+ Python 标准库。不需要任何硬件库。

## 接入（你的主循环每帧调用）

```python
import time
from shovel_guard import ShovelGuard

guard = ShovelGuard()

# 每帧（如 50Hz）：
shovel_raw = read_your_shovel_adc()      # 你的 ADC 读取，dict：{"left": 1320, "right": 1300, "valid": True}
result = guard.update(shovel_raw, active=True, now=time.monotonic())

# result = {"left":.., "right":.., "state":.., "reason":.., "hang":..}
if result["state"] != "IDLE":
    your_motor_cmd(result["left"], result["right"])   # 接管电机
else:
    your_normal_control()                              # 正常逻辑（巡台等）
```

- `active`：**推东西模式开关**——推东西时置 `True`（由视觉/数字红外给出）；不推时 `False`，模块完全不触发
- 电机命令约定：`(left, right)` 差速速度指令

## 算法说明（核心）

### 判定量

- 每帧取两路信号 `signal_max = max(left, right)`、`signal_min = min(left, right)`
- 各做 **window=9 帧滚动中值滤波**（抗单帧噪声尖峰）后再判定
- **悬空判定**：`filtered_max < SHOVEL_HANG_ENTER`（两路都低才算悬空——铲子整体出台）
- **收回判定**：`filtered_min > SHOVEL_HANG_CLEAR`（两路都恢复才算回台）

### 状态机

```
IDLE → filtered_max < ENTER → HANGED（第一帧即停车，防掉落优先）
HANGED → 连续 SHOVEL_HANG_CONFIRM 帧仍悬空 → REVERSE（倒车收回）
REVERSE → filtered_min > CLEAR 且倒车 ≥ 最短时长 → IDLE
       → 超时 SHOVEL_REVERSE_TIMEOUT → SAFE_STOP（信号恢复后回 IDLE）
```

- **第一帧就停车**：悬空确认的 3 帧防抖只作用于"是否倒车"，停车不等待——ADC 不稳时最多误停，不会误倒车
- **滞回**：CLEAR > ENTER，信号在边界抖动时不会反复触发/解除

### 阈值怎么定（重要：换车/换场地必重标）

当前 1400/1450 来自**我们场地的实测标定流程**，你们必须用自己的数据重采：

1. 采「悬空」：铲子完全伸出台面，**固定不动**采 10 秒（对着你们台下地面）
2. 采「台内」：铲子在台上正常推东西姿态，**固定不动**采 10 秒
3. 各自做 9 帧中值滤波，看分位数分布（p01/p50/p99）：
   - `ENTER` =（悬空上界 + 台内下界）的中点
   - `CLEAR` = 台内下界附近（须 > ENTER，形成滞回）
   - 要求：悬空上界 < 台内下界（区间不重叠），否则阈值方案不可用

### 我们踩过的坑（帮你避雷）

1. **测距型红外区分不了"台面 vs 浅色地面"**：两者都是近处反射面，信号重叠（1200~1700 全糊在一起）——**台下地面必须是深色（黑）**才有信号差；我们实测黑地面 99% 帧压在 1290~1354，浅色地面与台面不可分
2. **采集时必须固定姿态**：采集时铲子前后移动会让"台内"分布拉宽 300+，把真实间隙吞掉——固定不动采，分布跨度只有 ~90
3. **中值滤波窗口要和标定时一致**：阈值是按滤波后数据定的，运行时必须用同样的滤波，否则边界抖动
4. 信号随铲子位置**非单调**（台内 1444 → 台边缘 1697 → 出台 1335）——别用"信号高=安全"这种直觉，一切以标定数据为准

## 参数（config.py 的 SHOVEL_*）

| 参数 | 默认 | 含义 |
|---|---|---|
| `SHOVEL_IR_CHANNELS` | 6/7 | 铲子底下 2 路 ADC 通道（换车用 scan 确认接线） |
| `SHOVEL_FILTER_WINDOW` | 9 | 中值滤波窗口（启动前 9 帧不判定） |
| `SHOVEL_HANG_ENTER` | 1400 | 滤波后 signal_max < 此值 = 悬空（**按你们场地标定**） |
| `SHOVEL_HANG_CLEAR` | 1450 | 滤波后 signal_max > 此值 = 已收回台内（须 > ENTER） |
| `SHOVEL_HANG_CONFIRM` | 3 | 悬空确认帧数（防抖） |
| `SHOVEL_REVERSE_SPEED` | 400 | 倒车收回速度（≥ 电机死区下限） |
| `SHOVEL_REVERSE_MIN_SECONDS` | 0.3 | 最短倒车时长（防信号抖动提前停） |
| `SHOVEL_REVERSE_TIMEOUT` | 3.0 | 倒车超时兜底 → 停车待命 |

## 安全

- 模块会驱动电机倒车，接入前确认人与线缆远离车轮
- 悬空第一帧即停车（不等确认）——宁可误停，不可掉落
