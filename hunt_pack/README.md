# hunt_pack —— 猎杀模式移植包

从 robocup-2026-wheeled-combat `tools/calibrate.py cmd_hunt` 抽取，按你的新车
（detail/config.py，2026-08-14 实测标定）适配。仅依赖 Python 标准库。

## 文件

- `hunt.py` —— 猎杀模式逻辑：`hunt_run` 阻塞主循环（静止等待→红外→固定角粗转→
  IO4 确认/慢搜→视觉分类→buff 锁定/debuff 绕行/unknown 冷却→后向兜底），
  含 `DiagIR`（6 方位敌侦测）、`GraySafety`（灰度安全钩子）、`VisionAdapter`
  （视觉后台程序线程适配器）
- `hunt_config.py` —— 参数集中管理：红外通道/极性（你的新车 scan 实测）、
  `GRAY`（evade_zone）、`HUNT`（转向标定取自你的 motor_turn_calibration.csv）

## 依赖

- 你的 `gray.py`（`GrayRiskModel` 实例注入，不 import）
- 你的 `config.py` 里的灰度标定值（构造 `GrayRiskModel` 时注入，你项目的运行时方式）

## 接入步骤

1. 把 `hunt.py` + `hunt_config.py` 拷到你的项目（与 gray.py 同目录或 import 可达）
2. 照 `hunt.py` 模块 docstring 的"最小使用示例"接线：`MySensors` 组合
   `DiagIR` + `GraySafety`；视觉用 `VisionAdapter(client=你的接口)`
3. 你的视觉接口实现 `read_result() -> dict`：
   `{"found": bool, "kind": "buff"/"debuff", "center_x": float, "timestamp": 单调秒}`
   （字段不同只改 `hunt.py` 里的 `to_sample`）
4. 参数在 `hunt_config.py`：红外引脚改了这里同步；灰度参考值在**你的 config.py**
   里调；`GRAY["evade_zone"]` 绕行查边阈值
5. 运行：`python3 hunt.py`（打印使用说明）或 `import hunt` 后调 `hunt_run(...)`

## 行为语义

- buff：视觉居中后**只锁定保持静止**，回调 `on_locked("buff")`（锁定瞬间一次；
  推块动作以后由你在回调里注入）
- debuff：转离 90° + 横移 + 回正（`actuator` 非 None 才执行，源逻辑）
- unknown/识别失败：同方位冷却后继续静止等待
- 后向红外单独亮 → 按方位 5（正后）掉头兜底
- 安全：`fall_risk`=压白边（white_hits 非空）、`edge_risk`=near_edge、
  绕行横移查边 `min(zone) < evade_zone` 停

## CSV 落盘

`data/hunt_<时间戳>.csv`（你的约定：采集数据丢 data/），`type` 列：
`decision`（每轮决策一行）/ `align`（对准事件）。

## 已通过的 dev 测试

`dev/test_hunt_replay.py` 用你的 detail/data 实测 CSV 回放验证：

- GraySafety：内环（fall 0/edge 0）、四个同白（fall 全触发）、四个同黑（edge 全触发）、
  掉台暗值（edge 全触发、fall 0——白边语义）✓
- 转向参数与 motor_turn_calibration.csv 一致（45°/90°/135°/180°）✓
- `_drive_ok` 内环放行、黑带拦 ✓
- hunt_run 全流程：锁定 + on_locked 恰好一次 + CSV 落盘 ✓

⚠️ **注意**：`台边缘而且颜色浅.csv` 的 min_zone 中位 0.359，只比 `evade_zone`(0.35)
高 0.009——如果现场绕行会冲浅色边缘，把 `hunt_config.GRAY["evade_zone"]` 调高
（如 0.40）再测。
