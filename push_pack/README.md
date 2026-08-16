# push_pack —— 推能量块移植包

从 robocup-2026-wheeled-combat `hardware/actuator.py` 的
`push_and_retreat(block=True)` 抽取，按你的新车（detail/，2026-08-15 标定）
适配。回放模式仅依赖 Python 标准库；真机模式只 import 你的
detail/up_controller.py 硬件层。**推对手（block=False）不包含。**

## 文件

- `push_block.py` —— 推块状态机（step 驱动，不 sleep）：内嵌 `ShovelGuard`
  （铲下红外悬空检测）与 `GrayZones`（新车 zone 灰度模型）；
  `BlockPusher`（WARMUP 预热 → PUSH 两档前冲 → RETREAT 反馈式倒车 → DONE）；
  `--replay` PC 回放模式 + 真机单次推块模式（PUSH 确认 + CSV 落盘）
- `push_block_config.py` —— 参数集中管理：「调参数只改这一个文件」

## 接入步骤

1. 把 `push_block.py` + `push_block_config.py` 拷到你的项目
2. PC 回放：`python3 push_pack/push_block.py --replay detail/data/push_*.csv`
3. 真机：`python3 push_pack/push_block.py`（输入 PUSH 确认，执行一次推块；
   落盘 `detail/data/push_<时间戳>.csv`——该文件同时就是回放输入）
4. 参数都在 `push_block_config.py`，每个键带来源/取舍注释
5. 视觉纠偏：`BlockPusher(vision=你的视觉后端)`（read() → TargetSample 契约）

## 行为语义

- 起步预检：铲下 9 帧预热，起步前已悬空（车在沿上）→ 取消推块，直接倒车收回
- 前冲两档：快 500 → 慢 325（front zone ≤ 1.3 即减速——2026-08-15 白边掉台
  修复：白边段 front 变亮而非变暗，旧 0.45 减速线永不触发；1.3 起步即慢推，
  快档仅更内环亮区使用）
- **推下确认 = 铲下双路红外悬空事件**（min(两路)>670，9 帧中值，悬空=高极性）；
  双路悬空即停+确认，反馈式倒车收回（max(两路)<1360 + 固定 1s）
- front 单路悬空（zone ≤ -0.45）/侧向悬空/慢推超时/总时长上限 → 停+退**不确认**
- 后路悬空（zone ≤ -0.45）/数据断流 → 立即打断
- 档位切换 6 帧去抖（武字抗干扰）

## CSV 落盘

`detail/data/push_<时间戳>.csv`，列：`t, front, rear, left, right,
shovel_left, shovel_right, state, stage, left_cmd, right_cmd, guard_state,
hang, reason, confirmed, healthy`。

## 已通过的测试（2026-08-15）

- **ShovelGuard 真机数据对拍**：detail/data/shovel_20260815_162203.csv
  逐帧回放，2831/2831 帧状态与原实现一致 ✓
- 桩冒烟 4 场景（代码路径）：正常推下确认 / 起步已悬空取消 /
  front 单路悬空兜底 / 后路悬空打断 ✓
- 真机模式 PC 优雅退出 ✓

## ⚠️ 注意事项

1. **铲下红外标定**：2026-08-15 极性（悬空=高，AD4/5）已同步进
   detail/config.py 的 SHOVEL 段（08-12 旧标定 AD6/7、悬空=低已作废）。
2. **全推块回放待做**：detail/data 尚无含四路灰度的推块 CSV（按约定不拿
   合成/旧数据验）——真机首跑日志（detail/data/push_*.csv）即回放输入。
3. 真机首跑建议架起车轮验证方向，再落台实推。
