# main 使用与日志分析说明

## 职责

`main.py` 是唯一的生产真机入口，统一调度巡台、掉台回归、good 能量块追踪、
敌人搜索推动和铲子保护，并且每个控制周期只输出一组
`move_cmd(left, right)` 电机命令。

不要在比赛运行时同时启动 `dev/ring_patrol.py`、`dev/reentry.py`、
`dev/hunt.py` 或 `dev/proximity_probe.py`，否则多个进程会同时控制电机。

## 每次运行保存独立 CSV

直接运行即可使用时间戳文件名：

```bash
python3 main.py
```

默认日志路径为：

```text
data/main_YYYYMMDD_HHMMSS.csv
```

例如：

```text
data/main_20260816_203015.csv
data/main_20260816_203242.csv
```

每次启动都会创建一个新的时间戳 CSV，不会把多次测试合并到同一个文件。
程序每写一行都会刷新文件，按 `Ctrl+C` 正常退出时已采集的数据会保留。

限定单次测试时间也仍然会生成独立时间戳文件：

```bash
python3 main.py --seconds 60
```

只有明确需要固定文件名时才使用 `--log`：

```bash
python3 main.py --seconds 60 --log data/main_test.csv
```

`--log data/main_test.csv` 不会追加或合并旧数据，而是覆盖同名文件。因此连续
测试时建议不要传 `--log`，直接使用默认时间戳文件。

## 日志字段

分析时优先查看以下字段：

- `t`：本次运行开始后的秒数。
- `mode`：当前真正拥有电机控制权的模块。
- `state`：当前控制模块的状态。
- `reason`：进入当前状态的原因。
- `left_cmd/right_cmd`：最终发送给左右电机的命令。
- `patrol_state/reentry_state/hunt_state/enemy_state`：各策略内部状态。
- `shovel_left/right/valid`：铲子双路 ADC 原始数据及有效标志。
- `shovel_active`：铲子保护是否真正启用。
- `shovel_state`：铲子保护的 `IDLE/HANGED/REVERSE/SAFE_STOP` 状态。
- `shovel_preheat`：巡台灰度诊断标志，目前只记录，不会启用铲子保护。
- `vision_sequence/vision_status`：YOLO 帧序号和结果状态。
- `vision_has_good/vision_has_bad`：当前视觉结果中是否实际出现对应类别。
- `probe_vision_count`：前方近物确认期间已收到的不同 `no_target` 新帧数。
- `probe_vision_verdict`：敌人视觉确认结论，如 `waiting`、`no_target_1`、
  `good`、`bad`、`timeout` 或 `enemy_confirmed`。
- `good_offset_x/bad_offset_x`：目标相对画面中心的归一化横向偏差。
- `good_confidence`：当前 hunt 选中 good 的 YOLO 置信度。
- `good_acquire_count/good_locked`：中置信度获取计数与 good 身份锁定状态。
- `good_miss_count`：锁定后已连续丢失的不同视觉帧数。
- `healthy`：驱动和传感器数据是否健康。

## 视觉与红外优先级

传感器失效始终安全停车。尚未进入推动时，掉台回归和巡台灰度危险动作优先；已经进入
`GOOD_PUSH/ENEMY_PUSH` 后维持推动，直到铲子保护接管，这是当前比赛策略。正常巡台的
`CRUISE` 和 `MEDIUM_CRUISE` 状态下，按以下规则仲裁：

1. 新视觉帧识别到 good 时，hunt 追踪 good；未分类红外不抢占。
2. 红外方向对应已识别的近 bad 时，近 bad 避让优先于 good 追踪。
3. 新视觉帧没有任何 good/bad 时，红外可以中断巡台：正前停车等视觉，侧后先定角转向。
4. 视觉 `stale`、`error` 或没有数据时，红外只能停车等待，不能把目标直接确认为敌人。
5. `PROBE_VISION_WAIT` 仅对不同的 `vision_sequence` 计数；连续 3 帧 `no_target` 且正前红外仍有效，
   才开始 enemy `ENEMY_PUSH`。

同一控制周期内，good/bad 与 enemy、hunt 和 patrol 直接交接，不插入一帧巡台命令。一次
敌人处理结束后，对应红外必须连续清除 3 个轮询周期才会重新布防。

## 正常状态流程

good 能量块：

```text
hunt/ARC_* 或 BIG_TURN_*
（中置信度目标会先经过 hunt/GOOD_ACQUIRE）
（锁定后短暂丢帧会进入 hunt/GOOD_LOST_HOLD 并停车）
-> hunt/GOOD_CONFIRM（两个不同 YOLO 帧居中）
-> hunt/GOOD_PUSH
-> shovel_guard/HANGED
-> shovel_guard/REVERSE
-> patrol/WARMUP
```

敌人：

```text
proximity_probe/PROBE_TURN（侧面或后方发现）
-> proximity_probe/PROBE_VISION_WAIT（转后或正前红外触发，停车等待视觉）
-> enemy_push/ENEMY_PUSH（3 个不同序号的 no_target 新帧确认）
-> shovel_guard/HANGED
-> shovel_guard/REVERSE
-> patrol/WARMUP
```

普通巡台掉边：

```text
patrol/CRUISE
-> patrol/EDGE_AVOID
-> patrol/EDGE_TURN
-> patrol/RECOVER_FORWARD
```

掉台回归：

```text
reentry/TURN_*
-> reentry/ADC_APPROACH 或 ADC_CORRECT
-> reentry/REVERSE
-> reentry/SAFE_STOP
```

## 分析莫名后退

筛选同时满足 `left_cmd < 0` 和 `right_cmd < 0` 的行，再根据 `mode/state/reason`
判断来源：

- `patrol/EDGE_AVOID`：巡台主动离开边缘，属于正常后退。
- `shovel_guard/REVERSE`：推动时铲子悬空后的收回动作。
- `reentry/REVERSE`：掉台回归完成对墙矫正后的倒车。
- 其他状态出现双轮负命令：需要检查调度或电机命令是否异常。

正常情况下，视觉对准、`GOOD_CONFIRM`、敌人 `PROBE_TURN` 和普通巡台期间应满足：

```text
shovel_active = 0
shovel_state = IDLE
```

只有 `GOOD_PUSH`、敌人 `ENEMY_PUSH` 或已经开始的铲子恢复过程，
`shovel_active` 才应为 `1`。

`PROBE_VISION_WAIT` 时左右命令必须都是 `0`。若 `good/bad` 出现，视觉行为优先并取消敌人
候选；若 0.6 秒内没有足够的新帧，敌人候选超时取消。enemy 已经进入 `ENEMY_PUSH` 后，新的
good/bad 仍可中断 enemy 并立即交给 hunt；铲子保护继续拥有最高优先级。`HANGED` 仅是
临时停车，信号恢复后继续原来的 push；只有确认进入 `REVERSE` 或 `SAFE_STOP` 才结束 push。

## 建议测试方法

每种场景单独启动一次 `main.py`，不要在同一次日志中反复混合测试：

1. 只巡台 30 至 60 秒。
2. 单独追踪并推动 good。
3. 单独从正前方推动敌人。
4. 单独测试侧面敌人转向。
5. 单独触发一次掉台回归。

测试结束后保留对应的时间戳 CSV，并记录该文件测试的场景。这样分析状态切换时
不需要从一份混合日志中猜测现场发生了什么。

正前敌人测试还要核对：触发帧计数为 0、重复 `vision_sequence` 不增计数、
前红外在确认期间始终为 1，且只有 `probe_vision_verdict=enemy_confirmed` 后
电机命令才由 0 变为正值。
