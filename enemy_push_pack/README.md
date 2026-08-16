# enemy_push_pack —— 敌人推动模块移植包

从 robocup-2026-wheeled-combat 的「红外敌侦测 + 推对手」行为抽取，按你的新车
（detail/，2026-08-14/15 标定）适配。单文件自包含：内嵌 DiagIR（IO 部分）、
ShovelGuard（铲子悬空）、GrayZones（zone 灰度），真机只 import 你的
detail/up_controller.py 硬件层。**纯红外触发（只认前头 io0），不含视觉。**

## 文件

- `enemy_push.py` —— 敌人推动状态机（step 驱动，不 sleep）：
  `EnemyPusher`（IDLE → PUSH → RETREAT → COOLDOWN → IDLE）+ 真机循环
  （直接启动、CSV 落盘）
- `enemy_push_config.py` —— 参数集中管理：「调参数只改这一个文件」

## 接入步骤

1. 把两个文件拷到你的项目（与 detail/ 同仓库根即可）
2. 真机：`python3 enemy_push_pack/enemy_push.py`——**直接启动即开始**（无
   确认输入），前头红外亮就推，Ctrl+C 退出
3. 落盘 `detail/data/attack_<时间戳>.csv`
4. 参数都在 `enemy_push_config.py`，每个键带来源/取舍注释

## 行为语义

- **触发**：只认前头红外 io0 亮（对角/后向红外不参与）
- **分级速度**：front zone ≥ 1.3 快 700、< 1.3 慢 350（白边之前降速，
  给铲子防掉留反应时间；档位只降不升）
- **停线**：铲子双路悬空（ShovelGuard 防掉，悬空=高极性，确认推下）；
  白边四路 zone 全 ≥ 1.4 连续 6 帧 → 中断推（不确认）
- **节奏**：推完倒车 1.0s 收回 → 停 0.5s（attack_pause）→ io0 还亮就继续
  打（多次攻击）
- **兜底**：断流 / 后路悬空（zone ≤ -0.45）/ 倒车超时 → 打断 + 冷却 3s

## CSV 落盘

`detail/data/attack_<时间戳>.csv`，列：`t, front, rear, left, right,
shovel_left, shovel_right, state, left_cmd, right_cmd, guard_state, hang,
reason, confirmed, healthy`。

## 已通过的测试（2026-08-16）

- 桩冒烟：完整攻击链（推→铲子悬空停→退→短停→再推）/ 分级速度 700→350
  只降不升 / 白边四路全亮中断 / 后路悬空打断 / 无目标静止 / PC 优雅退出 ✓
- 真机日志 attack_20260816_092127 分析驱动两处修复：白边掉台（白边四路全亮
  保护，4 次掉台前 2.5s 内全命中）与双退（attack_pause 短停）

## ⚠️ 注意事项

1. **io0 锁存场景**（推墙/围栏）会反复推-收-推：白边保护/后路悬空兜底，
   真机遇墙死推时把 `attack_pause` 调大或加推时上限。
2. `slow_speed=350` 是试验值（此前实测新车 400 以下不可靠）——不转改回 400。
3. 白边保护误触（亮区推敌被中断）实测约 7 段/100s，表现为多退几轮不致命；
   阈值 `white_bright_threshold` 可调（调高更少误触但掉台保护窗口更窄）。
