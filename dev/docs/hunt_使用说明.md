# hunt 使用说明

## 职责

- `hunt.py`：纯决策状态机，只接收注入的 YOLO 与数字红外数据。
- `vision_tracker.py`：根据 good 的横向偏移输出对准和接近命令。
- `dev/hunt.py`：真机独立测试、CSV 采集与可选电机驱动。
- `main.py`：生产调度，只启动一个视觉进程并下发最终电机命令。

## 行为

- good 只由视觉分类，前头红外不负责确认 good；远近目标使用同一规则。
- 置信度不低于 `HUNT_GOOD_HIGH_CONFIDENCE` 时，单帧中断普通巡台并开始追踪。
- 中等置信度 good 先进入 `GOOD_ACQUIRE` 停车，收到第二个不同 YOLO 帧后追踪。
- 低于 `HUNT_GOOD_MIN_CONFIDENCE` 的 good 不打断巡台。
- 已锁定 good 临时丢失时进入 `GOOD_LOST_HOLD` 停车，最多保留两个新视觉帧；
  前头红外即使亮起也不会立即把目标交给敌人模块。
- 近距 bad 会立即取消尚未提交的 good 锁定，并按 bad 确认流程处理。
- 只有远处 bad 时忽略，不抢占巡台。
- 红外近距目标经两个不同 YOLO 帧确认是 bad 后，锁定方向原地转 90 度。
- bad 接近画面中心时优先朝可见 good 的方向转。
- 转向期间忽略新的方向变化；转完停车一帧并释放控制权。
- bad 避让不前进、不横移、不回正。

## PC 测试

```powershell
python -m unittest tests.test_hunt tests.test_hunt_tool tests.test_main -v
```

## 真机采集

默认只采集视觉、ADC、数字 IO 和 hunt 决策，不打开电机：

```bash
python3 dev/hunt.py collect --label hunt_scene --seconds 20
```

持续采集到手动按 `Ctrl+C`：

```bash
python3 dev/hunt.py collect --label hunt_scene --seconds 0
```

`--distance-cm` 只作为人工记录写入 CSV，不参与 hunt 控制。日志默认保存到
`data/<label>_<时间戳>.csv`。

## 真机驱动

先把车放在不会掉台的平整空地：

```bash
python3 dev/hunt.py collect --label hunt_drive --seconds 20 --drive
```

## good 推块与铲子保护

- 普通巡台是默认任务；可靠 good 会中断 `CRUISE/MEDIUM_CRUISE` 并接管电机。
- 巡台边缘逃生和掉台回归不会被尚未提交的 good 中断。
- good 未居中时继续使用视觉大转/小转对准。
- good 连续两个不同 YOLO 帧进入中心死区后切换为 `GOOD_PUSH`，即使目标离开画面也持续直推。
- 视觉对准和转向阶段不启用铲子保护，只有 `GOOD_PUSH` 已提交后才启用。
- 前/侧灰度边缘不结束 `GOOD_PUSH`；铲子双路悬空才是正常停线。
- 铲子悬空后立即停车，确认后倒车收回；推动期间忽略灰度掉台触发，硬件整体失效仍会停车。
- 倒车收回后必须等待当前 good 消失，才允许重新推动。
- CSV 的 `shovel_left/right/state/active/hang` 用于核对保护触发时序。

## good 参数与日志

参数集中在 `config.py`：

- `HUNT_GOOD_MIN_CONFIDENCE = 0.55`：低于此值忽略。
- `HUNT_GOOD_HIGH_CONFIDENCE = 0.80`：达到此值单帧快速接管。
- `HUNT_GOOD_ACQUIRE_FRAMES = 2`：中置信度需要的不同视觉帧数。
- `HUNT_GOOD_LOST_HOLD_FRAMES = 2`：锁定后允许丢失的新帧数。
- `HUNT_GOOD_LOST_HOLD_SECONDS = 0.35`：没有新视觉序号时的最长保留时间。
- `HUNT_GOOD_CONFIRM_FRAMES = 2`：居中后提交 `GOOD_PUSH` 的确认帧数。

分析 CSV 时查看 `confidence`、`selected_target`、`hunt_state`、
`good_acquire_count`、`good_miss_count` 和 `good_locked`。同一个 `sequence` 在
50 Hz 电机循环中会出现多次，但不会重复增加确认或丢帧计数。

按提示输入大写 `DRIVE` 后，程序才会通过 `UpController.move_cmd(left, right)`
驱动电机。按 `Ctrl+C` 会进入 `finally` 并调用 `UpController.close()` 停车。
