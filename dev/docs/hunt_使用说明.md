# hunt 使用说明

## 职责

- `hunt.py`：纯决策状态机，只接收注入的 YOLO 与数字红外数据。
- `vision_tracker.py`：根据 good 的横向偏移输出对准和接近命令。
- `dev/hunt.py`：真机独立测试、CSV 采集与可选电机驱动。
- `main.py`：生产调度，只启动一个视觉进程并下发最终电机命令。

## 行为

- 任意距离的 good 都会追踪；红外近距确认是 good 时仍继续追踪。
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

按提示输入大写 `DRIVE` 后，程序才会通过 `UpController.move_cmd(left, right)`
驱动电机。按 `Ctrl+C` 会进入 `finally` 并调用 `UpController.close()` 停车。
