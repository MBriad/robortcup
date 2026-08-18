# 近物候选确认与敌人推动测试

生产比赛只运行根目录 `main.py`。本工具只运行敌人状态机，单独验证六路数字
红外、定角转向、视觉排除和确认后的敌人推动。后台 YOLO 只负责排除 `good/bad`，
不会生成追踪命令；巡台、回台和 hunt 都不会创建。

## 文件职责

- `proximity_probe.py`：纯状态机，不导入硬件驱动。
- `dev/proximity_probe.py`：真机传感器读取、后台 YOLO、电机执行与 CSV。
- `config.py`：推动速度、分级限速、退回与冷却参数。
- `tests/test_proximity_probe.py`：PC 状态机测试。

## 无限真机测试

```bash
python3 dev/proximity_probe.py --label proximity_probe --seconds 0 --drive
```

输入 `DRIVE` 后启用电机，按 `Ctrl+C` 停止；CSV 默认写入 `data/`。

所有可调参数集中在 `config.py`：`PROBE_*` 配置候选转向（`PROBE_BRAKE_SECONDS`
为转向前的刹车归0时长）、视觉等待和 dev 日志；`ENEMY_*` 只配置确认敌人后的
推动/慢档、bad 打断帧数、后路保护与冷却。

## 仲裁规则

1. 正前红外触发时先停车并进入 `PROBE_VISION_WAIT`；触发当帧不计入确认。
2. 前红外持续有效且收到 3 个不同序号的 `no_target` 新帧后，才进入 `ENEMY_PUSH`。
3. 等待期间出现 `good` 或 `bad`、前红外消失会取消攻击；`stale/error` 不计数，
   最多等待 0.6 秒后超时取消。
4. 超时或视觉否决后，必须等前红外先清空，才允许重新确认同一方向的新目标。
5. `ENEMY_PUSH` 一旦确认，不再被后续视觉打断，持续到铲子保护或后路保护接管。
6. 正前空闲时，左前/右前/左后/右后/正后按 45°/135°/180°定角转向；转向前先进入
   `PROBE_BRAKE` 刹车归0（`PROBE_BRAKE_SECONDS`，运动中起转会让原地转标定失效）。
7. 转向途中出现 `good` 会立即取消近物候选；远处 `bad` 不打断定角转向，转完后再由视觉确认阻止误推。

## 推荐真机顺序

每次只放一个目标并单独保存 CSV：

1. 正前红外触发：应立即停车并进入 `PROBE_VISION_WAIT`。
2. 连续 3 个新 `no_target` 帧：应进入 `ENEMY_PUSH`；重复帧序号不能增加计数。
3. 等待时放入 good 或 bad：应取消攻击，电机保持停止。
4. 等待时遮挡摄像头或让前红外消失：应取消攻击，不得误推。
5. 左前/右前触发：应先停稳（`PROBE_BRAKE`，命令 0,0 约 `PROBE_BRAKE_SECONDS`）再朝对应方向转 45°。
6. 左后/右后触发：应先停稳再朝对应方向转 135°。
7. 正后触发：应先停稳再转 180°。
8. 推到铲子悬空：应停车、倒车收回，不能继续前冲。

CSV 分析重点查看 `vision_sequence`、`vision_status`、`vision_has_good`、
`vision_has_bad`、`probe_vision_count` 和 `probe_vision_verdict`。正常确认应看到
计数 `0 -> 1 -> 2 -> 3(enemy_confirmed)`，而不是在同一帧序号上连续增加。
