# dev/ 真机开发与调试工具使用文档

本目录汇集所有**真实硬件**测试、数据采集与离线标定工具。生产比赛只跑根目录
`main.py`；dev 工具用于换车 / 换场地后的标定、状态机验证与故障排查。

## 0. 通用原则

- **生产/dev 分离**：根目录策略模块（`ring_patrol.py`、`reentry.py`、`shovel_guard.py`、
  `vision_tracker.py`、`enemy_push.py` 等）只做纯逻辑，不碰硬件；本目录存放独立真机工具。
- **数据必须落盘**：所有采集/测试都会写 CSV 到 `data/`，阈值一律从 CSV 重算，
  **禁止凭记忆手填参数**。
- **参数统一走 [config.py](../config.py)**：通道映射、阈值、速度等默认值都从 config 注入；
  标定工具输出「建议值」后，人工核对再写回 config.py。
- **真机工具只能在树莓派上跑**（需要 `uptech` 库）；PC 上只能跑带 `--replay` 的回放模式
  和单元测试。
- **电机安全**：凡是会动轮子的工具，先确认车四周无人、线缆远离车轮；多数工具
  Ctrl+C 即急停并安全关停电机。

## 1. 工具总表

| 工具 | 用途 | 动电机 | 树莓派 |
|---|---|:-:|:-:|
| [dev/motor_tool.py](motor_tool.py) | 前进/后退/左右转方向与速度测试 | ✓ | ✓ |
| [dev/turn_tool.py](turn_tool.py) | 转角标定交互采集（直接转 N 度） | ✓ | ✓ |
| [dev/motor_push_back.py](motor_push_back.py) | 逐档测倒车上台所需力度 | ✓ | ✓ |
| [dev/gray_tool.py](gray_tool.py) | 四路灰度扫描 / 采集 | – | ✓ |
| [dev/ir_tool.py](ir_tool.py) | 前头两路模拟红外扫描 / 采集 | – | ✓ |
| [dev/digi_ir_tool.py](digi_ir_tool.py) | 六路数字红外扫描 / 采集 | – | ✓ |
| [dev/shovel_tool.py](shovel_tool.py) | 铲子底下两路红外扫描 / 采集 | – | ✓ |
| [dev/calibrate_gray.py](calibrate_gray.py) | 灰度模型离线重算 | – | 均可 |
| [dev/calibrate_front_adc.py](calibrate_front_adc.py) | 前头红外对齐模型离线重算 | – | 均可 |
| [dev/calibrate_shovel.py](calibrate_shovel.py) | 铲子悬空/收回阈值离线重算 | – | 均可 |
| [dev/ring_patrol.py](ring_patrol.py) | 巡台状态机真机测试 + CSV 日志 | ✓ | ✓ |
| [dev/reentry.py](reentry.py) | 掉台回归真机测试 / PC 灰度回放 | ✓(真机) | 真机 / PC |
| [dev/shovel_guard.py](shovel_guard.py) | 铲子防掉落真机测试 / PC 回放 | ✓(真机) | 真机 / PC |
| [dev/vision_tracker.py](vision_tracker.py) | YOLO 能量块对准真机测试 | 可选 | ✓ |
| [dev/enemy_push.py](enemy_push.py) | 后台 YOLO + 六路红外敌人搜索/推动集成测试 | 可选 | ✓ |

通用参数：`--motor-invert / --no-motor-invert`、`--motor-swap / --no-motor-swap`（电机类工具）
覆盖 config 的 `CHASSIS_MOTOR_INVERT / CHASSIS_MOTOR_SWAP` 默认值；
`--hz` 控制循环频率、`--seconds` 限制运行时长（0 = 手动 Ctrl+C 停止）、`--log` 指定
日志路径（状态机测试工具）。

## 2. 电机与底盘

### 2.1 dev/motor_tool.py —— 四种基本动作方向测试（换车第一件事）

```bash
python3 dev/motor_tool.py all            # 全部四种动作
python3 dev/motor_tool.py forward --ground
python3 dev/motor_tool.py turn-left --speed 500 --duration 0.8
```

- 默认**悬空**（抬轮，确认词 `LIFTED`）；`--ground` 为着地实车动作（确认词 `GROUND`），
  着地模式每个动作前有倒计时。
- 每个动作前回车准备，`s` 跳过，`q` 退出；动作后交互记录观察结果（y/n/?），
  前进/后退会询问实测距离 cm，全部写入 CSV。
- `--speed` 范围 400..800（400 为实测电机可靠驱动下限），`--duration` 动作时长，
  `--pause` 动作间隔，`--countdown` 着地倒计时秒数。
- 方向不对时：先用 `--motor-invert` / `--motor-swap` 现场修正，确认后把结果写回
  config.py 的 `CHASSIS_MOTOR_INVERT / CHASSIS_MOTOR_SWAP`。
- `DEV_MODE=False` 会禁用真机测试（代码内开关）。

### 2.2 dev/motor_push_back.py —— 倒车上台力度逐档测试

```bash
python3 dev/motor_push_back.py                            # 默认 500~900 共 5 档
python3 dev/motor_push_back.py --speeds 600               # 只测单档
python3 dev/motor_push_back.py --speeds 500,700 --reverse-seconds 3
```

- 每档流程：前进到准备位 → 停车 → 大力倒车冲台 → 停车 → 交互询问「上台了吗？」
  （y/n/?），结果存 `data/motor_push_back_时间.csv`。
- `--forward-speed / --forward-seconds` 控制准备段，`--reverse-seconds` 控制冲台时长。
- 纯电机测试：直接开 vendor 库裸控 CDS，**不经 up_controller.py**，不读传感器。

### 2.3 dev/turn_tool.py —— 转角标定交互采集（换车必测）

```bash
python3 dev/turn_tool.py                 # 按默认 10 档角度集，从缺档开始
python3 dev/turn_tool.py --angle 90      # 只补测 90°
```

- 角度集：**10 / 22.5 / 45 / 90 / 135 / 180**（已标定，共 12 组；补测其他角度用 `--angle N`），左右转各定稿一组。
- 每档流程：输入「速度 时长」（如 `500 1`）→ 车原地转 → 输入实测角度
  （**回车 = 刚好到位 → 定稿**；数字 = 记录后继续调参；`r` = 原参数重跑；
  `s` = 跳过；`q` = 保存退出）。转角与时长非线性，每个目标角独立实测，不做插值。
- 已定稿的档会从 CSV 读取续测（断电/中断可接着跑）。
- 产出：`data/motor_turn_calibration.csv`（单测格式 direction,angle,speed,duration）
  + `data/motor_turn_trials_时间.csv`（每次试验明细）。
- 定稿后同步进 config.py 的 `MOTOR_TURN_CALIBRATION`（左右两张表，按转向方向查）。

## 3. 传感器扫描与采集

四个采集工具结构一致：`scan` 持续刷新读数（Ctrl+C 退出），`collect` 采 CSV
（`--hz` 可选 10/50，默认 50；`--dur` 秒数定时或 0 = 手动 Ctrl+C 停止；
`--out` 不写则交互输入文件名，回车用默认时间戳名）。

### 3.1 dev/gray_tool.py —— 四路灰度

```bash
python3 dev/gray_tool.py scan                              # front/rear/left/right 映射值
python3 dev/gray_tool.py collect --hz 50                   # 输出 data/gray_时间.csv
```

- 通道映射来自 config `GRAY_CHANNELS`（front=adc2, rear=adc3, left=adc0, right=adc1），
  映射与归一化来自 `GRAY_ADC_MAX / GRAY_WHITE_ENTER`。
- CSV 字段：`t, front, rear, left, right`。

### 3.2 dev/ir_tool.py —— 前头两路模拟红外

```bash
python3 dev/ir_tool.py scan                # 全 10 路 ADC，L/R 标记前头两路（先确认接线）
python3 dev/ir_tool.py scan --mapped       # 映射后的 left/right/diff/valid
python3 dev/ir_tool.py collect             # 输出 data/front_adc_时间.csv
```

- 通道映射来自 config `IR_CHANNELS`（left=adc7, right=adc6，新车 2026-08-15 scan 实测）。
- CSV 字段：`t, left, right, diff, valid`。

### 3.3 dev/digi_ir_tool.py —— 六路数字红外

```bash
python3 dev/digi_ir_tool.py scan           # 8 位 IO 原始电平 + pin 映射提示
python3 dev/digi_ir_tool.py collect        # 原始 IO 与六路映射状态同存
```

- pin 映射与有效电平来自 config `DIGI_IR_PINS / DIGI_IR_BITS / DIGI_IR_ACTIVE_LEVEL`。
- CSV 同时保存原始掩码 `mask, io0..io7` 和映射状态
  `left_rear, left_front, right_rear, right_front, rear, front, valid`，便于接线排查。

### 3.4 dev/shovel_tool.py —— 铲子底下两路模拟红外

```bash
python3 dev/shovel_tool.py scan            # 全 10 路 ADC + shovel left/right（确认接线）
python3 dev/shovel_tool.py collect         # 交互选分组后采集
```

- collect 会交互询问分组：**1 = 悬空**（铲子出台，自动加 `shovel_hang_` 前缀）、
  **2 = 台内**（铲子正常贴台面，`shovel_stage_` 前缀）、**3 = 自定义**（不参与标定）。
- 文件名的 `hang / 悬空 / 出台` 或 `stage / 台内 / 台上` 关键词供
  calibrate_shovel.py 分组，自定义文件会被忽略。
- CSV 字段：`t, left, right, valid`。

## 4. 离线标定工具（PC 可跑）

### 4.1 dev/calibrate_gray.py —— 灰度巡台模型重算

```bash
python3 dev/calibrate_gray.py
python3 dev/calibrate_gray.py --data-dir data --out data/gray_model.csv
```

- 从 `data/` 下**固定文件名**的采集 CSV 重算：`四个同黑` / `四个同白` /
  `竖向边缘*` / `边缘*` / `武字数据*`（含中间旋转）/ `中轴*` / `对角轴*` /
  `更内环*`（4 个朝向）。缺文件会直接报错列出。
- 输出 `data/gray_model.csv`（edge/center/white 参考值、white_enter/clear）与
  `data/gray_model_summary.csv`（各组 zone_score 分位、near_edge_rate、white_hit_rate）。
- 核对摘要后把参数写回 config.py 的 `GRAY_*` 系列。

### 4.2 dev/calibrate_front_adc.py —— 前头红外对齐模型重算

```bash
python3 dev/calibrate_front_adc.py
python3 dev/calibrate_front_adc.py --window 9 --active-min 50
```

- 处理 `data/` 下所有前头 ADC CSV（字段 `t,left,right,diff,valid`），按文件名分类：
  `正对墙/正对着墙`（`居中位置`=middle / `最远`=far / 其余=near）、`左偏`、`右偏`；
  含「从90度」的按 start/transition/end 分阶段，否则整段 steady。
- 输出 `front_adc_processed.csv`（滤波明细）、`front_adc_summary.csv`（分组统计，
  含 `alignment_usable` 可用性判断）、`front_adc_model.csv`（diff_low/diff_high/
  signal_min 等建议值）。
- 要求存在「左偏 / 正对居中 / 右偏」三组 steady 数据且区间不重叠，否则报错重采。
- 核对后写回 config.py 的 `IR_ALIGNMENT_*` 系列。

### 4.3 dev/calibrate_shovel.py —— 铲子悬空/收回阈值重算

```bash
python3 dev/calibrate_shovel.py
```

- 按文件名关键词把 `data/` 下采集 CSV 分成悬空组（hang）与台内组（stage），
  先按 `SHOVEL_FILTER_WINDOW` 对每帧 max/min(两路) 做滚动中值滤波（与 guard 判据一致）。
- 新车极性（2026-08-15 起）：悬空=信号高、台内=信号低。
  ENTER = 台内 min p99 与悬空 min p01 的中点（min(两路) 高于它判悬空）；
  CLEAR = 台内 max p99 与悬空 max p01 的中点（max(两路) 低于它判收回，CLEAR>ENTER 滞回）。
- 两组区间重叠会报错并提示重采（悬空要真正伸出、台内要正常贴台面）。
- 输出 `data/shovel_model.csv` 并打印建议值，核对后写回 config.py 的
  `SHOVEL_HANG_ENTER / SHOVEL_HANG_CLEAR`。

## 5. 状态机真机测试与回放

三个状态机工具真机模式结构一致：独立拉起 `UpController` + 传感器，把状态机输出
直接送到电机，每帧写 CSV（字段含状态、原因、两轮命令、健康标志），Ctrl+C 安全停车。

### 5.1 dev/ring_patrol.py —— 巡台

```bash
python3 dev/ring_patrol.py                          # 一直跑到 Ctrl+C
python3 dev/ring_patrol.py --seconds 60 --hz 50     # 定时跑，便于复现
```

- 日志 `data/patrol_时间.csv` 含灰度原始值、zone_score、white_hits、state/reason、
  speed_level、转向量等，可直接作图分析行为。
- 新场地/新轮胎后重点观察：内环是否稳定、台边是否提前规避、白区能否逃出。

### 5.2 dev/reentry.py —— 掉台回归

```bash
python3 dev/reentry.py                               # 真机：真实掉台后自动触发
python3 dev/reentry.py --force-trigger               # 台架测试：启动即强制触发一次
python3 dev/reentry.py --replay data/xxx.csv         # PC 回放灰度 CSV
```

- `--force-trigger` 只用于台架验证状态机时序，**真实掉台测试不要传**。
- 回放模式读灰度 CSV（列须为 `t,front,rear,left,right`），红外/前头 ADC 用假数据，
  输出触发时刻、原因与 zone_score 范围。
- 真机日志 `data/reentry_时间.csv` 含灰度、六路数字红外、前头 ADC、fall/fall_edge、
  state/reason 与电机命令。

### 5.3 dev/shovel_guard.py —— 铲子防掉落

```bash
python3 dev/shovel_guard.py                          # 静止模式：悬空只停车
python3 dev/shovel_guard.py --active                 # 推东西模式：悬空触发停车+倒车收回
python3 dev/shovel_guard.py --replay data/xxx.csv    # PC 回放（t,left,right[,valid]）
```

- 生产里 active 由视觉/数字红外给出，dev 用 `--active` 模拟。
- 回放模式默认开启 active，逐帧打印非 IDLE 状态与原因。

### 5.4 dev/vision_tracker.py —— YOLO 能量块对准

```bash
python3 dev/vision_tracker.py                        # 只观察，不动电机（默认）
python3 dev/vision_tracker.py --drive                # 安全确认后启用转向
```

- 需要树莓派上 `rpi-yolo-pi4-int8-lto-8fps/` 目录的 `rpi_yolo_api.VisionClient`
  （YOLO 实际约 8 FPS，`--hz` 默认 50 只是安全轮询频率）。
- `--max-age-ms` 超过约 3 帧周期无新结果会立即停车（默认 450）。
- `--drive` 会要求输入 `DRIVE` 确认安全；日志 `data/vision_tracker_时间.csv`
  每个新视觉结果一行，含偏移、滤波误差、转向命令与状态。

## 6. 标准工作流：新车上手 / 换场地

按顺序执行，每一步的产物都落到文件，再写回 config.py：

1. **电机方向**：`dev/motor_tool.py all`（先悬空后 `--ground`）→ 定
   `CHASSIS_MOTOR_INVERT / CHASSIS_MOTOR_SWAP`；换车另需
   `dev/turn_tool.py` 采转角（左右分开）→ 写 `MOTOR_TURN_CALIBRATION`，
   及 `dev/motor_tool.py --ground forward/backward --speed 550 --duration 1.5`
   测直线距离 → 写 `PATROL_RECOVER_STEP_CM`。
2. **灰度巡台**：`gray_tool.py scan` 确认四路接线 → `collect` 采黑/白/边缘/武字中心/
   中轴对角轴/内环各姿态 → `calibrate_gray.py` → 写回 `GRAY_*`。
3. **前头红外对齐**：`ir_tool.py scan` 确认接线 → `collect` 采正对/左偏/右偏
   （含远距离正对墙，文件名按 4.2 的分类约定命名）→ `calibrate_front_adc.py` →
   写回 `IR_ALIGNMENT_*`。
4. **数字红外**：`digi_ir_tool.py scan` 确认六个 io 位与 `DIGI_IR_BITS` 一致。
5. **铲子**：`shovel_tool.py scan` 确认接线 → `collect` 采悬空/台内两组 →
   `calibrate_shovel.py` → 写回 `SHOVEL_HANG_*`。
6. **巡台验证**：`dev/ring_patrol.py --seconds 60`，看日志确认台边规避、白区逃出、
   内环稳定；调 `PATROL_*` 参数。
7. **掉台验证**：`dev/reentry.py --force-trigger` 台架过一遍 → 真实掉台复测
   `REENTRY_*` 参数（恢复距离、冲台力度）。
8. **倒车上台**（若掉台回归用倒车方案）：`dev/motor_push_back.py` 逐档测出最小可用
   力度 → 写回 `REENTRY_*`。
9. **铲子防掉落**：`dev/shovel_guard.py --active` 台架悬空验证停车+收回时序。
10. **PC 回归**：`python3 -m unittest discover -s tests -t . -v` 保证纯逻辑测试通过。

## 7. 验证清单（真机必做，不能只靠单测）

- [ ] 电机四动作方向正确（悬空 + 着地各过一遍）
- [ ] 巡台完整跑一圈：不掉台、白区能逃出、内环姿态稳定
- [ ] 掉台回归：台架强制触发 + 真实掉台各一次，能回到台上
- [ ] 铲子出台（悬空）能停住/收回，台内不误触发
- [ ] 每个标定工具的 CSV 输出人工核对过再写回 config.py
