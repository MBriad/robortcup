# dev/motor_tool.py 使用说明

## 用途

车体四种基本动作(前进 / 后退 / 原地左转 / 原地右转)的真机测试:
验证电机方向、测量直线距离、观察动作正确性。换车第一件事就是跑它。

## 运行(仅树莓派)

```bash
python3 dev/motor_tool.py <action> [--ground] [--speed N] [--duration S]
                                  [--pause S] [--countdown N] [--log PATH]
```

- `action`:`forward` / `backward` / `turn-left` / `turn-right` / `all`(四种都测)。
- `--ground`:着地实车模式;**不加默认悬空**(抬轮)。
- `--speed`:400~800(默认 400,400 是电机可靠下限)。
- `--duration`:动作时长秒(默认 0.5)。
- `--pause`:动作间隔秒(默认 0.8)。
- `--countdown`:着地模式每动作前倒计时秒数(默认 3)。
- `--motor-invert` / `--motor-swap`:临时覆盖 config 的方向修正。

## 安全确认

启动后按模式输入确认词,输错即取消:

- 悬空模式:输入 `LIFTED`
- 着地模式(`--ground`):输入 `GROUND`

## 交互流程

1. 每个动作前显示命令,回车执行,`s` 跳过,`q` 退出。
2. 动作结束后输入观察结果:`y` 正确 / `n` 错误 / 回车=不确定。
3. 前进/后退还会询问**实测移动距离 cm**(直接回车跳过)。
4. 全部写进 CSV,随时 Ctrl+C 急停。

## 输出文件

`data/motor_test_时间.csv`,字段:`step, mode, action, description, left_cmd,
right_cmd, duration, distance_cm, motor_invert, motor_swap, observed`。

## 典型用法

```bash
# 换车方向验证:先悬空过一遍,再着地过一遍
python3 dev/motor_tool.py all
python3 dev/motor_tool.py all --ground

# 直线标定:550 速度 1.5 秒的前进/后退实测距离(各测 3 次取均值)
python3 dev/motor_tool.py forward  --ground --speed 550 --duration 1.5
python3 dev/motor_tool.py backward --ground --speed 550 --duration 1.5
```

- 方向不对:现场用 `--motor-invert` / `--motor-swap` 验证正确组合后,
  写回 config.py 的 `CHASSIS_MOTOR_INVERT / CHASSIS_MOTOR_SWAP`。
- 直线标定结果填进 `data/motor_linear_calibration.csv` 并更新
  `PATROL_RECOVER_STEP_CM`。

## 注意事项

- 原地左右转**只记观察结果,不量角度**;量角度用 `dev/turn_tool.py`。
- 每个动作前后电机都会停,人与线缆远离车轮。
- 代码内 `DEV_MODE=False` 会禁用真机测试。
