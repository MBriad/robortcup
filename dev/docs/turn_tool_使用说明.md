# dev/turn_tool.py 使用说明

## 用途

标定「直接转到 N 度」所需的 (速度, 时长) 组合。转角与时长非线性(起步、摩擦、电池
都有影响),每个目标角独立实测、不做插值。定稿表同步进 config.py 的
`MOTOR_TURN_CALIBRATION`(左右两张表,按转向方向查),供巡台大转与掉台回归转向使用。

## 运行(仅树莓派)

```bash
python3 dev/turn_tool.py                 # 默认角度集,从缺档续测
python3 dev/turn_tool.py --angle 90      # 只测单个角度
python3 dev/turn_tool.py --angles 10,90,180
```

- 默认角度集:**10 / 22.5 / 45 / 90 / 135 / 180**,左右转各定稿一组
  (共 12 组,已标定档自动跳过;补测其他角度用 `--angle N`)。
- `--motor-invert` / `--motor-swap` 覆盖 config 默认值(一般不用加)。

## 每档操作流程

1. 车放平整地面,车头对准 0° 参考线(地上画线或手机指南针),人与线缆远离车轮。
2. 输入「速度 时长」,如 `500 1` → 车原地转这一下。
3. 输入实测角度:
   - **回车** = 正好到位 → 定稿,自动写入 `motor_turn_calibration.csv`;
   - **数字** = 记录实际角度,继续调参(再试新的速度/时长);
   - `r` = 原参数重跑;`s` = 跳过此档;`q` = 保存退出。
4. Ctrl+C 随时急停(电机立即停止)。

## 输出文件

| 文件 | 内容 |
|---|---|
| `data/motor_turn_calibration.csv` | 定稿表,列 `direction,angle,speed,duration`;已定稿档自动跳过,可断点续测 |
| `data/motor_turn_trials_时间.csv` | 每次试验明细(含实测角度),复盘调参过程用 |

## 定稿后

把 `motor_turn_calibration.csv` 拷回 PC 仓库,将整表同步进 config.py 的
`MOTOR_TURN_CALIBRATION` 左右两张表(单测 `tests/test_motor.py` 会校验
CSV 与 config 一致)。

## 注意事项

- 每档测完要把车摆回起点方向再测下一档(左/右转各自从 0° 开始)。
- 大角度(180/225/360)测量误差大,多试几次取稳定值;定稿后可以随时用
  `--angle N` 回来重测某一档。
