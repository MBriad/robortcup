"""vision_tracker_pack 精简参数（只含 VisionTracker 用到的 VISION_* 参数）。

完整参数见主项目 config.py；此处每参数注明来源。
来源：data/vision_tracker_20260816_110510.csv / 110551.csv（2026-08-16 新车实测）。
"""

# ---------- YOLO good 能量块追踪（vision_tracker.py VisionTracker 参数） ----------
VISION_DEAD_ZONE = 0.08        # 居中后直行；640 宽画面约为中心左右各 26 px
VISION_BIG_TURN_ENTER = 0.55   # 进入原地大转
VISION_BIG_TURN_CLEAR = 0.35   # 大转退出滞回，避免大小转反复切换
VISION_BIG_TURN_SPEED = 400    # 原地大转；新车 CDS 伺服最低可靠转速
VISION_ARC_INNER_SPEED = 400   # 小转内侧轮
VISION_ARC_OUTER_SPEED = 500   # 小转外侧轮；已验证的 500/400 差速
VISION_APPROACH_SPEED = 400    # 对准后直线接近
