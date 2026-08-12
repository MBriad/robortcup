"""真机单模块测试与数据回放入口。"""

import os


def prompt_output_path(default_path):
    """采集工具共用：--out 未给时交互询问文件名（存到同目录，回车用默认）。

    真机 Linux 终端上直接输文件名（支持中文），补 .csv 后缀。
    """
    default = default_path
    name = input(
        "输出文件名（保存到 %s，直接回车用默认 %s）：" % (
            os.path.dirname(default), os.path.basename(default),
        )
    ).strip()
    if not name:
        return default
    if not name.lower().endswith(".csv"):
        name += ".csv"
    return os.path.join(os.path.dirname(default), name)
