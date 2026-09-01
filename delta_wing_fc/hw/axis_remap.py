"""芯片系 → 弹体系。

MaixCAM2 默认板坐标（文档）：x 右、y 前（镜头）、z 上。
本飞控弹体：x 机头、y 右、z 下。

若 IMU 与板/镜头对齐，默认::

    body_x =  chip_y
    body_y =  chip_x
    body_z = -chip_z

装歪了只改 idx/sign，不要改控制律。
"""

from __future__ import annotations


def remap(
    vec: tuple[float, float, float],
    idx: tuple[int, int, int],
    sign: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        sign[0] * vec[idx[0]],
        sign[1] * vec[idx[1]],
        sign[2] * vec[idx[2]],
    )
