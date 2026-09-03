"""PC 上检查 42688 换算和坐标映射，不接板。"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from delta_wing_fc.hw.axis_remap import remap
from delta_wing_fc.hw.icm42688 import accel_raw_to_mps2, gyro_raw_to_rad_s


def main() -> None:
    # ±2000 dps 满量程 raw=32767 → 约 34.9 rad/s
    w = gyro_raw_to_rad_s(32767, 2000.0)
    assert 34.0 < w < 36.0, w
    # ±16 g、raw=2048 → 1 g
    a = accel_raw_to_mps2(2048, 16.0, 9.81)
    assert abs(a - 9.81) < 0.05, a

    # 芯片 y 前、x 右、z 上 → 弹体 x 前 y 右 z 下
    chip = (0.1, 0.2, 0.3)  # x,y,z chip
    body = remap(chip, (1, 0, 2), (1.0, 1.0, -1.0))
    assert body == (0.2, 0.1, -0.3), body
    print("icm42688 scale + remap ok")


if __name__ == "__main__":
    main()
