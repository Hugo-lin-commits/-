"""单拍冒烟：不接硬件，确认 step() 能跑通。"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from delta_wing_fc import FlightController, ImuSample, VisionSample


def main() -> None:
    fc = FlightController()
    imu = ImuSample(az=-9.81)
    vis = VisionSample(valid=True, px=140, py=100, w=20, h=20, t=0.4)
    out = None
    for k in range(80):
        t = k * 0.005
        if k == 10:
            imu = ImuSample(ax=120.0, az=-9.81)  # 模拟出发射过载
        elif k == 12:
            imu = ImuSample(az=-9.81, pitch=0.2)
        out = fc.step(t, 0.005, imu, vis if t > 0.35 else VisionSample())
    assert out is not None
    print("phase", fc.tel.phase)
    print("delta_deg", tuple(round(d, 2) for d in out.delta_deg))
    print("pwm_us", tuple(round(u, 1) for u in out.pwm_us))
    print("los_yaw_deg", round(fc.tel.los_yaw * 57.3, 2))


if __name__ == "__main__":
    main()
