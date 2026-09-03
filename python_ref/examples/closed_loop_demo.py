"""滚转通道简化闭环：确认三回路/串级都能把阶跃滚转拉回来。"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from delta_wing_fc.params import default_params
from delta_wing_fc.types import ImuSample
from delta_wing_fc.autopilot import Autopilot


def simulate(mode: str, t_end: float = 1.5) -> list[tuple[float, float]]:
    p = default_params()
    p.autopilot_mode = mode
    ap = Autopilot(p.roll, p.pitch, p.yaw, p.cascade, p.couple, mode, p.gravity)
    # 滚转：γ̈ = -dω γ̇ + dδ δr   （符号：正舵产生正角加速度）
    d_omega, d_delta = p.aero.d_omega, abs(p.aero.d_delta)
    gamma, p_rate = 0.35, 0.0  # 初始 20° 滚转
    dt = 0.005
    t = 0.0
    log = []
    while t < t_end:
        imu = ImuSample(p=p_rate, roll=gamma, az=-9.81)
        dp, dr, dy = ap.step(0.0, 0.0, 0.0, 0.0, 0.0, imu, dt, False)
        acc = -d_omega * p_rate + d_delta * dr
        p_rate += acc * dt
        gamma += p_rate * dt
        log.append((t, gamma))
        t += dt
    return log


def main() -> None:
    for mode in ("cascade_pid", "three_loop"):
        log = simulate(mode)
        g0 = log[0][1]
        g1 = log[-1][1]
        print(
            f"{mode:13s}  gamma0={g0*57.3:6.2f} deg  gamma_end={g1*57.3:6.2f} deg"
        )


if __name__ == "__main__":
    main()
