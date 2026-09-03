"""抗饱和 PID，串级姿态用。"""

from __future__ import annotations

from .params import CascadePidGains
from .types import clamp


class Pid:
    def __init__(self, g: CascadePidGains) -> None:
        self.g = g
        self.i = 0.0
        self.prev_e = 0.0
        self.has = False

    def reset(self) -> None:
        self.i = 0.0
        self.prev_e = 0.0
        self.has = False

    def step(self, e: float, dt: float) -> float:
        d = 0.0
        if self.has and dt > 0.0:
            d = (e - self.prev_e) / dt
        self.has = True
        self.prev_e = e

        p = self.g.kp * e
        self.i = clamp(self.i + self.g.ki * e * dt, -self.g.i_lim, self.g.i_lim)
        u = p + self.i + self.g.kd * d
        return clamp(u, -self.g.out_lim, self.g.out_lim)
