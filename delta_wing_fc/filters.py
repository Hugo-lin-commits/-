"""一阶低通、伪攻角估计。"""

from __future__ import annotations

from math import pi

from .types import clamp


class LowPass:
    def __init__(self, hz: float, x0: float = 0.0) -> None:
        self.hz = hz
        self.y = x0

    def reset(self, x0: float = 0.0) -> None:
        self.y = x0

    def step(self, x: float, dt: float) -> float:
        if dt <= 0.0 or self.hz <= 0.0:
            self.y = x
            return self.y
        a = 2.0 * pi * self.hz * dt
        k = a / (1.0 + a)
        self.y += k * (x - self.y)
        return self.y


class DifferentiatorLpf:
    """带低通的差分，用来从视线角得到 q̇。"""

    def __init__(self, hz: float) -> None:
        self.lpf = LowPass(hz)
        self.prev = 0.0
        self.has = False

    def reset(self) -> None:
        self.lpf.reset(0.0)
        self.prev = 0.0
        self.has = False

    def step(self, x: float, dt: float) -> float:
        if not self.has or dt <= 0.0:
            self.prev = x
            self.has = True
            self.lpf.reset(0.0)
            return 0.0
        raw = (x - self.prev) / dt
        self.prev = x
        return self.lpf.step(raw, dt)


class PseudoAoA:
    """西交伪攻角方案 1：用角速度一阶滤波，避免加速度计积分漂移。

    短周期近似 α̇ ≈ q − α / Tα  （侧滑同理 β̇ ≈ −r − β / Tα，符号随坐标系）
    """

    def __init__(self, T_alpha: float, lim: float) -> None:
        self.T = T_alpha
        self.lim = lim
        self.x = 0.0

    def reset(self, x0: float = 0.0) -> None:
        self.x = x0

    def step(self, rate: float, dt: float) -> float:
        # x_dot = rate - x / T
        if self.T <= 1e-4:
            self.x = rate
        else:
            self.x += (rate - self.x / self.T) * dt
        self.x = clamp(self.x, -self.lim, self.lim)
        return self.x
