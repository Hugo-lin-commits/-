"""导引律：像素姿态、比例导引、自适应滑模（西交 ISMCG）。"""

from __future__ import annotations

from math import sqrt

from .params import GuidanceParams
from .types import clamp


class RangeObserver:
    """西交：沿视线 ṙ=v, v̇=r q̇²，用 r0±δr、v0±δv 夹出上下界。"""

    def __init__(self, p: GuidanceParams) -> None:
        self.p = p
        self.r = p.r0
        self.v = -abs(p.v0)  # 接近为负
        self.r_hi = p.r0 + p.r0_err
        self.r_lo = max(0.5, p.r0 - p.r0_err)
        self.v_hi = -abs(p.v0) + p.v0_err
        self.v_lo = -abs(p.v0) - p.v0_err

    def reset(self) -> None:
        p = self.p
        self.r = p.r0
        self.v = -abs(p.v0)
        self.r_hi = p.r0 + p.r0_err
        self.r_lo = max(0.5, p.r0 - p.r0_err)
        self.v_hi = -abs(p.v0) + p.v0_err
        self.v_lo = -abs(p.v0) - p.v0_err

    def step(self, qdot: float, dt: float) -> None:
        q2 = qdot * qdot
        self.r += self.v * dt
        self.v += self.r * q2 * dt
        self.r_hi += self.v_hi * dt
        self.r_lo += self.v_lo * dt
        self.v_hi += self.r_hi * q2 * dt
        self.v_lo += self.r_lo * q2 * dt
        self.r = max(0.3, self.r)
        self.r_hi = max(0.3, self.r_hi)
        self.r_lo = max(0.3, self.r_lo)

    @property
    def vc(self) -> float:
        return abs(self.v)

    @property
    def dv(self) -> float:
        return 0.5 * abs(self.v_hi - self.v_lo)


class Guidance:
    def __init__(self, p: GuidanceParams, mode: str) -> None:
        self.p = p
        self.mode = mode
        self.obs = RangeObserver(p)
        self.k_hat = 0.0

    def reset(self) -> None:
        self.obs.reset()
        self.k_hat = 0.0

    def pixel_angle(self, los_p: float, los_y: float) -> tuple[float, float, float, float]:
        kp = self.p.pixel_angle_kp
        return kp * los_p, kp * los_y, 0.0, 0.0

    def png(self, qdot_p: float, qdot_y: float, dt: float) -> tuple[float, float]:
        qdot = sqrt(qdot_p * qdot_p + qdot_y * qdot_y)
        self.obs.step(qdot, dt)
        vc = self.obs.vc if self.obs.vc > 1.0 else self.p.vc_fallback
        n = self.p.N * vc
        # 过载指令（g）：a = N |vc| q̇ ，再 /g
        ay = n * qdot_p / 9.81
        az = n * qdot_y / 9.81
        return ay, az

    def ismcg(self, qdot_p: float, qdot_y: float, dt: float) -> tuple[float, float]:
        """a_mq = [(N - r_hi/r_lo + k̂) |v| + 2|Δv|] q̇

        西交：滑模面 s = r̂ q̇，自适应估计机动上界，不需要测目标加速度。
        """
        qdot = sqrt(qdot_p * qdot_p + qdot_y * qdot_y)
        self.obs.step(qdot, dt)
        r, r_hi, r_lo = self.obs.r, self.obs.r_hi, max(self.obs.r_lo, 0.3)
        vabs = abs(self.obs.v)
        ratio = r_hi / r_lo
        kdot = (1.0 / max(self.p.ismcg_gamma, 1e-3)) * (r * r / r) * vabs * (qdot * qdot)
        self.k_hat = clamp(self.k_hat + kdot * dt, 0.0, self.p.ismcg_k_max)
        gain = (self.p.ismcg_N - ratio + self.k_hat) * vabs + 2.0 * self.obs.dv
        gain = max(gain, 0.0)
        ay = gain * qdot_p / 9.81
        az = gain * qdot_y / 9.81
        return ay, az
