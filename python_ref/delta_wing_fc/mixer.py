"""轴对称 X 四片后缘舵混控。饱和时保滚转、再俯仰、最后牺牲偏航。"""

from __future__ import annotations

from .params import HardwareParams
from .types import MixerOut, clamp, rad2deg


def mix_from_axes(dp: float, dr: float, dy: float) -> tuple[float, float, float, float]:
    """等效舵偏（rad）→ 四片。编号见 HardwareParams。"""
    d1 = dp + dr + dy
    d2 = dp - dr - dy
    d3 = dp - dr + dy
    d4 = dp + dr - dy
    return d1, d2, d3, d4


def allocate(dp: float, dr: float, dy: float, sat: float) -> tuple[tuple[float, float, float, float], float, float, float, bool]:
    """优先滚转（相机固连、升力矢量不能歪），再俯仰，偏航最后。"""

    def peak(a: float, b: float, c: float) -> float:
        vals = mix_from_axes(a, b, c)
        return max(abs(v) for v in vals)

    sat = abs(sat)
    if peak(dp, dr, dy) <= sat + 1e-9:
        return mix_from_axes(dp, dr, dy), dp, dr, dy, False

    # 缩偏航
    lo, hi = 0.0, 1.0
    for _ in range(16):
        m = 0.5 * (lo + hi)
        if peak(dp, dr, m * dy) <= sat:
            lo = m
        else:
            hi = m
    dy *= lo
    if peak(dp, dr, dy) <= sat + 1e-9:
        return mix_from_axes(dp, dr, dy), dp, dr, dy, True

    # 再缩俯仰
    lo, hi = 0.0, 1.0
    for _ in range(16):
        m = 0.5 * (lo + hi)
        if peak(m * dp, dr, dy) <= sat:
            lo = m
        else:
            hi = m
    dp *= lo
    if peak(dp, dr, dy) <= sat + 1e-9:
        return mix_from_axes(dp, dr, dy), dp, dr, dy, True

    # 最后整体缩放（含滚转）
    pk = peak(dp, dr, dy)
    if pk > 1e-9:
        s = sat / pk
        dp, dr, dy = dp * s, dr * s, dy * s
    return mix_from_axes(dp, dr, dy), dp, dr, dy, True


class XMixer:
    def __init__(self, hw: HardwareParams) -> None:
        self.hw = hw

    def apply(self, dp_rad: float, dr_rad: float, dy_rad: float) -> MixerOut:
        sat = abs(self.hw.delta_max_deg) * 3.141592653589793 / 180.0
        raw, dp, dr, dy, sat_flag = allocate(dp_rad, dr_rad, dy_rad, sat)
        deg = []
        pwm = []
        for i, d in enumerate(raw):
            di = rad2deg(d) * self.hw.servo_sign[i] + self.hw.servo_trim_deg[i]
            di = clamp(di, -self.hw.delta_max_deg, self.hw.delta_max_deg)
            deg.append(di)
            pwm.append(self.hw.pwm_us_center + di * self.hw.pwm_us_per_deg)
        return MixerOut(
            delta_deg=(deg[0], deg[1], deg[2], deg[3]),
            pwm_us=(pwm[0], pwm[1], pwm[2], pwm[3]),
            dp=dp,
            dr=dr,
            dy=dy,
            saturated=sat_flag,
        )
