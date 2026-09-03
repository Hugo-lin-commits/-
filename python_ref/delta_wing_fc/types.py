"""基础类型与数学。"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, pi, sin


def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def wrap_pi(a: float) -> float:
    while a > pi:
        a -= 2.0 * pi
    while a < -pi:
        a += 2.0 * pi
    return a


def deg2rad(d: float) -> float:
    return d * pi / 180.0


def rad2deg(r: float) -> float:
    return r * 180.0 / pi


@dataclass
class ImuSample:
    """机体系：x 机头 y 右 z 下。欧拉角 3-2-1（西交）：yaw, pitch, roll。"""

    p: float = 0.0  # 滚转角速度
    q: float = 0.0  # 俯仰角速度
    r: float = 0.0  # 偏航角速度
    ax: float = 0.0  # 比力 m/s²
    ay: float = 0.0
    az: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class VisionSample:
    valid: bool = False
    px: float = 0.0  # blob 左上角 x（与深圳大学 OpenMV 协议一致）
    py: float = 0.0
    w: float = 0.0
    h: float = 0.0
    t: float = 0.0

    @property
    def cx(self) -> float:
        return self.px + 0.5 * self.w

    @property
    def cy(self) -> float:
        return self.py + 0.5 * self.h


@dataclass
class MixerOut:
    delta_deg: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    pwm_us: tuple[float, float, float, float] = (1500.0, 1500.0, 1500.0, 1500.0)
    dp: float = 0.0
    dr: float = 0.0
    dy: float = 0.0
    saturated: bool = False


@dataclass
class FcTelemetry:
    phase: str = "idle"
    ay_cmd: float = 0.0
    az_cmd: float = 0.0
    alpha_hat: float = 0.0
    beta_hat: float = 0.0
    los_pitch: float = 0.0
    los_yaw: float = 0.0
    los_pitch_rate: float = 0.0
    los_yaw_rate: float = 0.0
    mix: MixerOut = field(default_factory=MixerOut)
