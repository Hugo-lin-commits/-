"""像素 → 视线角 / 视线角速度。"""

from __future__ import annotations

from math import atan

from .filters import DifferentiatorLpf
from .params import HardwareParams
from .types import VisionSample


class LineOfSight:
    def __init__(self, hw: HardwareParams, lpf_hz: float) -> None:
        self.hw = hw
        self.dp = DifferentiatorLpf(lpf_hz)
        self.dy = DifferentiatorLpf(lpf_hz)
        self.pitch = 0.0
        self.yaw = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

    def reset(self) -> None:
        self.dp.reset()
        self.dy.reset()
        self.pitch = 0.0
        self.yaw = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

    def step(self, vis: VisionSample, dt: float) -> bool:
        if not vis.valid:
            return False
        hw = self.hw
        # 小角度可用 (c-cx)/fx，这里用 atan 覆盖边缘大偏差
        ey = atan((vis.cy - hw.cam_cy) / max(hw.cam_fy, 1.0))
        ex = atan((vis.cx - hw.cam_cx) / max(hw.cam_fx, 1.0))
        self.pitch = hw.los_pitch_sign * ey
        self.yaw = hw.los_yaw_sign * ex
        self.pitch_rate = self.dp.step(self.pitch, dt)
        self.yaw_rate = self.dy.step(self.yaw, dt)
        return True
