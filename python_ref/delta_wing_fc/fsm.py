"""飞行阶段。"""

from __future__ import annotations

from .params import FlightTiming
from .types import ImuSample, VisionSample


class FlightFsm:
    def __init__(self, t: FlightTiming) -> None:
        self.t = t
        self.phase = "idle"
        self.t0 = 0.0
        self.t_now = 0.0

    def reset(self) -> None:
        self.phase = "idle"
        self.t0 = 0.0
        self.t_now = 0.0

    def elapsed(self) -> float:
        return self.t_now - self.t0

    def step(self, t: float, imu: ImuSample, vis: VisionSample, vis_age: float) -> str:
        self.t_now = t
        acc = (imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az) ** 0.5
        launch_g = acc / 9.81

        if self.phase == "idle":
            if launch_g >= self.t.launch_acc_g:
                self.phase = "launch"
                self.t0 = t
        elif self.phase == "launch":
            if self.elapsed() >= self.t.t_launch_hold:
                self.phase = "coast"
        elif self.phase == "coast":
            if vis.valid and vis_age < self.t.vision_stale_s and self.elapsed() >= self.t.t_coast:
                self.phase = "guide"
                self.t0 = t
            elif self.elapsed() >= self.t.t_coast:
                self.phase = "seek"
        elif self.phase == "seek":
            if vis.valid and vis_age < self.t.vision_stale_s:
                self.phase = "guide"
                self.t0 = t
            elif self.elapsed() >= self.t.t_seek_max:
                self.phase = "hold"
        elif self.phase == "guide":
            if self.elapsed() >= self.t.t_terminal and self.elapsed() >= self.t.t_guide_min:
                # 剩最后一小段：仍导引但外层会缩小指令
                if vis_age > self.t.vision_stale_s:
                    self.phase = "terminal"
            if vis_age > 3.0 * self.t.vision_stale_s and self.elapsed() > self.t.t_guide_min:
                self.phase = "hold"
        elif self.phase == "terminal":
            pass
        elif self.phase == "hold":
            pass

        if t > self.t.t_total_max and self.phase not in ("idle",):
            self.phase = "done"
        return self.phase
