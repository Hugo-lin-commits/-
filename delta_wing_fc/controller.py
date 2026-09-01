"""顶层：传感器 → 导引 → 自动驾驶仪 → 混控。

每个控制周期调用一次 ``FlightController.step``。移植到 STM32 时保持这个调用顺序即可。
"""

from __future__ import annotations

from .autopilot import Autopilot
from .fsm import FlightFsm
from .guidance import Guidance
from .los import LineOfSight
from .mixer import XMixer
from .params import Params, default_params
from .types import FcTelemetry, ImuSample, MixerOut, VisionSample, clamp


class FlightController:
    def __init__(self, params: Params | None = None) -> None:
        self.p = params if params is not None else default_params()
        self.los = LineOfSight(self.p.hw, self.p.guid.los_lpf_hz)
        self.guid = Guidance(self.p.guid, self.p.guidance_mode)
        self.ap = Autopilot(
            self.p.roll,
            self.p.pitch,
            self.p.yaw,
            self.p.cascade,
            self.p.couple,
            self.p.autopilot_mode,
            self.p.gravity,
        )
        self.mix = XMixer(self.p.hw)
        self.fsm = FlightFsm(self.p.time)
        self.tel = FcTelemetry()
        self._last_vis_t = -1e9
        self._pitch_cmd0 = 0.0
        self._yaw_cmd0 = 0.0
        self._armed_att = False

    def reset(self) -> None:
        self.los.reset()
        self.guid.reset()
        self.ap.reset()
        self.fsm.reset()
        self.tel = FcTelemetry()
        self._last_vis_t = -1e9
        self._armed_att = False

    def _apply_imu_sign(self, imu: ImuSample) -> ImuSample:
        gs = self.p.hw.imu_gyro_sign
        acs = self.p.hw.imu_accel_sign
        return ImuSample(
            p=gs[0] * imu.p,
            q=gs[1] * imu.q,
            r=gs[2] * imu.r,
            ax=acs[0] * imu.ax,
            ay=acs[1] * imu.ay,
            az=acs[2] * imu.az,
            roll=imu.roll,
            pitch=imu.pitch,
            yaw=imu.yaw,
        )

    def step(self, t: float, dt: float, imu: ImuSample, vis: VisionSample) -> MixerOut:
        imu = self._apply_imu_sign(imu)
        if vis.valid:
            self._last_vis_t = vis.t if vis.t > 0.0 else t
        vis_age = t - self._last_vis_t
        phase = self.fsm.step(t, imu, vis, vis_age)

        if not self._armed_att and phase != "idle":
            self._pitch_cmd0 = imu.pitch
            self._yaw_cmd0 = imu.yaw
            self._armed_att = True

        saw = self.los.step(vis, dt) if vis.valid else False
        pitch_cmd = self._pitch_cmd0
        yaw_cmd = self._yaw_cmd0
        ay_cmd = 0.0
        az_cmd = 0.0
        use_accel = False
        scale = 1.0

        if phase in ("idle", "launch"):
            # 配平
            pitch_cmd, yaw_cmd = imu.pitch, imu.yaw
        elif phase in ("coast", "seek", "hold"):
            pitch_cmd, yaw_cmd = self._pitch_cmd0, self._yaw_cmd0
        elif phase in ("guide", "terminal"):
            mode = self.p.guidance_mode
            if mode == "pixel_angle" and saw:
                dp, dy, _, _ = self.guid.pixel_angle(self.los.pitch, self.los.yaw)
                pitch_cmd = imu.pitch + dp
                yaw_cmd = imu.yaw + dy
            elif mode == "png" and saw:
                ay_cmd, az_cmd = self.guid.png(self.los.pitch_rate, self.los.yaw_rate, dt)
                use_accel = True
            elif mode == "ismcg" and saw:
                ay_cmd, az_cmd = self.guid.ismcg(self.los.pitch_rate, self.los.yaw_rate, dt)
                use_accel = True
            elif mode == "hold":
                pitch_cmd, yaw_cmd = self._pitch_cmd0, self._yaw_cmd0
            else:
                pitch_cmd, yaw_cmd = self._pitch_cmd0, self._yaw_cmd0
            if phase == "terminal":
                scale = 0.45
        elif phase == "done":
            out = self.mix.apply(0.0, 0.0, 0.0)
            self.tel.phase = phase
            self.tel.mix = out
            return out

        ay_cmd *= scale
        az_cmd *= scale
        if not use_accel:
            pitch_cmd = self._pitch_cmd0 + scale * (pitch_cmd - self._pitch_cmd0)
            yaw_cmd = self._yaw_cmd0 + scale * (yaw_cmd - self._yaw_cmd0)

        dp, dr, dy = self.ap.step(
            self.p.roll_cmd_rad,
            pitch_cmd,
            yaw_cmd,
            ay_cmd,
            az_cmd,
            imu,
            dt,
            use_accel and self.p.autopilot_mode == "three_loop",
        )

        if use_accel and self.p.autopilot_mode == "cascade_pid":
            # 过载指令还没接到三回路时：把过载粗映射成姿态增量
            pitch_cmd = imu.pitch + clamp(ay_cmd * 0.15, -0.25, 0.25)
            yaw_cmd = imu.yaw + clamp(az_cmd * 0.15, -0.25, 0.25)
            dp, dr, dy = self.ap.step(
                self.p.roll_cmd_rad, pitch_cmd, yaw_cmd, 0.0, 0.0, imu, dt, False
            )

        out = self.mix.apply(dp, dr, dy)
        self.tel = FcTelemetry(
            phase=phase,
            ay_cmd=ay_cmd,
            az_cmd=az_cmd,
            alpha_hat=self.ap.alpha_hat,
            beta_hat=self.ap.beta_hat,
            los_pitch=self.los.pitch,
            los_yaw=self.los.yaw,
            los_pitch_rate=self.los.pitch_rate,
            los_yaw_rate=self.los.yaw_rate,
            mix=out,
        )
        return out
