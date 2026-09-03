"""滚转三回路 + 俯仰/偏航伪攻角三回路 + 串级 PID 回退。"""

from __future__ import annotations

from .filters import PseudoAoA
from .params import AccelThreeLoopParams, CascadeParams, CouplingParams, RollThreeLoopParams
from .pid import Pid
from .types import ImuSample, clamp, wrap_pi


class RollThreeLoop:
    def __init__(self, p: RollThreeLoopParams) -> None:
        self.p = p
        self.integ = 0.0

    def reset(self) -> None:
        self.integ = 0.0

    def step(self, cmd: float, imu: ImuSample, dt: float) -> float:
        # 外环比例 K0·e + 中环独立积分 K1·∫e + 内环速率阻尼 −Kg·p
        # 只有积分、没有 K0·e 时，滚转通道特征方程缺 s 项，Routh 不稳定。
        e = wrap_pi(cmd - imu.roll)
        self.integ = clamp(self.integ + e * dt, -self.p.int_lim, self.p.int_lim)
        u = self.p.K0 * e + self.p.K1 * self.integ - self.p.Kg * imu.p
        return self.p.k_act * u


class AccelThreeLoop:
    """西交 2.4.4.3：外环过载积分、中环伪攻角、内环角速率。"""

    def __init__(self, p: AccelThreeLoopParams, alpha_lim: float) -> None:
        self.p = p
        self.integ = 0.0
        self.aoa = PseudoAoA(p.T_alpha, alpha_lim)

    def reset(self) -> None:
        self.integ = 0.0
        self.aoa.reset()

    def step(self, a_cmd_g: float, a_meas_g: float, rate: float, dt: float) -> tuple[float, float]:
        p = self.p
        a_cmd_g = clamp(a_cmd_g, -p.accel_lim_g, p.accel_lim_g)
        e = p.K_DC * a_cmd_g - a_meas_g
        self.integ = clamp(self.integ + p.wi * e * dt, -p.int_lim, p.int_lim)
        alpha_hat = self.aoa.step(rate, dt)
        u = p.KA * self.integ - p.K_alpha * alpha_hat - p.Kg * rate
        return p.k_act * u, alpha_hat


class CascadeAtt:
    def __init__(self, ang: Pid, rate: Pid) -> None:
        self.ang = ang
        self.rate = rate

    def reset(self) -> None:
        self.ang.reset()
        self.rate.reset()

    def step(self, e_ang: float, rate: float, dt: float) -> float:
        rate_cmd = self.ang.step(e_ang, dt)
        return self.rate.step(rate_cmd - rate, dt)


class Autopilot:
    def __init__(
        self,
        roll_p: RollThreeLoopParams,
        pitch_p: AccelThreeLoopParams,
        yaw_p: AccelThreeLoopParams,
        cascade: CascadeParams,
        couple: CouplingParams,
        mode: str,
        g: float,
    ) -> None:
        self.mode = mode
        self.g = g
        self.couple = couple
        self.roll3 = RollThreeLoop(roll_p)
        self.pitch3 = AccelThreeLoop(pitch_p, couple.alpha_lim_rad)
        self.yaw3 = AccelThreeLoop(yaw_p, couple.beta_lim_rad)
        self.roll_pid = CascadeAtt(Pid(cascade.roll_ang), Pid(cascade.roll_rate))
        self.pitch_pid = CascadeAtt(Pid(cascade.pitch_ang), Pid(cascade.pitch_rate))
        self.yaw_pid = CascadeAtt(Pid(cascade.yaw_ang), Pid(cascade.yaw_rate))
        self.alpha_hat = 0.0
        self.beta_hat = 0.0

    def reset(self) -> None:
        self.roll3.reset()
        self.pitch3.reset()
        self.yaw3.reset()
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.alpha_hat = 0.0
        self.beta_hat = 0.0

    def _nz(self, imu: ImuSample) -> float:
        """法向过载（g）。z 向下时抬头机动 az 为负，取 -az/g 使正过载抬头。"""
        return -imu.az / self.g

    def _ny(self, imu: ImuSample) -> float:
        return imu.ay / self.g

    def step(
        self,
        roll_cmd: float,
        pitch_cmd: float,
        yaw_cmd: float,
        ay_cmd_g: float,
        az_cmd_g: float,
        imu: ImuSample,
        dt: float,
        use_accel_loop: bool,
    ) -> tuple[float, float, float]:
        """返回等效舵偏 dp, dr, dy（rad）。

        use_accel_loop: 导引给出过载指令时为 True；pixel_angle/hold 走姿态环。
        """
        if self.mode == "three_loop":
            dr = self.roll3.step(roll_cmd, imu, dt)
            if use_accel_loop:
                dp, self.alpha_hat = self.pitch3.step(ay_cmd_g, self._nz(imu), imu.q, dt)
                dy, self.beta_hat = self.yaw3.step(az_cmd_g, self._ny(imu), -imu.r, dt)
            else:
                # 无过载指令时用伪攻角回路跟踪姿态：把姿态误差当成等效过载需求
                dp, self.alpha_hat = self.pitch3.step(
                    wrap_pi(pitch_cmd - imu.pitch) * 2.0, self._nz(imu), imu.q, dt
                )
                dy, self.beta_hat = self.yaw3.step(
                    wrap_pi(yaw_cmd - imu.yaw) * 2.0, self._ny(imu), -imu.r, dt
                )
        else:
            dr = self.roll_pid.step(wrap_pi(roll_cmd - imu.roll), imu.p, dt)
            dp = self.pitch_pid.step(wrap_pi(pitch_cmd - imu.pitch), imu.q, dt)
            dy = self.yaw_pid.step(wrap_pi(yaw_cmd - imu.yaw), imu.r, dt)
            # 串级模式也跑伪攻角，供限幅与前馈
            self.alpha_hat = self.pitch3.aoa.step(imu.q, dt)
            self.beta_hat = self.yaw3.aoa.step(-imu.r, dt)

        dr += self.couple.roll_ff_alpha_beta * self.alpha_hat * self.beta_hat
        dy += self.couple.yaw_ff_p * imu.p

        # 按伪攻角削俯仰/偏航，防失速（西交外环限幅的原因）
        if abs(self.alpha_hat) > 0.85 * self.couple.alpha_lim_rad and dp * self.alpha_hat > 0:
            dp *= 0.3
        if abs(self.beta_hat) > 0.85 * self.couple.beta_lim_rad and dy * self.beta_hat > 0:
            dy *= 0.3
        return dp, dr, dy
