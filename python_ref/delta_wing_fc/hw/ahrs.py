"""飞镖用姿态：导轨上用加速度定初值，发射后 2 秒只积分陀螺。

高过载段加速度计不能当重力计；ICM-42688 零偏小，2 秒积分够用。
"""

from __future__ import annotations

from math import atan2, sqrt

from ..types import wrap_pi


class DartAhrs:
    def __init__(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.bp = 0.0
        self.bq = 0.0
        self.br = 0.0
        self._bias_n = 0
        self.in_flight = False

    def reset(self) -> None:
        self.roll = self.pitch = self.yaw = 0.0
        self.bp = self.bq = self.br = 0.0
        self._bias_n = 0
        self.in_flight = False

    def capture_bias(self, p: float, q: float, r: float) -> None:
        n = self._bias_n + 1
        k = 1.0 / n
        self.bp += (p - self.bp) * k
        self.bq += (q - self.bq) * k
        self.br += (r - self.br) * k
        self._bias_n = n

    def set_from_accel(self, ax: float, ay: float, az: float) -> None:
        # 弹体 z 下：静止 az≈−g。roll=atan2(ay,az)，pitch=atan2(−ax, |g|)
        self.roll = atan2(ay, az)
        self.pitch = atan2(-ax, sqrt(ay * ay + az * az) + 1e-9)
        self.yaw = 0.0

    def step(self, p: float, q: float, r: float, dt: float) -> None:
        p -= self.bp
        q -= self.bq
        r -= self.br
        self.roll = wrap_pi(self.roll + p * dt)
        self.pitch = wrap_pi(self.pitch + q * dt)
        self.yaw = wrap_pi(self.yaw + r * dt)
