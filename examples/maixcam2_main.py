"""MaixCAM2 板上入口：ICM-42688 + 飞控 + 四舵 PWM。

在 MaixVision 里打开本文件运行。PC 上没有 maix 库，不要当桌面脚本跑。
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from delta_wing_fc import FlightController, ImuSample, VisionSample, default_params
from delta_wing_fc.hw.ahrs import DartAhrs
from delta_wing_fc.hw.axis_remap import remap
from delta_wing_fc.hw.icm42688 import WHO_AM_I_42688P
from delta_wing_fc.hw.maixcam2 import open_icm42688_spi, open_servos, write_servos


def to_body(hw, acc, gyr):
    a = remap(acc, hw.imu_chip_idx, hw.imu_chip_sign)
    w = remap(gyr, hw.imu_chip_idx, hw.imu_chip_sign)
    return a, w


def main() -> None:
    p = default_params()
    p.autopilot_mode = "cascade_pid"
    p.guidance_mode = "pixel_angle"
    hw = p.hw

    chip, wid = open_icm42688_spi(hw.spi_id, hw.spi_freq)
    if wid != WHO_AM_I_42688P:
        raise RuntimeError("ICM-42688 WHO_AM_I=0x%02X, expected 0x47" % wid)
    servos = open_servos(list(hw.servo_pwm))
    fc = FlightController(p)
    ahrs = DartAhrs()

    # 静止采零偏 + 用重力定初始俯仰/滚转
    t_end = time.time() + 1.5
    n = 0
    ax_s = ay_s = az_s = 0.0
    while time.time() < t_end:
        acc, gyr = chip.read_si(p.gravity)
        acc, gyr = to_body(hw, acc, gyr)
        ahrs.capture_bias(gyr[0], gyr[1], gyr[2])
        ax_s += acc[0]
        ay_s += acc[1]
        az_s += acc[2]
        n += 1
        time.sleep(0.002)
    ahrs.set_from_accel(ax_s / n, ay_s / n, az_s / n)

    vis = VisionSample()
    t0 = time.time()
    t_prev = t0
    while True:
        now = time.time()
        dt = now - t_prev
        t_prev = now
        if dt <= 0.0 or dt > 0.05:
            dt = p.time.dt_ctrl

        acc, gyr = chip.read_si(p.gravity)
        acc, gyr = to_body(hw, acc, gyr)
        ahrs.step(gyr[0], gyr[1], gyr[2], dt)

        imu = ImuSample(
            p=gyr[0],
            q=gyr[1],
            r=gyr[2],
            ax=acc[0],
            ay=acc[1],
            az=acc[2],
            roll=ahrs.roll,
            pitch=ahrs.pitch,
            yaw=ahrs.yaw,
        )
        # 视觉：在此接入 MaixPy 找灯，填 vis.px/py/w/h/valid
        out = fc.step(now - t0, dt, imu, vis)
        write_servos(servos, out.pwm_us)

        # 飞控 200 Hz
        remain = p.time.dt_ctrl - (time.time() - now)
        if remain > 0:
            time.sleep(remain)


if __name__ == "__main__":
    main()
