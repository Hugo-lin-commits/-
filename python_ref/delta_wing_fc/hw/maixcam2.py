"""MaixCAM2 板级：SPI 读 ICM-42688 + 四路舵机 PWM。

只在板上 ``import maix``。PC 上跑 examples 不会加载本文件。
板载 LSM6DSOWTR 不要用，飞镖过载和转速会打满它的默认量程。
"""

from __future__ import annotations

from .icm42688 import Icm42688


class MaixSpiBus:
    def __init__(self, spi_dev) -> None:
        self.spi = spi_dev

    def xfer(self, data: list[int]) -> list[int]:
        raw = bytes(b & 0xFF for b in data)
        # MaixPy SPI.write_read 返回 bytes
        out = self.spi.write_read(raw, len(raw))
        return list(out)


def pwm_us_to_duty(us: float, period_us: float = 20000.0) -> float:
    """脉宽 μs → 占空比 %（50 Hz）。"""
    return max(2.5, min(12.5, 100.0 * us / period_us))


def open_icm42688_spi(spi_id: int = 2, freq: int = 8_000_000):
    from maix import pinmap, spi, err

    pins = {
        "B21": "SPI2_CS1",
        "B19": "SPI2_MISO",
        "B18": "SPI2_MOSI",
        "B20": "SPI2_SCK",
    }
    for pin, fn in pins.items():
        err.check_raise(pinmap.set_pin_function(pin, fn), "spi pinmap")
    dev = spi.SPI(id=spi_id, mode=spi.Mode.MASTER, freq=freq, polarity=0, phase=0, bits=8)
    chip = Icm42688(MaixSpiBus(dev))
    wid = chip.begin()
    return chip, wid


def open_servos(channels: list[tuple[str, int]]):
    """channels: [(pin_name, pwm_id), ...] 必须 4 路，按混控 1~4。"""
    from maix import pinmap, pwm, err

    outs = []
    for pin_name, pwm_id in channels:
        err.check_raise(pinmap.set_pin_function(pin_name, f"PWM{pwm_id}"), "pwm pinmap")
        outs.append(pwm.PWM(pwm_id, freq=50, duty=7.5, enable=True))
    return outs


def write_servos(pwms, pwm_us: tuple[float, float, float, float]) -> None:
    for ch, us in zip(pwms, pwm_us):
        ch.duty(pwm_us_to_duty(us))
