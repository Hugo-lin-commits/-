"""TDK ICM-42688(-P) 寄存器与换算。SPI 模式 0，读地址最高位置 1。"""

from __future__ import annotations

from math import pi

# Bank 0
REG_ACCEL_DATA_X1 = 0x1F
REG_GYRO_DATA_X1 = 0x25
REG_PWR_MGMT0 = 0x4E
REG_GYRO_CONFIG0 = 0x4F
REG_ACCEL_CONFIG0 = 0x50
REG_WHO_AM_I = 0x75
REG_BANK_SEL = 0x76

WHO_AM_I_42688P = 0x47

# GYRO_CONFIG0: FS[7:5] 000=±2000 dps；ODR[3:0] 0110=1 kHz
GYRO_FS_2000 = 0x00
GYRO_ODR_1KHZ = 0x06
# ACCEL_CONFIG0: FS[7:5] 000=±16 g；ODR 1 kHz
ACCEL_FS_16G = 0x00
ACCEL_ODR_1KHZ = 0x06

# 16-bit：满量程对应 ±32768
LSB_PER_DPS_2000 = 32768.0 / 2000.0  # 16.4
LSB_PER_G_16 = 32768.0 / 16.0  # 2048


def gyro_raw_to_rad_s(raw: int, fs_dps: float = 2000.0) -> float:
    return (raw / (32768.0 / fs_dps)) * pi / 180.0


def accel_raw_to_mps2(raw: int, fs_g: float = 16.0, g: float = 9.81) -> float:
    return (raw / (32768.0 / fs_g)) * g


def be16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 65536
    return v


class Icm42688:
    """通过 ``bus.xfer(list[int]) -> list[int]`` 读写。MaixCAM2 用 SPI2。"""

    def __init__(self, bus, fs_dps: float = 2000.0, fs_g: float = 16.0) -> None:
        self.bus = bus
        self.fs_dps = fs_dps
        self.fs_g = fs_g

    def _read(self, reg: int, n: int) -> list[int]:
        out = self.bus.xfer([reg | 0x80] + [0x00] * n)
        return list(out[1:])

    def _write(self, reg: int, val: int) -> None:
        self.bus.xfer([reg & 0x7F, val & 0xFF])

    def whoami(self) -> int:
        return self._read(REG_WHO_AM_I, 1)[0]

    def begin(self) -> int:
        self._write(REG_BANK_SEL, 0)
        wid = self.whoami()
        # 陀螺/加计低噪声模式
        self._write(REG_PWR_MGMT0, 0x0F)
        self._write(REG_GYRO_CONFIG0, GYRO_FS_2000 | GYRO_ODR_1KHZ)
        self._write(REG_ACCEL_CONFIG0, ACCEL_FS_16G | ACCEL_ODR_1KHZ)
        return wid

    def read_raw(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        a = self._read(REG_ACCEL_DATA_X1, 6)
        g = self._read(REG_GYRO_DATA_X1, 6)
        acc = (be16(a[0], a[1]), be16(a[2], a[3]), be16(a[4], a[5]))
        gyr = (be16(g[0], g[1]), be16(g[2], g[3]), be16(g[4], g[5]))
        return acc, gyr

    def read_si(self, g: float = 9.81) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        acc_r, gyr_r = self.read_raw()
        acc = tuple(accel_raw_to_mps2(v, self.fs_g, g) for v in acc_r)
        gyr = tuple(gyro_raw_to_rad_s(v, self.fs_dps) for v in gyr_r)
        return acc, gyr  # type: ignore[return-value]
