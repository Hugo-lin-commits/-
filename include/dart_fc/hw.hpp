#pragma once

#include "core.hpp"
#include <cstdint>
#include <vector>

namespace dart_fc {

inline constexpr std::uint8_t kIcmWhoAmIReg = 0x75;
inline constexpr std::uint8_t kIcmWhoAmIVal = 0x47;
inline constexpr std::uint8_t kIcmAccelDataX1 = 0x1F;
inline constexpr std::uint8_t kIcmGyroDataX1 = 0x25;
inline constexpr std::uint8_t kIcmPwrMgmt0 = 0x4E;
inline constexpr std::uint8_t kIcmGyroConfig0 = 0x4F;
inline constexpr std::uint8_t kIcmAccelConfig0 = 0x50;
inline constexpr std::uint8_t kIcmBankSel = 0x76;

inline int be16(std::uint8_t hi, std::uint8_t lo) {
    int v = (static_cast<int>(hi) << 8) | lo;
    if (v & 0x8000) v -= 65536;
    return v;
}

inline float gyro_raw_to_rad_s(int raw, float fs_dps = 2000.0f) {
    return (raw / (32768.0f / fs_dps)) * kDeg2Rad;
}

inline float accel_raw_to_mps2(int raw, float fs_g = 16.0f, float g = 9.81f) {
    return (raw / (32768.0f / fs_g)) * g;
}

struct SpiBus {
    virtual ~SpiBus() = default;
    virtual void xfer(std::uint8_t* data, int n) = 0;
};

class Icm42688 {
public:
    SpiBus* bus = nullptr;
    float fs_dps = 2000, fs_g = 16;
    explicit Icm42688(SpiBus* b = nullptr) : bus(b) {}

    void write_reg(std::uint8_t reg, std::uint8_t val) {
        std::uint8_t buf[2] = {static_cast<std::uint8_t>(reg & 0x7F), val};
        bus->xfer(buf, 2);
    }
    void read_reg(std::uint8_t reg, std::uint8_t* dst, int n) {
        std::vector<std::uint8_t> buf(static_cast<size_t>(n) + 1);
        buf[0] = static_cast<std::uint8_t>(reg | 0x80);
        bus->xfer(buf.data(), n + 1);
        std::memcpy(dst, buf.data() + 1, static_cast<size_t>(n));
    }
    std::uint8_t whoami() {
        std::uint8_t v = 0;
        read_reg(kIcmWhoAmIReg, &v, 1);
        return v;
    }
    std::uint8_t begin() {
        write_reg(kIcmBankSel, 0);
        const std::uint8_t id = whoami();
        write_reg(kIcmPwrMgmt0, 0x0F);
        write_reg(kIcmGyroConfig0, 0x06);    // ±2000 dps, 1 kHz
        write_reg(kIcmAccelConfig0, 0x06);  // ±16 g, 1 kHz
        return id;
    }
    void read_si(Vec3& acc, Vec3& gyr, float g = 9.81f) {
        std::uint8_t a[6], w[6];
        read_reg(kIcmAccelDataX1, a, 6);
        read_reg(kIcmGyroDataX1, w, 6);
        acc.x = accel_raw_to_mps2(be16(a[0], a[1]), fs_g, g);
        acc.y = accel_raw_to_mps2(be16(a[2], a[3]), fs_g, g);
        acc.z = accel_raw_to_mps2(be16(a[4], a[5]), fs_g, g);
        gyr.x = gyro_raw_to_rad_s(be16(w[0], w[1]), fs_dps);
        gyr.y = gyro_raw_to_rad_s(be16(w[2], w[3]), fs_dps);
        gyr.z = gyro_raw_to_rad_s(be16(w[4], w[5]), fs_dps);
    }
};

class DartAhrs {
public:
    float roll = 0, pitch = 0, yaw = 0;
    float bp = 0, bq = 0, br = 0;
    int bias_n = 0;
    void reset() {
        roll = pitch = yaw = bp = bq = br = 0;
        bias_n = 0;
    }
    void capture_bias(float p, float q, float r) {
        const int n = bias_n + 1;
        const float k = 1.0f / static_cast<float>(n);
        bp += (p - bp) * k;
        bq += (q - bq) * k;
        br += (r - br) * k;
        bias_n = n;
    }
    void set_from_accel(float ax, float ay, float az) {
        roll = std::atan2(ay, az);
        pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az) + 1e-9f);
        yaw = 0;
    }
    void step(float p, float q, float r, float dt) {
        p -= bp;
        q -= bq;
        r -= br;
        roll = wrap_pi(roll + p * dt);
        pitch = wrap_pi(pitch + q * dt);
        yaw = wrap_pi(yaw + r * dt);
    }
};

inline float pwm_us_to_duty(float us, float period_us = 20000.0f) {
    return clampf(100.0f * us / period_us, 2.5f, 12.5f);
}

}  // namespace dart_fc
