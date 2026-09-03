/**
 * MaixCAM2 板上入口（MaixCDK / C++）。
 *
 * 不要用板载 LSM6DSOWTR。ICM-42688 接 SPI2：
 *   B20=SCK  B18=MOSI  B19=MISO  B21=CS1
 *
 * 若本机 MaixCDK 的 SPI::write_read 签名不同，只改 MaixSpiBus::xfer。
 */

#include "dart_fc.hpp"

#include "maix_basic.hpp"
#include "maix_pinmap.hpp"
#include "maix_pwm.hpp"
#include "maix_spi.hpp"

#include <cstdio>
#include <memory>
#include <vector>

using namespace dart_fc;
using namespace maix;

class MaixSpiBus final : public SpiBus {
public:
    peripheral::spi::SPI spi;
    MaixSpiBus(int id, int freq)
        : spi(id, peripheral::spi::Mode::MASTER, freq, 0, 0, 8) {}

    void xfer(std::uint8_t* data, int n) override {
        std::vector<unsigned char> in(data, data + n);
        std::vector<unsigned char> out = spi.write_read(in, n);
        const int m = static_cast<int>(out.size()) < n ? static_cast<int>(out.size()) : n;
        for (int i = 0; i < m; ++i) data[i] = out[static_cast<size_t>(i)];
    }
};

static float now_s() { return static_cast<float>(time::ticks_ms()) * 0.001f; }

static int _main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    Params p = Params::defaults();
    p.autopilot_mode = AutopilotMode::CascadePid;
    p.guidance_mode = GuidanceMode::PixelAngle;

    err::check_raise(peripheral::pinmap::set_pin_function("B21", "SPI2_CS1"), "spi cs");
    err::check_raise(peripheral::pinmap::set_pin_function("B19", "SPI2_MISO"), "spi miso");
    err::check_raise(peripheral::pinmap::set_pin_function("B18", "SPI2_MOSI"), "spi mosi");
    err::check_raise(peripheral::pinmap::set_pin_function("B20", "SPI2_SCK"), "spi sck");

    MaixSpiBus bus(p.hw.spi_id, p.hw.spi_freq);
    Icm42688 imu_chip(&bus);
    imu_chip.fs_dps = p.hw.gyro_fs_dps;
    imu_chip.fs_g = p.hw.accel_fs_g;
    const std::uint8_t who = imu_chip.begin();
    if (who != kIcmWhoAmIVal) {
        log::error("ICM-42688 WHO_AM_I=0x%02X, expect 0x47", who);
        return 1;
    }
    log::info("ICM-42688 WHO_AM_I=0x%02X", who);

    std::unique_ptr<peripheral::pwm::PWM> servos[4];
    for (int i = 0; i < 4; ++i) {
        char fn[16];
        std::snprintf(fn, sizeof(fn), "PWM%d", p.hw.servo_pwm_id[i]);
        err::check_raise(peripheral::pinmap::set_pin_function(p.hw.servo_pin[i], fn), "pwm");
        servos[i] = std::make_unique<peripheral::pwm::PWM>(p.hw.servo_pwm_id[i], 50, 7.5, true);
    }

    FlightController fc(p);
    DartAhrs ahrs;

    float ax_s = 0, ay_s = 0, az_s = 0;
    int n = 0;
    const float t_cal = now_s() + 1.5f;
    while (now_s() < t_cal) {
        Vec3 acc, gyr;
        imu_chip.read_si(acc, gyr, p.gravity);
        acc = remap_axis(acc, p.hw.imu_chip_idx, p.hw.imu_chip_sign);
        gyr = remap_axis(gyr, p.hw.imu_chip_idx, p.hw.imu_chip_sign);
        ahrs.capture_bias(gyr.x, gyr.y, gyr.z);
        ax_s += acc.x;
        ay_s += acc.y;
        az_s += acc.z;
        ++n;
        time::sleep_ms(2);
    }
    if (n < 1) n = 1;
    ahrs.set_from_accel(ax_s / static_cast<float>(n), ay_s / static_cast<float>(n),
                         az_s / static_cast<float>(n));
    log::info("ahrs init roll=%.2f deg pitch=%.2f deg", rad2deg(ahrs.roll), rad2deg(ahrs.pitch));

    const float t0 = now_s();
    float t_prev = t0;
    VisionSample vis;  // 视觉接入后填 valid/px/py/w/h
    while (!app::need_exit()) {
        const float t = now_s();
        float dt = t - t_prev;
        t_prev = t;
        if (dt <= 0 || dt > 0.05f) dt = p.time.dt_ctrl;

        Vec3 acc, gyr;
        imu_chip.read_si(acc, gyr, p.gravity);
        acc = remap_axis(acc, p.hw.imu_chip_idx, p.hw.imu_chip_sign);
        gyr = remap_axis(gyr, p.hw.imu_chip_idx, p.hw.imu_chip_sign);
        ahrs.step(gyr.x, gyr.y, gyr.z, dt);

        ImuSample imu;
        imu.p = gyr.x;
        imu.q = gyr.y;
        imu.r = gyr.z;
        imu.ax = acc.x;
        imu.ay = acc.y;
        imu.az = acc.z;
        imu.roll = ahrs.roll;
        imu.pitch = ahrs.pitch;
        imu.yaw = ahrs.yaw;

        vis.t = t - t0;
        MixerOut out = fc.step(t - t0, dt, imu, vis);
        for (int i = 0; i < 4; ++i) servos[i]->duty(pwm_us_to_duty(out.pwm_us[i]));

        const int remain_ms = static_cast<int>((p.time.dt_ctrl - (now_s() - t)) * 1000.0f);
        if (remain_ms > 0) time::sleep_ms(remain_ms);
    }
    return 0;
}

int main(int argc, char** argv) {
    sys::register_default_signal_handle();
    CATCH_EXCEPTION_RUN_RETURN(_main, -1, argc, argv);
}
