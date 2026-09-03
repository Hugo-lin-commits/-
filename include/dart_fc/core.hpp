#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace dart_fc {

inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kRad2Deg = 180.0f / kPi;
inline constexpr float kDeg2Rad = kPi / 180.0f;

inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

inline float wrap_pi(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

inline float deg2rad(float d) { return d * kDeg2Rad; }
inline float rad2deg(float r) { return r * kRad2Deg; }

enum class Phase : std::uint8_t {
    Idle,
    Launch,
    Coast,
    Seek,
    Guide,
    Terminal,
    Hold,
    Done
};

inline const char* phase_name(Phase p) {
    switch (p) {
        case Phase::Idle: return "idle";
        case Phase::Launch: return "launch";
        case Phase::Coast: return "coast";
        case Phase::Seek: return "seek";
        case Phase::Guide: return "guide";
        case Phase::Terminal: return "terminal";
        case Phase::Hold: return "hold";
        case Phase::Done: return "done";
        default: return "?";
    }
}

enum class GuidanceMode : std::uint8_t { Hold, PixelAngle, Png, Ismcg };
enum class AutopilotMode : std::uint8_t { CascadePid, ThreeLoop };

struct Vec3 {
    float x = 0, y = 0, z = 0;
};

struct ImuSample {
    float p = 0, q = 0, r = 0;
    float ax = 0, ay = 0, az = 0;
    float roll = 0, pitch = 0, yaw = 0;
};

struct VisionSample {
    bool valid = false;
    float px = 0, py = 0, w = 0, h = 0, t = 0;
    float cx() const { return px + 0.5f * w; }
    float cy() const { return py + 0.5f * h; }
};

struct MixerOut {
    float delta_deg[4] = {0, 0, 0, 0};
    float pwm_us[4] = {1500, 1500, 1500, 1500};
    float dp = 0, dr = 0, dy = 0;
    bool saturated = false;
};

struct FcTelemetry {
    Phase phase = Phase::Idle;
    float ay_cmd = 0, az_cmd = 0;
    float alpha_hat = 0, beta_hat = 0;
    float los_pitch = 0, los_yaw = 0;
    float los_pitch_rate = 0, los_yaw_rate = 0;
    MixerOut mix;
};

struct HardwareParams {
    // 后视 1=右上 2=左上 3=左下 4=右下。混控在 control.hpp: δ1=δp+δr+δy
    float servo_sign[4] = {1, 1, 1, 1};
    float servo_trim_deg[4] = {0, 0, 0, 0};
    float delta_max_deg = 18.0f;
    float pwm_us_center = 1500.0f;
    float pwm_us_per_deg = 11.1f;

    float imu_gyro_sign[3] = {1, 1, 1};
    float imu_accel_sign[3] = {1, 1, 1};
    float gyro_fs_dps = 2000.0f;
    float accel_fs_g = 16.0f;
    int imu_chip_idx[3] = {1, 0, 2};
    float imu_chip_sign[3] = {1, 1, -1};

    float cam_width = 640, cam_height = 360;
    float cam_fx = 375, cam_fy = 378, cam_cx = 320, cam_cy = 180;
    float los_yaw_sign = -1, los_pitch_sign = 1;

    int spi_id = 2;
    int spi_freq = 8000000;
    const char* servo_pin[4] = {"A31", "A30", "A29", "A28"};
    int servo_pwm_id[4] = {7, 6, 5, 4};
};

struct AeroFrozen {
    float a_omega = 0.20f, a_alpha = 0.10f, a_delta = 5.00f;
    float b_alpha = 0.30f, b_delta = 0.06f;
    float d_omega = 0.05f, d_delta = 10.0f;
    float v_ref = 12.0f;
};

struct CascadePidGains {
    float kp, ki, kd, i_lim, out_lim;
};

struct CascadeParams {
    CascadePidGains roll_ang{4.0f, 0.4f, 0.05f, 0.4f, 2.5f};
    CascadePidGains roll_rate{0.35f, 0.0f, 0.0f, 0.2f, 0.35f};
    CascadePidGains pitch_ang{3.5f, 0.25f, 0.04f, 0.3f, 1.5f};
    CascadePidGains pitch_rate{0.28f, 0.0f, 0.0f, 0.15f, 0.30f};
    CascadePidGains yaw_ang{3.5f, 0.25f, 0.04f, 0.3f, 1.5f};
    CascadePidGains yaw_rate{0.28f, 0.0f, 0.0f, 0.15f, 0.30f};
};

struct RollThreeLoopParams {
    float K0 = 2.5f, K1 = 3.5f, Kg = 0.55f, int_lim = 0.25f, k_act = 1.0f;
};

struct AccelThreeLoopParams {
    float Kg = 0.90f, K_alpha = 0.12f, wi = 2.2f, KA = 1.0f, K_DC = 1.0f;
    float k_act = 1.0f, T_alpha = 0.18f, int_lim = 0.6f, accel_lim_g = 1.2f;
};

struct GuidanceParams {
    float N = 4.0f, vc_fallback = 12.0f, los_lpf_hz = 8.0f, pixel_angle_kp = 1.0f;
    float r0 = 18.0f, v0 = 12.0f, r0_err = 2.0f, v0_err = 2.0f;
    float ismcg_N = 4.0f, ismcg_gamma = 8.0f, ismcg_k_max = 6.0f;
};

struct FlightTiming {
    float dt_ctrl = 0.005f;
    float launch_acc_g = 8.0f;
    float t_launch_hold = 0.08f;
    float t_coast = 0.35f;
    float t_seek_max = 1.20f;
    float t_guide_min = 0.15f;
    float t_terminal = 0.12f;
    float t_total_max = 2.40f;
    float vision_stale_s = 0.08f;
};

struct CouplingParams {
    float roll_ff_alpha_beta = 0.0f;
    float yaw_ff_p = 0.0f;
    float alpha_lim_rad = 0.35f;
    float beta_lim_rad = 0.35f;
};

struct Params {
    HardwareParams hw;
    AeroFrozen aero;
    CascadeParams cascade;
    RollThreeLoopParams roll;
    AccelThreeLoopParams pitch;
    AccelThreeLoopParams yaw;
    GuidanceParams guid;
    FlightTiming time;
    CouplingParams couple;
    GuidanceMode guidance_mode = GuidanceMode::Png;
    AutopilotMode autopilot_mode = AutopilotMode::CascadePid;
    float roll_cmd_rad = 0.0f;
    float gravity = 9.81f;

    static Params defaults() {
        Params p;
        p.yaw = p.pitch;
        return p;
    }
};

inline Vec3 remap_axis(const Vec3& v, const int idx[3], const float sign[3]) {
    const float c[3] = {v.x, v.y, v.z};
    return {sign[0] * c[idx[0]], sign[1] * c[idx[1]], sign[2] * c[idx[2]]};
}

}  // namespace dart_fc
