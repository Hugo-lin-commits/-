#pragma once

#include "core.hpp"

namespace dart_fc {

class LowPass {
public:
    float hz = 8, y = 0;
    void reset(float x0 = 0) { y = x0; }
    float step(float x, float dt) {
        if (dt <= 0 || hz <= 0) {
            y = x;
            return y;
        }
        const float a = 2.0f * kPi * hz * dt;
        const float k = a / (1.0f + a);
        y += k * (x - y);
        return y;
    }
};

class DifferentiatorLpf {
public:
    LowPass lpf;
    float prev = 0;
    bool has = false;
    explicit DifferentiatorLpf(float hz) { lpf.hz = hz; }
    void reset() {
        lpf.reset(0);
        prev = 0;
        has = false;
    }
    float step(float x, float dt) {
        if (!has || dt <= 0) {
            prev = x;
            has = true;
            lpf.reset(0);
            return 0;
        }
        const float raw = (x - prev) / dt;
        prev = x;
        return lpf.step(raw, dt);
    }
};

class PseudoAoA {
public:
    float T = 0.18f, lim = 0.35f, x = 0;
    void reset(float x0 = 0) { x = x0; }
    float step(float rate, float dt) {
        if (T <= 1e-4f)
            x = rate;
        else
            x += (rate - x / T) * dt;
        x = clampf(x, -lim, lim);
        return x;
    }
};

class Pid {
public:
    CascadePidGains g{};
    float i = 0, prev_e = 0;
    bool has = false;
    explicit Pid(const CascadePidGains& gg) : g(gg) {}
    void reset() {
        i = 0;
        prev_e = 0;
        has = false;
    }
    float step(float e, float dt) {
        float d = 0;
        if (has && dt > 0) d = (e - prev_e) / dt;
        has = true;
        prev_e = e;
        i = clampf(i + g.ki * e * dt, -g.i_lim, g.i_lim);
        return clampf(g.kp * e + i + g.kd * d, -g.out_lim, g.out_lim);
    }
};

inline void mix_from_axes(float dp, float dr, float dy, float out[4]) {
    out[0] = dp + dr + dy;
    out[1] = dp - dr - dy;
    out[2] = dp - dr + dy;
    out[3] = dp + dr - dy;
}

inline float mix_peak(float dp, float dr, float dy) {
    float v[4];
    mix_from_axes(dp, dr, dy, v);
    float m = 0;
    for (int i = 0; i < 4; ++i) m = std::max(m, std::fabs(v[i]));
    return m;
}

inline void allocate(float& dp, float& dr, float& dy, float sat, float raw[4], bool& sat_flag) {
    sat = std::fabs(sat);
    if (mix_peak(dp, dr, dy) <= sat + 1e-9f) {
        mix_from_axes(dp, dr, dy, raw);
        sat_flag = false;
        return;
    }
    sat_flag = true;
    float lo = 0, hi = 1;
    for (int i = 0; i < 16; ++i) {
        const float m = 0.5f * (lo + hi);
        if (mix_peak(dp, dr, m * dy) <= sat)
            lo = m;
        else
            hi = m;
    }
    dy *= lo;
    if (mix_peak(dp, dr, dy) <= sat + 1e-9f) {
        mix_from_axes(dp, dr, dy, raw);
        return;
    }
    lo = 0;
    hi = 1;
    for (int i = 0; i < 16; ++i) {
        const float m = 0.5f * (lo + hi);
        if (mix_peak(m * dp, dr, dy) <= sat)
            lo = m;
        else
            hi = m;
    }
    dp *= lo;
    if (mix_peak(dp, dr, dy) <= sat + 1e-9f) {
        mix_from_axes(dp, dr, dy, raw);
        return;
    }
    const float pk = mix_peak(dp, dr, dy);
    if (pk > 1e-9f) {
        const float s = sat / pk;
        dp *= s;
        dr *= s;
        dy *= s;
    }
    mix_from_axes(dp, dr, dy, raw);
}

class XMixer {
public:
    HardwareParams hw;
    explicit XMixer(const HardwareParams& h) : hw(h) {}
    MixerOut apply(float dp, float dr, float dy) const {
        MixerOut o;
        float raw[4];
        const float sat = std::fabs(hw.delta_max_deg) * kDeg2Rad;
        allocate(dp, dr, dy, sat, raw, o.saturated);
        o.dp = dp;
        o.dr = dr;
        o.dy = dy;
        for (int i = 0; i < 4; ++i) {
            float di = rad2deg(raw[i]) * hw.servo_sign[i] + hw.servo_trim_deg[i];
            di = clampf(di, -hw.delta_max_deg, hw.delta_max_deg);
            o.delta_deg[i] = di;
            o.pwm_us[i] = hw.pwm_us_center + di * hw.pwm_us_per_deg;
        }
        return o;
    }
};

class LineOfSight {
public:
    HardwareParams hw;
    DifferentiatorLpf dp, dy;
    float pitch = 0, yaw = 0, pitch_rate = 0, yaw_rate = 0;
    LineOfSight(const HardwareParams& h, float lpf_hz) : hw(h), dp(lpf_hz), dy(lpf_hz) {}
    void reset() {
        dp.reset();
        dy.reset();
        pitch = yaw = pitch_rate = yaw_rate = 0;
    }
    bool step(const VisionSample& vis, float dt) {
        if (!vis.valid) return false;
        const float ey = std::atan((vis.cy() - hw.cam_cy) / std::max(hw.cam_fy, 1.0f));
        const float ex = std::atan((vis.cx() - hw.cam_cx) / std::max(hw.cam_fx, 1.0f));
        pitch = hw.los_pitch_sign * ey;
        yaw = hw.los_yaw_sign * ex;
        pitch_rate = dp.step(pitch, dt);
        yaw_rate = dy.step(yaw, dt);
        return true;
    }
};

class RangeObserver {
public:
    GuidanceParams p;
    float r = 18, v = -12, r_hi = 20, r_lo = 16, v_hi = -10, v_lo = -14;
    explicit RangeObserver(const GuidanceParams& gp) : p(gp) { reset(); }
    void reset() {
        r = p.r0;
        v = -std::fabs(p.v0);
        r_hi = p.r0 + p.r0_err;
        r_lo = std::max(0.5f, p.r0 - p.r0_err);
        v_hi = -std::fabs(p.v0) + p.v0_err;
        v_lo = -std::fabs(p.v0) - p.v0_err;
    }
    void step(float qdot, float dt) {
        const float q2 = qdot * qdot;
        r += v * dt;
        v += r * q2 * dt;
        r_hi += v_hi * dt;
        r_lo += v_lo * dt;
        v_hi += r_hi * q2 * dt;
        v_lo += r_lo * q2 * dt;
        r = std::max(0.3f, r);
        r_hi = std::max(0.3f, r_hi);
        r_lo = std::max(0.3f, r_lo);
    }
    float vc() const { return std::fabs(v); }
    float dv() const { return 0.5f * std::fabs(v_hi - v_lo); }
};

class Guidance {
public:
    GuidanceParams p;
    GuidanceMode mode;
    RangeObserver obs;
    float k_hat = 0;
    Guidance(const GuidanceParams& gp, GuidanceMode m) : p(gp), mode(m), obs(gp) {}
    void reset() {
        obs.reset();
        k_hat = 0;
    }
    void pixel_angle(float los_p, float los_y, float& dp, float& dy) const {
        dp = p.pixel_angle_kp * los_p;
        dy = p.pixel_angle_kp * los_y;
    }
    void png(float qdot_p, float qdot_y, float dt, float& ay, float& az) {
        const float qdot = std::sqrt(qdot_p * qdot_p + qdot_y * qdot_y);
        obs.step(qdot, dt);
        const float vc = obs.vc() > 1.0f ? obs.vc() : p.vc_fallback;
        const float n = p.N * vc;
        ay = n * qdot_p / 9.81f;
        az = n * qdot_y / 9.81f;
    }
    void ismcg(float qdot_p, float qdot_y, float dt, float& ay, float& az) {
        const float qdot = std::sqrt(qdot_p * qdot_p + qdot_y * qdot_y);
        obs.step(qdot, dt);
        const float rlo = std::max(obs.r_lo, 0.3f);
        const float vabs = std::fabs(obs.v);
        const float ratio = obs.r_hi / rlo;
        const float kdot = (1.0f / std::max(p.ismcg_gamma, 1e-3f)) * obs.r * vabs * qdot * qdot;
        k_hat = clampf(k_hat + kdot * dt, 0.0f, p.ismcg_k_max);
        float gain = (p.ismcg_N - ratio + k_hat) * vabs + 2.0f * obs.dv();
        if (gain < 0) gain = 0;
        ay = gain * qdot_p / 9.81f;
        az = gain * qdot_y / 9.81f;
    }
};

class FlightFsm {
public:
    FlightTiming t;
    Phase phase = Phase::Idle;
    float t0 = 0, t_now = 0;
    explicit FlightFsm(const FlightTiming& tt) : t(tt) {}
    void reset() {
        phase = Phase::Idle;
        t0 = t_now = 0;
    }
    float elapsed() const { return t_now - t0; }
    Phase step(float now, const ImuSample& imu, const VisionSample& vis, float vis_age) {
        t_now = now;
        const float acc = std::sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
        const float launch_g = acc / 9.81f;
        if (phase == Phase::Idle) {
            if (launch_g >= t.launch_acc_g) {
                phase = Phase::Launch;
                t0 = now;
            }
        } else if (phase == Phase::Launch) {
            if (elapsed() >= t.t_launch_hold) phase = Phase::Coast;
        } else if (phase == Phase::Coast) {
            if (vis.valid && vis_age < t.vision_stale_s && elapsed() >= t.t_coast) {
                phase = Phase::Guide;
                t0 = now;
            } else if (elapsed() >= t.t_coast) {
                phase = Phase::Seek;
            }
        } else if (phase == Phase::Seek) {
            if (vis.valid && vis_age < t.vision_stale_s) {
                phase = Phase::Guide;
                t0 = now;
            } else if (elapsed() >= t.t_seek_max) {
                phase = Phase::Hold;
            }
        } else if (phase == Phase::Guide) {
            if (elapsed() >= t.t_terminal && elapsed() >= t.t_guide_min) {
                if (vis_age > t.vision_stale_s) phase = Phase::Terminal;
            }
            if (vis_age > 3.0f * t.vision_stale_s && elapsed() > t.t_guide_min) phase = Phase::Hold;
        }
        if (now > t.t_total_max && phase != Phase::Idle) phase = Phase::Done;
        return phase;
    }
};

class RollThreeLoop {
public:
    RollThreeLoopParams p;
    float integ = 0;
    explicit RollThreeLoop(const RollThreeLoopParams& pp) : p(pp) {}
    void reset() { integ = 0; }
    float step(float cmd, const ImuSample& imu, float dt) {
        const float e = wrap_pi(cmd - imu.roll);
        integ = clampf(integ + e * dt, -p.int_lim, p.int_lim);
        return p.k_act * (p.K0 * e + p.K1 * integ - p.Kg * imu.p);
    }
};

class AccelThreeLoop {
public:
    AccelThreeLoopParams p;
    float integ = 0;
    PseudoAoA aoa;
    AccelThreeLoop(const AccelThreeLoopParams& pp, float alpha_lim) : p(pp) {
        aoa.T = pp.T_alpha;
        aoa.lim = alpha_lim;
    }
    void reset() {
        integ = 0;
        aoa.reset();
    }
    float step(float a_cmd_g, float a_meas_g, float rate, float dt, float& alpha_hat) {
        a_cmd_g = clampf(a_cmd_g, -p.accel_lim_g, p.accel_lim_g);
        const float e = p.K_DC * a_cmd_g - a_meas_g;
        integ = clampf(integ + p.wi * e * dt, -p.int_lim, p.int_lim);
        alpha_hat = aoa.step(rate, dt);
        return p.k_act * (p.KA * integ - p.K_alpha * alpha_hat - p.Kg * rate);
    }
};

class CascadeAtt {
public:
    Pid ang, rate;
    CascadeAtt(const CascadePidGains& a, const CascadePidGains& r) : ang(a), rate(r) {}
    void reset() {
        ang.reset();
        rate.reset();
    }
    float step(float e_ang, float r, float dt) {
        const float rate_cmd = ang.step(e_ang, dt);
        return rate.step(rate_cmd - r, dt);
    }
};

class Autopilot {
public:
    AutopilotMode mode;
    float g;
    CouplingParams couple;
    RollThreeLoop roll3;
    AccelThreeLoop pitch3, yaw3;
    CascadeAtt roll_pid, pitch_pid, yaw_pid;
    float alpha_hat = 0, beta_hat = 0;

    Autopilot(const RollThreeLoopParams& rp, const AccelThreeLoopParams& pp, const AccelThreeLoopParams& yp,
              const CascadeParams& c, const CouplingParams& cp, AutopilotMode m, float gg)
        : mode(m),
          g(gg),
          couple(cp),
          roll3(rp),
          pitch3(pp, cp.alpha_lim_rad),
          yaw3(yp, cp.beta_lim_rad),
          roll_pid(c.roll_ang, c.roll_rate),
          pitch_pid(c.pitch_ang, c.pitch_rate),
          yaw_pid(c.yaw_ang, c.yaw_rate) {}

    void reset() {
        roll3.reset();
        pitch3.reset();
        yaw3.reset();
        roll_pid.reset();
        pitch_pid.reset();
        yaw_pid.reset();
        alpha_hat = beta_hat = 0;
    }

    float nz(const ImuSample& imu) const { return -imu.az / g; }
    float ny(const ImuSample& imu) const { return imu.ay / g; }

    void step(float roll_cmd, float pitch_cmd, float yaw_cmd, float ay_cmd_g, float az_cmd_g, const ImuSample& imu,
              float dt, bool use_accel_loop, float& dp, float& dr, float& dy) {
        if (mode == AutopilotMode::ThreeLoop) {
            dr = roll3.step(roll_cmd, imu, dt);
            if (use_accel_loop) {
                dp = pitch3.step(ay_cmd_g, nz(imu), imu.q, dt, alpha_hat);
                dy = yaw3.step(az_cmd_g, ny(imu), -imu.r, dt, beta_hat);
            } else {
                dp = pitch3.step(wrap_pi(pitch_cmd - imu.pitch) * 2.0f, nz(imu), imu.q, dt, alpha_hat);
                dy = yaw3.step(wrap_pi(yaw_cmd - imu.yaw) * 2.0f, ny(imu), -imu.r, dt, beta_hat);
            }
        } else {
            dr = roll_pid.step(wrap_pi(roll_cmd - imu.roll), imu.p, dt);
            dp = pitch_pid.step(wrap_pi(pitch_cmd - imu.pitch), imu.q, dt);
            dy = yaw_pid.step(wrap_pi(yaw_cmd - imu.yaw), imu.r, dt);
            alpha_hat = pitch3.aoa.step(imu.q, dt);
            beta_hat = yaw3.aoa.step(-imu.r, dt);
        }
        dr += couple.roll_ff_alpha_beta * alpha_hat * beta_hat;
        dy += couple.yaw_ff_p * imu.p;
        if (std::fabs(alpha_hat) > 0.85f * couple.alpha_lim_rad && dp * alpha_hat > 0) dp *= 0.3f;
        if (std::fabs(beta_hat) > 0.85f * couple.beta_lim_rad && dy * beta_hat > 0) dy *= 0.3f;
    }
};

class FlightController {
public:
    Params p;
    LineOfSight los;
    Guidance guid;
    Autopilot ap;
    XMixer mix;
    FlightFsm fsm;
    FcTelemetry tel;
    float last_vis_t = -1e9f;
    float pitch_cmd0 = 0, yaw_cmd0 = 0;
    bool armed_att = false;

    explicit FlightController(Params pp = Params::defaults())
        : p(pp),
          los(pp.hw, pp.guid.los_lpf_hz),
          guid(pp.guid, pp.guidance_mode),
          ap(pp.roll, pp.pitch, pp.yaw, pp.cascade, pp.couple, pp.autopilot_mode, pp.gravity),
          mix(pp.hw),
          fsm(pp.time) {}

    void reset() {
        los.reset();
        guid.reset();
        ap.reset();
        fsm.reset();
        tel = {};
        last_vis_t = -1e9f;
        armed_att = false;
    }

    ImuSample apply_imu_sign(ImuSample imu) const {
        imu.p *= p.hw.imu_gyro_sign[0];
        imu.q *= p.hw.imu_gyro_sign[1];
        imu.r *= p.hw.imu_gyro_sign[2];
        imu.ax *= p.hw.imu_accel_sign[0];
        imu.ay *= p.hw.imu_accel_sign[1];
        imu.az *= p.hw.imu_accel_sign[2];
        return imu;
    }

    MixerOut step(float t, float dt, ImuSample imu, const VisionSample& vis) {
        imu = apply_imu_sign(imu);
        if (vis.valid) last_vis_t = vis.t > 0 ? vis.t : t;
        const float vis_age = t - last_vis_t;
        const Phase phase = fsm.step(t, imu, vis, vis_age);
        if (!armed_att && phase != Phase::Idle) {
            pitch_cmd0 = imu.pitch;
            yaw_cmd0 = imu.yaw;
            armed_att = true;
        }
        const bool saw = vis.valid ? los.step(vis, dt) : false;
        float pitch_cmd = pitch_cmd0;
        float yaw_cmd = yaw_cmd0;
        float ay_cmd = 0, az_cmd = 0;
        bool use_accel = false;
        float scale = 1.0f;

        if (phase == Phase::Idle || phase == Phase::Launch) {
            pitch_cmd = imu.pitch;
            yaw_cmd = imu.yaw;
        } else if (phase == Phase::Coast || phase == Phase::Seek || phase == Phase::Hold) {
            pitch_cmd = pitch_cmd0;
            yaw_cmd = yaw_cmd0;
        } else if (phase == Phase::Guide || phase == Phase::Terminal) {
            if (p.guidance_mode == GuidanceMode::PixelAngle && saw) {
                float dpp, dyy;
                guid.pixel_angle(los.pitch, los.yaw, dpp, dyy);
                pitch_cmd = imu.pitch + dpp;
                yaw_cmd = imu.yaw + dyy;
            } else if (p.guidance_mode == GuidanceMode::Png && saw) {
                guid.png(los.pitch_rate, los.yaw_rate, dt, ay_cmd, az_cmd);
                use_accel = true;
            } else if (p.guidance_mode == GuidanceMode::Ismcg && saw) {
                guid.ismcg(los.pitch_rate, los.yaw_rate, dt, ay_cmd, az_cmd);
                use_accel = true;
            } else {
                pitch_cmd = pitch_cmd0;
                yaw_cmd = yaw_cmd0;
            }
            if (phase == Phase::Terminal) scale = 0.45f;
        } else if (phase == Phase::Done) {
            tel.phase = phase;
            tel.mix = mix.apply(0, 0, 0);
            return tel.mix;
        }

        ay_cmd *= scale;
        az_cmd *= scale;
        if (!use_accel) {
            pitch_cmd = pitch_cmd0 + scale * (pitch_cmd - pitch_cmd0);
            yaw_cmd = yaw_cmd0 + scale * (yaw_cmd - yaw_cmd0);
        }

        float dp, dr, dy;
        ap.step(p.roll_cmd_rad, pitch_cmd, yaw_cmd, ay_cmd, az_cmd, imu, dt,
                use_accel && p.autopilot_mode == AutopilotMode::ThreeLoop, dp, dr, dy);

        if (use_accel && p.autopilot_mode == AutopilotMode::CascadePid) {
            pitch_cmd = imu.pitch + clampf(ay_cmd * 0.15f, -0.25f, 0.25f);
            yaw_cmd = imu.yaw + clampf(az_cmd * 0.15f, -0.25f, 0.25f);
            ap.step(p.roll_cmd_rad, pitch_cmd, yaw_cmd, 0, 0, imu, dt, false, dp, dr, dy);
        }

        MixerOut out = mix.apply(dp, dr, dy);
        tel.phase = phase;
        tel.ay_cmd = ay_cmd;
        tel.az_cmd = az_cmd;
        tel.alpha_hat = ap.alpha_hat;
        tel.beta_hat = ap.beta_hat;
        tel.los_pitch = los.pitch;
        tel.los_yaw = los.yaw;
        tel.los_pitch_rate = los.pitch_rate;
        tel.los_yaw_rate = los.yaw_rate;
        tel.mix = out;
        return out;
    }
};

}  // namespace dart_fc
