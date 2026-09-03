#include "dart_fc.hpp"

#include <cstdio>

using namespace dart_fc;

static void simulate(AutopilotMode mode, const char* name) {
    Params p = Params::defaults();
    p.autopilot_mode = mode;
    Autopilot ap(p.roll, p.pitch, p.yaw, p.cascade, p.couple, mode, p.gravity);
    const float d_omega = p.aero.d_omega;
    const float d_delta = std::fabs(p.aero.d_delta);
    float gamma = 0.35f, p_rate = 0;
    const float dt = 0.005f;
    float t = 0;
    while (t < 1.5f) {
        ImuSample imu;
        imu.p = p_rate;
        imu.roll = gamma;
        imu.az = -9.81f;
        float dp, dr, dy;
        ap.step(0, 0, 0, 0, 0, imu, dt, false, dp, dr, dy);
        const float acc = -d_omega * p_rate + d_delta * dr;
        p_rate += acc * dt;
        gamma += p_rate * dt;
        t += dt;
    }
    std::printf("%-13s  gamma0=20.05 deg  gamma_end=%6.2f deg\n", name, gamma * 57.3f);
}

int main() {
    simulate(AutopilotMode::CascadePid, "cascade_pid");
    simulate(AutopilotMode::ThreeLoop, "three_loop");
    return 0;
}
