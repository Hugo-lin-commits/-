#include "dart_fc.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace dart_fc;

static void check(bool ok, const char* msg) {
    if (!ok) {
        std::fprintf(stderr, "FAIL %s\n", msg);
        std::exit(1);
    }
}

int main() {
    const float w = gyro_raw_to_rad_s(32767, 2000.0f);
    check(w > 34.0f && w < 36.0f, "gyro scale");
    const float a = accel_raw_to_mps2(2048, 16.0f, 9.81f);
    check(std::fabs(a - 9.81f) < 0.05f, "accel scale");

    const int idx[3] = {1, 0, 2};
    const float sign[3] = {1, 1, -1};
    const Vec3 body = remap_axis({0.1f, 0.2f, 0.3f}, idx, sign);
    check(std::fabs(body.x - 0.2f) < 1e-6f && std::fabs(body.y - 0.1f) < 1e-6f &&
              std::fabs(body.z + 0.3f) < 1e-6f,
          "remap");

    Params p = Params::defaults();
    FlightController fc(p);
    ImuSample imu;
    imu.az = -9.81f;
    VisionSample vis;
    vis.valid = true;
    vis.px = 140;
    vis.py = 100;
    vis.w = 20;
    vis.h = 20;
    vis.t = 0.4f;
    MixerOut out;
    for (int k = 0; k < 80; ++k) {
        const float t = k * 0.005f;
        if (k == 10) {
            imu.ax = 120;
            imu.az = -9.81f;
        } else if (k == 12) {
            imu.ax = 0;
            imu.pitch = 0.2f;
            imu.az = -9.81f;
        }
        out = fc.step(t, 0.005f, imu, t > 0.35f ? vis : VisionSample{});
    }
    std::printf("phase %s\n", phase_name(fc.tel.phase));
    std::printf("delta_deg %.2f %.2f %.2f %.2f\n", out.delta_deg[0], out.delta_deg[1], out.delta_deg[2],
                out.delta_deg[3]);
    std::printf("hw remap + controller ok\n");
    return 0;
}
