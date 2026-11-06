#include "../../../master/src/mpu_heading.h"
#include "../../../master/src/heading_hold.h"
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace carl;

int main(int argc, char** argv) {
  assert(argc == 2);
  auto* file = std::fopen(argv[1], "r"); assert(file);
  MpuHeading m; HeadingHold h;
  unsigned t, samples = 0, controlled = 0;
  int a[3], g[3];
  float peakYaw = 0;
  uint32_t lastControl = 0;
  while (std::fscanf(file, "%u,%d,%d,%d,%d,%d,%d", &t, a, a+1, a+2, g, g+1, g+2) == 7) {
    int16_t accel[3], gyro[3];
    for (unsigned i = 0; i < 3; ++i) { accel[i] = int16_t(a[i]); gyro[i] = int16_t(g[i]); }
    if (m.accept(accel, gyro, 0, t, t < 5000)) {
      assert(h.acceptYaw(m.yaw(), m.rate(), t, t));
    } else h.invalidate();
    if (t - lastControl >= 20) {
      lastControl = t;
      const int16_t value = t < 5000 ? 0 : 200;
      assert(m.allowRun(t, value != 0) || value == 0);
      const auto out = h.update(t, {value, value, value, value}, true);
      if (value) {
        assert(h.state() == HeadingHold::HOLDING);
        assert(std::abs(out.fl - value) <= 1 && std::abs(out.fr - value) <= 1);
        peakYaw = fmaxf(peakYaw, fabsf(h.error()));
        ++controlled;
      }
    }
    ++samples;
  }
  assert(std::fclose(file) == 0);
  assert(samples == 2004 && controlled > 500 && peakYaw < 0.00872665f);
  std::printf("Live stationary MPU replay: %u samples, bias=(%.4f, %.4f, %.4f) rad/s, peak heading error=%.4f deg, throttle trim <=1 per-mille\n",
              samples, m.bias(0), m.bias(1), m.bias(2), peakYaw * 57.29578f);
}
