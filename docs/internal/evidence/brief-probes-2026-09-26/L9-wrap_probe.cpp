#include <cmath>
#include <cstdio>
#include <cstring>
#include "shulib/math/angle.hpp"
using shulib::math::Angle;
int main() {
  int differ = 0, n = 0;
  double samples[] = {0.0, 0.3, 0.9272952180016122, 1.0, -1.0, 2.5, -2.5, 3.0, -3.0, 0.5880026035475675};
  for (double b0 : samples) {
    Angle b = Angle::radians(b0);
    Angle plus = b + Angle::radians(Angle::kPi);
    Angle minus = b - Angle::radians(Angle::kPi);
    double p = plus.radians(), m = minus.radians();
    bool same = std::memcmp(&p, &m, sizeof p) == 0;
    std::printf("b=%.17g plus=%.17g minus=%.17g %s\n", b0, p, m, same ? "SAME" : "DIFFER");
  }
  for (int i = -100000; i <= 100000; ++i) {
    double b0 = i * (Angle::kPi / 100000.0);
    if (!std::isfinite(b0)) continue;
    Angle b = Angle::radians(b0);
    double p = (b + Angle::radians(Angle::kPi)).radians();
    double m = (b - Angle::radians(Angle::kPi)).radians();
    ++n; if (std::memcmp(&p, &m, sizeof p) != 0) ++differ;
  }
  std::printf("sweep: %d of %d bearings differ bitwise\n", differ, n);
}
