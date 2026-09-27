#include <cmath>
#include <cstdio>
#include <cstring>
#include "shulib/math/angle.hpp"
using shulib::math::Angle;
static void probe(double sx, double sy, double tx, double ty) {
  const double b0 = std::atan2(ty - sy, tx - sx);
  const Angle b = Angle::radians(b0);
  const double p = (b + Angle::radians(Angle::kPi)).radians();
  const double m = (b - Angle::radians(Angle::kPi)).radians();
  std::printf("start(%g,%g) -> (%g,%g): bearing=%.17g  +pi=%.17g  -pi=%.17g  %s\n", sx, sy, tx, ty, b0, p, m,
              std::memcmp(&p, &m, sizeof p) == 0 ? "SAME (mutant 7 equivalent here)" : "DIFFER");
}
int main() {
  probe(0, 0, 24, 0);      // G2 runner 10's geometry
  probe(0, 0, 18, 30);     // D1 sugar's target from the origin
  probe(-24, -12, 12, 6);
  probe(-24, -12, 0, 24);
  probe(-24, -12, 6, 18);
  probe(-24, -12, 18, 0);
  probe(-24, -12, -6, 20);
  probe(10, -20, -14, 4);
  probe(-24, -12, 12, 24);
}
