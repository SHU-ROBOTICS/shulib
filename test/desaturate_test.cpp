// Adversarial tests for desaturateUniform (the downstream wheel-speed safety
// scale, §13 #5). The properties that MUST hold, and the bugs each would catch:
//   * within-budget commands are returned UNCHANGED (no scale-up),
//   * over-budget commands scale so the peak lands EXACTLY on the limit,
//   * scaling is UNIFORM — wheel ratios (hence motion direction) are preserved,
//   * a non-positive budget is a contract violation (throws, never UB),
//   * an all-zero command is a no-op for any positive budget.

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/kinematics/desaturate.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::kinematics::desaturateUniform;
using shulib::kinematics::WheelSpeeds;
using shulib::units::Velocity;

namespace {
WheelSpeeds make(std::initializer_list<double> vs) {
    WheelSpeeds w{static_cast<int>(vs.size())};
    int i = 0;
    for (double v : vs) {
        w.set(i++, Velocity{v});
    }
    return w;
}
}  // namespace

TEST_CASE("desaturateUniform: a command within budget is returned unchanged") {
    const WheelSpeeds in = make({1.0, -2.0, 3.0, -1.5});
    const WheelSpeeds out = desaturateUniform(in, Velocity{10.0});  // peak 3 << 10
    CHECK(out.approxEqual(in));
}

TEST_CASE("desaturateUniform: a command exactly at budget is NOT scaled") {
    const WheelSpeeds in = make({6.0, -6.0, 3.0});  // peak == 6
    const WheelSpeeds out = desaturateUniform(in, Velocity{6.0});
    CHECK(out.approxEqual(in));  // boundary: <= means equal is left alone
}

TEST_CASE("desaturateUniform: an over-budget command scales the peak onto the limit") {
    const WheelSpeeds in = make({6.0, -12.0, 3.0});  // peak 12, budget 6 -> scale 0.5
    const WheelSpeeds out = desaturateUniform(in, Velocity{6.0});

    // peak now sits exactly on the limit
    CHECK(out.maxMagnitude().value() == doctest::Approx(6.0));
    // and the values are the uniformly-scaled originals
    CHECK(out[0].value() == doctest::Approx(3.0));
    CHECK(out[1].value() == doctest::Approx(-6.0));
    CHECK(out[2].value() == doctest::Approx(1.5));
}

TEST_CASE("desaturateUniform: scaling is uniform — wheel ratios (direction) preserved") {
    const WheelSpeeds in = make({4.0, -10.0, 2.0});  // over budget 5 -> scale 0.5
    const WheelSpeeds out = desaturateUniform(in, Velocity{5.0});

    // every wheel scaled by the SAME factor: out[i]/in[i] is constant.
    const double k = out[1].value() / in[1].value();
    CHECK(out[0].value() / in[0].value() == doctest::Approx(k));
    CHECK(out[2].value() / in[2].value() == doctest::Approx(k));
    // sign is preserved (no wheel flips direction)
    CHECK(out[0].value() > 0.0);
    CHECK(out[1].value() < 0.0);
}

TEST_CASE("desaturateUniform: an all-zero command is a no-op for any positive budget") {
    const WheelSpeeds in = make({0.0, 0.0, 0.0, 0.0});
    const WheelSpeeds out = desaturateUniform(in, Velocity{0.001});
    CHECK(out.approxEqual(in));  // peak 0 <= budget, no divide-by-zero
}

TEST_CASE("desaturateUniform: a non-positive budget is a contract violation") {
    const WheelSpeeds in = make({1.0, 2.0});
    CHECK_THROWS_AS((void)desaturateUniform(in, Velocity{0.0}), PreconditionError);
    CHECK_THROWS_AS((void)desaturateUniform(in, Velocity{-1.0}), PreconditionError);
}
