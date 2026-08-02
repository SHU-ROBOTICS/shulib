// Adversarial tests for sim::MotorModel — the plant's voltage→velocity law (chunk A2).
//
// The model's ONE load-bearing claim is that it is the EXACT inverse of the existing
// control::Feedforward relation at steady state (motor_model.hpp's honesty boundary:
// the plant proves logic, not constants). Every case below targets a specific way that
// inversion — or the first-order approach to it — could be wrong: a dropped kS term
// (a required A2 mutation), a wrong dead-band boundary, a wrong time constant, a
// step-size-dependent transient, or a dt=0 that teleports the velocity.

#include "doctest.h"

#include <cmath>
#include <limits>

#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/sim/motor_model.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::control::Feedforward;
using shulib::control::FeedforwardGains;
using shulib::sim::MotorModel;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {
// kS deliberately NON-zero and kV deliberately non-round: a model that drops kS or
// approximates the division would pass with "nice" gains and fail with these.
constexpr FeedforwardGains kGains{.kS = 1.2, .kV = 0.17, .kA = 0.0};
constexpr FeedforwardGains kLagGains{.kS = 1.2, .kV = 0.17, .kA = 0.051};  // τ = 0.3 s
}  // namespace

// ── THE defining property: Feedforward::calculate(v) holds the wheel at exactly v ──
// (kS is live on both sides, so dropping it from EITHER goes red — mutation target.)
TEST_CASE("sim MotorModel: inverts Feedforward exactly at steady state, swept both signs") {
    const MotorModel model{kGains};
    const Feedforward ff{kGains};
    for (double v : {-70.0, -33.3, -5.0, -0.25, 0.25, 1.0, 12.75, 41.9, 70.0}) {
        const Voltage volts = ff.calculate(Velocity{v});
        // kA = 0 ⇒ the steady state is reached in one step, from ANY starting velocity.
        const Velocity out = model.advance(Velocity{-13.0}, volts, Time{0.01});
        CHECK(out.value() == doctest::Approx(v).epsilon(1e-12));
    }
    // v = 0: FF commands 0 V, and 0 V sits inside the dead band → stays 0.
    CHECK(model.advance(Velocity{5.0}, ff.calculate(Velocity{0.0}), Time{0.01}).value() == 0.0);
}

// ── Dead band: |V| <= kS produces NO motion; just above it produces barely any ──
// Catches a missing dead band (v = V/kV at rest would be wrong), and the <= boundary.
TEST_CASE("sim MotorModel: static-friction dead band, exact boundary included") {
    const MotorModel model{kGains};
    // inside and exactly AT the band, both signs → 0
    for (double volts : {0.0, 0.5, -0.5, 1.2, -1.2}) {
        CHECK(model.advance(Velocity{0.0}, Voltage{volts}, Time{0.01}).value() == 0.0);
    }
    // just above the band → strictly positive but tiny: v = (|V|−kS)/kV
    const double just = model.advance(Velocity{0.0}, Voltage{1.2 + 0.017}, Time{0.01}).value();
    CHECK(just == doctest::Approx(0.1));  // (1.217 − 1.2)/0.17
    const double justNeg = model.advance(Velocity{0.0}, Voltage{-1.2 - 0.017}, Time{0.01}).value();
    CHECK(justNeg == doctest::Approx(-0.1));
}

// ── First-order lag: the time constant is kA/kV, verified against the analytic
// exponential — not against the model's own output. Catches a wrong τ or a wrong
// approach direction. ──
TEST_CASE("sim MotorModel: lag approaches steady state with tau = kA/kV exactly") {
    const MotorModel model{kLagGains};
    const double tau = kLagGains.kA / kLagGains.kV;  // 0.3 s
    const double vss = (12.0 - kLagGains.kS) / kLagGains.kV;  // steady state for 12 V
    // After exactly one τ from rest: v = vss·(1 − e^−1). Analytic, not model-derived.
    const Velocity atTau = model.advance(Velocity{0.0}, Voltage{12.0}, Time{tau});
    CHECK(atTau.value() == doctest::Approx(vss * (1.0 - std::exp(-1.0))).epsilon(1e-12));
    // After 20τ: indistinguishable from vss.
    const Velocity late = model.advance(Velocity{0.0}, Voltage{12.0}, Time{20.0 * tau});
    CHECK(late.value() == doctest::Approx(vss).epsilon(1e-8));
}

// ── Exponential exactness (semigroup): one big step == many small steps. This is what
// makes plant behaviour independent of the tick size — an Euler-style approximation
// here would make every closed-loop test's result depend on dt, silently. ──
TEST_CASE("sim MotorModel: one step of dt equals ten chained steps of dt/10 (exact update)") {
    const MotorModel model{kLagGains};
    const Voltage volts{7.3};
    const Velocity oneStep = model.advance(Velocity{-4.0}, volts, Time{0.1});
    Velocity chained{-4.0};
    for (int i = 0; i < 10; ++i) {
        chained = model.advance(chained, volts, Time{0.01});
    }
    CHECK(chained.value() == doctest::Approx(oneStep.value()).epsilon(1e-12));
}

// ── Reversal: +v under a negative command decays THROUGH zero toward the negative
// steady state, monotonically, finitely. Catches a sign trap in the lag update. ──
TEST_CASE("sim MotorModel: a commanded reversal decays smoothly through zero") {
    const MotorModel model{kLagGains};
    Velocity v{30.0};
    double prev = v.value();
    bool crossed = false;
    for (int i = 0; i < 600; ++i) {  // 6 s = 20τ: the residual (70·e^−20 ≈ 1e-7) is below tolerance
        v = model.advance(v, Voltage{-8.0}, Time{0.01});
        REQUIRE(std::isfinite(v.value()));
        CHECK(v.value() < prev);  // strictly monotonic toward the negative target
        crossed = crossed || (v.value() < 0.0);
        prev = v.value();
    }
    CHECK(crossed);  // it actually reversed, not just slowed
    CHECK(v.value() == doctest::Approx(-(8.0 - kLagGains.kS) / kLagGains.kV).epsilon(1e-6));
}

// ── dt = 0: time did not pass, so the velocity MUST NOT change — including with
// kA = 0, where the "instantaneous" branch would otherwise teleport to steady state.
// (This ordering is a genuine trap: the dt==0 early-out must precede the kA check.) ──
TEST_CASE("sim MotorModel: dt == 0 changes nothing, even in the instantaneous kA=0 model") {
    const MotorModel instant{kGains};
    const MotorModel lagged{kLagGains};
    CHECK(instant.advance(Velocity{3.7}, Voltage{12.0}, Time{0.0}).value() == 3.7);
    CHECK(lagged.advance(Velocity{3.7}, Voltage{12.0}, Time{0.0}).value() == 3.7);
}

// ── Saturating / extreme inputs stay finite and predictable ──
TEST_CASE("sim MotorModel: extreme voltage and huge dt remain finite and land on v_ss") {
    const MotorModel model{kLagGains};
    const Velocity v = model.advance(Velocity{0.0}, Voltage{12.0}, Time{1e6});
    REQUIRE(std::isfinite(v.value()));
    CHECK(v.value() == doctest::Approx((12.0 - kLagGains.kS) / kLagGains.kV));
}

// ── Contract: bad gains and bad dt are rejected loudly, not absorbed ──
TEST_CASE("sim MotorModel: precondition violations are red, not silent") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((MotorModel{{.kS = 0.0, .kV = 0.0, .kA = 0.0}}), PreconditionError);   // kV = 0
    CHECK_THROWS_AS((MotorModel{{.kS = 0.0, .kV = -1.0, .kA = 0.0}}), PreconditionError);  // kV < 0
    CHECK_THROWS_AS((MotorModel{{.kS = -0.1, .kV = 1.0, .kA = 0.0}}), PreconditionError);  // kS < 0
    CHECK_THROWS_AS((MotorModel{{.kS = 0.0, .kV = 1.0, .kA = -0.1}}), PreconditionError);  // kA < 0
    CHECK_THROWS_AS((MotorModel{{.kS = nan, .kV = 1.0, .kA = 0.0}}), PreconditionError);
    const MotorModel ok{kGains};
    CHECK_THROWS_AS((void)ok.advance(Velocity{0.0}, Voltage{1.0}, Time{-0.01}), PreconditionError);
    CHECK_THROWS_AS((void)ok.advance(Velocity{0.0}, Voltage{1.0}, Time{nan}), PreconditionError);
}
