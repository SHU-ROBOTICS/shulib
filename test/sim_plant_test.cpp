// Adversarial tests for sim::DrivePlant — the closed-loop host plant (chunk A2).
//
// Every expected value below is derived ANALYTICALLY in the test (X-drive/tank
// geometry + the feedforward inversion + rigid-body ω×r), never from the plant's own
// output — an open-loop test that trusts the plant to grade itself proves nothing.
//
// Targets: a dropped kS in the voltage→velocity law (required mutation #4), a flipped
// sign anywhere in the wheel→body direction chain (required mutation #2), an X-drive-
// only assumption (the plant must drive tank unchanged), synthesis sign errors on the
// tracking-wheel ω×r term (pinned here INDEPENDENTLY of PilonsOdometry, so an
// end-to-end sign cancellation is impossible), and every degenerate input in the
// brief: zero voltage, saturating voltage, zero dt, very large dt, and a reversal.

#include "doctest.h"

#include <cmath>
#include <limits>
#include <numbers>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::WheelSpeeds;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::sim::randomBodyTwist;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {
constexpr double kS = 1.2;
constexpr double kV = 0.17;

[[nodiscard]] SimHarnessConfig instantConfig() {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = kS, .kV = kV, .kA = 0.0};  // memoryless: distances exact by hand
    return cfg;
}

/// The steady wheel surface speed for |V| volts — the analytic FF inversion,
/// restated HERE so the test does not consult the model it is grading.
[[nodiscard]] double wheelSpeedFor(double volts) { return (volts - kS) / kV; }
}  // namespace

// ── Open-loop travel, X-drive, fully analytic (mutation #4's red: kS is live) ──
// Per-wheel voltages ±V give wheel speeds ∓u,∓u,±u,±u with u=(V−kS)/kV; the X-drive
// forward map yields body vx = √2·u (the classic body-faster-than-wheels property,
// derived from the coefficient geometry, not from the plant).
TEST_CASE("sim plant: X-drive open-loop forward travel matches the hand-derived distance") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    const double volts = 8.0;
    // canonical order FL,BL,BR,FR: forward needs negative surface speed on the left pair
    h.motor(0).setVoltage(Voltage{-volts});
    h.motor(1).setVoltage(Voltage{-volts});
    h.motor(2).setVoltage(Voltage{volts});
    h.motor(3).setVoltage(Voltage{volts});
    h.runTicks(200, Time{0.01});  // 2.0 s
    const double expected = std::numbers::sqrt2 * wheelSpeedFor(volts) * 2.0;  // √2·u·T
    CHECK(h.truePose().x().value() == doctest::Approx(expected).epsilon(1e-9));
    CHECK(h.truePose().y().value() == doctest::Approx(0.0).scale(1.0).epsilon(1e-9));
    CHECK(h.truePose().heading().approxEqual(Angle{}, 1e-9));
}

// ── Open-loop travel WITH the first-order lag. The plant is a DISCRETE-time system
// (documented ZOH: velocity advances to its end-of-tick value, held across the tick),
// so the oracle is the discrete geometric series, hand-derived here:
//     v_k = v_ss·(1 − q^k),  q = e^{−dt/τ}   ⇒   distance = dt·v_ss·(N − q(1−q^N)/(1−q))
// and the gap to the CONTINUOUS ramp integral must stay inside the documented O(dt)
// bound — so both the lag's effect on travel AND the discretization contract are
// verified (a plant that ignored the transient entirely would miss by v_ss·τ ≈ 17″). ──
TEST_CASE("sim plant: open-loop distance under lag matches the discrete analytic sum") {
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = kS, .kV = kV, .kA = 0.051};  // τ = 0.3 s
    SimHarness h{kin, cfg};
    const double volts = 8.0;
    h.motor(0).setVoltage(Voltage{-volts});
    h.motor(1).setVoltage(Voltage{-volts});
    h.motor(2).setVoltage(Voltage{volts});
    h.motor(3).setVoltage(Voltage{volts});
    const double T = 2.0, dt = 0.01;
    const int N = 200;
    h.runTicks(N, Time{dt});
    const double tau = 0.051 / kV;
    const double vssBody = std::numbers::sqrt2 * wheelSpeedFor(volts);
    const double q = std::exp(-dt / tau);
    const double qN = std::pow(q, static_cast<double>(N));
    const double discrete = dt * vssBody * (static_cast<double>(N) - q * (1.0 - qN) / (1.0 - q));
    CHECK(h.truePose().x().value() == doctest::Approx(discrete).epsilon(1e-9));
    // The discretization gap vs. the continuous integral is bounded by v_ss·dt/2·(…):
    const double continuous = vssBody * (T - tau * (1.0 - std::exp(-T / tau)));
    CHECK(std::abs(discrete - continuous) < vssBody * dt);  // the documented O(dt) bound
}

// ── Tank open-loop: straight and spin-in-place, both analytic (the plant must not
// be X-specific — a DoD line) ──
TEST_CASE("sim plant: tank open-loop straight line and pure spin match hand derivations") {
    const TankKinematics kin{Length{12.0}};  // halfTrack = 6
    SUBCASE("straight: equal voltages -> vx = u, no rotation") {
        SimHarness h{kin, instantConfig()};
        h.motor(0).setVoltage(Voltage{6.0});
        h.motor(1).setVoltage(Voltage{6.0});
        h.runTicks(150, Time{0.01});  // 1.5 s
        CHECK(h.truePose().x().value()
              == doctest::Approx(wheelSpeedFor(6.0) * 1.5).epsilon(1e-9));
        CHECK(h.truePose().y().value() == doctest::Approx(0.0).scale(1.0).epsilon(1e-9));
        CHECK(h.truePose().heading().approxEqual(Angle{}, 1e-9));
    }
    SUBCASE("spin: opposite voltages -> omega = u/halfTrack, position frozen") {
        SimHarness h{kin, instantConfig()};
        h.motor(0).setVoltage(Voltage{-6.0});
        h.motor(1).setVoltage(Voltage{6.0});
        h.runTicks(100, Time{0.01});  // 1.0 s
        const double omega = wheelSpeedFor(6.0) / 6.0;  // (right−left)/(2·halfTrack)
        CHECK(h.truePose().x().value() == doctest::Approx(0.0).scale(1.0).epsilon(1e-9));
        CHECK(h.truePose().y().value() == doctest::Approx(0.0).scale(1.0).epsilon(1e-9));
        CHECK(h.truePose().heading().approxEqual(Angle::radians(omega * 1.0), 1e-9));
    }
}

// ── Kinematics round-trip sweep (mutation #2's red): toWheels → FF → plant →
// observed twist must reproduce the command, and the plant's wheel spins must equal
// toWheels' output — swept over seeded random achievable twists, both drivetrains. ──
TEST_CASE("sim plant: commanded twist round-trips through the plant, swept, X-drive and tank") {
    SUBCASE("X-drive: fully holonomic commands") {
        const auto kin = xDrive(Length{7.0});
        SimHarness h{kin, instantConfig()};
        Rng rng{2024};
        for (int trial = 0; trial < 40; ++trial) {
            // Ranges keep every wheel INSIDE the ±12 V budget (peak ≈ 0.707·30 + 7·1.5
            // ≈ 31.7 in/s ⇒ ≈ 6.6 V): the round trip is only exact for achievable,
            // unclamped commands. The clamped regime has its own saturation test.
            const ChassisSpeeds cmd = randomBodyTwist(rng, 15.0, 1.5);
            h.commandBodyTwist(cmd);
            h.plant().step(Time{0.01});
            const auto observed = h.trueBodyTwist();
            CHECK(observed.vx().value() == doctest::Approx(cmd.vx().value()).scale(1.0).epsilon(1e-9));
            CHECK(observed.vy().value() == doctest::Approx(cmd.vy().value()).scale(1.0).epsilon(1e-9));
            CHECK(observed.omega().value()
                  == doctest::Approx(cmd.omega().value()).scale(1.0).epsilon(1e-9));
            const WheelSpeeds expect = kin.toWheels(cmd);
            const WheelSpeeds spin = h.plant().trueWheelSpin();
            REQUIRE(spin.size() == expect.size());
            CHECK(spin.approxEqual(expect, 1e-9));
        }
    }
    SUBCASE("tank: achievable commands (vy = 0)") {
        const TankKinematics kin{Length{12.0}};
        SimHarness h{kin, instantConfig()};
        Rng rng{99};
        for (int trial = 0; trial < 40; ++trial) {
            const ChassisSpeeds raw = randomBodyTwist(rng, 15.0, 1.5);  // inside ±12 V (see above)
            const ChassisSpeeds cmd{raw.vx(), Velocity{0.0}, raw.omega()};
            h.commandBodyTwist(cmd);
            h.plant().step(Time{0.01});
            const auto observed = h.trueBodyTwist();
            CHECK(observed.vx().value() == doctest::Approx(cmd.vx().value()).scale(1.0).epsilon(1e-9));
            CHECK(observed.vy().value() == doctest::Approx(0.0).scale(1.0).epsilon(1e-9));
            CHECK(observed.omega().value()
                  == doctest::Approx(cmd.omega().value()).scale(1.0).epsilon(1e-9));
        }
    }
}

// ── A commanded strafe on tank does nothing (honest non-holonomy, not an error) ──
TEST_CASE("sim plant: tank ignores a commanded strafe instead of faking one") {
    const TankKinematics kin{Length{12.0}};
    SimHarness h{kin, instantConfig()};
    h.commandBodyTwist(ChassisSpeeds{Velocity{0.0}, Velocity{20.0}, AngularVelocity{0.0}});
    h.runTicks(100, Time{0.01});
    CHECK(h.truePose().x().value() == 0.0);
    CHECK(h.truePose().y().value() == 0.0);
}

// ── Sensor-synthesis sign pins, INDEPENDENT of PilonsOdometry (so a synthesis sign
// error and an odometry sign error cannot cancel end-to-end): during a pure CCW spin,
// a forward wheel at left-offset b rolls at −ω·b and a lateral wheel at forward-
// offset a rolls at +ω·a (rigid body, v_point = ω × r). With the default harness
// offsets b = −3 (right of center) and a = −4.5 (behind center), CCW ω makes the
// forward wheel read POSITIVE travel and the lateral wheel NEGATIVE. ──
TEST_CASE("sim plant: tracking-wheel synthesis signs match rigid-body omega-cross-r") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    const double omega = 1.5;  // CCW
    h.commandBodyTwist(ChassisSpeeds{Velocity{0.0}, Velocity{0.0}, AngularVelocity{omega}});
    h.runTicks(100, Time{0.01});  // 1.0 s of pure spin
    const double rTrack = 1.0;    // default tracking diameter 2.0
    const double expectedForwardShaft = -omega * (-3.0) * 1.0 / rTrack;  // −ω·b·T / r  = +4.5
    const double expectedLateralShaft = omega * (-4.5) * 1.0 / rTrack;   // +ω·a·T / r  = −6.75
    CHECK(h.forwardEncoder().position().value()
          == doctest::Approx(expectedForwardShaft).epsilon(1e-9));
    CHECK(h.lateralEncoder().position().value()
          == doctest::Approx(expectedLateralShaft).epsilon(1e-9));
    // And during pure forward travel, the forward wheel reads exactly the distance.
    SimHarness h2{kin, instantConfig()};
    h2.commandBodyTwist(ChassisSpeeds{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.0}});
    h2.runTicks(100, Time{0.01});
    CHECK(h2.forwardEncoder().position().value() == doctest::Approx(10.0 / rTrack).epsilon(1e-9));
    CHECK(h2.lateralEncoder().position().value() == doctest::Approx(0.0).scale(1.0).epsilon(1e-9));
}

// ── IMU / GPS / drive-encoder synthesis mirror truth exactly (identity degradation) ──
TEST_CASE("sim plant: IMU, GPS and drive encoders report the synthesized truth") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    h.commandBodyTwist(ChassisSpeeds{Velocity{12.0}, Velocity{-5.0}, AngularVelocity{0.8}});
    h.runTicks(120, Time{0.01});
    CHECK(h.imu().heading().approxEqual(h.truePose().heading(), 1e-12));
    CHECK(h.imu().yawRate().value() == doctest::Approx(0.8).epsilon(1e-9));
    CHECK(h.gps().hasFix());
    CHECK(h.gps().pose().approxEqual(h.truePose(), Length{1e-9}, 1e-9));
    // drive encoder: shaft = spin·T / r, with r = 3.25/2
    const double spin0 = h.plant().trueWheelSpin()[0].value();
    CHECK(h.motor(0).position().value()
          == doctest::Approx(spin0 * 1.2 / (3.25 / 2.0)).epsilon(1e-9));
}

// ── Degenerates (the brief's explicit list) ──
TEST_CASE("sim plant: zero voltage means zero motion, with and without static friction") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    h.runTicks(100, Time{0.01});  // no commands ever issued
    CHECK(h.truePose().x().value() == 0.0);
    CHECK(h.truePose().y().value() == 0.0);
    CHECK(h.truePose().heading().radians() == 0.0);
    SimHarnessConfig noFriction;
    noFriction.plant.wheelFf = {.kS = 0.0, .kV = kV, .kA = 0.0};
    SimHarness h2{kin, noFriction};
    h2.runTicks(100, Time{0.01});
    CHECK(h2.truePose().x().value() == 0.0);
}

TEST_CASE("sim plant: a saturating command is clamped by the F4 motor contract, not the plant") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    h.motor(0).setVoltage(Voltage{-100.0});  // FakeMotor clamps to ±12 (F4)
    h.motor(1).setVoltage(Voltage{-100.0});
    h.motor(2).setVoltage(Voltage{100.0});
    h.motor(3).setVoltage(Voltage{100.0});
    CHECK(h.motor(2).commandedVoltage().value() == 12.0);  // the clamp actually happened
    h.runTicks(100, Time{0.01});
    const double expected = std::numbers::sqrt2 * wheelSpeedFor(12.0) * 1.0;
    CHECK(h.truePose().x().value() == doctest::Approx(expected).epsilon(1e-9));
    CHECK(std::isfinite(h.truePose().x().value()));
}

TEST_CASE("sim plant: dt == 0 is a true no-op (state, clock, sensors, and no record)") {
    const auto kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    SimHarness h{kin, instantConfig(), &sink};
    h.commandBodyTwist(ChassisSpeeds{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}});
    h.plant().step(Time{0.01});
    const auto before = h.sample();
    const auto recordsBefore = sink.recordCount();
    h.plant().step(Time{0.0});
    const auto after = h.sample();
    CHECK(after.t == before.t);
    CHECK(after.x == before.x);
    CHECK(after.vx == before.vx);
    CHECK(sink.recordCount() == recordsBefore);  // no phantom tick emitted
}

TEST_CASE("sim plant: a very large dt stays finite and lands on the analytic distance") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    h.commandBodyTwist(ChassisSpeeds{Velocity{30.0}, Velocity{0.0}, AngularVelocity{0.0}});
    h.plant().step(Time{10.0});  // one giant tick
    CHECK(std::isfinite(h.truePose().x().value()));
    CHECK(h.truePose().x().value() == doctest::Approx(300.0).epsilon(1e-9));
}

TEST_CASE("sim plant: a symmetric reversal returns exactly to the origin (memoryless model)") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    h.commandBodyTwist(ChassisSpeeds{Velocity{25.0}, Velocity{0.0}, AngularVelocity{0.0}});
    h.runTicks(100, Time{0.01});
    h.commandBodyTwist(ChassisSpeeds{Velocity{-25.0}, Velocity{0.0}, AngularVelocity{0.0}});
    h.runTicks(100, Time{0.01});
    CHECK(h.truePose().x().value() == doctest::Approx(0.0).scale(1.0).epsilon(1e-9));
    CHECK(h.truePose().y().value() == doctest::Approx(0.0).scale(1.0).epsilon(1e-9));
}

TEST_CASE("sim plant: contract violations are rejected loudly") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(h.plant().step(Time{-0.01}), PreconditionError);
    CHECK_THROWS_AS(h.plant().step(Time{nan}), PreconditionError);
    // a NaN voltage is refused at the F4 fake, before the plant ever sees it
    CHECK_THROWS_AS(h.motor(0).setVoltage(Voltage{nan}), PreconditionError);
}

// ── Initial pose config: truth AND the seeded sensors start there (a consumer
// constructing odometry from context sees the true start heading immediately) ──
TEST_CASE("sim plant: initial pose seeds truth and the sensors before the first tick") {
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig cfg = instantConfig();
    cfg.plant.initialPose =
        shulib::math::Pose2d{Length{24.0}, Length{-12.0}, Angle::degrees(30.0)};
    SimHarness h{kin, cfg};
    CHECK(h.truePose().approxEqual(cfg.plant.initialPose, Length{1e-12}, 1e-12));
    CHECK(h.imu().heading().approxEqual(Angle::degrees(30.0), 1e-12));
    CHECK(h.gps().pose().approxEqual(cfg.plant.initialPose, Length{1e-12}, 1e-12));
}
