// Adversarial tests for the A2 scenario layer: seeded determinism (constraint 4),
// the A1 telemetry cost contract, TermSink watchability (DoD), and — crucially — the
// proof that every A3 degradation seam is actually CONSULTED by the plant. Identity
// hooks alone can't prove that (a plant that ignored the policy entirely would pass
// every identity test); so a test-local hostile model overrides each hook with a
// distinctive transform and the run must visibly change in exactly the predicted way.
// (Those transforms live HERE, in the test, on purpose: A3 owns the real behaviours.)

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <string>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::TermSink;
using shulib::hal::fake::FakeCharSink;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::sim::DegradationModel;
using shulib::sim::GpsTruth;
using shulib::sim::randomBodyTwist;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::sim::TruthSample;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {
[[nodiscard]] SimHarnessConfig cfgWithSeed(std::uint64_t seed) {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.051};
    cfg.plant.seed = seed;
    return cfg;
}

/// One complete seeded scenario: a random twist redrawn every 25 ticks, 200 ticks.
/// Returns the per-tick truth stream. A pure function of (kin, seed) by design.
[[nodiscard]] std::vector<TruthSample> runScenario(std::uint64_t seed) {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, cfgWithSeed(seed)};
    std::vector<TruthSample> out;
    out.reserve(200);
    ChassisSpeeds cmd{};
    h.runTicks(200, Time{0.01}, [&](int tick) {
        if (tick % 25 == 0) {
            cmd = randomBodyTwist(h.rng(), 15.0, 1.5);
        }
        h.commandBodyTwist(cmd);
        out.push_back(h.sample());
    });
    out.push_back(h.sample());
    return out;
}

[[nodiscard]] bool byteIdentical(const std::vector<TruthSample>& a,
                                 const std::vector<TruthSample>& b) {
    return a.size() == b.size()
        && std::memcmp(a.data(), b.data(), a.size() * sizeof(TruthSample)) == 0;
}
}  // namespace

// ── Rng: pinned against the PUBLISHED SplitMix64 reference sequence, computed
// independently (python reimplementation, logged in A2-PROGRESS) — not against this
// implementation's own output. Any constant/shift typo reds here immediately. ──
TEST_CASE("sim Rng: matches the SplitMix64 reference vectors for seeds 0 and 42") {
    Rng r0{0};
    CHECK(r0.nextU64() == 0xE220A8397B1DCDAFULL);
    CHECK(r0.nextU64() == 0x6E789E6AA1B965F4ULL);
    CHECK(r0.nextU64() == 0x06C45D188009454FULL);
    Rng r42{42};
    CHECK(r42.nextU64() == 0xBDD732262FEB6E95ULL);
    CHECK(r42.nextU64() == 0x28EFE333B266F103ULL);
}

TEST_CASE("sim Rng: unit and uniform draws stay inside their intervals (sweep)") {
    Rng r{123};
    for (int i = 0; i < 10000; ++i) {
        const double u = r.nextUnit();
        CHECK(u >= 0.0);
        CHECK(u < 1.0);
    }
    Rng r2{456};
    for (int i = 0; i < 1000; ++i) {
        const double v = r2.uniform(-3.0, 7.5);
        CHECK(v >= -3.0);
        CHECK(v < 7.5);
    }
}

// ── Constraint 4 / DoD: byte-identical replay from a seed; a different seed
// genuinely diverges. memcmp on the packed samples — representations, not tolerances. ──
TEST_CASE("sim scenario: same seed replays byte-identically; different seed diverges") {
    const std::vector<TruthSample> runA = runScenario(2026);
    const std::vector<TruthSample> runB = runScenario(2026);
    CHECK(byteIdentical(runA, runB));
    const std::vector<TruthSample> runC = runScenario(2027);
    CHECK_FALSE(byteIdentical(runA, runC));
    // and the difference is physical, not representational noise:
    CHECK(std::abs(runA.back().x - runC.back().x) > 1e-3);
}

// ── DoD: a run is WATCHABLE via TermSink — one legible per-tick line per step ──
TEST_CASE("sim scenario: TermSink renders one per-tick line per plant step") {
    const auto kin = xDrive(Length{7.0});
    // Wiring note: TermSink wants a clock for log() lines, but stamps emit() lines
    // from the RECORD's own t (the A1 replay contract) — so the per-tick rendering
    // below is correct with a standalone render clock, and the SimHarness can take
    // the sink at construction without a circular clock dependency.
    FakeCharSink chars;
    shulib::hal::fake::FakeClock renderClock;
    TermSink sink{renderClock, chars};
    SimHarness h{kin, cfgWithSeed(1), &sink};
    h.commandBodyTwist(ChassisSpeeds{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.5}});
    h.runTicks(5, Time{0.01});
    const std::string& text = chars.text();
    REQUIRE_FALSE(text.empty());
    // exactly one framed line per tick
    CHECK(std::count(text.begin(), text.end(), '\n') == 5);
    CHECK(text.find("[t=") != std::string::npos);   // stamped from the record's t
    CHECK(text.find("[LOC]") != std::string::npos);  // idle-tick tag (no active command)
    CHECK(text.find("0.05") != std::string::npos);   // the final tick's timestamp appears
}

// ── A1 cost contract, both sides: a consuming sink gets a correctly populated record
// per tick; a non-consuming sink must never even see emit() (the plant must consult
// wantsRecord() via emitRecord — calling emit() unconditionally reds here) ──
TEST_CASE("sim scenario: per-tick records are populated for consumers and skipped otherwise") {
    const auto kin = xDrive(Length{7.0});
    SUBCASE("consuming sink: one record per tick, fields as synthesized") {
        FakeTelemetrySink sink;
        SimHarness h{kin, cfgWithSeed(1), &sink};
        h.commandBodyTwist(ChassisSpeeds{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.0}});
        h.runTicks(3, Time{0.01});
        REQUIRE(sink.recordCount() == 3);
        const auto& r = sink.recordAt(2);
        CHECK(r.t.value() == doctest::Approx(0.03));
        CHECK(r.dt.value() == doctest::Approx(0.01));
        CHECK(r.wheelCount == 4);
        CHECK(r.wheelVoltage[0].value() == h.motor(0).commandedVoltage().value());
        CHECK(r.imuYaw.approxEqual(h.imu().heading(), 1e-12));
        CHECK(r.batteryVoltage.value() == doctest::Approx(12.6));
    }
    SUBCASE("non-consuming sink: emit() is never reached (the lazy-build proof)") {
        // Deliberately contract-violating pair (wantsRecord false + observing emit):
        // legal ONLY as a test probe — it detects a plant that skips the wantsRecord
        // gate, which no compliant sink could observe.
        struct TrapSink final : shulib::hal::ITelemetrySink {
            int emits = 0;
            void log(shulib::hal::LogLevel, std::string_view, std::string_view) override {}
            [[nodiscard]] bool wantsRecord() const noexcept override { return false; }
            void emit(const shulib::diag::DebugRecord&) override { ++emits; }
        } trap;
        SimHarness h{kin, cfgWithSeed(1), &trap};
        h.runTicks(10, Time{0.01});
        CHECK(trap.emits == 0);
    }
}

// ── Seam liveness: every degradation hook must actually be routed through. Each
// override below applies a distinctive transform; each assertion can only pass if
// the plant consulted THAT hook. (A3 will replace these stubs with real models.) ──
TEST_CASE("sim scenario: every degradation seam is live (hostile stub changes the run)") {
    struct ProbeModel final : DegradationModel {
        Voltage effectiveVoltage(int, Voltage commanded, Time, Rng&) override {
            return Voltage{commanded.value() * 0.5};  // brownout-ish: half the volts arrive
        }
        Velocity wheelMotionVelocity(int, Velocity spin, Time, Rng&) override {
            return Velocity{spin.value() * 0.8};  // 20% slip: body moves less than wheels
        }
        AngleDim driveEncoderPosition(int, AngleDim trueShaft, Time, Rng&) override {
            return AngleDim{trueShaft.value() + 1.0};  // +1 rad bias
        }
        AngleDim trackingEncoderPosition(int, AngleDim trueShaft, Time, Rng&) override {
            return AngleDim{trueShaft.value() + 2.0};  // +2 rad bias
        }
        Angle imuHeading(Angle trueHeading, Time, Rng&) override {
            return trueHeading + Angle::radians(0.1);  // 0.1 rad boot bias
        }
        AngularVelocity imuYawRate(AngularVelocity trueRate, Time, Rng&) override {
            return AngularVelocity{trueRate.value() + 0.5};
        }
        bool imuReady(bool, Time) override { return false; }  // dropped out
        GpsTruth gps(const GpsTruth& truth, Time, Rng&) override {
            return GpsTruth{truth.pose, Length{99.0}, false};  // no-fix, error inflated
        }
        Voltage batteryVoltage(Voltage, Time, Rng&) override { return Voltage{11.0}; }
    } probe;

    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 0.0, .kV = 0.17, .kA = 0.0};  // kS=0: halving V halves speed
    SimHarness h{kin, cfg, nullptr, &probe};
    h.motor(0).setVoltage(Voltage{-8.0});
    h.motor(1).setVoltage(Voltage{-8.0});
    h.motor(2).setVoltage(Voltage{8.0});
    h.motor(3).setVoltage(Voltage{8.0});
    h.runTicks(100, Time{0.01});  // 1 s

    const double sqrt2 = std::numbers::sqrt2;
    const double fullSpeedX = sqrt2 * (8.0 / 0.17);        // no degradation would give this
    const double halfVoltSpin = (0.5 * 8.0) / 0.17;        // effectiveVoltage seam
    const double slippedX = sqrt2 * halfVoltSpin * 0.8;    // × wheelMotionVelocity seam
    CHECK(h.truePose().x().value() == doctest::Approx(slippedX * 1.0).epsilon(1e-9));
    CHECK(h.truePose().x().value() < 0.5 * fullSpeedX);    // visibly not the clean run
    // encoders: spin (NOT slipped motion) + the per-seam biases
    const double wheelR = 3.25 / 2.0;
    CHECK(h.motor(2).position().value()
          == doctest::Approx(halfVoltSpin * 1.0 / wheelR + 1.0).epsilon(1e-9));
    const double trackR = 1.0;
    const double bodyVx = slippedX / sqrt2 * sqrt2;  // = slippedX (heading 0, straight)
    CHECK(h.forwardEncoder().position().value()
          == doctest::Approx(bodyVx * 1.0 / trackR + 2.0).epsilon(1e-9));
    // IMU seams: heading biased, rate biased, readiness dropped — truth unaffected
    CHECK(h.imu().heading().approxEqual(h.truePose().heading() + Angle::radians(0.1), 1e-12));
    CHECK(h.imu().yawRate().value() == doctest::Approx(0.5));
    CHECK_FALSE(h.imu().isReady());
    // GPS + battery seams
    CHECK_FALSE(h.gps().hasFix());
    CHECK(h.gps().rmsError().value() == doctest::Approx(99.0));
    CHECK(h.battery().voltage().value() == doctest::Approx(11.0));
    CHECK(h.truePose().heading().radians() == doctest::Approx(0.0));  // truth stayed clean
}

// ── The jitter seam: a variable dt schedule integrates each tick's ACTUAL dt ──
TEST_CASE("sim scenario: the variable-dt runner integrates the schedule it is given") {
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 0.0, .kV = 0.17, .kA = 0.0};
    SimHarness h{kin, cfg};
    h.commandBodyTwist(ChassisSpeeds{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.0}});
    double total = 0.0;
    h.runTicksVariable(
        20, [&](int i) {
            const double dt = (i % 2 == 0) ? 0.005 : 0.02;  // jittering 5–20 ms
            total += dt;
            return Time{dt};
        },
        [](int) {});
    CHECK(h.clock().now().value() == doctest::Approx(total).epsilon(1e-12));
    CHECK(h.truePose().x().value() == doctest::Approx(10.0 * total).epsilon(1e-9));
}

// ── Clock lockstep: N fixed ticks advance the ONE injected clock by exactly N·dt ──
TEST_CASE("sim scenario: the plant advances the shared clock in lockstep with physics") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin};
    CHECK(h.clock().now().value() == 0.0);
    CHECK(&h.context().clock() == &h.clock());  // the code under test reads the SAME clock
    h.runTicks(250, Time{0.01});
    CHECK(h.clock().now().value() == doctest::Approx(2.5).epsilon(1e-12));
}

TEST_CASE("sim scenario: negative tick counts are rejected") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin};
    CHECK_THROWS_AS(h.runTicks(-1, Time{0.01}), PreconditionError);
}
