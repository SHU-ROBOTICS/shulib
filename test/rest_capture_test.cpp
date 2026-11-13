// calibration::RestCapture — shulib Calibrate's REST step — and settleFloorReport() (chunk R5a Part
// A, A.4), on the A2 host test plant. NOTHING HERE HAS RUN ON A ROBOT, and the plant CANNOT show the
// V5 IMU's noise STRUCTURE (A4 register HA-173): these cases prove the estimators on the two
// structures the tests can script — the A3 hostile IMU's WHITE heading noise with a drawn per-boot
// bias, and a test-local RANDOM-WALK heading — not which one a V5 has. Each case names the bug it
// catches and the mutation that must turn it red:
//   R1  the hostile IMU with a large drawn bias (rateBiasMax 10 °/min, seeds 1–8): the drift is the
//       drawn bias within 4σ of the slope estimator, the heading σ within its sampling bound, ρ ≈ 1
//       within its bound, the yaw-rate mean the bias again — and NO Turned, though the IMU "turns"
//       ~8° over the capture             (drift as end minus start: red; σ about the MEAN instead of
//                                          the line: red; Turned read from the IMU: red)
//   R2  a random-walk heading at HA-22's 0.5 °/s per 10 ms tick: σ_Δ/dt is the walk's σ within its
//       bound, ρ ≪ 1, the walk-model drift σ ≈ 0.42 °/min, and settleFloorReport does NOT warn
//       against 0.30 rad/s                 (settleFloorReport using √2·σ_line/dt: red)
//   R3  a capture straddling ±π (from 179.9°) measures the same drift as one from 0°
//                                          (regression on the WRAPPED heading: red)
//   R4  a push, FORWARD and BACKWARD, through the plant → Moved; a turn in place through the wheels,
//       COUNTER-CLOCKWISE and CLOCKWISE → Turned, not Moved; the IMU dropping out mid-capture →
//       ImuNotReady; every refusal 0 V + Coast  (a refusal that leaves the pushed volts on: red;
//                                                Moved or Turned judged in one direction only: red)
//   R5  a motor that never rests → RestTimeout naming it; an IMU that never readies → ImuNotReady;
//       both 0 V + Coast
//   R6  abort() in every phase; start() leaves every member at 0 V + Coast
//   R7  settleFloorReport's margins at their edges (exactly 2× does not warn; under it does; zero
//       noise is an infinite margin)
//   R8  RestCaptureConfig::valid() and the preconditions
//   R10 Moved and Turned at their BOUNDS, both directions: encoder readings scripted 1 % under each
//       bound finish Done and record exactly that motion; 1 % over refuses
//                                          (Turned on 2W; Moved on a tenth of the travel: red)
//   R11 a real nudge UNDER the bounds (one side pushed 0.47 in; a turn in place of 2.7°) on an IMU
//       with no drift: the capture finishes, its drift is the nudge's 6·θ·t0·(T − t0)/T³, inside
//       driftTurnBiasBound(), and the summary records the rotation that caused it; ρ is dragged down
//                                          (the encoder motion not recorded: red)
//   R12 the encoder speed σ at rest, with a different jitter AND a different scale per side: each
//       side's σ is the two-pass σ of its own median speed; settleFloorReport takes the larger
//                                          (the σ zeroed; the RIGHT side on the LEFT's scale: red)
//   R13 the Settle phase: a yaw rate that reads rotation delays the capture until it stops AND rest
//       has held restSettleTime; one moving tick restarts the settle; a rotation that never stops
//       refuses RestTimeout "still rotating"   (settle on the motors alone; no settle time; the
//                                               settle not restarted by motion: red)
//   R14 a quiet plant: ρ is UNDEFINED (structureRatioDefined() false), not "a random walk"; a
//       one-way IMU jump is the largest heading step by its MAGNITUDE  (the step kept signed: red)
//   R15 a side with NO finite reading on a tick — every member's velocity() NaN, then every
//       member's position() NaN through the capture's last ticks: the capture finishes, its summary
//       is valid() and equals a clean capture's, and unreadSides() counts each left-out reading
//                                          (a NaN speed added to the σ; a NaN travel recorded: red)
//   R9  every refusal code has a short ASCII name (the reasons are checked where each is produced)
// The numbers the IMU models run on are INVENTED (HA-20/21/22), and so are RestCaptureConfig's
// (HA-170).

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "shulib/calibration/drive_characterization.hpp"
#include "shulib/calibration/rest_capture.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::calibration::DriveSide;
using shulib::calibration::RestCapture;
using shulib::calibration::RestCaptureConfig;
using shulib::calibration::RestPhase;
using shulib::calibration::RestRefusal;
using shulib::calibration::restRefusalName;
using shulib::calibration::RestSummary;
using shulib::calibration::settleFloorReport;
using shulib::calibration::SettleFloorReport;
using shulib::calibration::SideMember;
using shulib::hal::BrakeMode;
using shulib::hal::fake::FakeMotor;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::sim::ImuHostileConfig;
using shulib::sim::ImuHostileModel;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

/// A RELATIVE tolerance. doctest's Approx is relative only for values above 1: its tolerance is
/// epsilon × (scale + max(|a|, |b|)) with a scale of 1 by default, so Approx(0.02).epsilon(1e-6)
/// accepts ±1e-6 — 5e-5 relative — and Approx(0.052).epsilon(0.1) accepts anything within 0.1 of
/// 0.052, twice the target itself (found 2026-10-04: a mutation survived exactly that). Every pin in
/// this file means relative, so each goes through here. The target must be nonzero: at scale 0 an
/// exact 0 never compares equal.
[[nodiscard]] doctest::Approx rel(double value, double epsilon) {
    return doctest::Approx(value).epsilon(epsilon).scale(0.0);
}

constexpr double kDt = 0.01;
constexpr double kR = 3.25 / 2.0;
constexpr double kW = 12.5;
constexpr double kDeg = Angle::kPi / 180.0;

/// Robot two's shape at rest on the plant: five members a side, and a REST capture over them.
struct Rig {
    static constexpr std::size_t kPer = 5;
    TankKinematics kin{Length{kW}};
    SimHarness h;
    std::array<FakeMotor, 2 * (kPer - 1)> extras{};
    std::array<SideMember, kPer> left{};
    std::array<SideMember, kPer> right{};
    double ipr[2] = {kR, kR};  ///< the DESCRIPTION's inches per radian, per side
    std::optional<RestCapture> cap;

    explicit Rig(shulib::sim::DegradationModel* degradation = nullptr, double headingDeg = 0.0,
                 const RestCaptureConfig& cfg = {}, std::uint64_t seed = 1, double iprScaleR = 1.0)
        : h{kin, plantAt(headingDeg, seed), nullptr, degradation} {
        for (int w = 0; w < 2; ++w) {
            std::array<FakeMotor*, kPer - 1> ex{};
            for (std::size_t k = 1; k < kPer; ++k) {
                ex[k - 1] = &extras[static_cast<std::size_t>(w) * (kPer - 1) + (k - 1)];
            }
            h.plant().attachCoupledMembers(w, std::span<FakeMotor* const>{ex.data(), ex.size()});
        }
        for (std::size_t k = 0; k < kPer; ++k) {
            left[k] = SideMember{&member(0, k), 1, static_cast<std::int8_t>(11 + k)};
            right[k] = SideMember{&member(1, k), 1, static_cast<std::int8_t>(16 + k)};
        }
        ipr[1] = kR * iprScaleR;
        cap.emplace(DriveSide{std::span<const SideMember>{left.data(), left.size()}, Length{ipr[0]}},
                    DriveSide{std::span<const SideMember>{right.data(), right.size()}, Length{ipr[1]}},
                    Length{kW}, h.imu(), h.clock(), cfg);
    }
    static SimHarnessConfig plantAt(double headingDeg, std::uint64_t seed) {
        SimHarnessConfig c;
        c.plant.seed = seed;
        c.plant.initialPose = shulib::math::Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(headingDeg)};
        return c;
    }
    [[nodiscard]] FakeMotor& member(int wheel, std::size_t k) {
        return k == 0 ? h.motor(wheel)
                      : extras[static_cast<std::size_t>(wheel) * (kPer - 1) + (k - 1)];
    }
    /// Command a wheel the way a HAND would push it: every member of the train at `volts`.
    void push(int wheel, double volts) {
        for (std::size_t k = 0; k < kPer; ++k) {
            member(wheel, k).setVoltage(Voltage{volts});
        }
    }
    [[nodiscard]] bool safe() {
        for (int w = 0; w < 2; ++w) {
            for (std::size_t k = 0; k < kPer; ++k) {
                if (member(w, k).commandedVoltage().value() != 0.0
                    || member(w, k).brakeMode() != BrakeMode::Coast) {
                    return false;
                }
            }
        }
        return true;
    }
};

/// Printable ASCII and no '%' (a panel paints it) — and not empty.
[[nodiscard]] bool screenSafe(const std::string& t) {
    if (t.empty()) {
        return false;
    }
    for (char c : t) {
        const auto u = static_cast<unsigned char>(c);
        if (u < 0x20 || u > 0x7E || c == '%') {
            return false;
        }
    }
    return true;
}

/// Run a capture to its end; `hook` runs after each plant step, before the next tick — so a scripted
/// reading is what that tick reads, and a scripted push is what the plant does at its next step.
/// A refusal's reason is checked for screen safety here, where it is produced.
RestPhase run(Rig& rig, const std::function<void(Rig&, int)>& hook = {}, int maxTicks = 20000) {
    RestCapture& c = *rig.cap;
    if (c.phase() == RestPhase::Idle) {
        c.start();
    }
    for (int i = 0; i < maxTicks; ++i) {
        c.tick();
        if (c.phase() == RestPhase::Done || c.phase() == RestPhase::Aborted) {
            break;
        }
        rig.h.plant().step(Time{kDt});
        if (hook) {
            hook(rig, i);
        }
    }
    if (c.phase() == RestPhase::Aborted) {
        CHECK(screenSafe(c.reason()));
    }
    return c.phase();
}

/// A RANDOM-WALK heading (a gyro's heading is the integral of its rate): each tick adds a Gaussian
/// step of `stepSigma` rad, drawn from the plant's seeded Rng, and the yaw rate reports that step
/// over dt. A test fixture, not a plant change.
struct WalkImu final : shulib::sim::DegradationModel {
    double stepSigma = 0.0;
    double walk = 0.0;
    double lastStep = 0.0;
    Angle imuHeading(Angle truth, Time, shulib::sim::Rng& rng) override {
        lastStep = stepSigma * rng.nextGaussian();
        walk += lastStep;
        return truth + Angle::radians(walk);
    }
    AngularVelocity imuYawRate(AngularVelocity truth, Time, shulib::sim::Rng&) override {
        return AngularVelocity{truth.value() + lastStep / kDt};
    }
};

/// A steady drift of `rate` rad/s plus white heading noise `sigma` — deterministic from the seed,
/// so two captures from different start headings see the SAME noise draws — and, optionally, a
/// one-way heading jump of `jump` rad at `jumpAt` seconds (an IMU glitch the encoders do not see).
struct DriftImu final : shulib::sim::DegradationModel {
    double rate = 0.0;
    double sigma = 0.0;
    double jump = 0.0;
    double jumpAt = 1e18;
    Angle imuHeading(Angle truth, Time now, shulib::sim::Rng& rng) override {
        const double glitch = now.value() >= jumpAt ? jump : 0.0;
        return truth + Angle::radians(rate * now.value() + sigma * rng.nextGaussian() + glitch);
    }
    AngularVelocity imuYawRate(AngularVelocity truth, Time, shulib::sim::Rng&) override {
        return AngularVelocity{truth.value() + rate};
    }
};

}  // namespace

// ═══ R1. The hostile IMU: white heading noise and a drawn per-boot bias ═════════════
// Bug caught: a drift taken as end minus start (its σ is √2·σ/T, √(N/6) ≈ 32× the slope's here), a
// heading σ about the MEAN (a 10 °/min drift inflates it to degrees), a Turned judged from the IMU
// (whose apparent rotation at rest IS the drift). Bounds, each derived from the sample:
//   * the slope's σ: σ_b = σ·√12/(√N·T) = 8.7e-4·3.46/(√6001·60) ≈ 6.5e-7 rad/s — 4σ_b;
//   * σ_line's relative sampling σ ≈ 1/√(2N) ≈ 0.9 % — 4× that;
//   * ρ = σ_Δ/(√2·σ_line): σ_Δ's relative σ is √(3/N)/2 ≈ 1.1 % (the steps are MA(1) with lag-1
//     correlation −½), σ_line's 0.9 %; at most their sum, 2 % — 4× that, 8 %;
//   * the yaw-rate mean: σ_r/√N = 8.7e-3/77 ≈ 1.1e-4 rad/s — 4×.
// EIGHT seeds, not one (the resumption of 2026-09-29, the R5a log): with seed 3 alone the mutation
// "drift as end minus start" SURVIVED — that seed's end-minus-start error happened to be 2.05 σ_b, a
// one-in-ten draw for a 32× wider estimator. A sampling-bound check on one realization is not
// adversarial. Measured over seeds 1–12 (scratch probe r1_probe.cpp): the slope's error is at most
// 2.18 σ_b; end minus start's is 8.6–41 σ_b on every seed but 3.
TEST_CASE("RestCapture R1: under the hostile IMU the drift is the drawn bias within 4 sigma, the "
          "noise within its bounds, rho near 1, and no Turned -- over eight seeds") {
    for (std::uint64_t seed = 1; seed <= 8; ++seed) {
        CAPTURE(seed);
        ImuHostileConfig ic;
        ic.rateBiasMax = AngularVelocity{10.0 * kDeg / 60.0};  // 10 °/min
        ImuHostileModel imu{ic};
        Rig rig{&imu, 0.0, RestCaptureConfig{}, seed};
        REQUIRE(run(rig) == RestPhase::Done);
        const RestSummary& s = rig.cap->summary();
        const double bias = imu.rateBiasRadPerS();
        const double n = static_cast<double>(s.samples);
        const double sigma = ic.headingNoiseSigmaRad;
        const double sigmaB = sigma * std::sqrt(12.0) / (std::sqrt(n) * s.durationS);
        CAPTURE(bias);
        CAPTURE(s.driftRadPerS);
        CHECK(std::abs(s.driftRadPerS - bias) <= 4.0 * sigmaB);
        CHECK(s.driftSigmaWhite == rel(sigmaB, 0.05));
        CHECK(s.headingSigmaRad == rel(sigma, 4.0 / std::sqrt(2.0 * n)));
        CHECK(s.structureRatioDefined());
        CHECK(s.structureRatio == rel(1.0, 0.08));
        CHECK(std::abs(s.yawRateMean - bias) <= 4.0 * ic.yawRateNoiseSigmaRadPerS / std::sqrt(n));
        CHECK(s.yawRateSigma == rel(ic.yawRateNoiseSigmaRadPerS, 0.04));
        CHECK(s.durationS >= 60.0);  // the capture ends on the first sample at or past 60 s
        CHECK(s.durationS < 60.0 + 1.5 * kDt);
        CHECK((s.samples == 6001U || s.samples == 6002U));
        CHECK(s.timing.count == s.samples - 1U);
        CHECK(s.timing.meanS == rel(kDt, 1e-9));
        CHECK(s.speedSigma[0] == 0.0);  // the plant's encoders at rest are exact
        CHECK(s.speedSigma[1] == 0.0);
        // The encoders saw nothing: no rotation, no translation, so no bias bound.
        CHECK(s.encoderTurnRad == 0.0);
        CHECK(s.encoderTurnMaxRad == 0.0);
        CHECK(s.encoderMoveMaxIn == 0.0);
        CHECK(s.driftTurnBiasBound() == 0.0);
        CHECK(s.maxHeadingStepRad > 2.0 * sigma);
        CHECK(s.maxHeadingStepRad < 10.0 * sigma);
        CHECK(rig.cap->refusal() == RestRefusal::None);
        CHECK(rig.safe());
        if (seed == 3) {
            // Seed 3 draws −7.7 °/min (seeds 1–8 draw 1.3–7.7 in magnitude): the IMU "turns" over
            // 5° in the minute, and nothing calls that a turn (Turned is the ENCODERS').
            REQUIRE(std::abs(bias) > 5.0 * kDeg / 60.0);
            CHECK(std::abs(s.driftDegPerMin()) > 5.0);
        }
    }
}

// ═══ R2. A random-walk heading ══════════════════════════════════════════════════════
// Bug caught: settleFloorReport judged on √2·σ_line/dt — right only for WHITE heading noise. Under
// a walk σ_line is the walk's wander about its line (its expectation over 6000 steps is about
// √(6000/15) = 20 steps' σ), so that formula reads ~0.2 rad/s against a true per-tick rate noise of
// ~0.009 (A4 register HA-173). Eight seeded walks, because a walk's wander varies a lot between
// realizations: the formula reads 0.14–0.29 rad/s over them and would warn FALSELY against 0.30 in
// seven of the eight — pinned. Bounds: σ_Δ is the white step's own σ, relative sampling σ
// 1/√(2N) — 4×; ρ ≈ 1/(√2·√(N/15)) ≈ 0.035 for a walk of N steps — asserted < 0.1.
TEST_CASE("RestCapture R2: under a random-walk heading the per-tick rate noise is the walk's, rho "
          "is far under 1, and the settle floor does not warn") {
    const shulib::motion::MotionConfig mc;
    int whiteWouldWarn = 0;
    for (std::uint64_t seed = 1; seed <= 8; ++seed) {
        CAPTURE(seed);
        WalkImu imu;
        imu.stepSigma = 0.5 * kDeg * kDt;  // HA-22: 0.5 °/s per 10 ms sample, integrated
        Rig rig{&imu, 0.0, RestCaptureConfig{}, seed};
        REQUIRE(run(rig) == RestPhase::Done);
        const RestSummary& s = rig.cap->summary();
        const double n = static_cast<double>(s.samples);
        const double walkRate = imu.stepSigma / kDt;  // 8.7e-3 rad/s
        CHECK(s.stepRateSigma == rel(walkRate, 4.0 / std::sqrt(2.0 * n)));
        CHECK(s.structureRatioDefined());
        CHECK(s.structureRatio < 0.1);
        CHECK(s.driftSigmaWalk * 60.0 / kDeg == rel(0.42, 0.05));  // °/min
        CHECK(std::abs(s.driftRadPerS) < 5.0 * s.driftSigmaWalk);
        const SettleFloorReport r = settleFloorReport(s, mc);
        CHECK(r.headingRateNoise == s.stepRateSigma);
        CHECK(r.headingRateFloor == mc.headingSettle.maxErrorRate);
        CHECK_FALSE(r.headingWarn);
        CHECK(r.headingMargin > 20.0);
        const double whiteFormula = std::sqrt(2.0) * s.headingSigmaRad / kDt;
        CAPTURE(whiteFormula);
        CHECK(whiteFormula > 10.0 * s.stepRateSigma);  // it overstates the rate noise badly …
        whiteWouldWarn += mc.headingSettle.maxErrorRate / whiteFormula < 2.0 ? 1 : 0;
    }
    CHECK(whiteWouldWarn == 7);  // … and would have warned falsely in 7 of these 8 walks
}

// ═══ R3. Across ±π ══════════════════════════════════════════════════════════════════
// Bug caught: the regression run on the WRAPPED heading — a capture that crosses ±π would jump by
// 2π mid-line and report a drift of radians per second.
TEST_CASE("RestCapture R3: a capture straddling +-pi (from 179.9 deg) measures the same drift as "
          "one from 0 deg") {
    const double rate = 10.0 * kDeg / 60.0;
    double drift[2] = {0.0, 0.0};
    const double start[2] = {0.0, 179.9};
    for (int i = 0; i < 2; ++i) {
        DriftImu imu;
        imu.rate = rate;
        imu.sigma = 0.05 * kDeg;
        Rig rig{&imu, start[i]};
        REQUIRE(run(rig) == RestPhase::Done);
        drift[i] = rig.cap->summary().driftRadPerS;
    }
    CHECK(drift[1] == rel(drift[0], 1e-9));
    CHECK(drift[0] == rel(rate, 0.01));
}

// ═══ R4. Moved, Turned, ImuNotReady ═════════════════════════════════════════════════
// Bug caught: a push not refused, a turn reported as a move (or missed), a dead IMU sampled as if
// alive, a refusal that leaves a motor driven or braked — and Moved or Turned judged in ONE
// direction only (the R5a review of 2026-10-04: a `turn > bound` without the |·| passed every case,
// because every push went forward and every turn counter-clockwise).
TEST_CASE("RestCapture R4: a push either way is Moved, a turn in place either way is Turned (not "
          "Moved), an IMU dropout is ImuNotReady; each leaves 0 V + Coast") {
    for (const double dir : {1.0, -1.0}) {
        CAPTURE(dir);
        SUBCASE("push") {
            Rig rig;
            const RestPhase end = run(rig, [dir](Rig& r, int) {
                if (r.cap->phase() == RestPhase::Capture && r.cap->captureSeconds() > 1.0) {
                    r.push(0, 2.0 * dir);
                    r.push(1, 2.0 * dir);
                }
            });
            REQUIRE(end == RestPhase::Aborted);
            CHECK(rig.cap->refusal() == RestRefusal::Moved);
            CHECK(rig.safe());
            CHECK(std::strstr(rig.cap->reason(), "moved") != nullptr);
            CHECK(std::strstr(rig.cap->reason(), dir > 0.0 ? "moved 0." : "moved -0.") != nullptr);
        }
        SUBCASE("turn in place") {
            Rig rig;
            const RestPhase end = run(rig, [dir](Rig& r, int) {
                if (r.cap->phase() == RestPhase::Capture && r.cap->captureSeconds() > 1.0) {
                    r.push(0, -2.0 * dir);  // dir +1: counter-clockwise; −1: clockwise
                    r.push(1, 2.0 * dir);
                }
            });
            REQUIRE(end == RestPhase::Aborted);
            CHECK(rig.cap->refusal() == RestRefusal::Turned);
            CHECK(rig.safe());
            CHECK(std::strstr(rig.cap->reason(), "turned") != nullptr);
            CHECK(std::strstr(rig.cap->reason(), dir > 0.0 ? "turned 3." : "turned -3.") != nullptr);
        }
    }
    SUBCASE("IMU dropout") {
        ImuHostileConfig ic;
        ic.rateBiasMax = AngularVelocity{0.0};
        ic.dropoutAt = Time{20.0};
        ImuHostileModel imu{ic};
        Rig rig{&imu};
        REQUIRE(run(rig) == RestPhase::Aborted);
        CHECK(rig.cap->refusal() == RestRefusal::ImuNotReady);
        CHECK(rig.cap->captureSeconds() > 15.0);  // mid-capture, not at the wait
        CHECK(rig.safe());
    }
}

// ═══ R5. Rest that never comes; an IMU that never readies ═══════════════════════════
TEST_CASE("RestCapture R5: a motor that never rests refuses RestTimeout naming it; an IMU that "
          "never readies refuses ImuNotReady") {
    SUBCASE("a motor that reads motion") {
        Rig rig;
        const RestPhase end = run(rig, [](Rig& r, int) {
            r.member(1, 2).setVelocity(AngularVelocity{0.3});  // port 18: a frozen, moving reading
        });
        REQUIRE(end == RestPhase::Aborted);
        CHECK(rig.cap->refusal() == RestRefusal::RestTimeout);
        CHECK(rig.cap->refusalPort() == 18);
        CHECK(std::strstr(rig.cap->reason(), "port 18") != nullptr);
        CHECK(rig.safe());
    }
    SUBCASE("an IMU that never readies") {
        ImuHostileConfig ic;
        ic.calibrationEnd = Time{1e9};
        ImuHostileModel imu{ic};
        Rig rig{&imu};
        REQUIRE(run(rig) == RestPhase::Aborted);
        CHECK(rig.cap->refusal() == RestRefusal::ImuNotReady);
        CHECK(rig.h.clock().now().value() > 5.0);  // imuReadyTimeout, and not much later
        CHECK(rig.h.clock().now().value() < 5.0 + 2.5 * kDt);
        CHECK(rig.safe());
    }
}

// ═══ R6. Abort in every phase; start() leaves the robot pushable ═════════════════════
TEST_CASE("RestCapture R6: start() leaves every member at 0 V + Coast; abort() in every phase does "
          "too") {
    const RestPhase at[] = {RestPhase::WaitImu, RestPhase::Settle, RestPhase::Capture, RestPhase::Done};
    for (RestPhase target : at) {
        CAPTURE(static_cast<int>(target));
        ImuHostileConfig ic;  // a calibration window, so WaitImu lasts
        ImuHostileModel imu{ic};
        Rig rig{&imu};
        rig.push(0, 5.0);
        rig.member(1, 3).setBrakeMode(BrakeMode::Hold);
        rig.cap->start();
        CHECK(rig.safe());
        bool aborted = false;
        for (int i = 0; i < 20000; ++i) {
            if (rig.cap->phase() == target) {
                rig.cap->abort("operator touched ABORT");
                aborted = true;
                break;
            }
            rig.cap->tick();
            rig.h.plant().step(Time{kDt});
        }
        REQUIRE(aborted);
        CHECK(rig.cap->phase() == RestPhase::Aborted);
        CHECK(rig.cap->refusal() == RestRefusal::Aborted);
        CHECK(rig.safe());
        CHECK_THROWS_AS((void)rig.cap->summary(), PreconditionError);
    }
}

// ═══ R7. settleFloorReport at its edges ═════════════════════════════════════════════
TEST_CASE("RestCapture R7: settleFloorReport's margins are floor over noise; exactly the warning "
          "factor does not warn, under it does; zero noise is an infinite margin") {
    shulib::motion::MotionConfig mc;
    mc.headingSettle.maxErrorRate = 0.3;
    mc.brakeSettle.maxError = 1.2;
    RestSummary s;
    s.stepRateSigma = 0.15;
    s.speedSigma[0] = 0.6;
    s.speedSigma[1] = 0.25;
    SettleFloorReport r = settleFloorReport(s, mc);
    CHECK(r.headingMargin == 2.0);
    CHECK_FALSE(r.headingWarn);
    CHECK(r.speedNoise == 0.6);  // the larger side
    CHECK(r.speedMargin == 2.0);
    CHECK_FALSE(r.speedWarn);
    s.stepRateSigma = std::nextafter(0.15, 1.0);
    s.speedSigma[1] = std::nextafter(0.6, 1.0);  // now the RIGHT side is the larger
    r = settleFloorReport(s, mc);
    CHECK(r.headingWarn);
    CHECK(r.speedWarn);
    CHECK(r.speedNoise == std::nextafter(0.6, 1.0));
    s = RestSummary{};
    r = settleFloorReport(s, mc);
    CHECK(std::isinf(r.headingMargin));
    CHECK(std::isinf(r.speedMargin));
    CHECK_FALSE(r.headingWarn);
    CHECK_FALSE(r.speedWarn);
    s.stepRateSigma = 0.1;  // margin 3 — warns only at a stricter factor
    CHECK_FALSE(settleFloorReport(s, mc).headingWarn);
    CHECK(settleFloorReport(s, mc, 4.0).headingWarn);
}

// ═══ R8. The config and the preconditions ═══════════════════════════════════════════
TEST_CASE("RestCapture R8: RestCaptureConfig::valid() names what it refuses; the constructor "
          "enforces it and the sides' shape") {
    char why[160];
    CHECK(RestCaptureConfig{}.valid(why, sizeof why));
    CHECK(RestCaptureConfig{}.restFloor.value() == shulib::calibration::CalibratorConfig{}.restFloor.value());
    auto bad = [&](auto set, const char* word) {
        RestCaptureConfig c;
        set(c);
        CHECK_FALSE(c.valid(why, sizeof why));
        CHECK(std::strstr(why, word) != nullptr);
    };
    bad([](RestCaptureConfig& c) { c.duration = Time{0.0}; }, "duration");
    bad([](RestCaptureConfig& c) { c.moveBound = Length{-1.0}; }, "moveBound");
    bad([](RestCaptureConfig& c) { c.settleMarginWarn = 0.5; }, "settleMarginWarn");
    bad([](RestCaptureConfig& c) { c.restTimeout = c.restSettleTime; }, "restTimeout");
    bad([](RestCaptureConfig& c) { c.timing.budget = c.timing.nominal; }, "timing");

    Rig rig;
    auto make = [&](std::span<const SideMember> l, std::span<const SideMember> r, double w,
                    const RestCaptureConfig& cfg = {}) {
        RestCapture c{DriveSide{l, Length{kR}}, DriveSide{r, Length{kR}}, Length{w}, rig.h.imu(),
                      rig.h.clock(), cfg};
        (void)c;
    };
    const std::span<const SideMember> L{rig.left.data(), rig.left.size()};
    const std::span<const SideMember> R{rig.right.data(), rig.right.size()};
    CHECK_NOTHROW(make(L, R, 12.5));
    CHECK_THROWS_AS(make(L, R, -1.0), PreconditionError);
    CHECK_THROWS_AS(make(L, L, 12.5), PreconditionError);
    CHECK_THROWS_AS(make(L, R, 12.5, RestCaptureConfig{Time{0.0}}), PreconditionError);
    std::array<SideMember, 5> badPort = rig.left;
    badPort[0].port = 0;
    CHECK_THROWS_AS(make(std::span<const SideMember>{badPort.data(), 5}, R, 12.5), PreconditionError);
    CHECK_THROWS_AS(
        RestCapture(DriveSide{L, Length{0.0}}, DriveSide{R, Length{kR}}, Length{12.5}, rig.h.imu(),
                    rig.h.clock()),
        PreconditionError);
    rig.cap->start();
    CHECK_THROWS_AS(rig.cap->start(), PreconditionError);
}

namespace {

/// Encoder readings scripted on top of a still robot: from 10 s to 11 s each wheel's reading moves
/// linearly to `travel[w]` inches of the side's surface (shaft offset = travel / the true radius),
/// then holds — the plant writes it into every member of the wheel. The robot does NOT move: the
/// IMU sees nothing, so this exercises the bounds, not the drift.
struct ScriptedEncoders final : shulib::sim::DegradationModel {
    double travel[2] = {0.0, 0.0};
    shulib::units::AngleDim driveEncoderPosition(int wheel, shulib::units::AngleDim trueShaft,
                                                 Time now, shulib::sim::Rng&) override {
        const double f = std::clamp(now.value() - 10.0, 0.0, 1.0);
        return shulib::units::AngleDim{trueShaft.value()
                                       + f * travel[static_cast<std::size_t>(wheel)] / kR};
    }
};

}  // namespace

// ═══ R10. Moved and Turned at their bounds, both directions ═════════════════════════
// Bug caught (the R5a review of 2026-10-04): R4's open-ended pushes refuse at ANY threshold, so a
// Turned computed on 2W (refusing at ~6°, not 3°) or a Moved on a tenth of the travel survived. Here
// the encoders read a translation or a rotation 1 % UNDER its bound — Done, with the summary
// recording exactly that motion — and 1 % OVER it — refused by name. Both directions of each.
TEST_CASE("RestCapture R10: encoder motion 1 pct under moveBound or turnBound finishes and is "
          "recorded; 1 pct over refuses -- either direction") {
    const RestCaptureConfig cfg;
    const double move = cfg.moveBound.value();
    const double turn = cfg.turnBound.value();
    struct Case {
        const char* what;
        double l;
        double r;
        RestRefusal over;
    };
    for (const double dir : {1.0, -1.0}) {
        for (const double frac : {0.99, 1.01}) {
            // Translation: both sides by x = frac·moveBound (centre travel x, rotation 0).
            // Rotation: a turn in place, each side by frac·turnBound·W/2 (rotation frac·turnBound,
            // centre travel 0) — dir +1 counter-clockwise (RIGHT forward), −1 clockwise.
            const double x = frac * move * dir;
            const double y = frac * turn * kW / 2.0 * dir;
            for (const Case k : {Case{"translation", x, x, RestRefusal::Moved},
                                 Case{"rotation", -y, y, RestRefusal::Turned}}) {
                CAPTURE(k.what);
                CAPTURE(dir);
                CAPTURE(frac);
                ScriptedEncoders enc;
                enc.travel[0] = k.l;
                enc.travel[1] = k.r;
                Rig rig{&enc};
                const RestPhase end = run(rig);
                if (frac < 1.0) {
                    REQUIRE(end == RestPhase::Done);
                    const RestSummary& s = rig.cap->summary();
                    const double wantTurn = (k.r - k.l) / kW;
                    CHECK(s.encoderTurnRad == doctest::Approx(wantTurn).epsilon(1e-9).scale(1e-12));
                    CHECK(s.encoderTurnMaxRad
                          == doctest::Approx(std::abs(wantTurn)).epsilon(1e-9).scale(1e-12));
                    CHECK(s.encoderMoveMaxIn
                          == doctest::Approx(std::abs(0.5 * (k.l + k.r))).epsilon(1e-9).scale(1e-12));
                    CHECK(s.driftTurnBiasBound()
                          == doctest::Approx(3.0 * std::abs(wantTurn) / s.durationS).scale(1e-15));
                    // The IMU saw nothing (the readings moved, the robot did not): no drift.
                    CHECK(std::abs(s.driftRadPerS) < 1e-12);
                } else {
                    REQUIRE(end == RestPhase::Aborted);
                    CHECK(rig.cap->refusal() == k.over);
                    CHECK(rig.safe());
                }
            }
        }
    }
}

// ═══ R11. A real nudge under the bounds: the drift it biases, and the record of it ════
// Bug caught (the R5a review of 2026-10-04): a rotation UNDER turnBound is not refused — a hand
// that turns the robot turns the IMU with it — and the drift's slope absorbs it with no trace in
// the summary. The bias of one step θ at t0 is 6·θ·t0·(T − t0)/T³ (rest_capture.hpp's header: 1.5·θ/T
// at mid-capture). On an IMU with NO drift, the drift reported is the nudge's and nothing else; the
// summary now records the rotation the encoders saw, driftTurnBiasBound() covers the drift, and ρ is
// dragged far below the white-noise 1 (a nudge reads like random-walk evidence: HA-173).
TEST_CASE("RestCapture R11: a nudge under the bounds biases the drift by the step's formula, inside "
          "driftTurnBiasBound(), and the summary records the rotation") {
    struct Nudge {
        const char* what;
        double l;
        double r;
        int ticks;
    };
    // The plant at rest (kA 0) turns a side at (2 − 1)/(12/70) = 5.83 in/s under 2 V.
    for (const Nudge nudge : {Nudge{"LEFT pushed, a pivot", 2.0, 0.0, 8},
                              Nudge{"turn in place", -2.0, 2.0, 5},
                              Nudge{"turn in place, clockwise", 2.0, -2.0, 5}}) {
        CAPTURE(nudge.what);
        ImuHostileConfig ic;
        ic.rateBiasMax = AngularVelocity{0.0};  // no drift: anything reported is the nudge
        ic.calibrationEnd = Time{0.0};
        ImuHostileModel imu{ic};
        Rig rig{&imu};
        double t0 = -1.0;  // capture seconds at the push's first tick
        double truthBefore = 0.0;
        int pushed = 0;
        const RestPhase end = run(rig, [&](Rig& r, int) {
            const bool on = r.cap->phase() == RestPhase::Capture && r.cap->captureSeconds() >= 30.0
                            && pushed < nudge.ticks;
            if (on && pushed == 0) {
                t0 = r.cap->captureSeconds();
                truthBefore = r.h.plant().truthState().theta;
            }
            r.push(0, on ? nudge.l : 0.0);
            r.push(1, on ? nudge.r : 0.0);
            pushed += on ? 1 : 0;
        });
        REQUIRE(end == RestPhase::Done);
        const RestSummary& s = rig.cap->summary();
        const double theta = rig.h.plant().truthState().theta - truthBefore;  // plant truth
        const double T = s.durationS;
        CAPTURE(theta / kDeg);
        CAPTURE(s.driftDegPerMin());
        REQUIRE(std::abs(theta) > 2.0 * kDeg);                     // a real nudge …
        REQUIRE(std::abs(theta) < rig.cap->config().turnBound.value());  // … under the bound
        // The record: the rotation the encoders saw IS the plant's (a tank, wheels pinned or
        // turning in place), at the end and at its largest.
        CHECK(s.encoderTurnRad == rel(theta, 1e-6));
        CHECK(s.encoderTurnMaxRad == rel(std::abs(theta), 1e-6));
        CHECK(s.encoderMoveMaxIn < rig.cap->config().moveBound.value());
        // The drift is the nudge's step formula (the push spans 0.05–0.08 s around t0: within 1 %)…
        const double step = 6.0 * theta * t0 * (T - t0) / (T * T * T);
        CHECK(s.driftRadPerS == rel(step, 0.01));
        // … many times HA-20's whole 1 °/min bound, and inside the bound the summary states.
        CHECK(std::abs(s.driftDegPerMin()) > 2.5);
        CHECK(std::abs(s.driftRadPerS) <= s.driftTurnBiasBound());
        CHECK(s.driftTurnBiasBound() == rel(3.0 * std::abs(theta) / T, 1e-6));
        // ρ is dragged down by the step (white noise alone gives ~1, R1).
        CHECK(s.structureRatioDefined());
        CHECK(s.structureRatio < 0.2);
    }
}

// ═══ R12. Encoder noise at rest, per side and in each side's own scale ══════════════
// Bug caught (the R5a review of 2026-10-04): the only assertion on the encoder speed σ was 0 on a
// plant whose encoders are exact, so the channel could be zeroed — and settleFloorReport's brake
// margin would then always be infinite — or the RIGHT side read in the LEFT side's scale. Here every
// member's velocity is jittered, uniformly and under the rest floor, with a different spread per
// side, and the RIGHT side's description scale is 1.3× the LEFT's. The reference is computed HERE:
// each tick the capture samples, the median of each side's five readings × that side's scale, then a
// two-pass sample σ.
TEST_CASE("RestCapture R12: the encoder speed sigma at rest is each side's own, in its own scale, "
          "and settleFloorReport takes the larger") {
    Rig rig{nullptr, 0.0, RestCaptureConfig{}, 1, 1.3};
    shulib::sim::Rng rng{2026};
    const double spread[2] = {0.01, 0.03};  // rad/s, uniform ±: always under restFloor (0.05)
    std::vector<double> seen[2];
    RestCapture& c = *rig.cap;
    c.start();
    for (int i = 0; i < 20000; ++i) {
        // What this tick will read: the plant wrote 0; the jitter rides on it.
        for (int w = 0; w < 2; ++w) {
            for (std::size_t k = 0; k < Rig::kPer; ++k) {
                const double u = 2.0 * rng.nextUnit() - 1.0;
                rig.member(w, k).setVelocity(AngularVelocity{spread[w] * u});
            }
        }
        double median[2];
        for (std::size_t s = 0; s < 2; ++s) {
            std::array<double, Rig::kPer> v{};
            for (std::size_t k = 0; k < Rig::kPer; ++k) {
                v[k] = rig.member(static_cast<int>(s), k).velocity().value();
            }
            std::sort(v.begin(), v.end());
            median[s] = v[Rig::kPer / 2] * rig.ipr[s];
        }
        const RestPhase before = c.phase();
        c.tick();
        const RestPhase after = c.phase();
        if ((after == RestPhase::Capture || after == RestPhase::Done) && before != RestPhase::Done) {
            seen[0].push_back(median[0]);  // this tick sampled
            seen[1].push_back(median[1]);
        }
        if (after == RestPhase::Done || after == RestPhase::Aborted) {
            break;
        }
        rig.h.plant().step(Time{kDt});
    }
    REQUIRE(c.phase() == RestPhase::Done);
    const RestSummary& s = c.summary();
    REQUIRE(seen[0].size() == s.samples);
    double sigma[2];
    for (std::size_t side = 0; side < 2; ++side) {
        double mean = 0.0;
        for (double x : seen[side]) {
            mean += x;
        }
        mean /= static_cast<double>(seen[side].size());
        double ss = 0.0;
        for (double x : seen[side]) {
            ss += (x - mean) * (x - mean);
        }
        sigma[side] = std::sqrt(ss / static_cast<double>(seen[side].size() - 1));
        CAPTURE(side);
        CHECK(s.speedSigma[side] == rel(sigma[side], 1e-9));
    }
    // Not vacuous: the RIGHT side is the noisier (3× the spread) AND on the larger scale (1.3×).
    CHECK(s.speedSigma[1] > 3.0 * s.speedSigma[0]);
    CHECK(s.speedSigma[0] > 0.0);
    const shulib::motion::MotionConfig mc;
    const SettleFloorReport r = settleFloorReport(s, mc);
    CHECK(r.speedNoise == s.speedSigma[1]);
    CHECK(r.speedFloor == mc.brakeSettle.maxError);
    CHECK(r.speedMargin == mc.brakeSettle.maxError / s.speedSigma[1]);
    CHECK(std::isfinite(r.speedMargin));
}

namespace {

/// An IMU whose yaw rate reads "still rotating" until `until` seconds (hands on the robot), then the
/// truth. Heading untouched.
struct HandsOn final : shulib::sim::DegradationModel {
    double until = 1e18;
    AngularVelocity imuYawRate(AngularVelocity truth, Time now, shulib::sim::Rng&) override {
        return now.value() < until ? AngularVelocity{0.2} : truth;
    }
};

/// The time of the first tick that sampled (Settle → Capture), or −1.
double captureStartOf(Rig& rig, const std::function<void(Rig&, int)>& hook = {}) {
    RestCapture& c = *rig.cap;
    c.start();
    for (int i = 0; i < 20000; ++i) {
        c.tick();
        if (c.phase() == RestPhase::Capture || c.phase() == RestPhase::Done) {
            return rig.h.clock().now().value();
        }
        if (c.phase() == RestPhase::Aborted) {
            CHECK(screenSafe(c.reason()));
            return -1.0;
        }
        rig.h.plant().step(Time{kDt});
        if (hook) {
            hook(rig, i);
        }
    }
    return -1.0;
}

}  // namespace

// ═══ R13. The Settle phase ══════════════════════════════════════════════════════════
// Bug caught (the R5a review of 2026-10-04): only "a motor that never rests" reached Settle's logic,
// so a settle judged on the motors alone (the IMU's yaw-rate half dropped), a capture that started on
// the first quiet tick (no settle time), and a settle NOT restarted by a moving tick all survived.
// The GAINS step's C13 pins the same rest definition; this is its REST twin.
TEST_CASE("RestCapture R13: the capture waits until the yaw rate is quiet AND rest has held "
          "restSettleTime; one moving tick restarts it; a rotation that never stops refuses") {
    const RestCaptureConfig cfg;
    const double settle = cfg.restSettleTime.value();
    SUBCASE("a yaw rate reading rotation until 1.2 s") {
        HandsOn hands;
        hands.until = 1.2;
        Rig rig{&hands};
        const double t = captureStartOf(rig);
        CAPTURE(t);
        CHECK(t >= 1.2 + settle - 1e-9);
        CHECK(t < 1.2 + settle + 3.0 * kDt);
    }
    SUBCASE("one moving tick mid-settle restarts the settle") {
        Rig rig;
        double movedAt = -1.0;
        const double t = captureStartOf(rig, [&](Rig& r, int) {
            const double now = r.h.clock().now().value();
            const bool on = movedAt < 0.0 && now >= 0.3;  // one tick, 0.3 s into a 0.5 s settle
            r.member(0, 1).setVelocity(AngularVelocity{on ? 0.3 : 0.0});
            if (on) {
                movedAt = now;
            }
        });
        CAPTURE(t);
        REQUIRE(movedAt > 0.0);
        CHECK(t >= movedAt + settle - 1e-9);
        CHECK(t < movedAt + settle + 3.0 * kDt);
    }
    SUBCASE("a rotation that never stops: RestTimeout, still rotating, no port") {
        HandsOn hands;
        Rig rig{&hands};
        CHECK(captureStartOf(rig) < 0.0);
        CHECK(rig.cap->refusal() == RestRefusal::RestTimeout);
        CHECK(rig.cap->refusalPort() == 0);
        CHECK(std::strstr(rig.cap->reason(), "still rotating") != nullptr);
        CHECK(rig.safe());
    }
}

// ═══ R14. A quiet plant, and an IMU glitch ═════════════════════════════════════════
// Bug caught (the R5a review of 2026-10-04): ρ = σ_Δ/(√2·σ_line) is 0/0 on a heading with no noise,
// and was reported as 0 — "a random walk" — with nothing to say it meant nothing; and the largest
// heading step kept SIGNED survived every case (white noise's largest step is as likely + as −).
// A one-way −3° jump at 30 s (an IMU glitch; the encoders see nothing) must be the largest step by
// its MAGNITUDE. (Its effect on the drift is HA-160's, not a nudge's: driftTurnBiasBound() is 0
// because the encoders saw nothing — the largest step is the evidence for it.)
TEST_CASE("RestCapture R14: on a quiet plant the structure ratio is undefined, not a random walk; a "
          "one-way IMU jump is the largest step by magnitude") {
    SUBCASE("quiet") {
        Rig rig;
        REQUIRE(run(rig) == RestPhase::Done);
        const RestSummary& s = rig.cap->summary();
        CHECK(s.headingSigmaRad == 0.0);
        CHECK_FALSE(s.structureRatioDefined());
        CHECK(s.structureRatio == 0.0);
        CHECK(s.driftRadPerS == 0.0);
        CHECK(s.maxHeadingStepRad == 0.0);
    }
    SUBCASE("a one-way jump of -3 deg") {
        DriftImu imu;
        imu.sigma = 0.05 * kDeg;
        imu.jump = -3.0 * kDeg;
        imu.jumpAt = 30.0;
        Rig rig{&imu};
        REQUIRE(run(rig) == RestPhase::Done);
        const RestSummary& s = rig.cap->summary();
        CHECK(s.maxHeadingStepRad == rel(3.0 * kDeg, 0.1));
        CHECK(s.encoderTurnMaxRad == 0.0);
        CHECK(s.driftTurnBiasBound() == 0.0);
        CHECK(s.driftRadPerS < -1.5 * 3.0 * kDeg / s.durationS * 0.9);  // the glitch's own bias
    }
}

// ═══ R15. A side with no finite reading ════════════════════════════════════════════
// Bug caught (found 2026-10-04 while answering the R5a review's non-finite finding): a whole
// side's velocity() NaN on ONE capture tick finished the capture Done with speedSigma NaN — a
// summary its own valid() refuses — because the median of five NaN is NaN and went straight into
// the σ. (One NaN member among finite ones is DIAG1 Part B's medianInPlace rule — left out of the
// median — which this base predates; the whole-side case is a NaN in either tree.) Here the RIGHT
// side's velocity() reads NaN on five capture ticks around 30 s, and the LEFT side's position()
// reads NaN on every tick from 59.5 s to the end — so the capture's LAST tick has no travel. On a
// quiet plant every number of the summary must equal a clean capture's, valid() must accept it, and
// unreadSides() must count exactly the readings left out.
TEST_CASE("RestCapture R15: a side with no finite reading on a tick is left out and counted; the "
          "summary stays finite, valid and equal to a clean capture's") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Rig clean;
    REQUIRE(run(clean) == RestPhase::Done);
    Rig rig;
    std::uint32_t poisoned = 0;
    const RestPhase end = run(rig, [&](Rig& r, int) {
        if (r.cap->phase() != RestPhase::Capture) {
            return;
        }
        const double t = r.cap->captureSeconds();  // the tick just taken; the next is t + dt
        if (t >= 30.0 && t < 30.0 + 4.5 * kDt) {
            for (std::size_t k = 0; k < Rig::kPer; ++k) {
                r.member(1, k).setVelocity(AngularVelocity{nan});
            }
            ++poisoned;
        }
        if (t >= 59.5) {
            for (std::size_t k = 0; k < Rig::kPer; ++k) {
                r.member(0, k).setPosition(shulib::units::AngleDim{nan});
            }
            ++poisoned;
        }
    });
    REQUIRE(end == RestPhase::Done);
    const RestSummary& s = rig.cap->summary();
    const RestSummary& c = clean.cap->summary();
    CHECK(poisoned > 50U);  // 5 speed ticks and ~50 travel ticks
    CHECK(rig.cap->unreadSides() == poisoned);
    CHECK(clean.cap->unreadSides() == 0U);
    CHECK(s.samples == c.samples);
    CHECK(std::isfinite(s.speedSigma[0]));
    CHECK(std::isfinite(s.speedSigma[1]));
    CHECK(s.speedSigma[1] == c.speedSigma[1]);
    CHECK(s.encoderTurnRad == c.encoderTurnRad);  // the last tick read no travel: nothing recorded
    CHECK(s.encoderTurnMaxRad == c.encoderTurnMaxRad);
    CHECK(s.encoderMoveMaxIn == c.encoderMoveMaxIn);
    CHECK(s.driftRadPerS == c.driftRadPerS);
    shulib::calibration::DriveCharacterization dc;
    dc.source = shulib::calibration::CharacterizationSource::Measured;
    dc.hasRest = true;
    dc.rest = s;
    char why[200] = "";
    const bool ok = dc.valid(why, sizeof why);
    const std::string whyText = why;
    CAPTURE(whyText);
    CHECK(ok);
}

// ═══ R9. The refusal codes ══════════════════════════════════════════════════════════
// Self-contained (the R5a review: it used to check reasons the other cases had collected, and failed
// when run alone). The reasons themselves are checked as they are produced, in run().
TEST_CASE("RestCapture R9: every refusal code has a short ASCII name") {
    for (int c = 0; c <= static_cast<int>(RestRefusal::Turned); ++c) {
        const std::string n = restRefusalName(static_cast<RestRefusal>(c));
        CAPTURE(n);
        CHECK(n != "?");
        CHECK(screenSafe(n));
    }
}
