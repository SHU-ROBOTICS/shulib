// calibration::DriveCharacterizer — shulib Calibrate's GAINS step — on the A2 host test plant
// (chunk R5a Part A, A.3). NOTHING HERE HAS RUN ON A ROBOT. The plant's motor model IS the law the
// step fits, so recovering its gains proves the ARITHMETIC and the safety rules, not physics (A4
// register HA-171); the plant also models neither a V5 Coast nor a V5 Brake at 0 V (HA-53, HA-143).
//
// Robot two's shape: five members a side (ports 11–15 LEFT, 16–20 RIGHT); a member wired backwards
// on each side (LEFT port 13, RIGHT port 16 — the plant's MemberFault::Reversed, sign −1 from the
// "description"); a slipping member on the RIGHT (port 19, MemberFault::Decoupled, drag 0.3). The
// sides' gains differ by about 5 % — L {1.0, 0.160, 0.020}, R {1.1, 0.168, 0.024} — and EVERY case
// runs in BOTH orientations (L faster, then R faster), as R5a's brief requires, because a left/right
// slip is invisible in a symmetric test. (C17 is the one exception: it runs the drivetrain
// calibrator's own rig, whose sides are equal by construction.)
//
// The harness run() checks EVERY tick, in every case that uses it: each member at the phase's
// command (Driving: sign × volts + Coast; Stopping: 0 V + Brake; otherwise 0 V + Coast); the
// segment's volts against its PROFILE computed here from the config and the clock (±rampRate·t,
// ±stepVolts — not the characterizer's own number); lastSample() against the motors, the IMU and
// the battery read here; and every reason and warning for screen safety. It also keeps the battery
// readings and tick counts each segment saw, so the report's numbers can be checked exactly.
//
// Each case names the bug it catches and the mutation that must turn it red:
//   C1  recovery: each side's gains to 1e-6 relative; four normal ends, each segment's direction
//       (+ − + −) and the step's volts; the result's every field is the fit's and the run's timing
//       is the report's, counted tick for tick  (swap L/R accumulation; forget the inches per
//                                                radian; admit u = 0 on the plant; drop the friction
//                                                column; StepBackward driving forward; a step at
//                                                rampMax; the ramp at twice its rate; the result's
//                                                side fields or ramp timing dropped; the run timing
//                                                not rebaselined across the rests)
//   C2  reversed members with signs from the description give the gains of a robot with none
//                                                (apply the sign twice: red)
//   C3  a slipping (decoupled) member leaves the gains unchanged      (mean instead of median)
//   C4  an inches-per-radian 10 % high moves kV and kA by exactly 1/1.1, kS not at all — on BOTH
//       sides, and on ONE side only (the other side untouched)    (a side scaled by the other's)
//   C5  the POWERED travel of each side is at most segmentDistance + that tick's travel, on plant
//       TRUTH; each side's stopping distance equals plant truth; every Stopping tick is 0 V + BRAKE
//       at every member, every other unpowered tick 0 V + COAST      (distance bound on the mean;
//       on the LEFT side only — red only R-faster; Coast instead of Brake at a normal end)
//   C6  a large asymmetry → CurveExceeded, 0 V + Coast on that tick, the message naming both sides'
//       travel, and the report keeping how far the segment got; at W = 25 in the guard is the
//       DESCRIPTION's                       (ψ_g with a fixed W: red; the refused segment not kept)
//   C7  the IMU jumping 37° a tick, alternating, in the StepBackward — an IMU spinning 90 °/s that
//       the encoders do not see — and a one-way 20° jump never refuse; the report shows all three
//                                          (curve guard from the IMU: red; the largest step signed)
//   C8  a stop longer than stopBound → StopTooLong before the next segment        (no stop bound)
//   C9  a member at tempGuard → TooHot naming the port, nothing powered; under it → runs; a member
//       that heats up during the run stops the NEXT segment; two hot members → the HOTTER is named
//                                          (temperature unchecked; the first hot member named)
//   C10 a missed tick while driving → MissedTick, 0 V + Coast; while at rest → nothing
//                                                  (no missed-tick guard)
//   C11 abort() in every phase → 0 V + Coast, the aborted segment keeping how far it got; on a
//       segment's FIRST tick, nothing inherited from the segment before (the panel's travel not
//       reset at a segment's start: red)
//   C12 a SignFlipped member → Fight naming its port, both sides
//   C13 a robot that never rests → RestTimeout, nothing powered; a disturbance at the start delays
//       the first powered tick until rest has held          (start a segment before rest: red)
//   C14 a member current over the guard → those samples excluded, counted ON ITS SIDE, the gains
//       unchanged — a LEFT member and a RIGHT member               (current guard off: red)
//   C15 an asymmetry past asymWarn but inside the guard → Done, with the warning, in "pct"
//   C16 hostility: encoder quantization + a per-member velocity jitter (invented) — the bound is
//       MEASURED, pinned no looser than 2× the measured error, both recorded
//   C17 end to end: the drivetrain calibrator's description feeds the step on the SAME plant
//   C18 the config's valid(W), its derived clear space, and the preconditions
//                                          (the sideways excursion without the stop: red)
//   C20 an IMU that never readies → ImuNotReady; a frozen reading after a segment → RestTimeout
//       "after a segment"; still rolling past stopBound → StopTooLong "still rolling"; a stop past
//       stopBound that rests → StopTooLong at rest; a fit that is not Ok → FitRefused naming each
//       side's verdict, no result — each 0 V + Coast                   (FitRefused branch skipped)
//   C21 ONE side's stop past stopBound, the other's short and the mean under it → StopTooLong
//                                          (the stop judged on the LEFT only, or on the mean: red)
//   C22 a drive that cannot move under stepVolts: the ramps end RampTop under rampMax, the steps end
//       Timeout at segmentTimeout, each braked then coasted         (no segment timeout; no ramp
//                                                                     top: red)
//   C23 late ticks while driving (20 ms, under commandTimeout) → each sample counted OffNominalDt
//       on both sides, each tick over budget, the gains exact     (the fit's dt fixed at nominal)
//   C24 a sagging battery and scripted member currents: each segment's battery min and mean are
//       exactly the readings it saw; lastSample()'s current, position and heading are the motors'
//       and the IMU's                      (battery mean = last; min = max; the sample's position,
//                                           current or heading zero)
//   C25 a segment that admitted NO sample on a side (a wall ahead; one side at its current limit
//       for a whole segment) → Done, emptySegmentWarning, warning() naming the first such segment
//       and side, the asymmetry warning kept after it      (never warned; RIGHT side unchecked; the
//                                                           last empty one named; the asymmetry
//                                                           text overwriting it: red)
//   C19 every refusal code has a name (the strings themselves are checked in run(), as they come)
// THE PLANT'S NUMBERS ARE INVENTED; so are CharacterizerConfig's (A4 register HA-170).

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "calibration_test_rig.hpp"
#include "shulib/calibration/drive_characterization.hpp"
#include "shulib/calibration/drive_characterizer.hpp"
#include "shulib/calibration/feedforward_fit.hpp"
#include "shulib/calibration/rest_capture.hpp"
#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/hostile/encoder_hostility.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::calibration::CharacterizerConfig;
using shulib::calibration::CharacterizerPhase;
using shulib::calibration::CharacterizerRefusal;
using shulib::calibration::characterizerRefusalName;
using shulib::calibration::CharacterizerSample;
using shulib::calibration::DriveCharacterization;
using shulib::calibration::DriveCharacterizer;
using shulib::calibration::DriveSide;
using shulib::calibration::FitExclusion;
using shulib::calibration::FitVerdict;
using shulib::calibration::kSegmentCount;
using shulib::calibration::SegmentEnd;
using shulib::calibration::SegmentKind;
using shulib::calibration::SegmentRecord;
using shulib::calibration::SideMember;
using shulib::control::FeedforwardGains;
using shulib::hal::BrakeMode;
using shulib::hal::fake::FakeMotor;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::sim::MemberFault;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Current;
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
constexpr double kR = 3.25 / 2.0;  // the plant's drive wheel radius: the TRUE inches per radian
constexpr FeedforwardGains kFast{.kS = 1.0, .kV = 0.160, .kA = 0.020};
constexpr FeedforwardGains kSlow{.kS = 1.1, .kV = 0.168, .kA = 0.024};

/// Which side is the faster one — every case runs both.
enum class Faster { Left, Right };
constexpr std::array<Faster, 2> kBoth{Faster::Left, Faster::Right};
[[nodiscard]] std::string name(Faster f) { return f == Faster::Left ? "L faster" : "R faster"; }

/// How a rig is built.
struct Plan {
    Faster faster = Faster::Left;
    FeedforwardGains fast = kFast;   ///< the faster side's plant gains
    FeedforwardGains slow = kSlow;   ///< the slower side's
    double trackWidth = 12.5;        ///< the plant's TRUE width, and the description's
    double iprScaleL = 1.0;          ///< the description's LEFT inches per radian over the true radius
    double iprScaleR = 1.0;          ///< the same, RIGHT
    bool reversed = true;            ///< robot two's backwards members (sign −1 in the description)
    bool decoupled = true;           ///< a slipping RIGHT member, drag 0.3
    shulib::sim::DegradationModel* degradation = nullptr;
    CharacterizerConfig cfg{};

    [[nodiscard]] FeedforwardGains leftGains() const { return faster == Faster::Left ? fast : slow; }
    [[nodiscard]] FeedforwardGains rightGains() const { return faster == Faster::Left ? slow : fast; }
};

[[nodiscard]] SimHarnessConfig plantFor(const Plan& p) {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = p.leftGains();          // wheel 0 (LEFT) on the shared slot …
    cfg.plant.wheelFfOverride[1] = p.rightGains();  // … and wheel 1 (RIGHT) on its own
    cfg.plant.decoupledDrag = p.decoupled ? 0.3 : 0.0;
    return cfg;
}

/// Robot two's shape on the plant, and a characterizer over it.
struct Rig {
    static constexpr std::size_t kPer = 5;
    TankKinematics kin;
    SimHarness h;
    std::array<FakeMotor, 2 * (kPer - 1)> extras{};
    std::array<SideMember, kPer> left{};
    std::array<SideMember, kPer> right{};
    double ipr[2] = {kR, kR};  ///< the DESCRIPTION's inches per radian, per side
    std::optional<DriveCharacterizer> ch;

    explicit Rig(const Plan& p)
        : kin{Length{p.trackWidth}}, h{kin, plantFor(p), nullptr, p.degradation} {
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
        if (p.reversed) {
            h.plant().setMemberFault(0, 2, MemberFault::Reversed);  // port 13
            left[2].sign = -1;
            h.plant().setMemberFault(1, 0, MemberFault::Reversed);  // port 16 — the primary
            right[0].sign = -1;
        }
        if (p.decoupled) {
            h.plant().setMemberFault(1, 3, MemberFault::Decoupled);  // port 19
        }
        ipr[0] = kR * p.iprScaleL;
        ipr[1] = kR * p.iprScaleR;
        ch.emplace(DriveSide{std::span<const SideMember>{left.data(), left.size()}, Length{ipr[0]}},
                   DriveSide{std::span<const SideMember>{right.data(), right.size()}, Length{ipr[1]}},
                   Length{p.trackWidth}, h.imu(), h.clock(), h.battery(), p.cfg);
    }

    [[nodiscard]] FakeMotor& member(int wheel, std::size_t k) {
        return k == 0 ? h.motor(wheel)
                      : extras[static_cast<std::size_t>(wheel) * (kPer - 1) + (k - 1)];
    }
    [[nodiscard]] FakeMotor& port(int p) {
        return p <= 15 ? member(0, static_cast<std::size_t>(p - 11))
                       : member(1, static_cast<std::size_t>(p - 16));
    }
    [[nodiscard]] const std::array<SideMember, kPer>& side(std::size_t s) const {
        return s == 0 ? left : right;
    }
};

/// The median over a side's members of sign × `read(motor)`, times the side's scale — computed
/// HERE, independently of rest_capture.hpp's helpers (five members: the middle of a sorted copy).
template <typename Read>
[[nodiscard]] double sideMedian(const Rig& rig, std::size_t s, Read read) {
    std::array<double, Rig::kPer> v{};
    for (std::size_t k = 0; k < Rig::kPer; ++k) {
        const SideMember& m = rig.side(s)[k];
        v[k] = static_cast<double>(m.sign) * read(*m.motor);
    }
    std::sort(v.begin(), v.end());
    return v[Rig::kPer / 2] * rig.ipr[s];
}

/// Printable ASCII and no '%' (drivetrain_calibrator.hpp's screen rule).
[[nodiscard]] bool screenSafe(const std::string& s) {
    for (char c : s) {
        const auto u = static_cast<unsigned char>(c);
        if (u < 0x20 || u > 0x7E || c == '%') {
            return false;
        }
    }
    return true;
}

/// What a run did, measured from the TRUTH side of the plant and at the motors.
struct Out {
    CharacterizerPhase end = CharacterizerPhase::Idle;
    int ticks = 0;
    double powered[4][2]{};      ///< true travel over powered intervals, per segment and side
    double lastPowered[4][2]{};  ///< the last powered interval's true travel
    double stop[4][2]{};         ///< true travel over stopping intervals
    int brakeViolations = 0;     ///< Stopping ticks with a member not at 0 V + Brake
    int coastViolations = 0;     ///< unpowered, non-stopping ticks with a member not at 0 V + Coast
    int drivingViolations = 0;   ///< Driving ticks with a member not at sign × volts, Coast
    int profileViolations = 0;   ///< Driving ticks whose volts are not the segment's profile
    int sampleViolations = 0;    ///< Driving/Stopping ticks whose lastSample() disagrees with the hardware
    int brakeTicks = 0;
    int poweredTicks = 0;
    int firstPoweredTick = -1;
    int segTicks[4]{};           ///< ticks that added a dt to each segment's timing
    double battMin[4]{};         ///< the battery readings each segment saw: least, sum, count
    double battSum[4]{};
    int battN[4]{};
};

/// The volts segment `kind` must command `elapsed` seconds in — computed from the CONFIG, not read
/// back from the characterizer.
[[nodiscard]] double profileVolts(const CharacterizerConfig& cfg, SegmentKind kind, double elapsed) {
    switch (kind) {
        case SegmentKind::QuasiForward: return cfg.rampRate * elapsed;
        case SegmentKind::QuasiBackward: return -cfg.rampRate * elapsed;
        case SegmentKind::StepForward: return cfg.stepVolts;
        case SegmentKind::StepBackward: return -cfg.stepVolts;
    }
    return 0.0;
}

/// Drive a run to its end: `hook` runs before each tick (fault injection); after each tick the
/// motors, the volts profile and the sample are checked against the phase the step says it is in;
/// then the plant steps.
Out run(Rig& rig, const std::function<void(Rig&, int)>& hook = {}, int maxTicks = 8000) {
    Out o;
    DriveCharacterizer& ch = *rig.ch;
    const CharacterizerConfig& cfg = ch.config();
    if (ch.phase() == CharacterizerPhase::Idle) {
        ch.start("host test");
    }
    double segStartT = 0.0;
    for (int i = 0; i < maxTicks; ++i) {
        if (hook) {
            hook(rig, i);
        }
        const CharacterizerPhase before = ch.phase();
        const std::size_t segBefore = ch.status().segment;
        const double battery = rig.h.battery().voltage().value();  // what this tick reads
        ch.tick();
        const CharacterizerPhase ph = ch.phase();
        const double now = rig.h.clock().now().value();
        const bool began = ph == CharacterizerPhase::Driving && before != CharacterizerPhase::Driving;
        if (began) {
            segStartT = now;
        }
        // The ticks a segment's numbers are over: its first (it seeds them), then every driving and
        // stopping tick — except one refused for a missed tick before it read anything.
        const bool missed = ph == CharacterizerPhase::Aborted
                            && ch.refusal() == CharacterizerRefusal::MissedTick;
        const bool accumulated = (before == CharacterizerPhase::Driving
                                  || before == CharacterizerPhase::Stopping)
                                 && !missed;
        if (began || accumulated) {
            const std::size_t s = std::min<std::size_t>(began ? ch.status().segment : segBefore, 3);
            o.battMin[s] = o.battN[s] == 0 ? battery : std::min(o.battMin[s], battery);
            o.battSum[s] += battery;
            ++o.battN[s];
            o.segTicks[s] += accumulated ? 1 : 0;
        }
        const auto seg = static_cast<std::size_t>(std::min<int>(ch.status().segment, 3));
        for (int w = 0; w < 2; ++w) {
            for (const SideMember& m : rig.side(static_cast<std::size_t>(w))) {
                const double v = m.motor->commandedVoltage().value();
                const BrakeMode b = m.motor->brakeMode();
                if (ph == CharacterizerPhase::Stopping) {
                    o.brakeViolations += (v != 0.0 || b != BrakeMode::Brake) ? 1 : 0;
                } else if (ph == CharacterizerPhase::Driving) {
                    const double want = static_cast<double>(m.sign) * ch.status().volts;
                    o.drivingViolations += (v != want || b != BrakeMode::Coast) ? 1 : 0;
                } else {
                    o.coastViolations += (v != 0.0 || b != BrakeMode::Coast) ? 1 : 0;
                }
            }
        }
        if (ph == CharacterizerPhase::Driving) {
            const double want = profileVolts(cfg, static_cast<SegmentKind>(seg), now - segStartT);
            o.profileViolations += ch.status().volts != want ? 1 : 0;
        }
        if (ph == CharacterizerPhase::Driving || ph == CharacterizerPhase::Stopping) {
            const CharacterizerSample& smp = ch.lastSample();
            bool bad = !smp.valid || smp.t != now || smp.segment != ch.status().segment
                       || smp.powered != (ph == CharacterizerPhase::Driving)
                       || smp.volts != (ph == CharacterizerPhase::Driving ? ch.status().volts : 0.0)
                       || smp.battery != battery
                       || smp.headingRad != rig.h.imu().heading().radians();
            for (std::size_t s = 0; s < 2; ++s) {
                double current = 0.0;
                for (const SideMember& m : rig.side(s)) {
                    current = std::max(current, std::abs(m.motor->current().value()));
                }
                bad = bad || smp.maxCurrent[s] != current
                      || smp.position[s]
                             != sideMedian(rig, s, [](const shulib::hal::IMotor& m) { return m.position().value(); })
                      || smp.speed[s]
                             != sideMedian(rig, s, [](const shulib::hal::IMotor& m) { return m.velocity().value(); });
            }
            o.sampleViolations += bad ? 1 : 0;
        }
        if (ph == CharacterizerPhase::Stopping) {
            ++o.brakeTicks;
        }
        if (ph == CharacterizerPhase::Driving) {
            ++o.poweredTicks;
            if (o.firstPoweredTick < 0) {
                o.firstPoweredTick = i;
            }
        }
        o.ticks = i + 1;
        if (ph == CharacterizerPhase::Done || ph == CharacterizerPhase::Aborted) {
            break;
        }
        rig.h.plant().step(Time{kDt});
        const auto spin = rig.h.plant().trueWheelSpin();
        for (std::size_t s = 0; s < 2; ++s) {
            const double d = spin[static_cast<int>(s)].value() * kDt;
            if (ph == CharacterizerPhase::Driving) {
                o.powered[seg][s] += d;
                o.lastPowered[seg][s] = d;
            } else if (ph == CharacterizerPhase::Stopping) {
                o.stop[seg][s] += d;
            }
        }
    }
    o.end = ch.phase();
    // Every string the step produced, checked where it is produced (the R5a review: a single
    // case collecting the others' strings failed when run alone).
    CHECK(screenSafe(ch.reason()));
    CHECK(screenSafe(ch.warning()));
    return o;
}

[[nodiscard]] bool relEq(double a, double b, double rel) {
    return std::abs(a - b) <= rel * std::abs(b);
}

void requireSideGains(const shulib::calibration::FitResult& f, const FeedforwardGains& g, double relTol) {
    const std::string verdict = shulib::calibration::fitVerdictName(f.verdict);
    CAPTURE(verdict);
    REQUIRE(f.verdict == FitVerdict::Ok);
    CHECK(f.gains.kS == rel(g.kS, relTol));
    CHECK(f.gains.kV == rel(g.kV, relTol));
    CHECK(f.gains.kA == rel(g.kA, relTol));
}

/// Every run-level check a clean run must pass.
void requireCleanTicks(const Out& o) {
    CHECK(o.brakeViolations == 0);
    CHECK(o.coastViolations == 0);
    CHECK(o.drivingViolations == 0);
    CHECK(o.profileViolations == 0);
    CHECK(o.sampleViolations == 0);
}

}  // namespace

// ═══ C1. Recovery, both orientations ════════════════════════════════════════════════
// Bug caught: the two sides' samples accumulated into each other's fits (swap L/R: red in BOTH
// orientations, because the sides differ), the scale forgotten (kV off by 1.625), the stop's u = 0
// samples admitted (the plant's own decay has no friction term: kS drops), the friction column
// lost; a segment driven the wrong way, or at the wrong volts (StepBackward is 2026-09-25's hard
// reverse — it must BE a reverse); a result whose fields Part B persists differing from the fit's;
// the rests between segments measured as ticks (false HA-32/34 evidence).
TEST_CASE("DriveCharacterizer C1: each side's gains recovered to 1e-6 on robot two's shape, in both "
          "orientations; each segment's direction and volts; the result is the fit's") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        Rig rig{p};
        const Out o = run(rig);
        const std::string reasonText = rig.ch->reason();
        CAPTURE(reasonText);
        REQUIRE(o.end == CharacterizerPhase::Done);
        const auto& rep = rig.ch->report();
        requireSideGains(rep.fit[0], p.leftGains(), 1e-6);
        requireSideGains(rep.fit[1], p.rightGains(), 1e-6);
        REQUIRE(rep.segmentCount == kSegmentCount);
        int timedTicks = 0;
        for (std::size_t s = 0; s < kSegmentCount; ++s) {
            CAPTURE(s);
            const SegmentRecord& seg = rep.segments[s];
            CHECK(seg.kind == static_cast<SegmentKind>(s));
            CHECK(seg.end == SegmentEnd::Distance);
            CHECK(seg.stopped);
            CHECK(std::abs(seg.encoderCurveRad) < rep.curveGuardRad);
            // Each segment's DIRECTION: forward, backward, forward, backward — on both sides.
            const double dir = (s % 2 == 0) ? 1.0 : -1.0;
            for (std::size_t side = 0; side < 2; ++side) {
                CHECK(dir * seg.travelIn[side] > 20.0);
                CHECK(dir * seg.stopIn[side] > 0.0);
            }
            // The volts each ended on: a ramp under its top (it ended on distance first), the
            // step's level exactly.
            if (s < 2) {
                CHECK(dir * seg.lastVolts > 0.0);
                CHECK(std::abs(seg.lastVolts) < p.cfg.rampMax);
            } else {
                CHECK(seg.lastVolts == dir * p.cfg.stepVolts);
            }
            // The segment's samples: some admitted, the stop's excluded as Stop, per side.
            for (std::size_t side = 0; side < 2; ++side) {
                CHECK(seg.admitted[side] > 20U);
                CHECK(seg.excluded[side][static_cast<std::size_t>(FitExclusion::Stop) - 1] > 10U);
            }
            // The segment's numbers are over exactly the ticks it ran.
            CHECK(seg.timing.count == static_cast<std::uint32_t>(o.segTicks[s]));
            CHECK(seg.timing.overBudget == 0U);
            CHECK(seg.batteryMin == o.battMin[s]);
            CHECK(seg.batteryMean == o.battSum[s] / static_cast<double>(o.battN[s]));
            CHECK(seg.batteryMin > 12.0);
            timedTicks += o.segTicks[s];
        }
        // The run's timing: every driving and stopping tick and nothing else — the rests between
        // segments are deliberate gaps, not ticks (rebaselined).
        CHECK(rep.timing.count == static_cast<std::uint32_t>(timedTicks));
        CHECK(rep.timing.count == static_cast<std::uint32_t>(o.poweredTicks + o.brakeTicks));
        CHECK(rep.timing.overDouble == 0U);
        CHECK(rep.timing.overBudget == 0U);
        CHECK(rep.timing.maxS < 2.0 * kDt);
        CHECK(rep.timing.minS > 0.5 * kDt);
        // The curve the ~5 % asymmetry produces is real and inside the guard (13–17° a segment).
        CHECK(std::abs(rep.segments[0].encoderCurveRad) > 5.0 * Angle::kPi / 180.0);
        // The plant has no IMU hostility here: the IMU and the encoders agree about the curve.
        CHECK(std::abs(rep.segments[0].imuMinusEncoderRad()) < 1e-6);
        const DriveCharacterization& r = rig.ch->result();
        char why[160];
        CHECK(r.valid(why, sizeof why));
        CHECK(r.source == shulib::calibration::CharacterizationSource::Measured);
        CHECK(r.hasGains);
        CHECK_FALSE(r.hasRest);
        // Every field Part B persists is the fit's, side by side (no field dropped or swapped).
        const shulib::calibration::SideCharacterization* sides[2] = {&r.left, &r.right};
        for (std::size_t s = 0; s < 2; ++s) {
            CAPTURE(s);
            const auto& fit = rep.fit[s];
            const auto& sc = *sides[s];
            CHECK(sc.gains.kS == fit.gains.kS);
            CHECK(sc.gains.kV == fit.gains.kV);
            CHECK(sc.gains.kA == fit.gains.kA);
            CHECK(sc.residualRms == fit.residualRms);
            CHECK(sc.alpha == fit.alpha);
            CHECK(sc.voltsMin == fit.voltsMin);
            CHECK(sc.voltsMax == fit.voltsMax);
            CHECK(sc.samples == fit.samples);
            CHECK(sc.kABelowResolution == fit.kABelowResolution);
            CHECK(sc.members == 5U);
            CHECK(fit.samples > 300U);
            CHECK(fit.residualRms < 1e-6);
            // The largest admitted volts: a ramp's last command (its end tick's sample is admitted)
            // or the step's level, whichever is larger — about 6 V either way on this plant.
            CHECK(fit.voltsMax == std::max({std::abs(rep.segments[0].lastVolts),
                                            std::abs(rep.segments[1].lastVolts), p.cfg.stepVolts}));
        }
        CHECK(std::memcmp(&r.rampTiming, &rep.timing, sizeof rep.timing) == 0);
        CHECK(r.combined().kV == rel(0.5 * (0.160 + 0.168), 1e-6));
        CHECK(rep.combined.kS == rel(0.5 * (1.0 + 1.1), 1e-6));
        CHECK(rep.kVAsymmetry == rel(0.008 / 0.164, 1e-5));
        CHECK_FALSE(rep.asymmetryWarning);
        CHECK_FALSE(rep.emptySegmentWarning);  // every segment measured both sides
        CHECK(std::string(rig.ch->warning()).empty());
        CHECK(std::strcmp(r.provenance, "host test") == 0);
        requireCleanTicks(o);
        CHECK(o.brakeTicks > 40);
    }
}

// ═══ C2. Reversed members give the same gains ═══════════════════════════════════════
// Bug caught: the sign applied twice (or not at all). The reversed members' readings then disagree
// with their mates, and their commands drive the train the wrong way: a fight on the robot, and
// either a Fight refusal or gains of a fifth of the drive here.
TEST_CASE("DriveCharacterizer C2: reversed members with their signs from the description give the "
          "gains of a robot with none") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan with;
        with.faster = f;
        with.decoupled = false;
        Plan without = with;
        without.reversed = false;
        Rig a{with};
        Rig b{without};
        REQUIRE(run(a).end == CharacterizerPhase::Done);
        REQUIRE(run(b).end == CharacterizerPhase::Done);
        for (std::size_t s = 0; s < 2; ++s) {
            CHECK(relEq(a.ch->report().fit[s].gains.kV, b.ch->report().fit[s].gains.kV, 1e-9));
            CHECK(relEq(a.ch->report().fit[s].gains.kS, b.ch->report().fit[s].gains.kS, 1e-9));
            CHECK(relEq(a.ch->report().fit[s].gains.kA, b.ch->report().fit[s].gains.kA, 1e-9));
        }
    }
}

// ═══ C3. A slipping member cannot move the gains (the median) ══════════════════════
// Bug caught: a side's speed taken as the MEAN of its members — the slipping member (reading 1.3×
// its mates through the drag) would drag every speed, and so kV, by 6 %.
TEST_CASE("DriveCharacterizer C3: a decoupled member leaves the gains unchanged") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan with;
        with.faster = f;
        Plan without = with;
        without.decoupled = false;
        Rig a{with};
        Rig b{without};
        REQUIRE(run(a).end == CharacterizerPhase::Done);
        REQUIRE(run(b).end == CharacterizerPhase::Done);
        for (std::size_t s = 0; s < 2; ++s) {
            CHECK(relEq(a.ch->report().fit[s].gains.kV, b.ch->report().fit[s].gains.kV, 1e-9));
            CHECK(relEq(a.ch->report().fit[s].gains.kS, b.ch->report().fit[s].gains.kS, 1e-9));
        }
        // …and it really does read differently from its mates (the case is not vacuous).
        CHECK(std::abs(a.port(19).position().value()) > 1.2 * std::abs(a.port(18).position().value()));
    }
}

// ═══ C4. The units chain, per side ══════════════════════════════════════════════════
// Bug caught: a gain not in the description's units. A scale 10 % high reads every speed 10 %
// high, so kV (and kA, which is kV × τ) come out 1/1.1 of the truth, and kS — volts — not at all.
// The R5a review of 2026-10-04: with ONE ipr for both sides, a step that applied the LEFT side's
// scale to the RIGHT fitted identical numbers — so here only ONE side's scale is high as well, and
// the other side must not move at all (landmine 7: gains are in each side's own units).
TEST_CASE("DriveCharacterizer C4: an inches-per-radian 10 pct high moves that side's kV and kA by "
          "exactly 1/1.1 and nothing else -- both sides, and each side alone") {
    struct Scale {
        const char* what;
        double l;
        double r;
    };
    for (const Scale sc : {Scale{"both", 1.1, 1.1}, Scale{"LEFT only", 1.1, 1.0},
                           Scale{"RIGHT only", 1.0, 1.1}}) {
        for (Faster f : kBoth) {
            CAPTURE(sc.what);
            const std::string orient = name(f);
            CAPTURE(orient);
            Plan p;
            p.faster = f;
            p.iprScaleL = sc.l;
            p.iprScaleR = sc.r;
            Rig rig{p};
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Done);
            const auto& rep = rig.ch->report();
            for (std::size_t s = 0; s < 2; ++s) {
                CAPTURE(s);
                const FeedforwardGains g = s == 0 ? p.leftGains() : p.rightGains();
                const double scale = s == 0 ? sc.l : sc.r;
                CHECK(rep.fit[s].gains.kV == rel(g.kV / scale, 1e-9));
                CHECK(rep.fit[s].gains.kA == rel(g.kA / scale, 1e-9));
                CHECK(rep.fit[s].gains.kS == rel(g.kS, 1e-9));
            }
            requireCleanTicks(o);
        }
    }
}

// ═══ C5. Travel, stop and the brake ═════════════════════════════════════════════════
// Bug caught: the distance bound judged on the MEAN of the sides (the faster side overruns), on the
// LEFT side only (red only when RIGHT is faster), Coast instead of Brake at a normal end, a stopping
// distance measured from the wrong baseline.
TEST_CASE("DriveCharacterizer C5: powered travel is bounded per side on plant truth; the stopping "
          "distance equals truth; a normal end is 0 V + Brake until rest, then Coast") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        Rig rig{p};
        const Out o = run(rig);
        REQUIRE(o.end == CharacterizerPhase::Done);
        const auto& rep = rig.ch->report();
        const double d = p.cfg.segmentDistance.value();
        for (std::size_t s = 0; s < kSegmentCount; ++s) {
            CAPTURE(s);
            for (std::size_t side = 0; side < 2; ++side) {
                CAPTURE(side);
                CHECK(std::abs(o.powered[s][side]) <= d + std::abs(o.lastPowered[s][side]) + 1e-9);
                CHECK(rep.segments[s].travelIn[side] == rel(o.powered[s][side], 1e-9));
                CHECK(rep.segments[s].stopIn[side] == rel(o.stop[s][side], 1e-9));
                CHECK(std::abs(rep.segments[s].stopIn[side]) > 0.5);  // it really rolled on
                CHECK(std::abs(rep.segments[s].stopIn[side]) < p.cfg.stopBound.value());
            }
            // The faster side is the one that reached the distance.
            const std::size_t fastSide = f == Faster::Left ? 0 : 1;
            CHECK(std::abs(rep.segments[s].travelIn[fastSide]) >= d);
            CHECK(std::abs(rep.segments[s].travelIn[1 - fastSide]) < d);
        }
        requireCleanTicks(o);
        CHECK(o.brakeTicks > 40);
    }
}

// ═══ C6. The curve guard ════════════════════════════════════════════════════════════
// Bug caught: no curve guard, a guard computed with a fixed width instead of the description's, a
// refusal that leaves the last volts or brakes, a message that does not say which side — and a
// report that, after the refusal, said the refused segment travelled 0 in and curved 0° (found
// while answering the R5a review: a refusal must leave the numbers behind it in the record).
TEST_CASE("DriveCharacterizer C6: a large asymmetry is CurveExceeded with 0 V + Coast on that "
          "tick, naming both sides, and the report keeps how far it got; the guard uses the "
          "description's width") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        // The slow side's kV 1.8× the fast side's: speeds ~45 % apart at the same volts, 1.5×
        // asymRefuse.
        p.slow = FeedforwardGains{.kS = 1.0, .kV = 0.288, .kA = 0.036};
        Rig rig{p};
        const Out o = run(rig);
        REQUIRE(o.end == CharacterizerPhase::Aborted);
        CHECK(rig.ch->refusal() == CharacterizerRefusal::CurveExceeded);
        CHECK(o.coastViolations == 0);  // the refusal tick included: 0 V + Coast, every member
        CHECK(o.brakeViolations == 0);
        CHECK(o.profileViolations == 0);
        CHECK(std::strstr(rig.ch->reason(), "LEFT travelled") != nullptr);
        CHECK(std::strstr(rig.ch->reason(), "RIGHT") != nullptr);
        CHECK(std::strstr(rig.ch->reason(), "dragging side") != nullptr);
        CHECK(rig.ch->report().segmentCount == 1U);
        // The curve had the sign of the slower side: LEFT faster turns the robot clockwise.
        const double curve = rig.ch->status().curveRad;
        CHECK((f == Faster::Left ? curve < 0.0 : curve > 0.0));
        CHECK(std::abs(curve) > rig.ch->report().curveGuardRad);
        // The REPORT keeps the refused segment's numbers — the ones its reason quotes.
        const SegmentRecord& seg = rig.ch->report().segments[0];
        CHECK(seg.end == SegmentEnd::None);
        CHECK_FALSE(seg.stopped);
        CHECK(seg.encoderCurveRad == curve);
        CHECK(seg.travelIn[0] == rig.ch->status().travelIn[0]);
        CHECK(seg.travelIn[1] == rig.ch->status().travelIn[1]);
        CHECK(seg.travelIn[f == Faster::Left ? 0 : 1] > 5.0);  // it really got somewhere
        CHECK(seg.imuYawRad == rel(curve, 1e-6));  // a truthful IMU here
        CHECK(seg.seconds > 1.0);
        CHECK(seg.lastVolts > 0.0);  // the ramp's last command, not the refusal's 0
        CHECK(seg.timing.count > 50U);
        CHECK(seg.stopIn[0] == 0.0);  // no stop: a refusal coasts, it does not brake
    }
    // At W = 25 in the guard is 36 × 0.30 / 25 = 0.432 rad; an asymmetry near 0.4 curves ~0.58 rad —
    // refused here, and NOT by a guard computed from W = 12.5 in (0.864 rad).
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        p.trackWidth = 25.0;
        p.slow = FeedforwardGains{.kS = 1.0, .kV = 0.26, .kA = 0.0325};
        Rig rig{p};
        const Out o = run(rig);
        REQUIRE(o.end == CharacterizerPhase::Aborted);
        CHECK(rig.ch->refusal() == CharacterizerRefusal::CurveExceeded);
        CHECK(rig.ch->report().curveGuardRad == rel(36.0 * 0.30 / 25.0, 1e-12));
        CHECK(std::abs(rig.ch->status().curveRad) < 36.0 * 0.30 / 12.5);
    }
}

namespace {

/// An IMU that lies in the three ways the encoder guard must ignore: an apparent spin of 90 °/s
/// through segment 0's driving; a ONE-WAY jump of −20° in segment 2 (StepForward), which stays; and
/// HA-160's shape in the StepBackward — the heading swinging 37° a tick, alternating in sign, for 9
/// steps, then left 37° off. Its yaw RATE tells the truth (so the rest definition is untouched).
/// `phase`/`segment` are set by the run's hook.
struct LyingImu final : shulib::sim::DegradationModel {
    CharacterizerPhase phase = CharacterizerPhase::Idle;
    int segment = 0;
    double spinOffset = 0.0;
    int oneWayTicks = 0;
    double oneWayOffset = 0.0;
    int jumpTicks = 0;
    double jumpOffset = 0.0;
    Angle imuHeading(Angle truth, Time, shulib::sim::Rng&) override {
        if (phase == CharacterizerPhase::Driving && segment == 0) {
            spinOffset += 90.0 * Angle::kPi / 180.0 * kDt;
        }
        if (phase == CharacterizerPhase::Driving && segment == 2) {
            if (++oneWayTicks == 30) {
                oneWayOffset = -20.0 * Angle::kPi / 180.0;
            }
        }
        if (phase == CharacterizerPhase::Driving && segment == 3) {
            ++jumpTicks;
            if (jumpTicks > 20 && jumpTicks <= 29) {
                jumpOffset = (jumpTicks % 2 == 1) ? 37.0 * Angle::kPi / 180.0 : 0.0;
            } else if (jumpTicks > 29) {
                jumpOffset = 37.0 * Angle::kPi / 180.0;
            }
        }
        return truth + Angle::radians(spinOffset + oneWayOffset + jumpOffset);
    }
};

}  // namespace

// ═══ C7. The IMU never refuses ═════════════════════════════════════════════════════
// Bug caught: a curve guard (or any refusal) read from the IMU — 2026-09-25's IMU jumped up to 37°
// a tick in a hard reverse (HA-160), and an IMU guard would end the very run that records it; and
// the largest step kept SIGNED (the one-way −20° jump would then not be its largest step).
TEST_CASE("DriveCharacterizer C7: an IMU that jumps 37 deg a tick in the StepBackward, spins 90 "
          "deg/s the encoders do not see, or jumps 20 deg one way, never refuses; the report shows "
          "all three") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        LyingImu imu;
        Plan p;
        p.faster = f;
        p.degradation = &imu;
        Rig rig{p};
        const Out o = run(rig, [&](Rig& r, int) {
            imu.phase = r.ch->phase();
            imu.segment = r.ch->status().segment;
        });
        const std::string reasonText = rig.ch->reason();
        CAPTURE(reasonText);
        REQUIRE(o.end == CharacterizerPhase::Done);
        const auto& rep = rig.ch->report();
        const double deg = Angle::kPi / 180.0;
        CHECK(rep.segments[3].maxHeadingStepRad == rel(37.0 * deg, 0.02));
        CHECK(rep.segments[3].imuMinusEncoderRad() == rel(37.0 * deg, 0.02));
        CHECK(rep.segments[0].imuMinusEncoderRad() > 90.0 * deg);  // seconds of a 90 °/s lie
        CHECK(std::abs(rep.segments[0].encoderCurveRad) < rep.curveGuardRad);
        CHECK(std::abs(rep.segments[1].imuMinusEncoderRad()) < 1e-6);  // a truthful segment
        // The one-way jump: a NEGATIVE step of 20° is the segment's largest (|·|, never signed).
        CHECK(rep.segments[2].maxHeadingStepRad == rel(20.0 * deg, 0.05));
        CHECK(rep.segments[2].imuMinusEncoderRad() == rel(-20.0 * deg, 0.02));
        requireSideGains(rep.fit[0], p.leftGains(), 1e-6);  // and the gains never noticed
        requireSideGains(rep.fit[1], p.rightGains(), 1e-6);
        requireCleanTicks(o);
    }
}

// ═══ C8. A stop too long ════════════════════════════════════════════════════════════
// Bug caught: no stop bound — a robot that rolls further than the lane was cleared for would go on
// to its next segment from wherever it ended up.
TEST_CASE("DriveCharacterizer C8: a stop longer than stopBound refuses as StopTooLong before the "
          "next segment") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        // τ = kA/kV ≈ 0.63 s: the plant's decay from ~25 in/s rolls ~15 in.
        p.fast = FeedforwardGains{.kS = 1.0, .kV = 0.160, .kA = 0.100};
        p.slow = FeedforwardGains{.kS = 1.1, .kV = 0.168, .kA = 0.105};
        Rig rig{p};
        const Out o = run(rig);
        REQUIRE(o.end == CharacterizerPhase::Aborted);
        CHECK(rig.ch->refusal() == CharacterizerRefusal::StopTooLong);
        CHECK(rig.ch->report().segmentCount == 1U);  // no further segment started
        const auto& seg = rig.ch->report().segments[0];
        CHECK(std::max(std::abs(seg.stopIn[0]), std::abs(seg.stopIn[1])) > p.cfg.stopBound.value());
        CHECK(std::abs(seg.stopIn[0]) == rel(std::abs(o.stop[0][0]), 1e-9));
        CHECK(o.coastViolations == 0);
        CHECK(o.brakeViolations == 0);
        CHECK(std::strstr(rig.ch->reason(), "to stop under Brake") != nullptr);
    }
}

// ═══ C9. Hot motors ═════════════════════════════════════════════════════════════════
// Bug caught: the temperature never checked — a segment started on a motor already near its
// throttle step, whose gains would then be measured half-throttled (HA-44) — and the refusal naming
// the FIRST hot member it met instead of the hottest.
TEST_CASE("DriveCharacterizer C9: a member at tempGuard refuses TooHot naming its port with nothing "
          "powered; under it the run goes on; heating mid-run stops the next segment; the hotter "
          "of two is named") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        SUBCASE("at the guard before the first segment") {
            Plan p;
            p.faster = f;
            Rig rig{p};
            rig.port(17).setTemperature(p.cfg.tempGuard);  // exactly the guard: refused
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::TooHot);
            CHECK(rig.ch->refusalPort() == 17);
            CHECK(o.poweredTicks == 0);
            CHECK(o.coastViolations == 0);
            CHECK(std::strstr(rig.ch->reason(), "port 17") != nullptr);
        }
        SUBCASE("just under the guard") {
            Plan p;
            p.faster = f;
            Rig rig{p};
            rig.port(17).setTemperature(std::nextafter(p.cfg.tempGuard, 0.0));
            CHECK(run(rig).end == CharacterizerPhase::Done);
            CHECK(rig.ch->report().segments[0].maxTemperature
                  == std::nextafter(p.cfg.tempGuard, 0.0));
        }
        SUBCASE("heating up during the run") {
            Plan p;
            p.faster = f;
            Rig rig{p};
            const Out o = run(rig, [](Rig& r, int) {
                if (r.ch->report().segmentCount >= 2 && r.ch->phase() == CharacterizerPhase::Stopping) {
                    r.port(12).setTemperature(50.0);
                }
            });
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::TooHot);
            CHECK(rig.ch->refusalPort() == 12);
            CHECK(rig.ch->report().segmentCount == 2U);  // the third never started
            CHECK(rig.ch->report().segments[1].stopped);
        }
        SUBCASE("two hot members: the HOTTER is named, whichever comes first") {
            for (const bool rightHotter : {true, false}) {
                CAPTURE(rightHotter);
                Plan p;
                p.faster = f;
                Rig rig{p};
                rig.port(12).setTemperature(rightHotter ? 46.0 : 50.0);  // LEFT: met first
                rig.port(18).setTemperature(rightHotter ? 50.0 : 46.0);
                const Out o = run(rig);
                REQUIRE(o.end == CharacterizerPhase::Aborted);
                CHECK(rig.ch->refusal() == CharacterizerRefusal::TooHot);
                CHECK(rig.ch->refusalPort() == (rightHotter ? 18 : 12));
                CHECK(std::strstr(rig.ch->reason(), "50.0 C") != nullptr);
                CHECK(o.poweredTicks == 0);
            }
        }
    }
}

// ═══ C10. Missed ticks ══════════════════════════════════════════════════════════════
// Bug caught: no missed-tick guard — a caller that stopped ticking would leave the robot driving at
// its last volts.
TEST_CASE("DriveCharacterizer C10: a missed tick while driving refuses with 0 V + Coast; while at "
          "rest it is nothing") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        SUBCASE("while driving") {
            Plan p;
            p.faster = f;
            Rig rig{p};
            bool skipped = false;
            const Out o = run(rig, [&](Rig& r, int) {
                if (!skipped && r.ch->phase() == CharacterizerPhase::Driving
                    && r.ch->status().segment == 1 && r.ch->status().volts < -3.0) {
                    skipped = true;
                    for (int k = 0; k < 11; ++k) {  // 0.11 s pass without a tick
                        r.h.plant().step(Time{kDt});
                    }
                }
            });
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::MissedTick);
            CHECK(o.coastViolations == 0);
            CHECK(std::strstr(rig.ch->reason(), "motors stopped") != nullptr);
            // The refused segment keeps how far it got (the last tick's travel, backward).
            CHECK(rig.ch->report().segments[1].travelIn[0] < -1.0);
            CHECK(rig.ch->report().segments[1].lastVolts < -3.0);
        }
        SUBCASE("at rest") {
            Plan p;
            p.faster = f;
            Rig rig{p};
            bool skipped = false;
            const Out o = run(rig, [&](Rig& r, int) {
                if (!skipped && r.ch->phase() == CharacterizerPhase::Rest
                    && r.ch->status().segment == 2) {
                    skipped = true;
                    for (int k = 0; k < 30; ++k) {  // 0.3 s pass without a tick, at rest
                        r.h.plant().step(Time{kDt});
                    }
                }
            });
            CHECK(skipped);
            CHECK(o.end == CharacterizerPhase::Done);
        }
    }
}

// ═══ C11. Abort in every phase ══════════════════════════════════════════════════════
TEST_CASE("DriveCharacterizer C11: abort() in every phase leaves every member at 0 V + Coast") {
    const CharacterizerPhase at[] = {CharacterizerPhase::WaitImu, CharacterizerPhase::Rest,
                                     CharacterizerPhase::Driving, CharacterizerPhase::Stopping,
                                     CharacterizerPhase::Done};
    for (Faster f : kBoth) {
        for (CharacterizerPhase target : at) {
            const std::string orient = name(f);
            CAPTURE(orient);
            CAPTURE(static_cast<int>(target));
            Plan p;
            p.faster = f;
            Rig rig{p};
            rig.ch->start("host test");
            bool aborted = false;
            for (int i = 0; i < 8000 && !aborted; ++i) {
                if (rig.ch->phase() == target && (target != CharacterizerPhase::Driving
                                                  || rig.ch->status().volts > 2.0)) {
                    rig.ch->abort("operator touched ABORT");
                    aborted = true;
                    break;
                }
                rig.ch->tick();
                rig.h.plant().step(Time{kDt});
            }
            REQUIRE(aborted);
            CHECK(rig.ch->phase() == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::Aborted);
            CHECK(std::string(rig.ch->reason()) == "operator touched ABORT");
            for (std::size_t s = 0; s < 2; ++s) {
                for (const SideMember& m : rig.side(s)) {
                    CHECK(m.motor->commandedVoltage().value() == 0.0);
                    CHECK(m.motor->brakeMode() == BrakeMode::Coast);
                }
            }
            CHECK_THROWS_AS((void)rig.ch->result(), PreconditionError);
            if (target == CharacterizerPhase::Driving) {
                // An abort mid-segment keeps how far the segment got, with the volts it was at.
                CHECK(rig.ch->report().segments[0].lastVolts > 2.0);
                CHECK(rig.ch->report().segments[0].seconds > 1.0);
            }
        }
        // An abort on a segment's FIRST tick: the segment has gone nowhere, and its record must not
        // inherit the previous segment's travel (the panel's numbers start at 0 with each segment).
        Plan p;
        p.faster = f;
        Rig rig{p};
        rig.ch->start("host test");
        bool aborted = false;
        for (int i = 0; i < 8000 && !aborted; ++i) {
            const CharacterizerPhase before = rig.ch->phase();
            rig.ch->tick();
            if (before == CharacterizerPhase::Rest && rig.ch->phase() == CharacterizerPhase::Driving
                && rig.ch->status().segment == 1) {
                rig.ch->abort("operator touched ABORT");
                aborted = true;
                break;
            }
            rig.h.plant().step(Time{kDt});
        }
        REQUIRE(aborted);
        const SegmentRecord& seg = rig.ch->report().segments[1];
        CHECK(rig.ch->report().segments[0].travelIn[0] > 30.0);  // segment 0 did go somewhere
        CHECK(seg.travelIn[0] == 0.0);
        CHECK(seg.travelIn[1] == 0.0);
        CHECK(seg.encoderCurveRad == 0.0);
        CHECK(seg.seconds == 0.0);
    }
}

// ═══ C12. A fighting member ═════════════════════════════════════════════════════════
TEST_CASE("DriveCharacterizer C12: a member whose reading fights its side refuses as Fight naming "
          "its port, on either side") {
    for (Faster f : kBoth) {
        for (int port : {14, 18}) {
            const std::string orient = name(f);
            CAPTURE(orient);
            CAPTURE(port);
            Plan p;
            p.faster = f;
            Rig rig{p};
            const int wheel = port <= 15 ? 0 : 1;
            const int k = port <= 15 ? port - 11 : port - 16;
            rig.h.plant().setMemberFault(wheel, k, MemberFault::SignFlipped);
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::Fight);
            CHECK(rig.ch->refusalPort() == port);
            CHECK(o.coastViolations == 0);
            CHECK(rig.ch->report().segmentCount == 1U);
        }
    }
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

}  // namespace

// ═══ C13. Rest before every segment ═════════════════════════════════════════════════
// Bug caught: a segment started before the robot is at rest — the first segment would then begin
// while a person still has hands on the robot, and a disturbed start biases the fit's first samples.
TEST_CASE("DriveCharacterizer C13: a robot that never rests refuses RestTimeout with nothing "
          "powered; a disturbance at the start delays the first powered tick until rest has held") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        SUBCASE("never") {
            HandsOn hands;
            Plan p;
            p.faster = f;
            p.degradation = &hands;
            Rig rig{p};
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::RestTimeout);
            CHECK(o.poweredTicks == 0);
            CHECK(o.coastViolations == 0);  // unpowered throughout, the refusal tick included
            CHECK(std::strstr(rig.ch->reason(), "still rotating") != nullptr);
        }
        SUBCASE("until 1.2 s") {
            HandsOn hands;
            hands.until = 1.2;
            Plan p;
            p.faster = f;
            p.degradation = &hands;
            Rig rig{p};
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Done);
            // The first powered tick comes only after the disturbance ended AND rest held 0.5 s.
            CHECK(o.firstPoweredTick * kDt >= 1.2 + p.cfg.restSettleTime.value() - 1e-9);
        }
    }
}

// ═══ C14. Over-current samples ══════════════════════════════════════════════════════
// Bug caught: the current guard off — samples taken at the current limit (where torque, not the law,
// sets the acceleration: HA-172) would enter the fit — and a current read on one side excluding
// samples on the other.
TEST_CASE("DriveCharacterizer C14: samples with a member over the current guard are excluded, "
          "counted on that member's side, and leave the gains unchanged") {
    for (Faster f : kBoth) {
        for (int port : {12, 17}) {
            const std::string orient = name(f);
            CAPTURE(orient);
            CAPTURE(port);
            const std::size_t side = port <= 15 ? 0 : 1;
            Plan p;
            p.faster = f;
            p.decoupled = false;
            Rig rig{p};
            int flagged = 0;
            const Out o = run(rig, [&](Rig& r, int) {
                // Mid-step, past the dead band (a dead-band sample is excluded as DeadBand first).
                const bool on = r.ch->phase() == CharacterizerPhase::Driving
                                && r.ch->status().segment == 2
                                && r.ch->lastSample().speed[side] > 5.0 && flagged < 25;
                r.port(port).setCurrent(Current{on ? 2.5 : 1.0});
                flagged += on ? 1 : 0;
            });
            REQUIRE(o.end == CharacterizerPhase::Done);
            const auto& seg = rig.ch->report().segments[2];
            const auto over = static_cast<std::size_t>(FitExclusion::OverCurrent) - 1;
            CHECK(seg.excluded[side][over] == 25U);
            CHECK(seg.excluded[1 - side][over] == 0U);
            requireSideGains(rig.ch->report().fit[side], side == 0 ? p.leftGains() : p.rightGains(),
                             1e-6);
            CHECK(o.sampleViolations == 0);  // the sample's current column saw it, on its side
        }
    }
}

// ═══ C15. The asymmetry warning ═════════════════════════════════════════════════════
TEST_CASE("DriveCharacterizer C15: sides 20 pct apart finish with the asymmetry warning, in pct") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        p.slow = FeedforwardGains{.kS = 1.0, .kV = 0.195, .kA = 0.024};
        Rig rig{p};
        const Out o = run(rig);
        REQUIRE(o.end == CharacterizerPhase::Done);
        CHECK(rig.ch->report().asymmetryWarning);
        CHECK_FALSE(rig.ch->report().emptySegmentWarning);
        // The asymmetry alone: its text from the first character (nothing joined in front).
        CHECK(std::string(rig.ch->warning()).rfind("the sides' kV differ by", 0) == 0);
        CHECK(rig.ch->report().kVAsymmetry == rel(0.035 / 0.1775, 1e-5));
        CHECK(std::strstr(rig.ch->warning(), "pct") != nullptr);
        CHECK(std::strstr(rig.ch->warning(), "mechanical") != nullptr);
    }
}

namespace {

/// Every member's velocity reading jittered by white noise after each plant step — the plant has no
/// velocity seam, so the test does it (both σ INVENTED; what a V5's velocity() does is HA-171).
/// `moving` while the step drives; `still` otherwise — and `still` must sit well under the rest
/// floor (0.05 rad/s), or no rest would ever hold: a real velocity noise at the rest floor would
/// refuse every Calibrate run (HA-136's floor is as invented as this σ).
struct Jitter {
    shulib::sim::Rng rng;
    double moving;
    double still;
    void apply(Rig& r) {
        const double sigma = r.ch->phase() == CharacterizerPhase::Driving ? moving : still;
        for (int w = 0; w < 2; ++w) {
            for (std::size_t k = 0; k < Rig::kPer; ++k) {
                FakeMotor& m = r.member(w, k);
                m.setVelocity(AngularVelocity{m.velocity().value() + sigma * rng.nextGaussian()});
            }
        }
    }
};

}  // namespace

// ═══ C16. Hostility: a bound MEASURED, then pinned ══════════════════════════════════
// Not a precision claim about a robot: the plant's encoders are quantized (the A3 EncoderHostileModel,
// 900 ticks a turn) and every member's velocity() is jittered by white noise — 0.3 rad/s (≈ 0.5
// in/s at the plant's radius) while driving, 0.01 rad/s otherwise, under the rest floor; INVENTED,
// the plant has no velocity seam. A noisy regressor biases α toward 0 (errors in variables), so kA
// reads LOW; the median over five members cuts the noise (feedforward_fit_test.cpp M7 pins the same
// bias on synthetic data, and HA-171 records it).
// The largest relative error over four seeds in BOTH orientations was MEASURED (the R5a progress
// log) and each bound pinned at no more than twice it.
TEST_CASE("DriveCharacterizer C16: under encoder quantization and a velocity jitter the gains stay "
          "within the measured, pinned bound") {
    double worst[3] = {0.0, 0.0, 0.0};
    for (Faster f : kBoth) {
        for (std::uint64_t seed : {1ULL, 2ULL, 3ULL, 4ULL}) {
            const std::string orient = name(f);
            CAPTURE(orient);
            CAPTURE(seed);
            shulib::sim::EncoderHostileModel enc{};
            Plan p;
            p.faster = f;
            p.degradation = &enc;
            Rig rig{p};
            Jitter jitter{shulib::sim::Rng{seed}, 0.3, 0.01};
            DriveCharacterizer& ch = *rig.ch;
            ch.start("host test");
            for (int i = 0; i < 8000; ++i) {
                ch.tick();
                if (ch.phase() == CharacterizerPhase::Done || ch.phase() == CharacterizerPhase::Aborted) {
                    break;
                }
                rig.h.plant().step(Time{kDt});
                jitter.apply(rig);
            }
            const std::string reasonText = ch.reason();
            CAPTURE(reasonText);
            REQUIRE(ch.phase() == CharacterizerPhase::Done);
            for (std::size_t s = 0; s < 2; ++s) {
                const FeedforwardGains g = s == 0 ? p.leftGains() : p.rightGains();
                const auto& got = ch.report().fit[s].gains;
                worst[0] = std::max(worst[0], std::abs(got.kS / g.kS - 1.0));
                worst[1] = std::max(worst[1], std::abs(got.kV / g.kV - 1.0));
                worst[2] = std::max(worst[2], std::abs(got.kA / g.kA - 1.0));
            }
        }
    }
    std::printf("[R5a C16] hostility worst relative error: kS %.5f kV %.5f kA %.5f\n", worst[0],
                worst[1], worst[2]);
    // MEASURED 2026-10-04 over both orientations (g++ 13.3, x86-64): kS 0.02726, kV 0.00925,
    // kA 0.09693 (L faster alone, 2026-09-29: kS 0.02599, kV 0.00925, kA 0.09693) — each bound
    // below is under twice its measurement. kA's is the loosest, as the errors-in-variables bias
    // predicts.
    CHECK(worst[0] <= 0.050);
    CHECK(worst[1] <= 0.018);
    CHECK(worst[2] <= 0.19);
    CHECK(worst[2] > worst[1]);  // the bias lands on kA first, not on kV
}

// ═══ C17. End to end on one plant ═══════════════════════════════════════════════════
// The drivetrain calibrator describes robot two's shape on the plant; the GAINS step then runs from
// THAT description — its signed ports, its scales, its effective width — on the same plant. The
// gains come out in the description's units, which is what the binding CRC of Part B protects.
// (One orientation: the calibrator's rig has equal sides by construction.)
TEST_CASE("DriveCharacterizer C17: the calibrator's description feeds the GAINS step on the same "
          "plant") {
    calib_rig::CalibRig rig{shulib::calibration::CalibratorConfig{},
                            calib_rig::calibPlant(0.0, calib_rig::kUsualKs, 0.0, 0.02)};
    rig.setFault(0, 2, MemberFault::Reversed);
    rig.cal.start(calib_rig::facts());
    REQUIRE(calib_rig::runCalibration(rig, calib_rig::RunOptions{})
            == shulib::calibration::Phase::Done);
    const shulib::hal::DrivetrainDescription desc = rig.cal.result();
    std::array<SideMember, 21> left{};
    std::array<SideMember, 21> right{};
    auto fill = [&](const shulib::hal::DriveSideDescription& side, std::array<SideMember, 21>& out) {
        for (std::size_t k = 0; k < side.count; ++k) {
            const int signedPort = side.signedPorts[k];
            const int port = std::abs(signedPort);
            std::size_t idx = 0;
            while (rig.ports[idx] != port) {
                ++idx;
            }
            // Calibrate's adapters are POSITIVE: the sign travels in the SideMember (rule 1).
            out[k] = SideMember{rig.candidates[idx], static_cast<std::int8_t>(signedPort > 0 ? 1 : -1),
                                static_cast<std::int8_t>(port)};
        }
    };
    fill(desc.left, left);
    fill(desc.right, right);
    DriveCharacterizer ch{DriveSide{std::span<const SideMember>{left.data(), desc.left.count},
                                    desc.left.inchesPerRadian},
                          DriveSide{std::span<const SideMember>{right.data(), desc.right.count},
                                    desc.right.inchesPerRadian},
                          desc.trackWidth, rig.h.imu(), rig.h.clock(), rig.h.battery()};
    ch.start("end to end");
    for (int i = 0; i < 8000; ++i) {
        ch.tick();
        if (ch.phase() == CharacterizerPhase::Done || ch.phase() == CharacterizerPhase::Aborted) {
            break;
        }
        rig.h.plant().step(Time{kDt});
    }
    const std::string reasonText = ch.reason();
    CAPTURE(reasonText);
    REQUIRE(ch.phase() == CharacterizerPhase::Done);
    const double scaleL = desc.left.inchesPerRadian.value() / calib_rig::kPlantRadius;
    const double scaleR = desc.right.inchesPerRadian.value() / calib_rig::kPlantRadius;
    std::printf("[R5a C17] e2e: scale L %.9f R %.9f; L kS %.6f kV %.6f kA %.6f; R kS %.6f kV %.6f "
                "kA %.6f\n",
                scaleL, scaleR, ch.report().fit[0].gains.kS, ch.report().fit[0].gains.kV,
                ch.report().fit[0].gains.kA, ch.report().fit[1].gains.kS,
                ch.report().fit[1].gains.kV, ch.report().fit[1].gains.kA);
    // In the description's units: kV and kA scale by the description's own scale error.
    for (std::size_t s = 0; s < 2; ++s) {
        const double scale = s == 0 ? scaleL : scaleR;
        CHECK(ch.report().fit[s].gains.kS == rel(calib_rig::kUsualKs, 1e-6));
        CHECK(ch.report().fit[s].gains.kV == rel(calib_rig::kKv / scale, 1e-6));
        CHECK(ch.report().fit[s].gains.kA == rel(0.02 / scale, 1e-6));
    }
}

// ═══ C18. The config and the preconditions ══════════════════════════════════════════
// The clear space: the R5a review of 2026-10-04 found the sideways excursion left out the STOP —
// a Done run on the host test plant swept 19.7 in sideways against the 14.6 in it printed. It is
// now the powered arc to the guard plus the whole stopBound (the header's "The clear space").
TEST_CASE("DriveCharacterizer C18: CharacterizerConfig::valid(W) names what it refuses, its clear "
          "space includes the stop, and the constructor enforces it") {
    char why[200];
    const CharacterizerConfig ok;
    CHECK(ok.valid(Length{12.5}, why, sizeof why));
    CHECK(ok.curveGuard(Length{12.5}) == rel(0.864, 1e-12));
    const double g = 36.0 * 0.30 / 12.5;
    CHECK(ok.sidewaysExcursion(Length{12.5})
          == rel(36.0 * (1.0 - std::cos(g)) / g + 12.0, 1e-12));
    CHECK(ok.sidewaysExcursion(Length{12.5}) == rel(26.6, 0.002));
    CHECK(ok.clearAheadAndBehind() == rel(48.0, 1e-12));
    CharacterizerConfig shortStop;
    shortStop.stopBound = Length{4.0};  // the stop's share moves with stopBound, one for one
    CHECK(shortStop.sidewaysExcursion(Length{12.5})
          == rel(ok.sidewaysExcursion(Length{12.5}) - 8.0, 1e-12));
    auto bad = [&](auto set, const char* word, double w = 12.5) {
        CharacterizerConfig c;
        set(c);
        CHECK_FALSE(c.valid(Length{w}, why, sizeof why));
        CHECK(std::strstr(why, word) != nullptr);
    };
    bad([](CharacterizerConfig& c) { c.asymRefuse = c.asymWarn; }, "asymRefuse");
    bad([](CharacterizerConfig& c) { c.asymRefuse = 0.1; }, "asymRefuse");
    bad([](CharacterizerConfig& c) { (void)c; }, "track width", 0.0);
    bad([](CharacterizerConfig& c) { (void)c; }, "pi", 3.0);  // ψ_g = 3.6 rad
    bad([](CharacterizerConfig& c) { c.rampMax = 12.5; }, "rampMax");
    bad([](CharacterizerConfig& c) { c.stepVolts = 0.0; }, "stepVolts");
    bad([](CharacterizerConfig& c) { c.stopBound = Length{0.0}; }, "stopBound");
    bad([](CharacterizerConfig& c) { c.tempGuard = 0.0; }, "tempGuard");
    bad([](CharacterizerConfig& c) { c.restTimeout = c.restSettleTime; }, "restTimeout");
    bad([](CharacterizerConfig& c) { c.timing.nominal = Time{0.02}; c.timing.budget = Time{0.03}; },
        "timing.nominal");
    bad([](CharacterizerConfig& c) { c.fit.minSamples = 1; }, "minSamples");

    Plan p;
    Rig rig{p};
    auto make = [&](std::span<const SideMember> l, std::span<const SideMember> r, double w) {
        DriveCharacterizer c{DriveSide{l, Length{kR}}, DriveSide{r, Length{kR}}, Length{w},
                             rig.h.imu(), rig.h.clock(), rig.h.battery()};
        (void)c;
    };
    const std::span<const SideMember> L{rig.left.data(), rig.left.size()};
    const std::span<const SideMember> R{rig.right.data(), rig.right.size()};
    CHECK_NOTHROW(make(L, R, 12.5));
    CHECK_THROWS_AS(make(L, R, 0.0), PreconditionError);
    CHECK_THROWS_AS(make(std::span<const SideMember>{}, R, 12.5), PreconditionError);
    CHECK_THROWS_AS(make(L, L, 12.5), PreconditionError);  // every port twice
    std::array<SideMember, 5> zeroSign = rig.left;
    zeroSign[1].sign = 0;
    CHECK_THROWS_AS(make(std::span<const SideMember>{zeroSign.data(), 5}, R, 12.5), PreconditionError);
    std::array<SideMember, 5> nullMotor = rig.left;
    nullMotor[4].motor = nullptr;
    CHECK_THROWS_AS(make(std::span<const SideMember>{nullMotor.data(), 5}, R, 12.5), PreconditionError);
    CHECK_THROWS_AS((void)rig.ch->result(), PreconditionError);  // not Done
    rig.ch->start("x");
    CHECK_THROWS_AS(rig.ch->start("again"), PreconditionError);
}

// ═══ C20. The refusals C1–C18 do not reach ══════════════════════════════════════════
// Added at the resumption of 2026-09-29 (the R5a log): three refusal paths had no case — an IMU that
// never readies, a stop that never comes to rest (a frozen moving reading, and a robot still rolling
// past stopBound when restTimeout runs out), and a fit that does not come out Ok. Each must end
// Aborted with every member at 0 V + Coast on the refusal tick, and a FitRefused run must hand back
// no result.
// Bug caught: a refused fit reported as Done (its gains would be persisted by Part C: the mutation
// "the FitRefused branch skipped" goes red here), a stop that never rests left braking forever, an
// IMU wait that powers a segment anyway.
TEST_CASE("DriveCharacterizer C20: an IMU that never readies, a stop that never rests, and a fit "
          "that is not Ok each refuse with 0 V + Coast") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        SUBCASE("the IMU never readies: ImuNotReady, nothing powered") {
            shulib::sim::ImuHostileConfig ic;
            ic.calibrationEnd = Time{1e9};
            shulib::sim::ImuHostileModel imu{ic};
            Plan p;
            p.faster = f;
            p.degradation = &imu;
            Rig rig{p};
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::ImuNotReady);
            CHECK(o.poweredTicks == 0);
            CHECK(o.coastViolations == 0);
            CHECK(rig.ch->report().segmentCount == 0U);
            // imuReadyTimeout (5 s) and not much later.
            CHECK(rig.h.clock().now().value() > p.cfg.imuReadyTimeout.value());
            CHECK(rig.h.clock().now().value() < p.cfg.imuReadyTimeout.value() + 2.5 * kDt);
        }
        SUBCASE("a frozen moving reading after a segment: RestTimeout naming the port, after the "
                "stop") {
            Plan p;
            p.faster = f;
            Rig rig{p};
            const Out o = run(rig, [](Rig& r, int) {
                if (r.ch->phase() == CharacterizerPhase::Stopping) {
                    r.port(18).setVelocity(AngularVelocity{0.3});  // over restFloor, forever
                }
            });
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::RestTimeout);
            CHECK(rig.ch->refusalPort() == 18);
            CHECK(std::strstr(rig.ch->reason(), "after a segment") != nullptr);
            CHECK(rig.ch->report().segmentCount == 1U);
            CHECK_FALSE(rig.ch->report().segments[0].stopped);
            CHECK(rig.ch->report().segments[0].timing.count > 300U);  // the wait's ticks, kept
            // It braked the whole wait, then coasted on the refusal tick.
            CHECK(o.brakeViolations == 0);
            CHECK(o.coastViolations == 0);
            CHECK(o.brakeTicks * kDt > p.cfg.restTimeout.value());
        }
        SUBCASE("still rolling past stopBound when restTimeout runs out: StopTooLong, still "
                "rolling") {
            Plan p;
            p.faster = f;
            // τ = kA/kV = 1.5 s: after 3 s of the plant's decay the robot is still moving at ~e^-2
            // of its end speed, and has rolled far past 12 in.
            p.fast = FeedforwardGains{.kS = 1.0, .kV = 0.160, .kA = 0.240};
            p.slow = FeedforwardGains{.kS = 1.1, .kV = 0.168, .kA = 0.252};
            Rig rig{p};
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::StopTooLong);
            CHECK(std::strstr(rig.ch->reason(), "still rolling") != nullptr);
            CHECK_FALSE(rig.ch->report().segments[0].stopped);
            CHECK(rig.ch->report().segmentCount == 1U);
            CHECK(o.brakeViolations == 0);
            CHECK(o.coastViolations == 0);
        }
        SUBCASE("a stop past stopBound that DOES come to rest: StopTooLong at rest, not still "
                "rolling") {
            Plan p;
            p.faster = f;
            // τ = 0.4 s and a faster drive (free speed ~92 in/s): ~35 in/s at the ramp's end rolls
            // ~14 in, and is under the rest floor in ~2.4 s — inside restTimeout.
            p.fast = FeedforwardGains{.kS = 1.0, .kV = 0.120, .kA = 0.048};
            p.slow = FeedforwardGains{.kS = 1.1, .kV = 0.126, .kA = 0.0504};
            Rig rig{p};
            const Out o = run(rig);
            const std::string reasonText = rig.ch->reason();
            CAPTURE(reasonText);
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::StopTooLong);
            CHECK(std::strstr(rig.ch->reason(), "still rolling") == nullptr);
            const auto& seg = rig.ch->report().segments[0];
            CHECK(seg.stopped);  // it reached rest: the distance is final
            CHECK(std::max(std::abs(seg.stopIn[0]), std::abs(seg.stopIn[1])) > p.cfg.stopBound.value());
            for (std::size_t side = 0; side < 2; ++side) {
                CHECK(seg.stopIn[side] == rel(o.stop[0][side], 1e-9));
            }
            CHECK(rig.ch->report().segmentCount == 1U);
            CHECK(o.brakeViolations == 0);
            CHECK(o.coastViolations == 0);
        }
        SUBCASE("a fit that is not Ok: FitRefused naming each side's verdict, and no result") {
            Plan p;
            p.faster = f;
            // kS ranges up to 1.05 V: the faster side (kS 1.0) is inside, the slower (1.1) is not.
            p.cfg.fit.kSMax = 1.05;
            Rig rig{p};
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Aborted);
            CHECK(rig.ch->refusal() == CharacterizerRefusal::FitRefused);
            const std::size_t slow = f == Faster::Left ? 1 : 0;
            CHECK(rig.ch->report().fit[slow].verdict == FitVerdict::Implausible);
            CHECK(rig.ch->report().fit[1 - slow].verdict == FitVerdict::Ok);
            const std::string reason = rig.ch->reason();
            CHECK(reason.find(slow == 1 ? "RIGHT IMPLAUSIBLE" : "LEFT IMPLAUSIBLE") != std::string::npos);
            CHECK(reason.find(slow == 1 ? "LEFT OK" : "RIGHT OK") != std::string::npos);
            CHECK(rig.ch->report().segmentCount == kSegmentCount);  // every segment ran
            CHECK(rig.ch->report().segments[3].stopped);  // and its record is the finished one
            CHECK(o.coastViolations == 0);
            CHECK(o.brakeViolations == 0);
            CHECK_THROWS_AS((void)rig.ch->result(), PreconditionError);
        }
    }
}

// ═══ C21. ONE side's stop too long ══════════════════════════════════════════════════
// Bug caught (the R5a review of 2026-10-04): StopTooLong judged on the LEFT side only, or on the
// MEAN of the sides — every earlier case rolled BOTH sides (or their mean) past the bound, so both
// survived. Here one side brakes short (τ = 0.10 s) and the other rolls on (τ = 0.45 s): its stop
// passes stopBound while the other's is a fraction of it and the mean is under it. Both
// orientations — the long side is LEFT once and RIGHT once.
TEST_CASE("DriveCharacterizer C21: one side rolling past stopBound while the other stops short "
          "refuses StopTooLong before the next segment, on either side") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        p.fast = FeedforwardGains{.kS = 1.0, .kV = 0.160, .kA = 0.160 * 0.45};  // rolls on
        p.slow = FeedforwardGains{.kS = 1.0, .kV = 0.168, .kA = 0.168 * 0.10};  // brakes short
        Rig rig{p};
        const Out o = run(rig);
        const std::string reasonText = rig.ch->reason();
        CAPTURE(reasonText);
        REQUIRE(o.end == CharacterizerPhase::Aborted);
        CHECK(rig.ch->refusal() == CharacterizerRefusal::StopTooLong);
        const std::size_t longSide = f == Faster::Left ? 0 : 1;
        const std::uint8_t n = rig.ch->report().segmentCount;
        REQUIRE(n >= 1U);
        const SegmentRecord& seg = rig.ch->report().segments[n - 1];
        const double bound = p.cfg.stopBound.value();
        CHECK(std::abs(seg.stopIn[longSide]) > bound);
        CHECK(std::abs(seg.stopIn[1 - longSide]) < 0.5 * bound);
        CHECK(0.5 * (std::abs(seg.stopIn[0]) + std::abs(seg.stopIn[1])) < bound);
        // Every EARLIER segment's stops were inside the bound — so this one refused, and no
        // segment ran after it.
        for (std::uint8_t s = 0; s + 1 < n; ++s) {
            CHECK(std::abs(rig.ch->report().segments[s].stopIn[longSide]) <= bound);
        }
        char quoted[40];  // the reason quotes the long side's stop, as the record keeps it
        std::snprintf(quoted, sizeof quoted, "%s %+.1f in", longSide == 0 ? "LEFT" : "RIGHT",
                      seg.stopIn[longSide] + 0.0);
        CHECK(std::strstr(rig.ch->reason(), quoted) != nullptr);
        CHECK(o.brakeViolations == 0);
        CHECK(o.coastViolations == 0);
    }
}

// ═══ C22. The segment's other ends: RampTop and Timeout ═════════════════════════════
// Bug caught (the R5a review of 2026-10-04): no case ended a segment any way but Distance, so the
// segmentTimeout and rampMax ends could be removed with every case green — and a drive that cannot
// cover segmentDistance would then be powered until the battery died. A drive whose kS (7 V) is
// above stepVolts (6 V): the ramps creep only past 7 V and reach rampMax first; the steps cannot
// move it at all and run out the clock. Each ends normally — braked, then coasted — and the fit,
// which saw kS = 7 V, is refused as Implausible. The step is powered segmentTimeout and no longer:
// this is the bounded exposure the header's "What it does NOT refuse" describes.
TEST_CASE("DriveCharacterizer C22: a drive that cannot cover the distance ends its ramps at RampTop "
          "and its steps at Timeout, each braked then coasted") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        p.fast = FeedforwardGains{.kS = 7.0, .kV = 0.160, .kA = 0.020};
        p.slow = FeedforwardGains{.kS = 7.0, .kV = 0.168, .kA = 0.024};
        Rig rig{p};
        const Out o = run(rig);
        const std::string reasonText = rig.ch->reason();
        CAPTURE(reasonText);
        REQUIRE(o.end == CharacterizerPhase::Aborted);
        CHECK(rig.ch->refusal() == CharacterizerRefusal::FitRefused);
        const auto& rep = rig.ch->report();
        REQUIRE(rep.segmentCount == kSegmentCount);
        const double rampTop = p.cfg.rampMax / p.cfg.rampRate;  // 5 s
        for (std::size_t s = 0; s < 2; ++s) {
            CAPTURE(s);
            const SegmentRecord& seg = rep.segments[s];
            CHECK(seg.end == SegmentEnd::RampTop);
            CHECK(seg.stopped);
            // The end tick commands nothing: the last volts are one tick's ramp under the top.
            CHECK(std::abs(seg.lastVolts) < p.cfg.rampMax);
            CHECK(std::abs(seg.lastVolts) >= p.cfg.rampMax - p.cfg.rampRate * kDt * 1.5);
            CHECK(seg.lastVolts * (s == 0 ? 1.0 : -1.0) > 0.0);
            CHECK(seg.seconds >= rampTop - 1e-9);
            CHECK(seg.seconds < rampTop + 1.5 * kDt);
            CHECK(std::abs(seg.travelIn[0]) < p.cfg.segmentDistance.value());
            CHECK(std::abs(seg.travelIn[0]) > 5.0);  // it crept, past 7 V
        }
        for (std::size_t s = 2; s < 4; ++s) {
            CAPTURE(s);
            const SegmentRecord& seg = rep.segments[s];
            CHECK(seg.end == SegmentEnd::Timeout);
            CHECK(seg.stopped);
            CHECK(seg.lastVolts == (s == 2 ? 1.0 : -1.0) * p.cfg.stepVolts);
            CHECK(seg.seconds >= p.cfg.segmentTimeout.value() - 1e-9);
            CHECK(seg.seconds < p.cfg.segmentTimeout.value() + 1.5 * kDt);
            CHECK(std::abs(seg.travelIn[0]) < 0.01);  // 6 V under kS 7 V: nothing moved
            CHECK(seg.admitted[0] == 0U);
            CHECK(seg.admitted[1] == 0U);
        }
        CHECK(rep.fit[0].verdict == FitVerdict::Implausible);
        CHECK(rep.fit[1].verdict == FitVerdict::Implausible);
        // Powered no longer than the four ends allow, and every end braked then coasted.
        CHECK(o.poweredTicks * kDt <= 2.0 * (rampTop + 2.0 * kDt)
                                         + 2.0 * (p.cfg.segmentTimeout.value() + 2.0 * kDt));
        requireCleanTicks(o);
    }
}

// ═══ C23. Late ticks while driving ══════════════════════════════════════════════════
// Bug caught (the R5a review of 2026-10-04): every case stepped the plant by exactly kDt, so the
// characterizer could hand the fit a FIXED nominal dt instead of the clock's with every case green
// — and on the robot a late tick (HA-34) would then enter the fit as a sample of the wrong law.
// Here every 25th driving tick comes 20 ms after the last (under commandTimeout's 100 ms, so not a
// missed tick): each such sample is excluded OffNominalDt on BOTH sides, each such tick is counted
// over budget in its segment's timing, and the gains stay exact.
TEST_CASE("DriveCharacterizer C23: a late tick while driving is excluded as OffNominalDt on both "
          "sides and counted over budget; the gains stay exact") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        Plan p;
        p.faster = f;
        Rig rig{p};
        int late[4] = {0, 0, 0, 0};
        int driving = 0;
        const Out o = run(rig, [&](Rig& r, int) {
            if (r.ch->phase() != CharacterizerPhase::Driving) {
                return;
            }
            if (++driving % 25 == 0) {
                r.h.plant().step(Time{kDt});  // this tick comes one period late
                ++late[std::min<int>(r.ch->status().segment, 3)];
            }
        });
        const std::string reasonText = rig.ch->reason();
        CAPTURE(reasonText);
        REQUIRE(o.end == CharacterizerPhase::Done);
        const auto& rep = rig.ch->report();
        const auto dt = static_cast<std::size_t>(FitExclusion::OffNominalDt) - 1;
        int total = 0;
        for (std::size_t s = 0; s < kSegmentCount; ++s) {
            CAPTURE(s);
            CHECK(late[s] > 0);
            CHECK(rep.segments[s].excluded[0][dt] == static_cast<std::uint32_t>(late[s]));
            CHECK(rep.segments[s].excluded[1][dt] == static_cast<std::uint32_t>(late[s]));
            CHECK(rep.segments[s].timing.overBudget == static_cast<std::uint32_t>(late[s]));
            // A 20 ms gap is EXACTLY twice nominal only to the last bit of two summed clock reads,
            // so it may or may not count as over 2× (T2 pins that boundary exactly); no more than
            // the late ticks ever do.
            CHECK(rep.segments[s].timing.overDouble <= static_cast<std::uint32_t>(late[s]));
            CHECK(rep.segments[s].timing.maxS < 2.0 * kDt + 1e-9);
            total += late[s];
        }
        CHECK(rep.timing.overBudget == static_cast<std::uint32_t>(total));
        CHECK(rep.fit[0].excluded[dt] == static_cast<std::uint32_t>(total));
        requireSideGains(rep.fit[0], p.leftGains(), 1e-6);
        requireSideGains(rep.fit[1], p.rightGains(), 1e-6);
        requireCleanTicks(o);
    }
}

namespace {

/// A battery that sags and ripples (a deterministic function of time — INVENTED; HA-46/HA-47 are
/// the real pack's), so a segment's minimum and mean are not one repeated number.
struct SaggingPack final : shulib::sim::DegradationModel {
    shulib::units::Voltage batteryVoltage(shulib::units::Voltage, Time now,
                                          shulib::sim::Rng&) override {
        const double t = now.value();
        return shulib::units::Voltage{12.8 - 0.02 * t + 0.15 * std::sin(9.0 * t)};
    }
};

}  // namespace

// ═══ C24. The battery, the current column and the raw sample ════════════════════════
// Bug caught (the R5a review of 2026-10-04): with a constant battery and no member current, a
// segment's battery MEAN taken as its last reading, its MINIMUM as its maximum, and the raw sample's
// current, position or heading column zeroed all survived. Part C's CSV records lastSample(), and
// its position column is what the laptop re-fit differences against velocity() (HA-171). Here the
// battery sags and ripples, every member reports its own current, and run() checks every sample
// against the hardware; each segment's battery numbers must be exactly the readings it saw — its
// first tick's included, so the minimum and the mean are over the SAME readings.
TEST_CASE("DriveCharacterizer C24: each segment's battery min and mean are exactly its readings, "
          "and every raw sample is the hardware's") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        SaggingPack pack;
        Plan p;
        p.faster = f;
        p.degradation = &pack;
        Rig rig{p};
        const Out o = run(rig, [](Rig& r, int i) {
            for (std::size_t s = 0; s < 2; ++s) {
                for (std::size_t k = 0; k < Rig::kPer; ++k) {
                    // Under the 2.3 A guard, different on every member and every tick.
                    const double amps = 0.3 + 0.2 * static_cast<double>(k) + 0.4 * static_cast<double>(s)
                                        + 0.1 * std::sin(0.37 * i + static_cast<double>(k));
                    r.member(static_cast<int>(s), k).setCurrent(Current{(k % 2 == 0 ? 1.0 : -1.0) * amps});
                }
            }
        });
        const std::string reasonText = rig.ch->reason();
        CAPTURE(reasonText);
        REQUIRE(o.end == CharacterizerPhase::Done);
        const auto& rep = rig.ch->report();
        for (std::size_t s = 0; s < kSegmentCount; ++s) {
            CAPTURE(s);
            const SegmentRecord& seg = rep.segments[s];
            CHECK(o.battN[s] > 100);
            CHECK(seg.batteryMin == o.battMin[s]);
            CHECK(seg.batteryMean == o.battSum[s] / static_cast<double>(o.battN[s]));
            CHECK(seg.batteryMean > seg.batteryMin + 0.05);  // the ripple: min and mean differ
        }
        requireCleanTicks(o);  // every raw sample matched the motors, the IMU and the pack
        requireSideGains(rep.fit[0], p.leftGains(), 1e-6);  // no current reached the guard
        requireSideGains(rep.fit[1], p.rightGains(), 1e-6);
    }
}

namespace {

/// A wall in front of the robot: every wheel's FORWARD volts deliver nothing (a stalled train does
/// not turn); backward is free. A test fixture, not a plant change.
struct WallAhead final : shulib::sim::DegradationModel {
    Voltage effectiveVoltage(int, Voltage commanded, Time, shulib::sim::Rng&) override {
        return commanded.value() > 0.0 ? Voltage{0.0} : commanded;
    }
};

}  // namespace

// ═══ C25. A segment that measured nothing on a side ═════════════════════════════════
// Bug caught (the R5a review of 2026-10-04): a robot blocked in front — a wall, a foot — ramped to
// 10 V for 5 s and held 6 V for 8 s against the block, admitted nothing forward, and finished Done
// on its backward segments alone with an EMPTY warning. It is still not refused (whether it should
// be is the team lead's: the header's "What it does NOT refuse"), but it is no longer silent: every
// segment that admitted no sample on a side sets emptySegmentWarning, and warning() leads with the
// first such segment and side. A side held at its current limit through a whole segment warns the
// same way — on THAT side only. Both orientations; with the asymmetry warning too, both are kept.
TEST_CASE("DriveCharacterizer C25: a segment that admitted no sample on a side finishes Done with a "
          "warning naming it -- a wall ahead, one side at its current limit, and with the asymmetry "
          "warning") {
    for (Faster f : kBoth) {
        const std::string orient = name(f);
        CAPTURE(orient);
        SUBCASE("a wall ahead: both forward segments measure nothing, on both sides") {
            WallAhead wall;
            Plan p;
            p.faster = f;
            p.degradation = &wall;
            // No slipping member: against a wall it would spin free while its stalled mates read 0,
            // and the fight monitor would (rightly) refuse — that is C12's business, not this case's.
            p.decoupled = false;
            Rig rig{p};
            const Out o = run(rig);
            const std::string reasonText = rig.ch->reason();
            CAPTURE(reasonText);
            REQUIRE(o.end == CharacterizerPhase::Done);
            const auto& rep = rig.ch->report();
            CHECK(rep.segments[0].end == SegmentEnd::RampTop);
            CHECK(rep.segments[2].end == SegmentEnd::Timeout);
            for (std::size_t s : {std::size_t{0}, std::size_t{2}}) {
                CAPTURE(s);
                for (std::size_t side = 0; side < 2; ++side) {
                    CHECK(rep.segments[s].admitted[side] == 0U);
                    CHECK(std::abs(rep.segments[s].travelIn[side]) < 0.01);
                    CHECK(rep.segments[s + 1].admitted[side] > 20U);  // backward: measured
                }
            }
            CHECK(rep.emptySegmentWarning);
            CHECK_FALSE(rep.asymmetryWarning);
            const std::string w = rig.ch->warning();
            CAPTURE(w);
            CHECK(w.rfind("QuasiForward admitted no LEFT sample (travel +0.0 in, ended on ramp top), "
                          "and 3 more segment-sides likewise",
                          0) == 0);  // the FIRST empty segment-side is named, and the rest counted
            CHECK(w.find("something in the way") != std::string::npos);
            CHECK(w.find("pct") == std::string::npos);
            // The backward segments alone still fit the plant's gains (the warning is about what
            // the fit did NOT see, not about what it got wrong).
            requireSideGains(rep.fit[0], p.leftGains(), 1e-6);
            requireSideGains(rep.fit[1], p.rightGains(), 1e-6);
            CHECK(rig.ch->result().source == shulib::calibration::CharacterizationSource::Measured);
            requireCleanTicks(o);
        }
        SUBCASE("one side at its current limit through StepForward: that side only") {
            for (int port : {12, 17}) {
                CAPTURE(port);
                const std::size_t side = port <= 15 ? 0 : 1;
                Plan p;
                p.faster = f;
                Rig rig{p};
                const Out o = run(rig, [port](Rig& r, int) {
                    const bool on = r.ch->phase() == CharacterizerPhase::Driving
                                    && r.ch->status().segment == 2;
                    r.port(port).setCurrent(Current{on ? 2.5 : 1.0});
                });
                REQUIRE(o.end == CharacterizerPhase::Done);
                const auto& rep = rig.ch->report();
                CHECK(rep.segments[2].admitted[side] == 0U);
                CHECK(rep.segments[2].admitted[1 - side] > 20U);
                CHECK(rep.emptySegmentWarning);
                const std::string w = rig.ch->warning();
                CAPTURE(w);
                CHECK(w.rfind(side == 0 ? "StepForward admitted no LEFT sample (travel +3"
                                        : "StepForward admitted no RIGHT sample (travel +3",
                              0) == 0);
                CHECK(w.find("ended on distance)") != std::string::npos);
                CHECK(w.find("more segment-sides") == std::string::npos);  // only the one
                requireSideGains(rep.fit[0], p.leftGains(), 1e-6);
                requireSideGains(rep.fit[1], p.rightGains(), 1e-6);
            }
        }
        SUBCASE("a wall ahead AND sides 20 pct apart: both warnings, the empty segment first") {
            WallAhead wall;
            Plan p;
            p.faster = f;
            p.degradation = &wall;
            p.decoupled = false;  // as above
            p.slow = FeedforwardGains{.kS = 1.0, .kV = 0.195, .kA = 0.024};  // C15's
            Rig rig{p};
            const Out o = run(rig);
            REQUIRE(o.end == CharacterizerPhase::Done);
            const auto& rep = rig.ch->report();
            CHECK(rep.emptySegmentWarning);
            CHECK(rep.asymmetryWarning);
            const std::string w = rig.ch->warning();
            CAPTURE(w);
            CHECK(w.rfind("QuasiForward admitted no LEFT sample", 0) == 0);
            const std::size_t join = w.find("; the sides' kV differ by ");
            REQUIRE(join != std::string::npos);
            CHECK(w.find("something in the way") < join);
            // Neither cut short: the asymmetry text runs to its end.
            CHECK(w.size() > join + 20);
            CHECK(w.compare(w.size() - std::strlen("not a tuning"), std::string::npos,
                            "not a tuning") == 0);
        }
    }
}

// ═══ C19. The refusal codes ═════════════════════════════════════════════════════════
// Self-contained (the R5a review: it used to check strings the other cases had collected, and
// failed when run alone). The strings themselves are checked as they are produced, in run().
TEST_CASE("DriveCharacterizer C19: every refusal code has a short ASCII name") {
    for (int c = 0; c <= static_cast<int>(CharacterizerRefusal::FitRefused); ++c) {
        const std::string n = characterizerRefusalName(static_cast<CharacterizerRefusal>(c));
        CAPTURE(n);
        CHECK(n != "?");
        CHECK(screenSafe(n));
    }
    for (int e = 0; e <= static_cast<int>(SegmentEnd::Timeout); ++e) {
        CHECK(std::string(shulib::calibration::segmentEndName(static_cast<SegmentEnd>(e))) != "?");
    }
}
