// The wall reset's GATES and its CALL'S CONTRACT (chunk L6-A1; the L6 brief's tests T3, T4, T5, T9,
// T10 and T6's second half). Each gate refuses its own case and only it, at its exact edge; a
// refused aim costs no time and reads nothing; a held value is never a reading; one call writes one
// line, commands nothing to the drive and returns within its window; a refusal changes nothing.
//
// The pure cases use an author's own wall line at x = 0 and a lens AT the robot's centre, so every
// edge lands on an exact binary value (the coordinator's rule: a test that claims an EQUALITY edge
// must land on it exactly). The call cases run on the tank rig over the host field plant
// (wall_reset_rig.hpp).

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "shulib/chassis/routine.hpp"
#include "shulib/chassis/wall_reset_call.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/distance.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/motion/hold_pose.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "wall_reset_rig.hpp"

namespace {

namespace loc = shulib::localization;
namespace ow = shulib::localization::override2026;
using loc::WallResetVerdict;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::math::Twist2d;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using wall_rig::kD2R;
using wall_rig::kPi;

// ── the pure fixture: an own line at x = 0 (the field at x > 0), a lens at the robot centre ─────

loc::FieldWall line0() { return loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1); }

loc::SideMount lens0() {
    loc::SideMount m{};
    m.measured = true;
    m.forward = Length{0.0};
    m.left = Length{0.0};
    m.height = Length{10.5};
    m.facing = AngleDim{90.0 * loc::kWallDegToRad};
    return m;
}

loc::WallResetRequest req0(double mc = 3.0) {
    loc::WallResetRequest r{};
    r.wall = line0();
    r.sensor = loc::WallSide::Left;
    r.maxCorrection = Length{mc};
    return r;
}

/// The robot at x = `px`, y = -38, nose along +y (the left lens's beam points -x, square).
Pose2d at(double px, double headingDeg = 90.0) {
    return Pose2d{Length{px}, Length{-38.0}, Angle::degrees(headingDeg)};
}

/// A window of samples, `dt` apart (exact binary by default), each `conf`.
std::vector<loc::WallSample> window(std::initializer_list<double> readings, double conf = 1.0,
                                    double dt = 0.03125) {
    std::vector<loc::WallSample> s;
    double t = 0.0;
    for (const double r : readings) {
        s.push_back(loc::WallSample{Length{r}, conf, false, Time{t}});
        t += dt;
    }
    return s;
}

loc::WallResetLimits lim0() {
    loc::WallResetLimits l{};
    l.window = Time{0.125};  // exact binary: five samples 0.03125 apart span it exactly
    return l;
}

loc::WallResetReport eval(const std::vector<loc::WallSample>& s, const Pose2d& pose,
                          const loc::WallResetRequest& rq, const loc::WallResetLimits& l,
                          const loc::SideMount& m = lens0(), const Twist2d& tw = Twist2d{}) {
    return loc::evaluateWallReset(std::span<const loc::WallSample>{s.data(), s.size()}, pose, tw, m, rq, l,
                                  loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
}

const auto kFive = {7.0, 7.0, 7.0, 7.0, 7.0};

}  // namespace

// Bug caught: a square gate whose comparison is flipped (M7) or off by its edge — 4.9 deg passes,
// 5.1 refuses NotSquare, and a beam pointing AWAY from the wall (the wrong side) is NotSquare too.
TEST_CASE("L6 T3 square: 4.9 deg passes, 5.1 deg refuses NotSquare; a beam facing away is NotSquare") {
    const loc::WallResetLimits l = lim0();
    CHECK(loc::preCheck(at(7.0, 94.9), lens0(), req0(), l, Length{0.0}) == WallResetVerdict::Seated);
    CHECK(loc::preCheck(at(7.0, 85.1), lens0(), req0(), l, Length{0.0}) == WallResetVerdict::Seated);
    CHECK(loc::preCheck(at(7.0, 95.1), lens0(), req0(), l, Length{0.0}) == WallResetVerdict::NotSquare);
    CHECK(loc::preCheck(at(7.0, 84.9), lens0(), req0(), l, Length{0.0}) == WallResetVerdict::NotSquare);
    CHECK(loc::preCheck(at(7.0, -90.0), lens0(), req0(), l, Length{0.0}) == WallResetVerdict::NotSquare);
    CHECK(loc::preCheck(at(7.0, 0.0), lens0(), req0(), l, Length{0.0}) == WallResetVerdict::NotSquare);
    loc::WallAim aim = loc::aimAt(at(7.0, -90.0), lens0(), line0(), l, Length{0.0});
    CHECK(aim.offSquare.value() == doctest::Approx(kPi).epsilon(1e-12));
}

// Bug caught: the footprint widening dropped (M8) — a beam centre 1 in OUTSIDE a span whose
// footprint (7 x tan 18 deg = 2.27 in) overlaps it must refuse; just past the footprint, it passes.
// The margin and the drift are zeroed so the footprint alone decides.
TEST_CASE("L6 T3 keep-out: a footprint that overlaps a span refuses, even with the centre outside it") {
    loc::WallResetLimits l = lim0();
    l.keepOutMargin = Length{0.0};
    const double half = 7.0 * std::tan(18.0 * loc::kWallDegToRad);
    const std::array<loc::WallKeepOut, 1> near{
        loc::WallKeepOut{Length{-37.0}, Length{-33.0}, loc::KeepOutKind::Fixed, "Loader"}};
    loc::WallResetRequest rq = req0();
    rq.wall = loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1, near);
    CHECK(loc::preCheck(at(7.0), lens0(), rq, l, Length{0.0}) == WallResetVerdict::KeepOut);
    const std::array<loc::WallKeepOut, 1> clear{
        loc::WallKeepOut{Length{-38.0 + half + 0.01}, Length{-33.0}, loc::KeepOutKind::Fixed, "Loader"}};
    rq.wall = loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1, clear);
    CHECK(loc::preCheck(at(7.0), lens0(), rq, l, Length{0.0}) == WallResetVerdict::Seated);
    // The margin widens it too, and so does the along drift (RC33).
    l.keepOutMargin = Length{1.0};
    CHECK(loc::preCheck(at(7.0), lens0(), rq, l, Length{0.0}) == WallResetVerdict::KeepOut);
    l.keepOutMargin = Length{0.0};
    CHECK(loc::preCheck(at(7.0), lens0(), rq, l, Length{0.02}) == WallResetVerdict::KeepOut);
    CHECK(loc::preCheck(at(7.0), lens0(), rq, l, Length{0.0}) == WallResetVerdict::Seated);
}

// Bug caught (RC34): the beam-height gate removed (M36) or off its edge — an expected range AT the
// ceiling passes, +0.01 in refuses BeamLow before any sample; the ceiling is (height - 6.5 - 0.5) /
// tan(cone), with the MEASURED cone when typed and the limits' 18 deg when not.
TEST_CASE("L6 T3 beam height: at the ceiling passes, +0.01 in refuses BeamLow; the cone used") {
    const loc::WallResetLimits l = lim0();
    const loc::SideMount m = lens0();
    const double ceiling = loc::beamCeiling(m, l).value();
    CHECK(ceiling == (10.5 - 6.5 - 0.5) / std::tan(18.0 * loc::kWallDegToRad));
    // The lens is AT the robot centre (left 0), square: the expected range is exactly px.
    CHECK(loc::aimAt(at(ceiling), m, line0(), l, Length{0.0}).expectedRange.value() == ceiling);
    CHECK(loc::preCheck(at(ceiling), m, req0(), l, Length{0.0}) == WallResetVerdict::Seated);
    CHECK(loc::preCheck(at(ceiling + 0.01), m, req0(), l, Length{0.0}) == WallResetVerdict::BeamLow);
    loc::SideMount measured = m;
    measured.coneHalfAngle = AngleDim{12.0 * loc::kWallDegToRad};
    CHECK(loc::beamCeiling(measured, l).value() == (10.5 - 6.5 - 0.5) / std::tan(12.0 * loc::kWallDegToRad));
    CHECK(loc::preCheck(at(ceiling + 0.01), measured, req0(), l, Length{0.0}) == WallResetVerdict::Seated);
    // A lens at or under 7 in has no ceiling at all.
    loc::SideMount low = m;
    low.height = Length{7.0};
    CHECK(loc::beamCeiling(low, l).value() == 0.0);
    low.height = Length{4.0};
    CHECK(loc::beamCeiling(low, l).value() == 0.0);
    CHECK(loc::preCheck(at(3.0), low, req0(), l, Length{0.0}) == WallResetVerdict::BeamLow);
}

// Bug caught (RC34, M37): the median's ceiling check removed — an estimate that put the wall NEARER
// than it is passes rule 4, and what the sensor saw (past the ceiling: a piece could be the nearest
// surface) must still refuse BeamLow.
TEST_CASE("L6 T3 beam height: an expected range under the ceiling with a median past it is BeamLow") {
    const loc::WallResetLimits l = lim0();
    const double ceiling = loc::beamCeiling(lens0(), l).value();
    const Pose2d pose = at(ceiling - 0.5);
    REQUIRE(loc::preCheck(pose, lens0(), req0(), l, Length{0.0}) == WallResetVerdict::Seated);
    const double r = ceiling + 0.01;
    const loc::WallResetReport rep = eval(window({r, r, r, r, r}), pose, req0(), l);
    CHECK(rep.verdict == WallResetVerdict::BeamLow);
    const double ok = ceiling;  // AT the ceiling (bias 0): passes
    CHECK(eval(window({ok, ok, ok, ok, ok}), pose, req0(), l).verdict == WallResetVerdict::Seated);
}

// Bug caught (RC34, M38): the piece zones' rule inverted — a PieceZone span applies while the cone is
// UNMEASURED and lifts once a measured cone is typed; a Toggle span lifts only by toggleChecked; a
// Fixed span never lifts.
TEST_CASE("L6 T3 keep-out kinds: PieceZone lifts with a measured cone, Toggle with the flag, Fixed never") {
    const loc::WallResetLimits l = lim0();
    for (const loc::KeepOutKind kind :
         {loc::KeepOutKind::PieceZone, loc::KeepOutKind::Toggle, loc::KeepOutKind::Fixed}) {
        const std::array<loc::WallKeepOut, 1> span{
            loc::WallKeepOut{Length{-39.0}, Length{-37.0}, kind, "span"}};
        loc::WallResetRequest rq = req0();
        rq.wall = loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1, span);
        loc::SideMount plain = lens0();
        loc::SideMount coned = lens0();
        coned.coneHalfAngle = AngleDim{12.0 * loc::kWallDegToRad};
        loc::SideMount flagged = lens0();
        flagged.toggleChecked = true;
        const auto v = [&](const loc::SideMount& m) { return loc::preCheck(at(7.0), m, rq, l, Length{0.0}); };
        CHECK(v(plain) == WallResetVerdict::KeepOut);
        CHECK(v(coned) == (kind == loc::KeepOutKind::PieceZone ? WallResetVerdict::Seated : WallResetVerdict::KeepOut));
        CHECK(v(flagged) == (kind == loc::KeepOutKind::Toggle ? WallResetVerdict::Seated : WallResetVerdict::KeepOut));
    }
}

// Bug caught: a stillness gate off its edge (0.99 in/s passes, 1.01 refuses Moving; 2.9 deg/s passes,
// 3.1 refuses), or judged once per window instead of on every tick (M9 — a push late in the window).
TEST_CASE("L6 T3 stillness: 0.99 / 1.01 in/s and 2.9 / 3.1 deg/s; a push late in the window is seen") {
    loc::WallResetLimits l = lim0();
    const auto tw = [](double v, double wDeg) {
        return Twist2d{Velocity{v}, Velocity{0.0}, AngularVelocity{wDeg * kD2R}};
    };
    CHECK(eval(window(kFive), at(7.0), req0(), l, lens0(), tw(0.99, 0.0)).verdict == WallResetVerdict::Seated);
    CHECK(eval(window(kFive), at(7.0), req0(), l, lens0(), tw(1.01, 0.0)).verdict == WallResetVerdict::Moving);
    CHECK(eval(window(kFive), at(7.0), req0(), l, lens0(), tw(0.0, 2.9)).verdict == WallResetVerdict::Seated);
    CHECK(eval(window(kFive), at(7.0), req0(), l, lens0(), tw(0.0, 3.1)).verdict == WallResetVerdict::Moving);
    // The sampler judges EVERY tick, over the trailing `window` (0.125 s here): still for five ticks
    // (0 .. 0.125 s — a window that, with a longer minSamples, does not end yet), then pushed at a
    // steady 1.5 in/s (y moves 0.046875 in a tick: exact binary). The push is seen INSIDE `window` of
    // its start — on its third tick the last 0.125 s moved 0.140625 in: 1.125 in/s, past the limit —
    // and the window ends there.
    l.minSamples = 20;
    shulib::hal::fake::FakeDistance d;
    d.setDistance(Length{7.0});
    d.setConfidence(1.0);
    const loc::WallSensor s{&d, nullptr, lens0(), ""};
    loc::WallSampler sampler{s, l};
    const AngularVelocity still{0.0};
    double y = -38.0;
    int tick = 0;
    for (; tick < 5; ++tick) {
        CHECK_FALSE(sampler.step(Time{0.03125 * tick}, Pose2d{Length{7.0}, Length{y}, Angle::degrees(90.0)}, still));
    }
    int doneAt = -1;
    for (; tick < 20 && doneAt < 0; ++tick) {
        y += 0.046875;
        if (sampler.step(Time{0.03125 * tick}, Pose2d{Length{7.0}, Length{y}, Angle::degrees(90.0)}, still)) {
            doneAt = tick;
        }
    }
    // Pushed from tick 5 (t = 0.15625): at tick 7 the last 0.125 s (ticks 3 -> 7) moved 0.140625 in.
    CHECK(doneAt == 7);
    CHECK(sampler.maxTwist().vx().value() == 1.125);
    const loc::WallResetReport rep =
        loc::evaluateWallReset(sampler.samples(), at(7.0), sampler.maxTwist(), lens0(), req0(), l,
                               loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
    CHECK(rep.verdict == WallResetVerdict::Moving);
    CHECK(rep.speedSeen.value() == 1.125);
    // A turn is judged on its own tick: 3.1 deg/s once ends the window at once.
    loc::WallSampler turning{s, l};
    CHECK_FALSE(turning.step(Time{0.0}, at(7.0), still));
    CHECK(turning.step(Time{0.03125}, at(7.0), AngularVelocity{3.1 * kD2R}));
}

// Bug caught (the L6-A1 review): stillness judged from ONE tick's finite-difference velocity. A still
// robot's estimate jitters about a fixed point (a still IMU's heading noise, moved onto the pose by
// tracking wheels off the turning centre — the register's HA-21): 0.02 in back and forth every tick
// is 2 in/s tick to tick, yet the robot is not going anywhere. Judged over the trailing `window` it
// reads at most 0.16 in/s and the window passes; a STEADY 1.01 in/s is still refused.
TEST_CASE("L6 T3 stillness: a pose jittering about a fixed point is still; a steady drift is not") {
    loc::WallResetLimits l = lim0();
    shulib::hal::fake::FakeDistance d;
    d.setDistance(Length{7.0});
    d.setConfidence(1.0);
    const loc::WallSensor s{&d, nullptr, lens0(), ""};
    const AngularVelocity still{0.0};
    loc::WallSampler jitter{s, l};
    int doneAt = -1;
    for (int tick = 0; tick < 64 && doneAt < 0; ++tick) {
        const double y = -38.0 + ((tick % 2) == 0 ? 0.0 : 0.02);
        if (jitter.step(Time{0.01 * tick}, Pose2d{Length{7.0}, Length{y}, Angle::degrees(90.0)}, still)) {
            doneAt = tick;
        }
    }
    CHECK(doneAt == 13);  // the window filled (t = 0.13 >= 0.125, 14 samples): not refused
    CHECK(jitter.maxTwist().vx().value() <= 0.02 / 0.125 + 1e-12);
    const loc::WallResetReport ok =
        loc::evaluateWallReset(jitter.samples(), at(7.0), jitter.maxTwist(), lens0(), req0(), l,
                               loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
    CHECK(ok.verdict == WallResetVerdict::Seated);
    // A steady drift at 1.01 in/s: refused once the window is `window` old.
    loc::WallSampler drift{s, l};
    doneAt = -1;
    for (int tick = 0; tick < 64 && doneAt < 0; ++tick) {
        const double y = -38.0 + 1.01 * 0.03125 * tick;
        if (drift.step(Time{0.03125 * tick}, Pose2d{Length{7.0}, Length{y}, Angle::degrees(90.0)}, still)) {
            doneAt = tick;
        }
    }
    CHECK(doneAt == 4);  // t = 0.125: the window's age reached `window` and the drift was seen
    const loc::WallResetReport moving =
        loc::evaluateWallReset(drift.samples(), at(7.0), drift.maxTwist(), lens0(), req0(), l,
                               loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
    CHECK(moving.verdict == WallResetVerdict::Moving);
    CHECK(moving.speedSeen.value() == doctest::Approx(1.01).epsilon(1e-12));
}

// Bug caught (the L6-A1 verification round): a NaN turn rate or a NaN pose judged STILL. The sampler
// keeps the window's worst speed and turn rate with std::max, which silently DROPS a NaN (every
// comparison with it is false), so without its two guards a NaN tick changed nothing and the window
// went on sampling to windowMax and could seat. Each non-finite input must end the window AT its
// tick, with no sample taken there, the worst value +infinity, and the verdict Moving. The rows at
// +/-infinity pass with or without the guards (std::max keeps an infinity); the NaN rows are the
// ones a dropped guard turns red. And the evaluator alone: a twist a caller hands it with a NaN is
// not still either (wallStill's comparisons are written so a NaN fails them).
TEST_CASE("L6 T3 stillness: a non-finite turn rate or pose ends the window AT that tick, judged Moving") {
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    loc::WallResetLimits l = lim0();
    l.minSamples = 20;  // the window would otherwise run on to windowMax (tick 16 at 0.03125 s)
    shulib::hal::fake::FakeDistance d;
    d.setDistance(Length{7.0});
    d.setConfidence(1.0);
    const loc::WallSensor s{&d, nullptr, lens0(), ""};
    struct Row {
        const char* what;
        int badTick;
        double turn;  // rad/s at the bad tick
        double x;     // the pose's x at the bad tick
        double y;     // the pose's y at the bad tick
        bool turnIsBad;
    };
    const Row rows[] = {
        {"NaN turn rate", 3, kNaN, 7.0, -38.0, true},
        {"NaN turn rate on the first tick", 0, kNaN, 7.0, -38.0, true},
        {"+inf turn rate", 3, kInf, 7.0, -38.0, true},
        {"-inf turn rate", 3, -kInf, 7.0, -38.0, true},
        {"NaN x", 3, 0.0, kNaN, -38.0, false},
        {"NaN y", 3, 0.0, 7.0, kNaN, false},
        {"NaN x on the first tick (the window's reference)", 0, 0.0, kNaN, -38.0, false},
        {"+inf x", 3, 0.0, kInf, -38.0, false},
        {"-inf y", 3, 0.0, 7.0, -kInf, false},
    };
    for (const Row& r : rows) {
        const std::string what{r.what};
        INFO(what);
        loc::WallSampler sampler{s, l};
        int doneAt = -1;
        for (int tick = 0; tick < 64 && doneAt < 0; ++tick) {
            const bool bad = tick == r.badTick;
            const Pose2d pose{Length{bad ? r.x : 7.0}, Length{bad ? r.y : -38.0}, Angle::degrees(90.0)};
            if (sampler.step(Time{0.03125 * tick}, pose, AngularVelocity{bad ? r.turn : 0.0})) {
                doneAt = tick;
            }
        }
        CHECK(doneAt == r.badTick);
        CHECK(sampler.samples().size() == static_cast<std::size_t>(r.badTick));  // none taken at the bad tick
        const Twist2d worst = sampler.maxTwist();
        if (r.turnIsBad) {
            CHECK(worst.omega().value() == kInf);
            CHECK(worst.vx().value() == 0.0);  // the pose never moved
        } else {
            CHECK(worst.vx().value() == kInf);
            CHECK(worst.omega().value() == 0.0);
        }
        CHECK_FALSE(loc::wallStill(worst, l));
        const loc::WallResetReport rep =
            loc::evaluateWallReset(sampler.samples(), at(7.0), worst, lens0(), req0(), l,
                                   loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
        CHECK(rep.verdict == WallResetVerdict::Moving);
        CHECK((r.turnIsBad ? rep.turnSeen.value() : rep.speedSeen.value()) == kInf);
    }
    // The evaluator handed a NaN twist directly (no sampler; the fixture's own limits, five samples
    // enough): refused Moving, never Seated.
    const loc::WallResetLimits l0 = lim0();
    const auto tw = [](double vx, double vy, double omega) {
        return Twist2d{Velocity{vx}, Velocity{vy}, AngularVelocity{omega}};
    };
    CHECK_FALSE(loc::wallStill(tw(kNaN, 0.0, 0.0), l0));
    CHECK_FALSE(loc::wallStill(tw(0.0, kNaN, 0.0), l0));
    CHECK_FALSE(loc::wallStill(tw(0.0, 0.0, kNaN), l0));
    CHECK(eval(window(kFive), at(7.0), req0(), l0, lens0(), tw(kNaN, 0.0, 0.0)).verdict == WallResetVerdict::Moving);
    CHECK(eval(window(kFive), at(7.0), req0(), l0, lens0(), tw(0.0, kNaN, 0.0)).verdict == WallResetVerdict::Moving);
    CHECK(eval(window(kFive), at(7.0), req0(), l0, lens0(), tw(0.0, 0.0, kNaN)).verdict == WallResetVerdict::Moving);
    // ... and the same window with a still twist SEATS (the rows above refuse for the NaN alone).
    CHECK(eval(window(kFive), at(7.0), req0(), l0, lens0(), tw(0.0, 0.0, 0.0)).verdict == WallResetVerdict::Seated);
}

// Bug caught: a 9999 sample skipped instead of refusing (M10), a confidence gate off its edge, a
// range gate that ignores the bias. Exact binary edges: bias 0.25, readings 2.25 / 16.25.
TEST_CASE("L6 T3 each sample: no return, confidence 0.50 / 0.49, range 2.0 / 1.99 and 16.0 / 16.01") {
    const loc::WallResetLimits l = lim0();
    loc::SideMount m = lens0();
    m.rangeBias = Length{0.25};
    const auto sv = [&](double r, double c) {
        return loc::sampleVerdict(loc::WallSample{Length{r}, c, false, Time{0.0}}, m, l);
    };
    CHECK(sv(7.0, 0.5) == WallResetVerdict::Seated);
    CHECK(sv(7.0, 0.49) == WallResetVerdict::LowConfidence);
    CHECK(sv(7.0, 0.0) == WallResetVerdict::NoReturn);
    CHECK(sv(393.66, 0.0) == WallResetVerdict::NoReturn);
    CHECK(sv(2.25, 1.0) == WallResetVerdict::Seated);  // 2.25 - 0.25 == 2.0 exactly
    CHECK(sv(2.24, 1.0) == WallResetVerdict::OutOfRange);
    CHECK(sv(16.25, 1.0) == WallResetVerdict::Seated);  // 16.0 exactly
    CHECK(sv(16.26, 1.0) == WallResetVerdict::OutOfRange);
    CHECK(loc::sampleVerdict(loc::WallSample{Length{7.0}, 1.0, true, Time{0.0}}, m, l)
          == WallResetVerdict::ReadFault);
    // In a window, ONE no-return sample refuses the window (M10: skipped, the rest would seat).
    std::vector<loc::WallSample> s = window(kFive);
    s[2].confidence = 0.0;
    s[2].distance = Length{393.66};
    const loc::WallResetReport rep = eval(s, at(7.0), req0(), l);
    CHECK(rep.verdict == WallResetVerdict::NoReturn);
    CHECK(rep.sampleIndex == 2);
}

// Bug caught: too few samples, or a window shorter than `window`, taken as enough; the spread gate
// removed (M11) or off its edge (0.5 exactly passes, 0.515625 refuses).
TEST_CASE("L6 T3 enough and consistent: 5 / 4 samples, the window's time, spread 0.5 / 0.515625") {
    loc::WallResetLimits l = lim0();
    CHECK(eval(window(kFive), at(7.0), req0(), l).verdict == WallResetVerdict::Seated);
    CHECK(eval(window({7.0, 7.0, 7.0, 7.0}), at(7.0), req0(), l).verdict == WallResetVerdict::TooFewSamples);
    l.minSamples = 4;  // four samples now enough by count — but they span 0.09375 s < 0.125 s
    CHECK(eval(window({7.0, 7.0, 7.0, 7.0}), at(7.0), req0(), l).verdict == WallResetVerdict::TooFewSamples);
    l = lim0();
    CHECK(eval(window({7.0, 7.5, 7.25, 7.0, 7.5}), at(7.25), req0(), l).verdict == WallResetVerdict::Seated);
    const loc::WallResetReport rep = eval(window({7.0, 7.515625, 7.25, 7.0, 7.5}), at(7.25), req0(), l);
    CHECK(rep.verdict == WallResetVerdict::Spread);
    CHECK(rep.spread.value() == 0.515625);
}

// Bug caught (M21): the MEAN used for the MEDIAN — one in-spread outlier moves a mean; the reported
// median and the seated value must be the window's middle reading.
TEST_CASE("L6 T3 the value used is the median, not the mean") {
    const loc::WallResetLimits l = lim0();
    const loc::WallResetReport rep = eval(window({7.0, 7.0, 7.0, 7.0, 7.5}), at(7.0), req0(), l);
    CHECK(rep.verdict == WallResetVerdict::Seated);
    CHECK(rep.median.value() == 7.0);
    CHECK(rep.measured.value() == doctest::Approx(7.0).epsilon(1e-12));
    const loc::WallResetReport even = eval(window({7.0, 7.0, 7.25, 7.5, 7.5, 7.5}), at(7.0), req0(), l);
    CHECK(even.median.value() == 7.375);
}

// Bug caught: a precision gate off its edge (the bound AT the limit passes, one ulp over refuses
// Imprecise), and an agreement gate off its edge or on the OTHER axis (M12 — the robot's y is
// -38, far from its x: compared on y every reset would refuse).
TEST_CASE("L6 T3 precise and agrees: the bound at its limit, the correction at its limit") {
    loc::WallResetLimits l = lim0();
    const Pose2d pose = at(7.0);
    const auto s = window({7.75, 7.75, 7.75, 7.75, 7.75});
    loc::WallResetReport rep = eval(s, pose, req0(3.0), l);
    REQUIRE(rep.verdict == WallResetVerdict::Seated);
    CHECK(rep.applied.value() == doctest::Approx(0.75).epsilon(1e-12));
    // The precision edge, exactly.
    l.errorBoundMax = rep.errorBound;
    CHECK(eval(s, pose, req0(3.0), l).verdict == WallResetVerdict::Seated);
    l.errorBoundMax = Length{std::nextafter(rep.errorBound.value(), 0.0)};
    CHECK(eval(s, pose, req0(3.0), l).verdict == WallResetVerdict::Imprecise);
    l = lim0();
    // The agreement edge, exactly: maxCorrection == |measured - before|.
    const double diff = std::abs(rep.measured.value() - rep.before.value());
    CHECK(eval(s, pose, req0(diff), l).verdict == WallResetVerdict::Seated);
    const loc::WallResetReport dis = eval(s, pose, req0(std::nextafter(diff, 0.0)), l);
    CHECK(dis.verdict == WallResetVerdict::Disagrees);
    CHECK(dis.applied.value() == 0.0);
}

// Bug caught (M13): an UNSET maxCorrection taken as infinite (or as anything) — 0 refuses BadRequest
// with the message naming the field and the fix; NaN and a negative are refused too.
TEST_CASE("L6 T3 maxCorrection UNSET (0) is refused by name; NaN and negative are refused") {
    const loc::WallResetLimits l = lim0();
    CHECK(loc::preCheck(at(7.0), lens0(), req0(0.0), l, Length{0.0}) == WallResetVerdict::BadRequest);
    CHECK(std::string{loc::wallResetProblem(lens0(), req0(0.0), l)}
          == "maxCorrection UNSET: say how far this wall may move x");
    loc::WallResetRequest ry = req0(0.0);
    ry.wall = loc::FieldWall::line("test wall", loc::Axis::Y, Length{0.0}, +1);
    CHECK(std::string{loc::wallResetProblem(lens0(), ry, l)}
          == "maxCorrection UNSET: say how far this wall may move y");
    CHECK(loc::preCheck(at(7.0), lens0(), req0(std::nan("")), l, Length{0.0}) == WallResetVerdict::BadRequest);
    CHECK(loc::preCheck(at(7.0), lens0(), req0(-1.0), l, Length{0.0}) == WallResetVerdict::BadRequest);
    loc::WallResetRequest bad = req0();
    bad.wall.inward = 0;
    CHECK(loc::preCheck(at(7.0), lens0(), bad, l, Length{0.0}) == WallResetVerdict::BadRequest);
    loc::WallResetLimits wide = l;
    wide.windowMax = Time{1.5};
    CHECK(loc::preCheck(at(7.0), lens0(), req0(), wide, Length{0.0}) == WallResetVerdict::BadRequest);
}

// Bug caught (T5, landmine 3): a held value taken as a reading. The count moves between distance()
// and confidence() of the LAST sample -> ReadFault (M15: the count read once per window; M16:
// compared only around distance()); it moved BEFORE the window and not during -> the window passes.
namespace {
/// A distance sensor whose read-fault count moves INSIDE the confidence() read of read number
/// `bumpAt` (0-based over confidence() calls): the second of a sample's two device reads failing.
class SplitReadSensor final : public shulib::hal::IDistance, public shulib::hal::IReadFaults {
public:
    explicit SplitReadSensor(int bumpAt) : bumpAt_{bumpAt} {}
    [[nodiscard]] Length distance() const override { return Length{7.0}; }
    [[nodiscard]] double confidence() const override {
        if (confReads_ == bumpAt_) {
            ++count_;
        }
        ++confReads_;
        return 1.0;
    }
    [[nodiscard]] int faultedReads() const noexcept override { return count_; }
    void bumpNow() { ++count_; }

private:
    int bumpAt_;
    mutable int confReads_ = 0;
    mutable int count_ = 0;
};
}  // namespace

TEST_CASE("L6 T5 the read-fault rule: a count that moves inside the last sample refuses ReadFault") {
    const loc::WallResetLimits l = lim0();
    const auto runWindow = [&](SplitReadSensor& sensor, const shulib::hal::IReadFaults* faults) {
        const loc::WallSensor s{&sensor, faults, lens0(), ""};
        loc::WallSampler sampler{s, l};
        double t = 0.0;
        while (!sampler.step(Time{t}, at(7.0), AngularVelocity{0.0})) {
            t += 0.03125;
        }
        return loc::evaluateWallReset(sampler.samples(), at(7.0), sampler.maxTwist(), lens0(), req0(), l,
                                      loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
    };
    SplitReadSensor last{4};  // the window is five samples (0 .. 0.125 s): the fifth's confidence()
    const loc::WallResetReport rf = runWindow(last, &last);
    CHECK(rf.verdict == WallResetVerdict::ReadFault);
    CHECK(rf.sampleIndex == 4);
    SplitReadSensor before{-1};
    before.bumpNow();  // the count moved BEFORE the window, never during it
    CHECK(runWindow(before, &before).verdict == WallResetVerdict::Seated);
    SplitReadSensor noChannel{4};
    const loc::WallResetReport na = runWindow(noChannel, nullptr);
    CHECK(na.verdict == WallResetVerdict::Seated);  // no count to read: accepted (the call says n/a)
}

// Bug caught (the L6-A1 review, S2): the read-fault count's FIRST read moved after distance() — only
// confidence() screened. A faulted port's adapter returns its HELD last distance on distance() and
// counts it, while the confidence() read after it can succeed: the count moves INSIDE distance() of
// the last sample, confidence() is healthy — that sample is not a reading and the window refuses.
namespace {
/// A distance sensor whose read-fault count moves INSIDE the distance() read number `bumpAt`
/// (0-based over distance() calls); confidence() always healthy.
class HeldDistanceSensor final : public shulib::hal::IDistance, public shulib::hal::IReadFaults {
public:
    explicit HeldDistanceSensor(int bumpAt) : bumpAt_{bumpAt} {}
    [[nodiscard]] Length distance() const override {
        if (distReads_ == bumpAt_) {
            ++count_;
        }
        ++distReads_;
        return Length{7.0};
    }
    [[nodiscard]] double confidence() const override { return 1.0; }
    [[nodiscard]] int faultedReads() const noexcept override { return count_; }

private:
    int bumpAt_;
    mutable int distReads_ = 0;
    mutable int count_ = 0;
};
}  // namespace

TEST_CASE("L6 T5 the read-fault rule: a count that moves inside distance() of the last sample refuses") {
    const loc::WallResetLimits l = lim0();
    HeldDistanceSensor held{4};  // the fifth sample's distance() (the window is 0 .. 0.125 s)
    const loc::WallSensor s{&held, &held, lens0(), ""};
    loc::WallSampler sampler{s, l};
    double t = 0.0;
    while (!sampler.step(Time{t}, at(7.0), AngularVelocity{0.0})) {
        t += 0.03125;
    }
    REQUIRE(sampler.samples().size() == 5U);
    CHECK(sampler.samples()[4].faultMoved);
    CHECK_FALSE(sampler.samples()[3].faultMoved);
    const loc::WallResetReport rf =
        loc::evaluateWallReset(sampler.samples(), at(7.0), sampler.maxTwist(), lens0(), req0(), l,
                               loc::headingTolerance(AngleDim{0.0}, l), Length{0.0});
    CHECK(rf.verdict == WallResetVerdict::ReadFault);
    CHECK(rf.sampleIndex == 4);
}

// Bug caught (the L6-A1 review): BadRequest's clauses unpinned — a reversed own-line span silently
// keeping nothing out (S23a), a typed 1-deg cone voiding the beam-height gate (S23b), a squareWithin
// past 20 deg (S23c), minSamples outside 1..64 (S23d), a non-finite mount field (S23e), a `window` the
// 64-sample buffer cannot span (C7). ONE bad field per row: preCheck says BadRequest, wallResetProblem
// names that field, and the edges land exactly (a value typed AT an edge is accepted).
TEST_CASE("L6 T3 BadRequest: every clause refuses its own field by name, at its exact edge") {
    struct Row {
        const char* what;
        void (*bend)(loc::SideMount&, loc::WallResetRequest&, loc::WallResetLimits&);
        const char* message;  // nullptr = accepted (an edge typed exactly)
    };
    static const std::array<loc::WallKeepOut, 1> reversed{
        loc::WallKeepOut{Length{-30.0}, Length{-40.0}, loc::KeepOutKind::Fixed, "reversed"}};
    static const std::array<loc::WallKeepOut, 1> nanSpan{
        loc::WallKeepOut{Length{std::numeric_limits<double>::quiet_NaN()}, Length{-40.0},
                         loc::KeepOutKind::Fixed, "nan"}};
    static const std::array<loc::WallKeepOut, 1> point{
        loc::WallKeepOut{Length{-40.0}, Length{-40.0}, loc::KeepOutKind::Fixed, "point"}};
    const char* const kMount = "the mount has a value that is not finite";
    const char* const kTol = "the mount's tolerances and accuracy must be finite and >= 0";
    const char* const kMountCone = "mount.coneHalfAngle must be 0 (unmeasured) or 3..30 degrees";
    const char* const kRange = "limits.rangeMin/rangeMax must satisfy 0 < rangeMin < rangeMax";
    const char* const kSquare = "limits.squareWithin must be above 0 and at most 20 degrees";
    const char* const kCone = "limits.coneHalfAngle must be 3..30 degrees";
    const char* const kSpread = "limits.spreadMax must be finite and above 0";
    const char* const kSamples = "limits.minSamples must be 1..64";
    const char* const kWindow = "limits.window/windowMax must satisfy 0 < window <= windowMax <= 1 s";
    const char* const kWindowBuf = "limits.window must be at most 0.6 s (the 64-sample buffer spans 0.63 s at 10 ms)";
    const char* const kConf = "limits.minConfidence must be 0..1";
    const char* const kStill = "limits.stillSpeed/stillTurn must be finite and above 0";
    const char* const kBound = "limits.errorBoundMax must be finite and above 0";
    const char* const kKeep = "limits.keepOutMargin/alongDriftRate must be finite and >= 0";
    const char* const kPiece = "limits.pieceTop/beamMargin must be finite and >= 0";
    const char* const kHead = "limits.headingFloor/headingPerTurn must be finite and >= 0";
    const Row rows[] = {
        {"maxCorrection inf", [](auto&, auto& r, auto&) { r.maxCorrection = Length{std::numeric_limits<double>::infinity()}; },
         "maxCorrection must be a finite distance above 0"},
        {"inward 2", [](auto&, auto& r, auto&) { r.wall.inward = 2; }, "wall.inward must be +1 or -1"},
        {"inward -1 (accepted)", [](auto&, auto& r, auto&) { r.wall.inward = -1; }, nullptr},
        {"coord NaN", [](auto&, auto& r, auto&) { r.wall.coord = Length{std::numeric_limits<double>::quiet_NaN()}; },
         "wall.coord is not finite"},
        {"keepOutCount 13", [](auto&, auto& r, auto&) { r.wall.keepOutCount = 13; },
         "wall.keepOutCount is past kMaxWallKeepOuts"},
        {"a REVERSED span (S23a)",
         [](auto&, auto& r, auto&) { r.wall = loc::FieldWall::line("w", loc::Axis::X, Length{0.0}, +1, reversed); },
         "a wall keep-out span is reversed or not finite"},
        {"a NaN span",
         [](auto&, auto& r, auto&) { r.wall = loc::FieldWall::line("w", loc::Axis::X, Length{0.0}, +1, nanSpan); },
         "a wall keep-out span is reversed or not finite"},
        {"a zero-length span (accepted)",
         [](auto&, auto& r, auto&) { r.wall = loc::FieldWall::line("w", loc::Axis::X, Length{0.0}, +1, point); },
         nullptr},
        {"forward NaN (S23e)", [](auto& m, auto&, auto&) { m.forward = Length{std::numeric_limits<double>::quiet_NaN()}; }, kMount},
        {"left inf", [](auto& m, auto&, auto&) { m.left = Length{std::numeric_limits<double>::infinity()}; }, kMount},
        {"height NaN", [](auto& m, auto&, auto&) { m.height = Length{std::numeric_limits<double>::quiet_NaN()}; }, kMount},
        {"facing NaN", [](auto& m, auto&, auto&) { m.facing = AngleDim{std::numeric_limits<double>::quiet_NaN()}; }, kMount},
        {"rangeBias NaN", [](auto& m, auto&, auto&) { m.rangeBias = Length{std::numeric_limits<double>::quiet_NaN()}; }, kMount},
        {"positionTolerance -0.1", [](auto& m, auto&, auto&) { m.positionTolerance = Length{-0.1}; }, kTol},
        {"facingTolerance NaN", [](auto& m, auto&, auto&) { m.facingTolerance = AngleDim{std::numeric_limits<double>::quiet_NaN()}; }, kTol},
        {"rangeAccuracy -0.1", [](auto& m, auto&, auto&) { m.rangeAccuracy = Length{-0.1}; }, kTol},
        {"tolerances 0 (accepted)",
         [](auto& m, auto&, auto&) {
             m.positionTolerance = Length{0.0};
             m.facingTolerance = AngleDim{0.0};
         },
         nullptr},
        {"mount cone 2 deg (S23b)", [](auto& m, auto&, auto&) { m.coneHalfAngle = AngleDim{2.0 * loc::kWallDegToRad}; }, kMountCone},
        {"mount cone just under 3 deg",
         [](auto& m, auto&, auto&) { m.coneHalfAngle = AngleDim{std::nextafter(3.0 * loc::kWallDegToRad, 0.0)}; }, kMountCone},
        {"mount cone 3 deg (accepted)", [](auto& m, auto&, auto&) { m.coneHalfAngle = AngleDim{3.0 * loc::kWallDegToRad}; }, nullptr},
        {"mount cone 30 deg (accepted)", [](auto& m, auto&, auto&) { m.coneHalfAngle = AngleDim{30.0 * loc::kWallDegToRad}; }, nullptr},
        {"mount cone 31 deg", [](auto& m, auto&, auto&) { m.coneHalfAngle = AngleDim{31.0 * loc::kWallDegToRad}; }, kMountCone},
        {"mount cone NaN", [](auto& m, auto&, auto&) { m.coneHalfAngle = AngleDim{std::numeric_limits<double>::quiet_NaN()}; }, kMountCone},
        {"rangeMin 0", [](auto&, auto&, auto& l) { l.rangeMin = Length{0.0}; }, kRange},
        {"rangeMax == rangeMin", [](auto&, auto&, auto& l) { l.rangeMax = l.rangeMin; }, kRange},
        {"rangeMax inf", [](auto&, auto&, auto& l) { l.rangeMax = Length{std::numeric_limits<double>::infinity()}; }, kRange},
        {"squareWithin 20 deg (accepted)", [](auto&, auto&, auto& l) { l.squareWithin = AngleDim{20.0 * loc::kWallDegToRad}; }, nullptr},
        {"squareWithin 21 deg (S23c)", [](auto&, auto&, auto& l) { l.squareWithin = AngleDim{21.0 * loc::kWallDegToRad}; }, kSquare},
        {"squareWithin just over 20 deg",
         [](auto&, auto&, auto& l) { l.squareWithin = AngleDim{std::nextafter(20.0 * loc::kWallDegToRad, 1.0)}; }, kSquare},
        {"squareWithin 0", [](auto&, auto&, auto& l) { l.squareWithin = AngleDim{0.0}; }, kSquare},
        {"limits cone 3 deg (accepted)", [](auto&, auto&, auto& l) { l.coneHalfAngle = AngleDim{3.0 * loc::kWallDegToRad}; }, nullptr},
        {"limits cone 30 deg (accepted)", [](auto&, auto&, auto& l) { l.coneHalfAngle = AngleDim{30.0 * loc::kWallDegToRad}; }, nullptr},
        {"limits cone just over 30 deg",
         [](auto&, auto&, auto& l) { l.coneHalfAngle = AngleDim{std::nextafter(30.0 * loc::kWallDegToRad, 1.0)}; }, kCone},
        {"limits cone 2 deg", [](auto&, auto&, auto& l) { l.coneHalfAngle = AngleDim{2.0 * loc::kWallDegToRad}; }, kCone},
        {"spreadMax 0", [](auto&, auto&, auto& l) { l.spreadMax = Length{0.0}; }, kSpread},
        {"spreadMax inf", [](auto&, auto&, auto& l) { l.spreadMax = Length{std::numeric_limits<double>::infinity()}; }, kSpread},
        {"minSamples 0 (S23d)", [](auto&, auto&, auto& l) { l.minSamples = 0; }, kSamples},
        {"minSamples 65", [](auto&, auto&, auto& l) { l.minSamples = 65; }, kSamples},
        {"minSamples 1 (accepted)", [](auto&, auto&, auto& l) { l.minSamples = 1; }, nullptr},
        {"minSamples 64 (accepted)", [](auto&, auto&, auto& l) { l.minSamples = 64; }, nullptr},
        {"window 0", [](auto&, auto&, auto& l) { l.window = Time{0.0}; }, kWindow},
        {"window > windowMax", [](auto&, auto&, auto& l) { l.window = Time{0.5}; l.windowMax = Time{0.25}; }, kWindow},
        {"windowMax 1 s (accepted)", [](auto&, auto&, auto& l) { l.windowMax = Time{1.0}; }, nullptr},
        {"windowMax just over 1 s", [](auto&, auto&, auto& l) { l.windowMax = Time{std::nextafter(1.0, 2.0)}; }, kWindow},
        {"window 0.6 s (accepted)", [](auto&, auto&, auto& l) { l.window = Time{0.6}; l.windowMax = Time{1.0}; }, nullptr},
        {"window 0.8 s: the buffer cannot span it (C7)",
         [](auto&, auto&, auto& l) { l.window = Time{0.8}; l.windowMax = Time{1.0}; }, kWindowBuf},
        {"minConfidence 1.01", [](auto&, auto&, auto& l) { l.minConfidence = 1.01; }, kConf},
        {"minConfidence NaN", [](auto&, auto&, auto& l) { l.minConfidence = std::numeric_limits<double>::quiet_NaN(); }, kConf},
        {"stillSpeed 0", [](auto&, auto&, auto& l) { l.stillSpeed = shulib::units::Velocity{0.0}; }, kStill},
        {"stillTurn inf", [](auto&, auto&, auto& l) { l.stillTurn = AngularVelocity{std::numeric_limits<double>::infinity()}; }, kStill},
        {"errorBoundMax 0", [](auto&, auto&, auto& l) { l.errorBoundMax = Length{0.0}; }, kBound},
        {"keepOutMargin -1", [](auto&, auto&, auto& l) { l.keepOutMargin = Length{-1.0}; }, kKeep},
        {"alongDriftRate NaN", [](auto&, auto&, auto& l) { l.alongDriftRate = std::numeric_limits<double>::quiet_NaN(); }, kKeep},
        {"pieceTop -1", [](auto&, auto&, auto& l) { l.pieceTop = Length{-1.0}; }, kPiece},
        {"beamMargin inf", [](auto&, auto&, auto& l) { l.beamMargin = Length{std::numeric_limits<double>::infinity()}; }, kPiece},
        {"headingFloor -0.01", [](auto&, auto&, auto& l) { l.headingFloor = AngleDim{-0.01}; }, kHead},
        {"headingPerTurn NaN", [](auto&, auto&, auto& l) { l.headingPerTurn = std::numeric_limits<double>::quiet_NaN(); }, kHead},
    };
    for (const Row& row : rows) {
        loc::SideMount m = lens0();
        loc::WallResetRequest rq = req0();
        loc::WallResetLimits l = lim0();
        row.bend(m, rq, l);
        INFO(row.what);
        const char* got = loc::wallResetProblem(m, rq, l);
        if (row.message == nullptr) {
            CHECK(got == nullptr);
        } else {
            REQUIRE(got != nullptr);
            CHECK(std::string{got} == row.message);
            CHECK(loc::preCheck(at(7.0), m, rq, l, Length{0.0}) == WallResetVerdict::BadRequest);
            // The evaluator refuses it too (it never judges an unusable request), naming the field.
            const loc::WallResetReport e = eval(window(kFive), at(7.0), rq, l, m);
            CHECK(e.verdict == WallResetVerdict::BadRequest);
            CHECK(std::string{e.detail} == row.message);
        }
    }
}

// Bug caught (the L6-A1 review): the worst-case bound DROPPING a NaN corner — a NaN tolerance gave a
// bound of 0, the best possible answer, and the precision gate passed. A non-finite tolerance now
// makes the bound +infinity (refused Imprecise), never 0.
TEST_CASE("L6 T2 a non-finite tolerance makes the error bound infinite, never 0") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const loc::SideMount m = lens0();
    CHECK(std::isfinite(loc::axisErrorBound(Length{7.0}, at(7.0), m, line0(), AngleDim{0.03}).value()));
    CHECK(std::isinf(loc::axisErrorBound(Length{7.0}, at(7.0), m, line0(), AngleDim{nan}).value()));
    loc::SideMount bad = m;
    bad.positionTolerance = Length{nan};
    CHECK(std::isinf(loc::axisErrorBound(Length{7.0}, at(7.0), bad, line0(), AngleDim{0.03}).value()));
    bad = m;
    bad.facingTolerance = AngleDim{nan};
    CHECK(std::isinf(loc::axisErrorBound(Length{7.0}, at(7.0), bad, line0(), AngleDim{0.03}).value()));
}

// Bug caught (RC34 with a bias, the L6-A1 review's S1/R5): the median's ceiling check judging the RAW
// median, not median - rangeBias. A sensor typed with a NEGATIVE bias (it reads 0.25 in short) puts
// the true range 0.25 in past the raw median: the edge sits at median = ceiling + bias exactly.
TEST_CASE("L6 T3 beam height with a bias: the median minus the bias is judged, at its exact edge") {
    const loc::WallResetLimits l = lim0();
    loc::SideMount m = lens0();
    m.rangeBias = Length{-0.25};
    const double ceiling = loc::beamCeiling(m, l).value();
    const double bias = m.rangeBias.value();
    const Pose2d pose = at(ceiling - 0.5);
    REQUIRE(loc::preCheck(pose, m, req0(), l, Length{0.0}) == WallResetVerdict::Seated);
    // The largest reading whose bias-corrected range is AT the ceiling (floating point decides).
    double r = ceiling + bias;
    while (!(r - bias <= ceiling)) {
        r = std::nextafter(r, 0.0);
    }
    while (std::nextafter(r, 100.0) - bias <= ceiling) {
        r = std::nextafter(r, 100.0);
    }
    const double past = std::nextafter(r, 100.0);
    REQUIRE(r < ceiling - 0.2);  // well under the ceiling RAW: only the bias puts it on the edge
    CHECK(eval(window({r, r, r, r, r}), pose, req0(), l, m).verdict == WallResetVerdict::Seated);
    const loc::WallResetReport low = eval(window({past, past, past, past, past}), pose, req0(), l, m);
    CHECK(low.verdict == WallResetVerdict::BeamLow);
}

// Bug caught (the L6-A1 review's S4/R4): a skewed beam's footprint is LOPSIDED, and the keep-out
// widening must use its LONGER side both ways. A span on the SHORT side, between the short and the
// long half-widths from the beam centre, must refuse (the smaller side used, or one fixed side, would
// pass it). Both skews: the long side is -along for one and +along for the other. The half-widths
// come from two edge rays cast by hand (independent of aimAt).
TEST_CASE("L6 T3 keep-out: a skewed beam's footprint applies its LONGER side both ways") {
    loc::WallResetLimits l = lim0();
    l.keepOutMargin = Length{0.0};
    const double cone = 18.0 * loc::kWallDegToRad;
    for (const double headingDeg : {94.9, 85.1}) {
        const double beam = headingDeg * kD2R + kPi / 2.0;  // the left lens: +90 deg from the nose
        // The lens at (7, -38); a ray at angle a meets x = 0 at y = -38 + (-7 / cos a) sin a.
        const auto hit = [](double a) { return -38.0 + (-7.0 / std::cos(a)) * std::sin(a); };
        const double centre = hit(beam);
        const double sideA = std::abs(hit(beam + cone) - centre);
        const double sideB = std::abs(hit(beam - cone) - centre);
        const double longSide = std::max(sideA, sideB);
        const double shortSide = std::min(sideA, sideB);
        REQUIRE(longSide - shortSide > 0.1);
        const loc::WallAim aim = loc::aimAt(at(7.0, headingDeg), lens0(), line0(), l, Length{0.0});
        CHECK(aim.along.value() == doctest::Approx(centre).epsilon(1e-12));
        CHECK(aim.beamHalfWidth.value() == doctest::Approx(longSide).epsilon(1e-12));
        // Which way along the wall is the SHORT side? (the sign of its edge ray's hit from the centre)
        const double shortHit = sideA < sideB ? hit(beam + cone) : hit(beam - cone);
        const double shortDir = shortHit > centre ? 1.0 : -1.0;
        const double gap = 0.5 * (shortSide + longSide);  // between the two half-widths
        const double nearEdge = centre + shortDir * gap;
        const std::array<loc::WallKeepOut, 1> span{loc::WallKeepOut{
            Length{shortDir > 0.0 ? nearEdge : nearEdge - 5.0}, Length{shortDir > 0.0 ? nearEdge + 5.0 : nearEdge},
            loc::KeepOutKind::Fixed, "span"}};
        loc::WallResetRequest rq = req0();
        rq.wall = loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1, span);
        INFO("heading " << headingDeg);
        CHECK(loc::preCheck(at(7.0, headingDeg), lens0(), rq, l, Length{0.0}) == WallResetVerdict::KeepOut);
        // Just past the LONG half-width on the short side: clear.
        const double farEdge = centre + shortDir * (longSide + 0.01);
        const std::array<loc::WallKeepOut, 1> clear{loc::WallKeepOut{
            Length{shortDir > 0.0 ? farEdge : farEdge - 5.0}, Length{shortDir > 0.0 ? farEdge + 5.0 : farEdge},
            loc::KeepOutKind::Fixed, "span"}};
        rq.wall = loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1, clear);
        CHECK(loc::preCheck(at(7.0, headingDeg), lens0(), rq, l, Length{0.0}) == WallResetVerdict::Seated);
    }
}

// Bug caught (the L6-A1 review's S24/S24b): the KeepOut line's "nearest clean band" wrong — the gaps
// BETWEEN spans never considered (a season wall then prints "clean none"), or an author's own line's
// open ends never considered. The band is where the beam CENTRE may land: each active span grown by
// the widening, merged, and the gap nearest the beam centre. Hand-built spans, so the answer is
// arithmetic: the widening w is the beam's half-width (margin and drift zeroed).
TEST_CASE("L6 T3 keep-out: the nearest clean band, between spans and past an own line's ends") {
    loc::WallResetLimits l = lim0();
    l.keepOutMargin = Length{0.0};
    const Pose2d pose = at(7.0, 90.0);  // the beam centre at along -38, square
    // Three spans; the beam centre (-38) inside the middle one. Grown by w (~2.27): the gap to the
    // right (-36 + w .. -25 - w, 4.27 in away) is nearer than the one to the left (-50 + w .. -44 - w,
    // 8.27 away), and nearer than either open end.
    const std::array<loc::WallKeepOut, 3> three{
        loc::WallKeepOut{Length{-55.0}, Length{-50.0}, loc::KeepOutKind::Fixed, "A"},
        loc::WallKeepOut{Length{-44.0}, Length{-36.0}, loc::KeepOutKind::Fixed, "B"},
        loc::WallKeepOut{Length{-25.0}, Length{-15.0}, loc::KeepOutKind::Fixed, "C"}};
    loc::WallResetRequest rq = req0();
    rq.wall = loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1, three);
    loc::WallAim aim = loc::aimAt(pose, lens0(), rq.wall, l, Length{0.0});
    const double w = aim.widening.value();
    REQUIRE(w == aim.beamHalfWidth.value());
    REQUIRE(aim.keepOutWhy != nullptr);
    CHECK(std::string{aim.keepOutWhy} == "B");
    CHECK(aim.cleanFound);
    CHECK(aim.cleanFrom.value() == -36.0 + w);
    CHECK(aim.cleanTo.value() == -25.0 - w);
    // A SEASON wall (frame Field) with the same spans: its ends are corners, never "open" — the same
    // gap; and with only one span there is no gap at all.
    loc::FieldWall season = rq.wall;
    season.frame = loc::WallFrame::Field;
    aim = loc::aimAt(pose, lens0(), season, l, Length{0.0});
    CHECK(aim.cleanFound);
    CHECK(aim.cleanFrom.value() == -36.0 + w);
    CHECK(aim.cleanTo.value() == -25.0 - w);
    const std::array<loc::WallKeepOut, 1> one{
        loc::WallKeepOut{Length{-44.0}, Length{-36.0}, loc::KeepOutKind::Fixed, "B"}};
    loc::FieldWall seasonOne = loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1, one);
    seasonOne.frame = loc::WallFrame::Field;
    aim = loc::aimAt(pose, lens0(), seasonOne, l, Length{0.0});
    CHECK_FALSE(aim.cleanFound);
    // An OWN line with that one span: its open ends count — the nearer is past the span's upper end.
    aim = loc::aimAt(pose, lens0(), loc::FieldWall::line("test wall", loc::Axis::X, Length{0.0}, +1, one), l,
                     Length{0.0});
    CHECK(aim.cleanFound);
    CHECK(aim.cleanFrom.value() == -36.0 + w);
    CHECK(aim.cleanTo.value() == std::numeric_limits<double>::infinity());
}

// Bug caught (the L6-A1 review's S28/S29): the tally counting a SEATED as refused, or the slot's
// side() handing back the LEFT sensor for both sides — public A1 API the everyday form (L6-A2) reads.
TEST_CASE("L6 T10 WallResetLog counts and keeps the last; WallSensors::side picks the named side") {
    loc::WallResetLog log{};
    loc::WallResetReport seated{};
    seated.verdict = WallResetVerdict::Seated;
    seated.measured = Length{-55.5};
    loc::WallResetReport refused{};
    refused.verdict = WallResetVerdict::Disagrees;
    refused.measured = Length{-60.25};
    log.record(seated);
    CHECK(log.seated == 1);
    CHECK(log.refused == 0);
    CHECK(log.last.measured.value() == -55.5);
    log.record(refused);
    log.record(refused);
    CHECK(log.seated == 1);
    CHECK(log.refused == 2);
    CHECK(log.last.verdict == WallResetVerdict::Disagrees);
    CHECK(log.last.measured.value() == -60.25);
    loc::WallSensors both{};
    both.left.absence = "left";
    both.right.absence = "right";
    CHECK(std::string{both.side(loc::WallSide::Left).absence} == "left");
    CHECK(std::string{both.side(loc::WallSide::Right).absence} == "right");
    CHECK(&both.side(loc::WallSide::Right) == &both.right);
}

// ── the call, on the tank rig ────────────────────────────────────────────────────────────────

namespace {
/// Counts every device read the reset makes (T4: a refused aim reads nothing).
class CountingDistance final : public shulib::hal::IDistance {
public:
    explicit CountingDistance(shulib::hal::IDistance& inner) : inner_{&inner} {}
    [[nodiscard]] Length distance() const override {
        ++reads;
        return inner_->distance();
    }
    [[nodiscard]] double confidence() const override {
        ++reads;
        return inner_->confidence();
    }
    mutable int reads = 0;

private:
    shulib::hal::IDistance* inner_;
};

/// The parking spot every call case uses: the left sensor 7 in from the left wall, at y = -38 (the
/// beam centre in the left wall's clean band right after a seed), nose along +y.
Pose2d park(double headingDeg = 90.0, double y = -38.0) {
    return Pose2d{Length{wall_rig::xForLeftReading(7.0)}, Length{y}, Angle::degrees(headingDeg)};
}

/// Everything a refused reset must leave as it was, bit for bit.
struct LocState {
    double x, y, th, bias, drift, odomX, odomY, travelX, travelY, turned;
};
LocState stateOf(wall_rig::WallRig& w) {
    const Pose2d p = w.loc().pose();
    return LocState{p.x().value(),
                    p.y().value(),
                    p.heading().radians(),
                    w.loc().headingBias().value(),
                    w.loc().distanceSinceCorrection().value(),
                    w.rig.odom.pose().x().value(),
                    w.rig.odom.pose().y().value(),
                    w.loc().travelSinceSeat(loc::Axis::X).value(),
                    w.loc().travelSinceSeat(loc::Axis::Y).value(),
                    w.loc().turnedSinceHeadingSeed().value()};
}
void requireSame(const LocState& a, const LocState& b) {
    CHECK(a.x == b.x);
    CHECK(a.y == b.y);
    CHECK(a.th == b.th);
    CHECK(a.bias == b.bias);
    CHECK(a.drift == b.drift);
    CHECK(a.odomX == b.odomX);
    CHECK(a.odomY == b.odomY);
    CHECK(a.travelX == b.travelX);
    CHECK(a.travelY == b.travelY);
    CHECK(a.turned == b.turned);
}
}  // namespace

// Bug caught (T4): a pre-check run AFTER the window (M14) — a refused aim then costs a window of
// time and reads the sensor — or the motion check dropped (M32): with a Tier-3 hold armed the reset
// would sample (the hold ticking), seat, and the hold would drive the robot to cancel the jump.
TEST_CASE("L6 T4 a refused aim costs no time and reads nothing; MotionActive cancels nothing") {
    struct Case {
        const char* name;
        Pose2d start;
        WallResetVerdict expect;
        bool unsetMount;
        bool noSensor;
    };
    const Case cases[] = {
        {"not square", park(100.0), WallResetVerdict::NotSquare, false, false},
        {"keep-out (the Toggle)", park(90.0, 0.0), WallResetVerdict::KeepOut, false, false},
        {"mount unset", park(), WallResetVerdict::MountUnset, true, false},
        {"no sensor", park(), WallResetVerdict::SensorAbsent, false, true},
    };
    for (const Case& c : cases) {
        wall_rig::WallRig w{c.start};
        w.idle(0.05);
        CountingDistance counting{w.leftPlant.sensor()};
        loc::WallSensor s = w.leftHandle();
        s.sensor = c.noSensor ? nullptr : &counting;
        s.absence = "port 4: no device";
        s.mount.measured = !c.unsetMount;
        loc::WallResetRequest rq{};
        rq.wall = ow::kLeftWall;
        rq.maxCorrection = Length{3.0};
        const double t0 = w.h().clock().now().value();
        const LocState before = stateOf(w);
        const loc::WallResetReport rep = shulib::chassis::resetFromWall(w.chassis, s, rq, w.limits);
        INFO(c.name);
        CHECK(rep.verdict == c.expect);
        CHECK(w.h().clock().now().value() == t0);
        CHECK(counting.reads == 0);
        requireSame(stateOf(w), before);
    }
    // MotionActive: a hold armed through the Tier-3 seam, the robot still.
    wall_rig::WallRig w{park()};
    w.idle(0.05);
    shulib::motion::HoldPose hold{w.chassis.deps(), 5.0};
    w.chassis.scheduler().async(hold);
    (void)w.chassis.scheduler().tick();
    REQUIRE(w.chassis.scheduler().hasActiveMotion());
    CountingDistance counting{w.leftPlant.sensor()};
    loc::WallSensor s = w.leftHandle();
    s.sensor = &counting;
    loc::WallResetRequest rq{};
    rq.wall = ow::kLeftWall;
    rq.maxCorrection = Length{3.0};
    const double t0 = w.h().clock().now().value();
    const LocState before = stateOf(w);
    const loc::WallResetReport rep = shulib::chassis::resetFromWall(w.chassis, s, rq, w.limits);
    CHECK(rep.verdict == WallResetVerdict::MotionActive);
    CHECK(w.h().clock().now().value() == t0);
    CHECK(counting.reads == 0);
    requireSame(stateOf(w), before);
    CHECK(w.chassis.scheduler().hasActiveMotion());  // the reset cancels nothing
    w.chassis.cancel();
}

// Bug caught (the L6-A1 review's R1/S26): the call's NOT LIVE check dropped — a reset on an estimate
// that is not live (the IMU still calibrating) would sample a window and then THROW from seatAxis's
// precondition mid-auton. It must refuse NotLive at once: no time, no read, one Warn line, nothing
// changed, no exception.
TEST_CASE("L6 T4 an estimate that is not live refuses NotLive: no time, no read, one line, no throw") {
    shulib::sim::ImuHostileConfig ic{};
    ic.calibrationEnd = Time{5.0};  // the IMU calibrates for 5 s: the estimate is not live before it
    shulib::sim::ImuHostileModel imu{ic};
    wall_rig::WallRig w{park(), shulib::sim::FieldModel::override2026(), wall_rig::trueLeft(),
                        wall_rig::trueRight(), &imu};
    w.idle(0.5);
    REQUIRE(w.loc().qualityClass() == loc::Localizer::Quality::Uninitialized);
    CountingDistance counting{w.leftPlant.sensor()};
    loc::WallSensor s = w.leftHandle();
    s.sensor = &counting;
    loc::WallResetRequest rq{};
    rq.wall = ow::kLeftWall;
    rq.maxCorrection = Length{3.0};
    const double t0 = w.h().clock().now().value();
    const LocState before = stateOf(w);
    const int lines = w.linesFrom("WALL");
    loc::WallResetReport rep{};
    CHECK_NOTHROW(rep = shulib::chassis::resetFromWall(w.chassis, s, rq, w.limits));
    CHECK(rep.verdict == WallResetVerdict::NotLive);
    CHECK(w.h().clock().now().value() == t0);
    CHECK(counting.reads == 0);
    requireSame(stateOf(w), before);
    CHECK(w.linesFrom("WALL") == lines + 1);
    CHECK(w.sink.last().level == shulib::hal::LogLevel::Warn);
    CHECK(w.sink.last().message == "left wall x REFUSED not live (LEFT sensor): the estimate is not live yet");
}

// Bug caught (the L6-A1 review's R2/W1 — M6c through the only production path): the call computing
// the precision gate's heading tolerance without the Localizer's turn counter. A robot that turned
// ~540 deg in place since its heading was seeded must be judged with 2 deg + 2 % of that (~12.8 deg)
// and refused IMPRECISE parked square; with its heading re-seeded (the counter back to 0) the same
// parking SEATS. The report's tolerance is the counter's, exactly.
TEST_CASE("L6 T4 the call's heading tolerance comes from the turn counter: turned 540 deg refuses imprecise") {
    wall_rig::WallRig w{park()};
    w.idle(0.3);
    for (const double h : {180.0, 270.0, 0.0, 90.0, 180.0, 90.0}) {
        (void)w.chassis.turnTo(Angle::degrees(h), {.timeout = Time{4.0}});
        (void)w.chassis.brake();
        w.idle(0.3);
    }
    const double turnedDeg = w.loc().turnedSinceHeadingSeed().value() / kD2R;
    CHECK(turnedDeg > 520.0);
    CHECK(turnedDeg < 580.0);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::Imprecise);
    CHECK(rep.headingTolerance.value()
          == loc::headingTolerance(w.loc().turnedSinceHeadingSeed(), w.limits).value());
    CHECK(rep.errorBound.value() > 1.0);
    // The heading re-seeded (as a FIELD start's would be): the counter is 0 and the same spot seats.
    w.loc().reseatHeading(w.loc().pose().heading());
    w.idle(0.05);
    CHECK(w.loc().turnedSinceHeadingSeed().value() == 0.0);
    const loc::WallResetReport fresh = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(fresh.verdict == WallResetVerdict::Seated);
    CHECK(fresh.headingTolerance.value() == w.limits.headingFloor.value());
    CHECK(fresh.errorBound.value() < 1.0);
}

// Bug caught (T9): a call that writes more than one line (M22: a line per sample) or a scheduler
// "waitUntil timed out" line (the predicate must END the wait), a line that is not ASCII or over 384
// bytes, the wrong level, a call that commands the drive, and a window that runs past windowMax (M23).
TEST_CASE("L6 T9 one call, one WALL line, no SCH line; the drive never commanded; the window bounded") {
    wall_rig::WallRig w{park()};
    w.idle(0.3);
    // Shift the estimate 1.5 in toward the field, as drift would (a reset should pull it back).
    w.loc().seatAxis(loc::Axis::X, w.loc().pose().x() + Length{1.5});
    w.idle(0.05);
    std::array<double, 2> volts{};
    for (int i = 0; i < 2; ++i) {
        volts[static_cast<std::size_t>(i)] = w.h().motor(i).commandedVoltage().value();
    }
    const int wallBefore = w.linesFrom("WALL");
    const int schBefore = w.linesFrom("SCH");
    const double xBefore = w.loc().pose().x().value();
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    REQUIRE(rep.verdict == WallResetVerdict::Seated);
    CHECK(w.linesFrom("WALL") == wallBefore + 1);
    CHECK(w.linesFrom("SCH") == schBefore);
    const auto& e = w.sink.last();
    CHECK(e.subsystem == "WALL");
    CHECK(e.level == shulib::hal::LogLevel::Info);
    CHECK(e.message.size() <= 384U);
    for (const char ch : e.message) {
        CHECK(static_cast<unsigned char>(ch) < 128U);
    }
    CHECK(e.message.rfind("left wall x SEATED (LEFT sensor): ", 0) == 0);
    CHECK(e.message.find("limit 3.00") != std::string::npos);
    CHECK(e.message.find("| faults ok") != std::string::npos);
    // The report carries the seat's numbers (the line test pins how each prints): `before` is the
    // estimate the window judged (the robot still: what it was before the call), `measured` is where
    // the estimate now is, `applied` the difference — and the line is exactly the report's line.
    CHECK(std::abs(rep.before.value() - xBefore) < 1e-9);
    CHECK(rep.measured.value() == w.loc().pose().x().value());
    CHECK(rep.applied.value() == rep.measured.value() - rep.before.value());
    CHECK(rep.applied.value() < -1.0);  // the estimate was 1.5 in toward the field: pulled back
    {
        loc::WallResetRequest rq{};
        rq.wall = ow::kLeftWall;
        rq.sensor = loc::WallSide::Left;
        rq.maxCorrection = Length{3.0};
        shulib::diag::lineformat::Line expected;
        loc::appendWallResetLine(expected, rep, rq, w.leftMount, w.limits);
        CHECK(e.message == std::string{expected.view()});
    }
    for (int i = 0; i < 2; ++i) {
        CHECK(w.h().motor(i).commandedVoltage().value() == volts[static_cast<std::size_t>(i)]);
    }
    CHECK(std::abs(w.loc().pose().x().value() - w.h().truePose().x().value()) < 0.1);

    // A REFUSED call: a Warn line, one only.
    w.loc().seatAxis(loc::Axis::X, w.loc().pose().x() + Length{2.0});
    const int before2 = w.linesFrom("WALL");
    const loc::WallResetReport dis2 = w.reset(ow::kLeftWall, loc::WallSide::Left, 1.0);
    CHECK(dis2.verdict == WallResetVerdict::Disagrees);
    CHECK(w.linesFrom("WALL") == before2 + 1);
    CHECK(w.sink.last().level == shulib::hal::LogLevel::Warn);
    CHECK(w.sink.last().message.rfind("left wall x REFUSED disagrees (LEFT sensor): wall says ", 0) == 0);
}

TEST_CASE("L6 T9 the window ends at windowMax by the clock, never later (M23)") {
    wall_rig::WallRig w{park()};
    w.idle(0.3);
    w.limits.minSamples = 64;  // 0.5 s at 10 ms holds 51 samples: only windowMax can end it
    const double t0 = w.h().clock().now().value();
    const int sch = w.linesFrom("SCH");
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    const double took = w.h().clock().now().value() - t0;
    CHECK(rep.verdict == WallResetVerdict::TooFewSamples);
    CHECK(took <= w.limits.windowMax.value() + 0.01 + 1e-9);
    CHECK(took >= w.limits.windowMax.value() - 1e-9);
    CHECK(w.linesFrom("SCH") == sch);
    // Every sample a no-return (the plant's sensor frozen at 9999): refused at once.
    w.leftPlant.config().dropoutProbability = 1.0;
    w.idle(0.05);
    const double t1 = w.h().clock().now().value();
    const loc::WallResetReport nr = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(nr.verdict == WallResetVerdict::NoReturn);
    CHECK(w.h().clock().now().value() - t1 <= 0.01 + 1e-9);
}

// Bug caught (T5's third case, landmine 7): no read-fault channel read as "faults ok". A sensor with
// no count is accepted and its line says "faults n/a".
TEST_CASE("L6 T5 a sensor with no read-fault count is accepted and says faults n/a") {
    wall_rig::WallRig w{park()};
    w.idle(0.3);
    loc::WallSensor s = w.leftHandle();
    s.faults = nullptr;
    loc::WallResetRequest rq{};
    rq.wall = ow::kLeftWall;
    rq.maxCorrection = Length{3.0};
    const loc::WallResetReport rep = shulib::chassis::resetFromWall(w.chassis, s, rq, w.limits);
    CHECK(rep.verdict == WallResetVerdict::Seated);
    CHECK_FALSE(rep.faultChannel);
    CHECK(w.sink.last().message.find("| faults n/a") != std::string::npos);
    CHECK(w.sink.last().message.find("faults ok") == std::string::npos);
    // A plant read fault mid-run with the channel present: refused ReadFault, nothing seated.
    w.leftPlant.fault(true);
    w.idle(0.05);
    const LocState before = stateOf(w);
    const loc::WallResetReport rf = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rf.verdict == WallResetVerdict::ReadFault);
    CHECK(stateOf(w).x == before.x);
    w.leftPlant.fault(false);
}

// Bug caught (T6's second half): a REFUSED reset that changes the estimate — pose, bias, drift term,
// the odometry's pose or a counter. A disagreement refusal after a full window.
TEST_CASE("L6 T6 a refused reset leaves the Localizer bit-identical") {
    wall_rig::WallRig w{park()};
    w.idle(0.3);
    // The estimate 4 in TOWARD the wall (an expected range of ~3 in: under the beam ceiling, so the
    // window runs and the refusal is the agreement gate's, after every sample).
    w.loc().seatAxis(loc::Axis::X, w.loc().pose().x() - Length{4.0});
    w.idle(0.05);
    const LocState before = stateOf(w);
    const loc::WallResetReport rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0);
    CHECK(rep.verdict == WallResetVerdict::Disagrees);
    CHECK(rep.applied.value() == 0.0);
    CHECK(rep.samples >= 5);
    requireSame(stateOf(w), before);
}

// Bug caught (T10, M24): continues() ignoring `required` — a required refusal must stop the chain
// ActionFailed (the Routine's policy safes the drive and skips the rest); a non-required refusal and a
// seat continue it.
TEST_CASE("L6 T10 then() and D3: seated continues, refused continues unless required") {
    for (const bool required : {false, true}) {
        for (const bool refuse : {false, true}) {
            wall_rig::WallRig w{park()};
            w.idle(0.3);
            if (refuse) {
                w.loc().seatAxis(loc::Axis::X, w.loc().pose().x() - Length{4.0});
                w.idle(0.05);
            }
            shulib::chassis::Routine r{w.chassis, "wall test"};
            loc::WallResetReport rep{};
            int nextRan = 0;
            r.then(
                [&] {
                    rep = w.reset(ow::kLeftWall, loc::WallSide::Left, 3.0, required);
                    return rep.continues();
                },
                "left wall x");
            r.then(
                [&] {
                    ++nextRan;
                    return true;
                },
                "next");
            INFO("required " << required << " refuse " << refuse);
            CHECK(rep.seated() == !refuse);
            CHECK(rep.required == required);
            const shulib::chassis::RoutineResult res = r.result();
            if (refuse && required) {
                CHECK_FALSE(res.ok);
                CHECK(res.cause == shulib::chassis::RoutineStopCause::ActionFailed);
                CHECK(std::string{res.stoppedName} == "left wall x");
                CHECK(res.skipped == 1);
                CHECK(nextRan == 0);
                for (int i = 0; i < 2; ++i) {
                    CHECK(w.h().motor(i).commandedVoltage().value() == 0.0);
                }
            } else {
                CHECK(res.ok);
                CHECK(nextRan == 1);
            }
        }
    }
}
