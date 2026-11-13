// The wall reset's EVERYDAY call over an auton's context — competition::resetFromWall(ctx, request) —
// on the host test plant (chunk L6-A2; the L6 brief's §1.12, D3, D4 (a), RC47, RC48 and test T17).
//
// The rig is a tank stack (wall_reset_rig.hpp's pieces) whose CONTEXT carries the wall-sensor slot by
// hand, so each side's sensor can be a COUNTING wrapper over the field plant's: "nothing was read" is
// then a number, not a hope. The Chassis's pacer is a RunGuard, and every call is made inside a real
// guarded run (the guard's run(), as competition::runAuton makes it), with an AutonContext over the
// run — its entry the start mode under test, its telemetry a sink of its own (the line must go THERE,
// not to the chassis's). The robot is truly parked beside the left wall, nose along +y, square, its
// left sensor 7 in from the wall; the estimate is put 2.5 in off on x, so a seat that happens is a
// 2.5-in jump nobody can miss.
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * A WALL RESET NEEDS A FIELD START: a RELATIVE start and an UNSET one (a routine entry that stated
//     none, and a route whose Start states no heading) are refused `not a field start` — the line naming
//     the mode — with the clock unmoved and NOTHING read, the estimate untouched (mutation M40: the
//     start check dropped — the RELATIVE auton SEATS a field number into its own frame). The start is
//     judged before the slot.
//   * THE GUARD'S WAIT: a deadline landing mid-window CUTS the reset at once — `cut`, zero lateness,
//     nothing seated (M41: sampling through the frozen chassis.waitUntil — the window runs on past the
//     deadline and SEATS). The CUT line's two forms split between NO sample ("came before the window
//     began") and ONE ("landed in the window"): a case sits on each side of that edge exactly (V11).
//   * NO SLOT → `no sensor`, reading nothing; a refused side → `sensor absent` with the Robot's
//     sentence; every outcome filed in the slot's log (counts and the last report); ONE line, to the
//     auton's own telemetry; `required` turns a refusal into a stopped chain (D3); a call outside a
//     live run is a PRECONDITION (a program error), before anything is read.
//   * AN EARLY REFUSAL IS THE REQUEST'S: the red wall (y) with the RIGHT sensor, so the report's side,
//     axis, `before`, fault channel and limit echo — and the line's Warn level — each have one right
//     answer that is not a default.
//   * THE SLOT'S LIMITS judge the reset (a typed 6.5-in ceiling refuses what the defaults seat).
//   * THROUGH THE REAL ROBOT AND runAuton: a robot file whose mount is UNMEASURED refuses `mount unset`
//     (the Robot keeps the file's word); a hold armed through the auton's Chassis refuses `motion
//     active` and is not cancelled; the robot file's typed limits are the ones judged.

#include "doctest.h"

#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/auton_wall_reset.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/distance.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/axis.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/hold_pose.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/field_plant.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"
#include "wall_reset_rig.hpp"
#include "wall_robot_rig.hpp"

namespace hal = shulib::hal;
namespace loc = shulib::localization;
namespace comp = shulib::competition;
using shulib::chassis::Routine;
using shulib::chassis::RoutineStopCause;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;
using namespace shulib::units::literals;

namespace {

constexpr double kTick = 0.01;

/// A distance sensor that counts every device read, over the field plant's.
struct CountingDistance final : hal::IDistance {
    explicit CountingDistance(hal::IDistance& in) : inner{&in} {}
    [[nodiscard]] Length distance() const override {
        ++reads;
        return inner->distance();
    }
    [[nodiscard]] double confidence() const override {
        ++reads;
        return inner->confidence();
    }
    hal::IDistance* inner;
    mutable int reads = 0;
};

/// A read-fault count that counts every time it is read.
struct CountingFaults final : hal::IReadFaults {
    explicit CountingFaults(const hal::IReadFaults& in) : inner{&in} {}
    [[nodiscard]] int faultedReads() const noexcept override {
        ++reads;
        return inner->faultedReads();
    }
    const hal::IReadFaults* inner;
    mutable int reads = 0;
};

/// wall_reset_rig.hpp's pacer (the drive plant, then both side sensors at the TRUE pose), plus a
/// DISABLE at a chosen pace: `requestAbandon()` on the guard — what the competition program does when
/// the field disables the robot mid-auton.
struct CtxPacer final : shulib::motion::ITickPacer {
    CtxPacer(shulib::sim::SimHarness& h, shulib::sim::SideSensorPlant& l, shulib::sim::SideSensorPlant& r)
        : inner{h, l, r} {}
    void pace() override {
        inner.pace();
        if (guard != nullptr && inner.paces == abandonAt) {
            guard->requestAbandon();
        }
    }
    wall_rig::WallPacer inner;
    shulib::sequence::RunGuard* guard = nullptr;
    int abandonAt = -1;  ///< the pace at which the guard is told to abandon (-1 = never)
};

/// Where the robot truly is: parked beside the left wall, nose along +y, its centred left sensor 7 in
/// from the wall (x = -70.205 + 7.5 + 7.0), at y = -38 — inside the left wall's clean band.
const Pose2d kParked{Length{wall_rig::xForLeftReading(7.0)}, Length{-38.0}, Angle::degrees(90.0)};

/// The context rig (file banner). `withSlot` false builds the context WITHOUT the wall-sensor slot.
struct CtxRig {
    shulib::kinematics::TankKinematics kin{Length{11.0}};
    shulib::hal::fake::FakeTelemetrySink chassisSink;  ///< the chassis context's telemetry
    shulib::hal::fake::FakeTelemetrySink autonSink;    ///< AutonContext::telemetry — the line goes here
    shulib::sim::FieldModel field{shulib::sim::FieldModel::override2026()};
    shulib::sim::SideSensorPlant leftPlant{field, wall_rig::trueLeft()};
    shulib::sim::SideSensorPlant rightPlant{field, wall_rig::trueRight()};
    motion_rig::MotionRig rig;
    CtxPacer pacer;
    shulib::sequence::RunGuard guard{pacer};
    CountingDistance leftCount{leftPlant.sensor()};
    CountingDistance rightCount{rightPlant.sensor()};
    CountingFaults leftFaults{leftPlant.faults()};
    CountingFaults rightFaults{rightPlant.faults()};
    loc::WallSensors walls{};
    shulib::chassis::RobotContext ctx;
    shulib::motion::MotionDeps deps;
    shulib::chassis::Chassis chassis;
    comp::AutonEntry entry{};

    explicit CtxRig(bool withSlot = true, const Pose2d& truth = kParked)
        : rig{kin, wall_rig::WallRig::configAt(truth), &chassisSink},
          pacer{rig.h, leftPlant, rightPlant},
          ctx{contextConfig(withSlot)},
          deps{.ctx = &ctx,
               .localizer = &rig.loc,
               .kinematics = &kin,
               .faults = &rig.latch,
               .health = &rig.health},
          chassis{deps, guard, motion_rig::chassisConfig()} {
        pacer.guard = &guard;
        walls.left = loc::WallSensor{&leftCount, &leftFaults, wall_rig::believedFrom(wall_rig::trueLeft()), ""};
        walls.right =
            loc::WallSensor{&rightCount, &rightFaults, wall_rig::believedFrom(wall_rig::trueRight()), ""};
        leftPlant.step(rig.h.truePose());
        rightPlant.step(rig.h.truePose());
        entry.name = "wall test";
        entry.kind = comp::AutonKind::Match;
        entry.endActionAt = Time{20.0};
        entry.hardStopAt = Time{25.0};
        entry.start = comp::StartPose::field(kParked);
        // Let the estimate go live and the sensors fill, then put x 2.5 in further from the wall
        // than the robot is: a seat would be a 2.5-in jump.
        (void)chassis.waitUntil([] { return false; }, Time{0.3});
        rig.loc.seatAxis(loc::Axis::X, rig.loc.pose().x() + Length{2.5});
        (void)chassis.waitUntil([] { return false; }, Time{0.05});
    }

    shulib::chassis::RobotContextConfig contextConfig(bool withSlot) {
        return shulib::chassis::RobotContextConfig{.clock = &rig.h.context().clock(),
                                                   .driveMotors = rig.h.context().driveMotors(),
                                                   .imu = &rig.h.context().imu(),
                                                   .gps = &rig.h.context().gps(),
                                                   .battery = &rig.h.context().battery(),
                                                   .telemetry = &chassisSink,
                                                   .tags = &rig.h.context().tags(),
                                                   .vision = &rig.h.context().vision(),
                                                   .walls = withSlot ? &walls : nullptr};
    }

    /// Run `scoring(AutonContext&)` inside ONE guarded run of this rig's entry (the guard's run(), as
    /// competition::runAuton makes it — without its start seat, so the estimate stays where the rig
    /// put it).
    template <typename Scoring>
    shulib::sequence::RunGuardReport run(Scoring&& scoring) {
        const shulib::sequence::RunGuardConfig cfg{.endActionAt = entry.endActionAt,
                                                   .hardStopAt = entry.hardStopAt};
        return guard.run(chassis, cfg,
                         [this, &scoring] {
                             comp::AutonContext actx{chassis, guard, autonSink, {}, entry};
                             scoring(actx);
                         },
                         [] {});
    }

    /// The left-wall request (D4's), `maxCorrection` 3 in.
    static loc::WallResetRequest leftWall(bool required = false) {
        loc::WallResetRequest req{};
        req.wall = loc::override2026::kLeftWall;
        req.sensor = loc::WallSide::Left;
        req.maxCorrection = Length{3.0};
        req.required = required;
        return req;
    }

    [[nodiscard]] int reads() const {
        return leftCount.reads + rightCount.reads + leftFaults.reads + rightFaults.reads;
    }
    [[nodiscard]] double now() { return rig.h.clock().now().value(); }

    static int wallLines(const shulib::hal::fake::FakeTelemetrySink& sink) {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == "WALL") {
                ++n;
            }
        }
        return n;
    }
    static std::string lastWallLine(const shulib::hal::fake::FakeTelemetrySink& sink) {
        for (int i = sink.size() - 1; i >= 0; --i) {
            if (sink.at(i).subsystem == "WALL") {
                return sink.at(i).message;
            }
        }
        return "";
    }
};

/// The pose's three numbers, compared bit for bit.
bool samePose(const Pose2d& a, const Pose2d& b) {
    return a.x().value() == b.x().value() && a.y().value() == b.y().value()
           && a.heading().radians() == b.heading().radians();
}

/// The WHOLE Localizer a refusal must leave bit-identical (the brief's T6 list — pose, heading bias,
/// drift term, the odometry's own pose — and the seat's bookkeeping: both axes' travel counters and the
/// turn since the heading seed). The robot is parked, so a window's ticks change none of it.
struct LocState {
    double x, y, th, bias, drift, odomX, odomY, travelX, travelY, turned;
};
LocState stateOf(CtxRig& r) {
    const Pose2d p = r.rig.loc.pose();
    return LocState{p.x().value(),
                    p.y().value(),
                    p.heading().radians(),
                    r.rig.loc.headingBias().value(),
                    r.rig.loc.distanceSinceCorrection().value(),
                    r.rig.odom.pose().x().value(),
                    r.rig.odom.pose().y().value(),
                    r.rig.loc.travelSinceSeat(loc::Axis::X).value(),
                    r.rig.loc.travelSinceSeat(loc::Axis::Y).value(),
                    r.rig.loc.turnedSinceHeadingSeed().value()};
}
void checkSame(const LocState& a, const LocState& b) {
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

// ═══ The start (M40) ═══════════════════════════════════════════════════════════════

// Bug caught (M40 — the start check dropped): a RELATIVE-start auton whose estimate's numbers happen to
// look like a field pose beside the left wall — its origin is wherever the robot started, so "the left
// wall at x = -70.2" names nothing in its frame — would SEAT a field number into a robot frame (here a
// 2.5-in jump), silently, inside maxCorrection. The refusal must read nothing and cost no time.
TEST_CASE("L6 T17 a RELATIVE start is refused 'not a field start' -- the line naming it -- with the clock "
          "unmoved, nothing read, the estimate untouched, the refusal filed") {
    CtxRig r;
    r.entry.start = comp::StartPose::relative();
    loc::WallResetReport rep{};
    Pose2d before{};
    Pose2d after{};
    double t0 = 0.0;
    double t1 = 0.0;
    int readsBefore = 0;
    (void)r.run([&](comp::AutonContext& ctx) {
        before = r.rig.loc.pose();
        t0 = r.now();
        readsBefore = r.reads();
        rep = comp::resetFromWall(ctx, CtxRig::leftWall());
        t1 = r.now();
        after = r.rig.loc.pose();
    });
    CHECK(rep.verdict == loc::WallResetVerdict::NotFieldStart);
    CHECK(std::string{rep.detail} == "RELATIVE");
    CHECK_FALSE(rep.seated());
    CHECK(rep.continues());  // not required: the auton goes on (D3)
    CHECK(t1 == t0);         // no time
    CHECK(r.reads() == readsBefore);
    CHECK(r.reads() == 0);   // nothing read, ever
    CHECK(rep.faultChannel);  // the named side HAS a read-fault channel — said as the sampler would
    CHECK(samePose(before, after));
    CHECK(rep.before.value() == before.x().value());
    // Filed in the slot's log, and ONE line — to the auton's telemetry, not the chassis's.
    CHECK(r.walls.log.refused == 1);
    CHECK(r.walls.log.seated == 0);
    CHECK(r.walls.log.last.verdict == loc::WallResetVerdict::NotFieldStart);
    CHECK(CtxRig::wallLines(r.autonSink) == 1);
    CHECK(CtxRig::wallLines(r.chassisSink) == 0);
    CHECK(CtxRig::lastWallLine(r.autonSink)
          == "left wall x REFUSED not a field start (LEFT sensor): start RELATIVE -- the season's walls are "
             "FIELD coordinates: declare a FIELD start");
}

// Bug caught: an UNSET start read as "not RELATIVE, so fine" — a routine entry that stated no start, or a
// route whose Start states no heading (effectiveStart() is UNSET for it) — and the line unable to tell
// the student which of the two it was.
TEST_CASE("L6 T17 an UNSET start -- a routine entry with none, a route whose Start states no heading -- is "
          "refused 'not a field start: start UNSET', reading nothing") {
    SUBCASE("a routine entry that stated no start") {
        CtxRig r;
        r.entry.start = comp::StartPose{};
        REQUIRE(r.entry.effectiveStart().mode() == comp::StartPose::Mode::Unset);
        loc::WallResetReport rep{};
        (void)r.run([&](comp::AutonContext& ctx) { rep = comp::resetFromWall(ctx, CtxRig::leftWall()); });
        CHECK(rep.verdict == loc::WallResetVerdict::NotFieldStart);
        CHECK(std::string{rep.detail} == "UNSET");
        CHECK(r.reads() == 0);
        CHECK(CtxRig::lastWallLine(r.autonSink).find("start UNSET -- ") != std::string::npos);
    }
    SUBCASE("a route whose Start states no heading") {
        CtxRig r;
        shulib::config::Route route{};
        route.name[0] = 'r';
        route.count = 1;
        route.waypoints[0].motion = shulib::config::WaypointMotion::Start;
        route.waypoints[0].headingMode = shulib::config::HeadingMode::Free;
        route.waypoints[0].x = kParked.x();
        route.waypoints[0].y = kParked.y();
        r.entry.start = comp::StartPose{};
        r.entry.route = &route;
        REQUIRE(r.entry.effectiveStart().mode() == comp::StartPose::Mode::Unset);
        loc::WallResetReport rep{};
        (void)r.run([&](comp::AutonContext& ctx) { rep = comp::resetFromWall(ctx, CtxRig::leftWall()); });
        CHECK(rep.verdict == loc::WallResetVerdict::NotFieldStart);
        CHECK(std::string{rep.detail} == "UNSET");
        CHECK(r.reads() == 0);
        // The same route with its Start's heading STATED is a FIELD start, and the reset goes on to read.
        route.waypoints[0].headingMode = shulib::config::HeadingMode::Fixed;
        route.waypoints[0].heading = Angle::degrees(90.0);
        REQUIRE(r.entry.effectiveStart().mode() == comp::StartPose::Mode::Field);
        (void)r.run([&](comp::AutonContext& ctx) { rep = comp::resetFromWall(ctx, CtxRig::leftWall()); });
        CHECK(rep.verdict != loc::WallResetVerdict::NotFieldStart);
        CHECK(r.reads() > 0);
    }
}

// Bug caught: the slot looked up first, so a RELATIVE auton on a robot without wall sensors is told
// "add the walls block" — and, once it does, is told the start is wrong. The start is the first word.
TEST_CASE("L6 T17 the start is judged BEFORE the slot: a RELATIVE start on a robot with no wall sensors reads "
          "'not a field start'") {
    CtxRig r{/*withSlot=*/false};
    r.entry.start = comp::StartPose::relative();
    loc::WallResetReport rep{};
    (void)r.run([&](comp::AutonContext& ctx) { rep = comp::resetFromWall(ctx, CtxRig::leftWall()); });
    CHECK(rep.verdict == loc::WallResetVerdict::NotFieldStart);
    CHECK(r.reads() == 0);
}

// ═══ A FIELD start: it seats ═══════════════════════════════════════════════════════

// Bug caught: an everyday call that refuses everything (the M40 test above would then pass for the
// wrong reason) — or seats without filing it, or files it in the chassis's sink instead of the auton's.
TEST_CASE("L6 T17 a FIELD start SEATS through the guard's wait: x back within 0.1 in of the truth, the seat "
          "filed, one Info line to the auton's telemetry") {
    CtxRig r;
    loc::WallResetReport rep{};
    double before = 0.0;
    (void)r.run([&](comp::AutonContext& ctx) {
        before = r.rig.loc.pose().x().value();
        rep = comp::resetFromWall(ctx, CtxRig::leftWall());
    });
    CAPTURE(loc::wallResetVerdictWord(rep.verdict));
    REQUIRE(rep.seated());
    CHECK(std::abs(r.rig.loc.pose().x().value() - r.rig.h.truePose().x().value()) < 0.1);
    CHECK(rep.applied.value() == doctest::Approx(-2.5).epsilon(0.04));
    CHECK(rep.before.value() == before);
    CHECK(r.reads() > 0);
    CHECK(r.walls.log.seated == 1);
    CHECK(r.walls.log.refused == 0);
    CHECK(r.walls.log.last.measured.value() == rep.measured.value());
    CHECK(CtxRig::wallLines(r.autonSink) == 1);
    CHECK(CtxRig::wallLines(r.chassisSink) == 0);
    CHECK(r.autonSink.at(r.autonSink.size() - 1).level == hal::LogLevel::Info);
    CHECK(CtxRig::lastWallLine(r.autonSink).rfind("left wall x SEATED (LEFT sensor): ", 0) == 0);

    // A second call in the same run is refused (maxCorrection UNSET): the log counts both, keeps the last.
    loc::WallResetReport second{};
    (void)r.run([&](comp::AutonContext& ctx) {
        loc::WallResetRequest unset = CtxRig::leftWall();
        unset.maxCorrection = Length{0.0};
        second = comp::resetFromWall(ctx, unset);
    });
    CHECK(second.verdict == loc::WallResetVerdict::BadRequest);
    CHECK(r.walls.log.seated == 1);
    CHECK(r.walls.log.refused == 1);
    CHECK(r.walls.log.last.verdict == loc::WallResetVerdict::BadRequest);
    CHECK(CtxRig::wallLines(r.autonSink) == 2);
}

// ═══ The guard's wait (M41) ════════════════════════════════════════════════════════

// Bug caught (M41 — the window sampled through the FROZEN chassis.waitUntil): a deadline landing in
// the window is acted on only when the window ends — up to windowMax late — and the reset SEATS after
// the buzzer. Through the guard's wait it is CUT on the tick the deadline passes, and nothing changes.
TEST_CASE("L6 T17 a deadline landing mid-window CUTS the reset at once -- zero lateness, nothing seated, the cut "
          "filed and said") {
    CtxRig r;
    r.entry.endActionAt = Time{0.5};
    r.entry.hardStopAt = Time{3.0};
    loc::WallResetReport rep{};
    double armedAt = 0.0;
    double calledAt = 0.0;
    double returnedAt = 0.0;
    LocState before{};
    LocState after{};
    const shulib::sequence::RunGuardReport report = r.run([&](comp::AutonContext& ctx) {
        armedAt = r.now();
        // Up to 0.05 s before the deadline, through the guard's own wait — then the reset, whose window
        // needs at least 0.15 s: the deadline lands inside it.
        (void)ctx.guard.waitFor([&] { return r.now() >= armedAt + 0.45 - 1e-9; }, Time{5.0});
        calledAt = r.now();
        before = stateOf(r);
        rep = comp::resetFromWall(ctx, CtxRig::leftWall());
        returnedAt = r.now();
        after = stateOf(r);
    });
    const double deadline = armedAt + 0.5;
    CAPTURE(loc::wallResetVerdictWord(rep.verdict));
    REQUIRE(calledAt < deadline);
    CHECK(rep.verdict == loc::WallResetVerdict::Cut);
    CHECK(rep.samples > 0);  // the window HAD started: the deadline landed inside it
    CHECK(returnedAt >= deadline - 1e-9);
    CHECK(returnedAt <= deadline + kTick + 1e-9);  // the tick it passed — zero lateness
    CHECK(report.scoringCut);
    // Nothing changed: the Localizer BIT-IDENTICAL across the cut window (a seat would have moved x
    // 2.5 in and re-based the X travel counter; any axis, the heading or the drift term touched shows).
    checkSame(after, before);
    CHECK(r.walls.log.refused == 1);
    CHECK(r.walls.log.last.verdict == loc::WallResetVerdict::Cut);
    CHECK(CtxRig::lastWallLine(r.autonSink)
          == "left wall x REFUSED cut (LEFT sensor): the run's deadline or a disable landed in the window; "
             "nothing changed");
}

// Bug caught: a disable (the field's, or the drive team's) that lands mid-window read as nothing —
// the window runs on and SEATS after the robot was told to stop. The guard's wait sees the abandon
// on the tick it lands, like a deadline.
TEST_CASE("L6 T17 a DISABLE landing mid-window cuts the reset too -- nothing seated, the cut filed") {
    CtxRig r;
    loc::WallResetReport rep{};
    LocState before{};
    LocState after{};
    double calledAt = 0.0;
    double returnedAt = 0.0;
    const shulib::sequence::RunGuardReport report = r.run([&](comp::AutonContext& ctx) {
        r.pacer.abandonAt = r.pacer.inner.paces + 5;  // five ticks into the window (it needs >= 15)
        calledAt = r.now();
        before = stateOf(r);
        rep = comp::resetFromWall(ctx, CtxRig::leftWall());
        returnedAt = r.now();
        after = stateOf(r);
    });
    CAPTURE(loc::wallResetVerdictWord(rep.verdict));
    CHECK(rep.verdict == loc::WallResetVerdict::Cut);
    CHECK(rep.samples > 0);
    CHECK(returnedAt - calledAt <= 6.0 * kTick + 1e-9);  // the tick the abandon landed, not windowMax
    CHECK(report.abandoned);
    checkSame(after, before);  // the Localizer BIT-IDENTICAL, as for the deadline's cut
    CHECK(r.walls.log.last.verdict == loc::WallResetVerdict::Cut);
}

// Bug caught (the fresh verifier's finding of 2026-11-10, its mutation V11: the CUT line's form chosen
// with `samples > 1`): the line's two forms split between NO sample and ONE, and a cut landing on the
// window's FIRST tick — exactly one sample taken — read "came before the window began", sending a
// student to debug a window that DID run. The equality edge, landed on exactly: the disable on the first
// tick after the call, so `samples == 1` (REQUIREd), and the line is the "landed in the window" form.
TEST_CASE("L6 T17 a DISABLE on the window's FIRST tick -- exactly one sample -- is a cut that LANDED in the "
          "window: the line's form at the one-sample edge") {
    CtxRig r;
    loc::WallResetReport rep{};
    LocState before{};
    LocState after{};
    double calledAt = 0.0;
    double returnedAt = 0.0;
    const shulib::sequence::RunGuardReport report = r.run([&](comp::AutonContext& ctx) {
        r.pacer.abandonAt = r.pacer.inner.paces + 1;  // the first tick of the window
        calledAt = r.now();
        before = stateOf(r);
        rep = comp::resetFromWall(ctx, CtxRig::leftWall());
        returnedAt = r.now();
        after = stateOf(r);
    });
    CAPTURE(loc::wallResetVerdictWord(rep.verdict));
    CAPTURE(rep.samples);
    REQUIRE(rep.samples == 1);  // ON the edge: one sample, not zero, not two
    CHECK(rep.verdict == loc::WallResetVerdict::Cut);
    CHECK(returnedAt - calledAt <= kTick + 1e-9);  // the tick the abandon landed
    CHECK(report.abandoned);
    checkSame(after, before);  // the Localizer BIT-IDENTICAL
    CHECK(r.walls.log.refused == 1);
    CHECK(r.walls.log.last.verdict == loc::WallResetVerdict::Cut);
    CHECK(r.walls.log.last.samples == 1);
    CHECK(CtxRig::lastWallLine(r.autonSink)
          == "left wall x REFUSED cut (LEFT sensor): the run's deadline or a disable landed in the window; "
             "nothing changed");
}

// Bug caught (a review finding of 2026-11-05): a call made after the run had ALREADY ended — a step
// before it overran the deadline — read as "the deadline landed in the window", which sends a student
// debugging a window that never ran. The guard's wait returns at once with no sample; the verdict is
// still CUT (nothing seated), and the line says the deadline came BEFORE the window began.
TEST_CASE("L6 T17 a call made after the deadline has passed is CUT with no sample, at once -- its line says the "
          "deadline came before the window began") {
    CtxRig r;
    r.entry.endActionAt = Time{0.3};
    r.entry.hardStopAt = Time{3.0};
    loc::WallResetReport rep{};
    double calledAt = 0.0;
    double returnedAt = 0.0;
    double deadline = 0.0;
    int readsBefore = 0;
    Pose2d before{};
    (void)r.run([&](comp::AutonContext& ctx) {
        deadline = r.now() + 0.3;
        // Run past the deadline (a step that overran it): the guard's own wait returns RunExpired.
        REQUIRE(ctx.guard.waitFor([] { return false; }, Time{5.0}) == shulib::sequence::GuardedWaitResult::RunExpired);
        calledAt = r.now();
        readsBefore = r.reads();
        before = r.rig.loc.pose();
        rep = comp::resetFromWall(ctx, CtxRig::leftWall());
        returnedAt = r.now();
    });
    CAPTURE(loc::wallResetVerdictWord(rep.verdict));
    REQUIRE(calledAt >= deadline - 1e-9);
    CHECK(rep.verdict == loc::WallResetVerdict::Cut);
    CHECK(rep.samples == 0);
    CHECK(returnedAt == calledAt);           // no tick: zero latency
    CHECK(r.reads() == readsBefore);         // nothing read
    CHECK(samePose(before, r.rig.loc.pose()));
    CHECK(r.walls.log.last.verdict == loc::WallResetVerdict::Cut);
    CHECK(CtxRig::lastWallLine(r.autonSink)
          == "left wall x REFUSED cut (LEFT sensor): the run's deadline or a disable came before the window "
             "began; nothing changed");
}

// ═══ The slot ══════════════════════════════════════════════════════════════════════

// Bug caught: a robot with no wall sensors (the host test plant's tester rig, a robot file without the
// block) dereferencing a null slot — or reading something anyway.
TEST_CASE("L6 T17 no slot: 'no sensor', the clock unmoved, nothing read, the line said") {
    CtxRig r{/*withSlot=*/false};
    loc::WallResetReport rep{};
    double t0 = 0.0;
    double t1 = 0.0;
    Pose2d before{};
    (void)r.run([&](comp::AutonContext& ctx) {
        before = r.rig.loc.pose();
        t0 = r.now();
        rep = comp::resetFromWall(ctx, CtxRig::leftWall());
        t1 = r.now();
    });
    CHECK(rep.verdict == loc::WallResetVerdict::NoSensor);
    CHECK_FALSE(rep.faultChannel);  // no slot: no sensor, so no fault channel either
    CHECK(t1 == t0);
    CHECK(r.reads() == 0);
    CHECK(samePose(before, r.rig.loc.pose()));
    CHECK(CtxRig::lastWallLine(r.autonSink)
          == "left wall x REFUSED no sensor (LEFT sensor): this robot declares no wall sensors");
    CHECK(r.walls.log.refused == 0);  // there is no slot to file it in
}

TEST_CASE("L6 T17 a refused side through the everyday form: 'sensor absent' with the Robot's sentence, filed, "
          "nothing read") {
    CtxRig r;
    r.walls.left.sensor = nullptr;
    r.walls.left.faults = nullptr;
    r.walls.left.absence = "port 4 has no distance sensor";
    loc::WallResetReport rep{};
    (void)r.run([&](comp::AutonContext& ctx) { rep = comp::resetFromWall(ctx, CtxRig::leftWall()); });
    CHECK(rep.verdict == loc::WallResetVerdict::SensorAbsent);
    CHECK(std::string{rep.detail} == "port 4 has no distance sensor");
    CHECK(r.reads() == 0);
    CHECK(r.walls.log.refused == 1);
    CHECK(CtxRig::lastWallLine(r.autonSink)
          == "left wall x REFUSED sensor absent (LEFT sensor): port 4 has no distance sensor");
    // The RIGHT side is the named side's: asking for it judges the RIGHT side's mount, not the left's
    // (the left's, refused above, would say SENSOR ABSENT again). The right beam faces away from the left
    // wall, so the pre-check refuses NOT SQUARE from that mount alone — reading no sensor at all.
    loc::WallResetRequest right = CtxRig::leftWall();
    right.sensor = loc::WallSide::Right;
    (void)r.run([&](comp::AutonContext& ctx) { rep = comp::resetFromWall(ctx, right); });
    CHECK(rep.verdict == loc::WallResetVerdict::NotSquare);
    CHECK(rep.side == loc::WallSide::Right);
    CHECK(r.reads() == 0);
}

// Bug caught (a review finding of 2026-11-05 — five mutants survived): an EARLY refusal (the start,
// the slot — decided before the sampler, by the everyday form's own code) that fills its report from
// the defaults instead of the request. Every other early-refusal case asks for the LEFT wall with the
// LEFT sensor, which ARE the report's defaults (axis x, side LEFT), so a report that forgot the side,
// the axis, the `before` coordinate, the fault channel, the `maxCorrection` echo or the Warn level
// would pass them all. Here the request is the RED wall (axis y) with the RIGHT sensor — whose
// read-fault channel is taken away while the LEFT one keeps its own — so each field has only one
// right answer, and the line names the red wall, y and the RIGHT sensor byte for byte.
TEST_CASE("L6 T17 an early refusal reports the REQUEST's wall, axis, side, before, fault channel and limit -- "
          "never the defaults -- at Warn") {
    loc::WallResetRequest red{};
    red.wall = loc::override2026::kRedWall;
    red.sensor = loc::WallSide::Right;
    red.maxCorrection = Length{3.0};
    // The record of the last WALL line in `sink`.
    const auto lastWall = [](const shulib::hal::fake::FakeTelemetrySink& sink) {
        for (int i = sink.size() - 1; i >= 0; --i) {
            if (sink.at(i).subsystem == "WALL") {
                return sink.at(i);
            }
        }
        FAIL("no WALL line");
        return sink.at(0);
    };
    const auto checkFields = [](const loc::WallResetReport& rep, double y) {
        CHECK(rep.axis == loc::Axis::Y);
        CHECK(rep.side == loc::WallSide::Right);
        CHECK(rep.before.value() == y);  // the estimate's Y, exactly — not its x
        CHECK_FALSE(rep.faultChannel);   // the RIGHT side has none here (the LEFT one does)
        CHECK(rep.maxCorrection.value() == 3.0);
        CHECK_FALSE(rep.required);
    };
    SUBCASE("a RELATIVE start, the slot present") {
        CtxRig r;
        r.walls.right.faults = nullptr;
        REQUIRE(r.walls.left.faults != nullptr);
        r.entry.start = comp::StartPose::relative();
        loc::WallResetReport rep{};
        double y = 0.0;
        (void)r.run([&](comp::AutonContext& ctx) {
            y = r.rig.loc.pose().y().value();
            rep = comp::resetFromWall(ctx, red);
        });
        REQUIRE(rep.verdict == loc::WallResetVerdict::NotFieldStart);
        REQUIRE(std::abs(y - r.rig.loc.pose().x().value()) > 1.0);  // x and y told apart
        checkFields(rep, y);
        CHECK(r.reads() == 0);
        CHECK(r.walls.log.last.side == loc::WallSide::Right);  // the filed copy is the same report
        CHECK(r.walls.log.last.axis == loc::Axis::Y);
        const auto rec = lastWall(r.autonSink);
        CHECK(rec.level == hal::LogLevel::Warn);
        CHECK(rec.message
              == "red wall y REFUSED not a field start (RIGHT sensor): start RELATIVE -- the season's walls are "
                 "FIELD coordinates: declare a FIELD start");
    }
    SUBCASE("no slot") {
        CtxRig r{/*withSlot=*/false};
        loc::WallResetReport rep{};
        double y = 0.0;
        (void)r.run([&](comp::AutonContext& ctx) {
            y = r.rig.loc.pose().y().value();
            rep = comp::resetFromWall(ctx, red);
        });
        REQUIRE(rep.verdict == loc::WallResetVerdict::NoSensor);
        checkFields(rep, y);
        const auto rec = lastWall(r.autonSink);
        CHECK(rec.level == hal::LogLevel::Warn);
        CHECK(rec.message == "red wall y REFUSED no sensor (RIGHT sensor): this robot declares no wall sensors");
    }
}

// ═══ The robot's own limits ════════════════════════════════════════════════════════

// Bug caught (a review finding of 2026-11-05 — the mutant survived): the everyday form judging with
// the LIBRARY's limits instead of the ones the robot's slot carries (the robot file's `walls.limits`,
// D6 / HA-211) — a team that typed a tighter limit would be judged by the defaults, silently. The
// parked robot's sensor reads 7 in: under the defaults (ceiling 16 in) it SEATS (the case above);
// with the slot's ceiling typed at 6.5 in, every sample is out of range and it refuses.
TEST_CASE("L6 T17 the slot's own limits judge the everyday reset: a typed 6.5-in range ceiling refuses the 7-in "
          "reading the defaults would seat") {
    CtxRig r;
    r.walls.limits.rangeMax = Length{6.5};
    loc::WallResetReport rep{};
    LocState before{};
    LocState after{};
    (void)r.run([&](comp::AutonContext& ctx) {
        before = stateOf(r);
        rep = comp::resetFromWall(ctx, CtxRig::leftWall());
        after = stateOf(r);
    });
    CAPTURE(loc::wallResetVerdictWord(rep.verdict));
    CHECK(rep.verdict == loc::WallResetVerdict::OutOfRange);
    checkSame(after, before);
    CHECK(r.walls.log.last.verdict == loc::WallResetVerdict::OutOfRange);
    CHECK(CtxRig::lastWallLine(r.autonSink).find("6.50") != std::string::npos);  // the typed ceiling, said
}

// ═══ D3 through then(), and a program error ════════════════════════════════════════

// Bug caught: `required` lost on the way through the context form — a refused required reset that the
// chain walks past, building the next leg on an estimate the author said must be re-seated first.
TEST_CASE("L6 T17 D3 through then(): a refused NON-required reset continues the chain; a refused REQUIRED one "
          "stops it ActionFailed") {
    CtxRig r;
    r.entry.start = comp::StartPose::relative();  // every reset below is refused
    shulib::chassis::RoutineResult loose{};
    shulib::chassis::RoutineResult required{};
    (void)r.run([&](comp::AutonContext& ctx) {
        Routine a{ctx, "not-required"};
        a.then([&ctx] { return comp::resetFromWall(ctx, CtxRig::leftWall(false)).continues(); }, "left wall x")
            .then([] { return true; }, "after");
        loose = a.result();
        Routine b{ctx, "required"};
        b.then([&ctx] { return comp::resetFromWall(ctx, CtxRig::leftWall(true)).continues(); }, "left wall x")
            .then([] { return true; }, "after");
        required = b.result();
    });
    CHECK(loose.ok);
    CHECK(loose.completed == 2);
    CHECK_FALSE(required.ok);
    CHECK(required.cause == RoutineStopCause::ActionFailed);
    CHECK(std::string{required.stoppedName} == "left wall x");
    CHECK(required.skipped == 1);
    CHECK(r.walls.log.refused == 2);
}

// Bug caught: a context form called outside any run (no deadline, no disable to honour) quietly
// sampling through a dead guard — or refusing as if it were a field condition. An AutonContext lives
// for ONE guarded run: outside it is a program error, loud, before anything is read.
TEST_CASE("L6 T17 a call with the guard NOT running is a PRECONDITION -- before the start check, reading "
          "nothing") {
    CtxRig r;
    REQUIRE_FALSE(r.guard.running());
    comp::AutonContext ctx{r.chassis, r.guard, r.autonSink, {}, r.entry};
    CHECK_THROWS_AS((void)comp::resetFromWall(ctx, CtxRig::leftWall()), shulib::PreconditionError);
    r.entry.start = comp::StartPose::relative();
    CHECK_THROWS_AS((void)comp::resetFromWall(ctx, CtxRig::leftWall()), shulib::PreconditionError);
    CHECK(r.reads() == 0);
    CHECK(CtxRig::wallLines(r.autonSink) == 0);
    CHECK(r.walls.log.refused == 0);
}

// ═══ Through the REAL Robot and runAuton: the setup checks and the robot file's limits ═══════
//
// The cases above hand the context a slot by hand. These build a robot::Robot from a robot file
// (wall_robot_rig.hpp — the fake provider, the field plant) and run the auton through
// competition::runAuton, the competition program's own call: what the robot file says must be what
// the everyday call judges, and the everyday form must run the explicit form's setup checks — not
// skip, reorder or "fix" them.

namespace {

/// Parked beside the left wall, nose along +y, the left sensor 7 in from the wall — and the auton's
/// FIELD start says exactly that.
const Pose2d kRobotParked{Length{wall_rig::xForLeftReading(7.0)}, Length{-38.0}, Angle::degrees(90.0)};

loc::WallResetReport gRobotRep{};
bool gRobotRequired = false;
bool gHoldArmedBefore = false;
bool gHoldStillArmed = false;
double gCallSeconds = -1.0;
LocState gRobotBefore{};
LocState gRobotAfter{};
shulib::localization::Localizer* gRobotLoc = nullptr;

LocState locStateOf(const shulib::localization::Localizer& l) {
    const Pose2d p = l.pose();
    return LocState{p.x().value(),
                    p.y().value(),
                    p.heading().radians(),
                    l.headingBias().value(),
                    l.distanceSinceCorrection().value(),
                    0.0,
                    0.0,
                    l.travelSinceSeat(loc::Axis::X).value(),
                    l.travelSinceSeat(loc::Axis::Y).value(),
                    l.turnedSinceHeadingSeed().value()};
}

/// The left-wall reset, timed, the Localizer snapshotted around it.
loc::WallResetReport timedReset(comp::AutonContext& ctx) {
    gRobotBefore = locStateOf(*gRobotLoc);
    const double t0 = ctx.chassis.deps().ctx->clock().now().value();
    const loc::WallResetReport rep = comp::resetFromWall(ctx, CtxRig::leftWall(gRobotRequired));
    gCallSeconds = ctx.chassis.deps().ctx->clock().now().value() - t0;
    gRobotAfter = locStateOf(*gRobotLoc);
    return rep;
}

/// D4's step through then(), then one more step.
shulib::chassis::RoutineResult resetThenStep(comp::AutonContext& ctx) {
    gRobotRep = {};
    Routine r{ctx, "reset-then-step"};
    r.then([&ctx] {
        gRobotRep = timedReset(ctx);
        return gRobotRep.continues();
    }, "left wall x");
    r.then([] { return true; }, "after");
    return r.result();
}

/// A Tier-3 hold armed through the auton's own Chassis, then the reset; the hold cancelled after.
shulib::chassis::RoutineResult holdThenReset(comp::AutonContext& ctx) {
    gRobotRep = {};
    shulib::motion::HoldPose hold{ctx.chassis.deps(), 5.0};
    ctx.chassis.scheduler().async(hold);
    (void)ctx.chassis.scheduler().tick();
    gHoldArmedBefore = ctx.chassis.scheduler().hasActiveMotion();  // checked outside: no REQUIRE in a run
    gRobotRep = timedReset(ctx);
    gHoldStillArmed = ctx.chassis.scheduler().hasActiveMotion();
    ctx.chassis.cancel();
    Routine r{ctx, "hold-then-reset"};
    return r.result();
}

/// The robot file: both side sensors at the plant's true mounts (wall_robot_rig.hpp).
shulib::config::RobotConfig robotFile() {
    return wall_robot::robotWith(wall_robot::wallsFor(wall_rig::trueLeft(), wall_rig::trueRight()));
}

/// Build `cfg` in a world at kRobotParked and run `fn` from a FIELD start there.
shulib::competition::AutonRunResult runOnRobot(wall_robot::WallWorld& world, const shulib::config::RobotConfig& cfg,
                                               shulib::competition::AutonRoutine fn) {
    REQUIRE(cfg.valid());
    shulib::robot::Robot& robot = world.build(cfg);
    gRobotLoc = &robot.localizer();
    const comp::AutonEntry e = wall_robot::entryFor("wall", fn, comp::StartPose::field(kRobotParked));
    return world.run(e);
}

}  // namespace

// Bug caught (a review finding of 2026-11-05): a Robot that loses `measured` while copying the robot
// file's mount (the boot line's MOUNT UNSET reads the CONFIG, so it would still say UNSET while the
// reset judged a "measured" mount of unmeasured numbers), or an everyday form that judges the mount
// differently from the explicit one. A robot file whose LEFT mount is unmeasured: every reset with it
// refuses MOUNT UNSET at once — nothing changed, no time — and D3 holds through then().
TEST_CASE("L6 T17 on the Robot: a robot file whose mount is UNMEASURED refuses 'mount unset' through runAuton -- "
          "nothing changed; the chain goes on unless required") {
    shulib::config::RobotConfig cfg = robotFile();
    cfg.sensors.walls.left.mount.measured = false;
    for (const bool required : {false, true}) {
        CAPTURE(required);
        gRobotRequired = required;
        wall_robot::WallWorld world{kRobotParked};
        const shulib::competition::AutonRunResult res = runOnRobot(world, cfg, &resetThenStep);
        REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
        CAPTURE(loc::wallResetVerdictWord(gRobotRep.verdict));
        CHECK(gRobotRep.verdict == loc::WallResetVerdict::MountUnset);
        CHECK(gCallSeconds == 0.0);
        checkSame(gRobotAfter, gRobotBefore);
        const loc::WallSensors& ws = world.robot->wallSensors();
        CHECK_FALSE(ws.left.mount.measured);  // what the Robot hands the reset: the file's word, kept
        CHECK(ws.log.refused == 1);
        CHECK(ws.log.seated == 0);
        CHECK(ws.log.last.verdict == loc::WallResetVerdict::MountUnset);
        CHECK(world.linesFrom("WALL") == 1);
        if (required) {
            CHECK_FALSE(res.routine.ok);
            CHECK(res.routine.cause == RoutineStopCause::ActionFailed);
            CHECK(res.routine.skipped == 1);
        } else {
            CHECK(res.routine.ok);
            CHECK(res.routine.completed == 2);
        }
    }
    gRobotRequired = false;
}

// Bug caught (a review finding of 2026-11-05): an everyday form that "helps" — cancelling an armed
// Tier-3 motion so it can sample — or that skips the motion check: the hold would then tick through
// the window and drive against the seat. With a hold armed through the auton's own Chassis, the reset
// refuses MOTION ACTIVE at once, changes nothing and cancels nothing.
TEST_CASE("L6 T17 on the Robot: a hold armed through the auton's Chassis refuses 'motion active' through runAuton "
          "-- at once, nothing changed, the hold still armed") {
    gHoldArmedBefore = false;
    gHoldStillArmed = false;
    wall_robot::WallWorld world{kRobotParked};
    const shulib::competition::AutonRunResult res = runOnRobot(world, robotFile(), &holdThenReset);
    REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    REQUIRE(gHoldArmedBefore);
    CAPTURE(loc::wallResetVerdictWord(gRobotRep.verdict));
    CHECK(gRobotRep.verdict == loc::WallResetVerdict::MotionActive);
    CHECK(gCallSeconds == 0.0);
    checkSame(gRobotAfter, gRobotBefore);
    CHECK(gHoldStillArmed);  // the reset cancels nothing
    CHECK(world.robot->wallSensors().log.last.verdict == loc::WallResetVerdict::MotionActive);
}

// Bug caught (a review finding of 2026-11-05 — the mutant survived): the robot file's typed limits
// (D6, HA-211) stored by the Robot but not the ones the everyday call judges with. The same parked
// robot, the same FIELD start: with the library's limits it SEATS; with the robot file's range ceiling
// typed at 6.5 in, its 7-in reading is out of range and the reset refuses, the typed ceiling in its
// line.
TEST_CASE("L6 T17 on the Robot: the robot file's typed limits judge the everyday reset through runAuton -- the "
          "defaults seat, a typed 6.5-in ceiling refuses") {
    {
        wall_robot::WallWorld world{kRobotParked};
        const shulib::competition::AutonRunResult res = runOnRobot(world, robotFile(), &resetThenStep);
        REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
        CAPTURE(loc::wallResetVerdictWord(gRobotRep.verdict));
        CHECK(gRobotRep.seated());
    }
    shulib::config::RobotConfig typed = robotFile();
    typed.sensors.walls.limits.rangeMax = Length{6.5};
    wall_robot::WallWorld world{kRobotParked};
    const shulib::competition::AutonRunResult res = runOnRobot(world, typed, &resetThenStep);
    REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    CAPTURE(loc::wallResetVerdictWord(gRobotRep.verdict));
    CHECK(gRobotRep.verdict == loc::WallResetVerdict::OutOfRange);
    checkSame(gRobotAfter, gRobotBefore);
    CHECK(world.robot->wallSensors().log.last.verdict == loc::WallResetVerdict::OutOfRange);
    bool saidCeiling = false;
    for (int i = 0; i < world.sink.size(); ++i) {
        if (world.sink.at(i).subsystem == "WALL") {
            saidCeiling = world.sink.at(i).message.find("(window 2.00..6.50)") != std::string::npos;
        }
    }
    CHECK(saidCeiling);
}
