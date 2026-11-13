// TANK1 — moveTo on a TANK steers back onto its line: the closed-loop proof on the A2 host test
// plant (motion/tank_steer.hpp's pure geometry is tank_steer_test.cpp).
//
// Every verdict is graded on GROUND TRUTH (h.truePose(), the per-tick TruthSample), never on the
// estimate the motion read. The weak side is a DegradationModel that scales ONE side's volts —
// the drift robot two showed (its sides differ by ~9 %; R3d's open-loop veer). Every number a
// check bounds was measured on the built code first (TANK1-PROGRESS.md) and is quoted beside the
// check as "measured"; a bound is the measurement plus a margin, never a number picked to pass.
// These are LAG-FREE plant numbers unless a case says otherwise (P13 adds motor lag): NOTHING here
// has run on a robot.
//
//   P1  the drift cases: settle with TANK1; maxSteer = 0 is today's tank, bit for bit
//   P2  reverse legs back onto the line (the direction factor's sign)
//   P3  the acceptance run (competition::acceptanceRun) with EITHER side weak
//   P4  bounded (the 15 degree cap), and no chatter near the target (the fade)
//   P5  the engage window (D2): 14.9 degrees steered, 15.1 degrees = today + ONE Warn
//   P6  never aims at the target POINT
//   P7  the record flag (TSTR): lit when TANK1 acts, dark on an even drive, never on X/H
//   P8  the yaw observer: its limit, its reset, IMU dropout, the omega clamp, a pinned robot
//   P9  StrafeTo / HoldPose / TurnTo / DriveBrake are unchanged on a tank, bit for bit
//   P10 X-drive and H-drive are unchanged, bit for bit (within one build — P10 (a), the
//       pristine-tree golden hashes, is an executor pre/post check, logged, not a CI test)
//   P11 the honest signal is strafeAuthority() == 0 exactly (a dead-strafe-wheel H-drive)
//   P12 robot two's odometry shape (drive encoders + IMU) sees the drift and steers it out
//   P13 the plant WITH motor lag: a regression fence that pins today's limits
//   P14 the observer never re-reads the IMU (ProsImu differentiated between CALLS until IMU1)
//   P15 no leg that settles with TANK1 off times out with it on (the Part A review's legs: a
//       start 10–14 degrees off the target heading, pointed at the line), and the lateral reach
//       has no holes (the lead — tank_steer.hpp)
//   P16 tankSteerStatus() reports the tick's cross, predicted cross and bend exactly
//
// TANK1 Part B — the result line's ` xt` column (the final cross-track error; the formatter's own
// bytes are test/motion_result_test.cpp):
//   XT1 the stats sink's formula on a hand-fed record stream (the FIRST live target decides
//       "did it translate"; the sign; beginMotion() forgets)
//   XT2 a tank moveTo's line carries xt: exactly the final estimate's line error, signed, true
//   XT3 a tank turn (also dragged by a weak side), a coasting brake and a hold of "here" do not;
//       a strafeTo and a hold of a spot away do
//   XT4 no X-drive or H-drive line ever does (an H-drive with no strafe authority is a tank)
//   XT5 no path data (NullSink), no column; maxSteer = 0 keeps the column
//   XT6 the auton shape (Routine: moveTo, turnTo, face, driveTo) — xt on the drives only
//   XT7 every outcome carries it: a user cancel and a fault abort mid-leg (and the "none yet"
//       record claims none)
//   XT8 guide 12's reading of the sign, with the steering ON: an xt of an inch or more names the
//       weak side on a leg that started on its line (a grid, with and without motor lag); a leg
//       that started beside its line can end on its start side; a smaller xt names no side

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/kinematics/h_drive.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/motion/drive_brake.hpp"
#include "shulib/motion/hold_pose.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/run_reporter.hpp"
#include "shulib/motion/strafe_to.hpp"
#include "shulib/motion/tank_steer.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/hostile/composed.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::MotionConfig;
using shulib::motion::MoveToPose;
using shulib::motion::TankLineError;
using shulib::motion::tankLineError;
using shulib::motion::TankSteerStatus;
using shulib::sim::TruthSample;
using shulib::units::Length;
using shulib::units::Time;

namespace {

namespace units = shulib::units;
namespace sim = shulib::sim;
namespace motion = shulib::motion;
namespace chassis = shulib::chassis;

constexpr double kDeg = Angle::kPi / 180.0;
constexpr double kTick = 0.01;

Pose2d P(double x, double y, double deg) {
    return Pose2d{Length{x}, Length{y}, Angle::degrees(deg)};
}

/// The weak side, plus the disturbances P8 needs. `wheel` 0 = LEFT, 1 = RIGHT (the tank's
/// kinematic wheel order). Windows are ABSOLUTE clock seconds; a negative bound = never.
struct TankWorld final : sim::DegradationModel {
    int wheel = 1;
    double scale = 1.0;
    double pinFrom = -1.0;   ///< both sides' volts forced to 0 in [pinFrom, pinTo): a pinned robot
    double pinTo = -1.0;
    double dropFrom = -1.0;  ///< the IMU reports not-ready in [dropFrom, dropTo)
    double dropTo = -1.0;
    double spinFrom = -1.0;  ///< the robot is turned (left side −spinVolts, right +spinVolts,
    double spinTo = -1.0;    ///< whatever is commanded) in [spinFrom, spinTo): a push
    double spinVolts = 0.0;

    TankWorld() = default;
    TankWorld(int w, double s) : wheel{w}, scale{s} {}

    units::Voltage effectiveVoltage(int w, units::Voltage commanded, Time now,
                                    sim::Rng& /*rng*/) override {
        if (now.value() >= pinFrom && now.value() < pinTo) {
            return units::Voltage{0.0};
        }
        if (now.value() >= spinFrom && now.value() < spinTo) {
            return units::Voltage{w == 0 ? -spinVolts : spinVolts};
        }
        return w == wheel ? units::Voltage{commanded.value() * scale} : commanded;
    }
    bool imuReady(bool trueReady, Time now) override {
        return (now.value() >= dropFrom && now.value() < dropTo) ? false : trueReady;
    }
};

MotionConfig tankOff() {
    MotionConfig m = motionConfig();
    m.tankSteer.maxSteer = 0.0;  // the documented off switch
    return m;
}

/// The pre-TANK1 engine, reached through MoveToPose's protected constructor with the DEFAULT
/// PoseMotionOptions (tankSteer false) — exactly the path StrafeTo/HoldPose use, and the path a
/// plain MoveToPose took before TANK1 existed. The oracle for "maxSteer = 0 is today's tank".
class UnsteeredMoveToPose final : public MoveToPose {
public:
    UnsteeredMoveToPose(const motion::MotionDeps& deps, const Pose2d& target,
                        const MotionConfig& config)
        : MoveToPose(deps, target, config, 0.0, motion::PoseMotionOptions{}) {}
};

int motWarns(const FakeTelemetrySink& s) {
    int n = 0;
    for (int i = 0; i < s.size(); ++i) {
        n += (s.at(i).level == LogLevel::Warn && s.at(i).subsystem == "MOT");
    }
    return n;
}

/// One run's per-tick evidence: exit, truth (memcmp-able), every commanded volt (bits), and a
/// status snapshot per tick for the motions that have one.
struct Trace {
    ExitReason exit = ExitReason::Running;
    int ticks = 0;
    std::vector<TruthSample> truth;
    std::vector<double> volts;
    std::vector<TankSteerStatus> status;
    int firstLiveTick = -1;        ///< index of the first tick that left WaitingForEstimate
    int motWarnsBeforeLive = -1;   ///< MOT Warns logged before that tick
    int motWarnsAtLive = -1;       ///< …and right after it
};

/// The controller-first hand loop of MotionRig::run, recording a Trace. `status` is read from
/// `mtp` when it is non-null (the motion under test, if it is a MoveToPose).
Trace runTraced(MotionRig& rig, motion::IMotion& m, const MoveToPose* mtp,
                const FakeTelemetrySink* sink, int maxTicks = 1500) {
    Trace t;
    m.start();
    for (int i = 0; i < maxTicks && t.exit == ExitReason::Running; ++i) {
        rig.loc.update();
        const int warnsBefore = sink != nullptr ? motWarns(*sink) : 0;
        t.exit = m.tick();
        ++t.ticks;
        if (t.firstLiveTick < 0 && m.state() != motion::MotionState::WaitingForEstimate) {
            t.firstLiveTick = i;
            t.motWarnsBeforeLive = warnsBefore;
            t.motWarnsAtLive = sink != nullptr ? motWarns(*sink) : 0;
        }
        if (mtp != nullptr) {
            t.status.push_back(mtp->tankSteerStatus());
        }
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            t.volts.push_back(rig.h.motor(w).commandedVoltage().value());
        }
        if (t.exit == ExitReason::Running) {
            rig.h.plant().step(Time{kTick});
        }
        t.truth.push_back(rig.h.sample());
    }
    return t;
}

bool sameTruth(const std::vector<TruthSample>& a, const std::vector<TruthSample>& b) {
    return a.size() == b.size()
           && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(TruthSample)) == 0);
}

/// Bit-identity of two runs' motion: exit, tick count, per-tick truth, exact volts (the G2
/// requireIdentical shape).
void requireIdentical(const Trace& a, const Trace& b) {
    REQUIRE(a.exit == b.exit);
    REQUIRE(a.ticks == b.ticks);
    REQUIRE(sameTruth(a.truth, b.truth));
    REQUIRE(a.volts.size() == b.volts.size());
    for (std::size_t i = 0; i < a.volts.size(); ++i) {
        REQUIRE(a.volts[i] == b.volts[i]);  // bits, not Approx
    }
}

/// Bit-identity of two record streams (every field, through the blackbox encoder's 428 bytes).
void requireSameRecords(const FakeTelemetrySink& a, const FakeTelemetrySink& b) {
    namespace bb = shulib::diag::blackbox;
    REQUIRE(a.recordCount() == b.recordCount());
    std::byte ba[bb::kTickPayloadBytes];
    std::byte bbuf[bb::kTickPayloadBytes];
    for (int i = 0; i < a.recordCount(); ++i) {
        const std::size_t na = bb::encodeTick(std::span<std::byte>{ba}, a.recordAt(i));
        const std::size_t nb = bb::encodeTick(std::span<std::byte>{bbuf}, b.recordAt(i));
        REQUIRE(na == bb::kTickPayloadBytes);
        REQUIRE(na == nb);
        REQUIRE(std::memcmp(ba, bbuf, na) == 0);
    }
}

void requireSameLog(const FakeTelemetrySink& a, const FakeTelemetrySink& b) {
    REQUIRE(a.size() == b.size());
    for (int i = 0; i < a.size(); ++i) {
        REQUIRE(a.at(i).level == b.at(i).level);
        REQUIRE(a.at(i).subsystem == b.at(i).subsystem);
        REQUIRE(a.at(i).message == b.at(i).message);
    }
}

/// The engaged-leg record counts a check reads: records of a running motion, how many carry
/// TSTR / SFB, and the sign changes of the commanded yaw rate inside the last 6 in (the fade
/// zone) — the chatter measure.
struct RecordAudit {
    int motion = 0;
    int tstr = 0;
    int sfb = 0;
    int yawFlipsNear = 0;
};

RecordAudit audit(const FakeTelemetrySink& s, int from = 0) {
    RecordAudit a;
    double lastW = 0.0;
    for (int i = from; i < s.recordCount(); ++i) {
        const auto& r = s.recordAt(i);
        if (r.activeCommandState == 0) {
            continue;
        }
        ++a.motion;
        a.tstr += r.tankSteerActive ? 1 : 0;
        a.sfb += r.strafeFallbackActive ? 1 : 0;
        if (std::hypot(r.errorX.value(), r.errorY.value()) < 6.0) {
            const double w = r.commanded.omega().value();
            if (std::abs(w) > 1e-3) {
                if (lastW != 0.0 && (w > 0.0) != (lastW > 0.0)) {
                    ++a.yawFlipsNear;
                }
                lastW = w;
            }
        }
    }
    return a;
}

/// A hand-loop tank leg: MoveToPose (or the unsteered oracle) on a 12 in tank, recording truth,
/// volts, status and the record stream.
struct TankLeg {
    TankKinematics kin{Length{12.0}};
    FakeTelemetrySink sink;
    TankWorld world;
    MotionRig rig;
    TankLeg(TankWorld w, const Pose2d& start, double plantKa = 0.0)
        : world{w}, rig{kin, cfgFor(start, plantKa), &sink, &world} {}
    static sim::SimHarnessConfig cfgFor(const Pose2d& start, double plantKa) {
        sim::SimHarnessConfig c = plantConfig();
        c.plant.initialPose = start;
        c.plant.wheelFf.kA = plantKa;  // 0 = the memoryless plant; > 0 = unmodelled motor lag
        return c;
    }
    [[nodiscard]] double now() { return rig.h.clock().now().value(); }
    Trace run(const Pose2d& target, const MotionConfig& mc, int maxTicks = 1500) {
        MoveToPose m{rig.deps, target, mc};
        return runTraced(rig, m, &m, &sink, maxTicks);
    }
    Trace runUnsteered(const Pose2d& target, const MotionConfig& mc, int maxTicks = 1500) {
        UnsteeredMoveToPose m{rig.deps, target, mc};
        return runTraced(rig, m, &m, &sink, maxTicks);
    }
};

double secs(const Trace& t) { return t.ticks * kTick; }

TankLineError lineOf(const Trace& t, const Pose2d& target) {
    const TruthSample& s = t.truth.back();
    return tankLineError(Pose2d{Length{s.x}, Length{s.y}, Angle::radians(s.theta)}, target);
}

/// The peak |true heading − target heading| (rad) over every tick of a run.
double peakHeadErr(const Trace& t, const Pose2d& target) {
    double peak = 0.0;
    for (const TruthSample& s : t.truth) {
        peak = std::max(peak, std::abs(Angle::radians(s.theta).errorTo(target.heading())));
    }
    return peak;
}

/// Chassis-level rig with a pacer that records the same Trace (truth + volts per pace).
struct TracePacer final : motion::ITickPacer {
    explicit TracePacer(sim::SimHarness& harness) : h{&harness} {}
    void pace() override {
        if (truth.size() > 100000) {
            throw std::runtime_error("TracePacer: pace cap exceeded");
        }
        for (int w = 0; w < h->motorCount(); ++w) {
            volts.push_back(h->motor(w).commandedVoltage().value());
        }
        h->plant().step(Time{kTick});
        truth.push_back(h->sample());
    }
    sim::SimHarness* h;
    std::vector<TruthSample> truth;
    std::vector<double> volts;
};

struct ChassisTank {
    TankKinematics kin{Length{12.0}};
    FakeTelemetrySink sink;
    TankWorld world;
    MotionRig rig;
    TracePacer pacer;
    chassis::Chassis chassis;
    ChassisTank(TankWorld w, const chassis::ChassisConfig& cc = chassisConfig(),
                const Pose2d& start = {}, double plantKa = 0.0)
        : world{w},
          rig{kin, TankLeg::cfgFor(start, plantKa), &sink, &world},
          pacer{rig.h},
          chassis{rig.deps, pacer, cc} {}
    [[nodiscard]] double now() { return rig.h.clock().now().value(); }
};

}  // namespace

// ═══ P1 — the drift cases ═══════════════════════════════════════════════════════════════

// Bug caught: the observer's sign flipped (M11 — it ADDS the weak side's yaw), the bend's sign
// flipped (M1), the observer compared against this tick's pre-pipeline omega (M15b — the drift
// comes back: x0.91 times out), or any change that stops a weak side's drift being removed. The
// 40 in auton from COMP-t's probe 4 through the public verb, one side weak: before TANK1 every
// one of these timed out (the maxSteer = 0 rows below).
TEST_CASE("P1 tank moveTo with one side weak: SETTLED on its line (truth), to a 25 % weak side, "
          "either side") {
    struct Case {
        int wheel;
        double scale;
        double measuredSecs;  // measured on the built code (TANK1-PROGRESS.md)
    };
    const Case cases[] = {{1, 0.97, 1.83}, {1, 0.95, 1.85}, {1, 0.91, 1.89},
                          {1, 0.85, 2.04}, {1, 0.75, 2.57}, {0, 0.91, 1.89}};
    const Pose2d target = P(40, 0, 0);
    for (const Case& c : cases) {
        CAPTURE(c.wheel);
        CAPTURE(c.scale);
        ChassisTank r{TankWorld{c.wheel, c.scale}};
        const double t0 = r.now();
        REQUIRE(r.chassis.moveTo(target) == ExitReason::Settled);
        const double t = r.now() - t0;
        CHECK(t <= c.measuredSecs + 0.2);
        const TankLineError e = tankLineError(r.rig.h.truePose(), target);
        CHECK(std::abs(e.cross) <= 0.12);  // measured ≤ 0.05 (x0.85)
        CHECK(std::abs(r.rig.h.truePose().heading().errorTo(target.heading())) <= 0.2 * kDeg);
        // The flag says TANK1 did it (P7 bounds the counts).
        CHECK(audit(r.sink).tstr > 0);
    }
}

// Bug caught: maxSteer = 0 not switching TANK1 off completely — the observer still running with
// the bend capped at 0 (M24), a Warn logged, a flag lit, or any byte of today's tank changed. The
// ORACLE is the pre-TANK1 engine itself (MoveToPose's protected constructor with the default
// PoseMotionOptions — tankSteer false — the path a plain MoveToPose took before TANK1): the two
// must agree in every record byte, every log line, every tick's truth and every volt. (That this
// oracle IS the pristine tree's tank was checked by hashing both — P10 (a), TANK1-PROGRESS.md; a
// closed-loop golden hash is not a CI test, because a libm change would turn it red.)
TEST_CASE("P1 the off switch: maxSteer = 0 restores the pre-TANK1 tank bit for bit, and it "
          "times out exactly where it used to") {
    struct Case {
        int wheel;
        double scale;
        double preTank1Y;  // COMP-t's probe 4 on the tree before TANK1 (TimedOut, END y)
    };
    const Case cases[] = {{1, 0.97, -0.93}, {1, 0.95, -1.56}, {1, 0.91, -2.86}, {0, 0.91, 2.86}};
    const Pose2d target = P(40, 0, 0);
    for (const Case& c : cases) {
        CAPTURE(c.wheel);
        CAPTURE(c.scale);
        TankLeg off{TankWorld{c.wheel, c.scale}, Pose2d{}};
        TankLeg oracle{TankWorld{c.wheel, c.scale}, Pose2d{}};
        const Trace a = off.run(target, tankOff());
        // The oracle keeps TANK1's DEFAULT config (maxSteer 15 degrees): only the option differs.
        const Trace b = oracle.runUnsteered(target, motionConfig());
        requireIdentical(a, b);
        requireSameRecords(off.sink, oracle.sink);
        requireSameLog(off.sink, oracle.sink);
        REQUIRE(a.exit == ExitReason::TimedOut);
        CHECK(std::abs(a.truth.back().y - c.preTank1Y) <= 0.01);
        CHECK(motWarns(off.sink) == 0);
        CHECK(audit(off.sink).tstr == 0);
        for (const TankSteerStatus& s : a.status) {
            REQUIRE_FALSE(s.eligible);
            REQUIRE(s.yawCorrection == 0.0);
        }
    }
}

// Bug caught: the recipe layer not reaching the same engine (Routine::moveTo is one plain
// MoveToPose — G2's route ≡ recipe). The auton line students write.
TEST_CASE("P1 through the recipe layer: Routine::moveTo on a 5 % and a 9 % weak side is ok()") {
    for (const double scale : {0.95, 0.91}) {
        CAPTURE(scale);
        ChassisTank r{TankWorld{1, scale}};
        chassis::Routine rt{r.chassis, "tank-40"};
        rt.moveTo(P(40, 0, 0));
        const chassis::RoutineResult res = rt.result();
        CHECK(res.ok);
        CHECK(res.completed == 1);
        CHECK(std::abs(tankLineError(r.rig.h.truePose(), P(40, 0, 0)).cross) <= 0.12);
    }
}

// ═══ P2 — reverse ═══════════════════════════════════════════════════════════════════════

// Bug caught: the direction factor taken without its sign (M2) — backing onto the line would
// steer the wrong way and a reverse leg would drift OFF it. Both reverse shapes: a target behind
// a robot facing 0 degrees, and L9's reverseTo shape (facing 180, target heading 180, the point
// behind). "It really backs up" is checked tick by tick on truth: x never moves the wrong way.
TEST_CASE("P2 reverse legs back onto the line: -40 in and the reverseTo shape, either side weak") {
    struct Case {
        const char* name;
        Pose2d start;
        Pose2d target;
        int wheel;
        bool xDecreases;
    };
    const Case cases[] = {
        {"-40 in, R x0.91", P(0, 0, 0), P(-40, 0, 0), 1, true},
        {"-40 in, L x0.91", P(0, 0, 0), P(-40, 0, 0), 0, true},
        {"reverseTo shape, R x0.91", P(0, 0, 180), P(40, 0, 180), 1, false},
        {"reverseTo shape, L x0.91", P(0, 0, 180), P(40, 0, 180), 0, false},
    };
    for (const Case& c : cases) {
        CAPTURE(c.name);
        TankLeg leg{TankWorld{c.wheel, 0.91}, c.start};
        const Trace t = leg.run(c.target, motionConfig());
        REQUIRE(t.exit == ExitReason::Settled);
        CHECK(secs(t) <= 1.90 + 0.2);                           // measured 1.90 s
        CHECK(std::abs(lineOf(t, c.target).cross) <= 0.1);     // measured 0.02
        CHECK(t.status.back().engaged);
        double prevX = c.start.x().value();
        for (const TruthSample& s : t.truth) {
            if (c.xDecreases) {
                REQUIRE(s.x <= prevX);
            } else {
                REQUIRE(s.x >= prevX);
            }
            REQUIRE(s.vx <= 1e-9);  // body velocity backwards (or stopped) on every tick
            prevX = s.x;
        }
    }
}

// ═══ P3 — the acceptance run, either side ═══════════════════════════════════════════════

// Bug caught: a steered tank that finishes the 40 in but not the library's own acceptance run
// (a turn, a face and a driveTo between), or one side only. Graded against the driveTo leg's OWN
// target heading — the approach bearing computed when the step ran, read from
// lastCompleted().targetPose — not a fixed 180 degrees: a weak side's turns translate the robot,
// so the bearing back to (0, 0) is not exactly 180 (measured 177.11 at R x0.91, -177.10 at L).
TEST_CASE("P3 the acceptance run FINISHES with either side 5 % or 9 % weak; 35 % weak stops "
          "honestly at step 1") {
    const shulib::competition::AutonEntry entry = shulib::competition::acceptanceRunEntry(
        shulib::competition::AutonKind::Match, Time{25.0}, Time{28.0});
    for (const int wheel : {1, 0}) {
        for (const double scale : {0.95, 0.91}) {
            CAPTURE(wheel);
            CAPTURE(scale);
            ChassisTank r{TankWorld{wheel, scale}};
            shulib::sequence::RunGuard guard{r.pacer};
            shulib::competition::AutonContext ctx{r.chassis, guard, r.sink, {}, entry};
            const chassis::RoutineResult res = shulib::competition::acceptanceRun(ctx);
            REQUIRE(res.ok);
            CHECK(res.completed == 4);
            const Pose2d end = r.rig.h.truePose();
            CHECK(std::hypot(end.x().value(), end.y().value()) <= 0.5);  // measured ≤ 0.37
            const motion::CompletedMotion& last = r.chassis.lastCompleted();
            CHECK(std::string{last.name} == "MoveToPose");
            CHECK(std::abs(end.heading().errorTo(last.targetPose.heading())) <= 0.02);  // ≤ 0.0030
            CHECK(r.rig.latch.raiseCount(shulib::diag::FaultCode::MotionTimeout) == 0);
        }
        // The honest stop: a 35 % weak side cannot hold the first leg's heading — step 1 times
        // out, ONE MOTION_TIMEOUT, and the run stops there (measured END (23.37, -/+0.49)).
        CAPTURE(wheel);
        ChassisTank r{TankWorld{wheel, 0.65}};
        shulib::sequence::RunGuard guard{r.pacer};
        shulib::competition::AutonContext ctx{r.chassis, guard, r.sink, {}, entry};
        const chassis::RoutineResult res = shulib::competition::acceptanceRun(ctx);
        CHECK_FALSE(res.ok);
        CHECK(res.completed == 0);
        CHECK(res.stoppedAt == 1);
        CHECK(res.exit == ExitReason::TimedOut);
        CHECK(r.rig.latch.raiseCount(shulib::diag::FaultCode::MotionTimeout) == 1);
        const Pose2d end = r.rig.h.truePose();
        CHECK(end.x().value() > 23.0);
        CHECK(end.x().value() < 24.0);
        // pulled toward the weak side
        CHECK((wheel == 1 ? -end.y().value() : end.y().value()) > 0.3);
    }
}

// ═══ P4 — bounded, and quiet at the target ══════════════════════════════════════════════

// Bug caught: the bend clamp removed (M3 — a 12 in offset asks for a 69 degree "bend", i.e. a
// planned turn toward the line) and the fade removed (M4 — the bend stays at full strength
// through arrival: the yaw command flips sign near the target, lateral 2 in takes 3.2 s); and
// TSTR blind to the BEND (the Part A review's S1: the flag's floor compared against the yaw
// correction alone) — on a clean drive the correction stays far under the floor, so on these
// legs only the bend's share can light the flag, and it must. LAG-FREE plant numbers: P13 pins
// what motor lag does to the same cases.
TEST_CASE("P4 the heading never leaves target +/- (maxSteer + 3 degrees); no yaw chatter inside "
          "the last 6 in; small offsets along the heading settle") {
    const double band = motionConfig().tankSteer.maxSteer + 3.0 * kDeg;
    {
        // Lateral 12 in: bounded and honest — it cannot remove 12 in over 40 in at 15 degrees.
        TankLeg leg{TankWorld{}, Pose2d{}};
        const Pose2d target = P(40, 12, 0);
        const Trace t = leg.run(target, motionConfig());
        REQUIRE(t.exit == ExitReason::TimedOut);
        const double xt = lineOf(t, target).cross;
        CHECK(xt < -4.9);  // measured -5.10
        CHECK(xt > -5.3);
        CHECK(peakHeadErr(t, target) <= band);  // measured 14.26 degrees
    }
    const double floor =
        shulib::motion::kTankSteerNoiseFraction * motionConfig().maxAngularSpeed.value();
    for (const double y : {2.0, 6.0, -2.0, -6.0}) {
        CAPTURE(y);
        TankLeg leg{TankWorld{}, Pose2d{}};
        const Pose2d target = P(40, y, 0);
        const Trace t = leg.run(target, motionConfig());
        REQUIRE(t.exit == ExitReason::Settled);
        CHECK(secs(t) <= 2.2);  // measured 1.81 (2 in and 6 in); fade removed: 3.21
        CHECK(peakHeadErr(t, target) <= band);
        CHECK(std::abs(lineOf(t, target).cross) <= 0.45);  // measured 0.09 (2 in), 0.36 (6 in)
        const RecordAudit a = audit(leg.sink);
        if (std::abs(y) == 2.0) {
            CHECK(a.yawFlipsNear <= 3);  // measured 1
        }
        // The bend alone lights TSTR here: the correction never gets near the floor.
        double peakCorr = 0.0;
        for (const TankSteerStatus& s : t.status) {
            peakCorr = std::max(peakCorr, std::abs(s.yawCorrection));
        }
        CHECK(peakCorr < floor / 2.0);  // measured 0.022 / 0.043 rad/s against a 0.12 floor
        CHECK(a.tstr >= (std::abs(y) == 2.0 ? 18 : 45));  // measured 37 / 91 of 181
    }
    // The drift cases: no sign change of the commanded yaw rate inside the last 6 in (measured 0
    // on every one; with the fade removed, 25).
    for (const double scale : {0.97, 0.95, 0.91, 0.85, 0.75}) {
        CAPTURE(scale);
        TankLeg leg{TankWorld{1, scale}, Pose2d{}};
        const Trace t = leg.run(P(40, 0, 0), motionConfig());
        REQUIRE(t.exit == ExitReason::Settled);
        CHECK(audit(leg.sink).yawFlipsNear <= 3);
    }
}

// ═══ P5 — the engage window (D2) ════════════════════════════════════════════════════════

// Bug caught: the engage decision latch removed (M9 — a leg the heading loop turns into the
// window mid-run gets steered: the library making the author's turn), the Warn emitted every
// tick or never (M10), or an un-steered leg that is not EXACTLY today's. A leg that starts on
// its line 14.9 degrees off its heading is steered and settles; at 15.1 degrees it runs exactly
// as with TANK1 off and says so ONCE, at the first live tick.
TEST_CASE("P5 the window: +/-14.9 degrees settles (clean and 9 % weak); +/-15.1 degrees is "
          "bit-identical to maxSteer = 0 plus exactly ONE golden Warn") {
    const Pose2d target = P(40, 0, 0);
    for (const double scale : {1.0, 0.91}) {
        for (const double deg : {14.9, -14.9}) {
            CAPTURE(scale);
            CAPTURE(deg);
            TankLeg leg{TankWorld{1, scale}, P(0, 0, deg)};
            const Trace t = leg.run(target, motionConfig());
            REQUIRE(t.exit == ExitReason::Settled);
            CHECK(secs(t) <= 1.93 + 0.2);  // measured 1.80 / 1.91 / 1.93
            CHECK(t.status.back().engaged);
            CHECK(motWarns(leg.sink) == 0);  // an engaged leg never Warns
        }
        for (const double deg : {15.1, -15.1}) {
            CAPTURE(scale);
            CAPTURE(deg);
            TankLeg on{TankWorld{1, scale}, P(0, 0, deg)};
            TankLeg off{TankWorld{1, scale}, P(0, 0, deg)};
            const Trace a = on.run(target, motionConfig());
            const Trace b = off.run(target, tankOff());
            requireIdentical(a, b);
            requireSameRecords(on.sink, off.sink);
            CHECK(a.status.back().eligible);
            CHECK_FALSE(a.status.back().engaged);
            // Exactly one Warn, logged BY the first live tick (none during the boot wait), and it
            // is the golden.
            REQUIRE(a.firstLiveTick >= 0);
            CHECK(a.motWarnsBeforeLive == 0);
            CHECK(a.motWarnsAtLive == 1);
            REQUIRE(motWarns(on.sink) == 1);
            CHECK(motWarns(off.sink) == 0);
            for (int i = 0; i < on.sink.size(); ++i) {
                if (on.sink.at(i).level == LogLevel::Warn && on.sink.at(i).subsystem == "MOT") {
                    CHECK(on.sink.at(i).message
                          == "tank moveTo not steered: it starts 15.1 deg off its target heading "
                             "(steers within 15.0 deg) -- turn first: face()");
                }
            }
        }
    }
}

// ═══ P6 — never aims at the point ═══════════════════════════════════════════════════════

// Bug caught: a bend computed from the bearing to the target POINT (M23 — a pursuit law: it
// would swing the robot toward (0, 24)). A point beside the robot ON ITS OWN HEADING gives the
// line nothing to steer along: the leg is engaged, times out honestly, and never rotates.
// A point beside the robot with a 90 degree heading asked is the author's turn: not steered.
TEST_CASE("P6 a point beside the robot: engaged, no rotation, no motion, TimedOut; asked to face "
          "90 degrees there: not steered, today's leg + the Warn") {
    {
        TankLeg leg{TankWorld{}, Pose2d{}};
        const Pose2d target = P(0, 24, 0);
        const Trace t = leg.run(target, motionConfig());
        REQUIRE(t.exit == ExitReason::TimedOut);
        REQUIRE(t.firstLiveTick >= 0);
        for (std::size_t i = static_cast<std::size_t>(t.firstLiveTick); i < t.status.size(); ++i) {
            REQUIRE(t.status[i].engaged);  // engaged from its first live tick to the end
        }
        CHECK(peakHeadErr(t, target) <= 1.0 * kDeg);  // measured 0
        for (const TruthSample& s : t.truth) {
            REQUIRE(std::hypot(s.x, s.y) <= 0.5);
        }
        CHECK(motWarns(leg.sink) == 0);
    }
    {
        TankLeg on{TankWorld{}, Pose2d{}};
        TankLeg off{TankWorld{}, Pose2d{}};
        const Pose2d target = P(0, 24, 90);
        const Trace a = on.run(target, motionConfig());
        const Trace b = off.run(target, tankOff());
        requireIdentical(a, b);
        CHECK_FALSE(a.status.back().engaged);
        REQUIRE(motWarns(on.sink) == 1);
        bool golden = false;
        for (int i = 0; i < on.sink.size(); ++i) {
            golden = golden
                     || on.sink.at(i).message
                            == "tank moveTo not steered: it starts 90.0 deg off its target "
                               "heading (steers within 15.0 deg) -- turn first: face()";
        }
        CHECK(golden);
    }
}

// ═══ P7 — the flag ══════════════════════════════════════════════════════════════════════

// Bug caught: the flag never set (M19), the record field not written (M20), or the floor
// removed (M18). The hostility clause is the ONLY one that kills M18: without the floor the
// clean even drive still lights 0 records and the 5 % weak leg still lights plenty, but under
// composed hostility on an EVEN drive IMU rate noise reaching the observer lights it on nearly
// every tick (measured: 14 → about 2 100 ticks over ten seeds). Keep that clause.
TEST_CASE("P7 TSTR: lit when TANK1 acts, dark on a clean even drive, rare under hostility on an "
          "even drive, and TermSink prints it") {
    {
        TankLeg leg{TankWorld{1, 0.95}, Pose2d{}};
        REQUIRE(leg.run(P(40, 0, 0), motionConfig()).exit == ExitReason::Settled);
        const RecordAudit a = audit(leg.sink);
        CHECK(a.tstr >= 40);  // measured 81 of 186
        // TermSink renders a lit record's flag.
        shulib::hal::fake::FakeCharSink out;
        shulib::hal::fake::FakeClock clock;
        shulib::diag::TermSink term{clock, out};
        bool rendered = false;
        for (int i = 0; i < leg.sink.recordCount() && !rendered; ++i) {
            if (leg.sink.recordAt(i).tankSteerActive) {
                term.emit(leg.sink.recordAt(i));
                rendered = out.text().find(" TSTR") != std::string::npos;
            }
        }
        CHECK(rendered);
    }
    {
        TankLeg leg{TankWorld{}, Pose2d{}};
        REQUIRE(leg.run(P(40, 0, 0), motionConfig()).exit == ExitReason::Settled);
        CHECK(audit(leg.sink).motion > 0);
        CHECK(audit(leg.sink).tstr == 0);
    }
    // Composed hostility, EVEN drive, ten seeds: the estimate boots (3 s), is re-zeroed, and the
    // leg drives 40 in along the heading the estimate has.
    const TankKinematics kin{Length{12.0}};
    int lit = 0;
    int records = 0;
    int settled = 0;
    for (std::uint64_t seed = 1; seed <= 10; ++seed) {
        sim::FullHostility full{};
        sim::SimHarnessConfig cfg = plantConfig();
        cfg.plant.seed = seed;
        FakeTelemetrySink sink;
        ChassisRig c{kin, cfg, &sink, &full.model()};
        for (int i = 0; i < 300; ++i) {
            c.chassis.wait(Time{kTick});
        }
        c.chassis.setPose(P(0, 0, 0));
        const Angle h0 = c.chassis.pose().heading();
        const Pose2d target{Length{40.0 * std::cos(h0.radians())},
                            Length{40.0 * std::sin(h0.radians())}, h0};
        const int from = sink.recordCount();
        settled += c.chassis.moveTo(target) == ExitReason::Settled ? 1 : 0;
        const RecordAudit a = audit(sink, from);
        lit += a.tstr;
        records += a.motion;
    }
    CHECK(settled == 10);
    REQUIRE(records > 1000);
    CHECK(lit * 50 <= records);  // ≤ 2 % (measured 20 of 2 125 = 0.94 %)
}

// ═══ P8 — the yaw observer ══════════════════════════════════════════════════════════════

// Bug caught: the observer's limit clamp removed (M12), or only ONE of its two bounds kept
// (the Part A review's S17a/S17b — each alone survived a one-sided test). A 50 % weak side needs
// more than the limit: the correction must reach it and never pass it (the leg then times out —
// bounded). A weak RIGHT side pulls the robot right, so the correction is NEGATIVE; a weak LEFT
// side makes it positive — each bound is reached, by its own side.
TEST_CASE("P8 (a) the yaw correction reaches yawCorrectionLimit, either way, and never exceeds "
          "it") {
    const double limit = motionConfig().tankSteer.yawCorrectionLimit;
    for (const int wheel : {1, 0}) {
        CAPTURE(wheel);
        TankLeg leg{TankWorld{wheel, 0.5}, Pose2d{}};
        const Trace t = leg.run(P(40, 0, 0), motionConfig());
        double peak = 0.0;
        double signedPeak = 0.0;
        for (const TankSteerStatus& s : t.status) {
            REQUIRE(std::abs(s.yawCorrection) <= limit);
            if (std::abs(s.yawCorrection) > peak) {
                peak = std::abs(s.yawCorrection);
                signedPeak = s.yawCorrection;
            }
        }
        CHECK(peak == limit);
        CHECK(signedPeak == (wheel == 1 ? -limit : limit));  // measured -1.500 / +1.500
        CHECK(t.exit == ExitReason::TimedOut);
    }
}

// Bug caught: the observer not re-armed by start() (M13): a re-started motion would begin by
// cancelling the previous run's weak side. After a 9 % weak leg the correction is non-zero;
// start() must zero it and forget the engage decision before the first live tick, and the next
// run on an even drive must not light the flag on its first record.
TEST_CASE("P8 (b) start() re-arms the observer and the engage decision") {
    TankLeg leg{TankWorld{1, 0.91}, Pose2d{}};
    MoveToPose m{leg.rig.deps, P(40, 0, 0), motionConfig()};
    const Trace t = runTraced(leg.rig, m, &m, &leg.sink);
    REQUIRE(t.exit == ExitReason::Settled);
    REQUIRE(m.tankSteerStatus().engaged);
    REQUIRE(std::abs(m.tankSteerStatus().yawCorrection) > 0.01);  // measured -0.067
    m.start();
    CHECK(m.tankSteerStatus().yawCorrection == 0.0);
    CHECK_FALSE(m.tankSteerStatus().engaged);
    CHECK(m.tankSteerStatus().eligible);
    // Second run on an EVEN drive from where the first ended.
    leg.world.scale = 1.0;
    const int from = leg.sink.recordCount();
    leg.rig.loc.update();
    (void)m.tick();
    REQUIRE(leg.sink.recordCount() > from);
    CHECK_FALSE(leg.sink.recordAt(from).tankSteerActive);
    CHECK(m.tankSteerStatus().engaged);
    // …and the lead's memory of the last commanded speed is gone too: no command yet this run,
    // so the first tick steers on the error now. (Not re-armed, it would predict from the
    // previous run's last command.)
    CHECK(m.tankSteerStatus().crossAhead == m.tankSteerStatus().cross);
}

// Bug caught: the observer ignoring isReady() (M14): while the IMU is not ready its rate is not
// a measurement, and the correction must HOLD rather than learn from it.
TEST_CASE("P8 (c) an IMU dropout mid-leg: the correction does not change while the IMU is not "
          "ready") {
    TankLeg leg{TankWorld{1, 0.91}, Pose2d{}};
    leg.world.dropFrom = leg.now() + 0.30;
    leg.world.dropTo = leg.now() + 0.60;
    MoveToPose m{leg.rig.deps, P(40, 0, 0), motionConfig()};
    m.start();
    int dropTicks = 0;
    double held = 0.0;
    bool inDrop = false;
    auto reason = ExitReason::Running;
    for (int i = 0; i < 1500 && reason == ExitReason::Running; ++i) {
        leg.rig.loc.update();
        reason = m.tick();
        const double c = m.tankSteerStatus().yawCorrection;
        if (!leg.rig.h.context().imu().isReady()) {
            if (!inDrop) {
                inDrop = true;
                held = c;
            }
            ++dropTicks;
            REQUIRE(c == held);  // measured change 0.000000
        }
        if (reason == ExitReason::Running) {
            leg.rig.h.plant().step(Time{kTick});
        }
    }
    CHECK(dropTicks >= 25);            // the dropout really happened mid-leg (measured 30)
    CHECK(std::abs(held) > 0.1);       // …while the observer had something to hold (-0.50)
    CHECK(reason == ExitReason::Settled);
}

// Bug caught: the observer comparing against last tick's PRE-clamp omega (M15a). At the default
// 6 rad/s the clamp never binds on a tank leg, so M15a is equivalent there; with the budget
// lowered to 0.6 rad/s the clamp binds during the 15 degree bend, and the clamp's own shortfall
// must NOT read as a disturbance. The only case that kills M15a (measured: peak 0.029
// rad/s; with M15a, 0.482). The leg is 50 in long: with a turn rate that slow, a 6 in
// offset over 40 in ends 0.63 in short with the lead (it settled without it, on the right phase
// of the under-damped line loop the lead removed — the Part A review) and over 50 in settles.
TEST_CASE("P8 (d) the omega clamp's shortfall is not a disturbance (maxAngularSpeed 0.6 rad/s, "
          "lateral 6 in over 50 in)") {
    MotionConfig mc = motionConfig();
    mc.maxAngularSpeed = units::AngularVelocity{0.6};
    TankLeg leg{TankWorld{}, Pose2d{}};
    const Trace t = leg.run(P(50, 6, 0), mc);
    CHECK(t.exit == ExitReason::Settled);  // measured 2.00 s
    for (const TankSteerStatus& s : t.status) {
        REQUIRE(std::abs(s.yawCorrection) <= 0.1);
    }
}

// Bug caught: the observer running before the motion has commanded anything (the Part A review's
// S6 — `hasOmegaCmd_` ignored, which survived every other case: they all start at rest, where the
// Localizer's rate is 0 anyway). A motion's very first tick has dt 0, which already stops the
// observer — that covers a motion chained after another (P14's boundary re-read); but a motion
// that WAITS for the estimate (a fresh Localizer is live from its second update) reaches its first
// live tick with dt > 0 and nothing commanded yet, so any rate the robot has there — here it is
// being pushed round — would be learned as a disturbance. The first commanding tick's correction
// must be exactly 0 though the Localizer reads a real rate (measured 0.78 rad/s after
// 1 waiting tick; with S6 the correction is dt/(yawObserverTime + dt) of that rate,
// 0.071 rad/s).
TEST_CASE("P8 (f) the observer does not run before the motion's first command, even when the "
          "robot is turning as the estimate goes live") {
    TankLeg leg{TankWorld{}, Pose2d{}};  // nothing has ticked: a fresh Localizer
    const double t0 = leg.now();
    leg.world.spinFrom = t0;  // something turns the robot from the start
    leg.world.spinTo = t0 + 1.0;
    leg.world.spinVolts = 2.0;
    MoveToPose m{leg.rig.deps, P(40, 0, 0), motionConfig()};
    m.start();
    int waited = 0;
    bool checked = false;
    for (int i = 0; i < 100 && !checked; ++i) {
        leg.rig.loc.update();
        REQUIRE(m.tick() == ExitReason::Running);
        if (m.state() == motion::MotionState::WaitingForEstimate) {
            ++waited;
        } else {
            const double rate = leg.rig.loc.twist().omega().value();
            REQUIRE(waited >= 1);           // it waited, so this first live tick has dt > 0
            REQUIRE(std::abs(rate) > 0.3);  // …and the robot IS turning as it goes live
            REQUIRE(m.tankSteerStatus().engaged);
            CHECK(m.tankSteerStatus().yawCorrection == 0.0);
            checked = true;
        }
        leg.rig.h.plant().step(Time{kTick});
    }
    CHECK(checked);
}

// A FENCE for HA-179's blast radius, not a feature: while the robot cannot follow a yaw command
// the observer integrates the heading error, bounded by its limit. Both sides pinned at 0 V for
// half a second mid-leg: the correction stays within its limit, the heading within 6 degrees,
// and the leg still arrives — where the same leg with TANK1 off times out (y -2.86).
TEST_CASE("P8 (e) a pinned robot: bounded correction, bounded heading, still SETTLED") {
    TankLeg leg{TankWorld{1, 0.91}, Pose2d{}};
    leg.world.pinFrom = leg.now() + 0.5;
    leg.world.pinTo = leg.now() + 1.0;
    const Pose2d target = P(40, 0, 0);
    const Trace t = leg.run(target, motionConfig());
    CHECK(t.exit == ExitReason::Settled);  // measured 2.41 s
    for (const TankSteerStatus& s : t.status) {
        REQUIRE(std::abs(s.yawCorrection) <= 1.5);  // measured peak 0.618
    }
    CHECK(peakHeadErr(t, target) <= 6.0 * kDeg);  // measured 1.93 degrees
    TankLeg off{TankWorld{1, 0.91}, Pose2d{}};
    off.world.pinFrom = off.now() + 0.5;
    off.world.pinTo = off.now() + 1.0;
    const Trace o = off.run(target, tankOff());
    CHECK(o.exit == ExitReason::TimedOut);
    CHECK(std::abs(o.truth.back().y - -2.86) <= 0.01);
}

// ═══ P9 — the siblings are unchanged on a tank ══════════════════════════════════════════

// Bug caught: PoseMotionOptions::tankSteer defaulting to true (M17 — StrafeTo and HoldPose would
// steer), or TANK1 reaching TurnTo / DriveBrake. Each sibling is run with TANK1's defaults and
// with maxSteer = 0: bit-identical. The HoldPose is the EXPLICIT-pose constructor on a target 3 in
// ahead and 1 in aside: a capture-here hold at rest computes the same doubles even if engaged (no
// error, no rate), so it cannot tell M17 apart — the offset hold and the 9 % weak StrafeTo can.
TEST_CASE("P9 StrafeTo, HoldPose (explicit pose), TurnTo and DriveBrake on a tank: bit-identical "
          "with TANK1 on and off") {
    enum class Kind { StrafeAlong, StrafeLateral, HoldThere, Turn, Brake };
    for (const double scale : {1.0, 0.91}) {
        for (const Kind k :
             {Kind::StrafeAlong, Kind::StrafeLateral, Kind::HoldThere, Kind::Turn, Kind::Brake}) {
            CAPTURE(scale);
            CAPTURE(static_cast<int>(k));
            auto runOne = [&](const MotionConfig& mc, TankLeg& leg) -> Trace {
                switch (k) {
                    case Kind::StrafeAlong: {
                        motion::StrafeTo m{leg.rig.deps, Length{30.0}, Length{0.0}, mc};
                        return runTraced(leg.rig, m, &m, &leg.sink);
                    }
                    case Kind::StrafeLateral: {
                        motion::StrafeTo m{leg.rig.deps, Length{30.0}, Length{6.0}, mc};
                        return runTraced(leg.rig, m, &m, &leg.sink);
                    }
                    case Kind::HoldThere: {
                        motion::HoldPose m{leg.rig.deps, P(3, 1, 0), 1.0, mc};
                        return runTraced(leg.rig, m, &m, &leg.sink);
                    }
                    case Kind::Turn: {
                        motion::TurnTo m{leg.rig.deps, Angle::degrees(90.0), mc};
                        return runTraced(leg.rig, m, nullptr, &leg.sink);
                    }
                    case Kind::Brake: {
                        motion::DriveBrake m{leg.rig.deps, mc};
                        return runTraced(leg.rig, m, nullptr, &leg.sink);
                    }
                }
                return Trace{};
            };
            TankLeg on{TankWorld{1, scale}, Pose2d{}};
            TankLeg off{TankWorld{1, scale}, Pose2d{}};
            const Trace a = runOne(motionConfig(), on);
            const Trace b = runOne(tankOff(), off);
            REQUIRE(a.ticks > 1);
            requireIdentical(a, b);
            requireSameRecords(on.sink, off.sink);
            requireSameLog(on.sink, off.sink);
            CHECK(audit(on.sink).tstr == 0);
            for (const TankSteerStatus& s : a.status) {
                REQUIRE_FALSE(s.eligible);
            }
            if (k == Kind::StrafeLateral) {
                // C3's tank honesty, unchanged: a sideways translate times out, SFB lit.
                CHECK(a.exit == ExitReason::TimedOut);
                CHECK(audit(on.sink).sfb > 50);
            }
        }
    }
}

// ═══ P10 (b) — the holonomic drives are unchanged ═══════════════════════════════════════

namespace {

/// Every verb through the facade, with a RunReporter so the result lines are in the log too —
/// the brief's holonomicScript (probe_proposed.cpp) plus one leg that starts ON its heading with
/// a 2 in offset: the leg an eligibility test that let an H-drive in (M6) would steer.
void holonomicScript(const shulib::kinematics::IKinematics& kin, bool hostile,
                     const chassis::ChassisConfig& cc, FakeTelemetrySink& sink, Trace& out) {
    sim::FullHostility full{};
    sim::SimHarnessConfig cfg = plantConfig();
    cfg.plant.seed = hostile ? 7 : 1;
    cfg.plant.initialPose = P(-12, 6, 30);
    MotionRig rig{kin, cfg, &sink, hostile ? &full.model() : nullptr};
    TracePacer pacer{rig.h};
    chassis::Chassis c{rig.deps, pacer, cc};
    motion::RunReporter rep{sink, c.scheduler()};
    c.wait(Time{0.5});
    (void)c.moveTo(P(18, -9, -20));
    (void)c.strafeTo(Length{0}, Length{12});
    (void)c.turnTo(Angle::degrees(135));
    const Pose2d legs[] = {P(10, 20, 90), P(-20, 20, 180)};
    (void)c.followTrajectory(legs);
    (void)c.moveTo(P(40, 0, 0), {.maxLinearSpeed = units::Velocity{30.0}});
    (void)c.hold(Time{0.4});
    (void)c.brake();
    const Pose2d here = c.pose();
    const double th = here.heading().radians();
    (void)c.moveTo(Pose2d{Length{here.x().value() + 20.0 * std::cos(th) - 2.0 * std::sin(th)},
                          Length{here.y().value() + 20.0 * std::sin(th) + 2.0 * std::cos(th)},
                          here.heading()});
    out.truth = pacer.truth;
    out.volts = pacer.volts;
    out.ticks = static_cast<int>(pacer.truth.size());
}

}  // namespace

// Bug caught: TANK1 reaching a drivetrain WITH strafe authority — an eligibility test looser
// than == 0 (M6: `< 0.5` lets the 0.35 H-bot in), or any tank-only arithmetic leaking into the
// shared path. Within ONE build (libm-independent): defaults vs maxSteer = 0 on the X-drive and
// the H-bot, clean and under composed hostility — every record byte, log line, truth sample and
// volt equal, and the flag never lit.
TEST_CASE("P10 (b) X-drive and H-drive: every verb bit-identical with TANK1's defaults and with "
          "maxSteer = 0, and TSTR never lit") {
    const auto xk = shulib::kinematics::xDrive(Length{7.0});
    const auto hk = hBotKinematics();
    chassis::ChassisConfig off = chassisConfig();
    off.motion.tankSteer.maxSteer = 0.0;
    struct Drive {
        const char* name;
        const shulib::kinematics::IKinematics* kin;
    };
    for (const Drive& d : {Drive{"X", &xk}, Drive{"H", &hk}}) {
        for (const bool hostile : {false, true}) {
            CAPTURE(d.name);
            CAPTURE(hostile);
            FakeTelemetrySink sa;
            FakeTelemetrySink sb;
            Trace a;
            Trace b;
            holonomicScript(*d.kin, hostile, chassisConfig(), sa, a);
            holonomicScript(*d.kin, hostile, off, sb, b);
            REQUIRE(a.ticks > 1000);
            REQUIRE(sameTruth(a.truth, b.truth));
            REQUIRE(a.volts == b.volts);
            requireSameRecords(sa, sb);
            requireSameLog(sa, sb);
            CHECK(audit(sa).tstr == 0);
            CHECK(motWarns(sa) == motWarns(sb));
        }
    }
}

// ═══ P11 — the honest tank signal ═══════════════════════════════════════════════════════

// Bug caught: eligibility decided by the kinematics TYPE (M7 — `dynamic_cast<const
// TankKinematics*>`): an H-drive whose strafe wheel is declared dead (strafeTractionDerate 0,
// authority exactly 0) cannot correct a sideways error by strafing either, and must be steered;
// the default H-bot (authority 0.35) must not be.
TEST_CASE("P11 an H-drive with a dead strafe wheel (authority 0) is steered; the default H-bot "
          "is not eligible") {
    const auto dead = shulib::kinematics::hDrive({.trackWidth = Length{11.0},
                                                  .strafeWheelOffset = Length{-4.0},
                                                  .strafeTractionDerate = 0.0});
    REQUIRE(dead.strafeAuthority() == 0.0);
    {
        FakeTelemetrySink sink;
        TankWorld world;
        MotionRig rig{dead, plantConfig(), &sink, &world};
        MoveToPose m{rig.deps, P(40, 2, 0), motionConfig()};
        const Trace t = runTraced(rig, m, &m, &sink);
        CHECK(t.status.back().eligible);
        CHECK(t.status.back().engaged);
        CHECK(t.exit == ExitReason::Settled);  // measured 1.81 s; TANK1 off: TimedOut at -2.00
    }
    {
        const auto hk = hBotKinematics();
        MotionRig rig{hk};
        MoveToPose m{rig.deps, P(40, 2, 0), motionConfig()};
        const Trace t = runTraced(rig, m, &m, nullptr);
        CHECK_FALSE(t.status.back().eligible);
        CHECK_FALSE(t.status.back().engaged);
    }
    // ANY strafe authority, however small, is not a tank: "exactly 0", not "below some
    // threshold" (the Part A review's S2 — `< 0.3` — survived a test that only tried 1.0,
    // 0.35 and 0). A 1 % strafe wheel, a 1e-9 one and the smallest positive double.
    for (const double derate : {0.01, 1e-9, std::numeric_limits<double>::denorm_min()}) {
        CAPTURE(derate);
        const auto weak = shulib::kinematics::hDrive({.trackWidth = Length{11.0},
                                                      .strafeWheelOffset = Length{-4.0},
                                                      .strafeTractionDerate = derate});
        REQUIRE(weak.strafeAuthority() > 0.0);
        MotionRig rig{weak};
        MoveToPose m{rig.deps, P(40, 2, 0), motionConfig()};
        const Trace t = runTraced(rig, m, &m, nullptr, 50);
        CHECK_FALSE(t.status.back().eligible);
        CHECK_FALSE(t.status.back().engaged);
    }
}

// ═══ P12 — robot two's odometry shape ═══════════════════════════════════════════════════

// Bug caught: a steering law that only works when the estimate has an independent lateral
// sensor. Robot two has drive encoders + IMU (no tracking wheels), wired the way the builder
// wires it (the stall check told there is no independent motion source). On this plant the tank
// never skids, so the drift is all heading — which the IMU sees; HA-180 registers that this
// is also true on the robot.
TEST_CASE("P12 drive-encoder odometry (robot two's shape): the 5 % and 9 % weak 40 in settle, "
          "either side") {
    const TankKinematics kin{Length{12.0}};
    for (const int wheel : {1, 0}) {
        for (const double scale : {0.95, 0.91}) {
            CAPTURE(wheel);
            CAPTURE(scale);
            TankWorld world{wheel, scale};
            const sim::SimHarnessConfig cfg = plantConfig();
            sim::SimHarness h{kin, cfg, nullptr, &world};
            const shulib::hal::DriveGeometry geom =
                shulib::hal::DriveGeometry::fromDiameter(cfg.plant.driveWheelDiameter);
            shulib::localization::DriveEncoderOdometry odom{
                h.imu(),         *h.context().driveMotors()[0], *h.context().driveMotors()[1],
                geom,            geom,
                Length{12.0},    cfg.plant.initialPose};
            shulib::localization::ComplementaryFusion fusion{};
            shulib::localization::Localizer loc{h.clock(), h.imu(), odom, fusion};
            FakeTelemetrySink faultSink;
            shulib::diag::FaultLatch latch{faultSink, h.clock()};
            shulib::diag::HealthMonitor health{latch};
            const motion::MotionDeps deps{.ctx = &h.context(),
                                          .localizer = &loc,
                                          .kinematics = &kin,
                                          .faults = &latch,
                                          .health = &health};
            PlantPacer pacer{h};
            chassis::ChassisConfig cc = chassisConfig();
            cc.motion.stall.setAllWheels(geom);
            cc.motion.stall.independentMotionSource = false;
            chassis::Chassis c{deps, pacer, cc};
            loc.setPose(cfg.plant.initialPose);
            const Pose2d target = P(40, 0, 0);
            REQUIRE(c.moveTo(target) == ExitReason::Settled);  // measured 1.85 / 1.89 s
            CHECK(std::abs(tankLineError(h.truePose(), target).cross) <= 0.12);  // ≤ 0.019
        }
    }
}

// ═══ P13 — the plant WITH motor lag (a regression fence, not a claim about the robot) ═══

// Every other case here runs the memoryless plant (kA 0). This one gives the PLANT a motor time
// constant the controller does not model (controller kA stays 0, as R5a will leave it):
// kA 0.02 is about 0.12 s, kA 0.03 about 0.18 s. It pins TODAY's limits so a regression is seen
// AND an improvement is noticed. (c) was a measured FAILURE until the Part A review: a 6 in
// sideways offset ended half an inch beside the target with lag, because the line loop was
// under-damped; the lead (tank_steer.hpp) damps it, and (c) was flipped as this comment asked
// (guide 14, the cookbook and HA-179 updated with it). (e) is the new measured failure — a 10 in
// offset over 40 in still ends off the line with lag. If (e) ever SETTLES, update the same three
// documents, then flip it.
TEST_CASE("P13 with unmodelled motor lag: the 9 % weak 40 in still settles; lateral 2 in and "
          "6 in settle; lateral 10 in times out, bounded (today's limit)") {
    const Pose2d target = P(40, 0, 0);
    for (const int wheel : {1, 0}) {
        CAPTURE(wheel);
        TankLeg leg{TankWorld{wheel, 0.91}, Pose2d{}, 0.02};
        const Trace t = leg.run(target, motionConfig());
        REQUIRE(t.exit == ExitReason::Settled);  // (a) measured 1.47 s
        CHECK(secs(t) <= 1.8);
        CHECK(std::abs(lineOf(t, target).cross) <= 0.3);  // measured 0.18
        CHECK(audit(leg.sink).yawFlipsNear <= 4);          // measured 1
    }
    {
        TankLeg leg{TankWorld{}, Pose2d{}, 0.02};
        const Trace t = leg.run(P(40, 2, 0), motionConfig());
        CHECK(t.exit == ExitReason::Settled);  // (b) measured 1.45 s
    }
    {
        TankLeg leg{TankWorld{}, Pose2d{}, 0.02};
        const Pose2d lat6 = P(40, 6, 0);
        const Trace t = leg.run(lat6, motionConfig());
        REQUIRE(t.exit == ExitReason::Settled);   // (c) measured 1.45 s
        CHECK(peakHeadErr(t, lat6) <= motionConfig().tankSteer.maxSteer + 3.0 * kDeg);  // 15.08
        CHECK(audit(leg.sink).yawFlipsNear <= 4);  // measured 2 (8 before the lead)
    }
    {
        TankLeg leg{TankWorld{}, Pose2d{}, 0.02};
        const Pose2d lat10 = P(40, 10, 0);
        const Trace t = leg.run(lat10, motionConfig());
        REQUIRE(t.exit == ExitReason::TimedOut);  // (e) the measured verdict
        const TankLineError e = lineOf(t, lat10);
        CHECK(std::abs(e.along) <= 0.2);          // measured 0.00: it got there along the line…
        CHECK(std::abs(e.cross) > 0.5);           // …but 1.59 in off it
        CHECK(peakHeadErr(t, lat10) <= motionConfig().tankSteer.maxSteer + 3.0 * kDeg);  // 15.09
    }
    for (const int wheel : {1, 0}) {
        CAPTURE(wheel);
        TankLeg leg{TankWorld{wheel, 0.91}, Pose2d{}, 0.03};
        const Trace t = leg.run(target, motionConfig());
        REQUIRE(t.exit == ExitReason::Settled);  // (d) measured 2.09 s
        CHECK(secs(t) <= 2.5);
        CHECK(audit(leg.sink).yawFlipsNear <= 4);  // measured 3
    }
}

// ═══ P14 — the observer never re-reads the IMU ══════════════════════════════════════════

namespace {

/// An IImu shaped like the PRE-IMU1 ProsImu's default rate (DifferentiateRotation on a
/// microsecond clock): the FIRST yawRate() of each clock instant answers truly; every later call
/// in the same instant answers 0 ("zero") or 0 with a rare 50x spike ("spike") — what a second
/// read of a differentiated rotation microseconds after the first would return on a robot (derived
/// from the adapter's code and PROS's documented 10 ms copy, HA-213 — never measured). It leaves
/// out what the real adapter also did: that second read moved the baseline, so the NEXT tick's
/// first read was damaged too (the IMU1 brief's §0.2). Since IMU1 ProsImu answers once per tick,
/// and IMU1's T8 (test/imu_rate_once_per_tick_test.cpp) runs this scenario through the REAL
/// adapter; P14 stays as the pin that the observer reads loc.twist(). FakeImu is stateless, so
/// without this no host test can see a second read.
enum class ImuShape { Plain, Zero, Spike };

struct ProsShapedImu final : shulib::hal::IImu {
    ProsShapedImu(shulib::hal::fake::FakeImu& inner, const shulib::hal::IClock& clock,
                  ImuShape s)
        : f{&inner}, clk{&clock}, shape{s} {}
    Angle heading() const override { return f->heading(); }
    bool isReady() const override { return f->isReady(); }
    Angle pitch() const override { return f->pitch(); }
    Angle roll() const override { return f->roll(); }
    units::AngularVelocity yawRate() const override {
        const double t = clk->now().value();
        if (t != lastT) {
            lastT = t;
            callsThisInstant = 0;
        }
        ++callsThisInstant;
        if (callsThisInstant == 1 || shape == ImuShape::Plain) {
            return f->yawRate();
        }
        ++secondReads;
        secondReadTimes.push_back(t);
        if (shape == ImuShape::Spike) {
            ++k;
            if ((k * 2654435761U) % 100U < 3U) {
                return units::AngularVelocity{f->yawRate().value() * 50.0};
            }
        }
        return units::AngularVelocity{0.0};
    }
    shulib::hal::fake::FakeImu* f;
    const shulib::hal::IClock* clk;
    ImuShape shape;
    mutable double lastT = -1.0;
    mutable int callsThisInstant = 0;
    mutable unsigned k = 0;
    mutable int secondReads = 0;
    mutable std::vector<double> secondReadTimes;
};

/// The Chassis stack with the RobotContext AND the Localizer sharing one ProsShapedImu, and no
/// telemetry sink (so no record — the record reads of the IMU were D4's, fixed in ProsImu by
/// IMU1).
struct ProsImuRig {
    sim::SimHarness h;
    ProsShapedImu imu;
    chassis::RobotContext ctx;
    shulib::localization::PilonsOdometry odom;
    shulib::localization::ComplementaryFusion fusion;
    shulib::localization::Localizer loc;
    FakeTelemetrySink faultSink;
    shulib::diag::FaultLatch latch;
    shulib::diag::HealthMonitor health;
    motion::MotionDeps deps;
    TracePacer pacer;
    chassis::Chassis chassis;
    ProsImuRig(const shulib::kinematics::IKinematics& kin, sim::DegradationModel* world,
               ImuShape shape)
        : h{kin, plantConfig(), nullptr, world},
          imu{h.imu(), h.clock(), shape},
          ctx{chassis::RobotContextConfig{.clock = &h.clock(),
                                          .driveMotors = h.context().driveMotors(),
                                          .imu = &imu,
                                          .gps = &h.gps(),
                                          .battery = &h.battery(),
                                          .telemetry = &h.context().telemetry(),
                                          .tags = &h.context().tags(),
                                          .vision = &h.context().vision()}},
          odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()},
          fusion{},
          loc{h.clock(), imu, odom, fusion},
          latch{faultSink, h.clock()},
          health{latch},
          deps{.ctx = &ctx,
               .localizer = &loc,
               .kinematics = &kin,
               .faults = &latch,
               .health = &health},
          pacer{h},
          chassis{deps, pacer, chassisConfig()} {
        loc.setPose(plantConfig().plant.initialPose);
    }
};

}  // namespace

// Bug caught: the observer reading ctx.imu().yawRate() (M28 — the brief's first draft). On the
// robot before IMU1 the Localizer had already made the tick's read, so a second read would have
// been ~0 (or a spike — derived from the adapter's code, never measured): the observer would learn
// a phantom disturbance on every tick. It must use the rate the
// Localizer read THIS tick (loc.twist()). Measured with the draft: zero mode changes every drift
// leg, spike mode times out the 40 in from x0.97 and stops the acceptance run. Since IMU1 ProsImu
// answers once per tick (its second read is the first's); this stays as the pin on loc.twist().
TEST_CASE("P14 on a ProsImu-shaped IMU the steered runs are bit-identical to the plain IMU, and "
          "the IMU is never read twice in a tick") {
    const TankKinematics kin{Length{12.0}};
    for (const ImuShape shape : {ImuShape::Zero, ImuShape::Spike}) {
        for (const int wheel : {1, 0}) {
            CAPTURE(static_cast<int>(shape));
            CAPTURE(wheel);
            TankWorld wa{wheel, 0.91};
            TankWorld wb{wheel, 0.91};
            ProsImuRig shaped{kin, &wa, shape};
            ProsImuRig plain{kin, &wb, ImuShape::Plain};
            REQUIRE(shaped.chassis.moveTo(P(40, 0, 0)) == ExitReason::Settled);
            REQUIRE(plain.chassis.moveTo(P(40, 0, 0)) == ExitReason::Settled);
            REQUIRE(sameTruth(shaped.pacer.truth, plain.pacer.truth));
            REQUIRE(shaped.pacer.volts == plain.pacer.volts);
            CHECK(shaped.imu.secondReads == 0);
        }
        // The acceptance run's four legs (the turn and the face included), 9 % weak, one step
        // at a time so the motion BOUNDARIES are known.
        CAPTURE(static_cast<int>(shape));
        TankWorld wa{1, 0.91};
        TankWorld wb{1, 0.91};
        ProsImuRig shaped{kin, &wa, shape};
        ProsImuRig plain{kin, &wb, ImuShape::Plain};
        chassis::Routine ra{shaped.chassis, "acceptance-run"};
        chassis::Routine rb{plain.chassis, "acceptance-run"};
        std::vector<double> boundaries;
        ra.moveTo(P(24, 0, 0));
        boundaries.push_back(shaped.h.clock().now().value());
        ra.turnTo(Angle::degrees(90.0));
        boundaries.push_back(shaped.h.clock().now().value());
        ra.face(Length{0.0}, Length{0.0});
        boundaries.push_back(shaped.h.clock().now().value());
        ra.driveTo(Length{0.0}, Length{0.0});
        rb.moveTo(P(24, 0, 0))
            .turnTo(Angle::degrees(90.0))
            .face(Length{0.0}, Length{0.0})
            .driveTo(Length{0.0}, Length{0.0});
        CHECK(ra.result().ok);
        REQUIRE(sameTruth(shaped.pacer.truth, plain.pacer.truth));
        REQUIRE(shaped.pacer.volts == plain.pacer.volts);
        // Second reads DO happen here — three, and each one is the LOCALIZER's, at a motion
        // boundary: the scheduler ticks the next motion at the instant the previous one exited
        // (no pace between two blocking verbs), so Localizer::update reads the rate twice at
        // that instant. That was pre-existing (fixed in ProsImu by IMU1), and it never reaches
        // TANK1: a motion's first tick has no commanded rate to compare against (hasOmegaCmd_ is
        // false until the first command), so the observer does not run there. What this pins: no
        // second read anywhere else — none inside any motion, so none on an observer tick.
        REQUIRE(shaped.imu.secondReadTimes.size() == boundaries.size());
        for (std::size_t i = 0; i < boundaries.size(); ++i) {
            CHECK(shaped.imu.secondReadTimes[i] == boundaries[i]);
        }
    }
}

// ═══ P15 — no leg that settled before TANK1 times out with it ══════════════════════════

// Bug caught: the bend steering on the cross-track error NOW (the lead removed, leadTime 0 — the
// law as Part A first built it) or the lead's sign flipped. The Part A review's finding: a leg
// that starts 10–15 degrees off its target heading, turned TOWARD its line, with its point on the
// arc the heading loop drives, SETTLED before TANK1 — and with a bend on the present error it ran
// at the line, crossed it near the target, and the fade left 0.5–0.7 in nobody could remove:
// TIMEOUT, no Warn (the leg is engaged). The lead (tank_steer.hpp) damps the line loop. (a) the
// review's four legs, through the public verb; (b) the family they come from, both turning
// directions: every leg that settles with maxSteer = 0 must settle steered (measured with no
// lead: 100 of these legs regress; with the lead, 0); (c) the lateral reach has no
// holes: a faced 50 in leg settles every offset from 1 to 8 in (with no lead, 4, 5 and 6 in timed
// out while 7 and 8 settled — the reach depended on the phase of an oscillation), and so does the
// same leg BACKING UP, 50 in behind — which is what catches a lead that ignores the sign of the
// speed (a reverse leg predicted as if it drove forward: measured, 3 to 7 in then time out 0.61
// to 0.90 in off the line).
TEST_CASE("P15 a leg that settles with TANK1 off settles with it on: starts 10-14 degrees off, "
          "pointed at the line; and the lateral reach has no holes") {
    // (a) the review's legs (start heading, target), through Chassis::moveTo on a clean plant.
    struct ReviewLeg {
        double startDeg;
        Pose2d target;
    };
    const ReviewLeg legs[] = {{-12.0, P(29.9075, -2.3538, 0)},
                              {-10.0, P(29.9440, -1.8315, 0)},
                              {-14.0, P(29.8357, -3.1357, 0)},
                              {-12.0, P(23.9340, -1.7786, 0)}};
    for (const ReviewLeg& l : legs) {
        CAPTURE(l.startDeg);
        CAPTURE(l.target.y().value());
        ChassisTank on{TankWorld{}, chassisConfig(), P(0, 0, l.startDeg)};
        const double t0 = on.now();
        REQUIRE(on.chassis.moveTo(l.target) == ExitReason::Settled);  // was TIMEOUT, xt -0.5..-0.7
        CHECK(on.now() - t0 <= 2.0);                                   // measured 1.50-1.62 s
        CHECK(motWarns(on.sink) == 0);
    }
    // (b) the family: starts ±10/±12/±14 degrees, points 24/30/36 in out, bearings 0 to 8
    // degrees on the side the start heading turns toward (the arc's side), every 0.25 degree.
    int offSettled = 0;
    int regressed = 0;
    for (const double startDeg : {10.0, 12.0, 14.0, -10.0, -12.0, -14.0}) {
        for (const double d : {24.0, 30.0, 36.0}) {
            for (int k = 0; k <= 32; ++k) {
                const double b = (startDeg > 0.0 ? 1.0 : -1.0) * k * 0.25 * kDeg;
                const Pose2d target{Length{d * std::cos(b)}, Length{d * std::sin(b)},
                                    Angle::radians(0.0)};
                TankLeg off{TankWorld{}, P(0, 0, startDeg)};
                if (off.run(target, tankOff()).exit != ExitReason::Settled) {
                    continue;  // not settling before TANK1 either: not this fence's business
                }
                ++offSettled;
                TankLeg on{TankWorld{}, P(0, 0, startDeg)};
                const Trace t = on.run(target, motionConfig());
                if (t.exit != ExitReason::Settled) {
                    ++regressed;
                    CAPTURE(startDeg);
                    CAPTURE(d);
                    CAPTURE(k);
                    CHECK(t.exit == ExitReason::Settled);
                }
            }
        }
    }
    CHECK(offSettled >= 120);  // measured 144: the fence is not vacuous
    CHECK(regressed == 0);
    // (c) the reach at 50 in, faced, clean, forward and backing up: every offset 1..8 in
    // settles (and 10 in does not — measured, bounded: P4's 12 in case says why).
    for (const double along : {50.0, -50.0}) {
        for (int y = 1; y <= 8; ++y) {
            CAPTURE(along);
            CAPTURE(y);
            TankLeg leg{TankWorld{}, Pose2d{}};
            const Pose2d target = P(along, y, 0);
            const Trace t = leg.run(target, motionConfig());
            CHECK(t.exit == ExitReason::Settled);
            CHECK(std::abs(lineOf(t, target).cross) <= 0.45);  // measured ≤ 0.29, both ways
        }
    }
}

// ═══ P16 — the status says what steered ═════════════════════════════════════════════════

// Bug caught: tankSteerStatus() not reporting the tick's numbers (the Part A review's S4: the
// cross and bend writes dropped, no test read them), or reporting something other than what the
// heading loop used. On every commanding tick of a clean lateral 2 in leg: cross is exactly the
// line error of the pose the motion read (the Localizer's, updated before the tick); bend is
// exactly the bend of the predicted cross; the first live tick (no command yet) predicts nothing,
// so crossAhead == cross == -2 and the bend is +0.2 rad exactly (0.1 rad/in × 2 in, full fade,
// left).
TEST_CASE("P16 tankSteerStatus(): cross, crossAhead and bend are the tick's, exactly") {
    TankLeg leg{TankWorld{}, Pose2d{}};
    const Pose2d target = P(40, 2, 0);
    const MotionConfig mc = motionConfig();
    MoveToPose m{leg.rig.deps, target, mc};
    m.start();
    auto reason = ExitReason::Running;
    int live = 0;
    int leading = 0;
    for (int i = 0; i < 1500 && reason == ExitReason::Running; ++i) {
        leg.rig.loc.update();
        const Pose2d seen = leg.rig.loc.pose();  // what this tick's motion reads
        reason = m.tick();
        const TankSteerStatus s = m.tankSteerStatus();
        // Every tick that commanded (the exit tick returns before the heading loop: the status
        // then still describes the last commanding tick, by design).
        if (m.state() != motion::MotionState::WaitingForEstimate
            && reason == ExitReason::Running) {
            REQUIRE(s.engaged);
            const TankLineError e = tankLineError(seen, target);
            REQUIRE(s.cross == e.cross);
            REQUIRE(s.bend
                    == shulib::motion::tankSteerAngle(TankLineError{.along = e.along,
                                                                    .cross = s.crossAhead},
                                                      mc.tankSteer));
            REQUIRE(std::abs(s.bend) <= mc.tankSteer.maxSteer);
            if (live == 0) {
                CHECK(s.cross == -2.0);
                CHECK(s.crossAhead == s.cross);
                CHECK(s.bend == doctest::Approx(0.2).epsilon(1e-15));
            }
            leading += s.crossAhead != s.cross ? 1 : 0;
            ++live;
        }
        if (reason == ExitReason::Running) {
            leg.rig.h.plant().step(Time{kTick});
        }
    }
    CHECK(reason == ExitReason::Settled);
    CHECK(live > 100);
    CHECK(leading > live / 2);  // the lead is live while the leg moves (measured 178 of 179)
}

// ═══ TANK1 Part B — the result line's ` xt` column ══════════════════════════════════════
//
// A tank result line of a motion that set out to TRANSLATE ends in ` xt <signed inches>"`: the last
// aggregated estimate's distance off the target's line (the line through the target point along
// the target heading), + to the LEFT of the target heading. Nothing else on any line moves; the
// X-drive and H-drive lines never gain it (except an H-drive with NO strafe authority — XT4 — which
// is a tank). Every number here was measured on the built change
// first (TANK1-PROGRESS.md, Part B) and is graded on ground truth where the case is about truth.

namespace {

/// Every result line — a [MOT] Info message — in `s`, in order (Warn lines are not result lines).
std::vector<std::string> resultLines(const FakeTelemetrySink& s) {
    std::vector<std::string> out;
    for (int i = 0; i < s.size(); ++i) {
        if (s.at(i).level == LogLevel::Info && s.at(i).subsystem == "MOT") {
            out.push_back(s.at(i).message);
        }
    }
    return out;
}

bool hasXt(const std::string& line) { return line.find(" xt ") != std::string::npos; }

bool endsWith(const std::string& s, const std::string& tail) {
    return s.size() >= tail.size() && s.compare(s.size() - tail.size(), tail.size(), tail) == 0;
}

/// What the formatter must render for `v`: ` xt `, the 6-wide 2-decimal number, the inch mark.
std::string xtToken(double v) {
    shulib::diag::lineformat::Line l;
    l.appendLiteral(" xt ");
    shulib::diag::lineformat::appendNum(l, v, 6, 2);
    l.appendLiteral("\"");
    return std::string{l.view()};
}

double dist(const Pose2d& a, const Pose2d& b) {
    return std::hypot(a.x().value() - b.x().value(), a.y().value() - b.y().value());
}

/// The estimate on command `id`'s FIRST live (Running) record — where the motion started.
Pose2d firstLiveMeasured(const FakeTelemetrySink& s, std::uint32_t id) {
    for (int i = 0; i < s.recordCount(); ++i) {
        const auto& r = s.recordAt(i);
        if (r.activeCommandId == id
            && r.activeCommandState == static_cast<std::uint8_t>(motion::MotionState::Running)) {
            return r.measuredPose;
        }
    }
    FAIL("no live record for command " << id);
    return Pose2d{};
}

/// Command `id`'s LAST record (its exit record, for a motion that exited).
shulib::diag::DebugRecord lastRecordOf(const FakeTelemetrySink& s, std::uint32_t id) {
    int at = -1;
    for (int i = 0; i < s.recordCount(); ++i) {
        at = s.recordAt(i).activeCommandId == id ? i : at;
    }
    REQUIRE(at >= 0);
    return s.recordAt(at);
}

}  // namespace

// Bug caught: "did it translate" answered from the LATEST target (M29 — the draft: a turn a weak
// side drags reads as a translation), from a record that is not live (a capture-at-live motion's
// waiting record carries a not-yet-real target), or never forgotten between motions; the
// cross-track error's sign flipped (M27) or taken from the wrong line. Hand-fed, so every number
// is exact.
TEST_CASE("XT1 the stats sink: 'did it translate' is the FIRST live target's question; xt is "
          "signed + left of the target heading, off the LATEST target's line; beginMotion() "
          "forgets") {
    using motion::MotionState;
    FakeTelemetrySink inner;
    motion::MotionStatsSink stats{inner};
    auto rec = [](std::uint32_t id, MotionState st, const Pose2d& measured, const Pose2d& target) {
        shulib::diag::DebugRecord r;
        r.activeCommandId = id;
        r.activeCommandState = static_cast<std::uint8_t>(st);
        r.measuredPose = measured;
        r.targetPose = target;
        return r;
    };

    // (a) A translation along 0 degrees ending 0.25 in LEFT of its line: xt +0.25 exactly. The
    // waiting record's far target and an idle (id 0) record do not count.
    stats.beginMotion();
    stats.emit(rec(1, MotionState::WaitingForEstimate, P(0, 0, 0), P(500, 0, 0)));
    stats.emit(rec(0, MotionState::Running, P(0, 0, 0), P(-300, 0, 0)));
    CHECK_FALSE(stats.hadDirection());
    CHECK(stats.crossTrack().value() == 0.0);
    stats.emit(rec(1, MotionState::Running, P(0, 0, 0), P(10, 0, 0)));
    stats.emit(rec(1, MotionState::Running, P(6, 1, 0), P(10, 0, 0)));
    stats.emit(rec(1, MotionState::Settled, P(10, 0.25, 0), P(10, 0, 0)));
    REQUIRE(stats.hasData());
    CHECK(stats.hadDirection());
    CHECK(stats.crossTrack().value() == 0.25);
    CHECK(inner.recordCount() == 5);  // still a pure observer

    // (b) beginMotion() forgets all of it (!hasData() => every aggregate reads its default).
    stats.beginMotion();
    CHECK_FALSE(stats.hasData());
    CHECK_FALSE(stats.hadDirection());
    CHECK(stats.crossTrack().value() == 0.0);

    // (c) A turn a weak side drags: TurnTo publishes its target AT the robot on every record, so
    // the LATEST target walks 0.86 in from the start while the FIRST is the start itself — no
    // direction (re-check item 3: the draft's latest-target test called this a translation).
    stats.emit(rec(2, MotionState::Running, P(0, 0, 0), P(0, 0, 90)));
    stats.emit(rec(2, MotionState::Running, P(0.3, -0.4, 40), P(0.3, -0.4, 90)));
    stats.emit(rec(2, MotionState::Settled, P(0.5, -0.7, 89), P(0.5, -0.7, 90)));
    REQUIRE(dist(stats.targetPose(), P(0, 0, 0)) > 0.8);  // the latest target DID move
    CHECK_FALSE(stats.hadDirection());

    // (d) A capture-at-live motion (a hold of "here"): its waiting record carries the default
    // target (the origin) while the robot is at (5, 5); its first LIVE target is (5, 5).
    stats.beginMotion();
    stats.emit(rec(3, MotionState::WaitingForEstimate, P(5, 5, 0), P(0, 0, 0)));
    stats.emit(rec(3, MotionState::Running, P(5, 5, 0), P(5, 5, 0)));
    stats.emit(rec(3, MotionState::Settled, P(5.02, 5, 0), P(5, 5, 0)));
    CHECK_FALSE(stats.hadDirection());

    // (e) The sign in a rotated frame: target heading 90 degrees (+y), whose LEFT is -x.
    for (const double endX : {-2.0, 2.0}) {
        CAPTURE(endX);
        stats.beginMotion();
        stats.emit(rec(4, MotionState::Running, P(0, 0, 90), P(0, 10, 90)));
        stats.emit(rec(4, MotionState::TimedOut, P(endX, 10, 90), P(0, 10, 90)));
        CHECK(stats.hadDirection());
        CHECK(stats.crossTrack().value() == -endX);
    }

    // (f) The line is the LATEST target's — the one targetPose() serves and the result line's
    // reader compares final() against — not the first's (a Tier-3 motion that moves its target).
    stats.beginMotion();
    stats.emit(rec(5, MotionState::Running, P(0, 0, 0), P(10, 0, 0)));
    stats.emit(rec(5, MotionState::Settled, P(20, 4, 0), P(20, 5, 0)));
    CHECK(stats.crossTrack().value() == -1.0);  // off y = 5 by -1, not off y = 0 by +4

    // (g) The boundary is kHoldEpsilonIn (0.1 in), inclusive — the one overshoot() uses.
    for (const double away : {0.1, 0.0999}) {
        CAPTURE(away);
        stats.beginMotion();
        stats.emit(rec(6, MotionState::Running, P(0, 0, 0), P(away, 0, 0)));
        CHECK(stats.hadDirection() == (away >= 0.1));
    }
}

// Bug caught: the column missing from a tank moveTo, its value not the final estimate's line error
// (the wrong pose, the wrong target, the sign flipped — M27), or the line's earlier bytes moved.
// Forward and backing up, both weak sides, a rotated line, a leg that arrives and legs that cannot
// (12 in off to either side: the steering's 15 degrees cannot remove 12 in over 40 — measured xt
// -/+5.10 — which is exactly the leg the column exists for). On the clean plant the estimate IS the
// truth, so the reported number must equal the true one.
TEST_CASE("XT2 a tank moveTo's result line ends in xt: the final estimate's signed distance off "
          "the target's line — exact, true, + left of the target heading") {
    struct Case {
        const char* name;
        Pose2d start;
        Pose2d target;
        int wheel;
        double scale;
        ExitReason exit;
        double measured;  // measured on the built change (TANK1-PROGRESS.md, Part B)
    };
    const Case cases[] = {
        {"40 in, even", P(0, 0, 0), P(40, 0, 0), 1, 1.00, ExitReason::Settled, 0.0},
        {"40 in, right side 9 % weak", P(0, 0, 0), P(40, 0, 0), 1, 0.91, ExitReason::Settled, 0.017},
        {"40 in, left side 9 % weak", P(0, 0, 0), P(40, 0, 0), 0, 0.91, ExitReason::Settled, -0.017},
        {"12 in to the left: ends RIGHT of its line", P(0, 0, 0), P(40, 12, 0), 1, 1.00,
         ExitReason::TimedOut, -5.096},
        {"12 in to the right: ends LEFT of its line", P(0, 0, 0), P(40, -12, 0), 1, 1.00,
         ExitReason::TimedOut, 5.096},
        {"heading 90: ends on the -x side of x = 12, its LEFT", P(0, 0, 90), P(12, 40, 90), 1, 1.00,
         ExitReason::TimedOut, 5.096},
        {"backing up 40 in, 12 in aside: ends right of heading 0", P(0, 0, 0), P(-40, 12, 0), 1,
         1.00, ExitReason::TimedOut, -5.096},
    };
    for (const Case& c : cases) {
        const std::string name{c.name};  // a string: CAPTURE of a const char* prints the pointer
        CAPTURE(name);
        ChassisTank r{TankWorld{c.wheel, c.scale}, chassisConfig(), c.start};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        REQUIRE(r.chassis.moveTo(c.target) == c.exit);
        const motion::CompletedMotion& m = r.chassis.lastCompleted();
        REQUIRE(m.hasPathData);
        REQUIRE(m.hasCrossTrack);
        // Exactly the line error of the pose final() prints, and of the motion's last record.
        CHECK(m.crossTrack.value() == tankLineError(m.finalPose, m.targetPose).cross);
        const shulib::diag::DebugRecord last = lastRecordOf(r.sink, m.id);
        CHECK(m.crossTrack.value() == tankLineError(last.measuredPose, last.targetPose).cross);
        // True (clean sensors: measured |reported - true| <= 2.4e-13).
        CHECK(std::abs(m.crossTrack.value() - tankLineError(r.rig.h.truePose(), c.target).cross)
              <= 1e-9);
        CHECK(std::abs(m.crossTrack.value() - c.measured) <= 0.01);
        // The line: the §18.3 bytes, ending in the duration's "s", then the column, once, last.
        const std::vector<std::string> lines = resultLines(r.sink);
        REQUIRE(lines.size() == 1);
        CHECK(endsWith(lines[0], "s" + xtToken(m.crossTrack.value())));
        CHECK(lines[0].find(" xt ") == lines[0].rfind(" xt "));
    }
}

// Bug caught: a motion that did not set out to translate gaining the column — a turn a weak side
// drags 0.81 in sideways (measured; M29, the draft's latest-target test, printed `xt 0.00"` on
// it), a brake whose target coasts 6.6 in with the robot (plant motor lag), a hold of "here" — or
// a translation that is not a moveTo losing it: a strafeTo (a tank cannot strafe — its final
// distance off its line IS its failure) and a hold of a spot away from the robot.
TEST_CASE("XT3 no xt on a tank turn (also one a weak side drags), a coasting brake or a hold of "
          "here; a strafeTo and a hold of a spot away carry it") {
    for (const double scale : {1.0, 0.91}) {
        CAPTURE(scale);
        ChassisTank r{TankWorld{1, scale}};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        REQUIRE(r.chassis.turnTo(Angle::degrees(90.0)) == ExitReason::Settled);
        const motion::CompletedMotion& m = r.chassis.lastCompleted();
        CHECK(m.hasPathData);
        CHECK_FALSE(m.hasCrossTrack);
        CHECK(m.crossTrack.value() == 0.0);
        const std::vector<std::string> lines = resultLines(r.sink);
        REQUIRE(lines.size() == 1);
        CHECK_FALSE(hasXt(lines[0]));
        CHECK(endsWith(lines[0], "s"));
        if (scale < 1.0) {
            // Non-vacuity: the turn's latest target (it follows the robot) ended 0.81 in (measured)
            // from where the turn began — a latest-target test WOULD call it a translation.
            CHECK(dist(m.targetPose, firstLiveMeasured(r.sink, m.id)) > 0.5);
        }
    }
    {
        // A brake on a robot still coasting (plant motor lag kA 0.02, about 0.12 s): a moveTo is
        // pre-empted 0.5 s in, at speed.
        ChassisTank r{TankWorld{}, chassisConfig(), Pose2d{}, 0.02};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        motion::MotionScheduler& sched = r.chassis.scheduler();
        MoveToPose mv{sched.deps(), P(60, 0, 0), motionConfig(), 8.0};
        sched.async(mv);
        (void)sched.waitUntil([] { return false; }, 0.5);
        REQUIRE(r.chassis.brake() == ExitReason::Settled);
        const motion::CompletedMotion& m = r.chassis.lastCompleted();
        CHECK(std::string{m.name} == "DriveBrake");
        CHECK_FALSE(m.hasCrossTrack);
        CHECK(dist(m.targetPose, firstLiveMeasured(r.sink, m.id)) > 3.0);  // measured 6.61
        const std::vector<std::string> lines = resultLines(r.sink);
        REQUIRE(lines.size() == 2);
        CHECK(lines[0].rfind("MoveToPose#1 ✗SUPERSEDED", 0) == 0);
        CHECK(hasXt(lines[0]));  // the pre-empted moveTo HAD set out to translate
        CHECK(lines[1].rfind("DriveBrake#2 ✓SETTLED", 0) == 0);
        CHECK_FALSE(hasXt(lines[1]));
    }
    {
        ChassisTank r{TankWorld{}};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        REQUIRE(r.chassis.hold(Time{0.5}) == ExitReason::Settled);
        CHECK_FALSE(r.chassis.lastCompleted().hasCrossTrack);
        motion::MotionScheduler& sched = r.chassis.scheduler();
        motion::HoldPose there{sched.deps(), P(3, 1, 0), 1.0, motionConfig()};
        sched.async(there);
        (void)sched.waitUntilSettled();
        const motion::CompletedMotion h = sched.lastCompleted();  // a COPY: the next verb reuses the slot
        CHECK(h.hasCrossTrack);
        CHECK(std::abs(h.crossTrack.value() - -1.0) <= 0.01);  // measured -1.000: 1 in aside, kept
        (void)r.chassis.strafeTo(Length{30.0}, Length{6.0});
        const motion::CompletedMotion s = r.chassis.lastCompleted();
        CHECK(s.exit == ExitReason::TimedOut);
        CHECK(s.hasCrossTrack);
        CHECK(std::abs(s.crossTrack.value() - -6.0) <= 0.01);  // measured -6.000: all 6 in of it
        const std::vector<std::string> lines = resultLines(r.sink);
        REQUIRE(lines.size() == 3);
        CHECK(lines[0].rfind("HoldPose#1 ", 0) == 0);
        CHECK_FALSE(hasXt(lines[0]));
        CHECK(lines[1].rfind("HoldPose#2 ", 0) == 0);
        CHECK(endsWith(lines[1], xtToken(h.crossTrack.value())));
        CHECK(lines[2].rfind("StrafeTo#3 ", 0) == 0);
        CHECK(endsWith(lines[2], xtToken(s.crossTrack.value())));
    }
}

// Bug caught: the column reaching a drivetrain that CAN strafe (M25 — the authority test dropped,
// or loosened: any threshold above 0 lets some real H-drive in). P10 (b) cannot see this: it
// compares TANK1 on against off, and both would carry the column. Every verb on the X-drive and the
// H-bot, clean and under composed hostility; then H-drives whose strafe wheel is 1 %, 1e-9 and the
// smallest double — none — and one declared dead (authority exactly 0): a tank, and it does.
TEST_CASE("XT4 no X-drive or H-drive result line carries xt (every verb, clean and hostile); an "
          "H-drive with no strafe authority at all is a tank and does") {
    const auto xk = shulib::kinematics::xDrive(Length{7.0});
    const auto hk = hBotKinematics();
    struct Drive {
        const char* name;
        const shulib::kinematics::IKinematics* kin;
    };
    for (const Drive& d : {Drive{"X", &xk}, Drive{"H", &hk}}) {
        for (const bool hostile : {false, true}) {
            CAPTURE(d.name);
            CAPTURE(hostile);
            FakeTelemetrySink sink;
            Trace t;
            holonomicScript(*d.kin, hostile, chassisConfig(), sink, t);
            const std::vector<std::string> lines = resultLines(sink);
            REQUIRE(lines.size() == 9);  // every verb's boundary reported
            int translations = 0;
            for (const std::string& line : lines) {
                CAPTURE(line);
                CHECK_FALSE(hasXt(line));
                CHECK(endsWith(line, "s"));
                translations += (line.rfind("MoveToPose#", 0) == 0
                                 || line.rfind("StrafeTo#", 0) == 0)
                                    ? 1
                                    : 0;
            }
            CHECK(translations == 6);  // non-vacuity: six lines a tank would give the column to
        }
    }
    for (const double derate : {0.0, 0.01, 1e-9, std::numeric_limits<double>::denorm_min()}) {
        CAPTURE(derate);
        const auto hd = shulib::kinematics::hDrive({.trackWidth = Length{11.0},
                                                    .strafeWheelOffset = Length{-4.0},
                                                    .strafeTractionDerate = derate});
        FakeTelemetrySink sink;
        MotionRig rig{hd, plantConfig(), &sink};
        TracePacer pacer{rig.h};
        chassis::Chassis c{rig.deps, pacer, chassisConfig()};
        motion::RunReporter rep{sink, c.scheduler()};
        (void)c.moveTo(P(40, 2, 0));
        const motion::CompletedMotion& m = c.lastCompleted();
        CHECK(m.hasCrossTrack == (derate == 0.0));
        if (derate != 0.0) {
            // "Valid iff hasCrossTrack; 0 otherwise" — although this leg DID end off its line.
            CHECK(m.crossTrack.value() == 0.0);
            CHECK(tankLineError(m.finalPose, m.targetPose).cross != 0.0);
        }
        const std::vector<std::string> lines = resultLines(sink);
        REQUIRE(lines.size() == 1);
        CHECK(hasXt(lines[0]) == (derate == 0.0));
    }
}

// Bug caught: a column with nothing behind it — a NullSink run (no records: the competition
// posture with diagnostics off) printing `xt 0.00"` beside `over n/a` — and the column following
// the steering's off switch: maxSteer = 0 restores the pre-TANK1 tank's motion bit for bit (P1),
// and its result line keeps the column (the changelog's one exception), which then reads the
// drift the steering exists to remove — here, all 2.86 in of it.
TEST_CASE("XT5 no record stream, no column; with maxSteer = 0 the column stays, and shows the "
          "drift") {
    {
        const TankKinematics kin{Length{12.0}};
        TankWorld world{1, 0.91};
        MotionRig rig{kin, plantConfig(), nullptr, &world};  // the harness's NullSink
        TracePacer pacer{rig.h};
        chassis::Chassis c{rig.deps, pacer, chassisConfig()};
        FakeTelemetrySink out;
        motion::RunReporter rep{out, c.scheduler()};
        REQUIRE_FALSE(c.scheduler().deps().ctx->telemetry().wantsRecord());
        REQUIRE(c.moveTo(P(40, 0, 0)) == ExitReason::Settled);
        const motion::CompletedMotion& m = c.lastCompleted();
        CHECK_FALSE(m.hasPathData);
        CHECK_FALSE(m.hasCrossTrack);
        CHECK(m.crossTrack.value() == 0.0);
        const std::vector<std::string> lines = resultLines(out);
        REQUIRE(lines.size() == 1);
        CHECK(lines[0].find(" over   n/a  drift  n/a ") != std::string::npos);
        CHECK_FALSE(hasXt(lines[0]));
        CHECK(endsWith(lines[0], "s"));
    }
    {
        chassis::ChassisConfig off = chassisConfig();
        off.motion.tankSteer.maxSteer = 0.0;
        ChassisTank r{TankWorld{1, 0.91}, off};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        REQUIRE(r.chassis.moveTo(P(40, 0, 0)) == ExitReason::TimedOut);
        const motion::CompletedMotion& m = r.chassis.lastCompleted();
        REQUIRE(m.hasCrossTrack);
        CHECK(std::abs(m.crossTrack.value() - -2.86) <= 0.01);  // measured -2.861 (P1's END y)
        CHECK(m.crossTrack.value() == m.finalPose.y().value());  // the line is y = 0 along 0 degrees
        const std::vector<std::string> lines = resultLines(r.sink);
        REQUIRE(lines.size() == 1);
        CHECK(endsWith(lines[0], "s xt  -2.86\""));
    }
    // Guide 12's reading of the sign: a weaker RIGHT side leaves a leg to the RIGHT of its line
    // (negative xt) whether it drove forward or backed up — measured -2.861 on the backing leg and
    // on L9's reverseTo shape too (a latest-heading or travel-direction sign would flip one).
    struct Shape {
        const char* name;
        Pose2d start;
        Pose2d target;
    };
    for (const Shape& sh : {Shape{"backing 40 in", P(0, 0, 0), P(-40, 0, 0)},
                            Shape{"reverseTo shape", P(0, 0, 180), P(40, 0, 180)}}) {
        const std::string name{sh.name};  // a string: CAPTURE of a const char* prints the pointer
        CAPTURE(name);
        chassis::ChassisConfig off = chassisConfig();
        off.motion.tankSteer.maxSteer = 0.0;
        ChassisTank r{TankWorld{1, 0.91}, off, sh.start};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        REQUIRE(r.chassis.moveTo(sh.target) == ExitReason::TimedOut);
        const motion::CompletedMotion& m = r.chassis.lastCompleted();
        REQUIRE(m.hasCrossTrack);
        CHECK(std::abs(m.crossTrack.value() - -2.86) <= 0.01);
    }
}

// Bug caught: the column on the wrong lines of the path students write — a Routine's moveTo, turnTo,
// face and driveTo (the acceptance run's shape) with one side 9 % weak: the two drives carry it,
// each within the 0.5 in a settled leg's translation tolerance allows (measured 0.08), and neither
// turn does, though the weak side drags the face() turn ~0.8 in (its latest target moves).
TEST_CASE("XT6 the auton shape through Routine, 9 % weak either side: xt on the two drives, on "
          "neither turn") {
    for (const int wheel : {1, 0}) {
        CAPTURE(wheel);
        ChassisTank r{TankWorld{wheel, 0.91}};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        chassis::Routine rt{r.chassis, "acceptance-run"};
        rt.moveTo(P(24, 0, 0))
            .turnTo(Angle::degrees(90.0))
            .face(Length{0.0}, Length{0.0})
            .driveTo(Length{0.0}, Length{0.0});
        REQUIRE(rt.result().ok);
        const std::vector<std::string> lines = resultLines(r.sink);
        REQUIRE(lines.size() == 4);
        CHECK(lines[0].rfind("MoveToPose#1 ✓SETTLED", 0) == 0);
        CHECK(hasXt(lines[0]));
        CHECK(lines[1].rfind("TurnTo#2 ✓SETTLED", 0) == 0);
        CHECK_FALSE(hasXt(lines[1]));
        CHECK(lines[2].rfind("TurnTo#3 ✓SETTLED", 0) == 0);
        CHECK_FALSE(hasXt(lines[2]));
        CHECK(lines[3].rfind("MoveToPose#4 ✓SETTLED", 0) == 0);
        const motion::CompletedMotion& last = r.chassis.lastCompleted();
        REQUIRE(last.hasCrossTrack);
        CHECK(std::abs(last.crossTrack.value()) <= 0.5);
        CHECK(endsWith(lines[3], xtToken(last.crossTrack.value())));
    }
}

// Bug caught: the column dropped on the outcomes a student debugs hardest — a leg the user
// cancelled, or one the fault policy aborted (ODO_STUCK against a wall) — by a rule or a formatter
// that keeps it only for exits that ran to the end (the Part B review's mutants N1, N10 and N11
// survived the suite before this case); and the "none yet" record (Chassis::lastCompleted()
// before any motion ran) claiming a cross-track number (N19). The leg starts 5 in right of its
// line, so the column has a real value to carry when the leg is stopped 0.7 s in.
TEST_CASE("XT7 every outcome carries xt: a user cancel and a fault abort mid-leg; the 'none yet' "
          "record claims none") {
    CHECK_FALSE(motion::CompletedMotion{}.hasCrossTrack);
    CHECK(motion::CompletedMotion{}.crossTrack.value() == 0.0);
    struct Stop {
        const char* name;
        bool fault;
        const char* lineStart;
        double measured;  // measured on the built change (TANK1-PROGRESS.md, Part B review)
    };
    for (const Stop& st : {Stop{"user cancel", false, "MoveToPose#1 ✗CANCELLED final(", -0.4772},
                           Stop{"fault abort", true, "MoveToPose#1 ✗FAULT_ABORT=ODO_STUCK final(",
                                -0.4536}}) {
        const std::string name{st.name};  // a string: CAPTURE of a const char* prints the pointer
        CAPTURE(name);
        ChassisTank r{TankWorld{}};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        const motion::CompletedMotion none = r.chassis.lastCompleted();  // before any motion ran
        CHECK(none.exit == ExitReason::Running);
        CHECK_FALSE(none.hasCrossTrack);
        CHECK(none.crossTrack.value() == 0.0);
        motion::MotionScheduler& sched = r.chassis.scheduler();
        MoveToPose mv{sched.deps(), P(40, 5, 0), motionConfig(), 8.0};
        sched.async(mv);
        (void)sched.waitUntil([] { return false; }, 0.7);
        if (st.fault) {
            r.rig.latch.raise(shulib::diag::FaultCode::OdoStuck, "TST", "XT7");
        } else {
            sched.cancel();
        }
        (void)sched.waitUntilSettled();
        const motion::CompletedMotion m = sched.lastCompleted();
        CHECK(m.exit == ExitReason::Cancelled);
        CHECK_FALSE(m.preempted);  // a user cancel / an abort, NOT a pre-emption (XT3 has that)
        CHECK(m.abortFault == (st.fault ? shulib::diag::FaultCode::OdoStuck
                                        : shulib::diag::FaultCode::None));
        REQUIRE(m.hasPathData);
        REQUIRE(m.hasCrossTrack);
        CHECK(m.crossTrack.value() == tankLineError(m.finalPose, m.targetPose).cross);
        CHECK(std::abs(m.crossTrack.value() - st.measured) <= 0.01);
        const std::vector<std::string> lines = resultLines(r.sink);
        REQUIRE(lines.size() == 1);
        CAPTURE(lines[0]);
        CHECK(lines[0].rfind(st.lineStart, 0) == 0);
        CHECK(endsWith(lines[0], "s" + xtToken(m.crossTrack.value())));
    }
}

// Bug caught: guide 12's reading of the sign going false with the steering ON — the case its
// sentence is about (a leg steered to the end, TSTR lit). XT5 pins the sign with the steering off;
// this pins what the guide tells a student to conclude from a steered leg's xt:
//   (a) on a leg that started ON its line, a TIMEOUT with |xt| >= 1 in names the weak side (right
//       weak => negative), forward, backing, the reverseTo shape and at heading 90, with and
//       without motor lag — a grid, so a steering change that overshoots the line by an inch, or a
//       sign taken off the travel direction (B13), turns it red. Measured over a far larger sweep
//       first (TANK1-PROGRESS.md, Part B review, finding 1): the largest |xt| any timed-out leg
//       ended with on its STRONG side was 0.65 in, at 0.36 s of plant lag;
//   (b) the guide's example that a smaller xt names no side: with plant lag kA 0.02 (about
//       0.12 s) a 60 in leg with its RIGHT side 25 % weak, steered to the end, stops 0.42 in to
//       the LEFT (if this goes red because the steering got better, the guide's sentence moves);
//   (c) a leg that started BESIDE its line (the target 12 in to the right, so the robot starts
//       left of the line) with its right side weak ends LEFT of the line — its start side.
TEST_CASE("XT8 guide 12's sign rule with the steering on: an inch or more names the weak side on a "
          "leg that started on its line; a smaller xt names no side; a leg started beside its "
          "line can end on its start side") {
    struct Shape {
        Pose2d start;
        Pose2d target;
    };
    const Shape shapes[] = {{P(0, 0, 0), P(24, 0, 0)},     {P(0, 0, 0), P(40, 0, 0)},
                            {P(0, 0, 0), P(60, 0, 0)},     {P(0, 0, 0), P(-40, 0, 0)},
                            {P(0, 0, 180), P(40, 0, 180)}, {P(0, 0, 90), P(0, 40, 90)}};
    int timedOut = 0;
    int named = 0;  // timed out with |xt| >= 1 (each must name the weak side)
    for (const double kA : {0.0, 0.02, 0.03}) {
        for (const int wheel : {1, 0}) {
            for (const double scale : {0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90}) {
                for (const Shape& sh : shapes) {
                    ChassisTank r{TankWorld{wheel, scale}, chassisConfig(), sh.start, kA};
                    if (r.chassis.moveTo(sh.target) != ExitReason::TimedOut) {
                        continue;
                    }
                    ++timedOut;
                    const motion::CompletedMotion& m = r.chassis.lastCompleted();
                    REQUIRE(m.hasCrossTrack);
                    const double xt = m.crossTrack.value();
                    if (std::abs(xt) < 1.0) {
                        continue;
                    }
                    ++named;
                    CAPTURE(kA);
                    CAPTURE(wheel);
                    CAPTURE(scale);
                    const double tx = sh.target.x().value();
                    const double ty = sh.target.y().value();
                    CAPTURE(tx);
                    CAPTURE(ty);
                    CAPTURE(xt);
                    CHECK((wheel == 1 ? xt < 0.0 : xt > 0.0));
                }
            }
        }
    }
    MESSAGE("XT8 (a): " << timedOut << " legs timed out, " << named << " with |xt| >= 1 in");
    CHECK(named >= 100);  // non-vacuity (measured: see the log)

    {
        ChassisTank r{TankWorld{1, 0.75}, chassisConfig(), Pose2d{}, 0.02};
        motion::RunReporter rep{r.sink, r.chassis.scheduler()};
        REQUIRE(r.chassis.moveTo(P(60, 0, 0)) == ExitReason::TimedOut);
        const motion::CompletedMotion& m = r.chassis.lastCompleted();
        REQUIRE(m.hasCrossTrack);
        CHECK(m.crossTrack.value() > 0.0);  // LEFT of its line, though the RIGHT side is weak
        CHECK(std::abs(m.crossTrack.value() - 0.415) <= 0.01);  // measured +0.4154
        bool lastLiveTstr = false;  // the leg's LAST live (Running) record: steered to the end
        for (int i = 0; i < r.sink.recordCount(); ++i) {
            const auto& rec = r.sink.recordAt(i);
            if (rec.activeCommandId == m.id
                && rec.activeCommandState
                       == static_cast<std::uint8_t>(motion::MotionState::Running)) {
                lastLiveTstr = rec.tankSteerActive;
            }
        }
        CHECK(lastLiveTstr);
        const std::vector<std::string> lines = resultLines(r.sink);
        REQUIRE(lines.size() == 1);
        CHECK(endsWith(lines[0], "s xt   0.42\""));  // the bytes guide 12 quotes
    }
    {
        ChassisTank r{TankWorld{1, 0.91}};
        REQUIRE(r.chassis.moveTo(P(40, -12, 0)) == ExitReason::TimedOut);
        const motion::CompletedMotion& m = r.chassis.lastCompleted();
        REQUIRE(m.hasCrossTrack);
        CHECK(m.crossTrack.value() > 1.0);  // measured +4.518: LEFT, where it started
    }
}
