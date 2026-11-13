// CURVE1-A — the curve follower closed-loop on the A2 host test plant (motion/curve_follow.hpp; the
// pure path is curve_path_test.cpp, the facade and the Routine routine_curve_test.cpp).
//
// Every verdict is graded on GROUND TRUTH (the plant's true pose), never on the estimate the motion
// read. The weak side scales ONE side's volts (TANK1's TankWorld semantics — motion_tank_steer_test.cpp
// keeps that struct file-local, and CV16 forbids editing that file, so the same few lines are
// re-declared here); motor lag is the PLANT's kA (tau = kA / kV: 0.12 s and 0.18 s). Every bound
// is the number measured on the built code (CURVE1-PROGRESS.md) plus a margin, quoted beside it.
// NOTHING here has run on a robot.
//
//   CV2   a straight curve is a moveTo; the plan's ONE Info line  [M3 the look-ahead's sign; R15
//                                                                 the plan line dropped]
//   CV3   off its start heading: 30/60/90 deg drive; 91/150 deg and the Invalid shapes are refused at
//         the first live tick, the robot never moves             [M4 the start rule removed]
//   CV4   hairpins: 18 and 9 in settle; the 6 in warns once, never trips a guard
//                                                                [M6 the bend speed limit dropped]
//   CV5   a DEAD side stops the curve (at 0.8 s, and EARLY: 0.1-0.5 s, either side); inside the
//         end zone only the budget does                         [M7, M8 the guards removed]
//   CV6   no false alarm on the 17 drivable routes x 3 weak sides x 3 lags; the end is STEERED; the
//         counts pinned; a short shove past 60 deg recovers  [M9, M10, M10b]; TANK1 switched off
//         (maxSteer 0) and a window narrower than the clean hand-off's (3 deg): no Warn inside a
//         curve, no yaw correction when off          [T1 no off-switch shortcut, T2 the observer
//                                                     ungated, T3 the clean hand-off ungated]
//   CV8   the end heading; case 3 (the end of the path outside TANK1's window)  [M12 hand-off 15 deg];
//         on an X-drive case 3 is silent (a holonomic end strafes)   [R13 the Warn on any drive]
//   CV10  reverse: the back leads every tick; refused in front, naming the BACK and faceAway()
//                                                                [M14 the reverse flag ignored; P3]
//   CV11  wait-for-live: built at the FIRST live tick, from the live pose   [M15 built at start];
//         a cancel while waiting invents no error                [R17]
//   CV11b start() fully re-arms: rebuilt from the live pose, observer 0, s from the start
//                                                                [R1, R2, R3]
//   CV12  the near-start drop's ONE Info line, closed loop       [R16 the drop line dropped]
//   CV13  the deadline belongs to the CURVE — even on the tick its end would settle
//                                                  [M17 the end's watchdog shorter; D1 member first]
//   CV14  the observer is TANK1's, bit for bit                  [M18 the clamp removed, curve only]
//   CV14b the observer holds while the IMU is not ready          [R14 its IMU gate removed]
//   CV15  a weak side and lag                                   [M19 the observer off]
//   CV16  NOTHING ELSE CHANGES — hashes measured at the merge base  [M20 any MoveToPose tick edit]
//   CV18  the records and the result line                       [M21 exit record before the stop]
//   CV19  the speed plan, closed loop                           (pure half: curve_path_test.cpp);
//         the FIRST command is one tick of accel, mid-auton      [R11 the ramp removed, R22 kFirstTickDt 0]
//   CV20  other drivetrains: X-drive and H-drive drive like a tank
//   CV23  THE RUN-UP RULE guide 05 states. A RED CV23 MEANS RE-MEASURING THE RULE AND UPDATING
//         docs/guide/05-getting-there.md IN THE SAME COMMIT — NEVER TUNING THE FOLLOWER TO PASS IT.
//         A later default change that moves the rule changes the guide in the same commit.
//                                                                [M27 lookAhead 12, M28 tangent 1/2]

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "curve_unchanged_route.hpp"
#include "motion_test_rig.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/h_drive.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/motion/curve_follow.hpp"
#include "shulib/motion/curve_path.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/run_reporter.hpp"
#include "shulib/motion/tank_steer.hpp"
#include "shulib/sim/degradation.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::CurveFollow;
using shulib::motion::CurveGuard;
using shulib::motion::CurveHandoff;
using shulib::motion::CurvePhase;
using shulib::motion::CurvePoint;
using shulib::motion::MotionConfig;
using shulib::units::Length;
using shulib::units::Time;

namespace {

namespace sim = shulib::sim;
namespace motion = shulib::motion;
namespace units = shulib::units;

constexpr double kPi = Angle::kPi;
constexpr double kDeg = kPi / 180.0;
constexpr double kTick = 0.01;

CurvePoint at(double x, double y, double deg) {
    return CurvePoint{Length{x}, Length{y}, Angle::degrees(deg)};
}
CurvePoint thr(double x, double y) { return CurvePoint{Length{x}, Length{y}}; }
Pose2d P(double x, double y, double deg) { return Pose2d{Length{x}, Length{y}, Angle::degrees(deg)}; }

/// The weak side (TankWorld's semantics: wheel 0 = LEFT, 1 = RIGHT), a side going DEAD from a time,
/// a shove (both sides driven to turn the robot in a window), and the IMU's boot window.
struct World final : sim::DegradationModel {
    int wheel = 1;
    double scale = 1.0;
    double deadFrom = 1e9;   ///< from this clock time the `wheel` side gets 0 V
    double spinFrom = -1.0;  ///< in [spinFrom, spinTo) left -spinVolts, right +spinVolts
    double spinTo = -1.0;
    double spinVolts = 0.0;
    double imuReadyAt = 0.0;  ///< the IMU reports not-ready before this clock time
    bool imuNever = false;    ///< ...or for ever
    double imuOffFrom = -1.0;  ///< ...or in [imuOffFrom, imuOffTo) (a drop-out mid-run)
    double imuOffTo = -1.0;

    units::Voltage effectiveVoltage(int w, units::Voltage c, Time now, sim::Rng&) override {
        if (now.value() >= spinFrom && now.value() < spinTo) {
            return units::Voltage{w == 0 ? -spinVolts : spinVolts};
        }
        if (w == wheel) {
            return units::Voltage{c.value() * (now.value() >= deadFrom ? 0.0 : scale)};
        }
        return c;
    }
    bool imuReady(bool trueReady, Time now) override {
        if (imuNever || now.value() < imuReadyAt
            || (now.value() >= imuOffFrom && now.value() < imuOffTo)) {
            return false;
        }
        return trueReady;
    }
};

/// A route: start pose, points, direction.
struct Route {
    const char* name;
    Pose2d start;
    std::vector<CurvePoint> pts;
    bool reverse = false;
};

/// The brief's 17 DRIVABLE routes (§0.4 less off150, which D3 refuses).
std::vector<Route> drivable() {
    return {
        {"straight48", P(0, 0, 0), {at(48, 0, 0)}},
        {"scurve", P(0, 0, 0), {thr(24, 12), at(48, 24, 0)}},
        {"lanechange", P(0, 0, 0), {at(48, 24, 0)}},
        {"bend90r36", P(0, 0, 0), {at(36, 36, 90)}},
        {"bend90r18", P(0, 0, 0), {at(18, 18, 90)}},
        {"bend90r12", P(0, 0, 0), {at(12, 12, 90)}},
        {"hairpin18", P(0, 0, 0), {at(36, 18, 90), at(0, 36, 180)}},
        {"hairpin9", P(0, 0, 0), {at(24, 9, 90), at(0, 18, 180)}},
        {"hairpin6", P(0, 0, 0), {at(18, 6, 90), at(0, 12, 180)}},
        {"off30", P(0, 0, 30), {at(48, 0, 0)}},
        {"off60", P(0, 0, 60), {at(48, 0, 0)}},
        {"off90", P(0, 0, 90), {at(48, 0, 0)}},
        {"snake", P(0, 0, 0), {thr(36, 18), thr(12, 48), thr(36, 72), at(60, 72, 0)}},
        {"fieldS", P(-48, -48, 90), {thr(-48, -12), thr(-12, 0), thr(24, 12), at(48, 48, 90)}},
        {"revS", P(0, 0, 180), {thr(24, 12), at(48, 24, 180)}, true},
        {"endfree", P(0, 0, 0), {thr(24, 12), thr(48, 24)}},
        {"crossing", P(0, 0, 0), {thr(30, 0), thr(36, 18), thr(24, 30), thr(12, 18), at(12, -24, -90)}},
    };
}
Route routeNamed(const char* n) {
    for (const Route& r : drivable()) {
        if (std::string{r.name} == n) {
            return r;
        }
    }
    throw std::runtime_error("no such route");
}

/// What one closed-loop curve did.
struct Run {
    ExitReason exit = ExitReason::Running;
    int ticks = 0;
    double secs = 0.0;          ///< clock at the exit
    double liveAt = -1.0;       ///< clock at the first tick that left WaitingForEstimate
    Pose2d truth{};             ///< the true pose at the exit
    Pose2d end{};               ///< the curve's end pose
    double travel = 0.0;        ///< total true distance travelled (in)
    double travelAfter = 0.0;   ///< true distance travelled after `afterT` (in)
    double maxOffPath = 0.0;    ///< worst true distance off the path (in), when measured
    double maxHeadingOff = 0.0;  ///< worst |travel - path direction| on a pursuit tick (rad)
    double maxLateral = 0.0;    ///< worst |true y| (in) — CV2's straight line is y = 0
    std::vector<std::string> warns;  ///< every Warn message
    int infos = 0;                   ///< MOT Info lines
    std::vector<std::string> infoLines;  ///< every MOT Info message
    motion::CurveStatus status{};    ///< the final status
    bool motorsZeroAtExit = false;   ///< every drive motor commanded 0 V on the exit tick
    double endHeadErr() const { return std::abs(truth.heading().errorTo(end.heading())); }
    double endPosErr() const { return posErr(truth, end); }
    [[nodiscard]] int countWarns(const char* text) const {
        int n = 0;
        for (const std::string& w : warns) {
            n += w.find(text) != std::string::npos ? 1 : 0;
        }
        return n;
    }
};

/// The rig for one run: a kinematics, a sink, the world, the motion rig.
struct Rig {
    TankKinematics tank{Length{12.0}};
    FakeTelemetrySink sink;
    World world;
    MotionRig rig;
    Rig(const Pose2d& start, World w, double tau = 0.0,
        const shulib::kinematics::IKinematics* kin = nullptr)
        : world{w}, rig{kin != nullptr ? *kin : tank, cfgFor(start, tau), &sink, &world} {}
    static sim::SimHarnessConfig cfgFor(const Pose2d& start, double tau) {
        sim::SimHarnessConfig c = plantConfig();
        c.plant.initialPose = start;
        c.plant.wheelFf.kA = tau * c.plant.wheelFf.kV;  // tau = kA / kV (unmodelled motor lag)
        return c;
    }
};

/// The true distance from `p` to the path, searched within +-12 in of `sHint` every 0.1 in.
double offPath(const motion::CurvePath& path, const Pose2d& p, double sHint) {
    double best = 1e18;
    const double lo = std::max(0.0, sHint - 12.0);
    const double hi = std::min(path.length(), sHint + 12.0);
    for (double s = lo; s <= hi + 1e-9; s += 0.1) {
        const auto c = path.at(s);
        best = std::min(best, std::hypot(c.x - p.x().value(), c.y - p.y().value()));
    }
    return best;
}

/// The newest record a MOTION emitted (activeCommandState != 0): the plant emits its own records
/// into the same sink, one per step.
const shulib::diag::DebugRecord& lastMotionRecord(const FakeTelemetrySink& s) {
    for (int i = s.recordCount() - 1; i >= 0; --i) {
        if (s.recordAt(i).activeCommandState != 0) {
            return s.recordAt(i);
        }
    }
    throw std::runtime_error("no motion record");
}
int motionRecords(const FakeTelemetrySink& s) {
    int n = 0;
    for (int i = 0; i < s.recordCount(); ++i) {
        n += s.recordAt(i).activeCommandState != 0 ? 1 : 0;
    }
    return n;
}

struct RunOpts {
    bool measureOffPath = false;
    double afterT = 1e9;
    int maxTicks = 2000;
    std::function<void(int, CurveFollow&, Rig&)> onTick;  ///< after each tick, before the step
};

/// The controller-first loop (MotionRig::run's shape), recording a Run.
Run drive(Rig& r, CurveFollow& m, const RunOpts& o = {}) {
    Run out;
    m.start();
    Pose2d prev = r.rig.h.truePose();
    for (int i = 0; i < o.maxTicks && out.exit == ExitReason::Running; ++i) {
        r.rig.loc.update();
        out.exit = m.tick();
        ++out.ticks;
        if (out.liveAt < 0.0 && m.state() != motion::MotionState::WaitingForEstimate) {
            out.liveAt = r.rig.h.clock().now().value();
        }
        const motion::CurveStatus& st = m.curveStatus();
        if (st.phase == CurvePhase::Pursuing) {
            out.maxHeadingOff = std::max(out.maxHeadingOff, st.headingOffPath);
        }
        if (o.measureOffPath && m.path().built() && st.phase != CurvePhase::Waiting) {
            out.maxOffPath = std::max(out.maxOffPath, offPath(m.path(), r.rig.h.truePose(), st.s));
        }
        if (o.onTick) {
            o.onTick(i, m, r);
        }
        if (out.exit != ExitReason::Running) {
            out.motorsZeroAtExit = true;
            for (int w = 0; w < r.rig.h.motorCount(); ++w) {
                out.motorsZeroAtExit =
                    out.motorsZeroAtExit && r.rig.h.motor(w).commandedVoltage().value() == 0.0;
            }
            break;
        }
        r.rig.h.plant().step(Time{kTick});
        const Pose2d t = r.rig.h.truePose();
        const double d = posErr(t, prev);
        out.travel += d;
        if (r.rig.h.clock().now().value() > o.afterT) {
            out.travelAfter += d;
        }
        out.maxLateral = std::max(out.maxLateral, std::abs(t.y().value()));
        prev = t;
    }
    out.secs = r.rig.h.clock().now().value();
    out.truth = r.rig.h.truePose();
    out.end = m.endPose();
    out.status = m.curveStatus();
    for (int i = 0; i < r.sink.size(); ++i) {
        if (r.sink.at(i).level == LogLevel::Warn) {
            out.warns.push_back(r.sink.at(i).message);
        }
        if (r.sink.at(i).level == LogLevel::Info && r.sink.at(i).subsystem == "MOT") {
            ++out.infos;
            out.infoLines.push_back(r.sink.at(i).message);
        }
    }
    return out;
}

/// One route, one world, one lag: build, drive, report.
Run runRoute(const Route& route, World w = {}, double tau = 0.0, MotionConfig cfg = motionConfig(),
             double timeout = 6.0, const RunOpts& o = {}) {
    Rig r{route.start, w, tau};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, cfg, timeout, route.reverse};
    return drive(r, m, o);
}

World weak(double scale) {
    World w;
    w.scale = scale;
    return w;
}

}  // namespace

// ═══ CV2 — a straight curve is a moveTo ══════════════════════════════════════════════════
// Bug caught: the look-ahead's sign flipped (M3: y -> -y) — the robot steers AWAY from its path
// and curls off it; or any lateral wobble a straight path should not have.
TEST_CASE("CV2 a one-point straight curve settles on its line like a moveTo") {
    const Run r = runRoute({"straight48", P(0, 0, 0), {at(48, 0, 0)}});
    CHECK(r.exit == ExitReason::Settled);
    CHECK(r.endPosErr() <= 0.5);
    CHECK(r.endHeadErr() <= 1.15 * kDeg);
    CAPTURE(r.maxLateral);
    CHECK(r.maxLateral <= 0.2);
    CHECK(r.status.handoff == CurveHandoff::Clean);
    CHECK(r.warns.empty());
    // ONE Info line when it starts (guide 11 teaches it): the points, the length, "no bend" (R15).
    CHECK(r.infoLines == std::vector<std::string>{"curve: 1 point, 48.0 in, no bend"});
    // The same with one side 9 % weak: the drift a pursuit must steer OUT (a flipped look-ahead
    // steers it further in and curls away). Measured: settles, worst |y| 0.096 in (even: 0).
    const Run w = runRoute({"straight48", P(0, 0, 0), {at(48, 0, 0)}}, weak(0.91));
    CHECK(w.exit == ExitReason::Settled);
    CAPTURE(w.maxLateral);
    CHECK(w.maxLateral <= 0.2);
}

// Bug caught: the plan's Info line dropped or garbled (R15) — guide 11 quotes THIS line, the snake's.
TEST_CASE("CV2 the plan's Info line, word for word as guide 11 quotes it (the snake)") {
    const Run r = runRoute(routeNamed("snake"));
    REQUIRE(r.infoLines.size() == 1);
    CHECK(r.infoLines[0] == "curve: 4 points, 143.4 in, tightest bend 7.4 in radius, bends at <= 21 in/s");
}

// ═══ CV3 — off its start heading; the refusals ═══════════════════════════════════════════
// Bug caught: the start rule removed (M4 — 150 deg drives a loop: the distance travelled is not 0);
// a refusal that moves the robot, takes more than a tick, or says nothing.
TEST_CASE("CV3 starts 30, 60 and 90 deg off the first chord settle, close to the path") {
    for (const char* n : {"off30", "off60", "off90"}) {
        CAPTURE(std::string{n});
        RunOpts o;
        o.measureOffPath = true;
        const Run r = runRoute(routeNamed(n), {}, 0.0, motionConfig(), 6.0, o);
        CHECK(r.exit == ExitReason::Settled);
        CAPTURE(r.maxOffPath);
        CHECK(r.maxOffPath <= 1.5);
        CHECK(r.endPosErr() <= 0.5);
    }
}

TEST_CASE("CV3 a first point 91 or 150 deg off, and every Invalid shape, is refused before moving") {
    struct Case {
        Route route;
        const char* words;
    };
    const std::vector<Case> cases = {
        {{"off91", P(0, 0, 91), {at(48, 0, 0)}}, "turn first: face()"},
        {{"off150", P(0, 0, 150), {at(48, 0, 0)}}, "turn first: face()"},
        {{"outback", P(0, 0, 0), {thr(24, 12), at(0, 0, 180)}}, "point 1 has no direction"},
        {{"loopmid", P(0, 0, 0), {at(24, 0, 180), at(0, 24, 180)}}, "point 1 doubles back"},
        {{"revHead", P(0, 0, 0), {at(36, 0, 180)}}, "point 1 doubles back"},
        {{"revEx180", P(0, 24, 0), {thr(-12, 30), at(-36, 36, 180)}, true}, "point 2 doubles back"},
        {{"atRobot", P(0, 0, 0), {at(0.99, 0, 0), at(48, 0, 0)}}, "point 1 is where the robot"},
        {{"tooClose", P(0, 0, 0), {thr(24, 0), at(24.99, 0, 0)}}, "point 2 is 0.99 in"},
        {{"nothingLeft", P(0, 0, 0), {thr(0.4, 0)}}, "no point left"},
    };
    for (const Case& c : cases) {
        CAPTURE(std::string{c.route.name});
        Rig r{c.route.start, {}};
        CurveFollow m{r.rig.deps, std::span<const CurvePoint>{c.route.pts}, motionConfig(), 6.0,
                      c.route.reverse};
        const Run run = drive(r, m);
        CHECK(run.exit == ExitReason::TimedOut);
        CHECK(run.travel == 0.0);                       // zero distance driven
        CHECK(run.secs == run.liveAt);                  // refused AT the first live tick
        CHECK(run.motorsZeroAtExit);
        REQUIRE(run.warns.size() == 1);                 // ONE Warn
        CHECK(run.warns[0].find(c.words) != std::string::npos);
        CHECK(run.warns[0].find("curve not driven: ") == 0);
        CHECK(r.rig.latch.raiseCount(shulib::diag::FaultCode::MotionTimeout) == 1);
        CHECK(m.curveStatus().phase == CurvePhase::Done);
    }
}

// ═══ CV4 — hairpins ═══════════════════════════════════════════════════════════════════════
// Bug caught: the bend speed limit dropped (M6 — the 9 in hairpin is taken at full speed and swings
// more than 3 in off its path), or a tight bend tripping a guard.
TEST_CASE("CV4 the 18 and 9 in hairpins settle on the path; the 6 in one warns ONCE, never trips") {
    for (const char* n : {"hairpin18", "hairpin9"}) {
        CAPTURE(std::string{n});
        RunOpts o;
        o.measureOffPath = true;
        const Run r = runRoute(routeNamed(n), {}, 0.0, motionConfig(), 6.0, o);
        CHECK(r.exit == ExitReason::Settled);
        CAPTURE(r.maxOffPath);
        CHECK(r.maxOffPath <= 1.5);
        CHECK(r.endHeadErr() <= 1.15 * kDeg);
        CHECK(r.warns.empty());
    }
    RunOpts o;
    o.measureOffPath = true;
    const Run r = runRoute(routeNamed("hairpin6"), {}, 0.0, motionConfig(), 6.0, o);
    CHECK((r.exit == ExitReason::Settled || r.exit == ExitReason::TimedOut));
    CHECK(r.countWarns("tightest bend") == 1);
    CHECK(r.warns.size() == 1);
    CHECK(r.status.guard == CurveGuard::None);
    CAPTURE(r.maxOffPath);
    CHECK(r.maxOffPath <= 3.0);
}

// ═══ CV5 — a dead side ═══════════════════════════════════════════════════════════════════
// Bug caught: either guard removed (M7, M8 — each alone, then both: with both gone the robot drives
// circles for the whole budget, RED on the 2.5 s bound and the 30 in of travel).
TEST_CASE("CV5 the right side dies 0.8 s in: the curve stops within 2.5 s, 'curve lost', <= 30 in") {
    for (const char* n : {"scurve", "bend90r36", "fieldS"}) {
        CAPTURE(std::string{n});
        World w;
        w.deadFrom = 0.8;
        RunOpts o;
        o.afterT = 0.8;
        const Run r = runRoute(routeNamed(n), w, 0.0, motionConfig(), 6.0, o);
        CAPTURE(r.secs);
        CAPTURE(r.travelAfter);
        CHECK(r.exit == ExitReason::TimedOut);
        CHECK(r.secs <= 0.8 + 2.5);
        CHECK(r.status.guard != CurveGuard::None);
        CHECK(r.countWarns("curve lost: ") == 1);
        CHECK(r.motorsZeroAtExit);
        CHECK(r.travelAfter <= 30.0);
    }
}

// Bug caught: the HEADING guard removed (M7 alone — the dead-side runs above are all caught by the
// progress guard too, so they cannot see it). A robot pushed round for half a second (the other
// robot, a field element) keeps "making progress" too little and too late for the 0.75 s progress
// guard; only the heading guard stops it while it is turned off its path.
TEST_CASE("CV5 a robot pushed round off its path for 0.5 s is stopped by the HEADING guard") {
    World w;
    w.spinFrom = 0.8;
    w.spinTo = 1.3;
    w.spinVolts = 12.0;
    const Run r = runRoute(routeNamed("bend90r36"), w);
    CAPTURE(r.secs);
    CHECK(r.exit == ExitReason::TimedOut);
    CHECK(r.status.guard == CurveGuard::Heading);
    CHECK(r.secs <= 1.3);  // stopped while still being pushed
    CHECK(r.countWarns("curve lost: heading ") == 1);
}

TEST_CASE("CV5 one motor of five lost (x0.8 from the start): no guard trips; the gentle routes settle") {
    // A delta from the brief's row ("SETTLED"), logged in CURVE1-PROGRESS.md with the per-route table:
    // measured on the built code with the shipped defaults (2026-11-12, after the review's speed-plan
    // fix), 10 of the 17 settle at x0.8 — the straight, the lane change, the 36 in bend, both wider
    // hairpins, the three off-heading starts, endfree and the crossing; the S, the 18 and 12 in bends,
    // the 6 in hairpin, the snake, the field S and revS end 0.50-0.76 in off and time out — the end
    // moveTo's short-leg limit (L2; the brief's re-check A1 found the same at x0.85 with no lag). The
    // brief's "SETTLED" came from P7, run on the superseded fast defaults. What a lost motor must NEVER
    // do is trip a guard: none of the 17 routes does.
    for (const char* n : {"straight48", "lanechange", "bend90r36"}) {
        CAPTURE(std::string{n});
        CHECK(runRoute(routeNamed(n), weak(0.8)).exit == ExitReason::Settled);
    }
    int settled = 0;
    for (const Route& route : drivable()) {
        CAPTURE(std::string{route.name});
        const Run r = runRoute(route, weak(0.8));
        settled += r.exit == ExitReason::Settled ? 1 : 0;
        CHECK(r.status.guard == CurveGuard::None);
        CHECK(r.endPosErr() <= 0.8);  // measured worst 0.759 (the snake)
    }
    CHECK(settled == 10);
}

// Bug caught: either guard removed (M7, M8) for a side that dies EARLY, on either side — the review of
// 2026-11-12 found an early death on a gentle bend outside the 0.8 s case's bound: the robot keeps
// making progress while it pivots, so only the heading guard stops it. Measured on the built code
// (CURVE1-PROGRESS.md): 84 runs (7 routes x left/right x 0.1/0.3/0.5 s), every one stopped by a guard
// with one "curve lost" Warn, at most 2.68 s after the death (the lane change, left side dead at
// 0.3 s) and at most 9.5 in of travel after it. Every death here is outside the end zone (inside it,
// the case below: only the budget).
TEST_CASE("CV5 a side dying EARLY (0.1-0.5 s), either side, 7 routes: a guard stops it within 2.75 s") {
    double worstAfter = 0.0;
    double worstTravel = 0.0;
    for (const char* n :
         {"scurve", "lanechange", "bend90r36", "fieldS", "snake", "straight48", "off60"}) {
        for (const int side : {0, 1}) {
            for (const double t : {0.1, 0.3, 0.5}) {
                CAPTURE(std::string{n});
                CAPTURE(side);
                CAPTURE(t);
                World w;
                w.wheel = side;
                w.deadFrom = t;
                RunOpts o;
                o.afterT = t;
                const Run r = runRoute(routeNamed(n), w, 0.0, motionConfig(), 6.0, o);
                CHECK(r.exit == ExitReason::TimedOut);
                CHECK(r.status.guard != CurveGuard::None);
                CHECK(r.countWarns("curve lost: ") == 1);
                CHECK(r.motorsZeroAtExit);
                worstAfter = std::max(worstAfter, r.secs - t);
                worstTravel = std::max(worstTravel, r.travelAfter);
            }
        }
    }
    CAPTURE(worstAfter);
    CAPTURE(worstTravel);
    CHECK(worstAfter <= 2.75);
    CHECK(worstTravel <= 10.0);
}

TEST_CASE("CV5 a side dying INSIDE the end zone: no guard there — the budget ends it, travel bounded") {
    struct Case {
        const char* route;
        double deadAt;
    };
    for (const Case& c : {Case{"scurve", 1.9}, Case{"bend90r36", 2.2}}) {
        CAPTURE(std::string{c.route});
        World w;
        w.deadFrom = c.deadAt;
        Rig r{routeNamed(c.route).start, w};
        const Route route = routeNamed(c.route);
        CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0};
        double remainingAtDeath = -1.0;
        RunOpts o;
        o.onTick = [&](int, CurveFollow& f, Rig& rr) {
            if (remainingAtDeath < 0.0 && rr.rig.h.clock().now().value() >= c.deadAt) {
                remainingAtDeath = f.path().length() - f.curveStatus().s;
            }
        };
        const Run run = drive(r, m, o);
        CAPTURE(remainingAtDeath);
        CHECK(remainingAtDeath <= 12.0);  // the death IS inside the end zone
        CHECK(run.exit == ExitReason::TimedOut);
        CHECK(std::abs(run.secs - 6.0) <= kTick + 1e-9);  // the WHOLE budget
        CHECK(run.status.guard == CurveGuard::None);
        CHECK(run.travel <= m.path().length() + 6.0);
        CHECK(run.motorsZeroAtExit);
    }
}

// ═══ CV6 — no false alarm, and the end is steered ════════════════════════════════════════
// Bug caught: the progress guard left ON in the slow end zone (M9 — false alarms); the heading
// guard's time window removed (M10 — the shove below); the last-resort hand-off ungated (M10b — the
// end MoveToPose's "turn first" Warns come back); a change that settles fewer routes.
TEST_CASE("CV6 the 17 routes x x1.00/x0.91/x0.85 x tau 0/0.12/0.18: no guard trip, no 'turn first'") {
    const double weaks[3] = {1.00, 0.91, 0.85};
    const double taus[3] = {0.0, 0.12, 0.18};
    // Measured on the built code (CURVE1-PROGRESS.md): settled per cell, rows = weak side, columns =
    // lag. R-A (the brief's re-check probe): 16 16 14 / 16 16 16 / 13 17 16 — every cell here equal
    // except (x1.00, 0.18 s): 12. The bar: none below R-A minus 1 — EXCEPT that one cell, a LOGGED
    // delta for the coordinator (CURVE1-PROGRESS.md, the review findings): the review's speed-plan fix
    // (2026-11-12) holds the robot to the bend grip into every knot, which the probe's plan did not;
    // there the field S (0.38 in, its settle's RATE bound) and the crossing (0.52 in) end just outside
    // the end moveTo's 0.5 in and never settle, even given 12 s — A1's stated residual (TANK1's
    // short-leg limit, L2; STEP1-A's positionTolerance is the answer). The cell swings 12-14 with
    // +-2 in/s^2 of bendAccel on the code before the fix: chaos at the 0.5 in edge, not a trend.
    const int pinned[3][3] = {{16, 16, 12}, {16, 16, 16}, {13, 17, 16}};
    const int ra[3][3] = {{16, 16, 14}, {16, 16, 16}, {13, 17, 16}};
    const int raSlack[3][3] = {{1, 1, 2}, {1, 1, 1}, {1, 1, 1}};  // 2 = the logged delta above
    double worstMiss = 0.0;
    int outsideWindow = 0;
    int outsideWindowLagFree = 0;
    double worstHeadingOff = 0.0;
    for (int wi = 0; wi < 3; ++wi) {
        for (int ti = 0; ti < 3; ++ti) {
            int settled = 0;
            for (const Route& route : drivable()) {
                CAPTURE(std::string{route.name});
                CAPTURE(weaks[wi]);
                CAPTURE(taus[ti]);
                const Run r = runRoute(route, weak(weaks[wi]), taus[ti]);
                CHECK(r.status.guard == CurveGuard::None);
                for (const std::string& w : r.warns) {
                    CHECK(w.find("turn first") == std::string::npos);
                    CHECK(w.find("not steered") == std::string::npos);
                }
                if (r.status.handoff == CurveHandoff::EndOfPath
                    && r.status.handoffHeadingError > 15.0 * kDeg) {
                    ++outsideWindow;
                    outsideWindowLagFree += taus[ti] == 0.0 ? 1 : 0;
                }
                worstHeadingOff = std::max(worstHeadingOff, r.maxHeadingOff);
                if (r.exit == ExitReason::Settled) {
                    ++settled;
                    // CV8: every settled route ends within the default settle, on the TRUE pose.
                    CHECK(r.endPosErr() <= 0.5);
                    CHECK(r.endHeadErr() <= 1.15 * kDeg);
                } else {
                    // A miss is a near miss, never a wander: measured worst 0.879 in (the 6 in
                    // hairpin, which is warned TightBend), every other under 0.71.
                    CHECK(r.exit == ExitReason::TimedOut);
                    worstMiss = std::max(worstMiss, r.endPosErr());
                }
            }
            CAPTURE(wi);
            CAPTURE(ti);
            CHECK(settled == pinned[wi][ti]);
            CHECK(settled >= ra[wi][ti] - raSlack[wi][ti]);
        }
    }
    // Measured: TWO hand-offs outside TANK1's window in 153, both bend90r12 under lag (0.12 s: 15.5 deg,
    // it still settles; 0.18 s: 21.0 deg) — R-A's count exactly; none lag-free.
    CHECK(outsideWindow == 2);
    CHECK(outsideWindowLagFree == 0);
    CAPTURE(worstMiss);
    CHECK(worstMiss <= 0.95);
    // Measured: the worst heading off the path on a healthy run, 25.9 deg (R-A 24.6) — the guard is 60.
    CAPTURE(worstHeadingOff / kDeg);
    CHECK(worstHeadingOff <= 30.0 * kDeg);
}

// Bug caught: TANK1's documented off switch (tankSteer.maxSteer = 0) leaving the curve waiting for a
// steering window that no longer exists (T1) — every curve not lined up within 0.3 in and 5 deg then
// creeps to the very end of its path and hands off with a false "curve end ... (steers within 0.0)"
// Warn that blames the author — or leaving the curve's own yaw observer on (T2): the team switches
// TANK1 off when it misbehaves on the robot, and the observer (HA-179's yaw-rate sign) is the part
// most likely to. Measured on the built code (CURVE1-PROGRESS.md): 17 routes x (x1.00, x0.91) x
// (tau 0, 0.12 s) = 68 runs, 56 settle (64 with TANK1 on), no curve or member Warn but TightBend's,
// no guard trip, the yaw correction 0 on every tick.
TEST_CASE("CV6 TANK1 switched off (maxSteer = 0): no window to wait for, no false Warn, no yaw correction") {
    MotionConfig cfg = motionConfig();
    cfg.tankSteer.maxSteer = 0.0;
    int settled = 0;
    for (const double ws : {1.00, 0.91}) {
        for (const double tau : {0.0, 0.12}) {
            for (const Route& route : drivable()) {
                CAPTURE(std::string{route.name});
                CAPTURE(ws);
                CAPTURE(tau);
                double worstYaw = 0.0;
                RunOpts o;
                o.onTick = [&](int, CurveFollow& f, Rig&) {
                    worstYaw = std::max(worstYaw, std::abs(f.curveStatus().yawCorrection));
                };
                const Run r = runRoute(route, weak(ws), tau, cfg, 6.0, o);
                settled += r.exit == ExitReason::Settled ? 1 : 0;
                CHECK(r.status.guard == CurveGuard::None);
                CHECK(worstYaw == 0.0);
                CHECK(r.status.handoff != CurveHandoff::EndOfPath);  // the window never held it back
                for (const std::string& w : r.warns) {
                    CHECK(w.rfind("curve: tightest bend ", 0) == 0);  // TightBend's, and nothing else
                }
            }
        }
    }
    CHECK(settled == 56);
}

// Bug caught: the CLEAN hand-off taken outside the end's steering window (T3) when that window
// (tankSteer.maxSteer) is narrower than the clean hand-off's 5 deg — the end MoveToPose then logs
// "tank moveTo not steered ... turn first: face()" INSIDE a curve, blaming the author for a turn the
// path never asked for. Measured (CURVE1-PROGRESS.md): with a 3 deg window, 17 routes x (x1.00,
// x0.91) x (tau 0, 0.12 s), no member Warn; the curve's own "curve end" Warn says the window is 3.0.
TEST_CASE("CV6 a steering window narrower than the clean hand-off's (3 deg): no member Warn in a curve") {
    MotionConfig cfg = motionConfig();
    cfg.tankSteer.maxSteer = 3.0 * kDeg;
    int curveEnd = 0;
    for (const double ws : {1.00, 0.91}) {
        for (const double tau : {0.0, 0.12}) {
            for (const Route& route : drivable()) {
                CAPTURE(std::string{route.name});
                CAPTURE(ws);
                CAPTURE(tau);
                const Run r = runRoute(route, weak(ws), tau, cfg);
                CHECK(r.status.guard == CurveGuard::None);
                CHECK(r.countWarns("not steered") == 0);
                CHECK(r.countWarns("turn first") == 0);
                if (r.status.handoff != CurveHandoff::EndOfPath) {
                    CHECK(r.status.handoffHeadingError <= 3.0 * kDeg + 1e-12);  // steered from the start
                }
                curveEnd += r.countWarns("(steers within 3.0)");
            }
        }
    }
    CHECK(curveEnd > 0);  // the narrow window IS reached for: the test is not vacuous
}

TEST_CASE("CV6 a short shove past 60 deg off the path (under lostTime) does not stop the curve") {
    // Both sides driven to spin the robot for 0.12 s, a third of the way along the 36 in bend: the
    // heading goes past 60 deg off the path for less than lostTime (0.25 s), then the curve recovers.
    World w;
    w.spinFrom = 0.6;
    w.spinTo = 0.72;
    w.spinVolts = 12.0;
    const Run r = runRoute(routeNamed("bend90r36"), w);
    CAPTURE(r.maxHeadingOff / kDeg);
    REQUIRE(r.maxHeadingOff > 60.0 * kDeg);  // the shove DID take it past the guard's angle
    CHECK(r.status.guard == CurveGuard::None);
    CHECK(r.exit == ExitReason::Settled);
}

// ═══ CV8 — the end heading, and the end of the path outside the window ═══════════════════
// Bug caught: the clean hand-off at 15 deg instead of 5 (M12 — the end inherits a heading TANK1's short
// leg cannot finish under lag); case 3 not handing over, handing over steered, or silent.
TEST_CASE("CV8 a free last point ends along the path's own end direction") {
    const Run r = runRoute(routeNamed("endfree"));
    REQUIRE(r.exit == ExitReason::Settled);
    // The end pose's heading is the path's end direction (24, 12) -> (48, 24): atan2(12, 24) at the
    // end of a Hermite whose last tangent is that chord.
    CHECK(std::abs(r.end.heading().radians() - std::atan2(12.0, 24.0)) <= 1e-9);
    CHECK(r.endHeadErr() <= 1.15 * kDeg);
}

TEST_CASE("CV8 the 12 in bend under 0.18 s of lag reaches the END of the path outside TANK1's window") {
    Rig r{P(0, 0, 0), {}, 0.18};
    const Route route = routeNamed("bend90r12");
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0};
    const Run run = drive(r, m);
    REQUIRE(run.status.handoff == CurveHandoff::EndOfPath);
    CHECK(run.status.handoffHeadingError > 15.0 * kDeg);
    REQUIRE(run.warns.size() == 1);
    CHECK(run.warns[0].find("curve end: ") == 0);
    CHECK(run.warns[0].find(" deg off the end heading at the end of the path (steers within 15.0) -- "
                            "unsteered; more run-up, a gentler last bend or a lower speed into it")
          != std::string::npos);
    REQUIRE(m.endMotion() != nullptr);
    CHECK_FALSE(m.endMotion()->tankSteerStatus().eligible);  // built with maxSteer = 0
}

TEST_CASE("CV8 the clean hand-off: within 5 deg and 0.3 in of the end's line, the end steered") {
    // Every lag-free route that hands off CLEAN does so inside the 5 deg / 0.3 in window, and its end
    // MoveToPose is steered (TANK1 engaged on the same pose).
    for (const Route& route : drivable()) {
        CAPTURE(std::string{route.name});
        Rig r{route.start, {}};
        CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0,
                      route.reverse};
        const Run run = drive(r, m);
        if (run.status.handoff == CurveHandoff::Clean) {
            CHECK(run.status.handoffHeadingError <= 5.0 * kDeg);
        }
        if (run.status.handoff == CurveHandoff::Clean
            || run.status.handoff == CurveHandoff::LastResort) {
            REQUIRE(m.endMotion() != nullptr);
            CHECK(m.endMotion()->tankSteerStatus().engaged);
        }
    }
}

// Bug caught: the "curve end ... unsteered" Warn on a holonomic drive (R13) — there the end MoveToPose
// strafes onto the end, nothing is unsteered, and the Warn would send the author after a problem that
// does not exist. Measured (CURVE1-PROGRESS.md): the 12 in bend on an X-drive and on an H-drive under
// 0.18 s of lag reaches the end of its path 20.5 deg off the end heading, hands off there and settles.
TEST_CASE("CV8 on an X-drive and an H-drive the end of the path outside the window is SILENT, and settles") {
    const auto xd = shulib::kinematics::xDrive(Length{7.0});
    const auto hb = hBotKinematics();
    const shulib::kinematics::IKinematics* kins[2] = {&xd, &hb};
    for (const auto* kin : kins) {
        const Route route = routeNamed("bend90r12");
        Rig r{route.start, {}, 0.18, kin};
        CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0};
        const Run run = drive(r, m);
        REQUIRE(run.status.handoff == CurveHandoff::EndOfPath);  // the case is REACHED
        CAPTURE(run.status.handoffHeadingError / kDeg);
        CHECK(run.status.handoffHeadingError > 15.0 * kDeg);
        CHECK(run.countWarns("curve end") == 0);
        CHECK(run.exit == ExitReason::Settled);
    }
}

// ═══ CV10 — reverse ═══════════════════════════════════════════════════════════════════════
// Bug caught: the reverse flag ignored in the path (M14), or a sign error that drives the arc
// mirrored (L10): the BACK must lead on every pursuit tick, and the path must be followed.
TEST_CASE("CV10 reverse curves: the back leads every pursuit tick, and they settle on the path") {
    const std::vector<Route> routes = {
        routeNamed("revS"),
        {"revBend", P(0, 0, 0), {at(-36, -36, 90)}, true},  // backing round a 90 deg bend
    };
    for (const Route& route : routes) {
        CAPTURE(std::string{route.name});
        Rig r{route.start, {}};
        CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0, true};
        int pursuitTicks = 0;
        double worstVx = -1e9;
        double minVx = 0.0;
        RunOpts o;
        o.measureOffPath = true;
        o.onTick = [&](int, CurveFollow& f, Rig& rr) {
            if (f.curveStatus().phase == CurvePhase::Pursuing && f.exitReason() == ExitReason::Running) {
                ++pursuitTicks;
                const auto& rec = lastMotionRecord(rr.sink);
                const auto body = shulib::math::fieldToRobot(rec.commanded, rec.measuredPose.heading());
                worstVx = std::max(worstVx, body.vx().value());
                minVx = std::min(minVx, body.vx().value());
            }
        };
        const Run run = drive(r, m, o);
        CHECK(run.exit == ExitReason::Settled);
        CAPTURE(run.maxOffPath);
        CHECK(run.maxOffPath <= 1.5);
        CHECK(pursuitTicks > 50);
        CHECK(worstVx <= 0.0);
        CHECK(minVx < -10.0);  // it really drove backwards
        CHECK(std::string{m.name()} == "ReverseCurveTo");
    }
}

// Bug caught: the reverse refusal worded for the NOSE (P3) — "N deg off the robot's heading -- turn
// first: face()" for a point in FRONT of a backing robot sends the author to face() the point, which
// points the back away from it: refused again. The words name the BACK and faceAway().
TEST_CASE("CV10 a reverse curve whose first point is in FRONT: refused, naming the BACK and faceAway()") {
    Rig r{P(0, 0, 0), {}};
    const std::vector<CurvePoint> pts = {thr(48, 0)};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{pts}, motionConfig(), 6.0, true};
    const Run run = drive(r, m);
    CHECK(run.exit == ExitReason::TimedOut);
    CHECK(run.travel == 0.0);
    CHECK(run.warns == std::vector<std::string>{
                           "curve not driven: its first point is 180 deg off the robot's back (at most "
                           "90) -- turn first: faceAway()"});
}

// ═══ CV11 — wait-for-live ════════════════════════════════════════════════════════════════
// Bug caught: the path built in the constructor or at start() (M15) — it would start from the pose
// seeded before the estimate was live, not from where the robot is when it goes live.
TEST_CASE("CV11 issued in the boot window: zero volts while waiting; built from the LIVE pose") {
    World w;
    w.imuReadyAt = 0.5;
    Rig r{P(0, 0, 0), w};
    r.rig.loc.setPose(P(0, 0, 0));
    const std::vector<CurvePoint> pts = {at(48, 12, 0)};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{pts}, motionConfig(), 6.0};
    int waitTicks = 0;
    bool moved = false;
    RunOpts o;
    o.onTick = [&](int, CurveFollow& f, Rig& rr) {
        if (f.state() == motion::MotionState::WaitingForEstimate) {
            ++waitTicks;
            for (int i = 0; i < rr.rig.h.motorCount(); ++i) {
                CHECK(rr.rig.h.motor(i).commandedVoltage().value() == 0.0);
            }
            if (!moved && rr.rig.h.clock().now().value() > 0.2) {
                rr.rig.loc.setPose(P(-6, 3, 0));  // the estimate moved before live
                moved = true;
            }
        }
    };
    const Run run = drive(r, m, o);
    CHECK(waitTicks > 10);
    REQUIRE(moved);
    REQUIRE(m.path().built());
    const auto s0 = m.path().at(0.0);
    CHECK(s0.x == doctest::Approx(-6.0).epsilon(1e-9));
    CHECK(s0.y == doctest::Approx(3.0).epsilon(1e-9));
    CHECK(run.liveAt >= 0.5 - 1e-9);
}

TEST_CASE("CV11 a never-live estimate exits TimedOut at the budget, having never moved") {
    World w;
    w.imuNever = true;
    Rig r{P(0, 0, 0), w};
    const std::vector<CurvePoint> pts = {at(48, 0, 0)};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{pts}, motionConfig(), 1.0};
    const Run run = drive(r, m);
    CHECK(run.exit == ExitReason::TimedOut);
    CHECK(run.travel == 0.0);
    CHECK(std::abs(run.secs - 1.0) <= kTick + 1e-9);
    CHECK_FALSE(m.path().built());
}

// Bug caught: a cancel while still waiting for a live estimate emitting a record with invented errors
// (R17) — there is no estimate to err against, and nothing was commanded.
TEST_CASE("CV11 a cancel while waiting for a live estimate: the exit record's errors and command are 0") {
    World w;
    w.imuNever = true;
    Rig r{P(0, 0, 0), w};
    const std::vector<CurvePoint> pts = {at(48, 12, 30)};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{pts}, motionConfig(), 6.0};
    m.start();
    r.rig.loc.update();
    REQUIRE(m.tick() == ExitReason::Running);
    REQUIRE(m.state() == motion::MotionState::WaitingForEstimate);
    const int before = motionRecords(r.sink);
    m.cancel();
    CHECK(motionRecords(r.sink) == before + 1);  // exactly one exit record
    const auto& rec = lastMotionRecord(r.sink);
    CHECK(rec.errorX.value() == 0.0);
    CHECK(rec.errorY.value() == 0.0);
    CHECK(rec.errorHeading.value() == 0.0);
    CHECK(rec.commanded.vx().value() == 0.0);
    CHECK(rec.commanded.omega().value() == 0.0);
    CHECK(m.exitReason() == ExitReason::Cancelled);
}

// ═══ CV11b — start() fully re-arms ══════════════════════════════════════════════════════
// Bug caught: a re-started curve keeping its OLD path (R1: never rebuilt from where the robot is now),
// its old yaw estimate (R2 — TANK1's own M13, on the curve), or its old closest point (R3: the
// forward-only search then starts mid-path and skips the start of the rebuilt one). motion.hpp's
// contract: start() re-arms fully; the scheduler re-starts a motion it re-runs.
TEST_CASE("CV11b start() again after a cancel: rebuilt from the LIVE pose, observer 0, s from the start") {
    Rig r{P(0, 0, 0), weak(0.85)};
    const std::vector<CurvePoint> pts = {thr(24, 12), at(48, 24, 0)};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{pts}, motionConfig(), 6.0};
    m.start();
    for (int i = 0; i < 60; ++i) {  // 0.6 s of pursuit on a 15 % weak side
        r.rig.loc.update();
        REQUIRE(m.tick() == ExitReason::Running);
        r.rig.h.plant().step(Time{kTick});
    }
    REQUIRE(m.curveStatus().phase == CurvePhase::Pursuing);
    REQUIRE(m.curveStatus().yawCorrection != 0.0);  // the observer HAS an estimate to forget
    REQUIRE(m.curveStatus().s > 3.0);               // ...and the search a point to forget
    const double oldLength = m.path().length();
    m.cancel();
    r.rig.loc.update();
    const Pose2d here = r.rig.loc.pose();
    m.start();
    REQUIRE(m.tick() == ExitReason::Running);
    REQUIRE(m.curveStatus().phase == CurvePhase::Pursuing);
    const auto s0 = m.path().at(0.0);
    CHECK(s0.x == doctest::Approx(here.x().value()).epsilon(1e-12));  // R1
    CHECK(s0.y == doctest::Approx(here.y().value()).epsilon(1e-12));
    CHECK(m.path().length() < oldLength - 3.0);  // the NEW, shorter path
    CHECK(m.curveStatus().yawCorrection == 0.0);  // R2
    CHECK(m.curveStatus().s <= 0.5);              // R3: from the new path's start
}

// ═══ CV12 — the near-start drop, closed loop ════════════════════════════════════════════
// Bug caught: the drop made silently (R16) — the brief's row says ONE Info line, and guide 11 quotes
// it. The plan line follows it, for the one point left.
TEST_CASE("CV12 the near-start drop, closed loop: ONE Info line names it, then the plan's") {
    const Run r = runRoute({"drop", P(0, 0, 0), {thr(0.4, 0), at(48, 0, 0)}});
    CHECK(r.exit == ExitReason::Settled);
    CHECK(r.infoLines == std::vector<std::string>{
                             "curve: point 1 is 0.4 in from the robot -- dropped; do not list the start",
                             "curve: 1 point, 48.0 in, no bend"});
    CHECK(r.warns.empty());
}

// ═══ CV13 — the deadline belongs to the curve ════════════════════════════════════════════
// Bug caught: the end MoveToPose armed with a timeout SHORTER than the curve's (M17) — it would time
// out first and the fault would name MoveToPose, not the curve the author wrote.
TEST_CASE("CV13 a curve that cannot finish exits at ITS timeout, naming CurveTo, even in its end") {
    // Too close for its sideways miss (guide 05's run-up rule): hands off, then cannot settle.
    const Route route{"tooClose", P(-12, 4, 0), {at(0, 0, 0)}};
    Rig r{route.start, {}};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 1.5};
    double handoffAt = -1.0;
    RunOpts o;
    o.onTick = [&](int, CurveFollow& f, Rig& rr) {
        if (handoffAt < 0.0 && f.curveStatus().phase == CurvePhase::Ending) {
            handoffAt = rr.rig.h.clock().now().value();
        }
    };
    const Run run = drive(r, m, o);
    CAPTURE(handoffAt);
    REQUIRE(handoffAt > 0.0);
    REQUIRE(handoffAt < 1.5);  // the end phase HAD begun
    CHECK(run.exit == ExitReason::TimedOut);
    CHECK(std::abs(run.secs - 1.5) <= kTick + 1e-9);
    bool namesCurve = false;
    bool namesMoveToPose = false;
    for (int i = 0; i < r.rig.faultSink.size(); ++i) {
        const std::string line = r.rig.faultSink.at(i).subsystem + " " + r.rig.faultSink.at(i).message;
        namesCurve = namesCurve || line.find("CurveTo") != std::string::npos;
        namesMoveToPose = namesMoveToPose || line.find("MoveToPose") != std::string::npos;
    }
    CHECK(r.rig.latch.raiseCount(shulib::diag::FaultCode::MotionTimeout) == 1);
    CHECK(namesCurve);
    CHECK_FALSE(namesMoveToPose);
    CHECK(run.motorsZeroAtExit);
}

// The one consequence of the watchdog running FIRST (curve_follow.hpp, the tick's order), pinned on
// its exact tick: a curve whose end would settle ON the tick its budget runs out exits TimedOut —
// the reverse of MoveToPose's own order (Settled beats a simultaneous TimedOut). Measured: the
// straight 48 in curve settles at 2.27 s; with exactly that budget it times out on that tick; 5 ms
// more and it settles. Bug caught: the end MoveToPose ticked before the curve's watchdog (D1 — the
// verdict would flip to Settled and the documented order would be a lie).
TEST_CASE("CV13 the curve's watchdog runs first: a settle on the deadline's own tick is TimedOut") {
    const Route route = routeNamed("straight48");
    const Run free = runRoute(route);
    REQUIRE(free.exit == ExitReason::Settled);
    const double settleAt = free.secs;  // the clock on the settling tick (the motion started at 0)
    const Run onIt = runRoute(route, {}, 0.0, motionConfig(), settleAt);
    CHECK(onIt.exit == ExitReason::TimedOut);
    CHECK(onIt.secs == settleAt);  // the SAME tick, landed on exactly
    const Run after = runRoute(route, {}, 0.0, motionConfig(), settleAt + 0.005);
    CHECK(after.exit == ExitReason::Settled);
    CHECK(after.secs == settleAt);
}

// ═══ CV14 — the observer is TANK1's ══════════════════════════════════════════════════════
// Bug caught: the curve's copy of the yaw observer drifting from MoveToPose's (M18: its clamp removed
// in the curve only). The sequence is MoveToPose's own: a weak-side leg with the robot PINNED for a
// while (the estimate integrates to the clamp), fed tick by tick to curveYawObserverStep with the
// same (measured, commanded, dt, ready) — the estimates must match bit for bit.
TEST_CASE("CV14 curveYawObserverStep reproduces MoveToPose's yaw estimate bit for bit, 200+ ticks") {
    struct Pinned final : sim::DegradationModel {
        units::Voltage effectiveVoltage(int w, units::Voltage c, Time now, sim::Rng&) override {
            if (now.value() >= 0.4 && now.value() < 0.9) {
                return units::Voltage{w == 0 ? 0.0 : c.value()};  // left pinned: a hard yaw
            }
            return w == 1 ? units::Voltage{c.value() * 0.85} : c;
        }
    } world;
    TankKinematics kin{Length{12.0}};
    FakeTelemetrySink sink;
    MotionRig rig{kin, plantConfig(), &sink, &world};
    motion::MoveToPose m{rig.deps, P(60, 0, 0), motionConfig(), 5.0};
    m.start();
    double est = 0.0;
    double lastCmd = 0.0;
    bool hasCmd = false;
    double lastT = 0.0;
    bool hasT = false;
    int ticks = 0;
    bool hitClamp = false;
    const motion::TankSteerConfig& ts = motionConfig().tankSteer;
    for (int i = 0; i < 400; ++i) {
        rig.loc.update();
        const double now = rig.h.clock().now().value();
        const double dt = hasT ? now - lastT : 0.0;
        lastT = now;
        hasT = true;
        const double measured = rig.loc.twist().omega().value();
        const bool ready = rig.h.context().imu().isReady();
        const ExitReason e = m.tick();
        if (hasCmd && dt > 0.0 && ready) {
            est = motion::curveYawObserverStep(est, measured, lastCmd, dt, ts);
        }
        if (m.state() != motion::MotionState::WaitingForEstimate) {
            REQUIRE(m.tankSteerStatus().engaged);
        }
        CHECK(est == m.tankSteerStatus().yawCorrection);  // bits
        hitClamp = hitClamp || std::abs(est) == ts.yawCorrectionLimit;
        lastCmd = lastMotionRecord(sink).commanded.omega().value();
        hasCmd = true;
        ++ticks;
        if (e != ExitReason::Running) {
            break;
        }
        rig.h.plant().step(Time{kTick});
    }
    CHECK(ticks >= 200);
    CHECK(hitClamp);  // the sequence reached the clamp, so a clamp-less copy would differ
}

// ═══ CV15 — a weak side and lag ══════════════════════════════════════════════════════════
// Bug caught: the yaw observer switched off in the curve (M19 — the probes' `ppnoobs` dropped at
// x0.85).
// ═══ CV14b — the observer's IMU gate ════════════════════════════════════════════════════
// Bug caught: the curve's observer ignoring its IMU-ready gate (R14) — MoveToPose holds its estimate
// while the IMU is not ready; the curve's copy must too, or it integrates a yaw rate nobody measured.
TEST_CASE("CV14b the yaw observer HOLDS its estimate while the IMU is not ready (0.6-0.9 s)") {
    World w;
    w.scale = 0.85;
    w.imuOffFrom = 0.6;
    w.imuOffTo = 0.9;
    const Route route{"long", P(0, 0, 0), {thr(36, 18), at(72, 36, 0)}};
    double held = 0.0;
    int inside = 0;
    bool changedInside = false;
    bool changedBefore = false;
    bool changedAfter = false;
    double prev = 0.0;
    RunOpts o;
    o.onTick = [&](int, CurveFollow& f, Rig& rr) {
        const double now = rr.rig.h.clock().now().value();
        if (f.curveStatus().phase != CurvePhase::Pursuing || f.exitReason() != ExitReason::Running) {
            return;
        }
        const double y = f.curveStatus().yawCorrection;
        if (now > 0.605 && now < 0.895) {
            if (inside == 0) {
                held = y;
            }
            ++inside;
            changedInside = changedInside || y != held;
        } else if (now < 0.6) {
            changedBefore = changedBefore || y != prev;
        } else if (now > 0.92 && now < 1.2) {
            changedAfter = changedAfter || y != prev;
        }
        prev = y;
    };
    (void)runRoute(route, w, 0.0, motionConfig(), 6.0, o);
    REQUIRE(inside > 20);
    CHECK(changedBefore);  // it IS estimating before the drop-out...
    CHECK_FALSE(changedInside);  // ...holds through it...
    CHECK(changedAfter);   // ...and resumes after it
}

TEST_CASE("CV15 the S, the 36 in bend and the field S settle at a 15 % weaker side and at 0.12 s lag") {
    for (const char* n : {"scurve", "bend90r36", "fieldS"}) {
        CAPTURE(std::string{n});
        const Run weakRun = runRoute(routeNamed(n), weak(0.85));
        CHECK(weakRun.exit == ExitReason::Settled);
        const Run lagRun = runRoute(routeNamed(n), {}, 0.12);
        CHECK(lagRun.exit == ExitReason::Settled);
    }
}

// ═══ CV16 — NOTHING ELSE CHANGES ═════════════════════════════════════════════════════════
// Bug caught: any edit to another verb's behaviour (M20: MoveToPose's tick given the curve's
// look-ahead). The six-leg auton of today's verbs on a tank (one side 9 % weak, so TANK1 is live)
// and on an X-drive: records (blackbox bytes), log lines and truth hashed; the literals were measured
// with the SAME header compiled against the merge base 1ed0e26 (CURVE1-PROGRESS.md) — bit-identical.
TEST_CASE("CV16 today's verbs are bit-identical to the merge base (tank and X-drive)") {
    TankKinematics tank{Length{12.0}};
    const auto xd = shulib::kinematics::xDrive(Length{7.0});
    CHECK(curve_cv16::routeHash(tank, 0.91) == 0x729434ff7e17ca34ULL);
    CHECK(curve_cv16::routeHash(xd, 1.0) == 0x0359c0df55b2f03bULL);
}

// ═══ CV18 — the records and the result line ══════════════════════════════════════════════
// Bug caught: an exit record emitted BEFORE the motors stop (M21 — a sink that reads the motors when
// a record arrives sees the exit record with volts still on); two records on a tick; a record whose
// target is not the end pose; the flag lit below its floor.
namespace {
/// Reads the drive motors' commanded volts whenever a record arrives.
struct SnoopSink final : shulib::hal::ITelemetrySink {
    shulib::hal::ITelemetrySink* inner = nullptr;
    sim::SimHarness* h = nullptr;
    int exitRecordsWithVolts = 0;
    void log(LogLevel l, std::string_view s, std::string_view m) override { inner->log(l, s, m); }
    [[nodiscard]] bool wantsRecord() const noexcept override { return true; }
    void emit(const shulib::diag::DebugRecord& r) override {
        const auto st = static_cast<motion::MotionState>(r.activeCommandState);
        if (st == motion::MotionState::TimedOut || st == motion::MotionState::Settled) {
            for (int w = 0; w < h->motorCount(); ++w) {
                exitRecordsWithVolts += h->motor(w).commandedVoltage().value() != 0.0 ? 1 : 0;
            }
        }
        inner->emit(r);
    }
};
}  // namespace

TEST_CASE("CV18 one record per tick, the end pose on every live record, the exit record's command 0") {
    const Route route = routeNamed("scurve");
    Rig r{route.start, weak(0.91)};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0};
    int lastCount = 0;
    int pursuitTicks = 0;
    const double floor = motion::kTankSteerNoiseFraction * motionConfig().maxAngularSpeed.value();
    RunOpts o;
    o.onTick = [&](int, CurveFollow& f, Rig& rr) {
        CHECK(motionRecords(rr.sink) == lastCount + 1);  // exactly one MOTION record per tick
        lastCount = motionRecords(rr.sink);
        const auto& rec = lastMotionRecord(rr.sink);
        CHECK(posErr(rec.targetPose, f.endPose()) == 0.0);
        CHECK(rec.targetPose.heading().radians() == f.endPose().heading().radians());
        if (f.curveStatus().phase == CurvePhase::Pursuing && f.exitReason() == ExitReason::Running) {
            ++pursuitTicks;
            CHECK(rec.tankSteerActive == (std::abs(f.curveStatus().yawCorrection) > floor));
        }
    };
    const Run run = drive(r, m, o);
    REQUIRE(run.exit == ExitReason::Settled);
    CHECK(pursuitTicks > 50);
    const auto& last = lastMotionRecord(r.sink);
    CHECK(last.commanded.vx().value() == 0.0);
    CHECK(last.commanded.omega().value() == 0.0);
}

TEST_CASE("CV18 the curve's own exit record comes AFTER its motors stop (a guard exit)") {
    const Route route = routeNamed("scurve");
    FakeTelemetrySink inner;
    World w;
    w.deadFrom = 0.8;
    TankKinematics tank{Length{12.0}};
    SnoopSink snoop;
    snoop.inner = &inner;
    MotionRig rig{tank, Rig::cfgFor(route.start, 0.0), &snoop, &w};
    snoop.h = &rig.h;
    CurveFollow m{rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0};
    REQUIRE(rig.run(m, 2000) == ExitReason::TimedOut);
    REQUIRE(m.curveStatus().guard != CurveGuard::None);
    CHECK(snoop.exitRecordsWithVolts == 0);
}

TEST_CASE("CV18 the result line reads 'CurveTo ... SETTLED' with the xt column on a tank") {
    const Route route = routeNamed("scurve");
    TankKinematics tank{Length{12.0}};
    FakeTelemetrySink sink;
    MotionRig rig{tank, Rig::cfgFor(route.start, 0.0), &sink};
    PlantPacer pacer{rig.h};
    shulib::chassis::Chassis chassis{rig.deps, pacer, chassisConfig()};
    motion::RunReporter rep{sink, chassis.scheduler()};
    REQUIRE(chassis.curveTo(std::span<const CurvePoint>{route.pts}) == ExitReason::Settled);
    std::string line;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).message.find("CurveTo#") != std::string::npos) {
            line = sink.at(i).message;
        }
    }
    CAPTURE(line);
    CHECK(line.find("SETTLED") != std::string::npos);
    CHECK(line.find(" xt ") != std::string::npos);
    CHECK(chassis.lastCompleted().hasCrossTrack);
    CHECK(std::string{chassis.lastCompleted().name} == "CurveTo");
}

// ═══ CV19 — the speed plan, closed loop ══════════════════════════════════════════════════
// The pure half (the plan itself: the 2 in look-ahead, the backward pass, the turn-rate cap) is
// curve_path_test.cpp; this half proves the follower OBEYS it on the plant.
TEST_CASE("CV19 the bend speed, the call's speed cap, and a monotone slowing into the end") {
    MotionConfig cfg = motionConfig();
    cfg.maxLinearSpeed = units::Velocity{30.0};  // the call's cap, as effectiveConfig applies it
    const Route route = routeNamed("bend90r18");  // radius 11.4 in
    Rig r{route.start, {}};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, cfg, 6.0};
    double worstOverCap = 0.0;
    double worstInBend = 0.0;
    double lastEndSpeed = 1e9;
    bool monotone = true;
    RunOpts o;
    o.onTick = [&](int, CurveFollow& f, Rig&) {
        const motion::CurveStatus& st = f.curveStatus();
        if (st.phase != CurvePhase::Pursuing || f.exitReason() != ExitReason::Running) {
            return;
        }
        worstOverCap = std::max(worstOverCap, st.commandedSpeed - 30.0);
        const double k = std::abs(f.path().at(st.s).curvature);
        if (k > 1.0 / 14.0) {  // inside the bend (radius under 14 in)
            worstInBend = std::max(worstInBend, st.commandedSpeed - std::sqrt(60.0 / k));
        }
        if (f.path().length() - st.s <= 12.0) {
            monotone = monotone && st.commandedSpeed <= lastEndSpeed + 1e-9;
            lastEndSpeed = st.commandedSpeed;
        }
    };
    const Run run = drive(r, m, o);
    CHECK(run.exit == ExitReason::Settled);
    CHECK(worstOverCap <= 1e-9);
    CAPTURE(worstInBend);
    CHECK(worstInBend <= 1.0);
    CHECK(monotone);
}

TEST_CASE("CV19 a 72 in straight into a 2.5 in bend: ALREADY at the bend's speed when it gets there") {
    const Route route{"straightIntoBend", P(0, 0, 0), {at(72, 0, 0), at(74.5, 2.5, 90)}};
    Rig r{route.start, {}};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0};
    double speedAtBend = -1.0;
    double lastSpeed = -1.0;
    double worstDrop = 0.0;
    double peak = 0.0;
    RunOpts o;
    o.onTick = [&](int, CurveFollow& f, Rig&) {
        const motion::CurveStatus& st = f.curveStatus();
        if (st.phase != CurvePhase::Pursuing || f.exitReason() != ExitReason::Running) {
            return;
        }
        peak = std::max(peak, st.commandedSpeed);
        if (lastSpeed >= 0.0 && speedAtBend < 0.0) {
            worstDrop = std::max(worstDrop, lastSpeed - st.commandedSpeed);
        }
        if (speedAtBend < 0.0 && st.s >= f.path().knotS(1)) {
            speedAtBend = st.commandedSpeed;
        }
        lastSpeed = st.commandedSpeed;
    };
    (void)drive(r, m, o);
    // The bend's planned speed: sqrt(bendAccel / k) at its tightest, capped by maxAngular / k.
    double kBend = 0.0;
    for (double s = m.path().knotS(1); s <= m.path().knotS(1) + 2.0; s += 0.01) {
        kBend = std::max(kBend, std::abs(m.path().at(s).curvature));
    }
    const double vBend = std::min(std::sqrt(60.0 / kBend), 6.0 / kBend);
    CAPTURE(speedAtBend);
    CAPTURE(vBend);
    CAPTURE(peak);
    CAPTURE(worstDrop);
    REQUIRE(peak > 45.0);  // it really was fast on the straight
    REQUIRE(speedAtBend >= 0.0);
    CHECK(speedAtBend <= vBend + 1.0);
    // Slowed at no more than decel: per 10 ms tick, decel x dt plus one tick's rounding of the plan
    // (speed^2 falls by 2 decel ds; ds per tick <= peak x dt).
    CHECK(worstDrop <= 120.0 * kTick * 1.5);
}

// Bug caught: the per-tick speed ramp removed (R11 — the first command jumps to the plan's 12 in/s,
// a wheel-slip risk on carpet) or the first tick's dt fallback 0 (R22 — the first command 0). The
// ordinary competition case: a curve issued mid-auton, the estimate ALREADY live, so its first tick
// has no measured dt.
TEST_CASE("CV19 a curve issued mid-auton: the first command is ONE tick of accel, the second at most two") {
    Rig r{P(0, 0, 0), {}};
    for (int i = 0; i < 300 && r.rig.loc.qualityClass()
                                   == shulib::localization::Localizer::Quality::Uninitialized;
         ++i) {
        r.rig.h.plant().step(Time{kTick});
        r.rig.loc.update();
    }
    REQUIRE(r.rig.loc.qualityClass() != shulib::localization::Localizer::Quality::Uninitialized);
    const std::vector<CurvePoint> pts = {at(48, 0, 0)};
    CurveFollow m{r.rig.deps, std::span<const CurvePoint>{pts}, motionConfig(), 6.0};
    const double accel = motionConfig().curve.accel;
    m.start();
    REQUIRE(m.tick() == ExitReason::Running);
    REQUIRE(m.curveStatus().phase == CurvePhase::Pursuing);
    CHECK(m.curveStatus().commandedSpeed == doctest::Approx(accel * 0.01).epsilon(1e-12));
    r.rig.h.plant().step(Time{kTick});
    r.rig.loc.update();
    REQUIRE(m.tick() == ExitReason::Running);
    CHECK(m.curveStatus().commandedSpeed > accel * 0.01);
    CHECK(m.curveStatus().commandedSpeed <= 2.0 * accel * 0.01 + 1e-9);
}

// ═══ CV20 — other drivetrains drive like a tank ══════════════════════════════════════════
TEST_CASE("CV20 the S on an X-drive and an H-drive: settles, body vy 0 every pursuit tick, no SFB") {
    const auto xd = shulib::kinematics::xDrive(Length{7.0});
    const auto hb = hBotKinematics();
    const shulib::kinematics::IKinematics* kins[2] = {&xd, &hb};
    for (const auto* kin : kins) {
        const Route route = routeNamed("scurve");
        Rig r{route.start, {}, 0.0, kin};
        CurveFollow m{r.rig.deps, std::span<const CurvePoint>{route.pts}, motionConfig(), 6.0};
        int pursuit = 0;
        int sfb = 0;
        double worstVy = 0.0;
        RunOpts o;
        o.onTick = [&](int, CurveFollow& f, Rig& rr) {
            const auto& rec = lastMotionRecord(rr.sink);
            sfb += rec.strafeFallbackActive ? 1 : 0;
            if (f.curveStatus().phase == CurvePhase::Pursuing && f.exitReason() == ExitReason::Running) {
                ++pursuit;
                const auto body = shulib::math::fieldToRobot(rec.commanded, rec.measuredPose.heading());
                worstVy = std::max(worstVy, std::abs(body.vy().value()));
            }
        };
        const Run run = drive(r, m, o);
        CHECK(run.exit == ExitReason::Settled);
        CHECK(pursuit > 50);
        CHECK(worstVy <= 1e-9);
        CHECK(sfb == 0);
    }
}

// ═══ CV21 — the size (HA-250) ════════════════════════════════════════════════════════════
TEST_CASE("CV21 the curve fits its stated stack budget") {
    static_assert(sizeof(CurveFollow) <= 6144);
    // Measured on the host (CURVE1-PROGRESS.md): 5,904 B. The ARM -fstack-usage frame is the real gate.
    CHECK(sizeof(CurveFollow) <= 6144);
}

// ═══ CV23 — THE RUN-UP RULE guide 05 states ══════════════════════════════════════════════
// The rule: with the SHIPPED CurveConfig (look-ahead 8 in), a one-point curveTo removes a sideways
// miss of up to 6 in from 2.5 ft back along the end's line, and up to 1 ft from 3 ft back — the nose
// within 20 deg of the end heading, a side up to 9 % weaker, motor lag 0 to 0.18 s.
// A RED CV23 MEANS RE-MEASURING THE RULE AND UPDATING docs/guide/05-getting-there.md IN THE SAME
// COMMIT — NEVER TUNING THE FOLLOWER TO PASS IT.
// Bug caught: a longer look-ahead (M27: 12 in) or a softer path (M28: tangent 1/2 x chord) that no
// longer arcs onto the end's line in the run-up the guide promises.
namespace {
Run runUp(double D, double S, double noseDeg, double weakScale, double tau,
          const MotionConfig& cfg = motionConfig()) {
    const Route route{"runup", P(D, S, noseDeg), {at(0, 0, 0)}};
    return runRoute(route, weak(weakScale), tau, cfg, 6.0);
}
}  // namespace

TEST_CASE("CV23 the run-up rule: (D, S) = (-30, +-2), (-30, +-6), (-36, +-12) — 60 runs a cell, all") {
    struct Cell {
        double D;
        double S;
    };
    const Cell cells[3] = {{-30, 2}, {-30, 6}, {-36, 12}};
    for (const Cell& c : cells) {
        int ok = 0;
        int runs = 0;
        for (const double side : {1.0, -1.0}) {
            for (const double nose : {0.0, 10.0, -10.0, 20.0, -20.0}) {
                for (const double w : {1.00, 0.91}) {
                    for (const double tau : {0.0, 0.12, 0.18}) {
                        const Run r = runUp(c.D, side * c.S, nose, w, tau);
                        ++runs;
                        const bool good = r.exit == ExitReason::Settled && r.endPosErr() <= 0.5
                                          && r.endHeadErr() <= 1.15 * kDeg;
                        ok += good ? 1 : 0;
                        if (!good) {
                            CAPTURE(c.D);
                            CAPTURE(side * c.S);
                            CAPTURE(nose);
                            CAPTURE(w);
                            CAPTURE(tau);
                            CAPTURE(r.endPosErr());
                            CHECK(good);
                        }
                    }
                }
            }
        }
        CHECK(runs == 60);
        CHECK(ok == 60);
        // LOGGED, not asserted: the smallest run-up (1 in grid) from which all 60 still pass — the
        // margin against the rule (the brief's probe: 28, 29 and 34 in).
        double smallest = -c.D;
        for (double d = -c.D - 1.0; d >= 12.0; d -= 1.0) {
            bool all = true;
            for (const double side : {1.0, -1.0}) {
                for (const double nose : {0.0, 10.0, -10.0, 20.0, -20.0}) {
                    for (const double w : {1.00, 0.91}) {
                        for (const double tau : {0.0, 0.12, 0.18}) {
                            const Run r = runUp(-d, side * c.S, nose, w, tau);
                            all = all && r.exit == ExitReason::Settled && r.endPosErr() <= 0.5
                                  && r.endHeadErr() <= 1.15 * kDeg;
                            if (!all) {
                                break;
                            }
                        }
                        if (!all) {
                            break;
                        }
                    }
                    if (!all) {
                        break;
                    }
                }
                if (!all) {
                    break;
                }
            }
            if (!all) {
                break;
            }
            smallest = d;
        }
        MESSAGE("CV23 cell S = " << c.S << " in: the rule's run-up " << -c.D
                                 << " in; the smallest passing run-up " << smallest << " in");
    }
}

TEST_CASE("CV23 too close for the rule, (-12, +-4): misses SAFELY — the whole budget, no guard, <= 2 in") {
    int timedOut = 0;
    double worst = 0.0;
    for (const double side : {1.0, -1.0}) {
        for (const double nose : {0.0, 20.0, -20.0}) {
            for (const double w : {1.00, 0.91}) {
                for (const double tau : {0.0, 0.12, 0.18}) {
                    const Run r = runUp(-12, side * 4, nose, w, tau);
                    CAPTURE(side);
                    CAPTURE(nose);
                    CAPTURE(w);
                    CAPTURE(tau);
                    CHECK(r.status.guard == CurveGuard::None);
                    // Every Warn is the curve's own: at most ONE "curve end ... unsteered", and D3's
                    // TightBend where the run-up makes a tight arc (the nose 20 deg away — measured).
                    int curveEnd = 0;
                    int tight = 0;
                    for (const std::string& m : r.warns) {
                        CHECK(m.find("turn first") == std::string::npos);
                        CHECK(m.find("not steered") == std::string::npos);
                        curveEnd += m.find("curve end: ") == 0 ? 1 : 0;
                        tight += m.find("curve: tightest bend ") == 0 ? 1 : 0;
                    }
                    CHECK(curveEnd <= 1);
                    CHECK(tight <= 1);
                    CHECK(static_cast<int>(r.warns.size()) == curveEnd + tight);
                    if (r.exit != ExitReason::Settled) {
                        ++timedOut;
                        CHECK(r.exit == ExitReason::TimedOut);
                        CHECK(std::abs(r.secs - 6.0) <= kTick + 1e-9);
                    }
                    worst = std::max(worst, r.endPosErr());
                    CHECK(r.endPosErr() <= 2.0);
                }
            }
        }
    }
    MESSAGE("CV23 too close: " << timedOut << " of 36 timed out at the budget; worst end "
                               << worst << " in off");
}
