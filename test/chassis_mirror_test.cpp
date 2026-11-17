// The Chassis's MIRRORED RUN — MIRROR1-A's T2, T3 and the scope half of T4 (the MIRROR1 brief's §2.2), on
// the host test plant. Everything here is the host test plant; nothing here has run on a robot.
//
// THE METHOD (the brief's probe P1, made a test): one route, written ONCE in robot one's numbers, runs
// twice — robot one from S with no mirror, robot two from the reflection of S inside a
// chassis::MirroredRun across Override's skills diagonal (y = -x) — each on its own plant, through the
// program's pacer chain (Chassis -> RunGuard -> MechanismService over robot two's real list -> the
// world). Robot two's TRUE pose after every pace must be the reflection of robot one's, to 1e-9 in and
// 1e-9 rad, on a tank AND on an X-drive; every trigger must fire on the same pace; every pose the route
// reads back must be robot one's numbers. The reflection the test compares with is its OWN (rf() below:
// (x, y, h) -> (-y, -x, -90 deg - h), written out by hand), never the library's.
//
// Each case attacks ONE rule and names the bug it would catch.

#include "doctest.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "auton_tester_rig.hpp"
#include "motion_test_rig.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/fake/fake_controller.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/reflection.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/curve_path.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

using namespace shulib::units::literals;
using shulib::chassis::Chassis;
using shulib::chassis::MirroredRun;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Frame;
using shulib::math::Pose2d;
using shulib::math::Reflection;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

constexpr double kPi = 3.14159265358979323846;
const Reflection kDiag = Reflection::across(Reflection::Line::AntiDiagonal);

/// The TEST's own skills reflection, written out by hand (file banner) — never the library's.
Pose2d rf(const Pose2d& p) {
    return Pose2d{Length{-p.y().value()}, Length{-p.x().value()},
                  Angle::radians(-kPi / 2.0 - p.heading().radians())};
}

/// The program's chain under one robot (file banner); records the TRUE pose and every slot's step count
/// after every pace.
struct Bot {
    struct World final : shulib::motion::ITickPacer {
        Bot* b = nullptr;
        long paces = 0;
        std::function<void()> onPace;
        void pace() override {
            if (++paces > 300000) {
                throw std::runtime_error("chassis_mirror_test: pace cap exceeded");
            }
            if (onPace) {
                onPace();
            }
            b->rig.h.plant().step(Time{0.01});
            b->truth.push_back(b->rig.h.truePose());
            std::array<std::uint32_t, 3> begun{};
            for (std::size_t i = 0; i < 3; ++i) {
                begun[i] = b->svc.slot(i).stepsBegun();
            }
            b->begun.push_back(begun);
        }
    };
    static shulib::sim::SimHarnessConfig config(const Pose2d& start) {
        shulib::sim::SimHarnessConfig c = motion_rig::plantConfig();
        c.plant.initialPose = start;
        return c;
    }

    shulib::hal::fake::FakeTelemetrySink sink;
    tester_rig::WeakSide weak;
    motion_rig::MotionRig rig;
    World world;
    tester_rig::RobotTwoFakes two;
    shulib::mechanism::MechanismService svc{world};
    shulib::sequence::RunGuard guard{svc};
    Chassis chassis;
    std::vector<Pose2d> truth;
    std::vector<std::array<std::uint32_t, 3>> begun;
    std::vector<Pose2d> seen;  ///< what the route read back through chassis.pose()

    Bot(const shulib::kinematics::IKinematics& kin, const Pose2d& start)
        : rig{kin, config(start), &sink, &weak}, chassis{rig.deps, guard, motion_rig::chassisConfig()} {
        world.b = this;
        svc.bind(shulib::bench::robot_two::kRobotTwoMechanisms, two.devs, rig.h.clock(), rig.latch, sink);
        for (int i = 0; i < 20; ++i) {
            chassis.wait(Time{0.01});  // the estimate live, nothing commanded
        }
        truth.clear();
        begun.clear();
    }
    [[nodiscard]] int count(const char* subsystem) const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            n += sink.at(i).subsystem == subsystem ? 1 : 0;
        }
        return n;
    }
    [[nodiscard]] std::string first(const char* subsystem) const {
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == subsystem) {
                return sink.at(i).message;
            }
        }
        return "";
    }
};

/// What the Routine's context constructor reads (no guard running: these runs need no runAuton).
struct Ctx {
    Chassis& chassis;
    shulib::mechanism::MechanismService* mechanismService;
};

/// The raw drive, both frames, then a brake — the `then()` a team writes for a manual push.
bool driveBothFrames(Chassis& c) {
    for (int i = 0; i < 25; ++i) {
        c.drive(ChassisSpeeds{Velocity{8.0}, Velocity{5.0}, AngularVelocity{0.6}}, Frame::Field);
        c.wait(Time{0.01});
    }
    for (int i = 0; i < 25; ++i) {
        c.drive(ChassisSpeeds{Velocity{-7.0}, Velocity{4.0}, AngularVelocity{-0.8}}, Frame::Body);
        c.wait(Time{0.01});
    }
    return c.brake() == shulib::control::ExitReason::Settled;
}

/// THE TANK ROUTE — every verb a tank route writes, kept on robot one's side of the diagonal (the
/// brief's landmine 13), with a STEP1-B trigger of each kind on both observer overloads, a pose read
/// back, a setPose, and drive() in both frames. Robot one's numbers.
RoutineResult tankRoute(Ctx& ctx, std::vector<Pose2d>& seen) {
    using namespace shulib::bench::robot_two;
    using shulib::chassis::carryOn;
    Routine r{ctx, "mirror T2 tank"};
    r.atPercent(50).startRun(clawPivot6, shulib::units::Voltage{6.0}, 0.3_s).driveTo(-12_in, -58_in);
    r.afterTurning(30_deg).startRun(clawPivot7, shulib::units::Voltage{6.0}, 0.3_s).face(-12_in, -40_in);
    r.afterDistance(6_in).startRun(clawPivot6, shulib::units::Voltage{-6.0}, 0.2_s).driveTo(-12_in, -40_in);
    r.turnTo(45_deg);
    r.moveTo(Pose2d{-2_in, -30_in, 45_deg});
    r.then([&] { seen.push_back(ctx.chassis.pose()); }, "read pose");
    r.faceAway(-24_in, -36_in);
    r.reverseTo(-24_in, -36_in);
    r.curveTo({{-6_in, -26_in}, {12_in, -32_in, -30_deg}});
    r.reverseCurveTo({{-6_in, -44_in}, {-24_in, -54_in, 0_deg}});
    r.followTrajectory({Pose2d{-10_in, -54_in, 0_deg}, Pose2d{2_in, -54_in, 0_deg}});
    r.onFail(carryOn).strafeTo(2_in, -50_in, {.timeout = 0.6_s});  // a tank: times out, on both
    r.then([&] { return driveBothFrames(ctx.chassis); }, "drive both frames");
    r.then(
        [&] {
            const Pose2d p = ctx.chassis.pose();
            seen.push_back(p);
            ctx.chassis.setPose(Pose2d{p.x() + 1.5_in, p.y() - 0.5_in, p.heading()});
        },
        "setPose");
    r.waitFor([&] { return ctx.chassis.pose().x().value() > -100.0; }, 0.3_s, "pose waitFor");
    r.face(10_in, -44_in);
    r.hold(300_ms);
    r.pause(200_ms);
    r.driveTo(10_in, -44_in);
    r.then([&] { seen.push_back(ctx.chassis.pose()); }, "read pose at the end");
    return r.result();
}

/// THE X-DRIVE ROUTE — the holonomic verbs (strafeTo moves here), a moveTo that turns while it drives
/// (never an exact 180 deg heading change: that is T3's), triggers, a curve, a trajectory, drive(). The
/// curve's stated end heading is -30 deg, NOT -45: -45 deg lies along the mirror line, so -90 - (-45) =
/// -45 is its own reflection and a mutant that skips reflecting a curve's headings (MM7) is invisible
/// there (the review pass's re-run found MM7 red on the tank route only).
RoutineResult xdriveRoute(Ctx& ctx, std::vector<Pose2d>& seen) {
    using namespace shulib::bench::robot_two;
    Routine r{ctx, "mirror T2 xdrive"};
    r.atPercent(40).startRun(clawPivot6, shulib::units::Voltage{6.0}, 0.3_s).moveTo(Pose2d{-20_in, -50_in, 30_deg});
    r.strafeTo(-20_in, -38_in);
    r.afterTurning(20_deg).startRun(clawPivot7, shulib::units::Voltage{6.0}, 0.3_s).turnTo(-60_deg);
    r.afterDistance(4_in).startRun(clawPivot6, shulib::units::Voltage{-6.0}, 0.2_s).driveTo(-8_in, -40_in);
    r.moveTo(Pose2d{-14_in, -30_in, 100_deg});
    r.then([&] { seen.push_back(ctx.chassis.pose()); }, "read pose");
    r.curveTo({{-4_in, -24_in}, {8_in, -34_in, -30_deg}});
    r.followTrajectory({Pose2d{-6_in, -46_in, 0_deg}, Pose2d{-24_in, -52_in, -20_deg}});
    r.then([&] { return driveBothFrames(ctx.chassis); }, "drive both frames");
    r.face(-30_in, -30_in);
    r.driveTo(-30_in, -36_in);
    r.then([&] { seen.push_back(ctx.chassis.pose()); }, "read pose at the end");
    return r.result();
}

struct PairRun {
    RoutineResult one{};
    RoutineResult two{};
    double maxPos = 0.0;
    double maxHead = 0.0;
    std::size_t paces = 0;
    bool samePaces = false;
    bool sameFires = false;
    bool sameSeen = false;
};

/// Run `route` as robot one from `start` and as robot two from its reflection, mirrored; compare.
template <class Route>
PairRun runPair(const shulib::kinematics::IKinematics& kin, const Pose2d& start, Route route) {
    PairRun out;
    Bot one{kin, start};
    Bot two{kin, rf(start)};
    {
        Ctx c{one.chassis, &one.svc};
        out.one = route(c, one.seen);
    }
    {
        const MirroredRun run{two.chassis, kDiag};
        Ctx c{two.chassis, &two.svc};
        out.two = route(c, two.seen);
    }
    CHECK(two.chassis.reflection().isIdentity());
    out.samePaces = one.truth.size() == two.truth.size();
    out.paces = std::min(one.truth.size(), two.truth.size());
    for (std::size_t i = 0; i < out.paces; ++i) {
        const Pose2d want = rf(one.truth[i]);
        out.maxPos = std::max(out.maxPos, std::hypot((want.x() - two.truth[i].x()).value(),
                                                     (want.y() - two.truth[i].y()).value()));
        out.maxHead = std::max(out.maxHead, std::fabs(want.heading().errorTo(two.truth[i].heading())));
    }
    out.sameFires = one.begun == two.begun;
    out.sameSeen = one.seen.size() == two.seen.size() && !one.seen.empty();
    for (std::size_t i = 0; out.sameSeen && i < one.seen.size(); ++i) {
        out.sameSeen = one.seen[i].approxEqual(two.seen[i], Length{1e-9}, 1e-9);
    }
    return out;
}

const shulib::kinematics::TankKinematics& tank() {
    static const shulib::kinematics::TankKinematics k{Length{12.0}};
    return k;
}
const shulib::kinematics::MatrixKinematics& xdrive() {
    static const shulib::kinematics::MatrixKinematics k = shulib::kinematics::xDrive(Length{7.0});
    return k;
}

}  // namespace

// Bug caught: any verb that does not reflect its field numbers (MM7 a curve's headings, MM8 an observer
// overload's target, MM13 a trajectory's later legs), pose() not reflected back (MM2 — face() and the
// reverse bearing compute from it), setPose() unreflected, drive(Body) keeping its turn (MM4), MM1 (the
// field heading as -h) — each makes robot two's path stop being robot one's reflection; a trigger that
// fires on a different pace. On a TANK, every verb a tank route writes.
TEST_CASE("MIRROR1 T2 tank: robot two's path is robot one's reflection pace by pace, every verb, every trigger") {
    const PairRun p = runPair(tank(), Pose2d{-36_in, -58_in, 0_deg}, tankRoute);
    CHECK(p.one.steps == 20);
    CHECK(p.one.completed == 19);  // the strafe times out on a tank — on BOTH (next line)
    CHECK(p.two.steps == p.one.steps);
    CHECK(p.two.completed == p.one.completed);
    CHECK(p.samePaces);
    CHECK(p.paces > 1500U);
    CHECK(p.maxPos <= 1e-9);
    CHECK(p.maxHead <= 1e-9);
    CHECK(p.sameFires);
    CHECK(p.sameSeen);
    MESSAGE("tank: " << p.paces << " paces, max |reflect(one) - two| " << p.maxPos << " in, " << p.maxHead << " rad");
}

// The same on the X-drive rig: strafeTo MOVES here, a moveTo turns while it drives, drive() is holonomic.
TEST_CASE("MIRROR1 T2 X-drive: robot two's path is robot one's reflection pace by pace, every verb, every trigger") {
    const PairRun p = runPair(xdrive(), Pose2d{-36_in, -58_in, 0_deg}, xdriveRoute);
    CHECK(p.one.ok);
    CHECK(p.one.steps == 12);
    CHECK(p.two.steps == p.one.steps);
    CHECK(p.two.completed == p.one.completed);
    CHECK(p.samePaces);
    CHECK(p.paces > 800U);
    CHECK(p.maxPos <= 1e-9);
    CHECK(p.maxHead <= 1e-9);
    CHECK(p.sameFires);
    CHECK(p.sameSeen);
    MESSAGE("x-drive: " << p.paces << " paces, max |reflect(one) - two| " << p.maxPos << " in, " << p.maxHead
                        << " rad");
}

// Bug caught: the trigger check above passing vacuously (nothing fired on either robot).
TEST_CASE("MIRROR1 T2 the triggers really fire, three times, on both robots") {
    Bot one{tank(), Pose2d{-36_in, -58_in, 0_deg}};
    Ctx c{one.chassis, &one.svc};
    (void)tankRoute(c, one.seen);
    REQUIRE_FALSE(one.begun.empty());
    CHECK(one.begun.back()[0] == 2U);  // pivot 6: at 50 % and after 6 in
    CHECK(one.begun.back()[1] == 1U);  // pivot 7: after 30 deg turned
}

// ═══ T3 the seam: F3's exact-180 tie ══════════════════════════════════════════════════════════════

namespace {
struct SeamRun {
    double maxPos = 0.0;
    double endPos = 0.0;
    double twoHeadingAt20 = 0.0;
    double oneHeadingAt20 = 0.0;
    int mirLinesTwo = 0;
    int mirLinesOne = 0;
    std::string mirLine;
};
/// A first turnTo of `deg` from the seated 0 (robot one) — then a drive (the brief's probe P1 C, as a
/// Routine: the drive's bearing is computed from pose(), so robot two's reads robot one's numbers).
SeamRun seam(double deg) {
    SeamRun s;
    const Pose2d start{-36_in, -58_in, 0_deg};
    Bot one{tank(), start};
    Bot two{tank(), rf(start)};
    const auto route = [deg](Chassis& c) {
        Routine r{c, "seam"};
        r.turnTo(Angle::degrees(deg));
        r.driveTo(-60_in, -58_in);
    };
    route(one.chassis);
    {
        const MirroredRun run{two.chassis, kDiag};
        route(two.chassis);
    }
    const std::size_t n = std::min(one.truth.size(), two.truth.size());
    for (std::size_t i = 0; i < n; ++i) {
        const Pose2d w = rf(one.truth[i]);
        s.maxPos = std::max(s.maxPos, std::hypot((w.x() - two.truth[i].x()).value(), (w.y() - two.truth[i].y()).value()));
    }
    const Pose2d w = rf(one.truth.back());
    s.endPos = std::hypot((w.x() - two.truth.back().x()).value(), (w.y() - two.truth.back().y()).value());
    s.oneHeadingAt20 = one.truth[20].heading().degrees();
    s.twoHeadingAt20 = two.truth[20].heading().degrees();
    s.mirLinesOne = one.count("MIR");
    s.mirLinesTwo = two.count("MIR");
    s.mirLine = two.first("MIR");
    return s;
}
}  // namespace

// Bug caught: the exact 180 deg tie "fixed" by changing F3 (LOCKED) or hidden; the [MIR] Warn missing,
// or raised on robot one. Measured (the brief's P1 C): with a first turn of EXACTLY 180 deg both robots
// turn COUNTERCLOCKWISE — robot two is NOT robot one's reflection mid-turn — and the paths stay within
// 0.2 in, the ends within 0.1 in.
TEST_CASE("MIRROR1 T3 an exact 180 deg mirrored turn sweeps counterclockwise on BOTH robots, said once") {
    const SeamRun s = seam(180.0);
    CHECK(s.oneHeadingAt20 > 0.0);           // robot one: CCW from 0
    CHECK(s.twoHeadingAt20 > -90.0);         // robot two: CCW from -90 too (its reflection would be CW)
    CHECK(s.twoHeadingAt20 < 0.0);
    CHECK(s.maxPos > 0.05);                  // it IS a divergence ...
    CHECK(s.maxPos <= 0.2);                  // ... of at most 0.2 in (P1 C: 0.155)
    CHECK(s.endPos <= 0.1);                  // ends within 0.1 in (P1 C: 0.09)
    CHECK(s.mirLinesOne == 0);
    CHECK(s.mirLinesTwo == 1);
    CHECK(s.mirLine
          == "mirrored turnTo turns 180.0 deg: within 1 deg of a half turn, robot two may sweep the same way as "
             "robot one, not the mirror way -- write two 90 deg turns where it matters");
    MESSAGE("exact 180: path within " << s.maxPos << " in, end within " << s.endPos << " in");
}

// Bug caught: the tie's Warn on a turn that is NOT the tie (noise in every log), or a near-180 turn not
// mirrored exactly; the Warn's 1 deg window narrowed or widened. 179.9 and 180.1 deg (= -179.9) are
// mirrored EXACTLY and warned (within 1 deg); so are 179.05 and 180.95 deg (0.95 deg from the half turn,
// 0.05 deg inside the window's edge — a window narrowed below 0.95 deg drops their line); 178.95 and
// 178.9 deg (1.05 and 1.1 deg away, 0.05 deg outside the edge — a window widened past 1.05 deg warns
// them) are mirrored exactly with NO line. The edge itself (exactly 179.0) is not claimed: its
// comparison sits at floating precision.
TEST_CASE("MIRROR1 T3 near-180 turns mirror exactly; warned within 1 deg of the half turn (179.05..180.95), not at 178.95") {
    for (const double deg : {179.9, 180.1, 179.05, 180.95, 178.95, 178.9}) {
        CAPTURE(deg);
        const SeamRun s = seam(deg);
        CHECK(s.maxPos <= 1e-9);
        CHECK(s.mirLinesOne == 0);
        CHECK(s.mirLinesTwo == (deg < 179.0 ? 0 : 1));
    }
}

// Bug caught (the brief's R5): the tie on a holonomic moveTo unsaid. On the X-drive rig a mirrored moveTo
// whose heading change is EXACTLY 180 deg rotates counterclockwise on both robots (MoveToPose's heading
// term uses errorTo, F3's tie). Its divergence was MEASURED first (MIRROR1-PROGRESS.md: the path up to
// 0.307 in from the reflection, the headings up to 3.09 rad — 177 deg — apart mid-leg) and is pinned
// here with its number; the one [MIR] line names the moveTo.
TEST_CASE("MIRROR1 T3 X-drive: an exact 180 deg moveTo heading change rotates CCW on both, said once") {
    const Pose2d start{-36_in, -58_in, 0_deg};
    Bot one{xdrive(), start};
    Bot two{xdrive(), rf(start)};
    const auto route = [](Chassis& c) { (void)c.moveTo(Pose2d{-20_in, -58_in, 180_deg}); };
    route(one.chassis);
    {
        const MirroredRun run{two.chassis, kDiag};
        route(two.chassis);
    }
    const std::size_t n = std::min(one.truth.size(), two.truth.size());
    double maxPos = 0.0;
    double maxHead = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Pose2d w = rf(one.truth[i]);
        maxPos = std::max(maxPos, std::hypot((w.x() - two.truth[i].x()).value(), (w.y() - two.truth[i].y()).value()));
        maxHead = std::max(maxHead, std::fabs(w.heading().errorTo(two.truth[i].heading())));
    }
    CHECK(one.truth[10].heading().degrees() > 0.0);   // robot one rotates CCW from 0
    CHECK(two.truth[10].heading().degrees() > -90.0); // robot two CCW from -90 (its reflection: CW)
    CHECK(maxHead > 3.0);                             // mid-leg the headings are far apart (3.087 rad) ...
    CHECK(maxPos > 0.30);                             // ... and the path, MEASURED 0.307 in, pinned
    CHECK(maxPos < 0.315);
    const Pose2d wEnd = rf(one.truth.back());
    const double endPos = std::hypot((wEnd.x() - two.truth.back().x()).value(), (wEnd.y() - two.truth.back().y()).value());
    MESSAGE("x-drive exact-180 moveTo: end within " << endPos << " in");
    CHECK(one.count("MIR") == 0);
    CHECK(two.count("MIR") == 1);
    CHECK(two.first("MIR")
          == "mirrored moveTo turns 180.0 deg: within 1 deg of a half turn, robot two may sweep the same way as "
             "robot one, not the mirror way -- turn first, in two 90 deg turns, where it matters");
    MESSAGE("x-drive exact-180 moveTo: path within " << maxPos << " in, heading apart up to " << maxHead << " rad");
}

namespace {
/// One route on robot one (unmirrored, from `start`) and on robot two (mirrored, from its reflection), on
/// `kin`: how far robot two's path and end are from robot one's reflection, both ends, the [MIR] lines.
struct TieRun {
    double maxPos = 0.0;
    double endPos = 0.0;
    Pose2d oneEnd{};
    Pose2d twoEnd{};
    double oneHeadingAt10 = 0.0;
    double twoHeadingAt10 = 0.0;
    int mirLinesOne = 0;
    int mirLinesTwo = 0;
    std::string mirLine;
    int notSteeredOne = 0;  ///< TANK1's "tank moveTo not steered ... -- turn first: face()" Warns
    int notSteeredTwo = 0;
};
int notSteered(const Bot& b) {
    int n = 0;
    for (int i = 0; i < b.sink.size(); ++i) {
        n += b.sink.at(i).subsystem == "MOT" && b.sink.at(i).message.find("turn first: face()") != std::string::npos ? 1 : 0;
    }
    return n;
}
TieRun tieRun(const shulib::kinematics::IKinematics& kin, const std::function<void(Chassis&)>& route) {
    TieRun s;
    const Pose2d start{-36_in, -58_in, 0_deg};
    Bot one{kin, start};
    Bot two{kin, rf(start)};
    route(one.chassis);
    {
        const MirroredRun run{two.chassis, kDiag};
        route(two.chassis);
    }
    const std::size_t n = std::min(one.truth.size(), two.truth.size());
    for (std::size_t i = 0; i < n; ++i) {
        const Pose2d w = rf(one.truth[i]);
        s.maxPos = std::max(s.maxPos, std::hypot((w.x() - two.truth[i].x()).value(), (w.y() - two.truth[i].y()).value()));
    }
    s.oneEnd = one.truth.back();
    s.twoEnd = two.truth.back();
    const Pose2d w = rf(s.oneEnd);
    s.endPos = std::hypot((w.x() - s.twoEnd.x()).value(), (w.y() - s.twoEnd.y()).value());
    s.oneHeadingAt10 = one.truth[10].heading().degrees();
    s.twoHeadingAt10 = two.truth[10].heading().degrees();
    s.mirLinesOne = one.count("MIR");
    s.mirLinesTwo = two.count("MIR");
    s.mirLine = two.first("MIR");
    s.notSteeredOne = notSteered(one);
    s.notSteeredTwo = notSteered(two);
    return s;
}
}  // namespace

// Bug caught (a review finding, 2026-11-13): the tie told as "a holonomic drive only, a fraction of an
// inch". On a TANK — the team's drivetrain — a mirrored moveTo (and a Routine's driveTo, which IS a
// moveTo) whose heading change is EXACTLY 180 deg also rotates counterclockwise on both robots; a tank
// translates only along its heading, so the sweep's direction moves where it ENDS. MEASURED first
// (MIRROR1-PROGRESS.md, the review section) and pinned with its numbers: a moveTo to (-20, -58) facing 180
// from (-36, -58) facing 0, and a driveTo to a point directly behind, each 4 s. And the advice the
// [MIR] line gives — turn first, in two 90 deg turns — mirrored EXACTLY, with no line.
// Bug caught (a verification finding, 2026-11-14): the docs told the 16 in leg's 5.7 in as the effect's
// SIZE ("up to 5.7 in", "a few inches"). It GROWS WITH THE LEG — robot one ends beside its target (the
// leg is unsteered: it slides sideways while it sweeps), robot two, sweeping the same way, as far beside
// its own on the other side of the mirror, so the two misses ADD. A 36 in leg is pinned beside the 16 in
// one (a scan of 4-60 in legs is in the log), with its controls and the 2x relation.
TEST_CASE("MIRROR1 T3 tank: an exact 180 deg moveTo / driveTo ENDS off the mirror by more the longer the leg, said once; turning first mirrors exactly") {
    const std::string legLine =
        "mirrored moveTo turns 180.0 deg: within 1 deg of a half turn, robot two may sweep the same way as robot "
        "one, not the mirror way -- turn first, in two 90 deg turns, where it matters";
    const TieRun mv = tieRun(tank(), [](Chassis& c) { (void)c.moveTo(Pose2d{-20_in, -58_in, 180_deg}, {.timeout = 4_s}); });
    MESSAGE("tank exact-180 moveTo: path within " << mv.maxPos << " in, end " << mv.endPos << " in; robot one ends ("
                                                 << mv.oneEnd.x().value() << ", " << mv.oneEnd.y().value() << ")");
    CHECK(mv.oneHeadingAt10 > 0.0);     // robot one sweeps CCW from 0 (29.4 deg at pace 10) ...
    CHECK(mv.twoHeadingAt10 > -90.0);   // ... and robot two CCW from -90 too (-60.6; its mirror: -119.4, CW)
    CHECK(mv.endPos > 5.65);            // MEASURED 5.705 in at the END (the path's worst is its end)
    CHECK(mv.endPos < 5.75);
    CHECK(mv.mirLinesOne == 0);
    CHECK(mv.mirLinesTwo == 1);
    CHECK(mv.mirLine == legLine);
    CHECK(mv.notSteeredOne == 1);       // the leg is already TANK1's unsteered one, on both robots
    CHECK(mv.notSteeredTwo == 1);
    // the SAME leg 36 in long, to (0, -58) facing 180: the end lands 2.5x as far off the mirror
    const TieRun lg = tieRun(tank(), [](Chassis& c) { (void)c.moveTo(Pose2d{0_in, -58_in, 180_deg}, {.timeout = 6_s}); });
    const double missOne16 = std::hypot(mv.oneEnd.x().value() + 20.0, mv.oneEnd.y().value() + 58.0);
    const double missOne36 = std::hypot(lg.oneEnd.x().value(), lg.oneEnd.y().value() + 58.0);
    MESSAGE("tank exact-180 moveTo, 36 in: path within " << lg.maxPos << " in, end " << lg.endPos << " in; robot one ends ("
                                                        << lg.oneEnd.x().value() << ", " << lg.oneEnd.y().value()
                                                        << "); robot one's miss 16 in " << missOne16 << ", 36 in " << missOne36);
    CHECK(lg.endPos > 14.15);           // MEASURED 14.199 in — 2.5x the 16 in leg's, on 2.25x the leg
    CHECK(lg.endPos < 14.25);
    CHECK(lg.mirLinesOne == 0);
    CHECK(lg.mirLinesTwo == 1);
    CHECK(lg.mirLine == legLine);
    // why it grows: robot one's own miss of its target (it slid sideways while it swept) — robot two's is
    // the same size on the other side, so the end lies TWICE robot one's miss off the mirror
    MESSAGE("end - 2 x robot one's miss: 16 in " << mv.endPos - 2.0 * missOne16 << ", 36 in " << lg.endPos - 2.0 * missOne36);
    CHECK(std::abs(mv.endPos - 2.0 * missOne16) < 1e-6);  // MEASURED 4.4e-8
    CHECK(std::abs(lg.endPos - 2.0 * missOne36) < 1e-6);  // MEASURED 2.8e-13
    // a Routine's driveTo to a point directly BEHIND is the same leg (a moveTo whose heading is the bearing)
    const TieRun dv = tieRun(tank(), [](Chassis& c) {
        Routine r{c, "behind"};
        r.driveTo(-50_in, -58_in, {.timeout = 4_s});
    });
    MESSAGE("tank driveTo directly behind: path within " << dv.maxPos << " in, end " << dv.endPos << " in");
    CHECK(dv.endPos > 4.94);            // MEASURED 4.988 in
    CHECK(dv.endPos < 5.04);
    CHECK(dv.mirLinesTwo == 1);
    CHECK(dv.mirLine == legLine);       // the line names the moveTo the driveTo runs
    // the advice: turn first, in two 90 deg turns, then the same drive — mirrored EXACTLY, no line
    const TieRun adv = tieRun(tank(), [](Chassis& c) {
        Routine r{c, "turn first"};
        r.turnTo(90_deg);
        r.turnTo(180_deg);
        r.driveTo(-50_in, -58_in, {.timeout = 4_s});
    });
    MESSAGE("tank, turned first in two 90 deg turns: path within " << adv.maxPos << " in, end " << adv.endPos << " in");
    CHECK(adv.maxPos <= 1e-9);
    CHECK(adv.mirLinesTwo == 0);
    CHECK(adv.notSteeredTwo == 0);
    // Why it matters on a robot: robot one ALONE, its target 0.5 deg either side of the half turn (no tie —
    // each sweeps the short way, one each way): where it ENDS moves by as much as the tie's divergence,
    // growing with the leg the same way. The IMU's noise makes such a leg that coin toss for each robot.
    const TieRun ccw = tieRun(tank(), [](Chassis& c) { (void)c.moveTo(Pose2d{-20_in, -58_in, 179.5_deg}, {.timeout = 4_s}); });
    const TieRun cw = tieRun(tank(), [](Chassis& c) { (void)c.moveTo(Pose2d{-20_in, -58_in, -179.5_deg}, {.timeout = 4_s}); });
    const double coin = std::hypot((ccw.oneEnd.x() - cw.oneEnd.x()).value(), (ccw.oneEnd.y() - cw.oneEnd.y()).value());
    MESSAGE("robot one's end, 179.5 vs -179.5 deg: " << coin << " in apart; mirrored to " << ccw.maxPos << " / " << cw.maxPos);
    // 0.5 deg off the tie each leg mirrors EXACTLY — so the tie alone causes the inches above (pinned at the
    // review's resumption, 2026-11-14: the controls were printed, not asserted) — and robot one's END moves
    // by what guide chapter 17 calls "as large as the tie's" — on the 16 in leg and on the 36 in one.
    CHECK(ccw.maxPos <= 1e-9);
    CHECK(cw.maxPos <= 1e-9);
    CHECK(coin > 5.78);  // MEASURED 5.831 in
    CHECK(coin < 5.88);
    const TieRun ccw36 = tieRun(tank(), [](Chassis& c) { (void)c.moveTo(Pose2d{0_in, -58_in, 179.5_deg}, {.timeout = 6_s}); });
    const TieRun cw36 = tieRun(tank(), [](Chassis& c) { (void)c.moveTo(Pose2d{0_in, -58_in, -179.5_deg}, {.timeout = 6_s}); });
    const double coin36 =
        std::hypot((ccw36.oneEnd.x() - cw36.oneEnd.x()).value(), (ccw36.oneEnd.y() - cw36.oneEnd.y()).value());
    MESSAGE("36 in: robot one's end, 179.5 vs -179.5 deg: " << coin36 << " in apart; mirrored to " << ccw36.maxPos << " / "
                                                            << cw36.maxPos);
    CHECK(ccw36.maxPos <= 1e-9);
    CHECK(cw36.maxPos <= 1e-9);
    CHECK(coin36 > 14.48);  // MEASURED 14.530 in
    CHECK(coin36 < 14.58);
}

// ═══ T4 the scope (its runner half — every way out of runAuton — is mirrored_auton_test.cpp's) ═════════

// Bug caught: MM3 (the destructor emptied — the mirror outliving its scope and steering the next run
// reflected); a nested run silently replacing the first; a run opened over a motion (its target
// reflected mid-flight); the identity accepted as a "mirror".
TEST_CASE("MIRROR1 T4 MirroredRun: on for its lifetime, off after it, refuses nesting, a live motion, the identity") {
    Bot b{tank(), Pose2d{-36_in, -58_in, 0_deg}};
    CHECK(b.chassis.reflection().isIdentity());
    const Pose2d real = b.rig.loc.pose();
    {
        const MirroredRun run{b.chassis, kDiag};
        CHECK(b.chassis.reflection() == kDiag);
        CHECK(b.chassis.pose().approxEqual(rf(real), Length{1e-12}, 1e-12));
        CHECK_THROWS_AS((void)MirroredRun(b.chassis, kDiag), shulib::PreconditionError);
        CHECK(b.chassis.reflection() == kDiag);  // the refused second one changed nothing
    }
    CHECK(b.chassis.reflection().isIdentity());
    CHECK(std::bit_cast<std::uint64_t>(b.chassis.pose().x().value())
          == std::bit_cast<std::uint64_t>(b.rig.loc.pose().x().value()));
    CHECK_THROWS_AS((void)MirroredRun(b.chassis, Reflection{}), shulib::PreconditionError);
    // a motion active (a Tier-3 async one): refused, nothing changed
    shulib::motion::MoveToPose m{b.chassis.deps(), Pose2d{-20_in, -58_in, 0_deg}, b.chassis.motionConfig(), 2.0};
    b.chassis.scheduler().async(m);
    CHECK_THROWS_AS((void)MirroredRun(b.chassis, kDiag), shulib::PreconditionError);
    CHECK(b.chassis.reflection().isIdentity());
    b.chassis.cancel();
    // a throw through the scope closes it
    CHECK_THROWS_AS(([&] {
                        const MirroredRun run{b.chassis, kDiag};
                        throw std::runtime_error("a routine threw");
                    }()),
                    std::runtime_error);
    CHECK(b.chassis.reflection().isIdentity());
}

// Bug caught: a mirrored curve's points copied wrong — a count outside 1..16 must still reach the
// curve's OWN precondition (not be swallowed or reflected into garbage), and 16 points must pass.
// (That the 16 are REFLECTED is the next case's.)
TEST_CASE("MIRROR1 T4 a mirrored curve: 0 and 17 points refused by the curve's own precondition, 16 accepted") {
    Bot b{tank(), Pose2d{-36_in, -58_in, 0_deg}};
    const MirroredRun run{b.chassis, kDiag};
    std::vector<shulib::chassis::CurvePoint> pts;
    CHECK_THROWS_WITH_AS((void)b.chassis.curveTo(std::span<const shulib::chassis::CurvePoint>{pts}),
                         doctest::Contains("1 to 16 points"), shulib::PreconditionError);
    for (int i = 0; i < 17; ++i) {
        pts.emplace_back(Length{-34.0 + 1.5 * i}, Length{-58.0});
    }
    CHECK_THROWS_WITH_AS((void)b.chassis.curveTo(std::span<const shulib::chassis::CurvePoint>{pts}),
                         doctest::Contains("1 to 16 points"), shulib::PreconditionError);
    pts.pop_back();
    CHECK_NOTHROW((void)b.chassis.curveTo(std::span<const shulib::chassis::CurvePoint>{pts}, {.timeout = 3_s}));
    // a NaN point is still the curve's own refusal, through the mirror
    std::vector<shulib::chassis::CurvePoint> nan{{Length{std::nan("")}, Length{0.0}}};
    CHECK_THROWS_WITH_AS((void)b.chassis.curveTo(std::span<const shulib::chassis::CurvePoint>{nan}),
                         doctest::Contains("finite"), shulib::PreconditionError);
}

// Bug caught (a review finding, 2026-11-13 — the mutant survived): the copy's bound off by one at its
// EDGE, `points.size() < kMaxCurvePoints` — a 16-point curve (a team's longest) passed through
// UNREFLECTED, robot two driving robot one's numbers. Exactly 16 points, a stated heading on the last,
// run on robot one (unmirrored) and robot two (mirrored, from the reflected start): robot two's path is
// robot one's reflection pace by pace (1e-9), and robot two ENDS at the reflection of the last point,
// nowhere near the point itself.
TEST_CASE("MIRROR1 T4 a mirrored 16-point curve (the most a curve takes) IS reflected: the path pace by pace") {
    std::vector<shulib::chassis::CurvePoint> pts;
    for (int i = 0; i < 15; ++i) {
        const double x = -34.0 + 1.5 * i;
        pts.emplace_back(Length{x}, Length{-58.0 + 0.02 * (x + 34.0) * (x + 34.0)});
    }
    pts.emplace_back(Length{-10.0}, Length{-46.0}, Angle::degrees(40.0));
    REQUIRE(pts.size() == shulib::motion::kMaxCurvePoints);
    const auto route = [&pts](Chassis& c) {
        (void)c.curveTo(std::span<const shulib::chassis::CurvePoint>{pts}, {.timeout = 6_s});
    };
    const TieRun r = tieRun(tank(), route);
    MESSAGE("16-point curve: path within " << r.maxPos << " in of the reflection; robot two ends (" << r.twoEnd.x().value()
                                           << ", " << r.twoEnd.y().value() << ")");
    CHECK(r.maxPos <= 1e-9);
    // robot two at the REFLECTION of the curve's end (46, 10), not at the end (-10, -46) itself
    CHECK(std::hypot(r.twoEnd.x().value() - 46.0, r.twoEnd.y().value() - 10.0) < 3.0);
    CHECK(std::hypot(r.twoEnd.x().value() + 10.0, r.twoEnd.y().value() + 46.0) > 50.0);
    CHECK(r.mirLinesTwo == 0);
}

// ═══ T4 the driver loop: a leaked mirror is CLEARED, said, never refused (the brief's R12) ════════════

namespace {
/// A controller holding the left stick half forward and the right stick a quarter right.
struct HeldSticks final : shulib::hal::IController {
    [[nodiscard]] double axis(shulib::hal::ControllerAxis a) const override {
        switch (a) {
            case shulib::hal::ControllerAxis::LeftY: return 0.5;
            case shulib::hal::ControllerAxis::RightX: return 0.25;
            default: return 0.0;
        }
    }
    [[nodiscard]] bool pressed(shulib::hal::ControllerButton) const override { return false; }
    [[nodiscard]] bool isConnected() const override { return true; }
};
struct LoopTick {
    double left = 0.0;
    double right = 0.0;
    bool identity = false;
};
/// Five ticks of the driver loop on `c` (ArcadeBody, the default profile).
std::vector<LoopTick> fiveTicks(motion_rig::ChassisRig& c) {
    HeldSticks pad;
    shulib::teleop::DriveShaper shaper{shulib::teleop::DriverProfile{}};
    std::vector<LoopTick> log;
    int k = 0;
    shulib::teleop::runDriverControl(
        c.chassis, pad, c.pacer, shaper,
        shulib::teleop::DriverLoopConfig{.tickPeriod = Time{0.01}, .rezeroButton = shulib::hal::ControllerButton::A},
        [&] { return k >= 5; }, [] { return shulib::teleop::HeadingSample{.radians = 0.0, .trusted = true}; },
        [&](const ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
            log.push_back(LoopTick{c.rig.h.motor(0).commandedVoltage().value(),
                                   c.rig.h.motor(1).commandedVoltage().value(), c.chassis.reflection().isIdentity()});
            ++k;
        });
    return log;
}
/// The first drive() record's FIELD command after record `from`.
ChassisSpeeds firstDriveCommand(const shulib::hal::fake::FakeTelemetrySink& sink, int from) {
    for (int i = from; i < sink.recordCount(); ++i) {
        if (sink.recordAt(i).activeCommandId == 0) {
            return sink.recordAt(i).commanded;
        }
    }
    return ChassisSpeeds{};
}
}  // namespace

// Bug caught: a mirror left on at driver control steering the driver REVERSED (a body command's turn
// negated) — or, the first print's design, REFUSED, leaving no drive for the whole period. The loop
// handed a Chassis whose mirror is on logs ONE [MIR] Warn, CLEARS it, and its first tick commands
// exactly what a fresh unmirrored loop commands — the volts and the record's FIELD command, bit for bit.
TEST_CASE("MIRROR1 T4 the driver loop CLEARS a leaked mirror, says so once, and drives the real field") {
    shulib::hal::fake::FakeTelemetrySink sinkA;
    shulib::hal::fake::FakeTelemetrySink sinkB;
    motion_rig::ChassisRig a{tank(), motion_rig::plantConfig(), &sinkA};
    motion_rig::ChassisRig b{tank(), motion_rig::plantConfig(), &sinkB};
    std::optional<MirroredRun> leaked;
    leaked.emplace(a.chassis, kDiag);  // the defect: a run that outlived its auton
    REQUIRE_FALSE(a.chassis.reflection().isIdentity());
    const int fromA = sinkA.recordCount();
    const int fromB = sinkB.recordCount();
    const std::vector<LoopTick> la = fiveTicks(a);
    const std::vector<LoopTick> lb = fiveTicks(b);
    REQUIRE(la.size() == 5U);
    REQUIRE(lb.size() == 5U);
    CHECK(la[0].identity);
    CHECK(std::bit_cast<std::uint64_t>(la[0].left) == std::bit_cast<std::uint64_t>(lb[0].left));
    CHECK(std::bit_cast<std::uint64_t>(la[0].right) == std::bit_cast<std::uint64_t>(lb[0].right));
    CHECK(la[0].left != la[0].right);  // the turn is in it: a negated turn would swap the two sides
    const ChassisSpeeds ca = firstDriveCommand(sinkA, fromA);
    const ChassisSpeeds cb = firstDriveCommand(sinkB, fromB);
    CHECK(std::bit_cast<std::uint64_t>(ca.vx().value()) == std::bit_cast<std::uint64_t>(cb.vx().value()));
    CHECK(std::bit_cast<std::uint64_t>(ca.vy().value()) == std::bit_cast<std::uint64_t>(cb.vy().value()));
    CHECK(std::bit_cast<std::uint64_t>(ca.omega().value()) == std::bit_cast<std::uint64_t>(cb.omega().value()));
    int mirA = 0;
    std::string line;
    for (int i = 0; i < sinkA.size(); ++i) {
        if (sinkA.at(i).subsystem == "MIR") {
            ++mirA;
            line = sinkA.at(i).message;
            CHECK(sinkA.at(i).level == shulib::hal::LogLevel::Warn);
        }
    }
    CHECK(mirA == 1);
    CHECK(line
          == "an auton's mirror was still on at driver control -- cleared; the driver drives the real field (a "
             "library defect: please report it)");
    for (int i = 0; i < sinkB.size(); ++i) {
        CHECK(sinkB.at(i).subsystem != "MIR");
    }
}
