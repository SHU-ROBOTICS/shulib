#pragma once
//
// mirror_unchanged_runs.hpp — MIRROR1-A's T10 fixture: "NOTHING ELSE CHANGES" at the RUNNER's level
// (the MIRROR1 brief's §2.2 T10, the CV16 method of test/curve_unchanged_route.hpp one layer up). Three
// autons, ALL OWNED BY THIS FILE: a copy of the 40 in auton as the team's file had it at the merge base
// (a RELATIVE MATCH auton, one moveTo), the library's acceptance run (competition::acceptanceRunEntry,
// RELATIVE MATCH), and one FIELD-start SKILLS auton of this file's own — every Chassis verb a route can
// write (driveTo, face, turnTo, moveTo, faceAway, reverseTo, curveTo, reverseCurveTo, followTrajectory,
// strafeTo, hold, pause, a waitFor reading pose(), a then() calling drive() in both frames and setPose(),
// a partway trigger of each kind on both observer overloads) with a park end action — each run through
// competition::runAuton on the host test plant, on a tank with one side 9 % weak (TANK1's steering
// live) and on an X-drive, over a mechanism list of this file's own (a copy of robot two's at the merge
// base: two claw pivots and a pneumatic claw, on fakes). Everything a run produces — every DebugRecord
// through the blackbox encoder's bytes, every log line, the true pose after every pace, the result's
// tag, seat and counts — folded into one 64-bit FNV-1a hash per drivetrain. The two hashes were
// MEASURED with this exact header compiled against MIRROR1-A's MERGE BASE (5f6633b, a clean `git
// archive` — MIRROR1-PROGRESS.md), and mirrored_auton_test.cpp pins them: an UNMIRRORED run may not
// move one byte.
//
// WHY NOTHING HERE IS THE TEAM'S (a review finding, 2026-11-13): the first draft ran robot two's REAL
// registry (src/team/autons.cpp) over robot two's REAL mechanism list (src/robots/mechanism_table.hpp),
// so a student re-tuning the 40 in auton, a third auton, robot two becoming robot one's build (ROB1-BT)
// or robot two's mirrored line (D2) would each have turned a LIBRARY test red. The pin is the library's
// unmirrored path; the routes and the list it runs are this file's, frozen as the base had them. Change
// this file and the hashes move: re-measure on a clean archive of the base, never against the tree.
// Self-contained on purpose (no MIRROR1 header, no src/ file is included — not even through a rig:
// the weak side below is a COPY of test/auton_tester_rig.hpp's WeakSide, because that rig includes
// robots/mechanism_table.hpp and two programs/ headers; a verification finding, 2026-11-14), so the same
// file compiles against the base and a change to robot two's list cannot reach it (checked by compiling
// this header with src/ OFF the include path: MIRROR1-PROGRESS.md, the verification round). Everything
// here is the host test plant; nothing here has run on a robot.

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string_view>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/diag/blackbox_format.hpp"
#include "shulib/hal/digital_out.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "routine_done_line.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/mechanism_routes.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

namespace mirror_t10 {

// ── this file's mechanisms: a COPY of robot two's list at the merge base (file banner) ─────────────────
namespace mech {
using namespace shulib::mechanism::words;
inline constexpr Power clawPivot6{.name = "claw pivot 6", .motors = {Forward}, .cartridge = Red, .safe = Hold};
inline constexpr Power clawPivot7{.name = "claw pivot 7", .motors = {Forward}, .cartridge = Red, .safe = Hold};
inline constexpr Pneumatic claw{
    .name = "claw",
    .positions = {{"closed", {Low}, grabs}, {"open", {High}}},
    .start = "closed",
    .safe = "closed",
    .moveTime = 0.5_s,
};
inline constexpr shulib::mechanism::MechanismTable kMechanisms =
    mechanisms(wire(clawPivot6, motors(6)), wire(clawPivot7, motors(7)), wire(claw, lines('A')));
}  // namespace mech

/// The periods the autons run in (the program's constants: a 30 s MATCH, a 60 s SKILLS run).
inline constexpr shulib::competition::AutonPeriods kPeriods{.match = shulib::units::Time{30.0},
                                                            .skills = shulib::units::Time{60.0}};

/// mech::kMechanisms on fakes, in its order: pivot 6, pivot 7, the claw (its start level, low).
struct Fakes {
    shulib::hal::fake::FakeMotor m6;
    shulib::hal::fake::FakeMotor m7;
    shulib::hal::IMotor* s6[1]{&m6};
    shulib::hal::IMotor* s7[1]{&m7};
    shulib::hal::MotorMechanism pivot6{s6, shulib::hal::BrakeMode::Hold, "claw pivot 6"};
    shulib::hal::MotorMechanism pivot7{s7, shulib::hal::BrakeMode::Hold, "claw pivot 7"};
    shulib::hal::fake::FakeDigitalOut lineA;
    shulib::hal::IDigitalOut* lines[1]{&lineA};
    shulib::hal::PneumaticMechanism claw{lines, false, "claw"};
    shulib::mechanism::Devices devs[3]{{.motors = &pivot6}, {.motors = &pivot7}, {.pneumatic = &claw}};
};

/// One side's volts scaled: wheel 0 = LEFT, 1 = RIGHT (the tank's kinematic order) — a COPY of
/// test/auton_tester_rig.hpp's tester_rig::WeakSide as the base had it (file banner: that rig includes
/// src/ files). A weak side turns a tank toward it: TANK1's steering live.
struct WeakSide final : shulib::sim::DegradationModel {
    int wheel = 1;
    double scale = 1.0;
    shulib::units::Voltage effectiveVoltage(int w, shulib::units::Voltage commanded, shulib::units::Time,
                                            shulib::sim::Rng&) override {
        return w == wheel ? shulib::units::Voltage{commanded.value() * scale} : commanded;
    }
};

/// FNV-1a, 64-bit (curve_cv16's).
struct Fnv {
    std::uint64_t h = 1469598103934665603ULL;
    void bytes(const void* p, std::size_t n) {
        const auto* b = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    }
    void text(std::string_view s) { bytes(s.data(), s.size()); }
    void num(double v) { bytes(&v, sizeof v); }
};

/// The world below the service: steps the plant and folds the TRUE pose into the hash after every pace.
struct HashWorld final : shulib::motion::ITickPacer {
    shulib::sim::SimHarness* h = nullptr;
    Fnv* fnv = nullptr;
    long paces = 0;
    void pace() override {
        if (++paces > 400000) {
            throw std::runtime_error("mirror_t10::HashWorld: pace cap exceeded");
        }
        h->plant().step(shulib::units::Time{0.01});
        const shulib::math::Pose2d t = h->truePose();
        fnv->num(t.x().value());
        fnv->num(t.y().value());
        fnv->num(t.heading().radians());
    }
};

/// The program's chain, as competition_program.cpp builds it, without the watch:
///     Chassis -> RunGuard -> MechanismService -> HashWorld
/// over this file's mechanism list on fakes (file banner) and its route ids.
struct Stack {
    WeakSide weak;
    shulib::hal::fake::FakeTelemetrySink sink;
    motion_rig::MotionRig rig;
    HashWorld world;
    Fakes two;
    shulib::mechanism::MechanismService svc;
    shulib::sequence::RunGuard guard;
    shulib::chassis::Chassis chassis;
    shulib::runner::MechanismRoutes routes;
    shulib::runner::CommandRegistry commands;

    static shulib::sim::SimHarnessConfig config(const shulib::math::Pose2d& start) {
        shulib::sim::SimHarnessConfig c = motion_rig::plantConfig();
        c.plant.initialPose = start;
        return c;
    }
    Stack(const shulib::kinematics::IKinematics& kin, double weakScale, const shulib::math::Pose2d& start, Fnv& fnv)
        : rig{kin, config(start), &sink, &weak},
          svc{world},
          guard{svc},
          chassis{rig.deps, guard, motion_rig::chassisConfig()},
          routes{chassis, svc, mech::kMechanisms, sink} {
        weak.wheel = 1;
        weak.scale = weakScale;
        world.h = &rig.h;
        world.fnv = &fnv;
        svc.bind(mech::kMechanisms, two.devs, rig.h.clock(), rig.latch, sink);
        routes.registerInto(commands);
        for (int i = 0; i < 30; ++i) {
            chassis.wait(shulib::units::Time{0.01});  // the estimate live, nothing commanded
        }
    }
};

/// Fold everything `s` produced (records as blackbox bytes, every log line) into `fnv`.
inline void foldSink(const shulib::hal::fake::FakeTelemetrySink& sink, Fnv& fnv) {
    namespace bb = shulib::diag::blackbox;
    std::byte buf[bb::kTickPayloadBytes];
    for (int i = 0; i < sink.recordCount(); ++i) {
        const std::size_t n = bb::encodeTick(std::span<std::byte>{buf}, sink.recordAt(i));
        fnv.bytes(buf, n);
    }
    for (int i = 0; i < sink.size(); ++i) {
        if (routine_d4::isStepDoneLine(sink.at(i).level, sink.at(i).subsystem, sink.at(i).message)) {
            continue;  // RMP1-K D4's line: after this golden's base (routine_done_line.hpp)
        }
        fnv.num(static_cast<double>(static_cast<int>(sink.at(i).level)));
        fnv.text(sink.at(i).subsystem);
        fnv.text(sink.at(i).message);
    }
}

/// Fold a run's result: the tag, the seat, the routine's counts, the guard's flags.
inline void foldResult(const shulib::competition::AutonRunResult& r, Fnv& fnv) {
    fnv.num(static_cast<double>(static_cast<int>(r.outcome)));
    fnv.text(std::string_view{r.why});
    fnv.num(r.seated.x().value());
    fnv.num(r.seated.y().value());
    fnv.num(r.seated.heading().radians());
    fnv.num(static_cast<double>(r.routine.steps));
    fnv.num(static_cast<double>(r.routine.completed));
    fnv.num(static_cast<double>(r.routine.stoppedAt));
    fnv.num(r.guard.endActionRan ? 1.0 : 0.0);
    fnv.num(r.guard.endActionSucceeded ? 1.0 : 0.0);
    fnv.num(r.guard.scoringCut ? 1.0 : 0.0);
    fnv.num(static_cast<double>(r.guard.pacesSeen));
}

// ── the autons (file banner): the 40 in auton's copy, and the FIELD auton (every verb) ────────────────

/// THE 40 IN AUTON, as src/team/autons.cpp had it at the merge base: one moveTo 40 in ahead, facing 0.
inline shulib::chassis::RoutineResult fortyInchesForward(shulib::competition::AutonContext& ctx) {
    using namespace shulib::units::literals;
    shulib::chassis::Routine r{ctx, "40in-forward"};
    r.moveTo(shulib::math::Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}

/// Its entry, as registered at the merge base (a RELATIVE MATCH auton, park at 27 s, stop at 29 s).
inline shulib::competition::AutonEntry fortyInchEntry() {
    using namespace shulib::units::literals;
    shulib::competition::AutonEntry e{};
    e.name = "40in forward";
    e.kind = shulib::competition::AutonKind::Match;
    e.routine = &fortyInchesForward;
    e.start = shulib::competition::StartPose::relative();
    e.endActionAt = 27_s;
    e.hardStopAt = 29_s;
    return e;
}

/// A FIELD start in the red wall's Quadrant (the MIRROR1 brief's probe start).
inline shulib::math::Pose2d fieldStart() {
    using namespace shulib::units::literals;
    return shulib::math::Pose2d{-36_in, -58_in, shulib::math::Angle{}};
}

/// Every verb a route can write, through the Routine and the Chassis (file banner).
inline shulib::chassis::RoutineResult everyVerb(shulib::competition::AutonContext& ctx) {
    using namespace shulib::units::literals;
    using shulib::math::Angle;
    using shulib::math::Pose2d;
    using namespace mech;
    shulib::chassis::Routine r{ctx, "every verb"};
    r.atPercent(50).startRun(clawPivot6, shulib::units::Voltage{6.0}, 0.3_s).driveTo(-12_in, -58_in);
    r.afterTurning(30_deg).startRun(clawPivot7, shulib::units::Voltage{6.0}, 0.3_s).face(-12_in, -40_in);
    r.afterDistance(6_in).startRun(clawPivot6, shulib::units::Voltage{-6.0}, 0.2_s).driveTo(-12_in, -40_in);
    r.turnTo(45_deg);
    r.moveTo(Pose2d{-2_in, -30_in, 45_deg});
    r.faceAway(-24_in, -36_in);
    r.reverseTo(-24_in, -36_in);
    // carryOn from here: a leg the weak side cannot finish must not skip the verbs after it (the hash
    // covers every verb's path, whatever its verdict).
    using shulib::chassis::carryOn;
    r.onFail(carryOn).curveTo({{-6_in, -26_in}, {12_in, -32_in, -30_deg}});
    r.onFail(carryOn).reverseCurveTo({{-6_in, -44_in}, {-24_in, -54_in, 0_deg}});
    r.onFail(carryOn).followTrajectory({Pose2d{-10_in, -54_in, 0_deg}, Pose2d{2_in, -54_in, 0_deg}});
    r.onFail(carryOn).strafeTo(2_in, -50_in, {.timeout = 1.5_s});
    // -0.0 written into a heading and a position on purpose: an identity that is "applied" (re-wrapped,
    // + 0.0) instead of branched around turns them into +0.0, and the records' bytes show it (MM12).
    r.onFail(carryOn).turnTo(shulib::math::Angle::radians(-0.0));
    r.onFail(carryOn).moveTo(Pose2d{shulib::units::Length{-0.0}, -50_in, shulib::math::Angle::radians(-0.0)});
    r.onFail(carryOn).waitFor([&ctx] { return ctx.chassis.pose().x().value() > -100.0; }, 0.5_s, "pose read");
    r.onFail(carryOn).then(
        [&ctx] {
            for (int i = 0; i < 20; ++i) {
                ctx.chassis.drive(shulib::math::ChassisSpeeds{shulib::units::Velocity{6.0},
                                                              shulib::units::Velocity{4.0},
                                                              shulib::units::AngularVelocity{0.5}},
                                  shulib::math::Frame::Field);
                (void)ctx.guard.pause(shulib::units::Time{0.01});
            }
            for (int i = 0; i < 20; ++i) {
                ctx.chassis.drive(shulib::math::ChassisSpeeds{shulib::units::Velocity{-6.0},
                                                              shulib::units::Velocity{3.0},
                                                              shulib::units::AngularVelocity{-0.5}},
                                  shulib::math::Frame::Body);
                (void)ctx.guard.pause(shulib::units::Time{0.01});
            }
            ctx.chassis.setPose(ctx.chassis.pose());
            return ctx.chassis.brake() == shulib::control::ExitReason::Settled;
        },
        "drive both frames");
    r.onFail(carryOn).face(10_in, -44_in);
    r.onFail(carryOn).hold(300_ms);
    r.pause(200_ms);
    r.onFail(carryOn).driveTo(10_in, -44_in);
    return r.result();
}

/// The park: back to the start's line.
inline bool parkAtStart(shulib::competition::AutonContext& ctx) {
    using namespace shulib::units::literals;
    return ctx.chassis.moveTo(shulib::math::Pose2d{-20_in, -52_in, 0_deg}, {.timeout = 3_s})
           == shulib::control::ExitReason::Settled;
}

/// The FIELD entry (a SKILLS run: 60 s, park at 55 s).
inline shulib::competition::AutonEntry everyVerbEntry() {
    using namespace shulib::units::literals;
    shulib::competition::AutonEntry e{};
    e.name = "every verb";
    e.kind = shulib::competition::AutonKind::Skills;
    e.routine = &everyVerb;
    e.start = shulib::competition::StartPose::field(fieldStart());
    e.endActionAt = 55_s;
    e.hardStopAt = 58_s;
    e.endAction = &parkAtStart;
    return e;
}

/// One run of `entry` on a fresh stack from `start`, folded into `fnv`.
inline void runOne(const shulib::kinematics::IKinematics& kin, double weakScale,
                   const shulib::competition::AutonEntry& entry, const shulib::math::Pose2d& start,
                   shulib::units::Time period, Fnv& fnv) {
    Stack s{kin, weakScale, start, fnv};
    const shulib::competition::AutonRunResult r =
        shulib::competition::runAuton(s.chassis, s.guard, entry, &s.commands, s.sink, s.svc, period);
    foldResult(r, fnv);
    foldSink(s.sink, fnv);
}

/// T10's hash on `kin`: the two RELATIVE autons (the 40 in auton's copy, the acceptance run) from the
/// origin, through a registry (its own checks run, as the program's do), then the FIELD auton from its
/// start (file banner).
inline std::uint64_t runnerHash(const shulib::kinematics::IKinematics& kin, double weakScale) {
    using namespace shulib::units::literals;
    Fnv fnv;
    shulib::competition::AutonRegistry reg{kPeriods};
    reg.add(fortyInchEntry());
    reg.add(shulib::competition::acceptanceRunEntry(shulib::competition::AutonKind::Match, 27_s, 29_s));
    for (std::size_t i = 0; i < reg.size(); ++i) {
        runOne(kin, weakScale, reg.at(i), shulib::math::Pose2d{}, reg.periodFor(reg.at(i)), fnv);
    }
    const shulib::competition::AutonEntry field = everyVerbEntry();
    runOne(kin, weakScale, field, fieldStart(), kPeriods.skills, fnv);
    fnv.num(static_cast<double>(reg.size()));
    return fnv.h;
}

}  // namespace mirror_t10
