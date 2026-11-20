#pragma once
//
// routine_golden_chains.hpp — STEP1-A's ST1 golden chains: the Routine under its DEFAULT rule (no
// onFail anywhere), run on the host test plant: SEVEN chains — every chassis step form (G1), five failure
// kinds (G2.0..G2.4) and robot x's mechanism steps (G6), the labels the probe prints. The SAME
// functions were compiled against the base tree (ef6162e, before STEP1-A existed) by the executor's
// probe (.claude/coordinator/scratch/step1-a/golden_probe.cpp), and the lines that probe printed are
// the literals test/routine_on_fail_test.cpp compares against — so a change to the default rule's
// counts, stop lines, skip lines or result shows as a diff against code that predates the change.
// It reads only what both trees have: the nine frozen RoutineResult fields and the RTN lines.
// Say "host test plant", never "simulator".

#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/control/exit_group.hpp"
#include "routine_done_line.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/units/literals.hpp"

namespace golden {

using namespace shulib::units::literals;

/// The world below a chassis-only tank: the A2 plant, one 10 ms step per pace.
struct World final : shulib::motion::ITickPacer {
    motion_rig::MotionRig* rig = nullptr;
    void pace() override { rig->h.plant().step(shulib::units::Time{0.01}); }
};
/// A 12 in tank on the memoryless plant, the Chassis paced by World.
struct Tank {
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
    motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
    World world;
    shulib::chassis::Chassis ch{rig.deps, world, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}};
    Tank() { world.rig = &rig; }
};

/// The nine frozen fields, one line.
inline std::string fields(const shulib::chassis::RoutineResult& r) {
    char b[200];
    std::snprintf(b, sizeof b, "ok=%d steps=%d completed=%d skipped=%d stoppedAt=%d stoppedName=%s cause=%d exit=%d mv=%d",
                  r.ok ? 1 : 0, r.steps, r.completed, r.skipped, r.stoppedAt, r.stoppedName,
                  static_cast<int>(r.cause), static_cast<int>(r.exit), static_cast<int>(r.mechanismVerdict));
    return b;
}
/// Every RTN line, "<level>|<message>".
inline std::vector<std::string> rtnLines(const shulib::hal::fake::FakeTelemetrySink& sink) {
    std::vector<std::string> out;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).subsystem == "RTN" && !routine_d4::isStepDoneLine(sink.at(i).level, sink.at(i).subsystem, sink.at(i).message)) {
            out.push_back(std::to_string(static_cast<int>(sink.at(i).level)) + "|" + sink.at(i).message);
        }
    }
    return out;
}

/// G1 — every chassis step form, all succeeding.
inline std::string chainEverything() {
    using shulib::math::Angle;
    using shulib::math::Pose2d;
    using shulib::units::Length;
    Tank t;
    shulib::chassis::Routine r{t.ch, "g1-everything"};
    r.startAt(Pose2d{Length{0.0}, Length{0.0}, Angle{}})
        .moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}})
        .face(30_in, 0_in)
        .driveTo(20_in, 0_in)
        .turnTo(Angle::degrees(90.0))
        .faceAway(20_in, -10_in)
        .reverseTo(20_in, -10_in, {.timeout = 4_s})
        .followTrajectory({Pose2d{Length{20.0}, Length{-4.0}, Angle::degrees(90.0)}})
        .brake()
        .hold(0.3_s)
        .pause(0.2_s)
        .waitFor([] { return true; }, 1_s)
        .then([] {}, "void")
        .then([] { return true; }, "bool")
        .then([] { return shulib::control::ExitReason::Settled; }, "exit")
        .then([] { return shulib::manipulation::MechanismOutcome::Succeeded; }, "outcome");
    std::string s = fields(r.result());
    for (const std::string& l : rtnLines(t.sink)) {
        s += "\n" + l;
    }
    return s;
}

/// G2.0..G2.4 — one failure kind each (five chains), then three later steps the stop skips. `kind`: 0 motion TIMEOUT
/// (a strafeTo off a tank's line), 1 wait TIMEOUT, 2 action FAILED (bool), 3 mechanism FAILED
/// (then()'s MechanismOutcome::TimedOut), 4 an incomplete trajectory.
inline std::string chainFails(int kind) {
    using shulib::math::Angle;
    using shulib::math::Pose2d;
    using shulib::units::Length;
    Tank t;
    shulib::chassis::Routine r{t.ch, "g2-fails"};
    r.driveTo(6_in, 0_in);
    switch (kind) {
        case 0: r.strafeTo(6_in, 12_in, {.timeout = 0.3_s}); break;
        case 1: r.waitFor([] { return false; }, 0.1_s, "wait for nothing"); break;
        case 2: r.then([] { return false; }, "says no"); break;
        case 3: r.then([] { return shulib::manipulation::MechanismOutcome::TimedOut; }, "grab"); break;
        default:
            r.followTrajectory({Pose2d{Length{30.0}, Length{0.0}, Angle{}}, Pose2d{Length{60.0}, Length{0.0}, Angle{}}},
                               {.timeout = 0.2_s});
            break;
    }
    r.driveTo(0_in, 0_in).pause(0.1_s).then([] { return true; }, "never");
    std::string s = fields(r.result());
    for (const std::string& l : rtnLines(t.sink)) {
        s += "\n" + l;
    }
    return s;
}

/// G6 — robot x's mechanism steps (the MECH-A4 rig), succeeding, then a blocking intake run that
/// times out waiting for a piece (MechanismStepFailed, TIMED_OUT), then three skipped steps.
inline std::string chainMechanisms() {
    using namespace mech_fixture::robot_x;
    auton_rig::RobotX x;
    struct Ctx {
        shulib::chassis::Chassis& chassis;
        shulib::mechanism::MechanismService* mechanismService;
    } ctx{x.chassis, &x.svc};
    shulib::chassis::Routine r{ctx, "g6-mechanisms"};
    r.home(lift, 1.5_s)
        .move(arm["score"], 1.5_s)
        .startSpinUp(flywheel["shoot"], 3_s)
        .set(hood["up"])
        .waitUntilDone(flywheel, 3_s)
        .run(intake, 12_volt, seesPiece, 0.4_s)  // no piece ever comes: TIMED_OUT
        .move(arm["high"], 1_s)
        .stop(flywheel)
        .startRun(intake, 12_volt, 1_s);
    std::string s = fields(r.result());
    for (const std::string& l : rtnLines(x.sink)) {
        s += "\n" + l;
    }
    return s;
}

}  // namespace golden
