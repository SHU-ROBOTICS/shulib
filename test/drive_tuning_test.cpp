// drive_tuning_test.cpp — a robot's own auton drive settings (its chassis table -> the boot's tuning).
#include "doctest.h"

#include <cmath>

#include "common/boot_source.hpp"
#include "route_maker_rig.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/units/literals.hpp"

namespace sb = shulib::bench;
using namespace shulib::units::literals;

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;

shulib::chassis::RoutineResult longDrive(shulib::competition::AutonContext& ctx) {
    shulib::chassis::Routine r{ctx, "long-drive"};
    r.face(-36_in, -15_in);
    r.driveTo(-36_in, -15_in);
    return r.result();
}
}  // namespace

TEST_CASE("drive tuning: a table with no settings leaves the library's defaults; robot one's slow the autons and soften the heading") {
    // BUG CAUGHT: a robot's settings not reaching the autons (they ran at 60 in/s and rocked at every target on
    // robot one, 2026-11-17), or a zero (unset) field overwriting a default.
    const shulib::chassis::ChassisConfig d{};
    shulib::chassis::ChassisConfig bench{};
    sb::applyDriveTuning(bench, sb::kBenchBotChassis);
    CHECK(bench.motion.maxLinearSpeed.value() == d.motion.maxLinearSpeed.value());
    CHECK(bench.motion.maxWheelSpeed.value() == d.motion.maxWheelSpeed.value());
    CHECK(bench.motion.maxAngularSpeed.value() == d.motion.maxAngularSpeed.value());
    CHECK(bench.motion.heading.kP == d.motion.heading.kP);
    CHECK(bench.motion.headingSettle.maxError == d.motion.headingSettle.maxError);
    CHECK(bench.motion.headingSettle.maxErrorRate == d.motion.headingSettle.maxErrorRate);

    shulib::chassis::ChassisConfig one{};
    sb::applyDriveTuning(one, sb::kRobotOneChassis);
    CHECK(one.motion.maxLinearSpeed.value() == 40.0);
    CHECK(one.motion.maxWheelSpeed.value() == 40.0);
    CHECK(one.motion.maxAngularSpeed.value() == 4.0);
    CHECK(one.motion.heading.kP == 2.4);
    CHECK(one.motion.headingSettle.maxError == doctest::Approx(4.0 * kDeg));
    CHECK(one.motion.headingSettle.maxErrorRate ==
          doctest::Approx(d.motion.headingSettle.maxErrorRate * 4.0 * kDeg / d.motion.headingSettle.maxError));
    one.motion.validate();  // raises on any value a motion would refuse
    // The twin's are robot one's (the same build).
    shulib::chassis::ChassisConfig two{};
    sb::applyDriveTuning(two, sb::kRobotTwoChassis);
    CHECK(two.motion.maxLinearSpeed.value() == one.motion.maxLinearSpeed.value());
    CHECK(two.motion.heading.kP == one.motion.heading.kP);
}

TEST_CASE("drive tuning: robot one's settings still finish a 69 in drive on the host test plant -- no faster than 40 in/s, arriving") {
    // BUG CAUGHT: settings so soft the drive never settles (a MOTION_TIMEOUT at every target), or a speed cap
    // that does not cap. The plant is the suite's tank, not robot one: this proves the settings drive, not their feel.
    shulib::chassis::ChassisConfig cfg{.motion = motion_rig::motionConfig()};
    sb::applyDriveTuning(cfg, sb::kRobotOneChassis);
    shulib::competition::AutonEntry e{};
    e.name = "long drive";
    e.kind = shulib::competition::AutonKind::Skills;
    e.routine = &longDrive;
    e.start = shulib::competition::StartPose::field(-36_in, 54.076_in, -90_deg);
    e.endActionAt = 57_s;
    e.hardStopAt = 59_s;
    rm_rig::Stack<rm_rig::OneFakes> s{cfg, rm_rig::startOf(e), sb::robot_one::kRobotOneMechanisms};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    CHECK(r.routine.ok);
    CHECK(s.count("RTN", "TIMEOUT") == 0);
    CHECK(s.world.peakSpeed <= 40.0 * 1.10);
    const auto t = s.truth();
    CHECK(std::hypot(t.x().value() + 36.0, t.y().value() + 15.0) < 1.0);
}
