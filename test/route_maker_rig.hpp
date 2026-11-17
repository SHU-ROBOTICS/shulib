#pragma once
//
// route_maker_rig.hpp — the stacks the route builder's GOLDENS run on (chunk RMP1-A1, 2026-11-14; the RMP1
// brief §2.2 "The export compiles AND runs", [R8]). Everything here runs on the HOST TEST PLANT: every plant
// constant is INVENTED (HA-183), and nothing here is a claim about a robot.
//
// Each golden is the route builder's export (test/goldens/route_maker/*.cpp — written by the page's own
// exporter, kept fresh by its node test) compiled with the suite's flags and RUN through the library's
// competition::runAuton, its FIELD start seated, on:
//   * ROBOT X (test/mechanism_fixtures.hpp — the BEHAVIOUR goldens, numbers the test owns): an arm, a
//     switch-homed lift and a flywheel on the §1.12 plants (mechanism_auton_rig.hpp's own numbers), an
//     intake, a three-position hood. `liftUpAtBoot` starts the lift 2 in up with its switch following the
//     plant, so it is NOT homed at boot (P6 (3): with the switch pressed at boot the plant never shows a
//     missing home — RC25 is the page's to catch, and this is RC25's twin);
//   * ROBOT ONE and ROBOT TWO (src/robots/mechanism_table.hpp — SMOKE only: compiled and run without a
//     throw, no verdict asserted, because robot one's facts are placeholders the builder types this week).
// The drive is the A2 plant's 12 in tank (motion_rig), placed at the entry's FIELD start — P1's method.

#include <cmath>
#include <functional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "fakes_by_kind.hpp"
#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "motion_test_rig.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/hal/fake/fake_digital_in.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_distance.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/mechanism_plant.hpp"

namespace rm_rig {

namespace mech = shulib::mechanism;
namespace fake = shulib::hal::fake;

/// The plant's config with its truth (and the estimate) starting at `start`.
[[nodiscard]] inline shulib::sim::SimHarnessConfig plantAt(const shulib::math::Pose2d& start) {
    shulib::sim::SimHarnessConfig cfg = motion_rig::plantConfig();
    cfg.plant.initialPose = start;
    return cfg;
}

/// Below the service: the mechanism plants (a hook), then the tank. Also samples the plant's peak speed.
struct World final : shulib::motion::ITickPacer {
    motion_rig::MotionRig* rig = nullptr;
    std::function<void()> stepMechanisms;
    long paces = 0;
    double peakSpeed = 0.0;  ///< the tank's largest true speed (in/s) seen, pace to pace
    double peakTurn = 0.0;   ///< the largest true turn rate (rad/s) seen
    double lastX = 0, lastY = 0, lastH = 0;
    bool seeded = false;
    void pace() override {
        if (paces > 200000) {
            throw std::runtime_error("rm_rig::World: pace cap exceeded -- a wait failed to end");
        }
        ++paces;
        if (stepMechanisms) {
            stepMechanisms();
        }
        rig->h.plant().step(shulib::units::Time{0.01});
        const shulib::math::Pose2d p = rig->h.truePose();
        if (seeded) {
            peakSpeed = std::max(peakSpeed, std::hypot(p.x().value() - lastX, p.y().value() - lastY) / 0.01);
            peakTurn = std::max(peakTurn, std::abs(std::remainder(p.heading().radians() - lastH, 2.0 * auton_rig::kPi)) / 0.01);
        }
        lastX = p.x().value();
        lastY = p.y().value();
        lastH = p.heading().radians();
        seeded = true;
    }
};

/// Robot x's mechanisms on fakes and plants (mechanism_auton_rig.hpp's RobotX, the same numbers).
struct XFakes {
    fake::FakeMotor armM, liftM1, liftM2, flyM, intakeM;
    shulib::hal::IMotor* am[1]{&armM};
    shulib::hal::IMotor* lm[2]{&liftM1, &liftM2};
    shulib::hal::IMotor* fm[1]{&flyM};
    shulib::hal::IMotor* im[1]{&intakeM};
    shulib::hal::MotorMechanism mArm{am, shulib::hal::BrakeMode::Hold, "arm"};
    shulib::hal::MotorMechanism mLift{lm, shulib::hal::BrakeMode::Hold, "lift"};
    shulib::hal::MotorMechanism mFly{fm, shulib::hal::BrakeMode::Coast, "flywheel"};
    shulib::hal::MotorMechanism mIntake{im, shulib::hal::BrakeMode::Coast, "intake"};
    fake::FakeDigitalOut lineC, lineD;
    shulib::hal::IDigitalOut* hl[2]{&lineC, &lineD};
    bool hoodSafe[2]{false, false};
    shulib::hal::PneumaticMechanism mHood{hl, std::span<const bool>{hoodSafe, 2}, "hood"};
    fake::FakeDigitalIn liftSwitch;
    fake::FakeDistance intakeSensor;
    fake::FakeMotor* armF[1]{&armM};
    fake::FakeMotor* liftF[2]{&liftM1, &liftM2};
    fake::FakeMotor* flyF[1]{&flyM};
    shulib::sim::ArmPlant armPlant{armF, auton_rig::armPlantConfig()};
    shulib::sim::ArmPlant liftPlant;
    shulib::sim::FlywheelPlant flyPlant{flyF, auton_rig::flywheelPlantConfig()};
    mech::Devices devs[5];

    static shulib::sim::ArmPlantConfig liftConfig(bool upAtBoot) {
        shulib::sim::ArmPlantConfig c = auton_rig::liftPlantConfig();
        if (upAtBoot) {
            c.startDeg = 2.0 / (auton_rig::kPi * 1.5) * 360.0;  // 2 in up its 1.5 in spool: off its switch
        }
        return c;
    }
    explicit XFakes(bool liftUpAtBoot)
        : liftPlant{liftF, liftConfig(liftUpAtBoot)},
          devs{{.motors = &mArm},
               {.motors = &mLift, .homeSwitch = &liftSwitch},
               {.motors = &mFly},
               {.motors = &mIntake, .pieceDistance = &intakeSensor},
               {.pneumatic = &mHood}} {
        intakeSensor.setConfidence(1.0);
        intakeSensor.setDistance(shulib::units::Length{auton_rig::kNoObjectIn});
        liftSwitch.setState(liftPlant.trueDeg() <= 0.5);
    }
    void step() {
        armPlant.step(0.01);
        liftPlant.step(0.01);
        flyPlant.step(0.01);
        liftSwitch.setState(liftPlant.trueDeg() <= 0.5);  // pressed at the bottom (spool degrees)
    }
    [[nodiscard]] double liftInches() const { return liftPlant.trueDeg() / 360.0 * auton_rig::kPi * 1.5; }
    [[nodiscard]] double armDeg() const { return armPlant.trueDeg(); }
};

/// Robot one's WHOLE list on fakes built by kind (RPV1's test/fakes_by_kind.hpp): the cascade, both side toggles and
/// the three pistons (ROB1-C2) — a SMOKE run.
struct OneFakes {
    struct NoPace final : shulib::motion::ITickPacer {
        void pace() override {}  // the rig's own world steps the drive; this layer steps only the mechanisms
    } noPace;
    tester_rig::FakesByKind kinds{shulib::bench::robot_one::kRobotOneMechanisms};
    std::span<const mech::Devices> devs = kinds.parts().devices();
    OneFakes() { kinds.inner = &noPace; }
    void step() { kinds.pace(); }
};

/// Robot two's WHOLE list — robot one's twin since ROB1-BT — on fakes built by kind, as OneFakes.
struct TwoFakes {
    OneFakes::NoPace noPace;
    tester_rig::FakesByKind kinds{shulib::bench::robot_two::kRobotTwoMechanisms};
    std::span<const mech::Devices> devs = kinds.parts().devices();
    TwoFakes() { kinds.inner = &noPace; }
    void step() { kinds.pace(); }
};

/// The stack: the fakes before the service (bind()'s lifetime rule), the service before the guard and the
/// chassis that pace it.
template <class Fakes>
struct Stack {
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
    motion_rig::MotionRig rig;
    Fakes fakes;
    World world;
    mech::MechanismService svc{world};
    shulib::sequence::RunGuard guard{svc};
    shulib::chassis::Chassis chassis;

    template <class... A>
    Stack(const shulib::math::Pose2d& start, const mech::MechanismTable& table, A&&... fakeArgs)
        : Stack(shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}, start, table,
                std::forward<A>(fakeArgs)...) {}
    /// The same, on a chassis config of the caller's (a robot's own drive settings).
    template <class... A>
    Stack(const shulib::chassis::ChassisConfig& cfg, const shulib::math::Pose2d& start, const mech::MechanismTable& table,
          A&&... fakeArgs)
        : rig{kin, plantAt(start), &sink}, fakes{std::forward<A>(fakeArgs)...}, chassis{rig.deps, guard, cfg} {
        world.rig = &rig;
        world.stepMechanisms = [this] { fakes.step(); };
        svc.bind(table, fakes.devs, rig.h.clock(), rig.latch, sink);
    }

    /// Run `entry` as the competition program and the Auton Tester do (runAuton), in a 60 s skills period.
    shulib::competition::AutonRunResult run(const shulib::competition::AutonEntry& entry) {
        return shulib::competition::runAuton(chassis, guard, entry, nullptr, sink, svc, shulib::units::Time{60.0});
    }
    /// How many log lines of `subsystem` contain `needle`.
    [[nodiscard]] int count(const char* subsystem, const std::string& needle) const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            const auto& e = sink.at(i);
            n += (e.subsystem == subsystem && e.message.find(needle) != std::string::npos) ? 1 : 0;
        }
        return n;
    }
    /// Every line of the run, for a failing check's message.
    [[nodiscard]] std::string transcript() const {
        std::string out;
        for (int i = 0; i < sink.size(); ++i) {
            const auto& e = sink.at(i);
            if (e.subsystem == "RTN" || e.subsystem == "COMP" || e.subsystem == "MECH" || e.subsystem == "MOT") {
                out += "[" + e.subsystem + "] " + e.message + "\n";
            }
        }
        return out;
    }
    /// The pose the plant's truth ended at.
    [[nodiscard]] shulib::math::Pose2d truth() { return rig.h.truePose(); }
};

/// The FIELD start of an entry (the stack places the plant there).
[[nodiscard]] inline shulib::math::Pose2d startOf(const shulib::competition::AutonEntry& e) {
    return e.start.mode() == shulib::competition::StartPose::Mode::Field ? e.start.pose() : shulib::math::Pose2d{};
}

}  // namespace rm_rig
