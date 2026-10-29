#pragma once
//
// mechanism_auton_rig.hpp — a robot's mechanisms in an AUTON on the host test plant, for the mechanism
// core's auton-step, run-while-driving, route and driver-loop tests (chunk MECH-A4, 2026-10-22). Say
// "host test plant", never "simulator": every plant constant here is INVENTED (HA-183).
//
// The rig is the brief's probe rig (P1, P5d, P9, P14) made a test fixture: the A2 tank plant
// (motion_rig::MotionRig, a 12 in tank) under a REAL Chassis whose pacer chain is the program's —
//
//     Chassis -> RunGuard -> MechanismService -> World (the plants step, the clock advances)
//
// — so a Routine's blocking drive paces the guard, which ticks the service, which commands the
// mechanisms, which the World then steps. Robot x (test/mechanism_fixtures.hpp, robot x's own
// declarations) has its ARM, LIFT and FLYWHEEL on the §1.12 models (sim::ArmPlant / FlywheelPlant),
// MISMATCHED against the controller on purpose (the arm's plant carries 2.0 V of gravity where the
// declaration states 1.6; the lift's 1.2 where it states 1.0; both plants slower than their cartridge),
// its intake's distance sensor scripted (no object, then a piece at `sensorSeesAt`), its lift's home
// switch following the lift plant (pressed at the bottom), its hood on two fake lines. Robot two has
// its claw pivots and its claw on fakes, no plant (nothing reads them back). The devices are built by
// hand — exactly the objects a program's robot::MechanismParts would build — so a test can reach each
// one; `serviceInChain = false` builds the CONTROL: Chassis -> RunGuard -> World, the service bound but
// never paced.
//
// Members are in LIFETIME order: the devices and the plants before the service (bind()'s rule), the
// service before the guard and the chassis that pace it.

#include <algorithm>
#include <cmath>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
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
#include "shulib/units/quantity.hpp"

namespace auton_rig {

namespace mech = shulib::mechanism;
namespace fake = shulib::hal::fake;
inline constexpr double kPi = 3.14159265358979323846;
/// PROS's 9999 mm "no object", as the distance adapter converts it: the honest far reading.
inline constexpr double kNoObjectIn = 9999.0 / 25.4;

/// The ARM's plant: Green 5:1 (40 rpm free at the joint), friction 0.3 V, τ 50 ms, gravity 2.0 V (the
/// declaration says 1.6 — mismatched), level at 45°, hard stops at −2° and 170°.
[[nodiscard]] inline shulib::sim::ArmPlantConfig armPlantConfig() {
    shulib::sim::ArmPlantConfig c;
    c.gearRatio = 5.0;
    c.kS = 0.3;
    c.kV = 12.0 / (200.0 * 2.0 * kPi / 60.0 / 5.0);
    c.kA = 0.05 * c.kV;
    c.kG = 2.0;
    c.levelDeg = 45.0;
    c.minStopDeg = -2.0;
    c.maxStopDeg = 170.0;
    return c;
}
/// The LIFT's plant: Red 1:1 on its 1.5 in spool, 10 % slower than its cartridge, constant gravity
/// 1.2 V (the declaration says 1.0), stops at 0 and 25 in of spool degrees.
[[nodiscard]] inline shulib::sim::ArmPlantConfig liftPlantConfig() {
    shulib::sim::ArmPlantConfig c;
    c.gearRatio = 1.0;
    c.kS = 0.3;
    c.kV = 12.0 / (100.0 * 2.0 * kPi / 60.0) * 1.1;
    c.kA = 0.05 * c.kV;
    c.kG = 1.2;
    c.constantGravity = true;
    c.minStopDeg = 0.0;
    c.maxStopDeg = 25.0 / (kPi * 1.5) * 360.0;
    return c;
}
/// The FLYWHEEL's plant: Blue geared up 6:1 (3600 rpm free), 5 % slower, friction 0.3 V, τ 0.2 s.
[[nodiscard]] inline shulib::sim::FlywheelPlantConfig flywheelPlantConfig() {
    shulib::sim::FlywheelPlantConfig c;
    c.gearRatio = 1.0 / 6.0;
    c.kS = 0.3;
    c.kV = 12.0 / (3600.0 * 2.0 * kPi / 60.0) * 1.05;
    c.kA = 0.2 * c.kV;
    return c;
}

/// Robot x's world below the service (file header). Each pace: the hook, the plants, the tank, the
/// lift's switch, the intake's sensor (a piece from `sensorSeesAt` on, until `sensorGoneAt`).
struct XWorld final : shulib::motion::ITickPacer {
    motion_rig::MotionRig* rig = nullptr;
    shulib::sim::ArmPlant* arm = nullptr;
    shulib::sim::ArmPlant* lift = nullptr;
    shulib::sim::FlywheelPlant* fly = nullptr;
    fake::FakeDistance* intakeSensor = nullptr;
    fake::FakeDigitalIn* liftSwitch = nullptr;
    bool liftSwitchFollowsPlant = true;
    double sensorSeesAt = 1e9;
    double sensorGoneAt = 1e9;
    bool sensorBlind = false;  ///< the intake's sensor gives no evidence (near, confidence 0)
    long paces = 0;
    std::function<void()> onPace;  ///< runs FIRST each pace (a test's sampler)
    void pace() override {
        if (paces > 200000) {
            throw std::runtime_error("XWorld: pace cap exceeded -- a wait failed to end");
        }
        ++paces;
        if (onPace) {
            onPace();
        }
        arm->step(0.01);
        lift->step(0.01);
        fly->step(0.01);
        rig->h.plant().step(shulib::units::Time{0.01});
        if (liftSwitchFollowsPlant) {
            liftSwitch->setState(lift->trueDeg() <= 0.5);  // pressed at the bottom (spool degrees)
        }
        const double t = rig->h.clock().now().value();
        if (sensorBlind) {
            intakeSensor->setConfidence(0.0);  // held near with no confidence: no evidence (UNKNOWN)
            intakeSensor->setDistance(shulib::units::Length{1.0});
        } else if (t >= sensorSeesAt && t < sensorGoneAt) {
            intakeSensor->setConfidence(1.0);
            intakeSensor->setDistance(shulib::units::Length{1.0});
        } else {
            intakeSensor->setConfidence(1.0);
            intakeSensor->setDistance(shulib::units::Length{kNoObjectIn});
        }
    }
};

/// Robot x's stack (file header). `serviceInChain` false = the CONTROL (the service never paced).
struct RobotX {
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
    motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
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
    shulib::sim::ArmPlant armPlant{armF, armPlantConfig()};
    shulib::sim::ArmPlant liftPlant{liftF, liftPlantConfig()};
    shulib::sim::FlywheelPlant flyPlant{flyF, flywheelPlantConfig()};
    XWorld world;
    mech::MechanismService svc{world};
    shulib::sequence::RunGuard guard;
    shulib::chassis::Chassis chassis;
    mech::Devices devs[5];

    /// `liftOnSwitchAtBoot` false: the lift is NOT on its switch at boot (it is not homed);
    /// `devices` may replace the devices before the bind (an ABSENT entry, a missing sensor).
    explicit RobotX(bool serviceInChain = true, bool liftOnSwitchAtBoot = true,
                    const std::function<void(mech::Devices (&)[5])>& devices = {})
        : guard{serviceInChain ? static_cast<shulib::motion::ITickPacer&>(svc)
                               : static_cast<shulib::motion::ITickPacer&>(world)},
          chassis{rig.deps, guard, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}},
          devs{{.motors = &mArm},
               {.motors = &mLift, .homeSwitch = &liftSwitch},
               {.motors = &mFly},
               {.motors = &mIntake, .pieceDistance = &intakeSensor},
               {.pneumatic = &mHood}} {
        intakeSensor.setConfidence(1.0);
        intakeSensor.setDistance(shulib::units::Length{kNoObjectIn});
        liftSwitch.setState(liftOnSwitchAtBoot);
        world.liftSwitchFollowsPlant = liftOnSwitchAtBoot;
        world.rig = &rig;
        world.arm = &armPlant;
        world.lift = &liftPlant;
        world.fly = &flyPlant;
        world.intakeSensor = &intakeSensor;
        world.liftSwitch = &liftSwitch;
        if (devices) {
            devices(devs);
        }
        svc.bind(mech_fixture::robot_x::kRobotXMechanisms, devs, rig.h.clock(), rig.latch, sink);
    }
    [[nodiscard]] double now() { return rig.h.clock().now().value(); }
    /// The lift's TRUE height, inches (its plant's spool degrees through the 1.5 in spool).
    [[nodiscard]] double liftInches() const { return liftPlant.trueDeg() / 360.0 * kPi * 1.5; }
    /// Every line of `subsystem`, in order.
    [[nodiscard]] std::vector<std::string> lines(const char* subsystem) const {
        std::vector<std::string> out;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == subsystem) {
                out.push_back(sink.at(i).message);
            }
        }
        return out;
    }
    /// How many lines of `subsystem` contain `needle` (any level).
    [[nodiscard]] int count(const char* subsystem, const std::string& needle) const {
        int n = 0;
        for (const std::string& l : lines(subsystem)) {
            n += l.find(needle) != std::string::npos ? 1 : 0;
        }
        return n;
    }
    /// How many lines of `subsystem` AT `level` contain `needle` (the review's finding: count() alone
    /// cannot see a Warn line lowered to Debug, which a LevelFilterSink on the brain would drop).
    [[nodiscard]] int countAt(shulib::hal::LogLevel level, const char* subsystem, const std::string& needle) const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            const auto& e = sink.at(i);
            n += (e.level == level && e.subsystem == subsystem && e.message.find(needle) != std::string::npos) ? 1 : 0;
        }
        return n;
    }
    /// Every claim released, every mechanism in its safe state's command (0 V; the hood's safe
    /// position 'down' = Low/Low).
    [[nodiscard]] bool allSafeAndReleased() const {
        return !mArm.claimed() && !mLift.claimed() && !mFly.claimed() && !mIntake.claimed() && !mHood.claimed() &&
               armM.commandedVoltage().value() == 0.0 && liftM1.commandedVoltage().value() == 0.0 &&
               flyM.commandedVoltage().value() == 0.0 && intakeM.commandedVoltage().value() == 0.0 &&
               !lineC.commanded() && !lineD.commanded();
    }
};

/// A world with no mechanism plant: the tank only (robot two's claws read nothing back).
struct TankWorld final : shulib::motion::ITickPacer {
    motion_rig::MotionRig* rig = nullptr;
    long paces = 0;
    std::function<void()> onPace;
    void pace() override {
        if (paces > 200000) {
            throw std::runtime_error("TankWorld: pace cap exceeded -- a wait failed to end");
        }
        ++paces;
        if (onPace) {
            onPace();
        }
        rig->h.plant().step(shulib::units::Time{0.01});
    }
};

/// Robot two's stack: its claw pivots and its claw (mech_fixture::robot_two) on fakes.
struct RobotTwo {
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
    motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
    fake::FakeMotor p6, p7;
    shulib::hal::IMotor* m6[1]{&p6};
    shulib::hal::IMotor* m7[1]{&p7};
    shulib::hal::MotorMechanism mech6{m6, shulib::hal::BrakeMode::Hold, "claw pivot 6"};
    shulib::hal::MotorMechanism mech7{m7, shulib::hal::BrakeMode::Hold, "claw pivot 7"};
    fake::FakeDigitalOut lineA;
    shulib::hal::IDigitalOut* cl[1]{&lineA};
    shulib::hal::PneumaticMechanism claw{cl, false, "claw"};  // the one-level form: safe = Low (closed)
    TankWorld world;
    mech::MechanismService svc{world};
    shulib::sequence::RunGuard guard{svc};
    shulib::chassis::Chassis chassis{rig.deps, guard, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}};
    mech::Devices devs[3]{{.motors = &mech6}, {.motors = &mech7}, {.pneumatic = &claw}};

    RobotTwo() {
        world.rig = &rig;
        svc.bind(mech_fixture::robot_two::kRobotTwoMechanisms, devs, rig.h.clock(), rig.latch, sink);
    }
    [[nodiscard]] std::vector<std::string> lines(const char* subsystem) const {
        std::vector<std::string> out;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == subsystem) {
                out.push_back(sink.at(i).message);
            }
        }
        return out;
    }
};

/// An auton entry for `routine`, the brief's probe schedule (a 13 s park, a 14 s hard stop), a
/// RELATIVE start.
[[nodiscard]] inline shulib::competition::AutonEntry entryFor(const char* name,
                                                              shulib::competition::AutonRoutine routine,
                                                              double endActionAt = 13.0, double hardStopAt = 14.0) {
    shulib::competition::AutonEntry e{};
    e.name = name;
    e.kind = shulib::competition::AutonKind::Match;
    e.routine = routine;
    e.start = shulib::competition::StartPose::relative();
    e.endActionAt = shulib::units::Time{endActionAt};
    e.hardStopAt = shulib::units::Time{hardStopAt};
    return e;
}

/// The match period the rig's autons run in.
inline constexpr shulib::units::Time kPeriod{30.0};

}  // namespace auton_rig
