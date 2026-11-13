// DEFECTS2 — a brake mode reaches the motor, asserted at the BOTTOM of real stacks (brief §1.5).
//
// ═══ WHY THIS FILE EXISTS ═══════════════════════════════════════════════════════════
// Until DEFECTS2 every stop in the library — a mechanism's declared Hold, the drive's
// "0 V + Brake" cancel, the driver loop's disable — reached a V5 as move_voltage(0), which PROS
// documents as ignoring the brake mode (vendored motors.h:253-254). No test could see it: the
// fakes echo whatever mode they are told, the A2 plant pins a wheel at 0 V whatever its mode, and
// the PROS shim had no brake(). pros_motor_adapter_test.cpp pins the ADAPTER's rule; this file
// pins that the rule is reached THROUGH the stacks that matter — MotorMechanism and a mechanism
// operation over real ProsMotors (T3's claw ports, 6 and 7), and a whole robot::Robot built over
// the REAL on-robot device provider (the composition shulib Teleop runs, through MotorGroup
// wrappers) for the motion cancel safe state and the driver-control loop. Every assertion reads
// the shim's per-port CONTROL STATE — Voltage (mode ignored) or Stopped (under which mode) —
// never an adapter's brakeMode(), which read "Hold" all through the defect.
//
// NO HARDWARE CLAIM: the shim tests the stacks against our BELIEF about PROS (HA-142); it cannot
// test the belief, and nothing here says how a V5 physically brakes or holds (HA-143..HA-146).

#include "doctest.h"

#include <array>
#include <cstdint>
#include <span>

#include "mechanism_test_rig.hpp"
#include "pros/shim_control.hpp"

#include "shulib/config/robot_config.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_controller.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/drive_brake.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"

using pros::shim::MotorControl;
using shulib::hal::BrakeMode;
using shulib::hal::ControllerAxis;
using shulib::hal::IMotor;
using shulib::hal::MotorCartridge;
using shulib::hal::MotorGroup;
using shulib::hal::MotorMechanism;
using shulib::hal::fake::FakeController;
using shulib::hal::pros::MotorGearset;
using shulib::hal::pros::ProsDeviceProvider;
using shulib::hal::pros::ProsMotor;
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::math::ChassisSpeeds;
using shulib::teleop::DriverLoopConfig;
using shulib::teleop::DriverProfile;
using shulib::teleop::DriveShaper;
using shulib::teleop::DriveShaperState;
using shulib::teleop::HeadingSample;
using shulib::teleop::runDriverControl;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

/// Port `p` is STOPPED by brake() under `mode` — the only state in which PROS applies a mode.
bool stoppedUnder(int p, pros::v5::MotorBrake mode) {
    const auto& s = pros::shim::motorState(p);
    return s.control == MotorControl::Stopped && s.stoppedUnder == mode;
}

/// Port `p` is under VOLTAGE control at exactly 0 mV, with `mode` in its register — the mode
/// ignored, which for Coast is exactly what Coast has always meant.
bool zeroVoltsWithMode(int p, pros::v5::MotorBrake mode) {
    const auto& s = pros::shim::motorState(p);
    return s.control == MotorControl::Voltage && s.lastVoltageMv == 0 && s.brake == mode;
}

// ── the robot fixture: robot two's shape over the REAL provider (pros_device_provider 1k) ────

/// Robot two's ports and signs, IMU 2, invented scales — this file is about the STOP, not
/// geometry.
constexpr std::int8_t kLeft[5] = {-11, 12, -13, 14, -15};
constexpr std::int8_t kRight[5] = {20, -19, 18, -17, 16};
constexpr int kDrivePorts[10] = {11, 12, 13, 14, 15, 20, 19, 18, 17, 16};

shulib::config::RobotConfig shimTankConfig() {
    shulib::hal::DrivetrainDescription d{.cartridge = MotorCartridge::Blue,
                                         .imuPort = 2,
                                         .trackWidth = Length{12.5},
                                         .source = shulib::hal::DescriptionSource::RobotMeasured,
                                         .provenance = "DEFECTS2 shim fixture: never measured"};
    for (std::size_t i = 0; i < 5; ++i) {
        d.left.signedPorts[i] = kLeft[i];
        d.right.signedPorts[i] = kRight[i];
    }
    d.left.count = 5;
    d.right.count = 5;
    d.left.inchesPerRadian = Length{1.375};
    d.right.inchesPerRadian = Length{1.375};
    return shulib::config::RobotConfig{
        .identity = {.name = "Shim Two", .team = "SHU"},
        .drivetrain = {.kind = shulib::config::DrivetrainKind::Tank, .tank = {.description = d}},
        .odometry = {.kind = shulib::config::OdometryKind::DriveEncoders}};
}

/// Plug robot two's devices into the shim registry (resetAll() first, by the caller).
void plugRobotTwo() {
    for (const int p : kDrivePorts) {
        pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
    }
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
}

/// Every drive port satisfies `pred`.
template <typename Pred>
bool everyDrivePort(Pred&& pred) {
    for (const int p : kDrivePorts) {
        if (!pred(p)) {
            return false;
        }
    }
    return true;
}

HeadingSample untrustedZero() { return HeadingSample{}; }

constexpr Time kTick{0.01};

}  // namespace

// ═══ 1. MotorMechanism over real ProsMotors — the claw's stack ═══════════════════════
TEST_CASE("DEFECTS2 s1: a MotorMechanism declared Hold reaches brake() at the bottom of the "
          "stack — directly, through an operation's exit, and through a MotorGroup") {
    // BUG CAUGHT (the adapter mutation "revert to plain move_voltage(0)", seen from above): T3's
    // claw pivots on ports 6 and 7 must HOLD their angle on release. Before DEFECTS2 their
    // declared Hold safe state went out as 0 V and the claw went limp — while mech.safeBrakeMode()
    // and every motor's brakeMode() said Hold.
    pros::shim::resetAll();
    ProsMotor left{6, MotorGearset::Green};
    ProsMotor right{-7, MotorGearset::Green};
    std::array<IMotor*, 2> motors{&left, &right};

    SUBCASE("applySafeState() from 8 V") {
        MotorMechanism claw{motors, BrakeMode::Hold, "claw"};
        claw.setVoltage(Voltage{8.0});
        REQUIRE(pros::shim::motorState(6).control == MotorControl::Voltage);
        claw.applySafeState();
        CHECK(stoppedUnder(6, pros::v5::MotorBrake::hold));
        CHECK(stoppedUnder(7, pros::v5::MotorBrake::hold));
        CHECK(pros::shim::motorState(6).brakeCalls == 1);
        CHECK(pros::shim::motorState(7).brakeCalls == 1);
    }
    SUBCASE("an operation re-sends its volts every tick, then exits into the declared Hold") {
        MotorMechanism claw{motors, BrakeMode::Hold, "claw"};
        mech_rig::OpRig rig;
        int tick = 0;
        RunUntilConfirmed op{claw, rig.deps,
                             RunUntilConfirmedConfig{
                                 .voltage = Voltage{6.0},
                                 .timeout = Time{1.0},
                                 .stall = StallConfig{.currentAtLeast = Current{2.0},
                                                      .speedAtMost = AngularVelocity{0.1},
                                                      .persistence = Time{0.05}}},
                             [&] { return tick >= 5; }, "close"};
        op.start();
        MechanismOutcome o = MechanismOutcome::Running;
        for (int i = 0; i < 50 && o == MechanismOutcome::Running; ++i) {
            o = op.tick();
            if (o == MechanismOutcome::Running) {
                CHECK(pros::shim::motorState(6).lastVoltageMv == 6000);
                CHECK(pros::shim::motorState(6).control == MotorControl::Voltage);
            }
            rig.clock.advance(kTick);
            ++tick;
        }
        REQUIRE(o == MechanismOutcome::Succeeded);
        CHECK(stoppedUnder(6, pros::v5::MotorBrake::hold));
        CHECK(stoppedUnder(7, pros::v5::MotorBrake::hold));
        CHECK(pros::shim::motorState(6).brakeCalls == 1);
    }
    SUBCASE("through a MotorGroup wrapper (landmine 3: the fix lives below every wrapper)") {
        MotorGroup pair{motors};
        std::array<IMotor*, 1> one{&pair};
        MotorMechanism claw{one, BrakeMode::Hold, "claw"};
        claw.setVoltage(Voltage{-5.0});
        claw.applySafeState();
        CHECK(stoppedUnder(6, pros::v5::MotorBrake::hold));
        CHECK(stoppedUnder(7, pros::v5::MotorBrake::hold));
    }
    SUBCASE("the control: a mechanism declared Coast still stops at 0 V, no brake()") {
        MotorMechanism intake{motors, BrakeMode::Coast, "intake"};
        intake.setVoltage(Voltage{8.0});
        intake.applySafeState();
        CHECK(zeroVoltsWithMode(6, pros::v5::MotorBrake::coast));
        CHECK(zeroVoltsWithMode(7, pros::v5::MotorBrake::coast));
        CHECK(pros::shim::motorState(6).brakeCalls == 0);
    }
}

// ═══ 2. The motion cancel safe state over a whole robot ════════════════════════════
TEST_CASE("DEFECTS2 s2: chassis.cancel() — the motion cancel safe state — reaches all ten drive "
          "motors as brake() under Brake, through the builder's MotorGroups") {
    // BUG CAUGHT (the adapter defect, seen from the facade): every cancel path (a user cancel, a
    // pre-emption, a fault abort, the panic stop) goes through applyCancelSafeState's "Brake,
    // then 0 V". On a V5 it coasted. Asserted on every one of robot two's ten ports, through the
    // REAL provider and the builder's two MotorGroups.
    pros::shim::resetAll();
    plugRobotTwo();
    const shulib::config::RobotConfig cfg = shimTankConfig();
    ProsDeviceProvider devices;
    REQUIRE(shulib::robot::validate(cfg, devices).ok);
    shulib::robot::Robot robot{cfg, devices};
    for (IMotor* side : robot.context().driveMotors()) {
        side->setVoltage(Voltage{6.0});
    }
    REQUIRE(everyDrivePort([](int p) {
        return pros::shim::motorState(p).control == MotorControl::Voltage
               && pros::shim::motorState(p).lastVoltageMv != 0;
    }));
    robot.chassis().cancel();  // no motion active: the PANIC STOP, which still safes the drive
    CHECK(everyDrivePort([](int p) { return stoppedUnder(p, pros::v5::MotorBrake::brake); }));
    CHECK(everyDrivePort([](int p) { return pros::shim::motorState(p).brakeCalls == 1; }));
}

// ═══ 3. The driver-control loop over a whole robot — the ruled release mode ═════════════
TEST_CASE("DEFECTS2 s3: the driver loop's release mode defaults to COAST (the team lead's "
          "ruling, 2026-09-26)") {
    // BUG CAUGHT: a default that brakes on release — a feel nobody chose; the drive was driven
    // coasting on 2026-09-25 and the ruling keeps it.
    CHECK(DriverLoopConfig{}.releaseBrakeMode == BrakeMode::Coast);
    constexpr DriverLoopConfig stated{.tickPeriod = kTick,
                                      .rezeroButton = shulib::hal::ControllerButton::A};
    CHECK(stated.releaseBrakeMode == BrakeMode::Coast);  // designated init leaves it at Coast
}

TEST_CASE("DEFECTS2 s4: a cancel inside the loop, then released sticks — the drive COASTS at "
          "the adapter in that same tick; a later disable still BRAKES") {
    // BUG CAUGHT (mutation "drop the per-tick release re-assert"): drive() pre-empting a motion
    // cancels it into Brake, and since DEFECTS2 a Brake at 0 V really brakes a V5 — so without
    // the loop re-asserting its release mode, the first released stick after any library stop
    // would BRAKE the drive, against the ruling. The pre-emption is real here (brakeCalls == 1
    // proves it braked first), and the loop must undo it before its tick ends.
    pros::shim::resetAll();
    plugRobotTwo();
    const shulib::config::RobotConfig cfg = shimTankConfig();
    ProsDeviceProvider devices;
    REQUIRE(shulib::robot::validate(cfg, devices).ok);
    shulib::robot::Robot robot{cfg, devices};
    shulib::chassis::Chassis& chassis = robot.chassis();
    FakeController pad;
    DriveShaper shaper{DriverProfile{}};
    shulib::motion::DriveBrake stopMotion{chassis.deps(), chassis.motionConfig(), 5.0};

    constexpr int kArm = 3;      // top of tick 3: a motion is armed, the sticks are released
    constexpr int kDisable = 7;  // tick 7: a field disable
    int k = 0;
    int checkedTicks = 0;
    runDriverControl(
        chassis, pad, devices.pacer(), shaper, DriverLoopConfig{.tickPeriod = kTick},
        [&] {
            if (k >= kDisable) {
                return true;
            }
            pad.setAxis(ControllerAxis::LeftY, k < kArm ? 1.0 : 0.0);
            if (k == kArm) {
                chassis.scheduler().async(stopMotion);  // this tick's drive() pre-empts it
            }
            return false;
        },
        [] { return untrustedZero(); },
        [&](const ChassisSpeeds&, bool, const DriveShaperState&) {
            if (k < kArm) {
                // Driving, and the release mode is already on the motors.
                CHECK(everyDrivePort([](int p) {
                    const auto& s = pros::shim::motorState(p);
                    return s.control == MotorControl::Voltage && s.lastVoltageMv != 0
                           && s.brake == pros::v5::MotorBrake::coast;
                }));
            } else {
                CAPTURE(k);
                // The pre-emption DID brake (once, at tick kArm) — and the tick ends coasting.
                CHECK(everyDrivePort(
                    [](int p) { return pros::shim::motorState(p).brakeCalls == 1; }));
                CHECK(everyDrivePort(
                    [](int p) { return zeroVoltsWithMode(p, pros::v5::MotorBrake::coast); }));
                ++checkedTicks;
            }
            ++k;
        });
    CHECK(checkedTicks == kDisable - kArm);
    CHECK_FALSE(chassis.scheduler().hasActiveMotion());
    // The disable, from rest at 0 V under Coast: the cancel's Brake re-issues the stop — BRAKED.
    CHECK(everyDrivePort([](int p) { return stoppedUnder(p, pros::v5::MotorBrake::brake); }));
    CHECK(everyDrivePort([](int p) { return pros::shim::motorState(p).brakeCalls == 2; }));
}

TEST_CASE("DEFECTS2 s5: a disable while DRIVING brakes every drive motor at the adapter") {
    // BUG CAUGHT: the release mode leaking into the disable path (a loop that re-asserted Coast
    // AFTER the cancel, or cancelled under the release mode) — the one stop the ruling says
    // SHOULD brake.
    pros::shim::resetAll();
    plugRobotTwo();
    const shulib::config::RobotConfig cfg = shimTankConfig();
    ProsDeviceProvider devices;
    REQUIRE(shulib::robot::validate(cfg, devices).ok);
    shulib::robot::Robot robot{cfg, devices};
    FakeController pad;
    pad.setAxis(ControllerAxis::LeftY, 1.0);
    DriveShaper shaper{DriverProfile{}};
    int k = 0;
    runDriverControl(
        robot.chassis(), pad, devices.pacer(), shaper, DriverLoopConfig{.tickPeriod = kTick},
        [&] { return k >= 5; }, [] { return untrustedZero(); },
        [&](const ChassisSpeeds&, bool, const DriveShaperState&) { ++k; });
    REQUIRE(k == 5);
    CHECK(everyDrivePort([](int p) { return stoppedUnder(p, pros::v5::MotorBrake::brake); }));
    CHECK(everyDrivePort([](int p) { return pros::shim::motorState(p).brakeCalls == 1; }));
}

TEST_CASE("DEFECTS2 s6: the release mode is on the motors at ENTRY — before the first drive() — "
          "even when the last program state was a Brake stop") {
    // BUG CAUGHT (mutation "drop the entry application"): PROS calls opcontrol() again after a
    // disable, and the previous call left the drive STOPPED under Brake. Until the first drive()
    // of the new call, nothing else would lift that stop; the heading source runs before
    // drive(), so it is where the entry guarantee is observable.
    pros::shim::resetAll();
    plugRobotTwo();
    const shulib::config::RobotConfig cfg = shimTankConfig();
    ProsDeviceProvider devices;
    REQUIRE(shulib::robot::validate(cfg, devices).ok);
    shulib::robot::Robot robot{cfg, devices};
    robot.chassis().cancel();  // the previous opcontrol()'s disable
    REQUIRE(everyDrivePort([](int p) { return stoppedUnder(p, pros::v5::MotorBrake::brake); }));

    FakeController pad;  // sticks centred
    DriveShaper shaper{DriverProfile{}};
    int k = 0;
    bool sawEntry = false;
    runDriverControl(
        robot.chassis(), pad, devices.pacer(), shaper, DriverLoopConfig{.tickPeriod = kTick},
        [&] { return k >= 2; },
        [&] {
            if (k == 0) {  // tick 0, before its drive()
                CHECK(everyDrivePort(
                    [](int p) { return zeroVoltsWithMode(p, pros::v5::MotorBrake::coast); }));
                sawEntry = true;
            }
            return untrustedZero();
        },
        [&](const ChassisSpeeds&, bool, const DriveShaperState&) { ++k; });
    CHECK(sawEntry);
}

TEST_CASE("DEFECTS2 s7: a caller's Hold release mode HOLDS on release — one brake() per port, "
          "not one per tick, through the loop's per-tick re-assert — and DRIVES again on the "
          "next stick") {
    // BUG CAUGHT (mutations "brake() on every zero", "the loop ignores cfg.releaseBrakeMode" and
    // — from the review — "atZero_ set only inside the zero branch"): the field is honoured; the
    // loop's every-tick re-assert of an UNCHANGED mode does not turn into a brake() per tick
    // (HA-144's creep — though the re-assert's set_brake_mode(Hold) does reach the held motor
    // every tick, which is HA-144's second, unsettled half); and after the hold, pushing the
    // stick DRIVES — a zero record that stuck after the first release would make the per-tick
    // re-assert stop the drive dead on every driving tick.
    constexpr int kRelease = 3;  // ticks 3..7 released
    constexpr int kRedrive = 8;  // ticks 8..10 driven again
    constexpr int kDisable = 11;
    pros::shim::resetAll();
    plugRobotTwo();
    const shulib::config::RobotConfig cfg = shimTankConfig();
    ProsDeviceProvider devices;
    REQUIRE(shulib::robot::validate(cfg, devices).ok);
    shulib::robot::Robot robot{cfg, devices};
    FakeController pad;
    DriveShaper shaper{DriverProfile{}};
    int k = 0;
    int heldTicks = 0;
    int redrivenTicks = 0;
    runDriverControl(
        robot.chassis(), pad, devices.pacer(), shaper,
        DriverLoopConfig{.tickPeriod = kTick, .releaseBrakeMode = BrakeMode::Hold},
        [&] {
            if (k >= kDisable) {
                return true;
            }
            pad.setAxis(ControllerAxis::LeftY, (k < kRelease || k >= kRedrive) ? 1.0 : 0.0);
            return false;
        },
        [] { return untrustedZero(); },
        [&](const ChassisSpeeds&, bool, const DriveShaperState&) {
            CAPTURE(k);
            if (k >= kRelease && k < kRedrive) {  // released: held, by ONE brake() per port
                CHECK(everyDrivePort(
                    [](int p) { return stoppedUnder(p, pros::v5::MotorBrake::hold); }));
                CHECK(everyDrivePort(
                    [](int p) { return pros::shim::motorState(p).brakeCalls == 1; }));
                ++heldTicks;
            } else if (k >= kRedrive) {  // driven again: under VOLTAGE, not re-stopped
                CHECK(everyDrivePort([](int p) {
                    const auto& s = pros::shim::motorState(p);
                    return s.control == MotorControl::Voltage && s.lastVoltageMv != 0;
                }));
                CHECK(everyDrivePort(
                    [](int p) { return pros::shim::motorState(p).brakeCalls == 1; }));
                ++redrivenTicks;
            }
            ++k;
        });
    REQUIRE(k == kDisable);
    CHECK(heldTicks == kRedrive - kRelease);
    CHECK(redrivenTicks == kDisable - kRedrive);
    // The disable, while driving: the cancel's Brake — the second stop per port.
    CHECK(everyDrivePort([](int p) { return stoppedUnder(p, pros::v5::MotorBrake::brake); }));
    CHECK(everyDrivePort([](int p) { return pros::shim::motorState(p).brakeCalls == 2; }));
}

TEST_CASE("DEFECTS2 s8: a panic stop from OUTSIDE the loop body — the panel's chassis.cancel(), "
          "no motion to pre-empt — is undone by the next tick's re-assert") {
    // BUG CAUGHT (review finding; mutation "re-assert only after drive() pre-empted a motion"):
    // the header says the release mode is re-asserted after EVERY drive(), not only after a
    // pre-emption, so the invariant does not depend on which path set another mode. s4's only
    // foreign Brake is a pre-emption, so a loop keyed on hasActiveMotion() passed it. Here the
    // Brake comes from perTick with nothing active: keyed on a pre-emption, the drive would stay
    // braked (sticks released) or brake at the next release (sticks held) — for the rest of
    // driver control.
    int kRelease = 0;
    constexpr int kPanic = 3;
    constexpr int kDisable = 8;
    SUBCASE("the sticks already released at the panic stop") { kRelease = 2; }
    SUBCASE("driving at the panic stop, released two ticks later") { kRelease = 5; }
    pros::shim::resetAll();
    plugRobotTwo();
    const shulib::config::RobotConfig cfg = shimTankConfig();
    ProsDeviceProvider devices;
    REQUIRE(shulib::robot::validate(cfg, devices).ok);
    shulib::robot::Robot robot{cfg, devices};
    shulib::chassis::Chassis& chassis = robot.chassis();
    FakeController pad;
    DriveShaper shaper{DriverProfile{}};
    int k = 0;
    int checkedTicks = 0;
    runDriverControl(
        chassis, pad, devices.pacer(), shaper, DriverLoopConfig{.tickPeriod = kTick},
        [&] {
            if (k >= kDisable) {
                return true;
            }
            pad.setAxis(ControllerAxis::LeftY, k < kRelease ? 1.0 : 0.0);
            return false;
        },
        [] { return untrustedZero(); },
        [&](const ChassisSpeeds&, bool, const DriveShaperState&) {
            CAPTURE(k);
            if (k == kPanic) {
                CHECK_FALSE(chassis.scheduler().hasActiveMotion());  // nothing to pre-empt
                chassis.cancel();
                // The panic stop really braked — the state the next tick must undo.
                CHECK(everyDrivePort(
                    [](int p) { return stoppedUnder(p, pros::v5::MotorBrake::brake); }));
            } else if (k > kPanic && k >= kRelease) {
                CHECK(everyDrivePort(
                    [](int p) { return zeroVoltsWithMode(p, pros::v5::MotorBrake::coast); }));
                CHECK(everyDrivePort(
                    [](int p) { return pros::shim::motorState(p).brakeCalls == 1; }));
                ++checkedTicks;
            }
            ++k;
        });
    CHECK(checkedTicks == kDisable - (kRelease > kPanic + 1 ? kRelease : kPanic + 1));
}
