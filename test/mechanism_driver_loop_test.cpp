// MECH-A4 — the DRIVER FILE's mechanism controls (mechanism/driver_inputs.hpp) through the REAL
// teleop::runDriverControl's mechanism-service overload, on the host test plant: a press on the entry
// tick ignored; a preset; a hold that wins mid-move and holds HERE on release; a newer preset beats a
// still-held hold, which then commands nothing until released (P5d); exactly one service tick per
// tick whatever pacer the loop is handed (P8); the service ticked BEFORE perTick; the driver-file
// homing moments (P2i) and U5's restart off a switch; robot two's claws (P9c); a controller drop; a
// stall that supersedes a held button; the disable; and the loop's two preconditions. Every case
// names the bug it would catch. Nothing here has run on a robot.

#include "doctest.h"

#include <cmath>
#include <string>

#include "mechanism_auton_rig.hpp"
#include "mechanism_driver_fixtures.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_controller.hpp"
#include "shulib/mechanism/driver_inputs.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/teleop_loop.hpp"

using auton_rig::RobotTwo;
using auton_rig::RobotX;
using shulib::hal::fake::FakeController;
using shulib::manipulation::MechanismOutcome;
using shulib::teleop::DriveShaper;
using shulib::teleop::HeadingSample;
namespace mech = shulib::mechanism;
namespace rx = mech_fixture::robot_x;
namespace r2 = mech_fixture::robot_two;
using Button = shulib::hal::ControllerButton;
using namespace shulib::units::literals;

namespace {

/// A scripted button event at a tick.
struct Ev {
    int tick;
    Button b;
    bool down;
};

/// Run robot x's driver loop for `ticks` ticks over `pacer` with `inputs`, applying `evs` at their
/// ticks (BEFORE that tick's disabled() returns); `each(k)` runs in perTick with the tick index.
template <class Each>
void drive(RobotX& x, shulib::motion::ITickPacer& pacer, mech::DriverInputs& inputs, FakeController& ctl,
           std::initializer_list<Ev> evs, int ticks, Each&& each) {
    DriveShaper shaper{shulib::teleop::DriverProfile{}};
    int k = 0;
    shulib::teleop::runDriverControl(
        x.chassis, ctl, pacer, shaper, rx::kLoop, x.svc, inputs,
        [&] {
            for (const Ev& e : evs) {
                if (e.tick == k) {
                    ctl.setPressed(e.b, e.down);
                }
            }
            return k++ >= ticks;
        },
        [] { return HeadingSample{}; },
        [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) { each(k - 1); });
}

}  // namespace

TEST_CASE("MECH-A4 P5d: robot x's driver file through the REAL runDriverControl -- the most recent input "
          "wins, a superseded hold commands nothing until released") {
    // Bug caught: a press already down at entry firing a preset; a hold that cannot take over a
    // running preset; a release that drops the arm instead of holding it HERE; a still-held hold that
    // fights a newer preset back (M23: the SUPERSEDED state removed); a toggle or hold-to-run that
    // does not reach its mechanism; a disable that leaves a motor energized or a claim taken.
    RobotX x;
    mech::DriverInputs inputs;
    inputs.bind(x.svc, rx::kControls);
    FakeController ctl;
    double targetAt5 = -1, voltsAt5 = -1, targetAt15 = -1, heldAt = -1, targetAt80 = -1, armAt240 = -1;
    double targetAt275 = -1, armAt349 = -1, armVoltsAt300 = 0, intakeAt390 = -1, intakeAt405 = -1;
    double flyAt540 = -1, flyAt600 = -1;
    bool atSpeedAt540 = false;
    bool r1DrovePastDown = false;
    drive(x, x.world, inputs, ctl,
          {{0, Button::Right, true}, {5, Button::Right, false},     // down ON the entry tick: ignored
           {10, Button::Right, true}, {15, Button::Right, false},   // a preset: score (95)
           {40, Button::R1, true}, {60, Button::R1, false},         // a hold mid-move wins; release: HERE
           {100, Button::Right, true}, {105, Button::Right, false}, // score again
           {250, Button::R1, true},                                 // R1 held ...
           {270, Button::Down, true}, {275, Button::Down, false},   // ... Down (stow) while R1 is STILL held
           {350, Button::R1, false},                                // R1 released (superseded)
           {380, Button::A, true}, {400, Button::A, false},         // the intake, held
           {430, Button::X, true}, {435, Button::X, false},         // the flywheel toggle: on
           {550, Button::X, true}, {555, Button::X, false}},        // again: off
          600, [&](int k) {
              if (k == 5) {
                  targetAt5 = x.svc.position(0).target();
                  voltsAt5 = x.armM.commandedVoltage().value();
              }
              if (k == 15) {
                  targetAt15 = x.svc.position(0).target();
              }
              if (k == 60) {
                  heldAt = x.svc.position(0).position();  // R1 released this tick: HOLD HERE
              }
              if (k == 80) {
                  targetAt80 = x.svc.position(0).target();
              }
              if (k == 240) {
                  armAt240 = x.armPlant.trueDeg();
              }
              if (k == 275) {
                  targetAt275 = x.svc.position(0).target();
              }
              if (k > 276 && k < 350 && x.armM.commandedVoltage().value() > 5.9 &&
                  x.armPlant.trueDeg() > 100.0) {
                  r1DrovePastDown = true;  // the superseded R1 jogging the arm up again
              }
              if (k == 300) {
                  armVoltsAt300 = x.armM.commandedVoltage().value();
              }
              if (k == 349) {
                  armAt349 = x.armPlant.trueDeg();
              }
              if (k == 390) {
                  intakeAt390 = x.intakeM.commandedVoltage().value();
              }
              if (k == 405) {
                  intakeAt405 = x.intakeM.commandedVoltage().value();
              }
              if (k == 540) {
                  flyAt540 = x.flyPlant.trueRpm();
                  atSpeedAt540 = x.svc.speed(2).atSpeed();
              }
              if (k == 599) {
                  flyAt600 = x.flyPlant.trueRpm();
              }
          });
    // the press already down at entry did nothing
    CHECK(targetAt5 == 0.0);
    CHECK(voltsAt5 == 0.0);
    // the seen press: score
    CHECK(targetAt15 == 95.0);
    // the hold took over mid-move and, released, holds HERE (inside the range, far from 95)
    CHECK(heldAt > 20.0);
    CHECK(heldAt < 80.0);
    CHECK(std::abs(targetAt80 - heldAt) < 1e-6);
    // score again, reached
    CHECK(std::abs(armAt240 - 95.0) < 1.5);
    // Down (stow) beat the still-held R1, and R1 then commanded NOTHING until released
    CHECK(targetAt275 == 0.0);
    CHECK_FALSE(r1DrovePastDown);
    CHECK(armVoltsAt300 < 0.0);  // the stow move drives it DOWN while R1 is still held
    CHECK(armAt349 < 10.0);
    // the intake while A is held, stopped on release
    CHECK(intakeAt390 == 12.0);
    CHECK(intakeAt405 == 0.0);
    // the flywheel toggle: at speed, then off
    CHECK(flyAt540 > 2900.0);
    CHECK(atSpeedAt540);
    CHECK(flyAt600 < flyAt540);
    CHECK_FALSE(x.svc.speed(2).spinning());
    // the disable: every motor kind safe, its claims released; the hood untouched (autonOnly)
    CHECK(x.armM.commandedVoltage().value() == 0.0);
    CHECK(x.flyM.commandedVoltage().value() == 0.0);
    CHECK(x.intakeM.commandedVoltage().value() == 0.0);
    CHECK_FALSE(x.mArm.claimed());
    CHECK_FALSE(x.mFly.claimed());
    CHECK_FALSE(x.mIntake.claimed());
}

TEST_CASE("MECH-A4 P8: EXACTLY one service tick per driver tick, whatever pacer the loop is handed") {
    // Bug caught: the loop's tickFromDriverLoop() plus a pace() through the service ticking it TWICE a
    // tick (every behaviour at double speed), or a raw pacer leaving it never ticked.
    for (int wiring = 0; wiring < 3; ++wiring) {
        CAPTURE(wiring);
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        FakeController ctl;
        shulib::motion::ITickPacer* pacer = wiring == 0   ? static_cast<shulib::motion::ITickPacer*>(&x.world)
                                            : wiring == 1 ? static_cast<shulib::motion::ITickPacer*>(&x.svc)
                                                          : static_cast<shulib::motion::ITickPacer*>(&x.guard);
        const long ticks0 = x.svc.ticks();
        const long paces0 = x.world.paces;
        int iterations = 0;
        bool tickedBeforePanel = true;
        drive(x, *pacer, inputs, ctl, {{50, Button::Right, true}, {60, Button::Right, false}}, 300, [&](int) {
            ++iterations;
            // M24's pin: the service has ALREADY ticked this tick when the panel runs
            tickedBeforePanel = tickedBeforePanel && (x.svc.ticks() - ticks0 == iterations);
        });
        CHECK(iterations == 300);
        CHECK(x.svc.ticks() - ticks0 == 300);
        CHECK(x.world.paces - paces0 == 300);
        CHECK(tickedBeforePanel);
    }
}

TEST_CASE("MECH-A4: a press is commanded AND ticked in the SAME tick, before the panel") {
    // Bug caught (M24): the service ticked after perTick — a panel would show last tick's command, and
    // a press would reach its motor one tick late.
    RobotX x;
    mech::DriverInputs inputs;
    inputs.bind(x.svc, rx::kControls);
    FakeController ctl;
    double intakeAtPress = -1.0;
    drive(x, x.world, inputs, ctl, {{20, Button::A, true}}, 40, [&](int k) {
        if (k == 20) {
            intakeAtPress = x.intakeM.commandedVoltage().value();
        }
    });
    CHECK(intakeAtPress == 12.0);
}

TEST_CASE("MECH-A4: both buttons of a hold = neither; a controller drop releases every hold that tick") {
    // Bug caught: two held buttons racing (the last one read wins), or a disconnected controller —
    // which reads 0 on every channel (HA-103) — leaving the intake running.
    RobotX x;
    mech::DriverInputs inputs;
    inputs.bind(x.svc, rx::kControls);
    FakeController ctl;
    double both = -1, dropped = -1, reconnected = -1;
    DriveShaper shaper{shulib::teleop::DriverProfile{}};
    int k = 0;
    shulib::teleop::runDriverControl(
        x.chassis, ctl, x.world, shaper, rx::kLoop, x.svc, inputs,
        [&] {
            if (k == 10) ctl.setPressed(Button::A, true);
            if (k == 20) ctl.setPressed(Button::B, true);  // both
            if (k == 30) ctl.setPressed(Button::B, false);
            if (k == 40) ctl.setConnected(false);  // A still "held" on the controller that dropped
            if (k == 50) ctl.setConnected(true);
            return k++ >= 60;
        },
        [] { return HeadingSample{}; },
        [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
            if (k - 1 == 25) both = x.intakeM.commandedVoltage().value();
            if (k - 1 == 45) dropped = x.intakeM.commandedVoltage().value();
            if (k - 1 == 55) reconnected = x.intakeM.commandedVoltage().value();
        });
    CHECK(both == 0.0);
    CHECK(dropped == 0.0);
    CHECK(reconnected == 12.0);  // a disconnect is a release, not an outside stop: A drives again
}

TEST_CASE("MECH-A4: a press must be SEEN -- a button held through a controller drop is not a new press on "
          "the reconnect") {
    // Bug caught: a disconnected controller read as "every button up", so the reconnect with the
    // toggle's button still down reads as a fresh press and flips the flywheel off (or a preset fires
    // again) without the driver touching anything.
    RobotX x;
    mech::DriverInputs inputs;
    inputs.bind(x.svc, rx::kControls);
    FakeController ctl;
    DriveShaper shaper{shulib::teleop::DriverProfile{}};
    int k = 0;
    bool spinningAfterReconnect = false;
    shulib::teleop::runDriverControl(
        x.chassis, ctl, x.world, shaper, rx::kLoop, x.svc, inputs,
        [&] {
            if (k == 10) ctl.setPressed(Button::X, true);  // the toggle: on — and X stays down
            if (k == 30) ctl.setConnected(false);
            if (k == 40) ctl.setConnected(true);  // X still down: not a press
            return k++ >= 60;
        },
        [] { return HeadingSample{}; },
        [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
            if (k - 1 == 55) spinningAfterReconnect = x.svc.speed(2).spinning();
        });
    CHECK(spinningAfterReconnect);
}

TEST_CASE("MECH-A4 (N10): a STALL while a button is held supersedes it -- it commands nothing until "
          "released, and drives again after a release and a new press") {
    // Bug caught: a held button re-driving a mechanism its own stall signature just stopped — the jam
    // the signature exists to end, restarted every tick (M23's class: the SUPERSEDED state removed).
    RobotX x;
    x.liftPlant.setObstacleDeg(6.0 / (auton_rig::kPi * 1.5) * 360.0);  // something above the lift at 6 in
    mech::DriverInputs inputs;
    inputs.bind(x.svc, rx::kControls);
    FakeController ctl;
    double voltsLate = -1.0;
    double voltsAgain = -1.0;
    drive(x, x.world, inputs, ctl, {{10, Button::L1, true}, {250, Button::L1, false}, {260, Button::L2, true}}, 280,
          [&](int k) {
              if (k == 240) voltsLate = x.liftM1.commandedVoltage().value();
              if (k == 270) voltsAgain = x.liftM1.commandedVoltage().value();
          });
    CHECK(x.count("MECH", "'lift': STALLED") == 1);
    CHECK(voltsLate == 0.0);     // L1 still held: superseded — nothing commanded, its safe state
    CHECK(voltsAgain == -12.0);  // released, then L2: it drives again
}

TEST_CASE("MECH-A4 P2i: homeAtDriverStart homes ONCE at entry when nothing homed it, and is skipped when "
          "the auton did; homeOnPress homes on a press") {
    // Bug caught: a stall- or switch-homed mechanism whose buttons are dead after a restart (U5), a
    // driver-start home that re-homes what the auton homed (match time and a moved zero), or a homing
    // button that does nothing.
    using namespace shulib::mechanism::words;
    static constexpr Arm armY{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60),
                              .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                              .range = {0, 160}, .gravity = cosine(1.6_volt, 45),
                              .home = homeByStall(0, -3_volt, Stall{.amps = 0.45, .belowMotorRpm = 3, .lasting = 0.1_s}),
                              .presets = {{"score", 95}}};
    static constexpr MechanismTable kY = mechanisms(wire(armY, motors(1)));
    static constexpr DriverControls kYControls =
        controls(kY, whileHeld(armY, Button::R1, Button::R2, 6_volt), preset(armY["score"], Button::Right, 1.5_s),
                 homeOnPress(armY, Button::Left, 1.5_s), homeAtDriverStart(armY, 1.5_s));
    for (int autonHomed = 0; autonHomed < 2; ++autonHomed) {
        CAPTURE(autonHomed);
        auton_rig::TankWorld w;
        shulib::hal::fake::FakeTelemetrySink sink;
        shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
        motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
        shulib::hal::fake::FakeMotor armM;
        shulib::hal::IMotor* am[1]{&armM};
        shulib::hal::MotorMechanism mArm{am, shulib::hal::BrakeMode::Hold, "arm"};
        shulib::hal::fake::FakeMotor* aF[1]{&armM};
        auto cfg = auton_rig::armPlantConfig();
        cfg.startDeg = 30.0;  // where the arm happens to be when the program starts
        shulib::sim::ArmPlant plant{aF, cfg};
        w.rig = &rig;
        w.onPace = [&plant] { plant.step(0.01); };
        mech::MechanismService svc{w};
        shulib::chassis::Chassis chassis{rig.deps, svc, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}};
        const mech::Devices devs[1] = {{.motors = &mArm}};
        svc.bind(kY, devs, rig.h.clock(), rig.latch, sink);
        REQUIRE_FALSE(svc.position(0).homed());
        if (autonHomed == 1) {
            svc.position(0).home(1.5_s);
            for (int i = 0; i < 200 && svc.position(0).running(); ++i) svc.pace();
            REQUIRE(svc.position(0).homed());
        }
        const std::uint32_t begun0 = svc.position(0).stepsBegun();
        mech::DriverInputs inputs;
        inputs.bind(svc, kYControls);
        FakeController ctl;
        DriveShaper shaper{shulib::teleop::DriverProfile{}};
        int k = 0;
        std::uint32_t begunAfterEntry = 0;
        shulib::teleop::runDriverControl(
            chassis, ctl, svc, shaper, rx::kLoop, svc, inputs,
            [&] {
                if (k == 150) ctl.setPressed(Button::Left, true);  // homeOnPress
                if (k == 155) ctl.setPressed(Button::Left, false);
                if (k == 220) ctl.setPressed(Button::Right, true);  // then a preset
                if (k == 225) ctl.setPressed(Button::Right, false);
                return k++ >= 400;
            },
            [] { return HeadingSample{}; },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                if (k - 1 == 0) begunAfterEntry = svc.position(0).stepsBegun();
            });
        CHECK(begunAfterEntry - begun0 == (autonHomed == 1 ? 0u : 1u));  // the driver-start home: once, or not at all
        CHECK(svc.position(0).homed());
        CHECK(svc.position(0).stepsBegun() - begun0 == (autonHomed == 1 ? 2u : 3u));  // + the press's home + the preset
        CHECK(svc.position(0).lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(std::abs(plant.trueDeg() - 93.0) < 3.0);  // reached "score" (95 reads ~2 deg off the true stop)
    }
}

TEST_CASE("MECH-A4 (U5): a switch-homed lift, the program restarted OFF its switch -- homeAtDriverStart "
          "homes it on the first tick, and its buttons then move it") {
    // Bug caught: the dead-buttons case U5 refuses at build — here the runtime half: after a restart
    // with the lift up off its switch, nothing has homed it, and without the driver-start home its
    // hold-to-run would command 0 V (no .unhomedVolts) for the whole match.
    RobotX x{true, /*liftOnSwitchAtBoot=*/false};
    REQUIRE_FALSE(x.svc.position(1).homed());
    x.world.liftSwitchFollowsPlant = true;  // the switch is real: pressed only at the bottom
    // the lift is UP when driver control starts (the plant starts at 0 in this rig: raise it by hand,
    // the service not ticked)
    for (int i = 0; i < 60; ++i) {
        x.liftM1.setVoltage(shulib::units::Voltage{8.0});
        x.liftM2.setVoltage(shulib::units::Voltage{8.0});
        x.liftPlant.step(0.01);
    }
    x.liftM1.setVoltage(shulib::units::Voltage{0.0});
    x.liftM2.setVoltage(shulib::units::Voltage{0.0});
    const double upAt = x.liftInches();
    REQUIRE(upAt > 2.0);
    mech::DriverInputs inputs;
    inputs.bind(x.svc, rx::kControls);
    FakeController ctl;
    bool homedEarly = false;
    double liftAfterHold = -1.0;
    drive(x, x.world, inputs, ctl, {{250, Button::L1, true}, {300, Button::L1, false}}, 400, [&](int k) {
        if (k == 200) homedEarly = x.svc.position(1).homed();
        if (k == 320) liftAfterHold = x.liftInches();
    });
    CHECK(homedEarly);  // the driver-start home drove it down onto its switch
    CHECK(x.count("MECH", "'lift': SUCCEEDED") >= 1);
    CHECK(liftAfterHold > 1.0);  // L1 then raised it: its buttons work
}

TEST_CASE("MECH-A4 P9c: robot two's claws -- the FIRST entry sends CLOSED whatever the auton left, A "
          "toggles, R1 drives pivot 6 at 12 V, a disable keeps the claw's level") {
    // Bug caught: T4's claw rules lost in the move to the service — the start position re-sent at every
    // entry (a claw that drops what it holds), a disable that touches the claw, a pivot not at full
    // power.
    RobotTwo two;
    mech::DriverInputs inputs;
    inputs.bind(two.svc, r2::kControls);
    FakeController ctl;
    two.lineA.set(true);  // the auton left the claw OPEN
    bool lineAtEntry[2]{};
    bool lineAfterA[2]{};
    double v6Held[2]{};
    for (int entry = 0; entry < 2; ++entry) {
        int k = 0;
        DriveShaper shaper{r2::kFeel};
        shulib::teleop::runDriverControl(
            two.chassis, ctl, two.svc, shaper, r2::kLoop, two.svc, inputs,
            [&] {
                if (k == 1) lineAtEntry[entry] = two.lineA.commanded();
                if (k == 20) ctl.setPressed(Button::A, true);
                if (k == 25) ctl.setPressed(Button::A, false);
                if (k == 30) lineAfterA[entry] = two.lineA.commanded();
                if (k == 40) ctl.setPressed(Button::R1, true);
                if (k == 50) v6Held[entry] = two.p6.commandedVoltage().value();
                if (k == 60) ctl.setPressed(Button::R1, false);
                return k++ > 80;
            },
            [] { return HeadingSample{}; },
            [](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {});
        CHECK(two.p6.commandedVoltage().value() == 0.0);  // the disable: the pivot safe
    }
    CHECK_FALSE(lineAtEntry[0]);  // first entry: START (closed = Low)
    CHECK(lineAfterA[0]);         // A opened it
    CHECK(lineAtEntry[1]);        // second entry: KEPT (open) — no START again
    CHECK_FALSE(lineAfterA[1]);   // A closed it
    CHECK(v6Held[0] == 12.0);
    CHECK(v6Held[1] == 12.0);
    CHECK_FALSE(two.lineA.commanded());  // the disable kept it closed
}

TEST_CASE("MECH-A4: the loop's preconditions -- the service and BOUND inputs together, no T3 binding beside "
          "them, no control on the re-zero button") {
    // Bug caught: two driver paths commanding one mechanism, inputs never bound (every control silent),
    // or a control the re-zero button also fires.
    RobotX x;
    FakeController ctl;
    DriveShaper shaper{shulib::teleop::DriverProfile{}};
    const auto never = [] { return true; };
    const auto heading = [] { return HeadingSample{}; };
    const auto panel = [](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {};
    {
        mech::DriverInputs unbound;
        CHECK_THROWS_AS(shulib::teleop::runDriverControl(x.chassis, ctl, x.world, shaper, rx::kLoop, x.svc, unbound,
                                                         never, heading, panel),
                        shulib::PreconditionError);
    }
    {
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        shulib::teleop::DriverLoopConfig onR1 = rx::kLoop;
        onR1.rezeroButton = Button::R1;  // the arm's hold takes R1
        CHECK_THROWS_AS(shulib::teleop::runDriverControl(x.chassis, ctl, x.world, shaper, onR1, x.svc, inputs, never,
                                                         heading, panel),
                        shulib::PreconditionError);
    }
    {
        // both paths at once (only the loop's one body can be handed both): refused, nothing commanded
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        shulib::hal::fake::FakeMotor m;
        shulib::hal::IMotor* mm[1]{&m};
        shulib::hal::MotorMechanism other{mm, shulib::hal::BrakeMode::Coast, "other"};
        shulib::teleop::MechanismBinding binding{other, Button::Left, Button::Right, shulib::units::Voltage{6.0}};
        shulib::teleop::MechanismBinding* const list[1]{&binding};
        CHECK_THROWS_AS(shulib::teleop::detail::driverLoop(x.chassis, ctl, x.world, shaper, rx::kLoop,
                                                           std::span<shulib::teleop::MechanismBinding* const>{list},
                                                           std::span<shulib::teleop::ToggleBinding* const>{}, &x.svc,
                                                           &inputs, never, heading, panel),
                        shulib::PreconditionError);
        CHECK(m.commandedVoltage().value() == 0.0);
    }
}

TEST_CASE("MECH-A4: DriverInputs::bind checks a driver file against the robot the SERVICE is bound to") {
    // Bug caught: a driver file built for one robot commanding another's same-named mechanism — a
    // control of the wrong kind, a preset at a stranger's value, a pneumatic position by index only.
    {
        RobotTwo two;  // robot x's controls on robot two: no 'arm'
        mech::DriverInputs inputs;
        CHECK_THROWS_AS(inputs.bind(two.svc, rx::kControls), shulib::PreconditionError);
        CHECK_FALSE(inputs.bound());
    }
    {
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        CHECK_THROWS_AS(inputs.bind(x.svc, rx::kControls), shulib::PreconditionError);  // twice
    }
    {
        using namespace shulib::mechanism::words;
        // A same-NAMED "arm" whose "score" is 80, not robot x's 95.
        static constexpr Arm otherArm{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60),
                                      .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
                                      .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0),
                                      .presets = {{"score", 80}}};
        static constexpr MechanismTable kOther = mechanisms(wire(otherArm, motors(1)));
        static constexpr DriverControls kOtherControls =
            controls(kOther, preset(otherArm["score"], Button::Right, 1_s));
        RobotX x;
        mech::DriverInputs inputs;
        CHECK_THROWS_AS(inputs.bind(x.svc, kOtherControls), shulib::PreconditionError);
    }
    {
        // an unbound service
        auton_rig::TankWorld w;
        mech::MechanismService svc{w};
        mech::DriverInputs inputs;
        CHECK_THROWS_AS(inputs.bind(svc, rx::kControls), shulib::PreconditionError);
        CHECK_THROWS_AS(inputs.enter(), shulib::PreconditionError);
    }
}

TEST_CASE("MECH-A4: the build-checked lists -- avoids(), every word at its limits, robot y's every word") {
    // Bug caught: a list that loses a control, avoids() blind to a hold's BACK button, or a word that
    // records the wrong kind.
    static_assert(rx::kControls.count == 9);
    static_assert(!rx::kControls.avoids(Button::R2));  // a hold's BACK
    static_assert(!rx::kControls.avoids(Button::X));   // a toggle
    static_assert(rx::kControls.avoids(Button::Y));    // the re-zero
    static_assert(rx::kControls.avoids(Button::Left));
    static_assert(!rx::kAltControls.avoids(Button::Left));  // a homeOnPress
    static_assert(r2::kControls.count == 3);
    static_assert(mech_fixture::robot_y::kControls.count == 10);
    static_assert(mech_fixture::robot_y::kControls.c[3].kind == mech::ControlKind::HomeAtDriverStart);
    static_assert(mech_fixture::robot_y::kControls.avoids(Button::R1) == false);
    static_assert(mech::detail::realBound(1e-9) && !mech::detail::realBound(0.0));
    CHECK(rx::kControls.all().size() == 9);
    CHECK(rx::kControls.c[8].kind == mech::ControlKind::AutonOnly);
    CHECK(rx::kControls.c[6].value == 3000.0);  // toggle(flywheel["shoot"]) carries the named speed
    CHECK(std::string{rx::kControls.c[6].presetName} == "shoot");
}

TEST_CASE("MECH-A4: DriverInputs::bind refuses a control whose mechanism is another KIND on this robot") {
    // Bug caught: a driver file whose "hood" is a power roller bound to a robot whose "hood" is a
    // pneumatic — its hold-to-run would index a pneumatic slot as a power one.
    using namespace shulib::mechanism::words;
    static constexpr Power hoodRoller{.name = "hood", .motors = {Forward}, .cartridge = Green, .safe = Coast};
    static constexpr MechanismTable kHood = mechanisms(wire(hoodRoller, motors(1)));
    static constexpr DriverControls kHoodControls =
        controls(kHood, whileHeld(hoodRoller, Button::R1, Button::R2, 6_volt));
    RobotX x;
    mech::DriverInputs inputs;
    CHECK_THROWS_AS(inputs.bind(x.svc, kHoodControls), shulib::PreconditionError);
    CHECK_FALSE(inputs.bound());
}

TEST_CASE("MECH-A4: the loop refuses driver inputs bound to ANOTHER service") {
    // Bug caught: a program with two services (or a test rig) handing the loop one service and inputs
    // bound to the other — the buttons would command mechanisms the loop never ticks.
    RobotX a;
    RobotX b;
    mech::DriverInputs inputs;
    inputs.bind(a.svc, rx::kControls);
    FakeController ctl;
    DriveShaper shaper{shulib::teleop::DriverProfile{}};
    CHECK(inputs.boundTo(a.svc));
    CHECK_FALSE(inputs.boundTo(b.svc));
    CHECK_THROWS_AS(shulib::teleop::runDriverControl(
                        b.chassis, ctl, b.world, shaper, rx::kLoop, b.svc, inputs, [] { return true; },
                        [] { return HeadingSample{}; },
                        [](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {}),
                    shulib::PreconditionError);
}

// ═══ MECH-A4's review round (2026-10-27): each case below is a finding's test ═══════════════════

namespace {

/// Raise robot x's lift off its switch by hand (the service not ticked): a restart mid-match.
void raiseLiftOffItsSwitch(RobotX& x) {
    for (int i = 0; i < 60; ++i) {
        x.liftM1.setVoltage(shulib::units::Voltage{8.0});
        x.liftM2.setVoltage(shulib::units::Voltage{8.0});
        x.liftPlant.step(0.01);
    }
    x.liftM1.setVoltage(shulib::units::Voltage{0.0});
    x.liftM2.setVoltage(shulib::units::Voltage{0.0});
}

/// How many of the lift's steps were CANCELLED because a newer command REPLACED them (the U5 defect's
/// line) — a disable's "driver control ended" is another cancel, and not this one.
int liftReplaced(const RobotX& x) {
    int n = 0;
    for (const std::string& l : x.lines("MECH")) {
        n += l.starts_with("'lift': CANCELLED") && l.find("(replaced by a newer command)") != std::string::npos ? 1 : 0;
    }
    return n;
}

}  // namespace

TEST_CASE("MECH-A4 (review): a controller DROP with every button UP fires NOTHING -- no toggle flips, no "
          "preset or homing press fires; a real press after the reconnect still works") {
    // Bug caught (the review's blocker): a disconnected tick fed "held" into each one-button control's
    // edge AND RETURNED the edge — a rising edge whenever the button was up — so a drop flipped robot
    // two's claw open on what it held, fired robot x's three arm presets and its flywheel toggle, and
    // (kAltControls) started a homing push. The overlay's form, restored: a drop is never a press.
    {
        RobotTwo two;
        mech::DriverInputs inputs;
        inputs.bind(two.svc, r2::kControls);
        FakeController ctl;
        DriveShaper shaper{r2::kFeel};
        int k = 0;
        bool lineBefore = true, lineDuring = true, lineAfter = true, lineAfterPress = false;
        std::uint32_t seqBefore = 0, seqAfter = 1;
        shulib::teleop::runDriverControl(
            two.chassis, ctl, two.svc, shaper, r2::kLoop, two.svc, inputs,
            [&] {
                if (k == 30) ctl.setConnected(false);
                if (k == 40) ctl.setConnected(true);
                if (k == 60) ctl.setPressed(Button::A, true);  // a real press after the reconnect
                if (k == 65) ctl.setPressed(Button::A, false);
                return k++ >= 80;
            },
            [] { return HeadingSample{}; },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                if (k - 1 == 29) {
                    lineBefore = two.lineA.commanded();
                    seqBefore = two.svc.slot(2).commandSeq();
                }
                if (k - 1 == 35) lineDuring = two.lineA.commanded();
                if (k - 1 == 55) {
                    lineAfter = two.lineA.commanded();
                    seqAfter = two.svc.slot(2).commandSeq();
                }
                if (k - 1 == 70) lineAfterPress = two.lineA.commanded();
            });
        CHECK_FALSE(lineBefore);  // closed (START) since entry
        CHECK_FALSE(lineDuring);  // the drop did not open it
        CHECK_FALSE(lineAfter);   // nor the reconnect
        CHECK(seqAfter == seqBefore);
        CHECK(lineAfterPress);    // A, really pressed, still opens it
    }
    {
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        FakeController ctl;
        std::uint32_t armSeq29 = 0, armSeq55 = 1, flySeq29 = 0, flySeq55 = 1;
        double target55 = -1.0, target75 = -1.0;
        bool spinning55 = true;
        drive(x, x.world, inputs, ctl, {{60, Button::Right, true}, {65, Button::Right, false}}, 80, [&](int k) {
            if (k == 29) {
                armSeq29 = x.svc.slot(0).commandSeq();
                flySeq29 = x.svc.slot(2).commandSeq();
                ctl.setConnected(false);  // read by the NEXT tick
            }
            if (k == 39) ctl.setConnected(true);
            if (k == 55) {
                armSeq55 = x.svc.slot(0).commandSeq();
                flySeq55 = x.svc.slot(2).commandSeq();
                target55 = x.svc.position(0).target();
                spinning55 = x.svc.speed(2).spinning();
            }
            if (k == 75) target75 = x.svc.position(0).target();
        });
        CHECK(armSeq55 == armSeq29);  // no preset fired (stow, score, high — each would bump it)
        CHECK(flySeq55 == flySeq29);  // the toggle did not fire
        CHECK(target55 == 0.0);
        CHECK_FALSE(spinning55);
        CHECK(target75 == 95.0);  // Right, really pressed after the reconnect: score
    }
    {
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kAltControls);  // homeOnPress(lift, Left) and the hood's three presets
        FakeController ctl;
        std::uint32_t liftBegun29 = 0, liftBegun55 = 1, hoodSeq29 = 0, hoodSeq55 = 1;
        drive(x, x.world, inputs, ctl, {}, 60, [&](int k) {
            if (k == 29) {
                liftBegun29 = x.svc.position(1).stepsBegun();
                hoodSeq29 = x.svc.slot(4).commandSeq();
                ctl.setConnected(false);
            }
            if (k == 39) ctl.setConnected(true);
            if (k == 55) {
                liftBegun55 = x.svc.position(1).stepsBegun();
                hoodSeq55 = x.svc.slot(4).commandSeq();
            }
        });
        CHECK(liftBegun55 == liftBegun29);  // no homing push begun
        CHECK(hoodSeq55 == hoodSeq29);      // no hood preset fired
        CHECK_FALSE(x.lineC.commanded());   // the hood still at its START, 'down'
        CHECK_FALSE(x.lineD.commanded());
    }
}

TEST_CASE("MECH-A4 (verification): a button pressed DURING a controller drop and still down at the "
          "reconnect is NOT a press -- released and pressed again, it is") {
    // Bug caught (the verification round's VR-1): a disconnected tick that only returned false, without
    // RECORDING the unknown level as held. The edge then still holds the level from BEFORE the drop
    // ("up"), so a button the driver pressed while the robot was not responding, still down when the
    // controller comes back, reads as a fresh press on the reconnect tick: robot two's claw flips, robot
    // x's flywheel toggles on and a preset fires, kAltControls begins a homing push — no new touch. The
    // two cases above cannot see it: a drop with every button UP, and a button held from BEFORE the drop
    // (whose edge already reads "held"). Here each button is up before the drop, pressed during it (tick
    // 35, the controller disconnected from tick 30), still down at the reconnect (tick 40) and through
    // tick 55; released at 58 and pressed again at 60, it fires.
    {
        RobotTwo two;
        mech::DriverInputs inputs;
        inputs.bind(two.svc, r2::kControls);
        FakeController ctl;
        DriveShaper shaper{r2::kFeel};
        int k = 0;
        bool lineBefore = true, lineAfter = true, lineAfterPress = false;
        std::uint32_t seqBefore = 0, seqAfter = 1, seqAfterPress = 0;
        shulib::teleop::runDriverControl(
            two.chassis, ctl, two.svc, shaper, r2::kLoop, two.svc, inputs,
            [&] {
                if (k == 30) ctl.setConnected(false);
                if (k == 35) ctl.setPressed(Button::A, true);  // pressed while the robot is not listening
                if (k == 40) ctl.setConnected(true);           // A still DOWN: not a press
                if (k == 58) ctl.setPressed(Button::A, false);
                if (k == 60) ctl.setPressed(Button::A, true);  // a real press: it fires
                if (k == 65) ctl.setPressed(Button::A, false);
                return k++ >= 80;
            },
            [] { return HeadingSample{}; },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                if (k - 1 == 29) {
                    lineBefore = two.lineA.commanded();
                    seqBefore = two.svc.slot(2).commandSeq();
                }
                if (k - 1 == 55) {
                    lineAfter = two.lineA.commanded();
                    seqAfter = two.svc.slot(2).commandSeq();
                }
                if (k - 1 == 70) {
                    lineAfterPress = two.lineA.commanded();
                    seqAfterPress = two.svc.slot(2).commandSeq();
                }
            });
        CHECK_FALSE(lineBefore);  // closed (START) since entry
        CHECK_FALSE(lineAfter);   // the reconnect with A down did not open it
        CHECK(seqAfter == seqBefore);
        CHECK(lineAfterPress);    // released and pressed again: it opens
        CHECK(seqAfterPress == seqBefore + 1);
    }
    {
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        FakeController ctl;
        std::uint32_t armSeq29 = 0, armSeq55 = 1, flySeq29 = 0, flySeq55 = 1;
        double target55 = -1.0, target75 = -1.0;
        bool spinning55 = true, spinning75 = false;
        drive(x, x.world, inputs, ctl,
              {{35, Button::X, true}, {35, Button::Right, true},     // during the drop
               {58, Button::X, false}, {58, Button::Right, false},   // released after the reconnect
               {60, Button::X, true}, {60, Button::Right, true},     // real presses
               {65, Button::X, false}, {65, Button::Right, false}},
              80, [&](int k) {
                  if (k == 29) {
                      armSeq29 = x.svc.slot(0).commandSeq();
                      flySeq29 = x.svc.slot(2).commandSeq();
                      ctl.setConnected(false);  // read by the NEXT tick (30)
                  }
                  if (k == 39) ctl.setConnected(true);  // tick 40: X and Right still DOWN
                  if (k == 55) {
                      armSeq55 = x.svc.slot(0).commandSeq();
                      flySeq55 = x.svc.slot(2).commandSeq();
                      target55 = x.svc.position(0).target();
                      spinning55 = x.svc.speed(2).spinning();
                  }
                  if (k == 75) {
                      target75 = x.svc.position(0).target();
                      spinning75 = x.svc.speed(2).spinning();
                  }
              });
        CHECK(armSeq55 == armSeq29);  // Right down at the reconnect: no "score" preset
        CHECK(flySeq55 == flySeq29);  // X down at the reconnect: the toggle did not fire
        CHECK(target55 == 0.0);
        CHECK_FALSE(spinning55);
        CHECK(target75 == 95.0);      // Right, released and pressed again: score
        CHECK(spinning75);            // X, released and pressed again: on
    }
    {
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kAltControls);  // homeOnPress(lift, Left) and the hood's three presets
        FakeController ctl;
        std::uint32_t liftBegun29 = 0, liftBegun55 = 1, liftBegun75 = 0, hoodSeq29 = 0, hoodSeq55 = 1,
                      hoodSeq75 = 0;
        drive(x, x.world, inputs, ctl,
              {{35, Button::Left, true}, {35, Button::Right, true},
               {58, Button::Left, false}, {58, Button::Right, false},
               {60, Button::Left, true}, {60, Button::Right, true},
               {65, Button::Left, false}, {65, Button::Right, false}},
              80, [&](int k) {
                  if (k == 29) {
                      liftBegun29 = x.svc.position(1).stepsBegun();
                      hoodSeq29 = x.svc.slot(4).commandSeq();
                      ctl.setConnected(false);
                  }
                  if (k == 39) ctl.setConnected(true);
                  if (k == 55) {
                      liftBegun55 = x.svc.position(1).stepsBegun();
                      hoodSeq55 = x.svc.slot(4).commandSeq();
                  }
                  if (k == 75) {
                      liftBegun75 = x.svc.position(1).stepsBegun();
                      hoodSeq75 = x.svc.slot(4).commandSeq();
                  }
              });
        CHECK(liftBegun55 == liftBegun29);      // Left down at the reconnect: no homing push begun
        CHECK(hoodSeq55 == hoodSeq29);          // Right down at the reconnect: no hood preset
        CHECK(liftBegun75 == liftBegun29 + 1);  // Left, released and pressed again: the home begins
        CHECK(hoodSeq75 == hoodSeq29 + 1);      // Right, released and pressed again: 'middle'
        CHECK(x.lineC.commanded());             // 'middle' = {High, Low}
        CHECK_FALSE(x.lineD.commanded());
    }
}

TEST_CASE("MECH-A4 (review, U5): while the DRIVER-START home runs, the lift's own buttons WAIT -- a touch "
          "at entry or during the home never cancels it, and the buttons work after it") {
    // Bug caught (the review's U5 finding): the lift's own hold, touched while its driver-start home
    // ran, replaced the home ("replaced by a newer command", CANCELLED) and — unhomed, no .unhomedVolts —
    // then commanded 0 V: the lift never homed and its buttons were dead for the match, exactly what
    // U5 exists to prevent. Each case: robot x restarted with the lift OFF its switch.
    struct Touch {
        const char* what;
        int at;  // -1: held before entry
        int release;
        Button b;
    };
    const Touch touches[] = {
        {"nothing touched", 1000, 1001, Button::L1},
        {"L1 held THROUGH entry, released at 0.10 s", -1, 10, Button::L1},
        {"L2 (down) tapped at 0.20 s", 20, 25, Button::L2},
        {"L1 (up) tapped at 0.50 s", 50, 55, Button::L1},
    };
    for (const Touch& t : touches) {
        CAPTURE(t.what);
        RobotX x{true, /*liftOnSwitchAtBoot=*/false};
        x.world.liftSwitchFollowsPlant = true;
        raiseLiftOffItsSwitch(x);
        REQUIRE(x.liftInches() > 2.0);
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        FakeController ctl;
        if (t.at < 0) ctl.setPressed(t.b, true);
        bool homed300 = false;
        double before = -1.0, after = -1.0;
        drive(x, x.world, inputs, ctl,
              {{t.at, t.b, true}, {t.release, t.b, false}, {400, Button::L1, true}, {450, Button::L1, false}}, 500,
              [&](int k) {
                  if (k == 300) homed300 = x.svc.position(1).homed();
                  if (k == 399) before = x.liftInches();
                  if (k == 455) after = x.liftInches();
              });
        CHECK(homed300);
        CHECK(liftReplaced(x) == 0);  // the home was never replaced
        CHECK(after > before + 0.5);                        // L1 later raises it: the buttons work
    }
}

TEST_CASE("MECH-A4 (review, U5): a hold held THROUGH the driver-start home is superseded -- nothing until it "
          "is released after the home; a preset or homing press during the home is dropped") {
    // Bug caught: a held button that drives the instant the home ends (a lift lurching up off the
    // switch it just found, with the driver's thumb never lifted), or a preset / homeOnPress press that
    // cancels the driver-start home (the dead buttons again) or fires late, after it.
    using namespace mech_fixture::robot_x;
    static constexpr DriverControls kLiftOnly = controls(kRobotXMechanisms,
        autonOnly(arm),
        whileHeld(lift, Button::L1, Button::L2, 12_volt),
        preset(lift["high"], Button::Up, 4_s),
        homeOnPress(lift, Button::Left, 1.5_s),
        homeAtDriverStart(lift, 1.5_s),
        autonOnly(flywheel),
        autonOnly(intake),
        autonOnly(hood));
    {
        // L1 held from before entry to 3.0 s: the home runs, ends, and L1 still commands nothing
        RobotX x{true, false};
        x.world.liftSwitchFollowsPlant = true;
        raiseLiftOffItsSwitch(x);
        mech::DriverInputs inputs;
        inputs.bind(x.svc, kLiftOnly);
        FakeController ctl;
        ctl.setPressed(Button::L1, true);
        double voltsAt250 = -1.0, liftAt299 = -1.0, voltsAt320 = -1.0;
        drive(x, x.world, inputs, ctl, {{300, Button::L1, false}, {310, Button::L1, true}}, 340, [&](int k) {
            if (k == 250) voltsAt250 = x.liftM1.commandedVoltage().value();
            if (k == 299) liftAt299 = x.liftInches();
            if (k == 320) voltsAt320 = x.liftM1.commandedVoltage().value();
        });
        CHECK(x.svc.position(1).homed());
        CHECK(voltsAt250 != 12.0);   // the home ended long before; L1, never released, does not drive
        CHECK(liftAt299 < 0.5);      // it stayed at the switch it found
        CHECK(voltsAt320 == 12.0);   // released, pressed again: it drives
    }
    {
        // Up (a preset) at 0.30 s and Left (homeOnPress) at 0.40 s, both DURING the home: dropped
        RobotX x{true, false};
        x.world.liftSwitchFollowsPlant = true;
        raiseLiftOffItsSwitch(x);
        mech::DriverInputs inputs;
        inputs.bind(x.svc, kLiftOnly);
        FakeController ctl;
        const std::uint32_t begun0 = x.svc.position(1).stepsBegun();
        std::uint32_t begunAt250 = 0;
        double targetAt250 = -1.0, targetAt320 = -1.0, voltsAt345 = 0.0;
        drive(x, x.world, inputs, ctl,
              {{30, Button::Up, true}, {35, Button::Up, false}, {40, Button::Left, true}, {45, Button::Left, false},
               {300, Button::Up, true}, {305, Button::Up, false}, {340, Button::L2, true}},
              360, [&](int k) {
                  if (k == 250) {
                      begunAt250 = x.svc.position(1).stepsBegun();
                      targetAt250 = x.svc.position(1).target();
                  }
                  if (k == 320) targetAt320 = x.svc.position(1).target();
                  if (k == 345) voltsAt345 = x.liftM1.commandedVoltage().value();
              });
        CHECK(begunAt250 - begun0 == 1u);  // the driver-start home only: both presses dropped
        CHECK(x.svc.position(1).homed());
        CHECK(targetAt250 < 1.0);    // it holds at the home, not at 'high'
        CHECK(targetAt320 == 22.0);  // Up after the home: 'high'
        // ... and only the DRIVER-START home makes a hold wait: L2 during that preset's move wins at
        // once (the most recent input wins — Q8), the move replaced
        CHECK(voltsAt345 == -12.0);
        CHECK(liftReplaced(x) == 1);  // the preset's move, replaced by L2 — never the home
    }
}

TEST_CASE("MECH-A4 (review): the flywheel toggle turns it OFF on its second press, before any disable") {
    // Bug caught: a toggle that never turns the flywheel off (P5d's checks ran after the disable, which
    // stops it anyway, and rpm falling through noise is no proof).
    RobotX x;
    mech::DriverInputs inputs;
    inputs.bind(x.svc, rx::kControls);
    FakeController ctl;
    bool onAt150 = false, onAt350 = true;
    double targetAt150 = -1.0;
    drive(x, x.world, inputs, ctl,
          {{10, Button::X, true}, {15, Button::X, false}, {200, Button::X, true}, {205, Button::X, false}}, 400,
          [&](int k) {
              if (k == 150) {
                  onAt150 = x.svc.speed(2).spinning();
                  targetAt150 = x.svc.speed(2).targetRpm();
              }
              if (k == 350) onAt350 = x.svc.speed(2).spinning();
          });
    CHECK(onAt150);
    CHECK(std::abs(targetAt150 - 3000.0) < 1e-6);  // the NAMED speed
    CHECK_FALSE(onAt350);                           // the second press stopped it
}

TEST_CASE("MECH-A4 (review): a PNEUMATIC preset sends ITS position's levels; a FLYWHEEL preset spins to "
          "ITS speed") {
    // Bug caught: a pneumatic preset that always sends position 0, or a flywheel preset at the wrong
    // speed — branches no other test pressed (kAltControls' hood presets were only compiled).
    {
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kAltControls);
        FakeController ctl;
        bool up[2]{}, middle[2]{true, true}, down[2]{true, true};
        drive(x, x.world, inputs, ctl,
              {{10, Button::Up, true}, {15, Button::Up, false}, {60, Button::Right, true}, {65, Button::Right, false},
               {110, Button::Down, true}, {115, Button::Down, false}},
              150, [&](int k) {
                  bool* into = k == 50 ? up : k == 100 ? middle : k == 140 ? down : nullptr;
                  if (into != nullptr) {
                      into[0] = x.lineC.commanded();
                      into[1] = x.lineD.commanded();
                  }
              });
        CHECK(up[0]);  // "up" = High, High
        CHECK(up[1]);
        CHECK(middle[0]);  // "middle" = High, Low
        CHECK_FALSE(middle[1]);
        CHECK_FALSE(down[0]);  // "down" = Low, Low
        CHECK_FALSE(down[1]);
    }
    {
        using namespace mech_fixture::robot_x;
        static constexpr DriverControls kFly = controls(kRobotXMechanisms,
            autonOnly(arm), autonOnly(lift),
            preset(flywheel["shoot"], Button::B, 2_s),
            preset(flywheel.at(1500), Button::A, 2_s),
            autonOnly(intake), autonOnly(hood));
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, kFly);
        FakeController ctl;
        double shoot = -1.0, slow = -1.0, rpmAt190 = -1.0;
        drive(x, x.world, inputs, ctl,
              {{10, Button::B, true}, {15, Button::B, false}, {200, Button::A, true}, {205, Button::A, false}}, 260,
              [&](int k) {
                  if (k == 20) shoot = x.svc.speed(2).targetRpm();
                  if (k == 190) rpmAt190 = x.flyPlant.trueRpm();
                  if (k == 210) slow = x.svc.speed(2).targetRpm();
              });
        CHECK(std::abs(shoot - 3000.0) < 1e-6);
        CHECK(rpmAt190 > 2900.0);
        CHECK(std::abs(slow - 1500.0) < 1e-6);
    }
}

TEST_CASE("MECH-A4 (review): a driver control's TIME LIMIT is the one it states -- a preset's, a driver-start "
          "home's, and a flywheel toggle's at-speed window (HA-182's 5 s)") {
    // Bug caught: a preset or a homing that runs ten times its stated limit (a jammed arm pushing for
    // 15 s instead of 1.5), or a toggle window that is not kToggleAtSpeedWindowSeconds.
    {
        using namespace mech_fixture::robot_x;
        static constexpr DriverControls kQuick = controls(kRobotXMechanisms,
            preset(arm["high"], Button::Up, 0.25_s),  // 140 degrees cannot be reached in 0.25 s
            autonOnly(lift), autonOnly(flywheel), autonOnly(intake), autonOnly(hood));
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, kQuick);
        FakeController ctl;
        drive(x, x.world, inputs, ctl, {{10, Button::Up, true}, {15, Button::Up, false}}, 300, [](int) {});
        CHECK(x.svc.position(0).lastStep().outcome == MechanismOutcome::TimedOut);
        CHECK(x.svc.position(0).lastStep().elapsed >= 0.25);
        CHECK(x.svc.position(0).lastStep().elapsed < 0.27);
    }
    {
        // the lift at its bottom stop with its switch never pressed: the driver-start home runs to its bound
        RobotX x{true, /*liftOnSwitchAtBoot=*/false};
        x.world.liftSwitchFollowsPlant = false;  // a switch that never reads pressed
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        FakeController ctl;
        drive(x, x.world, inputs, ctl, {}, 300, [](int) {});
        CHECK(x.svc.position(1).lastStep().outcome == MechanismOutcome::TimedOut);
        CHECK(x.svc.position(1).lastStep().elapsed >= 1.5);
        CHECK(x.svc.position(1).lastStep().elapsed < 1.52);
        CHECK_FALSE(x.svc.position(1).homed());
    }
    {
        // a flywheel whose named speed its plant cannot reach: the toggle's step ends TIMED_OUT at the
        // window, exactly kToggleAtSpeedWindowSeconds (the wheel keeps spinning — only the verdict is bounded)
        using namespace shulib::mechanism::words;
        static constexpr Flywheel fast{.name = "fast", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12),
                                       .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
                                       .atSpeed = within(60, 0.1_s), .presets = {{"shoot", 3550}}};
        static constexpr MechanismTable kFast = mechanisms(wire(fast, motors(1)));
        static constexpr DriverControls kFastControls = controls(kFast, toggle(fast["shoot"], Button::X));
        auton_rig::TankWorld w;
        shulib::hal::fake::FakeTelemetrySink sink;
        shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
        motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
        shulib::hal::fake::FakeMotor m;
        shulib::hal::IMotor* mm[1]{&m};
        shulib::hal::MotorMechanism mech1{mm, shulib::hal::BrakeMode::Coast, "fast"};
        shulib::hal::fake::FakeMotor* mf[1]{&m};
        shulib::sim::FlywheelPlant plant{mf, auton_rig::flywheelPlantConfig()};  // 5 % slow: tops out near 3400
        w.rig = &rig;
        w.onPace = [&plant] { plant.step(0.01); };
        mech::MechanismService svc{w};
        shulib::chassis::Chassis chassis{rig.deps, svc, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}};
        const mech::Devices devs[1] = {{.motors = &mech1}};
        svc.bind(kFast, devs, rig.h.clock(), rig.latch, sink);
        mech::DriverInputs inputs;
        inputs.bind(svc, kFastControls);
        FakeController ctl;
        DriveShaper shaper{shulib::teleop::DriverProfile{}};
        int k = 0;
        bool spinningAt600 = false;
        shulib::teleop::runDriverControl(
            chassis, ctl, svc, shaper, rx::kLoop, svc, inputs,
            [&] {
                if (k == 10) ctl.setPressed(Button::X, true);
                if (k == 15) ctl.setPressed(Button::X, false);
                return k++ >= 620;
            },
            [] { return HeadingSample{}; },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                if (k - 1 == 600) spinningAt600 = svc.speed(0).spinning();
            });
        CHECK(svc.speed(0).lastStep().outcome == MechanismOutcome::TimedOut);
        CHECK(svc.speed(0).lastStep().elapsed >= mech::kToggleAtSpeedWindowSeconds);
        CHECK(svc.speed(0).lastStep().elapsed < mech::kToggleAtSpeedWindowSeconds + 0.02);
        CHECK(spinningAt600);  // the wheel kept spinning past its verdict
        static_assert(mech::kToggleAtSpeedWindowSeconds == 5.0);  // HA-182's number, pinned
    }
}

TEST_CASE("MECH-A4 (review): DriverInputs::bind refuses EACH way a driver file can name a stranger's "
          "mechanism (A4-D10's every rule)") {
    // Bug caught (landmine 6's class): a driver file built for another robot binding to a same-named
    // mechanism that is not the one it was checked against — and then commanding it. One stranger per
    // rule, each a declaration the BUILD accepts against its own robot's list.
    using namespace shulib::mechanism::words;
    const auto refused = [](const DriverControls& c) {
        RobotX x;
        mech::DriverInputs inputs;
        CHECK_THROWS_AS(inputs.bind(x.svc, c), shulib::PreconditionError);
        CHECK_FALSE(inputs.bound());
    };
    static constexpr Stall kStall{.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s};
    {
        // a LIFT named "arm": the same kind (a position mechanism), other UNITS
        static constexpr Lift liftArm{.name = "arm", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                                      .shape = spool(1.5_in), .safe = Hold, .stall = kStall, .range = {0_in, 24_in},
                                      .gravity = noGravity, .home = restsAtStop(0_in)};
        static constexpr MechanismTable t = mechanisms(wire(liftArm, motors(1)));
        static constexpr DriverControls c = controls(t, whileHeld(liftArm, Button::R1, Button::R2, 6_volt));
        CAPTURE("units");
        refused(c);
    }
    {
        // a TWO-position "hood" toggled: robot x's hood has three
        static constexpr Pneumatic hood2{.name = "hood", .positions = {{"down", {Low}}, {"up", {High}}},
                                         .start = "down", .safe = "down", .moveTime = 0.3_s};
        static constexpr MechanismTable t = mechanisms(wire(hood2, lines('C')));
        static constexpr DriverControls c = controls(t, toggle(hood2, Button::X));
        CAPTURE("toggle on three positions");
        refused(c);
    }
    {
        // a flywheel toggle at a "shoot" of 2500: robot x's "shoot" is 3000
        static constexpr Flywheel fly2{.name = "flywheel", .motors = {Forward}, .cartridge = Blue,
                                       .gearing = gears(72, 12), .safe = Coast,
                                       .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
                                       .atSpeed = within(60, 0.1_s), .presets = {{"shoot", 2500}}};
        static constexpr MechanismTable t = mechanisms(wire(fly2, motors(4)));
        static constexpr DriverControls c = controls(t, toggle(fly2["shoot"], Button::X));
        CAPTURE("toggle's named speed");
        refused(c);
    }
    {
        // a constant 170 degrees: inside a stranger arm's 0..180, outside robot x's 0..160
        static constexpr Arm wide{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60),
                                  .safe = Hold, .stall = kStall, .range = {0, 180}, .gravity = noGravity,
                                  .home = restsAtStop(0)};
        static constexpr MechanismTable t = mechanisms(wire(wide, motors(1)));
        static constexpr DriverControls c = controls(t, preset(wide.to(170), Button::Up, 1_s));
        CAPTURE("constant outside the travel");
        refused(c);
    }
    {
        // a constant 4000 rpm: under a stranger's 4200 free, over robot x's 3600
        static constexpr Flywheel faster{.name = "flywheel", .motors = {Forward}, .cartridge = Blue,
                                         .gearing = gears(84, 12), .safe = Coast,
                                         .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
                                         .atSpeed = within(60, 0.1_s)};
        static constexpr MechanismTable t = mechanisms(wire(faster, motors(4)));
        static constexpr DriverControls c = controls(t, preset(faster.at(4000), Button::B, 1_s));
        CAPTURE("constant over the free speed");
        refused(c);
    }
    {
        // a pneumatic position by NAME: the stranger's index 1 is "open", robot x's hood index 1 is "middle"
        static constexpr Pneumatic hood3{.name = "hood", .positions = {{"down", {Low, Low}}, {"open", {High, Low}},
                                                                       {"up", {High, High}}},
                                         .start = "down", .safe = "down", .moveTime = 0.3_s};
        static constexpr MechanismTable t = mechanisms(wire(hood3, lines('C', 'D')));
        static constexpr DriverControls c = controls(t, preset(hood3["open"], Button::Right));
        CAPTURE("pneumatic position by name");
        refused(c);
    }
    {
        // a homing control bound to a robot whose "arm" is NEVER homed
        static constexpr Arm nh{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60),
                                .safe = Hold, .stall = kStall, .range = {0, 160}, .gravity = noGravity,
                                .home = neverHomed, .unhomedVolts = 4_volt};
        static constexpr MechanismTable kNh = mechanisms(wire(nh, motors(1)));
        static constexpr DriverControls c =
            controls(mech_fixture::robot_x::kRobotXMechanisms, whileHeld(mech_fixture::robot_x::arm, Button::R1, Button::R2, 6_volt),
                     homeOnPress(mech_fixture::robot_x::arm, Button::Left, 1_s), autonOnly(mech_fixture::robot_x::lift),
                     autonOnly(mech_fixture::robot_x::flywheel), autonOnly(mech_fixture::robot_x::intake),
                     autonOnly(mech_fixture::robot_x::hood));
        auton_rig::TankWorld w;
        shulib::hal::fake::FakeTelemetrySink sink;
        shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
        motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
        w.rig = &rig;
        shulib::hal::fake::FakeMotor m;
        shulib::hal::IMotor* mm[1]{&m};
        shulib::hal::MotorMechanism mechArm{mm, shulib::hal::BrakeMode::Hold, "arm"};
        mech::MechanismService svc{w};
        const mech::Devices devs[1] = {{.motors = &mechArm}};
        svc.bind(kNh, devs, rig.h.clock(), rig.latch, sink);
        mech::DriverInputs inputs;
        CAPTURE("homing a never-homed mechanism");
        CHECK_THROWS_AS(inputs.bind(svc, c), shulib::PreconditionError);
        CHECK_FALSE(inputs.bound());
    }
}

TEST_CASE("MECH-A4 (review): a control on a mechanism ABSENT this boot still BINDS -- the drive and every "
          "other control still run") {
    // Bug caught: bind() refusing a driver file because one motor did not answer at boot — a throw at
    // boot would cost the whole driver file (every mechanism) for one unplugged cable.
    RobotX x{true, true, [](mech::Devices (&d)[5]) { d[2] = mech::Devices{.absence = "port 4: nothing answered"}; }};
    mech::DriverInputs inputs;
    CHECK_NOTHROW(inputs.bind(x.svc, rx::kControls));
    REQUIRE(inputs.bound());
    FakeController ctl;
    double armVolts = 0.0;
    drive(x, x.world, inputs, ctl, {{10, Button::X, true}, {15, Button::X, false}, {20, Button::R1, true}}, 40,
          [&](int k) {
              if (k == 30) armVolts = x.armM.commandedVoltage().value();
          });
    CHECK(armVolts > 0.0);  // the arm's hold drives
    CHECK(x.count("MECH", "'flywheel': ABSENT in 0.00s") == 1);  // the toggle's step, said
    CHECK(x.flyM.commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A4 (review): a SUPERSEDED hold stays superseded through a controller drop; a hold held "
          "across a disable drives at once in the next entry") {
    // Bug caught: (1) a drop read as "the button was released", so a hold its own stall superseded
    // drives again on the reconnect with the button never let go — restarting the jam N10 ends;
    // (2) enter() not resetting a hold left Driving, so the next entry reads the service's new command
    // number as "something newer spoke" and the held button is dead (T3's rule: a held hold drives at once).
    {
        RobotX x;
        x.liftPlant.setObstacleDeg(6.0 / (auton_rig::kPi * 1.5) * 360.0);  // something above the lift at 6 in
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        FakeController ctl;
        bool stalledBy199 = false;
        double voltsAfterReconnect = -1.0;
        drive(x, x.world, inputs, ctl, {{10, Button::L1, true}}, 260, [&](int k) {
            if (k == 199) {
                stalledBy199 = x.count("MECH", "'lift': STALLED") == 1;
                ctl.setConnected(false);  // L1 stays DOWN on the controller
            }
            if (k == 209) ctl.setConnected(true);
            if (k == 240) voltsAfterReconnect = x.liftM1.commandedVoltage().value();
        });
        REQUIRE(stalledBy199);
        CHECK(voltsAfterReconnect == 0.0);                 // still superseded: L1 was never seen released
        CHECK(x.count("MECH", "'lift': STALLED") == 1);    // no second push into the jam
    }
    {
        RobotX x;
        mech::DriverInputs inputs;
        inputs.bind(x.svc, rx::kControls);
        FakeController ctl;
        ctl.setPressed(Button::R1, true);  // held through BOTH entries and the disable between them
        double volts[2]{};
        for (int entry = 0; entry < 2; ++entry) {
            drive(x, x.world, inputs, ctl, {}, 20, [&](int k) {
                if (k == 5) volts[entry] = x.armM.commandedVoltage().value();
            });
        }
        CHECK(volts[0] > 0.0);
        CHECK(volts[1] > 0.0);  // the second entry: R1 still held, it drives at once
    }
}

// ═══ ROB1-C2: the ONE-button hold — `whileHeld(power, button, volts)` ════════════════════════════

namespace {
// Robot x's intake on ONE button (A runs it, one way); B — the button a two-button hold would have taken
// for "back" — is FREE, and here the hood's "down" preset takes it.
inline constexpr mech::DriverControls kOneButton = mech::controls(rx::kRobotXMechanisms,
    mech::whileHeld(rx::arm, Button::R1, Button::R2, 6_volt),
    mech::whileHeld(rx::lift, Button::L1, Button::L2, 12_volt),
    mech::homeOnPress(rx::lift, Button::Left, 1.5_s),
    mech::toggle(rx::flywheel["shoot"], Button::X),
    mech::whileHeld(rx::intake, Button::A, 12_volt),  // ONE button: held = +12 V, let go = stop
    mech::preset(rx::hood["down"], Button::B),        // B: free beside it
    mech::preset(rx::hood["middle"], Button::Right),
    mech::preset(rx::hood["up"], Button::Up));
// The one-button control takes its one button and NOT a second: B is the hood's here, so the list compiles at
// all only because the intake left B free (a two-button hold on A/B beside this preset is refused by name —
// test/compile_refusals/d_one_button_hold_button_twice is the other half: its one button IS taken).
static_assert(!kOneButton.avoids(Button::A));
static_assert(kOneButton.avoids(Button::Down) && kOneButton.avoids(Button::Y) && !kOneButton.avoids(Button::R1));
static_assert(kOneButton.c[4].kind == mech::ControlKind::Hold && kOneButton.c[4].oneWay &&
              kOneButton.c[4].button == Button::A && kOneButton.c[4].volts == 12.0 &&
              kOneButton.c[4].mechanismKind == mech::Kind::Power);
static_assert(!mech::detail::takes(kOneButton.c[4], Button::B));  // no back button
static_assert(mech::detail::takes(kOneButton.c[4], Button::A));
// ...and a two-button hold is NOT one-way (the field defaults false; robot two's file is two-button holds).
static_assert(!rx::kControls.c[0].oneWay && !mech_fixture::robot_two::kControls.c[0].oneWay);
}  // namespace

TEST_CASE("ROB1-C2: a ONE-button hold runs its power mechanism ONE way while held and stops on release; the button beside it does nothing to it; a drop releases it") {
    // Bug caught: a one-button hold that reads a "back" button anyway (it would run backward on the button
    // another control owns — here B, the hood's preset); one that keeps running after its button is let go;
    // one whose button held through a controller drop keeps running; one that never runs (the hold's tick
    // treating `back == button` as BOTH held = neither).
    RobotX x;
    mech::DriverInputs inputs;
    inputs.bind(x.svc, kOneButton);
    FakeController ctl;
    double held = -99, released = -99, onB = -99, heldAgain = -99, dropped = -99, reconnected = -99;
    double mostNegative = 0.0;
    bool hoodMoved = false;
    DriveShaper shaper{shulib::teleop::DriverProfile{}};
    int k = 0;
    shulib::teleop::runDriverControl(
        x.chassis, ctl, x.world, shaper, rx::kLoop, x.svc, inputs,
        [&] {
            if (k == 10) ctl.setPressed(Button::A, true);
            if (k == 20) ctl.setPressed(Button::A, false);
            if (k == 30) ctl.setPressed(Button::B, true);  // the hood's preset — never the intake's back
            if (k == 35) ctl.setPressed(Button::B, false);
            if (k == 40) ctl.setPressed(Button::A, true);
            if (k == 50) ctl.setConnected(false);  // A still "held" on the controller that dropped
            if (k == 60) ctl.setConnected(true);
            if (k == 70) ctl.setPressed(Button::A, false);
            return k++ >= 80;
        },
        [] { return HeadingSample{}; },
        [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
            const double v = x.intakeM.commandedVoltage().value();
            mostNegative = std::min(mostNegative, v);
            if (k - 1 == 15) held = v;
            if (k - 1 == 25) released = v;
            if (k - 1 == 33) {
                onB = v;
                hoodMoved = x.lineC.commanded() || x.lineD.commanded();
            }
            if (k - 1 == 45) heldAgain = v;
            if (k - 1 == 55) dropped = v;
            if (k - 1 == 65) reconnected = v;
        });
    CHECK(held == 12.0);       // A held: the full +12 V, one way
    CHECK(released == 0.0);    // let go: stopped (its declared safe state)
    CHECK(onB == 0.0);         // B is not its back button: the intake does not move...
    CHECK_FALSE(hoodMoved);    // ...and the hood's "down" is Low on both lines (it went where it was)
    CHECK(heldAgain == 12.0);
    CHECK(dropped == 0.0);     // a disconnect is a release
    CHECK(reconnected == 12.0);  // still held after the reconnect: a hold is a level — it drives again
    CHECK(mostNegative == 0.0);  // NEVER backward: a one-button hold has no back
}

TEST_CASE("ROB1-C2 (review): DriverInputs::bind refuses a ONE-way hold on a POSITION mechanism -- a hand-built control the build never checked") {
    // Bug caught: bind() accepting `oneWay` on an arm (a Control is an aggregate, so a hand-built one gets past
    // controls()' named refusal): the arm would jog ONE way only, its "down" button silently its "up" one. The
    // review of ROB1-C2 found this precondition unpinned (RV5, `!c.oneWay &&` removed, survived the suite).
    mech::DriverControls oneWayArm = rx::kControls;  // robot x's own file: c[0] is the arm's two-button hold
    REQUIRE(oneWayArm.c[0].kind == mech::ControlKind::Hold);
    REQUIRE(oneWayArm.c[0].mechanismKind == mech::Kind::Position);
    {
        RobotX x;  // as written it binds (the refusal below is the one-way flag's, nothing else)
        mech::DriverInputs inputs;
        inputs.bind(x.svc, oneWayArm);
        CHECK(inputs.bound());
    }
    oneWayArm.c[0].oneWay = true;
    oneWayArm.c[0].back = oneWayArm.c[0].button;
    RobotX x;
    mech::DriverInputs inputs;
    CHECK_THROWS_AS(inputs.bind(x.svc, oneWayArm), shulib::PreconditionError);
    CHECK_FALSE(inputs.bound());
}
