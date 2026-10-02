#pragma once
//
// robot_two_driver.hpp — ROBOT TWO'S DRIVER CONTROL: what every stick and button does.
//
// This is the team's driver code (chunk T4). The competition program's opcontrol() runs it
// ("shulib Competition", src/programs/competition_program.cpp, chunk COMP-b, 2026-09-30 -- not yet
// on a robot: worksheet Station K), and so will the driver program that replaces "shulib Drive"
// (T4). To change what a button does, change it HERE -- and, until T4 moves the buttons
// out of src/robots/mechanism_table.hpp, there too (the re-zero also in "shulib Teleop"): `make`
// refuses a build in which they disagree. What the robot IS -- its ports, its cartridges, that a
// claw pivot is put in Hold when let go, that the claw starts CLOSED and which line level closes
// it -- is in src/robots/mechanism_table.hpp, and is not the driver's to choose.
//
// It runs on the library's front door. The program builds a robot::Robot from the calibration
// record on the SD card and hands it to bind(), once, in initialize(); opcontrol() calls run(),
// which is the library's driver loop. Nothing here draws on a screen or writes a log line: the
// program around it does both.
//
//   left stick   up / down: drive forward / back. Forward is the end the calibration record
//                calls the front (the camera end, on robot two's record of 2026-09-25).
//                Left / right does nothing: a tank cannot strafe.
//   right stick  left / right: turn
//   R1 / R2      the claw pivot on port 6: forward / back at full power while held. Let go and
//                it is put in Hold (whether Hold keeps a loaded claw up is unmeasured, HA-156)
//   L1 / L2      the claw pivot on port 7: the same
//   A            the claw: each press opens or closes it. The FIRST time driver control starts
//                in a run, the claw is told CLOSED, whatever autonomous left it at (the team
//                lead's ruling, 2026-09-26); a program with no autonomous has left it CLOSED,
//                so there it does not move. After that, a disable and the next start tell it
//                nothing. Which line level closes it is a guess (HA-159)
//   Y            field-centric re-zero. Field-centric is OFF, so Y changes nothing yet
//   let go of the sticks and the drive coasts (Coast is set on every drive motor)
//
// NOTHING HERE HAS RUN ON A ROBOT. It is tested on the host, through the PROS shim, against
// robot two's real mechanism table (test/robot_two_driver_test.cpp).

#include <cstddef>

#include "common/driver_controls.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/literals.hpp"

namespace shulib::bench {

using namespace units::literals;  // 12_volt, 10_ms

/// Robot two's driver: the buttons, the feel, and the loop.
///
/// ONE lives in the program's static storage for the whole run -- never on opcontrol()'s stack:
/// PROS deletes the driver-control task when the field disables the robot, and the loop holds
/// these bindings by address.
class RobotTwoDriver {
public:
    using Button = hal::ControllerButton;

    // ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════
    // One line per mechanism; the names are src/robots/mechanism_table.hpp's.
    static constexpr HoldControl kHolds[] = {
        {"claw pivot 6", Button::R1, Button::R2, 12_volt},  // forward / back, full power
        {"claw pivot 7", Button::L1, Button::L2, 12_volt},  // forward / back, full power
    };
    static constexpr ToggleControl kToggles[] = {
        {"claw", Button::A},  // each press opens or closes it
    };

    // ═══ THE FEEL ═════════════════════════════════════════════════════════════════════════
    // The library's default: arcade (left stick drives, right stick turns), a smooth deadband
    // of 0.05, no curve, no ramp -- numbers no driver has felt yet (HA-112). Field-centric is
    // OFF: ArcadeBody, not ArcadeField.
    static constexpr teleop::DriverProfile kFeel{.mode = teleop::DriveMode::ArcadeBody};

    static constexpr teleop::DriverLoopConfig kLoop{
        .tickPeriod = 10_ms,                        // the brain's tick
        .rezeroButton = Button::Y,                  // field-centric's "away from me"
        .releaseBrakeMode = hal::BrakeMode::Coast,  // let go of the sticks: the drive coasts
    };

    // ═══ HOW IT RUNS -- nothing below changes when a button does ══════════════════════════

    /// initialize(), once, right after the robot is built. False, with the reason in `why` and
    /// nothing bound, when the robot is not the one these controls are for. A mechanism it
    /// declares but could not build (a motor unplugged) gets no binding: its buttons do
    /// nothing, and the drive still runs. Call it inside the boot's try: a binding can throw.
    bool bind(robot::Robot& robot, char* why, std::size_t cap) {
        return bindings_.bind(robot, kHolds, kToggles, kLoop.rezeroButton, why, cap);
    }

    /// opcontrol(): drive until the field disables the robot, then return (PROS calls
    /// opcontrol() again when driver control comes back). `pacer` is the brain's tick,
    /// `disabled` asks the field, and `perTick` is the program's screen (it may do nothing).
    template <class IsDisabled, class PerTick>
    void run(robot::Robot& robot, motion::ITickPacer& pacer, IsDisabled&& disabled,
             PerTick&& perTick) {
        teleop::runDriverControl(robot.chassis(), controller_, pacer, feel_, kLoop,
                                 bindings_.holds(), bindings_.toggles(), disabled,
                                 [&robot] { return driverHeading(robot); }, perTick);
    }

    /// What bind() bound, for a program's screen and boot lines.
    const DriverBindings& bindings() const noexcept { return bindings_; }

    /// Forget every binding. A program whose boot caught a throw calls this before it
    /// destroys the robot, so nothing is left pointing at it.
    void unbind() noexcept { bindings_.unbind(); }

private:
    hal::pros::ProsController controller_{hal::pros::ControllerId::Master};  // the driver's
    teleop::DriveShaper feel_{kFeel};  // kept for the run: field-centric's zero survives a disable
    DriverBindings bindings_;
};

// ═══ THE CHECKS: `make` refuses a set of controls that could not work ═════════════════════
static_assert(controlButtonsDistinct(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles),
              "a button does two jobs (or a pivot's forward is its back)");
static_assert(controlsAvoid(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                            RobotTwoDriver::kLoop.rezeroButton),
              "a mechanism's button is the re-zero button");
static_assert(holdVoltsValid(RobotTwoDriver::kHolds), "a pivot's volts are not in (0, 12]");
static_assert(controlsNameTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                                kRobotTwoMechanisms),
              "a control names a mechanism robot two does not have, of the wrong kind, or twice");
static_assert(controlsCoverTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                                 kRobotTwoMechanisms),
              "one of robot two's mechanisms has no control");
static_assert(RobotTwoDriver::kLoop.tickPeriod
                  == units::Time{hal::pros::ProsTickPacer::kTickMs / 1000.0},
              "one tick is not the brain's");

}  // namespace shulib::bench
