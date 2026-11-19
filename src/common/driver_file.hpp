#pragma once
//
// driver_file.hpp — HOW A DRIVER FILE RUNS, written once (the mechanism core, chunk MECH-A5,
// 2026-10-29; the MECH brief's N6, the team lead's go of 2026-10-08). A team's driver file
// (src/team/robot_two_driver.hpp is the first) holds ONLY the team's decisions — which button does
// what (`kControls`), the feel (`kFeel`) and the loop's numbers (`kLoop`) — and names its driver in
// one line:
//
//     using RobotTwoDriver = DriverFile<robot_two::kControls, robot_two::kFeel, robot_two::kLoop>;
//
// Everything that is not a decision — binding the controls to the mechanism service, the controller,
// the feel's state across a disable, the call into the library's driver loop, the heading rule — is
// HERE, so a second robot's driver file copies none of it. (Until MECH-A5 robot two's driver file
// carried ~65 such lines itself — a class, bind(), run(), its bindings and six static_asserts — over
// src/common/driver_controls.hpp, which this file and mechanism::DriverControls replaced.)
//
// ── Where the six checks a driver file used to state went (the MECH brief's U6) ──────
//   * a button doing two jobs, a hold's volts outside (0, 12], a control on a mechanism the robot
//     does not have, a mechanism with no control — mechanism::controls(...)'s, AT BUILD, by name;
//   * a control on the loop's RE-ZERO button — THIS file's first static_assert (the harness's n57);
//   * a feel out of range (a curve outside [0, 1], a dead zone outside [0, 1)) — THIS file's third
//     static_assert (chunk RMP1-B, the RMP1 brief's D10; the harness's rm01);
//   * the loop's TICK — the brain's (hal::pros::ProsTickPacer::kTickMs), SUPPLIED here: a team's file
//     states no `.tickPeriod` (it is not a decision anyone may make), and one that states one anyway
//     must state the brain's — THIS file's second static_assert (the harness's n56);
//   * the claw row's widths — src/common/driver_panel.hpp's static_asserts, against the robot's list.
//
// ── Storage ─────────────────────────────────────────────────────────────────────────
// ONE DriverFile lives in the program's static storage for the whole run, beside the program's
// mechanism service: PROS deletes the driver-control task when the field disables the robot and runs
// no destructor, and the loop holds the inputs by address. bind() ONCE, at boot, inside the boot's try,
// AFTER the service is bound (mechanism::DriverInputs' rule). Its default constructor touches no
// hardware: the controller adapter reads nothing until it is read (hal/pros/controller.hpp).
//
// ── What the mechanisms do, and what decides it ──────────────────────────────────────
// The driver loop's service overload (teleop/teleop_loop.hpp) runs the controls through the ONE
// mechanism service that also runs them in autons (mechanism/driver_inputs.hpp states every rule:
// hold-to-run, a toggle's SEEN press, the most recent input wins). A disable stops every MOTOR
// mechanism (its safe state) and leaves a pneumatic at its level; the program's FIRST driver entry
// sends a pneumatic its START position — a piston goes back to its start whatever an auton left — and a
// later entry sends nothing (T4's ruling, kept). A leftover claim released at THIS entry is what a
// program's claw row shows as ` FREED` (releasedLeftoverAtEntry()).
//
// PROS-bound only through hal::pros::ProsController (the shim answers it on the host): the host suite
// runs robot two's real driver file through it (test/robot_two_driver_test.cpp). NOTHING HERE HAS RUN
// ON A ROBOT.

#include <cstddef>
#include <cstdio>
#include <string_view>
#include <utility>

#include "common/driver_heading.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/mechanism/driver_inputs.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// The brain's tick, as the driver loop's period (hal::pros::ProsTickPacer::kTickMs — 10 ms, HA-32).
inline constexpr units::Time kBrainTick{hal::pros::ProsTickPacer::kTickMs / 1000.0};

/// A robot's driver, from its driver file's three constants (file header). `Controls` is the file's
/// `controls(<the robot's list>, ...)`, `Feel` its DriverProfile, `Loop` its DriverLoopConfig (the
/// re-zero button and the release brake mode; no `.tickPeriod`).
template <const mechanism::DriverControls& Controls, const teleop::DriverProfile& Feel,
          const teleop::DriverLoopConfig& Loop>
class DriverFile {
public:
    /// The driver file's controls, as written.
    static constexpr const mechanism::DriverControls& kControls = Controls;
    /// The driver file's feel, as written.
    static constexpr const teleop::DriverProfile& kFeel = Feel;

    // THE THREE CHECKS this file makes (header): `make` refuses a driver file that could not work.
    static_assert(Controls.avoids(Loop.rezeroButton),
                  "a mechanism's button is the loop's re-zero button -- give the re-zero (kLoop's "
                  ".rezeroButton) a button no control takes");
    static_assert(Loop.tickPeriod.value() == 0.0 || Loop.tickPeriod == kBrainTick,
                  "leave .tickPeriod out of the driver file: the loop runs at the brain's tick");
    // RMP1-B (the brief's D10): out of range, a curve overshoots full speed or a dead zone inverts the stick.
    static_assert(Feel.valid(), "the feel is out of range: each .deadband must be at least 0 and below 1, and "
                                "each .expo (the curve) from 0 to 1");

    /// The loop's numbers as the library takes them: the driver file's, with the brain's tick.
    static constexpr teleop::DriverLoopConfig kLoop = [] {
        teleop::DriverLoopConfig c = Loop;
        c.tickPeriod = kBrainTick;
        return c;
    }();

    /// initialize(), ONCE, after the Robot is built and the program's mechanism service is bound over
    /// it, inside the boot's try. Checks the controls against the robot the SERVICE was bound to (the
    /// build checked them against the driver file's own robot's list; a program boots kMechanisms —
    /// the same list in its robot's builds, by the table file's static_asserts — so a refusal here is
    /// a composition gone wrong). True when bound. FALSE, the reason in `why` (bounded to `cap`,
    /// NUL-terminated) NAMING THE MECHANISM, and NOTHING bound, when the robot is not the one these
    /// controls are for — the program refuses the boot with its driver-file headline (T4's binder's
    /// contract and its sentences, kept since MECH-A5's review):
    ///   * a control names a mechanism the robot does not declare — "a control names '<name>', which
    ///     the robot does not declare";
    ///   * a control's mechanism is another kind on this robot — "'<name>' is a <kind> mechanism on
    ///     this robot: its control is for a <kind> one";
    ///   * a mechanism the robot declares that no control names (autonOnly(...) counts) — "the robot
    ///     declares '<name>', and no control names it";
    ///   * the rest of mechanism::DriverInputs::bind's checks (a preset, a position, a speed, units or
    ///     a homing that is not this robot's mechanism's) — the library's own sentence.
    /// A mechanism ABSENT this boot still binds: its buttons command a slot that does nothing, and the
    /// drive runs. A PROGRAM's mistake — a second bind, or a service not bound yet — is a
    /// PreconditionError (the boot's catch: "the library graph REFUSED to build").
    [[nodiscard]] bool bind(mechanism::MechanismService& mechanisms, char* why, std::size_t cap) {
        SHULIB_PRECONDITION(!inputs_.bound(), "DriverFile::bind: called twice -- bind once, at boot");
        SHULIB_PRECONDITION(mechanisms.bound(), "DriverFile::bind: the mechanism service is not bound -- bind it first");
        const bool say = why != nullptr && cap > 0;
        if (say) {
            why[0] = '\0';
        }
        // 1. Every control resolves BY NAME to a mechanism of its kind (T4's step 1, its words).
        for (const mechanism::Control& c : Controls.all()) {
            if (c.kind == mechanism::ControlKind::AutonOnly) {
                continue;
            }
            const mechanism::Slot* s = mechanisms.slotFor(c.mechanismName);
            if (s == nullptr) {
                if (say) {
                    std::snprintf(why, cap, "a control names '%.23s', which the robot does not declare",
                                  c.mechanismName);
                }
                return false;
            }
            if (s->kind() != c.mechanismKind) {
                if (say) {
                    std::snprintf(why, cap, "'%.23s' is a %s mechanism on this robot: its control is for a %s one",
                                  c.mechanismName, mechanism::kindWord(s->kind()),
                                  mechanism::kindWord(c.mechanismKind));
                }
                return false;
            }
        }
        // 2. Every mechanism the robot declares is named by a control, autonOnly(...) included (T4's
        //    step 2): one no control names is a robot these controls were not written for.
        for (std::size_t i = 0; i < mechanisms.size(); ++i) {
            const std::string_view name{mechanisms.slot(i).name()};
            bool named = false;
            for (const mechanism::Control& c : Controls.all()) {
                named = named || (c.mechanismName != nullptr && name == c.mechanismName);
            }
            if (!named) {
                if (say) {
                    std::snprintf(why, cap, "the robot declares '%.23s', and no control names it",
                                  mechanisms.slot(i).name());
                }
                return false;
            }
        }
        // 3. The library's own checks (a preset, a position, a speed, units, a homing). Nothing is
        //    stored before they pass, so a refusal leaves nothing bound.
        try {
            inputs_.bind(mechanisms, Controls);
        } catch (const PreconditionError& e) {
            if (say) {
                std::snprintf(why, cap, "%.150s", e.what());
            }
            return false;
        }
        return true;
    }

    /// opcontrol(): the library's driver loop until `disabled()` says stop, then return (PROS calls
    /// opcontrol() again when driver control comes back). `mechanisms` is the service bind() bound —
    /// the loop ticks it once per tick itself; `pacer` is the brain's tick (the raw one: nothing in
    /// driver control is a run guard's to end); `perTick` is the program's screen (one that takes a
    /// fourth argument is also handed the drive protections' state).
    template <class IsDisabled, class PerTick>
    void run(robot::Robot& robot, mechanism::MechanismService& mechanisms, motion::ITickPacer& pacer,
             IsDisabled&& disabled, PerTick&& perTick) {
        releasedBefore_ = mechanisms.leftoverClaimsReleased();  // THIS entry's releases are the ones after
        teleop::runDriverControl(robot.chassis(), controller_, pacer, feel_, kLoop, mechanisms, inputs_,
                                 std::forward<IsDisabled>(disabled), [&robot] { return driverHeading(robot); },
                                 std::forward<PerTick>(perTick));
    }

    /// True when the CURRENT driver-control entry released a claim another owner had left on one of
    /// `mechanisms`' mechanisms (the service logged which, `[WARN][MECH]`) — what a program's claw row
    /// shows as ` FREED`, as T3's bindings' `anyReleasedLeftoverClaim()` did until MECH-A5. False before
    /// the first run().
    [[nodiscard]] bool releasedLeftoverAtEntry(const mechanism::MechanismService& mechanisms) const noexcept {
        return everRan() && mechanisms.leftoverClaimsReleased() > releasedBefore_;
    }

    /// True once bind() has run.
    [[nodiscard]] bool bound() const noexcept { return inputs_.bound(); }

private:
    /// True once run() has begun (its entry's count is recorded).
    [[nodiscard]] bool everRan() const noexcept { return releasedBefore_ >= 0; }

    hal::pros::ProsController controller_{hal::pros::ControllerId::Master};  // the driver's
    teleop::DriveShaper feel_{Feel};  // kept for the run: field-centric's zero survives a disable
    mechanism::DriverInputs inputs_{};
    int releasedBefore_ = -1;  // the service's leftover-claim count when the current entry began
};

}  // namespace shulib::bench
