// main.cpp — THE DISPATCHER: the five PROS entry points, handing over to the program
// src/build_selection.hpp chose for this build (G1 Part 2c, 2026-09-26; the choice itself moved to
// that header at 2c's final run the same day).
//
// ═══ READ THIS FIRST ════════════════════════════════════════════════════════════════
// There is no robot in this file, and no build switch. Every program lives in its own file under
// src/programs/ (what a robot IS lives under src/robots/, what the programs share under
// src/common/, the team's own code under src/team/ — SRC1's layout, 2026-09-27), and which one
// this binary IS — and which ROBOT's tables it carries — was decided when it was built:
// `make ROBOT=... PROGRAM=...`, both validated by the Makefile, and read by ONE #if chain in
// src/build_selection.hpp, whose every branch sets the program (`kProgram`), the robot (`kRobotId`)
// and the identity beacon (`kVariantBeacon`) together (G1 Part 2c's final run, 2026-09-26: the team
// lead's ruling "one chain picks both" moved the chain out of this file, where it chose the program
// alone). This file holds exactly:
//   * the on-robot precondition handler, installed FIRST in initialize(), and attachFaultLatch(),
//     through which a program built in another file hands the handler its fault latch;
//   * one serial line naming the robot the build carries (the ROBOT BEACON, derived from kRobotId);
//   * initialize() / disabled() / competition_initialize() / autonomous() / opcontrol(), each a
//     `switch (kProgram)` with a case for EVERY program and no default (a new program is a
//     -Wswitch failure in all five), forwarding to that program's shulib::bench:: functions.
//
//   build                              program              where it lives
//   make                               "Bench Tests"        src/programs/bench_r3a.cpp (the bench bot)
//   make ROBOT=tank                    "Bench Tests"        src/programs/bench_r3a.cpp over robot
//                                                           two's signed table
//                                                           (src/robots/chassis_table.hpp)
//   make ROBOT=tank PROGRAM=drive      "shulib Drive"       src/programs/drive_program.cpp      (slot 1)
//   make ROBOT=tank PROGRAM=library    "shulib Teleop"      src/programs/teleop_program.cpp     (slot 2)
//                                                           + src/common/boot_source.{hpp,cpp}
//   make ROBOT=tank PROGRAM=calibrate  "shulib Calibrate"   src/programs/calibrate_program.cpp  (slot 4)
//   make ROBOT=xdrive                  the INVENTED X-drive src/programs/xdrive_program.cpp
//                                                           + src/robots/xdrive_config.hpp (HA-111)
// Four programs in four slots rather than one program with a chooser (the team lead's ruling,
// R3b Part 0b): the operator picks a program by NAME from the brain's slot list. Every .cpp under
// src/ compiles into every build; the linker's --gc-sections drops what the selected program never
// calls. The ROBOT is set by the same branch (kRobotId): the bench build carries the bench bot's
// tables, the four tank builds robot two's, the xdrive build the invented X-drive's -- an UNSET
// chassis table and no mechanisms (src/robots/chassis_table.hpp and src/robots/mechanism_table.hpp
// select them from kRobotId).
//
// ═══ WHAT RUNS, AND WHAT HAS RUN (no claim beyond the logs) ════════════════════════════
//   * NO LIBRARY-COMMANDED MOTION HAS RUN ON A ROBOT (R3b open). "shulib Drive" drove robot two
//     open-loop on 2026-09-17 (the team lead's report; no log was captured); the PREVIOUS
//     "shulib Teleop" (build v0.1.1-319-g2780462) drove it
//     from the calibration record on 2026-09-25 — eight starts, open-loop under driver control
//     through the library's own graph. THIS VERSION of "shulib Teleop" (G1 Part 2b) has not run
//     on hardware. No X-drive robot exists (ports invented, A4 register HA-111): the one X-drive
//     boot on record is the old hand-wired graph over FAKE devices, 2026-08-12, none since R1a.
//   * autonomous() commands NO motion in any build of this file, and no routine is ever added
//     here: autonomous routines run only in the competition program's autonomous() (the team
//     lead's ruling, 2026-09-25).
//   * M2's on-robot clause — validated on a V5 — is OPEN, owned by chunk R3.
//
// ═══ The PROS boundary ══════════════════════════════════════════════════════════════
// This TU (and main.h) may include <pros/...>; include/shulib/ may NOT — except
// include/shulib/hal/pros/ (the adapter tree, R1a), which is the ONE path the CI guard exempts, by
// exact path anchor. Everything else in the library remains PROS-free, and CI enforces it.

#include "main.h"

#include <cstddef>
#include <cstdio>

#include "programs/bench_r3a.hpp"
#include "build_selection.hpp"
#include "programs/calibrate_program.hpp"
#include "programs/drive_program.hpp"
#include "common/fault_latch_hook.hpp"
#include "common/state_screens.hpp"
#include "programs/teleop_program.hpp"
#include "programs/xdrive_program.hpp"

#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"

// ═══ WHICH PROGRAM, WHICH ROBOT? — src/build_selection.hpp ══════════════════════════════
// The build switches are read there and nowhere else (tools/src_build_gate.py's step 0 fails a
// switch named in this file): its ONE chain sets kProgram, kRobotId and kVariantBeacon in the same
// branch. Below, this file only switches on kProgram — a misspelt program is a compile error, and a
// program added to the enum but not to a switch is a -Wswitch warning the gate fails on. The beacon
// vouches for the BRANCH; the ROBOT BEACON (printed in initialize()) for the robot the branch set;
// the gate's step 8 reads each build's symbol table for the program the branch set and for what
// each case below calls (PROGRAM_ENTRY_SYMBOLS).
using shulib::bench::kProgram;
using shulib::bench::kRobotBeacon;
using shulib::bench::kVariantBeacon;
using shulib::bench::Program;

namespace {

/// The on-robot precondition policy (check.hpp §18.4): raise the fault code on the latch (visible
/// in telemetry and the run summary), then throw the same PreconditionError the host policy
/// throws — the motion scheduler catches it at the task boundary and converts it to FAULT_ABORT
/// plus a safe drivetrain state. Until a program has built its graph there is no latch, and the
/// handler only throws — still loud, still never-returns. Every program that builds a graph
/// ("shulib Teleop", the X-drive) catches a throw at its graph boundary and refuses on screen.
shulib::diag::FaultLatch* g_faults = nullptr;
[[noreturn]] void robotPreconditionHandler(const char* message) {
    if (g_faults != nullptr) {
        g_faults->raise(shulib::diag::FaultCode::Precondition, "CHK", message);
    }
    throw shulib::PreconditionError(message);
}

/// The competition status bits in words, for the boot banners (state_screens.hpp).
const char* bootStatusBits(char* buf, std::size_t n) {
    return shulib::bench::competitionBits(pros::c::competition_get_status(), buf, n);
}

}  // namespace

/// fault_latch_hook.hpp: point the precondition handler above at a latch that lives in another
/// translation unit — the program that built the graph calls it once the latch exists. Defined for
/// EVERY build (every .cpp under src/ compiles into every build); --gc-sections drops it where nothing
/// calls it.
void shulib::bench::attachFaultLatch(shulib::diag::FaultLatch* latch) noexcept {
    g_faults = latch;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * Order is load-bearing (check.hpp's concurrency contract): the precondition policy is installed
 * FIRST — before any program constructs an adapter (their constructors carry read-back
 * preconditions) and before any other task exists. Then one serial line naming the robot whose
 * tables this build carries, then the selected program's boot (its banner carries the variant
 * beacon).
 */
void initialize() {
    shulib::setPreconditionHandler(&robotPreconditionHandler);
    // The ROBOT this build carries the tables of (src/build_selection.hpp's kRobotBeacon, derived
    // from kRobotId): printed so it is linked, and so the gate can assert it (step 6) — a build whose
    // branch set another robot carries another robot's beacon, and FAILS.
    std::printf("[BLD] robot tables: %s (src/robots/chassis_table.hpp, src/robots/mechanism_table.hpp)\n",
                kRobotBeacon);
    char bits[64];
    switch (kProgram) {
        case Program::BenchTests:
            shulib::bench::testerInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::Drive:
            shulib::bench::driveInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::Teleop:
            shulib::bench::teleopInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::Calibrate:
            shulib::bench::calibrateInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::InventedXDrive:
            shulib::bench::xdriveInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
    }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or the VEX Competition
 * Switch, following either autonomous or opcontrol. Every program that drives only in driver
 * control says so on the brain's screen; the invented X-drive paints nothing, as it never did.
 * A case per program, never a default: a program added to the enum must say what it does here.
 */
void disabled() {
    switch (kProgram) {
        case Program::BenchTests:
        case Program::Drive:
        case Program::Teleop:
        case Program::Calibrate:
            shulib::bench::benchStateScreen(
                "DISABLED", "VEXos reports the robot DISABLED, so the driver task is not running.");
            return;
        case Program::InventedXDrive:
            return;
    }
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field Management System or
 * the VEX Competition Switch. A case per program, as in disabled().
 */
void competition_initialize() {
    switch (kProgram) {
        case Program::BenchTests:
        case Program::Drive:
        case Program::Teleop:
        case Program::Calibrate:
            shulib::bench::benchStateScreen(
                "FIELD CONTROL CONNECTED",
                "A competition switch or field is connected through the controller.");
            return;
        case Program::InventedXDrive:
            return;
    }
}

/**
 * Runs the user autonomous code.
 *
 * Commands NO motion in ANY build of this file, deliberately, and no routine is ever added here:
 * autonomous routines run only in the competition program's autonomous() (COMP), never inside a
 * bench, drive, calibrate or teleop program (the team lead's ruling, 2026-09-25). The v2 motions
 * exist and settle on the host test plant (test/chassis_routine_test.cpp chains them through the
 * same facade); no library-commanded motion has run on a robot.
 */
void autonomous() {
    switch (kProgram) {
        case Program::BenchTests:
        case Program::Drive:
            shulib::bench::benchStateScreen(
                "AUTONOMOUS",
                "No motion here, ever: the tester's DRIVE station lives under driver control.");
            return;
        case Program::Calibrate:
            shulib::bench::benchStateScreen(
                "AUTONOMOUS",
                "No motion here, ever: calibration powers the motors and lives under driver "
                "control, where a person is standing by the battery.");
            return;
        case Program::Teleop:
            // The program's own sentence, at most 50 characters (checked in teleop_program.cpp),
            // on the screen here and logged once by the program.
            shulib::bench::benchStateScreen("AUTONOMOUS", shulib::bench::kTeleopAutonomousReason);
            shulib::bench::teleopAutonomous();
            return;
        case Program::InventedXDrive:
            shulib::bench::xdriveAutonomous();  // one serial line; no screen, no motion
            return;
    }
}

/**
 * Runs the operator control code: the selected program's driver-control entry. Each returns on a
 * field DISABLE (its loop polls for it, so the drive is put in its safe state before PROS tears
 * the task down; PROS calls opcontrol() again when driver control resumes), except "Bench Tests",
 * which never returns and powers motors ONLY in its DRIVE station. "shulib Teleop" and the
 * X-drive drive through the library's loop, teleop::runDriverControl; "shulib Drive" and the
 * tester keep the pinned raw stick mapping (the G1 Part 2 brief's ruling M4).
 */
void opcontrol() {
    switch (kProgram) {
        case Program::BenchTests:
            shulib::bench::runR3a();
            return;
        case Program::Drive:
            shulib::bench::runDrive();
            return;
        case Program::Teleop:
            shulib::bench::runTeleop();
            return;
        case Program::Calibrate:
            shulib::bench::runCalibrate();
            return;
        case Program::InventedXDrive:
            shulib::bench::runXdrive();
            return;
    }
}
