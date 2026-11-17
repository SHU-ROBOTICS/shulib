// main.cpp -- hands PROS's five entry points to the program this build is (src/build_selection.hpp).
// No robot, port or routine lives here: your code goes in src/team/. What this file does and what to
// put in it: docs/where-your-code-goes.md.

#include "main.h"

#include <cstddef>

#include "programs/auton_tester_program.hpp"
#include "programs/bench_r3a.hpp"
#include "build_selection.hpp"
#include "programs/calibrate_program.hpp"
#include "programs/competition_program.hpp"
#include "programs/drive_program.hpp"
#include "programs/tune_program.hpp"
#include "common/fault_latch_hook.hpp"
#include "common/state_screens.hpp"
#include "programs/xdrive_program.hpp"

#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/pros/char_sink.hpp"

using shulib::bench::kProgram;
using shulib::bench::kRobotBeacon;
using shulib::bench::kVariantBeacon;
using shulib::bench::Program;

namespace {

// A broken library promise: raise it on the fault latch (once a program has one), then throw.
shulib::diag::FaultLatch* g_faults = nullptr;
[[noreturn]] void robotPreconditionHandler(const char* message) {
    if (g_faults != nullptr) {
        g_faults->raise(shulib::diag::FaultCode::Precondition, "CHK", message);
    }
    throw shulib::PreconditionError(message);
}

const char* bootStatusBits(char* buf, std::size_t n) {
    return shulib::bench::competitionBits(pros::c::competition_get_status(), buf, n);
}

}  // namespace

// A program calls this once its fault latch exists (fault_latch_hook.hpp).
void shulib::bench::attachFaultLatch(shulib::diag::FaultLatch* latch) noexcept {
    g_faults = latch;
}

void initialize() {
    shulib::setPreconditionHandler(&robotPreconditionHandler);  // first, before any device exists
    shulib::hal::pros::serialLine(
        "[BLD] robot tables: %s (src/robots/chassis_table.hpp, src/robots/mechanism_table.hpp)",
        kRobotBeacon);
    char bits[64];
    switch (kProgram) {
        case Program::BenchTests:
            shulib::bench::testerInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::Drive:
            shulib::bench::driveInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::Calibrate:
            shulib::bench::calibrateInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::InventedXDrive:
            shulib::bench::xdriveInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::AutonTester:
            shulib::bench::autonTesterInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::Competition:
            shulib::bench::competitionInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
        case Program::Tune:
            shulib::bench::tuneInitialize(kVariantBeacon, bootStatusBits(bits, sizeof bits));
            return;
    }
}

void disabled() {
    switch (kProgram) {
        case Program::BenchTests:
        case Program::Drive:
        case Program::Tune:
            shulib::bench::benchStateScreen(
                "DISABLED", "VEXos reports the robot DISABLED, so the driver task is not running.");
            return;
        case Program::Calibrate:
            shulib::bench::calibrateDisabled();  // the standard Calibrate's state screen (CAL1)
            return;
        case Program::InventedXDrive:
            return;
        case Program::AutonTester:
            // the tester's own task paints every state (one painter)
            return;
        case Program::Competition:
            shulib::bench::competitionDisabled();
            return;
    }
}

void competition_initialize() {
    switch (kProgram) {
        case Program::BenchTests:
        case Program::Drive:
        case Program::Tune:
            shulib::bench::benchStateScreen(
                "FIELD CONTROL CONNECTED",
                "A competition switch or field is connected through the controller.");
            return;
        case Program::Calibrate:
            shulib::bench::calibrateConnected();  // the standard Calibrate's state screen (CAL1)
            return;
        case Program::InventedXDrive:
            return;
        case Program::AutonTester:
            // the tester's own task paints every state (one painter)
            return;
        case Program::Competition:
            shulib::bench::competitionConnected();
            return;
    }
}

// Only shulib Competition runs an auton here (the one picked on its menu); never add a routine.
void autonomous() {
    switch (kProgram) {
        case Program::BenchTests:
            shulib::bench::benchStateScreen(
                "AUTONOMOUS",
                "No motion here, ever: the tester's DRIVE station lives under driver control.");
            return;
        case Program::Drive:
            shulib::bench::benchStateScreen("AUTONOMOUS", shulib::bench::kDriveAutonomousReason);
            shulib::bench::driveAutonomous();
            return;
        case Program::Calibrate:
            shulib::bench::calibrateAutonomous();  // the standard Calibrate's: no motion, ever (CAL1)
            return;
        case Program::InventedXDrive:
            shulib::bench::xdriveAutonomous();  // one serial line; no screen, no motion
            return;
        case Program::AutonTester:
            // NO auton here: the tester runs one only from its RUN
            return;
        case Program::Competition:
            shulib::bench::competitionAutonomous();
            return;
        case Program::Tune:
            shulib::bench::benchStateScreen(
                "AUTONOMOUS",
                "No motion here, ever: Tune moves mechanisms only under driver control, with no field.");
            return;
    }
}

void opcontrol() {
    switch (kProgram) {
        case Program::BenchTests:
            shulib::bench::runR3a();
            return;
        case Program::Drive:
            shulib::bench::runDrive();
            return;
        case Program::Calibrate:
            shulib::bench::runCalibrate();
            return;
        case Program::InventedXDrive:
            shulib::bench::runXdrive();
            return;
        case Program::AutonTester:
            // everything runs on the tester's own task, started by initialize()
            return;
        case Program::Competition:
            shulib::bench::runCompetition();
            return;
        case Program::Tune:
            shulib::bench::runTune();  // returns on a disable (every mechanism released); PROS calls it again
            return;
    }
}
