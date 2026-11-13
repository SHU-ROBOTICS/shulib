#pragma once
//
// hal/pros/tune — "shulib Tune" on a V5 brain (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's
// D7 (a), §1.3): the THIN shell that hands mechanism::TuneProgram the robot's real devices.
// Everything the program decides is in include/shulib/mechanism/tune_program.hpp and runs on the host
// test plant; this file only composes the PROS adapters — the brief's list, item by item:
//   * THE PROVIDER, with its SD log copy at /usd/tune_log.txt APPENDED across boots (the brief's
//     re-check #14: a reboot back into Tune must not erase the last session's COPY — the Competition
//     program's mode, hal::pros::ProsFileMode::Append). The copy carries every line the program logs,
//     the COPY blocks included; the terminal gets them all too. A missing card leaves the terminal;
//   * THE SCREEN (ProsBrainScreen), THE CONTROLLER (the master: every button a STOP, the left stick the
//     jog) and THE COMPETITION STATUS (pros::c::competition_get_status(), ProsTuneIo);
//   * THE TASK: the program runs on PROS's OPCONTROL task — the standard Calibrate's (hal/pros/calibrate.hpp),
//     not a task of its own like the Auton Tester's (a delta from the brief's "the task", logged with the
//     run: Tune moves nothing unless driver control runs with no field, so on a disable or the
//     start of autonomous it releases every mechanism and returns — PROS calls opcontrol() again later,
//     and the program, in this file's STATIC storage, carries on with the same mechanisms and numbers; a
//     task PROS deletes mid-tick leaves the service's re-entrancy flag set, which the next entry clears
//     (MechanismService::enterDriverControl()); and the host's PROS shim cannot construct a task).
//
// A ROBOT'S TUNE is two calls from its program file — src/programs/tune_program.cpp:
//     tuneInitialize(kMechanisms, label);   // initialize(): the boot lines, nothing made
//     runTune(kMechanisms);                 // opcontrol(): the program
// with the build's mechanism table. The first call MAKES the shell (a function-local static, so it is
// made once, on PROS's initialize task, and lives for the program); every later call uses the same one.
//
// THE STACK: the shell lives in static storage, not on a task's stack — the program's state (its
// samples, its view, the table's copy) is static too.
//
// NOTHING HERE HAS RUN ON A ROBOT. Its host test (test/tune_program_test.cpp) runs it through the host
// PROS shim, which tests the shell against our BELIEF about PROS, not the belief itself.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/misc.h"
#pragma GCC diagnostic pop

#include <cstdint>
#include <cstdio>

#include "shulib/hal/pros/brain_screen.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/mechanism/tune_program.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace shulib::hal::pros {

static_assert(mechanism::kTuneCompetitionDisabled == static_cast<std::uint8_t>(COMPETITION_DISABLED)
                  && mechanism::kTuneCompetitionAutonomous == static_cast<std::uint8_t>(COMPETITION_AUTONOMOUS)
                  && mechanism::kTuneCompetitionConnected == static_cast<std::uint8_t>(COMPETITION_CONNECTED),
              "tune_program.hpp's competition bits must be PROS's");

/// The Tune program's SD copy of its log: a BARE name (the provider owns the /usd/ prefix), APPENDED
/// across boots (header).
inline constexpr const char* kTuneLogFile = "tune_log.txt";

/// mechanism::TuneIo on the brain: pros::c::competition_get_status().
class ProsTuneIo final : public mechanism::TuneIo {
public:
    /// pros::c::competition_get_status().
    [[nodiscard]] std::uint8_t competitionStatus() override { return ::pros::c::competition_get_status(); }
};

/// Everything "shulib Tune" runs on, in one object (header): the provider with its Appended SD copy,
/// the screen, the master controller, the status, and the program over them. Lives in static storage
/// (tuneShell()); not copyable or movable.
class ProsTuneShell {
public:
    /// The shell for the robot's `table` (a constant that outlives it). `copyMountRoot` is the card's
    /// mount point (ending in '/'), injectable so a host test writes the same path to a scratch
    /// directory; `buildStamp` the build's (the caller's __DATE__ " " __TIME__). Makes NO device.
    ProsTuneShell(const mechanism::MechanismTable& table, const char* buildStamp, const char* copyMountRoot = "/usd/")
        : devices_{kTuneLogFile, ProsFileMode::Append, copyMountRoot},
          program_{table, mechanism::TuneDevices{devices_, screen_, controller_, io_}, buildStamp} {}
    /// Not copyable: the program borrows its members.
    ProsTuneShell(const ProsTuneShell&) = delete;
    /// Not copy-assignable.
    ProsTuneShell& operator=(const ProsTuneShell&) = delete;
    /// Not movable.
    ProsTuneShell(ProsTuneShell&&) = delete;
    /// Not move-assignable.
    ProsTuneShell& operator=(ProsTuneShell&&) = delete;
    /// Destroys the program, then the devices it borrowed.
    ~ProsTuneShell() = default;

    /// The program.
    [[nodiscard]] mechanism::TuneProgram& program() noexcept { return program_; }
    /// The provider (its SD copy's state: serialCopyOpen(), serialCopyAppendOpen()).
    [[nodiscard]] ProsDeviceProvider& devices() noexcept { return devices_; }

private:
    ProsDeviceProvider devices_;
    ProsBrainScreen screen_{};
    ProsController controller_{ControllerId::Master};
    ProsTuneIo io_{};
    mechanism::TuneProgram program_;
};

/// The program's ONE shell, made on the first call (header) over `table`. Every later call returns the
/// same shell, whatever it is passed.
inline ProsTuneShell& tuneShell(const mechanism::MechanismTable& table,
                                const char* buildStamp = __DATE__ " " __TIME__) {
    static ProsTuneShell shell{table, buildStamp};
    return shell;
}

/// initialize(): the boot lines — makes NOTHING on the robot. `label` is the build's beacon, or null;
/// `statusWords` the competition status in words, or null for its byte in hex.
inline void tuneInitialize(const mechanism::MechanismTable& table, const char* label = nullptr,
                           const char* statusWords = nullptr) {
    char hex[8];
    std::snprintf(hex, sizeof hex, "0x%02x", static_cast<unsigned>(::pros::c::competition_get_status()));
    tuneShell(table).program().bootBanner(label, statusWords != nullptr ? statusWords : hex);
}

/// opcontrol(): the program (mechanism::TuneProgram::run()) — returns when VEXos reports the robot
/// DISABLED or autonomous starting, every mechanism released first.
inline void runTune(const mechanism::MechanismTable& table) { tuneShell(table).program().run(); }

}  // namespace shulib::hal::pros
