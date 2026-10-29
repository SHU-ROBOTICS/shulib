#pragma once
//
// hal/pros/calibrate — the standard "shulib Calibrate" on a V5 brain (chunk CAL1, Part A2a,
// 2026-10-18): the THIN shell that hands calibration::CalibrateProgram the robot's real devices.
// Everything the program decides is in include/shulib/calibration/calibrate_program.hpp and runs on
// the host test plant; this file only composes the PROS adapters and answers CalibrateIo's four
// calls:
//   * line()              hal::pros::serialWrapped("R3D", text) — ONE write() per line (SER1: never
//                         stdout's shared, unguarded buffer), the "[R3D] " tag every worksheet and
//                         every Calibrate capture since R3d has read; a line past one serial line's
//                         256 bytes goes out in "[R3D] ... " pieces, one write() each;
//   * competitionStatus() pros::c::competition_get_status();
//   * delayMs()           pros::delay();
//   * readShaftDegrees()  THE WATCH'S READ: three READS of PROS's C API by port —
//                         pros::c::motor_get_position(), converted by the units
//                         motor_get_encoder_units() reports (counts by the gearing
//                         motor_get_gearing() reports: 1800 / 900 / 300 a turn, HA-15). NO motor
//                         object of any kind is constructed — neither a pros::Motor nor a ProsMotor,
//                         whose constructor WRITES the gearset (T3's rule for a claw: never geared,
//                         never set to Coast, never pulsed). The vendored libpros.a's three reads
//                         call only PROS's registry check, the port mutex, __errno and one firmware
//                         *Get each (disassembled for the CAL1 brief, 2026-10-08) — the belief CAL1
//                         registers as HA-187 (in Part A2b): a read writes nothing to the motor.
//                         False when PROS answers PROS_ERR_F, or a unit or gearing it does not know.
//
// A TEAM'S CALIBRATE is one file that states its facts and calls the five functions below from
// PROS's five entry points (21 lines; CAL1's next part adds the guide chapter that shows it whole).
// Include this header in ONE translation unit: its default build stamp is that unit's __DATE__ and
// __TIME__, so two units would carry two stamps for one inline function.
//
// The composition is runCalibrate()'s own frame: the registry, the provider over it (no log copy),
// the SD store (its default root), the screen and the IO — each borrowed by the program for its run.
//
// THE STACK it takes on PROS's opcontrol task (measured 2026-10-29 by GCC, never on a robot): the
// provider alone is 7,288 B on ARM and run() holds the 7,712 B calibrator, so the deepest path GCC
// can see in the team's one translation unit (arm-none-eabi-g++ -fstack-usage -fcallgraph-info=su,
// the team project's flags, over the 21-line file) is 22,464 B — runCalibrate 8,712, then
// CalibrateProgram::run 10,272, then the DONE path down to the safe save's CRC — BEFORE newlib's
// file and printf frames and VEXos's file calls, which it cannot see. PROS runs opcontrol() on a task
// of TASK_STACK_DEPTH_DEFAULT = 0x2000 words (32 KiB, include/pros/rtos.h:72): about 70 % of it
// before those frames (R3d's Calibrate, which this replaces in CAL1 Part B: 15,168 B, "about half").
// It fits; it is the margin to watch if the program's frame grows.
//
// NOTHING HERE HAS RUN ON A ROBOT in this form. Its host test (test/pros_calibrate_adapter_test.cpp)
// runs it through the host PROS shim, which tests the shell against our BELIEF about PROS, not the
// belief itself (shim_control.hpp says so in full).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdint>

#include "shulib/calibration/calibrate_program.hpp"
#include "shulib/calibration/calibrate_screen.hpp"
#include "shulib/hal/pros/brain_screen.hpp"
#include "shulib/hal/pros/char_sink.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/device_registry.hpp"
#include "shulib/hal/pros/file_store.hpp"

namespace shulib::hal::pros {

static_assert(calibration::kCompetitionDisabled == static_cast<std::uint8_t>(COMPETITION_DISABLED)
                  && calibration::kCompetitionAutonomous
                         == static_cast<std::uint8_t>(COMPETITION_AUTONOMOUS)
                  && calibration::kCompetitionConnected
                         == static_cast<std::uint8_t>(COMPETITION_CONNECTED),
              "calibrate_program.hpp's competition bits must be PROS's");

/// calibration::CalibrateIo on the brain (header): the serial line, the competition status, a
/// plain wait, and the watch's read through PROS's C API with no motor object.
class ProsCalibrateIo final : public calibration::CalibrateIo {
public:
    /// "[R3D] <text>" on the USB serial port: ONE write() per line, through
    /// serialWrapped() (SER1) — a text past one serial line goes out in "[R3D] ... " pieces.
    void line(const char* text) override { serialWrapped("R3D", text != nullptr ? text : ""); }
    /// pros::c::competition_get_status().
    [[nodiscard]] std::uint8_t competitionStatus() override {
        return ::pros::c::competition_get_status();
    }
    /// pros::delay(ms).
    void delayMs(std::uint32_t ms) override { ::pros::delay(ms); }
    /// THE WATCH'S READ (header): motor_get_position() on `port` in the units the motor reports,
    /// turned into degrees of output shaft — no motor object, nothing written. False when PROS
    /// answers PROS_ERR_F (not a motor, or not answering), or a unit or gearing it does not know.
    [[nodiscard]] bool readShaftDegrees(int port, double& degrees) override {
        const auto p = static_cast<std::int8_t>(port);
        const double raw = ::pros::c::motor_get_position(p);
        if (!std::isfinite(raw)) {
            return false;  // PROS_ERR_F is +infinity: not a motor, or not answering
        }
        switch (::pros::c::motor_get_encoder_units(p)) {
            case ::pros::E_MOTOR_ENCODER_DEGREES: degrees = raw; return true;
            case ::pros::E_MOTOR_ENCODER_ROTATIONS: degrees = raw * 360.0; return true;
            case ::pros::E_MOTOR_ENCODER_COUNTS: {
                const ::pros::motor_gearset_e_t g = ::pros::c::motor_get_gearing(p);
                const double perTurn = g == ::pros::E_MOTOR_GEARSET_36   ? 1800.0
                                       : g == ::pros::E_MOTOR_GEARSET_18 ? 900.0
                                       : g == ::pros::E_MOTOR_GEARSET_06 ? 300.0
                                                                         : 0.0;
                if (perTurn == 0.0) {
                    return false;
                }
                degrees = raw * 360.0 / perTurn;
                return true;
            }
            default: return false;
        }
    }
};

/// initialize(): the boot banner (constructs nothing, powers nothing). `label` is a build's beacon,
/// or null; `buildStamp` defaults to this translation unit's own (header: ONE unit).
inline void calibrateInitialize(const char* label = nullptr,
                                const char* buildStamp = __DATE__ " " __TIME__) {
    ProsCalibrateIo io;
    calibration::calibrateBootBanner(io, label, ::pros::c::competition_get_status(), buildStamp);
}

/// disabled(), competition_initialize() and autonomous(): Calibrate's competition-state screen —
/// `state` and `why` are calibrate_screen.hpp's words (checked at compile time to fit).
inline void calibrateStateScreen(const char* state, const char* why) {
    ProsBrainScreen screen;
    ProsCalibrateIo io;
    calibration::calibrateStateScreen(screen, io, state, why);
}

/// disabled(): the state screen, DISABLED.
inline void calibrateDisabled() {
    calibrateStateScreen(calibration::screen_text::kStateNameDisabled.data(),
                         calibration::screen_text::kStateWhyDisabled.data());
}

/// competition_initialize(): the state screen, FIELD CONNECTED.
inline void calibrateConnected() {
    calibrateStateScreen(calibration::screen_text::kStateNameConnected.data(),
                         calibration::screen_text::kStateWhyConnected.data());
}

/// autonomous(): the state screen, AUTONOMOUS — no motion here, ever.
inline void calibrateAutonomous() {
    calibrateStateScreen(calibration::screen_text::kStateNameAutonomous.data(),
                         calibration::screen_text::kStateWhyAutonomous.data());
}

/// opcontrol(): the standard Calibrate on this brain, over the robot's real devices (header).
/// Returns when the field disables the robot or starts autonomous (every motor at 0 V + Coast), or
/// when no run can start. `buildStamp` defaults to this translation unit's own.
inline void runCalibrate(const calibration::CalibrateFacts& facts,
                         const char* buildStamp = __DATE__ " " __TIME__) {
    ProsDeviceRegistry registry;
    ProsDeviceProvider devices{registry};
    ProsFileStore card;
    ProsBrainScreen screen;
    ProsCalibrateIo io;
    calibration::CalibrateProgram program{facts, {devices, registry, card, screen, io}, buildStamp};
    program.run();
}

}  // namespace shulib::hal::pros
