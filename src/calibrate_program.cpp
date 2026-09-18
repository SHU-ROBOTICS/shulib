// calibrate_program.cpp — "shulib Calibrate": ROBOT TWO MEASURES ITS OWN DRIVETRAIN
// (chunk R3d Part 2, 2026-09-18).
//
// ═══ WHAT THIS IS ══════════════════════════════════════════════════════════════════
// The composition root for calibration::DrivetrainCalibrator (R3d Part 1). It finds the
// motors from the brain's own registry, discovers the IMU port, runs the calibrator's six
// phases with the operator answering on the touch screen, and — when the run reaches Done —
// writes the measured hal::DrivetrainDescription to /usd/shulib_drivetrain.bin, reads it
// back, and only then says SAVED. "shulib Teleop" boots from that file.
//
// Everything a human needs is on the brain's screen AND on the USB serial port with a
// `[R3D]` prefix, because the team lead reads the user port raw (`pros lsusb`, `cat`).
//
// ═══ IT HAS NEVER RUN ON HARDWARE ══════════════════════════════════════════════════
// Every number the calibrator uses is INVENTED for a first run (A4 register HA-136), the
// procedure has been proven only against the A2 host plant, and no description has ever been
// produced by a physical drivetrain. The boot banner and the screen say so. The first run is
// WHEELS UP, and it is EXPECTED to refuse at the yaw floor there — that is the program
// working, not failing (worksheet Station G).
//
// ═══ WHY THE CANDIDATES COME FROM THE REGISTRY, NOT THE TABLE (the whole point) ════
// A calibration that starts from a typed port list can only ever CONFIRM the typing. The
// chassis table's ports and signs became unverified the moment robot two's drivetrain was
// rebuilt; this program exists so that stops mattering. So: every port the registry says
// carries a MOTOR is a candidate, and which of them drive — and which way each is wired, and
// which side it is on — is decided by pushing the robot, not by reading a file.
//
// THREE facts still come from a human, and only three (the parent spec's list):
//   * the CARTRIDGE — physically unobservable from inside (every cartridge spins the rotor
//     the same and the firmware divides by whatever it was told). Taken from the chassis
//     table's `cartridge` field, printed as a BELIEF, and the program REFUSES if it is Unset.
//   * the KIND — Tank. The calibrator refuses anything else; the Makefile refuses to build
//     this program for any other ROBOT.
//   * which end is the FRONT — answered on the screen, once, during the push.
// The IMU PORT IS NOT ONE OF THEM (ruling P1): the registry names the port whose device is an
// IMU, so the robot reads it. The table's `imuPort` is printed as a CROSS-CHECK and a
// mismatch is a WARNING, not a refusal — a re-plugged IMU is not an error, and the
// calibration is taken against wherever the IMU IS.
//
// ═══ SIGNS: THE CALIBRATOR APPLIES THEM, SO THE ADAPTERS ARE POSITIVE ══════════════
// Every ProsMotor here is built with a POSITIVE port (the calibrator's rule 1). The sign is
// what the procedure MEASURES; an adapter built with a negative port would apply it twice.
// The sign lands inside the description's `signedPorts` entry and is applied ONCE, by PROS,
// in the adapter "shulib Teleop" builds from the record.
//
// ═══ SAVED IS A MEASUREMENT, NOT A CLAIM (ruling P5) ═══════════════════════════════
// fflush is the strongest persist PROS exposes and HA-122's durability half is untested. So
// the Done path is: encode → write → flush → CLOSE → re-open through ProsBlockSource →
// read back → decode → compare the bytes. Only then does the screen say SAVED. Anything else
// paints NOT SAVED with the reason, in red, and says the result is on screen only.
//
// ═══ EVERY EXIT LEAVES THE MOTORS AT 0 V, COAST ════════════════════════════════════
// An RAII guard runs the all-stop on every C++ exit path (return, throw), the drive
// program's pattern. A FIELD DISABLE is NOT a C++ exit — PROS deletes the opcontrol task and
// a deleted task runs no destructors — so the loop polls competition_is_disabled() every
// tick and EXITS cleanly on it, which does run the guard. The calibrator ALSO stops every
// motor on every refusal and on abort(); the guard is the belt to that pair of braces.
//
// ═══ THE SCREEN IS RESISTIVE (session 2 §9) ════════════════════════════════════════
// Touches are read on the RELEASE EDGE (the release counter), a firm press is needed, and
// the button row is four large targets a thumb can hit. The gaps between buttons belong to
// the button on their left, for the reason bench_r3a.cpp:1935-1942 records: on robot two's
// brain the very first recorded tap released one pixel into a gap and did nothing.
//
// ═══ -Wformat-nonliteral ═══════════════════════════════════════════════════════════
// src/ compiles under it (Part 1 was bitten). Every message in this file is built with a
// LITERAL format string into a local buffer; nothing takes a `const char* format`.

#include "calibrate_program.hpp"

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

#include "pros/apix.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "chassis_table.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/drivetrain_calibrator.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/block_source.hpp"
#include "shulib/hal/pros/clock.hpp"
#include "shulib/hal/pros/device_registry.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

using calibration::Answer;
using calibration::DrivetrainCalibrator;
using calibration::Phase;

/// Compiled-in build stamp (the tester's reasoning, verbatim: the hash names the COMMIT, the
/// stamp names the BUILD, and only the stamp answers "did my upload land?").
constexpr const char* kBuildStamp = __DATE__ " " __TIME__;

constexpr int kTickMs = 10;
constexpr int kPanelEveryTicks = 10;  // 100 ms

// ── the brain screen: 480x240 usable (the tester measured it; lv_conf.h agrees) ──────
constexpr std::int16_t kUsableW = 480;
constexpr std::uint32_t kColBg = 0x101418, kColBar = 0x1E5AA8, kColText = 0xF0F0F0,
                        kColDim = 0x9AA4AE, kColGood = 0x39C36E, kColWarn = 0xE8B23A,
                        kColBad = 0xE2564A, kColSub = 0xC8D8F0, kColBtn = 0x243040,
                        kColBtnEdge = 0x4A6080;

// ── output: USB serial, always. Every line the screen shows goes here too ────────────
void emit(const char* line) {
    std::printf("[R3D] %s\n", line);
    std::fflush(stdout);
}

void emitf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void emitf(const char* fmt, ...) {
    char buf[220];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, args);
    va_end(args);
    emit(buf);
}

/// One candidate: the physical port the registry reported, and the adapter built on it with a
/// POSITIVE port number (the calibrator applies the sign — it is what the run MEASURES).
struct Candidate {
    int port = 0;
    std::optional<hal::pros::ProsMotor> motor;
};

/// 0 V + Coast on every constructed candidate. The calibrator does this itself on every
/// refusal and on abort(); this is the composition root's own copy, for the exit paths that
/// never reach the calibrator at all.
void stopAll(Candidate* c, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        if (c[i].motor) {
            c[i].motor->setVoltage(units::Voltage{0.0});
            c[i].motor->setBrakeMode(hal::BrakeMode::Coast);
        }
    }
}

/// The all-stop on every C++ exit path (header: a task kill is not one of them).
struct AllStopGuard {
    Candidate* c;
    const std::size_t* n;
    ~AllStopGuard() { stopAll(c, *n); }
};

/// The chassis table's cartridge as the adapter's gearset. Only ever called after Unset has
/// been refused, so the default is unreachable — and is Blue rather than a silent guess,
/// matching the tank graph's own helper.
hal::pros::MotorGearset gearsetOf(Cartridge c) {
    switch (c) {
        case Cartridge::Red: return hal::pros::MotorGearset::Red;
        case Cartridge::Green: return hal::pros::MotorGearset::Green;
        default: return hal::pros::MotorGearset::Blue;
    }
}

/// The table's cartridge as the description's, for HumanFacts.
hal::MotorCartridge cartridgeOf(Cartridge c) {
    switch (c) {
        case Cartridge::Red: return hal::MotorCartridge::Red;
        case Cartridge::Green: return hal::MotorCartridge::Green;
        default: return hal::MotorCartridge::Blue;
    }
}

// ── the touch button row ─────────────────────────────────────────────────────────────
// Four large targets across the bottom, whose LABELS change with what the calibrator will
// accept right now (DrivetrainCalibrator::accepts). A button that is not live is not drawn,
// so a thumb cannot reach it: answer() ENDS THE RUN on an out-of-order answer, which is right
// (a touch that silently did nothing would leave the operator believing a step had been
// taken) and which makes a wrong button destructive.
constexpr std::int16_t kBtnY0 = 186, kBtnH = 46, kBtnGap = 6;
constexpr std::size_t kMaxButtons = 4;

struct Button {
    Answer answer = Answer::Abort;
    const char* label = "";
};

/// The buttons live THIS tick, in a fixed order with ABORT always last. Returns how many.
std::size_t buttonsFor(const DrivetrainCalibrator& cal, Button* out) {
    std::size_t n = 0;
    const auto offer = [&](Answer a, const char* label) {
        if (n < kMaxButtons && cal.accepts(a)) {
            out[n++] = Button{a, label};
        }
    };
    offer(Answer::Done, "DONE");
    offer(Answer::FrontFirst, "FRONT 1st");
    offer(Answer::BackFirst, "BACK 1st");
    offer(Answer::Distance24, "24 in");
    offer(Answer::Distance48, "48 in");
    offer(Answer::Distance72, "72 in");
    // ABORT is always live and always last, so its position never moves under a thumb.
    if (n == kMaxButtons) {
        n = kMaxButtons - 1;
    }
    out[n++] = Button{Answer::Abort, "ABORT"};
    return n;
}

void buttonBox(std::size_t i, std::size_t total, std::int16_t& x0, std::int16_t& x1) {
    const auto count = static_cast<std::int16_t>(total);
    const std::int16_t w = static_cast<std::int16_t>((kUsableW - (count + 1) * kBtnGap) / count);
    x0 = static_cast<std::int16_t>(kBtnGap + static_cast<std::int16_t>(i) * (w + kBtnGap));
    x1 = static_cast<std::int16_t>(x0 + w);
}

/// -1 when the touch landed outside the row; otherwise the button index. The GAPS belong to
/// the button on their left (integer division of the pitch), for the reason
/// src/bench_r3a.cpp:1935-1942 records: on robot two's brain the first recorded tap released
/// one pixel into a gap and did nothing, on a resistive screen where a fingernail's release
/// point wanders a few pixels.
int hitTest(std::int16_t tx, std::int16_t ty, std::size_t total) {
    if (total == 0 || ty < kBtnY0 || ty > kBtnY0 + kBtnH) {
        return -1;
    }
    std::int16_t x0 = 0, x1 = 0;
    buttonBox(0, total, x0, x1);
    const std::int16_t gridX0 = x0;
    const std::int16_t pitch = static_cast<std::int16_t>(x1 - x0 + kBtnGap);
    buttonBox(total - 1, total, x0, x1);
    if (tx < gridX0 || tx > x1) {
        return -1;
    }
    const int i = (tx - gridX0) / pitch;
    return i < static_cast<int>(total) ? i : static_cast<int>(total) - 1;
}

void drawButtons(const Button* b, std::size_t n) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, kBtnY0 - 2, kUsableW, static_cast<std::int16_t>(kBtnY0 + kBtnH));
    for (std::size_t i = 0; i < n; ++i) {
        std::int16_t x0 = 0, x1 = 0;
        buttonBox(i, n, x0, x1);
        const bool abort = b[i].answer == Answer::Abort;
        pros::c::screen_set_pen(abort ? kColBad : kColBtn);
        pros::c::screen_fill_rect(x0, kBtnY0, x1, static_cast<std::int16_t>(kBtnY0 + kBtnH));
        pros::c::screen_set_pen(abort ? kColBad : kColBtnEdge);
        pros::c::screen_draw_rect(x0, kBtnY0, x1, static_cast<std::int16_t>(kBtnY0 + kBtnH));
        pros::c::screen_set_eraser(abort ? kColBad : kColBtn);
        pros::c::screen_set_pen(kColText);
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, static_cast<std::int16_t>(x0 + 8),
                                 static_cast<std::int16_t>(kBtnY0 + 12), "%.10s", b[i].label);
        pros::c::screen_set_eraser(kColBg);
    }
}

/// One row of the panel, erased then printed.
void rowText(std::int16_t y, std::uint32_t colour, const char* text) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, y, kUsableW, static_cast<std::int16_t>(y + 16));
    pros::c::screen_set_pen(colour);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, y, "%.58s", text);
}

/// The fixed header bar: name, what this is, build stamp, UNRUN.
void drawHeader() {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColBar);
    pros::c::screen_fill_rect(0, 0, kUsableW, 32);
    pros::c::screen_set_eraser(kColBar);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 2, "shulib CALIBRATE");
    pros::c::screen_set_pen(kColSub);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 20,
                             "the robot measures its OWN drivetrain -- NEVER RUN BEFORE");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 300, 20, "BUILD %s", kBuildStamp);
    pros::c::screen_set_eraser(kColBg);
}

/// Paint a full-screen refusal and say it on serial. Nothing is powered after this.
void paintRefusal(const char* headline, const char* why, const char* fix) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColBad);
    pros::c::screen_fill_rect(0, 0, kUsableW, 34);
    pros::c::screen_set_eraser(kColBad);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 6, "shulib CALIBRATE -- REFUSED");
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_set_pen(kColBad);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 48, "%.34s", headline);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 84, "%.58s", why);
    if (std::strlen(why) > 58) {
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 100, "%.58s", why + 58);
    }
    pros::c::screen_set_pen(kColDim);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 128, "%.58s", fix);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 156, "Nothing is powered. BUILD %s",
                             kBuildStamp);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 176,
                             "Bench Tests (slot 3) is the diagnosis tool.");
    emitf("REFUSED: %s -- %s", headline, why);
    emitf("  %s", fix);
}

/// Everything the competition-state screens say, from inside this program. The calibrator
/// runs ONLY in driver control, enabled — it powers motors, and a pretend match (HA-128)
/// would start it without anybody standing by the battery.
void paintNotDriverControl() {
    const std::uint8_t s = pros::c::competition_get_status();
    char bits[72];
    std::snprintf(bits, sizeof bits, "0x%02x field=%s disabled=%s auton=%s",
                  static_cast<unsigned>(s), (s & COMPETITION_CONNECTED) ? "yes" : "no",
                  (s & COMPETITION_DISABLED) ? "yes" : "no",
                  (s & COMPETITION_AUTONOMOUS) ? "yes" : "no");
    emitf("competition state changed: %s -- calibration runs ONLY in driver control, enabled",
          bits);
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColWarn);
    pros::c::screen_print_at(pros::E_TEXT_LARGE, 10, 30, "NOT DRIVER CONTROL");
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 80,
                             "shulib Calibrate powers the motors, so it runs ONLY in");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 96,
                             "DRIVER CONTROL, enabled. The run was stopped: 0 V, coast.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 120, "status bits %s", bits);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 148,
                             "Unplug any competition switch or field cable, and NEVER");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 164,
                             "start this program from the CONTROLLER's menu (HA-128).");
}

/// Write the description to the card and PROVE it is there (ruling P5): encode, write, flush,
/// close, re-open, read back, compare bytes. `why` receives the first failure. Returns true
/// only when the bytes on the card are byte-identical to the bytes encoded here.
[[nodiscard]] bool saveAndReadBack(const hal::DrivetrainDescription& desc, char* why,
                                   std::size_t cap) {
    why[0] = '\0';
    std::byte encoded[calibration::kRecordBytes]{};
    const std::size_t n = calibration::encodeRecord(desc, encoded);
    if (n != calibration::kRecordBytes) {
        std::snprintf(why, cap, "the record would not encode (%d of %d bytes)",
                      static_cast<int>(n), static_cast<int>(calibration::kRecordBytes));
        return false;
    }
    {
        hal::pros::ProsBlockSink sink{calibration::kRecordFileName};
        if (!sink.isOpen()) {
            std::snprintf(why, cap, "no SD card in the brain (or %s could not be opened)",
                          sink.path());
            return false;
        }
        if (!sink.write(std::span<const std::byte>{encoded, n})) {
            std::snprintf(why, cap, "the card refused the write -- full, or failing");
            return false;
        }
        if (!sink.flush()) {
            std::snprintf(why, cap, "the card refused the flush -- the bytes are not on it");
            return false;
        }
    }  // the sink's destructor closes the FILE*, so the read below sees a closed file
    // ── THE READ-BACK. fflush is the strongest persist PROS exposes and its durability half
    //    is untested (HA-122), so SAVED is a measurement or it is not said.
    hal::pros::ProsBlockSource source{calibration::kRecordFileName};
    if (!source.isOpen()) {
        std::snprintf(why, cap, "the file was written but will not re-open for reading");
        return false;
    }
    // A buffer LARGER than the record on purpose: a longer file must show up as a length
    // refusal, not as "the first 180 bytes happened to fit".
    std::byte readBack[calibration::kRecordBytes + 16]{};
    const std::size_t got = source.readAll(readBack);
    if (got != n) {
        std::snprintf(why, cap, "read back %d bytes of %d -- the card did not keep them all",
                      static_cast<int>(got), static_cast<int>(n));
        return false;
    }
    if (std::memcmp(readBack, encoded, n) != 0) {
        std::snprintf(why, cap, "the bytes on the card differ from the bytes written");
        return false;
    }
    hal::DrivetrainDescription decoded{};
    char inner[192];
    if (!calibration::decodeRecord(std::span<const std::byte>{readBack, got}, decoded, inner,
                                   sizeof inner)) {
        std::snprintf(why, cap, "the file on the card will not decode: %.120s", inner);
        return false;
    }
    return true;
}

/// The per-tick panel: phase, the prompt LARGE, the progress line, and the per-port table.
void drawPanel(const DrivetrainCalibrator& cal, const char* imuLine) {
    const calibration::CalibratorStatus& st = cal.status();
    const calibration::CalibrationReport& rep = cal.report();

    const char* phaseWord = "?";
    switch (st.phase) {
        case Phase::Idle: phaseWord = "IDLE"; break;
        case Phase::WaitImu: phaseWord = "WAITING FOR THE IMU"; break;
        case Phase::PushCapture: phaseWord = "1/4 PUSH -- which ports drive"; break;
        case Phase::SideAssign: phaseWord = "2/4 PULSES -- which side each is on"; break;
        case Phase::ScalePush: phaseWord = "3/4 TAPE -- inches per radian"; break;
        case Phase::SpinTest: phaseWord = "4/4 SPIN -- effective track width"; break;
        case Phase::Done: phaseWord = "DONE"; break;
        case Phase::Aborted: phaseWord = "STOPPED"; break;
    }
    rowText(36, kColSub, phaseWord);

    // The instruction for the person, LARGE — it is the only thing that matters most ticks.
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, 54, kUsableW, 84);
    pros::c::screen_set_pen(st.phase == Phase::Aborted ? kColBad
                            : st.phase == Phase::Done  ? kColGood
                                                       : kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 56, "%.44s", st.prompt);

    rowText(86, kColDim, st.progress);
    rowText(104, kColDim, imuLine);

    // The per-port table, two columns of rows: port, sign, side, both pushes' deltas.
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, 122, kUsableW, 182);
    int shown = 0;
    for (std::size_t i = 0; i < rep.portCount && shown < 12; ++i) {
        const calibration::PortObservation& o = rep.ports[i];
        if (o.sign == 0 && st.phase != Phase::PushCapture) {
            continue;  // not a drive motor: it has nothing to say after the push
        }
        char line[48];
        std::snprintf(line, sizeof line, "%2d %s%s %+6.0f %+6.0f", static_cast<int>(o.port),
                      o.sign == 0 ? "? " : (o.sign < 0 ? "- " : "+ "),
                      o.side == 0 ? "?" : (o.side < 0 ? "L" : "R"),
                      o.pushDeltaRad * 180.0 / 3.14159265358979323846,
                      o.tapeDeltaRad * 180.0 / 3.14159265358979323846);
        const std::uint32_t colour = o.sign == 0 ? kColDim : (o.side == 0 ? kColWarn : kColGood);
        pros::c::screen_set_pen(colour);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, static_cast<std::int16_t>(8 + (shown % 2) * 240),
                                 static_cast<std::int16_t>(122 + (shown / 2) * 16), "%s", line);
        ++shown;
    }
}

/// The final screen: what the run concluded, and whether it is on the card.
void paintOutcome(const DrivetrainCalibrator& cal, bool saved, const char* saveWhy) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, 36, kUsableW, 182);
    if (cal.phase() == Phase::Done) {
        char described[128];
        cal.result().describe(described, sizeof described);
        pros::c::screen_set_pen(kColGood);
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 40, "MEASURED");
        pros::c::screen_set_pen(kColText);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 74, "%.58s", described);
        if (std::strlen(described) > 58) {
            pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 90, "%.58s", described + 58);
        }
        pros::c::screen_set_pen(saved ? kColGood : kColBad);
        if (saved) {
            pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 118, "SAVED  /usd/%.24s",
                                     calibration::kRecordFileName);
            pros::c::screen_set_pen(kColDim);
            pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 150,
                                     "written, flushed, RE-READ and compared byte for byte.");
            pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 166,
                                     "shulib Teleop (slot 2) will boot from it.");
        } else {
            pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 118, "NOT SAVED");
            pros::c::screen_set_pen(kColText);
            pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 150, "%.58s", saveWhy);
            pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 166,
                                     "The result is ON SCREEN ONLY -- copy it down NOW.");
        }
        return;
    }
    pros::c::screen_set_pen(kColBad);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 40, "STOPPED -- nothing was saved");
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 78, "%.58s", cal.reason());
    if (std::strlen(cal.reason()) > 58) {
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 94, "%.58s", cal.reason() + 58);
    }
    if (std::strlen(cal.reason()) > 116) {
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 110, "%.58s", cal.reason() + 116);
    }
    pros::c::screen_set_pen(kColDim);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 140,
                             "Every motor is at 0 V and coasting. Fix the cause, run again.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 156,
                             "Wheels up, a yaw-floor refusal is EXPECTED -- put it on the floor.");
}

}  // namespace

void runCalibrate() {
    Candidate candidates[hal::kMaxSmartPorts];
    std::size_t n = 0;
    AllStopGuard guard{candidates, &n};  // every C++ exit from here on: 0 V, coast

    // ── 1. the banner: what this is, and what it is NOT ──────────────────────────────
    emit("");
    emit("################################################################");
    emit("#  shulib CALIBRATE -- robot two MEASURES ITS OWN DRIVETRAIN");
    emit("#  which ports drive, which way each is wired, which side, inches per radian");
    emit("#  per side, and the EFFECTIVE track width -- then writes them to the SD card.");
    emit("#  THIS PROGRAM HAS NEVER RUN ON HARDWARE. It POWERS THE MOTORS.");
    emit("#  Every threshold in it is INVENTED for a first run (A4 register HA-136).");
    emit("################################################################");
    emitf("BUILD STAMP: %s   <-- must match the build you just uploaded", kBuildStamp);
    {
        // compiledBuildHash() is a string_view: NEVER hand it to %s (the 2026-08-18 data abort).
        const std::string_view hash = diag::compiledBuildHash();
        if (hash.empty()) {
            emit("build hash : [ERROR] MISSING -- the build injected no hash (S18.5)");
        } else {
            emitf("build hash : %.*s", static_cast<int>(hash.size()), hash.data());
        }
    }
    emitf("robot      : %s", kChassis.robot);
    emitf("record file: /usd/%s (written only after a COMPLETE run)",
          calibration::kRecordFileName);
    emitf("opcontrol entry; competition status 0x%02x",
          static_cast<unsigned>(pros::c::competition_get_status()));
    drawHeader();

    // ── 2. the census: the brain's own registry, not the table ───────────────────────
    const hal::pros::ProsDeviceRegistry registry;
    std::int8_t ports[hal::kMaxSmartPorts]{};
    int imuPort = 0;
    int imuCount = 0;
    emit("STAGE 0  DEVICE CENSUS (registry only; nothing is constructed yet)");
    emit("  PORT  device      (the chassis table is NOT consulted for this)");
    for (int p = 1; p <= static_cast<int>(hal::kMaxSmartPorts); ++p) {
        const hal::DeviceKind kind = registry.pluggedKind(p);
        if (kind == hal::DeviceKind::None) {
            continue;
        }
        emitf("  %4d  %s", p, hal::deviceKindWord(kind));
        if (kind == hal::DeviceKind::Imu) {
            ++imuCount;
            imuPort = p;
        }
        if (kind == hal::DeviceKind::Motor && n < hal::kMaxSmartPorts) {
            candidates[n].port = p;
            ports[n] = static_cast<std::int8_t>(p);  // POSITIVE: the calibrator applies the sign
            ++n;
        }
    }
    emitf("  %u motor(s), %d IMU(s)", static_cast<unsigned>(n), imuCount);

    // ── 3. the three human facts, and the refusals that protect them ─────────────────
    if (kChassis.cartridge == Cartridge::Unset) {
        paintRefusal("cartridge UNSET",
                     "the cartridge cannot be measured from inside; read the insert colour off "
                     "a motor",
                     "type it into src/chassis_table.hpp, rebuild, upload");
        return;
    }
    emitf("CARTRIDGE BELIEF: %s -- stated by a human and UNVERIFIABLE from inside (every "
          "cartridge spins the rotor the same)", cartridgeWord(kChassis.cartridge));
    if (imuCount != 1) {
        char why[140];
        std::snprintf(why, sizeof why,
                      "the registry reports %d IMUs; this procedure needs exactly one", imuCount);
        paintRefusal("IMU count is not 1", why,
                     "plug in exactly one inertial sensor and run again (Bench Tests, slot 3)");
        return;
    }
    if (kChassis.imuPort != 0 && static_cast<int>(kChassis.imuPort) != imuPort) {
        // A WARNING, not a refusal (ruling P1): a re-plugged IMU is not an error, and the
        // calibration is taken against wherever the IMU IS.
        emitf("WARNING: the chassis table says IMU port %u, the registry says port %d -- using "
              "the REGISTRY (the table is a cross-check, not the truth)",
              static_cast<unsigned>(kChassis.imuPort), imuPort);
    }
    emitf("IMU        : port %d, DISCOVERED from the registry (table says %u)", imuPort,
          static_cast<unsigned>(kChassis.imuPort));
    if (n < 2) {
        char why[140];
        std::snprintf(why, sizeof why,
                      "the registry found %u motor(s); a drivetrain needs at least two",
                      static_cast<unsigned>(n));
        paintRefusal("too few motors", why,
                     "re-seat the drive cables and check the census (Bench Tests, slot 3)");
        return;
    }

    // ── 4. one ProsMotor per motor port, POSITIVE, each in its own try ───────────────
    // The registry said MOTOR; the adapter can still refuse (its constructor writes the
    // gearset and reads it back, HA-98). A port that refuses is dropped from the candidate
    // list with a warning, because a calibration of nine motors is still a calibration —
    // and it is named, because a silently missing motor is how a side comes up short.
    const hal::pros::MotorGearset gearset = gearsetOf(kChassis.cartridge);
    std::size_t built = 0;
    for (std::size_t i = 0; i < n; ++i) {
        try {
            candidates[built].port = candidates[i].port;
            ports[built] = static_cast<std::int8_t>(candidates[i].port);
            candidates[built].motor.emplace(ports[built], gearset);
            ++built;
        } catch (const PreconditionError& e) {
            emitf("WARNING: port %d answered MOTOR in the registry but its adapter REFUSED: "
                  "%s -- dropped from the candidates",
                  candidates[i].port, e.what());
        }
    }
    n = built;
    if (n < 2) {
        paintRefusal("too few usable motors",
                     "fewer than two motor adapters would construct; see the WARNING lines",
                     "re-seat the drive cables and check the census (Bench Tests, slot 3)");
        return;
    }
    stopAll(candidates, n);  // 0 V + Coast is the DECLARED starting state, not an inherited one

    hal::IMotor* motorPtrs[hal::kMaxSmartPorts]{};
    for (std::size_t i = 0; i < n; ++i) {
        motorPtrs[i] = &*candidates[i].motor;
    }

    // ── 5. the IMU, the clock, the pacer, the calibrator ─────────────────────────────
    hal::pros::ProsClock clock{};
    std::optional<hal::pros::ProsImu> imu;
    try {
        imu.emplace(static_cast<std::uint8_t>(imuPort), math::Angle{}, clock);
        imu->calibrate();  // ONCE, here (HA-05); the calibrator waits on isReady()
    } catch (const PreconditionError& e) {
        paintRefusal("the IMU adapter REFUSED", e.what(),
                     "check the sensor and its cable (Bench Tests, slot 3)");
        return;
    }
    emitf("IMU calibrating (HA-23 says ~2 s) -- keep the robot STILL until the screen says to "
          "push it");

    char provenance[96];
    std::snprintf(provenance, sizeof provenance, "shulib Calibrate %s", kBuildStamp);
    calibration::HumanFacts facts{};
    facts.kind = hal::DrivetrainKind::Tank;
    facts.cartridge = cartridgeOf(kChassis.cartridge);
    facts.imuPort = static_cast<std::uint8_t>(imuPort);
    facts.provenance = provenance;

    DrivetrainCalibrator cal{std::span<hal::IMotor* const>{motorPtrs, n},
                             std::span<const std::int8_t>{ports, n}, *imu, clock};
    cal.start(facts);
    emitf("calibrator started: %u candidate(s), kind Tank, cartridge %s, IMU port %d, "
          "provenance \"%s\"",
          static_cast<unsigned>(n), cartridgeWord(kChassis.cartridge), imuPort, provenance);
    emit("EVERY THRESHOLD BELOW IS INVENTED (HA-136). Wheels UP on the first run: the pulses");
    emit("turn nothing, and the run REFUSES at the yaw floor. That is the program WORKING.");

    // ── 6. the loop ──────────────────────────────────────────────────────────────────
    hal::pros::ProsTickPacer pacer{};
    char imuLine[64];
    std::snprintf(imuLine, sizeof imuLine, "IMU port %d (discovered)  |  %u candidates", imuPort,
                  static_cast<unsigned>(n));
    Button buttons[kMaxButtons];
    std::size_t buttonCount = 0;
    std::int32_t lastRelease = pros::c::screen_touch_status().release_count;
    Phase lastPhase = Phase::Idle;
    int tick = 0;
    bool stoppedByField = false;

    while (true) {
        // A field DISABLE (or an autonomous period) exits this loop cleanly so the guard runs
        // before PROS kills the task. Polled every tick: 10 ms is the most the robot can drive
        // past it.
        if (pros::c::competition_is_disabled()
            || (pros::c::competition_get_status() & COMPETITION_AUTONOMOUS) != 0) {
            cal.abort("the field disabled the robot, or autonomous started");
            stoppedByField = true;
            break;
        }

        cal.tick();

        // ── touch, on the RELEASE EDGE (the resistive-screen rule) ──
        const pros::screen_touch_status_s_t touch = pros::c::screen_touch_status();
        if (touch.release_count != lastRelease) {
            lastRelease = touch.release_count;
            const int hit = hitTest(touch.x, touch.y, buttonCount);
            emitf("touch: x=%d y=%d release=%ld hit=%d", static_cast<int>(touch.x),
                  static_cast<int>(touch.y), static_cast<long>(touch.release_count), hit);
            if (hit >= 0 && static_cast<std::size_t>(hit) < buttonCount) {
                const Button& b = buttons[static_cast<std::size_t>(hit)];
                emitf("ANSWER: %s", b.label);
                cal.answer(b.answer);
            }
        }

        if (cal.phase() == Phase::Done || cal.phase() == Phase::Aborted) {
            break;
        }

        // ── the panel, at 10 Hz, plus an immediate repaint on any phase change ──
        if (tick % kPanelEveryTicks == 0 || cal.phase() != lastPhase) {
            if (cal.phase() != lastPhase) {
                lastPhase = cal.phase();
                emitf("phase: %s | %s", cal.status().prompt, cal.status().progress);
            }
            drawPanel(cal, imuLine);
            buttonCount = buttonsFor(cal, buttons);
            drawButtons(buttons, buttonCount);
        }
        ++tick;
        pacer.pace();
    }

    // ── 7. the outcome ───────────────────────────────────────────────────────────────
    stopAll(candidates, n);
    if (stoppedByField) {
        paintNotDriverControl();
        emit("STOPPED by the field/competition state -- every motor at 0 V, coast. Nothing saved.");
        return;
    }

    bool saved = false;
    char saveWhy[160] = "";
    if (cal.phase() == Phase::Done) {
        const hal::DrivetrainDescription& d = cal.result();
        char described[128];
        d.describe(described, sizeof described);
        emitf("MEASURED: %s", described);
        emitf("provenance: %s", d.provenance);
        emitf("  LEFT  ports (signed, AS MEASURED): count %u", static_cast<unsigned>(d.left.count));
        for (std::size_t i = 0; i < d.left.count; ++i) {
            emitf("    %+d", static_cast<int>(d.left.signedPorts[i]));
        }
        emitf("  RIGHT ports (signed, AS MEASURED): count %u",
              static_cast<unsigned>(d.right.count));
        for (std::size_t i = 0; i < d.right.count; ++i) {
            emitf("    %+d", static_cast<int>(d.right.signedPorts[i]));
        }
        emitf("  inches per radian: LEFT %.6f  RIGHT %.6f", d.left.inchesPerRadian.value(),
              d.right.inchesPerRadian.value());
        emitf("  effective track width: %.4f in (LARGER than the tape's -- a turning tank "
              "scrubs; HA-135)", d.trackWidth.value());
        const calibration::CalibrationReport& rep = cal.report();
        emitf("  spread over the tape push: LEFT %.4f rad  RIGHT %.4f rad (port 18 is expected "
              "to show here -- HA-130)", rep.leftSpreadRad, rep.rightSpreadRad);
        emitf("  spin widths: CCW %.4f in, CW %.4f in, disagreement %.4f", rep.widthCcwIn,
              rep.widthCwIn, rep.widthDisagreement);

        char why[192];
        if (!d.valid(why, sizeof why)) {
            // A description that cannot boot a robot is not worth writing (ruling P-R4's other
            // half): the format's decoder would refuse it anyway, and a file on the card that
            // refuses at every boot is worse than no file.
            std::snprintf(saveWhy, sizeof saveWhy, "the measured description is not valid: %.120s",
                          why);
            emitf("NOT SAVED -- %s", saveWhy);
        } else {
            saved = saveAndReadBack(d, saveWhy, sizeof saveWhy);
            if (saved) {
                emitf("SAVED -- /usd/%s: written, flushed, RE-READ and compared byte for byte "
                      "(%d bytes). shulib Teleop (slot 2) will boot from it.",
                      calibration::kRecordFileName,
                      static_cast<int>(calibration::kRecordBytes));
                emit("COPY THE 'MEASURED:' LINE ABOVE DOWN. Then run this program a SECOND time");
                emit("and check the two runs agree -- a number that does not reproduce is not a");
                emit("measurement (the reproducibility tolerances are HA-136, invented).");
            } else {
                emitf("NOT SAVED -- %s", saveWhy);
                emit("THE RESULT IS ON SCREEN AND IN THIS LOG ONLY. Copy it down NOW.");
            }
        }
    } else {
        emitf("STOPPED: %s", cal.reason());
        emit("Every motor is at 0 V and coasting. Nothing was written to the card.");
    }
    paintOutcome(cal, saved, saveWhy);

    // Idle here rather than returning: opcontrol() returning ends the program and the brain
    // clears the screen, which would throw away the one copy of an UNSAVED result. The field
    // (or the brain's own stop button) ends it.
    while (!pros::c::competition_is_disabled()) {
        pros::delay(250);
    }
}

}  // namespace shulib::bench
