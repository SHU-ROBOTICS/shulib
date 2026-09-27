// calibrate_program.cpp — "shulib Calibrate": ROBOT TWO MEASURES ITS OWN DRIVETRAIN
// (chunk R3d Part 2, 2026-09-18; its screen REWRITTEN at R3d Part 2F-2, 2026-09-24, as a
// renderer of calibration::viewFor()).
//
// ═══ WHAT THIS IS ══════════════════════════════════════════════════════════════════
// The composition root for calibration::DrivetrainCalibrator. It finds the motors from the
// brain's own registry, discovers the IMU port, runs the calibrator with the operator answering
// on the touch screen, and — when the run reaches Done — writes the measured
// hal::DrivetrainDescription to /usd/shulib_drivetrain.bin, reads it back, and only then says
// SAVED. "shulib Teleop" boots from that file. The finished screen and the stopped screen both
// carry RUN AGAIN, which starts a fresh run inside the same program.
//
// ═══ WHAT HAS RUN ON HARDWARE, AND WHAT HAS NOT ════════════════════════════════════
// The Part 2 version of this program ran five starts on robot two on 2026-09-24 and finished
// none (each stopped at or before its side pulse — R3d-PROGRESS "Part 3"); the team lead could
// not follow its screen ("I CANT READ IT ... I DONT KNOW AFTER I PUSH IT WHAT IM SUPPOSED TO
// DO"). THIS version — Part 2F-1's one-push calibrator behind the screen below — FIRST RAN on
// robot two the evening of 2026-09-24: run 1 STOPPED on SPIN_DISAGREE, run 2 finished with no
// serial capture (R3d-PROGRESS "RUN 1" / "RUN 2"); its recorded reproduction run is still owed.
// (This paragraph said "HAS NEVER RUN ON HARDWARE" until G1 Part 2b's prose review.) Every calibrator
// threshold is INVENTED (A4 register HA-136), and every text
// budget, line height and position on the screen is an ESTIMATE derived from what earlier
// screens showed on a V5, not measured (Bench Tests' SCREEN RULER, button 9, settles them).
//
// ═══ THE SCREEN: THIS FILE RENDERS, calibrate_panel.hpp DECIDES ═════════════════════
// Everything the operator reads while a run is under way — the words, the one live number, the
// NEXT line, which buttons exist, where they are, when they arm — is decided by the PROS-free,
// host-tested calibration::viewFor() (include/shulib/calibration/calibrate_panel.hpp). This
// file only PAINTS a PanelView, and it paints only what changed (PanelPainter, below):
//   * a change of screen, or of any line's place or size, is a full repaint;
//   * a line whose COLOUR changed, the header, and any button change are painted the tick they
//     happen — the view is recomputed right after every answer too, never on a 10 Hz timer;
//   * a line whose TEXT changed (a live number) is repainted at most every kLiveRepaintSeconds.
//     PROS 4.2.2's print_at LEAKS its formatted buffer on every call (a read of the vendored
//     libpros.a found no free), so a live number reprinted every 10 ms tick would leak megabytes
//     over a few runs; 5 Hz is legible and leaks two orders of magnitude less. The program counts
//     its prints and reports the count at every outcome;
//   * an in-place text change never erases: the new text is printed padded with spaces to the
//     old text's length, and VEXos draws text pen-on-ERASER (an opaque cell — a hardware finding,
//     src/bench_r3a.cpp's menu), so the old characters are covered without a flicker.
// The program's own screens (starting, cannot start, not driver control, a program fault) use
// the same primitives and the same budgets, each literal checked at compile time below.
//
// ═══ ONLY A LITERAL "%s" EVER REACHES A PROS SCREEN CALL ═══════════════════════════
// PROS hands the already-formatted text to VEXos AS A FORMAT STRING, so a '%' that reaches the
// screen is re-read as a conversion with no argument behind it; and VEXos's fonts carry ASCII
// only. printText() (src/screen_text.hpp since G1 Part 2b, shared with "shulib Teleop"; it lived
// in this file until then) is the ONE path to screen_print_at: it copies the text into a buffer sized
// from the size's budget, turns '%' and control bytes into '?', a UTF-8 em/en dash into '-' and
// any other non-ASCII character into one '?' (the hal/pros adapters' precondition messages carry
// em dashes, and a refusal screen shows them), cuts it at the budget, and prints it through "%s".
// src/ compiles under -Wformat-nonliteral and -Wformat-truncation, which the src build gate
// FAILS on: every message here is built with a LITERAL format into a buffer of known size.
//
// ═══ TOUCH: calibration::TouchGate, AGAINST THE VIEW THAT IS ON THE SCREEN ══════════
// The screen is RESISTIVE and PROS reads a touch on its RELEASE; x/y are stale while idle and an
// error read carries -1 in every field. TouchGate owns all of that: the release edge only, error
// reads ignored and never replayed, the arming delay after every layout change (new buttons are
// painted GREYED and are dead for half a second, so a reflexive second tap lands on nothing),
// and a re-check against the calibrator before any answer is sent. This file hands it one
// sample per tick and the view it has just painted, and does what it says.
//
// ═══ RUN AGAIN: A NEW CALIBRATOR, NOTHING CARRIED OVER ═════════════════════════════
// There is no reset(). RUN AGAIN rebuilds the calibrator IN PLACE (std::optional::emplace, then
// start(); the IMU is already calibrated, so the IMU step passes on its first tick), rebuilds the
// TouchGate from the release that asked for it, and replaces this program's per-run record with
// a fresh one. What survives belongs to the PROGRAM, not to a run: the motor adapters, the IMU
// adapter, the clock, the pacer and the three stated facts.
//
// ═══ THE SERIAL RECORD (every line prefixed [R3D]) ═════════════════════════════════
// The team lead reads the user port raw (`pros lsusb`, `cat`). Besides the banner, the census and
// the outcome, a run prints: the config it used; a `screen:` line whenever the layout changes
// (what the operator was looking at); every touch, with the button it landed on and why it did
// or did not act; the push's live count as it changes; the PUSH TABLE at DONE (every candidate:
// travel as read, moved or still, the sign captured); the front answer and the signed drive
// ports; ONE LINE PER PULSE (port, level, yaw, its own travel, the others' largest, the verdict);
// every port LEFT OUT and why; ONE LINE PER SPIN DIRECTION; then the MEASURED line with the ports
// per side, both scales, the width and SAVED / NOT SAVED — or the STOPPED line with its refusal
// code and port; and the loop's worst gap between two ticks. Start B of the first bench night
// stopped at port 18 and the capture could not say why; these lines are what would have.
//
// ═══ WHY THE CANDIDATES COME FROM THE REGISTRY, NOT THE TABLE (the whole point) ════
// A calibration that starts from a typed port list can only ever CONFIRM the typing. Every port
// the registry says carries a MOTOR is a candidate, and which of them drive — which way each is
// wired, which side it is on — is decided by pushing the robot, not by reading a file.
// ONE EXCEPTION, since T3 (2026-09-26): a port src/mechanism_table.hpp DECLARES as a mechanism
// motor (robot two's claw pivots, 6 and 7) is dropped right after the census, before any adapter
// exists, and logged `port 6: claw pivot (declared) -- not a candidate, not touched`. The push
// could not tell a claw from a drive motor that did not turn, and step 4's adapter would write BLUE
// gearing to it and set it to Coast — a loaded claw would drop — before the run began. This is
// not a fourth human fact about the DRIVETRAIN (the three below stay three): it says which motors
// are NOT the drivetrain, which the registry cannot tell — it reports a motor's kind, never its
// job.
//
// THREE facts still come from a human, and only three (the parent spec's list):
//   * the CARTRIDGE — physically unobservable from inside. Taken from the chassis table's
//     `cartridge` field, printed as a BELIEF, and the program REFUSES if it is Unset.
//   * the KIND — Tank. The calibrator refuses anything else; the Makefile refuses to build this
//     program for any other ROBOT.
//   * which end is the FRONT — answered on the screen, once, after the push.
// The IMU PORT IS NOT ONE OF THEM (ruling P1): the registry names the port whose device is an
// IMU. The table's `imuPort` is printed as a CROSS-CHECK and a mismatch is a WARNING.
//
// ═══ SIGNS: THE CALIBRATOR APPLIES THEM, SO THE ADAPTERS ARE POSITIVE ══════════════
// Every ProsMotor here is built with a POSITIVE port (the calibrator's rule 1). The sign is what
// the procedure MEASURES; it lands inside the description's `signedPorts` and is applied ONCE,
// by PROS, in the adapter "shulib Teleop" builds from the record.
//
// ═══ SAVED IS A MEASUREMENT, NOT A CLAIM (ruling P5) ═══════════════════════════════
// encode → write → flush → CLOSE → re-open → read back → compare the bytes → decode. Only then
// is the panel told SaveOutcome::Saved; anything else is NotSaved, in red, "copy these numbers".
//
// ═══ EVERY EXIT LEAVES THE MOTORS AT 0 V, COAST ════════════════════════════════════
// An RAII guard runs the all-stop on every C++ exit path. A FIELD DISABLE is NOT a C++ exit —
// PROS deletes the opcontrol task and a deleted task runs no destructors — so the loop polls
// competition_is_disabled() every tick and EXITS cleanly on it, which does run the guard. The
// calibrator ALSO stops every motor on every refusal and on abort(); the guard is the belt to
// that pair of braces, and a PreconditionError inside the loop is caught once at its edge.

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

#include "mechanism_table.hpp"  // T3: the declared mechanism ports Calibrate never touches
#include "chassis_table.hpp"
#include "screen_text.hpp"  // printText(), g_screenPrints (moved there at G1 Part 2b)
#include "shulib/calibration/calibrate_panel.hpp"
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
using calibration::PanelButton;
using calibration::PanelButtonId;
using calibration::PanelLine;
using calibration::PanelRect;
using calibration::PanelScreen;
using calibration::PanelTone;
using calibration::PanelView;
using calibration::Phase;
using calibration::SaveOutcome;
using calibration::TextSize;
using calibration::TouchAction;
using calibration::TouchGate;
using calibration::TouchResult;
using calibration::TouchSample;

/// Compiled-in build stamp (the tester's reasoning, verbatim: the hash names the COMMIT, the
/// stamp names the BUILD, and only the stamp answers "did my upload land?"). The serial banner
/// prints it whole; the screen's header carries its time (kBuildTimeText).
constexpr const char* kBuildStamp = __DATE__ " " __TIME__;

/// The least time between two in-place repaints of one line whose TEXT changed and whose colour
/// did not — a live number. 5 Hz: legible, and it bounds PROS's per-print leak (header).
constexpr double kLiveRepaintSeconds = 0.2;

/// How often the push's live count may reach the serial log, at most, and how often it does at
/// least while nothing changes.
constexpr double kPushLogMinSeconds = 0.25;
constexpr double kPushLogKeepaliveSeconds = 5.0;

// ═══ OUTPUT: USB serial, always ═════════════════════════════════════════════════════
void emit(const char* line) {
    std::printf("[R3D] %s\n", line);
    std::fflush(stdout);
}

void emitf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void emitf(const char* fmt, ...) {
    char buf[240];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, args);
    va_end(args);
    emit(buf);
}

// ═══ THE SCREEN: colours, the ONE print path, the program's own words ════════════════

// The palette. Dark ground; the header bar's colour IS the state (blue unpowered, amber powered,
// green saved, red stopped). Contrast ratios (WCAG formula) against what each is drawn on:
// white text on the ground 16.2:1, amber 9.6, green 8.1, grey 7.3, red 5.0; white on the blue bar
// 6.0; the dark ground colour on the amber, green and red bars 9.6 / 8.1 / 5.0; white on the
// ABORT fill 5.6. The old panel's live buttons were a 1.38:1 fill on this ground.
constexpr std::uint32_t kColBg = 0x101418;        // the ground, and dark text on a bright bar
constexpr std::uint32_t kColText = 0xF0F0F0;      // ordinary text
constexpr std::uint32_t kColBlue = 0x1E5AA8;      // the unpowered header; the question boxes
constexpr std::uint32_t kColAmber = 0xE8B23A;     // powered: stand clear
constexpr std::uint32_t kColGreen = 0x39C36E;     // ready, saved, past the floor; DONE's fill
constexpr std::uint32_t kColRed = 0xE2564A;       // stopped, not saved; ABORT's edge
constexpr std::uint32_t kColGrey = 0x9AA4AE;      // quiet text; a greyed button's label
constexpr std::uint32_t kColGreyFill = 0x3A4450;  // a greyed (arming) button's fill
constexpr std::uint32_t kColAbortFill = 0xB0302A; // ABORT's fill (white on it: 5.6:1)
constexpr std::uint32_t kColEdge = 0xC8D8F0;      // a live button's edge; Blue text, if ever asked

/// A text tone as a colour on the ground.
std::uint32_t textColour(PanelTone t) {
    switch (t) {
        case PanelTone::Amber: return kColAmber;
        case PanelTone::Green: return kColGreen;
        case PanelTone::Red: return kColRed;
        case PanelTone::Grey: return kColGrey;
        case PanelTone::Blue: return kColEdge;  // no panel line is Blue today; light blue if one is
        case PanelTone::White: return kColText;
    }
    return kColText;
}

/// A header tone as the bar's fill, and the text colour that reads on it.
std::uint32_t barColour(PanelTone t) {
    switch (t) {
        case PanelTone::Amber: return kColAmber;
        case PanelTone::Green: return kColGreen;
        case PanelTone::Red: return kColRed;
        case PanelTone::Grey: return kColGreyFill;
        case PanelTone::Blue:
        case PanelTone::White: return kColBlue;
    }
    return kColBlue;
}
std::uint32_t barTextColour(PanelTone t) {
    return (t == PanelTone::Amber || t == PanelTone::Green || t == PanelTone::Red) ? kColBg
                                                                                   : kColText;
}

// formatOf(), the budget static_assert, g_screenPrints and printText() lived here until G1
// Part 2b (2026-09-26), which moved them VERBATIM into src/screen_text.hpp so "shulib Teleop"
// paints through the same path; every call below is unchanged, and g_screenPrints (now
// shulib::bench::g_screenPrints, found by the same unqualified name) still counts this program's
// prints.

// ── the header bar: the step word left (MEDIUM), the build time right (SMALL) ─────────
constexpr std::int16_t kHeaderWordX = 8;
constexpr std::int16_t kHeaderWordY = 4;
constexpr std::int16_t kBuildX = 340;
constexpr std::int16_t kBuildY = 7;
/// The build TIME on every screen's header, so a stale upload shows at a glance (worksheet G.1.1:
/// "if a stamp is not the build you just uploaded, the upload did not land"). The whole stamp,
/// with its date, is on the serial banner.
constexpr std::string_view kBuildTimeText = "BUILD " __TIME__;
// The header word (at most kHeaderCols MEDIUM characters) must end before the build time, and the
// build time before the right edge — at the widths the budgets themselves imply (472 px over the
// budget), which are ESTIMATES like the budgets.
static_assert(kHeaderWordX + static_cast<int>(calibration::kHeaderCols)
                          * (472 / static_cast<int>(calibration::kMediumCols))
                  < kBuildX,
              "the header word could reach the build time");
static_assert(kBuildX + static_cast<int>(kBuildTimeText.size())
                            * (472 / static_cast<int>(calibration::kSmallCols))
                  <= 472,
              "the build time could run off the right edge");

/// Fill the header bar with `tone`'s colour and write `word` and the build time on it.
void paintHeader(PanelTone tone, const char* word) {
    const std::uint32_t bar = barColour(tone);
    const PanelRect r = calibration::kHeaderBar;
    pros::c::screen_set_pen(bar);
    pros::c::screen_fill_rect(r.x0, r.y0, r.x1, r.y1);
    printText(TextSize::Medium, kHeaderWordX, kHeaderWordY, barTextColour(tone), bar, word);
    printText(TextSize::Small, kBuildX, kBuildY, barTextColour(tone), bar, kBuildTimeText.data());
}

/// Erase everything below the header bar, down to the last usable row.
void eraseContent() {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, static_cast<std::int16_t>(calibration::kHeaderBar.y1 + 1),
                               static_cast<std::int16_t>(calibration::kPanelWidth - 1),
                               calibration::kPanelLastRow);
}

/// One button: its fill says what it is (DONE green, a question or RUN AGAIN blue, ABORT dark
/// red, a greyed button — still arming — dark grey), a light edge, and its label centred at the
/// width the MEDIUM budget implies.
void paintButton(const PanelButton& b) {
    std::uint32_t fill = kColBlue;
    std::uint32_t edge = kColEdge;
    std::uint32_t label = kColText;
    if (!b.armed) {
        fill = kColGreyFill;
        edge = kColGrey;
        label = kColGrey;
    } else if (b.id == PanelButtonId::Abort) {
        fill = kColAbortFill;
        edge = kColRed;
    } else if (b.id == PanelButtonId::Done) {
        fill = kColGreen;
        label = kColBg;
    }
    const PanelRect r = b.rect;
    pros::c::screen_set_pen(fill);
    pros::c::screen_fill_rect(r.x0, r.y0, r.x1, r.y1);
    pros::c::screen_set_pen(edge);
    pros::c::screen_draw_rect(r.x0, r.y0, r.x1, r.y1);
    pros::c::screen_draw_rect(static_cast<std::int16_t>(r.x0 + 1),
                              static_cast<std::int16_t>(r.y0 + 1),
                              static_cast<std::int16_t>(r.x1 - 1),
                              static_cast<std::int16_t>(r.y1 - 1));
    constexpr int kCharW = 472 / static_cast<int>(calibration::kMediumCols);
    const int width = r.x1 - r.x0 + 1;
    const int height = r.y1 - r.y0 + 1;
    const int textW = static_cast<int>(std::strlen(b.label)) * kCharW;
    const int x = r.x0 + (width > textW + 8 ? (width - textW) / 2 : 4);
    const int y = r.y0 + (height - calibration::kMediumHeight) / 2;
    printText(TextSize::Medium, static_cast<std::int16_t>(x), static_cast<std::int16_t>(y), label,
              fill, b.label);
}

/// Where a button was, back to the ground.
void eraseButton(const PanelButton& b) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(b.rect.x0, b.rect.y0, b.rect.x1, b.rect.y1);
}

[[nodiscard]] bool sameButton(const PanelButton& a, const PanelButton& b) {
    return a.id == b.id && a.rect == b.rect && a.armed == b.armed
           && std::strcmp(a.label, b.label) == 0;
}

[[nodiscard]] bool samePlace(const PanelLine& a, const PanelLine& b) {
    return a.x == b.x && a.y == b.y && a.size == b.size;
}

/// The renderer: what is PAINTED, and a render() that brings the screen to a new view by
/// painting only the difference (the header's rules, at the top of this file).
class PanelPainter {
public:
    /// The next render() is a full repaint (a run was rebuilt, or another screen painted over it).
    void invalidate() { valid_ = false; }

    void render(const PanelView& v, double now) {
        bool full = !valid_ || v.screen != screen_ || v.lineCount != lineCount_;
        for (std::size_t i = 0; !full && i < v.lineCount; ++i) {
            full = !samePlace(v.lines[i], lines_[i]);
        }
        if (full) {
            eraseContent();
            paintHeader(v.headerTone, v.headerWord);
            for (std::size_t i = 0; i < v.lineCount; ++i) {
                const PanelLine& l = v.lines[i];
                printText(l.size, l.x, l.y, textColour(l.tone), kColBg, l.text);
                lineAt_[i] = now;
            }
            for (std::size_t b = 0; b < v.buttonCount; ++b) {
                paintButton(v.buttons[b]);
            }
            adopt(v);
            valid_ = true;
            return;
        }
        if (v.headerTone != headerTone_ || std::strcmp(v.headerWord, headerWord_) != 0) {
            paintHeader(v.headerTone, v.headerWord);
            headerTone_ = v.headerTone;
            headerWord_ = v.headerWord;
        }
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            const PanelLine& want = v.lines[i];
            PanelLine& have = lines_[i];
            const bool toneChanged = want.tone != have.tone;
            const bool textChanged = std::strcmp(want.text, have.text) != 0;
            if (!toneChanged && (!textChanged || now - lineAt_[i] < kLiveRepaintSeconds)) {
                continue;  // unchanged, or a live number that was repainted too recently
            }
            printText(want.size, want.x, want.y, textColour(want.tone), kColBg, want.text,
                      std::strlen(have.text));
            have = want;
            lineAt_[i] = now;
        }
        // Buttons: erase what vanished, then paint what is new or changed (armed, moved).
        for (std::size_t o = 0; o < buttonCount_; ++o) {
            bool kept = false;
            for (std::size_t b = 0; b < v.buttonCount && !kept; ++b) {
                kept = v.buttons[b].id == buttons_[o].id && v.buttons[b].rect == buttons_[o].rect;
            }
            if (!kept) {
                eraseButton(buttons_[o]);
            }
        }
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            bool same = false;
            for (std::size_t o = 0; o < buttonCount_ && !same; ++o) {
                same = sameButton(v.buttons[b], buttons_[o]);
            }
            if (!same) {
                paintButton(v.buttons[b]);
            }
        }
        buttonCount_ = v.buttonCount;
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            buttons_[b] = v.buttons[b];
        }
    }

private:
    void adopt(const PanelView& v) {
        screen_ = v.screen;
        headerTone_ = v.headerTone;
        headerWord_ = v.headerWord;
        lineCount_ = v.lineCount;
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            lines_[i] = v.lines[i];
        }
        buttonCount_ = v.buttonCount;
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            buttons_[b] = v.buttons[b];
        }
    }

    bool valid_ = false;
    PanelScreen screen_ = PanelScreen::NotStarted;
    PanelTone headerTone_ = PanelTone::Blue;
    const char* headerWord_ = "";  // the panel's words are string literals: the pointer stays valid
    PanelLine lines_[calibration::kPanelMaxLines]{};
    double lineAt_[calibration::kPanelMaxLines]{};
    std::uint8_t lineCount_ = 0;
    PanelButton buttons_[calibration::kPanelMaxButtons]{};
    std::uint8_t buttonCount_ = 0;
};

// ── the program's OWN screens: every literal here, checked against its budget ─────────
// The positions assume the panel's estimated line heights (LARGE 34, MEDIUM 20, SMALL 16 px) and
// keep every line's bottom above row 236. Each is a string LITERAL, so its .data() is the
// NUL-terminated string printText() reads.
namespace text {
// header words (kHeaderCols, MEDIUM)
inline constexpr std::string_view kHeaderStarting = "CALIBRATE";
inline constexpr std::string_view kHeaderCannot = "CANNOT START";
inline constexpr std::string_view kHeaderStopped = "STOPPED -- MOTORS OFF";
// LARGE (kLargeCols)
inline constexpr std::string_view kHandsOff = "HANDS OFF";
inline constexpr std::string_view kStartingUp = "STARTING UP";
inline constexpr std::string_view kCartridgeUnset = "CARTRIDGE UNSET";
inline constexpr std::string_view kImuCount = "IMU COUNT NOT 1";
inline constexpr std::string_view kTooFewMotors = "TOO FEW MOTORS";
inline constexpr std::string_view kMotorsRefused = "MOTORS REFUSED";
inline constexpr std::string_view kImuRefused = "IMU REFUSED";
inline constexpr std::string_view kProgramFault = "PROGRAM FAULT";
inline constexpr std::string_view kNotDriverControl = "NOT DRIVER CONTROL";
// MEDIUM (kMediumCols)
inline constexpr std::string_view kFindingDevices = "finding the motors and the IMU";
// SMALL (kSmallCols)
inline constexpr std::string_view kNothingPowered =
    "nothing is powered -- fix it, then restart slot 4";
inline constexpr std::string_view kDiagnosisTool = "Bench Tests (slot 3) is the diagnosis tool";
inline constexpr std::string_view kMoreOnSerial = "the full text is on the serial log";
inline constexpr std::string_view kFaultIdle = "every motor is at 0 V, coasting";
inline constexpr std::string_view kNdc1 = "It powers the motors, so it runs ONLY in";
inline constexpr std::string_view kNdc2 = "DRIVER CONTROL, enabled. Motors: 0 V, coasting.";
inline constexpr std::string_view kNdc3 = "Unplug any competition switch or field cable,";
inline constexpr std::string_view kNdc4 = "and NEVER start it from the CONTROLLER's menu.";
inline constexpr std::string_view kNdc5 = "Any result is on the serial log too ([R3D]).";

struct Checked {
    std::string_view text;
    std::size_t cols;
};
inline constexpr Checked kAll[] = {
    {kHeaderStarting, calibration::kHeaderCols},  {kHeaderCannot, calibration::kHeaderCols},
    {kHeaderStopped, calibration::kHeaderCols},   {kHandsOff, calibration::kLargeCols},
    {kStartingUp, calibration::kLargeCols},       {kCartridgeUnset, calibration::kLargeCols},
    {kImuCount, calibration::kLargeCols},         {kTooFewMotors, calibration::kLargeCols},
    {kMotorsRefused, calibration::kLargeCols},    {kImuRefused, calibration::kLargeCols},
    {kProgramFault, calibration::kLargeCols},     {kNotDriverControl, calibration::kLargeCols},
    {kFindingDevices, calibration::kMediumCols},  {kNothingPowered, calibration::kSmallCols},
    {kDiagnosisTool, calibration::kSmallCols},    {kMoreOnSerial, calibration::kSmallCols},
    {kFaultIdle, calibration::kSmallCols},        {kNdc1, calibration::kSmallCols},
    {kNdc2, calibration::kSmallCols},             {kNdc3, calibration::kSmallCols},
    {kNdc4, calibration::kSmallCols},             {kNdc5, calibration::kSmallCols},
    {kBuildTimeText, calibration::kSmallCols},
};
constexpr bool allFit() {
    for (const Checked& c : kAll) {
        if (!calibration::screenText(c.text, c.cols)) {
            return false;
        }
    }
    return true;
}
static_assert(allFit(), "calibrate_program: a screen literal does not fit its budget, or is not "
                        "ASCII, or contains a percent sign");
}  // namespace text

/// The first screen, while the census runs and the adapters and the IMU are built.
void paintStarting() {
    eraseContent();
    paintHeader(PanelTone::Blue, text::kHeaderStarting.data());
    printText(TextSize::Large, 8, 30, kColText, kColBg, text::kHandsOff.data());
    printText(TextSize::Large, 8, 66, kColText, kColBg, text::kStartingUp.data());
    printText(TextSize::Medium, 8, 110, kColGrey, kColBg, text::kFindingDevices.data());
}

/// A screen the run never gets past: a header, a LARGE headline, `why` wrapped in MEDIUM (three
/// lines at most), `fix` wrapped in SMALL (two lines), a footer, and where to look next — or, when
/// either text did not fit, that the whole of it is on the serial log. Nothing is powered after
/// this.
void paintCannot(std::string_view header, PanelTone tone, std::string_view headline,
                 const char* why, const char* fix, std::string_view footer) {
    eraseContent();
    paintHeader(tone, header.data());
    printText(TextSize::Large, 8, 34, kColRed, kColBg, headline.data());
    bool cut = false;
    const auto w = calibration::wrapWords<calibration::kMediumCols, 3>(why != nullptr ? why : "");
    for (std::size_t k = 0; k < w.count; ++k) {
        printText(TextSize::Medium, 8, static_cast<std::int16_t>(76 + 22 * static_cast<int>(k)),
                  kColText, kColBg, w.line[k]);
    }
    cut = cut || !w.complete;
    const auto f = calibration::wrapWords<calibration::kSmallCols, 2>(fix != nullptr ? fix : "");
    for (std::size_t k = 0; k < f.count; ++k) {
        printText(TextSize::Small, 8, static_cast<std::int16_t>(146 + 16 * static_cast<int>(k)),
                  kColAmber, kColBg, f.line[k]);
    }
    cut = cut || !f.complete;
    printText(TextSize::Small, 8, 186, kColGrey, kColBg, footer.data());
    printText(TextSize::Small, 8, 202, kColGrey, kColBg,
              cut ? text::kMoreOnSerial.data() : text::kDiagnosisTool.data());
}

/// Refused before any run could start (the census, the stated facts, an adapter): the screen,
/// and the same words on serial.
void paintRefusal(std::string_view headline, const char* why, const char* fix) {
    paintCannot(text::kHeaderCannot, PanelTone::Red, headline, why, fix, text::kNothingPowered);
    emitf("REFUSED: %.*s -- %s", static_cast<int>(headline.size()), headline.data(), why);
    emitf("  %s", fix);
}

/// The calibrator runs ONLY in driver control, enabled — it powers motors, and a pretend match
/// (HA-128) would start it without anybody standing by the battery.
void paintNotDriverControl() {
    const std::uint8_t s = pros::c::competition_get_status();
    char bits[calibration::kSmallCols + 1];
    std::snprintf(bits, sizeof bits, "status bits 0x%02x field=%s disabled=%s auton=%s",
                  static_cast<unsigned>(s), (s & COMPETITION_CONNECTED) ? "yes" : "no",
                  (s & COMPETITION_DISABLED) ? "yes" : "no",
                  (s & COMPETITION_AUTONOMOUS) ? "yes" : "no");
    emitf("competition state changed: %s -- calibration runs ONLY in driver control, enabled",
          bits);
    eraseContent();
    paintHeader(PanelTone::Red, text::kHeaderStopped.data());
    printText(TextSize::Large, 8, 34, kColAmber, kColBg, text::kNotDriverControl.data());
    printText(TextSize::Small, 8, 80, kColText, kColBg, text::kNdc1.data());
    printText(TextSize::Small, 8, 96, kColText, kColBg, text::kNdc2.data());
    printText(TextSize::Small, 8, 120, kColGrey, kColBg, bits);
    printText(TextSize::Small, 8, 146, kColText, kColBg, text::kNdc3.data());
    printText(TextSize::Small, 8, 162, kColText, kColBg, text::kNdc4.data());
    printText(TextSize::Small, 8, 188, kColGrey, kColBg, text::kNdc5.data());
}

// ═══ THE MOTORS ═════════════════════════════════════════════════════════════════════

/// One candidate: the physical port the registry reported, and the adapter built on it with a
/// POSITIVE port number (the calibrator applies the sign — it is what the run MEASURES).
struct Candidate {
    int port = 0;
    std::optional<hal::pros::ProsMotor> motor;
};

/// 0 V + Coast on every constructed candidate. The calibrator does this itself on every refusal
/// and on abort(); this is the composition root's own copy, for the exit paths that never reach
/// the calibrator at all.
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

/// The chassis table's cartridge as the adapter's gearset. Only ever called after Unset has been
/// refused, so the default is unreachable — and is Blue rather than a silent guess, as the tank
/// graph's matching helper in src/main.cpp was until G1 Part 2b retired it (the device provider's
/// own gearsetOf() now refuses a byte outside the enum instead of defaulting).
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

// ═══ THE SD CARD ════════════════════════════════════════════════════════════════════

/// Write the description to the card and PROVE it is there (ruling P5): encode, write, flush,
/// close, re-open, read back, compare bytes. `why` receives the first failure. Returns true only
/// when the bytes on the card are byte-identical to the bytes encoded here.
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
    // ── THE READ-BACK. fflush is the strongest persist PROS exposes and its durability half is
    //    untested (HA-122), so SAVED is a measurement or it is not said.
    hal::pros::ProsBlockSource source{calibration::kRecordFileName};
    if (!source.isOpen()) {
        std::snprintf(why, cap, "the file was written but will not re-open for reading");
        return false;
    }
    // A buffer LARGER than the record on purpose: a longer file must show up as a length refusal,
    // not as "the first 180 bytes happened to fit".
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

// ═══ THE SERIAL RECORD OF ONE RUN ═══════════════════════════════════════════════════

[[nodiscard]] bool finished(Phase p) { return p == Phase::Done || p == Phase::Aborted; }

const char* phaseWord(Phase p) {
    switch (p) {
        case Phase::Idle: return "IDLE";
        case Phase::WaitImu: return "IMU";
        case Phase::PushCapture: return "PUSH";
        case Phase::SideAssign: return "PULSES";
        case Phase::SpinTest: return "SPIN";
        case Phase::Done: return "DONE";
        case Phase::Aborted: return "STOPPED";
    }
    return "?";
}

const char* buttonWord(PanelButtonId id) {
    switch (id) {
        case PanelButtonId::Done: return "DONE";
        case PanelButtonId::Front: return "FRONT FIRST";
        case PanelButtonId::Back: return "BACK FIRST";
        case PanelButtonId::Abort: return "ABORT";
        case PanelButtonId::RunAgain: return "RUN AGAIN";
    }
    return "?";
}

const char* answerWord(Answer a) {
    switch (a) {
        case Answer::Done: return "DONE";
        case Answer::FrontFirst: return "FRONT FIRST";
        case Answer::BackFirst: return "BACK FIRST";
        case Answer::Abort: return "ABORT";
    }
    return "?";
}

/// What this program has printed and decided about ONE run. RUN AGAIN replaces it with a
/// value-initialised one, so a field added here later is reset by construction (ruling F2-R7).
struct RunRecord {
    int number = 1;
    Phase lastPhase = Phase::Idle;
    std::uint8_t pulsesPrinted = 0;
    std::uint8_t spinsPrinted = 0;
    std::uint8_t excludedPrinted = 0;
    bool outcomePrinted = false;
    SaveOutcome save = SaveOutcome::NotAttempted;
    bool haveLayout = false;
    std::uint32_t lastLayout = 0;
    // the push's live count, on change
    int lastMoved = -1;
    int lastReady = -1;
    int lastFar = -1;
    double lastPushLogAt = -1.0e9;
    // the loop's timing, while the run is under way
    double lastTickAt = -1.0;
    double worstGap = 0.0;
    std::uint32_t ticks = 0;
};

/// The config the run uses, once — so a capture can be read against the thresholds that
/// produced it (every one INVENTED, HA-136).
void emitConfig(const calibration::CalibratorConfig& c) {
    constexpr double kDeg = calibration::kDegreesToRadians;
    emitf("config: push %.0f in along a tape; a port MOVED past %.1f deg of shaft; the push "
          "straight within %.1f deg and at least %.2f rad (one wheel turn) of median travel",
          c.pushDistance.value(), c.movedThreshold.value() / kDeg, c.straightBound.value() / kDeg,
          c.minPushRad.value());
    char levels[48] = "";
    std::size_t used = 0;
    for (std::size_t k = 0; k < c.pulseLevelCount && k < calibration::kMaxPulseLevels; ++k) {
        const int w = std::snprintf(levels + used, sizeof levels - used, "%s%.1f",
                                    k > 0 ? "/" : "", c.pulseLevels[k].value());
        used += w > 0 ? static_cast<std::size_t>(w) : 0;
        used = used < sizeof levels ? used : sizeof levels - 1;
    }
    emitf("config: pulses ramp %s V for %.2f s each, a side needs %.1f deg of yaw; a port whose "
          "own shaft turns %.0f deg with nothing else moving is LEFT OUT; rest = motors under "
          "%.2f rad/s AND yaw rate under %.1f deg/s for %.1f s",
          levels, c.pulseTime.value(), c.pulseYawFloor.value() / kDeg,
          c.decoupledOwnMin.value() / kDeg, c.restFloor.value(),
          c.restYawRateFloor.value() / kDeg, c.restSettleTime.value());
    emitf("config: spin %.0f turns each way, levels %.1f then %.1f V; must turn %.1f deg within "
          "%.1f s per level; STOPS at %.1f deg the wrong way; %.0f s timeout; the two ways agree "
          "within %.1f pct",
          c.spinTurns, c.spinLevels[0].value(),
          c.spinLevels[c.spinLevelCount > 1 ? 1 : 0].value(), c.spinStartYaw.value() / kDeg,
          c.spinStartTime.value(), c.spinWrongWay.value() / kDeg, c.spinTimeout.value(),
          100.0 * c.spinAgreement);
}

/// The PUSH TABLE, right after DONE was sent — whatever the calibrator made of it (accepted, or
/// refused as too few / curved / too short: the table is then the diagnosis).
void emitPushTable(const DrivetrainCalibrator& cal) {
    const calibration::CalibrationReport& rep = cal.report();
    const calibration::CalibratorConfig& cfg = cal.config();
    emitf("PUSH TABLE at DONE (travel AS READ, before the FRONT/BACK answer; MOVED = past %.1f "
          "deg):",
          cfg.movedThreshold.value() / calibration::kDegreesToRadians);
    for (std::size_t i = 0; i < rep.portCount; ++i) {
        char line[96];
        calibration::formatPushRecord(rep.ports[i], line, sizeof line);
        emitf("  %s", line);
    }
    emitf("push: %u of %u candidates moved, drift %+.2f deg (bound %.2f), median travel %.2f rad "
          "(need %.2f)",
          static_cast<unsigned>(rep.drivePortCount), static_cast<unsigned>(rep.portCount),
          rep.pushStraightnessRad / calibration::kDegreesToRadians + 0.0,
          cfg.straightBound.value() / calibration::kDegreesToRadians,
          cal.status().pushTravelRad, cfg.minPushRad.value());
}

/// The front answer, and every drive port with the sign it now carries.
void emitFront(const DrivetrainCalibrator& cal, Answer a) {
    const calibration::CalibrationReport& rep = cal.report();
    emitf("front: %s -- %s", answerWord(a),
          a == Answer::BackFirst ? "every captured sign INVERTED" : "the signs stand as captured");
    char ports[hal::kMaxSmartPorts * 5 + 1] = "";
    std::size_t used = 0;
    for (std::size_t i = 0; i < rep.portCount; ++i) {
        if (rep.ports[i].sign == 0) {
            continue;
        }
        const int w = std::snprintf(ports + used, sizeof ports - used, " %+d",
                                    static_cast<int>(rep.ports[i].sign * rep.ports[i].port));
        used += w > 0 ? static_cast<std::size_t>(w) : 0;
        used = used < sizeof ports ? used : sizeof ports - 1;
    }
    emitf("drive ports, signed:%s  (%u to pulse, one at a time)", ports,
          static_cast<unsigned>(rep.drivePortCount));
}

/// One side of the result as a signed list: " -11 +12 -13".
void signedList(const hal::DriveSideDescription& s, char* out, std::size_t cap) {
    out[0] = '\0';
    std::size_t used = 0;
    for (std::size_t i = 0; i < s.count; ++i) {
        const int w = std::snprintf(out + used, cap - used, " %+d",
                                    static_cast<int>(s.signedPorts[i]));
        used += w > 0 ? static_cast<std::size_t>(w) : 0;
        used = used < cap ? used : cap - 1;
    }
}

void emitLoopStats(const RunRecord& run) {
    emitf("loop: %lu ticks, worst gap between two ticks %.0f ms (a powered step stops past "
          "100 ms); %lu screen prints so far",
          static_cast<unsigned long>(run.ticks), 1000.0 * run.worstGap,
          static_cast<unsigned long>(g_screenPrints));
}

/// Done: the MEASURED line and everything behind it, then the save — write, flush, re-read,
/// compare — whose outcome the panel is told. Called once per run, BEFORE the finished screen is
/// first painted, so it never shows "MEASURED" and then changes its mind.
SaveOutcome finishDone(const DrivetrainCalibrator& cal, const RunRecord& run) {
    const hal::DrivetrainDescription& d = cal.result();
    const calibration::CalibrationReport& rep = cal.report();
    char described[128];
    d.describe(described, sizeof described);
    emit(described);  // "MEASURED tank BLUE 600 rpm: L5/R4 ports, ipr L... R..., W ... in"
    emitf("provenance: %s", d.provenance);
    char list[hal::kMaxSmartPorts * 5 + 1];
    signedList(d.left, list, sizeof list);
    emitf("  LEFT  (signed, AS MEASURED):%s  (%u ports)", list,
          static_cast<unsigned>(d.left.count));
    signedList(d.right, list, sizeof list);
    emitf("  RIGHT (signed, AS MEASURED):%s  (%u ports)", list,
          static_cast<unsigned>(d.right.count));
    if (rep.excludedCount == 0) {
        emit("  LEFT OUT: none");
    } else {
        char out[hal::kMaxSmartPorts * 4 + 1] = "";
        std::size_t used = 0;
        for (std::size_t k = 0; k < rep.excludedCount; ++k) {
            const int w = std::snprintf(out + used, sizeof out - used, " %d",
                                        static_cast<int>(rep.excludedPorts[k]));
            used += w > 0 ? static_cast<std::size_t>(w) : 0;
            used = used < sizeof out ? used : sizeof out - 1;
        }
        emitf("  LEFT OUT:%s -- in no side of the description (%s)", out, cal.warning());
    }
    emitf("  inches per radian: LEFT %.6f  RIGHT %.6f  (L/R %.4f)", d.left.inchesPerRadian.value(),
          d.right.inchesPerRadian.value(),
          d.right.inchesPerRadian.value() > 0.0
              ? d.left.inchesPerRadian.value() / d.right.inchesPerRadian.value()
              : 0.0);
    emitf("  effective track width: %.4f in (LARGER than a tape's -- a turning tank scrubs; "
          "HA-135)",
          d.trackWidth.value());
    emitf("  push: %.0f in, drift %+.2f deg, median travel LEFT %.4f rad RIGHT %.4f rad, spread "
          "LEFT %.4f rad RIGHT %.4f rad (a short member shows as spread -- HA-130)",
          rep.pushDistanceIn, rep.pushStraightnessRad / calibration::kDegreesToRadians + 0.0,
          rep.leftMedianRad, rep.rightMedianRad, rep.leftSpreadRad, rep.rightSpreadRad);
    emitf("  spin widths: CCW %.4f in, CW %.4f in, disagreement %.2f pct", rep.widthCcwIn,
          rep.widthCwIn, 100.0 * rep.widthDisagreement);

    SaveOutcome outcome = SaveOutcome::NotSaved;
    char why[192];
    char saveWhy[160] = "";
    if (!d.valid(why, sizeof why)) {
        // A description that cannot boot a robot is not worth writing (ruling P-R4's other half):
        // the format's decoder would refuse it anyway, and a file on the card that refuses at
        // every boot is worse than no file.
        std::snprintf(saveWhy, sizeof saveWhy, "the measured description is not valid: %.120s",
                      why);
    } else if (saveAndReadBack(d, saveWhy, sizeof saveWhy)) {
        outcome = SaveOutcome::Saved;
    }
    if (outcome == SaveOutcome::Saved) {
        emitf("SAVED -- /usd/%s: written, flushed, RE-READ and compared byte for byte (%d bytes). "
              "shulib Teleop (slot 2) will boot from it.",
              calibration::kRecordFileName, static_cast<int>(calibration::kRecordBytes));
        emit("COPY THE 'MEASURED tank ...' LINE ABOVE DOWN. Then touch RUN AGAIN and check");
        emit("the two runs agree -- a number that does not reproduce is not a measurement");
        emit("(the tolerances, 2 pct for a scale and 3 pct for the width, are HA-136, invented).");
    } else {
        emitf("NOT SAVED -- %s", saveWhy);
        emit("THE RESULT IS ON THE SCREEN AND IN THIS LOG ONLY. Copy it down NOW. (A record saved");
        emit("by an EARLIER run, if any, is still on the card -- this run did not replace it.)");
    }
    emitLoopStats(run);
    return outcome;
}

/// Aborted: the refusal's code, its port, and its sentence.
void finishStopped(const DrivetrainCalibrator& cal, const RunRecord& run) {
    if (cal.refusalPort() > 0) {
        emitf("STOPPED [%s port %d]: %s", calibration::refusalName(cal.refusal()),
              cal.refusalPort(), cal.reason());
    } else {
        emitf("STOPPED [%s]: %s", calibration::refusalName(cal.refusal()), cal.reason());
    }
    emit("Every motor is at 0 V and coasting. Nothing was written to the card by this run.");
    emitLoopStats(run);
}

/// Everything the serial record owes after a tick or an answer: a phase change, each new pulse,
/// each port newly LEFT OUT, each finished spin direction, the push's live count, and — once — the
/// outcome (the save included).
void record(const DrivetrainCalibrator& cal, RunRecord& run, double now) {
    const calibration::CalibrationReport& rep = cal.report();
    const calibration::CalibratorStatus& st = cal.status();
    while (run.pulsesPrinted < rep.pulseCount) {
        char line[160];
        calibration::formatPulseRecord(rep.pulses[run.pulsesPrinted],
                                       static_cast<unsigned>(rep.drivePortCount), line,
                                       sizeof line);
        emit(line);
        ++run.pulsesPrinted;
    }
    while (run.excludedPrinted < rep.excludedCount) {
        emitf("LEFT OUT: port %d turns but does not move the robot (a slipping gear?) -- it is in "
              "no side and is never commanded again",
              static_cast<int>(rep.excludedPorts[run.excludedPrinted]));
        ++run.excludedPrinted;
    }
    while (run.spinsPrinted < rep.spinCount) {
        char line[160];
        calibration::formatSpinRecord(rep.spins[run.spinsPrinted], line, sizeof line);
        emit(line);
        ++run.spinsPrinted;
    }
    if (cal.phase() != run.lastPhase) {
        run.lastPhase = cal.phase();
        if (!finished(cal.phase())) {  // an ending is the outcome block's to say, below
            emitf("phase: %s | %s", phaseWord(cal.phase()), st.progress);
        }
    }
    if (cal.phase() == Phase::PushCapture && cal.accepts(Answer::Done)) {
        const int moved = st.movedNow;
        const int ready = cal.pushReady() ? 1 : 0;
        const int far = st.pushFarEnough ? 1 : 0;
        const bool changed = moved != run.lastMoved || ready != run.lastReady || far != run.lastFar;
        if ((changed && now - run.lastPushLogAt >= kPushLogMinSeconds)
            || now - run.lastPushLogAt >= kPushLogKeepaliveSeconds) {
            run.lastMoved = moved;
            run.lastReady = ready;
            run.lastFar = far;
            run.lastPushLogAt = now;
            emitf("push live: %s", st.progress);
        }
    }
    if (finished(cal.phase()) && !run.outcomePrinted) {
        run.outcomePrinted = true;
        emitf("===== RUN %d ENDED: %s =====", run.number,
              cal.phase() == Phase::Done ? "DONE" : "STOPPED");
        if (cal.phase() == Phase::Done) {
            run.save = finishDone(cal, run);
        } else {
            finishStopped(cal, run);
        }
    }
}

/// A `screen:` line whenever the LAYOUT changes — the header word, the first three lines and the
/// buttons — so a capture says what the operator was looking at when each touch landed.
void recordLayout(const PanelView& v, RunRecord& run) {
    const std::uint32_t key = calibration::layoutKey(v);
    if (run.haveLayout && key == run.lastLayout) {
        return;
    }
    run.haveLayout = true;
    run.lastLayout = key;
    char words[200] = "";
    std::size_t used = 0;
    for (std::size_t i = 0; i < v.lineCount && i < 3; ++i) {
        const int w = std::snprintf(words + used, sizeof words - used, "%s%s", i > 0 ? " / " : "",
                                    v.lines[i].text);
        used += w > 0 ? static_cast<std::size_t>(w) : 0;
        used = used < sizeof words ? used : sizeof words - 1;
    }
    char buttons[64] = "";
    used = 0;
    for (std::size_t b = 0; b < v.buttonCount; ++b) {
        const int w = std::snprintf(buttons + used, sizeof buttons - used, " [%s]",
                                    buttonWord(v.buttons[b].id));
        used += w > 0 ? static_cast<std::size_t>(w) : 0;
        used = used < sizeof buttons ? used : sizeof buttons - 1;
    }
    emitf("screen: %s: %s |%s", v.headerWord, words, buttons[0] != '\0' ? buttons : " no buttons");
}

/// One read of the touch screen, as TouchGate wants it.
TouchSample readTouch() {
    const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
    TouchSample s;
    s.error = t.touch_status == pros::E_TOUCH_ERROR;
    s.releaseCount = t.release_count;
    s.x = t.x;
    s.y = t.y;
    return s;
}

/// Why a new release did nothing, for the log: it landed on a button still greyed (arming), on a
/// button the calibrator no longer accepted, or on nothing.
const char* missWord(const PanelView& v, const TouchSample& s, const TouchResult& r) {
    if (r.hit >= 0) {
        return "ignored -- the calibrator no longer accepts it";
    }
    for (std::size_t b = 0; b < v.buttonCount; ++b) {
        if (!v.buttons[b].armed
            && calibration::contains(calibration::hitBox(v.buttons[b].rect), s.x, s.y)) {
            return "ignored -- the button was still GREYED (arming)";
        }
    }
    return "ignored -- no button there";
}

}  // namespace

void calibrateInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    // Moved VERBATIM from src/main.cpp's initialize() at G1 Part 2c (the dispatcher forwards
    // here); the bytes on serial are unchanged, except that the BUILD STAMP line now prints THIS
    // file's kBuildStamp — the same __DATE__ " " __TIME__ the banner printed, from main.cpp's own
    // copy (kCalibrateBuildStamp) until the move. One stamp for the program now, where main.cpp's
    // comment said a partial rebuild could show two.
    std::printf("\n[R3D] booted: shulib CALIBRATE build [%s].\n"
                "[R3D] robot two MEASURES ITS OWN DRIVETRAIN -- which ports drive, which way each\n"
                "[R3D] is wired, which side each is on, inches per radian per side, and the\n"
                "[R3D] EFFECTIVE track width -- then writes them to /usd/%s.\n"
                "[R3D] It POWERS THE MOTORS. THIS VERSION (R3d Part 2F) first ran on robot two on\n"
                "[R3D] 2026-09-24 -- run 1 STOPPED on SPIN_DISAGREE, run 2 finished (unrecorded);\n"
                "[R3D] the Part 2 version finished none of five starts. Every threshold in it is\n"
                "[R3D] INVENTED (A4 register HA-136).\n"
                "[R3D] no object graph is constructed at boot; the X-drive wiring is NOT built.\n"
                "[R3D] BUILD STAMP %s\n",
                variantBeacon, calibration::kRecordFileName, kBuildStamp);
    std::printf("[R3D] competition status at boot: %s -- calibration runs only in DRIVER "
                "CONTROL, enabled; any other state stops the run and says so.\n",
                competitionStatusBits);
    std::fflush(stdout);
}

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
    emit("#  It POWERS THE MOTORS. This build (Part 2F) first ran on robot two on");
    emit("#  2026-09-24: run 1 STOPPED on SPIN_DISAGREE, run 2 finished (unrecorded).");
    emit("#  (The Part 2 program ran five starts that morning and finished none.)");
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
    paintStarting();

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
            // ports[] is the ONE candidate list until step 4 builds from it (T3: the filter below
            // rewrites it in place, so a second copy here could only go stale).
            ports[n] = static_cast<std::int8_t>(p);  // POSITIVE: the calibrator applies the sign
            ++n;
        }
    }
    emitf("  %u motor(s), %d IMU(s)", static_cast<unsigned>(n), imuCount);
    {
        // A DECLARED MECHANISM PORT IS NEVER A DRIVE CANDIDATE (T3): src/mechanism_table.hpp
        // declares robot two's claw pivots on 6 and 7. They leave the list HERE, before step 4
        // builds any ProsMotor, so a claw is never geared BLUE, never set to Coast (a loaded claw
        // would drop) and never pulsed. One line per port says why the census's motor count and
        // the candidate count differ. "claw pivot" is the kind of every entry declared today.
        // It drops PORTS: a claw cable swapped with a drive cable at the brain is invisible to the
        // registry, and to this (A4 register HA-155) — a label on the cable is the only guard.
        // The re-listing is the table's pure function, run by the host suite; this block's text is
        // pinned by test/mechanism_table_test.cpp — change the two together.
        DroppedMechanismPorts dropped{};
        n = dropDeclaredMechanismPorts(kMechanisms, {ports, n}, dropped);
        for (std::size_t i = 0; i < dropped.count; ++i) {
            emitf("port %d: claw pivot (declared) -- not a candidate, not touched",
                  static_cast<int>(dropped.ports[i]));
        }
    }

    // ── 3. the three human facts, and the refusals that protect them ─────────────────
    if (kChassis.cartridge == Cartridge::Unset) {
        paintRefusal(text::kCartridgeUnset,
                     "the cartridge cannot be measured from inside; read the insert colour off a "
                     "motor",
                     "type it into src/chassis_table.hpp, rebuild, upload");
        return;
    }
    emitf("CARTRIDGE BELIEF: %s -- stated by a human and UNVERIFIABLE from inside (every "
          "cartridge spins the rotor the same)", cartridgeWord(kChassis.cartridge));
    if (imuCount != 1) {
        char why[140];
        std::snprintf(why, sizeof why,
                      "the registry reports %d IMUs; this procedure needs exactly one", imuCount);
        paintRefusal(text::kImuCount, why,
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
                      "the registry found %u motor(s) besides the declared mechanisms; a "
                      "drivetrain needs at least two",
                      static_cast<unsigned>(n));
        paintRefusal(text::kTooFewMotors, why,
                     "re-seat the drive cables and check the census (Bench Tests, slot 3)");
        return;
    }

    // ── 4. one ProsMotor per motor port, POSITIVE, each in its own try ───────────────
    // The registry said MOTOR; the adapter can still refuse (its constructor writes the gearset
    // and reads it back, HA-98). A port that refuses is dropped from the candidate list with a
    // warning, because a calibration of nine motors is still a calibration — and it is named,
    // because a silently missing motor is how a side comes up short.
    const hal::pros::MotorGearset gearset = gearsetOf(kChassis.cartridge);
    std::size_t built = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const int port = ports[i];  // the census's list, after the declared-mechanism drop
        try {
            candidates[built].port = port;
            ports[built] = static_cast<std::int8_t>(port);
            candidates[built].motor.emplace(ports[built], gearset);
            ++built;
        } catch (const PreconditionError& e) {
            emitf("WARNING: port %d answered MOTOR in the registry but its adapter REFUSED: "
                  "%s -- dropped from the candidates",
                  port, e.what());
        }
    }
    n = built;
    if (n < 2) {
        paintRefusal(text::kMotorsRefused,
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
        imu->calibrate();  // ONCE, here (HA-05); every run's calibrator waits on isReady()
    } catch (const PreconditionError& e) {
        paintRefusal(text::kImuRefused, e.what(),
                     "check the sensor and its cable (Bench Tests, slot 3)");
        return;
    }
    emit("IMU calibrating (HA-23 says ~2 s) -- keep the robot STILL until the screen says to "
         "push it");

    char provenance[96];
    std::snprintf(provenance, sizeof provenance, "shulib Calibrate %s", kBuildStamp);
    calibration::HumanFacts facts{};
    facts.kind = hal::DrivetrainKind::Tank;
    facts.cartridge = cartridgeOf(kChassis.cartridge);
    facts.imuPort = static_cast<std::uint8_t>(imuPort);
    facts.provenance = provenance;

    const std::span<hal::IMotor* const> motorSpan{motorPtrs, n};
    const std::span<const std::int8_t> portSpan{ports, n};
    std::optional<DrivetrainCalibrator> cal;
    cal.emplace(motorSpan, portSpan, *imu, clock);
    cal->start(facts);
    emitf("calibrator started: %u candidate(s), kind Tank, cartridge %s, IMU port %d, "
          "provenance \"%s\"",
          static_cast<unsigned>(n), cartridgeWord(kChassis.cartridge), imuPort, provenance);
    emitConfig(cal->config());
    emit("ONE PUSH, 48 in, forward, along a tape; then FRONT or BACK; then STAND CLEAR: one");
    emit("motor at a time jerks (3, then 6, then 9 V), then it spins both ways. INVENTED numbers.");

    // ── 6. the touch baseline, from a GOOD read (ruling F2-R6) ───────────────────────
    std::int32_t touchBase = 0;
    bool touchBaseKnown = false;
    for (int i = 0; i < 20 && !touchBaseKnown; ++i) {
        const TouchSample s = readTouch();
        if (!s.error) {
            touchBase = s.releaseCount;
            touchBaseKnown = true;
        } else {
            pros::delay(5);
        }
    }
    if (!touchBaseKnown) {
        emit("WARNING: every touch read at start was an ERROR read -- the first good read will "
             "set the baseline and is never acted on");
    }
    std::optional<TouchGate> gate;
    gate.emplace(touchBase);

    // ── 7. the loop ──────────────────────────────────────────────────────────────────
    hal::pros::ProsTickPacer pacer{};
    PanelPainter painter;
    RunRecord run{};
    int runNumber = 1;
    bool stoppedByField = false;
    emit("===== RUN 1 =====");

    try {
        while (true) {
            // A field DISABLE (or an autonomous period) exits this loop cleanly so the guard runs
            // before PROS kills the task. Polled every tick: 10 ms is the most the robot can drive
            // past it. A finished run is not aborted (abort() from Done would discard the result,
            // and nothing is powered) — the loop simply ends (ruling F2-R9).
            if (pros::c::competition_is_disabled()
                || (pros::c::competition_get_status() & COMPETITION_AUTONOMOUS) != 0) {
                if (!finished(cal->phase())) {
                    cal->abort("the field disabled the robot, or autonomous started");
                    emitf("===== RUN %d ENDED: STOPPED =====", run.number);
                    finishStopped(*cal, run);
                } else {
                    emit("the field disabled the robot after the run had finished -- nothing was "
                         "powered");
                }
                stoppedByField = true;
                break;
            }

            const double now = clock.now().value();
            if (!finished(cal->phase())) {
                if (run.lastTickAt >= 0.0 && now - run.lastTickAt > run.worstGap) {
                    run.worstGap = now - run.lastTickAt;
                }
                run.lastTickAt = now;
                ++run.ticks;
            }
            cal->tick();
            record(*cal, run, now);  // the serial record — and, at Done, the save

            // ── paint the view, then read the touch against THAT view ──
            PanelView drawn = gate->view(*cal, clock.now().value(), run.save);
            painter.render(drawn, now);
            recordLayout(drawn, run);

            const TouchSample sample = readTouch();
            if (!touchBaseKnown) {
                if (!sample.error) {
                    gate.emplace(sample.releaseCount);  // the baseline; this read is not acted on
                    touchBaseKnown = true;
                }
            } else {
                const TouchResult r = gate->touch(sample, drawn, *cal);
                if (r.newRelease) {
                    const char* on =
                        r.hit >= 0 ? buttonWord(drawn.buttons[static_cast<std::size_t>(r.hit)].id)
                                   : "nothing";
                    if (r.action == TouchAction::Answer) {
                        emitf("touch: x=%d y=%d release=%ld on %s -> ANSWER %s",
                              static_cast<int>(sample.x), static_cast<int>(sample.y),
                              static_cast<long>(sample.releaseCount), on, answerWord(r.answer));
                    } else if (r.action == TouchAction::RunAgain) {
                        emitf("touch: x=%d y=%d release=%ld on %s -> RUN AGAIN",
                              static_cast<int>(sample.x), static_cast<int>(sample.y),
                              static_cast<long>(sample.releaseCount), on);
                    } else {
                        emitf("touch: x=%d y=%d release=%ld on %s -> %s",
                              static_cast<int>(sample.x), static_cast<int>(sample.y),
                              static_cast<long>(sample.releaseCount), on,
                              missWord(drawn, sample, r));
                    }
                }
                if (r.action == TouchAction::Answer) {
                    cal->answer(r.answer);
                    if (r.answer == Answer::Done) {
                        emitPushTable(*cal);
                    } else if (r.answer == Answer::FrontFirst || r.answer == Answer::BackFirst) {
                        emitFront(*cal, r.answer);
                    }
                    record(*cal, run, now);  // a refusal at DONE, or the abort: its outcome now
                    // The step changed under the operator's finger: the new view goes up NOW,
                    // not on the next tick — its new buttons greyed for the arming delay.
                    drawn = gate->view(*cal, clock.now().value(), run.save);
                    painter.render(drawn, now);
                    recordLayout(drawn, run);
                } else if (r.action == TouchAction::RunAgain) {
                    // RUN AGAIN (ruling F2-R7): a NEW calibrator in place, a NEW gate based on the
                    // release that asked for it, a NEW per-run record. Nothing of the last run
                    // survives; the IMU is already calibrated, so the IMU step passes at once.
                    ++runNumber;
                    emit("");
                    emitf("===== RUN %d -- RUN AGAIN: a NEW calibrator, nothing carried over "
                          "=====",
                          runNumber);
                    stopAll(candidates, n);
                    cal.emplace(motorSpan, portSpan, *imu, clock);
                    cal->start(facts);
                    gate.emplace(sample.releaseCount);
                    run = RunRecord{};
                    run.number = runNumber;
                    painter.invalidate();
                }
            }
            pacer.pace();
        }
    } catch (const PreconditionError& e) {
        // Ruling F2-R8: a broken contract inside the loop is a program bug. Stop everything, say
        // so on the screen and the log, and idle — never a frozen screen over a running motor.
        stopAll(candidates, n);
        emitf("PROGRAM FAULT (a precondition broke inside the loop): %s", e.what());
        emit("every motor is at 0 V, coasting. This is a bug in the program, not the robot.");
        paintCannot(text::kHeaderStopped, PanelTone::Red, text::kProgramFault, e.what(),
                    "a bug in the program, not the robot: keep the serial log", text::kFaultIdle);
        while (!pros::c::competition_is_disabled()) {
            pros::delay(250);
        }
        return;
    }

    // ── 8. the field ended it ────────────────────────────────────────────────────────
    stopAll(candidates, n);
    if (stoppedByField) {
        paintNotDriverControl();
        emit("STOPPED by the field/competition state -- every motor at 0 V, coast.");
    }
}

}  // namespace shulib::bench
