// R3a — the bench validation entry point for the MEASURED tank robot.
//
// ═══ WHAT THIS IS ══════════════════════════════════════════════════════════════════
// A measuring instrument, not a configured robot. It answers "what is actually
// plugged into this brain, and does our conversion layer agree with the raw
// device?" — chunk R3a §4.2 item 1. No auton, no closed loop, and (see below)
// no motion of any kind.
//
// ═══ READ-ONLY, WITH TWO POWERED STATIONS — A RULING, AMENDED TWICE ══════════════════
// R3a's brief says the entry point "commands open-loop voltages on request".
// THE R3a BUILD DID NOT. The one thing open-loop voltage buys is identifying which
// port drives which wheel and in which direction — and that is obtainable with
// ZERO risk by turning a wheel BY HAND and watching the encoder move (bench
// worksheet Station 2). Powering 8 motors whose signs are unmeasured, on a robot
// whose port map is the thing under test, buys nothing the hand method does not
// and can lurch a 15-lb robot off a bench.
//   * The ruling said: "If open-loop commands are wanted later, they belong
//     behind an explicit opt-in with the wheels off the ground, and they are a
//     separate change." THAT CHANGE IS R3b SESSION 2 (2026-09-10): station 10
//     DRIVE is the ONE code path in this binary that calls setVoltage, and it
//     sits behind six safety gates (its own header below). Every other station
//     is still read-only, constructs no motor adapter, and never will -- but ONE:
//   * Station 13 MECHANISMS (the mechanism core's MECH-A6, 2026-10-30; the team lead's go of
//     2026-10-08 on the MECH brief, its Q30: "the bench page and the direction check in the
//     core") powers the robot's DECLARED mechanisms -- each cascade robot's cascade, side toggles
//     and pistons (robot two's claws until ROB1-BT) -- one at a time, through the library's robot::MechanismParts, behind its own six
//     gates, and runs the direction check. Its logic is PROS-free in bench_r3a.hpp
//     (mech_station::Station) and host-tested; its header below says the rest.
//   * It drives the robot through the library's ADAPTERS (hal/pros ProsMotor and
//     ProsController), never its motion stack -- the library has still not
//     driven a robot, and this station is labelled that way on every screen.
//   * The drivetrain it was written for is a SECOND robot: the season's tank
//     chassis, five COUPLED motors per side. Its chassis table (src/robots/chassis_table.hpp
//     since R3b Part 0b, shared with the old drive program until T4 Part 3) shipped UNSET until measured
//     (no port is ever invented -- HA-111 was that defect) and now carries the
//     measured ports AND signs; the bench bot keeps its measured, unsigned table.
//   * Station 12 AI VISION (R2a, 2026-09-24) WRITES THE CAMERA'S OWN SETTINGS -- tag and
//     object detection on, the USB bounding-box overlay on, a tag family when FAMILY is
//     pressed -- and says so on its screen and in the log. It powers nothing. Station 11
//     DISTANCE writes nothing at all.
//
// ═══ STATIONS 11 DISTANCE AND 12 AI VISION (R2a, 2026-09-24) ═══════════════════════
// The team lead's question: "how do we know the distance sensors and the ai visions sensors
// are working? is there logging or can I see the camera directly like what it sees?" These two
// stations SHOW what the sensors report -- raw PROS values, the distance adapter's canonical
// values beside them, the camera's detections -- and a RECORD button writes one `[R3A]` line
// per sensor / detection to serial and the SD log. They INTERPRET NOTHING (no bearing, no
// range, no frame): the AI Vision adapter (R2b) is written afterwards from these records, not
// from beliefs. Ports are DISCOVERED from the registry at station entry, never typed. The
// camera's own USB-C port streams video to VEX's AI Vision Utility; station 12 turns the
// bounding-box overlay on for that stream, so the video outlines what the station lists.
// Their buttons (and the menu's) go through the PROS-free geometry in bench_r3a.hpp, which the
// host suite tests; their text goes through screenText(), the one sanitising path.
//
// ═══ WHY IT PROBES BEFORE IT CONSTRUCTS ════════════════════════════════════════════
// Every hal/pros adapter ctor does a device read-back and raises a precondition
// when the port disagrees (motor.hpp, imu.hpp). That is right for a competition
// binary and WRONG for a discovery binary: the port map is precisely what is
// unknown, so constructing adapters first means one wrong port kills the session
// before it prints anything. So:
//   Stage 1  RAW registry census of all 21 ports. No adapter. Cannot fail.
//   Stage 2+ adapters constructed ONLY for what stage 1 found, each inside a
//            try/catch so a surprise reports itself instead of ending the run.
//
// ═══ THE SIDE LABEL IS LOAD-BEARING (R3a-PROGRESS §10.3) ═══════════════════════════
// A prior robot's source had LEFT and RIGHT SWAPPED, and its auton was tuned by
// driving it until it looked right — so the tuning absorbed the swap invisibly.
// shulib's TankKinematics fixes wheel order 0 = LEFT, 1 = RIGHT. This binary
// therefore prints the SIDE LABEL beside every port so a mirror is visible on
// screen rather than inferred later from a robot turning the wrong way.
// THE PORT GROUPS BELOW ARE A HYPOTHESIS TO BE FALSIFIED, not a configuration.
//
// ═══ OUTPUT GOES TWO PLACES ════════════════════════════════════════════════════════
// USB serial (readable live over `pros terminal`) AND, when a card is present, a
// plain-text file on the SD card. NOTE: this is NOT the E1 blackbox — the
// blackbox format v1 deliberately does not carry the log() message channel, so a
// text census cannot ride in it. This writes text bytes through the same
// ProsBlockSink device seam instead, which is exactly what that seam is for.

#include "programs/bench_r3a.hpp"
#include "programs/imu_timing.hpp"
#include "robots/chassis_table.hpp"
#include "robots/mechanism_table.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <optional>
#include <span>
#include <string_view>

#include "pros/ai_vision.h"
#include "pros/apix.h"
#include "pros/distance.h"
#include "pros/error.h"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/distance_conversion.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/battery.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/char_sink.hpp"
#include "shulib/hal/pros/clock.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/distance.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/hal/pros/line_display.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/hal/motor_conversion.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

// (The wiring hypothesis that lived here -- an IMU port and two port arrays --
//  became the per-variant CHASSIS TABLE below, at R3b Session 2. Same bench numbers;
//  now with an UNSET state, a provenance string, and a second robot.)

/// Compiled-in build stamp. __DATE__/__TIME__ are evaluated when THIS translation
/// unit is compiled, so the value changes on every rebuild -- which is the point.
/// The git hash (build_info.hpp) identifies the COMMIT; this identifies the BUILD,
/// and only the second one answers "did my upload actually land?" A silent upload
/// failure on 2026-08-18 left an old binary running and cost a debugging cycle
/// because nothing on screen distinguished the two.
constexpr const char* kBuildStamp = __DATE__ " " __TIME__;

// (kMaxPort -- physical smart ports 1..21, how found[] is indexed -- now comes from
//  chassis_table.hpp with the table.)

/// THE REGISTRY IS ZERO-INDEXED AND NOTHING ELSE IN PROS IS.
/// `apix.h`: "Returns the type of the device plugged into the ZERO-INDEXED port …
/// The V5 port number from 0-20". So registry index i is PHYSICAL PORT i+1.
///
/// Scanning 1..21 as if it were physical -- which this file did on 2026-08-18 --
/// shifts every reading by one port and invents a device at index 21 (physical 22,
/// which does not exist). It produced a confident, completely wrong port map, and
/// HA-120 had already predicted exactly this: it calls for "a CORRECTED 0-20
/// registry scan" precisely because the 2026-08-13 expander sighting came from
/// index 21. Do not re-derive this; the mapping is index -> index + 1.
constexpr int kMaxRegistryIndex = 20;
constexpr int kLoopSamples = 200; // 200 × 10 ms ≈ 2 s of cadence measurement

// ═══ THE CHASSIS TABLE — the ONE typed input (R3b Session 2, brief §3.1) ═══════════
// Lives in src/robots/chassis_table.hpp since R3b Part 0b, shared then with the old "shulib Drive" so
// the tester and that program could not disagree about which robot they were on (T4 Part 3 replaced
// it with a program that boots from the calibration record). It
// carries robot two's MEASURED SIGNS (signsMeasured = true); the bench bot's stays
// unsigned. The helpers below are this file's thin bindings of the header's pure
// functions to kChassis.

/// The side label printed beside a port -- §10.3's mirror check, now from the table.
const char* tableSide(int port) {
    const int s = tableSideOf(kChassis, port);
    if (s < 0) return "LEFT(table)";
    if (s > 0) return "RIGHT(table)";
    if (kChassis.imuPort != 0 && port == static_cast<int>(kChassis.imuPort)) return "IMU(table)";
    return "";
}

/// One letter for a live panel cell: L / R / ? (the table has no claim on this port).
const char* sideLetter(int port) {
    const int s = tableSideOf(kChassis, port);
    return s < 0 ? "L" : s > 0 ? "R" : "?";
}

// ── Output: USB serial always, SD card when one is installed. ─────────────────
hal::pros::ProsBlockSink* g_card = nullptr;

void screenEmit(const char* line);  // fwd: defined with the screen helpers below

void emit(const char* line) {
    // ONE write() per line (SER1, HA-189). This was `std::printf("%s\n", line)`, which the compiler
    // rewrites to `puts` — and newlib's puts sends the text and the newline as TWO writes into
    // stdout's shared, unguarded buffer. serialWrapped(nullptr, ...) sends the line exactly as
    // printed (no tag: the line carries its own), whole, and wraps a line past 255 characters.
    hal::pros::serialWrapped(nullptr, line);
    screenEmit(line);
    if (g_card != nullptr && g_card->isOpen()) {
        // Return value deliberately discarded: a full/absent card must not stop a
        // bench session, and isOpen() already reported the card's state up front.
        (void)g_card->write(std::as_bytes(std::span<const char>{line, std::strlen(line)}));
        const char nl = '\n';
        (void)g_card->write(std::as_bytes(std::span<const char>{&nl, 1}));
    }
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

// ═══ BRAIN SCREEN ══════════════════════════════════════════════════════════════
// The bencher runs this ALONE, with no laptop attached -- so the serial stream is
// invisible to them and the screen is the only live output they have. Every test
// therefore mirrors its lines to the screen; the SD file keeps the full-width
// detail (screen lines truncate at 54 chars) for reading afterwards.

/// MEASURED, not guessed: `liblvgl/lv_conf.h` sets LV_HOR_RES_MAX 480 and
/// LV_VER_RES_MAX **240**. The physical panel is 480x272; VEXos keeps the top strip
/// for its own status bar, and 240 is what a user program actually gets. An earlier
/// layout was verified on the host against 272, PASSED, and rendered its bottom row
/// off the panel -- the check was right and its constant was wrong.
constexpr std::int16_t kUsableW = 480;
constexpr std::int16_t kUsableH = 240;

// ── the screen's own layout. Content sits BELOW a fixed header so a section title
//    is always visible while results scroll under it.
constexpr std::int16_t kHeaderH = 34;
constexpr std::int16_t kFooterY = 222;
constexpr std::int16_t kContentY = kHeaderH + 6;
constexpr std::int16_t kLineH = 13;   // small-font row pitch; test 0 measures the truth
static_assert(kLineH == menu_grid::kSmallPitch, "the menu's SMALL second line is laid out at this pitch");
constexpr int kScreenLines = (kFooterY - kContentY) / kLineH;

// Palette. Dark ground, one accent, one warn, one good -- enough to read at a
// glance from arm's length over a robot, and no more.
constexpr std::uint32_t kColBg = 0x101418, kColBar = 0x1E5AA8, kColText = 0xF0F0F0,
                        kColDim = 0x9AA4AE, kColGood = 0x39C36E, kColWarn = 0xE8B23A,
                        kColBad = 0xE2564A, kColSub = 0xC8D8F0, kColEdge = 0x6E9AD8;

/// Truncation width for the panel. The SD file keeps full width, so an over-long
/// line loses nothing that matters; an unreadable overflow would.
constexpr int kScreenCols = 54;

// ═══ THE ONE SANITISING PATH TO THE PANEL (R2a, 2026-09-24) ══════════════════════════
// PROS hands already-formatted text to VEXos AS A FORMAT STRING (R3d Part 2F-2's finding,
// src/programs/calibrate_program.cpp's header), so a '%' that reaches the panel is re-read as a
// conversion with no argument behind it; and VEXos's fonts carry ASCII only. screenEmit() used
// to print whatever line it was handed through "%s" -- and station 10's own threshold line
// ("<25% of side max") and every adapter refusal (e.what(), with UTF-8 em dashes) reach it.
// So text now goes through panelSafe(): '%' and control bytes become '?', a UTF-8 em/en dash
// becomes '-', any other non-ASCII character becomes ONE '?', and the copy stops at the
// budget. screenText() then prints it through a literal "%s" -- the same rules as
// src/common/screen_text.hpp's printText(), the shared path "shulib Calibrate" and the programs that
// boot from the record use since G1 Part 2b. The tester deliberately keeps its OWN copy: it cuts at this file's 54-column
// scrolling-log width (kScreenCols), not the SMALL budget printText() cuts at, and counts its
// prints in its own g_screenPrints -- and those widths ran on hardware as they are (R2a-R3), so
// moving it onto the shared header would change what ran.
// The prints this file makes with a FIXED literal and numbers only (the ruler, the touch
// readout, the splash, MOTOR WATCH / DRIVE cells) cannot carry a '%' or a non-ASCII byte to
// the panel and are left as they ran on hardware (R2a-PROGRESS, ruling R2a-R3).

/// The widest text this file puts on one panel line: the scrolling log's kScreenCols.
constexpr std::size_t kPanelCap = static_cast<std::size_t>(kScreenCols) + 1;

/// screenText() calls since boot. PROS 4.2.2's print_at LEAKS its formatted buffer per call
/// (R3d Part 2F-2), so the live stations print only on change and report this count.
std::uint32_t g_screenPrints = 0;

/// `text` as it may appear on the panel, into `out` (at most min(budget, cap - 1) characters,
/// always terminated). Returns the length written.
std::size_t panelSafe(const char* text, std::size_t budget, char* out, std::size_t cap) {
    const std::size_t limit = budget < cap - 1 ? budget : cap - 1;
    std::size_t k = 0;
    const auto* p = reinterpret_cast<const unsigned char*>(text != nullptr ? text : "");
    while (*p != 0 && k < limit) {
        const unsigned char c = *p;
        if (c >= 0x80) {
            // A UTF-8 sequence. p[1] is read only when p[0] is not the terminator and p[2] only
            // when p[1] is 0x80 -- neither read passes the string's end.
            if (c == 0xE2 && p[1] == 0x80 && (p[2] == 0x93 || p[2] == 0x94)) {
                out[k++] = '-';  // an en or em dash
                p += 3;
                continue;
            }
            if (c >= 0xC0) out[k++] = '?';  // a lead byte: one '?' for the whole character
            ++p;                             // a continuation byte (or the lead just replaced)
            continue;
        }
        out[k++] = (c < 0x20 || c == 0x7F || c == '%') ? '?' : static_cast<char>(c);
        ++p;
    }
    out[k] = '\0';
    return k;
}

/// Print `text`, made panel-safe and cut at `budget`, padded with spaces to `padTo` characters
/// (never past the budget -- an in-place update covers the old, longer text: VEXos draws text
/// pen-on-ERASER, an opaque cell), through a literal "%s", `pen` on `eraser`.
void screenText(pros::text_format_e_t font, std::int16_t x, std::int16_t y, std::uint32_t pen,
                std::uint32_t eraser, const char* text, std::size_t budget,
                std::size_t padTo = 0) {
    char out[kPanelCap + 1];
    std::size_t k = panelSafe(text, budget, out, sizeof out);
    const std::size_t limit = budget < sizeof out - 1 ? budget : sizeof out - 1;
    while (k < padTo && k < limit) out[k++] = ' ';
    out[k] = '\0';
    pros::c::screen_set_eraser(eraser);
    pros::c::screen_set_pen(pen);
    pros::c::screen_print_at(font, x, y, "%s", out);
    ++g_screenPrints;
}

/// True when a LITERAL is fit for the panel as written: printable ASCII, no '%', at most
/// `budget` characters. Every menu literal is checked with it at compile time.
constexpr bool panelLiteralFits(const char* s, std::size_t budget) {
    std::size_t n = 0;
    for (; s[n] != '\0'; ++n) {
        const char c = s[n];
        if (c < 0x20 || c > 0x7E || c == '%') return false;
    }
    return n <= budget;
}

/// Severity, so a result can be READ rather than parsed. The bencher is crouched
/// over a robot at arm's length -- colour and position carry the verdict, and the
/// words are there for the log afterwards.
enum class Sev { Info, Good, Warn, Bad };

int g_screenLine = 0;
bool g_screenActive = false;
Sev g_lineSev = Sev::Info;      // severity of the line currently being emitted
Sev g_lastVerdict = Sev::Info;  // what the running test concluded

/// What each menu entry concluded last time it ran; -1 = never run. "Passed" and
/// "not tried yet" must look different, or a bencher cannot tell what is left.
/// Twelve since R2a (11 DISTANCE, 12 AI VISION), thirteen since MECH-A6 (13 MECHANISMS); the menu
/// grid in bench_r3a.hpp is sized to match, and both are static_asserted against the menu table below.
constexpr int kMaxMenu = 13;
int g_verdict[kMaxMenu] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

std::uint32_t sevColour(Sev s) {
    switch (s) {
        case Sev::Good: return kColGood;
        case Sev::Warn: return kColWarn;
        case Sev::Bad:  return kColBad;
        default:        return kColText;
    }
}

/// A test's overall conclusion. Escalates only -- one failed stage makes the whole
/// test failed, and a later Info line must not quietly clear it.
void setVerdict(Sev s) {
    if (static_cast<int>(s) > static_cast<int>(g_lastVerdict)) g_lastVerdict = s;
}

/// Blocks until a NEW release event arrives.
///
/// EDGE ON THE COUNTER, NOT THE STATUS VALUE -- this is the whole bug of
/// 2026-08-18. `screen.h` states the status "will be released by default if no
/// action was taken", so `touch_status == E_TOUCH_RELEASED` is ALSO the at-rest
/// value, and waiting for it returns instantly without anyone touching anything.
/// Worse, the x/y that come with it are STALE from the previous touch, so a menu
/// polling on the status value re-selects the last button forever.
/// `release_count` is monotonic, so an edge on it is an unambiguous "a new tap
/// happened" regardless of how the enum is meant to read. (The vendored docs name
/// the values E_TOUCH_EVENT_RELEASE/PRESS while the enum spells them
/// E_TOUCH_RELEASED/PRESSED, and their comments are transposed -- another reason
/// not to build behaviour on that value.)
///
/// AN ERROR READ IS NOT A TAP (R2a, 2026-09-24). `screen.h:726-729`: when the screen mutex
/// fails, the status comes back as E_TOUCH_ERROR with -1 in EVERY field -- so an edge on
/// release_count alone reads it as a new release, and the next GOOD read as another one at the
/// STALE x/y. The baseline comes from releaseBaseline() (below) and error reads are skipped.
void waitForTouch();

/// The release count from the first NON-error touch read -- the baseline every "new tap" edge
/// in this file is taken against (the menu loop, waitForTouch, the R2a stations' TapWatch).
std::int32_t releaseBaseline() {
    for (;;) {
        const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
        if (t.touch_status != pros::E_TOUCH_ERROR) return t.release_count;
        pros::delay(5);
    }
}

void waitForTouch() {
    const std::int32_t start = releaseBaseline();
    for (;;) {
        const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
        if (t.touch_status != pros::E_TOUCH_ERROR && t.release_count != start) return;
        pros::delay(20);
    }
}

/// Clears only the CONTENT region, leaving the header bar painted -- so the
/// section title stays on screen while its results scroll underneath.
void screenClear() {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, static_cast<std::int16_t>(kHeaderH), kUsableW, kUsableH);
    g_screenLine = 0;
}

/// The persistent header: coloured bar, section title, build stamp on the right.
void drawHeader(const char* title) {
    pros::c::screen_set_pen(kColBar);
    pros::c::screen_fill_rect(0, 0, kUsableW, kHeaderH);
    // Cut at 30 as before ("%.30s" until R2a) -- now through the sanitising path.
    screenText(pros::E_TEXT_MEDIUM, 8, 6, kColText, kColBar, title, 30);
    pros::c::screen_set_eraser(kColBg);
}

/// True for lines that exist only to shape an 80-column TERMINAL -- rule bars,
/// banner hashes, separators. They are correct on serial and in the SD file, and
/// they are noise on a ~54-column panel, where they arrive truncated mid-dashes.
/// THIS IS THE ACTUAL REASON THE UI LOOKED BAD: one string was being written for
/// two very different displays.
bool isTerminalDecoration(const char* line) {
    if (line[0] == '\0') return false;            // blank lines are real spacing
    const char c = line[0];
    if (c != '#' && c != '=') return false;       // only banner hashes and rule bars
    for (const char* p = line; *p != '\0'; ++p) {
        if (*p != c && *p != ' ') return false;   // has real content -> keep it
    }
    return true;                                   // nothing but the rule character
}

void screenEmit(const char* line) {
    if (!g_screenActive) return;
    if (isTerminalDecoration(line)) return;
    if (g_screenLine >= kScreenLines) {
        pros::c::screen_set_pen(kColWarn);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY, "more below - TOUCH to continue");
        waitForTouch();
        screenClear();
    }
    // Indented/continuation lines are supporting detail: dim them so the eye finds
    // the headline values first. Through the sanitising path (R2a): the same 54-column cut
    // as before, and a '%' or a UTF-8 dash in the line can no longer reach VEXos.
    const std::uint32_t pen = g_lineSev != Sev::Info ? sevColour(g_lineSev)
                              : (line[0] == ' ')     ? kColDim
                                                     : kColText;
    screenText(pros::E_TEXT_SMALL, 8,
               static_cast<std::int16_t>(kContentY + g_screenLine * kLineH), pen, kColBg, line,
               static_cast<std::size_t>(kScreenCols));
    ++g_screenLine;
}

/// A big fixed-position readout -- for a number somebody watches while physically
/// moving the robot, where a scrolling log would be unreadable.
void screenBig(int y, const char* text) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, static_cast<std::int16_t>(y), kUsableW,
                               static_cast<std::int16_t>(y + 36));
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_LARGE, 8, static_cast<std::int16_t>(y), "%s", text);
}

void drawHeader(const char* title);  // fwd

/// A section boundary. The SERIAL/SD form is a full-width bar; the SCREEN form is a
/// painted header, because a 50-character rule arrives on the panel as a row of
/// truncated dashes and reads as damage rather than structure.
/// Emit one line AND record what it means. Using this instead of emitf() at the
/// handful of lines that carry a verdict is what lets the menu show per-test
/// status and the header show a PASS/FAIL chip.
void emitS(Sev sev, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
void emitS(Sev sev, const char* fmt, ...) {
    char buf[220];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, args);
    va_end(args);
    g_lineSev = sev;
    setVerdict(sev);
    emit(buf);
    g_lineSev = Sev::Info;
}

/// A PASS / CHECK / FAIL chip in the header bar. Position is fixed and the colour
/// carries the meaning, so the answer is legible across a workbench without
/// reading a word of it.
void drawVerdictChip(Sev sev) {
    const char* word = sev == Sev::Good ? " PASS " : sev == Sev::Warn ? " CHECK "
                     : sev == Sev::Bad  ? " FAIL " : " DONE ";
    pros::c::screen_set_pen(sevColour(sev));
    pros::c::screen_fill_rect(388, 6, 472, 28);
    pros::c::screen_set_eraser(sevColour(sev));
    pros::c::screen_set_pen(kColBg);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 396, 9, "%s", word);
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_set_pen(kColText);
}

void rule(const char* title) {
    const bool wasActive = g_screenActive;
    g_screenActive = false;                       // keep the bar off the panel
    emit("");
    emitf("== %s ==========================================", title);
    g_screenActive = wasActive;
    if (wasActive) {
        drawHeader(title);
        screenClear();
    }
}

/// v5_device_e_t → a short human name. `default` is deliberate: an unknown code
/// prints its NUMBER rather than being silently folded into "none".
const char* deviceName(pros::c::v5_device_e_t t) {
    switch (t) {
        case pros::c::E_DEVICE_NONE:     return "-";
        case pros::c::E_DEVICE_MOTOR:    return "MOTOR";
        case pros::c::E_DEVICE_ROTATION: return "ROTATION";
        case pros::c::E_DEVICE_IMU:      return "IMU";
        case pros::c::E_DEVICE_DISTANCE: return "DISTANCE";
        case pros::c::E_DEVICE_RADIO:    return "RADIO";
        case pros::c::E_DEVICE_VISION:   return "VISION";
        case pros::c::E_DEVICE_ADI:      return "ADI-EXPANDER";
        case pros::c::E_DEVICE_OPTICAL:  return "OPTICAL";
        case pros::c::E_DEVICE_GPS:      return "GPS";
        case pros::c::E_DEVICE_AIVISION: return "AI-VISION";
        case pros::c::E_DEVICE_SERIAL:   return "SERIAL";
        default:                         return "UNKNOWN";
    }
}

/// A two-button question on the panel; returns true when the LEFT button was tapped.
/// Same shape as the IMU test's "WHICH WAY DID YOU TURN IT?" prompt (which keeps its own
/// inline copy, untouched -- it has run on hardware). Used by MOTOR WATCH's push-direction
/// question and DRIVE's wheels-off-the-ground confirmation. Edge on release_count, like
/// every other touch in this file.
bool twoButtonPrompt(const char* title, const char* sub, const char* leftBig,
                     const char* leftSmall, const char* rightBig, const char* rightSmall,
                     const char* foot1, const char* foot2) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 8, "%s", title);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 32, "%s", sub);
    pros::c::screen_set_pen(kColBar);
    pros::c::screen_fill_rect(6, 60, 234, 150);
    pros::c::screen_fill_rect(246, 60, 474, 150);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_set_eraser(kColBar);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 20, 82, "%s", leftBig);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 20, 112, "%s", leftSmall);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 262, 82, "%s", rightBig);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 262, 112, "%s", rightSmall);
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_set_pen(kColDim);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 170, "%s", foot1);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 184, "%s", foot2);

    const std::int32_t seen = pros::c::screen_touch_status().release_count;
    for (;;) {
        const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
        if (t.release_count != seen && t.y >= 60 && t.y <= 150) {
            return t.x < 240;
        }
        pros::delay(20);
    }
}

/// A live panel layout for N cells (R3b Session 2: "make it lay out N"). Two columns up
/// to ten cells, three beyond; the row pitch shrinks to fit and the font drops to SMALL
/// when the pitch no longer fits MEDIUM. For the bench bot's 8 this reproduces the
/// original two-columns-of-four layout exactly (x = 8 / 248, pitch 34, MEDIUM).
struct Grid {
    int cols;
    int rows;
    std::int16_t y0;
    std::int16_t pitch;
    std::int16_t cellW;
    pros::text_format_e_t font;
};

Grid gridFor(std::size_t n, std::int16_t y0, bool forceSmall) {
    Grid g{};
    g.y0 = y0;
    g.cols = n <= 10 ? 2 : 3;
    g.rows = static_cast<int>((n + static_cast<std::size_t>(g.cols) - 1)
                              / static_cast<std::size_t>(g.cols));
    if (g.rows < 1) g.rows = 1;
    int pitch = (kFooterY - 4 - y0) / g.rows;
    if (pitch > 34) pitch = 34;
    if (pitch < 13) pitch = 13;
    g.pitch = static_cast<std::int16_t>(pitch);
    g.cellW = static_cast<std::int16_t>(kUsableW / g.cols);
    g.font = (!forceSmall && pitch >= 26) ? pros::E_TEXT_MEDIUM : pros::E_TEXT_SMALL;
    return g;
}

/// Erase-then-print one cell of a Grid.
void gridCell(const Grid& g, std::size_t i, std::uint32_t colour, const char* text) {
    const int col = static_cast<int>(i) / g.rows;
    const int row = static_cast<int>(i) % g.rows;
    const std::int16_t cx = static_cast<std::int16_t>(8 + col * g.cellW);
    const std::int16_t cy = static_cast<std::int16_t>(g.y0 + row * g.pitch);
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(cx, cy, static_cast<std::int16_t>(cx + g.cellW - 16),
                               static_cast<std::int16_t>(cy + g.pitch - 4));
    pros::c::screen_set_pen(colour);
    pros::c::screen_print_at(g.font, cx, cy, "%s", text);
}

/// Probe the card in stages, because "SD LOGGING OFF" alone is not actionable:
/// ProsBlockSink only calls fopen() when usd_is_installed() is nonzero, and
/// isOpen() reads false in BOTH cases -- card-not-detected and file-open-failed
/// are different problems with different fixes. Uses its OWN probe file so it
/// cannot interfere with the real log.
void probeSdCard() {
    rule("SD CARD PROBE (HA-122)");
    const std::int32_t installed = pros::c::usd_is_installed();
    emitf("1. usd_is_installed()        : %ld", static_cast<long>(installed));
    if (installed == 0) {
        emitS(Sev::Bad, "   >> NO CARD DETECTED by VEXos. Not readable at all.");
        emit("   >> Card must be FAT32 (NOT exFAT -- that is the usual cause).");
        emit("   >> Also: reseat it, and POWER-CYCLE -- VEXos mounts at boot, so a");
        emit("   >> card inserted while running is never picked up.");
        return;
    }
    emitS(Sev::Good, "   >> card IS detected.");

    errno = 0;
    std::FILE* probe = std::fopen("/usd/probe.txt", "wb");
    emitf("2. fopen(/usd/probe.txt,wb) : %s", probe != nullptr ? "OK" : "FAILED");
    if (probe == nullptr) {
        emitS(Sev::Bad, "   >> errno=%d -- detected but NOT WRITABLE.", errno);
        emit("   >> errno 6 (ENXIO) = not a FAT32 drive. Reformat FAT32.");
        emit("   >> free space, and that the card really is FAT32.");
        return;
    }
    const char* payload = "shulib r3a probe\n";
    const std::size_t wrote = std::fwrite(payload, 1, std::strlen(payload), probe);
    const int flushed = std::fflush(probe);
    std::fclose(probe);
    emitf("3. fwrite/fflush            : %u bytes, fflush=%d", static_cast<unsigned>(wrote),
          flushed);
    if (wrote == std::strlen(payload) && flushed == 0) {
        emitS(Sev::Good, "   >> CARD IS FULLY WORKING. probe.txt written and flushed.");
    } else {
        emitS(Sev::Warn, "   >> partial write -- card may be full or failing.");
    }
}

/// SCREEN RULER -- the calibration display.
///
/// Three layout bugs shipped to this robot because font metrics and panel bounds
/// were REASONED rather than measured (a host check even passed against the wrong
/// height). PROS documents font *names* and no dimensions anywhere, so this draws
/// a ruler and lets a human read the numbers off, exactly as HA-107 prescribes for
/// the controller LCD's disputed column count. Read it once, write the numbers into
/// the constants, stop guessing.
void screenRuler(pros::c::v5_device_e_t*) {
    g_screenActive = false;
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();

    // A frame at the EXACT claimed bounds. If any edge is missing, the usable area
    // is not what lv_conf.h says and every layout constant is suspect.
    pros::c::screen_set_pen(kColWarn);
    pros::c::screen_draw_rect(0, 0, static_cast<std::int16_t>(kUsableW - 1),
                              static_cast<std::int16_t>(kUsableH - 1));

    // Column ruler: every 10th character is its tens digit, so the last VISIBLE
    // digit tells you how many columns that font actually fits.
    static char ruler[81];
    for (int i = 0; i < 80; ++i) ruler[i] = static_cast<char>('0' + ((i / 10) % 10));
    ruler[80] = '\0';

    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 4, 4, "S %s", ruler);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 4, 22, "M %s", ruler);
    pros::c::screen_print_at(pros::E_TEXT_LARGE, 4, 48, "L %s", ruler);

    // Row pitch: consecutive SMALL lines at the constant this file uses. If they
    // touch or overlap, kLineH is too small; if they gap badly, it is too large.
    pros::c::screen_set_pen(kColDim);
    for (int i = 0; i < 8; ++i) {
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 4,
                                 static_cast<std::int16_t>(90 + i * kLineH),
                                 "row %d at y=%d (pitch %d)", i, 90 + i * kLineH,
                                 static_cast<int>(kLineH));
    }

    // Vertical scale: ticks every 20px down the right edge. The last LABEL you can
    // read is the true bottom of the usable area.
    pros::c::screen_set_pen(kColGood);
    for (int y = 0; y < kUsableH; y += 20) {
        pros::c::screen_draw_line(static_cast<std::int16_t>(kUsableW - 40),
                                  static_cast<std::int16_t>(y),
                                  static_cast<std::int16_t>(kUsableW - 22),
                                  static_cast<std::int16_t>(y));
        pros::c::screen_print_at(pros::E_TEXT_SMALL, static_cast<std::int16_t>(kUsableW - 20),
                                 static_cast<std::int16_t>(y), "%d", y);
    }

    emit("SCREEN RULER drawn. READ THESE OFF THE PANEL and report them:");
    emitf("  claimed usable area : %dx%d (from liblvgl/lv_conf.h)", static_cast<int>(kUsableW),
          static_cast<int>(kUsableH));
    emitf("  row pitch in use    : %d px", static_cast<int>(kLineH));
    emitf("  truncation width    : %d chars", kScreenCols);
    emit("  1. is the WHOLE yellow frame visible, all four edges?");
    emit("  2. last readable digit on each of rows S / M / L?");
    emit("  3. do the grey 'row N' lines sit evenly, without touching?");
    emit("  4. highest green tick number still on screen?");
}

// ═══ STAGE 1 — the census. Raw registry only; constructs nothing. ═════════════
void census(pros::c::v5_device_e_t* found) {
    rule("STAGE 1  DEVICE CENSUS (raw registry, no adapters)");
    emit("registry index is ZERO-based; PORT below is the physical 1-21 number.");
    emit("PORT  idx  type            chassis table says");
    int motors = 0;
    for (int idx = 0; idx <= kMaxRegistryIndex; ++idx) {
        const pros::c::v5_device_e_t t =
            pros::c::registry_get_plugged_type(static_cast<std::uint8_t>(idx));
        const int port = idx + 1;   // <- the whole correction
        found[port] = t;
        if (t == pros::c::E_DEVICE_MOTOR) ++motors;
        if (t != pros::c::E_DEVICE_NONE) {
            emitf("  %2d   %2d  %-14s  %s  (code %d)", port, idx, deviceName(t),
                  tableSide(port), static_cast<int>(t));
        }
    }
    emit("");
    if (tableHasPorts(kChassis)) {
        const unsigned expect = static_cast<unsigned>(tableCount(kChassis));
        emitS(motors >= static_cast<int>(expect) ? Sev::Good : Sev::Bad,
              "motors found: %d   (chassis table expects %u: %u left + %u right)", motors,
              expect, static_cast<unsigned>(kChassis.leftCount),
              static_cast<unsigned>(kChassis.rightCount));
        emit("EMPTY PORTS ARE OMITTED. A table port missing here is a FINDING.");
    } else {
        emitS(Sev::Warn, "motors found: %d   (chassis table has NO PORTS yet -- side '?')",
              motors);
        emit("Report these port numbers, per side, and which end is the FRONT; they go");
        emit("into src/robots/chassis_table.hpp. Nothing is guessed in the meantime.");
    }
    emit("Index 21+ is NOT scanned: apix.h documents 0-20, and reading past it is");
    emit("what produced the phantom 'ADI expander' on 2026-08-18 (HA-120 predicted it).");
}

void imuTimingStage(std::uint8_t port);  // STAGE 2c (IMU1 Part B), below reportImu

// ═══ STAGE 2 — IMU: raw PROS beside our canonical conversion. ════════════════
void reportImu(bool present) {
    rule("STAGE 2  IMU (HA-02/03/04/05/23/108/109/110)");
    if (!present) {
        emitS(Sev::Bad, "no IMU on port %u -- the chassis table is WRONG. See the census.",
              static_cast<unsigned>(kChassis.imuPort));
        return;
    }
    hal::pros::ProsClock clock{};
    try {
        hal::pros::ProsImu imu{kChassis.imuPort, math::Angle{}, clock};
        // A FRESH adapter each run, so `calibrateStarted_` is false and the
        // HA-05 second-calibrate precondition does not fire -- the physical IMU
        // is genuinely re-zeroed on every run of this test. That is DELIBERATE
        // here (a repeatable rotate test wants a fresh zero) and is exactly the
        // opposite of what a competition binary must do, where re-zeroing under a
        // live bootHeading is HA-05's hazard. Announced rather than silent.
        imu.calibrate();
        emit("re-zeroing the IMU: every run of this test starts from a fresh zero.");
        emit("calibrating (HA-23 says ~2 s) -- timing it:");
        const std::uint32_t t0 = pros::millis();
        while (!imu.isReady() && (pros::millis() - t0) < 8000U) {
            pros::delay(20);
        }
        emitf("  isReady() after %lu ms  (HA-23 claims ~2000 ms)",
              static_cast<unsigned long>(pros::millis() - t0));
        emit("");
        emit("RAW PROS                     |  CANONICAL (shulib)");
        for (int i = 0; i < 5; ++i) {
            emitf("  rotation=%9.3f deg        |  heading=%9.4f rad = %8.3f deg",
                  pros::c::imu_get_rotation(kChassis.imuPort), imu.heading().radians(),
                  imu.heading().degrees());
            pros::delay(200);
        }
        emitf("  heading(raw)=%8.3f deg      |  pitch=%7.3f  roll=%7.3f (canonical rad)",
              pros::c::imu_get_heading(kChassis.imuPort), imu.pitch().radians(), imu.roll().radians());
        emitf("  screened reads (held last-good): %d", imu.faultedReads());
        emit("");
        emit("** NOW ROTATE THE ROBOT COUNTER-CLOCKWISE (to its left, seen from above). **");
        emit("   Canonical heading MUST INCREASE. If it decreases, HA-02's sign is wrong");
        emit("   and every turn this library ever commands would be mirrored.");

        // A big fixed readout rather than a scrolling log: this is a number somebody
        // watches WHILE turning the robot with both hands.
        const double startDeg = imu.heading().degrees();
        // CUMULATIVE raw, because canonical heading is WRAPPED to (-180,180] and a
        // turn past half a revolution then reports the WRONG SIGN: rotate 200 deg
        // CCW and canonical runs 0 -> 180 -> wraps to -180 -> -160, so a wrapped
        // difference reads -160 on a counter-clockwise turn. The 2026-08-19 run
        // swept 170.66 deg and passed only because it stayed under the boundary.
        // get_rotation() is cumulative and unbounded (HA-03, still unconfirmed past
        // 360 deg -- flagged where the verdict is printed).
        const double startRaw = pros::c::imu_get_rotation(kChassis.imuPort);
        screenClear();
        g_screenActive = false;  // suspend the scroll; the panel is the instrument now

        // EXPLICIT PIXEL POSITIONS, like every other draw in this file. This block
        // used screen_print with LINE INDICES (0,1,4,5,8) and mixed MEDIUM with
        // SMALL while doing it -- so "line 4" depended on a per-font row height
        // nobody has measured, and index 8 could land on top of the big readout.
        // It is the same class of bug as the off-screen menu row, on the ONE screen
        // whose answer mirrors every turn the library will ever command.
        pros::c::screen_set_pen(kColText);
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 44, "ROTATE THE ROBOT");
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 66, "COUNTER-CLOCKWISE (its LEFT)");
        pros::c::screen_set_pen(kColDim);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 96, "heading MUST INCREASE as you turn.");
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 110, "If it FALLS, HA-02's sign is wrong");
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 124, "and every turn would be mirrored.");
        pros::c::screen_set_pen(kColWarn);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY, "reading for ~20 s ...");
        // PATH, not just endpoints. Net displacement alone cannot tell "turned
        // left 90" from "left 90 then right 145" -- the second nets -55 and would
        // print HA-02 WRONG on a perfectly good IMU. The loop already samples every
        // 200 ms, so accumulate travel in each direction and count reversals; a
        // mixed turn is then DETECTED rather than silently mis-judged.
        double ccwTravel = 0.0, cwTravel = 0.0, prevRaw = startRaw;
        int reversals = 0, lastDir = 0;
        char big[64];
        for (int i = 0; i < 100; ++i) {  // ~20 s to rotate under
            const double rawNow = pros::c::imu_get_rotation(kChassis.imuPort);
            const double dCanon = -(rawNow - prevRaw);  // canonical delta, unwrapped
            prevRaw = rawNow;
            // 0.1 deg per 200 ms sample clears the at-rest noise floor, which the
            // 2026-08-19 log showed sitting around 0.02-0.06 deg.
            if (dCanon > 0.1) {
                ccwTravel += dCanon;
                if (lastDir == -1) ++reversals;
                lastDir = 1;
            } else if (dCanon < -0.1) {
                cwTravel += -dCanon;
                if (lastDir == 1) ++reversals;
                lastDir = -1;
            }
            const double deg = imu.heading().degrees();
            std::snprintf(big, sizeof big, "%+8.2f deg", deg);
            screenBig(146, big);
            std::snprintf(big, sizeof big, "delta %+.2f", deg - startDeg);
            pros::c::screen_set_eraser(kColBg);
            pros::c::screen_erase_rect(0, 188, kUsableW, 210);
            pros::c::screen_set_pen((deg - startDeg) >= 0.0 ? kColGood : kColBad);
            pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 190, "%-28s", big);
            emitf("   raw=%9.3f deg   canonical=%9.4f rad = %8.3f deg",
                  pros::c::imu_get_rotation(kChassis.imuPort), imu.heading().radians(), deg);
            pros::delay(200);
        }
        // canonical = -raw (HA-02, confirmed 2026-08-19), so negating the raw
        // cumulative delta gives an UNWRAPPED canonical delta, valid past 180 deg.
        const double moved = -(pros::c::imu_get_rotation(kChassis.imuPort) - startRaw);
        const double wrapped = imu.heading().degrees() - startDeg;

        // ── ASK WHICH WAY THEY ACTUALLY TURNED. ────────────────────────────────
        // The first version asked for a CCW turn and reported the delta, which
        // silently ASSUMED the operator turned the direction requested. Two runs on
        // 2026-08-19 moved -170.66 and +87.14 -- opposite directions, both valid
        // readings, and NOTHING in the log said which way the robot was actually
        // rotated. The verdict was therefore unusable from either run. An operator
        // who does not know CCW from CW (or simply spins it back) is not a mistake
        // to design against; it is the normal case. So record the action instead of
        // assuming it.
        pros::c::screen_set_eraser(kColBg);
        pros::c::screen_erase();
        pros::c::screen_set_pen(kColText);
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 8, "WHICH WAY DID YOU TURN IT?");
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 32, "looking DOWN at the robot from above");
        pros::c::screen_set_pen(kColBar);
        pros::c::screen_fill_rect(6, 60, 234, 150);
        pros::c::screen_fill_rect(246, 60, 474, 150);
        pros::c::screen_set_pen(kColText);
        pros::c::screen_set_eraser(kColBar);
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 20, 82, "LEFT / CCW");
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 20, 112, "anti-clockwise");
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 262, 82, "RIGHT / CW");
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 262, 112, "clockwise");
        pros::c::screen_set_eraser(kColBg);
        pros::c::screen_set_pen(kColDim);
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 170, "if you turned BOTH ways, or are not");
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 184, "sure, re-run the test.");

        bool turnedCcw = true;
        std::int32_t seen = pros::c::screen_touch_status().release_count;
        for (;;) {
            const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
            if (t.release_count != seen && t.y >= 60 && t.y <= 150) {
                turnedCcw = (t.x < 240);
                break;
            }
            pros::delay(20);
        }

        g_screenActive = true;
        screenClear();
        emitf("PATH: turned CCW %.1f deg total, CW %.1f deg total, %d reversal(s)",
          ccwTravel, cwTravel, reversals);
        emitf("NET : %+.2f deg (unwrapped, from cumulative raw)", moved);
        emitf("  wrapped canonical difference would have read %+.2f deg", wrapped);
        if ((moved > 0.0) != (wrapped > 0.0)) {
            emitS(Sev::Warn, "  ^ THE TWO DISAGREE: the turn passed 180 deg. The");
            emit("    unwrapped value is the correct one.");
        }
        if (moved > 360.0 || moved < -360.0) {
            emit("  (turn exceeded a full revolution -- also evidence for HA-03,");
            emit("   that get_rotation() is cumulative and unbounded.)");
        }
        emitf("operator says they turned %s", turnedCcw ? "LEFT / CCW" : "RIGHT / CW");
        // Judge on the DOMINANT direction of travel, and refuse to judge at all
        // when the turn went meaningfully both ways -- a net figure cannot separate
        // "left 90" from "left 90 then right 145".
        const double dominant = ccwTravel > cwTravel ? ccwTravel : cwTravel;
        const double minor    = ccwTravel > cwTravel ? cwTravel : ccwTravel;
        const bool dominantCcw = ccwTravel > cwTravel;

        if (dominant < 5.0) {
            emitS(Sev::Warn, "barely moved -- turn it further and re-run.");
        } else if (minor > 0.2 * dominant) {
            emitS(Sev::Warn, "MIXED TURN -- you went BOTH ways (%.0f CCW vs %.0f CW).",
                  ccwTravel, cwTravel);
            emit("  No verdict: a net figure cannot tell 'left 90' from 'left 90");
            emit("  then right 145'. Re-run and turn ONE way only.");
        } else if (dominantCcw == turnedCcw) {
            emitS(Sev::Good, "HA-02 CONFIRMED: canonical heading is CCW-POSITIVE.");
            emitf("  (dominant travel was %s, matching what you declared.)",
                  dominantCcw ? "CCW" : "CW");
        } else {
            emitS(Sev::Bad, "HA-02 WRONG: the sign is INVERTED. Every turn would mirror.");
            emitf("  You declared %s but the heading's dominant travel was %s.",
                  turnedCcw ? "CCW" : "CW", dominantCcw ? "CCW" : "CW");
        }

        // STAGE 2c (IMU1 Part B): the IMU's own timing, on the IMU this stage just calibrated.
        // Only on this path -- an adapter that refused the device (below) has nothing to time.
        imuTimingStage(kChassis.imuPort);
    } catch (const PreconditionError& e) {
        // The rotate test suspends the scrolling log; restore it or this report
        // would land on serial and the SD card but never on the panel the
        // bencher is actually looking at.
        g_screenActive = true;
        emitS(Sev::Bad, "IMU ADAPTER REFUSED THE DEVICE: %s", e.what());
        emit("(that refusal IS the measurement -- record it verbatim)");
    }
}

// ═══ STAGE 2c — THE IMU'S TIMING (IMU1 Part B, 2026-11-03; HA-213/214/215, HA-04, HA-109) ═══
// IMU1 Part A made ProsImu::yawRate() keep its answer for 5 ms, so every reader in one control
// tick gets one value. That fix, and the residual it leaves, rest on PROS's 10 ms copy of the
// IMU's readings (HA-213), the copy's unknown phase against the tick (HA-214) and a tick's reads
// falling within 5 ms of its first (HA-215); the gyro's own rate -- the cure for the residual --
// waits on its sign (HA-04) and its units (HA-109). None is measured. This stage is their first
// look (worksheet Station N), in four parts: 2c.1 the cadence, 2c.2 the phase, 2c.3 a stand-in
// for a tick's span, 2c.4 the gyro beside the rotation. What each part concludes is arithmetic in
// programs/imu_timing.hpp -- PROS-free, host-tested (test/bench_imu_timing_test.cpp) -- and so are
// the four loops that collect the numbers (runCadence / runPhase / runSpans / runGyro, templates the
// stage gives pros::micros(), delay_until and the C API's reads; the IMU1-B review, 2026-11-05).
// This function names the reads, runs the loops and prints.
//
// THE RULES IT KEEPS (the IMU1 brief's §1 item 5, its re-checks R10 and R18, landmines 15-17):
//   * READ-ONLY. It reads imu_get_rotation, imu_get_gyro_rate, micros(), the drive motors'
//     motor_get_* and the battery, and paces with pros::Task::delay_until; it commands no motor
//     and writes no device.
//   * NEVER ProsImu::yawRate(): it answers CANONICAL CCW rad/s, and held for 5 ms (IMU1 Part A)
//     a tight loop sees it refresh at most every 5 ms. 2c.4 compares ANGLES, both RAW PROS
//     degrees (R18).
//   * Paced like STAGE 5 (delay_until), never ProsTickPacer -- whose header pulls the motion
//     scheduler into this program -- and no Localizer, so 2c.3 is a STAND-IN BY PLAN (R10): the
//     device reads a motion tick makes between its two IMU reads, and one serial line. A LOWER
//     bound on a real tick's span, and it says so.
//   * It ASKS before it reads (START or SKIP -- so 7 RUN ALL, which runs this too, and an HA-02
//     check can pass it by) and records what it SAW, never what it asked for: a robot held still
//     shows no copy, and the stage counts that and says "turn it", never guesses (landmine 15;
//     the rotate test's 2026-08-19 lesson above).
//   * It BUSY-READS (2c.1 2 s, 2c.2 up to 9.5 ms in each of 100 ticks, 2c.4 3 s), so every task
//     below this one's priority -- the panel's redraw among them -- waits meanwhile; the panel
//     says so first. Fine in a program that commands no motor; never copy it into one that drives
//     (landmine 16). It ASSUMES PROS's copy is made above this task's priority -- unverified: a
//     loop that sees no change while the robot turns is what a wrong assumption looks like, and
//     the cadence line says so.
//   * Every line through emit() / emitf() / emitS() (SER1's one-call path), except 2c.3's own
//     serial line, which goes through hal::pros::serialLine() -- SER1's one call too -- because a
//     motion tick's record goes to serial only: emit() would also paint the panel (PROS 4.2.2's
//     print_at leaks per call -- R3d Part 2F-2) and write the SD card, which a tick does not.
// Station 2's own functions only (reportImu calls this): never kMenu, menu_grid or bench_r3a.hpp
// (MECH-A6's), and no new menu button.
namespace imu2c {

namespace it = imu_timing;

static_assert(it::kHoldUs == hal::pros::ProsImu::kRateHoldUs,
              "2c.3's verdict is judged against the library's rate hold: move them together");
static_assert(it::kPhaseWindowUs < 10U * 1000U,
              "2c.2's poll must end before the next 10 ms wake, or it makes that wake late");

constexpr std::uint64_t kCadenceUs = 2000000;  // 2c.1: about 200 copies at 10 ms
constexpr int kPhaseTicks = 100;               // 2c.2: one second of ticks
constexpr int kSpanTicks = 100;                // 2c.3: one second of ticks
constexpr std::uint64_t kGyroUs = 3000000;     // 2c.4: room for a >= 2 s first-to-last window
constexpr std::uint32_t kTickMs = 10;          // the motion tick's period, as STAGE 5 paces it

/// 2c.1's change instants. 512 is ~2.5 x a 10 ms copy's count in 2 s; a faster copy fills it, and
/// the cadence line then says how many changes it could not keep (the median is still judged).
constexpr std::size_t kMaxChanges = 512;

// Static storage, not the task's stack: ~7 KB the bench's task stack need not carry.
std::uint64_t g_changeAt[kMaxChanges];
std::uint32_t g_scratch[kMaxChanges];
it::TickPhase g_ticks[kPhaseTicks];
std::uint32_t g_spans[kSpanTicks];

// The fixed panel's words while hands are on the robot (the scroll is off). MEDIUM lines fit 28
// characters (the rotate test's widest), SMALL lines kScreenCols -- checked here, as the menu's.
constexpr const char* kTurnBig1 = "TURN IT SLOWLY";
constexpr const char* kTurnBig2 = "ONE WAY - KEEP TURNING";
constexpr const char* kTurnSmall1 = "by hand, until this says STOP (about 3 s)";
constexpr const char* kTurnSmall2 = "2c.1 cadence 2 s, then 2c.2 phase 1 s";
constexpr const char* kGyroBig1 = "NOW THE GYRO (2c.4)";
constexpr const char* kGyroBig2 = "TOUCH, THEN TURN IT";
constexpr const char* kGyroSmall1 = "ONE way, about a QUARTER TURN A SECOND,";
constexpr const char* kGyroSmall2 = "until this says STOP (about 3 s)";
constexpr const char* kSpanBig1 = "STOP - LET GO";
constexpr const char* kSpanSmall1 = "2c.3: timing a stand-in tick for 1 s";
constexpr const char* kSpanSmall2 = "nothing to do";
constexpr const char* kDoneBig1 = "STOP - DONE";
constexpr const char* kDoneSmall1 = "the four results follow";
constexpr const char* kReadStage2 = "STAGE 2 done -- read it, then TOUCH for 2c";
constexpr std::size_t kBigCols = 28;
static_assert(panelLiteralFits(kTurnBig1, kBigCols) && panelLiteralFits(kTurnBig2, kBigCols)
                  && panelLiteralFits(kGyroBig1, kBigCols) && panelLiteralFits(kGyroBig2, kBigCols)
                  && panelLiteralFits(kSpanBig1, kBigCols) && panelLiteralFits(kDoneBig1, kBigCols),
              "a STAGE 2c MEDIUM line no longer fits the panel");
static_assert(panelLiteralFits(kTurnSmall1, kScreenCols) && panelLiteralFits(kTurnSmall2, kScreenCols)
                  && panelLiteralFits(kGyroSmall1, kScreenCols)
                  && panelLiteralFits(kGyroSmall2, kScreenCols)
                  && panelLiteralFits(kSpanSmall1, kScreenCols)
                  && panelLiteralFits(kSpanSmall2, kScreenCols)
                  && panelLiteralFits(kDoneSmall1, kScreenCols)
                  && panelLiteralFits(kReadStage2, kScreenCols),
              "a STAGE 2c SMALL line no longer fits the panel");

/// The fixed panel: two MEDIUM lines and two SMALL ones, where the rotate test puts its own.
void panel(const char* big1, const char* big2, const char* small1, const char* small2) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 44, "%s", big1);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 66, "%s", big2);
    pros::c::screen_set_pen(kColDim);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 96, "%s", small1);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 110, "%s", small2);
}

/// The panel's footer line, replaced.
void foot(std::uint32_t pen, const char* text) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, kFooterY, kUsableW, kUsableH);
    pros::c::screen_set_pen(pen);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY, "%s", text);
}

/// A size_t / uint32 for "%lu".
unsigned long ul(std::size_t v) { return static_cast<unsigned long>(v); }

/// The stage's clock: pros::micros(), read before every device read (the loops in imu_timing.hpp).
std::uint64_t nowUs() { return pros::micros(); }

/// 2c.3's stand-in body, between the tick's two IMU reads (imu_timing.hpp's runSpans makes those
/// and stamps the span): each drive motor's position, velocity, current and voltage, the battery,
/// and one serial line. Ports are read by |port|: raw, as STAGE 3 reads them.
void standInBody(std::size_t tick, double rot, std::size_t& motorReads, std::size_t& motorErrors) {
    double pos[2] = {0.0, 0.0};
    double vel[2] = {0.0, 0.0};
    long cur[2] = {0, 0};
    long volt[2] = {0, 0};
    const std::int8_t* sides[2] = {kChassis.left, kChassis.right};
    const std::size_t counts[2] = {kChassis.leftCount, kChassis.rightCount};
    for (int s = 0; s < 2; ++s) {
        for (std::size_t k = 0; k < counts[s]; ++k) {
            const std::int8_t raw = static_cast<std::int8_t>(absPort(sides[s][k]));
            const double p = pros::c::motor_get_position(raw);
            const double v = pros::c::motor_get_actual_velocity(raw);
            const std::int32_t a = pros::c::motor_get_current_draw(raw);
            const std::int32_t u = pros::c::motor_get_voltage(raw);
            motorReads += 4;
            motorErrors += (std::isfinite(p) ? 0U : 1U) + (std::isfinite(v) ? 0U : 1U)
                           + (a == PROS_ERR ? 1U : 0U) + (u == PROS_ERR ? 1U : 0U);
            if (k == 0) {  // the line carries each side's first motor, as a record carries a few
                pos[s] = p;
                vel[s] = v;
                cur[s] = static_cast<long>(a);
                volt[s] = static_cast<long>(u);
            }
        }
    }
    const long battery = static_cast<long>(pros::c::battery_get_voltage());
    hal::pros::serialLine("[R3A] imu2c span %lu/%d (a stand-in record line) rot %.3f bat %ld"
                          " | L %.1f %.1f %ld %ld | R %.1f %.1f %ld %ld",
                          ul(tick + 1), kSpanTicks, rot, battery, pos[0], vel[0], cur[0], volt[0],
                          pos[1], vel[1], cur[1], volt[1]);
}

void reportCadence(const it::ChangeWatch& watch, const it::Cadence& cad, std::size_t overflow,
                   it::CadenceVerdict verdict) {
    emit("-- 2c.1 CADENCE: the raw rotation, a tight loop for 2 s (HA-213) --");
    emitf("  polls %lu, PROS_ERR %lu, changes %lu (raw deg, PROS's rotation)", ul(watch.polls()),
          ul(watch.badReads()), ul(watch.changes()));
    if (overflow > 0) {
        emitf("  (%lu more changes than the %lu kept: the first %lu judged)", ul(overflow),
              ul(kMaxChanges), ul(kMaxChanges));
    }
    emitf("  change intervals: n %lu  min %lu  median %lu  max %lu us", ul(cad.interval.n),
          ul(cad.interval.min), ul(cad.interval.median), ul(cad.interval.max));
    emitf("  in %lu..%lu us: %lu of %lu (below %lu, above %lu); their mean %.1f us",
          ul(it::kCopyPeriodUs - it::kBandUs), ul(it::kCopyPeriodUs + it::kBandUs), ul(cad.inBand),
          ul(cad.interval.n), ul(cad.below), ul(cad.above), cad.meanInBandUs);
    switch (verdict) {
        case it::CadenceVerdict::NoReads:
            emitS(Sev::Warn, "HA-213: every read was PROS_ERR -- no finding.");
            emit("  Check the IMU's cable and port (STAGE 1), and re-run.");
            break;
        case it::CadenceVerdict::NoChange:
            emitS(Sev::Warn, "HA-213: NO CHANGE SEEN in 2 s -- turn it and re-run.");
            if (watch.badReads() == 0) {
                emit("  (if it repeats WHILE turning: PROS's copy is not made while a loop");
                emit("   busy-reads -- that is the finding; write it down)");
            } else {
                emit("  (PROS_ERR above is not 0: the IMU dropped reads -- check its");
                emit("   cable; this run is no finding about PROS's copy)");
            }
            break;
        case it::CadenceVerdict::TooFew:
            emitS(Sev::Warn, "HA-213: %lu change(s) only -- turn steadily; re-run.",
                  ul(watch.changes()));
            emitf("  (a verdict needs at least %lu intervals between changes)", ul(it::kMinIntervals));
            break;
        case it::CadenceVerdict::Holds:
            emitS(Sev::Good, "HA-213 HOLDS: a change every ~10 ms (median %lu us).",
                  ul(cad.interval.median));
            break;
        case it::CadenceVerdict::Faster:
            emitS(Sev::Bad, "HA-213 WRONG, FASTER: median %lu us, not ~10000.",
                  ul(cad.interval.median));
            emit("  (IMU1's hold is still right; HA-214's residual is smaller)");
            break;
        case it::CadenceVerdict::Slower:
            emitS(Sev::Warn, "HA-213: median %lu us -- SLOWER than 10 ms, or",
                  ul(cad.interval.median));
            emit("  the robot was still for most of the 2 s. Re-run turning");
            emit("  steadily; if it repeats, HA-213 is WRONG (slower): a");
            emit("  tick's one read can be 0.");
            break;
    }
}

void reportPhase(const it::Phase& ph, it::PhaseVerdict verdict) {
    const double noise = it::kDriftSigmas * ph.driftSeUsPerS;
    emit("-- 2c.2 PHASE: 100 ticks paced by delay_until(10 ms) (HA-214) --");
    emitf("  copy's offset from the wake: first %ld  median %lu  last %ld us (%lu ticks)",
          static_cast<long>(ph.firstUs), ul(ph.medianUs), static_cast<long>(ph.lastUs),
          ul(ph.seen));
    emitf("  spread of those offsets: min %lu  max %lu us", ul(ph.minUs), ul(ph.maxUs));
    emitf("  no change within %lu us: %lu of %lu ticks", ul(it::kPhaseWindowUs), ul(ph.missed),
          ul(ph.ticks));
    if (ph.driftValid && ph.seValid) {
        emitf("  drift %+.1f +- %.1f us/s (%.0f SE; least squares, unwrapped at %lu us)",
              ph.driftUsPerS, noise, it::kDriftSigmas, ul(it::kCopyPeriodUs));
        emitf("  residual SD %.1f us (the wakes' jitter, mostly)", ph.residualSdUs);
    } else if (ph.driftValid) {
        emitf("  drift %+.1f us/s (no SE: under three ticks saw a change)", ph.driftUsPerS);
    } else {
        emit("  drift: not measurable (under two ticks saw a change)");
    }
    switch (verdict) {
        case it::PhaseVerdict::NoCadence:
            emitS(Sev::Warn, "HA-214: no verdict -- 2c.1 did not find the 10 ms copy.");
            break;
        case it::PhaseVerdict::NoChange:
            emitS(Sev::Warn, "HA-214: no tick saw a change -- turn it; re-run (or");
            emit("  the copy lands in the last 0.5 ms before every wake: then every re-run says so)");
            break;
        case it::PhaseVerdict::TooFew:
            emitS(Sev::Warn, "HA-214: only %lu ticks saw a change -- re-run.", ul(ph.seen));
            break;
        case it::PhaseVerdict::Fixed:
            emitS(Sev::Good, "HA-214: phase FIXED -- the copy lands ~%lu us after", ul(ph.medianUs));
            emitf("  the wake (|drift| and %.0f SE <= %.0f us/s over 1 s).", it::kDriftSigmas,
                  it::kFixedDriftUsPerS);
            emit("  Per boot: a re-run may differ.");
            break;
        case it::PhaseVerdict::Drifting:
            emitS(Sev::Warn, "HA-214: the phase DRIFTS %+.1f us/s -- it crosses", ph.driftUsPerS);
            emitf("  a whole 10 ms tick every ~%.0f s: every boot passes the bad band.",
                  it::secondsToCrossPeriod(ph.driftUsPerS, it::kCopyPeriodUs));
            break;
        case it::PhaseVerdict::NotSeparable:
            emitS(Sev::Warn, "HA-214: no verdict -- drift %+.1f +- %.1f us/s:", ph.driftUsPerS,
                  noise);
            emitf("  the wakes jitter too much to tell %.0f us/s from none", it::kFixedDriftUsPerS);
            emit("  in 1 s. Re-run; send the numbers either way.");
            break;
    }
}

void reportSpan(const it::Summary& spans, std::size_t motorReads, std::size_t motorErrors,
                it::SpanVerdict verdict) {
    emit("-- 2c.3 TICK SPAN, a stand-in: 100 ticks paced as 2c.2 (HA-215) --");
    emitf("  each: 2 IMU reads, %lu drive motors x 4 reads, the battery, 1 serial line",
          ul(kChassis.leftCount + kChassis.rightCount));
    emitf("  span, first IMU read to second: min %lu  median %lu  max %lu us", ul(spans.min),
          ul(spans.median), ul(spans.max));
    emit("  a LOWER bound on a motion tick's span: no Localizer arithmetic,");
    emit("  no record build, no motor command");
    emitf("  motor reads that returned PROS_ERR: %lu of %lu", ul(motorErrors), ul(motorReads));
    switch (verdict) {
        case it::SpanVerdict::NoTicks:
            emitS(Sev::Warn, "HA-215: nothing was measured.");
            break;
        case it::SpanVerdict::WellUnder:
            emitS(Sev::Good, "HA-215: well under the 5 ms hold (max %lu < %lu us);",
                  ul(spans.max), ul(it::kSpanWellUnderUs));
            emit("  a short stand-in is not proof -- the real tick does more.");
            break;
        case it::SpanVerdict::Close:
            emitS(Sev::Warn, "HA-215: longest stand-in tick %lu us -- over half", ul(spans.max));
            emit("  the 5 ms hold, and the real tick is longer still.");
            break;
        case it::SpanVerdict::Over:
            emitS(Sev::Bad, "HA-215 WRONG here: the stand-in spanned %lu us", ul(spans.max));
            emit("  -- at or over the 5 ms hold.");
            break;
    }
}

void reportGyro(const it::GyroTurn& turn, it::GyroVerdict verdict) {
    const double d = turn.rotationChangeDeg();
    const double z = turn.integratedZDeg();
    const double ratio = d != 0.0 ? std::fabs(z / d) : 0.0;
    emit("-- 2c.4 GYRO beside the rotation: a tight loop for 3 s (HA-04, HA-109) --");
    emitf("  polls %lu, dropped %lu (PROS_ERR), rotation changes %lu", ul(turn.polls()),
          ul(turn.badReads()), ul(turn.changes()));
    emitf("  window, first to last change: %.3f s", static_cast<double>(turn.windowUs()) * 1e-6);
    emitf("  rotation change  : %+9.2f deg (raw deg, PROS's rotation -- CW-positive per HA-02)", d);
    emitf("  gyro z integrated: %+9.2f deg (raw z, integrated -- trapezoid on micros())", z);
    if (d != 0.0) {
        emitf("  z / rotation: %+.4f   travel in the window: rising %.1f, falling %.1f deg",
              z / d, turn.risingDeg(), turn.fallingDeg());
    }
    if (it::mixedTurn(turn.risingDeg(), turn.fallingDeg())) {
        emit("  (the turn went BOTH ways -- fine here: both angles saw the same motion)");
    }
    switch (verdict) {
        case it::GyroVerdict::NoReads:
            emitS(Sev::Warn, "HA-04/HA-109: every poll was PROS_ERR -- no finding.");
            emit("  Check the IMU's cable and port (STAGE 1), and re-run.");
            break;
        case it::GyroVerdict::NoTurn:
            emitS(Sev::Warn, "HA-04/HA-109: %lu rotation change(s) -- no turn seen;",
                  ul(turn.changes()));
            emit("  turn it ONE way until the panel says STOP; re-run.");
            break;
        case it::GyroVerdict::TooShort:
            emitS(Sev::Warn, "HA-04/HA-109: turn spanned %.2f s (need >= %.0f s) --",
                  static_cast<double>(turn.windowUs()) * 1e-6,
                  static_cast<double>(it::kGyroMinWindowUs) * 1e-6);
            emit("  keep turning until the panel says STOP, and re-run.");
            break;
        case it::GyroVerdict::TooSmall:
            emitS(Sev::Warn, "HA-04/HA-109: turned only %.1f deg (need >= %.0f) --", std::fabs(d),
                  it::kGyroMinTurnDeg);
            emit("  turn it further (about a quarter turn a second), and re-run.");
            break;
        case it::GyroVerdict::Agree:
            emitS(Sev::Good, "HA-04 HOLDS: z has get_rotation()'s sign, so");
            emit("  GyroRateZ and the differentiated rate agree in sign");
            emit("  through ProsImu's one negate.");
            emitS(Sev::Good, "HA-109 HOLDS: the two agree within 10 percent --");
            emit("  z is deg/s about the yaw axis.");
            break;
        case it::GyroVerdict::OppositeSign:
            emitS(Sev::Bad, "HA-04 WRONG: z has the OPPOSITE sign to get_rotation()");
            emit("  -- GyroRateZ would answer the rate NEGATED: never");
            emit("  make it the default.");
            emitS(Sev::Good, "HA-109's scale holds: within 10 percent.");
            break;
        case it::GyroVerdict::RadiansSameSign:
            emitS(Sev::Bad, "HA-109 WRONG: |z| / |rotation| is %.4f, not ~1:", ratio);
            emit("  one degree's radians -- z is RADIANS per second.");
            emitS(Sev::Good, "HA-04 HOLDS: z has get_rotation()'s sign (a sign");
            emit("  does not depend on units).");
            break;
        case it::GyroVerdict::RadiansOppositeSign:
            emitS(Sev::Bad, "HA-109 WRONG: |z| / |rotation| is %.4f, not ~1:", ratio);
            emit("  one degree's radians -- z is RADIANS per second.");
            emitS(Sev::Bad, "HA-04 WRONG: z has the OPPOSITE sign to get_rotation()");
            emit("  -- GyroRateZ would answer the rate NEGATED.");
            break;
        case it::GyroVerdict::ScaleOff:
            emitS(Sev::Bad, "HA-109 WRONG: |z| / |rotation| is %.4f, not ~1.", ratio);
            emit("  (neither deg/s nor rad/s about the yaw axis: another axis --");
            emit("   the IMU's mounting -- or other units)");
            emitS(Sev::Warn, "HA-04: not judged -- z is not about the yaw axis.");
            break;
    }
    emit("  (whether get_rotation() itself is CW-positive is HA-02's: STAGE 2 above)");
}

}  // namespace imu2c

void imuTimingStage(std::uint8_t port) {
    namespace it = imu_timing;
    // STAGE 2's results stay on the panel until the operator has read them. Without this the
    // rule and prompt below would erase them the moment the HA-02 verdict printed; before IMU1
    // Part B they stayed up until TOUCH TO RETURN TO MENU (found on the resumed run's review).
    imu2c::foot(kColWarn, imu2c::kReadStage2);
    waitForTouch();  // STAGE 2's results stay readable until here
    // The rule and the intro go to serial + SD only: the prompt below erases the whole screen, so
    // painting them on the panel would only flash them (the IMU1-B review, 2026-11-05). The
    // prompt itself is the panel's instruction.
    g_screenActive = false;
    rule("STAGE 2c  IMU TIMING (HA-213/214/215; HA-04/HA-109)");
    emit("IMU1 Part B: the IMU's own timing, in four parts --");
    emit("  2c.1 cadence 2 s, 2c.2 phase 1 s: turn it SLOWLY, ONE way");
    emit("  2c.3 tick span 1 s: hands off");
    emit("  2c.4 gyro 3 s: ONE way, about a quarter turn a second");
    emit("It busy-reads the IMU: the panel freezes while it reads.");
    emit("It powers nothing, and reads raw PROS values only.");
    const bool start = twoButtonPrompt("STAGE 2c: THE IMU'S TIMING", "about 15 s, by hand; it powers nothing",
                                       "START", "then turn it by hand", "SKIP", "no 2c this run",
                                       "START, then turn the robot SLOWLY, ONE way, while",
                                       "the panel says TURN. It freezes while it reads.");
    // The panel stays the instrument from here (the scroll off); the log goes to serial + SD.
    if (!start) {
        g_screenActive = true;
        drawHeader("STAGE 2c  IMU TIMING");
        screenClear();
        emit("STAGE 2c SKIPPED by the operator -- nothing measured.");
        return;
    }
    emit("operator: START");

    // Every read the stage makes, named once: the raw rotation and the raw gyro z (R18).
    const auto rotation = [port] { return pros::c::imu_get_rotation(port); };
    const auto gyroZ = [port] { return pros::c::imu_get_gyro_rate(port).z; };

    // 2c.1 and 2c.2, one slow turn.
    imu2c::panel(imu2c::kTurnBig1, imu2c::kTurnBig2, imu2c::kTurnSmall1, imu2c::kTurnSmall2);
    imu2c::foot(kColWarn, "reading starts in 2 s -- hands on, start turning");
    pros::delay(2000);
    imu2c::foot(kColWarn, "READING -- keep turning (the panel is frozen)");
    pros::delay(20);  // let the panel draw before the loop holds the processor
    it::ChangeWatch cadenceWatch;
    const it::CadenceRun cadenceRun =
        it::runCadence(imu2c::nowUs, rotation, imu2c::kCadenceUs, cadenceWatch,
                       std::span<std::uint64_t>(imu2c::g_changeAt));
    std::uint32_t phasePrev = pros::millis();
    it::runPhase(imu2c::nowUs, [&phasePrev] { pros::Task::delay_until(&phasePrev, imu2c::kTickMs); },
                 rotation, std::span<it::TickPhase>(imu2c::g_ticks));

    // 2c.3, hands off.
    imu2c::panel(imu2c::kSpanBig1, "", imu2c::kSpanSmall1, imu2c::kSpanSmall2);
    pros::delay(20);
    std::size_t motorReads = 0;
    std::size_t motorErrors = 0;
    std::uint32_t spanPrev = pros::millis();
    it::runSpans(imu2c::nowUs, [&spanPrev] { pros::Task::delay_until(&spanPrev, imu2c::kTickMs); },
                 rotation,
                 [&motorReads, &motorErrors](std::size_t tick, double rot) {
                     imu2c::standInBody(tick, rot, motorReads, motorErrors);
                 },
                 std::span<std::uint32_t>(imu2c::g_spans));

    // 2c.4, a faster turn, after a touch.
    imu2c::panel(imu2c::kGyroBig1, imu2c::kGyroBig2, imu2c::kGyroSmall1, imu2c::kGyroSmall2);
    imu2c::foot(kColWarn, "TOUCH the screen to start");
    waitForTouch();
    emit("operator: touched for 2c.4");
    imu2c::foot(kColWarn, "reading starts in 2 s -- hands on, start turning");
    pros::delay(2000);
    imu2c::foot(kColWarn, "READING -- keep turning (the panel is frozen)");
    pros::delay(20);
    it::GyroTurn turn;
    it::runGyro(imu2c::nowUs, rotation, gyroZ, imu2c::kGyroUs, turn);
    imu2c::panel(imu2c::kDoneBig1, "", imu2c::kDoneSmall1, "");
    pros::delay(500);

    // The four result blocks, on the scroll again.
    g_screenActive = true;
    drawHeader("STAGE 2c  IMU TIMING");
    screenClear();
    const it::Cadence cad =
        it::cadenceFrom(std::span<const std::uint64_t>(imu2c::g_changeAt, cadenceRun.stored),
                        std::span<std::uint32_t>(imu2c::g_scratch));
    const it::CadenceVerdict cadVerdict = it::judgeCadence(cadenceWatch, cad.interval);
    imu2c::reportCadence(cadenceWatch, cad, cadenceRun.overflow, cadVerdict);

    const it::Phase ph = it::phaseFrom(std::span<const it::TickPhase>(imu2c::g_ticks),
                                       it::kCopyPeriodUs, std::span<std::uint32_t>(imu2c::g_scratch));
    imu2c::reportPhase(ph, it::judgePhase(ph, cadVerdict));

    const it::Summary spans = it::summarize(std::span<std::uint32_t>(imu2c::g_spans));
    imu2c::reportSpan(spans, motorReads, motorErrors, it::judgeSpan(spans));

    imu2c::reportGyro(turn, it::judgeGyro(turn));
}

// ═══ STAGE 3 — motors: raw beside canonical, one line each. ══════════════════
/// The cartridge each DEVICE reports. Read, never imposed.
const char* gearName(pros::motor_gearset_e_t g) {
    switch (g) {
        case pros::E_MOTOR_GEAR_RED:   return "RED 100";
        case pros::E_MOTOR_GEAR_GREEN: return "GRN 200";
        case pros::E_MOTOR_GEAR_BLUE:  return "BLU 600";
        default:                       return "UNKNOWN";
    }
}

/// NO MOTOR ADAPTER IS CONSTRUCTED HERE, AND THAT IS THE POINT.
///
/// The hal/pros motor adapter's constructor SETS the gearset on the device
/// (`motor_{port, toProsGears(gearset), degrees}`), and HA-98 records that motor
/// gearing lives IN THE DEVICE and PERSISTS ACROSS PROGRAMS. This file used to
/// construct it with an invented GREEN -- so a binary documented as READ-ONLY
/// would have silently rewritten every drive motor's cartridge configuration and
/// left it that way for every other program on the brain. If the fitted cartridges
/// are blue (600 rpm) and we stamp green (200 rpm), that is a 3x scaling error
/// handed to the next program to run, caused by the tool sent to MEASURE the robot.
///
/// So: raw values straight from the PROS C API, canonical values through the PURE
/// conversion function, and the cartridge READ BACK rather than asserted.
///
/// THE ONE EXCEPTION, deliberate and loud: station 10 DRIVE (R3b Session 2) DOES
/// construct the adapter, because driving through the adapter is the point of that
/// station -- and it prints the gearset it is about to WRITE as a belief first, and
/// refuses unless the chassis table names one. No other station constructs it.
void reportMotorGroup(const char* label, const std::int8_t* ports, std::size_t n,
                      const pros::c::v5_device_e_t* found) {
    emitf("-- %s --", label);
    for (std::size_t i = 0; i < n; ++i) {
        // |port|: the table's entries are SIGNED on a signed table, and this station reads
        // the raw device -- a negative port here would index found[] off the front and ask
        // PROS for a REVERSED reading, which is not what "raw" means.
        const int p = absPort(ports[i]);
        const std::int8_t raw = static_cast<std::int8_t>(p);
        if (found[p] != pros::c::E_DEVICE_MOTOR) {
            emitS(Sev::Bad, "  port %2d: NOT A MOTOR (census says %s)", p, deviceName(found[p]));
            continue;
        }
        const double rawDeg = pros::c::motor_get_position(raw);
        emitf("  p%-2d %s raw=%8.1fd canon=%7.3fr %4.1fC %4dmA", p,
              gearName(pros::c::motor_get_gearing(raw)), rawDeg,
              hal::motorPositionDegToCanonical(rawDeg).value(),
              pros::c::motor_get_temperature(raw),
              static_cast<int>(pros::c::motor_get_current_draw(raw)));
    }
}

void reportMotors(const pros::c::v5_device_e_t* found) {
    rule("STAGE 3  DRIVE MOTORS (HA-14/15/17/111)");
    emitS(Sev::Warn, "THE CARTRIDGE COLUMN IS NOT A MEASUREMENT OF THE HARDWARE.");
    emit("PROS exposes motor_SET_gearing(), so gearing is a SOFTWARE SETTING and");
    emit("motor_get_gearing() returns whatever a program last WROTE. A V5 motor");
    emit("cannot sense which cartridge is physically fitted. This column therefore");
    emit("says what the brain currently BELIEVES, which is a different fact and may");
    emit("even be a fact some earlier program invented.");
    emit("");
    emit("HA-15 CAN ONLY BE SETTLED BY LOOKING: the cartridge insert is visible");
    emit("through the motor housing. Read the colour off the motors themselves.");
    emit("A disagreement between the colour you see and the column below is a");
    emit("REAL FINDING -- it means the software is scaled wrong for the hardware.");
    emit("");
    emit("Nothing here writes to a motor: this STATION never constructs the adapter");
    emit("that would (header note above reportMotorGroup). Only station 10 DRIVE does.");
    emit("");
    if (tableHasPorts(kChassis)) {
        reportMotorGroup("TABLE: LEFT", kChassis.left, kChassis.leftCount, found);
        reportMotorGroup("TABLE: RIGHT", kChassis.right, kChassis.rightCount, found);
    } else {
        emitS(Sev::Warn, "chassis table has NO PORTS -- every census motor, side '?':");
        std::int8_t all[kMaxPort];
        std::size_t n = 0;
        for (int p = 1; p <= kMaxPort; ++p) {
            if (found[p] == pros::c::E_DEVICE_MOTOR) all[n++] = static_cast<std::int8_t>(p);
        }
        reportMotorGroup("EVERY CENSUS MOTOR (side ?)", all, n, found);
    }
    emit("");
    emit("TWO SEPARATE QUESTIONS. Do them in this order -- 'spin a wheel");
    emit("forward' is AMBIGUOUS on its own (forward for the wheel, or for");
    emit("the robot? seen from which side?), and a wrong reading here");
    emit("mirrors every turn the library will ever command.");
    emit("");
    emitS(Sev::Warn, "STEP 1 - WHICH WAY IS POSITIVE");
    emit("  a. Decide which end of the robot is its FRONT, and PHOTOGRAPH");
    emit("     it. The library's frame is +X forward, so this choice is");
    emit("     part of the measurement -- not an obvious fact.");
    emit("  b. Note every port's position above, then PUSH THE WHOLE ROBOT");
    emit("     FORWARD a foot or so, along the floor, front end leading.");
    emit("  c. Re-run this test. Every drive port moved. For EACH one write");
    emit("     down whether it went UP or DOWN.");
    emit("  Pushing the ROBOT removes the ambiguity: there is only one");
    emit("  forward for a robot, and no left/right or near/far side to it.");
    emit("");
    emitS(Sev::Warn, "STEP 2 - WHICH PORT IS WHICH WHEEL");
    emit("  a. Lift the robot so the wheels are off the ground.");
    emit("  b. Spin ONE wheel. Any direction -- it does not matter here.");
    emit("  c. Re-run. Exactly one port's number changed: that port drives");
    emit("     that wheel. Repeat per wheel.");
    emit("");
    emit("No motor is ever powered. Both steps are safe with the robot on a");
    emit("bench, and re-running this test is free.");
}

/// What the last MOTOR WATCH run concluded per port, and the SIGN CAPTURE the DRIVE
/// station consumes (R3b Session 2, brief §3.2). In-memory for ONE power cycle by
/// design -- persistence across boots is chunk R3d's, not this file's.
///
/// Two separate records, deliberately: `verdict[]` is the LAST run, whatever it was (a
/// single-wheel spin is a valid run that maps a port to a wheel); `sign[]` is written
/// ONLY by a qualifying whole-robot push -- every table port moved in one push -- so a
/// later single-wheel spin cannot revoke a good capture, and a partial push (one table
/// port silent) never yields signs at all. That is brief §3.3 gate 1's "moved count ==
/// table count", stated per port.
struct WatchCapture {
    bool ran = false;                        // any run completed this power cycle
    int moved = 0;                           // ports that moved in the LAST run
    std::int8_t verdict[kMaxPort + 1] = {};  // LAST run: +1 UP, -1 DOWN, 0 did not move
    bool signsValid = false;                 // a qualifying whole-robot push has happened
    bool signsFromBackwardPush = false;      // the operator said BACK first: readings inverted
    std::int8_t sign[kMaxPort + 1] = {};     // +1 positive port / -1 NEGATIVE port, for DRIVE
    int tableDisagreements = -1;             // vs a SIGNED table: -1 not compared, else the count
    char lastSummary[192] = "no MOTOR WATCH run yet this power cycle";
};
WatchCapture g_watch;

/// Which way the DRIVE station is being used: on blocks, or on the floor.
enum class DriveMode { WheelsUp, Ground };

/// What the DRIVE runs of this power cycle concluded -- the memory behind the TWO-STAGE
/// gate 2 (coordinator's correction to brief §3.3, recorded in the Session 2 log): the
/// first run in a power cycle must be wheels-up; a GROUND run is allowed only after a
/// wheels-up run that was CLEAN -- no cut, motors driven for at least 3 s, ceiling raised
/// to 6 V or more. In-memory for one power cycle, like the sign capture.
///
/// `cleanWheelsUp` is SET by a clean wheels-up run and CLEARED by any later run that CUT
/// (wheels-up or ground): a cut is new evidence of a fight, and the ground permission was
/// the claim that there is none. A later wheels-up run that is merely short or stays at 3 V
/// is not evidence against, and leaves the permission as it was.
struct DriveRecord {
    bool ran = false;                 // any DRIVE run completed this power cycle
    bool cleanWheelsUp = false;       // ground mode is unlocked
    DriveMode lastMode = DriveMode::WheelsUp;
    bool lastClean = false;
    char lastSummary[160] = "no DRIVE run yet this power cycle";
};
DriveRecord g_drive;

/// "+15 +16 -17 +18" for a side, from the sign capture; "?" marks a port with no sign.
void signedPortsString(const std::int8_t* ports, std::size_t n, char* buf, std::size_t cap) {
    buf[0] = '\0';
    for (std::size_t i = 0; i < n; ++i) {
        const int s = g_watch.sign[ports[i]];
        char one[8];
        std::snprintf(one, sizeof one, "%s%s%d", i ? " " : "", s > 0 ? "+" : s < 0 ? "-" : "?",
                      static_cast<int>(ports[i]));
        std::strncat(buf, one, cap - std::strlen(buf) - 1);
    }
}

/// MOTOR WATCH -- live capture, so nobody has to remember numbers.
///
/// The read-remember-re-run loop it replaces asked a person to note eight values,
/// perform a physical action, run the test again, and diff two tables in their
/// head. That is a transcription task handed to the one participant who cannot be
/// re-run. This zeroes a baseline, then displays every motor's DELTA live while the
/// robot is pushed or a wheel is spun, and writes the finished table to serial and
/// the SD card on exit.
///
/// Since R3b Session 2 it watches EVERY motor the census found (not a hypothesis
/// list -- up to 21, laid out for N), labels sides from the chassis table ('?' when
/// the table has no claim), and CAPTURES the verdict per port into g_watch. A
/// whole-robot push in which every table port moved is the only run that yields
/// SIGNS, and the panel then asks which way the robot was pushed rather than
/// assuming it (the IMU test's own 2026-08-19 lesson, in this file: two runs turned
/// opposite ways and nothing in the log said which).
///
/// Still READ-ONLY: positions are read straight from PROS, no adapter is
/// constructed, and no motor is ever powered here.
void motorWatch(pros::c::v5_device_e_t* found) {
    rule("MOTOR WATCH (live)");

    // Every motor in the census, in port order. Sides from the table when it has them.
    std::int8_t ports[kMaxPort];
    const char* side[kMaxPort];
    std::size_t n = 0;
    for (int p = 1; p <= kMaxPort; ++p) {
        if (found[p] == pros::c::E_DEVICE_MOTOR) {
            ports[n] = static_cast<std::int8_t>(p);
            side[n] = sideLetter(p);
            ++n;
        }
    }
    if (n == 0) {
        emitS(Sev::Bad, "no motors in the census -- run test 1 first, or check the cables.");
        return;
    }

    double base[kMaxPort];
    for (std::size_t i = 0; i < n; ++i) base[i] = pros::c::motor_get_position(ports[i]);

    emitf("Baseline taken over %u motor(s). Now do ONE of these:", static_cast<unsigned>(n));
    emit("  * PUSH THE WHOLE ROBOT, FRONT END LEADING -> every drive port moves;");
    emit("    the SIGNS are 'which way is positive', and DRIVE uses them as-is.");
    emit("    (UP and DOWN mixed WITHIN a side is NORMAL on a coupled gear train:");
    emit("    adjacent motors on one train spin opposite ways. Not a wiring fault.)");
    emit("  * SPIN ONE WHEEL (any direction) -> only that port moves; that is");
    emit("    the port->wheel map. It yields NO signs, by design.");
    emit("Watch the panel. TOUCH when you are done and it records the table.");

    g_screenActive = false;
    screenClear();
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 40,
                             "push the robot FRONT-first, or spin one wheel");
    pros::c::screen_set_pen(kColWarn);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY, "TOUCH when done - it records itself");

    // Live table, laid out for N, updated in place until a touch arrives.
    const Grid g = gridFor(n, 60, false);
    const std::int32_t startTouch = pros::c::screen_touch_status().release_count;
    char cell[40];
    while (pros::c::screen_touch_status().release_count == startTouch) {
        for (std::size_t i = 0; i < n; ++i) {
            const double d = pros::c::motor_get_position(ports[i]) - base[i];
            // Colour IS the reading: green rose, red fell, dim did not move.
            std::snprintf(cell, sizeof cell, "%s%-2d %s%8.0f", side[i], static_cast<int>(ports[i]),
                          d > 5.0 ? "UP  " : d < -5.0 ? "DOWN" : "--  ", d);
            gridCell(g, i, d > 5.0 ? kColGood : d < -5.0 ? kColBad : kColDim, cell);
        }
        pros::delay(80);
    }

    // ── record the run: the LAST-run verdict for every port, plus the bookkeeping that
    //    decides whether this was a qualifying whole-robot push ─────────────────────────
    double delta[kMaxPort];
    g_watch.ran = true;
    g_watch.moved = 0;
    for (int p = 0; p <= kMaxPort; ++p) g_watch.verdict[p] = 0;
    int tableMoved = 0, tableSilent = 0, nonTableMoved = 0;
    char silent[64] = "";
    auto noteSilent = [&](int port) {
        ++tableSilent;
        char one[8];
        std::snprintf(one, sizeof one, " %d", port);
        std::strncat(silent, one, sizeof silent - std::strlen(silent) - 1);
    };
    for (std::size_t i = 0; i < n; ++i) {
        delta[i] = pros::c::motor_get_position(ports[i]) - base[i];
        const std::int8_t v = delta[i] > 5.0 ? 1 : delta[i] < -5.0 ? -1 : 0;
        g_watch.verdict[ports[i]] = v;
        if (v != 0) ++g_watch.moved;
        if (tableSideOf(kChassis, ports[i]) != 0) {
            if (v != 0) ++tableMoved; else noteSilent(ports[i]);
        } else if (v != 0) {
            ++nonTableMoved;
        }
    }
    // A table port that is not a census motor cannot have moved: silent by definition.
    for (std::size_t i = 0; i < kChassis.leftCount; ++i) {
        if (found[absPort(kChassis.left[i])] != pros::c::E_DEVICE_MOTOR) noteSilent(absPort(kChassis.left[i]));
    }
    for (std::size_t i = 0; i < kChassis.rightCount; ++i) {
        if (found[absPort(kChassis.right[i])] != pros::c::E_DEVICE_MOTOR) noteSilent(absPort(kChassis.right[i]));
    }
    const bool qualifies = tableHasPorts(kChassis) && tableSilent == 0
                           && tableMoved == static_cast<int>(tableCount(kChassis));

    bool backFirst = false;
    if (qualifies) {
        // ASK which way the robot was pushed -- record the action, never assume it. A
        // back-first push inverts every sign, and DRIVE's gate 5 could NOT see that later:
        // the members would still agree with each other, all the wrong way.
        backFirst = !twoButtonPrompt("WHICH WAY DID YOU PUSH IT?",
                                     "the end the build team chose as the FRONT ...",
                                     "FRONT FIRST", "front end led", "BACK FIRST",
                                     "back end led",
                                     "signs are captured for DRIVE either way;",
                                     "not sure? re-run and push again, one way only.");
        g_watch.signsValid = true;
        g_watch.signsFromBackwardPush = backFirst;
        for (int p = 1; p <= kMaxPort; ++p) {
            g_watch.sign[p] = 0;
            if (tableSideOf(kChassis, p) != 0) {
                const std::int8_t v = g_watch.verdict[p];
                g_watch.sign[p] = backFirst ? static_cast<std::int8_t>(-v) : v;
            }
        }
    }

    g_screenActive = true;
    screenClear();
    emit("");
    emit("RECORDED -- net movement per port since baseline:");
    emit("side port      delta deg   verdict");
    for (std::size_t i = 0; i < n; ++i) {
        const double d = delta[i];
        const bool did = (d > 5.0 || d < -5.0);
        emitS(did ? Sev::Good : Sev::Info, "  %s  %2d  %10.0f   %s", side[i],
              static_cast<int>(ports[i]), d, d > 5.0 ? "UP" : d < -5.0 ? "DOWN" : "did not move");
    }
    emit("");
    if (g_watch.moved == 0) {
        emitS(Sev::Warn, "nothing moved. Push harder, or check the census.");
        std::snprintf(g_watch.lastSummary, sizeof g_watch.lastSummary, "last run: nothing moved");
    } else if (g_watch.moved == 1) {
        emit("ONE port moved -> that port drives the wheel you spun. NO signs captured:");
        emit("a single-wheel spin cannot say which way is forward for the ROBOT.");
        int which = 0;
        for (std::size_t i = 0; i < n; ++i) if (g_watch.verdict[ports[i]] != 0) which = ports[i];
        std::snprintf(g_watch.lastSummary, sizeof g_watch.lastSummary,
                      "last run: single-wheel spin (port %d) -- no signs", which);
    } else if (qualifies) {
        emitS(Sev::Good, "WHOLE-ROBOT PUSH, %s-first: every table port moved -> SIGNS CAPTURED.",
              backFirst ? "BACK" : "FRONT");
        if (backFirst) emit("  (back-first: every reading INVERTED to the front-first convention.)");
        emit("  DRIVE constructs each DOWN port with a NEGATIVE port number -- PROS reverses");
        emit("  it once, there, the ONE place a sign lives. Mixed UP/DOWN within a side is");
        emit("  normal on a coupled gear train; a port disagreeing with a side-mate is not");
        emit("  a fault here, it is a sign.");
        char l[80], r[80];
        signedPortsString(kChassis.left, kChassis.leftCount, l, sizeof l);
        signedPortsString(kChassis.right, kChassis.rightCount, r, sizeof r);
        emitf("  signed ports for DRIVE: LEFT %s | RIGHT %s", l, r);
        if (nonTableMoved > 0) {
            emitf("  also moved, NOT in the table (no sign taken): %d port(s)", nonTableMoved);
        }
        // ── THE CROSS-CHECK against a SIGNED table (R3b Part 0b, brief §2): the table's signs
        //    are typed, measured values with a date; this push is a fresh measurement. Per port,
        //    AGREES or DISAGREES. Any DISAGREES is a finding -- a motor re-wired, a cable moved to
        //    another port, or a wrong entry -- and DRIVE refuses on it, naming the port.
        g_watch.tableDisagreements = -1;
        if (kChassis.signsMeasured) {
            int disagree = 0;
            emit("  vs the table's MEASURED signs (chassis_table.hpp):");
            for (int p = 1; p <= kMaxPort; ++p) {
                const int ts = tableSignOf(kChassis, p);
                if (ts == 0) continue;
                const bool agrees = ts == g_watch.sign[p];
                if (!agrees) ++disagree;
                emitS(agrees ? Sev::Good : Sev::Bad, "    port %2d  table %c  push %c  %s", p,
                      ts > 0 ? '+' : '-', g_watch.sign[p] > 0 ? '+' : '-',
                      agrees ? "AGREES" : "DISAGREES");
            }
            g_watch.tableDisagreements = disagree;
            if (disagree == 0) {
                emitS(Sev::Good, "  every table port AGREES with this push.");
            } else {
                emitS(Sev::Bad, "  %d port(s) DISAGREE with the table. DRIVE will refuse. A finding:",
                      disagree);
                emit("  a re-wired motor, a moved cable, or a wrong table entry. Do not drive it");
                emit("  until the table and the robot agree.");
            }
        }
        std::snprintf(g_watch.lastSummary, sizeof g_watch.lastSummary,
                      "whole-robot push (%s-first): signs captured for all %u table ports%s",
                      backFirst ? "back" : "front", static_cast<unsigned>(tableCount(kChassis)),
                      g_watch.tableDisagreements > 0 ? " -- DISAGREES with the table"
                      : g_watch.tableDisagreements == 0 ? " -- agrees with the table" : "");
    } else if (!tableHasPorts(kChassis)) {
        emitS(Sev::Warn, "SEVERAL moved, but the chassis table has NO PORTS -- signs cannot be");
        emit("captured (there is nothing to attach them to). The UP/DOWN column above IS");
        emit("the sign convention for the front you pushed toward: report it WITH the port");
        emit("numbers per side, and it goes into src/robots/chassis_table.hpp.");
        std::snprintf(g_watch.lastSummary, sizeof g_watch.lastSummary,
                      "several moved; table has no ports -- no signs");
    } else {
        emitS(Sev::Warn, "PARTIAL: %d of %u table ports moved; silent:%s", tableMoved,
              static_cast<unsigned>(tableCount(kChassis)), silent);
        emit("No sign capture: DRIVE needs EVERY table port to move in ONE push. Push again,");
        emit("all wheels on the floor, firmer. A port that never moves is a finding.");
        std::snprintf(g_watch.lastSummary, sizeof g_watch.lastSummary,
                      "partial push: %d/%u table ports moved (silent:%s) -- no signs", tableMoved,
                      static_cast<unsigned>(tableCount(kChassis)), silent);
    }
    emit("This table is in /usd/r3a_log.txt. Re-run per wheel to build the map.");
}

// ═══ STATION 10 — DRIVE: THE FIRST STATION THAT POWERS MOTORS (R3b Session 2, §3.3) ════
// Hands-on, open-loop, through the library's ADAPTERS -- hal::pros::ProsMotor (signed
// ports from MOTOR WATCH's capture; the table's cartridge, which the ctor WRITES) and
// hal::pros::ProsController -- never through the motion stack. The first time either
// adapter runs on hardware; the run measures HA-94 onward. Everything else in this
// binary is read-only but station 13 MECHANISMS (MECH-A6): these two are the ONLY code paths
// that call setVoltage — this one on the drive's ProsMotors, station 13 on the declared
// mechanisms' (through robot::MechanismParts and its own six gates).
//
// SIX SAFETY GATES, in this order -- a station that skips one is the defect:
//   1. REFUSE unless the chassis table has ports and a cartridge and is consistent,
//      every table port is a census motor, and the SIGNS are known: from the TABLE when it
//      carries measured signs (robot two, since R3b Part 0b), otherwise from MOTOR WATCH's
//      capture of a WHOLE-ROBOT push this power cycle (the bench bot). When both exist and
//      disagree on any port, refuse and name the port. Names what is missing.
//   2. WHEELS OFF THE GROUND confirmed on the panel before the first volt -- TWO-STAGE
//      (the coordinator's correction, Session 2 log): the FIRST run in a power cycle must
//      be wheels-up. Answering NO opens GROUND mode only if a wheels-up run this power
//      cycle was CLEAN (no cut, >= 3 s driven, ceiling >= 6 V -- g_drive) AND a second
//      explicit confirmation ("ON THE GROUND? clear 3 m all round, a second person at the
//      battery") is answered YES; otherwise it refuses and names the reason. Ground mode
//      changes nothing else: ceiling resets to 3 V, gates 3-6 as below, the panel header
//      and the controller LCD say GROUND, the exit summary and log say which mode ran.
//   3. DEAD-MAN: motors are driven only while L1 is HELD (left index finger; thumbs stay
//      on the sticks; physically on the controller's top edge, not near the sticks).
//      Release, or a controller drop, -> 0 V, coast, that same tick.
//   4. VOLTAGE CEILING starts at 3 V; R1 (rising edge via hal::ButtonEdge -- its first
//      hardware use) steps it 3 -> 6 -> 9 -> 12 V and stops at 12. Shown on the panel AND
//      the controller LCD. Never starts above 3 V; re-entering the station resets it.
//   5. THE FIGHTING-MOTOR CUT-OUT -- the coupled-drivetrain hazard: five motors on one
//      gear train, and a wrong sign on one stalls it against the other four. While a
//      side is commanded above 1 V and its fastest member turns above 1 rad/s, a member
//      whose velocity sign is OPPOSITE the side's command, or whose |velocity| is under
//      25 % of the side's fastest, is disagreeing; 250 ms of that -- the same window as
//      the over-current cut, and long enough to survive a stick reversal while the
//      members coast the old way -- cuts the WHOLE drive to 0 V, names the port(s) and
//      the reason on every screen, and stays cut until the station is re-entered. A
//      whole side disagreeing is reported as "side runs OPPOSITE to the command": that
//      is a back-first push or a wrong front, not a fight, and it cuts too. Also cut:
//      any member above 2.4 A for 250 ms. Current and temperature per port, live.
//   6. EXIT (touch) -> every motor 0 V, coast -- and an RAII guard does the same on EVERY
//      other way out (a thrown precondition, a controller loss, the cut itself).
//
// Every threshold here is INVENTED for a first run and the panel says so; none is a
// measurement. Mapping: arcade through the same shulib::teleop::mapSticks() the library
// teleop loop uses -- left stick Y forward, right stick X yaw -- so the driver feels ONE
// robot in both programs. Per side: ceiling x (forward -/+ yawCcw), clamped to the
// ceiling, the SAME volts to every member of the side (a group is a voltage fan-out).
//
// HOW TO READ A CUT: if the ports named are exactly the ones constructed with a NEGATIVE
// port, then the vendored SDK's promise that reversal applies to telemetry too
// (pros/motors.hpp:51-52) did not hold on this firmware -- a register finding, not a
// wiring fault. A subset of a side named = that subset is signed wrong or mechanically
// fighting. A whole side named = the push was backwards, or the front is wrong.
namespace drive {
constexpr double kCeilingStartV = 3.0;
constexpr double kCeilingStepV = 3.0;
constexpr double kCeilingMaxV = 12.0;
constexpr double kMovingFloorRadS = 1.0;     // the panel's "moving" colour threshold; gate 5's own
                                             // thresholds live in teleop::SideMonitorConfig (below)
constexpr double kCurrentLimitA = 2.4;
constexpr double kCleanDrivenSeconds = 3.0;  // a wheels-up run must drive this long...
constexpr double kCleanCeilingV = 6.0;       // ...and reach this ceiling to unlock GROUND mode
constexpr int kTickMs = 10;
constexpr int kPersistTicks = 25;   // 250 ms, for the fight AND the over-current cut
constexpr int kPanelEveryTicks = 10;
constexpr int kLcdEveryTicks = 20;  // the firmware rate-limits LCD writes; 200 ms, on change
constexpr hal::ControllerButton kDeadMan = hal::ControllerButton::L1;
constexpr hal::ControllerButton kCeilingUp = hal::ControllerButton::R1;

// Gate 5's numbers are the shared evaluator's DEFAULTS (R3b Part 0b): pinned here so the
// station's documented thresholds (header comment above) and the header cannot drift apart
// without this file failing to compile.
static_assert(teleop::SideMonitorConfig{}.commandFloorV == 1.0);
static_assert(teleop::SideMonitorConfig{}.movingFloorRadS == kMovingFloorRadS);
static_assert(teleop::SideMonitorConfig{}.oppositeFloorRadS == 0.5);
static_assert(teleop::SideMonitorConfig{}.nearZeroFraction == 0.25);
static_assert(teleop::SideMonitorConfig{}.persistTicks == kPersistTicks);

hal::pros::MotorGearset gearsetFor(Cartridge c) {
    switch (c) {
        case Cartridge::Red:   return hal::pros::MotorGearset::Red;
        case Cartridge::Green: return hal::pros::MotorGearset::Green;
        default:               return hal::pros::MotorGearset::Blue;  // Unset never gets here (gate 1)
    }
}

struct Member {
    int port = 0;
    int side = 0;                               // -1 LEFT, +1 RIGHT
    std::int8_t signedPort = 0;                 // THE one place a sign lives
    std::optional<hal::pros::ProsMotor> motor;  // constructed in place, inside a try
    int disagreeTicks = 0;
    int overTicks = 0;
    double v = 0.0, amps = 0.0, temp = 0.0;     // this tick's readings
    double maxAmps = 0.0, maxTemp = 0.0;
};

/// 0 V + Coast on every constructed member. Called explicitly at the cut and at exit,
/// and by the guard's destructor on every other way out (gate 6).
void stopAll(Member* m, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        if (m[i].motor) {
            m[i].motor->setVoltage(units::Voltage{0.0});
            m[i].motor->setBrakeMode(hal::BrakeMode::Coast);
        }
    }
}

struct AllStopGuard {
    Member* m;
    const std::size_t* n;
    ~AllStopGuard() { stopAll(m, *n); }
};
}  // namespace drive

void driveStation(pros::c::v5_device_e_t* found) {
    using namespace drive;
    rule("DRIVE -- POWERS MOTORS (adapters, NOT the motion stack)");
    emitS(Sev::Warn, "THIS STATION POWERS THE DRIVE MOTORS. Every other station is read-only.");
    emit("hal::pros::ProsMotor + ProsController, open-loop volts, six gates. The library's");
    emit("motion stack is NOT in this loop -- the library has still never driven a robot.");
    emit("");

    // ── GATE 1: the table, the census, and the sign capture ──────────────────────
    char missing[96];
    if (describeMissing(kChassis, missing, sizeof missing, true)) {
        emitS(Sev::Bad, "REFUSED: chassis table UNSET -- missing: %s.", missing);
        emit("  Nothing is guessed. Fill kChassis in src/robots/chassis_table.hpp from the build team's");
        emit("  report (ports per side + the front they chose), rebuild, upload, re-run.");
        return;
    }
    char why[96];
    if (!tableConsistent(kChassis, why, sizeof why)) {
        emitS(Sev::Bad, "REFUSED: the chassis table contradicts itself -- %s.", why);
        return;
    }
    for (int p = 1; p <= kMaxPort; ++p) {
        if (tableSideOf(kChassis, p) != 0 && found[p] != pros::c::E_DEVICE_MOTOR) {
            emitS(Sev::Bad, "REFUSED: table port %d is not a motor in the census (%s).", p,
                  deviceName(found[p]));
            emit("  Re-run 1 DEVICE CENSUS; a table port absent from it is a finding.");
            return;
        }
    }
    // Where the signs come from (R3b Part 0b, brief §2): the TABLE when it carries measured
    // signs, MOTOR WATCH's capture otherwise. When both exist they must agree on every port,
    // or this station refuses and names the port -- a table that lies about a sign is exactly
    // the fight the cut-out exists for, and it is cheaper to refuse than to cut.
    const bool signsFromTable = kChassis.signsMeasured;
    if (!signsFromTable) {
        if (!g_watch.signsValid) {
            emitS(Sev::Bad, "REFUSED: no sign capture from a WHOLE-ROBOT push this power cycle.");
            emitf("  MOTOR WATCH says: %s", g_watch.lastSummary);
            emit("  Run 3 MOTOR WATCH: push the whole robot front-first so EVERY table port");
            emit("  moves, answer FRONT FIRST, then come back here. (This robot's table carries");
            emit("  no signs.)");
            return;
        }
        for (int p = 1; p <= kMaxPort; ++p) {
            if (tableSideOf(kChassis, p) != 0 && g_watch.sign[p] == 0) {
                emitS(Sev::Bad, "REFUSED: no sign captured for table port %d.", p);
                return;
            }
        }
    } else if (g_watch.signsValid) {
        for (int p = 1; p <= kMaxPort; ++p) {
            const int ts = tableSignOf(kChassis, p);
            if (ts != 0 && g_watch.sign[p] != 0 && g_watch.sign[p] != ts) {
                emitS(Sev::Bad, "REFUSED: port %d -- the table says %c, this power cycle's push "
                                "said %c.", p, ts > 0 ? '+' : '-', g_watch.sign[p] > 0 ? '+' : '-');
                emit("  The table's sign is a typed measurement with a date; the push is a fresh");
                emit("  one. They disagree, so one of them is wrong about THIS robot today: a");
                emit("  re-wired motor, a moved cable, or a bad entry. Fix the table or the robot.");
                return;
            }
        }
    }

    // ── the belief, loud, BEFORE anything is written to a motor ──────────────────
    emitS(Sev::Warn, "CARTRIDGE BELIEF: %s", cartridgeWord(kChassis.cartridge));
    emitf("  (%s)", kChassis.provenance);
    emit("  ProsMotor's constructor WRITES this gearset to every table motor and reads it");
    emit("  back; a WRONG belief passes that read-back. Volts are unaffected; the rad/s");
    emit("  shown scale by it. If the insert colour is not this: STOP, fix the table.");
    char l[80], r[80];
    if (signsFromTable) {
        portsToString(kChassis.left, kChassis.leftCount, true, l, sizeof l);
        portsToString(kChassis.right, kChassis.rightCount, true, r, sizeof r);
        emitf("signed ports: LEFT %s | RIGHT %s", l, r);
        emitf("  (- = reversed by PROS, from the TABLE's measured signs%s)",
              g_watch.signsValid ? "; this power cycle's push AGREES" : "; no push this power cycle");
    } else {
        signedPortsString(kChassis.left, kChassis.leftCount, l, sizeof l);
        signedPortsString(kChassis.right, kChassis.rightCount, r, sizeof r);
        emitf("signed ports: LEFT %s | RIGHT %s", l, r);
        emitf("  (- = reversed by PROS, from MOTOR WATCH%s)",
              g_watch.signsFromBackwardPush ? "; back-first push, readings inverted" : "");
    }
    emit("thresholds (INVENTED, first run): fight = >1V cmd and >1rad/s, opposite sign or");
    emit("  <25% of side max for 250ms; over-current 2.4A for 250ms; ceiling 3/6/9/12V.");

    // ── GATE 2, TWO-STAGE: wheels-up, or ground only after a CLEAN wheels-up run ─────
    DriveMode mode = DriveMode::WheelsUp;
    const bool wheelsUp = twoButtonPrompt(
        "ARE THE WHEELS OFF THE GROUND?", "robot on blocks: no wheel may touch anything",
        "YES, WHEELS UP", "power the motors", "NO", "it is on the floor",
        "this station POWERS the motors. Hold L1 to drive,",
        "R1 raises the 3V ceiling, TOUCH the screen to stop.");
    if (!wheelsUp) {
        if (!g_drive.cleanWheelsUp) {
            g_screenActive = true;
            screenClear();
            emitS(Sev::Bad, "REFUSED: ground driving needs a clean wheels-up run at >= 6 V "
                            "this power cycle first.");
            emitf("  DRIVE says: %s", g_drive.lastSummary);
            emit("  Clean = no cut, motors driven for at least 3 s, ceiling raised to 6 V or");
            emit("  more. Put the robot on blocks, run this station wheels-up, then come back.");
            return;
        }
        const bool ground = twoButtonPrompt(
            "ON THE GROUND?", "clear 3 m all round, a second person at the battery",
            "YES, GROUND MODE", "3 V first, then 6 V", "NO", "back to the menu",
            "ground mode: same six gates, ceiling resets to 3 V;",
            "above 6 V only with the team lead present.");
        if (!ground) {
            g_screenActive = true;
            screenClear();
            emitS(Sev::Warn, "not driven: ground mode was not confirmed.");
            return;
        }
        mode = DriveMode::Ground;
    }
    const char* const modeWord = mode == DriveMode::Ground ? "GROUND" : "WHEELS UP";
    emitS(Sev::Warn, "mode: %s%s", modeWord,
          mode == DriveMode::Ground ? " -- unlocked by a clean wheels-up run this power cycle"
                                    : "");

    // ── construct the adapters: the first time ProsMotor runs on hardware ─────────
    Member members[kMaxPort];
    std::size_t n = 0;
    AllStopGuard guard{members, &n};  // gate 6 on every exit path from here on
    const hal::pros::MotorGearset gearset = gearsetFor(kChassis.cartridge);
    for (int p = 1; p <= kMaxPort; ++p) {
        const int side = tableSideOf(kChassis, p);
        if (side == 0) continue;
        Member& m = members[n];
        m.port = p;
        m.side = side;
        // THE one place a sign lives: the table's entry AS TYPED when the table is signed;
        // otherwise the capture's verdict applied to the port number. Nothing after this
        // line negates anything.
        m.signedPort = signsFromTable ? tableSignedPort(kChassis, p)
                                      : static_cast<std::int8_t>(g_watch.sign[p] > 0 ? p : -p);
        try {
            m.motor.emplace(m.signedPort, gearset);
        } catch (const PreconditionError& e) {
            g_screenActive = true;
            screenClear();
            emitS(Sev::Bad, "MOTOR ADAPTER REFUSED port %d (signed %d): %s", p,
                  static_cast<int>(m.signedPort), e.what());
            emit("  (that refusal IS the measurement -- record it verbatim; nothing was powered)");
            return;
        }
        ++n;
    }
    stopAll(members, n);  // 0 V + Coast is the declared starting state, not an inherited one

    hal::pros::ProsController master{hal::pros::ControllerId::Master};
    hal::pros::ProsLineDisplay lcd{};
    if (!master.isConnected()) {
        g_screenActive = true;
        screenClear();
        emitS(Sev::Bad, "REFUSED: controller NOT CONNECTED -- pair it (test 4), re-enter.");
        return;
    }
    emitS(Sev::Good, "%u motors constructed through ProsMotor, gearset written and read back OK.",
          static_cast<unsigned>(n));
    emitf("driving, %s. Hold L1; R1 raises the ceiling; TOUCH the panel to stop.", modeWord);

    // ── the panel: fixed layout, the log suspended (the prompt erased the header bar) ──
    g_screenActive = false;
    drawHeader(mode == DriveMode::Ground ? "10 DRIVE -- GROUND MODE" : "10 DRIVE -- WHEELS UP");
    screenClear();
    pros::c::screen_set_pen(mode == DriveMode::Ground ? kColWarn : kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 40, "%s",
                             mode == DriveMode::Ground
                                 ? "GROUND MODE: L1 = drive  R1 = raise  TOUCH = EXIT (0 V)"
                                 : "HOLD L1 = drive   R1 = raise ceiling   TOUCH = EXIT (0 V)");
    const char* const lcdMode = mode == DriveMode::Ground ? "GROUND" : "DRIVE";
    const Grid g = gridFor(n, 84, true);

    // Gate 5's evaluators, one per side, over the members of that side in port order (the
    // bit positions in each verdict index sideIdx[s]).
    teleop::CoupledSideMonitor monitor[2];
    std::size_t sideIdx[2][kMaxPort];
    std::size_t sideN[2] = {0, 0};
    for (std::size_t i = 0; i < n; ++i) {
        const int s = members[i].side < 0 ? 0 : 1;
        sideIdx[s][sideN[s]++] = i;
    }

    hal::ButtonEdge ceilingEdge;
    double ceiling = kCeilingStartV;
    bool cut = false;
    char cutWhy[120] = "";
    const std::int32_t touch0 = pros::c::screen_touch_status().release_count;
    int tick = 0, drivenTicks = 0;
    double peakAbsCmd = 0.0;
    char lcdRow[3][24] = {"", "", ""};
    char big[64], cell[48];

    for (;;) {
        if (pros::c::screen_touch_status().release_count != touch0) break;  // gate 6

        const bool connected = master.isConnected();
        teleop::StickInput sticks{.connected = connected};
        if (connected) {
            sticks.leftY = master.axis(hal::ControllerAxis::LeftY);
            sticks.leftX = master.axis(hal::ControllerAxis::LeftX);
            sticks.rightX = master.axis(hal::ControllerAxis::RightX);
        }
        const teleop::DriveRequest req = teleop::mapSticks(sticks);  // zero when disconnected

        // GATE 4: the ceiling steps on R1's rising edge, never above 12 V.
        if (ceilingEdge.update(connected && master.pressed(kCeilingUp))) {
            ceiling = std::min(ceiling + kCeilingStepV, kCeilingMaxV);
            emitf("ceiling -> %.0f V", ceiling);
        }
        // GATE 3: dead-man. Not held, or controller gone, or cut -> 0 V this tick.
        const bool deadMan = connected && master.pressed(kDeadMan);
        double vL = 0.0, vR = 0.0;
        if (deadMan && !cut) {
            // The ONE arcade arithmetic (the old drive program's too; host-pinned):
            // ceiling x (forward -/+ yawCcw), clamped to the ceiling.
            const teleop::SideVolts sv = teleop::tankSideVolts(req, ceiling);
            vL = sv.left;
            vR = sv.right;
        }
        for (std::size_t i = 0; i < n; ++i) {
            members[i].motor->setVoltage(units::Voltage{members[i].side < 0 ? vL : vR});
        }
        if (vL != 0.0 || vR != 0.0) ++drivenTicks;
        peakAbsCmd = std::max(peakAbsCmd, std::max(std::abs(vL), std::abs(vR)));

        for (std::size_t i = 0; i < n; ++i) {
            Member& m = members[i];
            m.v = m.motor->velocity().value();
            m.amps = m.motor->current().value();
            m.temp = m.motor->temperature();
            m.maxAmps = std::max(m.maxAmps, m.amps);
            m.maxTemp = std::max(m.maxTemp, m.temp);
        }

        // GATE 5: the fighting-motor cut-out -- through the SHARED pure evaluator
        // (shulib/teleop/coupled_side_monitor.hpp, host-tested; R3b Part 0b extracted this
        // block into it, same thresholds, same 250 ms persistence) -- then the over-current cut.
        if (!cut) {
            for (int s = 0; s < 2; ++s) {
                const int side = s == 0 ? -1 : +1;
                const double cmd = side < 0 ? vL : vR;
                teleop::MemberSample samples[kMaxPort];
                for (std::size_t j = 0; j < sideN[s]; ++j) {
                    samples[j] = teleop::MemberSample{members[sideIdx[s][j]].v, true};
                }
                const teleop::SideVerdict verdict = monitor[s].update(
                    cmd, std::span<const teleop::MemberSample>{samples, sideN[s]});
                const int count = static_cast<int>(sideN[s]);
                const int persisted = verdict.persistedCount;
                char names[48] = "";
                for (std::size_t j = 0; j < sideN[s]; ++j) {
                    Member& m = members[sideIdx[s][j]];
                    m.disagreeTicks = monitor[s].disagreeTicks(j);  // mirrored for the panel
                    if (verdict.persistedMask & (std::uint32_t{1} << j)) {
                        char one[8];
                        std::snprintf(one, sizeof one, " %d", m.port);
                        std::strncat(names, one, sizeof names - std::strlen(names) - 1);
                    }
                }
                if (persisted > 0 && !cut) {
                    cut = true;
                    if (persisted == count) {
                        std::snprintf(cutWhy, sizeof cutWhy,
                                      "CUT: whole %s side ran OPPOSITE/stalled vs its command "
                                      "-- back-first push, or the front is wrong?",
                                      side < 0 ? "LEFT" : "RIGHT");
                    } else {
                        std::snprintf(cutWhy, sizeof cutWhy,
                                      "CUT: FIGHTING on %s -- port(s)%s disagree with side-mates",
                                      side < 0 ? "LEFT" : "RIGHT", names);
                    }
                }
            }
            for (std::size_t i = 0; i < n; ++i) {
                Member& m = members[i];
                m.overTicks = m.amps > kCurrentLimitA ? m.overTicks + 1 : 0;
                if (m.overTicks >= kPersistTicks && !cut) {
                    cut = true;
                    std::snprintf(cutWhy, sizeof cutWhy, "CUT: port %d over %.1f A for 250 ms",
                                  m.port, kCurrentLimitA);
                }
            }
            if (cut) {
                stopAll(members, n);
                emitS(Sev::Bad, "%s", cutWhy);
                emit("  the drive stays CUT until this station is re-entered (TOUCH to exit).");
                for (std::size_t i = 0; i < n; ++i) {
                    emitf("  at the cut: port %2d signed %+3d  v=%+7.2f rad/s  %5.2f A  %3.0f C",
                          members[i].port, static_cast<int>(members[i].signedPort), members[i].v,
                          members[i].amps, members[i].temp);
                }
            }
        }

        // The panel every 100 ms; the controller LCD every 200 ms, and only on change.
        if (tick % kPanelEveryTicks == 0) {
            std::snprintf(big, sizeof big, "CEILING %2.0f V   L %+5.1f V   R %+5.1f V", ceiling,
                          vL, vR);
            pros::c::screen_set_eraser(kColBg);
            pros::c::screen_erase_rect(0, 54, kUsableW, 82);
            pros::c::screen_set_pen(cut ? kColBad : deadMan ? kColGood : kColText);
            pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 56, "%s", big);
            for (std::size_t i = 0; i < n; ++i) {
                const Member& m = members[i];
                const std::uint32_t colour = (m.disagreeTicks >= kPersistTicks
                                              || m.overTicks >= kPersistTicks)
                                                 ? kColBad
                                             : (m.disagreeTicks > 0 || m.overTicks > 0) ? kColWarn
                                             : std::abs(m.v) > kMovingFloorRadS         ? kColGood
                                                                                        : kColDim;
                if (g.cols == 3) {
                    std::snprintf(cell, sizeof cell, "%s%+3d %+5.1f %3.1fA", m.side < 0 ? "L" : "R",
                                  static_cast<int>(m.signedPort), m.v, m.amps);
                } else {
                    std::snprintf(cell, sizeof cell, "%s%+3d %+6.1fr/s %4.2fA %3.0fC",
                                  m.side < 0 ? "L" : "R", static_cast<int>(m.signedPort), m.v,
                                  m.amps, m.temp);
                }
                gridCell(g, i, colour, cell);
            }
            pros::c::screen_set_eraser(kColBg);
            pros::c::screen_erase_rect(0, static_cast<std::int16_t>(kFooterY - 2), kUsableW,
                                       kUsableH);
            if (cut) {
                pros::c::screen_set_pen(kColBad);
                pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY, "%.54s", cutWhy);
            } else if (!connected) {
                pros::c::screen_set_pen(kColBad);
                pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY,
                                         "CONTROLLER NOT CONNECTED -- 0 V");
            } else if (deadMan) {
                pros::c::screen_set_pen(kColGood);
                pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY,
                                         "DRIVING (L1 held) -- release L1 to stop");
            } else {
                pros::c::screen_set_pen(kColWarn);
                pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY,
                                         "0 V -- hold L1 to drive (thresholds INVENTED)");
            }
        }
        if (tick % kLcdEveryTicks == 0) {
            char want[3][24];
            std::snprintf(want[0], sizeof want[0], "%s %2.0fV HOLD L1", lcdMode, ceiling);
            std::snprintf(want[1], sizeof want[1], "R1 up L%+4.1f R%+4.1f", vL, vR);
            std::snprintf(want[2], sizeof want[2], "%s",
                          cut ? "CUT - see brain" : !connected ? "CTRL LOST 0V"
                                                  : deadMan    ? "DRIVING"
                                                               : "hold L1 to drive");
            for (int row = 0; row < 3; ++row) {
                if (std::strcmp(want[row], lcdRow[row]) != 0) {
                    lcd.setLine(row, want[row]);
                    std::memcpy(lcdRow[row], want[row], sizeof lcdRow[row]);  // same size, NUL inside
                }
            }
        }
        pros::delay(kTickMs);
        ++tick;
    }

    // ── GATE 6: exit -> every motor 0 V, coast (the guard repeats it on scope exit) ──
    stopAll(members, n);
    lcd.setLine(0, mode == DriveMode::Ground ? "GROUND ended 0V" : "DRIVE ended 0V");
    lcd.setLine(1, cut ? "CUT - see brain" : "exited by touch");
    lcd.setLine(2, "");

    // ── the record behind gate 2's second stage (see DriveRecord) ────────────────
    const double drivenSeconds = drivenTicks * kTickMs / 1000.0;
    const bool clean = !cut && drivenSeconds >= kCleanDrivenSeconds && ceiling >= kCleanCeilingV;
    g_drive.ran = true;
    g_drive.lastMode = mode;
    g_drive.lastClean = clean;
    if (mode == DriveMode::WheelsUp && clean) {
        g_drive.cleanWheelsUp = true;   // ground mode unlocked for this power cycle
    } else if (cut) {
        g_drive.cleanWheelsUp = false;  // a cut, in either mode, is new evidence of a fight
    }
    std::snprintf(g_drive.lastSummary, sizeof g_drive.lastSummary,
                  "last run: %s, %s (driven %.1f s, ceiling %.0f V%s)", modeWord,
                  clean ? "CLEAN" : "not clean", drivenSeconds, ceiling, cut ? ", CUT" : "");

    g_screenActive = true;
    screenClear();
    emit("");
    emitS(cut ? Sev::Bad : Sev::Good, "DRIVE (%s) ended: %s", modeWord,
          cut ? cutWhy : "exited by touch, no cut");
    emitf("  %d ticks (%.1f s); motors driven %.1f s; ceiling reached %.0f V; peak |cmd| %.1f V",
          tick, tick * kTickMs / 1000.0, drivenSeconds, ceiling, peakAbsCmd);
    emitf("  this %s run was %s: driven %.1f s (need >= %.0f), ceiling %.0f V (need >= %.0f), %s",
          modeWord, clean ? "CLEAN" : "NOT clean", drivenSeconds, kCleanDrivenSeconds, ceiling,
          kCleanCeilingV, cut ? "CUT" : "no cut");
    emitf("  ground mode is %s for the rest of this power cycle.",
          g_drive.cleanWheelsUp ? "UNLOCKED" : "LOCKED (needs a clean wheels-up run at >= 6 V)");
    emit("  port side signed   max A   max C");
    for (std::size_t i = 0; i < n; ++i) {
        const Member& m = members[i];
        emitf("  %2d    %s    %+3d    %5.2f   %4.0f", m.port, m.side < 0 ? "L" : "R",
              static_cast<int>(m.signedPort), m.maxAmps, m.maxTemp);
    }
    emit("  Everything above is in /usd/r3a_log.txt. This was the first hardware run of");
    emit("  ProsMotor/ProsController (HA-94 onward): record what the panel showed, verbatim.");
}

// ═══ STAGE 4 — battery, controller, SD card. ═════════════════════════════════
void reportPlatform() {
    rule("STAGE 4  BATTERY / CONTROLLER / SD (HA-57/103/104/107/122)");
    hal::pros::ProsBattery battery{};
    emitf("battery | RAW %d mV  %.1f%% | CANON %.3f V", static_cast<int>(pros::c::battery_get_voltage()),
          pros::c::battery_get_capacity(), battery.voltage().value());

    hal::pros::ProsController master{hal::pros::ControllerId::Master};
    const bool connected = master.isConnected();
    emitS(connected ? Sev::Good : Sev::Warn, "controller master: %s",
          connected ? "CONNECTED" : "NOT CONNECTED");
    if (!connected) {
        emit("  ** PAIR THE CONTROLLER. ** master=0 blocked HA-57/103/104/107 on BOTH");
        emit("  previous bench sessions, and it also enables wireless upload + terminal.");
    } else {
        emitf("  LeftY=%+.3f LeftX=%+.3f RightX=%+.3f (canonical [-1,1]; PROS raw is +-127)",
              master.axis(hal::ControllerAxis::LeftY), master.axis(hal::ControllerAxis::LeftX),
              master.axis(hal::ControllerAxis::RightX));
    }

    const bool card = pros::c::usd_is_installed() != 0;
    emitf("sd card: usd_is_installed()=%d  -> %s", static_cast<int>(pros::c::usd_is_installed()),
          card ? "PRESENT (HA-122 first half CONFIRMED)" : "ABSENT");
    if (g_card != nullptr) {
        emitf("  text log file open: %s", g_card->isOpen() ? "YES (/usd/r3a_log.txt)" : "NO");
    }
}

// ═══ STAGE 5 — the loop rate this build actually sustains (HA-32/HA-102). ════
void measureLoopRate() {
    rule("STAGE 5  LOOP RATE (HA-32 claims ~100 Hz; HA-102 claims anchored)");
    std::uint32_t prev = pros::millis();
    std::uint32_t minDt = 0xFFFFFFFFU, maxDt = 0, total = 0;
    for (int i = 0; i < kLoopSamples; ++i) {
        pros::Task::delay_until(&prev, 10);
        const std::uint32_t now = pros::millis();
        static std::uint32_t last = 0;
        if (i > 0) {
            const std::uint32_t dt = now - last;
            if (dt < minDt) minDt = dt;
            if (dt > maxDt) maxDt = dt;
            total += dt;
        }
        last = now;
    }
    emitf("delay_until(10ms) over %d ticks: min=%lu ms  max=%lu ms  mean=%.2f ms",
          kLoopSamples, static_cast<unsigned long>(minDt), static_cast<unsigned long>(maxDt),
          static_cast<double>(total) / static_cast<double>(kLoopSamples - 1));
    emit("LOAD SCOPE: this measures the PACER ALONE -- no motion, no odometry, no");
    emit("fusion. HA-32's '~100 Hz under full stack load' stays OPEN; this is its floor.");
}

// ═══ STATIONS 11 DISTANCE AND 12 AI VISION — the shared pieces (R2a, 2026-09-24) ═══════
// Both stations: a fresh registry scan at entry (never the chassis table, never g_found);
// everything they print goes to serial + the SD log (g_screenActive off), and the panel is a
// FIXED live layout redrawn at most 5 Hz and only where a line changed; a button row at the
// bottom (menu_grid::kStationRow: slot 0 left, 1 RECORD, 2 DONE) whose buttons are GREY and
// dead for the first 0.5 s (menu buttons 11 and 12 sit exactly where RECORD and DONE are, so a
// reflexive double tap on the menu would otherwise record or exit at once); every touch logged
// with what it hit; error reads skipped. They judge nothing: the verdict is DONE, or CHECK when
// a sensor was missing or a read / setting returned PROS_ERR.
namespace sensors {

constexpr std::uint32_t kLiveMs = 200;  // the panel's refresh: 5 Hz -- PROS leaks every print
constexpr std::uint32_t kArmMs = 500;   // the buttons' arming delay (calibrate's kArmingDelaySeconds)
constexpr std::uint32_t kTickMs = 10;   // the loop; station 12's change timer polls the camera at it
constexpr std::size_t kSmall = calibration::kSmallCols;    // 50: an ESTIMATE until the ruler (HA-136)
constexpr std::size_t kMedium = calibration::kMediumCols;  // 36: likewise
/// One line of a station button, MEDIUM, from x0 + kLabelInsetX to x1 - 1 at the proven MEDIUM
/// upper bound (472 px per kMediumCols characters): 11.
constexpr std::size_t kRowCols =
    static_cast<std::size_t>(menu_grid::line1Px(menu_grid::kStationRow)) * calibration::kMediumCols
    / 472;
constexpr std::uint32_t kColGreyFill = 0x3A4450;  // an arming button's fill (calibrate's)
/// The status line (RECORD confirmations, the idle instruction), SMALL, above the button row.
constexpr std::int16_t kStatusY = 171;
static_assert(kStatusY + kLineH < menu_grid::kStationRow.y0,
              "the status line must end above the station buttons");
static_assert(panelLiteralFits("RECORD", kRowCols) && panelLiteralFits("DONE", kRowCols)
                  && panelLiteralFits("FAMILY", kRowCols) && panelLiteralFits("as found", kRowCols)
                  && panelLiteralFits("CIRCLE21H7", kRowCols),
              "a station button label does not fit its line");

/// One fixed line of a live panel, PRINTED only when its text or colour changes (PROS leaks
/// every print). An in-place change is padded to the old length: VEXos draws text
/// pen-on-eraser, so the old characters are covered without an erase and without a flicker.
struct LiveLine {
    pros::text_format_e_t font = pros::E_TEXT_SMALL;
    std::int16_t x = 8;
    std::int16_t y = 0;
    std::size_t budget = kSmall;
    char shown[kPanelCap + 1] = "";
    std::uint32_t colour = 0;
    bool painted = false;

    void place(pros::text_format_e_t f, std::int16_t py, std::size_t b) {
        font = f;
        y = py;
        budget = b;
        shown[0] = '\0';
        painted = false;
    }

    void set(const char* text, std::uint32_t pen) {
        char want[kPanelCap + 1];
        panelSafe(text, budget, want, sizeof want);
        if (painted && pen == colour && std::strcmp(want, shown) == 0) return;
        const std::size_t oldLen = painted ? std::strlen(shown) : 0;
        screenText(font, x, y, pen, kColBg, want, budget, oldLen);
        std::memcpy(shown, want, sizeof shown);
        colour = pen;
        painted = true;
    }
};

/// Station-row slot `slot` (0 left, 1 middle, 2 right), one or two MEDIUM lines; GREY while
/// `armed` is false.
void drawRowButton(int slot, const char* line1, const char* line2, bool armed) {
    const menu_grid::Rect r = menu_grid::buttonBox(menu_grid::kStationRow, slot);
    const std::uint32_t fill = armed ? kColBar : kColGreyFill;
    pros::c::screen_set_pen(fill);
    pros::c::screen_fill_rect(r.x0, r.y0, r.x1, r.y1);
    pros::c::screen_set_pen(armed ? kColEdge : kColDim);
    pros::c::screen_draw_rect(r.x0, r.y0, r.x1, r.y1);
    const std::uint32_t pen = armed ? kColText : kColDim;
    const auto tx = static_cast<std::int16_t>(r.x0 + menu_grid::kLabelInsetX);
    screenText(pros::E_TEXT_MEDIUM, tx, static_cast<std::int16_t>(r.y0 + 4), pen, fill, line1,
               kRowCols);
    if (line2 != nullptr && line2[0] != '\0') {
        screenText(pros::E_TEXT_MEDIUM, tx, static_cast<std::int16_t>(r.y0 + 26), pen, fill, line2,
                   kRowCols);
    }
    pros::c::screen_set_eraser(kColBg);
}

/// New releases on the panel; error reads skipped (R2a-R4).
class TapWatch {
public:
    TapWatch() : seen_{releaseBaseline()} {}
    /// True when a NEW release arrived; where it landed in (x, y).
    bool poll(int& x, int& y) {
        const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
        if (t.touch_status == pros::E_TOUCH_ERROR || t.release_count == seen_) return false;
        seen_ = t.release_count;
        x = t.x;
        y = t.y;
        return true;
    }

private:
    std::int32_t seen_;
};

/// What a touch on the station row means right now, for the log.
const char* rowTouchWord(int slot, bool armed, const char* slot0, const char* slot1,
                         const char* slot2) {
    const char* word = slot == 0 ? slot0 : slot == 1 ? slot1 : slot == 2 ? slot2 : nullptr;
    if (word == nullptr) return "no button there";
    return armed ? word : "a button, IGNORED (still arming, grey)";
}

/// Every PHYSICAL port the registry reports as `kind` right now, ascending -- registry index i
/// is physical port i + 1 (the census's correction, kMaxRegistryIndex's note). Returns the count.
/// `ports` must hold kMaxPort entries.
static_assert(kMaxRegistryIndex + 1 <= kMaxPort, "discover() can fill every registry index");
int discover(pros::c::v5_device_e_t kind, int* ports) {
    int n = 0;
    for (int idx = 0; idx <= kMaxRegistryIndex; ++idx) {
        if (pros::c::registry_get_plugged_type(static_cast<std::uint8_t>(idx)) == kind) {
            ports[n++] = idx + 1;
        }
    }
    return n;
}

/// "4 5" -- ports, space-separated.
void portList(const int* ports, int n, char* buf, std::size_t cap) {
    buf[0] = '\0';
    for (int i = 0; i < n; ++i) {
        char one[8];
        std::snprintf(one, sizeof one, "%s%d", i > 0 ? " " : "", ports[i]);
        std::strncat(buf, one, cap - std::strlen(buf) - 1);
    }
}

/// An int32 exactly as PROS returned it: the number, or "ERR" for PROS_ERR.
void rawInt(std::int32_t v, char* buf, std::size_t cap) {
    if (v == PROS_ERR) {
        std::snprintf(buf, cap, "ERR");
    } else {
        std::snprintf(buf, cap, "%ld", static_cast<long>(v));
    }
}

/// A signed double with 2 (panel) or 3 (record) decimals, or "ERR" when it is not finite
/// (PROS_ERR_F is INFINITY).
void signedOrErr(double v, bool record, char* buf, std::size_t cap) {
    if (!std::isfinite(v)) {
        std::snprintf(buf, cap, "ERR");
    } else if (record) {
        std::snprintf(buf, cap, "%+.3f", v);
    } else {
        std::snprintf(buf, cap, "%+.2f", v);
    }
}

/// A PROS setter's return, for the log.
void retWord(std::int32_t ret, int err, char* buf, std::size_t cap) {
    if (ret == PROS_ERR) {
        std::snprintf(buf, cap, "PROS_ERR (errno %d)", err);
    } else if (ret == PROS_SUCCESS) {
        std::snprintf(buf, cap, "PROS_SUCCESS");
    } else {
        std::snprintf(buf, cap, "%ld", static_cast<long>(ret));
    }
}

// ── 11 DISTANCE ──────────────────────────────────────────────────────────────

/// One read of one sensor: the RAW PROS C API, then the library adapter -- separate device
/// reads, so two samples taken back to back, never one atomic snapshot.
struct DistRead {
    std::int32_t mm = PROS_ERR;    ///< distance_get: mm per the PROS doc (HA-113)
    std::int32_t conf = PROS_ERR;  ///< distance_get_confidence: 0-63 per the doc (HA-115)
    std::int32_t size = PROS_ERR;  ///< distance_get_object_size: 0-400 per the doc
    double vel = 0.0;              ///< distance_get_object_velocity: m/s per the doc
    bool lib = false;              ///< the adapter exists
    double libIn = 0.0;            ///< ProsDistance::distance(), canonical inches
    double libConf = 0.0;          ///< ProsDistance::confidence(), [0, 1]
    int libFaulted = 0;            ///< ProsDistance::faultedReads(), since the station began
};

DistRead readDistance(int port, const hal::pros::ProsDistance* lib) {
    const auto p = static_cast<std::uint8_t>(port);
    DistRead r;
    r.mm = pros::c::distance_get(p);
    r.conf = pros::c::distance_get_confidence(p);
    r.size = pros::c::distance_get_object_size(p);
    r.vel = pros::c::distance_get_object_velocity(p);
    if (lib != nullptr) {
        r.lib = true;
        r.libIn = lib->distance().value();
        r.libConf = lib->confidence();
        r.libFaulted = lib->faultedReads();
    }
    return r;
}

/// Reads of one sensor since the station began: how many, how many were the in-band 9999
/// (the adapter's kDistanceNoObjectMm, HA-114), how many PROS_ERR.
struct DistTally {
    unsigned long reads = 0;
    unsigned long noObject = 0;
    unsigned long err = 0;
};

void tally(DistTally& t, const DistRead& r) {
    ++t.reads;
    if (r.mm == hal::kDistanceNoObjectMm) ++t.noObject;
    if (r.mm == PROS_ERR) ++t.err;
}

void paintDistance(LiveLine& a, LiveLine& b, LiveLine& c, int port, const DistRead& r,
                   const DistTally& t) {
    char mm[16], cf[16], sz[16], vel[16], line[96];
    rawInt(r.mm, mm, sizeof mm);
    rawInt(r.conf, cf, sizeof cf);
    rawInt(r.size, sz, sizeof sz);
    signedOrErr(r.vel, false, vel, sizeof vel);
    std::snprintf(line, sizeof line, "PORT %d RAW %5s mm conf %2s", port, mm, cf);
    a.set(line, r.mm == PROS_ERR ? kColBad : kColText);
    if (r.lib) {
        std::snprintf(line, sizeof line, "  LIB %7.2f in conf %4.2f", r.libIn, r.libConf);
    } else {
        std::snprintf(line, sizeof line, "  LIB -- adapter refused (log)");
    }
    b.set(line, kColSub);
    std::snprintf(line, sizeof line, "  size %s  vel %sm/s  %ld x%lu  ERR x%lu/%d", sz, vel,
                  static_cast<long>(hal::kDistanceNoObjectMm), t.noObject, t.err,
                  r.lib ? r.libFaulted : 0);
    c.set(line, (t.err > 0 || (r.lib && r.libFaulted > 0)) ? kColWarn : kColDim);
}

/// The RECORD line for one sensor -- serial + the SD log.
void recordDistance(int rec, int port, const DistRead& r, const DistTally& t, std::uint32_t now) {
    char mm[16], cf[16], sz[16], vel[16], li[24], lc[24];
    rawInt(r.mm, mm, sizeof mm);
    rawInt(r.conf, cf, sizeof cf);
    rawInt(r.size, sz, sizeof sz);
    signedOrErr(r.vel, true, vel, sizeof vel);
    if (r.lib) {
        std::snprintf(li, sizeof li, "%.3f", r.libIn);
        std::snprintf(lc, sizeof lc, "%.3f", r.libConf);
    } else {
        std::snprintf(li, sizeof li, "none");
        std::snprintf(lc, sizeof lc, "none");
    }
    emitf("[R3A] dist REC %d port %d raw %s mm conf %s size %s vel %s m/s -> lib %s in conf %s"
          " | reads %lu: %ld x%lu, PROS_ERR x%lu, lib faulted %d | t %lu ms",
          rec, port, mm, cf, sz, vel, li, lc, t.reads, static_cast<long>(hal::kDistanceNoObjectMm),
          t.noObject, t.err, r.lib ? r.libFaulted : 0, static_cast<unsigned long>(now));
}

// ── 12 AI VISION ─────────────────────────────────────────────────────────────

/// The four tag families PROS 4.2.2 names (ai_vision.h:117-122), in PROS's own spelling. The
/// Override field HAS AprilTags (one per goal -- the game manual, read 2026-09-24), but the
/// manual's text does not name their FAMILY, so it is UNKNOWN -- the FAMILY button cycles them.
struct Family {
    pros::aivision_tag_family_e_t id;
    const char* name;
};
constexpr Family kFamilies[] = {
    {pros::TAG_CIRCLE_21H7, "CIRCLE21H7"},
    {pros::TAG_16H5, "16H5"},
    {pros::TAG_25H9, "25H9"},
    {pros::TAG_61H11, "61H11"},
};
constexpr int kFamilyCount = static_cast<int>(sizeof kFamilies / sizeof kFamilies[0]);

/// The detected-type flags (ai_vision.h:57-62), as unsigned for comparison with the byte.
constexpr unsigned kTypeColor = pros::E_AIVISION_DETECTED_COLOR;
constexpr unsigned kTypeCode = pros::E_AIVISION_DETECTED_CODE;
constexpr unsigned kTypeObject = pros::E_AIVISION_DETECTED_OBJECT;
constexpr unsigned kTypeTag = pros::E_AIVISION_DETECTED_TAG;

const char* typeWord(unsigned type) {
    return type == kTypeColor ? "COLOR" : type == kTypeCode ? "CODE"
         : type == kTypeObject ? "OBJECT" : type == kTypeTag ? "TAG" : nullptr;
}

/// A detection-mode bitfield as PROS's names (ai_vision.h:70-76): "0x05 TAGS+OBJECTS".
void modesWord(std::int32_t v, int err, char* buf, std::size_t cap) {
    if (v == PROS_ERR) {
        std::snprintf(buf, cap, "PROS_ERR (errno %d)", err);
        return;
    }
    const auto bits = static_cast<unsigned long>(v);
    // Sized to its content: "TAGS+COLORS+OBJECTS+COLOR_MERGE" is 31 characters, so the line
    // ("0x" + at most 8 hex digits + " " + this) always fits the callers' 64-byte buffers.
    char names[32] = "";
    auto add = [&names](const char* w) {
        if (names[0] != '\0') std::strncat(names, "+", sizeof names - std::strlen(names) - 1);
        std::strncat(names, w, sizeof names - std::strlen(names) - 1);
    };
    if ((bits & pros::E_AIVISION_MODE_TAGS) != 0) add("TAGS");
    if ((bits & pros::E_AIVISION_MODE_COLORS) != 0) add("COLORS");
    if ((bits & pros::E_AIVISION_MODE_OBJECTS) != 0) add("OBJECTS");
    if ((bits & pros::E_AIVISION_MODE_COLOR_MERGE) != 0) add("COLOR_MERGE");
    std::snprintf(buf, cap, "0x%02lx %s", bits, names[0] != '\0' ? names : "(none)");
}

/// The class name the camera reports for `id`, printable ASCII only ('?' for anything else),
/// or "ERR". The PROS doc asks for a buffer of at least 20; 64 zeroed bytes are passed.
void className(std::uint8_t port, int id, char* out, std::size_t cap) {
    std::uint8_t raw[64] = {};
    const std::int32_t ret = pros::c::aivision_get_class_name(port, id, raw);
    raw[sizeof raw - 1] = 0;
    if (ret == PROS_ERR) {
        std::snprintf(out, cap, "ERR");
        return;
    }
    std::size_t k = 0;
    for (std::size_t i = 0; raw[i] != 0 && k + 1 < cap; ++i) {
        out[k++] = (raw[i] >= 0x20 && raw[i] < 0x7F) ? static_cast<char>(raw[i]) : '?';
    }
    out[k] = '\0';
}

/// Everything the camera reports right now: the count as returned, and up to 24 objects.
struct CamSnap {
    std::int32_t count = PROS_ERR;
    int n = 0;
    pros::aivision_object_s_t obj[AIVISION_MAX_OBJECT_COUNT];
};

void snapCamera(std::uint8_t port, CamSnap& s) {
    s.count = pros::c::aivision_get_object_count(port);
    s.n = (s.count == PROS_ERR || s.count <= 0) ? 0
        : s.count > AIVISION_MAX_OBJECT_COUNT   ? AIVISION_MAX_OBJECT_COUNT
                                                 : static_cast<int>(s.count);
    for (int i = 0; i < s.n; ++i) {
        s.obj[i] = pros::c::aivision_get_object(port, static_cast<std::uint32_t>(i));
    }
}

/// FNV-1a over the count and every object's bytes: two snapshots with the same signature are
/// (with overwhelming likelihood) the same data. The change timer runs on it.
std::uint32_t signature(const CamSnap& s) {
    std::uint32_t h = 2166136261U;
    auto mix = [&h](const void* data, std::size_t len) {
        const auto* b = static_cast<const unsigned char*>(data);
        for (std::size_t i = 0; i < len; ++i) {
            h ^= b[i];
            h *= 16777619U;
        }
    };
    mix(&s.count, sizeof s.count);
    for (int i = 0; i < s.n; ++i) mix(&s.obj[i], sizeof s.obj[i]);
    return h;
}

/// One detection with its fields COPIED OUT of the packed struct (landmine 2: never a pointer or
/// reference to a packed member), read per the header's own layout for its type -- COLOR and
/// CODE as `color` (x, y, w, h, angle), OBJECT as `element` (x, y, w, h, score), TAG as `tag`
/// (x0, y0 .. x3, y3). Nothing converted. `raw` keeps all 18 bytes for the record.
struct Det {
    unsigned id = 0;
    unsigned type = 0;
    int v[8] = {};
    unsigned char raw[sizeof(pros::aivision_object_s_t)] = {};
};

Det decode(const pros::aivision_object_s_t& o) {
    Det d;
    std::memcpy(d.raw, &o, sizeof o);
    d.id = o.id;
    d.type = o.type;
    if (d.type == kTypeColor || d.type == kTypeCode) {
        d.v[0] = o.object.color.xoffset;
        d.v[1] = o.object.color.yoffset;
        d.v[2] = o.object.color.width;
        d.v[3] = o.object.color.height;
        d.v[4] = o.object.color.angle;
    } else if (d.type == kTypeObject) {
        d.v[0] = o.object.element.xoffset;
        d.v[1] = o.object.element.yoffset;
        d.v[2] = o.object.element.width;
        d.v[3] = o.object.element.height;
        d.v[4] = o.object.element.score;
    } else if (d.type == kTypeTag) {
        d.v[0] = o.object.tag.x0;
        d.v[1] = o.object.tag.y0;
        d.v[2] = o.object.tag.x1;
        d.v[3] = o.object.tag.y1;
        d.v[4] = o.object.tag.x2;
        d.v[5] = o.object.tag.y2;
        d.v[6] = o.object.tag.x3;
        d.v[7] = o.object.tag.y3;
    }
    return d;
}

/// A tag's centre: the mean of its four corners -- arithmetic on the reported corners, the one
/// derived number these stations show (the brief asks for it).
double tagCentreX(const Det& d) { return (d.v[0] + d.v[2] + d.v[4] + d.v[6]) / 4.0; }
double tagCentreY(const Det& d) { return (d.v[1] + d.v[3] + d.v[5] + d.v[7]) / 4.0; }

/// The panel row(s) for detection `i`: returns 1, or 2 for a TAG (its corners on a second row).
int detRows(int i, const Det& d, std::uint8_t port, char (*rows)[64]) {
    const char* w = typeWord(d.type);
    if (w == nullptr) {
        std::snprintf(rows[0], 64, "%d TYPE 0x%02x id %u (raw on RECORD)", i, d.type & 0xFFU, d.id);
        return 1;
    }
    if (d.type == kTypeTag) {
        std::snprintf(rows[0], 64, "%d TAG id %u  centre %ld,%ld", i, d.id,
                      std::lround(tagCentreX(d)), std::lround(tagCentreY(d)));
        std::snprintf(rows[1], 64, "  corners %d,%d %d,%d %d,%d %d,%d", d.v[0], d.v[1], d.v[2],
                      d.v[3], d.v[4], d.v[5], d.v[6], d.v[7]);
        return 2;
    }
    if (d.type == kTypeObject) {
        char cls[48];
        className(port, static_cast<int>(d.id), cls, sizeof cls);
        std::snprintf(rows[0], 64, "%d OBJECT %u %.10s x%d y%d w%d h%d s%d", i, d.id, cls, d.v[0],
                      d.v[1], d.v[2], d.v[3], d.v[4]);
        return 1;
    }
    std::snprintf(rows[0], 64, "%d %s %u x%d y%d w%d h%d a%d", i, w, d.id, d.v[0], d.v[1], d.v[2],
                  d.v[3], d.v[4]);
    return 1;
}

/// The RECORD line for one detection -- serial + the SD log, with its 18 raw bytes in hex.
void recordDetection(int rec, int port, int i, const Det& d) {
    char hex[2 * sizeof d.raw + 1];
    for (std::size_t b = 0; b < sizeof d.raw; ++b) {
        std::snprintf(hex + 2 * b, 3, "%02x", static_cast<unsigned>(d.raw[b]));
    }
    const char* w = typeWord(d.type);
    if (w == nullptr) {
        emitf("[R3A] aiv REC %d port %d obj %d TYPE 0x%02x id %u (not COLOR 1 / CODE 2 / OBJECT 4"
              " / TAG 8) | raw %s", rec, port, i, d.type & 0xFFU, d.id, hex);
    } else if (d.type == kTypeTag) {
        emitf("[R3A] aiv REC %d port %d obj %d TAG id %u centre %.2f %.2f corners %d,%d %d,%d"
              " %d,%d %d,%d | raw %s", rec, port, i, d.id, tagCentreX(d), tagCentreY(d), d.v[0],
              d.v[1], d.v[2], d.v[3], d.v[4], d.v[5], d.v[6], d.v[7], hex);
    } else if (d.type == kTypeObject) {
        char cls[48];
        className(static_cast<std::uint8_t>(port), static_cast<int>(d.id), cls, sizeof cls);
        emitf("[R3A] aiv REC %d port %d obj %d OBJECT id %u class '%s' x %d y %d w %d h %d"
              " score %d | raw %s", rec, port, i, d.id, cls, d.v[0], d.v[1], d.v[2], d.v[3],
              d.v[4], hex);
    } else {
        emitf("[R3A] aiv REC %d port %d obj %d %s id %u x %d y %d w %d h %d angle %d | raw %s",
              rec, port, i, w, d.id, d.v[0], d.v[1], d.v[2], d.v[3], d.v[4], hex);
    }
}

/// How often the camera's data changes, on the port the panel shows (ruling R2a-R8): the
/// interval between consecutive changes of the whole snapshot's signature, polled every tick.
struct ChangeTimer {
    static constexpr int kRing = 16;
    bool haveSig = false;
    std::uint32_t sig = 0;
    std::uint32_t lastChange = 0;  // when the data last changed (0: not yet)
    unsigned long changes = 0;     // changes seen
    std::uint32_t ring[kRing] = {};
    int ringN = 0;
    int ringAt = 0;
    std::uint32_t minMs = 0xFFFFFFFFU;

    void feed(std::uint32_t s, std::uint32_t now) {
        if (!haveSig) {
            haveSig = true;
            sig = s;
            return;
        }
        if (s == sig) return;
        sig = s;
        ++changes;
        if (lastChange != 0) {
            const std::uint32_t dt = now - lastChange;
            ring[ringAt] = dt;
            ringAt = (ringAt + 1) % kRing;
            if (ringN < kRing) ++ringN;
            if (dt < minMs) minMs = dt;
        }
        lastChange = now;
    }
    unsigned long meanMs() const {
        unsigned long sum = 0;
        for (int i = 0; i < ringN; ++i) sum += ring[i];
        return ringN > 0 ? sum / static_cast<unsigned long>(ringN) : 0;
    }
    /// The panel's words for it.
    void describe(std::uint32_t now, std::uint32_t since, char* buf, std::size_t cap) const {
        const std::uint32_t quietFrom = lastChange != 0 ? lastChange : since;
        if (now - quietFrom > 2000U) {
            std::snprintf(buf, cap, "no change in the data for %lu s",
                          static_cast<unsigned long>((now - quietFrom) / 1000U));
        } else if (ringN == 0) {
            std::snprintf(buf, cap, "data changed %lu time(s); timing ...", changes);
        } else {
            std::snprintf(buf, cap, "data changes every ~%lu ms (min %lu, n %lu)", meanMs(),
                          static_cast<unsigned long>(minMs), changes);
        }
    }
};

/// Turn on what station 12 needs on one camera, and LOG every write and read-back (landmine 1:
/// whether the camera keeps these after the program ends is UNKNOWN, so a later program must be
/// able to see what this one set). Returns false when any write returned PROS_ERR.
bool setUpCamera(int port, int family) {
    const auto p = static_cast<std::uint8_t>(port);
    bool ok = true;
    char w[64];
    errno = 0;
    const std::int32_t before = pros::c::aivision_get_enabled_detection_types(p);
    modesWord(before, errno, w, sizeof w);
    emitf("[R3A] aiv port %d detection modes AS FOUND: %s", port, w);
    errno = 0;
    std::int32_t ret = pros::c::aivision_enable_detection_types(
        p, static_cast<std::uint8_t>(pros::E_AIVISION_MODE_TAGS | pros::E_AIVISION_MODE_OBJECTS));
    retWord(ret, errno, w, sizeof w);
    ok = ok && ret != PROS_ERR;
    emitf("[R3A] aiv port %d SET tag + object detection ON (colours left as found) -> %s", port, w);
    errno = 0;
    ret = pros::c::aivision_set_usb_bounding_box_overlay(p, true);
    retWord(ret, errno, w, sizeof w);
    ok = ok && ret != PROS_ERR;
    emitf("[R3A] aiv port %d SET USB bounding-box overlay ON -> %s", port, w);
    if (family >= 0) {
        errno = 0;
        ret = pros::c::aivision_set_tag_family_override(p, kFamilies[family].id);
        retWord(ret, errno, w, sizeof w);
        ok = ok && ret != PROS_ERR;
        emitf("[R3A] aiv port %d SET tag family %s (override: the ONLY family; re-applied, chosen"
              " earlier this power cycle) -> %s", port, kFamilies[family].name, w);
    } else {
        emitf("[R3A] aiv port %d tag family NOT set: the camera's own (as found) until FAMILY is"
              " pressed", port);
    }
    errno = 0;
    const std::int32_t after = pros::c::aivision_get_enabled_detection_types(p);
    modesWord(after, errno, w, sizeof w);
    emitf("[R3A] aiv port %d detection modes NOW: %s", port, w);
    return ok;
}

/// The camera's class-name table, to the log once per entry (what its model can name).
void logClassNames(int port) {
    const auto p = static_cast<std::uint8_t>(port);
    std::uint8_t scratch[64] = {};
    errno = 0;
    const std::int32_t count = pros::c::aivision_get_class_name(p, -1, scratch);
    if (count == PROS_ERR) {
        emitf("[R3A] aiv port %d class names: get_class_name(-1) -> PROS_ERR (errno %d)", port,
              errno);
        return;
    }
    emitf("[R3A] aiv port %d class names: get_class_name(-1) -> %ld (the count, per the PROS doc;"
          " it may read wrong for the first few hundred ms after plug-in)", port,
          static_cast<long>(count));
    const int m = count <= 0 ? 0
                : count > AIVISION_MAX_CLASSNAME_COUNT ? AIVISION_MAX_CLASSNAME_COUNT
                                                       : static_cast<int>(count);
    for (int i = 0; i < m; ++i) {
        char name[48];
        className(p, i, name, sizeof name);
        emitf("[R3A] aiv port %d class %d '%s'", port, i, name);
    }
}

/// The RECORD lines for one camera: a summary, then one line per detection (a fresh snapshot).
void recordCamera(int rec, int port, int family, const ChangeTimer* timer, int timedPort,
                  std::uint32_t now) {
    const auto p = static_cast<std::uint8_t>(port);
    CamSnap s;
    snapCamera(p, s);
    const double temp = pros::c::aivision_get_temperature(p);
    errno = 0;
    const std::int32_t modes = pros::c::aivision_get_enabled_detection_types(p);
    char cnt[16], tp[16], md[64], ch[96];
    rawInt(s.count, cnt, sizeof cnt);
    if (std::isfinite(temp)) {
        std::snprintf(tp, sizeof tp, "%.2f", temp);
    } else {
        std::snprintf(tp, sizeof tp, "ERR");
    }
    modesWord(modes, errno, md, sizeof md);
    if (timer != nullptr && timer->ringN > 0) {
        std::snprintf(ch, sizeof ch, "data changed every ~%lu ms (min %lu, n %lu)", timer->meanMs(),
                      static_cast<unsigned long>(timer->minMs), timer->changes);
    } else if (timer != nullptr) {
        std::snprintf(ch, sizeof ch, "changes not timed yet (%lu change(s) seen; 2 needed)",
                      timer->changes);
    } else {
        std::snprintf(ch, sizeof ch, "changes not timed here (the panel times port %d)", timedPort);
    }
    emitf("[R3A] aiv REC %d port %d count %s read %d temp %s C family %s modes %s | %s | t %lu ms",
          rec, port, cnt, s.n, tp, family >= 0 ? kFamilies[family].name : "as-found", md, ch,
          static_cast<unsigned long>(now));
    for (int i = 0; i < s.n; ++i) recordDetection(rec, port, i, decode(s.obj[i]));
}

}  // namespace sensors

/// The tag family station 12 last set this power cycle; -1 = never, the camera's own ("as
/// found"). In memory for one power cycle, like MOTOR WATCH's capture.
int g_tagFamily = -1;

/// STATION 11 — DISTANCE (R2a). Read-only: every DISTANCE port in the registry, the RAW PROS C
/// API beside the library adapter's canonical inches and confidence, the in-band 9999 and
/// PROS_ERR counts; RECORD writes one [R3A] dist line per sensor. Nothing is written to a sensor.
void distanceStation(pros::c::v5_device_e_t*) {
    using namespace sensors;
    rule("11 DISTANCE -- read-only");
    g_screenActive = false;  // serial + SD from here; the panel is the live view
    const std::uint32_t prints0 = g_screenPrints;
    const bool sdOn = g_card != nullptr && g_card->isOpen();

    int ports[kMaxPort];
    const int n = discover(pros::c::E_DEVICE_DISTANCE, ports);
    char list[96];
    portList(ports, n, list, sizeof list);
    emitf("[R3A] dist registry scan now: %d DISTANCE sensor(s)%s%s", n,
          n > 0 ? " on port(s) " : "", list);
    if (n == 0) {
        // Every line from here to the return is on the panel: each fits its 54 columns.
        g_screenActive = true;
        screenClear();
        emitS(Sev::Warn, "no DISTANCE sensor in the registry -- nothing to show.");
        emit("  check both ends of the cable, then run");
        emit("  1 DEVICE CENSUS: it lists what the brain sees.");
        emit("  (the old robot two: ports 4 and 5, 2026-09-24)");
        return;
    }
    emit("RAW = the PROS C API as it returns it: distance_get (mm), _get_confidence");
    emit("      (0-63), _get_object_size, _get_object_velocity (m/s).");
    emit("LIB = shulib's hal::pros::ProsDistance: distance() in inches, confidence() in");
    emit("      [0, 1]. Separate device reads: RAW and LIB are two samples, not one.");
    emit("Nothing is powered and nothing is written to a sensor.");

    // The adapter, one per sensor, each inside a try (the file's rule: a surprise reports itself
    // instead of ending the session).
    std::optional<hal::pros::ProsDistance> lib[kMaxPort];
    bool anyFault = false;
    for (int k = 0; k < n; ++k) {
        try {
            lib[k].emplace(static_cast<std::uint8_t>(ports[k]));
        } catch (const std::exception& e) {
            anyFault = true;
            emitf("[R3A] dist port %d ProsDistance REFUSED: %s", ports[k], e.what());
        }
    }

    // ── the live panel: two sensors in full, a status line, RECORD and DONE ──────
    screenClear();
    constexpr int kShownMax = 2;
    constexpr std::int16_t kBlockY = 40;
    constexpr std::int16_t kBlockPitch = 62;  // MEDIUM + MEDIUM (22 px pitch) + SMALL
    static_assert(kBlockY + (kShownMax - 1) * kBlockPitch + 44 + kLineH < kStatusY,
                  "the second sensor's block must end above the status line");
    const int shown = n < kShownMax ? n : kShownMax;
    LiveLine la[kShownMax], lb[kShownMax], lc[kShownMax], status;
    for (int k = 0; k < shown; ++k) {
        const auto y0 = static_cast<std::int16_t>(kBlockY + k * kBlockPitch);
        la[k].place(pros::E_TEXT_MEDIUM, y0, kMedium);
        lb[k].place(pros::E_TEXT_MEDIUM, static_cast<std::int16_t>(y0 + 22), kMedium);
        lc[k].place(pros::E_TEXT_SMALL, static_cast<std::int16_t>(y0 + 44), kSmall);
    }
    status.place(pros::E_TEXT_SMALL, kStatusY, kSmall);
    char idle[64];
    if (n > shown) {
        std::snprintf(idle, sizeof idle, "+%d more DISTANCE sensor(s): RECORD logs every one",
                      n - shown);
    } else {
        std::snprintf(idle, sizeof idle, "hold square to a wall at 6/12/24 in; RECORD each");
    }
    status.set(idle, kColWarn);
    drawRowButton(1, "RECORD", "", false);
    drawRowButton(2, "DONE", "", false);

    DistTally tallies[kMaxPort] = {};
    TapWatch taps;
    const std::uint32_t t0 = pros::millis();
    std::uint32_t lastPaint = t0 - kLiveMs;  // paint on the first tick
    bool armed = false;
    int records = 0;
    for (;;) {
        const std::uint32_t now = pros::millis();
        if (!armed && now - t0 >= kArmMs) {
            armed = true;
            drawRowButton(1, "RECORD", "", true);
            drawRowButton(2, "DONE", "", true);
        }
        int tx = 0, ty = 0;
        if (taps.poll(tx, ty)) {
            const int slot = menu_grid::hitTest(menu_grid::kStationRowHit, tx, ty);
            emitf("[R3A] dist touch x=%d y=%d -> %s", tx, ty,
                  rowTouchWord(slot, armed, nullptr, "RECORD", "DONE"));  // slot 0: no button
            if (armed && slot == 2) break;
            if (armed && slot == 1) {
                ++records;
                for (int k = 0; k < n; ++k) {
                    const DistRead r = readDistance(ports[k], lib[k] ? &*lib[k] : nullptr);
                    tally(tallies[k], r);
                    anyFault = anyFault || r.mm == PROS_ERR || (r.lib && r.libFaulted > 0);
                    recordDistance(records, ports[k], r, tallies[k], now);
                }
                if (g_card != nullptr) (void)g_card->flush();
                char s[64];
                std::snprintf(s, sizeof s, "RECORD %d: %d line(s) -> serial%s", records, n,
                              sdOn ? " + SD card" : " only (SD OFF)");
                status.set(s, sdOn ? kColGood : kColWarn);
            }
        }
        if (now - lastPaint >= kLiveMs) {
            lastPaint = now;
            for (int k = 0; k < n; ++k) {
                const DistRead r = readDistance(ports[k], lib[k] ? &*lib[k] : nullptr);
                tally(tallies[k], r);
                anyFault = anyFault || r.mm == PROS_ERR || (r.lib && r.libFaulted > 0);
                if (k < shown) paintDistance(la[k], lb[k], lc[k], ports[k], r, tallies[k]);
            }
        }
        pros::delay(kTickMs);
    }

    // The full record first, on serial + SD only; then a short summary on the panel too, every
    // line of it within the scrolling log's 54 columns.
    for (int k = 0; k < n; ++k) {
        emitf("[R3A] dist port %d: %lu reads since entry, %ld x%lu, PROS_ERR x%lu, lib faulted %d",
              ports[k], tallies[k].reads, static_cast<long>(hal::kDistanceNoObjectMm),
              tallies[k].noObject, tallies[k].err, lib[k] ? lib[k]->faultedReads() : 0);
    }
    emitf("[R3A] dist LEFT after %d RECORD(s): nothing written to any sensor; screen prints"
          " this station %lu", records, static_cast<unsigned long>(g_screenPrints - prints0));
    g_screenActive = true;
    screenClear();
    emit("");
    emitf("11 DISTANCE ended: %d RECORD(s)", records);
    for (int k = 0; k < n; ++k) {
        emitf("  port %d: %lu reads, %ld x%lu", ports[k], tallies[k].reads,
              static_cast<long>(hal::kDistanceNoObjectMm), tallies[k].noObject);
        emitf("    PROS_ERR x%lu, lib faulted %d", tallies[k].err,
              lib[k] ? lib[k]->faultedReads() : 0);
    }
    emit(sdOn ? "  RECORD lines: /usd/r3a_log.txt + serial, [R3A] dist"
              : "  SD OFF: RECORD lines on serial only, [R3A] dist");
    if (anyFault) {
        emitS(Sev::Warn, "  a read returned PROS_ERR (see the [R3A] dist lines)");
    }
}

/// STATION 12 — AI VISION (R2a). Writes the CAMERA's settings (tag + object detection on, the
/// USB bounding-box overlay on, a tag family on FAMILY) and says so; shows the object count,
/// temperature, how often the data changes, and each detection -- type, id or class name, box or
/// tag centre and corners -- as the header lays them out. Interprets nothing. RECORD writes a
/// summary and one line per detection per camera.
void aiVisionStation(pros::c::v5_device_e_t*) {
    using namespace sensors;
    int ports[kMaxPort];
    const int n = discover(pros::c::E_DEVICE_AIVISION, ports);
    int legacy[kMaxPort];
    const int nLegacy = discover(pros::c::E_DEVICE_VISION, legacy);
    char title[40];
    if (n > 0) {
        std::snprintf(title, sizeof title, "12 AI VISION -- port %d", ports[0]);
    } else {
        std::snprintf(title, sizeof title, "12 AI VISION -- none found");
    }
    rule(title);
    g_screenActive = false;  // serial + SD from here; the panel is the live view
    const std::uint32_t prints0 = g_screenPrints;
    const bool sdOn = g_card != nullptr && g_card->isOpen();

    char list[96], legacyList[96];
    portList(ports, n, list, sizeof list);
    portList(legacy, nLegacy, legacyList, sizeof legacyList);
    emitf("[R3A] aiv registry scan now: %d AI-VISION sensor(s)%s%s", n,
          n > 0 ? " on port(s) " : "", list);
    if (nLegacy > 0) {
        emitf("[R3A] aiv port(s) %s report VISION (code 11, the older Vision sensor): NOT driven"
              " by this station", legacyList);
    }
    if (n == 0) {
        // Every line from here to the return is on the panel: each fits its 54 columns.
        g_screenActive = true;
        screenClear();
        emitS(Sev::Warn, "no AI-VISION sensor in the registry: nothing to show.");
        if (nLegacy > 0) {
            emitf("  port(s) %s: VISION (code 11) -- the older sensor", legacyList);
        }
        emit("  check both ends of the cable, then run");
        emit("  1 DEVICE CENSUS: it must say AI-VISION.");
        emit("  (the old robot two: port 3, 2026-09-24)");
        return;
    }
    emit("This station WRITES THE CAMERA'S SETTINGS: tag + object detection ON, the USB");
    emit("bounding-box overlay ON, and a tag family when FAMILY is pressed. Whether the camera");
    emit("keeps them after this program ends is UNKNOWN -- every write is logged below.");
    emit("COLORS are left as found: colour detection needs colour descriptors, and this");
    emit("station defines none. Nothing is interpreted: no bearing, no distance.");

    bool anyFault = false;
    for (int k = 0; k < n; ++k) {
        if (!setUpCamera(ports[k], g_tagFamily)) anyFault = true;
        logClassNames(ports[k]);
    }
    const bool setOk = !anyFault;
    const auto p0 = static_cast<std::uint8_t>(ports[0]);

    // ── the live panel ──────────────────────────────────────────────────────────
    screenClear();
    constexpr int kRows = 4;
    constexpr std::int16_t kRowY = 116;
    static_assert(kRowY + (kRows - 1) * kLineH + kLineH < kStatusY,
                  "the last detection row must end above the status line");
    LiveLine said[3], counts, period, rows[kRows], status;
    said[0].place(pros::E_TEXT_SMALL, 38, kSmall);
    said[1].place(pros::E_TEXT_SMALL, 51, kSmall);
    said[2].place(pros::E_TEXT_SMALL, 64, kSmall);
    counts.place(pros::E_TEXT_MEDIUM, 79, kMedium);
    period.place(pros::E_TEXT_SMALL, 102, kSmall);
    for (int r = 0; r < kRows; ++r) {
        rows[r].place(pros::E_TEXT_SMALL, static_cast<std::int16_t>(kRowY + r * kLineH), kSmall);
    }
    status.place(pros::E_TEXT_SMALL, kStatusY, kSmall);
    {
        char s[64];
        if (setOk) {
            if (n > 1) {
                std::snprintf(s, sizeof s, "THIS STATION TURNED ON, on port %d (and %d more):",
                              ports[0], n - 1);
            } else {
                std::snprintf(s, sizeof s, "THIS STATION TURNED ON, on port %d:", ports[0]);
            }
            said[0].set(s, kColWarn);
            said[1].set("tag + object detection and the USB box overlay", kColWarn);
        } else {
            std::snprintf(s, sizeof s, "A SETTING FAILED on port %d -- see serial / SD", ports[0]);
            said[0].set(s, kColBad);
            said[1].set("tag + object detection or the overlay may be OFF", kColBad);
        }
        said[2].set("COLORS need color descriptors - none set here", kColDim);
    }
    status.set("move a piece left, then right; RECORD each spot", kColWarn);
    drawRowButton(0, "FAMILY", g_tagFamily >= 0 ? kFamilies[g_tagFamily].name : "as found", false);
    drawRowButton(1, "RECORD", "", false);
    drawRowButton(2, "DONE", "", false);

    ChangeTimer timer;
    CamSnap snap;
    TapWatch taps;
    const std::uint32_t t0 = pros::millis();
    std::uint32_t lastPaint = t0 - kLiveMs;  // paint on the first tick
    bool armed = false;
    bool familyFailed = false;  // a FAMILY write returned PROS_ERR: the exit must not claim it
    int records = 0;
    for (;;) {
        const std::uint32_t now = pros::millis();
        if (!armed && now - t0 >= kArmMs) {
            armed = true;
            drawRowButton(0, "FAMILY", g_tagFamily >= 0 ? kFamilies[g_tagFamily].name : "as found",
                          true);
            drawRowButton(1, "RECORD", "", true);
            drawRowButton(2, "DONE", "", true);
        }
        int tx = 0, ty = 0;
        if (taps.poll(tx, ty)) {
            const int slot = menu_grid::hitTest(menu_grid::kStationRowHit, tx, ty);
            emitf("[R3A] aiv touch x=%d y=%d -> %s", tx, ty,
                  rowTouchWord(slot, armed, "FAMILY", "RECORD", "DONE"));
            if (armed && slot == 2) break;
            if (armed && slot == 0) {
                g_tagFamily = (g_tagFamily + 1) % kFamilyCount;  // -1 (as found) -> the first
                for (int k = 0; k < n; ++k) {
                    char w[48];
                    errno = 0;
                    const std::int32_t ret = pros::c::aivision_set_tag_family_override(
                        static_cast<std::uint8_t>(ports[k]), kFamilies[g_tagFamily].id);
                    retWord(ret, errno, w, sizeof w);
                    if (ret == PROS_ERR) {
                        anyFault = true;
                        familyFailed = true;
                    }
                    emitf("[R3A] aiv port %d SET tag family %s (override: the ONLY family now) ->"
                          " %s | t %lu ms", ports[k], kFamilies[g_tagFamily].name, w,
                          static_cast<unsigned long>(now));
                }
                drawRowButton(0, "FAMILY", kFamilies[g_tagFamily].name, true);
                char s[64];
                std::snprintf(s, sizeof s, "tag family now %s -- hold the tag in view",
                              kFamilies[g_tagFamily].name);
                status.set(s, kColWarn);
            }
            if (armed && slot == 1) {
                ++records;
                for (int k = 0; k < n; ++k) {
                    recordCamera(records, ports[k], g_tagFamily, k == 0 ? &timer : nullptr,
                                 ports[0], now);
                }
                if (g_card != nullptr) (void)g_card->flush();
                char s[64];
                std::snprintf(s, sizeof s, "RECORD %d -> serial%s", records,
                              sdOn ? " + SD card" : " only (SD OFF)");
                status.set(s, sdOn ? kColGood : kColWarn);
            }
        }

        // The change timer polls every tick; the panel repaints at 5 Hz, only what changed.
        snapCamera(p0, snap);
        if (snap.count == PROS_ERR) anyFault = true;
        timer.feed(signature(snap), now);
        if (now - lastPaint >= kLiveMs) {
            lastPaint = now;
            char line[96], cnt[32], tp[16];
            if (snap.count == PROS_ERR) {
                std::snprintf(cnt, sizeof cnt, "ERR");
            } else if (snap.count > AIVISION_MAX_OBJECT_COUNT) {
                std::snprintf(cnt, sizeof cnt, "%ld (%d read)", static_cast<long>(snap.count),
                              snap.n);
            } else {
                std::snprintf(cnt, sizeof cnt, "%ld", static_cast<long>(snap.count));
            }
            const double temp = pros::c::aivision_get_temperature(p0);
            if (std::isfinite(temp)) {
                std::snprintf(tp, sizeof tp, "%.1f", temp);
            } else {
                std::snprintf(tp, sizeof tp, "ERR");
            }
            std::snprintf(line, sizeof line, "OBJECTS %s  TEMP %s C", cnt, tp);
            counts.set(line, snap.count == PROS_ERR ? kColBad : kColText);
            timer.describe(now, t0, line, sizeof line);
            period.set(line, kColDim);

            // The detections, as many as fit; a TAG takes two rows (its corners); when some do
            // not fit, the last row says how many more (RECORD logs them all).
            int row = 0;
            int shownDets = 0;
            for (int i = 0; i < snap.n; ++i) {
                const Det d = decode(snap.obj[i]);
                char two[2][64] = {"", ""};
                const int need = detRows(i, d, p0, two);
                const int limit = (i + 1 < snap.n) ? kRows - 1 : kRows;  // keep one for "+more"
                if (row + need > limit) break;
                const std::uint32_t pen = typeWord(d.type) == nullptr ? kColWarn
                                        : d.type == kTypeTag           ? kColSub
                                                                        : kColText;
                for (int j = 0; j < need; ++j) rows[row++].set(two[j], pen);
                ++shownDets;
            }
            if (shownDets < snap.n) {
                std::snprintf(line, sizeof line, "+%d more -- RECORD logs all %d",
                              snap.n - shownDets, snap.n);
                rows[row++].set(line, kColDim);
            }
            while (row < kRows) rows[row++].set("", kColDim);
        }
        pros::delay(kTickMs);
    }

    // The full record first, on serial + SD only; then a short summary on the panel too, every
    // line of it within the scrolling log's 54 columns.
    const char* const famWord = g_tagFamily >= 0 ? kFamilies[g_tagFamily].name : "as found";
    if (timer.ringN > 0) {
        emitf("[R3A] aiv port %d data changed %lu time(s); last %d intervals ~%lu ms, min %lu ms",
              ports[0], timer.changes, timer.ringN, timer.meanMs(),
              static_cast<unsigned long>(timer.minMs));
    } else {
        emitf("[R3A] aiv port %d data changed %lu time(s); not timed (2 needed)", ports[0],
              timer.changes);
    }
    // What was LEFT on the camera is claimed only as far as the writes said they succeeded.
    if (setOk) {
        emitf("[R3A] aiv LEFT port(s) %s, %d RECORD(s): tag + object detection ON, USB overlay ON,"
              " tag family %s%s; kept after this program? UNKNOWN; screen prints %lu", list,
              records, famWord, familyFailed ? " (a FAMILY write FAILED)" : "",
              static_cast<unsigned long>(g_screenPrints - prints0));
    } else {
        emitf("[R3A] aiv LEFT port(s) %s, %d RECORD(s): a SETTING write FAILED at entry (the SET"
              " lines above); what the camera has is UNKNOWN; screen prints %lu", list, records,
              static_cast<unsigned long>(g_screenPrints - prints0));
    }
    g_screenActive = true;
    screenClear();
    emit("");
    emitf("12 AI VISION ended: %d RECORD(s)", records);
    if (setOk) {
        emit("  LEFT ON THE CAMERA: tag + object detection ON,");
        emitf("  the USB overlay ON, tag family %s.", famWord);
        emit("  written to the SENSOR: whether it keeps them after");
        emit("  this program ends is UNKNOWN.");
    } else {
        emit("  a SETTING write FAILED at entry: what the camera");
        emit("  now has is UNKNOWN -- see the [R3A] aiv SET lines.");
    }
    if (familyFailed) emit("  a FAMILY write FAILED -- see the SET lines.");
    emit(sdOn ? "  RECORD lines: /usd/r3a_log.txt + serial, [R3A] aiv"
              : "  SD OFF: RECORD lines on serial only, [R3A] aiv");
    if (anyFault) {
        emitS(Sev::Warn, "  a setting or read returned PROS_ERR ([R3A] aiv)");
    }
}


// ═══ STATION 13 — MECHANISMS: THE SECOND STATION THAT POWERS (MECH-A6, 2026-10-30) ══════════════════════
// The mechanism core's last part (the MECH brief's §1.14). Every mechanism the robot DECLARES
// (kMechanisms — src/robots/mechanism_table.hpp; each cascade robot's cascade, side toggles and pistons —
// robot two's claws until ROB1-BT), shown one at a time with its readings live and in its declared units, and
// powered behind SIX GATES — through the library's own robot::MechanismParts over the
// hal::pros::ProsDeviceProvider (no Robot, no drivetrain, no mechanism service: a direction must be
// checkable before anything is homed), commanding each built hal::MotorMechanism / PneumaticMechanism
// directly, as station 10 commands its motors. THE LOGIC IS NOT HERE: the gates, the direction check, the
// readings and the RECORD lines are bench_r3a.hpp's mech_station::Station, host-tested in
// test/bench_mech_station_test.cpp. This function only reads the controller into it, the raw motor
// temperature beside it, paints what it says, shows its prompts and logs its lines.
//
// STATIC STORAGE, ONCE PER POWER CYCLE: robot::MechanismParts is built ONCE and BORROWS its configs (MECH-A1's
// D11), and the provider owns every adapter it made for the program's life — so the provider, the projected
// configs and the parts live in function-local statics, built on the first CLEAR of a power cycle; every
// later entry reuses them (and asks the CLEAR question again). The Station itself is made new at every entry:
// arming, cuts and the ceiling start over.
//
// The prints follow stations 11/12: live lines painted on change at most every 200 ms (PROS leaks every
// print), the station row armed after 0.5 s, every text through screenText()'s sanitising path.
namespace mech13 {

namespace ms = mech_station;

/// The parts station 13 builds, ONCE per power cycle (header).
struct Built {
    config::MechanismConfig configs[mechanism::kMaxMechanisms]{};
    robot::MechanismParts parts;
    bool attempted = false;  ///< the build ran this power cycle (MechanismParts builds once)
    bool failed = false;     ///< it threw: nothing on the page is powered this power cycle
    char why[160] = "";
};
Built& built() {
    static Built b;
    return b;
}
hal::pros::ProsDeviceProvider& provider() {
    static hal::pros::ProsDeviceProvider p;  // opens nothing until a mechanism asks for a port
    return p;
}
/// This entry's state machine (static: a Station is ~8 KB, too much for the task's stack).
std::optional<ms::Station>& station() {
    static std::optional<ms::Station> s;
    return s;
}

/// The RAW temperature of smart port `port` (PROS_ERR_F, infinity, when the read failed — "unknown",
/// never the adapter's held 20 C).
double rawTemperature(std::uint8_t port) { return pros::c::motor_get_temperature(port); }

/// A Station prompt through twoButtonPrompt, every string made panel-safe first.
bool prompt(const ms::PromptText& t) {
    char a[kPanelCap + 1], b[kPanelCap + 1], c[kPanelCap + 1], d[kPanelCap + 1], e[kPanelCap + 1],
        f[kPanelCap + 1], g[kPanelCap + 1], h[kPanelCap + 1];
    panelSafe(t.title, sensors::kMedium, a, sizeof a);
    panelSafe(t.sub, sensors::kSmall, b, sizeof b);
    panelSafe(t.yesBig, 14, c, sizeof c);
    panelSafe(t.yesSmall, 22, d, sizeof d);
    panelSafe(t.noBig, 14, e, sizeof e);
    panelSafe(t.noSmall, 22, f, sizeof f);
    panelSafe(t.foot1, sensors::kSmall, g, sizeof g);
    panelSafe(t.foot2, sensors::kSmall, h, sizeof h);
    emitf("[R3A] mech ASK: %s / %s", t.title, t.sub);
    const bool yes = twoButtonPrompt(a, b, c, d, e, f, g, h);
    emitf("[R3A] mech ANSWER: %s", yes ? t.yesBig : t.noBig);
    return yes;
}

// Every station 13 log line is ONE serial write: the Station keeps each at most this long (SER1).
static_assert(ms::Station::kLineChars + 1 == hal::pros::kSerialLineBytes,
              "station 13's longest log line is one serial write's content");

/// A stage-2 confirm's answer: YES, NO, or the dead-man let go (or the controller dropped) while it waited.
enum class Held : std::uint8_t { Yes, No, LetGo };

/// GATE 2's stage-2 prompt WITH THE DEAD-MAN WATCHED: twoButtonPrompt's own layout (that function is
/// stations 4 and 10's, untouched), but its wait polls the controller every tick — L1 let go, or the
/// controller gone, CLOSES the prompt at once as LetGo, so the Station makes whatever it powered safe that
/// tick (a claw fired before the prompt does not stay fired while nobody holds L1). Every string made
/// panel-safe first, as prompt() does.
Held promptHeld(const ms::PromptText& t, hal::pros::ProsController& master) {
    char a[kPanelCap + 1], b[kPanelCap + 1], c[kPanelCap + 1], d[kPanelCap + 1], e[kPanelCap + 1],
        f[kPanelCap + 1], g[kPanelCap + 1], h[kPanelCap + 1];
    panelSafe(t.title, sensors::kMedium, a, sizeof a);
    panelSafe(t.sub, sensors::kSmall, b, sizeof b);
    panelSafe(t.yesBig, 14, c, sizeof c);
    panelSafe(t.yesSmall, 22, d, sizeof d);
    panelSafe(t.noBig, 14, e, sizeof e);
    panelSafe(t.noSmall, 22, f, sizeof f);
    panelSafe(t.foot1, sensors::kSmall, g, sizeof g);
    panelSafe(t.foot2, sensors::kSmall, h, sizeof h);
    emitf("[R3A] mech ASK: %s / %s", t.title, t.sub);
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 8, "%s", a);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 32, "%s", b);
    pros::c::screen_set_pen(kColBar);
    pros::c::screen_fill_rect(6, 60, 234, 150);
    pros::c::screen_fill_rect(246, 60, 474, 150);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_set_eraser(kColBar);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 20, 82, "%s", c);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 20, 112, "%s", d);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 262, 82, "%s", e);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 262, 112, "%s", f);
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_set_pen(kColDim);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 170, "%s", g);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 184, "%s", h);
    const std::int32_t seen = pros::c::screen_touch_status().release_count;
    for (;;) {
        if (!master.isConnected() || !master.pressed(hal::ControllerButton::L1)) {
            emit("[R3A] mech ANSWER: none -- L1 let go (or the controller dropped) while it waited");
            return Held::LetGo;
        }
        const pros::screen_touch_status_s_t s = pros::c::screen_touch_status();
        if (s.release_count != seen && s.y >= 60 && s.y <= 150) {
            const bool yes = s.x < 240;
            emitf("[R3A] mech ANSWER: %s", yes ? t.yesBig : t.noBig);
            return yes ? Held::Yes : Held::No;
        }
        pros::delay(ms::kTickMs);
    }
}

/// Gate 6 on every way out of the station — the RAII half (a thrown precondition included).
struct ExitGuard {
    ms::Station* st;
    ~ExitGuard() {
        if (st != nullptr) st->exit();
    }
};

/// A four-answer slot (kAnswerRow4), two SMALL lines.
void drawAnswer4(int slot, const char* line1, const char* line2, bool armed) {
    const menu_grid::Rect r = menu_grid::buttonBox(menu_grid::kAnswerRow4, slot);
    const std::uint32_t fill = armed ? kColBar : sensors::kColGreyFill;
    pros::c::screen_set_pen(fill);
    pros::c::screen_fill_rect(r.x0, r.y0, r.x1, r.y1);
    pros::c::screen_set_pen(armed ? kColEdge : kColDim);
    pros::c::screen_draw_rect(r.x0, r.y0, r.x1, r.y1);
    const std::uint32_t pen = armed ? kColText : kColDim;
    const auto tx = static_cast<std::int16_t>(r.x0 + menu_grid::kLabelInsetX);
    constexpr std::size_t kCols =
        static_cast<std::size_t>(menu_grid::line1Px(menu_grid::kAnswerRow4)) * calibration::kSmallCols / 472;
    screenText(pros::E_TEXT_SMALL, tx, static_cast<std::int16_t>(r.y0 + 6), pen, fill, line1, kCols);
    screenText(pros::E_TEXT_SMALL, tx, static_cast<std::int16_t>(r.y0 + 24), pen, fill, line2, kCols);
    pros::c::screen_set_eraser(kColBg);
}

}  // namespace mech13

/// STATION 13 — MECHANISMS (MECH-A6). Powers ONE mechanism at a time behind six gates; the direction
/// check; the readings in declared units; RECORD writes `[R3A] mech` lines (header above).
void mechanismsStation(pros::c::v5_device_e_t*) {
    using namespace sensors;
    namespace ms = mech_station;
    rule("13 MECHANISMS -- POWERS");
    const mechanism::MechanismTable& list = kMechanisms;
    emitf("[R3A] mech robot %s: %u mechanism(s) declared (src/robots/mechanism_table.hpp)", kChassis.robot,
          static_cast<unsigned>(list.count));
    if (list.count == 0) {
        emitS(Sev::Warn, "this robot declares NO mechanism -- nothing to show.");
        emit("  a robot's mechanisms are declared in");
        emit("  src/robots/mechanism_table.hpp");
        emit("  (the cascade robots: ROBOT=one, ROBOT=tank).");
        return;
    }
    // Each line within the scrolling log's 54 columns.
    emitS(Sev::Warn, "POWERS MECHANISMS -- one at a time, only with L1 held.");
    emit("Six gates: only a built one; CLEAR, then a confirm per");
    emit("mechanism and per pneumatic fire; L1 is the dead-man;");
    emit("ceiling 3 V (R1: 6/9/12); a cut at a fight, 2.4 A for");
    emit("250 ms, its stall signature or a 3 s jog; DONE = safe.");
    emit("Each mechanism starts at 3 V; a power one never past");
    emit("its maxVolts. A check names what should move first.");
    emit("Every threshold INVENTED for a first run (HA-182).");
    emit("Nothing is saved: a verdict names the line to edit.");

    std::optional<ms::Station>& slot = mech13::station();
    slot.emplace(list, mech13::built().parts);
    ms::Station& st = *slot;
    mech13::ExitGuard guard{&st};  // gate 6 on every way out from here

    // ── GATE 2, stage 1: the CLEAR question, before anything is made or powered ──────
    // (the prompt erased the whole panel, header bar included: station 10's rule, redraw it)
    if (!mech13::prompt(st.clearText())) {
        g_screenActive = true;
        drawHeader("13 MECHANISMS -- POWERS");
        screenClear();
        emitS(Sev::Warn, "not cleared: nothing was made, nothing was powered.");
        return;
    }
    g_screenActive = true;
    drawHeader("13 MECHANISMS -- POWERS");
    screenClear();
    mech13::Built& b = mech13::built();
    if (!b.attempted) {
        b.attempted = true;
        const std::size_t n = ms::projectConfigs(list, std::span<config::MechanismConfig>{b.configs});
        try {
            b.parts.build(std::span<const config::MechanismConfig>{b.configs, n}, mech13::provider());
        } catch (const std::exception& e) {
            b.failed = true;
            std::snprintf(b.why, sizeof b.why, "%.150s", e.what());
        }
    }
    if (b.failed) {
        emitS(Sev::Bad, "REFUSED: the mechanisms could not be made:");
        emitf("  %s", b.why);
        emit("  (that refusal IS the finding -- record it;");
        emit("  nothing was powered; it stays for this power cycle)");
        return;
    }
    st.markCleared();

    // ── each mechanism's boot lines first (the programs' words: the declaration, one per role) ──
    for (std::size_t i = 0; i < st.count(); ++i) {
        emit(mechanismDeclarationLine(list, i).text);
        for (const mechanism::RoleFact& f : b.parts.roles(i)) {
            char line[kMechanismLineBytes];
            mechanism::roleLine(line, sizeof line, list.specs[i].name, f.role, f.port,
                                static_cast<mechanism::Line>(f.line), f.state, f.refusal, f.degradeOnly);
            if (f.state == mechanism::RoleState::Refused) {
                emitS(Sev::Warn, "%s", line);
            } else {
                emit(line);
            }
        }
        if (list.specs[i].kind == mechanism::Kind::Power) {
            mechanismStateLines(list, i, st.built(i), false, [](const MechanismLine& l) { emit(l.text); });
        } else if (list.specs[i].kind != mechanism::Kind::Pneumatic) {
            char line[kMechanismLineBytes];
            mechanism::startingValuesLine(line, sizeof line, list.specs[i]);
            emit(line);
        }
    }
    emit("TOUCH to open the live page.");
    waitForTouch();

    // ── the live page ──────────────────────────────────────────────────────────────
    g_screenActive = false;  // serial + SD from here; the panel is the live view
    const std::uint32_t prints0 = g_screenPrints;
    const bool sdOn = g_card != nullptr && g_card->isOpen();
    hal::pros::ProsController master{hal::pros::ControllerId::Master};

    constexpr std::size_t kShown = ms::Station::kShownLines;
    constexpr std::int16_t kTitleY = 40;
    constexpr std::int16_t kLinesY = 64;
    constexpr std::int16_t kGatesY = static_cast<std::int16_t>(kLinesY + kShown * kLineH);
    static_assert(kGatesY + kLineH <= kStatusY, "the gates line must end above the status line");
    LiveLine title, lines[kShown], gates, status;
    bool rowArmed = false;
    int rowKind = -1;  // what the row shows: 0 NEXT/RECORD/DONE, 3 three answers, 4 four answers
    std::uint32_t rowSince = 0;
    char statusText[96] = "hold L1: UP/DOWN jog, A checks its direction";  // the panel cuts it at 50
    std::uint32_t statusPen = kColWarn;

    const auto layout = [&]() {
        drawHeader("13 MECHANISMS -- POWERS");
        screenClear();
        title.place(pros::E_TEXT_MEDIUM, kTitleY, kMedium);
        for (std::size_t k = 0; k < kShown; ++k) {
            lines[k].place(pros::E_TEXT_SMALL, static_cast<std::int16_t>(kLinesY + static_cast<int>(k) * kLineH),
                           kSmall);
        }
        gates.place(pros::E_TEXT_SMALL, kGatesY, kSmall);
        status.place(pros::E_TEXT_SMALL, kStatusY, kSmall);
        rowKind = -1;
    };
    const auto drawRow = [&](int kind, bool armed) {
        pros::c::screen_set_eraser(kColBg);
        pros::c::screen_erase_rect(0, menu_grid::kStationRow.y0, kUsableW, kUsableH);
        if (kind == 0) {
            drawRowButton(0, "NEXT", "mechanism", armed);
            drawRowButton(1, "RECORD", "", armed);
            drawRowButton(2, "DONE", "all safe", armed);
            return;
        }
        for (int k = 0; k < st.answerCount(); ++k) {
            char l1[16], l2[16];
            st.answerWords(k, l1, l2);
            if (kind == 4) {
                mech13::drawAnswer4(k, l1, l2, armed);
            } else {
                drawRowButton(k, l1, l2, armed);
            }
        }
    };
    layout();

    TapWatch taps;
    const std::uint32_t t0 = pros::millis();
    std::uint32_t lastPaint = t0 - ms::kPaintMs;
    int records = 0;
    bool done = false;
    // A NEW verdict takes the reading lines until the operator moves on (L1 PRESSED again, NEXT, or 20 s).
    std::uint32_t verdictsShown = 0;
    std::uint32_t verdictSince = 0;
    bool showVerdict = false;
    hal::ButtonEdge l1Edge;
    std::size_t readPage = 0;  // the reading lines' page (more lines than the panel shows: a tap steps it)
    int shownKind = -1;        // what the reading lines show: 0 readings, 1 the question, 2 the announcement, 3 a verdict
    constexpr std::size_t kBufLines = 20;  // a verdict's whole sentence, wrapped (459 characters at most) — with room
    static_assert(kBufLines >= ms::Station::kMaxPanelLines, "every reading line of a mechanism has a place");
    while (!done) {
        const std::uint32_t now = pros::millis();

        // ── the row: what it shows follows the station's phase; it arms 0.5 s after it changes ──
        const int wantKind = st.phase() == ms::Phase::Asking ? (st.answerCount() > 3 ? 4 : 3) : 0;
        if (wantKind != rowKind) {
            rowKind = wantKind;
            rowArmed = false;
            rowSince = now;
            drawRow(rowKind, false);
        }
        if (!rowArmed && now - rowSince >= kArmMs) {
            rowArmed = true;
            drawRow(rowKind, true);
        }
        int tx = 0, ty = 0;
        if (taps.poll(tx, ty)) {
            const int hit = menu_grid::hitTest(rowKind == 4 ? menu_grid::kAnswerRow4Hit : menu_grid::kStationRowHit,
                                               tx, ty);
            emitf("[R3A] mech touch x=%d y=%d -> slot %d%s", tx, ty, hit, rowArmed ? "" : " (IGNORED: arming)");
            if (hit < 0 && ty >= kLinesY && ty < kGatesY) {
                ++readPage;  // a tap on the reading lines: their next page (it wraps)
                verdictSince = now;  // a verdict being read page by page stays up
                lastPaint = now - ms::kPaintMs;
            }
            if (rowArmed && hit >= 0) {
                if (rowKind == 0 && hit == 0) {
                    st.next();
                    showVerdict = false;
                    readPage = 0;
                    shownKind = -1;
                } else if (rowKind == 0 && hit == 1) {
                    ++records;
                    int written = 0;
                    emitf("[R3A] mech REC %d: ceiling %.0f V; the page's numbers (INVENTED, HA-182): announce %.2f s,"
                          " check jog %.2f s, did-not-move %.3f rad, drawing %.2f A, cut %.1f A / 250 ms, jog max %.1f s",
                          records, st.ceiling(), static_cast<double>(ms::kAnnounceMs) / 1000.0,
                          static_cast<double>(ms::kCheckJogMs) / 1000.0, ms::kMinMoveRad, ms::kDrawAmps,
                          ms::kCurrentLimitA, static_cast<double>(ms::kJogMaxMs) / 1000.0);
                    for (std::size_t i = 0; i < st.count(); ++i) {
                        st.record(records, i, mech13::rawTemperature, [&](const char* l) {
                            emit(l);
                            ++written;
                        });
                    }
                    if (g_card != nullptr) (void)g_card->flush();
                    std::snprintf(statusText, sizeof statusText, "RECORD %d: %d line(s) -> serial%s", records,
                                  written + 1, sdOn ? " + SD card" : " only (SD OFF)");
                    statusPen = sdOn ? kColGood : kColWarn;
                } else if (rowKind == 0 && hit == 2) {
                    done = true;
                } else if (rowKind != 0) {
                    st.answer(hit);
                }
            }
        }

        // ── the controller, the tick ─────────────────────────────────────────────────
        ms::Pad pad;
        pad.connected = master.isConnected();
        if (pad.connected) {
            pad.deadMan = master.pressed(hal::ControllerButton::L1);
            pad.raise = master.pressed(hal::ControllerButton::R1);
            pad.plus = master.pressed(hal::ControllerButton::Up);
            pad.minus = master.pressed(hal::ControllerButton::Down);
            pad.check = master.pressed(hal::ControllerButton::A);
            if (st.phase() == ms::Phase::Asking && master.pressed(hal::ControllerButton::B)) {
                st.cancelQuestion();
            }
        }
        if (!done) {
            st.tick(pad, now);
        }
        const bool l1Pressed = l1Edge.update(pad.connected && pad.deadMan);
        if (st.verdictCount() != verdictsShown) {
            verdictsShown = st.verdictCount();
            verdictSince = now;
            showVerdict = true;
            readPage = 0;
        } else if (showVerdict && (now - verdictSince >= 20000 || l1Pressed)) {
            showVerdict = false;  // L1 pressed again, or 20 s: back to the readings (the log keeps the verdict)
        }
        if (st.phase() == ms::Phase::Confirm) {
            // GATE 2, stage 2: nothing new is powered while it waits (the Station's rule); L1 let go — or the
            // controller dropped — closes it at once (promptHeld), and the Station makes safe what it powered.
            const mech13::Held answer = mech13::promptHeld(st.confirmText(), master);
            const bool heldNow = answer != mech13::Held::LetGo && master.isConnected() &&
                                 master.pressed(hal::ControllerButton::L1);
            st.confirm(answer == mech13::Held::Yes, heldNow);
            layout();
            taps = TapWatch{};  // the prompt's own tap is not a tap on this page
            lastPaint = pros::millis() - ms::kPaintMs;
        }
        // ── the station's log lines: serial + SD, the newest on the status line ────────
        for (std::size_t k = 0; k < st.eventCount(); ++k) {
            const char* e = st.event(k);
            emit(e);
            const char* shown = std::strncmp(e, "[R3A] mech ", 11) == 0 ? e + 11 : e;
            std::snprintf(statusText, sizeof statusText, "%.54s", shown);
            statusPen = std::strstr(e, "CUT") != nullptr || std::strstr(e, "REFUSED") != nullptr ? kColBad : kColWarn;
        }
        st.clearEvents();

        // ── the panel, at most 5 Hz ──────────────────────────────────────────────────
        if (now - lastPaint >= ms::kPaintMs) {
            lastPaint = now;
            const std::size_t i = st.selected();
            const mechanism::MechanismSpec& s = st.spec(i);
            char t[96];
            std::snprintf(t, sizeof t, "%u/%u '%.23s' %s%s", static_cast<unsigned>(i + 1),
                          static_cast<unsigned>(st.count()), s.name, mechanismKindWord(s),
                          !st.built(i) ? " ABSENT" : st.cut(i) ? " CUT" : st.live(i) ? " POWERED" : "");
            title.set(t, !st.built(i) || st.cut(i) ? kColBad : st.live(i) ? kColGood : kColText);
            // every line of what the page shows, then the page of them the panel has room for (a footer
            // says when there is more; a tap on the lines steps it)
            char shown[kBufLines][ms::Station::kPanelBytes]{};
            std::size_t all = 0;
            std::uint32_t pen = kColText;
            int kind = 0;
            if (st.phase() == ms::Phase::Asking) {
                char q[ms::Station::kTextBytes];
                st.questionText(q, sizeof q);
                all = ms::wrapText(q, kSmall, shown, kBufLines);
                pen = kColWarn;
                kind = 1;
            } else if (st.phase() == ms::Phase::Announcing) {
                char a[ms::Station::kTextBytes];
                st.announceText(a, sizeof a, now);
                all = ms::wrapText(a, kSmall, shown, kBufLines);
                pen = kColWarn;
                kind = 2;
            } else if (showVerdict && st.hasVerdict() && st.verdictOf() == i) {
                char v[ms::Station::kTextBytes];
                st.verdictPanelText(v, sizeof v);
                all = ms::wrapText(v, kSmall, shown, kBufLines);
                pen = st.verdict().finding == mechanism::DirectionFinding::Ok ? kColGood : kColBad;
                kind = 3;
            } else {
                all = st.panelLines(i, mech13::rawTemperature, shown, ms::Station::kMaxPanelLines);
            }
            if (kind != shownKind) {
                shownKind = kind;
                readPage = 0;
            }
            const ms::ReadingsPage page = ms::readingsPage(all, kShown, readPage);
            const std::size_t first = page.first;
            const std::size_t n = page.count;
            const bool paged = page.pages > 1;
            for (std::size_t k = 0; k < kShown; ++k) {
                if (paged && k + 1 == kShown) {
                    char foot[ms::Station::kPanelBytes];
                    ms::pageFooter(foot, sizeof foot, page);
                    lines[k].set(foot, kColDim);
                } else {
                    lines[k].set(k < n ? shown[first + k] : "", pen);
                }
            }
            char g[96];
            st.gatesText(i, pad.connected, g, sizeof g);
            const std::uint32_t gp = !pad.connected || st.cut(i) ? kColBad
                                     : st.phase() == ms::Phase::Jogging || st.live(i) ? kColGood
                                                                                       : kColWarn;
            gates.set(g, gp);
            status.set(statusText, statusPen);
        }
        pros::delay(ms::kTickMs);
    }

    // ── GATE 6: the exit -- every powered mechanism safe (the guard repeats it on scope exit) ──
    st.exit();
    for (std::size_t k = 0; k < st.eventCount(); ++k) emit(st.event(k));
    st.clearEvents();
    emitf("[R3A] mech LEFT after %d RECORD(s): every powered mechanism safe; screen prints this station %lu",
          records, static_cast<unsigned long>(g_screenPrints - prints0));
    g_screenActive = true;
    screenClear();
    emit("");
    emitf("13 MECHANISMS ended: %d RECORD(s); everything powered is safe", records);
    for (std::size_t i = 0; i < st.count(); ++i) {
        const mechanism::MechanismSpec& s = st.spec(i);
        emitf("  '%.23s' %s: %s", s.name, mechanismKindWord(s),
              !st.built(i) ? "ABSENT" : st.cut(i) ? "CUT (see the log)" : st.armed(i) ? "armed" : "not powered");
    }
    if (st.anyVerdict()) {
        // EVERY mechanism's latest verdict this entry (not only the last check's): a finding on any one is
        // the menu's FAIL dot
        setVerdict(st.anyFinding() ? Sev::Bad : Sev::Good);
        for (std::size_t i = 0; i < st.count(); ++i) {
            if (!st.hasVerdictFor(i)) {
                continue;
            }
            const bool ok = st.verdictFor(i).finding == mechanism::DirectionFinding::Ok;
            g_lineSev = ok ? Sev::Good : Sev::Bad;
            char v[ms::Station::kTextBytes];
            std::snprintf(v, sizeof v, "last check, '%.23s': %s: %s", st.spec(i).name,
                          ms::Station::findingWord(st.verdictFor(i).finding), st.verdictFor(i).fix);
            // every piece, however many (a fixed line budget would drop a long sentence's tail silently)
            ms::logPieces(v, nullptr, static_cast<std::size_t>(kScreenCols) - 2, [](const char* l) { emitf("  %s", l); });
        }
        g_lineSev = Sev::Info;
        emit("  nothing is saved: edit the robot file,");
        emit("  rebuild, upload, check again.");
    }
    emit(sdOn ? "  [R3A] mech lines: /usd/r3a_log.txt + serial"
              : "  SD OFF: [R3A] mech lines on serial only");
}

}  // namespace

namespace {

/// A menu entry carries its INSTRUCTIONS, not just its code.
///
/// Every test here reported findings and none of them said what the BENCHER has to
/// do -- and several are worthless without a physical action. Test 3 tells you
/// nothing unless somebody turns a wheel and runs it again; test 2 needs the robot
/// rotated while it watches. A person working alone at a bench cannot infer that
/// from a table of numbers, so each entry now states the action and the finish
/// condition, and both are printed into the SD log as part of the record.
///
/// THE LABEL AND THE BUTTON ARE SEPARATE SINCE R2a. `label` is the station's full name as the
/// LOG writes it (`>>>>>> <label>`) -- unchanged for 1-12, because the worksheet and every
/// earlier session's log quote it. `btn1`/`btn2` are what the three-column button has room
/// for: line 1 MEDIUM, up to kBtnLine1Cols characters, and (since MECH-A6's 36 px rows) line 2
/// SMALL, up to kBtnLine2Cols beside the status dot -- checked at compile time below.
struct MenuItem {
    const char* label;
    const char* btn1;  ///< the button's first line (MEDIUM): number and name
    const char* btn2;  ///< the button's second line (SMALL since MECH-A6), "" for none
    void (*run)(pros::c::v5_device_e_t*);
    bool handsOn;          ///< needs the bencher to physically do something
    const char* doThis;    ///< the action, imperative, one line
    const char* doneWhen;  ///< how they know it is finished
    bool powersMotors = false;  ///< a station that powers motors (10 and 13): RED stripe, not amber
};

pros::c::v5_device_e_t g_found[kMaxPort + 1] = {};

void tCensus(pros::c::v5_device_e_t* f)   { census(f); }
void tImu(pros::c::v5_device_e_t* f) {
    if (kChassis.imuPort == 0) {
        rule("STAGE 2  IMU");
        emitS(Sev::Bad, "IMU port UNSET in the chassis table (%s).", kChassis.robot);
        emit("Mount an IMU on any smart port, report the port number, and it goes into");
        emit("src/robots/chassis_table.hpp. Nothing is guessed in the meantime.");
        return;
    }
    reportImu(f[kChassis.imuPort] == pros::c::E_DEVICE_IMU);
}
void tMotors(pros::c::v5_device_e_t* f)   { reportMotors(f); }
void tPlatform(pros::c::v5_device_e_t*)   { reportPlatform(); }
void tLoopRate(pros::c::v5_device_e_t*)   { measureLoopRate(); }
void tSdProbe(pros::c::v5_device_e_t*)    { probeSdCard(); }
void tAll(pros::c::v5_device_e_t* f) {
    tSdProbe(f);
    tCensus(f);
    tImu(f);
    tMotors(f);
    tPlatform(f);
    tLoopRate(f);
}

constexpr MenuItem kMenu[] = {
    {"1  DEVICE CENSUS", "1 DEVICE", "CENSUS", &tCensus, false,
     "nothing - just read it",
     "you have noted every port and what is in it"},

    {"2  IMU + ROTATE", "2 IMU +", "ROTATE", &tImu, true,
     "turn the WHOLE ROBOT counter-clockwise (to its left)",
     "the big number ROSE while you turned. If it fell, say so"},

    {"3  MOTOR WATCH (live)", "3 MOTOR", "WATCH", &motorWatch, true,
     "push the robot forward, OR spin one wheel - it records itself",
     "the table showed UP/DOWN per port and you touched to save it"},

    {"4  BATT/CTRL", "4 BATT/CTRL", "", &tPlatform, true,
     "pair a controller to the brain if it says NOT CONNECTED",
     "controller reads CONNECTED (it unblocks 4 register entries)"},

    {"5  SD CARD PROBE", "5 SD CARD", "PROBE", &tSdProbe, false,
     "nothing - unless it fails, then reformat the card FAT32",
     "all three steps pass and the header chip reads PASS"},

    {"6  LOOP RATE", "6 LOOP RATE", "", &tLoopRate, false,
     "nothing - leave the robot still",
     "min/max/mean are printed"},

    {"7  RUN ALL", "7 RUN ALL", "", &tAll, true,
     "be ready to rotate the robot when test 2's readout appears",
     "every test above has run once"},

    {"8  MOTORS (static)", "8 MOTORS", "STATIC", &tMotors, false,
     "nothing - a one-shot snapshot. Use test 3 to capture movement",
     "you have seen each motor's raw and canonical position"},

    {"9  SCREEN RULER", "9 SCREEN", "RULER", &screenRuler, true,
     "read the four numbered questions off the panel and report them",
     "you have answered all four - they fix the layout constants"},

    {"10 DRIVE (POWERS)", "10 DRIVE", "POWERS", &driveStation, true,
     "WHEELS UP first (ground only after a clean 6 V wheels-up run). POWERS motors: hold L1",
     "you drove at 3 V wheels-up, raised the ceiling with R1, and touched to stop",
     true},

    // R2a (2026-09-24). Hands-on: the robot is held at taped distances / a piece is moved.
    {"11 DISTANCE", "11 DISTANCE", "", &distanceStation, true,
     "hold the robot square to a wall at 6, 12, 24 in (tape) - RECORD at each; then open space",
     "you have a RECORD number for 6, 12 and 24 in and for open space, per sensor"},

    {"12 AI VISION", "12 AI", "VISION", &aiVisionStation, true,
     "a game piece ~1 ft then ~2 ft ahead, moved left then right - RECORD each; FAMILY for a tag",
     "you have a RECORD number per spot, and which FAMILY sees your tag (if you have one)"},

    // MECH-A6 (2026-10-30). POWERS the robot's declared mechanisms, one at a time (Station P).
    {"13 MECHANISMS (POWERS)", "13 MECH", "POWERS", &mechanismsStation, true,
     "clear every mechanism's path. Hold L1: UP/DOWN jog, A checks its direction, R1 raises",
     "every mechanism has a direction verdict, readings RECORDed, switches pressed/released/unplugged",
     true},
};
constexpr int kMenuCount = static_cast<int>(sizeof kMenu / sizeof kMenu[0]);
static_assert(kMenuCount == kMaxMenu, "g_verdict[] must have one slot per menu entry");

// ═══ THE MENU GRID: 3 x 5 since MECH-A6 (bench_r3a.hpp's menu_grid, host-tested) ═══════════
// USABLE HEIGHT IS ~240, NOT 272. The panel is 480x272 but VEXos reserves the top
// strip for its own status bar, so anything drawn near y=272 is off-screen. The
// first layout was verified against 272 and PASSED -- the check was right, the
// constant was wrong, and the bottom row landed off the panel on real hardware
// (observed 2026-08-18). Everything stays inside y < 236: the 2 x 5 menu ended at
// y = 234 with 36 px buttons and could not take a sixth row (it would end near 274),
// so R2a's twelve stations went three across, four down, in 46 px buttons; MECH-A6's
// thirteenth needed a fifth row, so the buttons are 36 px again -- rows 38-74 / 78-114 /
// 118-154 / 158-194 / 198-234, room for 15 (the MECH brief's N5 (a)). The geometry and the hit
// test live in bench_r3a.hpp, PROS-free, where test/bench_menu_grid_test.cpp checks every pixel.
constexpr menu_grid::Spec kGrid = menu_grid::kMenu;
static_assert(kGrid.count == kMenuCount, "the grid must have one button per menu entry");

/// The label budgets, from the button's pixels at the proven upper bounds (472 px per
/// calibration::kMediumCols MEDIUM characters, and per kSmallCols SMALL ones -- R3d-PROGRESS
/// 15:43-15:50): line 1 (MEDIUM) runs from x0 + 8 to x1 - 1 (11 characters), line 2 (SMALL since
/// MECH-A6) stops short of the status dot (13). ESTIMATES until the SCREEN RULER is read (HA-136);
/// change the one constant and every label is re-checked.
constexpr std::size_t kBtnLine1Cols =
    static_cast<std::size_t>(menu_grid::line1Px(kGrid)) * calibration::kMediumCols / 472;
constexpr std::size_t kBtnLine2Cols =
    static_cast<std::size_t>(menu_grid::line2Px(kGrid)) * calibration::kSmallCols / 472;

/// Every menu literal fits where it is drawn, ASCII only, and without a '%'. The log label
/// and the instructions reach the panel through screenEmit()'s 54-column cut, so they are
/// checked for the characters only.
constexpr bool menuLiteralsFit() {
    for (const MenuItem& m : kMenu) {
        if (!panelLiteralFits(m.btn1, kBtnLine1Cols)) return false;
        if (!panelLiteralFits(m.btn2, kBtnLine2Cols)) return false;
        if (!panelLiteralFits(m.label, 200) || !panelLiteralFits(m.doThis, 200)
            || !panelLiteralFits(m.doneWhen, 200)) {
            return false;
        }
    }
    return true;
}
static_assert(menuLiteralsFit(), "a menu button line does not fit its budget, or a menu literal "
                                 "carries a non-ASCII byte or a '%'");

void drawMenu() {
    g_screenActive = false;
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();

    // ── header bar. Identity on the left, build stamp and SD state on the right,
    //    so "which build is this" and "am I being logged" are both answered
    //    without spending a row each on a 240px panel.
    pros::c::screen_set_pen(kColBar);
    pros::c::screen_fill_rect(0, 0, kUsableW, 32);
    pros::c::screen_set_eraser(kColBar);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 2, "Bench Tests");
    pros::c::screen_set_pen(kColSub);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 20, "READ-ONLY except 10 + 13 (power motors)");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 250, 20, "BUILD %s", kBuildStamp);

    // The sink opens its file at CONSTRUCTION, so a card inserted after the program
    // started is never picked up. An unattended bencher would otherwise run a whole
    // session believing it was logged (E1 principle 5: silent degradation is a bug).
    const bool logging = (g_card != nullptr) && g_card->isOpen();
    pros::c::screen_set_pen(logging ? kColGood : kColBad);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 250, 2,
                             logging ? "SD: logging" : "SD: OFF (test 5)");
    pros::c::screen_set_eraser(kColBg);

    for (int i = 0; i < kMenuCount; ++i) {
        const menu_grid::Rect r = menu_grid::buttonBox(kGrid, i);
        pros::c::screen_set_pen(kColBar);
        pros::c::screen_fill_rect(r.x0, r.y0, r.x1, r.y1);
        pros::c::screen_set_pen(kMenu[i].powersMotors ? kColBad : kColEdge);
        pros::c::screen_draw_rect(r.x0, r.y0, r.x1, r.y1);
        // An amber stripe means THIS ONE NEEDS YOUR HANDS. A RED stripe and edge mean
        // THIS ONE POWERS MOTORS (station 10 DRIVE, and 13 MECHANISMS since MECH-A6). Encoded
        // as form rather than words: the labels are already at the width the button allows.
        if (kMenu[i].handsOn) {
            pros::c::screen_set_pen(kMenu[i].powersMotors ? kColBad : kColWarn);
            pros::c::screen_fill_rect(
                r.x0, r.y0, static_cast<std::int16_t>(r.x0 + menu_grid::kStripeW - 1), r.y1);
        }
        // Text draws pen-on-eraser, so the eraser must match the button fill or
        // every label carries a black box behind it. Two lines since R2a: MEDIUM then, since
        // MECH-A6's 36 px rows, a SMALL second line.
        const auto tx = static_cast<std::int16_t>(r.x0 + menu_grid::kLabelInsetX);
        screenText(pros::E_TEXT_MEDIUM, tx, static_cast<std::int16_t>(r.y0 + menu_grid::kLine1DY),
                   kColText, kColBar, kMenu[i].btn1, kBtnLine1Cols);
        if (kMenu[i].btn2[0] != '\0') {
            screenText(pros::E_TEXT_SMALL, tx,
                       static_cast<std::int16_t>(r.y0 + menu_grid::kLine2DY), kColText, kColBar,
                       kMenu[i].btn2, kBtnLine2Cols);
        }
        pros::c::screen_set_eraser(kColBg);

        // Status dot: hollow until the test has run, then filled with its verdict.
        // This is the only thing on the menu that changes during a session, and it
        // answers "what have I already done" without a word of text. Bottom right of
        // every button, beside line 2 (bench_r3a.hpp's kDot* offsets, host-checked).
        const auto cx = static_cast<std::int16_t>(r.x1 - menu_grid::kDotDX);
        const auto cy = static_cast<std::int16_t>(r.y0 + menu_grid::kDotDY);
        if (g_verdict[i] < 0) {
            pros::c::screen_set_pen(kColEdge);
            pros::c::screen_draw_circle(cx, cy, menu_grid::kDotR);
        } else {
            pros::c::screen_set_pen(sevColour(static_cast<Sev>(g_verdict[i])));
            pros::c::screen_fill_circle(cx, cy, menu_grid::kDotR);
        }
    }
}

/// -1 when the touch landed outside the button grid, else the menu index. The rules --
/// no dead gap inside the grid, dead header and margins outside it, the right edge from the
/// rightmost column -- are menu_grid::hitTest's (bench_r3a.hpp), tested pixel by pixel on the
/// host, including against the 2 x 5 menu this replaced.
int hitTest(int tx, int ty) { return menu_grid::hitTest(kGrid, tx, ty); }

/// TOUCH READOUT (2026-09-10, robot two's brain). The menu registered no tap on a brain
/// whose own VEXos dialogs took taps fine, so the program now SAYS what it sees: every
/// change in the touch status -- state, x, y, press and release counts, and whether the
/// point hit a button -- goes to serial (and the SD log) and into the header bar, so
/// "the touchscreen does not work" becomes numbers instead of an argument. The two
/// failure modes this separates: the counters never move (the API is not delivering
/// taps to the program on this firmware) versus the counters move but x/y land outside
/// every button (a coordinate offset or a calibration problem). Diagnostic by design
/// and cheap to keep: the menu is idle whenever this prints.
void touchReadout(const pros::screen_touch_status_s_t& t, int hit) {
    emitf("touch: status=%d x=%d y=%d press=%ld release=%ld hit=%d",
          static_cast<int>(t.touch_status), static_cast<int>(t.x), static_cast<int>(t.y),
          static_cast<long>(t.press_count), static_cast<long>(t.release_count), hit);
    pros::c::screen_set_eraser(kColBar);
    pros::c::screen_set_pen(hit >= 0 ? kColGood : kColWarn);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 128, 4, "T %d,%d r%ld h%d   ",
                             static_cast<int>(t.x), static_cast<int>(t.y),
                             static_cast<long>(t.release_count), hit);
    pros::c::screen_set_eraser(kColBg);
}

}  // namespace

void testerInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    // Moved VERBATIM from src/main.cpp's initialize() at G1 Part 2c (the dispatcher forwards
    // here); the bytes on serial are unchanged but for MECH-A6's line naming station 13 (the
    // second station that powers). The X-drive graph is not constructed in this build — it lives
    // in src/programs/xdrive_program.cpp, which only the xdrive build calls.
    // ONE write() per line since SER1 (HA-189); the first carries the blank line before it. The
    // longest is 201 bytes (the status line, with 63 characters of bits).
    hal::pros::serialLine("\n[R3A] booted: bench-tester build [%s].", variantBeacon);
    hal::pros::serialLine("[R3A] read-only EXCEPT the tester's DRIVE station (R3b Session 2), "
                          "which powers");
    hal::pros::serialLine("[R3A] the drive motors through the hal/pros adapters behind six "
                          "safety gates --");
    hal::pros::serialLine("[R3A] never through the motion stack. The library has still not "
                          "driven a robot.");
    hal::pros::serialLine("[R3A] and its MECHANISMS station 13 (MECH-A6), which powers ONE declared "
                          "mechanism at a time behind its own six gates.");
    hal::pros::serialLine("[R3A] the X-drive object graph is deliberately NOT constructed.");
    hal::pros::serialLine("[R3A] competition status at boot: %s -- the tester runs only in DRIVER "
                          "CONTROL, enabled; any other state paints the screen with the reason.",
                          competitionStatusBits);
}

void runR3a() {
    // The SD text log, opened first so the banner itself is captured. One file per
    // boot (ProsBlockSink owns its FILE*), so a re-run means a power cycle.
    // 8.3-safe name on purpose: the vendored PROS headers document FAT32 as a
    // requirement but say nothing about name length, and a long name is a free
    // thing to rule out.
    static hal::pros::ProsBlockSink card{"r3a_log.txt"};
    g_card = &card;

    // Boot splash. Held for 2 s so the build stamp is seen even if a later stage
    // faults -- "which binary is actually running" must be answerable at a glance.
    pros::c::screen_set_eraser(0x000000);
    pros::c::screen_erase();
    pros::c::screen_set_pen(0x30C030);
    pros::c::screen_print_at(pros::E_TEXT_LARGE, 10, 40, "Bench Tests");
    pros::c::screen_set_pen(0xFFFFFF);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 10, 92, "BUILD %s", kBuildStamp);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 130, "if this stamp is not the one you just built,");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 146, "the upload did NOT land -- re-upload.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 172,
                             "read-only EXCEPT 10 DRIVE and 13 MECHANISMS (they power).");
    pros::delay(2000);

    emit("");
    emit("################################################################");
    emit("#  shulib R3a/R3b  BENCH TESTER  --  read-only EXCEPT 10 DRIVE + 13 MECHANISMS (POWER)");
    emit("#  (12 AI VISION writes the CAMERA's own settings -- never a motor; R2a)");
    emit("################################################################");
    // NEVER pass compiledBuildHash() straight to a %s: it returns std::string_view,
    // which is trivially copyable, so it goes through varargs WITHOUT A COMPILER
    // WARNING and vsnprintf reads its raw bytes as a char* -- a garbage pointer that
    // faults inside strlen. That was a real data-abort on the bench, 2026-08-18.
    // Also honours build_info.hpp's LOUDNESS CONTRACT: empty means MISSING, rendered
    // as an error, never as a plausible-looking placeholder.
    emitf("BUILD STAMP: %s   <-- must match the build you just uploaded", kBuildStamp);
    const std::string_view hash = diag::compiledBuildHash();
    if (hash.empty()) {
        emit("build hash : [ERROR] MISSING -- the build injected no hash (S18.5)");
    } else {
        emitf("build hash : %.*s", static_cast<int>(hash.size()), hash.data());
    }
    emitf("robot      : %s", kChassis.robot);
    {
        char l[80], r[80], imu[16];
        portsToString(kChassis.left, kChassis.leftCount, kChassis.signsMeasured, l, sizeof l);
        portsToString(kChassis.right, kChassis.rightCount, kChassis.signsMeasured, r, sizeof r);
        if (kChassis.imuPort != 0) {
            std::snprintf(imu, sizeof imu, "%u", static_cast<unsigned>(kChassis.imuPort));
        } else {
            std::snprintf(imu, sizeof imu, "UNSET");
        }
        emitf("table      : LEFT %s | RIGHT %s | cartridge %s | IMU %s | %s", l, r,
              cartridgeWord(kChassis.cartridge), imu,
              kChassis.measured ? "MEASURED" : "NOT measured");
        emitf("provenance : %s", kChassis.provenance);
        char missing[96];
        if (describeMissing(kChassis, missing, sizeof missing, false)) {
            emitS(Sev::Warn, "UNSET in the table: %s -- never guessed; the stations that need",
                  missing);
            emit("           them refuse until the build team reports and the table is filled.");
        }
        char why[96];
        if (!tableConsistent(kChassis, why, sizeof why)) {
            emitS(Sev::Bad, "TABLE CONTRADICTS ITSELF: %s -- fix src/robots/chassis_table.hpp first.", why);
        }
    }
    emit("!! SIDE LABELS ARE THE TABLE'S CLAIM, NOT A MEASUREMENT (R3a-PROGRESS S10.3);");
    emit("!! MOTOR WATCH measures polarity. Stations 10 DRIVE and 13 MECHANISMS power motors.");
    emitf("sd logging : %s", card.isOpen() ? "ON -> /usd/r3a_log.txt (overwritten each boot)"
                                           : "OFF -- no card at boot; screen output only");

    // The census runs once up front so every other test knows what exists; it is
    // also re-runnable from the menu.
    probeSdCard();   // BEFORE the census, so a card problem is the first thing seen
    census(g_found);
    card.flush();

    // ── the menu loop. Runs forever; the bencher drives it with no laptop. ──
    for (;;) {
        drawMenu();
        int choice = -1;
        // The baseline from a GOOD read (an E_TOUCH_ERROR read carries -1 everywhere -- R2a).
        std::int32_t seen = releaseBaseline();
        pros::screen_touch_status_s_t last = pros::c::screen_touch_status();
        // The at-rest values, once per menu draw -- hit-tested like any other reading, so a
        // tap that landed while a station was still printing shows where it WOULD have hit
        // rather than a false "h-1" (the first robot-two session read one exactly that way).
        touchReadout(last, hitTest(last.x, last.y));
        while (choice < 0) {
            const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
            if (t.touch_status != last.touch_status || t.x != last.x || t.y != last.y
                || t.press_count != last.press_count || t.release_count != last.release_count) {
                touchReadout(t, hitTest(t.x, t.y));  // what the program SEES, every change
                last = t;
            }
            // A NEW tap, not the at-rest state -- and not an ERROR read: that one carries -1 in
            // every field, and taking it as a tap made the NEXT good read a second "new" tap at
            // the STALE x/y (R2a, ruling R2a-R4). The readout above still shows it: evidence.
            if (t.touch_status != pros::E_TOUCH_ERROR && t.release_count != seen) {
                seen = t.release_count;
                choice = hitTest(t.x, t.y);  // -1 when it landed off any button: keep waiting
            }
            pros::delay(20);
        }

        screenClear();
        g_screenActive = true;
        g_lastVerdict = Sev::Info;          // each run judged on its own evidence
        emit("");
        emitf(">>>>>> %s", kMenu[choice].label);
        // The brief comes FIRST, before any data, and rides into the SD log with it.
        emitS(Sev::Warn, "DO NOW  : %s", kMenu[choice].doThis);
        emitS(Sev::Warn, "DONE IF : %s", kMenu[choice].doneWhen);
        g_lastVerdict = Sev::Info;          // the brief is instruction, not a verdict
        emit("");
        kMenu[choice].run(g_found);
        card.flush();
        if (choice < kMaxMenu) g_verdict[choice] = static_cast<int>(g_lastVerdict);

        g_screenActive = false;
        drawVerdictChip(g_lastVerdict);
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 6, kUsableH - 20, "TOUCH TO RETURN TO MENU");
        waitForTouch();
    }
}

}  // namespace shulib::bench
