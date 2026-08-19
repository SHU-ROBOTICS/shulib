// R3a — the bench validation entry point for the MEASURED tank robot.
//
// ═══ WHAT THIS IS ══════════════════════════════════════════════════════════════════
// A measuring instrument, not a configured robot. It answers "what is actually
// plugged into this brain, and does our conversion layer agree with the raw
// device?" — chunk R3a §4.2 item 1. No auton, no closed loop, and (see below)
// no motion of any kind.
//
// ═══ READ-ONLY — A DELIBERATE NARROWING OF §4.2, RECORDED ══════════════════════════
// R3a's brief says the entry point "commands open-loop voltages on request".
// THIS BUILD DOES NOT. The one thing open-loop voltage buys is identifying which
// port drives which wheel and in which direction — and that is obtainable with
// ZERO risk by turning a wheel BY HAND and watching the encoder move (bench
// worksheet Station 2). Powering 8 motors whose signs are unmeasured, on a robot
// whose port map is the thing under test, buys nothing the hand method does not
// and can lurch a 15-lb robot off a bench.
//   * If open-loop commands are wanted later, they belong behind an explicit
//     opt-in with the wheels off the ground, and they are a separate change.
//   * Recorded so this reads as a ruling rather than an omission.
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

#include "bench_r3a.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <span>
#include <string_view>

#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/hal/pros/battery.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/clock.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/hal/motor_conversion.hpp"
#include "shulib/math/angle.hpp"

namespace shulib::bench {
namespace {

// ── The wiring HYPOTHESIS. Measured 2026-08-13, amended by R3a-PROGRESS §9.1
//    (port 13 mechanically repaired ⇒ 8 motors, 4 per side, symmetric).
//    §5.3's "excluded, 4-vs-3 asymmetry" ruling is WITHDRAWN by §9.1.
//    Every number here is falsifiable by stage 1's census and is MEANT to be.
constexpr std::uint8_t kImuPort = 4;
constexpr std::int8_t kLeftPorts[] = {15, 16, 17, 18};
constexpr std::int8_t kRightPorts[] = {11, 12, 13, 14};
constexpr std::size_t kLeftCount = sizeof kLeftPorts / sizeof kLeftPorts[0];
constexpr std::size_t kRightCount = sizeof kRightPorts / sizeof kRightPorts[0];

/// Compiled-in build stamp. __DATE__/__TIME__ are evaluated when THIS translation
/// unit is compiled, so the value changes on every rebuild -- which is the point.
/// The git hash (build_info.hpp) identifies the COMMIT; this identifies the BUILD,
/// and only the second one answers "did my upload actually land?" A silent upload
/// failure on 2026-08-18 left an old binary running and cost a debugging cycle
/// because nothing on screen distinguished the two.
constexpr const char* kBuildStamp = __DATE__ " " __TIME__;

constexpr int kMaxPort = 21;   // PHYSICAL smart ports, 1..21 -- how humans and every
                               // other PROS API name them. found[] is indexed this way.

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

// ── Output: USB serial always, SD card when one is installed. ─────────────────
hal::pros::ProsBlockSink* g_card = nullptr;

void screenEmit(const char* line);  // fwd: defined with the screen helpers below

void emit(const char* line) {
    std::printf("%s\n", line);
    std::fflush(stdout);
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
constexpr int kScreenLines = (kFooterY - kContentY) / kLineH;

// Palette. Dark ground, one accent, one warn, one good -- enough to read at a
// glance from arm's length over a robot, and no more.
constexpr std::uint32_t kColBg = 0x101418, kColBar = 0x1E5AA8, kColText = 0xF0F0F0,
                        kColDim = 0x9AA4AE, kColGood = 0x39C36E, kColWarn = 0xE8B23A,
                        kColBad = 0xE2564A, kColSub = 0xC8D8F0, kColEdge = 0x6E9AD8;

/// Truncation width for the panel. The SD file keeps full width, so an over-long
/// line loses nothing that matters; an unreadable overflow would.
constexpr int kScreenCols = 54;

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
constexpr int kMaxMenu = 9;
int g_verdict[kMaxMenu] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};

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
void waitForTouch() {
    const std::int32_t start = pros::c::screen_touch_status().release_count;
    while (pros::c::screen_touch_status().release_count == start) {
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
    pros::c::screen_set_eraser(kColBar);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 6, "%.30s", title);
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
    char t[kScreenCols + 2];
    std::snprintf(t, sizeof t, "%.*s", kScreenCols, line);
    // Indented/continuation lines are supporting detail: dim them so the eye finds
    // the headline values first.
    pros::c::screen_set_pen(g_lineSev != Sev::Info ? sevColour(g_lineSev)
                            : (line[0] == ' ') ? kColDim
                                               : kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8,
                             static_cast<std::int16_t>(kContentY + g_screenLine * kLineH),
                             "%s", t);
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

/// Which side (if any) our hypothesis assigns this port — §10.3's mirror check.
const char* hypothesisedSide(int port) {
    for (std::size_t i = 0; i < kLeftCount; ++i) {
        if (kLeftPorts[i] == static_cast<std::int8_t>(port)) return "LEFT(hyp)";
    }
    for (std::size_t i = 0; i < kRightCount; ++i) {
        if (kRightPorts[i] == static_cast<std::int8_t>(port)) return "RIGHT(hyp)";
    }
    if (port == static_cast<int>(kImuPort)) return "IMU(hyp)";
    return "";
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
    emit("PORT  idx  type            our hypothesis");
    int motors = 0;
    for (int idx = 0; idx <= kMaxRegistryIndex; ++idx) {
        const pros::c::v5_device_e_t t =
            pros::c::registry_get_plugged_type(static_cast<std::uint8_t>(idx));
        const int port = idx + 1;   // <- the whole correction
        found[port] = t;
        if (t == pros::c::E_DEVICE_MOTOR) ++motors;
        if (t != pros::c::E_DEVICE_NONE) {
            emitf("  %2d   %2d  %-14s  %s  (code %d)", port, idx, deviceName(t),
                  hypothesisedSide(port), static_cast<int>(t));
        }
    }
    emit("");
    const unsigned expect = static_cast<unsigned>(kLeftCount + kRightCount);
    emitS(motors >= static_cast<int>(expect) ? Sev::Good : Sev::Bad,
          "motors found: %d   (hypothesis expects %u: %u left + %u right)", motors, expect,
          static_cast<unsigned>(kLeftCount), static_cast<unsigned>(kRightCount));
    emit("EMPTY PORTS ARE OMITTED. A hypothesised port missing here is a FINDING.");
    emit("Index 21+ is NOT scanned: apix.h documents 0-20, and reading past it is");
    emit("what produced the phantom 'ADI expander' on 2026-08-18 (HA-120 predicted it).");
}

// ═══ STAGE 2 — IMU: raw PROS beside our canonical conversion. ════════════════
void reportImu(bool present) {
    rule("STAGE 2  IMU (HA-02/03/04/05/23/108/109/110)");
    if (!present) {
        emitS(Sev::Bad, "no IMU on port %u -- hypothesis WRONG. See the census.",
              static_cast<unsigned>(kImuPort));
        return;
    }
    hal::pros::ProsClock clock{};
    try {
        hal::pros::ProsImu imu{kImuPort, math::Angle{}, clock};
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
                  pros::c::imu_get_rotation(kImuPort), imu.heading().radians(),
                  imu.heading().degrees());
            pros::delay(200);
        }
        emitf("  heading(raw)=%8.3f deg      |  pitch=%7.3f  roll=%7.3f (canonical rad)",
              pros::c::imu_get_heading(kImuPort), imu.pitch().radians(), imu.roll().radians());
        emitf("  screened reads (held last-good): %d", imu.faultedReads());
        emit("");
        emit("** NOW ROTATE THE ROBOT COUNTER-CLOCKWISE (to its left, seen from above). **");
        emit("   Canonical heading MUST INCREASE. If it decreases, HA-02's sign is wrong");
        emit("   and every turn this library ever commands would be mirrored.");

        // A big fixed readout rather than a scrolling log: this is a number somebody
        // watches WHILE turning the robot with both hands.
        const double startDeg = imu.heading().degrees();
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
        char big[64];
        for (int i = 0; i < 100; ++i) {  // ~20 s to rotate under
            const double deg = imu.heading().degrees();
            std::snprintf(big, sizeof big, "%+8.2f deg", deg);
            screenBig(146, big);
            std::snprintf(big, sizeof big, "delta %+.2f", deg - startDeg);
            pros::c::screen_set_eraser(kColBg);
            pros::c::screen_erase_rect(0, 188, kUsableW, 210);
            pros::c::screen_set_pen((deg - startDeg) >= 0.0 ? kColGood : kColBad);
            pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 190, "%-28s", big);
            emitf("   raw=%9.3f deg   canonical=%9.4f rad = %8.3f deg",
                  pros::c::imu_get_rotation(kImuPort), imu.heading().radians(), deg);
            pros::delay(200);
        }
        const double moved = imu.heading().degrees() - startDeg;

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
        emitf("heading moved %+.2f deg; operator says they turned %s", moved,
              turnedCcw ? "LEFT / CCW" : "RIGHT / CW");
        // Canonical is CCW-positive by F1. So a CCW turn must raise it.
        const bool agrees = turnedCcw ? (moved > 0.0) : (moved < 0.0);
        if (moved > -5.0 && moved < 5.0) {
            emitS(Sev::Warn, "barely moved -- turn it further and re-run.");
        } else if (agrees) {
            emitS(Sev::Good, "HA-02 CONFIRMED: canonical heading is CCW-POSITIVE.");
        } else {
            emitS(Sev::Bad, "HA-02 WRONG: the sign is INVERTED. Every turn would mirror.");
        }
    } catch (const PreconditionError& e) {
        // The rotate test suspends the scrolling log; restore it or this report
        // would land on serial and the SD card but never on the panel the
        // bencher is actually looking at.
        g_screenActive = true;
        emitS(Sev::Bad, "IMU ADAPTER REFUSED THE DEVICE: %s", e.what());
        emit("(that refusal IS the measurement -- record it verbatim)");
    }
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
void reportMotorGroup(const char* label, const std::int8_t* ports, std::size_t n,
                      const pros::c::v5_device_e_t* found) {
    emitf("-- %s --", label);
    for (std::size_t i = 0; i < n; ++i) {
        const int p = static_cast<int>(ports[i]);
        if (found[p] != pros::c::E_DEVICE_MOTOR) {
            emitS(Sev::Bad, "  port %2d: NOT A MOTOR (census says %s)", p, deviceName(found[p]));
            continue;
        }
        const double rawDeg = pros::c::motor_get_position(ports[i]);
        emitf("  p%-2d %s raw=%8.1fd canon=%7.3fr %4.1fC %4dmA", p,
              gearName(pros::c::motor_get_gearing(ports[i])), rawDeg,
              hal::motorPositionDegToCanonical(rawDeg).value(),
              pros::c::motor_get_temperature(ports[i]),
              static_cast<int>(pros::c::motor_get_current_draw(ports[i])));
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
    emit("Nothing here writes to a motor: this build never constructs the adapter");
    emit("that would (see the header note above reportMotorGroup).");
    emit("");
    reportMotorGroup("HYPOTHESISED LEFT", kLeftPorts, kLeftCount, found);
    reportMotorGroup("HYPOTHESISED RIGHT", kRightPorts, kRightCount, found);
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

/// MOTOR WATCH -- live capture, so nobody has to remember numbers.
///
/// The read-remember-re-run loop it replaces asked a person to note eight values,
/// perform a physical action, run the test again, and diff two tables in their
/// head. That is a transcription task handed to the one participant who cannot be
/// re-run. This zeroes a baseline, then displays every drive port's DELTA live
/// while the robot is pushed or a wheel is spun, and writes the finished table to
/// serial and the SD card on exit.
///
/// Still READ-ONLY: positions are read straight from PROS, no adapter is
/// constructed, and no motor is ever powered.
void motorWatch(pros::c::v5_device_e_t* found) {
    rule("MOTOR WATCH (live)");

    // Collect the drive ports that really are motors, in hypothesis order.
    std::int8_t ports[kLeftCount + kRightCount];
    const char* side[kLeftCount + kRightCount];
    std::size_t n = 0;
    for (std::size_t i = 0; i < kLeftCount; ++i) {
        if (found[kLeftPorts[i]] == pros::c::E_DEVICE_MOTOR) {
            ports[n] = kLeftPorts[i]; side[n] = "L"; ++n;
        }
    }
    for (std::size_t i = 0; i < kRightCount; ++i) {
        if (found[kRightPorts[i]] == pros::c::E_DEVICE_MOTOR) {
            ports[n] = kRightPorts[i]; side[n] = "R"; ++n;
        }
    }
    if (n == 0) {
        emitS(Sev::Bad, "no drive motors found -- run the census first.");
        return;
    }

    double base[kLeftCount + kRightCount];
    for (std::size_t i = 0; i < n; ++i) base[i] = pros::c::motor_get_position(ports[i]);

    emit("Baseline taken. Now do ONE of these:");
    emit("  * PUSH THE WHOLE ROBOT FORWARD  -> every port moves; the SIGNS");
    emit("    are the answer to 'which way is positive'.");
    emit("  * SPIN ONE WHEEL (any direction) -> only that port moves; that");
    emit("    is the port->wheel map.");
    emit("Watch the panel. TOUCH when you are done and it records the table.");

    g_screenActive = false;
    screenClear();
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 40, "push the robot, or spin one wheel");
    pros::c::screen_set_pen(kColWarn);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, kFooterY, "TOUCH when done - it records itself");

    // Live table: two columns of four, updated in place until a touch arrives.
    const std::int32_t startTouch = pros::c::screen_touch_status().release_count;
    char cell[40];
    while (pros::c::screen_touch_status().release_count == startTouch) {
        for (std::size_t i = 0; i < n; ++i) {
            const double d = pros::c::motor_get_position(ports[i]) - base[i];
            const std::int16_t cx = static_cast<std::int16_t>(i < 4 ? 8 : 248);
            const std::int16_t cy = static_cast<std::int16_t>(60 + (i % 4) * 34);
            pros::c::screen_set_eraser(kColBg);
            pros::c::screen_erase_rect(cx, cy, static_cast<std::int16_t>(cx + 224),
                                       static_cast<std::int16_t>(cy + 30));
            // Colour IS the reading: green rose, red fell, dim did not move.
            pros::c::screen_set_pen(d > 5.0 ? kColGood : d < -5.0 ? kColBad : kColDim);
            std::snprintf(cell, sizeof cell, "%s%-2d %s%8.0f", side[i], static_cast<int>(ports[i]),
                          d > 5.0 ? "UP  " : d < -5.0 ? "DOWN" : "--  ", d);
            pros::c::screen_print_at(pros::E_TEXT_MEDIUM, cx, cy, "%s", cell);
        }
        pros::delay(80);
    }

    g_screenActive = true;
    screenClear();
    emit("");
    emit("RECORDED -- net movement per port since baseline:");
    emit("side port      delta deg   verdict");
    int moved = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double d = pros::c::motor_get_position(ports[i]) - base[i];
        const bool did = (d > 5.0 || d < -5.0);
        if (did) ++moved;
        emitS(did ? Sev::Good : Sev::Info, "  %s  %2d  %10.0f   %s", side[i],
              static_cast<int>(ports[i]), d, d > 5.0 ? "UP" : d < -5.0 ? "DOWN" : "did not move");
    }
    emit("");
    if (moved == 0) {
        emitS(Sev::Warn, "nothing moved. Push harder, or check the census.");
    } else if (moved == 1) {
        emit("ONE port moved -> that port drives the wheel you spun.");
    } else {
        emit("SEVERAL moved -> this was a whole-robot push. The UP/DOWN");
        emit("column IS the sign convention, relative to the front you chose.");
        emit("Ports disagreeing with their side-mates are wired reversed.");
    }
    emit("This table is in /usd/r3a_log.txt. Re-run per wheel to build the map.");
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
struct MenuItem {
    const char* label;
    void (*run)(pros::c::v5_device_e_t*);
    bool handsOn;          ///< needs the bencher to physically do something
    const char* doThis;    ///< the action, imperative, one line
    const char* doneWhen;  ///< how they know it is finished
};

pros::c::v5_device_e_t g_found[kMaxPort + 1] = {};

void tCensus(pros::c::v5_device_e_t* f)   { census(f); }
void tImu(pros::c::v5_device_e_t* f)      { reportImu(f[kImuPort] == pros::c::E_DEVICE_IMU); }
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
    {"1  DEVICE CENSUS", &tCensus, false,
     "nothing - just read it",
     "you have noted every port and what is in it"},

    {"2  IMU + ROTATE", &tImu, true,
     "turn the WHOLE ROBOT counter-clockwise (to its left)",
     "the big number ROSE while you turned. If it fell, say so"},

    {"3  MOTOR WATCH (live)", &motorWatch, true,
     "push the robot forward, OR spin one wheel - it records itself",
     "the table showed UP/DOWN per port and you touched to save it"},

    {"4  BATT/CTRL", &tPlatform, true,
     "pair a controller to the brain if it says NOT CONNECTED",
     "controller reads CONNECTED (it unblocks 4 register entries)"},

    {"5  SD CARD PROBE", &tSdProbe, false,
     "nothing - unless it fails, then reformat the card FAT32",
     "all three steps pass and the header chip reads PASS"},

    {"6  LOOP RATE", &tLoopRate, false,
     "nothing - leave the robot still",
     "min/max/mean are printed"},

    {"7  RUN ALL", &tAll, true,
     "be ready to rotate the robot when test 2's readout appears",
     "every test above has run once"},

    {"8  MOTORS (static)", &tMotors, false,
     "nothing - a one-shot snapshot. Use test 3 to capture movement",
     "you have seen each motor's raw and canonical position"},

    {"9  SCREEN RULER", &screenRuler, true,
     "read the four numbered questions off the panel and report them",
     "you have answered all four - they fix the layout constants"},
};
constexpr int kMenuCount = static_cast<int>(sizeof kMenu / sizeof kMenu[0]);

// Two columns x three rows of touch targets. Deliberately large (232x62): this is
// operated by someone crouched over a robot, not with a mouse.
// USABLE HEIGHT IS ~240, NOT 272. The panel is 480x272 but VEXos reserves the top
// strip for its own status bar, so anything drawn near y=272 is off-screen. The
// first layout was verified against 272 and PASSED -- the check was right, the
// constant was wrong, and the bottom row landed off the panel on real hardware
// (observed 2026-08-18). Everything now stays inside y < 236 for margin:
// row 3 spans 184..226.
// 9 items => 5 rows on a 240px panel. 38 + 5*36 + 4*4 = 234, inside the bound
// with margin. Verified on the host before upload, against the CORRECT height.
constexpr std::int16_t kBtnW = 232, kBtnH = 36, kBtnX0 = 6, kBtnY0 = 38, kGap = 4;

void buttonBox(int i, std::int16_t& x0, std::int16_t& y0, std::int16_t& x1, std::int16_t& y1) {
    const std::int16_t col = static_cast<std::int16_t>(i % 2);
    const std::int16_t row = static_cast<std::int16_t>(i / 2);
    x0 = static_cast<std::int16_t>(kBtnX0 + col * (kBtnW + kGap));
    y0 = static_cast<std::int16_t>(kBtnY0 + row * (kBtnH + kGap));
    x1 = static_cast<std::int16_t>(x0 + kBtnW);
    y1 = static_cast<std::int16_t>(y0 + kBtnH);
}

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
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 20, "READ-ONLY - commands no motion");
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
        std::int16_t x0, y0, x1, y1;
        buttonBox(i, x0, y0, x1, y1);
        pros::c::screen_set_pen(kColBar);
        pros::c::screen_fill_rect(x0, y0, x1, y1);
        pros::c::screen_set_pen(kColEdge);
        pros::c::screen_draw_rect(x0, y0, x1, y1);
        // An amber stripe means THIS ONE NEEDS YOUR HANDS. Encoded as form rather
        // than words: the labels are already at the width the button allows.
        if (kMenu[i].handsOn) {
            pros::c::screen_set_pen(kColWarn);
            pros::c::screen_fill_rect(x0, y0, static_cast<std::int16_t>(x0 + 5), y1);
        }
        // Text draws pen-on-eraser, so the eraser must match the button fill or
        // every label carries a black box behind it.
        pros::c::screen_set_eraser(kColBar);
        pros::c::screen_set_pen(kColText);
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, static_cast<std::int16_t>(x0 + 10),
                                 static_cast<std::int16_t>(y0 + 10), "%s", kMenu[i].label);
        pros::c::screen_set_eraser(kColBg);

        // Status dot: hollow until the test has run, then filled with its verdict.
        // This is the only thing on the menu that changes during a session, and it
        // answers "what have I already done" without a word of text.
        const std::int16_t cx = static_cast<std::int16_t>(x1 - 16);
        const std::int16_t cy = static_cast<std::int16_t>(y0 + kBtnH / 2);
        if (g_verdict[i] < 0) {
            pros::c::screen_set_pen(kColEdge);
            pros::c::screen_draw_circle(cx, cy, 6);
        } else {
            pros::c::screen_set_pen(sevColour(static_cast<Sev>(g_verdict[i])));
            pros::c::screen_fill_circle(cx, cy, 6);
        }
    }
}

/// -1 when the touch landed outside every button.
int hitTest(std::int16_t tx, std::int16_t ty) {
    for (int i = 0; i < kMenuCount; ++i) {
        std::int16_t x0, y0, x1, y1;
        buttonBox(i, x0, y0, x1, y1);
        if (tx >= x0 && tx <= x1 && ty >= y0 && ty <= y1) return i;
    }
    return -1;
}

}  // namespace

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
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 172, "READ-ONLY build: commands no motion.");
    pros::delay(2000);

    emit("");
    emit("################################################################");
    emit("#  shulib R3a  BENCH VALIDATION  --  READ-ONLY, COMMANDS NO MOTION");
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
    emit("robot      : TANK BENCH BOT (the measured one) -- NOT the invented X-drive");
    emitf("hypothesis : LEFT %d/%d/%d/%d   RIGHT %d/%d/%d/%d   IMU %u   no GPS, no pods",
          kLeftPorts[0], kLeftPorts[1], kLeftPorts[2], kLeftPorts[3], kRightPorts[0],
          kRightPorts[1], kRightPorts[2], kRightPorts[3], static_cast<unsigned>(kImuPort));
    emit("!! SIDE LABELS ARE A HYPOTHESIS, NOT A CONFIGURATION (R3a-PROGRESS S10.3).");
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
        std::int32_t seen = pros::c::screen_touch_status().release_count;
        while (choice < 0) {
            const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
            if (t.release_count != seen) {   // a NEW tap, not the at-rest state
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
