// R3a — the bench validation entry point for the MEASURED tank robot.
//
// ═══ WHAT THIS IS ══════════════════════════════════════════════════════════════════
// A measuring instrument, not a configured robot. It answers "what is actually
// plugged into this brain, and does our conversion layer agree with the raw
// device?" — chunk R3a §4.2 item 1. No auton, no closed loop, and (see below)
// no motion of any kind.
//
// ═══ READ-ONLY, WITH ONE POWERED STATION — A RULING, AMENDED ONCE ═══════════════════
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
//     is still read-only, constructs no motor adapter, and never will.
//   * It drives the robot through the library's ADAPTERS (hal/pros ProsMotor and
//     ProsController), never its motion stack -- the library has still not
//     driven a robot, and this station is labelled that way on every screen.
//   * The drivetrain it was written for is a SECOND robot: the season's tank
//     chassis, five COUPLED motors per side. Its chassis table below ships UNSET
//     (no port is ever invented -- HA-111 was that defect); the bench bot keeps
//     its measured table.
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

#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/battery.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/clock.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/hal/pros/line_display.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/hal/motor_conversion.hpp"
#include "shulib/math/angle.hpp"
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

// ═══ THE CHASSIS TABLE — the ONE typed input (R3b Session 2, brief §3.1) ═══════════
// Per variant: which ports drive which side, the cartridge, the IMU port, and whether
// any of it was MEASURED. It holds what the tester needs about a drivetrain and cannot
// discover for itself -- polarity is discovered (MOTOR WATCH's hand push), and R3d will
// discover sides and scale too. Selected by the SAME define the Makefile's ROBOT switch
// sets, so `make ROBOT=tank` and this table cannot disagree about which robot it is.
//
// RULE: A VALUE IS NEVER INVENTED. An unset field is 0 / Unset, describeMissing() names
// it on screen, and the DRIVE station refuses until it is filled. HA-111's invented port
// map was exactly this defect class -- every adapter constructor threw at boot -- and it
// is not repeated for robot two. When the build team reports ports, THIS is the one
// place to edit (the tank table), with the date and the source beside the numbers.
enum class Cartridge { Unset, Red, Green, Blue };

struct ChassisTable {
    const char* robot;            // the banner's name for this robot
    std::int8_t left[kMaxPort];   // LEFT-side drive ports (physical 1..21), any order
    std::size_t leftCount;        // 0 = UNSET
    std::int8_t right[kMaxPort];  // RIGHT-side drive ports
    std::size_t rightCount;       // 0 = UNSET
    Cartridge cartridge;          // Unset = nobody has READ one off a motor yet
    std::uint8_t imuPort;         // 0 = UNSET (none mounted, or not reported)
    bool measured;                // true only when every SET field was read off the robot
    const char* provenance;       // who measured what, and when -- printed in the banner
};

#if defined(SHULIB_ROBOT_TANK_2026)
// ROBOT TWO -- the 2026 tank chassis (build team, stated 2026-09-10): five COUPLED motors
// per side driving four wheels per side.
// PORTS: reported by the team lead 2026-09-10, read off the robot while standing BEHIND
// it (the back of the robot against him, looking toward the front): LEFT 11 12 13 14 15
// and RIGHT 20 19 18 17 16, each listed BACK -> FRONT (11 and 20 are the rearmost motors,
// 15 and 16 the frontmost). THE FRONT is therefore the end where ports 15 and 16 sit --
// the end away from a person who reads the left side as 11..15. Order within a side is
// informational: the table is a set, and MOTOR WATCH captures each port's own sign.
// CARTRIDGE: BLUE, read off a motor by the build team 2026-09-10 -- a reported
// measurement, so it may be set; it is still printed as a BELIEF wherever the adapter is
// about to WRITE it. IMU: none mounted -> UNSET. Also reported, and deliberately NOT here
// because the tester uses neither: wheels 2.75 in, and "600 rpm" (reads as direct drive
// 1:1, UNCONFIRMED until tooth counts arrive -- that belongs to odometry, R3b Part 2).
// `measured` flipped TRUE on the evening of 2026-09-10: station 1's census showed MOTOR on
// all ten table ports (after cables on 11, 15 and 19 were re-seated), the IMU on port 2 and
// the radio on port 1, and two whole-robot pushes captured every port's sign
// (LEFT -11 +12 -13 +14 -15 | RIGHT +16 -17 +18 -19 +20, port 18 travelling ~20 % short
// both times). The cartridge is still a belief read off a motor, not a measurement.
constexpr ChassisTable kChassis = {
    .robot = "2026 TANK CHASSIS (robot two)",
    .left = {11, 12, 13, 14, 15},
    .leftCount = 5,
    .right = {20, 19, 18, 17, 16},
    .rightCount = 5,
    .cartridge = Cartridge::Blue,
    .imuPort = 2,
    .measured = true,
    .provenance = "ports + IMU 2 + radio 1: census 2026-09-10 evening, all ten motors; "
                  "front = the 15/16 end (team lead); cartridge BLUE off a motor",
};
#else
// THE BENCH BOT -- measured. 2026-08-13 census, amended by R3a-PROGRESS §9.1 (port 13
// mechanically repaired => 8 motors, 4 per side, symmetric; §5.3's asymmetry ruling
// withdrawn) and the sides confirmed by the §15-§19 whole-robot pushes. IMU on port 4.
// Cartridge BLUE per the team, while the brain was found configured GREEN against it
// (§20.3, "the cartridge fix is still owed") -- so the value here is the team's, and the
// DRIVE station prints it as a belief before writing it; reading the insert colour off a
// motor is worksheet 3.3 and is what settles HA-15.
constexpr ChassisTable kChassis = {
    .robot = "TANK BENCH BOT (measured) -- NOT the invented X-drive",
    .left = {15, 16, 17, 18},
    .leftCount = 4,
    .right = {11, 12, 13, 14},
    .rightCount = 4,
    .cartridge = Cartridge::Blue,
    .imuPort = 4,
    .measured = true,
    .provenance = "ports+IMU: 2026-08-13 census, sides by R3a-PROGRESS S15-S19 pushes; "
                  "cartridge BLUE per the team (brain was GREEN, S20.3) -- read the insert",
};
#endif

/// Both sides have ports. (The IMU and the cartridge are checked separately, because the
/// stations that need them differ: DRIVE needs ports + cartridge, the IMU test needs the
/// IMU port, the census needs nothing.)
constexpr bool tableHasPorts() { return kChassis.leftCount > 0 && kChassis.rightCount > 0; }
constexpr std::size_t tableCount() { return kChassis.leftCount + kChassis.rightCount; }

const char* cartridgeWord(Cartridge c) {
    switch (c) {
        case Cartridge::Red:   return "RED 100 rpm";
        case Cartridge::Green: return "GREEN 200 rpm";
        case Cartridge::Blue:  return "BLUE 600 rpm";
        default:               return "UNSET";
    }
}

/// -1 LEFT, +1 RIGHT, 0 not a drive port in the table (or the table has no ports).
int tableSideOf(int port) {
    for (std::size_t i = 0; i < kChassis.leftCount; ++i) {
        if (kChassis.left[i] == static_cast<std::int8_t>(port)) return -1;
    }
    for (std::size_t i = 0; i < kChassis.rightCount; ++i) {
        if (kChassis.right[i] == static_cast<std::int8_t>(port)) return +1;
    }
    return 0;
}

/// The side label printed beside a port -- §10.3's mirror check, now from the table.
const char* tableSide(int port) {
    const int s = tableSideOf(port);
    if (s < 0) return "LEFT(table)";
    if (s > 0) return "RIGHT(table)";
    if (kChassis.imuPort != 0 && port == static_cast<int>(kChassis.imuPort)) return "IMU(table)";
    return "";
}

/// One letter for a live panel cell: L / R / ? (the table has no claim on this port).
const char* sideLetter(int port) {
    const int s = tableSideOf(port);
    return s < 0 ? "L" : s > 0 ? "R" : "?";
}

/// Names every field a caller needs that is UNSET, comma-separated, into `buf`.
/// Returns true when something is missing. `forDrive` = ports + cartridge; otherwise
/// every field, IMU included -- the banner uses the full list.
bool describeMissing(char* buf, std::size_t n, bool forDrive) {
    buf[0] = '\0';
    bool any = false;
    auto add = [&](const char* what) {
        if (any) std::strncat(buf, ", ", n - std::strlen(buf) - 1);
        std::strncat(buf, what, n - std::strlen(buf) - 1);
        any = true;
    };
    if (kChassis.leftCount == 0) add("LEFT ports");
    if (kChassis.rightCount == 0) add("RIGHT ports");
    if (kChassis.cartridge == Cartridge::Unset) add("cartridge");
    if (!forDrive && kChassis.imuPort == 0) add("IMU port");
    return any;
}

/// "15 16 17 18" or "UNSET".
void portsToString(const std::int8_t* ports, std::size_t n, char* buf, std::size_t cap) {
    buf[0] = '\0';
    if (n == 0) {
        std::snprintf(buf, cap, "UNSET");
        return;
    }
    for (std::size_t i = 0; i < n; ++i) {
        char one[8];
        std::snprintf(one, sizeof one, "%s%d", i ? " " : "", static_cast<int>(ports[i]));
        std::strncat(buf, one, cap - std::strlen(buf) - 1);
    }
}

/// A table that contradicts itself is worse than an unset one: a port on both sides, a
/// port out of 1..21, or the IMU port listed as a drive port. Printed at boot, refused by
/// DRIVE. Returns true when the table is internally consistent.
bool tableConsistent(char* why, std::size_t cap) {
    why[0] = '\0';
    auto inRange = [](std::int8_t p) { return p >= 1 && p <= kMaxPort; };
    for (std::size_t i = 0; i < kChassis.leftCount; ++i) {
        if (!inRange(kChassis.left[i])) { std::snprintf(why, cap, "LEFT port %d is not 1..21", kChassis.left[i]); return false; }
        if (tableSideOf(kChassis.left[i]) != -1 || (kChassis.imuPort != 0 && kChassis.left[i] == static_cast<std::int8_t>(kChassis.imuPort))) {
            std::snprintf(why, cap, "port %d appears on both sides or is the IMU port", kChassis.left[i]);
            return false;
        }
        for (std::size_t j = i + 1; j < kChassis.leftCount; ++j) {
            if (kChassis.left[i] == kChassis.left[j]) { std::snprintf(why, cap, "LEFT lists port %d twice", kChassis.left[i]); return false; }
        }
    }
    for (std::size_t i = 0; i < kChassis.rightCount; ++i) {
        if (!inRange(kChassis.right[i])) { std::snprintf(why, cap, "RIGHT port %d is not 1..21", kChassis.right[i]); return false; }
        if (kChassis.imuPort != 0 && kChassis.right[i] == static_cast<std::int8_t>(kChassis.imuPort)) {
            std::snprintf(why, cap, "port %d is both RIGHT and the IMU port", kChassis.right[i]);
            return false;
        }
        for (std::size_t j = i + 1; j < kChassis.rightCount; ++j) {
            if (kChassis.right[i] == kChassis.right[j]) { std::snprintf(why, cap, "RIGHT lists port %d twice", kChassis.right[i]); return false; }
        }
    }
    return true;
}

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
constexpr int kMaxMenu = 10;
int g_verdict[kMaxMenu] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

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
    if (tableHasPorts()) {
        const unsigned expect = static_cast<unsigned>(tableCount());
        emitS(motors >= static_cast<int>(expect) ? Sev::Good : Sev::Bad,
              "motors found: %d   (chassis table expects %u: %u left + %u right)", motors,
              expect, static_cast<unsigned>(kChassis.leftCount),
              static_cast<unsigned>(kChassis.rightCount));
        emit("EMPTY PORTS ARE OMITTED. A table port missing here is a FINDING.");
    } else {
        emitS(Sev::Warn, "motors found: %d   (chassis table has NO PORTS yet -- side '?')",
              motors);
        emit("Report these port numbers, per side, and which end is the FRONT; they go");
        emit("into src/bench_r3a.cpp's chassis table. Nothing is guessed in the meantime.");
    }
    emit("Index 21+ is NOT scanned: apix.h documents 0-20, and reading past it is");
    emit("what produced the phantom 'ADI expander' on 2026-08-18 (HA-120 predicted it).");
}

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
///
/// THE ONE EXCEPTION, deliberate and loud: station 10 DRIVE (R3b Session 2) DOES
/// construct the adapter, because driving through the adapter is the point of that
/// station -- and it prints the gearset it is about to WRITE as a belief first, and
/// refuses unless the chassis table names one. No other station constructs it.
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
    emit("Nothing here writes to a motor: this STATION never constructs the adapter");
    emit("that would (header note above reportMotorGroup). Only station 10 DRIVE does.");
    emit("");
    if (tableHasPorts()) {
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
        if (tableSideOf(ports[i]) != 0) {
            if (v != 0) ++tableMoved; else noteSilent(ports[i]);
        } else if (v != 0) {
            ++nonTableMoved;
        }
    }
    // A table port that is not a census motor cannot have moved: silent by definition.
    for (std::size_t i = 0; i < kChassis.leftCount; ++i) {
        if (found[kChassis.left[i]] != pros::c::E_DEVICE_MOTOR) noteSilent(kChassis.left[i]);
    }
    for (std::size_t i = 0; i < kChassis.rightCount; ++i) {
        if (found[kChassis.right[i]] != pros::c::E_DEVICE_MOTOR) noteSilent(kChassis.right[i]);
    }
    const bool qualifies = tableHasPorts() && tableSilent == 0
                           && tableMoved == static_cast<int>(tableCount());

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
            if (tableSideOf(p) != 0) {
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
        std::snprintf(g_watch.lastSummary, sizeof g_watch.lastSummary,
                      "whole-robot push (%s-first): signs captured for all %u table ports",
                      backFirst ? "back" : "front", static_cast<unsigned>(tableCount()));
    } else if (!tableHasPorts()) {
        emitS(Sev::Warn, "SEVERAL moved, but the chassis table has NO PORTS -- signs cannot be");
        emit("captured (there is nothing to attach them to). The UP/DOWN column above IS");
        emit("the sign convention for the front you pushed toward: report it WITH the port");
        emit("numbers per side, and it goes into src/bench_r3a.cpp's table.");
        std::snprintf(g_watch.lastSummary, sizeof g_watch.lastSummary,
                      "several moved; table has no ports -- no signs");
    } else {
        emitS(Sev::Warn, "PARTIAL: %d of %u table ports moved; silent:%s", tableMoved,
              static_cast<unsigned>(tableCount()), silent);
        emit("No sign capture: DRIVE needs EVERY table port to move in ONE push. Push again,");
        emit("all wheels on the floor, firmer. A port that never moves is a finding.");
        std::snprintf(g_watch.lastSummary, sizeof g_watch.lastSummary,
                      "partial push: %d/%u table ports moved (silent:%s) -- no signs", tableMoved,
                      static_cast<unsigned>(tableCount()), silent);
    }
    emit("This table is in /usd/r3a_log.txt. Re-run per wheel to build the map.");
}

// ═══ STATION 10 — DRIVE: THE ONE STATION THAT POWERS MOTORS (R3b Session 2, §3.3) ════
// Hands-on, open-loop, through the library's ADAPTERS -- hal::pros::ProsMotor (signed
// ports from MOTOR WATCH's capture; the table's cartridge, which the ctor WRITES) and
// hal::pros::ProsController -- never through the motion stack. The first time either
// adapter runs on hardware; the run measures HA-94 onward. Everything else in this
// binary is read-only, and this is the ONLY code path that calls setVoltage.
//
// SIX SAFETY GATES, in this order -- a station that skips one is the defect:
//   1. REFUSE unless the chassis table has ports and a cartridge and is consistent,
//      every table port is a census motor, and MOTOR WATCH captured a sign for every
//      table port from a WHOLE-ROBOT push this power cycle. Names what is missing.
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
constexpr double kFightCommandFloorV = 1.0;  // gate 5 evaluates only above this |command|
constexpr double kMovingFloorRadS = 1.0;     // ... and only when the side's fastest is above this
constexpr double kNearZeroFraction = 0.25;   // under this fraction of the side's fastest = frozen
constexpr double kCurrentLimitA = 2.4;
constexpr double kCleanDrivenSeconds = 3.0;  // a wheels-up run must drive this long...
constexpr double kCleanCeilingV = 6.0;       // ...and reach this ceiling to unlock GROUND mode
constexpr int kTickMs = 10;
constexpr int kPersistTicks = 25;   // 250 ms, for the fight AND the over-current cut
constexpr int kPanelEveryTicks = 10;
constexpr int kLcdEveryTicks = 20;  // the firmware rate-limits LCD writes; 200 ms, on change
constexpr hal::ControllerButton kDeadMan = hal::ControllerButton::L1;
constexpr hal::ControllerButton kCeilingUp = hal::ControllerButton::R1;

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
    if (describeMissing(missing, sizeof missing, true)) {
        emitS(Sev::Bad, "REFUSED: chassis table UNSET -- missing: %s.", missing);
        emit("  Nothing is guessed. Fill kChassis in src/bench_r3a.cpp from the build team's");
        emit("  report (ports per side + the front they chose), rebuild, upload, re-run.");
        return;
    }
    char why[96];
    if (!tableConsistent(why, sizeof why)) {
        emitS(Sev::Bad, "REFUSED: the chassis table contradicts itself -- %s.", why);
        return;
    }
    for (int p = 1; p <= kMaxPort; ++p) {
        if (tableSideOf(p) != 0 && found[p] != pros::c::E_DEVICE_MOTOR) {
            emitS(Sev::Bad, "REFUSED: table port %d is not a motor in the census (%s).", p,
                  deviceName(found[p]));
            emit("  Re-run 1 DEVICE CENSUS; a table port absent from it is a finding.");
            return;
        }
    }
    if (!g_watch.signsValid) {
        emitS(Sev::Bad, "REFUSED: no sign capture from a WHOLE-ROBOT push this power cycle.");
        emitf("  MOTOR WATCH says: %s", g_watch.lastSummary);
        emit("  Run 3 MOTOR WATCH: push the whole robot front-first so EVERY table port");
        emit("  moves, answer FRONT FIRST, then come back here.");
        return;
    }
    for (int p = 1; p <= kMaxPort; ++p) {
        if (tableSideOf(p) != 0 && g_watch.sign[p] == 0) {
            emitS(Sev::Bad, "REFUSED: no sign captured for table port %d.", p);
            return;
        }
    }

    // ── the belief, loud, BEFORE anything is written to a motor ──────────────────
    emitS(Sev::Warn, "CARTRIDGE BELIEF: %s", cartridgeWord(kChassis.cartridge));
    emitf("  (%s)", kChassis.provenance);
    emit("  ProsMotor's constructor WRITES this gearset to every table motor and reads it");
    emit("  back; a WRONG belief passes that read-back. Volts are unaffected; the rad/s");
    emit("  shown scale by it. If the insert colour is not this: STOP, fix the table.");
    char l[80], r[80];
    signedPortsString(kChassis.left, kChassis.leftCount, l, sizeof l);
    signedPortsString(kChassis.right, kChassis.rightCount, r, sizeof r);
    emitf("signed ports: LEFT %s | RIGHT %s", l, r);
    emitf("  (- = reversed by PROS, from MOTOR WATCH%s)",
          g_watch.signsFromBackwardPush ? "; back-first push, readings inverted" : "");
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
        const int side = tableSideOf(p);
        if (side == 0) continue;
        Member& m = members[n];
        m.port = p;
        m.side = side;
        m.signedPort = static_cast<std::int8_t>(g_watch.sign[p] > 0 ? p : -p);
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
            vL = std::clamp(ceiling * (req.forward - req.yawCcw), -ceiling, ceiling);
            vR = std::clamp(ceiling * (req.forward + req.yawCcw), -ceiling, ceiling);
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

        // GATE 5: the fighting-motor cut-out, then the over-current cut.
        if (!cut) {
            for (int side = -1; side <= 1; side += 2) {
                const double cmd = side < 0 ? vL : vR;
                double vmax = 0.0;
                int count = 0;
                for (std::size_t i = 0; i < n; ++i) {
                    if (members[i].side == side) {
                        ++count;
                        vmax = std::max(vmax, std::abs(members[i].v));
                    }
                }
                const bool evaluate = std::abs(cmd) > kFightCommandFloorV && vmax > kMovingFloorRadS;
                const double expected = cmd > 0.0 ? 1.0 : -1.0;
                int persisted = 0;
                char names[48] = "";
                for (std::size_t i = 0; i < n; ++i) {
                    Member& m = members[i];
                    if (m.side != side) continue;
                    if (!evaluate) {
                        m.disagreeTicks = 0;
                        continue;
                    }
                    const bool oppositeSign =
                        (m.v * expected) < 0.0 && std::abs(m.v) > 0.5 * kMovingFloorRadS;
                    const bool nearZero = std::abs(m.v) < kNearZeroFraction * vmax;
                    m.disagreeTicks = (oppositeSign || nearZero) ? m.disagreeTicks + 1 : 0;
                    if (m.disagreeTicks >= kPersistTicks) {
                        ++persisted;
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
    bool powersMotors = false;  ///< the ONE station that powers motors: RED stripe, not amber
};

pros::c::v5_device_e_t g_found[kMaxPort + 1] = {};

void tCensus(pros::c::v5_device_e_t* f)   { census(f); }
void tImu(pros::c::v5_device_e_t* f) {
    if (kChassis.imuPort == 0) {
        rule("STAGE 2  IMU");
        emitS(Sev::Bad, "IMU port UNSET in the chassis table (%s).", kChassis.robot);
        emit("Mount an IMU on any smart port, report the port number, and it goes into");
        emit("src/bench_r3a.cpp's table. Nothing is guessed in the meantime.");
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

    {"10 DRIVE (POWERS)", &driveStation, true,
     "WHEELS UP first (ground only after a clean 6 V wheels-up run). POWERS motors: hold L1",
     "you drove at 3 V wheels-up, raised the ceiling with R1, and touched to stop",
     true},
};
constexpr int kMenuCount = static_cast<int>(sizeof kMenu / sizeof kMenu[0]);
static_assert(kMenuCount == kMaxMenu, "g_verdict[] must have one slot per menu entry");

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
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 20, "READ-ONLY except 10 DRIVE (powers motors)");
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
        pros::c::screen_set_pen(kMenu[i].powersMotors ? kColBad : kColEdge);
        pros::c::screen_draw_rect(x0, y0, x1, y1);
        // An amber stripe means THIS ONE NEEDS YOUR HANDS. A RED stripe and edge mean
        // THIS ONE POWERS MOTORS (station 10 DRIVE, the only one). Encoded as form
        // rather than words: the labels are already at the width the button allows.
        if (kMenu[i].handsOn) {
            pros::c::screen_set_pen(kMenu[i].powersMotors ? kColBad : kColWarn);
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

/// -1 when the touch landed outside the button grid. INSIDE the grid, the 4 px gaps between
/// buttons belong to the button above / to the left of them (integer division of the
/// pitch), so no point inside the grid is dead: on robot two's brain the very first recorded tap
/// (2026-09-10, `touch: x=364 y=117`) released one pixel into the gap between rows 2 and 3
/// and did nothing, on a resistive screen where a fingernail's release point wanders a few
/// pixels. A gap that swallows taps is a trap, not a feature; the header strip above the
/// grid and the margin below it still hit nothing, so a stale coordinate at (0,0) cannot
/// launch a station (the earlier touch-fix's invariant, kept).
int hitTest(std::int16_t tx, std::int16_t ty) {
    std::int16_t x0, y0, x1, y1;
    buttonBox(0, x0, y0, x1, y1);
    const std::int16_t gridX0 = x0, gridY0 = y0;
    buttonBox(kMenuCount - 1, x0, y0, x1, y1);
    const std::int16_t gridX1 = x1, gridY1 = y1;
    if (tx < gridX0 || tx > gridX1 || ty < gridY0 || ty > gridY1) return -1;
    const int col = std::min(1, static_cast<int>((tx - gridX0) / (kBtnW + kGap)));
    const int row = std::min(kMenuCount / 2 - 1 + kMenuCount % 2,
                             static_cast<int>((ty - gridY0) / (kBtnH + kGap)));
    const int i = row * 2 + col;
    return i < kMenuCount ? i : -1;
}

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
                             "read-only EXCEPT station 10 DRIVE, which powers motors.");
    pros::delay(2000);

    emit("");
    emit("################################################################");
    emit("#  shulib R3a/R3b  BENCH TESTER  --  read-only EXCEPT 10 DRIVE (POWERS MOTORS)");
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
        portsToString(kChassis.left, kChassis.leftCount, l, sizeof l);
        portsToString(kChassis.right, kChassis.rightCount, r, sizeof r);
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
        if (describeMissing(missing, sizeof missing, false)) {
            emitS(Sev::Warn, "UNSET in the table: %s -- never guessed; the stations that need",
                  missing);
            emit("           them refuse until the build team reports and the table is filled.");
        }
        char why[96];
        if (!tableConsistent(why, sizeof why)) {
            emitS(Sev::Bad, "TABLE CONTRADICTS ITSELF: %s -- fix src/bench_r3a.cpp first.", why);
        }
    }
    emit("!! SIDE LABELS ARE THE TABLE'S CLAIM, NOT A MEASUREMENT (R3a-PROGRESS S10.3);");
    emit("!! MOTOR WATCH measures polarity. Station 10 DRIVE is the ONLY one that powers motors.");
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
        pros::screen_touch_status_s_t last = pros::c::screen_touch_status();
        std::int32_t seen = last.release_count;
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
