#pragma once
//
// R3a's bench validation entry point (chunk R3a, §4.2 item 1), extended at R3b
// Session 2 with the per-variant chassis table and the DRIVE station, at R2a
// (2026-09-24) with stations 11 DISTANCE and 12 AI VISION and the three-column menu, and at
// MECH-A6 (2026-10-30, the mechanism core's last part) with station 13 MECHANISMS and the
// menu's fifth row.
//
// Declared in its own header so src/main.cpp can dispatch to it under the
// compile-time robot selector WITHOUT the invented X-drive ever being built —
// its ports are invented (HA-111), and on the bench robot the device provider
// would refuse its first port at boot (since G1 Part 2c the X-drive lives in
// src/programs/xdrive_program.cpp, which only the xdrive build calls). Both tester variants
// (ROBOT=bench, the measured bench bot; ROBOT=tank, the 2026 chassis whose
// table carries MEASURED SIGNS since R3b Part 0b) boot into this as "Bench
// Tests" -- unless built with PROGRAM=drive, which boots src/programs/drive_program.cpp
// ("shulib Drive") instead. The chassis table is src/robots/chassis_table.hpp (the old "shulib
// Drive" shared it until T4 Part 3; the new one boots from the calibration record).
//
// This lives in src/, not include/shulib/, deliberately: it makes raw
// <pros/*> calls to print RAW value beside CANONICAL value, which is the
// whole point of the chunk, and include/shulib/ is PROS-free by CI guard.
//
// THIS HEADER ITSELF IS PROS-FREE (since R2a): besides runR3a() it carries only the
// menu's pure GEOMETRY (menu_grid below) and — since MECH-A6 — station 13's pure parts
// (mech_station below: its six gates, the direction check, its readings and RECORD lines, over
// robot::MechanismParts), so test/bench_menu_grid_test.cpp and test/bench_mech_station_test.cpp can
// include it from src/ (`programs/bench_r3a.hpp`) and try to break them on the host -- the
// chassis_table.hpp precedent. Nothing here draws or reads PROS; bench_r3a.cpp draws from it and
// reads the controller into it.

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>

#include "common/driver_panel.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/manipulation/stall_detector.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/direction_check.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/robot/mechanism_parts.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// The whole bench session: device census, per-device raw-vs-canonical report,
/// loop-rate measurement, a live motor watch that captures signs — and, since R3b
/// Session 2, station 10 DRIVE, which POWERS the drive motors through the
/// hal/pros adapters behind six safety gates. Since R2a, station 11 DISTANCE shows the
/// distance sensors' raw PROS reads beside the library adapter's, and station 12 AI VISION
/// shows the camera's detections -- it WRITES the camera's own detection settings (tag +
/// object detection on, the USB bounding-box overlay on, a tag family when asked) and says
/// so on its screen, but powers nothing. Since MECH-A6, station 13 MECHANISMS shows every
/// declared mechanism in its declared units and POWERS one at a time behind its own six gates,
/// with the direction check. Every station except 10 and 13 commands no motion. Never returns.
void runR3a();

/// initialize()'s tester branch (the bench and tank builds; G1 Part 2c moved it here from
/// src/main.cpp, verbatim): the `booted: bench-tester build [<beacon>]` banner and the
/// `competition status at boot:` line, from the two strings src/main.cpp passes — the beacon is a
/// literal in src/build_selection.hpp's one chain, so only the build that selected this program
/// links it.
/// Constructs nothing and powers nothing; the program itself runs from opcontrol().
void testerInitialize(const char* variantBeacon, const char* competitionStatusBits);

/// THE MENU GRID — pure geometry (R2a, 2026-09-24; the 2 × 5 menu became 3 × 4 so twelve
/// stations fit without shrinking a target).
///
/// Every rectangle has INCLUSIVE corners, PROS's drawing convention: screen_fill_rect(x0, y0,
/// x1, y1) paints x1 - x0 + 1 columns. A Spec's `w`/`h` are therefore x1 - x0 and y1 - y0 of one
/// button (the old kBtnW/kBtnH), and `gap` is the PITCH minus `w` — gap - 1 pixels lie between
/// two buttons.
///
/// THE HIT-TEST RULES, kept from the 2-column menu and now tested:
///  * NO DEAD GAP inside the grid: a gap pixel belongs to the button above / to the left of it
///    (integer division of the pitch). On robot two's brain the first recorded tap (2026-09-10,
///    `touch: x=364 y=117`) released one pixel into a gap and did nothing, on a resistive screen
///    whose release point wanders a few pixels.
///  * DEAD ZONES OUTSIDE it: the header strip above, the margins beside and the strip below hit
///    nothing — so a stale or error coordinate ((0,0), or −1 from an E_TOUCH_ERROR read) can
///    never launch a station.
///  * THE RIGHT EDGE COMES FROM THE RIGHTMOST COLUMN, not from the last button: on a partial last
///    row the last button is not in the rightmost column, and taking the extent from it (the
///    2-column code's idiom) would make the whole right column dead.
namespace menu_grid {

/// A rectangle, inclusive corners.
struct Rect {
    std::int16_t x0;
    std::int16_t y0;
    std::int16_t x1;
    std::int16_t y1;
};

/// `count` buttons, `cols` per row, ROW-MAJOR (button i is at row i / cols, column i % cols),
/// the first one's top-left at (x0, y0).
struct Spec {
    int count;
    int cols;
    std::int16_t x0;
    std::int16_t y0;
    std::int16_t w;    ///< x1 - x0 of one button
    std::int16_t h;    ///< y1 - y0 of one button
    std::int16_t gap;  ///< pitch - w (and pitch - h)
};

/// Rows needed for `count` buttons.
constexpr int rowsOf(const Spec& g) {
    return g.cols > 0 ? (g.count + g.cols - 1) / g.cols : 0;
}

/// Columns actually occupied (fewer than `cols` when there is only one, partial, row).
constexpr int colsUsed(const Spec& g) { return g.count < g.cols ? g.count : g.cols; }

/// Button i's rectangle.
constexpr Rect buttonBox(const Spec& g, int i) {
    const int col = i % g.cols;
    const int row = i / g.cols;
    const int x0 = g.x0 + col * (g.w + g.gap);
    const int y0 = g.y0 + row * (g.h + g.gap);
    return Rect{static_cast<std::int16_t>(x0), static_cast<std::int16_t>(y0),
                static_cast<std::int16_t>(x0 + g.w), static_cast<std::int16_t>(y0 + g.h)};
}

/// The grid's extent: from the first button's top-left to the far corner of the RIGHTMOST
/// column and the LAST row (see the header note on why not the last button's corner).
constexpr Rect extent(const Spec& g) {
    const int x1 = g.x0 + (colsUsed(g) - 1) * (g.w + g.gap) + g.w;
    const int y1 = g.y0 + (rowsOf(g) - 1) * (g.h + g.gap) + g.h;
    return Rect{g.x0, g.y0, static_cast<std::int16_t>(x1), static_cast<std::int16_t>(y1)};
}

/// Which button (tx, ty) hits, or -1: outside the extent, or in an EMPTY slot of a partial
/// last row. Inside the extent, gap pixels belong to the button above / to the left.
constexpr int hitTest(const Spec& g, int tx, int ty) {
    if (g.count <= 0 || g.cols <= 0) return -1;
    const Rect e = extent(g);
    if (tx < e.x0 || tx > e.x1 || ty < e.y0 || ty > e.y1) return -1;
    int col = (tx - e.x0) / (g.w + g.gap);
    int row = (ty - e.y0) / (g.h + g.gap);
    // Defensive only: inside the extent the division already lands in range (the last
    // column's and row's gap lies OUTSIDE the extent). Kept so a future edit to extent()
    // cannot index past the grid.
    if (col > colsUsed(g) - 1) col = colsUsed(g) - 1;
    if (row > rowsOf(g) - 1) row = rowsOf(g) - 1;
    const int i = row * g.cols + col;
    return i < g.count ? i : -1;
}

/// The usable panel: 480 wide, and nothing drawn at y >= 236 (VEXos keeps the rows below;
/// liblvgl's lv_conf.h says 240, and the tester keeps a 4 px margin — a measured lesson,
/// bench_r3a.cpp's kUsableH note).
inline constexpr int kPanelW = 480;
inline constexpr int kLastRow = 235;
/// The menu's header bar is painted over y 0..32; the grid must start below it.
inline constexpr int kMenuHeaderBottom = 32;

/// THE MENU: thirteen stations, three across, FIVE rows (MECH-A6, 2026-10-30 — the MECH brief's N5 (a),
/// the team lead's go of 2026-10-08: "room for 15, so the rest's pages fit later") — 1 2 3 / 4 5 6 /
/// 7 8 9 / 10 11 12 / 13. Columns 6-159 / 163-316 / 320-473 (unchanged); rows 38-74 / 78-114 / 118-154 /
/// 158-194 / 198-234 (38 + 5 x 40 - 4 = 234 <= 235). Each button is 36 px tall (y1 - y0) — the 2 x 5
/// menu's height, which R2a's 3 x 4 menu (46 px) had grown past; stations 1-12 keep their numbers,
/// their logged names and their places in the grid (row, column), and the two slots after 13 hit
/// nothing until a later station fills them.
inline constexpr Spec kMenu{13, 3, 6, 38, 153, 36, 4};
/// The menu's CAPACITY: the five rows hold fifteen buttons (kMenu with every slot filled).
inline constexpr int kMenuSlots = 15;

// ── what is drawn INSIDE a menu button (bench_r3a.cpp's drawMenu), as offsets ──────
/// The hands-on stripe: x0 .. x0 + kStripeW - 1, full height.
inline constexpr int kStripeW = 6;
/// Both label lines start here (2 px clear of the stripe).
inline constexpr int kLabelInsetX = 8;
/// Line 1's top, from y0: the station's number and name, MEDIUM (its text cell is laid out
/// calibration::kMediumHeight = 20 px tall — an ESTIMATE, HA-136).
inline constexpr int kLine1DY = 2;
/// Line 2's top, from y0: the second word, SMALL since MECH-A6 — two MEDIUM lines (the 22 px
/// pitch R2a proved) need 44 px and a 36 px button has 35 inside its edges. It starts where line 1's
/// MEDIUM cell ends.
inline constexpr int kLine2DY = 22;
/// The SMALL font's row pitch the bench tester lays every SMALL line out at (bench_r3a.cpp's
/// kLineH, static_asserted equal): line 2 ends kSmallPitch below its top, inside the bottom edge.
inline constexpr int kSmallPitch = 13;
/// The status dot: centre (x1 - kDotDX, y0 + kDotDY), radius kDotR — beside line 2.
inline constexpr int kDotDX = 14;
inline constexpr int kDotDY = 28;
inline constexpr int kDotR = 5;
/// Line 2 stops this many pixels short of the dot's left edge.
inline constexpr int kDotClear = 3;

/// Pixels available to label line 1: from x0 + kLabelInsetX to x1 - 1 (the edge is drawn at x1).
constexpr int line1Px(const Spec& g) { return g.w - kLabelInsetX; }
/// Pixels available to label line 2: from x0 + kLabelInsetX to kDotClear short of the dot.
constexpr int line2Px(const Spec& g) {
    return (g.w - kDotDX - kDotR - kDotClear) - kLabelInsetX + 1;
}

/// THE STATIONS' BUTTON ROW (11 DISTANCE, 12 AI VISION, 13 MECHANISMS): three slots across the
/// bottom, the menu's columns — slot 0 (left) FAMILY on station 12, NEXT on 13 and empty on 11, slot 1
/// (middle) RECORD, slot 2 (right) DONE, so RECORD and DONE sit in the same place on every station.
/// Station 13's direction check puts its THREE answers in these slots while it asks.
inline constexpr Spec kStationRow{3, 3, 6, 186, 153, 48, 4};
/// Where a touch HITS the row: the same columns, extended 8 px ABOVE the drawn buttons (a
/// resistive release lands high — robot two's calibrate capture has one exactly on a button's
/// top edge). Nothing is drawn in those 8 px but the end of the status line.
inline constexpr Spec kStationRowHit{3, 3, 6, 178, 153, 56, 4};
/// Station 13's FOUR-answer row (MECH-A6): a three-position pneumatic's question — "which position
/// did it go to?" — needs its three names and "nothing / something else moved". Four slots across the
/// station row's band (x 6-117 / 121-232 / 236-347 / 351-462), its labels SMALL (two lines of 10).
inline constexpr Spec kAnswerRow4{4, 4, 6, 186, 111, 48, 4};
/// Where a touch HITS the four-answer row: its columns, extended 8 px above, like kStationRowHit.
inline constexpr Spec kAnswerRow4Hit{4, 4, 6, 178, 111, 56, 4};

}  // namespace menu_grid

// ═══ STATION 13 MECHANISMS — THE PURE PARTS (chunk MECH-A6, 2026-10-30) ════════════════════════════════
// The mechanism core's sixth and last part (the MECH brief's §1.14): Bench Tests' station 13 shows every
// mechanism the robot DECLARES — its readings live, in its declared units from power-up — and powers ONE
// at a time behind six gates, and runs the DIRECTION CHECK: "ONLY '<name>' should move", one short "+"
// jog, the encoders' findings, the person's answer, mechanism::judgeDirection / judgePneumatic's verdict
// naming the robot-file line to change. Everything below is PROS-free so test/bench_mech_station_test.cpp
// drives it on the host over robot::MechanismParts built on a fake provider; bench_r3a.cpp only reads the
// controller and the raw temperature, paints, and prompts.
//
// WHAT IT BUILDS: robot::MechanismParts over the build's list (kMechanisms, projected by
// mechanism::toMechanismConfig — projectConfigs() below) — no Robot, no drivetrain, no mechanism service:
// a mechanism's direction must be checkable before it is homed, so the station commands each built
// hal::MotorMechanism / hal::PneumaticMechanism DIRECTLY, as station 10 commands its motors. The ONLY code
// paths in Bench Tests that power anything are station 10's and this one.
//
// THE SIX GATES (station 10's, adapted — the MECH brief's §1.14; every number INVENTED, HA-182's page
// entries, and the screen says so):
//   1. REFUSE a mechanism that was not built (ABSENT: its page says the provider's sentence, and nothing
//      on it is powered).
//   2. TWO-STAGE CLEAR CONFIRM: stage 1 at every entry, before anything is made or powered ("is every
//      mechanism's path clear, hands away?" — making a 3-wire line DRIVES it to its start level, HA-119,
//      so the build itself waits for it); stage 2 before a MOTOR mechanism's first volt in this entry
//      ("POWER '<name>'?") and before EVERY pneumatic fire ("FIRE 'claw' -> 'open'? hands clear" — a
//      cylinder cannot be stopped mid-stroke, so each fire names its position and is confirmed).
//   3. DEAD-MAN: L1 held. Its release — or a controller drop — puts every mechanism this station
//      POWERED in its declared safe state THAT TICK (a pneumatic back to its safe position). A
//      mechanism the station never powered is never commanded, so nothing moves without a touch.
//   4. A VOLTS CEILING starting at 3 V; R1's press steps it 3 -> 6 -> 9 -> 12 and stops at 12.
//   5. CUT-OUTS, each cutting THAT mechanism to its safe state, named, until the station is re-entered:
//      two of its motors disagreeing for 250 ms (the coupled-drivetrain evaluator station 10 uses:
//      teleop::CoupledSideMonitor — a FIGHT, some motors against the others; every motor alike against
//      the command is a BACK-DRIVE, the direction judgement's finding, never a fight); any of its motors
//      above 2.4 A for 250 ms; its OWN stall signature when it states one; a hold-to-run jog held for its
//      stated maximum time (3 s).
//   6. EXIT: every powered mechanism safe — the station's own exit and bench_r3a.cpp's guard on every
//      other way out (a thrown precondition included).
//
// THE DIRECTION CHECK (Q30, Q1 (a), the re-check's D6): a MOTOR mechanism's check is one "+" jog at the
// ceiling for 0.4 s, recording each motor's encoder change and peak current and the angle sensor's change
// (its declared mounting applied) — mechanism::judgeDirection reports a fight, a motor that neither moved
// nor drew, every motor still while drawing, a back-drive or a sensor against its motors AT ONCE; otherwise
// the page asks "did '<name>' move its + way / the OTHER way / it did NOT move or something ELSE moved" and
// the answer is judged. A cut during the jog ends it there and the judgement runs over what the jog saw (a
// fight's cut at 250 ms names the fighting motor). A PNEUMATIC's check is a fire (stage 2's confirm) and
// "which position did it go to?" — its position names and "nothing / something else moved" —
// mechanism::judgePneumatic. NOTHING IS SAVED (Q1 (a)): the verdict names the line to edit; "edit the robot
// file, rebuild, upload".
//
// WHAT IT CANNOT SEE (landmine 17): a V5 motor reverses its command and its encoder together, so a motor
// declared the wrong way is invisible to its own encoder — the person's answer sees it. Nothing here has run
// on a robot.
namespace mech_station {

// ── THE PAGE'S STATED NUMBERS — HA-182's page entries: INVENTED for a first run (the screen says so) ──
/// The station's loop period, ms.
inline constexpr int kTickMs = 10;
/// Gate 4: the volts ceiling starts here ...
inline constexpr double kCeilingStartV = 3.0;
/// ... steps by this on each R1 press ...
inline constexpr double kCeilingStepV = 3.0;
/// ... and stops here.
inline constexpr double kCeilingMaxV = 12.0;
/// Gate 5: any one motor of a mechanism above this current ...
inline constexpr double kCurrentLimitA = 2.4;
/// ... for this many consecutive ticks (250 ms) is cut — the fight's window too (station 10's).
inline constexpr int kPersistTicks = 25;
/// Gate 5: a hold-to-run jog held this long, ms, is cut (a jog's stated maximum time). The station keeps
/// its clock in WHOLE MILLISECONDS (bench_r3a.cpp passes pros::millis()), so every time edge is exact.
inline constexpr std::uint32_t kJogMaxMs = 3000;
/// The direction check's "+" jog, ms, at the ceiling.
inline constexpr std::uint32_t kCheckJogMs = 400;
/// The direction check's ANNOUNCEMENT, ms: after A the page says "ONLY '<name>' should move -- watch it" and
/// drives NOTHING for this long, then the "+" jog (the brief's order: said first, then the jog). Its last
/// kCheckJogMs is the SETTLE window: a motor that turned kMinMoveRad or more in it is still moving (a coast,
/// a sag), and the check is abandoned — a baseline taken while it moves would judge the coast, not the jog.
inline constexpr std::uint32_t kAnnounceMs = 1000;
/// The direction check's "did not move": an encoder change below this, rad (about 2 deg at the motor).
inline constexpr double kMinMoveRad = 0.035;
/// The direction check's "drawing": a still motor whose peak current reached this, A, is pushing on
/// something.
inline constexpr double kDrawAmps = 0.4;
/// The live readings are painted at most this often, ms (5 Hz — PROS leaks every print; the panel
/// paints a line only when it changed).
inline constexpr std::uint32_t kPaintMs = 200;

// The fight's numbers are the shared evaluator's defaults — station 10's, pinned so they cannot drift.
static_assert(teleop::SideMonitorConfig{}.persistTicks == kPersistTicks);
static_assert(kCheckJogMs < kJogMaxMs, "the check's jog must end before a jog's maximum cuts it");
static_assert(kAnnounceMs >= kCheckJogMs, "the settle window is the announcement's last kCheckJogMs");

/// The configs station 13 builds from: `list`'s mechanisms projected (mechanism::toMechanismConfig), in
/// order, into `out` — the storage robot::MechanismParts BORROWS, so `out` must outlive the parts (the
/// station keeps both in static storage — MECH-A1's D11). Returns how many were written.
inline std::size_t projectConfigs(const mechanism::MechanismTable& list, std::span<config::MechanismConfig> out) {
    std::size_t n = 0;
    for (std::size_t i = 0; i < list.count && i < mechanism::kMaxMechanisms && n < out.size(); ++i) {
        out[n++] = mechanism::toMechanismConfig(list.specs[i], list.wiring[i]);
    }
    return n;
}

/// A mechanism's reading IN ITS DECLARED UNITS, through its motors and gearing, from POWER-UP (the
/// MECH brief's U1: a wrong gearing is a wrong angle on the screen, so the team reads `.range`, the presets
/// and the level angle off the robot by moving the arm by hand).
struct UnitsReading {
    /// A position or speed mechanism (a power or pneumatic one declares no units).
    bool has = false;
    /// The output shaft's travel since power-up, degrees: the motors' mean angle through the gearing.
    double outputDeg = 0.0;
    /// POSITION: where it is in its units, taking the power-up pose as its declared HOME (what
    /// `restsAtStop` believes at boot); SPEED: the output's speed, rpm.
    double value = 0.0;
    /// "deg", "in" or "rpm"; "" when `has` is false.
    const char* unit = "";
};

/// The reading of `s` from its motors' MEAN angle (rad, cumulative since power-up — hal::MotorMechanism::
/// meanPosition) and mean speed (rad/s): output degrees = the motor angle / the gearing's motor turns per
/// output turn; a position mechanism's units through its shape (mechanism::angleDegToPosition), from its
/// home position at power-up — so a 12:60 arm turned one output turn reads 360 deg, and a 1.5 in spool
/// one spool turn reads pi x 1.5 in.
[[nodiscard]] inline UnitsReading unitsReading(const mechanism::MechanismSpec& s, double meanMotorRad,
                                               double meanMotorRadS) {
    constexpr double kPi = 3.14159265358979323846;
    UnitsReading r;
    if (s.kind == mechanism::Kind::Position && s.position.gearRatio > 0.0) {
        const mechanism::PositionSpec& p = s.position;
        r.has = true;
        r.outputDeg = meanMotorRad * 180.0 / kPi / p.gearRatio;
        r.value = mechanism::angleDegToPosition(p, mechanism::positionToAngleDeg(p, p.homePosition) + r.outputDeg);
        r.unit = p.units == mechanism::Units::Inches ? "in" : "deg";
    } else if (s.kind == mechanism::Kind::Speed && s.speed.gearRatio > 0.0) {
        r.has = true;
        r.outputDeg = meanMotorRad * 180.0 / kPi / s.speed.gearRatio;
        r.value = meanMotorRadS / s.speed.gearRatio * 60.0 / (2.0 * kPi);
        r.unit = "rpm";
    }
    return r;
}

/// The reading's line on the panel (at most 50 characters for a 23-character-free word set; the RECORD
/// line carries every number in full): `ARM +360.0 deg (power-up = home 0.0; 5.00:1)`,
/// `LIFT +4.71 in (power-up = home 0.00; 1.00:1)`, `FLYWHEEL +3000 rpm out (0.17:1)`; "" when `r.has`
/// is false.
inline void unitsText(char* out, std::size_t cap, const mechanism::MechanismSpec& s, const UnitsReading& r) {
    if (out == nullptr || cap == 0) {
        return;
    }
    out[0] = '\0';
    if (!r.has) {
        return;
    }
    if (s.kind == mechanism::Kind::Speed) {
        std::snprintf(out, cap, "%s %+.0f rpm out (%.2f:1)", mechanismKindWord(s), r.value, s.speed.gearRatio);
        return;
    }
    const bool inches = s.position.units == mechanism::Units::Inches;
    std::snprintf(out, cap, inches ? "%s %+.2f %s (power-up = home %.2f; %.2f:1)"
                                   : "%s %+.1f %s (power-up = home %.1f; %.2f:1)",
                  mechanismKindWord(s), r.value, r.unit, s.position.homePosition, s.position.gearRatio);
}

/// A motor's temperature as the page shows it: `31C`, or `unknown` when the read FAILED (a non-finite
/// value — PROS_ERR_F is infinity). Never a guess: the library adapter holds its last good value and seeds
/// 20 C before any answer (hal/pros/motor.hpp), so the cpp passes the RAW PROS read, and a motor that
/// never answered is "unknown", never "cool".
inline void temperatureText(char* out, std::size_t cap, double rawC) {
    if (out == nullptr || cap == 0) {
        return;
    }
    if (!std::isfinite(rawC)) {
        std::snprintf(out, cap, "unknown");
    } else {
        std::snprintf(out, cap, "%.0fC", rawC);
    }
}

/// One motor's line: `m1 p6 +123.4deg +1.23r/s 0.42A 31C` — its role, its port, its position since
/// power-up (MOTOR degrees, as its adapter reads it: the declared direction applied), velocity, current
/// and temperature (temperatureText).
inline void motorText(char* out, std::size_t cap, std::size_t role, std::uint8_t port, double posRad,
                      double velRadS, double amps, double rawTempC) {
    if (out == nullptr || cap == 0) {
        return;
    }
    constexpr double kPi = 3.14159265358979323846;
    char t[16];
    temperatureText(t, sizeof t, rawTempC);
    std::snprintf(out, cap, "m%u p%u %+.1fdeg %+.2fr/s %.2fA %s", static_cast<unsigned>(role + 1),
                  static_cast<unsigned>(port), std::clamp(posRad * 180.0 / kPi, -99999.9, 99999.9),
                  std::clamp(velRadS, -999.0, 999.0), std::clamp(amps, -99.0, 99.0), t);
}

/// A LIMIT SWITCH's line — the level it reads, what that means IF HA-184 holds (a VEX switch reads 1 when
/// pressed), or that its read FAILED: `home sw B: reads 1 = PRESSED (if HA-184)`. The page asks for the
/// switch pressed, released AND with its cable out (HA-184's settle): an unplugged line reads a LEVEL,
/// never a failed read, so only the three readings side by side say what it does.
inline void switchText(char* out, std::size_t cap, const char* what, char letter, bool level, bool pressedLevel,
                       bool faultMoved) {
    if (out == nullptr || cap == 0) {
        return;
    }
    if (faultMoved) {
        std::snprintf(out, cap, "%s %c: NOT ANSWERING (a read failed)", what, letter);
        return;
    }
    std::snprintf(out, cap, "%s %c: reads %d = %s (if HA-184)", what, letter, level ? 1 : 0,
                  level == pressedLevel ? "PRESSED" : "released");
}

/// What a piece sensor's sample says — the mechanism service's POSITIVE-EVIDENCE rule (mechanism/service.hpp,
/// Slot::seen()), mirrored here for the page and pinned by its test at the same edges: a distance AT or past
/// its threshold is no piece, a near one with confidence 0 is no evidence (a held value), a sample taken while
/// its read-fault count moved is not answering.
enum class Evidence : std::uint8_t {
    Sees,          ///< a fresh reading that sees a piece
    NoPiece,       ///< a fresh reading that does not
    NoEvidence,    ///< not a reading (a non-finite one, or near with confidence 0)
    NotAnswering,  ///< its read-fault count moved: a held value
};

/// The evidence word.
[[nodiscard]] constexpr const char* evidenceWord(Evidence e) noexcept {
    switch (e) {
        case Evidence::Sees: return "SEES a piece";
        case Evidence::NoPiece: return "no piece";
        case Evidence::NoEvidence: return "no evidence";
        case Evidence::NotAnswering: return "NOT ANSWERING";
    }
    return "?";
}

/// A distance piece sensor's evidence (the service's rule, above).
[[nodiscard]] inline Evidence distanceEvidence(double inches, double confidence, double seesWithinIn,
                                               bool faultMoved) noexcept {
    if (faultMoved) {
        return Evidence::NotAnswering;
    }
    if (!std::isfinite(inches) || !std::isfinite(confidence)) {
        return Evidence::NoEvidence;
    }
    if (inches >= seesWithinIn) {
        return Evidence::NoPiece;
    }
    return confidence > 0.0 ? Evidence::Sees : Evidence::NoEvidence;
}

/// An optical piece sensor's evidence: a proximity at or above its threshold sees a piece.
[[nodiscard]] inline Evidence opticalEvidence(double proximity, double seesProximity, bool faultMoved) noexcept {
    if (faultMoved) {
        return Evidence::NotAnswering;
    }
    if (!std::isfinite(proximity)) {
        return Evidence::NoEvidence;
    }
    return proximity >= seesProximity ? Evidence::Sees : Evidence::NoPiece;
}

/// A read-fault count watched between two of the page's samples: "moved" when it changed since the last
/// sample (the service's rule compares against its previous tick the same way — landmine 22). Seeded at
/// the station's entry, so a sensor that failed before the station began is not blamed on its first paint.
class FaultWatch {
public:
    /// Remember `count` as the last seen, without judging it.
    void seed(int count) noexcept { seen_ = count; }
    /// True when `count` differs from the last one seen; it becomes the last seen.
    [[nodiscard]] bool moved(int count) noexcept {
        const bool m = count != seen_;
        seen_ = count;
        return m;
    }

private:
    int seen_ = 0;
};

/// The controller as the station reads it, one tick (bench_r3a.cpp fills it from hal::pros::ProsController).
struct Pad {
    bool connected = false;  ///< the controller answers
    bool deadMan = false;    ///< L1 held — gate 3
    bool raise = false;      ///< R1 held — its press steps the ceiling (gate 4)
    bool plus = false;       ///< UP held — a motor mechanism's "+" jog; a pneumatic's fire, on the press
    bool minus = false;      ///< DOWN held — a motor mechanism's "-" jog
    bool check = false;      ///< A held — the direction check, on the press
};

/// Where the page is.
enum class Phase : std::uint8_t {
    Idle,        ///< the live readings; nothing powered unless L1 and a jog are held
    Confirm,     ///< a stage-2 confirm is pending (bench_r3a.cpp shows it — confirmText())
    Announcing,  ///< the direction check says what should move, driving nothing (kAnnounceMs — announceText())
    Jogging,     ///< the direction check's "+" jog is running
    Asking,      ///< the direction check's question is pending (answerCount() answers)
};

/// The text of a two-button prompt (bench_r3a.cpp's twoButtonPrompt: title, subtitle, the YES side's two
/// lines, the NO side's two, two foot lines). Each line is worded to fit where the prompt prints it — the
/// title 36 MEDIUM characters, the subtitle and the foot lines 50 SMALL ones, a side's lines 14 and 22 —
/// with a 23-character mechanism name and a 15-character position the longest cut short on the panel
/// (bench_r3a.cpp cuts each to its budget); the log line carries every one whole.
struct PromptText {
    char title[64]{};
    char sub[96]{};
    char yesBig[32]{};
    char yesSmall[64]{};
    char noBig[32]{};
    char noSmall[64]{};
    char foot1[96]{};
    char foot2[96]{};
};

/// Split `text` into lines of at most `width` characters at spaces (a word longer than a line is cut), into
/// `out` (at most `maxLines`, each NUL-terminated). Returns how many lines were written; the panel shows a
/// verdict's sentence this way.
template <std::size_t N>
std::size_t wrapText(const char* text, std::size_t width, char (*out)[N], std::size_t maxLines) {
    std::size_t lines = 0;
    if (text == nullptr || width == 0 || width >= N) {
        return 0;
    }
    std::size_t at = 0;
    const std::size_t len = std::strlen(text);
    while (at < len && lines < maxLines) {
        while (at < len && text[at] == ' ') ++at;
        if (at >= len) break;
        std::size_t take = std::min(width, len - at);
        if (at + take < len && text[at + take] != ' ') {
            std::size_t back = take;
            while (back > 0 && text[at + back] != ' ') --back;
            if (back > 0) take = back;
        }
        std::memcpy(out[lines], text + at, take);
        out[lines][take] = '\0';
        ++lines;
        at += take;
    }
    return lines;
}

/// The number `absoluteSensor(at, reads)` takes for an angle sensor whose RAW reading is `rawDeg`: the raw
/// reading in [0, 360), to its tenth (the mechanism service's zeroAbsolute() matches the declaration's
/// `reads` against the RAW sensor — its mounting is applied to both sides — and the declaration refuses a
/// `reads` outside [0, 360)). Never the "as mounted" number: for a Reversed sensor that one is the joint's
/// direction, not what the declaration takes.
[[nodiscard]] inline double absoluteReadsDeg(double rawDeg) noexcept {
    if (!std::isfinite(rawDeg)) {
        return rawDeg;
    }
    double r = std::round(std::fmod(rawDeg, 360.0) * 10.0) / 10.0;
    if (r < 0.0) r += 360.0;
    if (r >= 360.0) r -= 360.0;
    return r + 0.0;  // never "-0.0"
}

/// Split one log `text` into lines of at most `width` characters, broken at spaces (a word longer than a
/// line is cut), handing each to `out(const char*)`: the first is the text's start, every later one starts
/// with `cont` — a "[R3A] mech ..." prefix naming what it continues — so a grep of the tag keeps every piece
/// and a path (it holds no space) stays whole on one line. Station 13's lines go out through bench_r3a.cpp's
/// emit(), ONE serial write each only up to 255 characters (hal::pros::kSerialLineBytes less its '\n',
/// SER1): a longer one would go out as two writes, the second untagged. `width` above `cont`'s length
/// by 32 or more; at most 511 characters of `text` are read.
template <class Out>
void logPieces(const char* text, const char* cont, std::size_t width, Out&& out) {
    constexpr std::size_t kMax = 511;
    if (text == nullptr) {
        return;
    }
    const std::size_t len = std::min(std::strlen(text), kMax);
    const std::size_t contLen = cont != nullptr ? std::strlen(cont) : 0;
    if (width == 0 || width > kMax || contLen + 32 > width) {
        return;
    }
    char piece[kMax + 1];
    std::size_t at = 0;
    bool first = true;
    while (at < len) {
        const std::size_t head = first ? 0 : contLen;
        if (!first) {
            while (at < len && text[at] == ' ') ++at;
            if (at >= len) break;
        }
        std::size_t take = std::min(width - head, len - at);
        if (at + take < len && text[at + take] != ' ') {
            std::size_t back = take;
            while (back > 0 && text[at + back] != ' ') --back;
            if (back > 0) take = back;
        }
        if (head > 0) std::memcpy(piece, cont, head);
        std::memcpy(piece + head, text + at, take);
        piece[head + take] = '\0';
        out(static_cast<const char*>(piece));
        at += take;
        first = false;
    }
}

/// Which of a mechanism's reading lines the live page shows (readingsPage()).
struct ReadingsPage {
    std::size_t first = 0;  ///< the first line shown
    std::size_t count = 0;  ///< how many are shown from it
    std::size_t page = 0;   ///< this page, 0-based
    std::size_t pages = 1;  ///< how many pages
};

/// The page `page` (any number: it wraps) of `n` reading lines on a panel with room for `shown` lines: all
/// of them when they fit; otherwise `shown − 1` a page, the last line kept for pageFooter() — so a 4-motor
/// lift's home switch and its HA-184 question are never cut off the panel, only on its second page.
[[nodiscard]] constexpr ReadingsPage readingsPage(std::size_t n, std::size_t shown, std::size_t page) noexcept {
    ReadingsPage p;
    if (shown < 2 || n <= shown) {
        p.count = std::min(n, shown);
        return p;
    }
    const std::size_t body = shown - 1;
    p.pages = (n + body - 1) / body;
    p.page = page % p.pages;
    p.first = p.page * body;
    p.count = std::min(body, n - p.first);
    return p;
}

/// The paged panel's last line: `-- page 1/2: TAP the readings for more --`.
inline void pageFooter(char* out, std::size_t cap, const ReadingsPage& p) {
    if (out == nullptr || cap == 0) {
        return;
    }
    std::snprintf(out, cap, "-- page %u/%u: TAP the readings for more --", static_cast<unsigned>(p.page + 1),
                  static_cast<unsigned>(p.pages));
}

/// STATION 13's state machine (the header above): the six gates, the direction check, the readings and the
/// RECORD lines, over a list and the parts built from it. One per entry into the station (bench_r3a.cpp
/// keeps it in static storage and makes a new one at each entry: arming, cuts and the ceiling start over).
/// It commands the parts' mechanisms directly and keeps a short queue of LOG lines (events) the caller
/// drains every tick.
class Station {
public:
    /// The longest LOG line it writes, characters: hal::pros::kSerialLineBytes less its '\n' (bench_r3a.cpp
    /// asserts the two equal), so every line is ONE serial write (SER1, HA-189). A longer text — a verdict's
    /// sentence is up to 383 characters — is queued as tagged pieces (logPieces()).
    static constexpr std::size_t kLineChars = 255;
    /// A queued log line's storage, NUL included.
    static constexpr std::size_t kLineBytes = kLineChars + 1;
    /// The longest TEXT it formats before splitting it into lines — a verdict's or the question's whole
    /// sentence and its prefix — NUL included (bench_r3a.cpp sizes its panel buffers by it).
    static constexpr std::size_t kTextBytes = 512;
    /// How many log lines it holds between two drains (bench_r3a.cpp drains every tick; a tick's worst is a
    /// cut, an at-once verdict in three pieces and a ceiling step — 16 leaves room).
    static constexpr std::size_t kMaxEvents = 16;
    /// The longest panel line, NUL included.
    static constexpr std::size_t kPanelBytes = 96;
    /// The panel's SMALL columns a wrapped sentence is cut to (calibration::kSmallCols, an ESTIMATE until the
    /// SCREEN RULER is read — HA-136).
    static constexpr std::size_t kPanelColumns = 50;
    /// The reading lines the live page has room for (bench_r3a.cpp's panel: they end above the gates line).
    static constexpr std::size_t kShownLines = 7;
    /// The most reading lines one mechanism can have — every role at once: four motors, its units, the angle
    /// sensor's three, the home switch's two and a piece switch's two (a declaration names fewer: robot x's
    /// lift has six). More than kShownLines: the page is paged (readingsPage()).
    static constexpr std::size_t kMaxPanelLines = mechanism::kMaxMotorRoles + 1 + 3 + 2 + 2;

    /// Over `list` and `parts` (both must outlive the station). `parts` may be built later — before the
    /// stage-1 clear is marked (bench_r3a.cpp builds it on the first YES of a power cycle); until then
    /// nothing is built and every action is refused.
    Station(const mechanism::MechanismTable& list, robot::MechanismParts& parts) : list_{list}, parts_{parts} {
        count_ = list.count < mechanism::kMaxMechanisms ? list.count : mechanism::kMaxMechanisms;
        for (std::size_t i = 0; i < count_; ++i) {
            const mechanism::StallSpec* st = stallOf(list.specs[i]);
            if (st != nullptr && st->stated()) {
                mech_[i].stall.emplace(manipulation::StallConfig{units::Current{st->amps},
                                                                 units::AngularVelocity{st->speedRadS},
                                                                 units::Time{st->time}});
            }
            mech_[i].pneuPos = list.specs[i].kind == mechanism::Kind::Pneumatic ? list.specs[i].pneumatic.start : 0;
        }
    }

    Station(const Station&) = delete;
    Station& operator=(const Station&) = delete;

    // ── what is declared, and what was built (gate 1) ─────────────────────────────────────
    /// How many mechanisms the robot declares.
    [[nodiscard]] std::size_t count() const noexcept { return count_; }
    /// Mechanism `i`'s declaration.
    [[nodiscard]] const mechanism::MechanismSpec& spec(std::size_t i) const { return list_.specs[at(i)]; }
    /// Whether mechanism `i` was built (gate 1: only a built one is ever powered).
    [[nodiscard]] bool built(std::size_t i) const noexcept {
        return i < count_ && i < parts_.size() && parts_.builtAt(i) != nullptr;
    }
    /// Why mechanism `i` was not built — the provider's sentence; "" when it was.
    [[nodiscard]] const char* absence(std::size_t i) const noexcept {
        if (i >= count_) {
            return "no mechanism is declared at this index";
        }
        if (i >= parts_.size()) {
            return "not made yet: answer the station's CLEAR question first";
        }
        return parts_.absence(i);
    }

    // ── gate 2, stage 1: the station-wide CLEAR, before anything is made or powered ──────
    /// Whether the stage-1 clear was answered YES this entry.
    [[nodiscard]] bool cleared() const noexcept { return cleared_; }
    /// The stage-1 YES (bench_r3a.cpp calls it after building the parts). Seeds the read-fault watches, and
    /// each pneumatic's position from its lines' DEVICE readback: the parts live for the whole power cycle and
    /// a Station is new at every entry, so a line an earlier entry left in its SAFE position is where it is
    /// now — not its start (robot y's grip: start 'loose', safe 'shut'). A readback that matches no declared
    /// position is said, and its first fire goes to the safe position.
    void markCleared() {
        cleared_ = true;
        for (std::size_t i = 0; i < count_ && i < parts_.size(); ++i) {
            const mechanism::Devices& d = parts_.devices()[i];
            if (d.pieceFaults != nullptr) mech_[i].pieceWatch.seed(d.pieceFaults->faultedReads());
            if (d.homeSwitchFaults != nullptr) mech_[i].homeWatch.seed(d.homeSwitchFaults->faultedReads());
            const mechanism::MechanismSpec& s = list_.specs[i];
            if (d.pneumatic == nullptr || s.kind != mechanism::Kind::Pneumatic) {
                continue;
            }
            mech_[i].pneuKnown = false;
            for (std::uint8_t p = 0; p < s.pneumatic.positionCount && !mech_[i].pneuKnown; ++p) {
                bool all = true;
                for (std::size_t l = 0; l < s.lineCount && l < mechanism::kMaxLineRoles; ++l) {
                    all = all && d.pneumatic->commandedLevel(l) == s.pneumatic.positions[p].levels[l];
                }
                if (all) {
                    mech_[i].pneuPos = p;
                    mech_[i].pneuKnown = true;
                }
            }
            if (!mech_[i].pneuKnown) {
                mech_[i].pneuPos = s.pneumatic.safe;
                event("[R3A] mech '%.23s': its lines match NO declared position -- its first fire goes to '%.15s'",
                      s.name, s.pneumatic.positions[s.pneumatic.safe].name);
            } else if (mech_[i].pneuPos != s.pneumatic.start) {
                event("[R3A] mech '%.23s' holds '%.15s' (an earlier entry left it there), not its start '%.15s'",
                      s.name, s.pneumatic.positions[mech_[i].pneuPos].name,
                      s.pneumatic.positions[s.pneumatic.start].name);
            }
        }
    }
    /// Whether a pneumatic's start position drives any of its lines HIGH — making that line MOVES its
    /// cylinder (a Low line is the valve's unpowered state, where it sat before the program).
    [[nodiscard]] static bool startsHigh(const mechanism::MechanismSpec& s) noexcept {
        if (s.kind != mechanism::Kind::Pneumatic) {
            return false;
        }
        for (std::size_t l = 0; l < s.lineCount && l < mechanism::kMaxLineRoles; ++l) {
            if (s.pneumatic.positions[s.pneumatic.start].levels[l]) {
                return true;
            }
        }
        return false;
    }
    /// The stage-1 prompt: what this YES does. The FIRST clear of a power cycle makes the mechanisms, and
    /// making a pneumatic's line DRIVES it to its start level (HA-119) — a start with a High line MOVES its
    /// cylinder, and the YES says so. A later entry makes nothing (the parts live for the power cycle): its
    /// YES drives no line, and the prompt says that instead.
    [[nodiscard]] PromptText clearText() const {
        PromptText t;
        const bool made = parts_.size() > 0;
        std::snprintf(t.title, sizeof t.title, "STATION 13 POWERS MECHANISMS");
        std::snprintf(t.sub, sizeof t.sub, "every path clear, hands away? (%u declared)",
                      static_cast<unsigned>(count_));
        std::snprintf(t.yesBig, sizeof t.yesBig, "YES, CLEAR");
        bool moves = false;
        for (std::size_t i = 0; i < count_; ++i) {
            moves = moves || (!made && startsHigh(list_.specs[i]));
        }
        std::snprintf(t.yesSmall, sizeof t.yesSmall, "%s",
                      made ? "nothing moves on YES" : moves ? "a cylinder MOVES" : "nothing moves yet");
        std::snprintf(t.noBig, sizeof t.noBig, "NO");
        std::snprintf(t.noSmall, sizeof t.noSmall, "back to the menu");
        std::size_t n = 0;
        for (std::size_t i = 0; i < count_ && !made; ++i) {
            const mechanism::MechanismSpec& s = list_.specs[i];
            if (s.kind != mechanism::Kind::Pneumatic || n + 1 >= sizeof t.foot1) {
                continue;
            }
            const int w = std::snprintf(t.foot1 + n, sizeof t.foot1 - n, "%s'%.23s' -> '%.15s'%s",
                                        n == 0 ? "making a line DRIVES it: " : ", ", s.name,
                                        s.pneumatic.positions[s.pneumatic.start].name, startsHigh(s) ? " MOVES" : "");
            n = w < 0 ? n : std::min(sizeof t.foot1 - 1, n + static_cast<std::size_t>(w));
        }
        if (made) {
            std::snprintf(t.foot1, sizeof t.foot1, "made earlier this power cycle: no line is driven");
        } else if (n == 0) {
            std::snprintf(t.foot1, sizeof t.foot1, "making a motor mechanism moves nothing");
        }
        std::snprintf(t.foot2, sizeof t.foot2, "then hold L1 and confirm each one before it moves");
        return t;
    }

    // ── which mechanism the page shows (one at a time; only it is ever powered) ─────────
    /// The mechanism on the page.
    [[nodiscard]] std::size_t selected() const noexcept { return selected_; }
    /// Show the NEXT mechanism (NEXT on the panel): the shown one is made safe first if it is powered, a
    /// running check is abandoned, a pending question dropped — and the ceiling goes back to its START
    /// (gate 4 is a gentle FIRST motion for every mechanism: one raised for a heavy pivot never carries over
    /// to the next one's first jog).
    void next() {
        if (count_ == 0) {
            return;
        }
        leaveSelected("NEXT pressed");
        selected_ = (selected_ + 1) % count_;
        event("[R3A] mech page: '%.23s' (%s)", list_.specs[selected_].name, mechanismKindWord(list_.specs[selected_]));
        if (ceiling_ != kCeilingStartV) {
            event("[R3A] mech ceiling %.0f V -> %.0f V: every mechanism starts at %.0f V (R1 raises it)", ceiling_,
                  kCeilingStartV, kCeilingStartV);
            ceiling_ = kCeilingStartV;
        }
    }

    // ── gate 4 ─────────────────────────────────────────────────────────────────────────
    /// The volts ceiling now.
    [[nodiscard]] double ceiling() const noexcept { return ceiling_; }
    /// The most mechanism `i` may be driven at now: the ceiling — and for a POWER mechanism never above its
    /// declared maxVolts, "the most any command may ask" (the mechanism service clamps every run to it; a team
    /// that lowered it protects something). A position or speed mechanism declares no such limit (its
    /// unhomedVolts is the service's rule before homing; the page exists to check a direction BEFORE homing,
    /// behind its own gates).
    [[nodiscard]] double voltsFor(std::size_t i) const noexcept {
        if (i < count_ && list_.specs[i].kind == mechanism::Kind::Power) {
            return std::min(ceiling_, list_.specs[i].power.maxVolts);
        }
        return ceiling_;
    }
    /// Whether mechanism `i` declares a maxVolts below the ceiling's most (the page says it).
    [[nodiscard]] bool capped(std::size_t i) const noexcept {
        return i < count_ && list_.specs[i].kind == mechanism::Kind::Power &&
               list_.specs[i].power.maxVolts < kCeilingMaxV;
    }

    // ── gate 5 ─────────────────────────────────────────────────────────────────────────
    /// Whether mechanism `i` was cut this entry.
    [[nodiscard]] bool cut(std::size_t i) const noexcept { return i < count_ && mech_[i].cut; }
    /// Why it was cut ("" when it was not).
    [[nodiscard]] const char* cutWhy(std::size_t i) const noexcept { return i < count_ ? mech_[i].cutWhy : ""; }

    // ── gate 2, stage 2 ────────────────────────────────────────────────────────────────
    /// Whether motor mechanism `i` was confirmed for power this entry.
    [[nodiscard]] bool armed(std::size_t i) const noexcept { return i < count_ && mech_[i].armed; }
    /// Where the page is.
    [[nodiscard]] Phase phase() const noexcept { return phase_; }
    /// The pending confirm's prompt (phase() == Confirm).
    [[nodiscard]] PromptText confirmText() const {
        PromptText t;
        const mechanism::MechanismSpec& s = list_.specs[at(selected_)];
        if (pendingFire_) {
            const char* to = s.pneumatic.positions[pendingTarget_].name;
            const char* safe = s.pneumatic.positions[s.pneumatic.safe].name;
            std::snprintf(t.title, sizeof t.title, "FIRE '%.23s' -> '%.15s'?", s.name, to);
            std::snprintf(t.sub, sizeof t.sub, "hands clear: it moves when you touch YES");
            std::snprintf(t.yesBig, sizeof t.yesBig, "YES, FIRE");
            std::snprintf(t.yesSmall, sizeof t.yesSmall, "keep L1 held");
            std::snprintf(t.noBig, sizeof t.noBig, "NO");
            std::snprintf(t.noSmall, sizeof t.noSmall, "nothing fires");
            std::snprintf(t.foot1, sizeof t.foot1, "ONLY '%.23s' should move. Letting go of L1", s.name);
            std::snprintf(t.foot2, sizeof t.foot2, "returns it to '%.15s' (its safe position).", safe);
        } else {
            std::snprintf(t.title, sizeof t.title, "POWER '%.23s'?", s.name);
            std::snprintf(t.sub, sizeof t.sub, "ONLY it may move: path clear, hands away?");
            std::snprintf(t.yesBig, sizeof t.yesBig, "YES, POWER IT");
            std::snprintf(t.yesSmall, sizeof t.yesSmall, "UP/DOWN jog, A checks");
            std::snprintf(t.noBig, sizeof t.noBig, "NO");
            std::snprintf(t.noSmall, sizeof t.noSmall, "nothing is powered");
            std::snprintf(t.foot1, sizeof t.foot1, "hold L1 throughout: letting go stops it that tick");
            if (capped(selected_)) {
                std::snprintf(t.foot2, sizeof t.foot2, "ceiling %.0f V, at most %.1f V (maxVolts); R1 raises", ceiling_,
                              s.power.maxVolts);
            } else {
                std::snprintf(t.foot2, sizeof t.foot2, "ceiling %.0f V, R1 raises; numbers INVENTED (HA-182)",
                              ceiling_);
            }
        }
        return t;
    }
    /// The stage-2 answer. `deadManHeldNow`: L1 held at the moment of the answer (read after the prompt
    /// returned; bench_r3a.cpp's prompt also closes the moment L1 is let go, answering false/false). L1 let
    /// go grants NOTHING — a pneumatic is not fired, a motor mechanism is not armed — and is the dead-man's
    /// release: whatever the station powered goes safe here. A motor mechanism is only ARMED by YES: it moves
    /// on the next press of UP, DOWN or A.
    void confirm(bool yes, bool deadManHeldNow) {
        if (phase_ != Phase::Confirm) {
            return;
        }
        phase_ = Phase::Idle;
        // A confirm is only ever asked with L1 held: L1 let go during it is the dead-man's release, here and
        // now — whatever the station powered (a claw fired before) goes safe.
        if (!deadManHeldNow && anyLive()) {
            safeLive();
            event("[R3A] mech L1 released during the prompt: every powered mechanism safe");
        }
        heldWas_ = deadManHeldNow;
        const std::size_t i = selected_;
        const mechanism::MechanismSpec& s = list_.specs[i];
        if (!pendingFire_) {
            mech_[i].armed = yes && deadManHeldNow;
            if (mech_[i].armed) {
                event("[R3A] mech ARMED '%.23s': hold L1, then UP / DOWN jog it, A checks its direction", s.name);
            } else if (!deadManHeldNow) {
                event("[R3A] mech not armed: '%.23s' -- L1 was let go during the prompt; hold L1, press again",
                      s.name);
            } else {
                event("[R3A] mech not armed: '%.23s' -- nothing powered", s.name);
            }
            return;
        }
        pendingFire_ = false;
        const char* to = s.pneumatic.positions[pendingTarget_].name;
        if (!yes && !deadManHeldNow) {
            event("[R3A] mech not fired: '%.23s' -> '%.15s' -- L1 was let go during the prompt", s.name, to);
            return;
        }
        if (!yes) {
            event("[R3A] mech not fired: '%.23s' -> '%.15s' was answered NO", s.name, to);
            return;
        }
        if (!deadManHeldNow) {
            event("[R3A] mech NOT fired: '%.23s' -- L1 was not held when YES was touched; hold L1 and press A again",
                  s.name);
            return;
        }
        if (!built(i) || mech_[i].cut) {
            event("[R3A] mech NOT fired: '%.23s' -- %s", s.name, mech_[i].cut ? mech_[i].cutWhy : absence(i));
            return;
        }
        fire(i, pendingTarget_);
    }

    // ── the tick ───────────────────────────────────────────────────────────────────────
    /// One tick (every kTickMs): gates 3, 4 and 5 and the direction check's jog; at most one mechanism
    /// powered — the shown one. `nowMs` is the clock, whole milliseconds (pros::millis()).
    void tick(const Pad& pad, std::uint32_t nowMs) {
        const bool held = pad.connected && pad.deadMan;
        // A PRESS is a release-then-press SEEN on a connected controller: while it is gone the edges keep their
        // last level, so an arrow still held when it comes back is not a new press (nothing resumes by itself).
        const bool plusPress = pad.connected && plusEdge_.update(pad.plus);
        const bool minusPress = pad.connected && minusEdge_.update(pad.minus);
        const bool checkPress = pad.connected && checkEdge_.update(pad.check);
        // GATE 4: the ceiling steps on R1's press, never above 12 V (a power mechanism's maxVolts still caps it).
        if (pad.connected && raiseEdge_.update(pad.raise)) {
            const double was = ceiling_;
            ceiling_ = std::min(ceiling_ + kCeilingStepV, kCeilingMaxV);
            if (capped(selected_)) {
                event("[R3A] mech ceiling %.0f V -> %.0f V%s; '%.23s' at most %.1f V (its maxVolts)", was, ceiling_,
                      was == ceiling_ ? " (the most)" : "", list_.specs[selected_].name,
                      list_.specs[selected_].power.maxVolts);
            } else {
                event("[R3A] mech ceiling %.0f V -> %.0f V%s", was, ceiling_, was == ceiling_ ? " (the most)" : "");
            }
        }
        // GATE 3: the dead-man. Released, or the controller gone: every powered mechanism safe THIS tick.
        if (!held) {
            if (heldWas_) {
                if (phase_ == Phase::Announcing || phase_ == Phase::Jogging) {
                    abandonCheck(pad.connected ? "L1 was let go before the check's jog ended -- nothing judged"
                                               : "the controller DROPPED during the check -- nothing judged");
                }
                if (anyLive()) {
                    safeLive();
                    event("[R3A] mech %s: every powered mechanism safe",
                          pad.connected ? "L1 released" : "controller DROPPED");
                }
            }
            heldWas_ = false;
            jogDir_ = 0;
            return;
        }
        heldWas_ = true;
        if (phase_ == Phase::Confirm || phase_ == Phase::Asking) {
            return;  // nothing new is powered while a prompt or the question waits
        }
        if (phase_ == Phase::Announcing) {
            stepAnnounce(nowMs);
            return;
        }
        if (phase_ == Phase::Jogging) {
            stepCheck(nowMs);
            return;
        }
        const std::size_t i = selected_;
        if (i >= count_) {
            return;
        }
        const mechanism::MechanismSpec& s = list_.specs[i];
        const bool press = plusPress || minusPress || checkPress;
        if (press && !cleared_) {
            event("[R3A] mech REFUSED: the station's CLEAR question was not answered YES");
            return;
        }
        if (press && !built(i)) {  // GATE 1
            event("[R3A] mech REFUSED '%.23s': it was not built -- %.200s", s.name, absence(i));
            return;
        }
        if (press && mech_[i].cut) {  // GATE 5's latch
            event("[R3A] mech REFUSED '%.23s': CUT (%.200s) -- leave and re-enter station 13", s.name, mech_[i].cutWhy);
            return;
        }
        if (!cleared_ || !built(i) || mech_[i].cut) {
            return;
        }
        if (s.kind == mechanism::Kind::Pneumatic) {
            if (plusPress || checkPress) {  // GATE 2: EVERY fire is confirmed, naming its position
                pendingFire_ = true;
                // the NEXT declared position from where its lines are; from an unknown one, its safe position
                pendingTarget_ = mech_[i].pneuKnown
                                     ? static_cast<std::uint8_t>((mech_[i].pneuPos + 1) % s.pneumatic.positionCount)
                                     : s.pneumatic.safe;
                phase_ = Phase::Confirm;
            }
            return;
        }
        if (!mech_[i].armed) {  // GATE 2: a motor mechanism's first volt this entry is confirmed
            if (press) {
                pendingFire_ = false;
                phase_ = Phase::Confirm;
            }
            return;
        }
        if (checkPress && !mech_[i].live) {
            startCheck(nowMs);
            return;
        }
        if (checkPress) {  // a check starts from REST: never the jog reversed in one tick, nor a moving baseline
            event("[R3A] mech REFUSED A: '%.23s' is being jogged -- let go of the arrow first; a check starts "
                  "from rest",
                  s.name);
        }
        // A hold-to-run jog: it starts on an arrow's PRESS (L1 already held) and runs while that arrow is held.
        if (plusPress || minusPress) {
            jogDir_ = plusPress ? +1 : -1;
            jogStartMs_ = nowMs;
            resetWatch(i, nowMs);
        }
        const bool stillHeld = (jogDir_ > 0 && pad.plus) || (jogDir_ < 0 && pad.minus);
        if (jogDir_ == 0 || !stillHeld) {
            if (mech_[i].live) {
                safeOne(i);
                event("[R3A] mech '%.23s': jog released -- safe", s.name);
            }
            jogDir_ = 0;
            return;
        }
        drive(i, jogDir_ * voltsFor(i), nowMs);
        if (!mech_[i].cut && nowMs - jogStartMs_ >= kJogMaxMs) {
            char why[96];
            std::snprintf(why, sizeof why, "a jog held for its stated maximum, %.1f s (HA-182)",
                          static_cast<double>(kJogMaxMs) / 1000.0);
            cutMech(i, why);
        }
    }

    // ── the direction check's question ─────────────────────────────────────────────────
    /// How many answers the pending question has: three for a motor mechanism, a pneumatic's positions
    /// plus "nothing / something else moved"; 0 when none is pending.
    [[nodiscard]] int answerCount() const noexcept {
        if (phase_ != Phase::Asking) {
            return 0;
        }
        const mechanism::MechanismSpec& s = list_.specs[checkIdx_];
        return s.kind == mechanism::Kind::Pneumatic ? s.pneumatic.positionCount + 1 : 3;
    }
    /// Answer `slot`'s two label lines (each NUL-terminated, at most 15 characters).
    void answerWords(int slot, char (&line1)[16], char (&line2)[16]) const {
        line1[0] = '\0';
        line2[0] = '\0';
        if (slot < 0 || slot >= answerCount()) {
            return;
        }
        const mechanism::MechanismSpec& s = list_.specs[checkIdx_];
        if (s.kind != mechanism::Kind::Pneumatic) {
            static constexpr const char* kA[3][2] = {{"ITS + WAY", ""}, {"THE OTHER", "WAY"}, {"NOT MOVED /", "ELSE MOVED"}};
            std::snprintf(line1, sizeof line1, "%s", kA[slot][0]);
            std::snprintf(line2, sizeof line2, "%s", kA[slot][1]);
            return;
        }
        if (slot == s.pneumatic.positionCount) {
            std::snprintf(line1, sizeof line1, "NOTHING /");
            std::snprintf(line2, sizeof line2, "ELSE MOVED");
            return;
        }
        std::snprintf(line1, sizeof line1, "%.15s", s.pneumatic.positions[slot].name);
    }
    /// The question, as the page asks it.
    void questionText(char* out, std::size_t cap) const {
        if (out == nullptr || cap == 0) {
            return;
        }
        out[0] = '\0';
        if (phase_ != Phase::Asking) {
            return;
        }
        const mechanism::MechanismSpec& s = list_.specs[checkIdx_];
        if (s.kind == mechanism::Kind::Pneumatic) {
            std::snprintf(out, cap, "ONLY '%.23s' should have moved. Told '%.15s': which position did it go to?",
                          s.name, s.pneumatic.positions[pneuCommanded_].name);
        } else {
            std::snprintf(out, cap,
                          "ONLY '%.23s' should have moved: did it go its + way, the OTHER way, or did it NOT move "
                          "(or something ELSE moved)?",
                          s.name);
        }
    }
    /// The person's answer (the panel's answer buttons, left to right): for a motor mechanism 0 its + way, 1
    /// the other way, 2 it did NOT move or something ELSE moved; for a pneumatic its position `slot`, or
    /// "nothing / something else" at slot positionCount. Anything else is ignored.
    void answer(int slot) {
        if (phase_ != Phase::Asking || slot < 0 || slot >= answerCount()) {
            return;
        }
        const mechanism::MechanismSpec& s = list_.specs[checkIdx_];
        const mechanism::MechanismWiring& w = list_.wiring[checkIdx_];
        char said[32];
        if (s.kind == mechanism::Kind::Pneumatic) {
            const auto saw = slot == s.pneumatic.positionCount ? mechanism::kNothingOrSomethingElseMoved
                                                               : static_cast<std::uint8_t>(slot);
            verdict_ = mechanism::judgePneumatic(s, w, pneuCommanded_, saw);
            std::snprintf(said, sizeof said, "%.15s", saw == mechanism::kNothingOrSomethingElseMoved
                                                          ? "nothing / else" : s.pneumatic.positions[saw].name);
        } else {
            static constexpr mechanism::PersonSaw kSaw[3] = {mechanism::PersonSaw::ItsPlusWay,
                                                            mechanism::PersonSaw::TheOtherWay,
                                                            mechanism::PersonSaw::NothingOrSomethingElseMoved};
            static constexpr const char* kSaid[3] = {"its + way", "the other way", "not moved / else"};
            observation_.person = kSaw[slot];
            verdict_ = mechanism::judgeDirection(s, w, observation_, kMinMoveRad, kDrawAmps);
            std::snprintf(said, sizeof said, "%s", kSaid[slot]);
        }
        std::snprintf(answered_, sizeof answered_, "%s", said);
        phase_ = Phase::Idle;
        keepVerdict();
        char cont[64];
        std::snprintf(cont, sizeof cont, "[R3A] mech CHECK '%.23s' (cont.) ", s.name);
        eventCont(cont, "[R3A] mech CHECK '%.23s': answered '%s' -> %s: %.383s", s.name, said,
                  findingWord(verdict_.finding), verdict_.fix);
    }
    /// Drop the pending question (B, or NEXT): nothing is judged.
    void cancelQuestion() {
        if (phase_ != Phase::Asking) {
            return;
        }
        phase_ = Phase::Idle;
        event("[R3A] mech CHECK '%.23s': question dropped -- nothing judged", list_.specs[checkIdx_].name);
    }

    // ── the verdict ────────────────────────────────────────────────────────────────────
    /// Whether the LAST check (the one verdictOf() names) ended in a verdict — false again while a new check
    /// runs, and after one is abandoned.
    [[nodiscard]] bool hasVerdict() const noexcept { return hasVerdict_; }
    /// The last verdict as the panel shows it (wrapped to kPanelColumns and paged by bench_r3a.cpp): its
    /// finding, its whole sentence, and "nothing is saved" — at most 459 characters.
    void verdictPanelText(char* out, std::size_t cap) const {
        if (out == nullptr || cap == 0) {
            return;
        }
        std::snprintf(out, cap, "%s: %.383s -- edit the robot file, rebuild, upload (nothing is saved)",
                      findingWord(verdict_.finding), verdict_.fix);
    }
    /// Whether mechanism `i` has a verdict this entry — its LATEST check's (a check abandoned, or still
    /// running, leaves the one before it standing: a cable re-plugged and checked again is that mechanism's
    /// answer now).
    [[nodiscard]] bool hasVerdictFor(std::size_t i) const noexcept { return i < count_ && mech_[i].hasVerdict; }
    /// Mechanism `i`'s latest verdict (hasVerdictFor(i)).
    [[nodiscard]] const mechanism::DirectionVerdict& verdictFor(std::size_t i) const { return mech_[at(i)].verdict; }
    /// What the person answered in mechanism `i`'s latest verdict ("" when it needed no answer).
    [[nodiscard]] const char* answeredFor(std::size_t i) const noexcept { return i < count_ ? mech_[i].answered : ""; }
    /// Whether ANY mechanism's latest verdict this entry is a finding (not OK) — the menu's FAIL dot.
    [[nodiscard]] bool anyFinding() const noexcept {
        for (std::size_t i = 0; i < count_; ++i) {
            if (mech_[i].hasVerdict && mech_[i].verdict.finding != mechanism::DirectionFinding::Ok) {
                return true;
            }
        }
        return false;
    }
    /// Whether any mechanism has a verdict this entry.
    [[nodiscard]] bool anyVerdict() const noexcept {
        for (std::size_t i = 0; i < count_; ++i) {
            if (mech_[i].hasVerdict) {
                return true;
            }
        }
        return false;
    }
    /// How many verdicts this entry has reached — the page shows each NEW one until the operator moves on.
    [[nodiscard]] std::uint32_t verdictCount() const noexcept { return verdicts_; }
    /// The mechanism the last verdict is about.
    [[nodiscard]] std::size_t verdictOf() const noexcept { return checkIdx_; }
    /// The last verdict.
    [[nodiscard]] const mechanism::DirectionVerdict& verdict() const noexcept { return verdict_; }
    /// The last check's observation (a motor mechanism's).
    [[nodiscard]] const mechanism::DirectionObservation& observation() const noexcept { return observation_; }
    /// A finding as the page and the log name it.
    [[nodiscard]] static constexpr const char* findingWord(mechanism::DirectionFinding f) noexcept {
        switch (f) {
            case mechanism::DirectionFinding::Ok: return "OK";
            case mechanism::DirectionFinding::DidNotMove: return "DID NOT MOVE";
            case mechanism::DirectionFinding::MotorFights: return "A MOTOR FIGHTS";
            case mechanism::DirectionFinding::FightOrJam: return "FIGHT OR JAM";
            case mechanism::DirectionFinding::SensorReversed: return "SENSOR REVERSED";
            case mechanism::DirectionFinding::MechanismReversed: return "REVERSED";
            case mechanism::DirectionFinding::CableOnAnotherMechanism: return "CABLE ELSEWHERE";
            case mechanism::DirectionFinding::AwaitingPerson: return "ASKING";
            case mechanism::DirectionFinding::BackDriven: return "BACK-DRIVEN";
        }
        return "?";
    }

    // ── gate 6, and the powered state ──────────────────────────────────────────────────
    /// Whether mechanism `i` is powered by the station now (driven, or a pneumatic fired).
    [[nodiscard]] bool live(std::size_t i) const noexcept { return i < count_ && mech_[i].live; }
    /// The volts commanded to mechanism `i` this tick (0 when it is not driven).
    [[nodiscard]] double commanded(std::size_t i) const noexcept { return i < count_ ? mech_[i].commanded : 0.0; }
    /// Every powered mechanism to its declared safe state (the dead-man's release, the exit, a NEXT).
    void safeLive() {
        for (std::size_t i = 0; i < count_; ++i) {
            if (mech_[i].live) {
                safeOne(i);
            }
        }
    }
    /// GATE 6: leave the station — a running check abandoned, a question dropped, every powered mechanism
    /// safe. bench_r3a.cpp calls it on DONE and its guard on every other way out; idempotent.
    void exit() {
        if (phase_ == Phase::Announcing || phase_ == Phase::Jogging) {
            abandonCheck("the station was left during the check -- nothing judged");
        }
        if (phase_ == Phase::Asking || phase_ == Phase::Confirm) {
            phase_ = Phase::Idle;
            pendingFire_ = false;
        }
        safeLive();
    }

    // ── the readings, for the panel and RECORD ─────────────────────────────────────────
    /// Mechanism `i`'s live panel lines into `out` (at most `max`), from its devices — `rawTempC(port)` is the
    /// RAW temperature read of smart port `port` (non-finite = its read failed). Returns how many.
    /// Lines: one per motor (motorText) or per line ("line A: commanded HIGH") and the position, the units
    /// reading, the angle sensor (raw and as mounted; the number absoluteSensor takes; HA-186's question), the
    /// home switch (and HA-184's question), the piece sensor (a switch: HA-184's question too) — at most
    /// kMaxPanelLines (bench_r3a.cpp pages them, readingsPage()). A read-fault count is SAMPLED here.
    template <class TempOf>
    std::size_t panelLines(std::size_t i, TempOf&& rawTempC, char (*out)[kPanelBytes], std::size_t max) {
        std::size_t n = 0;
        if (i >= count_) {
            return 0;
        }
        const mechanism::MechanismSpec& s = list_.specs[i];
        const mechanism::MechanismWiring& w = list_.wiring[i];
        if (!built(i)) {
            lineAt(out, n, max, "ABSENT -- nothing on this page is powered:");
            char wrapped[4][kPanelBytes];
            const std::size_t k = wrapText(absence(i), kPanelColumns, wrapped, 4);
            for (std::size_t j = 0; j < k; ++j) {
                lineAt(out, n, max, "%s", wrapped[j]);
            }
            return n;
        }
        const mechanism::Devices& d = parts_.devices()[i];
        if (d.motors != nullptr) {
            const auto motors = d.motors->motors();
            for (std::size_t m = 0; m < motors.size() && m < mechanism::kMaxMotorRoles && n < max; ++m) {
                motorText(out[n++], kPanelBytes, m, w.motors[m], motors[m]->position().value(),
                          motors[m]->velocity().value(), motors[m]->current().value(), rawTempC(w.motors[m]));
            }
            const UnitsReading r =
                unitsReading(s, d.motors->meanPosition().value(), d.motors->meanVelocity().value());
            if (r.has && n < max) {
                unitsText(out[n++], kPanelBytes, s, r);
            }
        } else if (d.pneumatic != nullptr) {
            for (std::size_t l = 0; l < s.lineCount && l < mechanism::kMaxLineRoles; ++l) {
                lineAt(out, n, max, "line %c: commanded %s", mechanism::lineLetter(w.lines[l]),
                    d.pneumatic->commandedLevel(l) ? "HIGH" : "LOW");
            }
            if (mech_[i].pneuKnown) {
                lineAt(out, n, max, "position: '%.15s'%s", s.pneumatic.positions[mech_[i].pneuPos].name,
                    mech_[i].live ? " (FIRED)" : "");
            } else {
                lineAt(out, n, max, "position: UNKNOWN (no declared one matches)");
            }
        }
        if (d.angleSensor != nullptr) {
            constexpr double kDeg = 180.0 / 3.14159265358979323846;
            const double raw = d.angleSensor->position().value() * kDeg;
            lineAt(out, n, max, "angle p%u: raw %+.1fdeg, as mounted %+.1fdeg", static_cast<unsigned>(w.angleSensor), raw,
                s.angleSensor.reversed ? -raw : raw);
            lineAt(out, n, max, "  absoluteSensor(at, reads): reads = %.1f", absoluteReadsDeg(raw));
            lineAt(out, n, max, "  RECORD it at 2 marks: power-cycled, restarted");  // HA-186's settle (48: the panel's 50)
        }
        if (d.homeSwitch != nullptr) {
            const bool level = d.homeSwitch->state();
            const bool moved =
                d.homeSwitchFaults != nullptr && mech_[i].homeWatch.moved(d.homeSwitchFaults->faultedReads());
            if (n < max) {
                switchText(out[n++], kPanelBytes, "home sw", mechanism::lineLetter(w.homeSwitch), level,
                           s.homeSwitch.pressedLevel, moved);
            }
            lineAt(out, n, max, "  RECORD it pressed, released, and cable OUT");  // HA-184's settle
        }
        if (n < max && pieceLine(i, d, out[n], kPanelBytes)) {
            ++n;
            if (d.pieceSwitch != nullptr) {  // a piece SWITCH is a limit switch too: HA-184's settle
                lineAt(out, n, max, "  RECORD it pressed, released, and cable OUT");
            }
        }
        return n;
    }

    /// The direction check's announcement (phase() == Announcing), for the panel: what should move, what
    /// will drive it, and when.
    void announceText(char* out, std::size_t cap, std::uint32_t nowMs) const {
        if (out == nullptr || cap == 0) {
            return;
        }
        out[0] = '\0';
        if (phase_ != Phase::Announcing) {
            return;
        }
        const mechanism::MechanismSpec& s = list_.specs[checkIdx_];
        const std::uint32_t since = nowMs - announceStartMs_;
        const std::uint32_t left = since < kAnnounceMs ? kAnnounceMs - since : 0;
        std::snprintf(out, cap,
                      "CHECK '%.23s': ONLY '%.23s' should move -- watch it. A + jog at %.1f V for %.2f s starts in "
                      "%.1f s; keep L1 held.",
                      s.name, s.name, observation_.commandVolts, static_cast<double>(kCheckJogMs) / 1000.0,
                      static_cast<double>(left) / 1000.0);
    }

    /// The live page's GATES line for mechanism `i` (at most kPanelColumns): the controller missing, a cut,
    /// the check, powered, or the ceiling and the buttons (a power mechanism's maxVolts said when it caps).
    void gatesText(std::size_t i, bool connected, char* out, std::size_t cap) const {
        if (out == nullptr || cap == 0) {
            return;
        }
        if (!connected) {
            std::snprintf(out, cap, "CONTROLLER NOT CONNECTED -- nothing can be powered");
        } else if (cut(i)) {
            std::snprintf(out, cap, "CUT: %.44s", cutWhy(i));
        } else if (phase_ == Phase::Announcing) {
            std::snprintf(out, cap, "CHECK: watch it -- keep L1 held");
        } else if (phase_ == Phase::Jogging) {
            std::snprintf(out, cap, "CHECK: jog +%.1f V -- keep L1 held", observation_.commandVolts);
        } else if (live(i)) {
            std::snprintf(out, cap, "POWERED %+.1f V -- let go of L1 to stop", commanded(i));
        } else if (capped(i)) {
            std::snprintf(out, cap, "ceiling %.0fV, max %.1fV | L1+UP/DN | L1+A | R1", ceiling_,
                          list_.specs[i].power.maxVolts);
        } else {
            std::snprintf(out, cap, "ceiling %.0fV | L1+UP/DN jog | L1+A check | R1 raise", ceiling_);
        }
    }

    /// RECORD: mechanism `i`'s lines (`[R3A] mech REC <rec> ...`, every number in full) handed to
    /// `emit(const char*)` — its kind and built state, each motor or line, the units reading, every sensor,
    /// and ITS latest verdict this entry. Every line is at most kLineChars: a longer one goes as tagged
    /// pieces (`[R3A] mech REC <rec> '<name>' (cont.) ...`, logPieces()).
    template <class TempOf, class Emit>
    void record(int rec, std::size_t i, TempOf&& rawTempC, Emit&& emit) {
        if (i >= count_) {
            return;
        }
        constexpr double kDeg = 180.0 / 3.14159265358979323846;
        const mechanism::MechanismSpec& s = list_.specs[i];
        const mechanism::MechanismWiring& w = list_.wiring[i];
        char cont[64];
        std::snprintf(cont, sizeof cont, "[R3A] mech REC %d '%.23s' (cont.) ", rec, s.name);
        char line[kTextBytes];
        const auto out = [&]() { logPieces(line, cont, kLineChars, emit); };
        if (!built(i)) {
            std::snprintf(line, sizeof line, "[R3A] mech REC %d '%.23s' %s ABSENT: %.200s", rec, s.name, mechanismKindWord(s),
                          absence(i));
            out();
            return;
        }
        char most[64] = "";
        if (capped(i)) {
            std::snprintf(most, sizeof most, " (its maxVolts %.1f V: the jog and the check at most %.1f V)",
                          s.power.maxVolts, voltsFor(i));
        }
        std::snprintf(line, sizeof line, "[R3A] mech REC %d '%.23s' %s built, %s, ceiling %.0f V%s%s%.200s", rec, s.name,
                      mechanismKindWord(s), mech_[i].armed ? "armed" : "not armed", ceiling_, most,
                      mech_[i].cut ? ", CUT: " : "", mech_[i].cut ? mech_[i].cutWhy : "");
        out();
        const mechanism::Devices& d = parts_.devices()[i];
        if (d.motors != nullptr) {
            const auto motors = d.motors->motors();
            for (std::size_t m = 0; m < motors.size() && m < mechanism::kMaxMotorRoles; ++m) {
                char t[16];
                temperatureText(t, sizeof t, rawTempC(w.motors[m]));
                std::snprintf(line, sizeof line,
                              "[R3A] mech REC %d '%.23s' motor %u port %u pos %+.3f rad (%+.1f deg) vel %+.3f rad/s "
                              "cur %.3f A temp %s",
                              rec, s.name, static_cast<unsigned>(m + 1), static_cast<unsigned>(w.motors[m]),
                              motors[m]->position().value(), motors[m]->position().value() * kDeg,
                              motors[m]->velocity().value(), motors[m]->current().value(), t);
                out();
            }
            const UnitsReading r = unitsReading(s, d.motors->meanPosition().value(), d.motors->meanVelocity().value());
            if (r.has) {
                std::snprintf(line, sizeof line,
                              "[R3A] mech REC %d '%.23s' %s %+.4f %s from power-up (output %+.2f deg through %.4f "
                              "motor turns per output turn; the power-up pose taken as its home %.4f)",
                              rec, s.name, mechanismKindWord(s), r.value, r.unit, r.outputDeg,
                              s.kind == mechanism::Kind::Speed ? s.speed.gearRatio : s.position.gearRatio,
                              s.kind == mechanism::Kind::Speed ? 0.0 : s.position.homePosition);
                out();
            }
        } else if (d.pneumatic != nullptr) {
            for (std::size_t l = 0; l < s.lineCount && l < mechanism::kMaxLineRoles; ++l) {
                std::snprintf(line, sizeof line, "[R3A] mech REC %d '%.23s' line %u 3-wire %c commanded %s", rec,
                              s.name, static_cast<unsigned>(l + 1), mechanism::lineLetter(w.lines[l]),
                              d.pneumatic->commandedLevel(l) ? "HIGH" : "LOW");
                out();
            }
            if (mech_[i].pneuKnown) {
                std::snprintf(line, sizeof line, "[R3A] mech REC %d '%.23s' position '%.15s'%s", rec, s.name,
                              s.pneumatic.positions[mech_[i].pneuPos].name, mech_[i].live ? " (fired, L1 held)" : "");
            } else {
                std::snprintf(line, sizeof line, "[R3A] mech REC %d '%.23s' position UNKNOWN: its lines match no "
                              "declared position", rec, s.name);
            }
            out();
        }
        if (d.angleSensor != nullptr) {
            const double raw = d.angleSensor->position().value();
            std::snprintf(line, sizeof line,
                          "[R3A] mech REC %d '%.23s' angle sensor port %u raw %+.4f rad (%+.2f deg) as mounted "
                          "%+.2f deg; absoluteSensor(at, reads) takes reads = %.1f (the RAW reading, 0..360) "
                          "(HA-186: at power-up, absolute within one turn)",
                          rec, s.name, static_cast<unsigned>(w.angleSensor), raw, raw * kDeg,
                          (s.angleSensor.reversed ? -raw : raw) * kDeg, absoluteReadsDeg(raw * kDeg));
            out();
        }
        if (d.homeSwitch != nullptr) {
            const int faults = d.homeSwitchFaults != nullptr ? d.homeSwitchFaults->faultedReads() : 0;
            std::snprintf(line, sizeof line,
                          "[R3A] mech REC %d '%.23s' home switch 3-wire %c reads %d (pressedLevel %d; HA-184), "
                          "read faults %d -- RECORD it pressed, released and with its cable OUT",
                          rec, s.name, mechanism::lineLetter(w.homeSwitch), d.homeSwitch->state() ? 1 : 0,
                          s.homeSwitch.pressedLevel ? 1 : 0, faults);
            out();
        }
        if (d.pieceSwitch != nullptr) {  // a piece SWITCH: its raw level and pressedLevel, HA-184's settle
            const int faults = d.pieceFaults != nullptr ? d.pieceFaults->faultedReads() : 0;
            std::snprintf(line, sizeof line,
                          "[R3A] mech REC %d '%.23s' piece switch 3-wire %c reads %d (pressedLevel %d; HA-184), "
                          "read faults %d -- RECORD it pressed, released and with its cable OUT",
                          rec, s.name, mechanism::lineLetter(w.pieceSwitch), d.pieceSwitch->state() ? 1 : 0,
                          s.pieceSensor.pressedLevel ? 1 : 0, faults);
            out();
        } else {
            char piece[kPanelBytes];
            if (pieceLine(i, d, piece, sizeof piece)) {
                std::snprintf(line, sizeof line, "[R3A] mech REC %d '%.23s' %s", rec, s.name, piece);
                out();
            }
        }
        if (mech_[i].hasVerdict) {
            std::snprintf(line, sizeof line, "[R3A] mech REC %d '%.23s' last check: %s (%s): %.383s", rec, s.name,
                          findingWord(mech_[i].verdict.finding),
                          mech_[i].answered[0] != '\0' ? mech_[i].answered : "no answer needed", mech_[i].verdict.fix);
            out();
        }
    }

    // ── the log lines ─────────────────────────────────────────────────────────────────
    /// How many log lines wait.
    [[nodiscard]] std::size_t eventCount() const noexcept { return events_; }
    /// Log line `k` (oldest first).
    [[nodiscard]] const char* event(std::size_t k) const noexcept { return k < events_ ? event_[k] : ""; }
    /// Forget the drained lines.
    void clearEvents() noexcept { events_ = 0; }

private:
    struct Mech {
        bool armed = false;
        bool live = false;
        bool cut = false;
        char cutWhy[200]{};
        double commanded = 0.0;
        int overTicks[mechanism::kMaxMotorRoles]{};
        teleop::CoupledSideMonitor monitor{};
        std::optional<manipulation::StallDetector> stall;
        std::uint8_t pneuPos = 0;
        bool pneuKnown = true;  ///< its lines' readback matched a declared position (markCleared())
        FaultWatch pieceWatch;
        FaultWatch homeWatch;
        bool hasVerdict = false;              ///< a check of it ended in a verdict this entry
        mechanism::DirectionVerdict verdict;  ///< its LATEST verdict
        char answered[32]{};                  ///< what the person answered for it ("" when not asked)
    };

    [[nodiscard]] std::size_t at(std::size_t i) const {
        SHULIB_PRECONDITION(i < count_, "station 13: no mechanism is declared at this index");
        return i;
    }

    /// The declaration's stall signature, by kind (a pneumatic has none).
    static const mechanism::StallSpec* stallOf(const mechanism::MechanismSpec& s) noexcept {
        switch (s.kind) {
            case mechanism::Kind::Power: return &s.power.stall;
            case mechanism::Kind::Position: return &s.position.stall;
            case mechanism::Kind::Speed: return &s.speed.stall;
            case mechanism::Kind::Pneumatic: return nullptr;
        }
        return nullptr;
    }

    /// Queue one log line of at most kLineChars (full: the newest replaces the last). Copied, not printed: a
    /// `%s` into kLineBytes from logPieces()' 512-byte piece is GCC's -Wformat-truncation on the brain's
    /// optimised builds (the src gate's disallowed warning), though a piece never passes kLineChars.
    void pushLine(const char* line) {
        const std::size_t k = events_ < kMaxEvents ? events_++ : kMaxEvents - 1;
        const std::size_t n = std::min(std::strlen(line), kLineChars);
        std::memcpy(event_[k], line, n);
        event_[k][n] = '\0';
    }
    /// Queue a formatted log TEXT as lines of at most kLineChars, every later piece starting `cont`.
    void vEvent(const char* cont, const char* fmt, va_list args) {
        char text[kTextBytes];
        std::vsnprintf(text, sizeof text, fmt, args);
        logPieces(text, cont, kLineChars, [this](const char* l) { pushLine(l); });
    }
    [[gnu::format(printf, 2, 3)]] void event(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vEvent("[R3A] mech (cont.) ", fmt, args);
        va_end(args);
    }
    [[gnu::format(printf, 3, 4)]] void eventCont(const char* cont, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vEvent(cont, fmt, args);
        va_end(args);
    }

    /// The check at checkIdx_ ended in verdict_: counted (the page shows each NEW one) and kept as that
    /// mechanism's latest.
    void keepVerdict() {
        hasVerdict_ = true;
        ++verdicts_;
        Mech& m = mech_[checkIdx_];
        m.hasVerdict = true;
        m.verdict = verdict_;
        std::snprintf(m.answered, sizeof m.answered, "%s", answered_);
    }

    [[nodiscard]] bool anyLive() const noexcept {
        for (std::size_t i = 0; i < count_; ++i) {
            if (mech_[i].live) {
                return true;
            }
        }
        return false;
    }

    /// The declared safe state on mechanism `i` (the motors' brake mode then 0 V; a pneumatic's safe
    /// position), and it is no longer powered.
    void safeOne(std::size_t i) {
        if (hal::IMechanism* m = parts_.builtAt(i)) {
            m->applySafeState();
        }
        if (list_.specs[i].kind == mechanism::Kind::Pneumatic) {
            mech_[i].pneuPos = list_.specs[i].pneumatic.safe;  // (only a FIRED one is live: fire() made it known)
        }
        mech_[i].live = false;
        mech_[i].commanded = 0.0;
    }

    /// Forget the cut-outs' streaks — a new jog (or a check) starts its windows over, at `nowMs`.
    void resetWatch(std::size_t i, std::uint32_t nowMs) {
        watchStartMs_ = nowMs;
        Mech& m = mech_[i];
        m.monitor.reset();
        for (int& t : m.overTicks) t = 0;
        if (m.stall) {
            m.stall->reset();
        }
    }

    /// GATE 5's latch: mechanism `i` safe, named, until the station is re-entered.
    void cutMech(std::size_t i, const char* why) {
        safeOne(i);
        Mech& m = mech_[i];
        m.cut = true;
        std::snprintf(m.cutWhy, sizeof m.cutWhy, "%.190s", why);
        event("[R3A] mech CUT '%.23s': %.200s -- it stays cut until you leave and re-enter station 13",
              list_.specs[i].name, m.cutWhy);
        jogDir_ = 0;
        if (phase_ == Phase::Jogging && checkIdx_ == i) {
            finishCheck(true);
        }
    }

    /// Drive motor mechanism `i` at `volts` this tick and watch it (gate 5).
    void drive(std::size_t i, double volts, std::uint32_t nowMs) {
        hal::MotorMechanism* mm = parts_.motorAt(i);
        if (mm == nullptr) {
            return;
        }
        mm->setVoltage(units::Voltage{volts});
        mech_[i].live = true;
        mech_[i].commanded = volts;
        watch(i, volts, nowMs);
    }

    /// GATE 5's cut-outs over this tick's readings of motor mechanism `i`.
    void watch(std::size_t i, double volts, std::uint32_t nowMs) {
        hal::MotorMechanism* mm = parts_.motorAt(i);
        Mech& m = mech_[i];
        const auto motors = mm->motors();
        const std::size_t n = std::min(motors.size(), mechanism::kMaxMotorRoles);
        // a FIGHT: some of its motors against the others (every one alike is a back-drive, not a fight)
        if (n >= 2) {
            teleop::MemberSample samples[mechanism::kMaxMotorRoles];
            for (std::size_t k = 0; k < n; ++k) {
                samples[k] = teleop::MemberSample{motors[k]->velocity().value(), true};
            }
            const teleop::SideVerdict v = m.monitor.update(volts, std::span<const teleop::MemberSample>{samples, n});
            if (v.persistedCount > 0 && v.persistedCount < v.presentCount) {
                char which[48] = "";
                for (std::size_t k = 0; k < n; ++k) {
                    if (v.persistedMask & (std::uint32_t{1} << k)) {
                        char one[12];
                        std::snprintf(one, sizeof one, " %u", static_cast<unsigned>(k + 1));
                        std::strncat(which, one, sizeof which - std::strlen(which) - 1);
                    }
                }
                char why[160];
                std::snprintf(why, sizeof why, "its motor(s)%s disagree with their mates for 250 ms (a FIGHT)", which);
                cutMech(i, why);
                return;
            }
        }
        // over-current: any one motor above 2.4 A for 250 ms
        for (std::size_t k = 0; k < n; ++k) {
            m.overTicks[k] = motors[k]->current().value() > kCurrentLimitA ? m.overTicks[k] + 1 : 0;
            if (m.overTicks[k] >= kPersistTicks) {
                char why[160];
                std::snprintf(why, sizeof why, "motor %u (port %u) above %.1f A for 250 ms",
                              static_cast<unsigned>(k + 1), static_cast<unsigned>(list_.wiring[i].motors[k]),
                              kCurrentLimitA);
                cutMech(i, why);
                return;
            }
        }
        // its OWN stall signature, when it states one — its clock from this jog's start, so the window's
        // edge is exact (a difference of two small whole-millisecond values)
        if (m.stall && m.stall->update(units::Time{static_cast<double>(nowMs - watchStartMs_) / 1000.0},
                                       mm->maxCurrent(), mm->meanVelocity())) {
            const mechanism::StallSpec* st = stallOf(list_.specs[i]);
            char why[160];
            std::snprintf(why, sizeof why, "its stall signature (%.2f A or more, slower than %.1f rad/s at the motor, "
                          "for %.2f s)", st->amps, st->speedRadS, st->time);
            cutMech(i, why);
        }
    }

    /// The direction check begins (A, the mechanism at rest): the ANNOUNCEMENT — "ONLY '<name>' should move",
    /// nothing driven for kAnnounceMs; its volts fixed now.
    void startCheck(std::uint32_t nowMs) {
        const std::size_t i = selected_;
        if (parts_.motorAt(i) == nullptr) {
            return;
        }
        const mechanism::MechanismSpec& s = list_.specs[i];
        observation_ = mechanism::DirectionObservation{};
        observation_.commandVolts = voltsFor(i);
        checkIdx_ = i;
        announceStartMs_ = nowMs;
        settleSampled_ = false;  // (no hold-to-run is under way: tick() refuses A while one is live)
        hasVerdict_ = false;
        answered_[0] = '\0';
        phase_ = Phase::Announcing;
        event("[R3A] mech CHECK '%.23s': ONLY '%.23s' should move -- watch it: a + jog at %.1f V for %.2f s in "
              "%.1f s",
              s.name, s.name, observation_.commandVolts, static_cast<double>(kCheckJogMs) / 1000.0,
              static_cast<double>(kAnnounceMs) / 1000.0);
    }

    /// One tick of the announcement: nothing driven. At the start of its last kCheckJogMs, every motor's angle
    /// is taken; when it ends, a motor that turned kMinMoveRad or more since then is STILL MOVING (a coast, a
    /// sag) — the check is abandoned, nothing judged; otherwise the jog starts.
    void stepAnnounce(std::uint32_t nowMs) {
        const std::size_t i = checkIdx_;
        hal::MotorMechanism* mm = parts_.motorAt(i);
        const auto motors = mm->motors();
        const std::uint32_t since = nowMs - announceStartMs_;
        if (!settleSampled_ && since >= kAnnounceMs - kCheckJogMs) {
            settleSampled_ = true;
            for (std::size_t m = 0; m < motors.size() && m < mechanism::kMaxMotorRoles; ++m) {
                settleRad_[m] = motors[m]->position().value();
            }
        }
        if (since < kAnnounceMs) {
            return;
        }
        for (std::size_t m = 0; m < motors.size() && m < mechanism::kMaxMotorRoles; ++m) {
            const double moved = motors[m]->position().value() - settleRad_[m];
            if (std::abs(moved) >= kMinMoveRad) {
                char why[160];
                std::snprintf(why, sizeof why,
                              "still MOVING when its jog was due (motor %u turned %+.3f rad in the last %.2f s) -- "
                              "let it settle, then press A again; nothing judged",
                              static_cast<unsigned>(m + 1), moved, static_cast<double>(kCheckJogMs) / 1000.0);
                abandonCheck(why);
                return;
            }
        }
        startJog(nowMs);
    }

    /// The check's "+" jog begins: every motor's angle and the angle sensor's are its baseline.
    void startJog(std::uint32_t nowMs) {
        const std::size_t i = checkIdx_;
        hal::MotorMechanism* mm = parts_.motorAt(i);
        const auto motors = mm->motors();
        for (std::size_t m = 0; m < motors.size() && m < mechanism::kMaxMotorRoles; ++m) {
            startRad_[m] = motors[m]->position().value();
        }
        const mechanism::Devices& d = parts_.devices()[i];
        observation_.hasAngleSensor = d.angleSensor != nullptr;
        sensorStartRad_ = d.angleSensor != nullptr ? d.angleSensor->position().value() : 0.0;
        checkStartMs_ = nowMs;
        phase_ = Phase::Jogging;
        resetWatch(i, nowMs);
        event("[R3A] mech CHECK '%.23s': the + jog at %.1f V", list_.specs[i].name, observation_.commandVolts);
        stepCheck(nowMs);
    }

    /// One tick of the check's jog: drive, sample the peaks, end at its time (or at a cut).
    void stepCheck(std::uint32_t nowMs) {
        const std::size_t i = checkIdx_;
        if (nowMs - checkStartMs_ >= kCheckJogMs) {
            finishCheck(false);
            return;
        }
        drive(i, observation_.commandVolts, nowMs);
        if (phase_ != Phase::Jogging) {
            return;  // a cut ended it (cutMech -> finishCheck)
        }
        samplePeaks(i);
    }

    void samplePeaks(std::size_t i) {
        hal::MotorMechanism* mm = parts_.motorAt(i);
        const auto motors = mm->motors();
        for (std::size_t m = 0; m < motors.size() && m < mechanism::kMaxMotorRoles; ++m) {
            observation_.motorPeakAmps[m] = std::max(observation_.motorPeakAmps[m], std::abs(motors[m]->current().value()));
        }
    }

    /// The jog ends: the encoders' changes, the mechanism safe, judgeDirection — a finding at once, or the
    /// question.
    void finishCheck(bool wasCut) {
        const std::size_t i = checkIdx_;
        hal::MotorMechanism* mm = parts_.motorAt(i);
        const mechanism::MechanismSpec& s = list_.specs[i];
        if (mm != nullptr) {
            samplePeaks(i);
            const auto motors = mm->motors();
            for (std::size_t m = 0; m < motors.size() && m < mechanism::kMaxMotorRoles; ++m) {
                observation_.motorDeltaRad[m] = motors[m]->position().value() - startRad_[m];
            }
        }
        const mechanism::Devices& d = parts_.devices()[i];
        if (d.angleSensor != nullptr) {
            const double delta = d.angleSensor->position().value() - sensorStartRad_;
            observation_.sensorDeltaRad = s.angleSensor.reversed ? -delta : delta;  // as the declaration mounts it
        }
        if (mech_[i].live) {
            safeOne(i);
        }
        observation_.person = mechanism::PersonSaw::NotAsked;
        verdict_ = mechanism::judgeDirection(s, list_.wiring[i], observation_, kMinMoveRad, kDrawAmps);
        if (verdict_.finding == mechanism::DirectionFinding::AwaitingPerson) {
            phase_ = Phase::Asking;
            event("[R3A] mech CHECK '%.23s': the encoders found nothing wrong%s -- asking which way it moved", s.name,
                  wasCut ? " (the jog was CUT)" : "");
            return;
        }
        phase_ = Phase::Idle;
        keepVerdict();
        char cont[64];
        std::snprintf(cont, sizeof cont, "[R3A] mech CHECK '%.23s' (cont.) ", s.name);
        eventCont(cont, "[R3A] mech CHECK '%.23s'%s -> %s: %.383s", s.name, wasCut ? " (the jog was CUT)" : "",
                  findingWord(verdict_.finding), verdict_.fix);
    }

    /// The check ended without a verdict (the dead-man, NEXT, the exit, still moving when its jog was due).
    void abandonCheck(const char* why) {
        const std::size_t i = checkIdx_;
        if (mech_[i].live) {
            safeOne(i);
        }
        phase_ = Phase::Idle;
        event("[R3A] mech CHECK '%.23s': %s", list_.specs[i].name, why);
    }

    /// A pneumatic FIRE (after its confirm, L1 held): every line to the position's levels; then the question.
    void fire(std::size_t i, std::uint8_t target) {
        hal::PneumaticMechanism* pm = parts_.pneumaticAt(i);
        if (pm == nullptr) {
            return;
        }
        const mechanism::MechanismSpec& s = list_.specs[i];
        bool levels[mechanism::kMaxLineRoles]{};
        for (std::size_t l = 0; l < s.lineCount && l < mechanism::kMaxLineRoles; ++l) {
            levels[l] = s.pneumatic.positions[target].levels[l];
        }
        pm->setLevels(std::span<const bool>{levels, s.lineCount});
        mech_[i].pneuPos = target;
        mech_[i].pneuKnown = true;  // every line now holds a declared position's level
        mech_[i].live = true;
        pneuCommanded_ = target;
        checkIdx_ = i;
        hasVerdict_ = false;
        answered_[0] = '\0';
        phase_ = Phase::Asking;
        event("[R3A] mech FIRED '%.23s' -> '%.15s' (L1 held; letting go returns it to '%.15s')", s.name,
              s.pneumatic.positions[target].name, s.pneumatic.positions[s.pneumatic.safe].name);
    }

    /// Leaving the shown mechanism (NEXT): a check abandoned, a question dropped, it made safe.
    void leaveSelected(const char* why) {
        if (phase_ == Phase::Announcing || phase_ == Phase::Jogging) {
            abandonCheck(why);
        }
        if (phase_ == Phase::Asking) {
            cancelQuestion();
        }
        if (phase_ == Phase::Confirm) {
            phase_ = Phase::Idle;
            pendingFire_ = false;
        }
        if (selected_ < count_ && mech_[selected_].live) {
            safeOne(selected_);
        }
        jogDir_ = 0;
    }

    /// The piece sensor's line into `out`, when the mechanism declares one (true when it wrote one). The
    /// read-fault count is read AFTER the sample — the service's order: a read that failed just now counts.
    bool pieceLine(std::size_t i, const mechanism::Devices& d, char* out, std::size_t cap) {
        const mechanism::MechanismSpec& s = list_.specs[i];
        const mechanism::MechanismWiring& w = list_.wiring[i];
        if (s.pieceSensor.kind == mechanism::SensorKind::None) {
            return false;
        }
        if (d.pieceDistance != nullptr) {
            const double in = d.pieceDistance->distance().value();
            const double conf = d.pieceDistance->confidence();
            const bool moved = d.pieceFaults != nullptr && mech_[i].pieceWatch.moved(d.pieceFaults->faultedReads());
            std::snprintf(out, cap, "piece p%u: %.2fin conf %.2f -> %s (within %.2fin)",
                          static_cast<unsigned>(w.pieceSensorPort), std::clamp(in, -9999.0, 9999.0),
                          std::clamp(conf, -9.0, 9.0),
                          evidenceWord(distanceEvidence(in, conf, s.pieceSensor.seesWithinIn, moved)),
                          s.pieceSensor.seesWithinIn);
        } else if (d.pieceOptical != nullptr) {
            const double p = d.pieceOptical->proximity();
            const bool moved = d.pieceFaults != nullptr && mech_[i].pieceWatch.moved(d.pieceFaults->faultedReads());
            std::snprintf(out, cap, "piece p%u: proximity %.2f -> %s (at %.2f)",
                          static_cast<unsigned>(w.pieceSensorPort), std::clamp(p, -9.0, 9.0),
                          evidenceWord(opticalEvidence(p, s.pieceSensor.seesProximity, moved)),
                          s.pieceSensor.seesProximity);
        } else if (d.pieceSwitch != nullptr) {
            const bool level = d.pieceSwitch->state();
            const bool moved = d.pieceFaults != nullptr && mech_[i].pieceWatch.moved(d.pieceFaults->faultedReads());
            switchText(out, cap, "piece sw", mechanism::lineLetter(w.pieceSwitch), level, s.pieceSensor.pressedLevel,
                       moved);
        } else if (w.pieceSensorPort != 0) {
            std::snprintf(out, cap, "piece p%u: MISSING this boot -- it runs without one",
                          static_cast<unsigned>(w.pieceSensorPort));
        } else {
            std::snprintf(out, cap, "piece sw %c: MISSING this boot -- it runs without one",
                          mechanism::lineLetter(w.pieceSwitch));
        }
        return true;
    }

    /// Append one formatted panel line to `out` at `n` when there is room.
    [[gnu::format(printf, 4, 5)]] static void lineAt(char (*out)[kPanelBytes], std::size_t& n, std::size_t max,
                                                     const char* fmt, ...) {
        if (n >= max) {
            return;
        }
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(out[n++], kPanelBytes, fmt, args);
        va_end(args);
    }

    const mechanism::MechanismTable& list_;
    robot::MechanismParts& parts_;
    std::size_t count_ = 0;
    Mech mech_[mechanism::kMaxMechanisms]{};
    std::size_t selected_ = 0;
    bool cleared_ = false;
    double ceiling_ = kCeilingStartV;
    hal::ButtonEdge raiseEdge_;
    hal::ButtonEdge plusEdge_;
    hal::ButtonEdge minusEdge_;
    hal::ButtonEdge checkEdge_;
    bool heldWas_ = false;
    int jogDir_ = 0;
    std::uint32_t jogStartMs_ = 0;
    std::uint32_t watchStartMs_ = 0;
    Phase phase_ = Phase::Idle;
    bool pendingFire_ = false;
    std::uint8_t pendingTarget_ = 0;
    // the direction check
    std::size_t checkIdx_ = 0;
    std::uint32_t checkStartMs_ = 0;
    std::uint32_t announceStartMs_ = 0;
    bool settleSampled_ = false;
    double settleRad_[mechanism::kMaxMotorRoles]{};
    double startRad_[mechanism::kMaxMotorRoles]{};
    double sensorStartRad_ = 0.0;
    mechanism::DirectionObservation observation_{};
    std::uint8_t pneuCommanded_ = 0;
    mechanism::DirectionVerdict verdict_{};
    bool hasVerdict_ = false;
    std::uint32_t verdicts_ = 0;
    char answered_[32]{};
    // the log
    char event_[kMaxEvents][kLineBytes]{};
    std::size_t events_ = 0;
};

}  // namespace mech_station
}  // namespace shulib::bench
