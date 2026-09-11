// drive_program.cpp — "shulib Drive": robot two drives from the sticks, through the
// library's hal/pros ADAPTERS, with dead-port tolerance (chunk R3b Part 0b, 2026-09-10).
//
// ═══ WHAT THIS IS ══════════════════════════════════════════════════════════════════
// The bench tester's DRIVE station (src/bench_r3a.cpp, station 10) with the menu, the
// sign-capture dependency, the voltage ceiling and the dead-man REMOVED, and dead-port
// tolerance ADDED. Built as its OWN program in its own slot (`make ROBOT=tank
// PROGRAM=drive`; `pros upload --slot 1 --name "shulib Drive"`), because the operator picks
// a program by name from the brain's slot list and "Drive" must mean drive.
//
// ═══ WHAT IT IS NOT ════════════════════════════════════════════════════════════════
// The library's motion stack. No Chassis::drive(), no scheduler, no odometry, no heading.
// Ten ProsMotors, one ProsController, open-loop volts: the SAME adapters the library will
// drive through, exercised for real, and nothing above them. The banner says so at boot.
// The library has still never driven a robot.
//
// ═══ THE DEGRADATION RULE (team lead, 2026-09-10) ══════════════════════════════════
// "Some ports might die during comp; 1–2 dead ports shouldn't stop driving, just let it be
// a warning that gets logged." So: every ProsMotor is constructed inside its own try; a port
// that refuses is a WARNING (log, panel, LCD) and the program drives on without it, under a
// PURE policy (shulib/teleop/drivetrain_degradation.hpp, host-tested): refuse only when a
// side has fewer than three answering motors or more than two are dead in total. A member
// can also go ABSENT at runtime (shulib/teleop/coupled_side_monitor.hpp's
// MemberAbsenceDetector): logged once, dropped from the fight detector, still commanded
// (harmless), shown grey. A dead port is NEVER a silent zero.
//
// ═══ THE TWO PROTECTIONS, NON-FATAL ════════════════════════════════════════════════
// Kept from the station and made 1-second cuts that log, count and RE-ARM — a match must not
// end on a transient, and a fight must still be visible:
//   * FIGHT: the station's gate-5 logic, through the shared pure CoupledSideMonitor — a
//     member opposite its side's command (above 0.5 rad/s) or under 25 % of the side's fastest,
//     while the side is commanded above 1 V and moving above 1 rad/s, for 250 ms.
//   * OVER-CURRENT: any member above 2.4 A for 250 ms.
// On either: BOTH sides to 0 V for 1 s, the port(s) and reason logged, the cut counted, then
// re-arm (the monitors' streaks reset, so the same fight has to persist again for the full
// 250 ms before it cuts again).
//
// ═══ SIGNS LIVE IN ONE PLACE ═══════════════════════════════════════════════════════
// The table's signed entry (src/chassis_table.hpp: LEFT -11 +12 -13 +14 -15 | RIGHT +20 -19
// +18 -17 +16, measured 2026-09-10) goes to ProsMotor AS TYPED; PROS applies the reversal
// once, in the adapter. Nothing in this file negates anything.
//
// ═══ EVERY EXIT LEAVES THE MOTORS AT 0 V, COAST ════════════════════════════════════
// An RAII guard runs stopAll() on every C++ exit path (return, throw). A FIELD DISABLE is
// NOT a C++ exit — PROS deletes the opcontrol task, and a deleted task runs no destructors
// (VEXos itself cuts motor output on DISABLED; that is the mechanism that actually stops the
// robot then). So this loop polls competition_is_disabled() every tick and EXITS cleanly on
// it, which does run the guard before the task is torn down. Stated so nobody believes the
// guard covers a kill.

#include "drive_program.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "chassis_table.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/battery.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/line_display.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/teleop/drivetrain_degradation.hpp"
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

/// Compiled-in build stamp (the tester's reasoning, verbatim: the hash names the COMMIT, the
/// stamp names the BUILD, and only the stamp answers "did my upload land?").
constexpr const char* kBuildStamp = __DATE__ " " __TIME__;

/// Full battery volts to the sticks. Chunk T2 owns curves, slew and per-driver limits; until
/// then the driver gets the whole pack and the deadband-only mapping (HA-112).
constexpr double kMaxDriveV = 12.0;

constexpr int kTickMs = 10;
constexpr int kPersistTicks = 25;      // 250 ms: the over-current window (the monitor's own is
                                       // the same number, pinned below)
constexpr double kCurrentLimitA = 2.4;  // INVENTED for a first run (R3b Session 2)
constexpr int kCutTicks = 100;          // the 1 s non-fatal cut
constexpr int kPanelEveryTicks = 10;    // 100 ms
constexpr int kLcdEveryTicks = 20;      // 200 ms = the 5 Hz ceiling; writes only on change
constexpr int kMaxCutsLogged = 50;      // after this many, cuts are counted but no longer
                                        // logged per port (a permanent fight must not fill
                                        // the card)

static_assert(teleop::SideMonitorConfig{}.persistTicks == kPersistTicks);
static_assert(teleop::SideMonitorConfig{}.commandFloorV == 1.0);
static_assert(teleop::SideMonitorConfig{}.movingFloorRadS == 1.0);
static_assert(teleop::SideMonitorConfig{}.oppositeFloorRadS == 0.5);
static_assert(teleop::SideMonitorConfig{}.nearZeroFraction == 0.25);

// ── the brain screen: 480x240 usable (the tester measured it; lv_conf.h agrees) ──────
constexpr std::int16_t kUsableW = 480;
constexpr std::int16_t kUsableH = 240;
constexpr std::uint32_t kColBg = 0x101418, kColBar = 0x1E5AA8, kColText = 0xF0F0F0,
                        kColDim = 0x9AA4AE, kColGood = 0x39C36E, kColWarn = 0xE8B23A,
                        kColBad = 0xE2564A, kColGrey = 0x5A6470, kColSub = 0xC8D8F0;

// ── Output: USB serial always, SD card when one is installed (T5's no-card rule). ────
hal::pros::ProsBlockSink* g_card = nullptr;

void emit(const char* line) {
    std::printf("%s\n", line);
    std::fflush(stdout);
    if (g_card != nullptr && g_card->isOpen()) {
        // Return values discarded on purpose: a full/absent card must not stop a drive
        // program (isOpen() already reported the card's state, once, at boot).
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

void flushCard() {
    if (g_card != nullptr) (void)g_card->flush();
}

/// One drive motor, as this program sees it.
struct Member {
    int port = 0;
    int side = 0;                                // -1 LEFT, +1 RIGHT
    std::int8_t signedPort = 0;                  // THE one place a sign lives (from the table)
    std::optional<hal::pros::ProsMotor> motor;   // nullopt = the adapter refused at boot
    bool absent = false;                         // refused at boot, or died at runtime
    const char* absentWhy = "";
    teleop::MemberAbsenceDetector absence{};
    int overTicks = 0;
    int disagreeTicks = 0;                       // mirrored from the side's monitor, for colour
    double v = 0.0, amps = 0.0, temp = 0.0;
    double maxAmps = 0.0, maxTemp = 0.0;
};

/// 0 V + Coast on every constructed member (absent-at-runtime ones included: harmless).
void stopAll(Member* m, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        if (m[i].motor) {
            m[i].motor->setVoltage(units::Voltage{0.0});
            m[i].motor->setBrakeMode(hal::BrakeMode::Coast);
        }
    }
}

/// The all-stop on every C++ exit path (header: a task kill is not one of them).
struct AllStopGuard {
    Member* m;
    const std::size_t* n;
    ~AllStopGuard() { stopAll(m, *n); }
};

/// The one-line list of dead ports, "dead: 18 19" or "all N ok".
void deadPortsLine(const Member* m, std::size_t n, char* buf, std::size_t cap) {
    buf[0] = '\0';
    int dead = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!m[i].absent) continue;
        char one[8];
        std::snprintf(one, sizeof one, "%s%d", dead ? " " : "dead: ", m[i].port);
        std::strncat(buf, one, cap - std::strlen(buf) - 1);
        ++dead;
    }
    if (dead == 0) std::snprintf(buf, cap, "all %u ok", static_cast<unsigned>(n));
}

/// The degradation policy over the members as they stand right now.
teleop::DegradationVerdict evaluate(const Member* m, std::size_t n) {
    teleop::SideCount left{}, right{};
    for (std::size_t i = 0; i < n; ++i) {
        teleop::SideCount& s = m[i].side < 0 ? left : right;
        ++s.expected;
        if (!m[i].absent) ++s.answering;
    }
    return teleop::evaluateDegradation(left, right);
}

/// Paint a full-screen refusal and say it on serial/SD. The program then returns: nothing
/// is powered, and the screen keeps the reason until the next program starts.
void paintRefusal(const char* headline, const char* why, const char* fix) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColBad);
    pros::c::screen_fill_rect(0, 0, kUsableW, 34);
    pros::c::screen_set_eraser(kColBad);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 6, "shulib DRIVE -- REFUSED");
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_set_pen(kColBad);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 48, "%.34s", headline);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 84, "%.54s", why);
    pros::c::screen_set_pen(kColDim);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 110, "%.54s", fix);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 140, "Nothing is powered. BUILD %s", kBuildStamp);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 160, "Bench Tests (slot 3) is the diagnosis tool.");
    emitf("REFUSED: %s -- %s", headline, why);
    emitf("  %s", fix);
    flushCard();
}

/// The fixed header bar: name, build stamp, SD state.
void drawHeader(bool logging) {
    pros::c::screen_set_pen(kColBar);
    pros::c::screen_fill_rect(0, 0, kUsableW, 32);
    pros::c::screen_set_eraser(kColBar);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 2, "shulib DRIVE");
    pros::c::screen_set_pen(kColSub);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 20, "adapters, NOT the motion stack");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 250, 20, "BUILD %s", kBuildStamp);
    pros::c::screen_set_pen(logging ? kColGood : kColBad);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 250, 2, logging ? "SD: logging" : "SD: OFF");
    pros::c::screen_set_eraser(kColBg);
}

/// One row of the panel, erased then printed.
void rowText(std::int16_t x, std::int16_t y, std::int16_t w, std::uint32_t colour,
             const char* text) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(x, y, static_cast<std::int16_t>(x + w),
                               static_cast<std::int16_t>(y + 16));
    pros::c::screen_set_pen(colour);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, x, y, "%s", text);
}

}  // namespace

void runDrive() {
    // The SD text log, opened first so the banner itself is captured. One file per boot
    // (the sink owns its FILE*). Static: opcontrol() can be entered more than once per boot
    // (field enable/disable cycles) and the file must not be re-truncated each time.
    static hal::pros::ProsBlockSink card{"drive_log.txt"};
    g_card = &card;
    static int entries = 0;
    ++entries;

    // ── 1. the banner: serial + SD ────────────────────────────────────────────────
    emit("");
    emit("################################################################");
    emit("#  shulib DRIVE  --  robot two drives from the sticks through the hal/pros ADAPTERS");
    emit("#  NOT the library's motion stack (no Chassis, no odometry, no heading -- R3b Parts 1-3).");
    emit("#  The library has still not driven a robot; this program drives one through its adapters.");
    emit("################################################################");
    emitf("BUILD STAMP: %s   <-- must match the build you just uploaded", kBuildStamp);
    // compiledBuildHash() is a string_view: NEVER hand it to %s (the 2026-08-18 data abort).
    const std::string_view hash = diag::compiledBuildHash();
    if (hash.empty()) {
        emit("build hash : [ERROR] MISSING -- the build injected no hash (S18.5)");
    } else {
        emitf("build hash : %.*s", static_cast<int>(hash.size()), hash.data());
    }
    emitf("robot      : %s", kChassis.robot);
    {
        char l[80], r[80];
        portsToString(kChassis.left, kChassis.leftCount, kChassis.signsMeasured, l, sizeof l);
        portsToString(kChassis.right, kChassis.rightCount, kChassis.signsMeasured, r, sizeof r);
        emitf("table      : LEFT %s | RIGHT %s | cartridge %s | %s | signs %s", l, r,
              cartridgeWord(kChassis.cartridge), kChassis.measured ? "MEASURED" : "NOT measured",
              kChassis.signsMeasured ? "IN THE TABLE (measured)" : "NOT in the table");
        emitf("provenance : %s", kChassis.provenance);
    }
    if (kChassis.imuPort != 0) {
        emitf("IMU        : port %u -- in the table, NOT used by this program (no heading, no "
              "odometry: those are R3b Parts 1-3)", static_cast<unsigned>(kChassis.imuPort));
    } else {
        emit("IMU        : UNSET in the table (not used by this program)");
    }
    emitf("sd logging : %s", card.isOpen() ? "ON -> /usd/drive_log.txt (one file per boot)"
                                           : "OFF -- no card at boot; serial only (T5: never stop for a missing card)");
    emitf("opcontrol entry #%d this boot; competition status 0x%02x", entries,
          static_cast<unsigned>(pros::c::competition_get_status()));
    emitf("policy     : %.0f V to the sticks (T2 owns curves/slew/limits); no dead-man; fight cut "
          "and over-current cut are 1 s, non-fatal, counted; refuse if a side < 3 answering or "
          "> 2 dead in total", kMaxDriveV);
    flushCard();

    // ── 2. the table must be SIGNED, set, and consistent, or nothing is powered ──────
    char why[96];
    if (!kChassis.signsMeasured) {
        paintRefusal("chassis table has NO SIGNS",
                     "only a table with MEASURED signs can drive (robot two's has them)",
                     "this build is for a robot whose signs were never typed in: use Bench Tests");
        return;
    }
    if (describeMissing(kChassis, why, sizeof why, true)) {
        paintRefusal("chassis table UNSET", why, "fill src/chassis_table.hpp, rebuild, upload");
        return;
    }
    if (!tableConsistent(kChassis, why, sizeof why)) {
        paintRefusal("chassis table CONTRADICTS ITSELF", why,
                     "fix src/chassis_table.hpp, rebuild, upload");
        return;
    }

    // ── 3. construct one ProsMotor per table port, each inside its own try ─────────
    Member members[kMaxPort];
    std::size_t n = 0;
    AllStopGuard guard{members, &n};  // every C++ exit from here on: 0 V, coast
    const hal::pros::MotorGearset gearset =
        kChassis.cartridge == Cartridge::Red     ? hal::pros::MotorGearset::Red
        : kChassis.cartridge == Cartridge::Green ? hal::pros::MotorGearset::Green
                                                 : hal::pros::MotorGearset::Blue;
    emitf("CARTRIDGE BELIEF: %s -- ProsMotor WRITES this to every motor and reads it back; a "
          "wrong belief passes the read-back (volts unaffected; rad/s scale by it)",
          cartridgeWord(kChassis.cartridge));
    for (int p = 1; p <= kMaxPort; ++p) {
        const int side = tableSideOf(kChassis, p);
        if (side == 0) continue;
        Member& m = members[n];
        m.port = p;
        m.side = side;
        m.signedPort = tableSignedPort(kChassis, p);  // AS TYPED; never negated here
        try {
            m.motor.emplace(m.signedPort, gearset);
            emitf("port %2d (%s, signed %+3d): ProsMotor constructed, gearset written and read back",
                  p, side < 0 ? "LEFT " : "RIGHT", static_cast<int>(m.signedPort));
        } catch (const PreconditionError& e) {
            // THE WARNING, not a fault: logged with the port and the side, shown on the panel
            // and the LCD below, and the program drives on without it (policy permitting).
            m.absent = true;
            m.absentWhy = "refused at boot (no device, or not a motor)";
            emitf("WARNING: port %2d (%s, signed %+3d) DEAD AT BOOT -- adapter refused: %s", p,
                  side < 0 ? "LEFT " : "RIGHT", static_cast<int>(m.signedPort), e.what());
        }
        ++n;
    }
    stopAll(members, n);  // 0 V + Coast is the declared starting state, not an inherited one

    // ── 4. the degradation policy, once at boot ───────────────────────────────────
    teleop::DegradationVerdict policy = evaluate(members, n);
    char dead[64];
    deadPortsLine(members, n, dead, sizeof dead);
    emitf("degradation: %s (left dead %d, right dead %d, total %d) -- %s",
          policy.verdict == teleop::DriveVerdict::Drive            ? "DRIVE"
          : policy.verdict == teleop::DriveVerdict::DriveDegraded  ? "DRIVE_DEGRADED"
                                                                   : "REFUSE",
          policy.leftDead, policy.rightDead, policy.totalDead, dead);
    if (policy.verdict == teleop::DriveVerdict::Refuse) {
        char headline[64];
        std::snprintf(headline, sizeof headline, "%d of %u motors dead", policy.totalDead,
                      static_cast<unsigned>(n));
        paintRefusal(headline, policy.reason, dead);
        return;  // the guard leaves every constructed motor at 0 V, coast
    }
    if (policy.verdict == teleop::DriveVerdict::DriveDegraded) {
        emitf("WARNING: driving DEGRADED -- %s", dead);
    }
    flushCard();

    // ── 5. the driver's hands, the LCD, the battery ────────────────────────────────
    hal::pros::ProsController master{hal::pros::ControllerId::Master};
    hal::pros::ProsLineDisplay lcd{};
    hal::pros::ProsBattery battery{};
    emitf("controller : %s (a disconnected controller commands 0 V)",
          master.isConnected() ? "CONNECTED" : "NOT CONNECTED");
    emitf("battery    : %.2f V", battery.voltage().value());
    emit("DRIVING. Left stick Y = forward, right stick X = yaw. Every state change is logged.");
    flushCard();

    // ── the panel: fixed layout, two columns (LEFT members | RIGHT members) ──────────
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    drawHeader(card.isOpen());
    constexpr std::int16_t kRowsY = 74, kRowPitch = 22, kColW = 236;
    constexpr std::int16_t kWarnY = 190, kCutY = 208;
    std::size_t sideIdx[2][kMaxPort];
    std::size_t sideN[2] = {0, 0};
    for (std::size_t i = 0; i < n; ++i) {
        const int s = members[i].side < 0 ? 0 : 1;
        sideIdx[s][sideN[s]++] = i;
    }
    teleop::CoupledSideMonitor monitor[2];

    // ── loop state ────────────────────────────────────────────────────────────────
    bool refused = false;          // the policy said REFUSE at runtime: 0 V until disabled
    int cutTicksLeft = 0;          // > 0 while the 1 s cut is in force
    int cutCount = 0;
    char cutWhy[120] = "";
    char lcdRow[3][24] = {"", "", ""};
    char line[80];
    int tick = 0;
    int drivenTicks = 0;
    double vL = 0.0, vR = 0.0;
    const char* exitWhy = "field DISABLED (competition_is_disabled)";

    for (;; ++tick) {
        // A field DISABLE exits this loop cleanly so the guard runs before PROS kills the
        // task (header). Polled every tick: 10 ms is the most the robot can drive past it.
        if (pros::c::competition_is_disabled()) break;

        // ── sticks -> the ONE shared mapping -> the ONE shared per-side arithmetic ──
        const bool connected = master.isConnected();
        teleop::StickInput sticks{.connected = connected};
        if (connected) {
            sticks.leftY = master.axis(hal::ControllerAxis::LeftY);
            sticks.leftX = master.axis(hal::ControllerAxis::LeftX);
            sticks.rightX = master.axis(hal::ControllerAxis::RightX);
        }
        const teleop::DriveRequest req = teleop::mapSticks(sticks);  // all-zero when disconnected
        const teleop::SideVolts sv = teleop::tankSideVolts(req, kMaxDriveV);
        vL = sv.left;
        vR = sv.right;

        // ── the cut in force, and its re-arm ─────────────────────────────────────
        if (cutTicksLeft > 0) {
            --cutTicksLeft;
            vL = vR = 0.0;
            if (cutTicksLeft == 0) {
                // RE-ARM: the streaks start over, so the same fight must persist again for the
                // full 250 ms before it cuts again. Logged as a state change.
                monitor[0].reset();
                monitor[1].reset();
                for (std::size_t i = 0; i < n; ++i) members[i].overTicks = 0;
                emitf("re-armed after cut #%d (1 s at 0 V)", cutCount);
                flushCard();
            }
        }
        if (refused) vL = vR = 0.0;

        // ── command: the same volts to every constructed member of a side ───────────
        for (std::size_t i = 0; i < n; ++i) {
            if (members[i].motor) {
                members[i].motor->setVoltage(units::Voltage{members[i].side < 0 ? vL : vR});
            }
        }
        if (vL != 0.0 || vR != 0.0) ++drivenTicks;

        // ── read every constructed member ─────────────────────────────────────────
        for (std::size_t i = 0; i < n; ++i) {
            Member& m = members[i];
            if (!m.motor) continue;
            m.v = m.motor->velocity().value();
            m.amps = m.motor->current().value();
            m.temp = m.motor->temperature();
            m.maxAmps = std::max(m.maxAmps, m.amps);
            m.maxTemp = std::max(m.maxTemp, m.temp);
        }

        // ── runtime absence: a port that died since boot ───────────────────────────
        bool newlyAbsent = false;
        for (int s = 0; s < 2; ++s) {
            const double cmd = s == 0 ? vL : vR;
            for (std::size_t j = 0; j < sideN[s]; ++j) {
                Member& m = members[sideIdx[s][j]];
                if (!m.motor || m.absent) continue;
                double matesFastest = 0.0;
                for (std::size_t k = 0; k < sideN[s]; ++k) {
                    const Member& o = members[sideIdx[s][k]];
                    if (k == j || o.absent || !o.motor) continue;
                    matesFastest = std::max(matesFastest, std::abs(o.v));
                }
                if (m.absence.update(m.motor->faultedReads(), m.v, cmd, matesFastest)) {
                    m.absent = true;
                    m.absentWhy = m.absence.reason();
                    newlyAbsent = true;
                    emitf("WARNING: port %2d (%s) went ABSENT at runtime: %s -- dropped from the "
                          "fight detector, still commanded (harmless)", m.port,
                          m.side < 0 ? "LEFT" : "RIGHT", m.absentWhy);
                }
            }
        }
        if (newlyAbsent) {
            policy = evaluate(members, n);
            deadPortsLine(members, n, dead, sizeof dead);
            emitf("degradation re-evaluated: %s (left dead %d, right dead %d, total %d) -- %s",
                  policy.verdict == teleop::DriveVerdict::Drive            ? "DRIVE"
                  : policy.verdict == teleop::DriveVerdict::DriveDegraded  ? "DRIVE_DEGRADED"
                                                                           : "REFUSE",
                  policy.leftDead, policy.rightDead, policy.totalDead, dead);
            if (policy.verdict == teleop::DriveVerdict::Refuse && !refused) {
                refused = true;
                stopAll(members, n);
                std::snprintf(cutWhy, sizeof cutWhy, "REFUSED: %s", policy.reason);
                emitf("%s -- 0 V until the field disables; then fix the robot", cutWhy);
            }
            flushCard();
        }

        // ── the fight cut (shared monitor) and the over-current cut ────────────────
        if (cutTicksLeft == 0 && !refused) {
            bool cutNow = false;
            for (int s = 0; s < 2 && !cutNow; ++s) {
                const double cmd = s == 0 ? vL : vR;
                teleop::MemberSample samples[kMaxPort];
                for (std::size_t j = 0; j < sideN[s]; ++j) {
                    const Member& m = members[sideIdx[s][j]];
                    samples[j] = teleop::MemberSample{m.v, !m.absent && m.motor.has_value()};
                }
                const teleop::SideVerdict verdict = monitor[s].update(
                    cmd, std::span<const teleop::MemberSample>{samples, sideN[s]});
                char names[48] = "";
                for (std::size_t j = 0; j < sideN[s]; ++j) {
                    Member& m = members[sideIdx[s][j]];
                    m.disagreeTicks = monitor[s].disagreeTicks(j);
                    if (verdict.persistedMask & (std::uint32_t{1} << j)) {
                        char one[8];
                        std::snprintf(one, sizeof one, " %d", m.port);
                        std::strncat(names, one, sizeof names - std::strlen(names) - 1);
                    }
                }
                if (verdict.persistedCount > 0) {
                    cutNow = true;
                    if (verdict.persistedCount == verdict.presentCount) {
                        std::snprintf(cutWhy, sizeof cutWhy,
                                      "CUT: whole %s side OPPOSITE/stalled vs command (ports%s)",
                                      s == 0 ? "LEFT" : "RIGHT", names);
                    } else {
                        std::snprintf(cutWhy, sizeof cutWhy, "CUT: FIGHT on %s -- port(s)%s",
                                      s == 0 ? "LEFT" : "RIGHT", names);
                    }
                }
            }
            for (std::size_t i = 0; i < n && !cutNow; ++i) {
                Member& m = members[i];
                if (!m.motor || m.absent) {
                    m.overTicks = 0;
                    continue;
                }
                m.overTicks = m.amps > kCurrentLimitA ? m.overTicks + 1 : 0;
                if (m.overTicks >= kPersistTicks) {
                    cutNow = true;
                    std::snprintf(cutWhy, sizeof cutWhy, "CUT: port %d over %.1f A for 250 ms",
                                  m.port, kCurrentLimitA);
                }
            }
            if (cutNow) {
                stopAll(members, n);
                cutTicksLeft = kCutTicks;
                ++cutCount;
                if (cutCount <= kMaxCutsLogged) {
                    emitf("%s -- both sides 0 V for 1 s, cut #%d, then re-arm", cutWhy, cutCount);
                    for (std::size_t i = 0; i < n; ++i) {
                        const Member& m = members[i];
                        emitf("  at the cut: port %2d signed %+3d %s v=%+7.2f rad/s %5.2f A %3.0f C",
                              m.port, static_cast<int>(m.signedPort),
                              m.absent ? "ABSENT" : "      ", m.v, m.amps, m.temp);
                    }
                    if (cutCount == kMaxCutsLogged) {
                        emit("  (further cuts are counted on the panel but no longer logged per port)");
                    }
                    flushCard();
                }
            }
        }

        // ── the panel (100 ms) ─────────────────────────────────────────────────────
        if (tick % kPanelEveryTicks == 0) {
            const char* mode = refused ? "REFUSED" : cutTicksLeft > 0 ? "CUT"
                               : policy.verdict == teleop::DriveVerdict::DriveDegraded ? "DEGRADED"
                                                                                       : "DRIVE";
            std::snprintf(line, sizeof line, "%-8s %4.1fV  B%5.2fV  L %+5.1fV  R %+5.1fV  %s", mode,
                          kMaxDriveV, battery.voltage().value(), vL, vR,
                          connected ? "" : "NO CTRL");
            pros::c::screen_set_eraser(kColBg);
            pros::c::screen_erase_rect(0, 36, kUsableW, 70);
            pros::c::screen_set_pen(refused || cutTicksLeft > 0 ? kColBad
                                    : !connected                ? kColWarn
                                    : (vL != 0.0 || vR != 0.0)  ? kColGood
                                                                : kColText);
            pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 40, "%s", line);
            for (int s = 0; s < 2; ++s) {
                for (std::size_t j = 0; j < sideN[s]; ++j) {
                    const Member& m = members[sideIdx[s][j]];
                    const std::uint32_t colour =
                        m.absent                                                    ? kColGrey
                        : (cutTicksLeft > 0 && (m.disagreeTicks >= kPersistTicks
                                                || m.overTicks >= kPersistTicks))   ? kColBad
                        : (m.disagreeTicks > 0 || m.overTicks > 0)                  ? kColWarn
                        : std::abs(m.v) > 1.0                                       ? kColGood
                                                                                    : kColDim;
                    if (m.absent) {
                        std::snprintf(line, sizeof line, "%s%+3d  ABSENT", s == 0 ? "L" : "R",
                                      static_cast<int>(m.signedPort));
                    } else {
                        std::snprintf(line, sizeof line, "%s%+3d %+6.1fr/s %4.2fA %3.0fC%s",
                                      s == 0 ? "L" : "R", static_cast<int>(m.signedPort), m.v,
                                      m.amps, m.temp,
                                      m.disagreeTicks >= kPersistTicks ? " FIGHT"
                                      : m.overTicks >= kPersistTicks   ? " AMPS"
                                                                       : "");
                    }
                    rowText(static_cast<std::int16_t>(8 + s * kColW),
                            static_cast<std::int16_t>(kRowsY + static_cast<int>(j) * kRowPitch),
                            static_cast<std::int16_t>(kColW - 12), colour, line);
                }
            }
            std::snprintf(line, sizeof line, "WARNINGS: %s", dead);
            rowText(8, kWarnY, static_cast<std::int16_t>(kUsableW - 16),
                    policy.totalDead > 0 ? kColWarn : kColDim, line);
            if (cutCount > 0 || refused) {
                std::snprintf(line, sizeof line, "cuts: %d  last: %.60s", cutCount, cutWhy);
            } else {
                std::snprintf(line, sizeof line, "cuts: 0  (fight or >2.4 A = 1 s at 0 V, then re-arm)");
            }
            rowText(8, kCutY, static_cast<std::int16_t>(kUsableW - 16),
                    refused ? kColBad : cutTicksLeft > 0 ? kColBad : cutCount > 0 ? kColWarn : kColDim,
                    line);
        }

        // ── the controller LCD (200 ms, on change only) ───────────────────────────
        if (tick % kLcdEveryTicks == 0) {
            char want[3][24];
            std::snprintf(want[0], sizeof want[0], "%s %2.0fV B%4.1fV",
                          refused ? "REFUSD" : cutTicksLeft > 0 ? "CUT  "
                          : policy.verdict == teleop::DriveVerdict::DriveDegraded ? "DEGRD" : "DRIVE",
                          kMaxDriveV, battery.voltage().value());
            std::snprintf(want[1], sizeof want[1], "%.19s", dead);
            std::snprintf(want[2], sizeof want[2], "%.19s",
                          refused ? "REFUSED see brain"
                          : cutCount > 0 ? cutWhy + 5 /* past "CUT: " */
                          : !connected   ? "no controller 0V"
                                         : "ok");
            for (int row = 0; row < 3; ++row) {
                if (std::strcmp(want[row], lcdRow[row]) != 0) {
                    lcd.setLine(row, want[row]);
                    std::memcpy(lcdRow[row], want[row], sizeof lcdRow[row]);
                }
            }
        }

        pros::delay(kTickMs);
    }

    // ── exit: every motor 0 V, coast (the guard repeats it), and the summary ────────
    stopAll(members, n);
    lcd.setLine(0, "DRIVE stopped 0V");
    lcd.setLine(1, dead);
    lcd.setLine(2, cutCount > 0 ? "cuts: see log" : "no cuts");
    emitf("DRIVE ended: %s", exitWhy);
    emitf("  %d ticks (%.1f s); motors driven %.1f s; cuts %d; %s", tick, tick * kTickMs / 1000.0,
          drivenTicks * kTickMs / 1000.0, cutCount, dead);
    if (cutCount > 0) emitf("  last cut: %s", cutWhy);
    emit("  port side signed   max A   max C   state");
    for (std::size_t i = 0; i < n; ++i) {
        const Member& m = members[i];
        emitf("  %2d    %s    %+3d    %5.2f   %4.0f   %s", m.port, m.side < 0 ? "L" : "R",
              static_cast<int>(m.signedPort), m.maxAmps, m.maxTemp,
              m.absent ? m.absentWhy : "ok");
    }
    emit("  Everything above is in /usd/drive_log.txt when a card was present at boot.");
    flushCard();
}

}  // namespace shulib::bench
