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
#include <span>

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
#include "shulib/hal/pros/motor.hpp"
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

constexpr int kMaxPort = 21;      // the brain's smart ports; 21 is deliberate (see census note)
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

constexpr int kScreenLines = 19;      // small font, 480x272 panel
constexpr int kScreenCols = 54;
int g_screenLine = 0;
bool g_screenActive = false;

/// Blocks until the panel is touched and released. The release edge is what is
/// waited on, not the press: a held finger would otherwise fire every poll.
void waitForTouch() {
    while (pros::c::screen_touch_status().touch_status != pros::E_TOUCH_RELEASED) {
        pros::delay(20);
    }
    pros::delay(150);  // debounce so one tap is not read as two
}

void screenClear() {
    pros::c::screen_erase();
    g_screenLine = 0;
}

void screenEmit(const char* line) {
    if (!g_screenActive) return;
    if (g_screenLine >= kScreenLines) {
        pros::c::screen_print(pros::E_TEXT_SMALL, static_cast<std::int16_t>(kScreenLines),
                              "-- screen full: TOUCH for more (SD has it all) --");
        waitForTouch();
        screenClear();
    }
    char t[kScreenCols + 2];
    std::snprintf(t, sizeof t, "%.*s", kScreenCols, line);
    pros::c::screen_print(pros::E_TEXT_SMALL, static_cast<std::int16_t>(g_screenLine++), "%s", t);
}

/// A big fixed-position readout -- for a number somebody watches while physically
/// moving the robot, where a scrolling log would be unreadable.
void screenBig(int y, const char* text) {
    pros::c::screen_set_eraser(0x000000);
    pros::c::screen_erase_rect(0, static_cast<std::int16_t>(y), 480,
                               static_cast<std::int16_t>(y + 34));
    pros::c::screen_print_at(pros::E_TEXT_LARGE, 6, static_cast<std::int16_t>(y), "%s", text);
}

void rule(const char* title) {
    emit("");
    emitf("== %s ==========================================", title);
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

// ═══ STAGE 1 — the census. Raw registry only; constructs nothing. ═════════════
void census(pros::c::v5_device_e_t* found) {
    rule("STAGE 1  DEVICE CENSUS (raw registry, no adapters)");
    emit("port  type            our hypothesis");
    int motors = 0;
    for (int p = 1; p <= kMaxPort; ++p) {
        const pros::c::v5_device_e_t t =
            pros::c::registry_get_plugged_type(static_cast<std::uint8_t>(p));
        found[p] = t;
        if (t == pros::c::E_DEVICE_MOTOR) ++motors;
        if (t != pros::c::E_DEVICE_NONE) {
            emitf("  %2d  %-14s  %s  (code %d)", p, deviceName(t), hypothesisedSide(p),
                  static_cast<int>(t));
        }
    }
    emit("");
    emitf("motors found: %d   (hypothesis expects %u: %u left + %u right)", motors,
          static_cast<unsigned>(kLeftCount + kRightCount), static_cast<unsigned>(kLeftCount),
          static_cast<unsigned>(kRightCount));
    emit("EMPTY PORTS ARE OMITTED ABOVE. If a hypothesised port is missing from this");
    emit("list, the hypothesis is wrong -- that is a FINDING, record it and continue.");
    emit("NOTE port 21: scanned deliberately. A device here is outside PROS's own");
    emit("documented 1-20 range and is exactly what the 2026-08-13 expander report read.");
}

// ═══ STAGE 2 — IMU: raw PROS beside our canonical conversion. ════════════════
void reportImu(bool present) {
    rule("STAGE 2  IMU (HA-02/03/04/05/23/108/109/110)");
    if (!present) {
        emitf("no IMU on port %u -- hypothesis WRONG. Find it in the census above.",
              static_cast<unsigned>(kImuPort));
        return;
    }
    hal::pros::ProsClock clock{};
    try {
        hal::pros::ProsImu imu{kImuPort, math::Angle{}, clock};
        imu.calibrate();
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
        pros::c::screen_print(pros::E_TEXT_MEDIUM, 0, "ROTATE THE ROBOT");
        pros::c::screen_print(pros::E_TEXT_MEDIUM, 1, "COUNTER-CLOCKWISE (to its LEFT)");
        pros::c::screen_print(pros::E_TEXT_SMALL, 4, "heading MUST INCREASE. If it falls,");
        pros::c::screen_print(pros::E_TEXT_SMALL, 5, "HA-02's sign is wrong (mirrored turns).");
        char big[64];
        for (int i = 0; i < 100; ++i) {  // ~20 s to rotate under
            const double deg = imu.heading().degrees();
            std::snprintf(big, sizeof big, "%+8.2f deg", deg);
            screenBig(150, big);
            std::snprintf(big, sizeof big, "delta %+.2f", deg - startDeg);
            pros::c::screen_print(pros::E_TEXT_MEDIUM, 8, "%-28s", big);
            emitf("   raw=%9.3f deg   canonical=%9.4f rad = %8.3f deg",
                  pros::c::imu_get_rotation(kImuPort), imu.heading().radians(), deg);
            pros::delay(200);
        }
        g_screenActive = true;
        screenClear();
        emitf("VERDICT INPUT: heading moved %+.2f deg over the window. CCW must be POSITIVE.",
              imu.heading().degrees() - startDeg);
    } catch (const PreconditionError& e) {
        emitf("IMU ADAPTER REFUSED THE DEVICE: %s", e.what());
        emit("(that refusal IS the measurement -- record it verbatim)");
    }
}

// ═══ STAGE 3 — motors: raw beside canonical, one line each. ══════════════════
void reportMotorGroup(const char* label, const std::int8_t* ports, std::size_t n,
                      const pros::c::v5_device_e_t* found) {
    emitf("-- %s --", label);
    for (std::size_t i = 0; i < n; ++i) {
        const int p = static_cast<int>(ports[i]);
        if (found[p] != pros::c::E_DEVICE_MOTOR) {
            emitf("  port %2d: NOT A MOTOR (census says %s) -- hypothesis wrong", p,
                  deviceName(found[p]));
            continue;
        }
        try {
            hal::pros::ProsMotor m{ports[i], hal::pros::MotorGearset::Green};
            emitf("  port %2d | RAW pos=%10.2f deg temp=%5.1fC cur=%5dmA | CANON pos=%9.4f rad",
                  p, pros::c::motor_get_position(ports[i]),
                  pros::c::motor_get_temperature(ports[i]),
                  static_cast<int>(pros::c::motor_get_current_draw(ports[i])),
                  m.position().value());
        } catch (const PreconditionError& e) {
            emitf("  port %2d: ADAPTER REFUSED: %s", p, e.what());
        }
    }
}

void reportMotors(const pros::c::v5_device_e_t* found) {
    rule("STAGE 3  DRIVE MOTORS (HA-14/15/17/111 -- cartridge is a GUESS: Green)");
    emit("The gearset below is HA-15's INVENTED stand-in. If a ctor refuses, the real");
    emit("cartridge differs -- that refusal is the measurement.");
    emit("");
    reportMotorGroup("HYPOTHESISED LEFT", kLeftPorts, kLeftCount, found);
    reportMotorGroup("HYPOTHESISED RIGHT", kRightPorts, kRightCount, found);
    emit("");
    emit("** TURN ONE WHEEL BY HAND, FORWARD. ** Re-run shows which port's position");
    emit("moved and WHICH WAY. That is the port->wheel map AND the direction sign,");
    emit("with no motor ever powered. Worksheet Station 2.");
}

// ═══ STAGE 4 — battery, controller, SD card. ═════════════════════════════════
void reportPlatform() {
    rule("STAGE 4  BATTERY / CONTROLLER / SD (HA-57/103/104/107/122)");
    hal::pros::ProsBattery battery{};
    emitf("battery | RAW %d mV  %.1f%% | CANON %.3f V", static_cast<int>(pros::c::battery_get_voltage()),
          pros::c::battery_get_capacity(), battery.voltage().value());

    hal::pros::ProsController master{hal::pros::ControllerId::Master};
    const bool connected = master.isConnected();
    emitf("controller master: %s", connected ? "CONNECTED" : "NOT CONNECTED");
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
        emitf("  text log file open: %s", g_card->isOpen() ? "YES (/usd/r3a_bench.txt)" : "NO");
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

struct MenuItem {
    const char* label;
    void (*run)(pros::c::v5_device_e_t*);
};

pros::c::v5_device_e_t g_found[kMaxPort + 1] = {};

void tCensus(pros::c::v5_device_e_t* f)   { census(f); }
void tImu(pros::c::v5_device_e_t* f)      { reportImu(f[kImuPort] == pros::c::E_DEVICE_IMU); }
void tMotors(pros::c::v5_device_e_t* f)   { reportMotors(f); }
void tPlatform(pros::c::v5_device_e_t*)   { reportPlatform(); }
void tLoopRate(pros::c::v5_device_e_t*)   { measureLoopRate(); }
void tAll(pros::c::v5_device_e_t* f) {
    tCensus(f);
    tImu(f);
    tMotors(f);
    tPlatform(f);
    tLoopRate(f);
}

constexpr MenuItem kMenu[] = {
    {"1  DEVICE CENSUS",    &tCensus},
    {"2  IMU + ROTATE",     &tImu},
    {"3  MOTORS (by hand)", &tMotors},
    {"4  BATT/CTRL/SD",     &tPlatform},
    {"5  LOOP RATE",        &tLoopRate},
    {"6  RUN ALL",          &tAll},
};
constexpr int kMenuCount = static_cast<int>(sizeof kMenu / sizeof kMenu[0]);

// Two columns x three rows of touch targets. Deliberately large (232x62): this is
// operated by someone crouched over a robot, not with a mouse.
constexpr std::int16_t kBtnW = 232, kBtnH = 62, kBtnX0 = 6, kBtnY0 = 46, kGap = 6;

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
    pros::c::screen_set_eraser(0x000000);
    pros::c::screen_erase();
    pros::c::screen_set_pen(0xFFFFFF);
    pros::c::screen_print(pros::E_TEXT_MEDIUM, 0, "shulib R3a bench  -  READ-ONLY, no motion");
    pros::c::screen_print(pros::E_TEXT_SMALL, 2, "tap a test.  everything also saved to /usd/r3a_bench.txt");
    for (int i = 0; i < kMenuCount; ++i) {
        std::int16_t x0, y0, x1, y1;
        buttonBox(i, x0, y0, x1, y1);
        pros::c::screen_set_pen(0x1E5AA8);
        pros::c::screen_fill_rect(x0, y0, x1, y1);
        pros::c::screen_set_pen(0xFFFFFF);
        pros::c::screen_draw_rect(x0, y0, x1, y1);
        pros::c::screen_print_at(pros::E_TEXT_MEDIUM, static_cast<std::int16_t>(x0 + 10),
                                 static_cast<std::int16_t>(y0 + 20), "%s", kMenu[i].label);
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
    static hal::pros::ProsBlockSink card{"r3a_bench.txt"};
    g_card = &card;

    emit("");
    emit("################################################################");
    emit("#  shulib R3a  BENCH VALIDATION  --  READ-ONLY, COMMANDS NO MOTION");
    emit("################################################################");
    emitf("build hash : %s", diag::compiledBuildHash());
    emit("robot      : TANK BENCH BOT (the measured one) -- NOT the invented X-drive");
    emitf("hypothesis : LEFT %d/%d/%d/%d   RIGHT %d/%d/%d/%d   IMU %u   no GPS, no pods",
          kLeftPorts[0], kLeftPorts[1], kLeftPorts[2], kLeftPorts[3], kRightPorts[0],
          kRightPorts[1], kRightPorts[2], kRightPorts[3], static_cast<unsigned>(kImuPort));
    emit("!! SIDE LABELS ARE A HYPOTHESIS, NOT A CONFIGURATION (R3a-PROGRESS S10.3).");

    // The census runs once up front so every other test knows what exists; it is
    // also re-runnable from the menu.
    census(g_found);
    card.flush();

    // ── the menu loop. Runs forever; the bencher drives it with no laptop. ──
    for (;;) {
        drawMenu();
        int choice = -1;
        while (choice < 0) {
            const pros::screen_touch_status_s_t t = pros::c::screen_touch_status();
            if (t.touch_status == pros::E_TOUCH_RELEASED) {
                choice = hitTest(t.x, t.y);
                if (choice >= 0) pros::delay(150);  // debounce
            }
            pros::delay(20);
        }

        screenClear();
        g_screenActive = true;
        emit("");
        emitf(">>>>>> %s", kMenu[choice].label);
        kMenu[choice].run(g_found);
        card.flush();

        g_screenActive = false;
        pros::c::screen_print(pros::E_TEXT_MEDIUM, kScreenLines, "TOUCH TO RETURN TO MENU");
        waitForTouch();
    }
}

}  // namespace shulib::bench
