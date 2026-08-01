// Tests for diag/term_sink.hpp — the §18.3 terminal formatter. Output is asserted as
// EXACT golden strings through the injected FakeCharSink; "readable, column-aligned"
// is a byte-level claim here, not an eyeballed one. What each targets:
//  * GOLDEN SHAPES: the leveled-message and per-tick line formats (incl. the §18.3
//    details that are easy to get subtly wrong: Info carries no level tag; [WARN][SEQ]
//    butt together with no space; emit() stamps from the RECORD's t, not the clock).
//  * THE UGLY CASES the brief names: NaN, ±Inf, huge and tiny magnitudes, an empty
//    subsystem tag, and messages long/hostile enough to threaten the framing.
//  * UNAVOIDABLE SANITIZATION: control bytes cannot break the one-line framing — the
//    legacy escapeJSONString failure mode, designed against by construction.

#include "doctest.h"

#include <algorithm>
#include <limits>
#include <string>

#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::DebugRecord;
using shulib::diag::FaultCode;
using shulib::diag::TermSink;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeCharSink;
using shulib::hal::fake::FakeClock;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
namespace units = shulib::units;

namespace {
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDegToRad = Angle::kPi / 180.0;

struct Rig {
    FakeCharSink out;
    FakeClock clock;
    TermSink sink{clock, out};
};

/// The synthetic "MoveToPose#7 mid-run" tick used across the golden tests.
DebugRecord midRunTick() {
    DebugRecord r;
    r.t = units::Time{12.34};
    r.activeCommandId = 7;
    r.activeCommandState = 1;
    r.targetPose = Pose2d{units::Length{24.0}, units::Length{36.0}, Angle::degrees(90.0)};
    r.errorX = units::Length{0.4};
    r.errorY = units::Length{0.2};
    r.errorHeading = units::AngleDim{0.3 * kDegToRad};
    r.commanded = ChassisSpeeds{units::Velocity{18.0}, units::Velocity{4.0},
                                units::AngularVelocity{0.1}};
    r.quality = 0.91;
    return r;
}
}  // namespace

TEST_CASE("TermSink.log: golden lines per level — Info bare, others [LEVEL][TAG] butted") {
    Rig rig;
    rig.clock.set(units::Time{12.41});
    rig.sink.log(LogLevel::Info, "EKF", "gps fix ACCEPT mahal 1.9");
    CHECK(rig.out.text() == "[t=  12.41] [EKF] gps fix ACCEPT mahal 1.9\n");

    rig.out.clear();
    rig.clock.set(units::Time{12.51});
    rig.sink.log(LogLevel::Warn, "SEQ", "intakeUntilCapture retry 1/3 (optical=none)");
    // NO space between [WARN] and [SEQ] — the §18.3 shape, easy to regress.
    CHECK(rig.out.text() == "[t=  12.51] [WARN][SEQ] intakeUntilCapture retry 1/3 (optical=none)\n");

    rig.out.clear();
    rig.clock.set(units::Time{100.0});
    rig.sink.log(LogLevel::Error, "IMU", "lost mid-run");
    CHECK(rig.out.text() == "[t= 100.00] [ERROR][IMU] lost mid-run\n");

    rig.out.clear();
    rig.sink.log(LogLevel::Debug, "MOT", "tick detail");
    CHECK(rig.out.text() == "[t= 100.00] [DEBUG][MOT] tick detail\n");

    rig.out.clear();
    rig.sink.log(LogLevel::Trace, "MOT", "hot-path detail");
    CHECK(rig.out.text() == "[t= 100.00] [TRACE][MOT] hot-path detail\n");
}

TEST_CASE("TermSink.log: the timestamp comes from the injected clock and advances") {
    Rig rig;
    rig.sink.log(LogLevel::Info, "A", "first");
    rig.clock.advance(units::Time{1.5});
    rig.sink.log(LogLevel::Info, "A", "second");
    CHECK(rig.out.text() == "[t=   0.00] [A] first\n[t=   1.50] [A] second\n");
}

TEST_CASE("TermSink.emit: golden per-tick line (the §18.3 target shape, active command)") {
    Rig rig;
    rig.clock.set(units::Time{999.0});  // must be IGNORED: emit stamps from the record
    rig.sink.emit(midRunTick());
    CHECK(rig.out.text() ==
          "[t=  12.34] [MOT] cmd#7▸1 tgt(  24.0,  36.0,  90.0°) "
          "err(  0.40\",  0.20\",  0.3°) v(  18.0,   4.0, 0.10) q=0.91\n");
}

TEST_CASE("TermSink.emit: an idle tick (no active command) is tagged [LOC]") {
    Rig rig;
    DebugRecord r;
    r.t = units::Time{0.05};
    rig.sink.emit(r);
    CHECK(rig.out.text() ==
          "[t=   0.05] [LOC] idle tgt(   0.0,   0.0,   0.0°) "
          "err(  0.00\",  0.00\",  0.0°) v(   0.0,   0.0, 0.00) q=0.00\n");
}

TEST_CASE("TermSink.emit: state flags and the fault code appear if and only if set") {
    Rig rig;
    DebugRecord r = midRunTick();
    r.deadReckoning = true;
    r.strafeFallbackActive = true;
    r.clampedThisTick = true;
    r.fault = FaultCode::GpsGateReject;
    rig.sink.emit(r);
    CHECK(rig.out.text() ==
          "[t=  12.34] [MOT] cmd#7▸1 tgt(  24.0,  36.0,  90.0°) "
          "err(  0.40\",  0.20\",  0.3°) v(  18.0,   4.0, 0.10) q=0.91"
          " DR SFB CLMP flt=GPS_GATE_REJECT\n");
}

TEST_CASE("TermSink.emit: NaN and ±Inf render as deterministic right-aligned tokens — "
          "never libc's locale-dependent forms") {
    Rig rig;
    DebugRecord r;
    r.t = units::Time{3.0};
    r.activeCommandId = 9;
    r.activeCommandState = 2;
    r.targetPose = Pose2d{units::Length{kNan}, units::Length{kInf}, Angle{}};
    r.errorX = units::Length{-kInf};
    r.errorY = units::Length{kNan};
    r.commanded = ChassisSpeeds{units::Velocity{kNan}, units::Velocity{0.0},
                                units::AngularVelocity{0.0}};
    r.quality = kNan;
    rig.sink.emit(r);
    CHECK(rig.out.text() ==
          "[t=   3.00] [MOT] cmd#9▸2 tgt(   NaN,  +Inf,   0.0°) "
          "err(  -Inf\",   NaN\",  0.0°) v(   NaN,   0.0, 0.00) q= NaN\n");
}

TEST_CASE("TermSink.emit: pathological magnitudes compact to %.3g instead of exploding "
          "the column to 300 digits; tiny values stay in-column") {
    Rig rig;
    DebugRecord r;
    r.targetPose = Pose2d{units::Length{1e300}, units::Length{-1e300}, Angle{}};
    r.errorX = units::Length{1e-9};        // tiny → renders as an in-column 0.00
    r.errorY = units::Length{123456.78};   // wide-but-sane → plain, column widens slightly
    rig.sink.emit(r);
    CHECK(rig.out.text() ==
          "[t=   0.00] [LOC] idle tgt(1e+300,-1e+300,   0.0°) "
          "err(  0.00\",123456.78\",  0.0°) v(   0.0,   0.0, 0.00) q=0.00\n");
}

TEST_CASE("TermSink.emit: the F3 ±180° tie-break is visible — a -180° heading renders "
          "as +180.0, never -180.0") {
    Rig rig;
    DebugRecord r;
    r.targetPose = Pose2d{units::Length{0.0}, units::Length{0.0}, Angle::degrees(-180.0)};
    rig.sink.emit(r);
    CHECK(rig.out.text() ==
          "[t=   0.00] [LOC] idle tgt(   0.0,   0.0, 180.0°) "
          "err(  0.00\",  0.00\",  0.0°) v(   0.0,   0.0, 0.00) q=0.00\n");
}

TEST_CASE("TermSink.log: an empty subsystem tag renders as [] and cannot derail the line") {
    Rig rig;
    rig.sink.log(LogLevel::Info, "", "bare");
    CHECK(rig.out.text() == "[t=   0.00] [] bare\n");
}

TEST_CASE("TermSink.log: control bytes are sanitized to '?' — an embedded newline or ESC "
          "sequence cannot break the one-line framing or the terminal") {
    Rig rig;
    rig.sink.log(LogLevel::Info, "SYS", "a\nb\tc");
    CHECK(rig.out.text() == "[t=   0.00] [SYS] a?b?c\n");

    rig.out.clear();
    rig.sink.log(LogLevel::Info, "SYS", "\x1b[31mred\x7f");
    CHECK(rig.out.text() == "[t=   0.00] [SYS] ?[31mred?\n");

    // The framing proof: however hostile the message, exactly ONE '\n' per call.
    rig.out.clear();
    rig.sink.log(LogLevel::Info, "S\nY", "x\n\n\ny");
    const std::string& text = rig.out.text();
    CHECK(std::count(text.begin(), text.end(), '\n') == 1);
}

TEST_CASE("TermSink.log: over-long messages truncate with … at the cap; exactly-at-cap "
          "does not") {
    Rig rig;
    const std::string prefix = "[t=   0.00] [LONG] ";

    rig.sink.log(LogLevel::Info, "LONG", std::string(250, 'A'));
    CHECK(rig.out.text() == prefix + std::string(200, 'A') + "…\n");

    rig.out.clear();
    rig.sink.log(LogLevel::Info, "LONG", std::string(200, 'A'));  // exactly the cap
    CHECK(rig.out.text() == prefix + std::string(200, 'A') + "\n");
}

TEST_CASE("TermSink.log: truncation never splits a multi-byte UTF-8 sequence") {
    Rig rig;
    // 199 ASCII bytes + a 2-byte '°' straddling the 200-byte cap: a naive byte cut
    // would emit half a code point (mojibake); the cut must back off to byte 199.
    rig.sink.log(LogLevel::Info, "UTF", std::string(199, 'A') + "°");
    CHECK(rig.out.text() == "[t=   0.00] [UTF] " + std::string(199, 'A') + "…\n");
}

TEST_CASE("TermSink.log: an over-long subsystem tag truncates with … inside the brackets") {
    Rig rig;
    rig.sink.log(LogLevel::Info, "ABCDEFGHIJKLMNOPQRST", "x");  // 20 > the 16-byte cap
    CHECK(rig.out.text() == "[t=   0.00] [ABCDEFGHIJKLMNOP…] x\n");
}

TEST_CASE("TermSink: wantsRecord() is true — overridden as a pair with emit(), per the "
          "seam contract") {
    Rig rig;
    const shulib::hal::ITelemetrySink& sink = rig.sink;
    CHECK(sink.wantsRecord());
}

TEST_CASE("TermSink: a synthetic tick stream renders the §18.3 target shape end to end "
          "(per-tick lines + leveled messages — the A1 DoD)") {
    Rig rig;

    rig.sink.emit(midRunTick());

    rig.clock.set(units::Time{12.41});
    rig.sink.log(LogLevel::Info, "EKF", "gps fix ACCEPT resid(0.8,0.5) mahal 1.9");

    DebugRecord settling = midRunTick();
    settling.t = units::Time{12.50};
    settling.activeCommandState = 2;
    settling.errorX = units::Length{0.02};
    settling.errorY = units::Length{0.01};
    settling.errorHeading = units::AngleDim{0.1 * kDegToRad};
    settling.commanded = ChassisSpeeds{units::Velocity{2.0}, units::Velocity{0.5},
                                       units::AngularVelocity{0.01}};
    settling.quality = 0.95;
    rig.sink.emit(settling);

    rig.clock.set(units::Time{12.51});
    rig.sink.log(LogLevel::Warn, "SEQ", "intakeUntilCapture retry 1/3 (optical=none)");

    CHECK(rig.out.text() ==
          "[t=  12.34] [MOT] cmd#7▸1 tgt(  24.0,  36.0,  90.0°) "
          "err(  0.40\",  0.20\",  0.3°) v(  18.0,   4.0, 0.10) q=0.91\n"
          "[t=  12.41] [EKF] gps fix ACCEPT resid(0.8,0.5) mahal 1.9\n"
          "[t=  12.50] [MOT] cmd#7▸2 tgt(  24.0,  36.0,  90.0°) "
          "err(  0.02\",  0.01\",  0.1°) v(   2.0,   0.5, 0.01) q=0.95\n"
          "[t=  12.51] [WARN][SEQ] intakeUntilCapture retry 1/3 (optical=none)\n");
}
