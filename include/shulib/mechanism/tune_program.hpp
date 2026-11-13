#pragma once
//
// mechanism/tune_program.hpp — "shulib Tune", the program that tunes a robot's MECHANISMS on the brain
// (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's D7 (a) and D8 (a), with its re-check amendments
// #3, #7, #8, #10 and #14). PROS-free: hal/pros/tune.hpp is its thin shell on a V5.
//
// ── Why its own program, and why in the library ─────────────────────────────────────
// A mechanism's numbers are found ON THE ROBOT (the team lead, 2026-10-29: "things need to be able to
// be tuned"): a string cascade's gains, top speed, homing push and current cap. The match programs run
// EXACTLY the robot file's numbers, so tuning lives in a program of its own, uploaded beside them
// (`make ROBOT=<any robot with mechanisms> PROGRAM=tune`, slot 7) — never a page of shulib Drive, where
// one stray tap in practice would leave the robot driving on numbers nobody wrote down (D7 (b)). It is
// LIBRARY code over the HAL's seams — the standard Calibrate's pattern (calibration/calibrate_program.hpp)
// — so every robot, robot one's cascade and a robot made in the panel later, runs the same program:
// nothing in THIS file names a robot. (The BUILD names each robot once, as it does for every program:
// a robot that registers a mechanism table adds its ROBOT value to the Makefile's MECHANISM_ROBOTS, a
// `<robot> && SHULIB_PROGRAM_TUNE` branch to src/build_selection.hpp's one chain and its define to that
// file's Tune guard, and a `<robot>-tune` build to tools/src_build_gate.py's tables — the src build gate
// fails a robot with mechanisms that is missing from any of them: its step 0, or the guard's #error.)
//
// ── What it builds, and what it never builds ────────────────────────────────────────
// ONLY the robot's mechanisms: each one's config projected from the robot file
// (mechanism::toMechanismConfig, kept in this object beside the parts it builds), robot::MechanismParts
// over the provider, ONE MechanismService bound over a LiveTuning's COPY of the table (live_tuning.hpp).
// It NEVER constructs a chassis, a drive motor or a Robot (landmine 17: a robot on blocks is not
// assumed), and NOTHING is made before stage 1 of the clear confirm (READY): making a 3-wire line
// drives it to its start level (HA-119 — robot two's claw moves at boot).
//
// ── Its safety rules: the Auton Tester's (re-check #7) ──────────────────────────────
//   * EVERY mechanism is released to its declared safe state (MechanismService::leaveDriverControl():
//     a running step CANCELLED, every motor mechanism inert in its safe brake mode, a pneumatic keeping
//     its level — HA-157) on ANY of the controller's twelve buttons, on the controller DISCONNECTING, on
//     a field or switch being plugged in, on the page's STOP, and when VEXos reports the robot DISABLED
//     or autonomous starting (then run() returns: PROS calls opcontrol() again later, and this object,
//     in the shell's static storage, carries on). A stop is applied BEFORE the tick that follows the
//     read that saw it — never a tick later.
//   * READY (which makes the mechanisms — a pneumatic moves), TEST, HOME, STRING CHECK and the stick are
//     REFUSED, each with one sentence, unless a controller is connected (it carries the stop), no
//     button is held, VEXos reports no field or switch, and (but READY) the mechanism is built (and,
//     for a position TEST, homed).
//   * The stick starts a jog only on a FRESH push (centred, then pushed): one held over a change of
//     mechanism, a stop, GO or another page moves nothing until it is let go and pushed again.
//   * STAGE 2 of the clear confirm: a mechanism's FIRST motion in the session waits for GO.
//   * A TEST move's time limit is the plan's own time at the live speed and acceleration, × 2, plus the
//     settle time and 0.5 s, at most 10 s (tunePositionBound()); the service reports TIMED_OUT there.
//   * A change is applied only BETWEEN steps: a running test greys every button but STOP, and live
//     tuning itself refuses a change while its mechanism's step runs (LiveTuning, rule 3).
//
// ── The current cap stays in the motor (re-check #8, D12 (b); A4 register HA-220) ─────
// A cap changed live is WRITTEN into each motor and a motor keeps a limit after the program ends, and
// the next program, stating no cap, writes nothing. So this program puts back EVERY limit it changed —
// the robot file's cap, or the motor's 2.5 A default when the file states none — on RESET (LiveTuning
// re-derives it), on LEAVING that mechanism's page (‹ ›) and on EXIT (restoreCap(): LiveTuning::apply of
// the file's cap; on EXIT, a RESET when that is refused). The tuned cap then leaves the live numbers, so
// restoreCap() first COPIES the mechanism's numbers to the log when no COPY carries them yet; a page
// whose cap cannot go back (a live signature at or above the file's cap) is not left — the page says
// "lower it, or RESET". ONLY the shown mechanism carries a live cap: LiveTuning's UNDO is global, so an
// UNDO whose next undo would change ANOTHER mechanism's cap (the cap put back when its page was left, or
// a RESET of a raised cap) is checked BEFORE it is made and waits — nothing changes, the page says "UNDO
// it on its page" (undoWaitsOn(); every change is tracked to know whose each undo is). Stopping the program from the brain runs no
// code: the CLEAR page and the TUNE page's safety line (any mechanism's cap changed) say "EXIT before you
// stop".
//
// ── A TEST move and its report (re-check #3, #7) ────────────────────────────────────
// TEST A / TEST B: a position mechanism moves to its first and last preset (with fewer than two, a
// quarter and three quarters of its live range); a flywheel spins up to its first and last preset (with
// fewer, half and nine tenths of its top speed — INVENTED); a power mechanism runs at +maxVolts / −maxVolts
// for kTunePowerTestSeconds (INVENTED). Each is an ordinary service step. After every tick the program
// records one sample (the clock, where it is or how fast, the most current any of its motors draws);
// when a position or speed move SUCCEEDS it records ONE MORE SECOND (its hold, or its spin at speed)
// with every button but STOP greyed. Then tuneReportFrom() works the report out of the samples — the
// time to the verdict, the farthest past the target, the settle (the first sample inside "reached" to
// the verdict), the most current, the final error, the HOLD current (the mean over that second) — and
// a WARNING when the stall signature is not above that hold (an ordinary hold could trip it: V1) or the
// cap not above the peak (the cap limited the move: landmine 18). A flywheel is stopped after its
// second; a position mechanism goes on holding. The report is on the REPORT page and in the log (tag
// TUNEPG, the terminal and the SD copy).
//
// ── COPY ────────────────────────────────────────────────────────────────────────────
// LiveTuning::copyBlock() into the provider's sink: on the brain the terminal AND /usd/tune_log.txt,
// APPENDED across boots (re-check #14 — hal/pros/tune.hpp). The COPY button writes ONE block for EVERY
// mechanism changed (D9 (a)); EXIT writes one for every mechanism changed since its last COPY, so the
// live numbers — which are not kept — are in the log before the program can be stopped. Until the
// panel's import exists (TUNE1-B2, after the panel), a student copies the block's lines into the robot
// file BY HAND.
//
// ── The stick (a delta from the brief, logged: D8 (a) says "a student jogs it to the right height and
// reads the number off", and nothing else on the page can) ───────────────────────────
// The LEFT stick's up/down moves the shown POSITION or POWER mechanism by hold-to-run (the service's
// jog() / drive()) at up to kTuneJogMostVolts (INVENTED) beyond kTuneJogDeadband, and holds where it is
// (or stops) when let go. Never a button: every button is a STOP. A position mechanism before homing
// moves only by its declaration's `.unhomedVolts` (the service's rule) — 0 means not at all.
//
// The live numbers never persist: leaving the program, or a reboot, runs the robot file's numbers again
// (every program but this one always does). NOTHING HERE HAS RUN ON A ROBOT: its tests drive it over
// FakeDeviceProvider and the host test plant (its constants invented — HA-183).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <optional>
#include <span>
#include <string_view>

#include "shulib/config/robot_config.hpp"
#include "shulib/control/trapezoid_profile.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/brain_screen.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/device_provider.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/live_tuning.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/tune_panel.hpp"
#include "shulib/mechanism/until.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/robot/mechanism_parts.hpp"
#include "shulib/ui/touch_panel.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::mechanism {

// ═══ The program's numbers ═════════════════════════════════════════════════════════════

/// The competition status bit "disabled" (PROS's competition_get_status(); hal/pros/tune.hpp checks
/// the three against PROS's own values at compile time). Hex: the API reference tool reads `<<` as a
/// bracket.
inline constexpr std::uint8_t kTuneCompetitionDisabled = 0x01;
/// The competition status bit "autonomous".
inline constexpr std::uint8_t kTuneCompetitionAutonomous = 0x02;
/// The competition status bit "a field or switch is connected".
inline constexpr std::uint8_t kTuneCompetitionConnected = 0x04;
/// A TEST move's time limit at most, seconds (the brief's re-check #7).
inline constexpr double kTuneTestMostSeconds = 10.0;
/// A TEST move's time limit: the plan's time times this (re-check #7) ...
inline constexpr double kTuneTestPlanFactor = 2.0;
/// ... plus the settle time plus this, seconds (re-check #7).
inline constexpr double kTuneTestExtraSeconds = 0.5;
/// How long after a position or speed move SUCCEEDS the program records its hold (re-check #3: "the
/// HOLD current over the second after it settles"), seconds.
inline constexpr double kTuneHoldSeconds = 1.0;
/// HOME's time limit, seconds — INVENTED (a library choice: the TEST cap reused).
inline constexpr double kTuneHomeSeconds = 10.0;
/// A power mechanism's TEST run, seconds — INVENTED (a library choice).
inline constexpr double kTunePowerTestSeconds = 0.5;
/// A flywheel's TEST spin-up time limit, seconds — INVENTED (a library choice; at most the TEST cap).
inline constexpr double kTuneSpeedTestSeconds = 5.0;
/// The stick's most volts on the shown mechanism — INVENTED (a library choice, half of full power).
inline constexpr double kTuneJogMostVolts = 6.0;
/// The stick's deadband, of full deflection — INVENTED (a resting thumb or drift moves nothing).
inline constexpr double kTuneJogDeadband = 0.15;
/// The most samples one test keeps: a 10 s move and its second at 10 ms, with room.
inline constexpr std::size_t kTuneMaxSamples = 1200;
/// The log tag of the program's own lines (the COPY block's is LiveTuning's "TUNE").
inline constexpr const char* kTuneTag = "TUNEPG";

// ═══ The pure parts: the targets, the time limit, the report ═════════════════════════════

/// The longest test-target name, NUL included (a preset's name is at most 15 characters).
inline constexpr std::size_t kTuneTargetNameBytes = sizeof(Preset::name);

/// TEST A's and TEST B's targets for one mechanism (header): a position's or a flywheel's first and
/// last preset, or (fewer than two) a quarter and three quarters of the range / half and nine tenths
/// of the top speed; a power mechanism's +maxVolts and −maxVolts. A pneumatic has none (both 0).
struct TuneTargets {
    /// TEST A's target (the mechanism's unit; volts for a power mechanism).
    double a = 0.0;
    /// TEST B's target.
    double b = 0.0;
    /// TEST A's name ("floor", "1/4 range", "forward").
    char aName[kTuneTargetNameBytes]{};
    /// TEST B's name.
    char bName[kTuneTargetNameBytes]{};
};

/// The targets of `s` (TuneTargets says how).
[[nodiscard]] inline TuneTargets tuneTargets(const MechanismSpec& s) noexcept {
    TuneTargets t{};
    const auto name = [](char* out, const char* text) {
        std::snprintf(out, kTuneTargetNameBytes, "%.*s", static_cast<int>(kTuneTargetNameBytes - 1), text);
    };
    switch (s.kind) {
        case Kind::Position: {
            const PositionSpec& p = s.position;
            if (p.presetCount >= 2) {
                t.a = p.presets[0].value;
                t.b = p.presets[p.presetCount - 1].value;
                name(t.aName, p.presets[0].name);
                name(t.bName, p.presets[p.presetCount - 1].name);
            } else {
                t.a = p.minPosition + 0.25 * (p.maxPosition - p.minPosition);
                t.b = p.minPosition + 0.75 * (p.maxPosition - p.minPosition);
                name(t.aName, "1/4 range");
                name(t.bName, "3/4 range");
            }
            return t;
        }
        case Kind::Speed: {
            const SpeedSpec& q = s.speed;
            if (q.presetCount >= 2) {
                t.a = q.presets[0].value;
                t.b = q.presets[q.presetCount - 1].value;
                name(t.aName, q.presets[0].name);
                name(t.bName, q.presets[q.presetCount - 1].name);
            } else {
                t.a = 0.5 * q.maxRpm;
                t.b = 0.9 * q.maxRpm;
                name(t.aName, "half speed");
                name(t.bName, "9/10 speed");
            }
            return t;
        }
        case Kind::Power:
            t.a = s.power.maxVolts;
            t.b = -s.power.maxVolts;
            name(t.aName, "forward");
            name(t.bName, "backward");
            return t;
        case Kind::Pneumatic:
            return t;
    }
    return t;
}

/// A position TEST move's TIME LIMIT (re-check #7): the plan's own time at the LIVE speed and
/// acceleration — the trapezoid the service plans for that move, in output degrees from `fromDeg` (where
/// it is now) to `target` clamped inside the live travel limits, by the service's one scale (the
/// steepest units per degree, detail::unitsPerOutputDeg) — × kTuneTestPlanFactor, + the settle time +
/// kTuneTestExtraSeconds, at most kTuneTestMostSeconds. PRECONDITION (the profile's): the live top speed
/// and acceleration are finite and above 0 (live tuning never sets them to 0).
[[nodiscard]] inline double tunePositionBound(const MechanismSpec& s, double fromDeg, double target) {
    const PositionSpec& p = s.position;
    const double scale = 1.0 / std::max(detail::unitsPerOutputDeg(p), 1e-9);
    const double a = positionToAngleDeg(p, p.minPosition);
    const double b = positionToAngleDeg(p, p.maxPosition);
    const double toDeg =
        std::clamp(positionToAngleDeg(p, std::clamp(target, p.minPosition, p.maxPosition)), std::min(a, b),
                   std::max(a, b));
    const control::TrapezoidProfile plan{toDeg - fromDeg,
                                         control::ProfileConstraints{p.maxVelocity * scale, p.maxAcceleration * scale}};
    return std::min(kTuneTestMostSeconds,
                    kTuneTestPlanFactor * plan.duration() + p.settleTime + kTuneTestExtraSeconds);
}

/// One sample of a test: after a tick, the clock, where it is (its unit) or how fast (output rpm), and
/// the most current any of its motors draws.
struct TuneSample {
    /// The clock, s.
    double t = 0.0;
    /// Where it is, or how fast.
    double x = 0.0;
    /// The most current any of its motors draws, A.
    double amps = 0.0;
};

/// What a test was, for tuneReportFrom().
struct TuneMoveFacts {
    /// Which test.
    TuneTest test = TuneTest::None;
    /// The mechanism's kind.
    Kind kind = Kind::Position;
    /// Its unit word.
    const char* unit = "";
    /// The target.
    double target = 0.0;
    /// Where it was (or how fast) when the test began.
    double start = 0.0;
    /// "Reached" (a position's within, units) or the at-speed band (a flywheel's, rpm).
    double within = 0.0;
    /// The verdict.
    manipulation::MechanismOutcome outcome = manipulation::MechanismOutcome::Running;
    /// Seconds from the command to the verdict.
    double seconds = 0.0;
    /// The time limit.
    double bound = 0.0;
    /// Its stall signature's amps (0: none).
    double stallAmps = 0.0;
    /// Its current cap (0: none).
    double capAmps = 0.0;
    /// What stopped it (CANCELLED), else "".
    const char* stoppedBy = "";
};

/// THE REPORT (header) worked out of the samples: `move` from the first tick after the command to the
/// tick that reached the verdict (inclusive), `hold` the second after a SUCCEEDED position or speed
/// move. The direction of travel is the sign of (target − start) (+ when equal). Overshoot: the most any
/// move sample is past the target in that direction, at least 0 (n/a for a power run). Settle: from the
/// first move sample inside `within` of the target to the last move sample — SUCCEEDED only. Peak: the
/// most `amps` over the move. Final error: the last move sample minus the target. Hold: the mean `amps`
/// over `hold` (SUCCEEDED and at least one hold sample; else n/a). The warnings: a stated stall
/// signature not above the hold; a stated cap not above the peak.
[[nodiscard]] inline TuneReport tuneReportFrom(std::span<const TuneSample> move, std::span<const TuneSample> hold,
                                               const TuneMoveFacts& f) noexcept {
    TuneReport r{};
    r.test = f.test;
    r.kind = f.kind;
    r.unit = f.unit;
    r.target = f.target;
    r.verdict = manipulation::mechanismOutcomeName(f.outcome);
    r.seconds = f.seconds;
    r.bound = f.bound;
    r.stallAmps = f.stallAmps;
    r.capAmps = f.capAmps;
    r.stoppedBy = f.stoppedBy != nullptr ? f.stoppedBy : "";
    for (const TuneSample& s : move) {
        r.peakAmps = std::max(r.peakAmps, s.amps);
    }
    const bool succeeded = f.outcome == manipulation::MechanismOutcome::Succeeded;
    if (f.kind != Kind::Power && !move.empty()) {
        const double dir = f.target - f.start < 0.0 ? -1.0 : 1.0;
        double past = 0.0;
        for (const TuneSample& s : move) {
            past = std::max(past, dir * (s.x - f.target));
        }
        r.overshoot = past;
        r.finalError = move.back().x - f.target;
        r.hasFinalError = true;
        if (succeeded) {
            for (const TuneSample& s : move) {
                if (std::abs(s.x - f.target) <= f.within) {
                    r.settle = move.back().t - s.t;
                    break;
                }
            }
        }
        if (succeeded && !hold.empty()) {
            double sum = 0.0;
            for (const TuneSample& s : hold) {
                sum += s.amps;
            }
            r.holdAmps = sum / static_cast<double>(hold.size());
        }
    }
    r.warnStallNotAboveHold = f.stallAmps > 0.0 && r.holdAmps >= 0.0 && f.stallAmps <= r.holdAmps;
    r.warnCapNotAbovePeak = f.capAmps > 0.0 && f.capAmps <= r.peakAmps;
    return r;
}

/// The label of a TEST button: "A 22", "B -1.5", or a power run's "A FWD" / "B REV" — at most 7
/// characters (a target is shown clamped to ±9999).
inline void tuneTestLabel(char* out, std::size_t cap, char which, Kind k, double target) noexcept {
    if (k == Kind::Power) {
        std::snprintf(out, cap, "%c %s", which, target < 0.0 ? "REV" : "FWD");
        return;
    }
    const double v = std::isfinite(target) ? std::clamp(target, -9999.0, 9999.0) : 0.0;
    if (std::abs(v) >= 100.0) {
        std::snprintf(out, cap, "%c %.0f", which, v);
    } else {
        std::snprintf(out, cap, "%c %.3g", which, v);
    }
}

// ═══ What the program needs from its platform ═══════════════════════════════════════════

/// The one thing the program reads that is on no HAL seam: the competition status byte. On the brain:
/// hal::pros::ProsTuneIo (hal/pros/tune.hpp); on the host: a test's fake.
class TuneIo {
public:
    /// Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted.
    virtual ~TuneIo() = default;
    /// Default-constructible.
    TuneIo() = default;
    /// Copyable at the base.
    TuneIo(const TuneIo&) = default;
    /// Movable at the base.
    TuneIo(TuneIo&&) = default;
    /// Copy-assignable at the base.
    TuneIo& operator=(const TuneIo&) = default;
    /// Move-assignable at the base.
    TuneIo& operator=(TuneIo&&) = default;
    /// The competition status byte (the kTuneCompetition* bits).
    [[nodiscard]] virtual std::uint8_t competitionStatus() = 0;
};

/// The devices the program runs on, all BORROWED: each must outlive the program.
struct TuneDevices {
    hal::IDeviceProvider& devices;  ///< the mechanisms' devices by port; the clock; the pacer; the sink
    hal::IBrainScreen& screen;      ///< the brain's screen and touch
    hal::IController& controller;   ///< the master controller: every button a STOP; the stick
    TuneIo& io;                     ///< the competition status
};

/// The screen's colours.
namespace tune_colours {
/// The ground.
inline constexpr std::uint32_t kGround = 0x000000;
/// Ordinary text.
inline constexpr std::uint32_t kText = 0xFFFFFF;
/// Grey (a quiet line, a greyed button's label).
inline constexpr std::uint32_t kGrey = 0x808080;
/// A greyed button's fill.
inline constexpr std::uint32_t kGreyFill = 0x303030;
/// Blue (a header, a button).
inline constexpr std::uint32_t kBlue = 0x1E5AA8;
/// Light blue (a title's text).
inline constexpr std::uint32_t kLightBlue = 0x7FB2FF;
/// Amber (attention).
inline constexpr std::uint32_t kAmber = 0xF0C000;
/// Green (good, GO).
inline constexpr std::uint32_t kGreen = 0x30C030;
/// Red (stopped, refused, STOP).
inline constexpr std::uint32_t kRed = 0xE03030;
}  // namespace tune_colours

/// "SHULIB TUNE" (header). Construct it — in STATIC storage (PROS deletes the opcontrol task on a mode
/// change; the service's slots point into this object) — over the robot's mechanism table and its
/// devices; call bootBanner() from initialize() and run() from opcontrol(). Not copyable or movable.
class TuneProgram {
public:
    /// `declared` is the robot file's table (a constant that outlives the program); `d` and `buildStamp`
    /// are borrowed and must outlive it. Constructs NOTHING on the robot (no device is asked for).
    TuneProgram(const MechanismTable& declared, TuneDevices d, const char* buildStamp)
        : d_{d}, stamp_{buildStamp != nullptr ? buildStamp : ""}, tuning_{declared},
          faults_{d.devices.telemetry(), d.devices.clock()}, svc_{d.devices.pacer()} {
        for (std::size_t i = 0; i < tuning_.count(); ++i) {
            std::memcpy(static_cast<void*>(&copiedAs_[i]), &tuning_.spec(i), sizeof(MechanismSpec));
        }
    }
    /// Not copyable: it borrows the robot's devices, and the service's slots point into it.
    TuneProgram(const TuneProgram&) = delete;
    /// Not copy-assignable.
    TuneProgram& operator=(const TuneProgram&) = delete;
    /// Not movable.
    TuneProgram(TuneProgram&&) = delete;
    /// Not move-assignable.
    TuneProgram& operator=(TuneProgram&&) = delete;
    /// The service ends any slot still holding its claim (its destructor); nothing else to release.
    ~TuneProgram() = default;

    /// initialize(): the boot lines (constructs nothing, powers nothing). `label` is the build's beacon
    /// (or null), `statusWords` the competition status at boot in words (or null).
    void bootBanner(const char* label, const char* statusWords) {
        char line[220];
        std::snprintf(line, sizeof line,
                      "booted: shulib TUNE build [%s] -- live tuning of this robot's %u mechanism%s; NOTHING is "
                      "made until READY on the screen",
                      label != nullptr ? label : "", static_cast<unsigned>(tuning_.count()),
                      tuning_.count() == 1 ? "" : "s");
        say(hal::LogLevel::Info, line);
        std::snprintf(line, sizeof line, "BUILD STAMP: %.40s   <-- must match the build you just uploaded", stamp_);
        say(hal::LogLevel::Info, line);
        const std::string_view hash = diag::compiledBuildHash();
        if (hash.empty()) {
            say(hal::LogLevel::Warn, "build hash : MISSING -- the build injected no hash");
        } else {
            std::snprintf(line, sizeof line, "build hash : %.*s", static_cast<int>(std::min<std::size_t>(hash.size(), 40)),
                          hash.data());
            say(hal::LogLevel::Info, line);
        }
        std::snprintf(line, sizeof line, "competition status at boot: %.80s (Tune moves nothing with a field or "
                      "switch plugged in)",
                      statusWords != nullptr ? statusWords : "(not given)");
        say(hal::LogLevel::Info, line);
        say(hal::LogLevel::Info,
            "this program NEVER makes the drive; ANY controller button stops every mechanism; the numbers changed "
            "here are not kept -- COPY them into the robot file");
    }

    /// opcontrol(): the loop, one pass every control tick, until VEXos reports the robot DISABLED or
    /// autonomous starting — every mechanism released first — or forever. Re-entered by PROS after a
    /// disable: the object carries on (the made mechanisms, the live numbers, the page).
    void run() {
        enter();
        while (step()) {
        }
    }

    /// ONE pass of the loop (header): read the status, the controller and the screen; stop on any abort
    /// cause; act on a touch or the stick; paint; tick the service once (it then advances the world);
    /// record a test's sample. False when driver control ended (everything released): the caller
    /// returns. Exposed for the host tests, which drive the loop pass by pass.
    bool step() {
        try {
            return stepImpl();
        } catch (const std::exception& e) {
            programFault(e.what());
        } catch (...) {
            programFault("an unknown exception");
        }
        return true;
    }

    /// What run()/step() do when (re-)entered: the service's driver-control entry (a step left running by a
    /// deleted task CANCELLED, every motor mechanism made safe), a full repaint. Called by run().
    void enter() {
        if (made_) {
            svc_.enterDriverControl();
        }
        clearTest();
        jogging_ = false;
        repaint_ = true;
    }

    /// COPY mechanism `i`'s block (LiveTuning::copyBlock) into the provider's sink — on the brain the
    /// terminal and the SD copy — and say how many lines on the page. Returns the count.
    /// PRECONDITION: `i` below the table's count.
    int copy(std::size_t i) {
        const int n = tuning_.copyBlock(i, d_.devices.telemetry());
        std::memcpy(static_cast<void*>(&copiedAs_[i]), &tuning_.spec(i), sizeof(MechanismSpec));
        char s[200];
        if (n == 0) {
            std::snprintf(s, sizeof s, "'%.23s': COPY -- 0 lines: its numbers are the robot file's", tuning_.spec(i).name);
        } else {
            std::snprintf(s, sizeof s,
                          "'%.23s': COPY -- %d line%s to the terminal and /usd/tune_log.txt: put them in the "
                          "robot file",
                          tuning_.spec(i).name, n, n == 1 ? "" : "s");
        }
        setStatus(s);
        say(hal::LogLevel::Info, s);
        return n;
    }

    /// The COPY button (D9 (a): "for each mechanism changed, ONE block"): a block for EVERY mechanism
    /// whose numbers differ from the robot file's — or, when none does, the shown one's (0 lines, which
    /// says so). Returns the lines copied in all.
    int copyChanged() {
        std::size_t changedCount = 0;
        int lines = 0;
        for (std::size_t i = 0; i < tuning_.count(); ++i) {
            if (tuning_.changed(i)) {
                ++changedCount;
                lines += copy(i);
            }
        }
        if (changedCount == 0) {
            return copy(shown_);
        }
        if (changedCount > 1) {
            char s[200];
            std::snprintf(s, sizeof s,
                          "COPY: %d lines from %u mechanisms to the terminal and /usd/tune_log.txt: put them in the "
                          "robot file",
                          lines, static_cast<unsigned>(changedCount));
            setStatus(s);
            say(hal::LogLevel::Info, s);
        }
        return lines;
    }

    /// True when mechanism `i`'s numbers differ from the robot file's AND from what its last COPY wrote
    /// (a change no COPY carries yet: what EXIT, or a cap going back, COPIES first).
    [[nodiscard]] bool uncopied(std::size_t i) const {
        return tuning_.changed(i) && std::memcmp(&tuning_.spec(i), &copiedAs_[i], sizeof(MechanismSpec)) != 0;
    }

    // ── what the host tests read ───────────────────────────────────────────────────────
    /// The page on the screen.
    [[nodiscard]] TunePage page() const noexcept { return page_; }
    /// True once READY made the mechanisms.
    [[nodiscard]] bool made() const noexcept { return made_; }
    /// The shown mechanism's index.
    [[nodiscard]] std::size_t shown() const noexcept { return shown_; }
    /// The selected number's position in the shown mechanism's list.
    [[nodiscard]] std::size_t selected() const noexcept { return param_; }
    /// The view last painted (what a touch is tested against).
    [[nodiscard]] const TuneView& drawn() const noexcept { return drawn_; }
    /// The last TEST's report.
    [[nodiscard]] const TuneReport& report() const noexcept { return report_; }
    /// The last TEST's samples: the move's.
    [[nodiscard]] std::span<const TuneSample> moveSamples() const noexcept {
        return std::span<const TuneSample>{samples_.data(), moveCount_};
    }
    /// ... and its hold second's.
    [[nodiscard]] std::span<const TuneSample> holdSamples() const noexcept {
        return std::span<const TuneSample>{samples_.data() + moveCount_, sampleCount_ - moveCount_};
    }
    /// The last TEST's time limit, s.
    [[nodiscard]] double lastBound() const noexcept { return test_.bound; }
    /// The status sentence on the page.
    [[nodiscard]] const char* status() const noexcept { return status_; }
    /// A test, a homing or a test's hold second is running.
    [[nodiscard]] bool busy() const noexcept { return phase_ != Phase::Idle; }
    /// Live tuning (the table the service runs).
    [[nodiscard]] LiveTuning& tuning() noexcept { return tuning_; }
    /// The service (bound once made).
    [[nodiscard]] MechanismService& service() noexcept { return svc_; }
    /// The built parts.
    [[nodiscard]] const robot::MechanismParts& parts() const noexcept { return parts_; }
    /// Whether mechanism `i` passed stage 2 of the clear confirm this session.
    [[nodiscard]] bool confirmed(std::size_t i) const noexcept { return i < kMaxMechanisms && confirmed_[i]; }
    /// How many current caps EXIT (or leaving a page) wrote back, in all.
    [[nodiscard]] int capsWrittenBack() const noexcept { return capsBack_; }
    /// How many mechanisms the last EXIT COPIED (changed, never copied) and RESET (a cap that could not
    /// go back alone).
    [[nodiscard]] int copiedAtExit() const noexcept { return exitCopied_; }
    /// ... and RESET.
    [[nodiscard]] int resetAtExit() const noexcept { return exitReset_; }
    /// How many undo records the program tracks (which mechanism each is): always LiveTuning's
    /// undoDepth() — every change goes through the tracking.
    [[nodiscard]] std::size_t undoTrackedDepth() const noexcept { return undoWhoCount_; }

private:
    enum class Phase : std::uint8_t { Idle, Moving, Holding, Homing };
    enum class Motion : std::uint8_t { None, TestA, TestB, Home, Jog, StringCheck };

    // ── the loop ───────────────────────────────────────────────────────────────────────
    bool stepImpl() {
        if (faulted_) {
            paint();
            d_.devices.pacer().pace();
            return !driverControlEnded(d_.io.competitionStatus());
        }
        const std::uint8_t status = d_.io.competitionStatus();
        if (driverControlEnded(status)) {
            const char* why = (status & kTuneCompetitionDisabled) != 0 ? "VEXos reports the robot DISABLED"
                                                                        : "autonomous started";
            if (made_) {
                stopAll(why);
            }
            char line[160];
            std::snprintf(line, sizeof line, "%s -- every mechanism released; Tune waits for driver control", why);
            say(hal::LogLevel::Warn, line);
            return false;
        }
        field_ = (status & kTuneCompetitionConnected) != 0;
        controller_ = d_.controller.isConnected();
        buttonHeld_ = anyButton();
        // THE ABORT SET (re-check #7): checked FIRST, applied before this pass's tick.
        const char* cause = buttonHeld_ ? "a controller button"
                            : !controller_ ? "the controller disconnected"
                            : field_       ? "a field or switch was plugged in"
                                           : nullptr;
        if (made_ && cause != nullptr && (cause != lastCause_ || phase_ != Phase::Idle || jogging_)) {
            stopAll(cause);
        }
        lastCause_ = cause;
        const double now = d_.devices.clock().now().value();
        // the touch, against what was drawn
        const hal::ScreenTouch raw = d_.screen.touch();
        const ui::TouchSample sample{raw.error, raw.releaseCount, raw.x, raw.y};
        if (!edge_.has_value()) {
            if (!sample.error) {
                edge_.emplace(sample.releaseCount);
            }
        } else if (edge_->take(sample)) {
            TuneButton b{};
            if (tuneHit(drawn_, sample.x, sample.y, b)) {
                press(b);
            }
        }
        // the stick's EDGE, read on EVERY page: a jog starts only on a fresh push (centred, then pushed) —
        // never carried over a mechanism change, a stop, GO or another page
        const bool pushed = stickPushed();
        const bool stickEdge = pushed && !stickWasOut_;
        stickWasOut_ = pushed;
        if (made_ && page_ == TunePage::Tune) {
            stick(stickEdge);
        }
        paintAt(now);
        if (made_) {
            svc_.pace();  // the mechanisms tick, then the world advances (the provider's pacer)
            afterTick();
        } else {
            d_.devices.pacer().pace();
        }
        return true;
    }

    [[nodiscard]] static bool driverControlEnded(std::uint8_t status) noexcept {
        return (status & kTuneCompetitionDisabled) != 0 || (status & kTuneCompetitionAutonomous) != 0;
    }

    [[nodiscard]] bool anyButton() const {
        for (int b = 0; b <= static_cast<int>(hal::ControllerButton::A); ++b) {
            if (d_.controller.pressed(static_cast<hal::ControllerButton>(b))) {
                return true;
            }
        }
        return false;
    }

    // ── making (stage 1) ─────────────────────────────────────────────────────────────────
    void make() {
        const std::size_t n = tuning_.count();
        for (std::size_t i = 0; i < n; ++i) {
            configs_[i] = toMechanismConfig(tuning_.table().specs[i], tuning_.table().wiring[i]);
        }
        parts_.build(std::span<const config::MechanismConfig>{configs_.data(), n}, d_.devices);
        hal::IDeviceProvider& p = d_.devices;
        svc_.bind(tuning_.table(), parts_.devices(), p.clock(), faults_, p.telemetry());
        tuning_.attach(svc_);
        svc_.enterDriverControl();
        made_ = true;
        char line[240];
        int built = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const MechanismSpec& s = tuning_.spec(i);
            for (const RoleFact& r : parts_.roles(i)) {
                roleLine(line, sizeof line, s.name, r.role, r.port, static_cast<Line>(r.line), r.state, r.refusal,
                         r.degradeOnly);
                say(r.state == RoleState::Refused ? hal::LogLevel::Warn : hal::LogLevel::Info, line);
            }
            if (parts_.builtAt(i) != nullptr) {
                ++built;
            }
            startingValuesLine(line, sizeof line, s);
            if (line[0] != '\0') {
                say(hal::LogLevel::Info, line);
            }
        }
        std::snprintf(line, sizeof line,
                      "READY: %d of %u mechanism%s made (no drive motor is ever made here); each one's first motion "
                      "waits for GO",
                      built, static_cast<unsigned>(n), n == 1 ? "" : "s");
        say(hal::LogLevel::Info, line);
    }

    // ── a touch ─────────────────────────────────────────────────────────────────────────
    void press(TuneButton b) {
        switch (page_) {
            case TunePage::Clear:
                if (b == TuneButton::Ready) {
                    // READY makes the mechanisms — a pneumatic MOVES to its start (HA-119): never without a
                    // connected controller (it carries the STOP), with a field or switch, or with a button
                    // held (re-check #7). The page greys READY then; this is the same rule for a touch read
                    // against the page drawn a tick before the state changed.
                    const char* why = !controller_ ? "no controller is connected (it carries the STOP)"
                                      : field_     ? "a field or competition switch is plugged in"
                                      : buttonHeld_ ? "a controller button is held: let go first"
                                                    : nullptr;
                    if (why != nullptr) {
                        char s[160];
                        std::snprintf(s, sizeof s, "READY refused -- %s; nothing was made", why);
                        setStatus(s);
                        say(hal::LogLevel::Warn, s);
                        return;
                    }
                    make();
                    page_ = TunePage::Tune;
                    setStatus("READY: pick a number with ^ v, change it with - +, then TEST");
                }
                return;
            case TunePage::Tune:
                pressTune(b);
                return;
            case TunePage::Confirm:
                if (b == TuneButton::Go) {
                    confirmed_[shown_] = true;
                    page_ = TunePage::Tune;
                    say(hal::LogLevel::Info, "stage 2: GO -- its first motion confirmed clear");
                    if (pending_ == Motion::Jog) {
                        setStatus("GO: let the left stick centre, then push it up / down to move it");
                    } else {
                        requestMotion(pending_);
                    }
                } else if (b == TuneButton::Back) {
                    page_ = TunePage::Tune;
                    setStatus("BACK: nothing moved");
                }
                pending_ = Motion::None;
                return;
            case TunePage::Report:
                if (b == TuneButton::Again) {
                    page_ = TunePage::Tune;
                    requestMotion(report_.test == TuneTest::B ? Motion::TestB : Motion::TestA);
                } else if (b == TuneButton::Stop) {
                    stopAll("STOP on the screen");
                    page_ = TunePage::Tune;
                    setStatus("STOP: every mechanism released (its safe state)");
                } else if (b == TuneButton::Back) {
                    page_ = TunePage::Tune;
                }
                return;
            case TunePage::StringCheck:
                if (b == TuneButton::Back) {
                    page_ = TunePage::Tune;
                }
                return;
            case TunePage::Exited:
                if (b == TuneButton::TuneAgain) {
                    page_ = TunePage::Tune;
                    if (exitReset_ > 0) {
                        char s[160];
                        std::snprintf(s, sizeof s,
                                      "TUNE AGAIN: live numbers kept, but %d mechanism%s RESET at EXIT (each one's "
                                      "block is in the log)",
                                      exitReset_, exitReset_ == 1 ? " was" : "s were");
                        setStatus(s);
                    } else {
                        setStatus("TUNE AGAIN: your live numbers are kept; the caps went back to the file's");
                    }
                }
                return;
            case TunePage::Stopped:
                return;
        }
    }

    void pressTune(TuneButton b) {
        if (jogging_ && b != TuneButton::Stop && b != TuneButton::Exit) {
            endJog();  // a tap while the stick moves a mechanism ends that jog FIRST (on the mechanism it moves)
        }
        const std::size_t n = tuning_.count();
        const ParameterList params = parametersOf(tuning_.spec(shown_));
        switch (b) {
            case TuneButton::PrevMech:
            case TuneButton::NextMech: {
                if (!restoreCap(shown_, false)) {
                    return;  // its cap could not go back: the status says why, the page stays
                }
                shown_ = b == TuneButton::NextMech ? (shown_ + 1) % n : (shown_ + n - 1) % n;
                param_ = 0;
                return;
            }
            case TuneButton::PrevParam:
                if (params.count > 0) {
                    param_ = (param_ + params.count - 1) % params.count;
                }
                return;
            case TuneButton::NextParam:
                if (params.count > 0) {
                    param_ = (param_ + 1) % params.count;
                }
                return;
            case TuneButton::Minus:
            case TuneButton::Plus: {
                if (params.count == 0) {
                    return;
                }
                const Tunable id = params.items[param_ % params.count].id;
                const Applied a = tracked(shown_, [&] {
                    return tuning_.step(shown_, id, b == TuneButton::Plus ? +1 : -1, fine_);
                });
                setStatus(a.sentence);
                say(a.ok ? hal::LogLevel::Info : hal::LogLevel::Warn, a.sentence);
                return;
            }
            case TuneButton::Fine:
                fine_ = !fine_;
                setStatus(fine_ ? "FINE on: a gain moves by x or / 1.05" : "FINE off: a gain moves by x or / 1.25");
                return;
            case TuneButton::TestA:
                requestMotion(Motion::TestA);
                return;
            case TuneButton::TestB:
                requestMotion(Motion::TestB);
                return;
            case TuneButton::Home:
                requestMotion(Motion::Home);
                return;
            case TuneButton::Undo: {
                // ONLY the shown mechanism may carry a live cap (a cap stays in the motor, and only the shown
                // page's RESET and ‹ › put it back). LiveTuning's UNDO is global, so the undo it would make
                // NEXT is checked BEFORE it is made (undoWho_: which mechanism, and its cap before): one that
                // would change ANOTHER mechanism's cap away from the file's — the cap put back when its page
                // was left, or a RESET of a raised cap — waits for that mechanism's page, and NOTHING changes
                if (const char* other = undoWaitsOn(); other != nullptr) {
                    char s[200];
                    std::snprintf(s, sizeof s,
                                  "'%.23s': UNDO waits on another page -- the next undo would change its current cap "
                                  "(only its own page may change a cap): show it (< >) and UNDO there",
                                  other);
                    say(hal::LogLevel::Warn, s);
                    std::snprintf(s, sizeof s, "UNDO waits: next is '%.23s''s cap -- UNDO it on its page", other);
                    setStatus(s);
                    return;
                }
                const Applied a = undoTracked();
                setStatus(a.sentence);
                say(a.ok ? hal::LogLevel::Info : hal::LogLevel::Warn, a.sentence);
                return;
            }
            case TuneButton::Reset: {
                const Applied a = tracked(shown_, [&] { return tuning_.reset(shown_); });
                setStatus(a.sentence);
                say(a.ok ? hal::LogLevel::Info : hal::LogLevel::Warn, a.sentence);
                return;
            }
            case TuneButton::Copy:
                (void)copyChanged();
                return;
            case TuneButton::StringCheck:
                if (const char* why = motionRefusal(Motion::StringCheck)) {
                    refuseWith(Motion::StringCheck, why);
                    return;
                }
                page_ = TunePage::StringCheck;
                return;
            case TuneButton::Stop:
                stopAll("STOP on the screen");
                setStatus("STOP: every mechanism released (its safe state)");
                return;
            case TuneButton::Exit:
                exitAll();
                return;
            default:
                return;
        }
    }

    // ── motion: the refusals, stage 2, the start ────────────────────────────────────────
    [[nodiscard]] static const char* motionWord(Motion m) noexcept {
        switch (m) {
            case Motion::TestA: return "TEST A";
            case Motion::TestB: return "TEST B";
            case Motion::Home: return "HOME";
            case Motion::Jog: return "the stick";
            case Motion::StringCheck: return "STRING CHECK";
            case Motion::None: return "";
        }
        return "";
    }

    /// Why `m` cannot start now (one sentence's reason), or nullptr.
    [[nodiscard]] const char* motionRefusal(Motion m) {
        if (!controller_) {
            return "no controller is connected (it carries the STOP)";
        }
        if (field_) {
            return "a field or competition switch is plugged in (Tune runs at the bench only)";
        }
        if (buttonHeld_) {
            return "a controller button is held (every button is a STOP): let go first";
        }
        if (m != Motion::Jog && m != Motion::StringCheck && stickPushed()) {
            return "the stick is pushed (it moves the mechanism): let go of it first";
        }
        if (m == Motion::StringCheck) {
            return nullptr;
        }
        const Slot* s = svc_.slotPtr(shown_);
        if (s == nullptr || !s->built()) {
            return "it is ABSENT this boot (the terminal says why)";
        }
        const MechanismSpec& spec = tuning_.spec(shown_);
        if (spec.kind == Kind::Pneumatic) {
            return "a pneumatic has nothing to tune here";
        }
        if (spec.kind == Kind::Speed && m == Motion::Jog) {
            return "the stick does not spin a flywheel (TEST A / TEST B do)";
        }
        if (m == Motion::Home && (spec.kind != Kind::Position || spec.position.home == HomeMethod::None)) {
            return "it has no homing step";
        }
        if ((m == Motion::TestA || m == Motion::TestB) && spec.kind == Kind::Position &&
            !svc_.position(shown_).homed()) {
            return "it has not found its zero: press HOME first";
        }
        return nullptr;
    }

    void refuseWith(Motion m, const char* why) {
        char s[240];
        std::snprintf(s, sizeof s, "'%.23s': %s refused -- %s", tuning_.spec(shown_).name, motionWord(m), why);
        setStatus(s);
        say(hal::LogLevel::Warn, s);
    }

    void requestMotion(Motion m) {
        if (const char* why = motionRefusal(m)) {
            refuseWith(m, why);
            return;
        }
        if (!confirmed_[shown_]) {
            pending_ = m;
            page_ = TunePage::Confirm;
            return;
        }
        if (m == Motion::Home) {
            startHome();
        } else if (m == Motion::TestA || m == Motion::TestB) {
            startTest(m == Motion::TestA ? TuneTest::A : TuneTest::B);
        }
    }

    void startHome() {
        clearTest();
        PositionSlot& slot = svc_.position(shown_);
        slot.home(units::Time{kTuneHomeSeconds});
        phase_ = Phase::Homing;
        peak_ = 0.0;
        char s[160];
        std::snprintf(s, sizeof s, "'%.23s': HOME -- its homing step (at most %.0f s)", tuning_.spec(shown_).name,
                      kTuneHomeSeconds);
        setStatus(s);
        say(hal::LogLevel::Info, s);
    }

    void startTest(TuneTest which) {
        clearTest();
        const MechanismSpec& s = tuning_.spec(shown_);
        const TuneTargets tg = tuneTargets(s);
        const double target = which == TuneTest::A ? tg.a : tg.b;
        test_ = TuneMoveFacts{};
        test_.test = which;
        test_.kind = s.kind;
        test_.target = target;
        test_.capAmps = s.currentCapAmps;
        const StallSpec* st = detail::liveStall(s);
        test_.stallAmps = st != nullptr && st->stated() ? st->amps : 0.0;
        switch (s.kind) {
            case Kind::Position: {
                PositionSlot& slot = svc_.position(shown_);
                test_.unit = detail::tuneUnitWord(s.position.units);
                test_.start = slot.position();
                test_.within = s.position.tolerance;
                test_.bound = tunePositionBound(s, slot.jointDeg(), target);
                slot.moveTo(target, units::Time{test_.bound}, true);
                break;
            }
            case Kind::Speed: {
                SpeedSlot& slot = svc_.speed(shown_);
                test_.unit = "rpm";
                test_.start = slot.rpm();
                test_.within = s.speed.atSpeedBandRpm;
                test_.bound = kTuneSpeedTestSeconds;
                slot.spinUp(target, units::Time{test_.bound}, true);
                break;
            }
            case Kind::Power: {
                PowerSlot& slot = svc_.power(shown_);
                test_.unit = "V";
                test_.bound = kTunePowerTestSeconds + kTuneTestExtraSeconds;
                slot.run(units::Voltage{target}, Until::time(units::Time{kTunePowerTestSeconds}),
                         units::Time{test_.bound});
                break;
            }
            case Kind::Pneumatic:
                return;
        }
        phase_ = Phase::Moving;
        char line[200];
        std::snprintf(line, sizeof line, "'%.23s': TEST %s -- to %s %.3f %s, time limit %.2f s", s.name,
                      which == TuneTest::B ? "B" : "A", which == TuneTest::B ? tg.bName : tg.aName, target, test_.unit,
                      test_.bound);
        setStatus(line);
        say(hal::LogLevel::Info, line);
    }

    // ── after each tick: the samples, the verdicts ───────────────────────────────────────
    [[nodiscard]] double sampleX() {
        const MechanismSpec& s = tuning_.spec(testIndex_);
        if (s.kind == Kind::Position) {
            return svc_.position(testIndex_).position();
        }
        if (s.kind == Kind::Speed) {
            return svc_.speed(testIndex_).rpm();
        }
        return 0.0;
    }
    [[nodiscard]] double ampsOf(std::size_t i) const {
        hal::MotorMechanism* m = parts_.motorAt(i);
        return m != nullptr ? m->maxCurrent().value() : 0.0;
    }

    void record() {
        if (sampleCount_ >= kTuneMaxSamples) {
            return;
        }
        samples_[sampleCount_++] =
            TuneSample{d_.devices.clock().now().value(), sampleX(), ampsOf(testIndex_)};
    }

    void afterTick() {
        switch (phase_) {
            case Phase::Idle:
                return;
            case Phase::Homing: {
                peak_ = std::max(peak_, ampsOf(testIndex_));
                const Slot& s = svc_.slot(testIndex_);
                if (s.running()) {
                    return;
                }
                const StepReport& r = s.lastStep();
                char line[200];
                std::snprintf(line, sizeof line, "'%.23s': HOME %s in %.2f s, its push's peak %.2f A%s",
                              s.name(), manipulation::mechanismOutcomeName(r.outcome), r.elapsed, peak_,
                              r.outcome == manipulation::MechanismOutcome::Succeeded ? " -- its zero is found" : "");
                setStatus(line);
                say(r.outcome == manipulation::MechanismOutcome::Succeeded ? hal::LogLevel::Info : hal::LogLevel::Warn,
                    line);
                phase_ = Phase::Idle;
                return;
            }
            case Phase::Moving: {
                record();
                const Slot& s = svc_.slot(testIndex_);
                if (s.running()) {
                    return;
                }
                moveCount_ = sampleCount_;
                test_.outcome = s.lastStep().outcome;
                test_.seconds = s.lastStep().elapsed;
                verdictT_ = sampleCount_ > 0 ? samples_[sampleCount_ - 1].t : 0.0;
                if (test_.outcome == manipulation::MechanismOutcome::Succeeded && test_.kind != Kind::Power) {
                    phase_ = Phase::Holding;
                    return;
                }
                finishTest();
                return;
            }
            case Phase::Holding: {
                const double t = d_.devices.clock().now().value();
                if (t - verdictT_ <= kTuneHoldSeconds + 1e-9) {
                    record();
                }
                if (svc_.slot(testIndex_).lastStep().outcome != manipulation::MechanismOutcome::Succeeded) {
                    holdCut_ = true;  // its hold ended inside the second (a stall signature tripped on it)
                }
                if (t - verdictT_ >= kTuneHoldSeconds - 1e-9) {
                    finishTest();
                }
                return;
            }
        }
    }

    /// The report from the samples; `holdStopped`: a STOP ended its hold second (the hold is then n/a,
    /// with no warning of its own) — unless the hold had ALREADY ENDED (the stall tripped on it) before
    /// that stop: the ENDED hold and its warning come first, the stop is still said ("stopped by").
    void finishTest(bool holdStopped = false) {
        phase_ = Phase::Idle;
        const std::span<const TuneSample> move{samples_.data(), moveCount_};
        const std::span<const TuneSample> hold{samples_.data() + moveCount_, sampleCount_ - moveCount_};
        report_ = tuneReportFrom(move, hold, test_);
        if (holdCut_) {
            // its hold ENDED inside the second: its stall signature tripped on an ordinary hold (V1) — the
            // finding the hold second exists for, so a STOP pressed after it never hides it
            report_.holdAmps = -1.0;
            report_.holdEnded = true;
            report_.warnStallNotAboveHold = test_.stallAmps > 0.0;
        } else if (holdStopped) {
            report_.holdAmps = -1.0;
            report_.holdStopped = true;
            report_.warnStallNotAboveHold = false;
        }
        if (test_.kind == Kind::Speed) {
            svc_.speed(testIndex_).stop();  // a flywheel is stopped after its report's second
        }
        page_ = TunePage::Report;
        logReport();
    }

    void logReport() {
        const TuneReport& r = report_;
        const char* name = tuning_.spec(testIndex_).name;
        char line[240];
        std::snprintf(line, sizeof line, "'%.23s': TEST %s to %.3f %s: %s in %.2f s (limit %.2f s)%s%s", name,
                      r.test == TuneTest::B ? "B" : "A", r.target, r.unit, r.verdict, r.seconds, r.bound,
                      r.stoppedBy[0] != '\0' ? " -- stopped by " : "", r.stoppedBy);
        say(std::strcmp(r.verdict, "SUCCEEDED") == 0 ? hal::LogLevel::Info : hal::LogLevel::Warn, line);
        std::snprintf(line, sizeof line,
                      "'%.23s': report -- overshoot %.4f %s, settle %.2f s, peak %.2f A, final error %+.4f %s, hold "
                      "%.2f A (mean over the second after it settled; -1 = n/a)",
                      name, r.overshoot, r.unit, r.settle, r.peakAmps, r.hasFinalError ? r.finalError : 0.0, r.unit,
                      r.holdAmps);
        say(hal::LogLevel::Info, line);
        if (r.holdEnded) {  // the page's own flag: the page and the log say the same
            if (r.warnStallNotAboveHold) {
                std::snprintf(line, sizeof line,
                              "'%.23s': WARNING -- its hold ENDED inside the second after it settled (its stall "
                              "signature tripped on the hold): raise the signature above the heaviest hold",
                              name);
            } else {
                std::snprintf(line, sizeof line, "'%.23s': its hold ENDED inside the second after it settled",
                              name);
            }
            say(hal::LogLevel::Warn, line);
        } else if (r.warnStallNotAboveHold) {
            std::snprintf(line, sizeof line,
                          "'%.23s': WARNING -- its stall signature %.2f A is not above its hold %.2f A: an ordinary "
                          "hold can trip it (STALLED)",
                          name, r.stallAmps, r.holdAmps);
            say(hal::LogLevel::Warn, line);
        }
        if (r.warnCapNotAbovePeak) {
            std::snprintf(line, sizeof line,
                          "'%.23s': WARNING -- its current cap %.2f A is not above the move's peak %.2f A: the cap "
                          "limited the move",
                          name, r.capAmps, r.peakAmps);
            say(hal::LogLevel::Warn, line);
        }
    }

    void clearTest() {
        phase_ = Phase::Idle;
        testIndex_ = shown_;
        sampleCount_ = 0;
        moveCount_ = 0;
        holdCut_ = false;
    }

    // ── the stick ───────────────────────────────────────────────────────────────────────
    /// The left stick is pushed past its deadband.
    [[nodiscard]] bool stickPushed() const {
        const double y = d_.controller.axis(hal::ControllerAxis::LeftY);
        return std::isfinite(y) && std::abs(y) > kTuneJogDeadband;
    }

    /// The stick on the TUNE page; `edge`: it was pushed THIS pass (centred the pass before). A jog STARTS
    /// only on an edge — a stick still held over a mechanism change, a stop, GO or another page moves
    /// nothing until it is let go and pushed again.
    void stick(bool edge) {
        const double y = d_.controller.axis(hal::ControllerAxis::LeftY);
        const MechanismSpec& s = tuning_.spec(shown_);
        if (!stickPushed()) {
            if (jogging_) {
                endJog();
            }
            return;
        }
        if (phase_ != Phase::Idle || lastCause_ != nullptr) {
            return;
        }
        if (!jogging_) {
            if (!edge) {
                return;
            }
            if (const char* why = motionRefusal(Motion::Jog)) {
                refuseWith(Motion::Jog, why);
                return;
            }
            if (!confirmed_[shown_]) {
                pending_ = Motion::Jog;
                page_ = TunePage::Confirm;
                return;
            }
        }
        const double cap = s.kind == Kind::Power ? std::min(kTuneJogMostVolts, s.power.maxVolts) : kTuneJogMostVolts;
        const double volts = std::clamp(y, -1.0, 1.0) * cap;
        if (s.kind == Kind::Position) {
            svc_.position(shown_).jog(units::Voltage{volts});
        } else if (s.kind == Kind::Power) {
            svc_.power(shown_).drive(units::Voltage{volts});
        }
        jogging_ = true;
    }

    void endJog() {
        const MechanismSpec& s = tuning_.spec(shown_);
        if (s.kind == Kind::Position) {
            svc_.position(shown_).holdHere();
        } else if (s.kind == Kind::Power) {
            svc_.power(shown_).stop();
        }
        jogging_ = false;
    }

    // ── stop, the caps, EXIT ─────────────────────────────────────────────────────────────
    /// Every mechanism released to its safe state NOW (header: the abort set); a running test reported
    /// as stopped by `cause`.
    void stopAll(const char* cause) {
        const Phase was = phase_;
        svc_.leaveDriverControl();
        jogging_ = false;
        char line[200];
        std::snprintf(line, sizeof line, "STOP -- %s: every mechanism released (its declared safe state)", cause);
        say(hal::LogLevel::Warn, line);
        if (was == Phase::Moving || was == Phase::Holding) {
            if (was == Phase::Moving) {
                moveCount_ = sampleCount_;
                const Slot& s = svc_.slot(testIndex_);
                test_.outcome = s.lastStep().outcome;
                test_.seconds = s.lastStep().elapsed;
            }
            test_.stoppedBy = cause;
            finishTest(was == Phase::Holding);
        } else if (was == Phase::Homing) {
            phase_ = Phase::Idle;
        }
        phase_ = Phase::Idle;
    }

    // ── who each undo touches (the UNDO cap rule) ─────────────────────────────────────
    // LiveTuning keeps its undo records private (which mechanism each one is) and its UNDO is global. So
    // EVERY change this program makes goes through tracked() / undoTracked(), which keep undoWho_ in step
    // with LiveTuning's own records — the same push, the same oldest-falls-off past kLiveUndoDepth, the
    // same pop — holding for each one the mechanism and its current cap BEFORE it (what an undo of it
    // brings back). undoTrackedDepth() == LiveTuning::undoDepth() always (the host tests check it).

    /// Run one LiveTuning change of mechanism `i` (apply, step or reset) and record it when it committed:
    /// LiveTuning records exactly the changes that alter the spec's bytes (an "unchanged" apply, an
    /// "already the file's" reset and every refusal record nothing).
    template <class Change>
    Applied tracked(std::size_t i, Change&& change) {
        MechanismSpec before;
        std::memcpy(static_cast<void*>(&before), &tuning_.spec(i), sizeof before);
        const Applied a = change();
        if (a.ok && std::memcmp(&before, &tuning_.spec(i), sizeof before) != 0) {
            if (undoWhoCount_ == kLiveUndoDepth) {
                undoWhoStart_ = (undoWhoStart_ + 1) % kLiveUndoDepth;
                --undoWhoCount_;
            }
            undoWho_[(undoWhoStart_ + undoWhoCount_) % kLiveUndoDepth] = UndoWho{i, before.currentCapAmps};
            ++undoWhoCount_;
        }
        return a;
    }
    /// LiveTuning::undo(), the record popped when it undid (a refused undo keeps its record).
    Applied undoTracked() {
        const Applied a = tuning_.undo();
        if (a.ok && undoWhoCount_ > 0) {
            --undoWhoCount_;
        }
        return a;
    }
    /// The name of the mechanism — NOT the shown one — whose current cap the next undo would change away
    /// from the robot file's, or null when UNDO may run here.
    [[nodiscard]] const char* undoWaitsOn() const {
        if (undoWhoCount_ == 0) {
            return nullptr;
        }
        const UndoWho& top = undoWho_[(undoWhoStart_ + undoWhoCount_ - 1) % kLiveUndoDepth];
        if (top.index == shown_ || top.capBefore == tuning_.declared().specs[top.index].currentCapAmps) {
            return nullptr;
        }
        return tuning_.spec(top.index).name;
    }

    /// Mechanism `i`'s live current cap differs from the robot file's.
    [[nodiscard]] bool capDiffers(std::size_t i) const {
        return tuning_.spec(i).currentCapAmps != tuning_.declared().specs[i].currentCapAmps;
    }

    /// Put mechanism `i`'s current cap back to the robot file's (header: the cap stays in the motor) —
    /// LiveTuning::apply of the file's number, after COPYING its numbers to the log when no COPY carries
    /// them yet (the tuned cap leaves the live numbers here: it must not leave the student's hands too).
    /// `orReset` (EXIT): when that apply is refused, RESET the mechanism (its numbers COPIED first). True
    /// when its cap is the file's afterwards; the status says what happened in a sentence the page's
    /// two lines hold, the log says it whole.
    bool restoreCap(std::size_t i, bool orReset, bool* didReset = nullptr) {
        if (didReset != nullptr) {
            *didReset = false;
        }
        if (!capDiffers(i)) {
            return true;
        }
        const char* name = tuning_.spec(i).name;
        const double file = tuning_.declared().specs[i].currentCapAmps;
        const bool copiedNow = uncopied(i);
        if (copiedNow) {
            char c[200];
            std::snprintf(c, sizeof c,
                          "'%.23s': its cap goes back to the robot file's, so its changed numbers are COPIED first "
                          "(below)",
                          name);
            say(hal::LogLevel::Info, c);
            (void)copy(i);
        }
        Applied a = tracked(i, [&] { return tuning_.apply(i, Tunable::CurrentCap, file); });
        bool reset = false;
        if (!a.ok && orReset) {
            say(hal::LogLevel::Warn, a.sentence);  // why the cap alone could not go back
            a = tracked(i, [&] { return tuning_.reset(i); });
            reset = a.ok;
        }
        char s[240];  // the log line within the log's 200 bytes: 102 + a 23-character name + 72
        if (a.ok) {
            ++capsBack_;
            if (didReset != nullptr) {
                *didReset = reset;
            }
            std::snprintf(s, sizeof s,
                          "'%.23s': its current cap back to the robot file's (a cap set here stays in the motor "
                          "after the program) -- %.72s",
                          name, a.sentence);
            say(reset ? hal::LogLevel::Warn : hal::LogLevel::Info, s);
            char limit[48];
            if (file > 0.0) {
                std::snprintf(limit, sizeof limit, "the file's %.2f A", file);
            } else {
                std::snprintf(limit, sizeof limit, "%s", "the motor's 2.5 A (the file states none)");
            }
            if (reset) {
                std::snprintf(s, sizeof s,
                              "'%.23s': RESET to the file's numbers (a signature >= its cap); its numbers are "
                              "COPIED in the log",
                              name);
            } else {
                std::snprintf(s, sizeof s, "'%.23s': its cap is %s again%s", name, limit,
                              copiedNow ? "; its numbers were COPIED first" : "");
            }
            setStatus(s);
            return true;
        }
        say(hal::LogLevel::Warn, a.sentence);  // LiveTuning's whole sentence: why
        const bool signature = a.why == LiveRefusal::BuildRefuses &&
                               a.build == Refusal::AStallSignatureMustBeBelowTheCurrentCap;
        if (signature) {
            std::snprintf(s, sizeof s,
                          "'%.23s': its cap cannot go back to the file's %.2f A: a live stall or homing signature is "
                          "at or above it -- lower that signature, or RESET it (its numbers are COPIED)",
                          name, file);
            say(hal::LogLevel::Warn, s);
            std::snprintf(s, sizeof s,
                          "'%.23s': can't leave: a signature >= the file's cap %.2f A -- lower it, or RESET (your "
                          "numbers are COPIED)",
                          name, file);
        } else {
            std::snprintf(s, sizeof s,
                          "'%.23s': its cap cannot go back to the file's yet (the line above says why) -- try again, "
                          "or RESET it (its numbers are COPIED)",
                          name);
            say(hal::LogLevel::Warn, s);
            std::snprintf(s, sizeof s,
                          "'%.23s': can't leave: its cap can't go back yet (the terminal says why) -- try again, or "
                          "RESET",
                          name);
        }
        setStatus(s);
        return false;
    }

    void exitAll() {
        stopAll("EXIT");
        // the live numbers are not kept: every change no COPY carries yet is COPIED to the log first
        // (the terminal and /usd/tune_log.txt), so EXIT — and a RESET it may need below — loses nothing
        int copied = 0;
        for (std::size_t i = 0; i < tuning_.count(); ++i) {
            if (uncopied(i)) {
                (void)copy(i);
                ++copied;
            }
        }
        int back = 0;
        int reset = 0;
        for (std::size_t i = 0; i < tuning_.count(); ++i) {
            if (!capDiffers(i)) {
                continue;
            }
            bool didReset = false;
            if (restoreCap(i, true, &didReset)) {
                ++back;
                if (didReset) {  // the cap alone could not go back: every number is the file's now
                    ++reset;
                    char r[200];
                    std::snprintf(r, sizeof r,
                                  "EXIT: '%.23s': RESET to the robot file's numbers -- a live signature is at or "
                                  "above the file's cap (its numbers were COPIED above)",
                                  tuning_.spec(i).name);
                    say(hal::LogLevel::Warn, r);
                }
            }
        }
        exitedCaps_ = back;
        exitCopied_ = copied;
        exitReset_ = reset;
        char s[160];
        std::snprintf(s, sizeof s, "EXIT: %d current cap%s written back; every mechanism released -- you may stop "
                                   "the program now",
                      back, back == 1 ? "" : "s");
        say(hal::LogLevel::Info, s);
        if (copied > 0) {
            std::snprintf(s, sizeof s,
                          "EXIT: %d mechanism%s changed numbers COPIED above -- put them into the robot file by hand",
                          copied, copied == 1 ? "'s" : "s'");
            say(hal::LogLevel::Info, s);
        }
        setStatus(s);
        page_ = TunePage::Exited;
    }

    void programFault(const char* what) noexcept {
        faulted_ = true;
        page_ = TunePage::Stopped;
        std::snprintf(faultWhy_, sizeof faultWhy_, "%s", what != nullptr ? what : "");
        try {
            if (made_) {
                svc_.leaveDriverControl();
            }
            char line[240];
            std::snprintf(line, sizeof line, "Tune STOPPED on a program error: %.160s -- every mechanism released",
                          faultWhy_);
            say(hal::LogLevel::Error, line);
        } catch (...) {
        }
    }

    // ── the page ───────────────────────────────────────────────────────────────────────
    void setStatus(const char* s) {
        std::snprintf(status_, sizeof status_, "%s", s != nullptr ? s : "");
    }
    /// The status as the page shows it: a sentence about the SHOWN mechanism without its leading
    /// "'<name>': " (the title line names it) — two 50-character lines hold the rest of the sentence; the
    /// log has it whole. A sentence naming another mechanism keeps its name.
    [[nodiscard]] const char* screenStatus(const char* shownName) const {
        const std::size_t n = std::strlen(shownName);
        if (n > 0 && status_[0] == '\'' && std::strncmp(status_ + 1, shownName, n) == 0 && status_[n + 1] == '\'' &&
            status_[n + 2] == ':' && status_[n + 3] == ' ') {
            return status_ + n + 4;
        }
        return status_;
    }
    void say(hal::LogLevel level, const char* text) { d_.devices.telemetry().log(level, kTuneTag, text); }

    [[nodiscard]] TuneFacts facts() {
        TuneFacts f{};
        f.page = page_;
        f.mechCount = tuning_.count();
        f.mech = shown_;
        f.controller = controller_;
        f.field = field_;
        f.buttonHeld = buttonHeld_;
        f.busy = phase_ != Phase::Idle;
        f.fine = fine_;
        f.status = status_;
        f.undoDepth = tuning_.undoDepth();
        f.report = report_;
        f.confirmWhat = motionWord(pending_);
        f.limitsWritten = exitedCaps_;
        f.copiedAtExit = exitCopied_;
        f.resetAtExit = exitReset_;
        f.stoppedWhy = faultWhy_;
        if (tuning_.count() == 0) {
            return f;
        }
        for (std::size_t i = 0; i < tuning_.count(); ++i) {
            f.anyCapChanged = f.anyCapChanged || capDiffers(i);
        }
        const MechanismSpec& s = tuning_.spec(shown_);
        f.status = screenStatus(s.name);
        TuneMechFacts& m = f.m;
        m.name = s.name;
        m.kind = s.kind;
        m.units = s.kind == Kind::Position ? s.position.units : Units::Degrees;
        m.changed = tuning_.changed(shown_);
        m.capChanged = s.currentCapAmps != tuning_.declared().specs[shown_].currentCapAmps;
        if (made_) {
            const Slot* slot = svc_.slotPtr(shown_);
            m.built = slot != nullptr && slot->built();
            m.lastVerdict = slot != nullptr && slot->stepsBegun() > 0
                                ? manipulation::mechanismOutcomeName(slot->lastStep().outcome)
                                : "";
            MechanismService& svc = svc_;
            if (m.built && s.kind == Kind::Position) {
                PositionSlot& p = svc.position(shown_);
                m.homed = p.homed();
                m.now = p.position();
                m.target = p.target();
                m.active = p.holding() || p.running() || p.heldButtonMoving();
            } else if (m.built && s.kind == Kind::Speed) {
                SpeedSlot& q = svc.speed(shown_);
                m.now = q.rpm();
                m.target = q.targetRpm();
                m.active = q.spinning();
            } else if (m.built && s.kind == Kind::Power) {
                m.now = svc.power(shown_).commandedVolts();
            }
        }
        m.homes = s.kind == Kind::Position && s.position.home != HomeMethod::None;
        const ParameterList params = parametersOf(s);
        f.paramCount = params.count;
        if (params.count > 0) {
            const std::size_t k = param_ % params.count;
            f.param = k;
            const Parameter& p = params.items[k];
            f.sel = TuneParamFacts{p.name, tunableValue(s, p.id), tunableValue(tuning_.declared().specs[shown_], p.id),
                                   tunableUnit(s, p.id), p.rule};
            f.sel.capNone = p.id == Tunable::CurrentCap && f.sel.value == 0.0 && f.sel.file == 0.0;
            if (k + 1 < params.count) {
                f.next.name = params.items[k + 1].name;
            }
        }
        const TuneTargets tg = tuneTargets(s);
        f.testA = tg.a;
        f.testB = tg.b;
        f.testAName = tg.aName[0] != '\0' ? "A" : "";
        f.testBName = tg.bName[0] != '\0' ? "B" : "";
        return f;
    }

    void paintAt(double now) {
        TuneFacts f = facts();
        f.armed = false;
        TuneView v = tuneView(f);
        const double since = arming_.observe(v.layoutKey(), now);
        if (ui::ArmingClock::armedAfter(since)) {
            f.armed = true;
            v = tuneView(f);
        }
        labelTests(v, f);
        render(v, now);
    }

    void paint() { paintAt(d_.devices.clock().now().value()); }

    /// TEST A / TEST B carry their targets ("A 22", "B REV").
    void labelTests(TuneView& v, const TuneFacts& f) const {
        for (std::size_t i = 0; i < v.buttonCount; ++i) {
            TuneButtonView& b = v.buttons[i];
            if (b.id == TuneButton::TestA) {
                tuneTestLabel(b.label, sizeof b.label, 'A', f.m.kind, f.testA);
            } else if (b.id == TuneButton::TestB) {
                tuneTestLabel(b.label, sizeof b.label, 'B', f.m.kind, f.testB);
            }
        }
    }

    [[nodiscard]] static std::uint32_t textColour(ui::Tone t) noexcept {
        switch (t) {
            case ui::Tone::Blue: return tune_colours::kLightBlue;
            case ui::Tone::Amber: return tune_colours::kAmber;
            case ui::Tone::Green: return tune_colours::kGreen;
            case ui::Tone::Red: return tune_colours::kRed;
            case ui::Tone::Grey: return tune_colours::kGrey;
            case ui::Tone::White: return tune_colours::kText;
        }
        return tune_colours::kText;
    }
    [[nodiscard]] static hal::ScreenTextSize sizeOf(ui::TextSize s) noexcept {
        return s == ui::TextSize::Large    ? hal::ScreenTextSize::Large
               : s == ui::TextSize::Medium ? hal::ScreenTextSize::Medium
                                           : hal::ScreenTextSize::Small;
    }

    void paintButton(const TuneButtonView& b) {
        std::uint32_t fill = tune_colours::kBlue;
        std::uint32_t label = tune_colours::kText;
        if (!b.armed) {
            fill = tune_colours::kGreyFill;
            label = tune_colours::kGrey;
        } else if (b.tone == ui::Tone::Red) {
            fill = tune_colours::kRed;
        } else if (b.tone == ui::Tone::Green) {
            fill = tune_colours::kGreen;
            label = tune_colours::kGround;
        }
        hal::IBrainScreen& s = d_.screen;
        s.fillRect(b.rect.x0, b.rect.y0, b.rect.x1, b.rect.y1, fill);
        s.outlineRect(b.rect.x0, b.rect.y0, b.rect.x1, b.rect.y1, tune_colours::kText);
        constexpr int kCharW = 472 / static_cast<int>(ui::kSmallCols);
        const int width = b.rect.x1 - b.rect.x0 + 1;
        const int textW = static_cast<int>(std::strlen(b.label)) * kCharW;
        const int x = b.rect.x0 + (width > textW + 4 ? (width - textW) / 2 : 2);
        const int y = b.rect.y0 + (b.rect.y1 - b.rect.y0 + 1 - ui::kSmallHeight) / 2;
        s.print(hal::ScreenTextSize::Small, static_cast<std::int16_t>(x), static_cast<std::int16_t>(y), label, fill,
                b.label, tuneLabelCols(b.rect));
    }

    /// Bring the screen to `v`: a new layout (or a forced repaint) erases and paints everything; else a
    /// line is repainted when its text or tone changed (a live number at most every 0.1 s), a button when
    /// its label or armed state did.
    void render(const TuneView& v, double now) {
        const bool full = repaint_ || v.layoutKey() != drawn_.layoutKey() || v.page != drawn_.page ||
                          v.lineCount != drawn_.lineCount;
        hal::IBrainScreen& s = d_.screen;
        if (full) {
            s.eraseAll(tune_colours::kGround);
            for (std::size_t i = 0; i < v.lineCount; ++i) {
                const TuneLine& l = v.lines[i];
                s.print(sizeOf(l.size), l.x, l.y, textColour(l.tone), tune_colours::kGround, l.text, ui::budgetFor(l.size));
                lineAt_[i] = now;
            }
            for (std::size_t b = 0; b < v.buttonCount; ++b) {
                paintButton(v.buttons[b]);
            }
            drawn_ = v;
            repaint_ = false;
            return;
        }
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            const TuneLine& want = v.lines[i];
            TuneLine& have = drawn_.lines[i];
            const bool toneChanged = want.tone != have.tone;
            const bool textChanged = std::strcmp(want.text, have.text) != 0;
            if (!toneChanged && (!textChanged || now - lineAt_[i] < 0.1)) {
                continue;
            }
            s.print(sizeOf(want.size), want.x, want.y, textColour(want.tone), tune_colours::kGround, want.text,
                    ui::budgetFor(want.size), std::strlen(have.text));
            have = want;
            lineAt_[i] = now;
        }
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            const TuneButtonView& want = v.buttons[b];
            TuneButtonView& have = drawn_.buttons[b];
            if (want.armed != have.armed || want.tone != have.tone || std::strcmp(want.label, have.label) != 0) {
                paintButton(want);
                have = want;
            }
        }
    }

    // ── state (lifetime order: the table copy, the configs and the parts BEFORE the service) ──
    TuneDevices d_;
    const char* stamp_;
    LiveTuning tuning_;
    std::array<config::MechanismConfig, kMaxMechanisms> configs_{};
    robot::MechanismParts parts_{};
    diag::FaultLatch faults_;
    MechanismService svc_;
    bool made_ = false;
    bool faulted_ = false;
    char faultWhy_[160]{};
    TunePage page_ = TunePage::Clear;
    std::size_t shown_ = 0;
    std::size_t param_ = 0;
    bool fine_ = false;
    bool field_ = false;
    bool controller_ = false;
    bool buttonHeld_ = false;
    const char* lastCause_ = nullptr;
    bool jogging_ = false;
    bool stickWasOut_ = false;
    std::array<bool, kMaxMechanisms> confirmed_{};
    Motion pending_ = Motion::None;
    Phase phase_ = Phase::Idle;
    std::size_t testIndex_ = 0;
    TuneMoveFacts test_{};
    std::array<TuneSample, kTuneMaxSamples> samples_{};
    std::size_t sampleCount_ = 0;
    std::size_t moveCount_ = 0;
    double verdictT_ = 0.0;
    bool holdCut_ = false;
    double peak_ = 0.0;
    TuneReport report_{};
    int capsBack_ = 0;
    int exitedCaps_ = 0;
    int exitCopied_ = 0;
    int exitReset_ = 0;
    std::array<MechanismSpec, kMaxMechanisms> copiedAs_{};  ///< each mechanism as its last COPY wrote it
    /// One undo record's mechanism and its current cap before the change (what undoing it brings back).
    struct UndoWho {
        std::size_t index = 0;
        double capBefore = 0.0;
    };
    std::array<UndoWho, kLiveUndoDepth> undoWho_{};  ///< in step with LiveTuning's records (tracked())
    std::size_t undoWhoStart_ = 0;
    std::size_t undoWhoCount_ = 0;
    char status_[400]{};
    std::optional<ui::ReleaseEdge> edge_{};
    ui::ArmingClock arming_{};
    TuneView drawn_{};
    std::array<double, kTuneMaxLines> lineAt_{};
    bool repaint_ = true;
};

}  // namespace shulib::mechanism
