#pragma once
//
// mechanism/live_tuning.hpp — LIVE TUNING's library half: a mechanism's numbers changed WHILE THE
// ROBOT RUNS, inside the same rules as the build, undone, and copied back out as the lines its robot
// file needs (chunk TUNE1-A, 2026-11-10; the MECH-S/TUNE1 brief's D8 (a) and D9 (a), with its
// re-check amendments #7 and #14).
//
// ── Why ─────────────────────────────────────────────────────────────────────────────
// The team lead (2026-10-29): "things need to be able to be tuned" — a string lift's gains, speeds and
// limits found ON THE ROBOT, its response watched, the numbers then written into the robot file. Every
// number a mechanism runs with is its spec's (mechanism/spec.hpp), and the service's slots read it —
// but not all of it every tick: the PIDs COPY their gains at bind, the stall detectors their
// signatures, the soft stop its limits in degrees (the brief's P6, landmine 10). A spec changed under a
// slot without telling it changes nothing, or worse, half of the numbers. So live tuning owns the
// table the service runs, changes it ONLY through apply() / undo() / reset(), and every change is
// followed by MechanismService::rederive() — the slot re-reads every copy.
//
// ── How it is wired (the Tune program, TUNE1-B1, is the one caller) ──────────────────
//     static mechanism::LiveTuning tuning{kRobotTwoMechanisms};   // a COPY of the robot file's table
//     service.bind(tuning.table(), devices, clock, faults, sink); // the service runs THE COPY
//     tuning.attach(service);                                     // checked: the service is over it
// The robot file's table is never written (it is a constant): RESET puts its numbers back, bit for bit,
// and no number persists in the robot file — leaving the program or a reboot runs the robot file's
// numbers again; every program but Tune always does. ONE EXCEPTION, in the MOTORS: a current cap
// changed live is written into each motor (MECH-S-A's boot path), and a motor keeps a limit after the
// program that set it ends (believed — HA-220); the next program reads it and says so but, stating no
// cap, writes nothing (D12 (b)). So RESET — or UNDO back to the file's cap — every mechanism whose cap
// changed BEFORE leaving: that writes the robot file's cap back, or the motor's 2.5 A default when it
// states none. Nothing here does it at the program's end (a static object's destructor is no place to
// drive a motor). The match programs never include this header (TUNE1-B1's build gate checks their
// linked symbols).
//
// ── The rules (D8 (a)) ───────────────────────────────────────────────────────────────
// What is live, per kind (parametersOf()): a POSITION mechanism (Arm, Lift) — its ten `.tuning`
// numbers (kP, kI, kD, kS, kV, kA, top speed, acceleration, within, settle), the gravity hold (kG, when
// it has gravity), the stall signature, the homing push and its signature (when it homes by switch or
// by stall), the current cap and the travel limits; a FLYWHEEL — kS, kV, kP, kI, the at-speed band and
// time, the stall signature, the cap (no kA: SpeedTuning cannot say one, so the page cannot either —
// re-check #14); a POWER mechanism — maxVolts, the stall signature (when it states one) and the cap.
// NOT live: presets (a goal finds its preset BY VALUE — P7: change "high" and `find(lift["high"])`
// answers nobody), the gearing, the shape, the homing method and position, a pneumatic's anything.
// A change is APPLIED only when every one of these holds, checked BEFORE the spec is written:
//   1. the changed declaration passes THE BUILD'S OWN CHECK (specRefusal — its sentence, word for word);
//   2. THE LIVE BOUNDS the build lacks (tuningRefusal — P9: the build accepts a NaN kP, a negative kS,
//      kV = 1e9 and a 500 in/s top speed on a lift that turns at most 7.85 in/s): every number finite;
//      kP, kI, kD and kV above 0 and the settle time never 0 — never a value the robot file CANNOT SAY
//      (a `.tuning` 0 means "the starting value"); kS and kA not negative; kV at most twice its starting
//      value (12 V over the output's free speed — kLiveKvMostTimesStarting, an INVENTED library bound);
//      the top speed at most the output's free speed (the starting profile's own scale); a travel limit
//      only NARROWED (never wider than the robot file's — and the build already refuses one past a
//      preset or the home position); the homing push never turned around; a stated cap never removed
//      by a change (UNDO and RESET put it back). Each bound applies to a number that DIFFERS from the
//      robot file's — a number as the file states it is the build's to judge, so a file the build
//      accepted can always be tuned — and a number the file states PAST a bound (a 10 in/s top speed
//      over a 7.85 in/s output) may step back TOWARD it: kS, kA, kV and the top speed are refused only
//      when further past than the file's own number;
//   3. THE SLOT'S STATE: not while the service ticks (from a predicate, a sink — anything a tick calls),
//      not while that mechanism's STEP runs (a change is applied BETWEEN steps), and for the travel
//      limits never past the target it is HOLDING now, never while a held button moves it, and —
//      holding nothing — never past WHERE IT IS (its next hold, a HOME or a let-go, begins where it is
//      clamped inside the limits: a jump) (PositionSlot::takesRange — re-check #7: a hold is never moved,
//      now or at the next hold; the draft's re-clamp asked 153 V) — each refusal one sentence, and nothing
//      changed.
// Each [−] / [+] press is stepped(): a gain ×/÷ 1.25 (FINE ×/÷ 1.05); kA, which starts at 0 and cannot
// leave it by a factor, first steps to kV × 0.01 s (a 10 ms lag's feedforward), then ×/÷ 1.25, and a
// [−] below that first step goes back to 0 (which the file can say); volts and amps ±0.05; a speed, an
// acceleration, a stall's rpm or the at-speed band ×/÷ 1.05 (about 5 %); "within" ±0.05 in or ±0.5 deg;
// a time ±0.05 s; a travel limit ±0.25 in or ±1 deg. A factor's result is rounded to six significant
// figures and a step's to the micro-unit, so the copy's numbers stay short; a step never removes a cap
// (0 A would be NO cap — the motor's full 2.5 A).
//
// ── Undone (D8 (a)) ──────────────────────────────────────────────────────────────────
// undo(): the LAST change (any mechanism's — the sentence names it), up to kLiveUndoDepth (16) back; the
// 17th is refused. reset(i): mechanism `i` back to the robot file's numbers, bit for bit (itself one
// undoable change). Both obey rule 3 (an undo that would narrow a range past a hold, or past where it
// is, waits). Every copy of a spec here is a byte copy, so "bit for bit" is exactly that.
//
// ── Copied back out (D9 (a), re-check #14) ───────────────────────────────────────────
// copyBlock(i, sink) logs ONE block of `[TUNE]` lines (tag "TUNE"): a begin line naming the
// declaration's own file and line, then THE SMALLEST SET of members whose declaration builds to exactly
// the live spec, then an end line with the count. The cascade's (mechanism_live_tuning_rig.hpp's, a file
// with no `.tuning`), after one [+] on kP and a top speed of 2 in/s:
//     'cascade': copy begins -- write these into its declaration (tuned live): <its file>:<its line>
//     .tuning.kP = 0.3125
//     .tuning.maxSpeed = 2
//     .tuning.maxAcceleration = 23.561944901923447
//     'cascade': copy ends -- 3 lines
// Which members — a PATCH, member by member (TUNE1-B2's import must MERGE it, never replace the
// declaration): every member changed live, written WHOLE as the declaration writes it (`.stall = {...}`,
// `.home = homeByStall(...)`, `.gravity = constantGravity(...)`), and no other (`.stall`, `.range`,
// `.home`, `.gravity`, `.currentCap` as the file states them stay as written); and — when any `.tuning`
// number changed live — the WHOLE `.tuning` the declaration then needs (the student replaces
// `.tuning = {...}` with it): every number changed live, then, one at a time in the derivation's order,
// every number whose derivation from the set so far would differ from the live one — found by running
// the file's own derivation (detail::positionStart / speedStart) on the candidate and comparing. So a
// changed top speed carries the acceleration it was tuned with (landmine 12), a number the file states
// that the library would work out differently stays, and an untouched kV stays derived (a stated kV
// would go stale on a cartridge change). A number the file states AT the value the library would work
// out anyway cannot be told from a derived one (the spec holds values, not statements): it is LEFT OUT
// — the block rebuilds the same spec today, but after a cartridge or gearing change that number follows
// the library's derivation. With no `.tuning` number changed, no `.tuning` line: the file's own `.tuning`
// stays as written (no other live number moves a derived one — they hang on the gearing, the cartridge
// and the shape). The `.tuning` lines name each number the C++ way the student writes them
// into `.tuning = {...}`, in its member order (C++ needs designated initializers in that order). Every
// number is printed SHORTEST-ROUND-TRIP (std::to_chars), so it reads back as the same double. Every line
// is at most 200 bytes (TermSink cuts a message there — landmine 16), by construction: a number is at
// most kCopyNumberChars (24) characters, and the longest line — a stall homing one — is 77 fixed bytes
// and five numbers, 197 at most. The cost the team carries (D9): a number a COPY states stays as tuned
// if the cartridge, gearing or spool changes later — RESET and re-tune after such a change. On the
// host the `_s`/`_in`/`_volt` literals read through long double (x87, 80 bits) and then round to
// double; on the V5's ARM a long double IS a double, so a robot file reads each number exactly.
//
// PROS-free, allocation-free. NOTHING HERE HAS RUN ON A ROBOT: the tests drive it over the host test
// plant (its constants invented — HA-183).

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string_view>
#include <system_error>
#include <type_traits>

#include "shulib/core/check.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace shulib::mechanism {

static_assert(std::is_trivially_copyable_v<MechanismSpec> && std::is_trivially_copyable_v<MechanismTable>,
              "live tuning copies specs and tables BYTE FOR BYTE (memcpy): bit-for-bit undo and reset");

/// How many changes UNDO reaches back (D8 (a)): the 17th undo in a row is refused.
inline constexpr std::size_t kLiveUndoDepth = 16;
/// The most a LIVE kV may be, as a multiple of its starting value (12 V over the output's free speed) —
/// an INVENTED library bound (TUNE1-A): it refuses P9's kV = 1e9 and leaves a real motor's slower-than-
/// nominal kV room.
inline constexpr double kLiveKvMostTimesStarting = 2.0;
/// kA's first [+] from 0, as a lag on kV: kA = kV × this, seconds (re-check #14 — a 10 ms lag).
inline constexpr double kLiveFirstKaSeconds = 0.01;
/// The longest number the copy block and a sentence print: std::to_chars' shortest round-trip form of
/// any double is at most 24 characters ("-2.2250738585072014e-308").
inline constexpr std::size_t kCopyNumberChars = 24;

/// ONE LIVE NUMBER — every number some kind lets live tuning change (parametersOf() says which a given
/// mechanism offers). The value is in the DECLARATION's unit: a stall's rpm at the motor (the spec
/// keeps rad/s), a travel limit in degrees or inches.
enum class Tunable : std::uint8_t {
    KP,               ///< kP — a position's or a flywheel's
    KI,               ///< kI — the hold loop's (position) or the speed loop's (flywheel)
    KD,               ///< kD (position)
    KS,               ///< kS, V (position, flywheel)
    KV,               ///< kV (position, flywheel)
    KA,               ///< kA (position ONLY: a flywheel's cannot be stated)
    MaxSpeed,         ///< the smooth move's top speed, units/s (position)
    MaxAcceleration,  ///< its acceleration, units/s² (position)
    Within,           ///< "reached" within this, units (position)
    Settle,           ///< ... for this long, s (position)
    GravityHold,      ///< kG, V (a position mechanism with gravity)
    StallAmps,        ///< the stall signature's amps
    StallRpm,         ///< its "below this motor rpm"
    StallLasting,     ///< its lasting, s
    HomeVolts,        ///< the homing push, V (homing by switch or by stall)
    HomeStallAmps,    ///< the homing signature's amps (homing by stall)
    HomeStallRpm,     ///< its "below this motor rpm"
    HomeStallLasting, ///< its lasting, s
    CurrentCap,       ///< the current cap, A per motor (0: none)
    RangeMin,         ///< the lower travel limit (narrowed only)
    RangeMax,         ///< the upper travel limit (narrowed only)
    AtSpeedBand,      ///< a flywheel's "at speed" band, rpm
    AtSpeedTime,      ///< ... and how long, s
    MaxVolts,         ///< a power mechanism's most volts
};
/// How many Tunable values there are.
inline constexpr std::size_t kTunableCount = 24;

/// How one [−] / [+] press moves a number (D8 (a); header).
enum class StepRule : std::uint8_t {
    Gain,      ///< ×/÷ 1.25, FINE ×/÷ 1.05
    ZeroGain,  ///< kA: from 0 to kV × 0.01 s, then ×/÷ 1.25; below that first step, 0
    Volts,     ///< ±0.05 V
    Amps,      ///< ±0.05 A
    Percent,   ///< ×/÷ 1.05 (about 5 %): a speed, an acceleration, a stall's rpm, the at-speed band
    Reached,   ///< ±0.05 in, or ±0.5 deg
    Seconds,   ///< ±0.05 s
    Limit,     ///< ±0.25 in, or ±1 deg
};

/// ONE live number as the page shows it and the robot file says it.
struct Parameter {
    /// Which number.
    Tunable id = Tunable::KP;
    /// Its name on the page ("kP", "top speed", "stall amps").
    const char* name = "";
    /// How a press moves it.
    StepRule rule = StepRule::Gain;
    /// How the robot file names it (".tuning.kP", ".stall", ".home").
    const char* member = "";
};

/// A mechanism's live numbers, in the page's order (gains first).
struct ParameterList {
    /// The numbers.
    std::array<Parameter, kTunableCount> items{};
    /// How many are real.
    std::size_t count = 0;
};

/// Why live tuning refused a change (each with its one sentence, liveRefusalText()).
enum class LiveRefusal : std::uint8_t {
    None = 0,             ///< applied
    NotLive,              ///< not a live number of this mechanism
    BuildRefuses,         ///< the build's own check refuses the changed declaration (its sentence)
    NotANumber,           ///< a number that is not finite
    GainNotAbove0,        ///< kP, kI, kD or kV at or below 0
    FeedforwardNegative,  ///< kS or kA below 0
    SettleIs0,            ///< a settle time of exactly 0
    KvPastTwiceStarting,  ///< kV past kLiveKvMostTimesStarting × its starting value
    TopSpeedPastFree,     ///< the top speed past the output's free speed
    RangeWidened,         ///< a travel limit wider than the robot file's
    HomingPushTurned,     ///< the homing push turned around
    CapRemoved,           ///< a stated cap removed by a change
    StepRunning,          ///< a step runs on it: between steps only
    InsideATick,          ///< the service is ticking: between ticks only
    LeavesOutHeldTarget,  ///< the new limits leave out the target it holds
    LeavesOutWhereItIs,   ///< holding nothing, the new limits leave where it is further outside them
    HeldButtonMoving,     ///< a held button moves it while the limits change
    NothingToUndo,        ///< undo() with no change left to undo
};

/// The SENTENCE for a live refusal (for BuildRefuses: refusalText(specRefusal(...)) says it); nullptr
/// for None.
[[nodiscard]] constexpr const char* liveRefusalText(LiveRefusal r) noexcept {
    switch (r) {
        case LiveRefusal::None:
            return nullptr;
        case LiveRefusal::NotLive:
            return "it is not a live number of this mechanism (the robot file's to change)";
        case LiveRefusal::BuildRefuses:
            return "the build would refuse the changed declaration";
        case LiveRefusal::NotANumber:
            return "every number must be a finite number";
        case LiveRefusal::GainNotAbove0:
            return "kP, kI, kD and kV must be above 0 (the robot file cannot say 0: a 0 there means the "
                   "starting value)";
        case LiveRefusal::FeedforwardNegative:
            return "kS and kA must not be negative";
        case LiveRefusal::SettleIs0:
            return "the settle time cannot be 0 (the robot file cannot say 0: a 0 there means the starting "
                   "value, 0.1 s)";
        case LiveRefusal::KvPastTwiceStarting:
            return "kV is past twice its starting value (12 V over the output's free speed)";
        case LiveRefusal::TopSpeedPastFree:
            return "the top speed is past the output's free speed (its cartridge at 12 V through the gearing)";
        case LiveRefusal::RangeWidened:
            return "a travel limit may only be narrowed -- never wider than the robot file's";
        case LiveRefusal::HomingPushTurned:
            return "the homing push cannot turn around live (it must keep pushing toward its home end)";
        case LiveRefusal::CapRemoved:
            return "a stated current cap cannot be removed by a change (0 A would be no cap: the motor's full "
                   "2.5 A) -- UNDO or RESET puts the robot file's back";
        case LiveRefusal::StepRunning:
            return "a step is running on it: a change waits until the step ends";
        case LiveRefusal::InsideATick:
            return "the service is ticking: a change is applied between ticks";
        case LiveRefusal::LeavesOutHeldTarget:
            return "the new travel limits leave out the target it is holding: move it inside first";
        case LiveRefusal::LeavesOutWhereItIs:
            return "the new travel limits leave out where it is (its next hold would jump to the new limit): move "
                   "it inside first";
        case LiveRefusal::HeldButtonMoving:
            return "a held button is moving it: let go before changing a travel limit";
        case LiveRefusal::NothingToUndo:
            return "nothing to undo";
    }
    return "an unknown refusal";
}

namespace detail {
inline constexpr double kLivePi = 3.14159265358979323846;

/// A stall's motor rpm as the spec keeps it, rad/s — kinds.hpp's stallSpec(), the SAME expression in
/// the same order, so a number set here reads back from a declaration bit for bit.
[[nodiscard]] constexpr double liveRadS(double rpm) noexcept { return rpm * 2.0 * 3.14159265358979323846 / 60.0; }

/// `v` in its SHORTEST round-trip form (std::to_chars; at most kCopyNumberChars characters).
struct ShortNumber {
    /// The text, NUL-terminated.
    char text[kCopyNumberChars + 8]{};
};
[[nodiscard]] inline ShortNumber shortest(double v) noexcept {
    ShortNumber n{};
    const std::to_chars_result r = std::to_chars(n.text, n.text + kCopyNumberChars + 1, v);
    if (r.ec != std::errc{}) {
        n.text[0] = '?';
        n.text[1] = '\0';
        return n;
    }
    *r.ptr = '\0';
    return n;
}
/// The motor rpm whose liveRadS() is exactly `radS`, the SHORTEST such one when several are (a spec's
/// rad/s back to the declaration's rpm: 5 rpm, not 5.000000000000001). Searched within a few ulps of
/// the plain inverse; the plain inverse when none is exact.
[[nodiscard]] inline double liveRpm(double radS) noexcept {
    const double guess = radS * 60.0 / (2.0 * kLivePi);
    if (!std::isfinite(guess)) {
        return guess;
    }
    double c = guess;
    for (int k = 0; k < 8; ++k) {
        c = std::nextafter(c, -std::numeric_limits<double>::infinity());
    }
    double best = guess;
    std::size_t bestLen = 1000;
    for (int k = 0; k < 17; ++k) {
        if (liveRadS(c) == radS) {
            const std::size_t len = std::strlen(shortest(c).text);
            if (len < bestLen) {
                bestLen = len;
                best = c;
            }
        }
        c = std::nextafter(c, std::numeric_limits<double>::infinity());
    }
    return best;
}
/// 10^k for 0 <= k <= 22, exact.
[[nodiscard]] constexpr double livePow10(int k) noexcept {
    double p = 1.0;
    for (int i = 0; i < k; ++i) {
        p *= 10.0;
    }
    return p;
}
/// `v` rounded to six significant figures (a factor step's result — header).
[[nodiscard]] inline double roundSix(double v) noexcept {
    if (!std::isfinite(v) || v == 0.0) {
        return v;
    }
    const int k = 5 - static_cast<int>(std::floor(std::log10(std::abs(v))));
    if (k >= 0 && k <= 22) {
        const double p = livePow10(k);
        return std::round(v * p) / p;
    }
    if (k < 0 && k >= -22) {
        const double p = livePow10(-k);
        return std::round(v / p) * p;
    }
    return v;
}
/// `v` rounded to the micro-unit (an additive step's result — header).
[[nodiscard]] inline double roundMicro(double v) noexcept {
    return std::isfinite(v) ? std::round(v * 1e6) / 1e6 + 0.0 : v;
}
/// The stall signature a kind watches its behaviour with (nullptr for a pneumatic).
[[nodiscard]] inline StallSpec* liveStall(MechanismSpec& s) noexcept {
    switch (s.kind) {
        case Kind::Power:
            return &s.power.stall;
        case Kind::Position:
            return &s.position.stall;
        case Kind::Speed:
            return &s.speed.stall;
        case Kind::Pneumatic:
            return nullptr;
    }
    return nullptr;
}
/// The same, read-only.
[[nodiscard]] inline const StallSpec* liveStall(const MechanismSpec& s) noexcept {
    return liveStall(const_cast<MechanismSpec&>(s));  // NOLINT: reads only
}
/// The number `t` of `s` as a mutable reference, or nullptr where `s`'s kind has none (the stall's rpm
/// is NOT here: the spec keeps rad/s — tunableValue() / setTunable() convert).
[[nodiscard]] inline double* liveField(MechanismSpec& s, Tunable t) noexcept {
    const bool pos = s.kind == Kind::Position;
    const bool spd = s.kind == Kind::Speed;
    StallSpec* st = liveStall(s);
    switch (t) {
        case Tunable::KP:
            return pos ? &s.position.kP : spd ? &s.speed.kP : nullptr;
        case Tunable::KI:
            return pos ? &s.position.kI : spd ? &s.speed.kI : nullptr;
        case Tunable::KD:
            return pos ? &s.position.kD : nullptr;
        case Tunable::KS:
            return pos ? &s.position.kS : spd ? &s.speed.kS : nullptr;
        case Tunable::KV:
            return pos ? &s.position.kV : spd ? &s.speed.kV : nullptr;
        case Tunable::KA:
            return pos ? &s.position.kA : nullptr;
        case Tunable::MaxSpeed:
            return pos ? &s.position.maxVelocity : nullptr;
        case Tunable::MaxAcceleration:
            return pos ? &s.position.maxAcceleration : nullptr;
        case Tunable::Within:
            return pos ? &s.position.tolerance : nullptr;
        case Tunable::Settle:
            return pos ? &s.position.settleTime : nullptr;
        case Tunable::GravityHold:
            return pos ? &s.position.kG : nullptr;
        case Tunable::StallAmps:
            return st != nullptr ? &st->amps : nullptr;
        case Tunable::StallRpm:
            return nullptr;
        case Tunable::StallLasting:
            return st != nullptr ? &st->time : nullptr;
        case Tunable::HomeVolts:
            return pos ? &s.position.homeVolts : nullptr;
        case Tunable::HomeStallAmps:
            return pos ? &s.position.homeStall.amps : nullptr;
        case Tunable::HomeStallRpm:
            return nullptr;
        case Tunable::HomeStallLasting:
            return pos ? &s.position.homeStall.time : nullptr;
        case Tunable::CurrentCap:
            return s.kind != Kind::Pneumatic ? &s.currentCapAmps : nullptr;
        case Tunable::RangeMin:
            return pos ? &s.position.minPosition : nullptr;
        case Tunable::RangeMax:
            return pos ? &s.position.maxPosition : nullptr;
        case Tunable::AtSpeedBand:
            return spd ? &s.speed.atSpeedBandRpm : nullptr;
        case Tunable::AtSpeedTime:
            return spd ? &s.speed.atSpeedTime : nullptr;
        case Tunable::MaxVolts:
            return s.kind == Kind::Power ? &s.power.maxVolts : nullptr;
    }
    return nullptr;
}
/// A position spec's output free speed, deg/s (its cartridge at 12 V through the gearing — kinds.hpp's
/// positionStart() scale).
[[nodiscard]] inline double liveFreeDegS(const MechanismSpec& s) noexcept {
    return s.position.gearRatio > 0.0 ? cartridgeRpm(s.cartridge) * 6.0 / s.position.gearRatio : 0.0;
}
}  // namespace detail

/// Whether mechanism `s` lets live tuning change `t` (header: what is live, per kind).
[[nodiscard]] inline bool offered(const MechanismSpec& s, Tunable t) noexcept {
    switch (s.kind) {
        case Kind::Power:
            if (t == Tunable::StallAmps || t == Tunable::StallRpm || t == Tunable::StallLasting) {
                return s.power.stall.stated();  // a signature from nothing needs all three at once
            }
            return t == Tunable::CurrentCap || t == Tunable::MaxVolts;
        case Kind::Position: {
            const PositionSpec& p = s.position;
            switch (t) {
                case Tunable::GravityHold:
                    return p.gravity != Gravity::None;  // noGravity cannot say a hold
                case Tunable::HomeVolts:
                    return p.home == HomeMethod::Switch || p.home == HomeMethod::Stall;
                case Tunable::HomeStallAmps:
                case Tunable::HomeStallRpm:
                case Tunable::HomeStallLasting:
                    return p.home == HomeMethod::Stall;
                case Tunable::AtSpeedBand:
                case Tunable::AtSpeedTime:
                case Tunable::MaxVolts:
                    return false;
                default:
                    return true;
            }
        }
        case Kind::Speed:
            switch (t) {
                case Tunable::KP:
                case Tunable::KI:
                case Tunable::KS:
                case Tunable::KV:
                case Tunable::AtSpeedBand:
                case Tunable::AtSpeedTime:
                case Tunable::StallAmps:
                case Tunable::StallRpm:
                case Tunable::StallLasting:
                case Tunable::CurrentCap:
                    return true;
                default:
                    return false;
            }
        case Kind::Pneumatic:
            return false;
    }
    return false;
}

/// The number `t` of `s`, in the declaration's unit (a stall's rpm at the motor — the shortest one that
/// is exactly the spec's rad/s); 0 where `s`'s kind has none.
[[nodiscard]] inline double tunableValue(const MechanismSpec& s, Tunable t) noexcept {
    if (t == Tunable::StallRpm) {
        const StallSpec* st = detail::liveStall(s);
        return st != nullptr ? detail::liveRpm(st->speedRadS) : 0.0;
    }
    if (t == Tunable::HomeStallRpm) {
        return s.kind == Kind::Position ? detail::liveRpm(s.position.homeStall.speedRadS) : 0.0;
    }
    const double* f = detail::liveField(const_cast<MechanismSpec&>(s), t);  // NOLINT: reads only
    return f != nullptr ? *f : 0.0;
}
/// Set the number `t` of `s` to `v` (the declaration's unit; a stall's rpm as kinds.hpp turns it into
/// rad/s). Nothing for a number `s`'s kind does not have. No rule is checked here (tuningRefusal()).
inline void setTunable(MechanismSpec& s, Tunable t, double v) noexcept {
    if (t == Tunable::StallRpm) {
        if (StallSpec* st = detail::liveStall(s); st != nullptr) {
            st->speedRadS = detail::liveRadS(v);
        }
        return;
    }
    if (t == Tunable::HomeStallRpm) {
        if (s.kind == Kind::Position) {
            s.position.homeStall.speedRadS = detail::liveRadS(v);
        }
        return;
    }
    if (double* f = detail::liveField(s, t); f != nullptr) {
        *f = v;
    }
}

/// The unit `t` of `s` is shown in ("V/deg", "in/s", "A per motor").
[[nodiscard]] inline const char* tunableUnit(const MechanismSpec& s, Tunable t) noexcept {
    const bool inches = s.kind == Kind::Position && s.position.units == Units::Inches;
    const bool spd = s.kind == Kind::Speed;
    switch (t) {
        case Tunable::KP:
            return spd ? "V per rad/s" : "V/deg";
        case Tunable::KI:
            return spd ? "V per rad" : "V/(deg*s)";
        case Tunable::KD:
            return "V/(deg/s)";
        case Tunable::KS:
        case Tunable::GravityHold:
        case Tunable::HomeVolts:
        case Tunable::MaxVolts:
            return "V";
        case Tunable::KV:
            return spd ? "V per rad/s" : "V/(deg/s)";
        case Tunable::KA:
            return "V/(deg/s^2)";
        case Tunable::MaxSpeed:
            return inches ? "in/s" : "deg/s";
        case Tunable::MaxAcceleration:
            return inches ? "in/s^2" : "deg/s^2";
        case Tunable::Within:
        case Tunable::RangeMin:
        case Tunable::RangeMax:
            return inches ? "in" : "deg";
        case Tunable::Settle:
        case Tunable::StallLasting:
        case Tunable::HomeStallLasting:
        case Tunable::AtSpeedTime:
            return "s";
        case Tunable::StallAmps:
        case Tunable::HomeStallAmps:
            return "A";
        case Tunable::StallRpm:
        case Tunable::HomeStallRpm:
            return "motor rpm";
        case Tunable::CurrentCap:
            return "A per motor";
        case Tunable::AtSpeedBand:
            return "rpm";
    }
    return "";
}

namespace detail {
/// Every parameter, in the PAGE's order (gains first), with its name, rule and member.
inline constexpr std::array<Parameter, kTunableCount> kLiveParameters{{
    {Tunable::KP, "kP", StepRule::Gain, ".tuning.kP"},
    {Tunable::KI, "kI", StepRule::Gain, ".tuning.kI"},
    {Tunable::KD, "kD", StepRule::Gain, ".tuning.kD"},
    {Tunable::KS, "kS", StepRule::Volts, ".tuning.kS"},
    {Tunable::KV, "kV", StepRule::Gain, ".tuning.kV"},
    {Tunable::KA, "kA", StepRule::ZeroGain, ".tuning.kA"},
    {Tunable::MaxSpeed, "top speed", StepRule::Percent, ".tuning.maxSpeed"},
    {Tunable::MaxAcceleration, "acceleration", StepRule::Percent, ".tuning.maxAcceleration"},
    {Tunable::Within, "within", StepRule::Reached, ".tuning.within"},
    {Tunable::Settle, "settle", StepRule::Seconds, ".tuning.settle"},
    {Tunable::GravityHold, "gravity hold", StepRule::Volts, ".gravity"},
    {Tunable::MaxVolts, "max volts", StepRule::Volts, ".maxVolts"},
    {Tunable::AtSpeedBand, "at-speed band", StepRule::Percent, ".atSpeed"},
    {Tunable::AtSpeedTime, "at-speed time", StepRule::Seconds, ".atSpeed"},
    {Tunable::StallAmps, "stall amps", StepRule::Amps, ".stall"},
    {Tunable::StallRpm, "stall rpm", StepRule::Percent, ".stall"},
    {Tunable::StallLasting, "stall lasting", StepRule::Seconds, ".stall"},
    {Tunable::HomeVolts, "homing push", StepRule::Volts, ".home"},
    {Tunable::HomeStallAmps, "homing amps", StepRule::Amps, ".home"},
    {Tunable::HomeStallRpm, "homing rpm", StepRule::Percent, ".home"},
    {Tunable::HomeStallLasting, "homing lasting", StepRule::Seconds, ".home"},
    {Tunable::CurrentCap, "current cap", StepRule::Amps, ".currentCap"},
    {Tunable::RangeMin, "lower limit", StepRule::Limit, ".range"},
    {Tunable::RangeMax, "upper limit", StepRule::Limit, ".range"},
}};
}  // namespace detail

/// The descriptor of `t` (its page name, step rule and member).
[[nodiscard]] inline const Parameter& parameterOf(Tunable t) noexcept {
    for (const Parameter& p : detail::kLiveParameters) {
        if (p.id == t) {
            return p;
        }
    }
    return detail::kLiveParameters[0];
}
/// The live numbers mechanism `s` offers, in the page's order (header: what is live, per kind; a
/// pneumatic offers none).
[[nodiscard]] inline ParameterList parametersOf(const MechanismSpec& s) noexcept {
    ParameterList out{};
    for (const Parameter& p : detail::kLiveParameters) {
        if (offered(s, p.id)) {
            out.items[out.count++] = p;
        }
    }
    return out;
}

/// THE LIVE RULES on a whole spec (header, rules 1 and 2): the build's own check FIRST (BuildRefuses —
/// specRefusal(tuned) names which), then the live bounds the build lacks, each applied to a number that
/// differs from `declared` (the robot file's) — and kS, kA, kV and the top speed only when that number is
/// further past its bound than `declared`'s own (a file's number past one may step back toward it).
/// Rule 3, the slot's state, is LiveTuning::apply()'s.
/// NotLive when the two are not one mechanism's kind.
[[nodiscard]] inline LiveRefusal tuningRefusal(const MechanismSpec& declared, const MechanismSpec& tuned) noexcept {
    if (declared.kind != tuned.kind) {
        return LiveRefusal::NotLive;
    }
    if (specRefusal(tuned) != Refusal::None) {
        return LiveRefusal::BuildRefuses;
    }
    const auto differs = [&](Tunable t) {
        const double a = tunableValue(declared, t);
        const double b = tunableValue(tuned, t);
        return !(a == b) || std::signbit(a) != std::signbit(b);
    };
    const auto isLive = [&](Tunable t) { return offered(tuned, t) && differs(t); };
    // A number the robot file states PAST a bound may step back TOWARD it: a bound refuses a live number
    // only when it is also further past than the file's own (TUNE1-A's review — else every [−] on a file's
    // 10 in/s top speed over a 7.85 in/s output would be refused, and the page has only [−] and [+]).
    const auto pastAbove = [&](Tunable t, double bound) {
        const double v = tunableValue(tuned, t);
        return v > bound && v > tunableValue(declared, t);
    };
    const auto pastBelow0 = [&](Tunable t) {
        const double v = tunableValue(tuned, t);
        return v < 0.0 && v < tunableValue(declared, t);
    };
    for (const Parameter& p : detail::kLiveParameters) {
        if (isLive(p.id) && !std::isfinite(tunableValue(tuned, p.id))) {
            return LiveRefusal::NotANumber;
        }
    }
    for (const Tunable t : {Tunable::KP, Tunable::KI, Tunable::KD, Tunable::KV}) {
        if (isLive(t) && !(tunableValue(tuned, t) > 0.0)) {
            return LiveRefusal::GainNotAbove0;
        }
    }
    for (const Tunable t : {Tunable::KS, Tunable::KA}) {
        if (isLive(t) && pastBelow0(t)) {
            return LiveRefusal::FeedforwardNegative;
        }
    }
    if (isLive(Tunable::Settle) && tunableValue(tuned, Tunable::Settle) == 0.0) {
        return LiveRefusal::SettleIs0;
    }
    if (isLive(Tunable::KV)) {
        const double startKv = tuned.kind == Kind::Position ? positionStartingValues(declared).kV
                                                            : speedStartingValues(declared).kV;
        if (pastAbove(Tunable::KV, kLiveKvMostTimesStarting * startKv)) {
            return LiveRefusal::KvPastTwiceStarting;
        }
    }
    if (isLive(Tunable::MaxSpeed) &&
        pastAbove(Tunable::MaxSpeed, detail::liveFreeDegS(tuned) * detail::unitsPerOutputDeg(tuned.position))) {
        return LiveRefusal::TopSpeedPastFree;
    }
    if ((isLive(Tunable::RangeMin) && tuned.position.minPosition < declared.position.minPosition) ||
        (isLive(Tunable::RangeMax) && tuned.position.maxPosition > declared.position.maxPosition)) {
        return LiveRefusal::RangeWidened;
    }
    if (isLive(Tunable::HomeVolts) &&
        std::signbit(tuned.position.homeVolts) != std::signbit(declared.position.homeVolts)) {
        return LiveRefusal::HomingPushTurned;
    }
    if (isLive(Tunable::CurrentCap) && tuned.currentCapAmps == 0.0 && declared.currentCapAmps != 0.0) {
        return LiveRefusal::CapRemoved;
    }
    return LiveRefusal::None;
}
/// tuningRefusal() as a SENTENCE (nullptr: allowed) — for BuildRefuses, the build's own words.
[[nodiscard]] inline const char* tuningProblem(const MechanismSpec& declared, const MechanismSpec& tuned) noexcept {
    const LiveRefusal r = tuningRefusal(declared, tuned);
    return r == LiveRefusal::BuildRefuses ? refusalText(specRefusal(tuned)) : liveRefusalText(r);
}

/// ONE [−] (`direction` < 0) or [+] (> 0) press on number `t` of `s` (header: the step rules; `fine`
/// makes a gain's factor 1.05). The NEXT value — nothing is applied. Unchanged for a number `s` does
/// not offer, or for `direction` 0.
[[nodiscard]] inline double steppedValue(const MechanismSpec& s, Tunable t, int direction, bool fine = false) noexcept {
    const double v = tunableValue(s, t);
    if (!offered(s, t) || direction == 0 || !std::isfinite(v)) {
        return v;
    }
    const double dir = direction > 0 ? 1.0 : -1.0;
    const bool inches = s.kind == Kind::Position && s.position.units == Units::Inches;
    const double factor = fine ? 1.05 : 1.25;
    switch (parameterOf(t).rule) {
        case StepRule::Gain:
            return detail::roundSix(dir > 0.0 ? v * factor : v / factor);
        case StepRule::ZeroGain: {
            const double first = detail::roundSix(tunableValue(s, Tunable::KV) * kLiveFirstKaSeconds);
            if (v == 0.0) {
                return dir > 0.0 ? first : 0.0;
            }
            if (dir > 0.0) {
                return detail::roundSix(v * factor);
            }
            const double down = detail::roundSix(v / factor);
            return down < first ? 0.0 : down;  // below the first step: back to 0, which the file can say
        }
        case StepRule::Volts:
            return detail::roundMicro(v + dir * 0.05);
        case StepRule::Amps: {
            const double from = t == Tunable::CurrentCap && v == 0.0 ? 2.5 : v;  // no cap reads as the motor's 2.5 A
            const double next = detail::roundMicro(from + dir * 0.05);
            return t == Tunable::CurrentCap && next < 0.001 ? v : next;  // a step never removes a cap
        }
        case StepRule::Percent:
            return detail::roundSix(dir > 0.0 ? v * 1.05 : v / 1.05);
        case StepRule::Reached:
            return detail::roundMicro(v + dir * (inches ? 0.05 : 0.5));
        case StepRule::Seconds:
            return detail::roundMicro(v + dir * 0.05);
        case StepRule::Limit:
            return detail::roundMicro(v + dir * (inches ? 0.25 : 1.0));
    }
    return v;
}

/// What one change, undo or reset did.
struct Applied {
    /// True when applied (or when nothing needed to change).
    bool ok = false;
    /// Why it was refused (None when ok).
    LiveRefusal why = LiveRefusal::None;
    /// For BuildRefuses: the build's own reason.
    Refusal build = Refusal::None;
    /// One sentence for the page: "'lift': kP 0.25 -> 0.3125", or the refusal and why.
    char sentence[320]{};
};

/// LIVE TUNING (header): the robot's mechanism table as a mutable COPY the service runs, the robot
/// file's own table for "narrowed only" and RESET, the changes, undo and the copy block. Lives in the
/// program's static storage (the service's slots point into it); non-copyable and non-movable.
class LiveTuning {
public:
    /// A copy of `declared` (the robot file's table — a constant that outlives this object).
    explicit LiveTuning(const MechanismTable& declared) noexcept : declared_{&declared} {
        std::memcpy(static_cast<void*>(&live_), &declared, sizeof live_);
    }
    /// Pinned: the service's slots point into its table.
    LiveTuning(const LiveTuning&) = delete;
    /// Non-copy-assignable (the copy constructor's reason).
    LiveTuning& operator=(const LiveTuning&) = delete;
    /// Non-movable (the copy constructor's reason).
    LiveTuning(LiveTuning&&) = delete;
    /// Non-move-assignable (the copy constructor's reason).
    LiveTuning& operator=(LiveTuning&&) = delete;
    /// Nothing to release (the service it is attached to is the program's).
    ~LiveTuning() = default;

    /// THE TABLE THE SERVICE RUNS — bind the service over this, then attach() it.
    [[nodiscard]] const MechanismTable& table() const noexcept { return live_; }
    /// The robot file's table (never written).
    [[nodiscard]] const MechanismTable& declared() const noexcept { return *declared_; }
    /// How many mechanisms.
    [[nodiscard]] std::size_t count() const noexcept { return live_.count < kMaxMechanisms ? live_.count : kMaxMechanisms; }
    /// Mechanism `i`'s live spec. PRECONDITION: `i` below count().
    [[nodiscard]] const MechanismSpec& spec(std::size_t i) const {
        SHULIB_PRECONDITION(i < count(), "LiveTuning::spec: no mechanism at this index");
        return live_.specs[i];
    }
    /// The service that runs this table. PRECONDITIONS: `service` bound over table() (each of its slots'
    /// spec IS this table's — landmine 10: a change here must reach the slots); attached once.
    void attach(MechanismService& service) {
        SHULIB_PRECONDITION(service_ == nullptr, "LiveTuning::attach: attached twice");
        SHULIB_PRECONDITION(service.bound() && service.size() == count(),
                            "LiveTuning::attach: the service is not bound over this table (bind it over table())");
        for (std::size_t i = 0; i < count(); ++i) {
            const Slot* s = service.slotPtr(i);
            SHULIB_PRECONDITION(s != nullptr && &s->spec() == &live_.specs[i],
                                "LiveTuning::attach: the service is not bound over this table (bind it over table())");
        }
        service_ = &service;
    }
    /// True once attach() has run.
    [[nodiscard]] bool attached() const noexcept { return service_ != nullptr; }
    /// True when mechanism `i`'s live numbers differ from the robot file's (byte for byte).
    [[nodiscard]] bool changed(std::size_t i) const {
        SHULIB_PRECONDITION(i < count(), "LiveTuning::changed: no mechanism at this index");
        return std::memcmp(&live_.specs[i], &declared_->specs[i], sizeof(MechanismSpec)) != 0;
    }
    /// Mechanism `i`'s live number `t` (tunableValue()).
    [[nodiscard]] double value(std::size_t i, Tunable t) const { return tunableValue(spec(i), t); }
    /// The value one press would set (steppedValue()); nothing applied.
    [[nodiscard]] double stepped(std::size_t i, Tunable t, int direction, bool fine = false) const {
        return steppedValue(spec(i), t, direction, fine);
    }
    /// How many changes undo() can still reach (at most kLiveUndoDepth).
    [[nodiscard]] std::size_t undoDepth() const noexcept { return undoCount_; }

    /// CHANGE mechanism `i`'s number `t` to `v` (the declaration's unit), inside the rules (header, rules
    /// 1–3, checked BEFORE the spec is written): applied, re-derived by the service (its one Info line)
    /// and undoable — or refused with its sentence and NOTHING changed. Setting a number to what it is
    /// already is ok and changes nothing. PRECONDITIONS: attached; `i` below count().
    Applied apply(std::size_t i, Tunable t, double v) {
        SHULIB_PRECONDITION(attached(), "LiveTuning::apply: attach() the service first");
        SHULIB_PRECONDITION(i < count(), "LiveTuning::apply: no mechanism at this index");
        const MechanismSpec& now = live_.specs[i];
        const double was = tunableValue(now, t);
        Applied a{};
        if (!offered(now, t)) {
            return refuse(a, i, t, was, v, LiveRefusal::NotLive);
        }
        MechanismSpec candidate;
        std::memcpy(static_cast<void*>(&candidate), &now, sizeof candidate);
        setTunable(candidate, t, v);
        if (std::memcmp(&candidate, &now, sizeof candidate) == 0) {
            a.ok = true;
            const detail::ShortNumber n = detail::shortest(was);
            std::snprintf(a.sentence, sizeof a.sentence, "'%.23s': %.16s unchanged (%.24s)", now.name,
                          parameterOf(t).name, n.text);
            return a;
        }
        const LiveRefusal r = tuningRefusal(declared_->specs[i], candidate);
        if (r != LiveRefusal::None) {
            a.build = r == LiveRefusal::BuildRefuses ? specRefusal(candidate) : Refusal::None;
            return refuse(a, i, t, was, v, r);
        }
        if (const LiveRefusal s = stateRefusal(i, candidate); s != LiveRefusal::None) {
            return refuse(a, i, t, was, v, s);
        }
        char what[96];
        whatChanged(what, sizeof what, t, was, v);
        return commit(a, i, candidate, Record::Made::Change, t, what);
    }
    /// One [−] (`direction` < 0) or [+] (> 0) press: apply(i, t, stepped(i, t, direction, fine)).
    Applied step(std::size_t i, Tunable t, int direction, bool fine = false) {
        return apply(i, t, stepped(i, t, direction, fine));
    }
    /// UNDO the last change (any mechanism's — the sentence names it), up to kLiveUndoDepth back: that
    /// mechanism's numbers before it, bit for bit, re-derived. Refused — and kept, to undo later — while
    /// that mechanism's step runs, inside a tick, or when it would narrow a range past a hold (rule 3);
    /// refused with nothing left to undo. PRECONDITION: attached.
    Applied undo() {
        SHULIB_PRECONDITION(attached(), "LiveTuning::undo: attach() the service first");
        Applied a{};
        if (undoCount_ == 0) {
            a.why = LiveRefusal::NothingToUndo;
            std::snprintf(a.sentence, sizeof a.sentence, "%s", liveRefusalText(a.why));
            return a;
        }
        const std::size_t top = (undoStart_ + undoCount_ - 1) % kLiveUndoDepth;
        const Record& rec = undo_[top];
        const std::size_t i = rec.index;
        if (const LiveRefusal s = stateRefusal(i, rec.before); s != LiveRefusal::None) {
            a.why = s;
            std::snprintf(a.sentence, sizeof a.sentence, "'%.23s': undo refused -- %.180s", live_.specs[i].name,
                          liveRefusalText(s));
            return a;
        }
        char what[112];
        if (rec.kind == Record::Made::Reset) {
            std::snprintf(what, sizeof what, "undo: its RESET (the numbers before it are back)");
        } else {
            char change[96];
            whatChanged(change, sizeof change, rec.what, tunableValue(live_.specs[i], rec.what),
                        tunableValue(rec.before, rec.what));
            std::snprintf(what, sizeof what, "undo: %.100s", change);
        }
        MechanismSpec before;
        std::memcpy(static_cast<void*>(&before), &rec.before, sizeof before);
        --undoCount_;  // popped first: commit() pushes nothing for an undo
        return commit(a, i, before, Record::Made::Undo, rec.what, what);
    }
    /// RESET mechanism `i` to the robot file's numbers, bit for bit, re-derived (one undoable change).
    /// Refused while its step runs or inside a tick (rule 3; the robot file's range — never narrower than
    /// the live one — always holds a hold the live range held, and never leaves where it is further out).
    /// PRECONDITIONS: attached; `i` below count().
    Applied reset(std::size_t i) {
        SHULIB_PRECONDITION(attached(), "LiveTuning::reset: attach() the service first");
        SHULIB_PRECONDITION(i < count(), "LiveTuning::reset: no mechanism at this index");
        Applied a{};
        if (!changed(i)) {
            a.ok = true;
            std::snprintf(a.sentence, sizeof a.sentence, "'%.23s': already the robot file's numbers",
                          live_.specs[i].name);
            return a;
        }
        if (const LiveRefusal s = stateRefusal(i, declared_->specs[i]); s != LiveRefusal::None) {
            a.why = s;
            std::snprintf(a.sentence, sizeof a.sentence, "'%.23s': RESET refused -- %.180s", live_.specs[i].name,
                          liveRefusalText(s));
            return a;
        }
        MechanismSpec file;
        std::memcpy(static_cast<void*>(&file), &declared_->specs[i], sizeof file);
        return commit(a, i, file, Record::Made::Reset, Tunable::KP, "RESET: the robot file's numbers");
    }

    /// COPY (D9 (a), header): mechanism `i`'s block of `[TUNE]` lines into `sink` (tag "TUNE", Info) — a
    /// begin line, the SMALLEST set of members whose declaration builds to exactly the live spec, and
    /// an end line with their count. Returns that count (0 for an untuned one: the block says so with
    /// no member line). PRECONDITION: `i` below count().
    int copyBlock(std::size_t i, hal::ITelemetrySink& sink) const {
        SHULIB_PRECONDITION(i < count(), "LiveTuning::copyBlock: no mechanism at this index");
        const MechanismSpec& s = live_.specs[i];
        int n = 0;
        forEachCopyLine(i, [&n](const char*) { ++n; });
        char line[240];
        const char* file = s.declaredFile != nullptr ? s.declaredFile : "its robot file";
        const std::size_t len = std::strlen(file);
        std::snprintf(line, sizeof line, "'%.23s': copy begins -- write these into its declaration (tuned live): %.60s:%u",
                      s.name, file + (len > 60 ? len - 60 : 0), static_cast<unsigned>(s.declaredLine));
        sink.log(hal::LogLevel::Info, "TUNE", line);
        forEachCopyLine(i, [&sink](const char* text) { sink.log(hal::LogLevel::Info, "TUNE", text); });
        std::snprintf(line, sizeof line, "'%.23s': copy ends -- %d line%s", s.name, n, n == 1 ? "" : "s");
        sink.log(hal::LogLevel::Info, "TUNE", line);
        return n;
    }

private:
    /// One undo record: which mechanism, what it was before, and what the change was.
    struct Record {
        /// What made it.
        enum class Made : std::uint8_t { Change, Reset, Undo };
        MechanismSpec before;  ///< the mechanism's spec before the change, byte for byte
        std::uint8_t index;    ///< which mechanism
        Made kind;             ///< a change or a reset
        Tunable what;          ///< a change's number
    };

    /// Rule 3 — the slot's state (header): a tick running, the mechanism's step running, or (a position
    /// mechanism) `candidate`'s travel limits leaving out its hold, changing under a held button, or —
    /// holding nothing — leaving where it is further outside them.
    [[nodiscard]] LiveRefusal stateRefusal(std::size_t i, const MechanismSpec& candidate) const {
        if (!service_->canRederive(i)) {
            const Slot* s = service_->slotPtr(i);
            return s != nullptr && s->running() ? LiveRefusal::StepRunning : LiveRefusal::InsideATick;
        }
        if (candidate.kind == Kind::Position) {
            const PositionSlot& p = service_->position(i);
            if (!p.takesRange(candidate.position)) {
                return p.holding()            ? LiveRefusal::LeavesOutHeldTarget
                       : p.heldButtonMoving() ? LiveRefusal::HeldButtonMoving
                                              : LiveRefusal::LeavesOutWhereItIs;
            }
        }
        return LiveRefusal::None;
    }
    /// Why the SERVICE refused a re-derive stateRefusal() had let through (commit()): the same questions
    /// from the slot's state alone — no second sensor read decides it. A step or a tick when it cannot
    /// re-derive; else (only a position slot refuses) the hold, the held button, or where it is.
    [[nodiscard]] LiveRefusal lateRefusal(std::size_t i) const {
        if (!service_->canRederive(i)) {
            const Slot* s = service_->slotPtr(i);
            return s != nullptr && s->running() ? LiveRefusal::StepRunning : LiveRefusal::InsideATick;
        }
        const PositionSlot& p = service_->position(i);
        return p.holding()            ? LiveRefusal::LeavesOutHeldTarget
               : p.heldButtonMoving() ? LiveRefusal::HeldButtonMoving
                                      : LiveRefusal::LeavesOutWhereItIs;
    }
    /// "kP 0.25 -> 0.3125" into `out`.
    static void whatChanged(char* out, std::size_t cap, Tunable t, double from, double to) {
        const detail::ShortNumber a = detail::shortest(from);
        const detail::ShortNumber b = detail::shortest(to);
        std::snprintf(out, cap, "%.16s %.24s -> %.24s", parameterOf(t).name, a.text, b.text);
    }
    /// A refusal's sentence: "'lift': kD 0.004 -> 0 refused -- <why>".
    Applied& refuse(Applied& a, std::size_t i, Tunable t, double from, double to, LiveRefusal r) const {
        a.ok = false;
        a.why = r;
        char what[96];
        whatChanged(what, sizeof what, t, from, to);
        const char* why = r == LiveRefusal::BuildRefuses ? refusalText(a.build) : liveRefusalText(r);
        if (r == LiveRefusal::LeavesOutHeldTarget || r == LiveRefusal::LeavesOutWhereItIs) {
            const PositionSlot& p = service_->position(i);
            const PositionSpec& q = live_.specs[i].position;
            const char* u = q.units == Units::Inches ? "in" : "deg";
            const bool held = r == LiveRefusal::LeavesOutHeldTarget;
            std::snprintf(a.sentence, sizeof a.sentence,
                          "'%.23s': %.90s refused -- it %s at %.2f %s, outside the new limits: move it inside first",
                          live_.specs[i].name, what, held ? "holds" : "is",
                          std::clamp(held ? p.target() : p.position(), -99999.0, 99999.0), u);
            return a;
        }
        std::snprintf(a.sentence, sizeof a.sentence, "'%.23s': %.90s refused -- %.180s", live_.specs[i].name, what,
                      why);
        return a;
    }
    /// Write `next` as mechanism `i`'s spec, the old one recorded (a change or a reset; an undo has popped
    /// its own), and have the service re-derive it — its Info line says `what`. The SERVICE's answer is
    /// the one that counts: stateRefusal() asked the same questions before the write, but the where-it-is
    /// rule reads the sensor, and a reading that moved since (a coasting or drifting lift, jitter at the
    /// limit) makes the service refuse here (TUNE1-A's verification round). Then the old spec is put back,
    /// byte for byte, nothing is recorded, and the refusal says why in the sentence the same request
    /// refused before the write would have given.
    Applied& commit(Applied& a, std::size_t i, const MechanismSpec& next, Record::Made kind, Tunable t,
                    const char* what) {
        MechanismSpec old;
        std::memcpy(static_cast<void*>(&old), &live_.specs[i], sizeof old);
        std::memcpy(static_cast<void*>(&live_.specs[i]), &next, sizeof next);
        if (!service_->rederive(i, what)) {
            std::memcpy(static_cast<void*>(&live_.specs[i]), &old, sizeof old);
            const LiveRefusal why = lateRefusal(i);
            if (kind == Record::Made::Undo) {
                ++undoCount_;  // not undone after all: the record stays
            }
            if (kind == Record::Made::Change) {
                return refuse(a, i, t, tunableValue(old, t), tunableValue(next, t), why);
            }
            a.ok = false;
            a.why = why;
            std::snprintf(a.sentence, sizeof a.sentence, "'%.23s': %s refused -- %.180s", live_.specs[i].name,
                          kind == Record::Made::Undo ? "undo" : "RESET", liveRefusalText(why));
            return a;
        }
        if (kind != Record::Made::Undo) {
            push(i, old, kind, t);
        }
        a.ok = true;
        a.why = LiveRefusal::None;
        std::snprintf(a.sentence, sizeof a.sentence, "'%.23s': %.200s", live_.specs[i].name, what);
        return a;
    }
    /// Record a change; the oldest falls off past kLiveUndoDepth.
    void push(std::size_t i, const MechanismSpec& before, Record::Made kind, Tunable t) {
        if (undoCount_ == kLiveUndoDepth) {
            undoStart_ = (undoStart_ + 1) % kLiveUndoDepth;
            --undoCount_;
        }
        Record& r = undo_[(undoStart_ + undoCount_) % kLiveUndoDepth];
        std::memcpy(static_cast<void*>(&r.before), &before, sizeof before);
        r.index = static_cast<std::uint8_t>(i);
        r.kind = kind;
        r.what = t;
        ++undoCount_;
    }

    /// Each MEMBER LINE of mechanism `i`'s copy block, in the declaration's member order, to `emit`
    /// (header: which members). Every line is at most 200 bytes by construction: each number at most
    /// kCopyNumberChars (24), the longest line (a stall homing one) 77 fixed bytes and five numbers.
    template <class Emit>
    void forEachCopyLine(std::size_t i, Emit&& emit) const {
        const MechanismSpec& s = live_.specs[i];
        const MechanismSpec& d = declared_->specs[i];
        if (s.kind == Kind::Pneumatic) {
            return;
        }
        char line[240];
        const auto num = [](double v) { return detail::shortest(v); };
        const auto stallDiffers = [](const StallSpec& a, const StallSpec& b) {
            return std::memcmp(&a, &b, sizeof a) != 0;
        };
        const bool inches = s.kind == Kind::Position && s.position.units == Units::Inches;
        const char* len = inches ? "_in" : "";
        // .stall
        if (const StallSpec* st = detail::liveStall(s); st != nullptr && stallDiffers(*st, *detail::liveStall(d))) {
            std::snprintf(line, sizeof line, ".stall = {.amps = %.24s, .belowMotorRpm = %.24s, .lasting = %.24s_s}",
                          num(st->amps).text, num(detail::liveRpm(st->speedRadS)).text, num(st->time).text);
            emit(line);
        }
        // .currentCap
        if (std::memcmp(&s.currentCapAmps, &d.currentCapAmps, sizeof(double)) != 0) {
            std::snprintf(line, sizeof line, ".currentCap = {.amps = %.24s}", num(s.currentCapAmps).text);
            emit(line);
        }
        if (s.kind == Kind::Power) {
            if (std::memcmp(&s.power.maxVolts, &d.power.maxVolts, sizeof(double)) != 0) {
                std::snprintf(line, sizeof line, ".maxVolts = %.24s_volt", num(s.power.maxVolts).text);
                emit(line);
            }
            return;
        }
        if (s.kind == Kind::Speed) {
            const SpeedSpec& q = s.speed;
            if (q.atSpeedBandRpm != d.speed.atSpeedBandRpm || q.atSpeedTime != d.speed.atSpeedTime) {
                std::snprintf(line, sizeof line, ".atSpeed = within(%.24s, %.24s_s)", num(q.atSpeedBandRpm).text,
                              num(q.atSpeedTime).text);
                emit(line);
            }
            speedTuningLines(s, d, emit);
            return;
        }
        const PositionSpec& p = s.position;
        const PositionSpec& dp = d.position;
        if (p.minPosition != dp.minPosition || p.maxPosition != dp.maxPosition) {
            std::snprintf(line, sizeof line, ".range = {.min = %.24s%.3s, .max = %.24s%.3s}", num(p.minPosition).text, len,
                          num(p.maxPosition).text, len);
            emit(line);
        }
        if (std::memcmp(&p.kG, &dp.kG, sizeof(double)) != 0) {
            if (p.gravity == Gravity::Cosine) {
                std::snprintf(line, sizeof line, ".gravity = cosine(%.24s_volt, %.24s)", num(p.kG).text,
                              num(p.horizontalAt).text);
            } else {
                std::snprintf(line, sizeof line, ".gravity = constantGravity(%.24s_volt)", num(p.kG).text);
            }
            emit(line);
        }
        if (std::memcmp(&p.homeVolts, &dp.homeVolts, sizeof(double)) != 0 || stallDiffers(p.homeStall, dp.homeStall)) {
            if (p.home == HomeMethod::Stall) {
                std::snprintf(line, sizeof line,
                              ".home = homeByStall(%.24s%.3s, %.24s_volt, {.amps = %.24s, .belowMotorRpm = %.24s, "
                              ".lasting = %.24s_s})",
                              num(p.homePosition).text, len, num(p.homeVolts).text, num(p.homeStall.amps).text,
                              num(detail::liveRpm(p.homeStall.speedRadS)).text, num(p.homeStall.time).text);
            } else {
                std::snprintf(line, sizeof line, ".home = homeBySwitch(%.24s%.3s, %.24s_volt)", num(p.homePosition).text,
                              len, num(p.homeVolts).text);
            }
            emit(line);
        }
        positionTuningLines(s, d, emit);
    }

    /// The `.tuning` lines of a position mechanism: none when no `.tuning` number changed live; else the
    /// SMALLEST stated set (header): the live-changed numbers, then — ONE AT A TIME, in the member order,
    /// which is the derivation's own (only the acceleration hangs on another number, the top speed before
    /// it) — the first number whose derivation from the set so far differs from the live one, until none
    /// does. A number added so must be stated (everything it is derived from already matches), so no
    /// number is stated that would re-derive the same (landmine 12) — a file's own statement of a number
    /// AT its derived value included (it rebuilds the same; it is not kept — the header).
    template <class Emit>
    static void positionTuningLines(const MechanismSpec& s, const MechanismSpec& d, Emit&& emit) {
        constexpr std::size_t kN = 10;
        const auto field = [](const PositionSpec& p, std::size_t k) {
            const double f[kN] = {p.kP, p.kI, p.kD, p.kS, p.kV, p.kA, p.maxVelocity, p.maxAcceleration, p.tolerance,
                                  p.settleTime};
            return f[k];
        };
        const auto bits = [](double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; };
        bool stated[kN]{};
        bool anyChanged = false;
        for (std::size_t k = 0; k < kN; ++k) {
            stated[k] = !bits(field(s.position, k), field(d.position, k));
            anyChanged = anyChanged || stated[k];
        }
        if (!anyChanged) {
            return;  // the file's own .tuning stays as written
        }
        for (std::size_t round = 0; round <= kN; ++round) {
            PositionTuning t{};
            double* to[kN] = {&t.kP, &t.kI, &t.kD, &t.kS, &t.kV, &t.kA, &t.maxSpeed, &t.maxAcceleration, &t.within,
                              nullptr};
            for (std::size_t k = 0; k < kN; ++k) {
                if (stated[k] && to[k] != nullptr) {
                    *to[k] = field(s.position, k);
                }
            }
            if (stated[kN - 1]) {
                t.settle = units::Time{s.position.settleTime};
            }
            PositionSpec r = s.position;
            detail::positionStart(r, t, s.cartridge, detail::unitsPerOutputDeg(r),
                                  r.units == Units::Inches ? 0.25 : 1.5);
            bool more = false;
            for (std::size_t k = 0; k < kN && !more; ++k) {
                if (!stated[k] && !bits(field(r, k), field(s.position, k))) {
                    stated[k] = true;  // the FIRST that differs only: a later one may follow it once stated
                    more = true;
                }
            }
            if (!more) {
                break;
            }
        }
        static constexpr const char* kNames[kN] = {"kP", "kI", "kD", "kS", "kV", "kA", "maxSpeed", "maxAcceleration",
                                                   "within", "settle"};
        char line[96];
        for (std::size_t k = 0; k < kN; ++k) {
            if (stated[k]) {
                std::snprintf(line, sizeof line, ".tuning.%.16s = %.24s%s", kNames[k],
                              detail::shortest(field(s.position, k)).text, k == kN - 1 ? "_s" : "");
                emit(line);
            }
        }
    }
    /// The `.tuning` lines of a flywheel (SpeedTuning: kS, kV, kP, kI), by the same rule (each of its four
    /// is derived from nothing else).
    template <class Emit>
    static void speedTuningLines(const MechanismSpec& s, const MechanismSpec& d, Emit&& emit) {
        constexpr std::size_t kN = 4;
        const auto field = [](const SpeedSpec& q, std::size_t k) {
            const double f[kN] = {q.kS, q.kV, q.kP, q.kI};
            return f[k];
        };
        const auto bits = [](double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; };
        bool stated[kN]{};
        bool anyChanged = false;
        for (std::size_t k = 0; k < kN; ++k) {
            stated[k] = !bits(field(s.speed, k), field(d.speed, k));
            anyChanged = anyChanged || stated[k];
        }
        if (!anyChanged) {
            return;  // the file's own .tuning stays as written
        }
        for (std::size_t round = 0; round <= kN; ++round) {
            SpeedTuning t{};
            double* to[kN] = {&t.kS, &t.kV, &t.kP, &t.kI};
            for (std::size_t k = 0; k < kN; ++k) {
                if (stated[k]) {
                    *to[k] = field(s.speed, k);
                }
            }
            SpeedSpec r = s.speed;
            const double freeRpm = r.gearRatio > 0.0 ? detail::cartridgeRpm(s.cartridge) / r.gearRatio : 0.0;
            detail::speedStart(r, t, freeRpm * 2.0 * 3.14159265358979323846 / 60.0);
            bool more = false;
            for (std::size_t k = 0; k < kN && !more; ++k) {
                if (!stated[k] && !bits(field(r, k), field(s.speed, k))) {
                    stated[k] = true;
                    more = true;
                }
            }
            if (!more) {
                break;
            }
        }
        static constexpr const char* kNames[kN] = {"kS", "kV", "kP", "kI"};
        char line[64];
        for (std::size_t k = 0; k < kN; ++k) {
            if (stated[k]) {
                std::snprintf(line, sizeof line, ".tuning.%.4s = %.24s", kNames[k], detail::shortest(field(s.speed, k)).text);
                emit(line);
            }
        }
    }

    const MechanismTable* declared_;      ///< the robot file's table
    MechanismTable live_{};               ///< THE COPY the service runs
    MechanismService* service_ = nullptr; ///< attached
    std::array<Record, kLiveUndoDepth> undo_{};  ///< a ring of the last changes
    std::size_t undoStart_ = 0;           ///< its oldest
    std::size_t undoCount_ = 0;           ///< how many
};

}  // namespace shulib::mechanism
