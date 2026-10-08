#pragma once
//
// competition::AutonRegistry — the autons a competition program can run, as DATA (chunk COMP-a,
// 2026-09-26; the COMP brief's rulings C2, C3, C6 and amendments B15, B16; OnRefusal::DropEntry at
// T4 Part 3, the team lead's ruling of 2026-10-02 on COMP-b's C5).
//
// ── What an auton is here ───────────────────────────────────────────────────────────
// An AutonEntry is a row of data the STUDENTS' file registers, from `initialize()`, by an explicit
// call — never at static-initialization time (the file is compiled into every build, and a
// constructor at namespace scope runs before the on-robot precondition handler exists, in an
// order across translation units nobody controls). A row names the auton, says whether it is a
// MATCH auton or a SKILLS run, points at EITHER a routine function OR a config::Route, declares
// where the robot starts, and states its schedule. No field has a default that means something:
// every "unset" is refused, by name, when the row is registered.
//
// ── The start pose: FIELD or RELATIVE, re-seated when the auton starts (ruling C2) ──
// `StartPose::field(x, y, heading)` — the robot is placed at a known field pose. `StartPose::
// relative()` — wherever the robot is placed becomes (0, 0) facing 0. Either way the auton runner
// (match_runner.hpp) RE-SEATS the pose — position AND heading — at the instant the auton starts,
// which also discards whatever the IMU drifted while the robot waited. The IMU's boot heading is
// never the start heading (hardware-assumptions HA-05, amended at COMP-a).
//
// ── Routes (B16): the route's Start waypoint IS the start pose ──────────────────────
// A route whose waypoint 0 is a `Start` already says where the robot starts. So a route entry may
// leave `start` unset (it takes the route's), or state the SAME field pose; a DIFFERENT one — or
// RELATIVE — is refused, because the route would setPose() to its own Start one tick after the
// runner re-seated somewhere else. A route with no Start takes the entry's, which must then be set.
// The Start's HEADING is the start heading only when the waypoint STATES it — HeadingMode::Fixed,
// the one mode under which config::Waypoint documents `heading` as read. A `Free` Start says "the
// robot begins at (x, y)" and nothing about which way it faces, so the seat will not read its
// `heading` field (a default 0 there would otherwise be seated silently): such an entry must
// declare its own FIELD start at the Start's (x, y), which supplies the heading, or be refused.
// (A decision-consistency check against F8, COMP-a's review: before B16 no code read a Start's
// heading at all — PathRunner's Start leg seeds position only — so this rule is where the
// field's meaning is settled, and it keeps route.hpp's "read only when Fixed" true.)
//
// ── The schedule (C6, B15): stated, never defaulted, bounded by the period ──────────
// `endActionAt` / `hardStopAt` are sequence::RunGuardConfig's two instants, measured from the
// auton's start. They default to 0 — UNSET — and an unset or nonsensical schedule is refused
// here, before a match rather than in one. The PERIOD an entry runs in follows from its KIND and
// the PROGRAM's two constants (AutonPeriods — the library states no match length anywhere,
// run_guard.hpp's banner); `hardStopAt` past the period is refused. A margin INSIDE the period is
// recommended (the guide will say so): the period is when the field disables the robot, and a
// hard stop that lands on it is a hard stop that never had a tick to act.
//
// ── A refused entry: the boot, or just that auton (the team lead's ruling of 2026-10-02) ──
// By default add() THROWS on an entry check() refuses (B15: refusals happen at registration, not in
// a match), so a program's initialize() refuses its boot by name. The team lead ruled otherwise for
// the competition program (COMP-b's review, its C5: "drop just that auton" — an auton bug must not
// cost the match's driver period), and the Auton Tester, which registers through the same function,
// does the same (the coordinator's extension of the ruling, T4 Part 3; the team lead confirmed it
// 2026-10-08): a registry built with
// OnRefusal::DropEntry registers NOTHING for a refused entry, keeps check()'s sentence (refusal(),
// which names the auton first), and RETURNS — so the students' registration function goes on to its
// next entry, the refused one is never on the menu (AutonMenu names it there instead), and the
// program logs every sentence. The rules themselves are unchanged: the same check(), the same
// sentences.
//
// ── Names (B25) ─────────────────────────────────────────────────────────────────────
// A name is shown as "AUTON: <name>" on ONE medium line of the brain screen, so it is at most
// kMaxAutonNameChars = ui::kMediumCols - 7 = 29 printable ASCII characters, no '%', unique, and
// never "NONE" (the selector's own choice). The name pointer is BORROWED — a string literal.
//
// PROS-free, allocation-free (fixed capacity), no std::function. NOTHING HERE HAS RUN ON A ROBOT.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string_view>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/ui/touch_panel.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::competition {

/// The most autons one registry holds. Fixed capacity so registration never allocates; sixteen is
/// several pages of the selector and more than a season has ever needed.
inline constexpr std::size_t kMaxAutons = 16;

/// The longest auton name: "AUTON: " (7 characters) plus the name must fit ONE medium line (B25).
inline constexpr std::size_t kMaxAutonNameChars = ui::kMediumCols - 7;
static_assert(kMaxAutonNameChars == 29, "B25 fixes the name budget at kMediumCols - 7 = 29");

/// The most refused entries one registry keeps the sentences of (refusal()); refusedCount() counts
/// past it.
inline constexpr std::size_t kMaxAutonRefusals = 8;

/// What AutonRegistry::add() does with an entry its check() refuses (file banner: A refused entry).
enum class OnRefusal : std::uint8_t {
    Throw = 0,      ///< throw PreconditionError carrying check()'s sentence (B15; the default)
    DropEntry = 1,  ///< register nothing, keep check()'s sentence (refusal()), and return
};

/// The selector's reserved choice. No registered auton may be called this.
inline constexpr std::string_view kNoAutonName = "NONE";

/// What kind of run an auton is — which PERIOD it runs in (B15). Unset is refused.
enum class AutonKind : std::uint8_t {
    Unset = 0,   ///< not stated — refused at registration
    Match = 1,   ///< a head-to-head match's autonomous period
    Skills = 2,  ///< an autonomous coding skills run
};

/// The kind as a word for screens and logs ("MATCH", "SKILLS"; "UNSET"; "UNKNOWN" for a bad byte).
[[nodiscard]] constexpr const char* autonKindWord(AutonKind k) noexcept {
    switch (k) {
        case AutonKind::Unset: return "UNSET";
        case AutonKind::Match: return "MATCH";
        case AutonKind::Skills: return "SKILLS";
    }
    return "UNKNOWN";
}

/// The two periods an auton can run in — the PROGRAM's constants, from the game manual (the COMP
/// brief's C6 records the 2026-27 Override numbers: a 30 s match autonomous, a 60 s skills run).
/// REQUIRED: both default to 0, which validate() refuses, because a library default here would be
/// an invented match length (run_guard.hpp's banner — the HA-51 cautionary tale).
struct AutonPeriods {
    /// The match autonomous period.
    units::Time match{0.0};
    /// The autonomous skills period.
    units::Time skills{0.0};

    /// Refuse a period that could not mean anything: both finite and > 0.
    void validate() const {
        SHULIB_PRECONDITION(std::isfinite(match.value()) && match.value() > 0.0,
                            "AutonPeriods: the match period must be stated, finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(skills.value()) && skills.value() > 0.0,
                            "AutonPeriods: the skills period must be stated, finite and > 0");
    }

    /// The period for `kind` — 0 for Unset (which no valid entry has).
    [[nodiscard]] units::Time periodFor(AutonKind kind) const noexcept {
        switch (kind) {
            case AutonKind::Match: return match;
            case AutonKind::Skills: return skills;
            case AutonKind::Unset: break;
        }
        return units::Time{0.0};
    }
};

/// Where an auton starts (ruling C2): a FIELD pose, or RELATIVE — wherever the robot is placed
/// becomes (0, 0) facing 0. Either way the auton runner re-seats position AND heading at the
/// instant the auton starts. Default-constructed it is UNSET, which a routine entry may not be.
class StartPose {
public:
    /// Which of the three it is.
    enum class Mode : std::uint8_t {
        Unset = 0,     ///< not stated (a route entry may take its route's Start instead)
        Field = 1,     ///< a known field pose
        Relative = 2,  ///< (0, 0) facing 0, wherever the robot is placed
    };

    /// Unset.
    constexpr StartPose() noexcept = default;

    /// The robot is placed at field pose (x, y) facing `heading`.
    [[nodiscard]] static StartPose field(units::Length x, units::Length y, math::Angle heading) {
        return StartPose{Mode::Field, math::Pose2d{x, y, heading}};
    }
    /// The same, from a pose.
    [[nodiscard]] static StartPose field(const math::Pose2d& pose) {
        return StartPose{Mode::Field, pose};
    }
    /// Wherever the robot is placed becomes (0, 0) facing 0.
    [[nodiscard]] static StartPose relative() { return StartPose{Mode::Relative, math::Pose2d{}}; }

    /// Which kind of start this is.
    [[nodiscard]] Mode mode() const noexcept { return mode_; }
    /// The pose the runner seats: the field pose, or (0, 0, 0) for RELATIVE (and for Unset).
    [[nodiscard]] math::Pose2d pose() const noexcept { return pose_; }
    /// True unless Unset.
    [[nodiscard]] bool isSet() const noexcept { return mode_ != Mode::Unset; }

    /// Same mode and the same pose (positions within 1e-9 in, headings within 1e-9 rad).
    [[nodiscard]] bool sameAs(const StartPose& o) const noexcept {
        return mode_ == o.mode_ && std::abs((pose_.x() - o.pose_.x()).value()) <= 1e-9
               && std::abs((pose_.y() - o.pose_.y()).value()) <= 1e-9
               && pose_.heading().approxEqual(o.pose_.heading(), 1e-9);
    }

private:
    StartPose(Mode m, const math::Pose2d& p) noexcept : mode_{m}, pose_{p} {}
    Mode mode_ = Mode::Unset;
    math::Pose2d pose_{};
};

/// The start mode as a word ("FIELD", "RELATIVE", "UNSET").
[[nodiscard]] constexpr const char* startModeWord(StartPose::Mode m) noexcept {
    switch (m) {
        case StartPose::Mode::Unset: return "UNSET";
        case StartPose::Mode::Field: return "FIELD";
        case StartPose::Mode::Relative: return "RELATIVE";
    }
    return "UNKNOWN";
}

struct AutonEntry;

/// What a routine gets to work with while it runs. Built by the runner for one run; every
/// reference is valid for exactly that run.
struct AutonContext {
    /// The facade — every verb an auton is written with.
    chassis::Chassis& chassis;
    /// The run's guard. Its waitFor()/pause() are the only waits the deadline (and a mid-auton
    /// disable) can cut: a frozen wait (Chassis::wait, Routine::pause) runs to its own end first.
    sequence::RunGuard& guard;
    /// Where the auton's own log lines go.
    hal::ITelemetrySink& telemetry;
    /// The mechanisms the guard SAFES at the cut and at the end — the Robot's PRESENT ones (B9).
    /// For the guard's safing only (B25): a routine that commands a mechanism looks it up by name
    /// from the Robot, not from this list.
    std::span<hal::IMechanism* const> mechanisms;
    /// The entry being run.
    const AutonEntry& entry;
};

/// An auton's routine: runs the auton and returns the Routine's verdict (a bare ExitReason would
/// lose where and why it stopped — routine.hpp's argument).
using AutonRoutine = chassis::RoutineResult (*)(AutonContext&);

/// An auton's end action — the park — run by the guard when scoring ends (run_guard.hpp). Returns
/// success. nullptr means the default: `chassis.brake()`, Settled = success.
using AutonEndAction = bool (*)(AutonContext&);

/// One registered auton (file banner). Every field without a meaningful default is refused when
/// unset, at registration.
struct AutonEntry {
    /// Shown on the screen and in the log: at most kMaxAutonNameChars printable ASCII, no '%',
    /// unique, never "NONE". BORROWED — a string literal.
    const char* name = nullptr;
    /// MATCH or SKILLS: which period it runs in. Unset is refused.
    AutonKind kind = AutonKind::Unset;
    /// The routine to run — EXACTLY one of `routine` / `route` is set.
    AutonRoutine routine = nullptr;
    /// The route to run through a runner::PathRunner — BORROWED, must outlive the registry. A route
    /// entry needs a runner::CommandRegistry at run time (the runner refuses without one).
    const config::Route* route = nullptr;
    /// Where the robot starts (file banner: FIELD or RELATIVE; a route may take its Start's).
    StartPose start{};
    /// When scoring stops and the end action starts, from the auton's start. 0 = UNSET, refused.
    units::Time endActionAt{0.0};
    /// The unconditional safe floor, from the auton's start. 0 = UNSET, refused; must be >=
    /// endActionAt and within the kind's period.
    units::Time hardStopAt{0.0};
    /// The park. nullptr = `chassis.brake()`.
    AutonEndAction endAction = nullptr;

    /// The start pose the runner seats: `start`, or — for a route entry that left it unset — the
    /// route's Start waypoint as a FIELD pose (B16), when that waypoint STATES its heading
    /// (HeadingMode::Fixed). Unset when neither says, which includes a route whose Start is
    /// `Free`: it states no heading, and the seat will not invent one (file banner, Routes).
    [[nodiscard]] StartPose effectiveStart() const {
        if (!start.isSet() && route != nullptr && route->count > 0
            && route->waypoints[0].motion == config::WaypointMotion::Start
            && route->waypoints[0].headingMode == config::HeadingMode::Fixed) {
            const config::Waypoint& w = route->waypoints[0];
            return StartPose::field(w.x, w.y, w.heading);
        }
        return start;
    }
};

/// A registration verdict: one bool and the sentence a person reads — naming the auton first.
struct AutonVerdict {
    /// True when the entry may be registered / run.
    bool ok = false;
    /// The FIRST thing wrong, beginning "auton '<name>': ". Empty when `ok`.
    char why[192]{};
};

/// Can `e` run in a period of `period`? PURE and TOTAL (never throws, never allocates). The
/// checks, in order: the name (present, printable ASCII, no '%', within kMaxAutonNameChars, not
/// "NONE"); the kind; exactly one of routine/route; a route that is itself valid; the start pose
/// (B16's rules for a route; set, for a routine) and its finiteness; the schedule (stated, finite,
/// endActionAt > 0, hardStopAt >= endActionAt); and hardStopAt within `period`. It does NOT check
/// uniqueness (the registry does) or a route's command registry (the runner does, at run time).
[[nodiscard]] inline AutonVerdict checkAutonEntry(const AutonEntry& e, units::Time period) noexcept {
    AutonVerdict v;
    // The name goes into the sentence only once it has passed the screen test; until then a
    // placeholder stands in, so the sentence itself is always safe to put on the brain screen.
    const char* shown = e.name;
    const auto refuse = [&v](const char* name, const char* what) noexcept {
        std::snprintf(v.why, sizeof v.why, "auton '%s': %s", name, what);
        return v;
    };
    if (e.name == nullptr || e.name[0] == '\0') {
        return refuse("(unnamed)", "has no name");
    }
    const std::string_view name{e.name, std::strlen(e.name)};
    if (!ui::screenText(name, kMaxAutonNameChars)) {
        return refuse("(bad name)",
                      "the name must be 1..29 printable ASCII characters with no percent sign "
                      "(it is shown as 'AUTON: <name>' on one screen line)");
    }
    if (name == kNoAutonName) {
        return refuse(shown, "NONE is the selector's own choice -- pick another name");
    }
    if (e.kind != AutonKind::Match && e.kind != AutonKind::Skills) {
        return refuse(shown, "its kind is not set -- say MATCH or SKILLS");
    }
    if ((e.routine == nullptr) == (e.route == nullptr)) {
        return refuse(shown, e.routine == nullptr ? "has neither a routine nor a route -- give one"
                                                  : "has BOTH a routine and a route -- give one");
    }
    if (e.route != nullptr) {
        // 192, runner::PathRunner's own size for the same call (path_runner.hpp), and the outer
        // copy bounded: at -Os GCC inlines Route::valid() here and SEES its longest sentence
        // (up to 173 bytes) -- a 160-byte buffer is a -Wformat-truncation in every V5 build the
        // first time a src/ file registers an auton (found by COMP-t, 2026-09-27: the students'
        // src/team/autons.cpp is that first file; an inline function nothing calls is never
        // analysed, so neither the ARM header gate nor the host suite could see it).
        char routeWhy[192] = "";
        if (!e.route->valid(routeWhy, sizeof routeWhy)) {
            std::snprintf(v.why, sizeof v.why, "auton '%s': its route is invalid: %.150s", shown,
                          routeWhy);
            return v;
        }
        const config::Waypoint& first = e.route->waypoints[0];
        const bool routeHasStart = first.motion == config::WaypointMotion::Start;
        // A Free Start states no heading (file banner, Routes): the entry's own heading is then
        // the only one there is, so only the place is compared.
        const bool startStatesHeading =
            routeHasStart && first.headingMode == config::HeadingMode::Fixed;
        if (routeHasStart && e.start.isSet()) {
            const math::Angle stated = startStatesHeading ? first.heading : e.start.pose().heading();
            if (!e.start.sameAs(StartPose::field(first.x, first.y, stated))) {
                return refuse(shown,
                              "declares a start pose different from its route's Start waypoint "
                              "-- leave it unset or make them equal");
            }
        }
        if (routeHasStart && !startStatesHeading && !e.start.isSet()) {
            return refuse(shown,
                          "its route's Start states no heading (Free) -- make it Fixed, or "
                          "declare a FIELD start pose at the same (x, y)");
        }
        if (!routeHasStart && !e.start.isSet()) {
            return refuse(shown,
                          "its route has no Start waypoint, so the entry must declare a start "
                          "pose (FIELD or RELATIVE)");
        }
    } else if (!e.start.isSet()) {
        return refuse(shown, "has no start pose -- declare FIELD(x, y, heading) or RELATIVE");
    }
    const math::Pose2d seat = e.effectiveStart().pose();
    if (!std::isfinite(seat.x().value()) || !std::isfinite(seat.y().value())) {
        return refuse(shown, "its start pose is not finite");
    }
    const double act = e.endActionAt.value();
    const double stop = e.hardStopAt.value();
    if (act == 0.0 && stop == 0.0) {
        return refuse(shown, "its schedule is UNSET -- state endActionAt and hardStopAt");
    }
    if (!std::isfinite(act) || act <= 0.0) {
        return refuse(shown, "endActionAt must be finite and > 0");
    }
    if (!std::isfinite(stop) || stop < act) {
        return refuse(shown, "hardStopAt must be finite and >= endActionAt");
    }
    const double p = period.value();
    if (!std::isfinite(p) || p <= 0.0) {
        return refuse(shown, "the period it would run in is not stated");
    }
    if (stop > p) {
        std::snprintf(v.why, sizeof v.why,
                      "auton '%s': hardStopAt %.2f s is past its %s period of %.2f s -- the field "
                      "disables the robot first",
                      shown, stop, autonKindWord(e.kind), p);
        return v;
    }
    v.ok = true;
    v.why[0] = '\0';
    return v;
}

/// The autons a program can run: fixed capacity, filled by explicit add() calls from
/// `initialize()` (file banner), read by the selector and the runner. Single-task while it is
/// being filled; read-only afterwards, so any task may read it once registration is over.
class AutonRegistry {
public:
    /// `periods` are the PROGRAM's two constants (REQUIRED — AutonPeriods::validate()).
    /// `onRefusal`: what add() does with an entry check() refuses (file banner: A refused entry) —
    /// throw (the default), or drop just that entry and keep its sentence.
    explicit AutonRegistry(const AutonPeriods& periods, OnRefusal onRefusal = OnRefusal::Throw)
        : periods_{periods}, onRefusal_{onRefusal} {
        periods_.validate();
    }

    /// Would `e` be accepted? checkAutonEntry() against its kind's period, then uniqueness and
    /// capacity. Pure: registers nothing.
    [[nodiscard]] AutonVerdict check(const AutonEntry& e) const noexcept {
        AutonVerdict v = checkAutonEntry(e, periods_.periodFor(e.kind));
        if (!v.ok) {
            return v;
        }
        if (find(e.name) != nullptr) {
            std::snprintf(v.why, sizeof v.why, "auton '%s': the name is already registered",
                          e.name);
            v.ok = false;
            return v;
        }
        if (count_ >= kMaxAutons) {
            std::snprintf(v.why, sizeof v.why, "auton '%s': the registry is full (%d autons)",
                          e.name, static_cast<int>(kMaxAutons));
            v.ok = false;
        }
        return v;
    }

    /// Register `e` (COPIED; its name and route stay borrowed) when check(e) passes. When it does
    /// not: under OnRefusal::Throw (the default) a PreconditionError carrying check()'s sentence,
    /// which names the auton, so a program's `initialize()` try block can log it and refuse the
    /// boot by name (ruling B15: refusals happen at REGISTRATION, not in a match); under
    /// OnRefusal::DropEntry nothing is registered, the sentence is kept (refusal()) and counted
    /// (refusedCount()), and add() returns — never throws — so the next entry is registered as if
    /// the refused one had never been written (the team lead's ruling of 2026-10-02).
    void add(const AutonEntry& e) {
        const AutonVerdict v = check(e);
        if (!v.ok && onRefusal_ == OnRefusal::DropEntry) {
            if (refused_ < kMaxAutonRefusals) {
                refusals_[refused_] = v;
            }
            ++refused_;
            return;
        }
        SHULIB_PRECONDITION(v.ok, v.why);
        entries_[count_++] = e;
    }

    /// What add() does with a refused entry (the constructor's choice).
    [[nodiscard]] OnRefusal onRefusal() const noexcept { return onRefusal_; }
    /// How many entries add() DROPPED (OnRefusal::DropEntry) — every one, kept or not. Always 0
    /// under OnRefusal::Throw, where a refusal throws instead.
    [[nodiscard]] std::size_t refusedCount() const noexcept { return refused_; }
    /// The sentence of dropped entry `i`, in the order add() met them — check()'s, beginning
    /// "auton '<name>': " — for `i` below both refusedCount() and kMaxAutonRefusals; "" otherwise.
    [[nodiscard]] const char* refusal(std::size_t i) const noexcept {
        return i < refused_ && i < kMaxAutonRefusals ? refusals_[i].why : "";
    }

    /// How many autons are registered.
    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    /// True when none are.
    [[nodiscard]] bool empty() const noexcept { return count_ == 0; }
    /// The auton at `index` (registration order). Precondition: index < size().
    [[nodiscard]] const AutonEntry& at(std::size_t index) const {
        SHULIB_PRECONDITION(index < count_, "AutonRegistry::at: index out of range");
        return entries_[index];
    }
    /// The auton called `name`, or nullptr.
    [[nodiscard]] const AutonEntry* find(std::string_view name) const noexcept {
        if (name.data() == nullptr) {
            return nullptr;
        }
        for (std::size_t i = 0; i < count_; ++i) {
            if (std::string_view{entries_[i].name} == name) {
                return &entries_[i];
            }
        }
        return nullptr;
    }
    /// The program's two periods, as given.
    [[nodiscard]] const AutonPeriods& periods() const noexcept { return periods_; }
    /// The period `e` runs in.
    [[nodiscard]] units::Time periodFor(const AutonEntry& e) const noexcept {
        return periods_.periodFor(e.kind);
    }

private:
    AutonPeriods periods_;
    OnRefusal onRefusal_ = OnRefusal::Throw;
    AutonEntry entries_[kMaxAutons]{};
    std::size_t count_ = 0;
    AutonVerdict refusals_[kMaxAutonRefusals]{};
    std::size_t refused_ = 0;
};

}  // namespace shulib::competition
