#pragma once
//
// mechanism/until.hpp — WHEN A STEP IS DONE: a power step's done-conditions (the mechanism core,
// chunk MECH-A2, 2026-10-15).
//
// A step that runs a mechanism ends when ANY of its conditions holds — "this much time has passed",
// "it stalled", "a piece sensor sees a piece", "a piece sensor stops seeing one" — and ALWAYS also at
// its BOUND, the time limit every step states. `Until` never replaces the bound (requirements P4:
// every action is bounded and says how it ended); it only lets a step end sooner, and says which
// condition ended it (`DoneBy`, the word the step's log line names).
//
//     r.run(intake, 12_volt, seesPiece, 3_s)              // until ITS piece sensor sees a piece
//     r.run(intake, 12_volt, seesPiece.orAfter(2.5_s), 3_s)  // ... or after 2.5 s: SUCCEEDED by time
//     r.run(intake, -6_volt, stopsSeeingPiece, 1_s)       // until it no longer sees one (an eject)
//     r.run(roller, 12_volt, seesPiece(grip), 2_s)        // until ANOTHER mechanism's sensor sees one
//     r.run(catapult, 6_volt, stalls, 1_s)                // until it stalls: a squeeze, SUCCEEDED
//
// (`r.run(...)` is the auton's step, MECH-A4; the mechanism service runs the condition,
// mechanism/service.hpp.)
//
// ── A condition that can never be met is a mistake, said where it is made ────────────
// `seesPiece(arm)` — another mechanism's sensor, on a mechanism that declares none — FAILS THE BUILD
// at the auton's line (detail::that_mechanism_has_no_piece_sensor). The forms the build cannot see
// (`seesPiece` / `stopsSeeingPiece` on the step's OWN mechanism, `stalls` on one with no stall
// signature) are refused at the STEP, before anything is commanded (the service's preconditions):
// the condition and the declaration are two arguments of one run-time call, and the host test of
// every registered auton runs each step against its robot's real list (MECH-A5), so the refusal
// fails the suite before any upload.
//
// ── What a condition reads ───────────────────────────────────────────────────────────
// A piece sensor is read as POSITIVE evidence (the service's Slot::seen()): YES, NO or UNKNOWN, and
// UNKNOWN — a sensor declared but missing this boot, a distance held near with no confidence, a
// sample taken while its read-fault count moved — satisfies NEITHER `seesPiece` nor
// `stopsSeeingPiece`. A condition that names ANOTHER mechanism stores that mechanism's NAME, not
// its address: the service finds it by name at run time (a declaration copied without `inline` has
// one address per translation unit — the brief's P12). A declaration written without `.name` is
// on no robot's list: the words refuse it at build (A1's `a_mechanism_has_no_name`), and the plain
// `Until::sees(decl)` stores an EMPTY name — never "this mechanism" — which the step refuses.
//
// ONE mechanism's sensor per condition. Two different NAMES joined by `|` are `conflicting()` at
// once. The step's OWN sensor beside a NAME (`seesPiece | stopsSeeingPiece(intake)`) is one sensor
// when the step runs on that very mechanism and two otherwise — only the step knows which, so the
// condition keeps both facts (`sensor()` and `readsOwnSensorToo()`) and the step decides.
//
// Plain constexpr data: a bit set, a time, which mechanism's sensor. PROS-free, allocation-free.

#include <cstddef>
#include <cstdint>

#include "shulib/mechanism/spec.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::mechanism {

/// Which done-condition ended a step — the word the step's log line names (`doneByWord`).
enum class DoneBy : std::uint8_t {
    None = 0,     ///< no condition: the bound, a refusal, a cancel
    Time,         ///< its time condition (a duration, or `orAfter`'s)
    Reached,      ///< a position or speed step reached its target (MECH-A3)
    Stalled,      ///< it stalled (`stalls`: SUCCEEDED; a stall signature otherwise: STALLED)
    Sees,         ///< a piece sensor saw a piece
    StopsSeeing,  ///< a piece sensor stopped seeing a piece
};

/// The words of a `DoneBy`, as the step's log line prints them. Never null; an out-of-range value
/// reads "?".
[[nodiscard]] constexpr const char* doneByWord(DoneBy d) noexcept {
    switch (d) {
        case DoneBy::None: return "-";
        case DoneBy::Time: return "time";
        case DoneBy::Reached: return "reached";
        case DoneBy::Stalled: return "stalled";
        case DoneBy::Sees: return "sees a piece";
        case DoneBy::StopsSeeing: return "stopped seeing a piece";
    }
    return "?";
}

/// A step's done-conditions (header): any combination of a time, a stall and a piece sensor's
/// seeing or not seeing, ALWAYS beside the step's bound. Combined with `|` (either ends the step) or
/// `orAfter` (a sensor condition, or after a time).
class Until {
public:
    /// "This mechanism": no other mechanism named — the condition reads the step's OWN sensor.
    static constexpr const char* kSelf = nullptr;

    /// Done when `t` has passed since the step started (a desired duration: SUCCEEDED, by time).
    [[nodiscard]] static constexpr Until time(units::Time t) noexcept {
        return Until{kTime, t.value(), kSelf, false, false};
    }
    /// Done when the mechanism STALLS — a squeeze that closes until it is tight: SUCCEEDED, not a
    /// fault. Without it, a stated stall signature that trips is the STALLED verdict and a
    /// MECHANISM_STALLED fault. Needs the mechanism's stall signature (refused at the step without).
    [[nodiscard]] static constexpr Until stalled() noexcept { return Until{kStalled, 0.0, kSelf, false, false}; }
    /// Done when THIS mechanism's piece sensor sees a piece.
    [[nodiscard]] static constexpr Until sees() noexcept { return Until{kSees, 0.0, kSelf, false, false}; }
    /// Done when THIS mechanism's piece sensor stops seeing a piece.
    [[nodiscard]] static constexpr Until stopsSeeing() noexcept { return Until{kStops, 0.0, kSelf, false, false}; }
    /// Done when ANOTHER mechanism's piece sensor sees a piece (the roller runs until the GRIP sees
    /// one), named by that mechanism's declaration of any kind and found by its NAME at run time.
    /// The words (`seesPiece(grip)`) also refuse, at build, a mechanism that declares no sensor and
    /// a declaration with no `.name`. Here, a declaration with no `.name` stores an EMPTY name —
    /// never kSelf, which would read the step's own sensor in its place — and the step refuses it.
    template <class Declared>
    [[nodiscard]] static constexpr Until sees(const Declared& other) noexcept {
        return Until{kSees, 0.0, nameOrEmpty(other.name.text), false, false};
    }
    /// Done when ANOTHER mechanism's piece sensor stops seeing a piece (`sees(other)`'s rules).
    template <class Declared>
    [[nodiscard]] static constexpr Until stopsSeeing(const Declared& other) noexcept {
        return Until{kStops, 0.0, nameOrEmpty(other.name.text), false, false};
    }
    /// Never done by a condition: only the bound ends the step (TIMED_OUT) — a run that is meant to
    /// last its whole time limit while the robot does something else.
    [[nodiscard]] static constexpr Until bound() noexcept { return Until{0, 0.0, kSelf, false, false}; }

    /// Either condition ends the step. Two TIME conditions keep the EARLIER. The sensor conditions
    /// must read ONE mechanism's sensor (header): two different NAMES (`seesPiece(grip) |
    /// stopsSeeingPiece(claw)`) are `conflicting()` and the step refuses them; the step's OWN sensor
    /// beside a name keeps the name and sets `readsOwnSensorToo()`, which the step resolves (one
    /// sensor when it runs on that mechanism, refused otherwise).
    [[nodiscard]] constexpr Until operator|(const Until& o) const noexcept {
        const bool mine = (bits_ & (kSees | kStops)) != 0;
        const bool theirs = (o.bits_ & (kSees | kStops)) != 0;
        bool clash = conflict_ || o.conflict_;
        const char* name = kSelf;
        bool own = false;
        if (mine) {
            name = sensor_;
            own = sensor_ == kSelf || selfToo_;
        }
        if (theirs) {
            if (name != kSelf && o.sensor_ != kSelf && !sameName(name, o.sensor_)) {
                clash = true;  // two different mechanisms named: no step can read both
            }
            if (name == kSelf) {
                name = o.sensor_;
            }
            own = own || o.sensor_ == kSelf || o.selfToo_;
        }
        const bool myTime = (bits_ & kTime) != 0;
        const bool theirTime = (o.bits_ & kTime) != 0;
        double t = 0.0;
        if (myTime && theirTime) {
            t = time_ < o.time_ ? time_ : o.time_;
        } else if (myTime) {
            t = time_;
        } else if (theirTime) {
            t = o.time_;
        }
        return Until{static_cast<std::uint8_t>(bits_ | o.bits_), t, name, own && name != kSelf, clash};
    }

    /// "... or after `t`, whichever comes first" — the time ends it SUCCEEDED, by time
    /// (`seesPiece.orAfter(2.5_s)`: a missed piece does not end the auton). `t` must be shorter
    /// than the step's time limit (refused at the step otherwise: the bound would always win).
    [[nodiscard]] constexpr Until orAfter(units::Time t) const noexcept { return *this | Until::time(t); }

    /// Whether the condition `bit` (kTime, kStalled, kSees, kStops) is part of it.
    [[nodiscard]] constexpr bool has(std::uint8_t bit) const noexcept { return (bits_ & bit) != 0; }
    /// The time condition's duration, seconds (0 when there is none).
    [[nodiscard]] constexpr double seconds() const noexcept { return time_; }
    /// The NAME of the mechanism whose piece sensor the condition reads (kSelf = the step's own; ""
    /// = a declaration with no `.name`, which no robot's list has).
    [[nodiscard]] constexpr const char* sensor() const noexcept { return sensor_; }
    /// True when the condition ALSO reads the step's own sensor beside the one `sensor()` names
    /// (`seesPiece | stopsSeeingPiece(intake)`): one sensor if the step runs on that mechanism, two
    /// otherwise — the step resolves it (operator|). Always false when `sensor()` is kSelf.
    [[nodiscard]] constexpr bool readsOwnSensorToo() const noexcept { return selfToo_; }
    /// True when `|` joined sensor conditions NAMING two different mechanisms (operator|): the step
    /// refuses it rather than silently read one of them.
    [[nodiscard]] constexpr bool conflicting() const noexcept { return conflict_; }

    /// The time condition's bit.
    static constexpr std::uint8_t kTime = 1;
    /// The stall condition's bit (2 is left free: "reached" is a position step's own, not a condition).
    static constexpr std::uint8_t kStalled = 4;
    /// The "sees a piece" condition's bit.
    static constexpr std::uint8_t kSees = 8;
    /// The "stops seeing a piece" condition's bit.
    static constexpr std::uint8_t kStops = 16;

private:
    constexpr Until(std::uint8_t b, double t, const char* s, bool selfToo, bool clash) noexcept
        : bits_{b}, time_{t}, sensor_{s}, selfToo_{selfToo}, conflict_{clash} {}
    /// A declaration's name as a condition stores it: a missing one (null) becomes "" — a name no
    /// robot's list can have (A1 refuses an empty name) — so it can never read as kSelf.
    [[nodiscard]] static constexpr const char* nameOrEmpty(const char* text) noexcept {
        return text != nullptr ? text : "";
    }
    /// Two NAMES (neither kSelf) are one mechanism when their text is the same — compared by content:
    /// one name has one address per translation unit at most, not one address overall.
    [[nodiscard]] static constexpr bool sameName(const char* a, const char* b) noexcept {
        std::size_t i = 0;
        while (a[i] != '\0' && a[i] == b[i]) {
            ++i;
        }
        return a[i] == b[i];
    }
    std::uint8_t bits_;
    double time_;
    const char* sensor_;  ///< kSelf, or another mechanism's name ("" for a declaration with none)
    bool selfToo_;        ///< the step's own sensor too, beside sensor_ (never with kSelf)
    bool conflict_;       ///< two different names joined
};

namespace detail {
/// A consteval refusal, NOT constexpr on purpose: calling it from a consteval context stops the
/// build and GCC names it at the auton's line — `seesPiece(arm)` / `stopsSeeingPiece(hood)` name a
/// mechanism that declares no piece sensor (an arm, a lift and a flywheel have none at all).
inline void that_mechanism_has_no_piece_sensor() {}
/// Refuses (at build) a declaration with no `.name` (A1's refusal, by its own name: such a
/// declaration is on no robot's list, so no step could find its sensor) and one with no piece sensor.
template <class Declared>
consteval void needsPieceSensor(const Declared& other) {
    if (other.name.text == nullptr || other.name.text[0] == '\0') {
        refused::a_mechanism_has_no_name();
    }
    if constexpr (requires { other.pieceSensor.kind; }) {
        if (other.pieceSensor.kind == SensorKind::None) {
            that_mechanism_has_no_piece_sensor();
        }
    } else {
        that_mechanism_has_no_piece_sensor();  // an Arm, a Lift or a Flywheel declares none
    }
}
}  // namespace detail

/// THE WORD `seesPiece`: as itself, the step's OWN piece sensor; called with another mechanism's
/// declaration, THAT one's (`seesPiece(grip)`) — refused AT BUILD when that one declares none.
struct SeesPiece : Until {
    /// The step's own sensor.
    constexpr SeesPiece() noexcept : Until{Until::sees()} {}
    /// Another mechanism's piece sensor; a declaration with none fails the build, named.
    template <class Declared>
    [[nodiscard]] consteval Until operator()(const Declared& other) const {
        detail::needsPieceSensor(other);
        return Until::sees(other);
    }
};
/// THE WORD `stopsSeeingPiece`: as `seesPiece`, for "no longer sees a piece".
struct StopsSeeingPiece : Until {
    /// The step's own sensor.
    constexpr StopsSeeingPiece() noexcept : Until{Until::stopsSeeing()} {}
    /// Another mechanism's piece sensor; a declaration with none fails the build, named.
    template <class Declared>
    [[nodiscard]] consteval Until operator()(const Declared& other) const {
        detail::needsPieceSensor(other);
        return Until::stopsSeeing(other);
    }
};
/// Until the step's piece sensor (or another's, `seesPiece(grip)`) sees a piece.
inline constexpr SeesPiece seesPiece{};
/// Until it stops seeing one.
inline constexpr StopsSeeingPiece stopsSeeingPiece{};
/// Until the mechanism stalls (a squeeze: SUCCEEDED).
inline constexpr Until stalls = Until::stalled();

/// The auton's condition words, beside the robot file's (kinds.hpp, wiring.hpp): an auton that says
/// `using namespace robot_two;` — whose namespace says `using namespace mechanism::words;` — has them.
namespace words {
/// Until a piece sensor sees a piece.
using mechanism::seesPiece;
/// Until a piece sensor stops seeing one.
using mechanism::stopsSeeingPiece;
/// Until the mechanism stalls.
using mechanism::stalls;
}  // namespace words

}  // namespace shulib::mechanism
