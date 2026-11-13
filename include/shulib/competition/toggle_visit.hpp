#pragma once
//
// competition::checkAndFlipToggle — the Toggle flip for a SKILLS run, as ONE route step (chunk AF1S-A1,
// 2026-11-11; the AF1S brief's D1–D11 and its re-check R1–R12, the team lead's answers of 2026-11-10).
// At a LOOK SPOT the route parked the robot on, square in front of a named Toggle, it looks at the
// Toggle with the front AI Vision camera, does NOTHING if the Toggle already shows its Quadrant's
// colour, and otherwise drives straight in, pushes with the flipper, retracts, drives straight back
// out and looks again — a second push only when the first turned the Toggle to the THIRD colour,
// never a third. Every doubt does nothing more and is reported; the route's own choice after a failed
// step (its onFail, or the default stop) decides what comes next. The form this run gives is the
// EXPLICIT one, over handles (the host tests and a hand-wired Tier-3 program); the everyday form over
// an auton's context is AF1S-A2's.
//
//     r.then([&] {
//         return competition::checkAndFlipToggle(cam, ctx.chassis, ctx.guard, ctx.mechanismService,
//                                                ctx.entry, kRedWallVisit, ctx.telemetry).outcome();
//     }, "red wall Toggle");
//
// ── "Ours" in a skills run ──────────────────────────────────────────────────────────────────────
// Each Toggle's QUADRANT colour (vision/override_elements.hpp's table, A4 register HA-243 — a reading
// of the manual's drawings, v2.0 <RSC3>a-iii-1: "Toggles only confer Ownership of Pins when the Toggle
// matches the color of its Quadrant"; v2.1's change log keeps it in substance). The route names WHICH
// Toggle; "ours" is never typed, so it cannot be typed wrong. A MATCH auton is refused (NOT SKILLS):
// its "ours" is the alliance's, which comes after the 21st (ALLY1).
//
// ── What "FLIPPED" means — and what it cannot (A4 register HA-232) ──────────────────────────────
// It means "the Toggle SHOWS our colour after a FULL stroke" — never "it is set". The game counts a
// Toggle as set only when it is fully seated and no robot touches it (v2.0 <SC4>); a Toggle stuck
// part-way through its roll shows the face it is rolling TO, and the camera reads that colour "sure"
// (VIS1-A2's banner; the AF1S brief's probe P5(c): 58 of 108 stuck positions). So a success line
// always ENDS "seating not checked", and a stroke that stopped short — a push that STALLED, TIMED OUT
// or was CANCELLED, or a passive drive that did not reach the push spot — and then reads ours is NOT a
// success: OURS AFTER SHORT STROKE. That the flipper pushes on the camera's word ALONE is the team
// lead's ruling (2026-11-03, "just the ai vision sensor", confirmed for this step on 2026-11-10 — the
// brief's D11): no sensor sees colour at the push point.
//
// ── The rules, in order (each one a test) ───────────────────────────────────────────────────────
//   1. ALLOWED — reading NOTHING, commanding nothing: a SKILLS auton (else NOT SKILLS); a usable camera
//      (the feed, a measured mount, a tracker config the tracker accepts — else NO CAMERA); the flipper
//      found on this robot, built, homed, idle and AT `back` within its "reached" tolerance — a
//      pneumatic's `flipBound` and `backBound` each at least its move time plus its confirm window,
//      the most its slot waits for a verdict, so rule 4 never counts less than the step will wait
//      (else FLIPPER NOT READY — the step NEVER moves it there itself: the route does); with a FIELD start,
//      the estimate's heading within +-45 degrees of the direction TOWARD the named Toggle's wall —
//      minus kTogglePlaces' inward normal: right wall 0, blue wall +90, left wall 180, red wall -90
//      (else WRONG WALL). A RELATIVE start skips the wall check and the line says so.
//   2. LOOK — a FRESH tracker, fed from the look's start on every read of the feed (ticked inside the
//      run guard's deadline-aware wait, so the mechanism service and any motion keep ticking and a
//      deadline cuts it), with the estimate and the caller's pose bound widened for the image's age
//      (vision::boundAtImage, `maxImageAge`) — up to `lookWithin` for a SURE colour, else UNKNOWN
//      BEFORE with the tracker's reason. Sure, the band must be AT THE SPOT: its WHOLE range interval
//      inside [lookFrom - offBy, lookFrom + offBy] — CONTAINMENT, never overlap (the interval is 5–18
//      in wide on the host camera model: an overlap test accepted a robot 8–11 in too far) — and its
//      whole bearing interval inside the tracker's window; with a FIELD start, also the estimate's lens
//      within `offBy` of `lookFrom` from the wall's inner face, along its normal — which takes the
//      Toggle's face to sit AT that inner face (where it sits is UNVERIFIED, A4 register HA-201: a real
//      set-back shifts this window by that much; Step 0.4 measures it). Else NOT AT THE SPOT. This check is
//      COARSE: the camera catches only gross arrival errors; where the flipper lands is the route's
//      odometry (A4 register HA-244).
//   3. OURS ALREADY — ALREADY OURS, nothing pushed.
//   4. TIME — need(n) = n x (ceil(driveBound) + ceil(maxImageAge) + ceil(flipBound) + ceil(backBound) +
//      ceil(settle) + ceil(driveBound) + ceil(lookWithin) + 2 ticks) + margin, ceil() up to the 10-ms
//      tick (ceil(maxImageAge): the most the drive in HOLDS on "camera frozen?", rule 5). Push 1
//      only when the guard's remaining() >= need(maxPushes) (a visit that may need two pushes is not
//      begun unless both fit), push 2 only when >= need(1). Else NOT ENOUGH TIME / NO TIME FOR PUSH 2.
//      worstCaseSeconds(request) is need(maxPushes) — the look before it adds at most `lookWithin`.
//   5. FLIP, every push PAIRED — drive straight IN by lookFrom - pushFrom along the current heading
//      (NEVER a blocking Chassis verb: a motion::MoveToPose started with scheduler().async and waited
//      through the guard, its predicate ticking the feed and the tracker on every read, cancelled on
//      every exit); command `flipper`; on ANY verdict, `back` at once; wait for it; `settle`; drive
//      straight back OUT to the look spot. The camera SEEN FROZEN on the drive in (the lens moved past
//      `frozenMoveIn` and the band's box did not move, still so on a picture taken after that was first
//      seen — readAt - maxImageAge, HA-192: a live camera's lagging pictures catch up within
//      maxImageAge — so the look the push would rest on may be a stale picture): the drive in stops
//      there, nothing is pushed, the robot drives out (FROZEN BEFORE PUSH). Raised but not yet that old
//      (a maxImageAge longer than the rest of the drive in), the step HOLDS before anything reaches the
//      Toggle — an active flipper's drive in arrives and nothing is pushed yet; a PASSIVE bar's drive
//      in (the stroke itself) stops where the question is raised — and keeps reading, at most
//      ceil(maxImageAge) a drive in (rule 4 counts it): the evidence turning stale, or the question
//      still open when that runs out, is FROZEN BEFORE PUSH; the question clearing (a live camera's
//      lagging pictures caught up) lets the push go on (a passive bar drives on, within what is left of
//      driveBound). With a pose bound stated, this check (and rule 6's "camera frozen?") reads an
//      UNGATED twin of the tracker: the position gate drops a frozen picture's band as "no band" once
//      the lens has moved it off the Toggle's place. A drive in that did not arrive: nothing pushed, the robot drives out (DRIVE FAILED); a
//      drive OUT that did not arrive: DRIVE FAILED, no look after (it would not be from the look
//      spot). A retract that does not reach `back`: FLIPPER STUCK and the robot does
//      NOT drive out (a flipper caught on the Toggle would drag it — <GG9>). If, while the flipper is
//      OUT (from the push's command until `back`'s; a passive bar: from the end of the drive in until
//      the drive out), the time left falls to backBound + settle + driveBound + margin, `back` is
//      commanded AT ONCE (the push is replaced), there is no settle, the robot drives out as soon as it
//      is back, and the visit ends RETRACTED FOR TIME — the robot must be OFF the Toggle at 0:00
//      (<SC4>b). Rule 4 makes that unreachable while every stage keeps to its bound; it is for time lost
//      OUTSIDE them (a brain's loop that stalled). A PASSIVE flipper (a fixed bar): the drive in and out
//      IS the stroke; no mechanism is commanded.
//   6. LOOK AGAIN — moving, and only after the retract (the brief's re-check R3, R4, R8). The tracker
//      has been fed on EVERY read since the look before (so a camera that froze anywhere — at the look
//      spot, on the way in, at the push spot, on the way out — has been moved past `frozenMoveIn` and
//      reads "camera frozen?"); the after-colour counts only from a SURE view on a read whose IMAGE is
//      after the retract's end (readAt - maxImageAge >= the retract's end — the read's time is not the
//      picture's, HA-192) AND after the lens has moved more than `frozenMoveIn` from where it was at
//      the retract's end (belt and braces: the look after begins only once the drive out SETTLED at
//      the look spot, and the excursion is more than twice `frozenMoveIn`, so with any settle
//      tolerance under the excursion less `frozenMoveIn` this clause cannot refuse a read — the one
//      host test that reaches it settles within 11 in of a 12-in excursion) — up to `lookWithin` after
//      reaching the look spot. Ours after a FULL stroke:
//      FLIPPED. Ours after a short one: OURS AFTER SHORT STROKE. The colour it had before: NO CHANGE (a
//      sticky Toggle, or a camera frozen since before the push — both stop). Not sure: UNKNOWN AFTER.
//      Push 1 and the THIRD colour: rule 4's push-2 check, then 5 and 6 once more; push 2 not ours:
//      STILL NOT OURS. Never a third push.
//   7. ONE [TGL] line per visit through the caller's sink — Info for the two successes, Warn otherwise;
//      ASCII, at most 160 bytes; every success line ends "seating not checked". The service's own
//      [MECH] lines for each push and retract appear beside it.
//
// ── What it cannot catch (stated, not hidden) ───────────────────────────────────────────────────
// A Toggle stuck part-way that reads as its destination (above); a camera frozen while the robot is
// parked, IF the step were changed to look from where it pushed (rule 6 is why it cannot); a camera
// frozen at the look spot whose stale picture shows the Toggle is ALREADY ours — nothing is driven,
// so nothing can show it frozen (a stale not-ours picture IS caught: the drive in moves the lens, rule
// 5); a PASSIVE bar that reaches the Toggle before the lens has moved `frozenMoveIn` on the drive in
// (before a frozen camera CAN look frozen) strokes it on the stale look, and the verdict is still
// FROZEN BEFORE PUSH (host test plant: a bar touching 2 in into a 16-in drive in); a Toggle that changes after the look, while the camera is still live on the drive in (too close
// to name a colour at most mounts); for a PNEUMATIC flipper, whether the cylinder truly reached the
// end of its stroke or truly went back — its slot reports SUCCEEDED once the commanded level has been
// held for its move time (nothing senses the rod), so for a pneumatic "a FULL stroke" and "reached
// `back`" mean "the move time passed": a cylinder blocked part-way by a jammed Toggle still reads as
// FLIPPED's full stroke, and one caught on the Toggle on the way back is driven out with it — OURS
// AFTER A SHORT STROKE and FLIPPER STUCK cannot come from a pneumatic (A4 register HA-235); a lookalike
// band the tracker's gates cannot tell from a face (A4 register HA-199 — the position gate, with a pose
// the team trusts, is what turns those away); a robot touching the Toggle after the step ends; the
// pieces the robot holds (<SG6>: holding more than one Pin or one Cup, it may not touch a Toggle — the
// route's to respect).
//
// A programming error is LOUD (a precondition): the guard not running; a request that names no Toggle,
// leaves a number UNSET, puts the push spot at or beyond the look spot, an `offBy` of `lookFrom` or
// more, a flipper and a back of two mechanisms or the same goal twice, or `maxPushes` other than 1 or
// 2 — refused AT BUILD when the request is built through toggleVisit(...) as a constant; and an
// excursion lookFrom - pushFrom of twice the tracker's `frozenMoveIn` or less (the look after could
// not see a frozen camera).
//
// PROS-free, sim-free, allocation-free; single-task. It never runs in the background and never reads
// the camera outside its own waits. NOTHING HERE HAS RUN ON A ROBOT: every rule has run against the
// host test plant (the drive, the arm and the host Toggle and camera models) and nothing else.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>

#include "shulib/chassis/chassis.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/config/camera_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/camera_feed.hpp"
#include "shulib/vision/camera_geometry.hpp"
#include "shulib/vision/override_elements.hpp"
#include "shulib/vision/toggle_tracker.hpp"

namespace shulib::competition {

/// The control tick the time check rounds every bound UP to, seconds: the brain's 10-ms tick
/// (hal::pros::ProsTickPacer::kTickMs, A4 register HA-32) — the grid every wait ends on.
inline constexpr double kToggleVisitTickSeconds = 0.010;

/// Slack past a wait's own bound given to its backstop timeout, seconds. Unreachable by construction
/// (each wait's predicate, the mechanism service's deadline or the motion's watchdog ends it first);
/// code-level belt and braces, not an HA entry (Routine's kMechanismWaitSlackSeconds is the same idea).
inline constexpr double kToggleVisitWaitSlackSeconds = 0.25;

/// WHICH flipper and which of its goals — a preset of ONE Arm or Lift, a position of ONE Pneumatic, or
/// (default-constructed) a PASSIVE flipper: a fixed bar the robot's own drive in and out strokes. It
/// converts IMPLICITLY from a mechanism goal, so `.flipper = flipper["flip"]`.
struct FlipperGoal {
    /// Which kind of flipper the goal is for.
    enum class Kind : std::uint8_t {
        Passive = 0,  ///< no mechanism: the drive in and out is the stroke (default)
        Position,     ///< an Arm's or a Lift's preset or constant
        Pneumatic,    ///< a Pneumatic's named position
    };
    /// The kind (Passive when default-constructed).
    Kind kind = Kind::Passive;
    /// The goal when Position.
    mechanism::PositionGoal position{};
    /// The goal when Pneumatic.
    mechanism::PneumaticGoal pneumatic{};

    /// A PASSIVE flipper.
    constexpr FlipperGoal() noexcept = default;
    /// An Arm's or a Lift's goal (`arm["flip"]`, `lift.to(6_in)`).
    constexpr FlipperGoal(const mechanism::PositionGoal& g) noexcept  // NOLINT: implicit on purpose
        : kind{Kind::Position}, position{g} {}
    /// A Pneumatic's position (`flipper["out"]`).
    constexpr FlipperGoal(const mechanism::PneumaticGoal& g) noexcept  // NOLINT: implicit on purpose
        : kind{Kind::Pneumatic}, pneumatic{g} {}

    /// True for a passive flipper.
    [[nodiscard]] constexpr bool passive() const noexcept { return kind == Kind::Passive; }
    /// The mechanism's name ("" for a passive one).
    [[nodiscard]] constexpr const char* mechanismName() const noexcept {
        const char* n = kind == Kind::Position    ? position.mechanismName
                        : kind == Kind::Pneumatic ? pneumatic.mechanismName
                                                  : "";
        return n != nullptr ? n : "";
    }
};

/// One visit's numbers — every one the TEAM's, from Step 0 at the robot; 0 = UNSET and refused by name
/// (the core's rule: no library default). `flipBound` and `backBound` may stay 0 for a PASSIVE flipper
/// (nothing is commanded); any value given is still counted in the time check.
struct ToggleVisitRequest {
    /// WHICH Toggle (`toggles::kRedWall`). "Ours" is its Quadrant's colour (HA-243).
    vision::override2026::ToggleId toggle{};
    /// The push: a preset of ONE Arm or Lift, a position of ONE Pneumatic, or default = PASSIVE.
    FlipperGoal flipper{};
    /// The retract: another goal of the SAME mechanism (default = PASSIVE, with a passive `flipper`).
    FlipperGoal back{};
    /// The LOOK SPOT: lens to the Toggle's face, horizontal, inches (Step 0.4). That the camera names the colour
    /// from there on a real field is A4 register HA-245 (the host camera model's prediction, unmeasured). With
    /// a FIELD start it is ALSO compared with the estimate's lens distance from the wall's INNER face — the
    /// face taken to sit at that inner face (A4 register HA-201: UNVERIFIED).
    units::Length lookFrom{};
    /// How far off the look spot is accepted: the larger half-width of the range interval the bench page
    /// prints AT the look spot, plus the route's arrival allowance, inches (Step 0.4 — no default). A COARSE
    /// check: where the flipper lands is the route's driving (A4 register HA-244).
    units::Length offBy{};
    /// The PUSH SPOT: lens to the face where the flipper (or the bar) rolls the Toggle, inches (Step 0.5; HA-244).
    units::Length pushFrom{};
    /// The longest a look waits for a sure colour, seconds (Step 0.3).
    units::Time lookWithin{};
    /// The push's own time limit, seconds (Step 0.1: the worst of five). It, or the flipper's stall signature,
    /// ends a push a sticky Toggle stops before it damages the Toggle — A4 register HA-235. For a pneumatic it
    /// must be at least the cylinder's move time plus its confirm window (else FLIPPER NOT READY).
    units::Time flipBound{};
    /// The retract's own time limit, seconds (Step 0.1). For a pneumatic, at least its move time plus its
    /// confirm window, as `flipBound`.
    units::Time backBound{};
    /// How long the Toggle takes to sit flat after the retract, seconds (Step 0.1; HA-232).
    units::Time settle{};
    /// The drive in's — and the drive out's — own time limit, seconds (Step 0.6; HA-244).
    units::Time driveBound{};
    /// Spare time kept at the end of the visit, seconds.
    units::Time margin{};
    /// The camera's image age at its read, seconds (Step 0.2; HA-192) — rule 2's pose bound, rule 5's
    /// frozen check (how long the drive in may HOLD to tell a frozen camera from a lagging one — counted
    /// in rule 4) and rule 6's "a picture from after the retract".
    units::Time maxImageAge{};
    /// At most this many pushes: 1 or 2 (a 3 is refused). On the research's reading every skills Toggle
    /// needs TWO from yellow, through the other colour (A4 register HA-231) — the rule holds either way.
    std::uint8_t maxPushes = 2;
};

/// What is wrong with a request, if anything (each a refusal, at build through toggleVisit(...) or at
/// run time as a precondition).
enum class ToggleVisitProblem : std::uint8_t {
    None = 0,             ///< nothing
    ToggleUnset,          ///< no Toggle named
    NotFinite,            ///< a number is NaN or infinite
    LookFromUnset,        ///< lookFrom UNSET
    OffByUnset,           ///< offBy UNSET
    PushFromUnset,        ///< pushFrom UNSET
    PushNotNearer,        ///< pushFrom >= lookFrom: the push spot must be nearer than the look spot
    OffByTooLarge,        ///< offBy >= lookFrom: the window would reach the face
    LookWithinUnset,      ///< lookWithin UNSET
    FlipBoundUnset,       ///< flipBound UNSET for a mechanism flipper (or negative)
    BackBoundUnset,       ///< backBound UNSET for a mechanism flipper (or negative)
    SettleUnset,          ///< settle UNSET
    DriveBoundUnset,      ///< driveBound UNSET
    MarginUnset,          ///< margin UNSET
    MaxImageAgeUnset,     ///< maxImageAge UNSET
    MaxPushes,            ///< maxPushes not 1 or 2
    FlipperKindsDiffer,   ///< the flipper and the back are of different kinds (one passive, or arm vs pneumatic)
    TwoMechanisms,        ///< the flipper and the back name two mechanisms
    SameGoalTwice,        ///< the flipper and the back are the same goal: no stroke
};

namespace detail {

/// Two goals of the SAME position mechanism (name and units) — and a different target.
[[nodiscard]] constexpr ToggleVisitProblem flipperPairProblem(const FlipperGoal& f, const FlipperGoal& b) noexcept {
    if (f.kind != b.kind) {
        return ToggleVisitProblem::FlipperKindsDiffer;
    }
    if (f.kind == FlipperGoal::Kind::Position) {
        if (!mechanism::detail::same(f.position.mechanismName, b.position.mechanismName)
            || f.position.units != b.position.units) {
            return ToggleVisitProblem::TwoMechanisms;
        }
        if (f.position.value == b.position.value) {
            return ToggleVisitProblem::SameGoalTwice;
        }
    } else if (f.kind == FlipperGoal::Kind::Pneumatic) {
        if (!mechanism::detail::same(f.pneumatic.mechanismName, b.pneumatic.mechanismName)) {
            return ToggleVisitProblem::TwoMechanisms;
        }
        if (f.pneumatic.position == b.pneumatic.position) {
            return ToggleVisitProblem::SameGoalTwice;
        }
    }
    return ToggleVisitProblem::None;
}

// The consteval refusals (the mechanism core's idiom, mechanism/kinds.hpp): calling one (it is not
// constexpr) stops the build, and GCC names it — the message a student reads is the function's name.
/// No Toggle named.
inline void name_which_toggle_like_toggles_kRedWall() {}
/// A number is NaN or infinite.
inline void a_toggle_visit_number_is_not_finite() {}
/// A number the step needs is UNSET.
inline void a_toggle_visit_number_is_unset_every_one_is_the_teams_from_step_0() {}
/// The push spot is not nearer than the look spot.
inline void the_push_spot_must_be_nearer_than_the_look_spot() {}
/// offBy reaches the face.
inline void offBy_must_be_less_than_lookFrom() {}
/// maxPushes is not 1 or 2.
inline void maxPushes_is_1_or_2_never_3() {}
/// One goal passive and one not, or two kinds.
inline void the_flipper_and_back_are_different_kinds_of_flipper() {}
/// Two mechanisms.
inline void the_flipper_and_back_name_two_mechanisms() {}
/// The same goal twice.
inline void the_flipper_and_back_are_the_same_goal() {}

}  // namespace detail

/// The first problem with `r`, or None. Pure, total, constexpr.
[[nodiscard]] constexpr ToggleVisitProblem toggleVisitProblemOf(const ToggleVisitRequest& r) noexcept {
    if (vision::override2026::toggleIndex(r.toggle) < 0) {
        return ToggleVisitProblem::ToggleUnset;
    }
    const double numbers[] = {r.lookFrom.value(),  r.offBy.value(),  r.pushFrom.value(), r.lookWithin.value(),
                              r.flipBound.value(), r.backBound.value(), r.settle.value(), r.driveBound.value(),
                              r.margin.value(),    r.maxImageAge.value()};
    for (const double n : numbers) {
        if (!config::finiteNumber(n)) {
            return ToggleVisitProblem::NotFinite;
        }
    }
    if (!(r.lookFrom.value() > 0.0)) {
        return ToggleVisitProblem::LookFromUnset;
    }
    if (!(r.offBy.value() > 0.0)) {
        return ToggleVisitProblem::OffByUnset;
    }
    if (!(r.pushFrom.value() > 0.0)) {
        return ToggleVisitProblem::PushFromUnset;
    }
    if (!(r.pushFrom.value() < r.lookFrom.value())) {
        return ToggleVisitProblem::PushNotNearer;
    }
    if (!(r.offBy.value() < r.lookFrom.value())) {
        return ToggleVisitProblem::OffByTooLarge;
    }
    if (!(r.lookWithin.value() > 0.0)) {
        return ToggleVisitProblem::LookWithinUnset;
    }
    const bool passive = r.flipper.passive();
    if (passive ? r.flipBound.value() < 0.0 : !(r.flipBound.value() > 0.0)) {
        return ToggleVisitProblem::FlipBoundUnset;
    }
    if (passive ? r.backBound.value() < 0.0 : !(r.backBound.value() > 0.0)) {
        return ToggleVisitProblem::BackBoundUnset;
    }
    if (!(r.settle.value() > 0.0)) {
        return ToggleVisitProblem::SettleUnset;
    }
    if (!(r.driveBound.value() > 0.0)) {
        return ToggleVisitProblem::DriveBoundUnset;
    }
    if (!(r.margin.value() > 0.0)) {
        return ToggleVisitProblem::MarginUnset;
    }
    if (!(r.maxImageAge.value() > 0.0)) {
        return ToggleVisitProblem::MaxImageAgeUnset;
    }
    if (r.maxPushes != 1 && r.maxPushes != 2) {
        return ToggleVisitProblem::MaxPushes;
    }
    return detail::flipperPairProblem(r.flipper, r.back);
}

/// The problem as a sentence for a person, or nullptr for None.
[[nodiscard]] constexpr const char* toggleVisitProblemSentence(ToggleVisitProblem p) noexcept {
    switch (p) {
        case ToggleVisitProblem::None: return nullptr;
        case ToggleVisitProblem::ToggleUnset: return "no Toggle named -- .toggle = toggles::kRedWall (or another)";
        case ToggleVisitProblem::NotFinite: return "a number is not a finite number";
        case ToggleVisitProblem::LookFromUnset: return "lookFrom UNSET -- lens to the face at the look spot (Step 0.4)";
        case ToggleVisitProblem::OffByUnset: return "offBy UNSET -- the look spot's tolerance (Step 0.4)";
        case ToggleVisitProblem::PushFromUnset: return "pushFrom UNSET -- lens to the face at the push spot (Step 0.5)";
        case ToggleVisitProblem::PushNotNearer: return "pushFrom must be nearer the Toggle than lookFrom";
        case ToggleVisitProblem::OffByTooLarge: return "offBy must be less than lookFrom";
        case ToggleVisitProblem::LookWithinUnset: return "lookWithin UNSET -- how long a look waits (Step 0.3)";
        case ToggleVisitProblem::FlipBoundUnset: return "flipBound UNSET -- the push's time limit (Step 0.1)";
        case ToggleVisitProblem::BackBoundUnset: return "backBound UNSET -- the retract's time limit (Step 0.1)";
        case ToggleVisitProblem::SettleUnset: return "settle UNSET -- how long the Toggle takes to sit flat (Step 0.1)";
        case ToggleVisitProblem::DriveBoundUnset: return "driveBound UNSET -- the drive in's time limit (Step 0.6)";
        case ToggleVisitProblem::MarginUnset: return "margin UNSET -- the spare time kept at the end";
        case ToggleVisitProblem::MaxImageAgeUnset: return "maxImageAge UNSET -- the camera's image age (Step 0.2)";
        case ToggleVisitProblem::MaxPushes: return "maxPushes must be 1 or 2";
        case ToggleVisitProblem::FlipperKindsDiffer: return "the flipper and back are different kinds of flipper";
        case ToggleVisitProblem::TwoMechanisms: return "the flipper and back name two mechanisms";
        case ToggleVisitProblem::SameGoalTwice: return "the flipper and back are the same goal (no stroke)";
    }
    return "an unknown problem";
}

/// The first problem with `r` as a sentence, or nullptr.
[[nodiscard]] constexpr const char* toggleVisitRequestProblem(const ToggleVisitRequest& r) noexcept {
    return toggleVisitProblemSentence(toggleVisitProblemOf(r));
}

/// A request checked AT BUILD: `inline constexpr auto kRedWallVisit = competition::toggleVisit({...});`
/// — a problem stops the build, naming it (the core's build-check idiom). A request built while the
/// auton runs is checked by the step itself, as a precondition.
[[nodiscard]] consteval ToggleVisitRequest toggleVisit(ToggleVisitRequest r) {
    switch (toggleVisitProblemOf(r)) {
        case ToggleVisitProblem::None: break;
        case ToggleVisitProblem::ToggleUnset: detail::name_which_toggle_like_toggles_kRedWall(); break;
        case ToggleVisitProblem::NotFinite: detail::a_toggle_visit_number_is_not_finite(); break;
        case ToggleVisitProblem::LookFromUnset:
        case ToggleVisitProblem::OffByUnset:
        case ToggleVisitProblem::PushFromUnset:
        case ToggleVisitProblem::LookWithinUnset:
        case ToggleVisitProblem::FlipBoundUnset:
        case ToggleVisitProblem::BackBoundUnset:
        case ToggleVisitProblem::SettleUnset:
        case ToggleVisitProblem::DriveBoundUnset:
        case ToggleVisitProblem::MarginUnset:
        case ToggleVisitProblem::MaxImageAgeUnset:
            detail::a_toggle_visit_number_is_unset_every_one_is_the_teams_from_step_0();
            break;
        case ToggleVisitProblem::PushNotNearer: detail::the_push_spot_must_be_nearer_than_the_look_spot(); break;
        case ToggleVisitProblem::OffByTooLarge: detail::offBy_must_be_less_than_lookFrom(); break;
        case ToggleVisitProblem::MaxPushes: detail::maxPushes_is_1_or_2_never_3(); break;
        case ToggleVisitProblem::FlipperKindsDiffer: detail::the_flipper_and_back_are_different_kinds_of_flipper(); break;
        case ToggleVisitProblem::TwoMechanisms: detail::the_flipper_and_back_name_two_mechanisms(); break;
        case ToggleVisitProblem::SameGoalTwice: detail::the_flipper_and_back_are_the_same_goal(); break;
    }
    return r;
}

/// `x` seconds rounded UP to the 10-ms tick (a value already on the grid stays).
[[nodiscard]] inline double toggleVisitCeilTick(double x) noexcept {
    return std::ceil(x / kToggleVisitTickSeconds - 1e-9) * kToggleVisitTickSeconds;
}

/// Rule 4's need(n), seconds: n x (ceil(driveBound) + ceil(maxImageAge) + ceil(flipBound) + ceil(backBound) +
/// ceil(settle) + ceil(driveBound) + ceil(lookWithin) + 2 ticks) + margin — every bound rounded UP to the
/// tick; ceil(maxImageAge) is the most the drive in may HOLD on "camera frozen?" (rule 5).
[[nodiscard]] inline double toggleVisitNeedSeconds(const ToggleVisitRequest& r, int pushes) noexcept {
    const double one = toggleVisitCeilTick(r.driveBound.value()) + toggleVisitCeilTick(r.maxImageAge.value())
                       + toggleVisitCeilTick(r.flipBound.value())
                       + toggleVisitCeilTick(r.backBound.value()) + toggleVisitCeilTick(r.settle.value())
                       + toggleVisitCeilTick(r.driveBound.value()) + toggleVisitCeilTick(r.lookWithin.value())
                       + 2.0 * kToggleVisitTickSeconds;
    return pushes * one + r.margin.value();
}

/// The time check's own sum for the whole visit: need(maxPushes), seconds — what a route's budget (and
/// the route preview, RPV1) should allow for this step after the look before it (itself at most
/// `lookWithin`). The ideal host plant flips in less; the robot will not.
[[nodiscard]] inline double worstCaseSeconds(const ToggleVisitRequest& r) noexcept {
    return toggleVisitNeedSeconds(r, r.maxPushes);
}

/// How a visit ended (appended-only, like every enum a route shares).
enum class ToggleVisitVerdict : std::uint8_t {
    Unset = 0,             ///< a default report — no visit ran (never returned by the step)
    AlreadyOurs,           ///< SUCCEEDS: it showed ours before; nothing pushed (seating not checked)
    Flipped,               ///< SUCCEEDS: it showed ours after a FULL stroke (seating not checked)
    NotSkills,             ///< not a SKILLS auton: no "ours" (the alliance's comes with ALLY1)
    NoCamera,              ///< no usable camera for this step
    FlipperNotReady,       ///< the flipper absent, not homed, busy, or not at `back` (never moved there)
    WrongWall,             ///< a FIELD start, and the robot does not face the named Toggle's wall
    UnknownBefore,         ///< no sure colour at the look (the tracker's reason): nothing pushed
    NotAtSpot,             ///< the band is not wholly inside the look spot's window: nothing pushed or driven
    NotEnoughTime,         ///< the time left does not cover need(maxPushes): nothing pushed or driven
    NoChange,              ///< after push 1 it shows the colour it had before (sticky, or a frozen camera)
    StillNotOurs,          ///< after the last allowed push it is still not ours
    UnknownAfter,          ///< no sure colour after the push (the reason): it stops
    NoTimeForSecondPush,   ///< push 1 turned it to the third colour; need(1) does not fit
    FlipperStuck,          ///< the retract did not reach `back`: the robot stays (it does not drag the Toggle)
    Cut,                   ///< the run's deadline or a disable cut the step
    OursAfterShortStroke,  ///< ours after a stroke that stopped short — maybe stuck part-way: NOT counted
    RetractedForTime,      ///< the retract-now rule fired: back at once, out, and done
    // APPENDED at AF1S-A1 beyond the brief's list (the brief names no verdict for these three):
    DriveFailed,           ///< the drive in or out did not arrive within driveBound
    SeenNotOurs,           ///< lookAtToggle only: a sure colour at the spot, not ours (the look call never pushes)
    FrozenBeforePush,      ///< the camera seen frozen on the drive in: nothing pushed, the robot drove out
};

/// The verdict as words for a log line, a bench page or the route preview.
[[nodiscard]] constexpr const char* toggleVisitWord(ToggleVisitVerdict v) noexcept {
    switch (v) {
        case ToggleVisitVerdict::Unset: return "UNSET";
        case ToggleVisitVerdict::AlreadyOurs: return "ALREADY OURS";
        case ToggleVisitVerdict::Flipped: return "FLIPPED";
        case ToggleVisitVerdict::NotSkills: return "NOT SKILLS";
        case ToggleVisitVerdict::NoCamera: return "NO CAMERA";
        case ToggleVisitVerdict::FlipperNotReady: return "FLIPPER NOT READY";
        case ToggleVisitVerdict::WrongWall: return "WRONG WALL";
        case ToggleVisitVerdict::UnknownBefore: return "UNKNOWN BEFORE";
        case ToggleVisitVerdict::NotAtSpot: return "NOT AT THE SPOT";
        case ToggleVisitVerdict::NotEnoughTime: return "NOT ENOUGH TIME";
        case ToggleVisitVerdict::NoChange: return "NO CHANGE";
        case ToggleVisitVerdict::StillNotOurs: return "STILL NOT OURS";
        case ToggleVisitVerdict::UnknownAfter: return "UNKNOWN AFTER";
        case ToggleVisitVerdict::NoTimeForSecondPush: return "NO TIME FOR PUSH 2";
        case ToggleVisitVerdict::FlipperStuck: return "FLIPPER STUCK";
        case ToggleVisitVerdict::Cut: return "CUT";
        case ToggleVisitVerdict::OursAfterShortStroke: return "OURS AFTER A SHORT STROKE";
        case ToggleVisitVerdict::RetractedForTime: return "RETRACTED FOR TIME";
        case ToggleVisitVerdict::DriveFailed: return "DRIVE FAILED";
        case ToggleVisitVerdict::SeenNotOurs: return "SEEN, NOT OURS";
        case ToggleVisitVerdict::FrozenBeforePush: return "FROZEN BEFORE PUSH";
    }
    return "UNKNOWN";
}

/// A Quadrant's colour as the camera's colour (RED, BLUE; Unknown for Unset).
[[nodiscard]] constexpr vision::ToggleColour toggleColourOf(vision::override2026::QuadrantColour q) noexcept {
    switch (q) {
        case vision::override2026::QuadrantColour::Red: return vision::ToggleColour::Red;
        case vision::override2026::QuadrantColour::Blue: return vision::ToggleColour::Blue;
        case vision::override2026::QuadrantColour::Unset: break;
    }
    return vision::ToggleColour::Unknown;
}

/// "Ours" for `id` in a skills run: its Quadrant's colour as the camera names it (Unknown for Unset).
[[nodiscard]] constexpr vision::ToggleColour oursFor(vision::override2026::ToggleId id) noexcept {
    return toggleColourOf(vision::override2026::quadrantColour(id));
}

/// One visit, as a VALUE (rule 7's line says the same in words).
struct ToggleVisitReport {
    /// How it ended.
    ToggleVisitVerdict verdict = ToggleVisitVerdict::Unset;
    /// Which Toggle.
    vision::override2026::ToggleId toggle{};
    /// The target: its Quadrant's colour.
    vision::ToggleColour ours = vision::ToggleColour::Unknown;
    /// The colour at the look before (Unknown when not sure).
    vision::ToggleColour before = vision::ToggleColour::Unknown;
    /// The colour at the last look after (Unknown when not sure or none was made).
    vision::ToggleColour after = vision::ToggleColour::Unknown;
    /// The tracker's reason at the end of the look before.
    vision::ToggleWhy whyBefore = vision::ToggleWhy::NoRead;
    /// The tracker's reason at the end of the last look after.
    vision::ToggleWhy whyAfter = vision::ToggleWhy::NoRead;
    /// True when the last look after saw a sure colour it could not count: no picture from after the
    /// retract, or the lens not yet moved past `frozenMoveIn` since it.
    bool afterTooEarly = false;
    /// Pushes made (a passive stroke counts as one).
    std::uint8_t pushes = 0;
    /// Whether each push's stroke was FULL (the push SUCCEEDED after a drive in that arrived; a passive
    /// drive that reached the push spot).
    bool fullStroke[2]{false, false};
    /// The band's range interval at the look before, inches (valid when a band passed).
    vision::Interval range{};
    /// The band's bearing interval at the look before, radians (valid when a band passed).
    vision::Interval bearing{};
    /// True when the tracker's POSITION gate ran on the look before's sure view (the tracker's config
    /// turns it on, and the camera handles stated a pose bound it accepts).
    bool positionChecked = false;
    /// With a FIELD start: the estimate's lens distance from the Toggle's wall at the look, inches.
    double lensFromWallIn = 0.0;
    /// True when the wall and the field distance were checked (a FIELD start).
    bool wallChecked = false;
    /// The guard's remaining() when the step began, seconds.
    double secondsLeftAtStart = 0.0;
    /// The guard's remaining() at the last time check, seconds (0 when none was made).
    double secondsLeftAtCheck = 0.0;
    /// What that check needed, seconds (0 when none was made).
    double secondsNeeded = 0.0;
    /// The step's own time, seconds.
    double seconds = 0.0;
    /// Each push's StepReport (a default one for a passive stroke or a push never made).
    mechanism::StepReport push[2]{};
    /// Each retract's StepReport (likewise).
    mechanism::StepReport back[2]{};
    /// The drive in's and out's exit for the last stroke (Running when none was made).
    control::ExitReason driveIn = control::ExitReason::Running;
    /// Likewise, the drive out.
    control::ExitReason driveOut = control::ExitReason::Running;
    /// A short reason beside some verdicts (FLIPPER NOT READY's, NO CAMERA's, NOT AT THE SPOT's) — a
    /// library literal or the flipper's absence sentence; "" when none.
    const char* detail = "";

    /// True for the two successes: ALREADY OURS and FLIPPED.
    [[nodiscard]] constexpr bool ok() const noexcept {
        return verdict == ToggleVisitVerdict::AlreadyOurs || verdict == ToggleVisitVerdict::Flipped;
    }
    /// What a route's then() takes: SUCCEEDED for ok(); CANCELLED for CUT (the route stops whatever its
    /// on-failure choice); STALLED for FLIPPER STUCK, or a visit that ended short of ours (NO CHANGE,
    /// STILL NOT OURS, UNKNOWN AFTER) after a push that STALLED; TIMED_OUT for NOT ENOUGH TIME, NO TIME
    /// FOR PUSH 2 and RETRACTED FOR TIME; UNCONFIRMED for every other doubt, OURS AFTER A SHORT STROKE
    /// included.
    [[nodiscard]] constexpr manipulation::MechanismOutcome outcome() const noexcept {
        using O = manipulation::MechanismOutcome;
        switch (verdict) {
            case ToggleVisitVerdict::AlreadyOurs:
            case ToggleVisitVerdict::Flipped: return O::Succeeded;
            case ToggleVisitVerdict::Cut: return O::Cancelled;
            case ToggleVisitVerdict::FlipperStuck: return O::Stalled;
            case ToggleVisitVerdict::NoChange:
            case ToggleVisitVerdict::StillNotOurs:
            case ToggleVisitVerdict::UnknownAfter:
                return pushes > 0 && push[pushes - 1].outcome == O::Stalled ? O::Stalled : O::Unconfirmed;
            case ToggleVisitVerdict::NotEnoughTime:
            case ToggleVisitVerdict::NoTimeForSecondPush:
            case ToggleVisitVerdict::RetractedForTime: return O::TimedOut;
            default: break;
        }
        return O::Unconfirmed;
    }
};

/// The camera handles the explicit form reads — the robot's ONE camera feed and the mount the library
/// believes, the TEAM's tracker numbers, and the pose bound the caller states. The step builds its OWN
/// tracker from `tracker` and `model` for each visit, so the hold that counts always began inside it.
struct ExplicitToggleCamera {
    /// The camera's feed — ticked inside the step's waits (nullptr: no camera).
    vision::CameraFeed* feed = nullptr;
    /// The mount the library believes, built from a placement cameraModelProblem() accepted (nullptr: no
    /// camera).
    const vision::CameraModel* model = nullptr;
    /// The team's tracker numbers (toggleTrackerConfigProblem() must accept them).
    vision::ToggleTrackerConfig tracker{};
    /// The pose bound the caller states, inches: how far the estimate may be off. 0 = NONE stated —
    /// the tracker's position gate is then skipped (its view says unchecked), never run on a pose
    /// taken as exact.
    units::Length poseTolerance{};
    /// The heading bound the caller states, degrees (used only with `poseTolerance`).
    double headingToleranceDeg = 0.0;
};

/// The band at the look spot? nullptr when its WHOLE range interval lies inside [lookFrom - offBy,
/// lookFrom + offBy] (both ends included) and its WHOLE bearing interval inside +-bearingWindowDeg;
/// otherwise what is wrong. Containment, never overlap (file banner, rule 2). Pure.
[[nodiscard]] inline const char* bandAtLookSpotProblem(const vision::ToggleView& v, const ToggleVisitRequest& r,
                                                       double bearingWindowDeg) noexcept {
    const double lo = r.lookFrom.value() - r.offBy.value();
    const double hi = r.lookFrom.value() + r.offBy.value();
    if (!v.range.valid || !std::isfinite(v.range.lo) || !std::isfinite(v.range.hi) || v.range.lo < lo
        || v.range.hi > hi) {
        return "the band's range is not wholly inside the look spot's window";
    }
    const double win = bearingWindowDeg * vision::kDegToRad;
    if (!v.bearing.valid || !(v.bearing.lo >= -win) || !(v.bearing.hi <= win)) {
        return "the band's bearing is not wholly inside the window";
    }
    return nullptr;
}

/// The direction TOWARD `id`'s wall, F1 radians (minus kTogglePlaces' inward normal): right 0, blue +pi/2,
/// left pi, red -pi/2. PRECONDITION-free: an Unset id answers 0.
[[nodiscard]] inline double towardWallRadians(vision::override2026::ToggleId id) noexcept {
    const int i = vision::override2026::toggleIndex(id);
    if (i < 0) {
        return 0.0;
    }
    const vision::override2026::TogglePlace& p = vision::override2026::kTogglePlaces[i];
    return std::atan2(-p.inY, -p.inX) + 0.0;  // + 0.0: the right wall's -0 (atan2(-0, 1)) reads 0, not "-0"
}

namespace detail {

/// One visit (the file banner's rules), over explicit handles. Built and run once by the calls below.
class ToggleVisit {
public:
    ToggleVisit(ExplicitToggleCamera& cam, chassis::Chassis& chassis, sequence::RunGuard& guard,
                mechanism::MechanismService* service, const AutonEntry& entry, const ToggleVisitRequest& request,
                hal::ITelemetrySink& sink)
        : cam_{cam}, chassis_{chassis}, guard_{guard}, svc_{service}, entry_{entry}, req_{request}, sink_{sink} {}

    /// The whole visit (`lookOnly`: rules 1–3 only, SEEN NOT OURS when not ours).
    ToggleVisitReport run(bool lookOnly) {
        SHULIB_PRECONDITION(guard_.running(),
                            "competition::checkAndFlipToggle: no run is live -- call it from an auton's routine "
                            "(runAuton runs it under its guard)");
        const char* problem = toggleVisitRequestProblem(req_);
        SHULIB_PRECONDITION(problem == nullptr, problem != nullptr ? problem : "");
        start_ = now();
        rep_.toggle = req_.toggle;
        rep_.ours = oursFor(req_.toggle);
        rep_.secondsLeftAtStart = guard_.remaining().value();
        if (!allowed()) {
            return finish();
        }
        tracker_.emplace(cam_.tracker, *cam_.model);
        if (cam_.poseTolerance.value() > 0.0) {
            frozenWatch_.emplace(cam_.tracker, *cam_.model);  // the frozen check, never position-gated
        }
        if (!lookBefore()) {
            return finish();
        }
        if (rep_.before == rep_.ours) {
            return finish(ToggleVisitVerdict::AlreadyOurs);
        }
        if (lookOnly) {
            return finish(ToggleVisitVerdict::SeenNotOurs);
        }
        if (!timeAllows(req_.maxPushes, ToggleVisitVerdict::NotEnoughTime)) {
            return finish();
        }
        vision::ToggleColour was = rep_.before;
        for (int push = 1; push <= req_.maxPushes; ++push) {
            if (push == 2 && !timeAllows(1, ToggleVisitVerdict::NoTimeForSecondPush)) {
                return finish();
            }
            if (!stroke(push)) {
                return finish();
            }
            if (!lookAfter()) {
                return finish();
            }
            const vision::ToggleColour seen = rep_.after;
            if (seen == rep_.ours) {
                return finish(rep_.fullStroke[push - 1] ? ToggleVisitVerdict::Flipped
                                                        : ToggleVisitVerdict::OursAfterShortStroke);
            }
            if (push == 1 && seen == was) {
                return finish(ToggleVisitVerdict::NoChange);
            }
            if (push == req_.maxPushes) {
                return finish(ToggleVisitVerdict::StillNotOurs);
            }
            was = seen;  // the THIRD colour after push 1: rule 4's push-2 check, then 5 and 6 once more
        }
        return finish(ToggleVisitVerdict::StillNotOurs);  // unreachable: the loop returns on its last push
    }

private:
    // ── rule 1 ────────────────────────────────────────────────────────────────────────────────────
    bool allowed() {
        if (entry_.kind != AutonKind::Skills) {
            rep_.verdict = ToggleVisitVerdict::NotSkills;
            rep_.detail = autonKindWord(entry_.kind);
            return false;
        }
        if (cam_.feed == nullptr || cam_.model == nullptr) {
            rep_.verdict = ToggleVisitVerdict::NoCamera;
            rep_.detail = "no camera feed on this robot";
            return false;
        }
        if (const char* p = config::cameraPlacementProblem(cam_.model->placement()); p != nullptr) {
            rep_.verdict = ToggleVisitVerdict::NoCamera;
            rep_.detail = p;
            return false;
        }
        if (const char* p = vision::toggleTrackerConfigProblem(cam_.tracker); p != nullptr) {
            rep_.verdict = ToggleVisitVerdict::NoCamera;
            rep_.detail = p;
            return false;
        }
        SHULIB_PRECONDITION(req_.lookFrom.value() - req_.pushFrom.value() > 2.0 * cam_.tracker.frozenMoveIn,
                            "competition::checkAndFlipToggle: lookFrom - pushFrom must be MORE than twice the "
                            "tracker's frozenMoveIn -- the look after a push must move the lens past the frozen "
                            "check's distance (a camera frozen at the push spot would read stale and sure)");
        if (!flipperReady()) {
            rep_.verdict = ToggleVisitVerdict::FlipperNotReady;
            return false;
        }
        const StartPose::Mode mode = entry_.effectiveStart().mode();
        rep_.wallChecked = mode == StartPose::Mode::Field;
        if (rep_.wallChecked) {
            const double heading = estimate().heading().radians();
            const double toward = towardWallRadians(req_.toggle);
            const double off = std::fabs(std::remainder(heading - toward, 2.0 * math::Angle::kPi));
            if (!(off <= math::Angle::kPi / 4.0)) {
                rep_.verdict = ToggleVisitVerdict::WrongWall;
                return false;
            }
        }
        return true;
    }

    bool flipperReady() {
        const FlipperGoal& f = req_.flipper;
        if (f.passive()) {
            return true;
        }
        if (svc_ == nullptr) {
            rep_.detail = "no mechanism service (the program passed none)";
            return false;
        }
        if (f.kind == FlipperGoal::Kind::Position) {
            mechanism::PositionSlot* s = svc_->find(f.position);
            if (s == nullptr || svc_->find(req_.back.position) != s) {
                rep_.detail = "is not on this robot with these presets";
                return false;
            }
            if (!s->built()) {
                rep_.detail = s->absence();
                return false;
            }
            if (!s->homed()) {
                rep_.detail = "is not homed (home it first)";
                return false;
            }
            if (s->running()) {
                rep_.detail = "is still running a step";
                return false;
            }
            if (!(std::fabs(s->position() - req_.back.position.value) <= s->spec().position.tolerance)) {
                rep_.detail = "is not at its back position (put it there first)";
                return false;
            }
            position_ = s;
            return true;
        }
        mechanism::PneumaticSlot* s = svc_->find(f.pneumatic);
        if (s == nullptr || svc_->find(req_.back.pneumatic) != s) {
            rep_.detail = "is not on this robot with these positions";
            return false;
        }
        if (!s->built()) {
            rep_.detail = s->absence();
            return false;
        }
        // Rule 4 counts the request's bounds; the slot waits up to its own (move time + confirm window).
        if (req_.flipBound.value() < s->lastStepBound() || req_.backBound.value() < s->lastStepBound()) {
            rep_.detail = "takes longer than flipBound or backBound (its move time; Step 0.1)";
            return false;
        }
        if (s->running()) {
            rep_.detail = "is still running a step";
            return false;
        }
        if (s->commandedPosition() != req_.back.pneumatic.position) {
            rep_.detail = "is not at its back position (put it there first)";
            return false;
        }
        pneumatic_ = s;
        return true;
    }

    // ── the camera, read on EVERY wait ───────────────────────────────────────────────────────────
    [[nodiscard]] double now() const { return chassis_.deps().ctx->clock().now().value(); }
    [[nodiscard]] math::Pose2d estimate() const { return chassis_.deps().localizer->pose(); }

    /// The lens's field position for `p` (the tracker's own arithmetic).
    void lensOf(const math::Pose2d& p, double& x, double& y) const {
        const double th = p.heading().radians();
        const double fwd = cam_.model->placement().forward.value();
        const double lft = cam_.model->placement().left.value();
        x = p.x().value() + std::cos(th) * fwd - std::sin(th) * lft;
        y = p.y().value() + std::sin(th) * fwd + std::cos(th) * lft;
    }

    /// Tick the feed; a NEW read goes to the tracker with the estimate and the stated bound, widened for
    /// the image's age.
    void readCamera() {
        if (!cam_.feed->tick()) {
            return;
        }
        const math::Pose2d p = estimate();
        vision::PoseBound b{};
        b.pose = p;
        const bool stated = cam_.poseTolerance.value() > 0.0;
        b.positionTolerance = units::Length{stated ? cam_.poseTolerance.value() : HUGE_VAL};
        b.headingToleranceDeg = stated ? cam_.headingToleranceDeg : HUGE_VAL;
        const math::Twist2d tw = chassis_.deps().localizer->twist();
        const double speed = std::hypot(tw.vx().value(), tw.vy().value());
        const double turn = tw.omega().value() / vision::kDegToRad;
        const hal::VisionFrame& f = cam_.feed->latest();
        tracker_->update(f, vision::boundAtImage(b, speed, turn, req_.maxImageAge.value()));
        if (frozenWatch_) {
            // With a pose bound stated the tracker's POSITION gate runs, and a frozen picture is exactly
            // what drifts off a Toggle's place as the lens moves: the gate drops it ("no band") before the
            // tracker can say "camera frozen?". So the frozen check also runs on a twin fed with NO bound
            // (the gate skipped) — the same reads, the same frozenMoveIn.
            vision::PoseBound open{};
            open.pose = p;
            open.positionTolerance = units::Length{HUGE_VAL};
            open.headingToleranceDeg = HUGE_VAL;
            frozenWatch_->update(f, vision::boundAtImage(open, speed, turn, req_.maxImageAge.value()));
        }
        lensOf(p, readLensX_, readLensY_);
        haveRead_ = true;
        // "camera frozen?" counts as EVIDENCE only once it holds on a read whose picture is from after the
        // read that raised it (readAt - maxImageAge, HA-192): a live camera whose pictures lag behind a
        // robot that just started moving shows an unmoved box for up to maxImageAge, and the tracker clears
        // that as soon as its box moves; a frozen camera's never moves.
        if (frozenWatchView().why == vision::ToggleWhy::CameraFrozen) {
            if (!(frozenSince_ >= 0.0)) {
                frozenSince_ = f.readAt.value();
            }
            frozenStale_ = f.readAt.value() - req_.maxImageAge.value() >= frozenSince_;
        } else {
            frozenSince_ = -1.0;
            frozenStale_ = false;
        }
    }

    /// The view the frozen check reads: the ungated twin's when a pose bound is stated, else the tracker's.
    [[nodiscard]] vision::ToggleView frozenWatchView() const {
        return frozenWatch_ ? frozenWatch_->view(now()) : tracker_->view(now());
    }

    // ── rule 2 ────────────────────────────────────────────────────────────────────────────────────
    bool lookBefore() {
        const double until = now() + req_.lookWithin.value();
        vision::ToggleView v{};
        const sequence::GuardedWaitResult w = guard_.waitFor(
            [&] {
                readCamera();
                v = tracker_->view(now());
                return v.colour != vision::ToggleColour::Unknown || now() >= until;
            },
            units::Time{req_.lookWithin.value() + kToggleVisitWaitSlackSeconds});
        if (w == sequence::GuardedWaitResult::RunExpired) {
            rep_.verdict = ToggleVisitVerdict::Cut;
            return false;
        }
        rep_.whyBefore = v.why;
        rep_.range = v.range;
        rep_.bearing = v.bearing;
        rep_.positionChecked = v.positionChecked;
        if (v.colour == vision::ToggleColour::Unknown) {
            rep_.verdict = ToggleVisitVerdict::UnknownBefore;
            return false;
        }
        rep_.before = v.colour;
        if (const char* p = bandAtLookSpotProblem(v, req_, cam_.tracker.bearingWindowDeg); p != nullptr) {
            rep_.verdict = ToggleVisitVerdict::NotAtSpot;
            rep_.detail = p;
            return false;
        }
        if (rep_.wallChecked) {  // a FIELD start: the estimate's lens from the wall's INNER face (HA-201)
            const vision::override2026::TogglePlace& w2 =
                vision::override2026::kTogglePlaces[vision::override2026::toggleIndex(req_.toggle)];
            double lx = 0.0;
            double ly = 0.0;
            lensOf(estimate(), lx, ly);
            rep_.lensFromWallIn = (lx - w2.x) * w2.inX + (ly - w2.y) * w2.inY;
            if (!(std::fabs(rep_.lensFromWallIn - req_.lookFrom.value()) <= req_.offBy.value())) {
                rep_.verdict = ToggleVisitVerdict::NotAtSpot;
                rep_.detail = "the estimate is not at the look spot";
                estimateOffSpot_ = true;
                return false;
            }
        }
        return true;
    }

    // ── rule 4 ────────────────────────────────────────────────────────────────────────────────────
    bool timeAllows(int pushes, ToggleVisitVerdict refusal) {
        rep_.secondsNeeded = toggleVisitNeedSeconds(req_, pushes);
        rep_.secondsLeftAtCheck = guard_.remaining().value();
        if (rep_.secondsLeftAtCheck >= rep_.secondsNeeded) {
            return true;
        }
        rep_.verdict = refusal;
        return false;
    }

    // ── rule 5's retract-now ─────────────────────────────────────────────────────────────────────
    /// While the flipper is OUT — from the push's command until `back` is commanded (a passive bar: from
    /// the end of the drive in until the drive out begins) — the time left at or below backBound + settle
    /// + driveBound + margin: `back` at once (the push replaced: the most recent command wins), no
    /// settle, and the drive out. Rule 4's check makes this unreachable while every stage keeps to its
    /// bound (at `back`'s command at least lookWithin + 2 ticks more is left); it is what keeps the robot
    /// off the Toggle at 0:00 when time was lost OUTSIDE the bounds (a brain's loop that stalled) — the run's
    /// end SAFES a flipper, it does not retract it (A4 register HA-234).
    void retractNowCheck() {
        if (!flipperOut_ || retractForTime_) {
            return;
        }
        const double threshold =
            req_.backBound.value() + req_.settle.value() + req_.driveBound.value() + req_.margin.value();
        if (guard_.remaining().value() > threshold) {
            return;
        }
        retractForTime_ = true;
        if (!req_.flipper.passive() && !backCommanded_) {
            commandBack();  // a push still running is replaced: the most recent command wins
        }
    }

    [[nodiscard]] bool mechanismRunning() const {
        return position_ != nullptr ? position_->running() : pneumatic_ != nullptr && pneumatic_->running();
    }
    [[nodiscard]] mechanism::StepReport mechanismReport() const {
        return position_ != nullptr ? position_->lastStep() : pneumatic_->lastStep();
    }
    void commandPush() {
        flipperOut_ = true;
        backCommanded_ = false;
        cancelledPush_ = false;
        if (position_ != nullptr) {
            position_->moveTo(req_.flipper.position.value, req_.flipBound, req_.flipper.position.computed);
        } else {
            pneumatic_->set(req_.flipper.pneumatic.position);
        }
    }
    void commandBack() {
        pushReport_ = mechanismReport();
        if (pushReport_.outcome == manipulation::MechanismOutcome::Running) {
            cancelledPush_ = true;  // still running (retract-now, or a wait's backstop): `back` replaces it
        }
        backCommanded_ = true;
        flipperOut_ = false;
        if (position_ != nullptr) {
            position_->moveTo(req_.back.position.value, req_.backBound, req_.back.position.computed);
        } else {
            pneumatic_->set(req_.back.pneumatic.position);
        }
    }
    /// A mechanism step's own wait bound: the request's for a motor flipper; the robot file's move time
    /// for a pneumatic (its slot's lastStepBound()).
    [[nodiscard]] double mechanismBound(units::Time requestBound) const {
        return position_ != nullptr ? requestBound.value() : pneumatic_->lastStepBound();
    }

    // ── the drive in and out: async + the guard's wait, never a blocking verb (R3) ───────────────
    /// The motion is cancelled on EVERY exit while it is still the scheduler's (as Chassis's own
    /// DetachGuard): a throw, a cut, a wait that ran out — it never dangles in the slot.
    class MotionGuard {
    public:
        MotionGuard(motion::MotionScheduler& s, std::uint32_t id) noexcept : s_{s}, id_{id} {}
        ~MotionGuard() {
            if (s_.hasActiveMotion() && s_.activeCommandId() == id_) {
                s_.cancel();
            }
        }
        MotionGuard(const MotionGuard&) = delete;
        MotionGuard& operator=(const MotionGuard&) = delete;

    private:
        motion::MotionScheduler& s_;
        std::uint32_t id_;
    };

    /// How a drive watches "camera frozen?" (rule 5): not at all (the drive out), stop on the EVIDENCE
    /// (frozenStale_: an active flipper's drive in — nothing touches the Toggle before the push), or stop
    /// as soon as it is RAISED (a passive bar's drive in, which IS the stroke: it must not reach the
    /// Toggle while the question is open).
    enum class FrozenWatch : std::uint8_t { None, StopIfStale, StopIfRaised };

    /// Is "camera frozen?" raised on the frozen check's view (the evidence may not be stale yet)?
    [[nodiscard]] bool frozenRaised() const { return frozenWatchView().why == vision::ToggleWhy::CameraFrozen; }

    /// Drive to `target` (field frame) within `bound` seconds (driveBound when negative), the camera read
    /// on every tick. False when the run's deadline CUT it (the verdict is set); `exit` is the motion's own
    /// verdict otherwise. `watch` (the drive IN): "camera frozen?" held past a picture taken after it was
    /// raised (readCamera) ends the drive there (cancelled) and sets frozenOnDriveIn_ — the look the push
    /// rests on may be stale; with StopIfRaised, the question merely RAISED ends it too (cancelled) and
    /// sets pausedOnDriveIn_ — driveIn() holds and decides.
    bool drive(const math::Pose2d& target, control::ExitReason& exit, FrozenWatch watch = FrozenWatch::None,
               double bound = -1.0) {
        const double limit = bound >= 0.0 ? bound : req_.driveBound.value();
        motion::MotionScheduler& s = chassis_.scheduler();
        motion::MoveToPose m{chassis_.deps(), target, chassis_.motionConfig(), limit};
        const int before = s.completedCount();
        bool frozen = false;
        bool paused = false;
        s.async(m);
        const MotionGuard cancelOnExit{s, s.activeCommandId()};
        const sequence::GuardedWaitResult w = guard_.waitFor(
            [&] {
                readCamera();
                retractNowCheck();
                if (watch != FrozenWatch::None && frozenStale_) {
                    frozen = true;
                    return true;
                }
                if (watch == FrozenWatch::StopIfRaised && frozenRaised()) {
                    paused = true;
                    return true;
                }
                return s.completedCount() > before;
            },
            units::Time{limit + kToggleVisitWaitSlackSeconds});
        if (w == sequence::GuardedWaitResult::RunExpired) {
            rep_.verdict = ToggleVisitVerdict::Cut;
            return false;
        }
        if (frozen || paused) {
            frozenOnDriveIn_ = frozen;
            pausedOnDriveIn_ = paused;
            exit = control::ExitReason::Cancelled;  // cancelOnExit stops it as this returns
            return true;
        }
        exit = s.completedCount() > before ? s.lastExitReason() : control::ExitReason::TimedOut;
        return true;
    }

    /// Rule 5's HOLD: "camera frozen?" is raised but its evidence is not yet stale — the robot stands where
    /// it is and keeps reading, at most `left` seconds (the drive in's share of ceil(maxImageAge), spent
    /// across its holds): the evidence turning stale, or the question still open when `left` runs out,
    /// sets frozenOnDriveIn_ (nothing is pushed); the question clearing (a live camera's pictures caught
    /// up) ends the hold with the camera believed. False when the run's deadline CUT it.
    bool holdOnFrozen(double& left) {
        const double t0 = now();
        const double end = t0 + left;
        if (!waitWhileReading([&] { return frozenStale_ || !frozenRaised() || now() >= end; }, left)) {
            return false;
        }
        left = std::max(0.0, left - (now() - t0));
        if (frozenStale_ || frozenRaised()) {
            frozenOnDriveIn_ = true;
        }
        return true;
    }

    /// Rule 5's drive in, straight to `in`. An ACTIVE flipper: the drive stops on the frozen EVIDENCE;
    /// when it ARRIVES with "camera frozen?" raised but not yet stale, it HOLDS there before any push (a
    /// maxImageAge longer than the time from the question being raised to the arrival would otherwise let
    /// the push rest on a picture the tracker already doubts). A PASSIVE bar (the drive in IS the stroke):
    /// it stops as soon as the question is raised, HOLDS, and — the camera believed again — drives on
    /// within what is left of driveBound. Every hold is counted in rule 4 (ceil(maxImageAge) a push).
    /// False when the run's deadline CUT it.
    bool driveIn(const math::Pose2d& in, control::ExitReason& exit) {
        const bool passive = req_.flipper.passive();
        double holdLeft = toggleVisitCeilTick(req_.maxImageAge.value());
        double moveLeft = req_.driveBound.value();
        for (;;) {
            pausedOnDriveIn_ = false;
            const double t0 = now();
            if (!drive(in, exit, passive ? FrozenWatch::StopIfRaised : FrozenWatch::StopIfStale, moveLeft)) {
                return false;
            }
            moveLeft -= now() - t0;
            if (frozenOnDriveIn_) {
                return true;
            }
            const bool hold = passive ? pausedOnDriveIn_ : (exit == control::ExitReason::Settled && frozenRaised());
            if (!hold) {
                return true;
            }
            if (!holdOnFrozen(holdLeft)) {
                return false;
            }
            if (frozenOnDriveIn_ || !passive) {
                return true;  // frozen: nothing pushed; or an active flipper's camera believed: push
            }
            if (!(moveLeft > 0.0)) {
                exit = control::ExitReason::TimedOut;  // no drive time left to finish the stroke
                return true;
            }
        }
    }

    /// Wait until `done()` (the camera read on every tick, the retract-now check run) or `bound`. False
    /// when the run's deadline CUT it.
    template <typename Done>
    bool waitWhileReading(Done&& done, double bound) {
        const sequence::GuardedWaitResult w = guard_.waitFor(
            [&] {
                readCamera();
                retractNowCheck();
                return done();
            },
            units::Time{bound + kToggleVisitWaitSlackSeconds});
        if (w == sequence::GuardedWaitResult::RunExpired) {
            rep_.verdict = ToggleVisitVerdict::Cut;
            return false;
        }
        return true;
    }

    // ── rule 5 ────────────────────────────────────────────────────────────────────────────────────
    /// One stroke, from the look spot back to it. False when the visit ends here (the verdict set).
    bool stroke(int push) {
        const int k = push - 1;
        const math::Pose2d look = estimate();
        const double d = req_.lookFrom.value() - req_.pushFrom.value();
        const double th = look.heading().radians();
        const math::Pose2d in{units::Length{look.x().value() + d * std::cos(th)},
                              units::Length{look.y().value() + d * std::sin(th)}, look.heading()};
        rep_.pushes = static_cast<std::uint8_t>(push);
        flipperOut_ = false;
        backCommanded_ = false;
        cancelledPush_ = false;
        pushReport_ = mechanism::StepReport{};
        control::ExitReason inExit = control::ExitReason::Running;
        if (!driveIn(in, inExit)) {
            return false;
        }
        rep_.driveIn = inExit;
        if (frozenOnDriveIn_) {
            // The camera showed itself frozen before anything reached the Toggle: nothing pushed, out.
            rep_.pushes = static_cast<std::uint8_t>(push - 1);
            control::ExitReason outExit = control::ExitReason::Running;
            if (!drive(look, outExit)) {
                return false;
            }
            rep_.driveOut = outExit;
            rep_.verdict = ToggleVisitVerdict::FrozenBeforePush;
            return false;
        }
        const bool arrived = inExit == control::ExitReason::Settled;
        const bool passive = req_.flipper.passive();
        if (passive) {
            // The drive in IS the stroke; "the retract's end" is the moment it ends, and the bar is OUT
            // (at the Toggle) until the drive out begins.
            rep_.fullStroke[k] = arrived;
            markRetractEnd();
            flipperOut_ = true;
        } else {
            if (!arrived) {
                // Nothing pushed from a spot the robot did not reach: out, and DRIVE FAILED.
                rep_.pushes = static_cast<std::uint8_t>(push - 1);
                control::ExitReason outExit = control::ExitReason::Running;
                if (!drive(look, outExit)) {
                    return false;
                }
                rep_.driveOut = outExit;
                rep_.verdict = ToggleVisitVerdict::DriveFailed;
                rep_.detail = "in";
                return false;
            }
            commandPush();
            if (!waitWhileReading([&] { return !mechanismRunning() || backCommanded_; },
                                  mechanismBound(req_.flipBound))) {
                return false;
            }
            if (!backCommanded_) {
                commandBack();  // on ANY verdict: every push paired
            }
            if (cancelledPush_) {
                pushReport_.outcome = manipulation::MechanismOutcome::Cancelled;
            }
            rep_.push[k] = pushReport_;
            if (!waitWhileReading([&] { return !mechanismRunning(); }, mechanismBound(req_.backBound))) {
                return false;
            }
            rep_.back[k] = mechanismReport();
            if (mechanismRunning()) {
                rep_.back[k].outcome = manipulation::MechanismOutcome::TimedOut;
            }
            if (rep_.back[k].outcome != manipulation::MechanismOutcome::Succeeded) {
                rep_.verdict = ToggleVisitVerdict::FlipperStuck;  // the robot does NOT drive out
                return false;
            }
            rep_.fullStroke[k] = rep_.push[k].outcome == manipulation::MechanismOutcome::Succeeded;
            markRetractEnd();
        }
        if (!retractForTime_) {
            const double until = now() + req_.settle.value();
            if (!waitWhileReading([&] { return retractForTime_ || now() >= until; }, req_.settle.value())) {
                return false;
            }
        }
        flipperOut_ = false;
        control::ExitReason outExit = control::ExitReason::Running;
        if (!drive(look, outExit)) {
            return false;
        }
        rep_.driveOut = outExit;
        if (retractForTime_) {
            rep_.verdict = ToggleVisitVerdict::RetractedForTime;
            return false;
        }
        if (outExit != control::ExitReason::Settled) {
            rep_.verdict = ToggleVisitVerdict::DriveFailed;
            rep_.detail = "out";
            return false;
        }
        return true;
    }

    void markRetractEnd() {
        retractEnd_ = now();
        lensOf(estimate(), retractLensX_, retractLensY_);
    }

    // ── rule 6 ────────────────────────────────────────────────────────────────────────────────────
    /// Does this view count as the look after (rule 6)?
    [[nodiscard]] bool countsAfter(const vision::ToggleView& v) const {
        if (v.colour == vision::ToggleColour::Unknown || !haveRead_) {
            return false;
        }
        const hal::VisionFrame& f = cam_.feed->latest();
        if (!f.usable() || !(f.readAt.value() - req_.maxImageAge.value() >= retractEnd_)) {
            return false;  // R8: a picture from before the retract's end
        }
        return std::hypot(readLensX_ - retractLensX_, readLensY_ - retractLensY_) > cam_.tracker.frozenMoveIn;
    }

    bool lookAfter() {
        const double until = now() + req_.lookWithin.value();
        vision::ToggleView v{};
        bool counted = false;
        if (!waitWhileReading(
                [&] {
                    v = tracker_->view(now());
                    if (frozenWatch_) {
                        const vision::ToggleView w = frozenWatch_->view(now());
                        if (w.why == vision::ToggleWhy::CameraFrozen) {
                            v = w;  // frozen, though the gated tracker says "no band" or holds a stale view
                        }
                    }
                    counted = countsAfter(v);
                    return counted || now() >= until;
                },
                req_.lookWithin.value())) {
            return false;
        }
        rep_.whyAfter = v.why;
        rep_.afterTooEarly = !counted && v.colour != vision::ToggleColour::Unknown;
        rep_.after = counted ? v.colour : vision::ToggleColour::Unknown;
        if (!counted) {
            rep_.verdict = ToggleVisitVerdict::UnknownAfter;
            return false;
        }
        return true;
    }

    // ── rule 7 ────────────────────────────────────────────────────────────────────────────────────
    ToggleVisitReport finish(ToggleVisitVerdict v) {
        rep_.verdict = v;
        return finish();
    }
    ToggleVisitReport finish() {
        rep_.seconds = now() - start_;
        char line[161];
        writeLine(line, sizeof line);
        sink_.log(rep_.ok() ? hal::LogLevel::Info : hal::LogLevel::Warn, "TGL", line);
        return rep_;
    }

    void writeLine(char* out, std::size_t n) const {
        using vision::toggleColourWord;
        const char* name = vision::override2026::toggleName(rep_.toggle);
        const char* wall = rep_.wallChecked ? "" : ", wall not checked (relative start)";
        const char* plural = rep_.pushes == 1 ? "push" : "pushes";
        const char* flipper = req_.flipper.mechanismName();
        switch (rep_.verdict) {
            case ToggleVisitVerdict::AlreadyOurs:
                std::snprintf(out, n, "%s: already %s (ours), nothing pushed, %.1fs%s, seating not checked", name,
                              toggleColourWord(rep_.before), rep_.seconds, wall);
                return;
            case ToggleVisitVerdict::Flipped:
                std::snprintf(out, n, "%s: %s -> %s after %d %s, %.1fs (ours %s)%s, seating not checked", name,
                              toggleColourWord(rep_.before), toggleColourWord(rep_.after), rep_.pushes, plural,
                              rep_.seconds, toggleColourWord(rep_.ours), wall);
                return;
            case ToggleVisitVerdict::NotSkills:
                std::snprintf(out, n, "%s: not a SKILLS auton (%s) -- nothing pushed", name, rep_.detail);
                return;
            case ToggleVisitVerdict::NoCamera:
                std::snprintf(out, n, "%s: no camera -- %.100s -- nothing pushed", name, rep_.detail);
                return;
            case ToggleVisitVerdict::FlipperNotReady:
                std::snprintf(out, n, "%s: flipper not ready -- '%.23s' %.70s -- nothing pushed", name, flipper,
                              rep_.detail);
                return;
            case ToggleVisitVerdict::WrongWall:
                std::snprintf(out, n, "%s: wrong wall -- heading %.0f deg, its wall is at %.0f deg -- nothing pushed",
                              name, estimate().heading().degrees(), towardWallRadians(rep_.toggle) / vision::kDegToRad);
                return;
            case ToggleVisitVerdict::UnknownBefore:
                std::snprintf(out, n, "%s: UNKNOWN before (%s) -- nothing pushed%s", name,
                              vision::toggleWhyWords(rep_.whyBefore), wall);
                return;
            case ToggleVisitVerdict::NotAtSpot:
                if (estimateOffSpot_) {
                    std::snprintf(out, n, "%s: not at the spot -- estimate %.1f in from the wall, look spot %.1f +- %.1f "
                                  "in -- nothing pushed", name, rep_.lensFromWallIn, req_.lookFrom.value(),
                                  req_.offBy.value());
                } else {
                    std::snprintf(out, n, "%s: not at the spot -- band at %.1f-%.1f in, look spot %.1f +- %.1f in -- "
                                  "nothing pushed", name, rep_.range.lo, rep_.range.hi, req_.lookFrom.value(),
                                  req_.offBy.value());
                }
                return;
            case ToggleVisitVerdict::NotEnoughTime:
                std::snprintf(out, n, "%s: not now -- %.1fs left, needs %.1fs -- nothing pushed", name,
                              rep_.secondsLeftAtCheck, rep_.secondsNeeded);
                return;
            case ToggleVisitVerdict::NoChange:
                std::snprintf(out, n, "%s: NO CHANGE -- still %s after 1 push (sticky, or a camera frozen since "
                              "before)%s", name, toggleColourWord(rep_.after), wall);
                return;
            case ToggleVisitVerdict::StillNotOurs:
                std::snprintf(out, n, "%s: STILL NOT OURS -- %s after %d %s (ours %s)%s", name,
                              toggleColourWord(rep_.after), rep_.pushes, plural, toggleColourWord(rep_.ours), wall);
                return;
            case ToggleVisitVerdict::UnknownAfter:
                std::snprintf(out, n, "%s: UNKNOWN after %d %s (%s) -- %s before", name, rep_.pushes, plural,
                              rep_.afterTooEarly ? "no picture after the retract" : vision::toggleWhyWords(rep_.whyAfter),
                              toggleColourWord(rep_.before));
                return;
            case ToggleVisitVerdict::NoTimeForSecondPush:
                std::snprintf(out, n, "%s: no time for push 2 -- %s after 1 push, %.1fs left, needs %.1fs", name,
                              toggleColourWord(rep_.after), rep_.secondsLeftAtCheck, rep_.secondsNeeded);
                return;
            case ToggleVisitVerdict::FlipperStuck:
                std::snprintf(out, n, "%s: FLIPPER STUCK -- '%.23s' did not reach its back position (%s); the robot "
                              "stays put", name, flipper,
                              manipulation::mechanismOutcomeName(rep_.back[rep_.pushes > 0 ? rep_.pushes - 1 : 0].outcome));
                return;
            case ToggleVisitVerdict::Cut:
                std::snprintf(out, n, "%s: CUT by the run's end after %d %s, %.1fs", name, rep_.pushes, plural,
                              rep_.seconds);
                return;
            case ToggleVisitVerdict::OursAfterShortStroke:
                std::snprintf(out, n, "%s: %s after a SHORT stroke (push %s) -- maybe stuck part-way, not counted",
                              name, toggleColourWord(rep_.after),
                              req_.flipper.passive()
                                  ? "drive short"
                                  : manipulation::mechanismOutcomeName(rep_.push[rep_.pushes > 0 ? rep_.pushes - 1 : 0].outcome));
                return;
            case ToggleVisitVerdict::RetractedForTime:
                std::snprintf(out, n, "%s: retracted for time after %d %s -- %.1fs left, the robot is off it", name,
                              rep_.pushes, plural, guard_.remaining().value());
                return;
            case ToggleVisitVerdict::DriveFailed:
                std::snprintf(out, n, "%s: the drive %s did not arrive (%s) after %d %s", name, rep_.detail,
                              rep_.detail[0] == 'i' ? exitWord(rep_.driveIn) : exitWord(rep_.driveOut), rep_.pushes,
                              plural);
                return;
            case ToggleVisitVerdict::SeenNotOurs:
                std::snprintf(out, n, "%s: %s seen (ours %s), look only -- nothing pushed%s", name,
                              toggleColourWord(rep_.before), toggleColourWord(rep_.ours), wall);
                return;
            case ToggleVisitVerdict::FrozenBeforePush:
                std::snprintf(out, n, "%s: camera frozen? seen on the drive in -- push %d not made, %s before, the "
                              "robot drove back out", name, rep_.pushes + 1, toggleColourWord(rep_.before));
                return;
            case ToggleVisitVerdict::Unset: break;
        }
        std::snprintf(out, n, "%s: %s", name, toggleVisitWord(rep_.verdict));
    }

    [[nodiscard]] static const char* exitWord(control::ExitReason e) noexcept {
        switch (e) {
            case control::ExitReason::Settled: return "settled";
            case control::ExitReason::TimedOut: return "timed out";
            case control::ExitReason::Cancelled: return "cancelled";
            default: break;
        }
        return "running";
    }

    ExplicitToggleCamera& cam_;
    chassis::Chassis& chassis_;
    sequence::RunGuard& guard_;
    mechanism::MechanismService* svc_;
    const AutonEntry& entry_;
    const ToggleVisitRequest& req_;
    hal::ITelemetrySink& sink_;
    ToggleVisitReport rep_{};
    std::optional<vision::ToggleTracker> tracker_{};
    std::optional<vision::ToggleTracker> frozenWatch_{};  ///< ungated twin for "camera frozen?" (a bound stated)
    mechanism::PositionSlot* position_ = nullptr;
    mechanism::PneumaticSlot* pneumatic_ = nullptr;
    mechanism::StepReport pushReport_{};
    double start_ = 0.0;
    double retractEnd_ = 0.0;
    double retractLensX_ = 0.0;
    double retractLensY_ = 0.0;
    double readLensX_ = 0.0;
    double readLensY_ = 0.0;
    bool haveRead_ = false;
    bool flipperOut_ = false;
    bool backCommanded_ = false;
    bool cancelledPush_ = false;
    bool retractForTime_ = false;
    bool estimateOffSpot_ = false;
    bool frozenOnDriveIn_ = false;
    bool pausedOnDriveIn_ = false;  ///< a passive drive in stopped on "camera frozen?" RAISED (driveIn holds)
    double frozenSince_ = -1.0;  ///< the read that raised "camera frozen?" (readAt), or -1
    bool frozenStale_ = false;   ///< "camera frozen?" held on a picture from after frozenSince_
};

}  // namespace detail

/// THE STEP, explicit form (file banner): look at `request.toggle` from the look spot the route parked
/// the robot on; nothing more if it already shows its Quadrant's colour; otherwise drive in, push,
/// retract, drive out, look again — at most `maxPushes` pushes, the second only on the third colour.
/// Blocking, through `guard`'s deadline-aware waits only; at most the look before plus
/// worstCaseSeconds(request). Returns the report, a VALUE; `outcome()` is what a then() returns. ONE
/// [TGL] line to `sink`. `service` may be nullptr for a PASSIVE flipper. PRECONDITIONS (a programming
/// error, thrown): `guard` is running; toggleVisitRequestProblem(request) is nullptr; and, with a usable
/// camera, lookFrom - pushFrom is more than twice the tracker's frozenMoveIn.
[[nodiscard]] inline ToggleVisitReport checkAndFlipToggle(ExplicitToggleCamera& camera, chassis::Chassis& chassis,
                                                          sequence::RunGuard& guard,
                                                          mechanism::MechanismService* service,
                                                          const AutonEntry& entry, const ToggleVisitRequest& request,
                                                          hal::ITelemetrySink& sink) {
    detail::ToggleVisit visit{camera, chassis, guard, service, entry, request, sink};
    return visit.run(/*lookOnly=*/false);
}

/// Rules 1–3 ONLY — it never moves anything: is it allowed, what colour is the Toggle, is the robot at
/// the look spot. ALREADY OURS, SEEN NOT OURS (a sure colour at the spot, not ours), or the refusal
/// that stopped it — the building block for a team that writes the flip itself (the brief's D1 (c);
/// the checked step is the recommended path). ONE [TGL] line. The preconditions of checkAndFlipToggle.
[[nodiscard]] inline ToggleVisitReport lookAtToggle(ExplicitToggleCamera& camera, chassis::Chassis& chassis,
                                                    sequence::RunGuard& guard, mechanism::MechanismService* service,
                                                    const AutonEntry& entry, const ToggleVisitRequest& request,
                                                    hal::ITelemetrySink& sink) {
    detail::ToggleVisit visit{camera, chassis, guard, service, entry, request, sink};
    return visit.run(/*lookOnly=*/true);
}

}  // namespace shulib::competition
