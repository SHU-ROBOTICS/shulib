#pragma once
//
// leg_progress.hpp — how far a drive or a turn has got, as a PARTWAY TRIGGER reads it (chunk STEP1-B,
// 2026-11-11; the STEP1 brief's D7 (a)). PURE: geometry only — no chassis, no scheduler, no sequence
// include — so every rule here is host-tested on its own (test/leg_progress_test.cpp).
//
// ── What "60 % of the drive" measures (D7 (a), the team lead's answer of 2026-11-03) ────────────
// A DRIVE's progress is the distance covered ALONG THE STRAIGHT LINE from where the leg started to
// its target: the projection of (pose − start) on the unit start→target vector. Not the distance
// still to go (a robot that starts off its line would gain "progress" by moving sideways), not the
// odometry's path length (a wobbly leg would reach "60 %" early). It can pass 100 % (an overshoot)
// and read below 0 (a robot pushed backwards); a trigger compares it with `>=`, once.
// A TURN's progress is the angle turned in the direction of the start's shortest signed error, over
// that error's size — the same short way TurnTo turns (F3's errorTo; an exact ±180° resolves CCW,
// as TurnTo's does). The turned angle is `startHeading.errorTo(heading)` with the turn's sign, so
// it is honest while the robot has turned less than 180° from its start in either direction; a turn
// near 180° that OVERSHOOTS past 180° from its start wraps to a negative angle (a mark at or below
// 100 % has been crossed by then — the wrap only hides an overshoot, never a mark short of it).
//
// ── A zero-size leg has no direction (the brief's B5) ───────────────────────────────────────────
// A drive whose target is (numerically) where it started — under kMinLegLength — or a turn to the
// heading it already has — under kMinTurnRad — has no "percent" and no direction to measure along:
// `hasDirection` is false and every progress field reads 0, so nothing fires DURING such a leg.
// (What fires when it settles is the Routine's rule: chassis/routine.hpp, B3.)

#include <cmath>

#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// A leg's progress, as a partway trigger reads it — handed to a chassis::LegObserver after every
/// tick of the leg that did not end it (chassis/chassis.hpp's observer overloads). OPEN to new fields
/// (an aggregate's trailing field is additive): no pin covers its field list (the STEP1 brief's D1).
struct LegProgress {
    /// True once the leg's motion has had its first LIVE tick (a live estimate; the start pose is
    /// captured there). False through the boot wait: nothing else here is meaningful then.
    bool live = false;
    /// The share of the leg done: `along / length` on a drive, `turnedRad / sizeRad` on a turn
    /// (1.0 = at the target; above 1 = past it). 0 when the leg has no direction.
    double fraction = 0.0;
    /// A DRIVE's distance covered along the straight line from its live start to its target (D7 (a));
    /// 0 on a turn, and on a drive with no direction.
    units::Length along{0.0};
    /// A TURN's angle turned toward its target since its live start, radians (the file header); 0 on
    /// a drive, and on a turn with no direction.
    double turnedRad = 0.0;
    // ── the leg's own geometry (APPENDED with the four above at STEP1-B — a Routine needs it AFTER
    //    the leg, to tell a mark ON the leg from one past it: the brief's B3) ─────────────────────
    /// False for a zero-size leg (the file header, B5): nothing measures its progress.
    bool hasDirection = false;
    /// A DRIVE's straight-line length from its live start to its target; 0 on a turn.
    units::Length length{0.0};
    /// A TURN's size: |the start's shortest signed error to the target|, radians; 0 on a drive.
    double sizeRad = 0.0;
    /// The leg's EFFECTIVE position arrival tolerance — the settle distance it runs with (the robot's
    /// own, or the one a MotionOptions::positionTolerance loosened: motion::loosenedSettle).
    units::Length positionTolerance{0.0};
    /// The leg's EFFECTIVE heading arrival tolerance, radians (as positionTolerance, on the heading).
    double headingToleranceRad = 0.0;
};

/// Below this a drive has no direction (inches): the projection would divide by a length that is
/// numerical noise. The same order as Routine's bearingTo() refusal (1e-9 in), kept looser because a
/// fraction over a length of 1e-7 in is meaningless even where a bearing is defined.
inline constexpr double kMinLegLength = 1e-6;
/// Below this a turn has no direction (radians) — kMinLegLength's reason, on the heading.
inline constexpr double kMinTurnRad = 1e-9;

/// A DRIVE leg's progress at `pose`, the leg having started (live) at `start` and aiming at
/// `target`'s POSITION (its heading is not read): the projection of (pose − start) on the unit
/// start→target vector (the file header). `live` is set; the tolerances are left 0 for the caller.
[[nodiscard]] inline LegProgress legProgressAlong(const math::Pose2d& start, const math::Pose2d& target,
                                                  const math::Pose2d& pose) noexcept {
    LegProgress p;
    p.live = true;
    const double dx = target.x().value() - start.x().value();
    const double dy = target.y().value() - start.y().value();
    const double length = std::hypot(dx, dy);
    if (!(length >= kMinLegLength)) {
        return p;  // no direction: every progress field 0, length 0 (B5)
    }
    p.hasDirection = true;
    p.length = units::Length{length};
    const double along =
        ((pose.x().value() - start.x().value()) * dx + (pose.y().value() - start.y().value()) * dy) / length;
    p.along = units::Length{along};
    p.fraction = along / length;
    return p;
}

/// A TURN leg's progress at `heading`, the turn having started (live) at `startHeading` and aiming at
/// `target`: the angle turned in the direction of the start's shortest signed error, over its size
/// (the file header). `live` is set; the tolerances are left 0 for the caller.
[[nodiscard]] inline LegProgress turnProgress(math::Angle startHeading, math::Angle target,
                                              math::Angle heading) noexcept {
    LegProgress p;
    p.live = true;
    const double e0 = startHeading.errorTo(target);  // shortest signed; exact ±180° → +π (CCW)
    const double size = std::abs(e0);
    if (!(size >= kMinTurnRad)) {
        return p;  // no direction (B5)
    }
    p.hasDirection = true;
    p.sizeRad = size;
    const double turned = (e0 > 0.0 ? 1.0 : -1.0) * startHeading.errorTo(heading);
    p.turnedRad = turned;
    p.fraction = turned / size;
    return p;
}

}  // namespace shulib::motion
