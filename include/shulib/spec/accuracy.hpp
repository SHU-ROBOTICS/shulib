#pragma once
//
// accuracy.hpp — the accuracy targets of Freeze Register ROW F2, the LOCKED
// spec the autonomous is measured against (master plan §7, ratified
// 2026-06-08). "F2" here is the REGISTER ROW, not chunk F2 (the sequence
// engine, 2026-08-13) — the name collision is real and this line is the
// disambiguation.
//
// This is the SINGLE SOURCE OF TRUTH for the numbers. The system-level
// acceptance tests at M2/M3 assert against these constants, and a guard test
// pins them so they cannot drift without a deliberate Freeze change.
//
// HEADING < 1.0 deg is a HARD requirement; position targets flex around it.

#include "shulib/units/quantity.hpp"

namespace shulib::spec {

/// The one HARD target: |heading error| must ALWAYS be strictly below this.
/// A bare magnitude in DEGREES — not radians, not a units:: quantity — so
/// comparing against it means converting the estimate's radians first, as the
/// acceptance test does. The position targets flex around this one; it never
/// flexes around them.
inline constexpr double kHeadingErrorMaxDeg = 1.0;
/// What closed-loop docking on a tag is expected to reach (degrees), stated
/// beside the cap so the two are read together. An ASPIRATION, not a gate:
/// nothing asserts a run against it; the only test that mentions it checks that
/// it is tighter than kHeadingErrorMaxDeg.
inline constexpr double kDockedHeadingTypicalDeg = 0.5;

// Position errors, in canonical inches.
/// Absolute position error, in canonical inches, at the END of a 60 s autonomous
/// on the v1 estimator (GPS + tracking wheels + IMU) — the loosest of the three,
/// because it is the one that accumulates for a whole run.
inline constexpr units::Length kPositionErrorEndOfRun{1.0};
/// Run-to-run SPREAD (canonical inches), not error against truth: how far apart
/// the same routine's end poses may land across repeats. Deliberately tighter
/// than the absolute target — a repeatable robot can be trimmed, a scattered one
/// cannot.
inline constexpr units::Length kRepeatability{0.75};
/// Final alignment error (canonical inches) for closed-loop vision docking on a
/// tag (M3). The tightest number here because the loop closes on the thing being
/// docked to, so it never pays the dead-reckoning accumulation the other two do.
inline constexpr units::Length kDockedPositionError{0.25};

}  // namespace shulib::spec
