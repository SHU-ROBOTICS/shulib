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

// Heading error magnitude (degrees). The firm team spec: ALWAYS < this.
inline constexpr double kHeadingErrorMaxDeg = 1.0;
// Closed-loop docking on a tag typically beats the hard cap; an aspiration, not a gate.
inline constexpr double kDockedHeadingTypicalDeg = 0.5;

// Position errors, in canonical inches.
inline constexpr units::Length kPositionErrorEndOfRun{1.0};  // end-of-60s, v1 (GPS+wheels+IMU)
inline constexpr units::Length kRepeatability{0.75};         // run-to-run spread
inline constexpr units::Length kDockedPositionError{0.25};   // closed-loop vision docking (M3)

}  // namespace shulib::spec
