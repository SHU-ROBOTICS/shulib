#pragma once
//
// The shulib API version — the mechanism behind the Freeze Register's promise.
//
// The register (docs/roadmap.md, "Freeze Register") says a LOCKED contract
// changes "only with a schemaVersion/API-version bump and a migration path".
// Until D2 (2026-08-12) no such version existed anywhere in the tree; this
// header is that mechanism, made inspectable in code instead of prose.
//
// ═══ The policy: what a version bump concretely IS ═══════════════════════════════
// shulib is a header-only C++ library, so its API contract is SOURCE
// compatibility — what user programs compile to and mean:
//
//   * BREAKING (bump kApiMajor): any change that can make a previously-
//     compiling user program fail to compile or silently change meaning —
//     removing or renaming a frozen member or type; changing a frozen
//     signature's parameter, return, const, ref or noexcept shape; changing
//     documented semantics a frozen surface carries (pre-empt, cancel's safe
//     state, wait-for-live, the fault policy...); re-meaning or renumbering
//     an existing enumerator. A major bump REQUIRES a migration note next to
//     the Freeze Register row: the old spelling, the new spelling, the
//     mechanical rewrite, and why the break was worth it.
//
//   * ADDITIVE (bump kApiMinor): every previously-valid program compiles
//     unchanged with unchanged behaviour — new members, new overloads, new
//     options-struct fields whose default preserves the old behaviour,
//     appended enumerators. This is the intended growth path of every frozen
//     surface; freezing well means never needing a major bump.
//
// Frozen C++ surfaces are additionally enforced STRUCTURALLY: F6 carries a
// compile-time signature pin (test/f6_signature_pin_test.cpp) that fails the
// build, naming F6, if a frozen signature changes shape. If that pin sent
// you here and the change is intended: you are making a breaking change —
// update the pin, bump kApiMajor, write the migration note, and update the
// register row. If the change can be re-expressed additively instead, do
// that; it is almost always cheaper for everyone downstream.
//
// The data-schema freezes (F7/F8: .vexbot sub-schemas; F9: the SHUL/2 wire
// protocol) freeze DATA, not C++ source. They carry their own schemaVersion
// fields inside the artifacts (designed at Phase G/H, not here) and are
// governed by this same breaking-vs-additive policy.

namespace shulib {

/// Bumped ONLY for breaking changes to a frozen public surface, always with
/// a migration note (policy above). 2 = the shulib v2 rebuild; the surface
/// frozen at D2 (2026-08-12, register row F6) is API 2.0.
inline constexpr int kApiMajor = 2;

/// Bumped for additive extensions of a frozen surface (new verbs, new
/// options fields, appended enumerators). Reset to 0 on a major bump.
/// 1 = chunk F1's additive growth (2026-08-13): RoutineStopCause gains the
/// appended MechanismFailed enumerator, then() accepts a fourth return type
/// (manipulation::MechanismOutcome), and FaultCode appends MechanismStalled —
/// every one the documented additive path, no frozen member changed shape.
/// (F1 is also the change that PROVED this path works: the D2/D3 pin tests
/// had hard-asserted `kApiMinor == 0`, fencing off the growth this header
/// calls "the intended path" — a conflation fixed in those pins at F1.)
/// 2 = chunk G2's additive growth (2026-09-18), the change that LOCKED
/// register row F8 (the `paths[]` sub-schema + the command-id vocabulary):
/// `config` gains `kMaxMarkerIdBytes` (naming the 24 G1 spelled as a
/// literal), `commandIdValid`/`commandIdView`/`markerIdView`, the
/// `BuiltinCommand` table and `findBuiltinCommand`; the new `runner`
/// namespace adds `CommandRegistry`, `PathRunner` and `writeManifest`; and
/// `Route::valid()` gains four rules and RELAXES one (G2 ruling R6:
/// `MoveToPose` now accepts `HeadingMode::Free`, meaning "keep the heading
/// the estimate has"). No frozen member changed shape, no enumerator moved,
/// and the relaxation is additive in the only direction that matters —
/// every route that validated under 2.1 still validates under 2.2.
/// 2.2 also carries what landed between 2.1 and G2 without a bump of its own
/// (R3b's appended FaultCode::MotorGroupDisagree, the new calibration/,
/// config/, robot/ and teleop/ headers) and ONE tightened precondition —
/// MotionDeps now requires the motor count to EQUAL the wheel count — whose
/// major-bump question is open. docs/changelog.md, API 2.2, lists all of it.
/// 3 = chunk TANK1 (2026-09-27): `moveTo` on a tank steers back onto its
/// line. It rewrites one sentence a FROZEN header carries — F6's
/// strafe-fallback semantics said "On tank, laterally-offset targets honestly
/// exit TimedOut", and after TANK1 some of them arrive — so read strictly it is
/// neither additive ("unchanged behaviour") nor a break (nothing stops
/// compiling, no F6 signature moves). Ruled MINOR (the team lead's go,
/// 2026-09-27): the changed sentence stated a LIMIT and the change relaxes it,
/// the direction G2's R6 took above; and it is recorded as a BEHAVIOUR CHANGE
/// with its three visible effects and a one-line way back
/// (`ChassisConfig.motion.tankSteer.maxSteer = 0` restores the pre-TANK1 tank's
/// motion, records and truth bit for bit). X-drive and H-drive are unchanged
/// byte for byte (except an H-drive configured with NO strafe authority —
/// strafeTractionDerate 0 — which cannot strafe either and is steered like a
/// tank). Added: `motion::TankSteerConfig` + `MotionConfig::tankSteer`,
/// `motion/tank_steer.hpp`, `MoveToPose::tankSteerStatus()`,
/// `DebugRecord::tankSteerActive` (blackbox tick flags bit 2, TermSink
/// " TSTR"); and with its Part B (2026-09-28, the third visible effect — no
/// further bump) the result line's ` xt` column on a tank's translating
/// motions: `CompletedMotion` and `diag::MotionResult` gained `hasCrossTrack`
/// + `crossTrack`, `MotionStatsSink` gained `hadDirection()` + `crossTrack()`.
/// docs/changelog.md, API 2.3, lists all of it.
/// 4 = chunk L9 Part A1 (2026-09-29): two new `Routine` steps, `reverseTo(x, y, options)` and
/// `faceAway(x, y, options)` — `driveTo` and `face` turned around (the bearing to the point plus
/// π, the same `+ π` a route's `reverse` DriveTo adds) — APPENDED to the FROZEN register row F10
/// by the path its banner names ("new steps"), and pinned there. No existing member changed
/// shape, no enumerator moved, no behaviour changed — every 2.3 program compiles and runs
/// exactly as before.
/// docs/changelog.md, API 2.4, lists it.
inline constexpr int kApiMinor = 4;

/// "major.minor", for session headers / logs that want one printable token.
inline constexpr const char* kApiVersionString = "2.4";

}  // namespace shulib
