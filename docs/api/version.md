<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/version.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `version.hpp`

The shulib API version — the mechanism behind the Freeze Register's promise.

This header declares **3** constants.

Extracted from [`include/shulib/version.hpp`](../../include/shulib/version.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kApiMajor`](#kapimajor) — *constant*
- [`kApiMinor`](#kapiminor) — *constant*
- [`kApiVersionString`](#kapiversionstring) — *constant*

<a id="kapimajor"></a>

## `kApiMajor`

```cpp
inline constexpr int kApiMajor = 2
```

Bumped ONLY for breaking changes to a frozen public surface, always with a migration note (policy above). 2 = the shulib v2 rebuild; the surface frozen at D2 (2026-08-12, register row F6) is API 2.0.

*constant, declared at [`include/shulib/version.hpp:48`](../../include/shulib/version.hpp#L48).*

<a id="kapiminor"></a>

## `kApiMinor`

```cpp
inline constexpr int kApiMinor = 5
```

Bumped for additive extensions of a frozen surface (new verbs, new options fields, appended enumerators). Reset to 0 on a major bump. 1 = chunk F1's additive growth (2026-08-13): RoutineStopCause gains the appended MechanismFailed enumerator, then() accepts a fourth return type (manipulation::MechanismOutcome), and FaultCode appends MechanismStalled — every one the documented additive path, no frozen member changed shape. (F1 is also the change that PROVED this path works: the D2/D3 pin tests had hard-asserted `kApiMinor == 0`, fencing off the growth this header calls "the intended path" — a conflation fixed in those pins at F1.) 2 = chunk G2's additive growth (2026-09-18), the change that LOCKED register row F8 (the `paths[]` sub-schema + the command-id vocabulary): `config` gains `kMaxMarkerIdBytes` (naming the 24 G1 spelled as a literal), `commandIdValid`/`commandIdView`/`markerIdView`, the `BuiltinCommand` table and `findBuiltinCommand`; the new `runner` namespace adds `CommandRegistry`, `PathRunner` and `writeManifest`; and `Route::valid()` gains four rules and RELAXES one (G2 ruling R6: `MoveToPose` now accepts `HeadingMode::Free`, meaning "keep the heading the estimate has"). No frozen member changed shape, no enumerator moved, and the relaxation is additive in the only direction that matters — every route that validated under 2.1 still validates under 2.2. 2.2 also carries what landed between 2.1 and G2 without a bump of its own (R3b's appended FaultCode::MotorGroupDisagree, the new calibration/, config/, robot/ and teleop/ headers) and ONE tightened precondition — MotionDeps now requires the motor count to EQUAL the wheel count — whose major-bump question is open. docs/changelog.md, API 2.2, lists all of it. 3 = chunk TANK1 (2026-09-27): `moveTo` on a tank steers back onto its line. It rewrites one sentence a FROZEN header carries — F6's strafe-fallback semantics said "On tank, laterally-offset targets honestly exit TimedOut", and after TANK1 some of them arrive — so read strictly it is neither additive ("unchanged behaviour") nor a break (nothing stops compiling, no F6 signature moves). Ruled MINOR (the team lead's go, 2026-09-27): the changed sentence stated a LIMIT and the change relaxes it, the direction G2's R6 took above; and it is recorded as a BEHAVIOUR CHANGE with its three visible effects and a one-line way back (`ChassisConfig.motion.tankSteer.maxSteer = 0` restores the pre-TANK1 tank's motion, records and truth bit for bit). X-drive and H-drive are unchanged byte for byte (except an H-drive configured with NO strafe authority — strafeTractionDerate 0 — which cannot strafe either and is steered like a tank). Added: `motion::TankSteerConfig` + `MotionConfig::tankSteer`, `motion/tank_steer.hpp`, `MoveToPose::tankSteerStatus()`, `DebugRecord::tankSteerActive` (blackbox tick flags bit 2, TermSink " TSTR"); and with its Part B (2026-09-28, the third visible effect — no further bump) the result line's ` xt` column on a tank's translating motions: `CompletedMotion` and `diag::MotionResult` gained `hasCrossTrack` + `crossTrack`, `MotionStatsSink` gained `hadDirection()` + `crossTrack()`. docs/changelog.md, API 2.3, lists all of it. 4 = chunk L9 Part A1 (2026-09-29): two new `Routine` steps, `reverseTo(x, y, options)` and `faceAway(x, y, options)` — `driveTo` and `face` turned around (the bearing to the point plus π, the same `+ π` a route's `reverse` DriveTo adds) — APPENDED to the FROZEN register row F10 by the path its banner names ("new steps"), and pinned there. No existing member changed shape, no enumerator moved, no behaviour changed — every 2.3 program compiles and runs exactly as before. docs/changelog.md, API 2.4, lists it. 5 = chunk MECH-A4 (2026-10-22), the mechanism core's autons, driver controls and route binder, every one the additive path: `Routine` gains a SECOND constructor `Routine r{ctx, "name"}` (a new overload — the frozen `Routine(Chassis&, const char*)` is untouched), the mechanism steps (move / startMove / home / startHome / run / startRun / set / spinUp / startSpinUp / stop / waitUntilDone) and lastMechanism() — added by register row F10's additive path and NOT pinned: whether they join F10's freeze is an OPEN decision (the register's F10 row; they take the unfrozen mechanism seam's types, as then() does) —, the APPENDED enumerator `RoutineStopCause::MechanismStepFailed` (== 5, pinned) and the appended field `RoutineResult::mechanismVerdict`; `competition::AutonContext` appends `mechanismService` (with a default, so COMP-a's positional initializer still compiles) and `runAuton` gains a mechanism- service overload; `teleop::runDriverControl` gains one too; F8 (LOCKED) grows by its own "new registry members" path — `runner::CommandEntry::doc` and a four-argument `CommandRegistry::on(id, handler, argCount, doc)`, whose doc the manifest publishes (a registry with no docs writes the same bytes as 2.4); and the new headers mechanism/driver_inputs.hpp, mechanism/direction_check.hpp and runner/mechanism_routes.hpp. No frozen member changed shape, no enumerator moved, no behaviour of a 2.4 program changed. ONE thing to do, as at 2.1: a `switch` over `RoutineStopCause` that lists every value needs a `MechanismStepFailed` case (under `-Wswitch -Werror` it stops compiling without one — shulib's competition program and Auton Tester each gained one); every other 2.4 program compiles and runs exactly as before. docs/changelog.md, API 2.5, lists all of it.

*constant, declared at [`include/shulib/version.hpp:123`](../../include/shulib/version.hpp#L123).*

<a id="kapiversionstring"></a>

## `kApiVersionString`

```cpp
inline constexpr const char* kApiVersionString = "2.5"
```

"major.minor", for session headers / logs that want one printable token.

*constant, declared at [`include/shulib/version.hpp:126`](../../include/shulib/version.hpp#L126).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 40 lines</summary>

```text

 The shulib API version — the mechanism behind the Freeze Register's promise.

 The register (docs/roadmap.md, "Freeze Register") says a LOCKED contract
 changes "only with a schemaVersion/API-version bump and a migration path".
 Until D2 (2026-08-12) no such version existed anywhere in the tree; this
 header is that mechanism, made inspectable in code instead of prose.

 ═══ The policy: what a version bump concretely IS ═══════════════════════════════
 shulib is a header-only C++ library, so its API contract is SOURCE
 compatibility — what user programs compile to and mean:

   * BREAKING (bump kApiMajor): any change that can make a previously-
     compiling user program fail to compile or silently change meaning —
     removing or renaming a frozen member or type; changing a frozen
     signature's parameter, return, const, ref or noexcept shape; changing
     documented semantics a frozen surface carries (pre-empt, cancel's safe
     state, wait-for-live, the fault policy...); re-meaning or renumbering
     an existing enumerator. A major bump REQUIRES a migration note next to
     the Freeze Register row: the old spelling, the new spelling, the
     mechanical rewrite, and why the break was worth it.

   * ADDITIVE (bump kApiMinor): every previously-valid program compiles
     unchanged with unchanged behaviour — new members, new overloads, new
     options-struct fields whose default preserves the old behaviour,
     appended enumerators. This is the intended growth path of every frozen
     surface; freezing well means never needing a major bump.

 Frozen C++ surfaces are additionally enforced STRUCTURALLY: F6 carries a
 compile-time signature pin (test/f6_signature_pin_test.cpp) that fails the
 build, naming F6, if a frozen signature changes shape. If that pin sent
 you here and the change is intended: you are making a breaking change —
 update the pin, bump kApiMajor, write the migration note, and update the
 register row. If the change can be re-expressed additively instead, do
 that; it is almost always cheaper for everyone downstream.

 The data-schema freezes (F7/F8: .vexbot sub-schemas; F9: the SHUL/2 wire
 protocol) freeze DATA, not C++ source. They carry their own schemaVersion
 fields inside the artifacts (designed at Phase G/H, not here) and are
 governed by this same breaking-vs-additive policy.
```

</details>
