# G2 — `PathRunner` + the command-id registry + the manifest → freeze F8

> **Execution brief.** Written 2026-09-18 by the coordinator during the autonomous session
> ([session log](SESSION-2026-09-18-autonomous.md)), tree `fe8e270` (G1 Part 1 landed). The
> build-order entry ("G2 — `PathRunner` + command-id registry → freeze F8"), master plan §16.3 and
> §17, and [`docs/vexbuilder-integration.md`](../../vexbuilder-integration.md) §4 are the spec.
> Live log: `docs/internal/chunks/G2-PROGRESS.md`, created FIRST. **Executor model: `opus`. Do not
> commit. Do not touch `src/`** — the team lead's bench session runs on packages built from
> `5e4ce65`, and `src/` must stay as committed until it is over.

---

## 0. Why this chunk, why now

The team lead's stated goal: *a bot made without coding.* G1 gave the data types (a `Route` is 32
waypoints with markers; a marker is an id and typed args — never code). G2 is the keystone the master
plan names (§16.3): **the auton is data a non-coder authors; the handlers are code a coder registers
once.** `runner.on("intake_in", …)` binds an id to an action; the runner executes a `Route` through
the facade's verbs and fires the markers. Nothing here needs VexBuilder: a `Route` is built in C++ by
a test today and parsed from a `.vexbot` by G3 tomorrow. It runs on the host plant like every
routine before it.

**And it freezes F8.** The register row F8 is "`paths[]` sub-schema + command-id vocabulary inside
`.vexbot`". What freezes at G2 is (a) the IN-MEMORY vocabulary — `config::Route`/`Waypoint`/`Marker`
and their enums (G1), the registry's public API, and the built-in ids — pinned by a signature test
in the F6/F10 mould; and (b) the FILE schema's field names and value vocabularies, written into
`vexbuilder-integration.md` §4.1 as the contract G3 parses and VexBuilder writes. G3 implements the
parser against a frozen schema rather than inventing one.

---

## 1. What exists to build on (read these)

| Piece | Where | What matters |
|---|---|---|
| The route types (G1) | `include/shulib/config/route.hpp` | `Route{name, waypoints[32], count, valid()}`, `Waypoint{x, y, heading, headingMode, motion, reverse, constraints, markers[4], markerCount}`, `WaypointMotion{Start, MoveToPose, DriveTo, TurnTo, StrafeTo, Hold}`, `HeadingMode{Fixed, Free}`, `Marker{id[24], args[2], argCount}`, `IRouteSource`/`StaticRoutes`. **G2 executes exactly this vocabulary and freezes it** |
| The facade (F6, frozen) | `include/shulib/chassis/chassis.hpp:264-451` | `moveTo(Pose2d, MotionOptions) → ExitReason`, `strafeTo(x, y, opts)`, `turnTo(Angle, opts)`, `hold(Time, opts)`, `wait(Time)`, `brake(opts)`, `setPose(Pose2d)`, `cancel()`, `pose()`; `MotionOptions{timeout, maxLinearSpeed, maxAngularSpeed}` |
| The recipe's arithmetic (F10, frozen — read, do not touch) | `include/shulib/chassis/routine.hpp:213-300` | `driveTo(x, y)` = `moveTo({x, y, bearingTo(x, y)})`; `face(x, y)` = `turnTo(bearingTo)`; the chain error policy (first failure → stop → `cancel()` → skip the rest, counted → report); `then()` deliberately unfrozen. **D1 ruled G2 does NOT consume the recipe chain** (roadmap F12 row): the runner drives the facade directly and reproduces `bearingTo` bit-identically (pin it against `Routine`) |
| The end-of-run guard | `include/shulib/sequence/run_guard.hpp` | `RunGuard` is an `ITickPacer` with a run deadline; a runner under a guard needs nothing special — the chassis was built with the guard as its pacer. `GuardedWaitResult` is the sequence layer's vocabulary |
| The mechanism seam (F1/F11, unfrozen) | `include/shulib/manipulation/mechanism_op.hpp`, `include/shulib/hal/mechanism.hpp` | `IMechanismOp` + `MechanismOutcome`; a handler that runs a mechanism operation returns its outcome. G2 does not construct mechanisms (the config types exist; T3/G2's registry binds what a coder hands it) |
| Faults and telemetry | `include/shulib/diag/fault.hpp` (append-only `FaultCode`), `include/shulib/hal/telemetry_sink.hpp` (`log(LogLevel, subsystem, message)`) | "an unknown id logs a WARN and is skipped, never crashes" — a Warn LINE and a counter, never a latched fault |
| The freeze machinery | `test/routine_signature_pin_test.cpp` (F10), `test/f6_signature_pin_test.cpp`, `include/shulib/version.hpp` | the concept-templated exact-cast pins with `{ call } noexcept` compound requirements; the register row format; the breaking-vs-additive policy |
| The file schema promised to VexBuilder | `docs/vexbuilder-integration.md` §4.1–4.3 | `headingMode: "locked"\|"tangent"\|"free"`, `motion: "move"\|"strafe"\|"turn"\|"arc"`, `constraints{maxLinearSpeed?, maxAngularSpeed?, timeout?}`, `markers[{id, args?}]`; the `NONE` boundary-marker question; "the manifest does not exist yet (it ships with G2)" |
| The legacy vocabulary | `docs/legacy-command-vocabulary.md` §3–5 | one motion id ever emitted (`MOVE_WITH_HEADING` ≡ `moveTo`); `NONE` = a segment boundary; five aspirational manipulation ids; G4's importer requirements |
| A host rig | `test/motion_test_rig.hpp`, `test/chassis_routine_test.cpp`, `test/robot_builder_test.cpp` (G1's "two lines" rig) | how a `Chassis` on the plant is driven in a test; the routine tests' ground-truth grading |

---

## 2. Scope

### IN

1. **`include/shulib/runner/command_registry.hpp`** — `runner::CommandRegistry` (fixed capacity
   `kMaxCommands = 32`, allocation-free, no `std::function`): `on(std::string_view id, Handler)`
   where `Handler{MechanismOutcome (*fn)(void* ctx, const config::Marker&); void* ctx}` plus a
   template `on(id, Callable&)` that stores a NON-OWNING pointer to the caller's callable (it must
   outlive the registry — the tree's ownership convention) and adapts `void`/`bool`/
   `MechanismOutcome` returns exactly as `Routine::then()` does (`void` = always succeeds, `bool`,
   `MechanismOutcome`); `find(id)`, `count()`, `ids()`; duplicate registration is a precondition
   violation (two handlers for one id is a programming error); an id longer than
   `config::kMaxMarkerIdBytes` (24 — G1 wrote `char id[24]` without naming the constant; ADD it to
   `route.hpp` additively and use it in both places), empty, or with characters outside `[a-z0-9_]` is refused
   (`valid()` on the id — the vocabulary rule below).
2. **The built-in ids** — library-owned, handled by the runner itself, never registrable by a team
   (registering one is a precondition violation): `wait_ms(args[0])`, `brake`, `hold_ms(args[0])`,
   `log(args…)` (a Warn-free Info line with the marker's args — the non-coder's "print"). Their
   names and arg meanings are part of F8. **`NONE` is NOT an id**: the segment boundary the legacy
   planner emitted is the `Route`'s own structure (a waypoint is a segment), and G4's importer folds
   `NONE` rows into waypoints — the answer to the integration doc's open question, recorded there.
3. **`include/shulib/runner/path_runner.hpp`** — `runner::PathRunner{chassis::Chassis&,
   const CommandRegistry&, hal::ITelemetrySink&}`; `RunReport run(const config::Route&)`, BLOCKING
   (like the facade's verbs; the guard is the pacer), one waypoint at a time in order:
   - `Start` (index 0 only — `Route::valid()` enforces) → `chassis.setPose(x, y, heading)`.
   - `MoveToPose` → `moveTo({x, y, heading}, opts)`; `HeadingMode::Free` → the heading argument is
     the CURRENT estimate's heading (hold what you have); `reverse` on a `MoveToPose` is invalid
     (there is no direction to reverse) — `valid()` refuses it.
   - `DriveTo` → `moveTo({x, y, bearingTo(x, y)})` exactly as `Routine::driveTo` (pinned
     bit-identical to it); `reverse` → bearing + π (drive backwards to the point).
   - `TurnTo` → `turnTo(heading)`; `HeadingMode::Free` invalid here (`valid()`).
   - `StrafeTo` → `strafeTo(x, y)` (a tank reports its strafe authority 0 and the facade's own
     fallback applies — nothing here special-cases the drivetrain).
   - `Hold` → `hold(constraints.timeout)`.
   - Per-waypoint `Constraints` → `MotionOptions` (0 = the facade's default, as G1 defined them).
   - After the motion SETTLES, the waypoint's markers fire IN ORDER; a marker fires ONLY on
     `ExitReason::Settled` (a timed-out leg does not fire its markers — a marker that fires at the
     wrong place is worse than one that does not fire).
   - **Error policy = D1's, verbatim in meaning:** the first non-Settled leg, the first handler
     returning failure, or the first `MechanismOutcome` other than success → STOP → `chassis.cancel()`
     (0 V + Brake) → skip the rest (counted) → `RunReport{stoppedAt, reason, legsRun, legsSkipped,
     markersFired, unknownIds, lastExit}`. Preconditions throw through untouched.
   - **An unknown id** → one `LogLevel::Warn` line naming the id and the waypoint index, counted
     in `RunReport::unknownIds`, the route CONTINUES (the integration doc's guarantee: "logs a
     warning and is skipped; never crashes").
   - `cancel()` from another context is the facade's; the runner adds nothing that a task must own.
4. **`include/shulib/runner/manifest.hpp`** — `writeManifest(const CommandRegistry&, hal::ICharSink&)`:
   one JSON object per line (`{"id":"intake_in","builtin":false,"args":0}` — a line-oriented
   format because `ICharSink` is line-oriented and the picker reads a small file), the built-ins
   FIRST with their arg counts and meanings, then the registered ids in registration order; a
   `manifestSchemaVersion` field on the first line. Byte-exact golden test. The manifest is what
   VexBuilder's picker reads (integration doc ask #4).
5. **The F8 freeze:** `test/f8_signature_pin_test.cpp` pinning `config::Route`/`Waypoint`/`Marker`/
   `Constraints`, the three enums' values, `kMaxWaypoints`/`kMaxMarkersPerWaypoint`/
   `kMaxMarkerArgs`/`kMaxMarkerIdBytes`, `CommandRegistry`'s public members, `PathRunner::run`,
   `RunReport`, and the built-in id strings — compile-time, naming F8 in the message, in F10's
   mould (concept-templated exact casts, `{ call } noexcept` where noexcept is promised).
   `version.hpp`'s policy applies unchanged; `kApiMinor` is bumped (additive growth).
6. **The file schema reconciled and FROZEN in `docs/vexbuilder-integration.md` §4.1:** the JSON
   field names and value vocabularies stated to match the in-memory types exactly —
   `motion: "start"|"move"|"drive"|"turn"|"strafe"|"hold"` (the doc's `"arc"` is DROPPED: no facade
   verb exists for it and F8 freezes nothing the library cannot do; recorded as a named gap with the
   sentence that adds it later as an additive value), `headingMode: "fixed"|"free"` (the doc's
   `"locked"` → `"fixed"`, `"tangent"` → `drive` with `fixed` heading computed by the runner — stated
   in a mapping table), `reverse`, `constraints{maxLinearSpeed, maxAngularSpeed, timeout}` in
   canonical units (in/s, rad/s, s — say so), `markers[{id, args:[int…]}]` (typed args are an
   INTEGER ARRAY, ≤ 2 — the doc's `args: {…}` object becomes a positional array, because the
   in-memory type is `int32_t args[2]` and a picker can label positions from the manifest). The
   `NONE` question answered (item 2). G3 parses THIS.
7. Tests (§4), the DoD (§5), the documentation contract (§6).

### OUT

- Parsing any file → **G3**. The `.shupaths` importer → **G4**. Mechanism construction → **T3**.
- A non-blocking / async runner (a route that runs beside teleop) → later, additive.
- `"arc"` motion, spline/blended trajectories → the roadmap's blending item (a measured Frontier).
- Freezing `then()` or the mechanism seam (F11 stays open by design).

### REJECTED

- **`std::function` handlers** (heap, and the tree bans it on the V5); the non-owning callable
  pointer is the `Routine::then()` precedent's ownership rule applied to a table.
- **Unknown id = fault abort.** A typo in a picker must not end a match; the doc promised "skipped".
- **Firing markers before the leg settles** (a marker at the wrong place is a robot doing the wrong
  thing in the wrong spot).
- **A free-form id vocabulary.** `[a-z0-9_]`, ≤ 23 bytes, so a manifest line, a picker label and a
  handler name are the same string on every platform and no escaping ever exists.

---

## 3. Rulings

**R1 — The runner reproduces `Routine::driveTo`'s bearing bit for bit and is PINNED against it.**
Two layers computing "face the point" two ways is how a route and a recipe come to disagree by an
ulp and then by a degree. Rejected: calling `Routine` from the runner (D1's ruling that G2 does not
consume the chain; the chain's eager semantics and result type are a different contract).

**R2 — Markers fire on Settled only; the leg's result is the gate.** See REJECTED. A `Hold` leg
whose timeout elapsed is "settled" for this purpose (a hold ends by design).

**R3 — Built-ins are library-owned and un-registerable.** A team that registers `brake` to mean
something else breaks every route another team exported; the precondition is the fence.

**R4 — The manifest is line-oriented JSON through `ICharSink`.** One record per line keeps the
sink's contract ("one write carries one complete line"), keeps the picker's parser trivial, and
keeps the file diffable. Rejected: a single JSON document (needs a serializer with escaping the
vocabulary rule makes unnecessary).

**R5 — What F8 freezes and what it leaves additive.** Frozen: the in-memory types and their
enum VALUES (persisted by G3's codegen as literals), the registry's members, `run()`'s signature,
`RunReport`'s fields, the built-in id strings, the file schema's field names. Additive under
`kApiMinor`: new enum values at the end, new built-ins, new `RunReport` fields, new optional file
fields. Breaking (major bump + migration): everything else. The register row says so.

**R6 — `HeadingMode::Free` on `MoveToPose` means "keep the current estimated heading"**, read at
the moment the leg starts. Rejected: "don't care, let the controller pick" (there is no such mode
in MoveToPose; heading is always commanded).

---

## 4. Tests — `test/command_registry_test.cpp`, `test/path_runner_test.cpp`, `test/manifest_test.cpp`, `test/f8_signature_pin_test.cpp`

| # | Test | Property | Mutation |
|---|---|---|---|
| 1 | Registry: `on`/`find`/`count`/`ids`; duplicate id ⇒ precondition; built-in id ⇒ precondition; invalid id chars/length/empty ⇒ refused; capacity 32 then precondition | the table's rules | drop the duplicate check; widen the charset |
| 2 | Handler adaptation: `void` always succeeds, `bool false` fails, `MechanismOutcome` passes through — pinned against `Routine::then()`'s adaptation on the same callables | one adaptation rule in the tree | flip the bool |
| 3 | **Runner on the plant, bit-identical to the recipe:** a route of `Start, DriveTo, TurnTo, MoveToPose(Fixed), Hold` vs the same legs written with `Routine` on an identical rig — per-tick truth `memcmp`, exit reasons, tick counts | R1; the runner adds a way to author, not a physics | change the bearing formula (`atan2` args swapped) |
| 4 | Tank and X-drive both: the route above settles on ground truth within the routine tests' bounds | the vocabulary works on both drivetrains | — |
| 5 | Markers fire in order after Settled; a `TimedOut` leg fires none and stops the route; `RunReport` counts (legsRun, legsSkipped, markersFired) hand-computed | R2 + the error policy | fire before settle; continue past a timeout |
| 6 | Unknown id: one Warn line naming id + index, counted, the route CONTINUES to the end | the guarantee | make it stop; make it silent |
| 7 | Handler failure (`bool false`; `MechanismOutcome` non-success) ⇒ stop, `cancel()` observed at the motors (0 V + Brake), the rest skipped and counted | D1's policy through the runner | skip the cancel |
| 8 | Built-ins: `wait_ms` advances the plant clock by exactly the arg; `hold_ms` holds; `brake`; `log` emits an Info line with the args | library-owned ids | — |
| 9 | `HeadingMode::Free` on `MoveToPose` commands the estimate's heading at leg start (a rig with a known heading) | R6 | use 0 instead |
| 10 | `reverse` on `DriveTo` ⇒ bearing + π, robot arrives backwards (ground truth heading) | the tank's reverse | drop the π |
| 11 | Manifest golden: exact bytes for a registry with the built-ins + two ids + one with args; schema version on line 1; through `FakeCharSink` one line per write | R4 | reorder built-ins |
| 12 | F8 pins: the test file compiles; a deliberate mutation of one frozen signature FAILS THE BUILD naming F8 (run it, paste the message, restore) | the freeze is mechanical | — |
| 13 | `Route::valid()` gains the G2 rules (reverse on MoveToPose; Free on TurnTo; a built-in id with the wrong arg count) — each the sole cause | data cannot ask for what cannot run | drop a rule |

Every mutation RUN, observed red on a fresh binary (edit → `touch` the test → `--target shulib_tests/fast` → require the link line), message logged, restored.

---

## 5. DoD

- [ ] `runner/command_registry.hpp`, `runner/path_runner.hpp`, `runner/manifest.hpp`; every public
      member documented; ARM gate green; PROS-free and sim-free
- [ ] Tests 1–13 green; every mutation logged red; the pre-existing suite green with pre-existing
      cases moved by zero assertions (except the CI-guard mirror's three per new header, stated)
- [ ] **F8 frozen**: the pin test, the register row `✅ LOCKED 2026-09-18` with the frozen list and
      the additive list, `version.hpp` minor bump with the note, the schema in the integration doc
- [ ] `src/` untouched; the six-build gate green
- [ ] `G2-PROGRESS.md` complete; no hardware claim (a route has executed on the plant only)

---

## 6. Documentation, in the same commit

Touch: `docs/api/` regenerated; `docs/roadmap.md` — the F8 register row (✅ LOCKED, what is frozen,
what is additive, the pin test named), the M5 "Path authoring" bullets (`PathRunner` and the
registry → `[x]` for the runner's host half; `io/Trajectory` reader stays `[ ]` — G3), "You are
here"; `docs/vexbuilder-integration.md` §4.1–4.3 (the reconciled, frozen schema; the mapping table
old→new names; the `NONE` answer; "the manifest exists" with its format); `docs/internal/build-order.md`
— the G2 status line, the first `Next:` block (one sentence); `include/shulib/version.hpp` (minor
bump, the F8 note); `PROJECT-BRIEFING.md` regenerated. NOT touched, with why: `README.md` and the
guide (a chapter "Your first data-driven routine" is written when G3 can load one from a file —
say so in the log), `docs/hardware-assumptions.md` (no hardware belief), `src/`, the master plan
(§16.3's sketch is what this implements).

Prose review logged per changed/added source file: grep `docs/` and `README.md` for `PathRunner`,
`runner.on`, `command-id`, `command id`, `manifest`, `F8`, `paths[]`, `markers`, `code_template`;
read every hit; fix what is stale in the files you may touch; list the rest.

---

## 7. Landmines

1. **`Routine::bearingTo` is private** — reproduce its exact expression (`atan2` of the deltas from
   the CURRENT pose, wrapped by `Angle`) and PIN equality; do not touch `routine.hpp` (F10's pins
   fail the build).
2. **`chassis.cancel()` is the safe state, not `brake()`** — D1's policy is `cancel()`.
3. **A handler runs on the caller's task, synchronously, between legs** — a handler that blocks for
   seconds blocks the route; say so in the header (T3 will own asynchronous mechanism operations).
4. **No `std::string`, no `std::function`, no heap** in `runner/`; ids are `char[24]`, compare with
   `std::string_view` semantics on the NUL-terminated buffer.
5. **The manifest's built-in order is part of its golden** — append only.
6. **Do not "fix" the doc's schema toward the in-memory types by changing the in-memory types** —
   G1's `Route` is what the runner executes and what the pin freezes; the DOC converges to the code
   (with the mapping table), not the other way.
7. **No hardware claim.** A route has executed on the plant only.

---

## 8. Verification (the coordinator)

`verify.sh`; read `path_runner.hpp`'s `run()` in full against `Routine`'s error policy and the
bearing pin; re-run mutation 3 (the bearing) and mutation 5 (fire before settle) by hand; run the
F8 pin's deliberate mutation and read the build's message; confirm `src/` untouched and the register
row.
