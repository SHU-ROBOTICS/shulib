<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/runner/path_runner.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `path_runner.hpp`

PathRunner — the engine that EXECUTES a Route.

This header declares **3** types (18 members) and **1** free function.

Extracted from [`include/shulib/runner/path_runner.hpp`](../../include/shulib/runner/path_runner.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class RunStopCause`](#enum-class-runstopcause)
  - [`None`](#runstopcause-none)
  - [`MotionFailed`](#runstopcause-motionfailed)
  - [`MarkerFailed`](#runstopcause-markerfailed)
- [`runStopCauseWord`](#runstopcauseword) — *free function*
- [`struct RunReport`](#struct-runreport)
  - [`stoppedAt`](#runreport-stoppedat)
  - [`reason`](#runreport-reason)
  - [`legsRun`](#runreport-legsrun)
  - [`legsSkipped`](#runreport-legsskipped)
  - [`markersFired`](#runreport-markersfired)
  - [`unknownIds`](#runreport-unknownids)
  - [`lastExit`](#runreport-lastexit)
  - [`ok`](#runreport-ok)
- [`class PathRunner`](#class-pathrunner)
  - [`PathRunner`](#pathrunner-pathrunner)
  - [`PathRunner (overload 2)`](#pathrunner-pathrunner-2)
  - [`operator=`](#pathrunner-operator-eq)
  - [`PathRunner (overload 3)`](#pathrunner-pathrunner-3)
  - [`operator= (overload 2)`](#pathrunner-operator-eq-2)
  - [`~PathRunner`](#pathrunner-destructor-pathrunner)
  - [`run`](#pathrunner-run)

<a id="enum-class-runstopcause"></a>

## `enum class RunStopCause`

```cpp
enum class RunStopCause : std::uint8_t
```

Why a route stopped early. `None` = it ran to the end. Append-only, like every verdict vocabulary in this project: a new stop cause arrives as a new enumerator, never as a re-meaning of an existing one, because log lines carry these spellings.

*enum class, declared at [`include/shulib/runner/path_runner.hpp:97`](../../include/shulib/runner/path_runner.hpp#L97).*

<a id="runstopcause-none"></a>

### `RunStopCause::None`

```cpp
None = 0
```

Every waypoint ran and every marker succeeded.

*enumerator, declared at [`include/shulib/runner/path_runner.hpp:99`](../../include/shulib/runner/path_runner.hpp#L99).*

<a id="runstopcause-motionfailed"></a>

### `RunStopCause::MotionFailed`

```cpp
MotionFailed = 1
```

A leg exited non-Settled (`RunReport::lastExit` carries which).

*enumerator, declared at [`include/shulib/runner/path_runner.hpp:101`](../../include/shulib/runner/path_runner.hpp#L101).*

<a id="runstopcause-markerfailed"></a>

### `RunStopCause::MarkerFailed`

```cpp
MarkerFailed = 2
```

A marker's handler — registered or built-in — returned an outcome other than `MechanismOutcome::Succeeded`.

*enumerator, declared at [`include/shulib/runner/path_runner.hpp:104`](../../include/shulib/runner/path_runner.hpp#L104).*

<a id="runstopcauseword"></a>

## `runStopCauseWord`

```cpp
[[nodiscard]] inline const char* runStopCauseWord(RunStopCause c) noexcept
```

The stop cause as a word, for banners and reports. "UNKNOWN" for a byte outside the enum.

*free function, declared at [`include/shulib/runner/path_runner.hpp:108`](../../include/shulib/runner/path_runner.hpp#L108).*

<a id="struct-runreport"></a>

## `struct RunReport`

```cpp
struct RunReport
```

What one route run did — the whole-run verdict.  A bare `ExitReason` would lose WHERE the route broke and HOW MUCH of it never ran, which is exactly `RoutineResult`'s and `TrajectoryResult`'s argument one tier up. The counts are here so that "the auton just did not score" has an answer other than a guess.

*struct, declared at [`include/shulib/runner/path_runner.hpp:122`](../../include/shulib/runner/path_runner.hpp#L122).*

<a id="runreport-stoppedat"></a>

### `RunReport::stoppedAt`

```cpp
int stoppedAt = -1
```

The 0-BASED index of the waypoint the route stopped at; **-1 when nothing stopped it**. Zero-based with a -1 sentinel, deliberately unlike `RoutineResult::stoppedAt` (1-based, 0 = none): waypoint 0 is a real, addressable waypoint here, and the number must match the index printed in this run's own log lines and drawn in the path editor. Re-basing it would make a report and the transcript describing it disagree.

*field, declared at [`include/shulib/runner/path_runner.hpp:128`](../../include/shulib/runner/path_runner.hpp#L128).*

<a id="runreport-reason"></a>

### `RunReport::reason`

```cpp
RunStopCause reason = RunStopCause::None
```

What kind of thing stopped the route. Read this BEFORE `lastExit` — only `MotionFailed` puts a failing motion verdict there.

*field, declared at [`include/shulib/runner/path_runner.hpp:131`](../../include/shulib/runner/path_runner.hpp#L131).*

<a id="runreport-legsrun"></a>

### `RunReport::legsRun`

```cpp
int legsRun = 0
```

Waypoints whose verb actually executed, including the one that failed.

*field, declared at [`include/shulib/runner/path_runner.hpp:133`](../../include/shulib/runner/path_runner.hpp#L133).*

<a id="runreport-legsskipped"></a>

### `RunReport::legsSkipped`

```cpp
int legsSkipped = 0
```

Waypoints never attempted because the route stopped before them.

*field, declared at [`include/shulib/runner/path_runner.hpp:135`](../../include/shulib/runner/path_runner.hpp#L135).*

<a id="runreport-markersfired"></a>

### `RunReport::markersFired`

```cpp
int markersFired = 0
```

Markers whose handler RAN — including the one that failed, and including the built-ins. An unknown id is not counted here (nothing ran); see `unknownIds`.

*field, declared at [`include/shulib/runner/path_runner.hpp:138`](../../include/shulib/runner/path_runner.hpp#L138).*

<a id="runreport-unknownids"></a>

### `RunReport::unknownIds`

```cpp
int unknownIds = 0
```

Markers whose id matched no built-in and no registration: warned, skipped, counted. A non-zero count on a route that "worked" is the single most useful number in this struct.

*field, declared at [`include/shulib/runner/path_runner.hpp:141`](../../include/shulib/runner/path_runner.hpp#L141).*

<a id="runreport-lastexit"></a>

### `RunReport::lastExit`

```cpp
control::ExitReason lastExit = control::ExitReason::Running
```

The most recent MOTION leg's verdict. `Running` means no motion leg has run yet (a route of a lone `Start`, or an empty one) — the project's "no verdict here" convention, the same one `Routine::lastTrajectory()` uses.

*field, declared at [`include/shulib/runner/path_runner.hpp:145`](../../include/shulib/runner/path_runner.hpp#L145).*

<a id="runreport-ok"></a>

### `RunReport::ok`

```cpp
[[nodiscard]] bool ok() const noexcept
```

True when the route ran to the end with every marker succeeding.

*function, declared at [`include/shulib/runner/path_runner.hpp:148`](../../include/shulib/runner/path_runner.hpp#L148).*

<a id="class-pathrunner"></a>

## `class PathRunner`

```cpp
class PathRunner
```

The engine that executes a `config::Route` through the frozen `Chassis` verbs, firing each waypoint's markers through a `CommandRegistry` once the leg settles (file banner).  runner::CommandRegistry registry; registry.on("intake_in", intakeIn); runner::PathRunner runner{chassis, registry, telemetry}; const runner::RunReport report = runner.run(*routes.route("left_side_rush"));  Borrows all three (they must outlive it) and owns nothing. Non-copyable and non-movable for the same reason `Routine` is: it holds references, and a second handle would be a second opinion about a run that has one.

*class, declared at [`include/shulib/runner/path_runner.hpp:162`](../../include/shulib/runner/path_runner.hpp#L162).*

<a id="pathrunner-pathrunner"></a>

### `PathRunner::PathRunner`

```cpp
explicit PathRunner(chassis::Chassis& chassis, const CommandRegistry& registry, hal::ITelemetrySink& telemetry) noexcept
```

Borrows `chassis` (the verbs), `registry` (the id → action table) and `telemetry` (where the run's narrative goes). All three must outlive this object. The registry is `const`: running a route never registers anything.

*function, declared at [`include/shulib/runner/path_runner.hpp:167`](../../include/shulib/runner/path_runner.hpp#L167).*

<a id="pathrunner-pathrunner-2"></a>

### `PathRunner::PathRunner (overload 2)`

```cpp
PathRunner(const PathRunner&) = delete
```

Non-copyable, non-movable (class comment).

*function, declared at [`include/shulib/runner/path_runner.hpp:172`](../../include/shulib/runner/path_runner.hpp#L172).*

<a id="pathrunner-operator-eq"></a>

### `PathRunner::operator=`

```cpp
PathRunner& operator=(const PathRunner&) = delete
```

*Covered by the comment on [`PathRunner (overload 2)`](#pathrunner-pathrunner-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/runner/path_runner.hpp:173`](../../include/shulib/runner/path_runner.hpp#L173).*

<a id="pathrunner-pathrunner-3"></a>

### `PathRunner::PathRunner (overload 3)`

```cpp
PathRunner(PathRunner&&) = delete
```

*Covered by the comment on [`PathRunner (overload 2)`](#pathrunner-pathrunner-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/runner/path_runner.hpp:174`](../../include/shulib/runner/path_runner.hpp#L174).*

<a id="pathrunner-operator-eq-2"></a>

### `PathRunner::operator= (overload 2)`

```cpp
PathRunner& operator=(PathRunner&&) = delete
```

*Covered by the comment on [`PathRunner (overload 2)`](#pathrunner-pathrunner-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/runner/path_runner.hpp:175`](../../include/shulib/runner/path_runner.hpp#L175).*

<a id="pathrunner-destructor-pathrunner"></a>

### `PathRunner::~PathRunner`

```cpp
~PathRunner() = default
```

*Covered by the comment on [`PathRunner (overload 2)`](#pathrunner-pathrunner-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/runner/path_runner.hpp:176`](../../include/shulib/runner/path_runner.hpp#L176).*

<a id="pathrunner-run"></a>

### `PathRunner::run`

```cpp
[[nodiscard]] RunReport run(const config::Route& route)
```

Execute `route`, blocking until it finishes or stops, and report what happened.  Precondition: `route.valid()` — the refusal message is `valid()`'s own, so a bad route names the waypoint and the field rather than failing somewhere downstream. Validating at the door rather than per leg is `followTrajectory`'s rule: a contradiction at waypoint 7 must not let waypoints 0–6 drive first.  The mapping from waypoint to verb, in full: * `Start`      — `setPose(x, y, heading)`; no motion, no verdict. Legal at index 0 only. * `MoveToPose` — `moveTo({x, y, heading})`. With `HeadingMode::Free` the commanded heading is the ESTIMATE'S heading read at the leg's start ("keep what you have"; ruling R6) — `moveTo` always commands a heading, so there is no "don't care" to pass. * `DriveTo`    — `moveTo({x, y, bearingTo(x, y)})`, arriving facing the point, exactly `Routine::driveTo`. `reverse` adds π, so the robot arrives BACKWARDS onto the point — exactly `Routine::reverseTo` (a tank route turns its back to the point first with an authored `TurnTo`, as a recipe does with `faceAway`; HA-167 registers the belief that a tank backs up as far as its calibration says). * `TurnTo`     — `turnTo(heading)`; `x` and `y` are not read. * `StrafeTo`   — `strafeTo(x, y)`. Nothing here special-cases the drivetrain: a tank reports strafe authority 0 and the facade's own honest fallback applies. * `Hold`       — `hold(constraints.timeout)`; for that verb the timeout IS the duration. Each waypoint's `Constraints` become the leg's `MotionOptions` field for field (0 = the robot's configured default, the same sentinel both types already use).

*function, declared at [`include/shulib/runner/path_runner.hpp:201`](../../include/shulib/runner/path_runner.hpp#L201).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 71 lines, click to expand</summary>

```text

 PathRunner — the engine that EXECUTES a Route (chunk G2, 2026-09-18; master plan §16.3).

 ── What this file finally closes ───────────────────────────────────────────────────
 This team has authored three parallel data representations of an autonomous routine and
 executed none of them (docs/legacy-command-vocabulary.md §1). What was missing every time
 was not the format — it was this: a thing that walks the waypoints, calls the drivetrain
 verbs, and fires the markers. G1 built the data; this builds the runner; G3 fills the data
 from a `.vexbot` file. A Route built by hand in C++ and a Route parsed from a file are the
 same object and run through the same code, which is what makes the file path testable
 before the file format exists.

 ── It drives the FACADE, never the recipe chain (ruling D1, restated at G2 R1) ─────
 `chassis::Routine` is the Tier-2 recipe layer and this runner is NOT one of its clients.
 The chain is EAGER and fluent — built for a human writing steps in program order — and its
 verdict type, its step numbering and its log vocabulary all describe an authored chain, not
 a table of waypoints. So the runner calls `Chassis` directly.

 That leaves one thing the two layers must agree on EXACTLY: the bearing. `Routine::driveTo`
 computes "face the point" as `atan2` of the deltas from the live pose estimate, and if this
 file computed it any other way — a different wrap, a different pose read, `atan2(dx, dy)` —
 a route and a recipe aiming at the same point would disagree by an ulp today and by a
 degree after the first refactor. `bearingTo` below is `Routine::bearingTo`'s expression,
 character for character, and a test on identical rigs pins the two runs BIT-IDENTICAL.
 (It is duplicated rather than shared because `Routine::bearingTo` is private, and F10 — the
 frozen `Routine` surface — grows only additively: a new public member there would be a new
 frozen step, not a helper for this file. F1 and L9 both grew it that way.) The same holds for
 the one other piece of arithmetic here: `reverse`'s `+ π` below is `Routine::reverseTo`'s,
 character for character, and pinned bit-identical to it on a tank and on the X-drive.

 ── The error policy is D1's, in meaning, verbatim ──────────────────────────────────
 A route that keeps driving after a failed leg is chasing the field from a position it is
 not at. So on the FIRST leg that exits non-Settled, the FIRST handler that reports failure,
 or the FIRST marker outcome other than `Succeeded`:
   1. the route STOPS,
   2. the drive is put in the defined safe state — `Chassis::cancel()` (0 V + Brake,
      idempotent), NOT `brake()`: `brake()` is a blocking MOTION that can itself time out,
      and the safe state must not be able to fail,
   3. every later waypoint is SKIPPED, counted, and visible in the report,
   4. one Warn names the route, the waypoint and the cause.
 PRECONDITION THROWS ARE NOT POLICY: a NaN or a nonsense duration throws out of the facade
 exactly as it would from any other caller. A programming error stays loud.

 ── Markers fire on Settled, and only on Settled (ruling R2) ────────────────────────
 A waypoint's markers run IN ORDER after its motion settles. A leg that TIMED OUT fires
 none of them, because a marker that fires at the wrong place is a robot doing the wrong
 thing in the wrong spot — strictly worse than a marker that does not fire at all. (A
 `Hold` whose window simply elapses IS settled: `Chassis::hold` returns Settled when it held
 within tolerance to the end, which is a hold ending by design.)

 ── An unknown id is DATA, not a defect ─────────────────────────────────────────────
 `vexbuilder-integration.md` §4.2 promises VexBuilder, in writing: "an unknown id logs a
 warning and is skipped; it never crashes." So an id no handler and no built-in matches
 costs exactly one `LogLevel::Warn` line naming the id and the waypoint index, one increment
 of `RunReport::unknownIds`, and nothing else — no latched fault, no stop. A typo in a
 picker must not end a match. (An id outside the CHARSET is a different thing and never
 reaches here: `Route::valid()` refuses it, because no registry could ever hold it.)

 ── Blocking, single-task, and the guard is the pacer ───────────────────────────────
 `run()` blocks exactly as the facade verbs it calls do, on the caller's task. It owns no
 loop and no deadline: a route that must end with the match is wrapped in a
 `sequence::RunGuard`, whose pacer cuts the active motion and whose end action parks the
 robot — the runner needs nothing special for that to work, because the Chassis was
 constructed with the guard as its pacer.

 NO HARDWARE CLAIM. A Route has executed on the A2 host plant only. The library's motion
 stack has never driven a physical robot.

 FROZEN: F8 (docs/roadmap.md), pinned by test/f8_signature_pin_test.cpp.

 PROS-free, sim-free, allocation-free; no std::string, no std::function.
```

</details>
