<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/match_runner.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `match_runner.hpp`

competition::runAuton — run ONE registered auton under a RunGuard, from its start pose to its guaranteed end.

This header declares **2** types (15 members) and **3** free functions.

Extracted from [`include/shulib/competition/match_runner.hpp`](../../include/shulib/competition/match_runner.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class AutonRunOutcome`](#enum-class-autonrunoutcome)
  - [`Refused`](#autonrunoutcome-refused)
  - [`NotStarted`](#autonrunoutcome-notstarted)
  - [`RoutineRan`](#autonrunoutcome-routineran)
  - [`RouteRan`](#autonrunoutcome-routeran)
- [`autonRunOutcomeWord`](#autonrunoutcomeword) — *free function*
- [`struct AutonRunResult`](#struct-autonrunresult)
  - [`outcome`](#autonrunresult-outcome)
  - [`why`](#autonrunresult-why)
  - [`guard`](#autonrunresult-guard)
  - [`routine`](#autonrunresult-routine)
  - [`route`](#autonrunresult-route)
  - [`armedAt`](#autonrunresult-armedat)
  - [`liveAfter`](#autonrunresult-liveafter)
  - [`seated`](#autonrunresult-seated)
  - [`imuAtSeat`](#autonrunresult-imuatseat)
  - [`mirrored`](#autonrunresult-mirrored)
  - [`started`](#autonrunresult-started)
- [`runAuton`](#runauton) — *free function*
- [`runAuton (overload 2)`](#runauton-2) — *free function*

<a id="enum-class-autonrunoutcome"></a>

## `enum class AutonRunOutcome`

```cpp
enum class AutonRunOutcome : std::uint8_t
```

How a runAuton call ended — the TAG on AutonRunResult (B16). Read it before any other field.

*enum class, declared at [`include/shulib/competition/match_runner.hpp:108`](../../include/shulib/competition/match_runner.hpp#L108).*

<a id="autonrunoutcome-refused"></a>

### `AutonRunOutcome::Refused`

```cpp
Refused
```

Refused before the guard armed; `why` says what. Nothing moved; `guard` is empty.

*enumerator, declared at [`include/shulib/competition/match_runner.hpp:110`](../../include/shulib/competition/match_runner.hpp#L110).*

<a id="autonrunoutcome-notstarted"></a>

### `AutonRunOutcome::NotStarted`

```cpp
NotStarted
```

The guard armed but the auton never started: the estimate did not go live before the run ended — the deadline, or an abandon (`guard.abandoned` says which). Nothing was re-seated; the guard's cancel-all still ran, and — unless abandoned — so did the end-of-run slot, filled by `chassis.brake()` and NEVER by the entry's own end action (that park is written for a start pose this run never seated; file banner, step 6). `why` says what.

*enumerator, declared at [`include/shulib/competition/match_runner.hpp:116`](../../include/shulib/competition/match_runner.hpp#L116).*

<a id="autonrunoutcome-routineran"></a>

### `AutonRunOutcome::RoutineRan`

```cpp
RoutineRan
```

The routine ran; `routine` holds its verdict.

*enumerator, declared at [`include/shulib/competition/match_runner.hpp:118`](../../include/shulib/competition/match_runner.hpp#L118).*

<a id="autonrunoutcome-routeran"></a>

### `AutonRunOutcome::RouteRan`

```cpp
RouteRan
```

The route ran; `route` holds its report.

*enumerator, declared at [`include/shulib/competition/match_runner.hpp:120`](../../include/shulib/competition/match_runner.hpp#L120).*

<a id="autonrunoutcomeword"></a>

## `autonRunOutcomeWord`

```cpp
[[nodiscard]] constexpr const char* autonRunOutcomeWord(AutonRunOutcome o) noexcept
```

The outcome as a word ("REFUSED", "NOT_STARTED", "ROUTINE_RAN", "ROUTE_RAN").

*free function, declared at [`include/shulib/competition/match_runner.hpp:124`](../../include/shulib/competition/match_runner.hpp#L124).*

<a id="struct-autonrunresult"></a>

## `struct AutonRunResult`

```cpp
struct AutonRunResult
```

What one runAuton call did (B16's tagged result).

*struct, declared at [`include/shulib/competition/match_runner.hpp:135`](../../include/shulib/competition/match_runner.hpp#L135).*

<a id="autonrunresult-outcome"></a>

### `AutonRunResult::outcome`

```cpp
AutonRunOutcome outcome = AutonRunOutcome::Refused
```

The tag.

*field, declared at [`include/shulib/competition/match_runner.hpp:137`](../../include/shulib/competition/match_runner.hpp#L137).*

<a id="autonrunresult-why"></a>

### `AutonRunResult::why`

```cpp
char why[192]{}
```

For Refused and NotStarted: the sentence, naming the auton. Empty otherwise.

*field, declared at [`include/shulib/competition/match_runner.hpp:139`](../../include/shulib/competition/match_runner.hpp#L139).*

<a id="autonrunresult-guard"></a>

### `AutonRunResult::guard`

```cpp
sequence::RunGuardReport guard{}
```

The guard's own account — valid unless Refused. `guard.pacesSeen == 0` after a run that did work means the Chassis was not built with this guard as its pacer.

*field, declared at [`include/shulib/competition/match_runner.hpp:142`](../../include/shulib/competition/match_runner.hpp#L142).*

<a id="autonrunresult-routine"></a>

### `AutonRunResult::routine`

```cpp
chassis::RoutineResult routine{}
```

The routine's verdict — valid only when `outcome == RoutineRan`.

*field, declared at [`include/shulib/competition/match_runner.hpp:144`](../../include/shulib/competition/match_runner.hpp#L144).*

<a id="autonrunresult-route"></a>

### `AutonRunResult::route`

```cpp
runner::RunReport route{}
```

The route's report — valid only when `outcome == RouteRan`.

*field, declared at [`include/shulib/competition/match_runner.hpp:146`](../../include/shulib/competition/match_runner.hpp#L146).*

<a id="autonrunresult-armedat"></a>

### `AutonRunResult::armedAt`

```cpp
units::Time armedAt{0.0}
```

The clock reading at which the guard ARMED (valid unless Refused) — the instant the auton's schedule is measured from. The program subtracts its own autonomous()-entry reading from this and logs the hand-off latency (B12: arm as close to autonomous() as the hand-off allows, and say how close that was).

*field, declared at [`include/shulib/competition/match_runner.hpp:151`](../../include/shulib/competition/match_runner.hpp#L151).*

<a id="autonrunresult-liveafter"></a>

### `AutonRunResult::liveAfter`

```cpp
units::Time liveAfter{0.0}
```

Seconds from the guard arming to the estimate being live — 0 only when it was live already. "Live" is `qualityClass() != Uninitialized`, which a Localizer never reports on its first update() (localizer.hpp, Quality), so one nothing has ticked yet takes two ticks here even with an IMU that was ready all along.

*field, declared at [`include/shulib/competition/match_runner.hpp:156`](../../include/shulib/competition/match_runner.hpp#L156).*

<a id="autonrunresult-seated"></a>

### `AutonRunResult::seated`

```cpp
math::Pose2d seated{}
```

The pose the re-seat set (the entry's effective start) — valid for RoutineRan / RouteRan.

*field, declared at [`include/shulib/competition/match_runner.hpp:158`](../../include/shulib/competition/match_runner.hpp#L158).*

<a id="autonrunresult-imuatseat"></a>

### `AutonRunResult::imuAtSeat`

```cpp
math::Angle imuAtSeat{}
```

The IMU's raw heading at the re-seat — what the start heading REPLACED (valid as `seated`).

*field, declared at [`include/shulib/competition/match_runner.hpp:160`](../../include/shulib/competition/match_runner.hpp#L160).*

<a id="autonrunresult-mirrored"></a>

### `AutonRunResult::mirrored`

```cpp
bool mirrored = false
```

MIRROR1-A: true when the entry was a MIRRORED one — `seated` is then robot two's REAL start (the reflection of the entry's own), and the screens' START line says MIR.

*field, declared at [`include/shulib/competition/match_runner.hpp:163`](../../include/shulib/competition/match_runner.hpp#L163).*

<a id="autonrunresult-started"></a>

### `AutonRunResult::started`

```cpp
[[nodiscard]] bool started() const noexcept
```

True when the auton's routine or route actually started.

*function, declared at [`include/shulib/competition/match_runner.hpp:166`](../../include/shulib/competition/match_runner.hpp#L166).*

<a id="runauton"></a>

## `runAuton`

```cpp
[[nodiscard]] inline AutonRunResult runAuton(chassis::Chassis& chassis, sequence::RunGuard& guard, const AutonEntry& entry, const runner::CommandRegistry* commands, hal::ITelemetrySink& telemetry, std::span<hal::IMechanism* const> mechanisms, units::Time period)
```

Run `entry` under `guard` (file banner: the order of events).  * `chassis` MUST be the Chassis built with `guard` as its pacer (robot::BuildOptions::pacer) — the guard counts paces and the result says so if it never saw one. * `commands` is the command registry a ROUTE entry's markers fire through; required for a route entry (refused without one), ignored for a routine entry, and may be nullptr then. * `mechanisms` is what the guard safes at the cut and at the end — the Robot's PRESENT mechanisms (B9). No nullptr (refused, before arming). * `period` is the period this run is in — the PROGRAM's constant for the entry's kind (C6).  PRECONDITIONS (a programming error, thrown): the guard is not already running, and no motion is active on the chassis (the re-seat must not move a heading under a live motion — B14). Every other problem is a REFUSAL with a sentence, returned without arming.

*free function, declared at [`include/shulib/competition/match_runner.hpp:364`](../../include/shulib/competition/match_runner.hpp#L364).*

<a id="runauton-2"></a>

## `runAuton (overload 2)`

```cpp
[[nodiscard]] inline AutonRunResult runAuton(chassis::Chassis& chassis, sequence::RunGuard& guard, const AutonEntry& entry, const runner::CommandRegistry* commands, hal::ITelemetrySink& telemetry, mechanism::MechanismService& mechanisms, units::Time period)
```

Run `entry` under `guard` WITH the robot's MECHANISM SERVICE (chunk MECH-A4, API 2.5; the file banner's three differences): the service's enterAutonomous() first, the guard's span is `mechanisms.mechanisms()`, and the routine's AutonContext carries the service, so `Routine r{ctx, "name"}` reaches every mechanism by its declaration. Every rule of the overload above holds unchanged. `mechanisms` belongs in the chassis's pacer chain under the guard, bound at boot before any auton can run; an UNBOUND one has no mechanisms (an empty span for the guard), and every mechanism step then ends ABSENT, saying the service is not bound — the drive steps still run.

*free function, declared at [`include/shulib/competition/match_runner.hpp:381`](../../include/shulib/competition/match_runner.hpp#L381).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 78 lines, click to expand</summary>

```text

 competition::runAuton — run ONE registered auton under a RunGuard, from its start pose to its
 guaranteed end (chunk COMP-a, 2026-09-26; the COMP brief's rulings C2, C5, C6 and amendments
 B9, B10, B12, B14, B16).

 ── The order of events, and why each is where it is ────────────────────────────────
   1. REFUSE, with a sentence, anything that cannot run — BEFORE the guard arms: an entry that
      fails the registry's own check against the period it is about to run in (an unset
      schedule, a hard stop past the period — C6; the guard's run() would THROW on the first),
      a route entry with no command registry, a null in the mechanisms span (B9: the span is the
      Robot's PRESENT mechanisms; an absent one is left out and logged by name where its name is
      known — the program). A refused auton moves nothing and returns `Refused`.
   2. ARM THE GUARD FIRST (B12). The run's clock starts as close to autonomous() as the hand-off
      allows, so the schedule the students wrote is measured from the start the field measures.
   3. Inside the guarded run, WAIT FOR A LIVE ESTIMATE with the guard's own wait (B12) — the one
      wait the deadline and a mid-auton disable can cut. An auton started while the IMU is still
      calibrating is slow, not wrong: it spends its first seconds here, motionless.
   4. RE-SEAT position AND heading on ONE tick (C2, B14): Chassis::setPose then
      Localizer::reseatHeading, with no update between. Only now — a re-seat during calibration
      would compute the heading bias from calibration garbage — and only with no motion active.
   5. Run the routine (or the route through a runner::PathRunner), with an AutonContext.
   6. The guard does the rest: the cut at endActionAt, the end action (the entry's, or
      `chassis.brake()`), the hard floor, the full cancel-all, and — if the program's watch
      called requestAbandon() — the skipped end action (run_guard.hpp's ABANDON banner).
      A run that never STARTED (step 3 ran out) gets `chassis.brake()` as its end action
      WHATEVER the entry says: an entry's park is written for the frame step 4 seats, and that
      frame was never seated. Were the IMU to come live during the end action's runway, a
      field-frame park would otherwise drive to its pose in the boot frame — the IMU's boot 0
      for a heading and wherever the estimate began for a position (COMP-a review finding).
 The result is TAGGED (B16): the RunGuardReport always, plus EITHER the routine's RoutineResult
 OR the route's RunReport, never a mix a reader has to guess about.

 ── With the robot's MECHANISM SERVICE (chunk MECH-A4, API 2.5) ─────────────────────
 The second overload takes the program's mechanism::MechanismService instead of a span. Three
 things differ, nothing above moves (both overloads run ONE body, detail::runAutonImpl):
   0. AUTON ENTRY, right after the preconditions and before step 1: the service's
      enterAutonomous(). PROS deletes the driver-control task on a mode change, so driver
      control's own exit may never have run — a driver's intake left at 12 V would otherwise run
      through the whole auton (the MECH brief's D1, measured on the host test plant: 269 of 269
      auton paces at 12 V without the call, 0 with it). It clears the service's re-entrancy flag,
      releases a claim another owner left (said, by name), stops every motor mechanism's
      behaviour (its safe state) and leaves a pneumatic's level alone. It runs even when the
      entry is then refused: whatever driver control left running stops either way.
   1. The guard's span is the service's mechanisms() — exactly the Robot's built ones (B9).
   5. The routine's AutonContext carries the service (`mechanismService`), so `Routine r{ctx,
      "name"}` reaches every mechanism by its declaration; a route's mechanism markers reach it
      through the program's registry (runner/mechanism_routes.hpp).
 The service must be IN the chassis's pacer chain, under the guard (Chassis -> RunGuard ->
 MechanismService -> the program's watch -> the tick pacer): a routine whose mechanism step
 waits on a service that never ticks stops, saying so (chassis/routine.hpp).

 ── A MIRRORED entry (chunk MIRROR1-A, 2026-11-13; competition/field_mirror.hpp) ─────
 An entry `competition::mirrored(...)` marked runs robot one's route on robot two, reflected. Three
 things differ, nothing above moves:
   4. The re-seat writes the REFLECTED start (field_mirror.hpp's seatedStart: a FIELD start through
      the entry's FieldMirror, a RELATIVE one through the robot-frame flip) — position AND heading
      on one tick, as ever — so `seated` is robot two's REAL start, where the drivers must place it.
      Its line's mode word reads "FIELD MIRRORED" / "RELATIVE MIRRORED", and a second line names the
      mirror and robot one's start as written (both ASCII, both under the 160-byte buffer and the
      terminal's 200-byte cut at their worst case — test/mirrored_auton_test.cpp measures them).
   5-6. ONE chassis::MirroredRun is opened after the re-seat and closed after the end action, around
      the routine (or route) AND the park, so the park runs mirrored too; it closes on every way out
      (a normal end, the deadline's cut, an abandon, a throw unwinding through runAuton) — before
      runAuton returns, so everything read afterwards (the program's END estimate) is the real
      field. The arming line's start word says MIRRORED as well, so a mirrored run that never
      starts is still named one.
 An UNMIRRORED entry runs byte for byte as before (test/mirror_unchanged_runs.hpp's hash, measured
 at the merge base, pinned). runAuton refuses (a precondition) a Chassis whose mirror is still on —
 a leaked run would reflect an unmirrored auton.

 ── What it does not do ─────────────────────────────────────────────────────────────
 It owns no task and reads no competition state: the program's auton task calls it, and the
 program's watch (a pacer under the guard) abandons it. It does not catch: a routine that THROWS
 (a precondition — a programming error) is safed by the guard on the unwind and the exception
 reaches the caller, whose task body catches it and says what broke (COMP-b). It never falls back
 to a default auton, a default schedule or a default start.

 PROS-free, allocation-free. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
