<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/acceptance_run.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `acceptance_run.hpp`

competition::acceptanceRun — the LIBRARY's acceptance run: the one routine shulib ships.

This header declares **2** free functions and **1** constant.

Extracted from [`include/shulib/competition/acceptance_run.hpp`](../../include/shulib/competition/acceptance_run.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kAcceptanceRunName`](#kacceptancerunname) — *constant*
- [`acceptanceRun`](#acceptancerun) — *free function*
- [`acceptanceRunEntry`](#acceptancerunentry) — *free function*

<a id="kacceptancerunname"></a>

## `kAcceptanceRunName`

```cpp
inline constexpr const char* kAcceptanceRunName = "TEST 24in turn back"
```

The acceptance run's name on the selector and in the log (screen-safe, within the budget).

*constant, declared at [`include/shulib/competition/acceptance_run.hpp:34`](../../include/shulib/competition/acceptance_run.hpp#L34).*

<a id="acceptancerun"></a>

## `acceptanceRun`

```cpp
[[nodiscard]] inline chassis::RoutineResult acceptanceRun(AutonContext& ctx)
```

The acceptance run (file banner): four motions in the START frame, ending on the start position facing 180 degrees. Returns the Routine's verdict.

*free function, declared at [`include/shulib/competition/acceptance_run.hpp:39`](../../include/shulib/competition/acceptance_run.hpp#L39).*

<a id="acceptancerunentry"></a>

## `acceptanceRunEntry`

```cpp
[[nodiscard]] inline AutonEntry acceptanceRunEntry(AutonKind kind, units::Time endActionAt, units::Time hardStopAt)
```

The registry row for the acceptance run: its name, `kind`, the routine, a RELATIVE start and the schedule the CALLER states (never defaulted — the COMP brief's C6). The end action is the default (brake). Register it like any other entry.

*free function, declared at [`include/shulib/competition/acceptance_run.hpp:51`](../../include/shulib/competition/acceptance_run.hpp#L51).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 22 lines</summary>

```text

 competition::acceptanceRun — the LIBRARY's acceptance run: the one routine shulib ships
 (chunk COMP-a, 2026-09-26; the COMP brief's ruling C3 and amendment B18).

 ── What it is, and what it is not ──────────────────────────────────────────────────
 It is NOT season content. It is the test a team runs to see the competition program, the
 selector, the guard and the motion stack work end to end on THEIR robot before trusting any
 auton of their own: drive 24 in forward, turn 90 degrees left, then drive back to the start —
 on a tank, "back to the start" means FACING it first (a tank cannot drive sideways, and the
 library never plans a turn for you — chassis.hpp's D12), so the run is four motions:

     moveTo (24, 0) facing 0  ->  turnTo 90  ->  face (0, 0)  ->  driveTo (0, 0)

 It is written in the START FRAME: register it with StartPose::relative(), and wherever the robot
 is placed becomes (0, 0) facing 0. The EXPECTED END is the start position, facing the opposite
 way it started (180 degrees in the start frame): on the tape, back on the mark it was placed on,
 turned round. Every leg takes the robot's configured motion defaults (no number here is invented
 for it). The season's autons are the students' (the guardrail): the students' file registers
 this entry beside their own, and otherwise ships empty apart from the registration pattern.

 PROS-free, allocation-free. Host-tested on the plant (test/competition_runner_test.cpp); NOTHING
 HERE HAS RUN ON A ROBOT.
```

</details>
