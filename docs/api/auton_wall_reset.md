<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/auton_wall_reset.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `auton_wall_reset.hpp`

competition::resetFromWall — the wall reset's EVERYDAY call, over the auton's context (chunk L6-A2, 2026-11-03; the L6 brief's D4 (a) and its §1.12 — the team lead's answers of 2026-10-22, and his ruling of 2026-11-03 that the wall reset be callable from a…

This header declares **1** free function.

Extracted from [`include/shulib/competition/auton_wall_reset.hpp`](../../include/shulib/competition/auton_wall_reset.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`resetFromWall`](#resetfromwall) — *free function*

<a id="resetfromwall"></a>

## `resetFromWall`

```cpp
[[nodiscard]] inline localization::WallResetReport resetFromWall(AutonContext& ctx, const localization::WallResetRequest& request)
```

THE EVERYDAY CALL (file banner): re-seat ONE axis of the auton's estimate against `request.wall` from the robot's own `request.sensor` side sensor — refused, reading nothing, when the auton's start is not a FIELD start (`not a field start`, the mode named) or the robot declares no wall sensors (`no sensor`); otherwise every check of the explicit call (chassis/wall_reset_call.hpp), its window sampled through `ctx.guard.waitFor` so a deadline or a disable mid-window CUTS it (`cut`, nothing seated, no lateness). Every outcome is filed in the robot's wall-sensor log; ONE `[WALL]` line goes to `ctx.telemetry`. Blocking, at most `windowMax` plus a tick. Returns the report — a VALUE; `continues()` is what a `then()` returns: SEATED, or REFUSED and not `required`. In a MIRRORED auton (MIRROR1-A, the file banner) the wall is its image and the sensor the other side's, and the line says so. PRECONDITION: `ctx.guard` is running (an AutonContext lives for one guarded run).

*free function, declared at [`include/shulib/competition/auton_wall_reset.hpp:125`](../../include/shulib/competition/auton_wall_reset.hpp#L125).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 61 lines, click to expand</summary>

```text

 competition::resetFromWall — the wall reset's EVERYDAY call, over the auton's context (chunk
 L6-A2, 2026-11-03; the L6 brief's D4 (a) and its §1.12 — the team lead's answers of 2026-10-22, and
 his ruling of 2026-11-03 that the wall reset be callable from a code auton before the first
 competition).

 ── The form every team auton uses ───────────────────────────────────────────────────────────
     r.then([&] { return competition::resetFromWall(ctx,
                      {.wall = localization::override2026::kLeftWall,
                       .sensor = localization::WallSide::Left,
                       .maxCorrection = 3_in}).continues(); }, "left wall x");
 It re-seats ONE axis of the estimate against a field wall from the robot's own side distance
 sensor — the robot file's `walls` block, built by robot::Robot — exactly as the explicit call
 (chassis/wall_reset_call.hpp) does over handles, and through the SAME sampler, so no gate is written
 twice. It sits in competition/ because its argument does: the auton's context carries three things
 a Chassis cannot see — the ENTRY (so the start mode), the RUN GUARD (so a deadline-aware wait) and
 the auton's own telemetry.

 ── In order ─────────────────────────────────────────────────────────────────────────────────
   1. THE START. Not a FIELD start (`ctx.entry.effectiveStart()`: RELATIVE, or UNSET — a route whose
      Start states no heading) → REFUSED `not a field start`, its line naming the mode, before
      anything is read. The season's walls and every keep-out are FIELD coordinates; under a
      RELATIVE start the robot's place relative to every field element is unknown, so no keep-out
      could be trusted — an author's own wall line included — and a reset would seat a field number
      into a robot frame. A wall reset needs a FIELD start.
   2. THE SENSOR. The robot's wall-sensor slot through `ctx.chassis.deps().ctx->wallSensors()` (the
      scheduler's shadow context carries it): none → REFUSED `no sensor` (this robot file declares
      no wall sensors — and the host test plant's tester rig, which builds no Robot, has none); the
      named side, its mount and the robot's limits.
   3. EVERYTHING ELSE exactly as the explicit form (the sensor present, the mount measured, the
      estimate live, no motion armed, the request sane, square, the keep-outs, the beam height, then
      the window and its verdict) — EXCEPT THE WAIT: the window is sampled through the run guard's
      deadline-aware `waitFor`, so a deadline or a disable that lands mid-window CUTS it at once —
      REFUSED `cut`, nothing seated — instead of running out a frozen wait up to `windowMax` late.
   4. THE OUTCOME, every one, filed in the slot's log (seated and refused counts and the last
      report — what the Auton Tester will read, from L6-B), and ONE `[WALL]` line through
      `ctx.telemetry`.
 A refusal is a VALUE: the estimate exactly as it was, and `continues()` true unless the request
 says `required` (then a `then()` returning it stops the chain). It commands nothing to the drive:
 stop first. No sensor ever seats the heading.

 ── A MIRRORED auton (chunk MIRROR1-A, 2026-11-13; competition/field_mirror.hpp) ──────────────────
 The route is written in robot one's numbers; the reset works BELOW the Chassis's mirror (it reads the
 localizer), so it swaps for itself — after rule 1 (a FIELD start), before everything else: the
 request's wall becomes its IMAGE under the entry's FieldMirror (a season wall: the season wall at the
 reflected line, with its real name and keep-outs — robot one's red wall is robot two's right wall; an
 own line: reflected geometrically) and its sensor the OTHER side's (LEFT <-> RIGHT); maxCorrection,
 required and name are sizes and labels and stay. The ONE [WALL] line then names the REAL wall and
 sensor, with the mirrored note INSIDE the sensor's parenthesis, near the line's start where the
 terminal's 200-byte cut cannot reach it:
     right wall x SEATED (LEFT sensor, mirrored from red wall RIGHT): 54.70 -> 55.69 (moved 0.98, ...
 Robot two then reads the side sensor robot one's route NEVER uses: its mount must be measured too (an
 unmeasured one refuses MOUNT UNSET), and if the two sit at different distances ahead of the centre,
 robot two's beam lands elsewhere along the wall (A4 register HA-259).

 ── A programming error is loud ──────────────────────────────────────────────────────────────
 An AutonContext lives for exactly one guarded run (competition::runAuton builds it inside the
 guard's run), so a call with the guard NOT running is a program error — a PRECONDITION, not a
 refusal (the guard's own waits say the same).

 PROS-free, sim-free, allocation-free. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
