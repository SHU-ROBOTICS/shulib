# Diagnostics & Observability — the extended plan (WS13)

> Companion to [`roadmap.md`](roadmap.md) and master plan §18. `roadmap.md` carries the milestone
> checkboxes; this document is the **full WS13 backlog**, with each item placed in the chunk that
> should own it and the reason it belongs there.
>
> **Why a separate document:** observability is a cross-cutting discipline, not a feature. It touches
> every chunk, and the backlog is now large enough that burying it inside milestone bullets loses the
> reasoning. Created 2026-08-06.

---

## ⚠️ The one genuinely time-sensitive constraint

**`DebugRecord`'s field set becomes expensive to change at H1, and free to change until then.**

F9 freezes the wire serialization of *exactly* that record. After the freeze, adding a field means a
version bump plus a migration path across every sink and the VexBuilder overlay. Before it, adding a
field is a one-line edit.

**Therefore: every diagnostic below that needs a new per-tick field must have that field added to the
schema before H1** — ideally at C5, the next diagnostics chunk. The *behaviour* can land whenever;
the *field* should land early. Each item below marks whether it needs schema space.

This is the only part of this document that can't be safely deferred.

---

## What exists today (A1 + A3 + C1)

~1,050 lines across `include/shulib/diag/` and the `ITelemetrySink` seam:

| Piece | What |
|---|---|
| `DebugRecord` | the per-tick snapshot — **one record, many sinks** |
| `ITelemetrySink` | `log()` (leveled, tagged) + `emit(record)` + `wantsRecord()` + lazy `emitRecord()` |
| `NullSink` / `TermSink` / `FakeTelemetrySink` | zero-cost default / human-readable terminal / recording double |
| `FaultCode` + `FaultLatch` | 10 stable append-only codes; **first**-fault latched (root cause, not cascade) |
| `FiniteGuard` | NaN/Inf caught, logged, recovered — never propagated |
| `LoopMonitor` | control-tick dt-budget overrun |
| `HealthMonitor` | sensor health (A3) |
| `OdoStallCheck` | spin-vs-motion cross-check → `ODO_STUCK` (C1) |
| `trace.hpp` | compile-time `TRACE` strip, proven by ARM `-Os` asm diff |

Already earned its keep: defeating `FiniteGuard` as a mutation **aborted fatally** (a `+∞` reached the
pose — the guard is load-bearing), and C1 as `TermSink`'s first consumer found a real defect.

---

## Already planned

| Item | Chunk |
|---|---|
| Per-motion result line (target vs final · overshoot · drift · time · exit-reason) | C5 |
| Session header (git hash, routine id, alliance, port map, battery start) | C5 |
| End-of-run summary block | C5 |
| `SdSink` — binary blackbox to `/usd/` | E1 |
| Estimator introspection (residual, Mahalanobis, accept/reject reason, covariance trace) | E1 |
| Latched brownout marker + graceful-end contract | E1 |
| `SHUL/2` wire protocol (**F9 freeze**) | H1 |
| Run record/replay | H2 |
| On-brain live PID/FF tuner | H3 |

---

## New — added 2026-08-06

### For C5 (with the motion layer, next diagnostics chunk)

**D-1. Per-subsystem log levels.** Levels are global today. Turning `[MOT]` up to `DEBUG` while holding
`[LOC]` at `WARN` is the single most-used debugging move in practice, and its absence forces a rebuild
to chase anything.
*Schema: no.*

**D-2. Per-channel rate limiting / throttling.** §18.3 already calls for it; A1 deferred it here. A
high-rate channel must not drown the terminal or push the loop over budget. Needs a **dropped-count**
so throttling is visible rather than silent — a silent drop reads as "nothing happened."
*Schema: yes — a dropped-record counter.*

**D-3. Tick-time attribution.** `LoopMonitor` detects an overrun but cannot say **who** consumed the
budget. As C2/C3/C4 add loop work this becomes the question you actually need answered: localization
4 ms, motion 2 ms, sinks 1 ms. Cheap to add now, and it turns "the loop is slow" into a name.
*Schema: yes — per-subsystem tick-time slots. **Add the fields at C5 even if attribution lands later.***

**D-4. Controller-screen fault display.** The V5 controller has a small LCD. Showing the latched fault
code and a one-word state there means a student at the field, with no laptop, can tell *why* the robot
stopped. Very cheap; disproportionately useful on a competition day.
*Schema: no.*

**D-5. Physical-plausibility invariants.** Extend `FiniteGuard`'s log-and-recover posture beyond
finiteness: per-tick pose delta within a physical maximum, commanded velocity within the drivetrain's
capability, wheel commands consistent with the commanded twist. Each violation is a fault, not a
crash. A3 proved this class of guard catches real defects.
*Schema: no (reuses `FaultCode`).*

### For E1 (with `SdSink`)

**D-6. Flight recorder — the highest-value item in this document.** Keep the last N ticks in a RAM ring
buffer at all times, and **dump only when a fault fires**. Competition builds cannot afford
always-on logging, but when something breaks, the 200 ticks *before* the fault are exactly what you
need and exactly what you don't have. Pairs naturally with `SdSink` (it becomes the dump target) and
with `FaultLatch` (the first fault becomes the trigger).
*Schema: no — it stores existing records.*

**D-7. Fault-triggered dump + post-run auto-triage.** On fault, flush the flight recorder and emit a
short triage block: which fault, at what tick, what the state was, what preceded it. The end-of-run
summary answers "how did it go"; this answers "why did it break."
*Schema: no.*

**D-8. Routine-level watchdog.** C1/C2 bound each *motion*. Nothing yet bounds a whole *routine*.
Composes with F2's guaranteed-park guard — the park must fire even if the routine as a whole wedges,
not merely if one motion does.
*Schema: no.*

### For H2 (with record/replay)

**D-9. Replay-as-regression-test.** A recorded real run becomes a permanent test fixture: feed the
recorded sensor stream back through the estimator offline and assert the result. This converts every
field session into durable tests, and is the **only** way to get real-hardware coverage into CI.
Arguably the highest-leverage item once hardware exists.
*Schema: no.*

**D-10. A/B trace diff.** Run the same routine twice, diff the traces, report the first tick where they
diverge and by how much. The standard tool for chasing intermittent behaviour, which is precisely the
class of bug that costs matches.
*Schema: no.*

**D-11. Error-budget attribution as a shipped tool.** C1's three-way regression — error vs. move count,
vs. distance travelled, vs. elapsed time — cleanly separated a chaining defect from a scale bias from
localizer drift, and validated itself against a deliberately miscalibrated twin. It was written as a
one-off test; it should be a reusable diagnostic that runs against any recorded routine.
*Schema: no.*

### Frontier

**D-12. On-brain status screen** (the existing `BrainHud`, made concrete): pose, quality, latched
fault, battery, elapsed. The no-laptop glance.
**D-13. Cloud run library / auto-tune from replays** — already on the Frontier list; depends on D-9.

---

## Placement summary

| Chunk | Items |
|---|---|
| **C5** | D-1, D-2, D-3, D-4, D-5 — **plus the schema fields for D-2 and D-3** |
| **E1** | D-6, D-7, D-8 |
| **H1** | *(F9 freeze — everything above must have its fields in by here)* |
| **H2** | D-9, D-10, D-11 |
| **Frontier** | D-12, D-13 |

**Nothing here needs building right now.** The only thing that cannot wait is **reserving schema space
in `DebugRecord` at C5** for D-2's dropped-count and D-3's tick-time slots — because after H1 those
become a version bump instead of a one-line edit.

---

## Principles these all follow

1. **One record, many sinks.** New diagnostics extend `DebugRecord`; sinks only ever *format*. Bench,
   terminal, field, and sim traces stay directly comparable.
2. **Zero cost in a competition build.** `NullSink` default and `wantsRecord()` so the record isn't
   even built. Anything added here honours that.
3. **Structured fields, not essays.** The legacy code's prose diagnosis trees and ALL-CAPS banners are
   the anti-pattern §18 exists to prevent.
4. **Log and recover, never crash.** A diagnostic that can take down an auton is worse than no
   diagnostic.
5. **Silent degradation is a bug.** Throttling, dropping, and truncating must all be *counted and
   reported* — a silent drop reads as "nothing happened."
