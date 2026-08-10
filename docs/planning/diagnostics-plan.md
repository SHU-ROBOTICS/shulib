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

### ✅ DISCHARGED at C5 (2026-08-10) — what H1/F9 inherits

The schema space was reserved, and the fields are wire-pinned by test
(`test/debug_record_test.cpp`). **What F9 is about to freeze, exactly:**

| Field | Type | Producer today | Reserved for later |
|---|---|---|---|
| `droppedRecords` | `uint32` | `RateLimitedSink` stamps cumulative drops onto every forwarded record (D-2, LIVE) | — |
| `droppedLines` | `uint32` | same (D-2, LIVE) | — |
| `tickPhase[0]` Localization | `Time` | scheduler attribution, when an attribution clock is configured (D-3, LIVE; one-tick lag documented on the field) | — |
| `tickPhase[1]` Motion | `Time` | same (D-3, LIVE) | — |
| `tickPhase[2]` Health | `Time` | **0 — RESERVED** | E1+ (separable health timing) |
| `tickPhase[3]` Telemetry | `Time` | **0 — RESERVED** | E1+ (separable sink timing) |
| `tickPhase[4]` Scheduler | `Time` | **0 — RESERVED** | E1+ (scheduler bookkeeping) |
| `tickPhase[5]` User | `Time` | **0 — RESERVED** | G2 markers / mechanisms |
| `tickPhase[6..7]` | `Time` | **0 — SPARE** | unassigned pre-freeze capacity: a new phase is a vocabulary append, never a wire reshape |

The `TickPhase` index vocabulary (values 0–5 defined, 6–7 spare; `kTickPhaseSlots = 8`) lives in
`debug_record.hpp` beside `GateReason`, explicit-valued and append-only. `FaultCode` gained
`Implausible = 10` (append-only) for D-5. **Also on the seam since C5** (not per-tick, but F9's
neighbours): `ITelemetrySink::summarize(RunSummary)` — a second record type (`RunSummary`, value
semantics, bounded strings) that E1's SdSink and H1's wire may serialize; and the §18.4 boundary
vocabulary `diag::MotionOutcome` (0–4, wire-pinned). H1 should freeze `DebugRecord`, `GateReason`,
`TickPhase`, `FaultCode`, and decide whether SHUL/2 v1 also carries `RunSummary`/`MotionOutcome` or
defers them to v2 — both are already wire-stable either way.

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

### For C5 (with the motion layer, next diagnostics chunk) — ✅ ALL FIVE DELIVERED at C5 (2026-08-10)

**D-1. ✅ DELIVERED. Per-subsystem log levels.** Levels are global today. Turning `[MOT]` up to
`DEBUG` while holding `[LOC]` at `WARN` is the single most-used debugging move in practice, and its
absence forces a rebuild to chase anything.
*Schema: no.*
*As built:* `diag::LevelFilterSink` — a decorator with a global level + 16 per-tag overrides;
filters the log() channel only (records/summaries are data, not chatter; filtering is explicit
config, NOT counted as a drop — the D-2 distinction, documented in-header). `test/level_filter_test.cpp`.

**D-2. ✅ DELIVERED. Per-channel rate limiting / throttling.** §18.3 already calls for it; A1
deferred it here. A high-rate channel must not drown the terminal or push the loop over budget.
Needs a **dropped-count** so throttling is visible rather than silent — a silent drop reads as
"nothing happened."
*Schema: yes — a dropped-record counter.* → **fields `droppedRecords`/`droppedLines` in the record
(see the discharge table above).**
*As built:* `diag::RateLimitedSink` — per-tag token buckets for Info/Debug/Trace lines + a record
bucket; **Error/Warn and summarize() are exempt by contract**; drops are counted (accessors),
stamped onto every forwarded record, announced with one Warn notice per episode end, and reported in
the run summary. `test/rate_limit_test.cpp` + the e2e flood case.

**D-3. ✅ DELIVERED. Tick-time attribution.** `LoopMonitor` detects an overrun but cannot say
**who** consumed the budget. As C2/C3/C4 add loop work this becomes the question you actually need
answered: localization 4 ms, motion 2 ms, sinks 1 ms. Cheap to add now, and it turns "the loop is
slow" into a name.
*Schema: yes — per-subsystem tick-time slots.* → **`tickPhase[8]` + the `TickPhase` vocabulary (see
the discharge table above — Localization/Motion live; Health/Telemetry/Scheduler/User reserved).**
*As built:* `diag::TickAttribution` (RAII phase scopes on a SEPARATE injected attribution clock —
the sim clock doesn't advance intra-tick; nullptr in `MotionSchedulerConfig` = off = zero cost);
the scheduler brackets Localization and Motion, stamps the last COMPLETED tick's breakdown onto
records (one-tick lag, documented on the field), and the LOOP_OVERRUN path logs
`overrun attribution: loc …ms · mot …ms · other …ms (worst mot)` — the overrun now has a NAME.
`test/tick_attribution_test.cpp` + the e2e attribution case.

**D-4. ✅ DELIVERED (content + seam; PROS glue → R1, as planned).** Controller-screen fault display.
The V5 controller has a small LCD. Showing the latched fault code and a one-word state there means a
student at the field, with no laptop, can tell *why* the robot stopped.
*Schema: no.*
*As built:* `hal::ILineDisplay` (3×19, HA-57) + `hal::fake::FakeLineDisplay` +
`diag::ControllerFaultDisplay` (OK/FAULT + run clock; the FIRST fault by name; battery + count;
rewrites only changed rows — write discipline pinned by write-count test). The
`pros::Controller::set_text` adapter is R1's, behind the seam. `test/controller_display_test.cpp`.

**D-5. ✅ DELIVERED. Physical-plausibility invariants.** Extend `FiniteGuard`'s log-and-recover
posture beyond finiteness: per-tick pose delta within a physical maximum, commanded velocity within
the drivetrain's capability, wheel commands consistent with the commanded twist. Each violation is a
fault, not a crash. A3 proved this class of guard catches real defects.
*Schema: no (reuses `FaultCode`).* → **`FaultCode::Implausible = 10` appended (append-only rule).**
*As built:* `diag::PoseDeltaGuard` (scheduler-run, episode-gated, dt-scaled, NOT judged during the
boot window — the estimate isn't a physical trajectory until it exists; envelope HA-56) +
`commandWithinCapability`/`recoverWheelVoltage` wired into the ONE command pipeline as a
self-audit (a NaN/over-ceiling volt never reaches a motor). Honest scope: the pose-delta fault is
ADVISORY (never rewrites the pose — principle 4); "wheel commands consistent with the commanded
twist" is implemented as finite-and-within-battery-ceiling per wheel, not a full inverse-kinematics
consistency proof (that needs E-phase estimator introspection). `test/plausibility_test.cpp` incl.
the hostile-pipeline wiring case (born from a green mutation — C5-COMPLETED §mutations).

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

### Legacy evidence, mined at C6 (2026-08-10) — requirements input, not new items

The C6 salvage audit read the legacy diagnostics before C7 deletes them. Two capabilities the old
code had (in essay form — the exact anti-pattern principle 3 bans) are worth naming as *evidence*
for items already on this list, plus one genuinely uncovered detector:

- **Per-tracking-wheel stuck identification** (`legacy odometry.cpp`): mutual comparison of the
  three tracking wheels — zero-delta-while-others-move (500 ms threshold) plus long-run L/R travel
  ratio (< 50%) — told the team **which** wheel died, not just *that* odometry was implausible.
  v2 today: `OdoStallCheck` (C1) detects "odometry stuck vs drive spinning" at motion level, and
  its own header defers the **estimator-side detector to E-phase**. When that E-phase detector is
  built, the legacy mutual-comparison approach is the proven requirements input: it needs
  *which-wheel* attribution, structured (`fault=ODO_STUCK wheel=L`), not a banner. **Owner: the
  E-phase estimator-side detector named in `motion/odo_stall_check.hpp`.**
- **Drive-side veer/imbalance triage** (`legacy main.cpp` [VEER]/[OPCTL]): L/R average-velocity
  imbalance % + per-side temperature deltas, with a diagnosis matrix (slower AND hotter ⇒ tight
  gearbox; balanced motors but veering ⇒ odom-wheel contact). The team lived this failure hard
  enough to build it twice. v2 today: nothing equivalent — `HealthMonitor` covers sensor/power
  pathology, not mechanical asymmetry. This is **bench/pit tooling, not per-tick competition
  diagnostics**: it belongs beside D-11 as a replay-time analysis (H2) or an R-phase bench routine,
  fed by the per-wheel fields `DebugRecord` already carries. Not scheduled here — recorded so the
  need isn't rediscovered from scratch on a field day.
- Corroboration: the legacy logger chunked its output to ~900-byte packets with inter-chunk delays
  — they hit V5 serial backpressure in practice, which is principle 5's "counted and reported"
  drop-budget argument made real (E1's byte/tick budget + drop counters).

---

## Placement summary

| Chunk | Items |
|---|---|
| **C5** | ✅ DONE (2026-08-10): D-1, D-2, D-3, D-4, D-5 — **and the schema fields for D-2 and D-3 are reserved** (discharge table at the top) |
| **E1** | D-6, D-7, D-8 |
| **H1** | *(F9 freeze — everything above HAS its fields in as of C5; the discharge table is the inventory to freeze)* |
| **H2** | D-9, D-10, D-11 |
| **Frontier** | D-12, D-13 |

**The time-sensitive constraint is DISCHARGED.** The C5 row above was the only thing that could not
wait; the reserved fields are wire-pinned by `test/debug_record_test.cpp`, and the "Already planned"
C5 rows (per-motion result line, session header, end-of-run summary) shipped in the same chunk
(`diag/motion_result.hpp`, `diag/session_info.hpp`, `diag/run_summary.hpp` + `TermSink::summarize`,
glued by `motion/run_reporter.hpp`). See `chunks/C5-COMPLETED.md`.

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
