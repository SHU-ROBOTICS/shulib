# Chunk C5 — per-motion results, session header, run summary

> **Phase C, chunk 5 of 7.** Predecessors: C1–C4 ✅.
> This is the chunk that makes a run **legible** — and it closes M2's "the run is legible in real
> time on the terminal" clause.

**Workstream:** WS13 (Diagnostics) · **Milestone:** M2 · **Spec:** master plan §18.3 + [`diagnostics-plan.md`](../diagnostics-plan.md)

---

## Why this chunk

A1 built the instrument; C1–C4 built the thing worth watching. Today a run emits per-tick records —
correct, but a wall of them. C5 turns that into something a human reads at a glance:

- **a session header** so you know *which binary* ran and under what conditions
- **a per-motion result line** so each motion reports how it actually went
- **a run summary** so the whole auton fits on one screen

§18.3 of the master plan carries the **exact target shape**. Match it.

It also lands the near-term items from [`diagnostics-plan.md`](../diagnostics-plan.md) — including the
one genuinely time-sensitive thing in this whole phase.

---

## ⚠️ The time-sensitive part: reserve schema space now

**`DebugRecord`'s field set is free to change until H1 and expensive after** — F9 freezes the wire
serialization of exactly that record, after which a new field means a version bump plus migration
across every sink and the VexBuilder overlay.

**Two items below need per-tick fields. Add the fields in this chunk even if the behaviour lands
later:**
- **D-2** — a dropped-record counter (throttling must never be silent)
- **D-3** — per-subsystem tick-time slots

That is the only thing here that cannot be safely deferred.

---

## What already exists

| Thing | Where | Note |
|---|---|---|
| `DebugRecord`, `TermSink`, `LogLevel` | `diag/` | the record + the formatter to extend |
| `FaultCode`, `FaultLatch` | `diag/fault.hpp` | first-fault latching for the summary |
| `LoopMonitor` | `diag/loop_monitor.hpp` | detects overrun; **cannot attribute it** — D-3 fixes that |
| `FiniteGuard` | `diag/finite_guard.hpp` | the log-and-recover posture D-5 extends |
| **`CompletedMotion`** | `motion/motion_scheduler.hpp` | **C2 staged this for you** — id / name / exit / abortFault |
| `strafeFallbackActive` | `diag/debug_record.hpp` | C3 populates it per tick — surface it |
| `ICharSink` | `hal/char_sink.hpp` | the injectable output seam |
| The facade | `chassis/chassis.hpp` | C4's routine-level entry point |

**Read first:** master plan **§18.3** (the target output), [`diagnostics-plan.md`](../diagnostics-plan.md)
(D-1…D-5), `A1-COMPLETED.md` (the cost contract and what A1 deferred here), `C2-COMPLETED.md` (§ what
`CompletedMotion` stages), `C4-COMPLETED.md` (the three contract clarifications — the settle-tail one
is result-line-relevant).

---

## Scope

### In
1. **Session header** — git build hash, routine id, alliance/side, port map, battery start. **The
   first record of every run.**
2. **Per-motion result line** — target vs final · overshoot · drift · time · exit-reason
3. **End-of-run summary block** — one screen: scored/failed, heading max & final, gating rejects,
   brownout, worst loop dt, first fault, build hash, battery start→end
4. **D-1** — per-subsystem log levels
5. **D-2** — per-channel rate limiting, with a **counted, reported** drop *(schema field)*
6. **D-3** — tick-time attribution *(schema fields)*
7. **D-4** — controller-screen fault content, **behind a seam** (see constraint 4)
8. **D-5** — physical-plausibility invariants (pose delta, velocity, command consistency)

### Out
- `SdSink` → E1 · `SHUL/2` wire + F9 freeze → H1 · flight recorder (D-6) → E1
- Estimator introspection (residuals, Mahalanobis) → E1, when correctors exist
- The actual PROS controller write → R1

---

## Design constraints

### 1. §18.3 is the spec, and it is exact
The master plan shows the literal target output — column-aligned, subsystem-tagged, never prose.
Golden-test it. **Structured fields, not essays**: the legacy code's prose diagnosis trees and
ALL-CAPS banners are the anti-pattern §18 exists to prevent.

### 2. The session header's job is reproducibility
Its point is that six months later you can tell **which binary ran**. The git build hash is the
load-bearing field — if it can't be obtained, say so loudly rather than emitting a plausible
placeholder. A wrong hash is worse than an absent one.

### 3. Numbers must be true
A result line claiming "settled, 0.1 in drift" while the robot is 6 inches off is worse than no line
at all. Cross-check the reported values **against ground truth** in tests, the way C1 measured
settled-vs-truth divergence. Report what the motion *did*, not what it *believed*.

### 4. The controller display needs a seam, not a PROS call
The core is PROS-free and CI enforces it. Build the **content and the seam**; the actual V5 controller
write is R1's glue, exactly as `ICharSink` was done. Same pattern.

### 5. Silent degradation is a bug
Throttling, dropping, truncating — all counted and reported. A silent drop reads as "nothing
happened," which is how you spend an afternoon debugging a problem that was never there.

### 6. Zero cost in a competition build — still
`NullSink` default plus `wantsRecord()` so the record isn't even built. Everything added here honours
that; measure it, don't assume it. Override `wantsRecord()` and `emit()` as a **pair**.

### 7. Don't re-derive what C2 staged
`CompletedMotion` already carries id/name/exit/abortFault. Use it. If it's missing a field the result
line needs, add it **there** (rule 4), don't shadow it.

---

## Test requirements

Every test names the bug it would catch.

- **Golden output end-to-end** — session header → ticks → per-motion lines → summary, byte-exact
  against the §18.3 shape; plus the ugly cases (NaN, ±inf, huge magnitudes, long names, empty tags)
- **Numbers are true** — per-motion reported values cross-checked against ground truth across a swept
  routine, on all three drivetrains
- **Session header** — carries the real build hash; a missing hash is loud, never silently plausible
- **Summary correctness** — first fault is the *first*, worst loop dt is the worst, counts are right;
  verify against a run with injected faults
- **D-1** — a per-subsystem level actually filters that subsystem and no other
- **D-2** — under flood, drops occur, are **counted**, and the count is reported; nothing silently vanishes
- **D-3** — attributed tick times sum to the measured total within tolerance; a deliberately slow
  subsystem is correctly named
- **D-5** — each invariant fires on an injected violation, raises a fault, and **recovers** (no crash)
- **Cost** — with `NullSink`, no record is built and no formatting work happens
- **Hostile** — a full routine under A3 composed hostility produces a coherent, complete report

**Mutations — go well past four.** The last four chunks each found something a normal test missed:
two green holes (C1), a vacuous test (C2), a structurally-uncatchable mutation (C3), two green
survivors including *fault observables dark in teleop* (C4). **A green mutation or a vacuous test is
the most valuable thing you can find.** Gate the runner on build success — C4 nearly misread a
non-compiling mutation as green off a stale binary.

---

## Definition of Done

- [ ] Session header is the first record of every run and carries a real build hash
- [ ] Per-motion result line matches §18.3; its numbers verified against ground truth
- [ ] Run summary fits one screen and is correct under injected faults
- [ ] D-1 through D-5 implemented; **D-2 and D-3's schema fields present in `DebugRecord`**
- [ ] Controller-display content built behind a seam; PROS glue explicitly deferred to R1
- [ ] Throttling/dropping is counted and reported — never silent
- [ ] `NullSink` cost unchanged, measured
- [ ] **M2's "the run is legible in real time on the terminal" clause closes**
- [ ] Suite green under strict `-Werror`; both guards pass; ARM gate passes

---

## Live progress log — required

`docs/planning/chunks/C5-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`docs/planning/chunks/C5-COMPLETED.md`** at C1–C4 depth (570/605/654/600 lines), with
the **"What we now know for certain, and what we do not"** section.

Also **update [`diagnostics-plan.md`](../diagnostics-plan.md)** — mark D-1…D-5 delivered, and record
which schema fields were reserved for later items, so H1 inherits an accurate picture of what F9 is
about to freeze.

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **A result line that lies is worse than no result line.** Verify against truth.
- **A silent drop is a bug**, not an optimization.
- **Don't emit a plausible-looking fake build hash.** Absent and loud beats wrong and quiet.
- **Don't call PROS from core.** Seam now, glue at R1.
- **Reserve D-2 and D-3's schema fields even if the behaviour slips** — after H1 they cost a migration.
