# Chunk A1 — `DebugRecord` + `TermSink` + fault discipline

> **Phase A, chunk 1 of 39.** See [`build-order.md`](../build-order.md) for why this is first.
> This file is also the **template** for every later chunk brief.

**Workstream:** WS13 (Diagnostics & observability) · **Milestone:** M2 · **Spec:** master plan §18

---

## Why this chunk is first

Every chunk after this one is debugged *through* the instrument built here — including A2's plant,
which is a simulator whose output you need to be able to read. Building the motion layer first and
instrumenting afterwards means debugging the hardest code in the project blind, then retrofitting
observability into loops already shaped without it.

---

## What already exists (build on it, don't re-invent)

| Thing | Where | Note |
|---|---|---|
| `ITelemetrySink` seam | `include/shulib/hal/telemetry_sink.hpp` | **Read its header comment first — it pre-specifies how this chunk must extend it.** |
| `LogLevel` enum | same file | `Error/Warn/Info/Debug/Trace`, already defined |
| `NullSink` | `include/shulib/hal/null_sink.hpp` | zero-cost competition default |
| `FakeTelemetrySink` | `include/shulib/hal/fake/fake_telemetry_sink.hpp` | recording double, bounds-checked |
| `SHULIB_PRECONDITION` | `include/shulib/core/check.hpp` | **carries a `TODO(§18.4)` this chunk resolves** |
| Units & math types | `units/`, `math/` | use the typed quantities, not bare doubles, wherever a field has a dimension |
| Existing tests | `test/telemetry_sink_test.cpp` | shows the conventions to match |

**Conventions:** header-only under `include/shulib/`, `#pragma once`, a header comment explaining
*why*, namespace `shulib::<area>`. Tests are `test/<name>_test.cpp` — CMake globs them, so no build
file edit is needed. Strict flags: `-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion
-Wsign-conversion -Wdouble-promotion`.

**Hard constraint:** everything lands in the PROS-free core. No `#include <pros/...>` — CI fails on it.

---

## Scope

### In
1. **`DebugRecord`** — the per-tick snapshot schema (§18.2)
2. **`ITelemetrySink::emit(const DebugRecord&)`** — added *additively* (see constraints)
3. **`TermSink`** — the human-readable, column-aligned terminal formatter (§18.3)
4. **Fault discipline** (§18.4) — stable numeric fault-code enum, latched first-fault, NaN/Inf
   invariant asserts that log-and-recover, loop-overrun / tick-timing detection
5. **Compile-time `TRACE` strip** — provably zero-cost in a competition build

### Out — do not build these here
- **Per-motion result line, session header, run-summary block** → chunk **C5**. They need motion data
  that doesn't exist yet. Build the record and the formatter; C5 populates and adds those.
- `SdSink` → E1 · `Shul2Sink` / the `SHUL/2` wire → H1 · on-brain HUD → Frontier
- Any `hal/pros` glue → R1

### Explicitly rejected
**Do not port `include/legacy/shulib/logger.hpp` or `src/legacy/shulib/logger.cpp`.** The roadmap
phrases this as "fix the three inherited bugs before building on it," but the clean-room principle
supersedes: **re-derive, don't copy** — the same call that turned up a real legacy bug when `arcStep`
was rewritten. Study the three defects as *failure modes to design against*, then write fresh:

| Legacy defect | Location | Design against it |
|---|---|---|
| `escapeJSONString` defined but never applied | `logger.hpp:141` | Any escaping/sanitizing path must be *unavoidable* by construction, not a helper a caller can forget |
| Dead `sendDebugMessages` | `logger.hpp:172`, called `logger.cpp:17` | No dead paths; if it isn't reachable and tested, it doesn't ship |
| Racing flush | `logger.cpp` | Define the concurrency contract explicitly and document it in the header |

Also design against the anti-patterns §18 names directly: a ~250-line hot-loop wheel-health block,
prose diagnosis trees, ALL-CAPS banners, raw `std::cout` from inside motion loops. **Structured
fields, not essays.**

---

## Design constraints (the load-bearing ones)

### 1. `emit()` must be additive — non-pure virtual with a default no-op body
`telemetry_sink.hpp` already specifies this:

> *The per-tick DebugRecord emit (§18.2) is added behind this SAME seam at M2 … as a NON-pure virtual
> with a default (no-op) body — so every existing sink keeps compiling. THAT is what makes it additive
> and F9-versioned, never a break (a pure-virtual addition would break all implementers).*

A pure virtual here breaks `NullSink`, `FakeTelemetrySink`, and every future sink. **Non-negotiable.**

### 2. A null sink must cost nothing — including record *population*
The subtle failure: if the tick loop always builds a `DebugRecord` and hands it to `NullSink`, you pay
full construction cost to throw it away. Competition builds must not pay for diagnostics they discard.

Provide a way to skip population entirely when nothing consumes it (a cheap `wantsRecord()` query, a
compile-time policy, or equivalent). **Record the mechanism you chose and why in the header** — this is
a real design decision with alternatives, so it belongs in the chunk's decision log.

### 3. `DebugRecord` must carry fields for systems that don't exist yet
Per §18.2, define the **full** field set now and leave the not-yet-built ones unpopulated:

`t`, `dt`, target and measured `Pose2d`, per-axis error, commanded `(vx,vy,ω)`, per-wheel voltage and
current, IMU yaw + yaw-rate, active command id/state, **dead-reckon flag**, **quality flag**,
covariance trace / filter trust weights, gating `(residual, Mahalanobis, reason)`, applied-correction
`(dx,dy,dθ)` + `clampedThisTick` (this audits the never-snap invariant, §13 #4),
`strafeFallbackActive` (§13 #5), **fault code**, battery voltage and current.

**Why it matters that this is complete now:** F9 freezes the wire serialization of *exactly this
record* at H1. Reshaping the schema later breaks every sink and the VexBuilder overlay at once. Getting
the field set right is more important than getting any single formatter right.

Use typed units (`units::Voltage`, `Angle`, …) wherever a field has a dimension.

### 4. `TermSink` output must be assertable in a host test
Do **not** hard-code `std::cout`. Take an injectable character sink so tests can assert on exact
output — otherwise "readable, column-aligned" is an untestable claim. This mirrors the injected-clock
pattern already used across `control/`.

### 5. Resolve `check.hpp`'s `TODO(§18.4)`
It currently throws on a precondition violation, with a TODO noting that on-robot it should route to
the fault log plus a safe fallback so one bad reading degrades gracefully instead of aborting the
auton. **The call sites must not change — only the policy.** Build that policy seam here; host/test
builds keep throwing so contract breaches still turn tests red.

### 6. Faults log and recover — they never crash
A NaN pose, a sensor pathology, or a loop overrun raises a fault code and continues on a safe
fallback. Latch the **first** fault distinctly from the cascade that follows: root cause is what you
need at 2am, and it is the first one, not the loudest.

---

## Test requirements

Per the roadmap's testing discipline — **tests must try to break the code**, not confirm it works.

**Required:**
- **Zero-cost proof for `TRACE`** — demonstrate the call is compiled out in a competition build, not
  merely skipped at runtime.
- **Null-sink cost proof** — a `DebugRecord` is not populated when nothing consumes it.
- **Formatter golden output** — exact expected `TermSink` text for a synthetic tick, including column
  alignment. Also cover the ugly cases: NaN, ±inf, very large and very small magnitudes, an empty
  subsystem tag, and a message long enough to threaten the column layout.
- **First-fault latching** — inject a cascade; assert the *first* fault is retained, not the last.
- **NaN/Inf invariants** — a NaN pose is caught, logged, and recovered from; the process survives.
- **Loop-overrun detection** — fires at the boundary; assert the exact `>=` vs `>` edge.
- **Additivity** — a sink implementing only `log()` still compiles and runs (this is the F4-safety
  test; it is the one that would catch a pure-virtual regression).
- **Fault-code stability** — pin the numeric values so a reordering of the enum is caught. These
  numbers go on the F9 wire later.

**Mutation checks (prove red, then restore) — at minimum:**
- Break first-fault latching to keep the *last* fault → the cascade test must go red
- Loosen the loop-overrun boundary comparison → the edge test must go red
- Remove the NaN guard → the invariant test must go red

Record in the chunk notes which mutations were run and that each went red.

---

## Definition of Done

- [ ] A synthetic tick stream renders the §18.3 target shape (per-tick lines + leveled messages;
      per-motion results and the summary block are C5's)
- [ ] `TRACE` is **provably** stripped at compile time in a competition build
- [ ] A deliberately injected NaN is caught, logged, and recovered from — no crash, no propagation
- [ ] `DebugRecord` carries the complete §18.2 field set
- [ ] `emit()` is non-pure with a default no-op; `NullSink` and `FakeTelemetrySink` compile untouched
- [ ] `check.hpp`'s `TODO(§18.4)` is resolved with call sites unchanged
- [ ] Full host suite green under strict `-Werror`; the v2 core still cross-compiles for ARM
- [ ] CI PROS-free guard still passes

---

## Documentation contract — all six, before A2 starts

1. **Roadmap checkboxes flipped with cited evidence** — the `DebugRecord`, `TermSink`, and fault-code
   items under M2's WS13 block. Cite file + test name + case/assertion count. `[~]` if only partly met.
2. **`roadmap.md` "you are here" updated** — A1 done, A2 next.
3. **Design notes in the headers** — *why*, not just *what*. Especially the null-sink cost mechanism
   (constraint 2) and the concurrency contract (replacing the legacy racing flush).
4. **Test evidence recorded** — case count, what each test would catch, which mutations went red.
5. **Decisions recorded** — at minimum: the null-sink cost mechanism, the `TRACE` strip mechanism, the
   `check.hpp` policy seam, and the clean-room rejection of `logger.hpp`.
6. **Freeze Register** — no freeze in A1. But note in the `DebugRecord` header that **F9 (H1) will
   freeze the wire serialization of this schema**, so field changes after this point carry that cost.

---

## Landmines

- **The `DebugRecord` field set is the highest-stakes decision in this chunk** — higher than any
  formatting choice. F9 serializes it verbatim. Get it complete; formatters are cheap to change later.
- **A pure-virtual `emit()` breaks every existing sink.** The header warns about exactly this.
- **Don't let diagnostics cost anything in a competition build** — that is the whole point of
  `NullSink` being the default, and it's easy to undermine by always building the record.
- **Don't build the run summary here.** It needs motion data. C5 owns it.
- **Don't port the legacy logger.** Re-derive.
