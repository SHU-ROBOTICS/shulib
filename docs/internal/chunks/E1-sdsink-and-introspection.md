# Chunk E1 — `SdSink` + estimator introspection

> **Phase E, chunk 1 of 4.** Predecessor: Phase D complete (D1–D3).
> **Ordered first in the phase for the same reason A1 was ordered first overall:** fusion is the
> hardest thing in this project to debug, and you cannot debug what you did not record.

**Workstream:** WS13 (diagnostics) · **Milestone:** M3 · **Freezes:** none (F9 is H1's)

---

## Why this chunk exists, and why it is *here*

Phase E replaces "the estimate drifts and we hope it is small" with a measured bound. Every chunk
after this one — `GpsCorrector` (E2), `AprilTagCorrector` (E3), the EKF (E4) — makes a *decision*
every tick about whether to trust a sensor fix. Those decisions are where fusion goes wrong, and
they are invisible unless something writes them down.

Two consequences drive the ordering:

**1. A field run with no laptop is currently undiagnosable.** The terminal is the primary debug
surface (A1/C5), and at a competition there is no terminal. `SdSink` is the counterpart: a binary
record on the brain's SD card that makes a run reconstructable afterwards.

**2. The `< 1.0°` claim needs certifying, not asserting.** Per-correction residual, Mahalanobis
distance, accept/reject reason and covariance trace are the quantities that *prove* the accuracy
target. They are the difference between "we measured it" and "it looked fine."

---

## What already exists — read these before designing anything

| Thing | Where | Why it matters to you |
|---|---|---|
| **The estimator record fields — ALREADY DECLARED, currently unpopulated** | `diag/debug_record.hpp` | `covarianceTrace`, `gateResidualX/Y/Heading`, `gateMahalanobis`, `gateReason`, `correctionDx/Dy` exist, each tagged `— E2/E3/E4`. **You are not adding these fields.** |
| `GateReason` enum (explicit-valued, append-only) | `diag/debug_record.hpp:45` | The accept/reject vocabulary already exists |
| `ICharSink` — the diagnostics output seam | `hal/char_sink.hpp` | **TEXT and line-oriented**: "one write() call carries one complete line". See tension 2. |
| `TermSink`, `RateLimitedSink`, `LevelFilterSink` | `diag/` | The sink idiom, the drop-counter pattern (`droppedRecords`/`droppedLines`) |
| `RunSummary`, `SessionInfo`, build hash | `diag/run_summary.hpp`, `session_info.hpp`, `build_info.hpp` | The provenance record already exists; do not re-invent it |
| `FaultLatch` — first-fault capture | `diag/fault.hpp` | D-6's dump trigger |
| The `IPoseSource`/`ICorrector`/`IFusionPolicy` seam | `localization/` | Built EKF-ready at M2. **Fill it in; do not reshape it.** |
| A1's cost contract (`emitRecord` laziness) | `hal/telemetry_sink.hpp` | A disabled sink must cost ~nothing |

**Read first:** `diagnostics-plan.md` §"For E1 (with `SdSink`)" (**D-6, D-7, D-8**) and §18.4's
boundary note; `A1-COMPLETED.md` (the sink contract and the cost contract); `C5-COMPLETED.md`
(the drop-counter and reserved-schema work you are extending); `debug_record.hpp` in full.

---

## Three tensions the source documents leave unresolved — resolve them explicitly

These were found while writing this brief. Each gets a ruling in the completion record.

### T1 — "double-buffered **off-task** writes" contradicts a standing decision

`build-order.md` specifies off-task writes. **C2/C4 decided there is no background task**
(C4 §2 row 5, §5 D3): *"caller-paced stays; NO background task — a task would be the tree's first
two-task design, unbuildable PROS-free, and would end host determinism."*

You cannot have both. Resolve it, and prefer keeping the standing decision: buffer in RAM, flush
**synchronously at caller-chosen boundaries** (motion boundary, fault, auton end), with the byte
budget and drop-counter absorbing the cost. If you conclude a task is genuinely required, that is a
finding that reopens a decided question — say so loudly and do not implement it unilaterally.

**Why this matters beyond tidiness:** host determinism is what makes every closed-loop test in this
project reproducible from a seed. A background writer would end that.

### T2 — the only output seam is text, and a blackbox is binary

`ICharSink::write(std::string_view)` is documented as line-oriented and synchronous. A binary
blackbox is neither. Decide: a **new additive seam** (e.g. `IBlockSink` taking
`std::span<const std::byte>`), or a redefinition of `ICharSink`'s contract.

Prefer the new seam. `ICharSink` is deliberately **not** part of the frozen F4 ten, so adding a
sibling is cheap and honest; redefining a documented contract to mean something else is how a seam
stops meaning anything. R1 owns the on-robot `/usd/` adapter; E1 owns the interface and a host fake.

### T3 — the DoD asks to reconstruct gating decisions that do not exist yet

The DoD says *"every gating decision is reconstructable after the fact from the file alone."*
**At E1 there are no correctors** — E2/E3/E4 build them, and the record fields are unpopulated by
design. Taken literally the clause is vacuous today.

The honest reading, and the one to implement: E1 delivers the **recording path and the
introspection surface**, and proves it end to end with a **deliberately synthetic corrector** in
tests — one that emits known residuals, a known Mahalanobis value, and each `GateReason` in turn —
so that when E2 lands, the only new thing is real numbers. Say plainly in the completion record that
the clause is **half-closed at E1** and names E2 as the owner of the other half. Do not report the
DoD green on the strength of a test double.

---

## Scope

### In

1. **`SdSink`** — binary blackbox: versioned header, session/provenance record (reuse
   `SessionInfo` + the build hash), fixed-width per-tick records, RAM buffering with a byte budget,
   drop-to-counter back-pressure, explicit flush.
2. **The binary output seam** (T2) plus a host fake that captures bytes for byte-exact tests.
3. **A reader** — a decoder that turns a blackbox file back into records. **Non-negotiable:** a
   format nothing can read is not a record. The reader is what makes the DoD testable at all.
4. **Estimator introspection plumbing** — the path by which a corrector's residual / Mahalanobis /
   `GateReason` / covariance trace reach a record, proven with a synthetic corrector (T3).
5. **D-6, the flight recorder** — *"the highest-value item in this document"*: a RAM ring of the
   last N ticks, always on, dumped **only when a fault fires**. Competition builds cannot afford
   always-on logging; the 200 ticks *before* the fault are exactly what you need and exactly what
   you never have.
6. **D-7 — fault-triggered dump + triage block**: which fault, at what tick, what preceded it.
7. **Latched brownout marker + the graceful-end contract** — a run that dies mid-auton must leave a
   readable file, not a truncated one.

### Out

- **Real correctors** → E2 (`GpsCorrector`), E3 (`AprilTagCorrector`), E4 (EKF). E1 populates
  nothing real; it makes the path exist and proves it with a double.
- **The on-robot `/usd/` adapter** → R1. PROS-free stands; E1 ships the seam and the fake.
- **`SHUL/2` and the F9 freeze** → H1. See constraint 4.
- **D-8 (routine-level watchdog)** → it composes with F2's guaranteed-park; leave it there unless
  it falls out for free, and say which you did.
- **Replay-as-regression-test (D-9)** → H2.

### Explicitly rejected

- **A text blackbox.** Fixed-width binary is the point: bounded per-tick cost and a byte budget you
  can reason about. A CSV would blow the budget and still need a parser.
- **Always-on SD writing in the competition build.** That is what D-6 exists to avoid.
- **Inventing new `DebugRecord` fields for values that already have them.** They exist; populate
  them.

---

## Load-bearing design constraints

### 1. A format nothing can read is not a record
Ship the decoder with the encoder, and test them as a **round trip on ground truth**: encode a known
record stream, decode it, assert field-by-field equality. This is the single most important test in
the chunk. A blackbox is worthless the first time it is truly needed if it cannot be opened.

### 2. Versioned from byte zero
The header carries a format version. A blackbox outlives the code that wrote it — a file from three
weeks ago must either decode or say plainly that it cannot. **A decoder that mis-reads an old file
is worse than one that refuses it.**

### 3. Bounded cost, and honest about it when exceeded
Per-tick cost is fixed-width by construction. When the budget is exhausted, **drop and count** —
never block, never grow unboundedly, never silently lose. C5's `droppedRecords` pattern is the
precedent; follow it, and make the count visible in the run summary.

### 4. The schema is free until H1, and this chunk is where it gets expensive
`DebugRecord` may still change (F9 freezes it at H1), and C5 deliberately reserved spare capacity.
**But an on-disk format is a persistence contract the moment a file exists.** If E1's binary layout
and the future SHUL/2 wire diverge gratuitously, H1 pays. Note explicitly what H1 inherits.

### 5. Free when disabled
A1's cost contract: with the blackbox off, the hot path pays approximately nothing — no formatting,
no copying, no branch-heavy bookkeeping. Pin it; do not assert it.

### 6. The flight recorder must survive the thing that triggers it
D-6 dumps on fault. The fault may be a brownout — the condition least compatible with a long
synchronous write. Decide what "graceful end" means concretely (how much is written, in what order,
and what a truncated file looks like to the reader) and **test the truncated case**, because that is
the case that will actually occur.

### 7. Standing contracts
Injected clock; PROS-free; strict `-Werror`; both CI guards; the ARM gate; the doc gates (every new
public member documented, or the build fails); any invented constant gets an `HA-nn` entry — ring
size, byte budget and assumed SD write latency are all **invented** until R4 measures them.

---

## Test requirements

Every test names, in a comment, the bug it would catch.

- **Round trip on ground truth** (constraint 1) — encode → decode → field-by-field equality, across
  a stream containing every `GateReason`, faults, and boundary values.
- **Byte-exact golden** for the header and one full tick record, so a silent layout change is loud.
- **Version handling** — a file with an unknown version is *refused*, not misread.
- **Budget exhaustion** — drops are counted, the count is reported, nothing blocks or grows without
  bound, and the file still decodes.
- **Truncated file** (constraint 6) — a run cut mid-write decodes up to the cut and says so.
- **Flight recorder** — the ring holds the last N; on fault the dump contains the ticks *preceding*
  it; with no fault, nothing is written.
- **Cost when disabled** — pinned, not asserted.
- **Introspection path** — with a synthetic corrector, every field arrives in the decoded file with
  the value the corrector produced. This is T3's honest half of the DoD.

### Mutations

- Change a field's byte offset in the encoder only → the round trip and the golden must both go red.
- Flip the drop-counter increment → the budget test must go red.
- Make the ring overwrite the wrong end → the flight-recorder test must go red.
- Break version rejection → the unknown-version test must go red.
- **A mutation that stays GREEN is a hole — log it, close it with a test that fails alone, and give
  it a prominent place in the record.** Every chunk so far found one; D1 found two, D2 two, D3 four.
- Gate the runner on build success.

---

## Definition of Done

- [ ] `SdSink` writes a versioned, session-stamped, fixed-width binary blackbox
- [ ] **A decoder exists**, and the round trip is proven on ground truth
- [ ] Budget exhaustion drops-and-counts; the count is visible in the run summary
- [ ] Truncated and unknown-version files behave as designed, tested
- [ ] D-6 flight recorder: ring always on, dumped only on fault, preceding ticks present
- [ ] D-7 triage block emitted on fault
- [ ] Brownout marker latched; graceful-end contract defined and tested
- [ ] Introspection path proven with a synthetic corrector; **T3's honest scoping stated**
- [ ] T1 and T2 ruled explicitly, with rejected alternatives
- [ ] Invented constants registered as `HA-nn`
- [ ] Cost-when-disabled pinned
- [ ] Suite green; both guards; ARM gate; doc gates

---

## Live progress log — required

`docs/internal/chunks/E1-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`E1-COMPLETED.md`** at the depth of C1–C5 / D1–D3. Give **T1/T2/T3 their own
section** — they are decisions later chunks inherit.

Guide impact: the diagnostics chapter (11) gains the blackbox; `guide-maintenance.md`'s table names
what to update. Any new public member needs a `///` comment or the build fails.

**Do not commit. Do not push.**

---

## Landmines

- **Don't add a background task.** T1. It would end host determinism and reopen a decided question.
- **Don't redefine `ICharSink`.** T2 — add a sibling seam.
- **Don't claim the gating DoD is met.** T3 — half of it is E2's, and a test double is not evidence
  of a real gate.
- **Don't ship an encoder without a decoder.** An unreadable record is not a record.
- **Don't let the budget block the control loop.** Drop and count.
- **Don't invent `DebugRecord` fields** that already exist unpopulated.
- **Don't tune the ring size or byte budget as if they were measured.** They are guesses until R4;
  register them.
