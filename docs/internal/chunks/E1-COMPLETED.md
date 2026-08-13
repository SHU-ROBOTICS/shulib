# Chunk E1 — COMPLETED (2026-08-12)

> Completion record for [`E1-sdsink-and-introspection.md`](E1-sdsink-and-introspection.md).
> Everything below is **as actually observed** — commands run, outputs captured, mutations
> executed and watched. Changes are in the working tree, **uncommitted**, pending DoD review.
> Live log: [`E1-PROGRESS.md`](E1-PROGRESS.md).

---

## 0. The one-paragraph version

A run is now recordable without a laptop. `diag::SdSink` writes a **versioned, session-stamped,
fixed-width binary blackbox**, and — because a format nothing can read is not a record — the
**decoder ships in the same chunk** and is held to the format by byte-exact goldens rather than by
its agreement with the encoder. The default posture is diagnostics-plan **D-6**: a RAM flight
recorder that costs the card nothing until a fault fires, then writes the triage block first and
the preceding ticks after it, in that order, because the fault may be the brownout that cuts the
write short. **D-7**'s triage lands twice from one struct — into the file and onto the terminal at
run end. The estimator-introspection path is built end to end and proven with a **synthetic**
corrector; the real numbers are E2/E3/E4's, and this record does not claim otherwise. Along the
way the chunk found that **`DebugRecord::fault` had no producer anywhere in the tree** — the field
the whole flight recorder triggers on, and one the user guide has documented since C8 — and fixed
it in the layer that owns record population. Suite **752 / 936,895**; both guards, the ARM gate
(110 headers) and all four doc gates clean. **27 mutations executed, two green holes found and
closed.** E1 freezes nothing.

---

## 1. What was ruled, built, and changed

### Ruled (the three tensions get their own section, §2)

| Tension | Ruling | The rejected alternative |
|---|---|---|
| **T1** off-task writes vs "no background task" | **Caller-paced, synchronous.** No task. | A writer task — would end host determinism |
| **T2** text seam vs binary blackbox | **New sibling seam `hal::IBlockSink`.** | Redefining `ICharSink`'s line contract |
| **T3** the gating DoD cannot close at E1 | **Half-closed, and said so.** | Reporting the DoD green off a test double |

### Built

| Piece | File | Role |
|---|---|---|
| `IBlockSink` | `include/shulib/hal/block_sink.hpp` *(new, 63 L)* | The binary output seam (T2). `write(span<const byte>) -> bool`, `flush()`. Bool-valued on purpose: a dying card is the NORMAL failure of a blackbox |
| `FakeBlockSink` | `include/shulib/hal/fake/fake_block_sink.hpp` *(new, 86 L)* | Recording device + `setCapacity()` — models the card that accepts a prefix and then dies, which is how the truncated case gets tested |
| The format, v1 | `include/shulib/diag/blackbox_format.hpp` *(new, 759 L)* | Magic/version/header, typed frames, the 428-byte tick record, encode + decode for every frame kind, bounds-latching `ByteWriter`/`ByteReader` |
| `BlackboxReader` | `include/shulib/diag/blackbox_reader.hpp` *(new, 198 L)* | **The decoder.** Refuses what it cannot read, skips what it does not know, never throws, reports truncation as a result |
| `SdSink` | `include/shulib/diag/sd_sink.hpp` *(new, 496 L)* | The sink: flight ring, byte budget with drop-and-count, fault dump, brownout latch, graceful end |
| Triage renderer | `include/shulib/diag/triage.hpp` *(new, 83 L)* | D-7 on the terminal, from the same `TriageInfo` the file carries |
| `GateAudit` | `include/shulib/localization/correction.hpp` *(modified)* | The introspection carrier: residuals, Mahalanobis, trace, reason. Embedded in `FusionResult` and `AppliedCorrection` — both APPENDS, no signature moves |
| Audit producer | `include/shulib/localization/complementary_fusion.hpp` *(modified)* | The shipping policy now fills what it genuinely knows |
| Audit carrier | `include/shulib/localization/localizer.hpp` *(modified)* | Passes the audit through to `lastCorrection()` |
| Record stamping | `include/shulib/motion/motion_scheduler.hpp` *(modified)* | `CommandIdStampSink` stamps the estimator fields **and the tick's fault** onto every record |
| Summary field + render | `diag/run_summary.hpp`, `diag/term_sink.hpp` *(modified)* | `blackboxDropped`, shown only when non-zero |
| Report glue | `include/shulib/motion/run_reporter.hpp` *(modified)* | Optional blackbox → the drop count in the summary, and D-7's post-run triage |

**New tests:** `test/blackbox_format_test.cpp` (20 cases / 20,348 assertions),
`test/sd_sink_test.cpp` (20 / 209), `test/blackbox_introspection_test.cpp` (5 / 116).
**45 new cases, +20,679 assertions.**

### Changed, and why

- **`DebugRecord::fault` gained its first producer.** See §7.1 — this is a finding, not a feature.
- **The applied-correction fields gained a producer for every motion**, not just `MoveToPose`.
- **`RunSummary` gained one field.** Pre-F9, additive, and rendered only when non-zero so no C5
  golden moved (all of them still pass byte-for-byte).
- **`RunReporter` gained one optional constructor argument** (default `nullptr`), exactly like the
  rate limiter before it.

---

## 2. THE THREE TENSIONS

These are decisions later chunks inherit, so each gets its reasoning, not just its verdict.

### T1 — "double-buffered off-task writes" vs "there is no background task"

`build-order.md` specifies off-task writes for `SdSink`. C4 §2 row 5 / §5 D3 decided the opposite
for the tree as a whole: *"caller-paced stays; NO background task — a task would be the tree's
first two-task design, unbuildable PROS-free, and would end host determinism."* Both cannot hold.

**RULING: the standing decision stands. There is no task, no thread, and no asynchrony anywhere in
this chunk.** Records are encoded into a caller-owned RAM buffer synchronously on the caller's
task; bytes reach the device only when the caller says so — `flush()` at a motion boundary,
`close()` at auton end — with exactly one exception (§5).

**Rejected: a writer task.** Three separate reasons, any one of which is sufficient:

1. **Host determinism is load-bearing.** Every closed-loop test in this project is reproducible
   from a seed *because* there is exactly one task and the clock is injected. A background writer
   makes the interleaving of "when did bytes leave" nondeterministic, and the first symptom would
   be a sim test that fails one run in fifty — the most expensive kind of failure this project can
   buy.
2. **It is unbuildable where the core lives.** The core is PROS-free by CI guard; a task needs
   `pros::Task` (or an RTOS abstraction that does not exist and would itself be a chunk).
3. **It buys less than it looks like.** The reason to write off-task is to keep a slow SD write out
   of the control tick. Caller-paced writing achieves the same thing by *choosing where the write
   happens* — and a boundary flush is a place the loop already has slack.

**What the ruling costs, stated plainly:** a caller who never flushes will fill the buffer and lose
frames. That is why the budget **drops and counts** rather than blocking or growing, why the count
is written into the file *and* surfaced in the run summary, and why the recommended posture (the
flight recorder) stages almost nothing in the first place. The cost is real and it is visible; a
background task's cost would have been invisible and intermittent.

**If a future chunk reopens this:** the honest trigger would be R4 measuring an SD flush at, say,
80 ms, which would make even a boundary flush unaffordable. The answer then is still not a task in
`diag/` — it is fewer flush points, or a task at the *application* layer (R1/F-phase) that owns its
own determinism story. Nothing in the format or the sink would change; only who calls `flush()`.

### T2 — the only output seam was text; a blackbox is binary

`hal::ICharSink` is documented line-oriented: *"One write() call carries one complete line, so an
implementation that is atomic per call never interleaves lines."* That sentence is what makes
`TermSink`'s framing golden meaningful. A blackbox is neither text nor line-oriented — it contains
every byte value including `0x00` and `0x0A`.

**RULING: a new additive sibling seam, `hal::IBlockSink`.** `write(std::span<const std::byte>)`,
returning `bool`, plus a non-pure `flush()`. `ICharSink` is untouched.

**Rejected: widening `ICharSink`'s contract** to "bytes, sometimes lines". It keeps one seam at the
price of that seam meaning nothing — and it would silently invalidate the framing argument every
`TermSink` golden rests on, without a single test going red. `ICharSink` is deliberately **not**
part of the frozen F4 ten (it is itself an additive diagnostics seam from A1), so a sibling is
cheap and honest.

**Rejected: base64 or hex over `ICharSink`.** A text blackbox is explicitly rejected by the brief,
and it would pay 33–100% more bytes for a format that still needs a parser.

**One design detail worth inheriting:** `write()` returns `bool` and is `[[nodiscard]]`. A void
write would make "the card filled up mid-run" invisible, and a caller that cannot notice a
truncated file cannot report one. R1 owns the `/usd/` adapter behind this seam; E1 ships the
interface and the host fake.

### T3 — the DoD asks to reconstruct gating decisions that do not exist yet

The DoD says *"every gating decision is reconstructable after the fact from the file alone."* **At
E1 there are no correctors.** E2 builds `GpsCorrector`, E3 `AprilTagCorrector`, E4 the EKF. Read
literally, the clause is vacuous today: zero decisions are perfectly reconstructable.

**RULING: build the whole path, prove it with a deliberately synthetic corrector, and report the
clause HALF-CLOSED.** What exists and is proven:

- the carrier (`GateAudit`) on the fusion seam, filled by the policy, kept by the `Localizer`,
  stamped onto every record, encoded, decoded;
- `ComplementaryFusion` — the policy that actually ships today — filling what it genuinely knows:
  the verdict (`None` / `Accepted` / `RejectedInnovation`), the innovation it acted on, and its
  scalar trust weight in `covarianceTrace` (which is exactly what `debug_record.hpp` reserves that
  slot for until an EKF exists);
- a synthetic corrector + policy emitting **every** `GateReason` including the two only a real
  corrector will ever produce (`RejectedNoFix`, `RejectedHighYawRate`), so their path is proven
  before E2 needs it.

**What is NOT proven, and must not be read out of this chunk:**

- **No real gate has been evaluated.** A test double producing the number 6.5 and that number
  arriving in a file proves plumbing, not gating.
- **`gateMahalanobis` is 0 in every real path** and will be until E4. A complementary filter has no
  covariance to normalise by; writing a plausible-looking distance would be the lying-number
  failure C5 banned.
- **Nothing here certifies `< 1°`.** That claim needs real correctors and real runs.

The DoD line is reported `[~]`, and the roadmap says the same thing in public.

---

## 3. The decision docket — every other choice with a viable alternative

### D1 — Fixed-width binary, IEEE-754 **binary64 everywhere**, no narrowing

Per-wheel arrays and the eight tick-phase slots are `double` on disk, where `float` would have
saved ~35% of every record (428 → ~280 bytes).
**Rejected: narrowing to binary32.** The chunk's central promise is that a decoded record equals
the encoded one **field by field**. With narrowing that sentence quietly becomes "equals after a
documented rounding", every round-trip test grows an epsilon, and any future comparison of two runs
inherits a rounding term. Bytes are cheap on an SD card; a fuzzy record is not.
**Consequence, owned:** a 200-tick dump is ~87 KB, so it does not fit the recommended 64 KiB buffer
and writes in two device calls. Tested (`SdSink: a dump larger than the buffer flushes as it
goes`), and cheaper than 88 KB of permanently-reserved RAM.

### D2 — Typed frames with a length prefix, not a flat record array

Every frame is `{u8 type, u8 reserved, u16 payloadBytes}` + payload.
**Rejected: a bare array of fixed-size tick records** (simplest possible reader). It cannot carry
the summary, the triage block or the end stamp, and it gives a future writer no way to add a frame
kind that today's reader can survive. With the prefix, an unknown type is **skipped by its declared
length and counted** — the one property that lets a v1 decoder read a v1.1 file without ever
guessing at content it does not understand.

### D3 — Refuse, never misread — and cross-check the width

An unknown `formatVersion` is refused whole (`ReadStatus::UnsupportedVersion`), and so is a file
whose self-declared `tickRecordBytes` disagrees with this build (`LayoutMismatch`).
**Rejected: best-effort decoding of an unknown version.** A wrong number read confidently sends a
2am investigation somewhere false; a refusal sends it to the git history. The width cross-check
exists for one specific human error — changing the layout and forgetting to bump the version — and
a refused file still exposes `header().formatVersion`, so you can find the build that wrote it.

### D4 — The header is staged **lazily**; a run with nothing to say writes nothing

`open()` records provenance and takes the epoch, but the 256 header bytes are only staged when the
first frame is.
**Rejected: writing the header at `open()`.** It would put bytes on the card for every run
including the clean ones, which is precisely the always-on cost D-6 exists to avoid. `close()`
likewise writes nothing when nothing was ever staged.

### D5 — Never auto-flush (outside the dump)

When a frame does not fit, it is dropped whole and counted. The buffer is never grown and the
device is never written to behind the caller's back.
**Rejected: flushing automatically when the buffer fills.** That places an unpredictable
multi-millisecond SD write inside an arbitrary control tick — the exact cost this whole design is
arranged to avoid — and it does it at a tick nobody chose.

### D6 — `log()` is counted, not carried

The blackbox v1 does not carry the message channel; `SdSink::log()` increments a counter and the
count is written into the end frame.
**Rejected: carrying text frames.** Variable-length text in a fixed-width format is a different
problem (bounded vs truncated strings, per-line framing, budget accounting per byte rather than per
frame), and the terminal already owns text. **Rejected: silently ignoring the lines.** Principle 5:
silent degradation is a bug. Counting them means a reader can always see that N lines existed
elsewhere. *(Honest consequence: the FaultLatch's structured fault DETAIL text is not in the file.
The fault CODE, its time and the fault tick's full record are — via the record's `fault` field and
the triage frame.)*

### D7 — `blackboxDropped` is its own summary field, shown only when non-zero

**Rejected: reusing `droppedRecords`.** Those are D-2 rate-limiter drops on the terminal channel;
merging two different failures into one number is how a diagnostic starts lying.
**Rejected: always printing it, like the D-2 counters.** Those channels always exist, so "0" is a
positive health claim about them. A blackbox often is not attached at all, and
`blackbox dropped 0` on a run with no blackbox is a claim about something that never ran.

### D8 — The triage block rides `log(Error)`, not a drawn box

`RunSummary` draws a box on the character device; the triage block does not.
**Rejected: a box.** This is a fault report, and §18.4's discipline is structured, leveled,
greppable lines. Riding `log(Error)` also means a message-only sink still receives it, and D-2's
limiter is forbidden by contract from throttling it.

### D9 — Storage is caller-owned spans, not internal arrays or a template

`SdSinkStorage{ring, buffer}`, with an `SdSinkBuffers<Ticks, Bytes>` helper for the common case.
**Rejected: fixed internal arrays.** 200 records plus 64 KiB is ~150 KB — far more than a PROS task
stack holds, so a sink constructed as a local would smash the stack. Caller-owned storage makes
"put this at file scope" a visible decision instead of an invisible hazard.
**Rejected: making the sink a template on the sizes.** It works, but it puts the sizes in the type,
which makes `RunReporter`'s optional `const SdSink*` (and any future non-template holder) awkward
for no gain. A construction-time precondition covers the one size that must never be too small.

### D10 — The estimator stamp lives in the scheduler's decorator

`CommandIdStampSink` already stamps the command id and the tick-phase slots onto **every** record;
it now stamps the estimator fields and the tick's fault too.
**Rejected: stamping in each motion.** That is the arrangement that produced the hole in the first
place (only `MoveToPose` did it). One place, structurally unforgettable — the same argument that
put id stamping there at C2.
**Rejected: giving the decorator a `Localizer*` and pulling.** The scheduler pushes after
`update()` instead, which keeps the decorator ignorant of localization's shape and keeps the value
consistent for every record in the tick.

---

## 4. The format, in one page

```text
offset  bytes  content
     0    256  FILE HEADER
              0 "SHBB" · 4 version(u16) · 6 headerBytes(u16) · 8 tickRecordBytes(u16)
             10 flags(u16) · 12 epochSeconds(f64) · 20 ringCapacity(u32) · 24 byteBudget(u32)
             28 buildHash[48] · 76 routineId[32] · 108 alliance[16] · 124 side[16]
            140 portMap[96] · 236 reserved[20]
   256      4  FRAME PREFIX  {u8 type, u8 reserved, u16 payloadBytes}   … then payload
              type 1 Tick(428) · 2 Summary(168) · 3 Triage(24+428) · 4 End(28)
```

The tick payload follows `debug_record.hpp`'s own declaration order, with the small scalars
gathered into one block so there is no implicit padding anywhere. Every offset is pinned
individually by the golden test (§8).

**What H1 (F9) inherits, explicitly.** This is a persistence contract the moment a file exists, but
it is deliberately *not* the SHUL/2 wire — that is streamed, sequenced and versioned on its own
terms. What H1 should reuse: (a) the **field order** of `encodeTick()`, so a pre-freeze schema
append lands at the end on both; (b) the **refuse-don't-misread** rule; (c) the **frame prefix**
idea, which is what makes unknown content skippable. What H1 must decide for itself: sequencing,
framing over a lossy link, and whether `RunSummary`/`MotionOutcome` ride v1 (both are already
wire-stable either way — C5's discharge table).

---

## 5. The flight recorder, and the graceful-end contract

**D-6 as built.** Default posture: `streamTicks = false`. Every record enters a fixed RAM ring
(default 200, HA-58), overwriting the oldest. The device sees **zero bytes** until a fault arrives.
Proven: 500 records through a ring-only sink leave `device.empty()` true and `writeCalls() == 0`.

**The trigger** is `DebugRecord::fault != None`, first fault only — the FaultLatch precedent. A
cascade must not dump twenty times, both because the first fault is the root cause and because
twenty dumps would blow the budget at the worst possible moment.

**The dump order is the graceful-end contract.** Concretely, in this order:

1. the file header (which build, which routine, when),
2. the **triage frame** — fault code, fault time, tick index, preceding-tick count, the latched
   brownout marker, **and the complete record of the fault tick**,
3. the preceding ticks, **oldest first**,
4. (at `close()`) the summary frame and the end frame.

A file cut anywhere after step 2 still answers *what broke, when, and in what state*. A file with
**no end frame ended abruptly** — that absence is the signal, and `BlackboxReader` reports
`truncated()` with the frames before the cut all delivered intact.

**Rejected: newest-first history.** It protects the most recent ticks against a cut, but it puts
every reader in reverse and makes a partial file's ordering depend on where the cut fell. Embedding
the fault tick in the triage frame buys the same protection for the one record that matters most,
without inverting the file.

**The one place a write happens without the caller asking** is this dump (`flushOnFault`, default
true). The reasoning: the fault has already happened, the run is already compromised, and the
evidence is worth one late tick. A caller who disagrees sets `flushOnFault = false` and the bytes
wait for the next `flush()` — bounded, counted, and their choice. Both paths are tested.

**Brownout.** The marker latches from the record stream (`FaultCode::Brownout`) or from
`markBrownout()` for a caller that detects it another way, and it reaches both the triage frame and
the end frame. It never unlatches: a battery that recovers does not erase the fact that it
collapsed. *(HA-19 says the V5 brownout kills motors first and the CPU survives, which is why a
synchronous dump is survivable at all. If R3 finds otherwise, the dump order is already arranged
for the worse world.)*

---

## 6. The introspection path, end to end

```text
ICorrector::propose()          the fix, and how far it sits from the prediction
   → IFusionPolicy::fuse()     the verdict + the innovation it was rendered on  → FusionResult::audit
   → Localizer                 kept on AppliedCorrection, alongside the applied dx/dy
   → CommandIdStampSink        stamped into DebugRecord's §18.2 gating slots, on EVERY record
   → SdSink                    encoded into the tick frame
   → BlackboxReader            decoded, field for field
```

Not one signature on the `IPoseSource` / `ICorrector` / `IFusionPolicy` seam moved. `GateAudit` is a
new value type; `FusionResult` and `AppliedCorrection` gained a member each, appended, so every
existing positional construction still compiles and means the same thing.

**What the shipping policy fills today** (`ComplementaryFusion`): `reason` — `Accepted` when a
proposal passed the gate, `RejectedInnovation` when one was rejected (including for
non-finiteness — a NaN is outside every bound, and inventing a new enumerator for it would be
inventing gate vocabulary E2 owns), `None` when no proposal arrived; `residualX/Y` — the innovation
the verdict was rendered on (the strongest accepted proposal's, else the first rejected one's);
`covarianceTrace` — the tier's scalar trust weight.

**One edge, documented rather than hidden:** on a `dt == 0` tick the per-tick budget allows no
motion, so a proposal can pass the gate with nothing applied. The audit reports `Accepted`, because
the field audits *the gate's verdict*; whether the nudge moved anything is `correctionDx/Dy`.

---

## 7. Findings

### 7.1 A REAL DEFECT, pre-existing: `DebugRecord::fault` had no producer

`grep` for anything writing `.fault =` on a record outside a test: **nothing in the entire tree.**

Consequences that were live before this chunk:

- `TermSink` has rendered ` flt=NAME` since A1, and **it could never appear on a real run.**
- The user guide has documented it since C8 — chapter 11, "a fault was raised *this tick*: this is
  where you find the exact moment something happened" — describing a field that was always `None`.
  A guide that confidently describes a field nothing fills is worse than one that omits it.
- D-6's trigger, planned in `diagnostics-plan.md` since 2026-08-06, had nothing to trigger on.

**Fixed in the layer that owns record population** (Rule 4): the scheduler snapshots the fault count
at tick start and `CommandIdStampSink` stamps the most recent fault raised during the tick, but only
onto records that do not already carry one (a producer that knows better wins — unlike the command
id, which the scheduler owns outright). Honest scope, stated on the decorator: a fault raised
*later* in the same tick lands on the next record, and the `FaultLatch` remains the authority on the
first-fault root cause.

Proven end to end without injecting anything: `test/blackbox_introspection_test.cpp` drives a real
`LoopMonitor` overrun, and the record emitted by that same tick carries `LOOP_OVERRUN` and triggers
a real dump.

### 7.2 A second producer gap: the applied-correction fields

Only `MoveToPose` stamped `correctionDx/Dy/clampedThisTick`. `TurnTo`, `StrafeTo`, `DriveBrake`,
`HoldPose` and the idle record all left them zero — so for most of a run the never-snap invariant
the §13 #4 audit exists for was simply blank. Fixed in the same one place.

### 7.3 GREEN HOLE #1 — the end frame could report zeros

**Mutation:** hard-code `EndInfo::tickFrames` and the byte figure to 0. **Whole suite green.**
The close test asserted `tickFrames == 0` for a ring-only run (true either way) and never looked at
the byte figure at all. A file that under-reports its own contents sends a reader hunting for data
that was never missing; one that over-reports hides a real gap.
**Closed by** `SdSink: the end frame's own counts describe the run truthfully`, which fails alone.

**And closing it found a real defect.** The field had been "bytes the device confirmed", which for
the most common run shape — caller-paced flushing, everything staged until `close()` — reads **0**
on a full file. Redefined as **`bytesBefore`: the frame's own offset in the file**, which a reader
can *verify* against where it actually found the frame. A self-describing footer that says 0 is
worse than no footer.

### 7.4 GREEN HOLE #2 — the sink's own header was never checked

**Mutation:** have `SdSink` write `epoch = 0, ringCapacity = 0, byteBudget = 0` into the header.
**Whole suite green.** The format tests pinned those fields by calling `encodeHeader` directly,
which proves the FORMAT can carry them and says exactly nothing about whether the sink supplies
them. An epoch of 0 mis-times an entire file; a ring capacity of 0 removes a reader's only way to
know how deep the captured history goes.
**Closed by** `SdSink: the header the SINK stamps describes the sink`, which fails alone.

**The lesson these two share, and it is worth carrying to E2:** the suite tested what the FORMAT
could carry and never what the SINK put in it. Any layer that both produces and serializes its own
metadata has this blind spot available to it.

### 7.5 A green mutation that is NOT a hole — and why I am not inventing a test for it

**Mutation:** make `stage()` commit the cursor even when the payload encode fails. **Green.**
Investigated rather than assumed: that branch is **unreachable by construction**. Every encoder
checks its size up front (§7.6) and `stage()` always hands it a span of exactly `payloadBytes`, so
`encode()` cannot return a short count. The guard is deliberate defence against a *future* encoder
that fails for a non-size reason, and it stays — but writing a test that pretends to exercise dead
code would be decoration, and this record says so instead.

### 7.6 A fix the tests found first: encoders wrote partial frames

`ByteWriter` refuses an append that would not fit, but the *encoders* wrote field by field until
they overflowed — leaving a partial record in the buffer and then reporting failure. Nothing
consumed it (the sink only advances its cursor on a full encode), but a half-written frame sitting
in a shared staging buffer is exactly the thing a later optimistic cursor update turns into corrupt
data. All six encoders now check the size up front and are **whole-or-nothing**.

### 7.7 A test bug worth keeping as a lesson

The first version of the drop-count e2e test called `resize()` on a `std::vector` the `SdSink`
already held a span over — dangling storage, and the sink was writing into freed memory. The
caller-owned-storage contract has to be honoured by the caller, and the very first caller to get it
wrong was a test. The header now says "never on a task stack" and "caller-owned"; it is worth
saying again in R1's adapter review.

---

## 8. Test inventory (45 cases; every case names its bug in-file)

**`blackbox_format_test.cpp` (20 / 20,348)**

- *Round trip on ground truth* — a record with a **different value in every field**, encoded,
  decoded, compared field by field (longhand, so a failure names the field; a `memcmp` would
  compare padding and say nothing).
- *A whole file of ticks, in order*, with provenance read back from the header — catches a cursor
  that fails to advance.
- *Every `GateReason` × a spread of `FaultCode`s* — catches an enum written in the wrong width.
- *Boundary values*: denorm_min, `-0.0` (sign checked — a bit copy, not a parse), `double` max and
  lowest, NaN, ±∞, 1e300, u32 max, u8 max.
- *Summary, end and triage frames* round-trip; the triage frame carries the fault tick's **whole**
  record.
- **Byte-exact goldens** (19,193 of the assertions): the 256-byte header, hand-derived from the
  documented layout with literal IEEE-754 bit patterns; **44 tick fields each written alone**, each
  checked at its documented offset **and with every other byte of the 428 asserted zero** (the half
  that catches two fields overlapping); a default record encoding to 428 zeros; the frame prefix.
- *Refusal*: unknown version (and the version is still readable), width mismatch, bad magic, empty
  file, stub header.
- *Damage*: a cut mid-frame (three whole frames delivered, `truncated()`, no end frame), a cut
  inside a frame prefix, an unknown frame type skipped by length and counted, a known type with the
  wrong payload size skipped rather than decoded, a **non-finite heading** decoded to zero with
  `corrupt` raised instead of tripping `Angle`'s precondition.
- *Undersized buffers*: encode writes nothing and says so; decode of a short payload is refused.

**`sd_sink_test.cpp` (20 / 209)**

- *No fault ⇒ nothing written* — 500 records, zero bytes, zero `write()` calls, ring full.
- *The dump carries the ticks PRECEDING the fault, oldest first* — ring of 4, ten ticks, fault on
  the eleventh: triage first, then ticks 6–9, and the fault tick appears **exactly once**.
- *Only the first fault dumps.*
- *Budget exhaustion*: 10 ticks into a 3-frame budget ⇒ 7 dropped, 3 staged, nothing written behind
  the caller's back, buffer not grown, and the flushed file **still decodes** with three intact
  frames and no truncation.
- *The drop count rides into the summary frame* — and the sink's LIVE count wins over a stale one
  the caller assembled.
- *A device that dies mid-write* leaves a truncated, readable file; the frames that never landed are
  counted as drops.
- *Disabled costs nothing* — `wantsRecord()` false, the `emitRecord` builder never invoked
  (`builds == 0`), nothing written even when handed a fault.
- *`close()` stamps the graceful end*, with the message count visible in it.
- *The end frame's counts describe the run* (hole #1) and *the header describes the sink* (hole #2).
- *Over-long provenance truncates* rather than overflowing the fixed-width header.
- *Brownout latches* through recovery, into triage and end; *`markBrownout()`* works from outside
  the record stream.
- *`triggerDump()`* for a fault that never rode a record.
- *Streaming mode adds only the triage frame* (no duplicated history).
- *A dump larger than the buffer* flushes as it goes, in one valid stream, losing nothing.
- *A buffer too small for a triage frame is refused at construction.*
- *No ring at all* still writes the triage block.
- *The `SdSinkBuffers` helper* wires real storage.

**`blackbox_introspection_test.cpp` (5 / 116)**

- *The synthetic path*: six ticks, every `GateReason` in turn, every gating field asserted in the
  **decoded file** against the value the script chose — plus a check that all six reasons really
  appeared (a constant stamp would pass the per-tick loop otherwise).
- *The real policy*: `ComplementaryFusion` with a `FakeCorrector` — no fix ⇒ `None`; a 2-inch fix
  inside the gate ⇒ `Accepted`, residual 2.0, trust 0.4, a non-zero applied nudge, Mahalanobis
  honestly 0; a 40-inch fix ⇒ `RejectedInnovation`, residual 40.0, nothing applied.
- *A real fault*: a genuine `LoopMonitor` overrun ⇒ the record carries it ⇒ the ring dumps, triage
  first, with four healthy preceding ticks.
- *D-7 on the terminal*: printed after the summary on a faulted run, absent entirely on a clean one.
- *The drop count in the run summary*: shown when real, absent when there is no blackbox.

---

## 9. Mutation campaign — 27 executed (break → **build gate** → run → OBSERVE → restore)

The runner refuses to report a result for a mutation that does not compile (D1 tripped this twice;
C4 nearly misread a non-compiling mutation off a stale binary), and re-runs the full suite after
restoring.

| # | Mutation | Result |
|---|---|---|
| 1 | Encoder writes `gateResidualY` before `gateResidualX` (encoder only) | **RED** — 6 cases: round trip AND golden AND the e2e |
| **1b** | **The same swap in encoder AND decoder (symmetric)** | **RED — and only the GOLDEN caught it.** 748/749 cases still passed |
| 2 | Drop counter never increments | **RED** — 3 cases |
| 3 | Ring overwrites the wrong end (keeps oldest) | **RED** — 2 cases, 6 assertions |
| 4 | Version rejection removed | **RED** |
| 5 | Layout cross-check removed | **RED** |
| 6 | Triage written AFTER the history | **RED** — 2 cases |
| 7 | Every fault dumps (first-fault latch removed) | **RED** |
| 8 | Reader never reports truncation | **RED** — 2 cases |
| 9 | `close()` skips the end frame | **RED** — 3 cases |
| 10 | `wantsRecord()` true when disabled | **RED** |
| 11 | Estimator stamp dropped | **RED** — 36 assertions |
| 12 | Tick-fault stamp removed | **RED** — 2 cases |
| 13 | `ComplementaryFusion` reports `Accepted` for a rejected fix | **RED** |
| 14 | End frame reports zeros | **GREEN — HOLE #1** (§7.3) |
| 15 | Sink header epoch/ring/budget zeroed | **GREEN — HOLE #2** (§7.4) |
| 16 | `ByteWriter::text` stops NUL-padding | **RED** — 4 cases, 54 assertions |
| 17 | Provenance copy drops its bound | **RED** |
| 14R | Hole #1's mutation, after the new test | **RED**, sole detector |
| 15R | Hole #2's mutation, after the new test | **RED**, sole detector |
| 18 | `stage()` commits a half-encoded frame | **GREEN — unreachable branch** (§7.5) |
| 19 | `flush()` forgets to reset the cursor | **RED** — 2 cases |
| 20 | Triage `precedingTicks` always 0 | **RED** — 3 cases |
| 21 | Device-failure drops not counted | **RED** |
| 22 | Reader never reports `sawEnd` | **RED** |
| 23 | `summarize()` writes the caller's drop count, not the live one | **RED** |
| 24 | `close()` writes a file for a run with nothing to say | **RED** |

**24 red, 3 green (2 holes closed, 1 unreachable).** After restoration the suite is green and
`grep -rn MUTATION include/ test/` shows only the two "FOUND BY MUTATION" test comments and a
pre-existing note in `sim/truth_integrator.hpp`.

**Mutation 1b is the one to remember.** Moving a field in the encoder *and* the decoder together
leaves the round trip perfectly happy — the two sides agree, and every file ever written is now
unreadable. Only the byte-exact per-field golden saw it. That is not an argument about test design;
it is an observation from a run.

---

## 10. Cost when disabled — pinned, not asserted

With `SdSinkConfig{.enabled = false}`: `wantsRecord()` returns false, so `hal::emitRecord` never
invokes the builder — the record is not merely discarded, it is **never populated**. The test
counts builder invocations across 100 emit attempts and asserts `builds == 0`, the same shape as
A1's null-sink proof. Nothing is written even when the sink is handed a fault directly, and
`recordsSeen()` / `ringSize()` stay 0. Mutation 10 (`wantsRecord()` hard-coded true) turns it red.

With the sink *enabled* but in the default flight-recorder posture, the per-tick cost is a record
copy into the ring and a branch — no formatting, no encoding, no IO. The encoding cost is paid only
by frames that are actually staged.

---

## 11. What we know for certain, and what we do not

**Known, with evidence**

- A blackbox file written by this code can be read back **field for field** by this code, across
  every `GateReason`, a spread of faults, and boundary values including NaN, ±∞ and `-0.0`.
- The layout is pinned byte by byte, independently of the encoder, at 44 field offsets.
- An unknown version, a mismatched record width, a bad magic, a truncated file, an unknown frame
  type and a non-finite heading all behave as designed — no throw, no out-of-bounds read, no
  confident misreading.
- A ring-only run puts **zero bytes** on the device until a fault fires; a fault dump contains the
  ticks preceding it, oldest first, with triage first.
- Budget exhaustion drops whole frames, counts them, keeps the file decodable, and reports the count
  in the run summary and in the file.
- The introspection path carries a corrector's numbers to a decoded file, with a synthetic gate.
- `DebugRecord::fault` now has a producer, proven by a real `LoopMonitor` overrun.

**NOT known, stated plainly**

- **Nothing here has touched an SD card.** There is no `/usd/` adapter (R1's), no measured write
  latency (HA-60), and no evidence about what a real brain does when the card is slow, full or
  absent. Every timing claim in this chunk is an assumption with a register entry.
- **No real gating decision has been recorded.** T3. The path is proven; the content is E2/E3/E4's.
- **The ring depth and the byte budget are guesses** (HA-58, HA-59). Nobody has yet measured how far
  before a fault its cause sits, or what RAM the real program can spare.
- **The brownout path has never been exercised by a real brownout.** The latch and the dump order
  are designed for it and tested with a fake device; HA-19's "the CPU survives" is unverified.
- **`SdSink` has never run under a real 100 Hz load.** Its per-tick cost is a ring copy by
  construction, but "by construction" is not a measurement.

---

## 12. Freeze Register note

**E1 freezes nothing.** F9 (the SHUL/2 wire serialization of `DebugRecord`) is H1's, and this chunk
deliberately does not pre-empt it — §4 says what H1 should reuse and what it must decide for itself.

Two things about the *blackbox* format are worth being precise about, because they are easy to
confuse with a freeze:

- The on-disk layout is a **persistence contract** from the moment a file exists — but the
  enforcement is the version field, not the Freeze Register. Changing the layout is allowed; doing
  it **without bumping `kFormatVersion`** is what breaks files, and the reader's width cross-check
  exists to catch exactly that mistake.
- `DebugRecord` itself is still free to change until H1. E1 added no field to it and reshaped none —
  it populated six that A1 declared and left unpopulated by design.

---

## 13. Verification (actually run, outputs as observed)

```text
$ cmake --build build/test -j$(nproc) && ./build/test/shulib_tests | tail -6
[doctest] test cases:    752 |    752 passed | 0 failed | 3 skipped
[doctest] assertions: 936895 | 936895 passed | 0 failed |
[doctest] Status: SUCCESS!
```

Baseline before the chunk: **707 / 916,216 / 3 skipped**. (The 3 skips are the pre-existing M0
acceptance stubs, unchanged.)

```text
$ grep -rnE '#\s*include\s*[<"]pros/' include/shulib   → GUARD1 PASS
$ grep -rnE --exclude-dir=sim '#\s*include\s*[<"]shulib/sim/' include/shulib   → GUARD2 PASS
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
    -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
ARM GATE CLEAN          # 110 headers (104 before this chunk + 6 new)
```

```text
$ python3 tools/api_doc_tool.py self-test | check-coverage | check-fresh | check-examples | check-removability
api_doc_tool self-test: OK
doc example scan: 343 quoted lines, all verbatim (3 source files)
removability: no public doc references docs/internal/
ALL DOC GATES PASS
```

Nothing was committed; `git status` shows 7 modified headers, 6 new headers, 3 new tests and the
documentation edits, all in the working tree.

---

## 14. Deliberately left for later (named handoffs)

- **The `/usd/` adapter** → **R1**. E1 ships `hal::IBlockSink` + `FakeBlockSink`; the PROS `FILE*`
  implementation, the filename policy (one file per run? per session?) and card-absent handling are
  R1's. R1 should re-check HA-60 the first time it runs.
- **Real gating content** → **E2** (`GpsCorrector`), **E3** (`AprilTagCorrector`), **E4** (EKF).
  E2 fills `RejectedNoFix`/`RejectedHighYawRate`; E4 fills `gateMahalanobis` and replaces the
  trust-weight reading of `covarianceTrace`.
- **D-8, the routine-level watchdog** → **F2** (or the `Routine`-layer whole-chain deadline named in
  D3's record). It did **not** fall out of this chunk for free, and half-building it here would have
  put the deadline in the wrong layer. Reasoning recorded in `diagnostics-plan.md`.
- **`SHUL/2` + the F9 freeze** → **H1**; **replay-as-regression-test (D-9)** → **H2**. §4 lists what
  H1 inherits from this format.
- **Measuring the guesses** → **R4**: HA-58 (ring depth), HA-59 (RAM budget), HA-60 (flush cost).
- **A blackbox *reading* tool** (a host CLI that prints a `.blk` file) — the decoder exists and is
  the hard part; a `main()` around it is a small R- or H-phase convenience, and H2's replay work
  wants it anyway.

---

## 15. DoD checklist (brief §Definition of Done)

- [x] `SdSink` writes a versioned, session-stamped, fixed-width binary blackbox —
      `diag/blackbox_format.hpp` v1 + `diag/sd_sink.hpp`; header goldens pin the version and the
      provenance
- [x] **A decoder exists**, round trip proven on ground truth — `diag/blackbox_reader.hpp`;
      field-for-field equality across every `GateReason`, faults and boundary values
- [x] Budget exhaustion drops-and-counts, count visible in the run summary — `droppedFrames()`,
      the summary frame, the end frame, and `· blackbox dropped N` on the terminal
- [x] Truncated and unknown-version files behave as designed, tested — refusal, cut mid-frame, cut
      mid-prefix, unknown frame type, mis-sized frame, corrupt heading
- [x] D-6 flight recorder: ring always on, dumped only on fault, preceding ticks present
- [x] D-7 triage block on fault — in the file and on the terminal, from one struct
- [x] Brownout marker latched; graceful-end contract defined (§5) and tested
- [~] Introspection path proven with a synthetic corrector; **T3's honest scoping stated** — the
      path is complete and proven; **no real gate exists at E1**, `gateMahalanobis` is 0 until E4,
      and nothing here certifies `< 1°`
- [x] T1 and T2 ruled explicitly, with rejected alternatives — §2
- [x] Invented constants registered as `HA-nn` — HA-58 (ring depth), HA-59 (byte budget), HA-60
      (assumed flush cost), each labelled in-header
- [x] Cost-when-disabled pinned — builder never invoked, mutation-proven
- [x] Suite green; both guards; ARM gate; doc gates — §13

**Not claimed:** that the blackbox works on a robot (no adapter, no card, no measurement), that any
real gating decision has been recorded, or that the accuracy target is certified.
