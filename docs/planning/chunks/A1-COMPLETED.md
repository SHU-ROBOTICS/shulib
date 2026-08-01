# Chunk A1 — COMPLETED (2026-08-01)

> Completion record for [`A1-debugrecord-termsink.md`](A1-debugrecord-termsink.md).
> Everything below is **as actually observed** — commands run, outputs captured, mutations executed.
> Changes are in the working tree, uncommitted, pending DoD review.

---

## 1. What was built

| Piece | File | Role |
|---|---|---|
| `FaultCode` + `FaultLatch` | `include/shulib/diag/fault.hpp` *(new)* | Wire-stable §18.4 fault enum; latched first-fault + cascade counting; crash-proof `raise()` |
| `DebugRecord` + `GateReason` | `include/shulib/diag/debug_record.hpp` *(new)* | The complete §18.2 per-tick snapshot schema, typed units, F9-freeze-aware |
| Finite guards | `include/shulib/diag/finite_guard.hpp` *(new)* | NaN/Inf log-and-recover (`recoverFinite` / `recoverFinitePose` / `isFinitePose`) |
| `LoopMonitor` | `include/shulib/diag/loop_monitor.hpp` *(new)* | Loop-overrun / tick-timing detection against a dt budget; `worstDt()` for C5's summary |
| `SHULIB_TRACE` | `include/shulib/diag/trace.hpp` *(new)* | Compile-time TRACE strip (default OFF = stripped; `-DSHULIB_ENABLE_TRACE` for dev) |
| `TermSink` | `include/shulib/diag/term_sink.hpp` *(new)* | The §18.3 column-aligned terminal formatter; renders both channels |
| `ICharSink` | `include/shulib/hal/char_sink.hpp` *(new)* | The injectable byte-output seam (NOT part of the frozen F4 ten; additive) |
| `FakeCharSink` | `include/shulib/hal/fake/fake_char_sink.hpp` *(new)* | Recording char device for golden-output tests |
| Seam extension | `include/shulib/hal/telemetry_sink.hpp` *(modified)* | `emit()` (non-pure, no-op default) + `wantsRecord()` + the `emitRecord()` lazy-build helper |
| Policy seam | `include/shulib/core/check.hpp` *(modified)* | The §18.4 precondition policy handler — resolves the former `TODO(M0 / §18.4)` |
| Fake extension | `include/shulib/hal/fake/fake_telemetry_sink.hpp` *(modified)* | Records the emit channel too (bounds-checked, paired `wantsRecord()` override) |
| Doc-only | `include/shulib/hal/null_sink.hpp` *(modified)* | Header note: inheriting the defaults IS the cost mechanism + the additivity proof. **No code change** — that is the point |
| CI guard | `.github/workflows/ci.yml` *(modified)* | `include/shulib/diag` added to the PROS-free guard scope |

**New tests:** `test/debug_record_test.cpp` (8 cases), `test/fault_test.cpp` (7),
`test/loop_monitor_test.cpp` (7), `test/finite_guard_test.cpp` (8), `test/term_sink_test.cpp` (15),
`test/trace_strip_test.cpp` (2), `test/trace_enabled_test.cpp` (1), `test/check_policy_test.cpp` (5),
plus 2 cases added to `test/telemetry_sink_test.cpp` (now 6). **55 new cases / 215 new assertions.**

**Layout decision:** the WS13 discipline got its own `include/shulib/diag/` (schema, faults,
guards, monitor, trace, formatter) — sinks are formatting/diagnostics logic, not hardware, and E1's
`SdSink` + C5's result/summary code will land there too instead of crowding `hal/`. The one genuine
device seam (`ICharSink`, where bytes physically go) went to `hal/` with its fake, mirroring
`ITelemetrySink`'s own placement. The CI guard was extended to cover `diag/`.

---

## 2. Decision log (every choice with a viable alternative)

### D1 — Null-sink cost mechanism: runtime `wantsRecord()` + lazy `emitRecord(sink, buildFn)`
The tick loop asks the sink whether it consumes records; the record is built by a callable invoked
**only if** it does. With `NullSink` the per-tick cost is one virtual bool call — population never
happens (proven by test).
**Rejected:** *always-build-and-emit* (pays ~30 field writes/pose copies per tick to discard —
the exact failure the brief flags); *compile-time sink policy (template/#ifdef)* (truly zero cost,
but bifurcates the build and breaks the single runtime `ITelemetrySink&` seam F4 froze — dev builds
must switch sinks without recompiling the core — all to save one virtual call per 10 ms);
*`wantsRecord()` defaulting to `true`* (every message-only sink would silently pay full population
for a no-op `emit()`; the defaults must agree — default no-op emit ⇒ default false — at the cost
that `emit()`/`wantsRecord()` must be overridden **as a pair**, which the seam header states loudly
and both shipped consuming sinks obey under test).
The `emitRecord()` helper exists so lazy population is the path of least resistance — same lesson
as the legacy `escapeJSONString` defect: a step callers must remember is a step that gets skipped.

### D2 — TRACE strip mechanism: `true ? (void)0 : (void)(sink.log(...))`
**Rejected:** plain `((void)0)` (arguments no longer type-checked — a trace call can bit-rot until
someone enables tracing months later; variables used only in traces trip `-Wunused` under
`-Werror`); *runtime level check* (still evaluates/builds the message every tick — fails the §18.3
zero-cost requirement outright). The chosen form has language-guaranteed non-evaluation of the
false branch (proven by the side-effect-counter test), keeps arguments odr-used and type-checked,
and folds to nothing under `-Os` (proven by ARM asm diff, §4). Default is **stripped**; dev builds
opt in with `-DSHULIB_ENABLE_TRACE`. The flag is per-TU at preprocess time, which is what lets one
test binary prove both configurations.

### D3 — `check.hpp` policy seam: installable non-returning handler; both policies throw
`SHULIB_PRECONDITION` call sites are untouched. The default handler throws `PreconditionError`
(host tests stay red on breach); R1's on-robot handler will raise `FaultCode::Precondition` on the
latch **and then throw the same error**, which the motion scheduler catches at the task boundary →
`FAULT_ABORT` → safe state. Recovery at the *motion boundary*, not the call site — preconditions
guard invariants (bounds, non-null) past which continuing is UB, so "log and continue right here"
is not a safe fallback; unwinding to the nearest boundary that CAN recover is.
**Rejected:** *returning handlers* (execution would continue past a violated invariant —
e.g. `FakeTelemetrySink::at` would index out of range); *`#ifdef` policy split* (two compiled
behaviors, and the test suite could no longer exercise the routing); *`std::function` slot*
(allocation + indirection on a hot-adjacent path; a plain function pointer suffices — R1's handler
reaches its context via the robot singleton it already owns). A contract-violating handler that
returns hits `std::terminate()` — reachable only by breaking the seam's stated contract, kept so
`precondition_failed` honestly remains `[[noreturn]]`. `nullptr` restores the default so the slot
can never be left empty. Handler installation is init-time single-threaded by contract (no atomics
on the read path) — stated in the header.

### D4 — Clean-room rejection of `logger.hpp` (per the brief's "Explicitly rejected")
Nothing was ported. Each legacy defect became a structural design rule:
| Legacy defect | Designed against, how |
|---|---|
| `escapeJSONString` written but never called | Sanitization is **unavoidable by construction**: `TermSink::Line::appendSanitized` is the *only* path caller text takes into a line; there is no unsanitized route to the device. Pinned by the control-byte + framing tests |
| Dead `sendDebugMessages` (declared, called, never defined) | No dead paths shipped: every public member of every new class is reached by at least one test; nothing exists that isn't exercised |
| Manual `update()` racing the background flush task | The race is **designed out, not locked around**: no background task, no shared mutable buffers exist anywhere in `diag/` — `TermSink` holds zero mutable state and each call performs exactly one `write()`. Every header states its concurrency contract explicitly |
Also designed against §18's named anti-patterns: structured `key=value` fault lines (never prose),
no hot-loop essays, no raw `std::cout` (bytes go through the injected `ICharSink`).

### D5 — `DebugRecord` shape choices (the schema is the chunk's highest-stakes artifact)
- **Complete §18.2 field set now**, unpopulated where the producer doesn't exist yet; every field
  documents its producing chunk. F9 freeze warning at the top of the header.
- **Typed units everywhere a field has a dimension** (F3): `Time`, `Length`, `Voltage`, `Current`,
  `AngularVelocity`, `Pose2d`, `ChassisSpeeds`. Angular *errors/deltas* are `units::AngleDim`
  (radians, non-wrapping — a difference is not a heading); absolute headings are `math::Angle`.
- **Per-wheel capacity = `kinematics::WheelSpeeds::kMaxWheels`** (aliased, so they can never
  diverge — pinned by test) rather than an independent constant.
- **`qualityClass` as a raw byte mirroring `Localizer::Quality`** so `diag/` stays a dependency
  leaf (localization may later include diag, never the reverse); the numeric mapping is pinned by a
  test that includes both, so a reorder of either enum goes red.
- **`GateReason` vocabulary defined now** (None/Accepted/RejectedInnovation/RejectedMahalanobis/
  RejectedNoFix/RejectedHighYawRate) from the master plan's E2–E4 gating language, wire-pinned —
  defining it at first serialization consumer (E2) would have meant reshaping the frozen record.
- **`activeCommandId`/`activeCommandState` as wire-stable raw ints** whose vocabulary C1/C2 will
  own — reserving the slot without inventing motion semantics that don't exist yet.
- **One `covarianceTrace` slot** for "covariance trace / filter trust weights" (§18.2's own
  either/or) — semantics follow the active fusion policy; a second slot would freeze a distinction
  the spec doesn't make.
- **`fault` = the fault raised THIS tick**; the latch owns the first-fault distinction. (§18.2
  lists one "fault code"; the run summary at C5 reports the latch.)

### D6 — Forward-declared `DebugRecord` in `telemetry_sink.hpp`
The seam declares `emit(const diag::DebugRecord&)` against a forward declaration (legal for a
reference parameter with a no-op body), so the M1-era seam header stays include-light and `hal/`
does not depend on `diag/`; implementers that read the record include the schema themselves.
**Rejected:** including the schema from the seam (inverts the layering: every HAL user would pull
in the full diagnostics schema); putting `DebugRecord` inside `hal/` (WS13 owns the schema, and
E1/C5 will grow `diag/` around it).

### D7 — Formatter semantics pinned rather than left vague (each is a golden test)
`[t=%7.2f]` fixed width (deviates from the §18.3 sketch's unpadded `[t=12.34]` — alignment across
ticks IS the requirement; the sketch is a shape, not a byte spec) · Info lines carry no level tag,
other levels butt `[LEVEL][TAG]` (matches the sketch exactly) · `emit()` stamps from the
**record's** `t`, not the clock (replayed records must render identically to live ones) ·
non-finite → deterministic `NaN`/`+Inf`/`-Inf` tokens (libc's `nan`/`-nan(0x…)` varies) ·
pathological magnitudes re-render `%.3g` (bounded widening instead of a 300-digit column) ·
control bytes → `?` (framing unbreakable) · truncation backs off UTF-8 continuation bytes (no
mojibake) with `…` · idle ticks tagged `[LOC]`, active ones `[MOT] cmd#N▸S`.

### D8 — `FaultLatch::raise()` is `noexcept` with an internal catch-all
The error path must be unconditionally safe. Latch state updates **before** the log attempt, so
even a contract-violating (throwing) sink or clock leaves the fault latched; the throw is
swallowed. Raising `FaultCode::None` is a defensive no-op rather than a precondition — a
precondition throw inside fault-raising would convert a bad raise into a dead robot.
**Rejected:** letting `raise()` throw (breaks every failure handler that calls it); preconditioning
`code != None` (crash risk on the one path that must never crash).

### D9 — `LoopMonitor` boundary: overrun iff `dt >= budget` (inclusive)
The budget is a hard deadline: a loop that consumes its entire budget has zero margin and the next
tick already starts late. Consequence (documented + tested): the budget must be configured strictly
above the nominal period (a healthy 10 ms loop has dt == 10 ms every tick). The exact edge is
pinned with float-exact values (1.0 → 1.5 gives dt == 0.5 exactly), so `>=` vs `>` is genuinely
distinguishable — and was proven so by mutation (§5).

---

## 3. Test inventory (what each would catch)

`test/` totals moved **246 cases / 521,908 assertions → 301 / 522,123** (4 pre-existing M0
acceptance stubs still deliberately skipped, unchanged).

**telemetry_sink_test.cpp (6, +2 new)** — *additivity*: `MessageOnlySink` implements only `log()`;
its compilation + run is the test that goes red if `emit()`/`wantsRecord()` ever become pure
(the F4-breaking regression). *NullSink through the interface* wants no records, drops an emitted
record silently.

**debug_record_test.cpp (8)** — default record is a valid "quiet" tick; **every §18.2 field
populated and read back** (deleting/retyping a field breaks this file first); typed-unit + wire-width
`static_assert` pins; per-wheel capacity tied to `WheelSpeeds::kMaxWheels`; `GateReason` numeric
pins; `qualityClass` ↔ `Localizer::Quality` mapping pins (reorder of either enum → red);
**the null-sink cost proof** (builder not invoked for `NullSink`); build-once-and-delivered for a
consuming sink; `FakeTelemetrySink` record history ordered/bounds-checked/cleared.

**fault_test.cpp (7)** — `FaultCode` numeric pins (F9) + underlying-type pin; §18.4 name spellings
+ out-of-range → `"UNKNOWN"`; **cascade retains the FIRST fault** (code AND timestamp) while
counting all; exact structured log lines with the FIRST marker distinct from cascade; raising
`None` is a no-op; `clear()` opens a new run (new FIRST, new time); **a throwing sink cannot crash
`raise()` and the latch still latches** (raise is `noexcept` — an escaped throw would terminate the
suite, so reaching the asserts is the proof).

**loop_monitor_test.cpp (7)** — first tick baselines (huge absolute time ≠ huge dt); healthy
cadence never faults, `worstDt` tracks the max not the last; **the exact `>=` edge** (0.499999
passes, float-exact 0.5 faults); pinned structured overrun message (`dt=0.7500 budget=0.5000`);
overruns cascade into the latch without usurping an earlier first fault; `reset()` makes a
deliberate gap non-reportable while preserving history, and re-arms afterwards; non-positive
budget rejected.

**finite_guard_test.cpp (8)** — `isFinitePose` across NaN/±Inf per axis (+ the Angle-can't-be-NaN
claim asserted next to its consumer); finite double passes exactly, no fault; NaN/±Inf caught,
logged **with the given code** (not hard-wired), replaced by fallback; **non-finite fallback
degrades to 0 / origin — the return is finite unconditionally**; the DoD case: injected NaN pose →
caught, logged as `NAN_POSE`, recovered to last-known-good, next healthy tick passes (no sticky
state); every axis combination; finite pose never altered.

**term_sink_test.cpp (15)** — byte-exact goldens: per-level message lines (Info bare;
`[WARN][SEQ]` butted — the easy regression); clock-stamped and advancing; **the §18.3 per-tick line
for an active command**; idle `[LOC]` line; flags/fault appear iff set; **NaN/±Inf tokens**;
**1e300 → `%.3g` compaction + tiny/wide-but-sane values**; the F3 ±180° tie-break visible
(−180° renders `180.0`); empty subsystem `[]`; control-byte sanitization + the exactly-one-`\n`
framing proof; over-long message truncation with `…` (and exactly-at-cap NOT truncated);
**UTF-8 boundary backing-off** (a `°` straddling the cap); over-long subsystem truncation;
`wantsRecord()` true; **the full synthetic tick stream transcript — the A1 DoD case**.

**trace_strip_test.cpp (2, default/stripped TU)** — argument expressions (message builder AND sink
lookup) **never evaluated**, sink sees nothing — stronger than a sink-sees-nothing proxy, which a
runtime level check would pass; expansion safe in an unbraced `if/else` (doesn't eat the else-arm).

**trace_enabled_test.cpp (1, `-DSHULIB_ENABLE_TRACE` TU)** — the enabled path logs at Trace with
the given tag/message, arguments evaluated **exactly once** (catches both a dead-in-all-configs
strip and macro double-evaluation).

**check_policy_test.cpp (5)** — default policy throws `PreconditionError` with the exact message
(everything existing depends on this); an installed handler receives the violation + message
instead; a passing check never consults the policy; `setPreconditionHandler` returns the previous
handler and restoring restores behavior; `nullptr` resets to default. All handler installs are
RAII-restored so the rest of the suite keeps the throwing policy even on assertion failure.

---

## 4. Verification (actually run, outputs as observed)

```text
$ cmake -S test -B build/test && cmake --build build/test && ./build/test/shulib_tests
[doctest] test cases:    301 |    301 passed | 0 failed | 4 skipped
[doctest] assertions: 522123 | 522123 passed | 0 failed |
[doctest] Status: SUCCESS!
```

```text
$ <the ci.yml guard grep, with include/shulib/diag added to its scope>
core is PROS-free (guard passes, incl. new diag/)
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # TU includes ALL 63 v2 headers
ARM CROSS-COMPILE: CLEAN
```

**Extra zero-cost proof (beyond the brief):** two ARM `-Os` functions differing only by one
stripped `SHULIB_TRACE` call compile to **instruction-identical** bodies
(`add r1, r1, r1, lsl #1 · add r0, r1, #1 · bx lr` both) — the strip is free at the machine-code
level on the competition target, not merely side-effect-free on the host.

---

## 5. Mutation checks (each executed: break → build → run → observe red → restore)

| # | Mutation | Required? | Observed result |
|---|---|---|---|
| 1 | `FaultLatch` latches the **last** fault instead of the first | yes | **RED** — 2 cases failed (`fault_test.cpp:75`, `loop_monitor_test.cpp:117`, both `firstFault()` checks); 299/301 |
| 2 | `LoopMonitor` boundary loosened `>=` → `>` | yes | **RED** — the edge case failed all 3 of its boundary asserts (`loop_monitor_test.cpp:85-87`); 300/301 |
| 3 | NaN guard removed (`isFinitePose` → `true`) | yes | **RED** — 4 cases / 13 assertions failed across `finite_guard_test.cpp`; 297/301 |
| 4 | Stripped TRACE branch made to evaluate args (`true ?` → `false ?`) | extra | **RED** — both strip cases failed (builds/lookups counted, sink non-empty); 299/301 |
| 5 | `emitRecord` guard dropped (always build) | extra | **RED** — the null-sink cost proof failed (`debug_record_test.cpp:165`, `builds == 0`); 300/301 |
| 6 | `[WARN]` given a stray trailing space | extra | **RED** — the Warn golden AND the transcript failed byte-compare; 299/301 |
| 7 | `setPreconditionHandler` silently drops the install | extra | **RED** — routing + restorability cases failed (5 asserts, `check_policy_test.cpp:62-82`); 299/301. *Note: the first attempt of this mutation didn't compile (`-Werror` unused-parameter) — the interim test output was a stale binary and was discarded; the mutation was adjusted with `(void)handler;` and re-run for the genuine red above* |

After each restoration the full suite was rebuilt and re-run; final state
`grep -rn MUTATION include/ test/` is empty and the suite is green (301/522,123).

---

## 6. Discovered about existing code

- **`Angle` cannot hold a non-finite value** (factories reject) — load-bearing for
  `isFinitePose` checking only x/y; now asserted next to its consumer in `finite_guard_test.cpp`
  so a future weakening of `Angle` fails adjacently.
- **`Localizer::Quality` had no pinned numeric values** — it now effectively does, via the
  `qualityClass` mapping test (a reorder of that enum is caught by A1's suite).
- **`IClock::now()` is not `noexcept`** — consumed inside `FaultLatch::raise()`'s try-block so a
  contract-violating clock can't crash the error path. Not changed (F4 frozen; the contract is
  "doesn't throw", the seam just no longer trusts it with the run).
- **Legacy `sendDebugMessages` is worse than "dead"**: declared (`logger.hpp:172`) and *called*
  (`logger.cpp:17`) but never defined — the legacy TU cannot even link. Reinforces C7's deletion
  rationale; nothing was salvaged from it.
- Markdown lint (MD060 table style) flags pre-existing tables in `roadmap.md` — cosmetic, known,
  deliberately outside CI per the M0 notes; not introduced by this chunk.

## 7. Deliberately left for later chunks

- **Per-motion result line, session header, run-summary block** → C5 (needs motion data; the
  `worstDt()` and first-fault quantities the summary will read are already collected).
- **Rate-budgeting / high-rate-channel throttling** (§18.2 "rate-budgeted", §18.3 "throttled") →
  the tick-loop producer (C1/C5) — there is no high-rate producer to throttle yet, and the budget
  belongs to the producer, not the formatter.
- **Exit-reason codes on every `IMotion`** → C1/C2 (`IMotion` doesn't exist; `ExitReason` +
  `MotionTimeout`/fault vocabulary are ready for it).
- **The on-robot precondition handler + stdout `ICharSink` adapter** → R1 (the seams and their
  contracts are built and tested; only the PROS glue is deferred).
- **`SdSink`** → E1; **`SHUL/2` wire + F9 freeze** → H1 (the schema they serialize is now fixed
  in shape).
- **ARM compile in CI** → A4 as planned (verified manually this chunk; the all-headers TU lives in
  the session scratchpad and A4 should commit a permanent copy).

## 8. DoD checklist (brief §Definition of Done)

- [x] Synthetic tick stream renders the §18.3 target shape — `term_sink_test.cpp` transcript case (exact bytes)
- [x] `TRACE` provably stripped at compile time — args-unevaluated test + identical-ARM-asm probe
- [x] Injected NaN caught, logged, recovered — `finite_guard_test.cpp` DoD case; no crash, no propagation
- [x] `DebugRecord` carries the complete §18.2 field set — populate-every-field case + type pins
- [x] `emit()` non-pure with default no-op; `NullSink`/`FakeTelemetrySink` compile (NullSink untouched by construction; the fake's emit-recording is an additive test-double upgrade, not a compile fix)
- [x] `check.hpp` `TODO(§18.4)` resolved, call sites unchanged — policy seam + `check_policy_test.cpp`
- [x] Host suite green under strict `-Werror` — 301 cases / 522,123 assertions
- [x] v2 core cross-compiles for ARM — all 63 headers, strict flags, clean
- [x] CI PROS-free guard passes — with `diag/` added to its scope
