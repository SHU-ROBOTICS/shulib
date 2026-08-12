# Chunk D2 — COMPLETED (ruled 2026-08-11; constructed + verified 2026-08-12; working tree pending review)

> **F6 is LOCKED. The public `Chassis` API is frozen as API 2.0 — and the freeze is a
> mechanism, not a comment.** Every frozen member's exact type is asserted at compile time
> (`test/f6_signature_pin_test.cpp`, 36 pins — mutation-proven member by member); the
> version bump the Freeze Register has promised since M0 now exists in code
> (`include/shulib/version.hpp`, with the written breaking-vs-additive policy); the register
> row enumerates the real surface by group and names its exclusions out loud, `Routine`
> first. On the way to the badge, D2 ruled on all nine items of D1's facade critique plus
> its own thirteen docket questions — time is **retyped** to `units::Time` with the full
> suite output byte-identical, `wait(units::Time)` is **adopted** as an additive void verb,
> `brake`/`hold` **join** the freeze — and the mutation campaign found **two green holes**
> (a pin blind to noexcept-dropping; a call site blind to a thousand-fold duration mistake),
> both closed with sole-detector tests.

Suite: **690 cases / 916,050 assertions** green under strict `-Werror` (3 pre-existing
skips unchanged). Baseline entering D2: 686 / 916,026 (D1 exit). Both CI guards pass; all
**104** v2 headers ARM-cross-compile clean (103 + `version.hpp`); the four-term
removability gate is empty. 44 mutations executed and observed (§12): 41 red where red was
required, 2 green survivors that were genuine holes (found, closed, re-run red by exactly
the one new detector each), 2 invariant-preserved greens explained rather than excused
(§10.3), and 1 hypothesis-confirming green (non-copyability is doubly structural). The
guide's verbatim-coupling scan reads **zero drifted lines** across all chapters.

The full decision record — every ruling, timestamped, in the order it was made — is
`D2-PROGRESS.md` beside this file. This record is the curated version; the log is the
evidence of how it happened, including the mid-chunk pause and resume.

---

## 1. What was ruled, built, and changed

**The rulings (the chunk's real product, §2):** 9/9 items of D1-COMPLETED §2 ruled; the
docket's A1–A4 (what exactly freezes), B1–B3 (the now-or-never changes), C1–C6 (the
deliberate confirms) all ruled, each with its rejected alternative recorded.

**New header (1):** `include/shulib/version.hpp` — `kApiMajor = 2`, `kApiMinor = 0`,
`kApiVersionString = "2.0"`, and the written policy for what a breaking change vs an
additive change *is* for a header-only C++ API (§5). This is the mechanism behind the
register's "changes only with an API-version bump and a migration path" sentence, which had
no machinery behind it for five locked rows.

**New test file (1):** `test/f6_signature_pin_test.cpp` — 36 compile-time pins covering
every frozen member and the frozen semantics (§6), plus the runner-visible case pinning the
version constants.

**Facade changes (`chassis.hpp`) — the two sanctioned constructions, nothing else:**

- **The B1 retype:** `MotionOptions.timeoutSeconds : double` → `MotionOptions.timeout :
  units::Time` (deliberately *renamed*, not just retyped, so no call site could survive
  unexamined); `hold(units::Time)`; `waitUntil(pred, units::Time)`. The facade converts to
  the scheduler's interior seconds with `.value()` at the motion boundary — the identity,
  because F3 makes `Time` canonical seconds (§3).
- **The B2 verb:** `void wait(units::Time)` — commands nothing, advances the world,
  Warn-free by construction, watchdog-slack bounded (§4). `kWaitBackstopSeconds` moved here
  from `Routine` with its deliberate not-an-HA-entry rationale intact.
- The header banner: `STATUS: BUILT, DELIBERATELY NOT FROZEN` → `STATUS: FROZEN — F6,
  LOCKED 2026-08-12`, with the enforcement mechanism and all three exclusions stated.

**Recipe-layer changes (`routine.hpp`):** `hold`/`pause`/`waitFor` retyped to match
(unfrozen surface, but the raw-double carve-out dies everywhere at once); `pause()` is now
a **pure delegation** to `Chassis::wait` — before D2 it was the one step carrying its own
Tier-3 clock-deadline plumbing, so "every step is exactly one Chassis call" is true again;
a new `STATUS: DELIBERATELY OUTSIDE F6` banner (silence in `chassis/` reads as frozen —
ruling A2 applied to the file itself, not only the register row).

**Docs:** the F6 register row rewritten and flipped (§8); the roadmap's "you are here" and
M2/M7 status text updated in the same pass; guide chapters 08/09/10/12/14 + README +
`docs/guide/README.md` re-quoted and swept (§9); root `README.md` example retyped;
`guide-maintenance.md` D2 row marked executed.

**Test changes:** ~100 call sites across 6 test files converted by the retype (stage 1,
before any new test landed); new `wait()` behavior + misuse cases; the constraint-4
absolute-duration pin; `hold`/`wait`/`waitUntil` + `Routine::hold`/`pause` bare-number
compile-rejection pins; the C1-hole clock-bound detector in `guide-09a`.

**No motion-layer changes. No scheduler changes.** The scheduler's interior
`waitUntil(pred, double timeoutSeconds)` signature is deliberately untouched — that is
F3's internal-seconds convention, not public surface (§3).

---

## 2. THE DECISION DOCKET — every ruling, with its rejected alternative

> The centrepiece. This is what someone reads in two years to learn why the API is shaped
> the way it is. Format per ruling: **the ruling**, the reasoning, and **the rejected
> alternative** — including the rulings that simply accept D1's recommendation, because
> *"accepted, having checked X"* is a decision and silence is not. The D1 §2 item each
> ruling answers is noted inline; §2.D maps all nine.

### 2.A What, exactly, is frozen

#### A1 — The row ENUMERATES the surface by group and NAMES the exclusions

The F6 row now lists the whole surface: construction (both ctor spellings +
non-copy/move), the blocking verbs (`moveTo`/`strafeTo`/`turnTo`/`followTrajectory` span +
brace/`brake`/`hold`/`wait`), the manual verb (`drive(ChassisSpeeds, Frame)`, `Frame`
required), control (`cancel`/`waitUntil`), state (`pose`/`setPose`/`strafeAuthority`/
`lastExitReason`/`lastCompleted`/`motionConfig`), the Tier-3 seam (`deps()`/`scheduler()`
both overloads), the three public types, and the documented semantics the header carries
(blocking + watchdog, pre-empt, cancel safe state, fault policy, wait-for-live,
strafe-fallback visibility). It then names what is *not* frozen (A2/A4 below).
**Rejected: keep the five-verb row.** A row that lists five things while the header exposes
twenty fails the brief's own test — "nobody can later say what was frozen" — and turns the
LOCKED badge into a liability precisely for the people it exists to serve.

#### A2 — `Routine` does NOT join F6, and the exclusion is stated out loud

Accepted D1 §12's recommendation, having checked three things: (a) D3's cookbook is
`Routine`'s second consumer and may still reshape step spellings — freezing now would
repeat the exact mistake the C4→D1→D2 order was built to avoid (freezing before a second
consumer); (b) `Routine` delegates only, so freezing the facade already fixes everything
recipes rely on — a recipe's *behavior* is frozen even while its *spelling* is not; (c) the
B1 retype lands in `Routine` now anyway, so D3 inherits typed time rather than a carve-out.
The exclusion is stated in the register row, in `chassis.hpp`'s banner, and in a new
`routine.hpp` STATUS banner — because `Routine` lives in `chassis/`, where silence reads as
"frozen too". **Rejected: freeze `Routine` as well** — it would lock spellings the cookbook
has not yet consumed, for no gain the facade freeze doesn't already deliver.

#### A3 — `ChassisConfig` / `MotionOptions` / `TrajectoryResult` ARE frozen, with carve-outs stated

A frozen signature over an unfrozen type freezes nothing, so the three types that appear in
frozen signatures freeze too — with the carve-outs said precisely: `MotionOptions`'
*existing* fields are frozen (name/type/meaning) while the field *set* is additive-open
(C4 D10's design — post-freeze knobs are new fields with a 0/"config default" meaning);
`ChassisConfig`'s two members are frozen but the member types' own fields belong to their
layers (the additive path C6 verifies); `TrajectoryResult` is fully frozen including
`succeeded()`. **Rejected: a signatures-only freeze** — renaming `MotionOptions.timeout`
would then "break no frozen signature" while breaking every call site in existence.

#### A4 — The Tier-3 seam (`deps()`/`scheduler()`) is INSIDE F6; the layers behind it are not

Accepted C4 §8's argument: freezing the facade without the seam would freeze users *out* of
the lower layers — the no-ceiling promise is load-bearing, so the accessors' existence and
exact types freeze. The named exclusion: `MotionScheduler`'s and `MotionDeps`' own member
surfaces stay outside F6 — those are C1/C2's layers, and claiming them here would freeze
two whole layers through the back door. Under-claim per the brief's constraint 1.
**Rejected: pull the seam types' surfaces into F6** — a freeze should never annex territory
its review never examined.

### 2.B The now-or-never changes

#### B1 — ADOPT the time retype: `units::Time` joins the typed surface everywhere *(rules D1 §2.1)*

Time was the one untyped physical dimension on a surface whose entire misuse story is
"wrong units do not compile". `hold(500)` from someone thinking in milliseconds *compiled*
and held pose for 500 s of a 15 s match — a match-loss the type system was born to
prevent. Ruled: `MotionOptions.timeoutSeconds` → `units::Time timeout` (**renamed** so no
call site survives unexamined — a type-only change with the old name would have let
`{.timeoutSeconds = x}` sites fail with a confusing message instead of an honest
"no such field"); `hold(units::Time)`; `waitUntil(pred, units::Time)`; `Routine`'s
`hold`/`pause`/`waitFor` retyped to match even though unfrozen, so the carve-out dies
everywhere at once. The facade converts at the motion boundary with `.value()` — F3 makes
`Time` canonical seconds, so the conversion is the identity and bit-preservation is by
construction; §3 shows it was also *proven*, not just argued. The `units::Time` constructor
is `explicit` (verified at `quantity.hpp:33` before ruling), which is what makes this a
compile-caught migration rather than a silent one. **Rejected: keep raw doubles and
document the carve-out** — "decide later" was the same as deciding never (the brief's own
framing), and it would have enshrined the one untyped dimension permanently.

#### B2 — ADOPT `wait(units::Time)`, an additive facade verb returning void *(rules D1 §2.2)*

D1's finding, verified: the "sit still for the alliance partner" beat is universally
needed, the naive spelling `waitUntil([]{ return false; }, t)` logs a spurious Warn on
every deliberate pause (the transcript lies about intent), and the Warn-free spelling
required Tier-3 plumbing plus a sacrificial timeout — a Tier-2 concept needing Tier-3
plumbing, which the no-cliff constraint forbids. Ruled: `void wait(units::Time)`. Void
because a wait has no failure mode short of a stalled pacer, which trips the scheduler's
loud precondition — a programming error, not a verdict; returning `ExitReason`/`WaitResult`
would invite dead branches on an unreachable timeout (and a fourth result vocabulary, see
C4). Precondition finite && > 0, consistent with `hold`/`pause` — a computed possibly-zero
wait belongs at `waitUntil`'s tier, where 0 is an honest poll. `Routine::pause` became a
pure delegation, restoring "every step is exactly one Chassis call";
`kPauseBackstopSeconds` moved down as `kWaitBackstopSeconds`, same value, same deliberate
non-HA rationale. **Rejected: name it `sleep`** (implies thread semantics the single-task
model does not have) **and rejected: decline + bless the predicate idiom** (leaves every
team rediscovering the Warn problem one auton at a time).

#### B3 — ADOPT `brake`/`hold` into F6 *(rules D1 §2.6)*

Accepted D1 §2.6, having checked: every complete D1 routine used both; parking via the
Tier-3 seam would be a Tier-2 cliff (§17 forbids); C4 D9 built them precisely for this
decision. `hold` retyped per B1. **Rejected: delete before freeze** (C4's stated fallback)
— an auton surface that cannot park is incomplete, and D1's usage evidence is exactly the
second-consumer signal the ordering was designed to produce.

### 2.C The accepts — confirmed deliberately, not inherited

#### C1 — CLOSE the runtime-`abortFaultMask` deferral *(rules D1 §2.3; closes C2 §11 flag 6)*

Mask is construction-time only at F6. Checked: no D1 test or step design ever reached for
runtime mutation; the expert path (construction config + Tier-3 seam) exists; C2's Phase-R
fault-statistics revisit stands unchanged. Closing forecloses nothing — a setter would be
ADDITIVE later. **Rejected: add a setter now** — speculative surface in a freeze chunk.

#### C2 — KEEP the branch-on-strafe-fallback rejection *(rules D1 §2.4; closes C3 §11 flag 1)*

Checked the precise D1 finding: a `waitUntil` predicate cannot see the fallback either (no
accessor exists, by design), so it is unreachable at *every* tier — not merely awkward at
Tier 2 — and `strafeAuthority()` covers authoring-time budgeting. Mid-leg reaction to
fallback is re-planning, which is G2+ territory. The rejection is additively reversible (a
getter could be added later), which is exactly why it need not be reversed now.
**Rejected: add a fallback getter into the freeze** — a live-polled bool invites
control-flow coupling to a telemetry concept (C3 §11's original reasoning, still sound).

#### C3 — Tank field-vocabulary sugar STAYS in the recipe layer *(rules D1 §2.5)*

`face`/`driveTo` are argument sugar (bearing computed at step-run time), bit-identical to
the hand idiom (pinned by D1 M4/M5/M19), and the *unfrozen* recipe layer can still iterate
on spellings before D3's cookbook. **Rejected: facade `turnToFace`/`moveToPoint`** —
freezes sugar spellings before the cookbook has shaped them, on the wrong tier.

#### C4 — ACCEPT the three result vocabularies; FLAG for F2 *(rules D1 §2.7)*

`ExitReason` / `TrajectoryResult` / `WaitResult` are each right locally; only spanning
consumers pay, and D1 absorbed the cost in ~30 lines. `wait()` returning void deliberately
adds NO fourth vocabulary. FLAGGED: F2's combinators span all three — that designer must
read D1 §2.7 first. **Rejected: unify into one result type at the freeze** — it would
flatten `TrajectoryResult.completedLegs` (strategy-relevant) and re-mean `WaitResult`
(whose timeout is a branch, not a failure) to buy uniformity nobody asked for.

#### C5 — All four C4 §8 tensions CONFIRMED no-change, each with its additive path named *(rules D1 §2.8)*

Facade-level async → behind `scheduler()` until F2 (D1 §3's eager-vs-deferred analysis
already closed it); `hold` disturbance radius → additive `MotionOptions` field if the
R-phase shows need; per-leg trajectory results → `TrajectoryResult` sufficient at Tier 2,
per-leg detail is C5-record material; per-verb settle tolerance → additive `MotionOptions`
field. Every "no" here is safe *because* its additive path exists — that is constraint 2
applied four times. **Rejected: pre-building any of the four** — speculative surface again.

#### C6 — The `maxWheelSpeed` additive path, CHECKED STRUCTURALLY *(closes C3 §11 flag 4's D2 clause)*

`ChassisConfig.motion` is the whole `MotionConfig`; the facade stores it by whole-struct
copy (`cfg_{config.motion}`) and `effectiveConfig` starts from a whole-struct copy — grep
confirms no field-by-field `MotionConfig` marshalling anywhere in `chassis.hpp` or
`routine.hpp`, so a future per-wheel field flows through with **zero facade reshape**; a
per-call override is additive via a new `MotionOptions` field plus one `effectiveConfig`
line. R5's measurement deferral unchanged (C3 D13). **Rejected: take C4's word for it** —
"the freeze review checks the additive path exists" was written as an instruction to check,
and a freeze review that checks structurally is the only kind worth having.

### 2.D The nine D1 §2 items, mapped

| D1 §2 item | Ruling | Verdict |
|---|---|---|
| 2.1 time untyped | B1 | **ADOPT** the retype |
| 2.2 no wait verb | B2 | **ADOPT** `wait(units::Time)` → void |
| 2.3 runtime mask deferral | C1 | **CLOSE** (construction-time only) |
| 2.4 branch-on-fallback | C2 | **KEEP** the rejection |
| 2.5 tank sugar placement | C3 | **ACCEPT** (recipe layer keeps it) |
| 2.6 brake/hold adoption | B3 | **ADOPT** into F6 |
| 2.7 three result vocabularies | C4 | **ACCEPT** for F6, **FLAG** for F2 |
| 2.8 the §8 tension list | C5 | **CONFIRM ×4**, additive paths named |
| 2.9 what was good to build on | — | Evidence absorbed into A1's grouping and §8's row |

---

## 3. The time retype (B1) — behaviour-preserving, and proven so

Constraint 3 set the bar: not merely green, but **identical numbers**. The method was C3's
bit-identity method, staged so nothing else could contaminate the comparison:

1. **Baseline capture** before any change: full suite output (686 / 916,026), including
   the hostile-IMU drift MESSAGE digits — the most conversion-sensitive printed numbers in
   the suite.
2. **Stage 1 = the pure retype alone** (headers + all ~100 call sites; no new tests, no
   new verb). Suite re-run: output **byte-identical** to the baseline, `diff` clean. The
   retype moved no number — as it must, since `Time` is canonical seconds (F3) and
   `.value()` at the motion boundary is the identity.
3. **Stage 2 = `wait` + pause delegation**: output *still* byte-identical (pause rides the
   same `sched_.waitUntil` machinery one layer down).

Deliberate scope edge: `run_report_e2e`'s `sched.waitUntil(pred, seconds)` call sites kept
raw doubles — that is the scheduler's interior signature (F3 internal-seconds), not the
frozen facade edge. The public/interior line is now: **typed `units::Time` at every public
facade parameter; seconds-double inside the motion stack; `.value()` exactly once, at the
boundary.**

Misuse is now rejected at compile time on both tiers (`hold(500)`, `wait(500)`,
`waitUntil(pred, 0.5)`, `Routine::hold(0.3)`, `Routine::pause(0.5)` — all
`static_assert`-pinned as non-compiling), and the conversion boundary itself is pinned
against hand-computed absolutes, not sibling literals (§10.2, §12 B-series).

---

## 4. `wait()` (B2) — the verb, and where its old implementation went

`Chassis::wait(units::Time)` blocks while commanding nothing: the world advances (sensors,
health, any active motion keep ticking — same contract as `waitUntil`), the drive keeps
whatever state the last verb left it in, nothing is logged, no fault is raised. The
deadline predicate is time-monotone, so the internal `waitUntil` backstop
(`duration + kWaitBackstopSeconds`) is unreachable slack, never a reachable timeout — the
Warn-free property is by construction, not by suppression. Behavior pinned by three new
cases (§11): commands-nothing/advances-exactly/silent-faultless; zero/negative/NaN throw;
and the 2500 ms absolute-clock pin. `Routine::pause` delegates; its D1 tests (stays put,
stays quiet, continues) now exercise the facade verb through the chain, and B3's
backstop-to-zero probe (§10.3) confirmed the slack claim experimentally.

---

## 5. The version mechanism — the register's promise, made real

The register has said since M0 that LOCKED contracts change "only with a
`schemaVersion`/API-version bump and a migration path" — and until D2, `grep` for any
version identifier over `include/` and `src/` returned nothing. F1–F5 were locked against
a promise with no machinery. `include/shulib/version.hpp` is now that machinery:

- **BREAKING (bump `kApiMajor`)**: any change that can make a previously-compiling program
  fail to compile or silently change meaning — remove/rename a frozen member or type;
  change a frozen signature's parameter, return, const, ref, or **noexcept** shape; change
  documented semantics a frozen surface carries; re-mean or renumber an existing
  enumerator. A major bump REQUIRES a migration note next to the register row: old
  spelling, new spelling, the mechanical rewrite, and why the break was worth it.
- **ADDITIVE (bump `kApiMinor`)**: every previously-valid program compiles unchanged with
  unchanged behaviour — new members, new overloads, new options-struct fields whose
  default preserves the old behaviour, appended enumerators.

The surface frozen at D2 is **API 2.0** (`kApiMajor = 2` deliberately matches "the shulib
v2 rebuild" so the number means something to a human). Scope discipline honored: the
policy covers **API** freezes; F7/F8/F9 freeze *data* and carry their own `schemaVersion`
fields inside the artifacts (designed at Phase G/H), governed by this same
breaking-vs-additive vocabulary — noted in the header, not designed there.

---

## 6. The signature pin — and the two lessons it taught

36 compile-time pins in `test/f6_signature_pin_test.cpp`, one uniform macro
(`SHULIB_F6_PIN`) whose failure text names F6, the lock date, and the exact procedure for
an *intended* break (read `version.hpp`; bump; migration note; register row; pins last).
Coverage was cross-checked member-by-member against `chassis.hpp`: all 20 public `Chassis`
members (both ctor spellings, non-copy/move, 8 verbs, `cancel`, `waitUntil`, 6 state
accessors, `deps`, both `scheduler` overloads), all 10 type members across
`ChassisConfig`/`MotionOptions`/`TrajectoryResult`, the terse defaulted spellings, and one
**negative** pin (below). Additive extension stays legal by design: pins assert "the frozen
shape still exists", never "no new shapes may exist" (`static_cast` overload selection, not
`decltype(&member)`, so a future additive overload cannot break the pin).

### 6.1 Lesson 1 — every member pin is a concept templated on the class

The first pin draft used non-template requires-expressions. Proof #1 (reshape `hold`)
failed the build — but with a raw "invalid static_cast" diagnostic at the pin site, not
the named F6 message: in a **non-dependent** context an invalid cast inside a
requires-expression is a hard error, and the assert never got to speak. The redesign
routes every member pin through a **concept templated on the class** (`F6MoveTo<Chassis>`
etc.), making any reshape a substitution failure in a dependent context: the concept
quietly evaluates false and the named static_assert fires. Field pins use
`decltype(T::field)` inside a concept for the same reason — a renamed field must produce
the F6 message, not an undeclared-identifier error. The bar was the brief's own sentence:
*fails the build with a message naming F6* — and "fails the build" alone is not that.

### 6.2 Lesson 2 — the noexcept blindness (campaign hole #1, found by mutation)

Campaign mutations A16 (`lastExitReason` loses `noexcept`) and A31 (`succeeded()` loses
`noexcept`) **stayed green** against the exact-cast pins: for a *non-overloaded* member,
the compiler accepts the `static_cast` that *adds* noexcept, so the cast cannot see
noexcept being *dropped*. (`scheduler()` was caught anyway — it is overloaded, and
target-type matching over an overload set refuses a potentially-throwing candidate, an
asymmetry worth knowing.) Since §5's policy explicitly lists a noexcept change as
BREAKING, this was a real enforcement gap: a future chunk could have un-noexcept'd the
whole observability surface without a single red line. **Closed:** all 7 noexcept-carrying
pins now pair the cast with a compound requirement `{ call } noexcept`, which observes the
actual call's noexcept-ness and cannot be fooled. Re-run: all six single-overload
noexcept-drops (A16b/A31b/A34–A37) now RED with the named message, the compound
requirement being the sole detector.

### 6.3 The negative pin

`drive(speeds)` without a `Frame` must NOT compile — a *frozen semantic*, not a frozen
signature: adding a default `Frame` would change no member-pointer type (additives are
normally legal) yet would reopen the silent-frame-assumption bug class this rebuild exists
to prevent. Pin 36 asserts the frameless call stays uncompilable, with the F6 message;
campaign mutation A10 (give `Frame` a default) is caught by this pin and nothing else in
the TU.

---

## 7. The 18-row ledger, re-checked (C4 §2's instruction to D2, executed)

C4 wrote *"anything missed here becomes permanent at D2"*. Every row was re-checked
against the header as frozen — in code, this session, not from memory:

**From C2 §11 (eight):** (1) verbs are still `async`+`waitUntilSettled` via `runBlocking`;
`waitUntil`/`cancel` re-exported — the retype typed the *facade edge* only, the
scheduler's interior seconds-double signature is untouched (F3); `tick()`+counters
reachable via `scheduler()`. (2) Pre-empt in `drive()` (cancel-if-active) verbatim.
(3) `WaitResult` re-exported unmodified. (4) `ExitReason` unmodified, no facade enum.
(5) Caller-paced `ITickPacer` in the ctor; no background task anywhere. (6) Mask is
construction-time via `ChassisConfig.scheduler` — the runtime-mutation deferral this row
carried is the one ruling C1 CLOSED. (7) Stamping structural: every verb builds from
`sched_.deps()`; no raw-deps path in the facade. (8) No new `IMotion` member needed at D2
either — two chunks in a row, the C1/C2 contract set has sufficed.

**From C3 §11 (five):** (1) no fallback getter exists — ruling C2 kept the rejection.
(2) `strafeAuthority()` passthrough intact. (3) turn-while-drive semantics carried in the
banner (now part of the *frozen* documented semantics). (4) `maxWheelSpeed` additive path
re-verified structurally — ruling C6's grep. (5) drivetrain-as-config via
`MotionDeps.kinematics`, no facade drivetrain enum.

**From C1 §11 (five):** (1) borrow-exactly-`MotionDeps`, own nothing below it — the ctor
unchanged. (2) zero motion ticking in the facade. (3) `MotionState` untouched; `drive()`
records id 0. (4) saturation choreography still the one `command_pipeline.hpp` copy.
(5) wait-for-live semantics documented + `drive()`'s boot gate intact.

Verdict: **18/18 rows hold at the freeze.** The three rows that carried open deferrals
aimed at D1/D2 (C2 #6, C3 #1, C3 #4) are precisely the ones the docket ruled (C1, C2, C6)
— the ledger's forwarding mechanism worked as designed.

---

## 8. The register row, rewritten, then flipped

Order mattered and was honored: the row became *accurate* first (A1's enumeration, A2's
`Routine` exclusion stated, A3's carve-outs, A4's seam boundary), the pin was *proven*
(§6, §12), the sweep was *complete* (§9) — and only then did the badge flip:
**✅ LOCKED 2026-08-12.** The row now carries the enforcement pointers (pin file + version
header) so a reader of the register can find the mechanism from the promise in one hop.
The roadmap's "you are here" F6 sentence, the M2 checkbox annotation, and the M2/M7 status
paragraphs were updated in the same working-tree state — a LOCKED row beside a
"deliberately NOT frozen" narrative would have been the two-contradictory-claims state the
brief's constraint 5 exists to prevent.

---

## 9. The documentation sweep — the guide re-quote and the four notices, atomic

**The drift (found at the pause, fixed first on resume):** the B1 rename invalidated 21
```cpp lines across chapters 08 (5), 09 (12), 10 (4) — the chapters quote
`guide_examples_test.cpp` *verbatim*, and the retype had made them lie. All 21 re-quoted
**from the test file** (never hand-edited toward agreement — the top mistake in
`guide-maintenance.md`). The mechanical scan (every non-blank ```cpp line must appear in
the test file) reads **zero** across all chapters. Prose the scan cannot see was swept by
grep: ch. 08's two inline `.timeoutSeconds` quotes + typed-units sentence (now names
`_s`/`_ms`), ch. 09's options prose and steps table (including the now-false "pause
delegates to `waitUntil`" → `wait`), ch. 10's "the one plain-double exception" paragraph
(the exception is dead; kept as *history*, deliberately), ch. 12's "raise `timeoutSeconds`"
fix-advice, and the root README's four-line example. Transcripts needed **no** re-capture:
stages 1–2 proved the suite's full output byte-identical, so every printed digit in every
excerpt is still a real captured digit.

**The wait verb entered the guide** (ch. 10 gained a `wait(duration)` section; ch. 09's
table already had `pause`) — the API chapter claims to cover every public operation, and
after stage 2 it silently didn't.

**The notice sweep (atomic):** `chassis.hpp` banner → FROZEN with mechanism + exclusions;
`routine.hpp` → new DELIBERATELY-OUTSIDE-F6 banner; ch. 09 notice → facade frozen /
`Routine` deliberately unfrozen until D3, stated in both directions (the subtle distinction
the brief flagged: what a step *does* is settled, what it is *called* is not, and the
second-consumer rule is named as the reason); ch. 10 notice → frozen, with the pin and the
version policy linked; ch. 14 → the "API is not frozen" limitation *fell* (retitled to the
one thing still open, the recipe spellings); guide README check-order → the breaking-change
procedure now, the D3 softening later; `guide-maintenance.md` D2 row → executed. Gate grep
across all five locations: **0** stale not-frozen claims; every surviving "unfrozen" is the
deliberate `Routine`-until-D3 statement.

---

## 10. Findings

### 10.1 HOLE #1 (the pin): noexcept-dropping was invisible on non-overloaded members

Found by campaign mutations A16/A31 staying green; mechanism, closure, and re-run evidence
in §6.2. The general lesson for F7/F8/F9's future enforcement tests: **an exact-type cast
is not an exact-type test** — pin properties that conversions can erase (noexcept here;
cv-qualification conversions are the analogous trap elsewhere) with a detector that
*observes* the property, not one that merely *names* it.

### 10.2 HOLE #2 (a call site): a thousand-fold duration mistake survived every assertion

Campaign C1 mutated `guide-09a`'s call site `.hold(300_ms)` → `.hold(300_s)` — the exact
author mistake B1 exists to make uncomfortable, at 1000× — and the case stayed **green**:
it asserted outcomes (ok / step counts / final pose on ground truth) but never the clock,
so a first-auton demo that takes 308 s of a 15 s match passed. This is constraint 4's
"same literal on both sides" blindness appearing at a *call site* rather than a conversion
boundary. **Closed** with a sole detector inside the `guide-09a` case (outside every quoted
listing — drift scan re-verified 0): the whole recipe must finish in **< 12.0 s of
simulated clock**, a hand-computed bound (honest run ≈ 8 s; the mutation ≈ 306 s). Proven
green-honest / red-mutated with the new check as the only failing assertion, then
restored. The detector doubles as an honest product claim: the guide's first complete
recipe fits a match window with slack.

### 10.3 Invariant-preserved greens — explained, not excused

- **A33** (`Chassis(const Chassis&) = delete` → `= default`): green **as hypothesized
  before running** — the `MotionScheduler` member is non-copyable, so the copy ctor is
  implicitly deleted anyway and the pinned *property* survives. Non-copyability is doubly
  structural; the explicit deletes are documentation.
- **B3** (`kWaitBackstopSeconds` 1.0 → 0.0): green — the deadline predicate wins the
  same-tick tie, so the backstop is unreachable slack *even at zero*. The header's "any
  value that clears one tick behaves identically" is conservative; the experiment
  strengthens the non-HA-entry rationale (no behavior depends on the value at all).

### 10.4 Process finds

- **The pause-note truncation:** the original pause section said chapters "08 and 09" had
  drifted — written off a truncated grep listing showing 12 of 21 hits; chapter 10 had
  drifted too. Corrected in the log before resume work began. Lesson: inventory claims in
  a handoff note deserve the same re-verification as any other claim.
- **The drift the scan cannot see:** the verbatim scan finds code-block drift only. The
  resume inventory found six *prose* sites quoting the dead spelling plus one *behavioral*
  doc lie (ch. 09's "pause delegates to `waitUntil`") that no mechanical check watches.
  A prose-grep for renamed identifiers belongs in every future rename's checklist.
- **The lock date is the flip date:** the pin macro and `version.hpp` initially said
  2026-08-11 (the session date); the badge flips 2026-08-12. Aligned to the flip date —
  the freeze's timestamp should be the moment the promise took effect.

---

## 11. Test inventory (new at D2; every case names its bug in-file)

**`test/f6_signature_pin_test.cpp` — 36 compile-time pins + 1 runner case:**
construction ×4 · verbs ×8 (incl. `wait`) · `drive`+`cancel`+`waitUntil` ×3 ·
negative frameless-`drive` ×1 · state ×6 · seam ×3 · type members ×10 · terse spellings ×1;
the runner case pins `kApiMajor == 2` / `kApiMinor == 0` / `"2.0"` so the version
mechanism cannot silently vanish or drift from the frozen surface's own claim.

**`test/chassis_facade_test.cpp` additions:**
- 11 new compile-rejection pins (`hold`/`wait` × double/int/Time; `waitUntil` ×
  double/Time; positive spellings alongside — the negative tests ARE the assertions).
- *D2 wait: commands nothing, advances exactly the window, stays silent* — motors at the
  settled 0 V, no travel, elapsed in [0.7, 0.75) (one tick of slack, not the backstop), no
  Warn, no fault.
- *D2 wait misuse: zero, negative, NaN throw* — `PreconditionError` all three.
- *D2 duration pin: 2500_ms is 2.5 s of simulated clock, absolutely* — constraint 4's
  test: `static_assert` on the conversion constants + runtime elapsed in [2.5, 2.55)
  against the hand-written absolute, never a sibling literal.

**`test/chassis_recipe_test.cpp` additions:** 5 compile-rejection pins
(`Routine::hold`/`pause` × double/int/Time).

**`test/guide_examples_test.cpp` addition:** the §10.2 clock-bound detector in
`guide-09a` (whole-recipe elapsed < 12.0 s, hand-computed).

Suite delta over the chunk: 686 → 690 cases; 916,026 → 916,050 assertions. (The retype
itself added zero cases by design — stage 1 was proven on an unchanged suite.)

---

## 12. Mutation campaign (44 executed: break → build-gate → run → OBSERVE → restore)

Method: Category A ran as a scripted driver compiling **only the pin TU** with the exact
suite flags — the compile is the build, and with no runner there is no stale-binary risk
(the C4 §4.6 / D1 lesson, sidestepped structurally). Categories B/C rebuilt the full
suite with the runner gated on build success. Every mutation restored from memory;
`chassis.hpp` and `literals.hpp` verified sha256-identical to pre-campaign afterwards;
pristine compile re-verified after every restore.

**Category A — every frozen member reshaped in turn (39 executions):**

| Mutations | Result |
|---|---|
| A01 ctor default dropped · A02 `moveTo` by-value · A03 `strafeTo` y→double · A04 `turnTo`→double · A05 span loses const · A06 brace overload deleted · A07 `brake` renamed · A08 `hold` Time→double · A09 `wait` Time→double · A11 `cancel` gains param · A12 `waitUntil`→double · A13 `pose` loses const · A14 `setPose` by-value · A15 `strafeAuthority`→float · A17 `lastCompleted` by-value · A18 `motionConfig` loses const · A19 `deps` by-value · A20 `scheduler` loses noexcept (overloaded — caught) · A21 const overload deleted · A22 `timeout` renamed · A23 `maxLinearSpeed`→double · A24 `maxAngularSpeed`→Velocity · A25 `validate` loses const · A26 `ChassisConfig::motion` renamed · A27 `::scheduler` renamed · A28 `exit`→int · A29 `completedLegs`→long · A30 `totalLegs`→long · A32 `brake` default dropped (terse pin) | **RED + NAMED** (29) |
| A10 `drive` gains a **default Frame** | **RED + NAMED** — caught by the negative pin alone (§6.3) |
| A16 `lastExitReason` loses noexcept · A31 `succeeded` loses noexcept | **GREEN → HOLE #1** (§6.2, §10.1) |
| A16b/A31b/A34/A35/A36/A37 — all six single-overload noexcept-drops vs the strengthened pin | **RED + NAMED**, compound requirement the sole detector |
| A33 copy-ctor delete→default | **GREEN, hypothesis confirmed** (§10.3) |

**Category B — the conversion boundary (3):**
B1 `_ms` factor /1000→/100 → build FAILS at the constraint-4 `static_assert`
((2500_ms).value() == 2.5). B2 same mutation with both static_asserts disabled → build ok,
runtime **RED** (`25 < 2.55` fails) — the clock-absolute detector is non-vacuous alone.
B3 backstop→0.0 → **GREEN, invariant-preserved** (§10.3).

**Category C — call-site units (2):**
C1 `.hold(300_ms)`→`.hold(300_s)` → **GREEN → HOLE #2** (§10.2); after the detector:
**RED** (`306.01 < 12` the only failure), restored. C3 `guide-08c` starved budget
0.5_s→5_s → **RED 3 ways** (TimedOut expectation, fault count, transcript) — the
starved-budget case is not vacuous.

Tally: **44 executed, 44 observed.** 41 red where the freeze required red (after
closures); 2 genuine holes found green and closed with sole detectors; 2
invariant-preserved greens explained; 1 hypothesis-confirming green. Both holes got the
prominent treatment the brief demands (§10.1, §10.2) — they are the campaign's product,
not its embarrassment.

---

## 13. What we now know for certain, and what we do not

**Certain (proven this chunk):**
- The frozen surface cannot be reshaped, member by member, without the build failing with
  a message that names F6 and the member — proven by 36 observed mutations, not asserted.
- The retype moved nothing: byte-identical full-suite output, twice (stage 1, stage 2).
- A bare number is not a duration anywhere on either tier — compile-rejection pinned.
- `2500_ms` is 2.5 s of simulated clock, held against a hand-computed absolute with two
  independent detectors (compile + runtime), each proven able to fire alone.
- The guide quotes the compiled examples verbatim (scan: 0), and the first complete recipe
  fits a match window with slack (< 12 s, clock-asserted).
- All 18 inherited-shape ledger rows hold at the freeze; the three that carried deferrals
  are closed by name (C1/C2/C6).

**Not known / deliberately open:**
- `Routine`'s spellings — unfrozen until D3's cookbook consumes them (A2). The register
  row, both header banners, and three guide chapters say so out loud.
- Everything hardware: the freeze is a *source-compatibility* promise, not a field
  promise. Nothing here has run on a robot; the accuracy numbers remain simulation
  numbers under modeled hostility (unchanged claim, restated so this record cannot be
  read as more than it is).
- F2's combinator design must read D1 §2.7 before spanning the three result vocabularies
  (flag C4); F7/F8/F9 reuse §5's policy vocabulary but design their own `schemaVersion`
  carriage at Phase G/H.
- The noexcept lesson (§6.2) is recorded for the future data-freeze enforcement tests;
  whether the analogous cv-conversion trap bites there is unexamined — the next freeze
  author should check it deliberately.

**The one-sentence verdict:** the facade survived its second consumer, its critique was
ruled on item by item rather than inherited, and the freeze that resulted is enumerated,
mechanized, and mutation-proven — F6 is locked because it earned it, and the badge now
means exactly what the register says it means.
