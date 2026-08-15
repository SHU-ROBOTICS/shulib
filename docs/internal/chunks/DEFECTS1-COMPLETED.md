# DEFECTS1 — COMPLETED (2026-08-15)

> Triage and resolution of the API defects DOCS2 reported and deliberately left in place.
> Written FROM [`DEFECTS1-PROGRESS.md`](DEFECTS1-PROGRESS.md) (the live log), not instead of it.
> Predecessor: DOCS2. Successor: **the release to `main`**, then R3.
> Brief: [`DEFECTS1-api-defect-triage.md`](DEFECTS1-api-defect-triage.md).
> Input, now annotated in place with every outcome:
> [`DOCS2-API-DEFECTS.md`](DOCS2-API-DEFECTS.md).

## The one-sentence honest status

**84 items triaged and closed — 59 FIX, 15 REJECT, 6 ARGUE, 4 DEFER** — with 21 mutations run
and 19 observed red; but **two of the three green mutations were my own tests failing to test**,
one fix (`I13`) was **retracted** after the suite proved it wrong, and one (`D12`/`D14`) is
applied and **not pinned by anything**, which is stated here rather than smoothed over.

## What the list actually was

Three populations, mixed on purpose and separated here:

| | Count | |
|---|---:|---|
| **FIX** | 59 | real, and fixed |
| **REJECT** | 15 | refuted, already fixed, or a recorded decision |
| **ARGUE** | 6 | real; the fix is breaking or needs a ruling — written up below, **not applied** |
| **DEFER** | 4 | real; owned by a later chunk |

**Six of the fifteen REJECTs were already fixed at DOCS2 itself** (`D3` `D7` `D10` `I1` `I11`
`O1`) — verified on the generated pages, not taken on trust. A chunk that had started fixing
instead of triaging would have "fixed" them.

**`A9` is the one that justifies the whole triage step.** Its evidence makes two claims about
`IClock` and a two-line probe refutes both: `IClock c = someProsClock;` does **not** compile
(the type is abstract), and the assignment that does compile discards **nothing** —
`sizeof(IClock)` is 8, the vptr alone, and both clocks keep their values. The header already
carried the ruling, with reasons. `I12` fails the same way: its premise that
`maxNudgeRate == 0` and `maxGain == 0` "produce the identical behaviour" is false, and the
configuration it calls a misconfiguration is a **useful** one — a heading-only corrector.

## The three findings this chunk added

1. **`D2`'s arithmetic was already wrong.** The banner claimed the fault-name column fit was
   "checked by static math here". There was no such check — and writing it showed the sum had
   been false since chunk **F1** appended `MECHANISM_STALLED` (17 characters against row 1's
   15-character budget). On a real robot a jammed intake paints `flt MECHANISM_STALL`.
2. **`N1`, a defect not on the list.** A tracking pod dead or unenumerated at construction
   baselines at 0, so on the tick it enumerates `TrackingWheel` differences its true cumulative
   position against that 0: **28.4 in of phantom translation, probe-measured**, waved through by
   a plausibility gate that checked |Δθ| and never |Δtravel|.
3. **`E2` was sharper than reported.** The finding said the constructor leaves brake mode
   inherited. The consequence nobody had stated: `brakeMode_` is *also* the T7 fallback, so a
   port left in Hold by a previous session and dying before any command reported **Coast
   forever** — the fallback contradicting the device from boot.

## The mutation campaign — 21 run, 19 red, and the three greens are the real output

| Green | What it exposed |
|---|---|
| **M13** (`A5`) | **My test was wrong.** Filling the line to `kCapacity - 2` leaves ZERO room, and `appendRaw` with zero room writes nothing — so no partial ellipsis ever appeared. The bug needs 1–2 bytes free *after* the copy. Refilled to `kCapacity - 4`; **M13b is red.** |
| **M21** (`D12`/`D14`) | **My test was worthless.** It checked that a freshly constructed `HoldPose` had not exited — true of every `HoldPose`. Two attempts at the real scenario failed because `MotionRig`'s localizer is seeded live in its constructor. Test removed; the gap is named below. |
| **M15 / M20 (first spellings)** | Not green — **failed to build**, and that is the runner working: M15 changed a rendered signature so `check-fresh` blocked the relink, and running the stale binary reported the old GREEN. C4's trap, caught by gating on build success. Every later mutation was made **line-count-neutral**. |

## Two retractions

**`I13` — FIX → DEFER.** I gated `applied` on `o.dPos > 0` and it reddened two E4 tests. They
were right: an accepted fix with **zero innovation** legitimately moves no position while still
shrinking the covariance, so `dPos` is not a proxy for "this update did something". The honest
fix needs a `moved` flag set where the clamp computes its scale — EKF internals E4 sized against
invented noise and R4 re-measures. The defect, the failed proxy and the reason are now written
into the branch where they live.

**`A28`'s first fix crashed.** It called the full `cancel()`, mirroring F2's `WaitUnwindGuard`,
and the test **SIGABRTed**. Motions live on the caller's stack, and the idiom that *creates*
A28 — construct scheduler, construct motion, leave the scope — destroys them in reverse, so
`active_` dangles by the time the destructor runs. The destructor now does the half that needs
no motion: it commands the safe state directly and records **no** boundary, because recording
one honestly means reading an object that may no longer exist.

---

## 🔴 ARGUE — six real defects NOT fixed, each needing a decision

*(Flagged with the 🔴 convention so `briefing_status.py` carries them as open defects.)*

### 🔴 A15 / A16 — an atomic sensor read needs an F4 signature change

`confidence()` takes its own second `get_distance()`, so the seam's prescribed
`confidence()`-then-`distance()` pair spans two device samples; every `IGps` reader likewise
takes its own sample, so the mandated `hasFix()`-then-`pose()` pair can report "fix" and then
hand back a pose the class has just decided is fix-less — six device reads per corrector tick.

**Why not fixed:** both cures are breaking. Adding a `sample()`/`refresh()` verb to `IDistance`
or `IGps` changes a **row-F4-LOCKED** interface. Caching inside the adapter needs a validity
window measured in milliseconds, which is a constant nobody has measured and which would need
a clock the adapters do not hold. **The decision:** accept a wider F4 (a major version bump plus
a migration note), or accept a measured cache window as an HA entry at R4. Bench-resolvable.

### 🔴 A26 — `Twist2d` carries no frame while `ChassisSpeeds` demands one

`IPoseSource::twist()` is field-frame, `IKinematics::forward()` is body-frame, and handing one
to the other compiles silently and is wrong by a rotation of θ. The command side made the
opposite choice deliberately: `Frame` is required, with no default anywhere.

**Why not fixed:** every type-level cure — a frame tag parameter, distinct `FieldTwist`/
`BodyTwist` types, a wrapper — changes a type used across localization, kinematics and the sim
plant. **The decision:** whether the guard is worth a wide breaking change before M5 freezes
more surface against it.

### 🔴 A31 — `StrafeTo` inherits a setter that discards half its argument

`StrafeTo` publicly inherits `MoveToPose::setTarget(Pose2d)`, accepts and reports a heading, and
silently overwrites it at the first live tick. Its own constructor is honest (it takes x and y);
the inherited setter is not.

**Why not fixed:** narrowing it means changing a public inherited signature — a `Length`-pair
overload plus hiding or deleting the base's. **The decision:** whether `StrafeTo`/`HoldPose`
should narrow the base surface they extend, which is also a question about how much of
`MoveToPose`'s `protected` surface is contract.

### 🔴 I20 — the accuracy spec's heading targets are bare degrees

`kHeadingErrorMaxDeg` is a `double` three lines above a typed `units::Length`, in the one file
that is the single source of truth for the accuracy spec, guarding the one **hard** requirement.

**Why not fixed:** `spec/accuracy.hpp` **is Freeze Register row F2**, LOCKED. Retyping it is a
frozen-contract change, and the literals are what the acceptance tests compare against.
**The decision:** whether row F2 is amended, which is a version-bump conversation.

### 🔴 I21 — the angle literals cannot be `constexpr` without a cliff

`90_deg` cannot initialise a `constexpr` variable while `24_in` can.

**Measured, not argued.** The precondition is *not* the blocker — a `SHULIB_PRECONDITION` sits
in the untaken branch of a ternary and constant-evaluates fine. The blocker is `std::remainder`
in `wrapRad`: GCC accepts it in a constant expression as an extension (host **and**
`arm-none-eabi` both compile it clean), **clang rejects it**:

```
error: constexpr variable 'k' must be initialized by a constant expression
note: non-constexpr function 'remainder' cannot be used in a constant expression
```

An identity-interval early-out makes `remainder` unreachable for an already-wrapped input, and
clang then accepts `degrees(90.0)` — 0 mismatches over 1,028,583 samples including exact
boundaries. **But it buys a cliff:** `90_deg` becomes constexpr and `315_deg` does not, with no
rule a caller can see, and it costs ~6 lines inside **LOCKED row F3** including its private
trusted constructor and four accessors. A capability that works for some literals and not
others is worse than one that consistently does not. **The decision is the cliff**, not the
feasibility.

---

## DEFER — four, with the owner named

- **`A17` → R4.** `yawRate()`'s consuming rebase needs measured call patterns and a real loop
  rate before a design can be chosen; today the value each caller gets depends on who read first,
  and attaching a telemetry sink changes what the Localizer sees.
- **`A29` → R3/R4.** There is no gear-ratio concept anywhere in the library, and the A2 sim
  plant bakes 1:1 in as well. Larger than this chunk.
- **`E3` → F3.** `IOptical` has **zero consumers outside `hal/`**, there is no honest finite seed
  for hue (NaN is forbidden at this seam by F4), and the chunk that writes the first consumer
  owns the validity decision.
- **`E4` → R4/T2.** A polled watchdog cannot beat a stopped control task without a supervisory
  task, and nothing in the library owns one.

## Not finished, named honestly

- **`[~]` `D12`/`D14` are fixed and NOT PINNED.** Mutation M21 stayed green. `MotionRig`'s
  localizer is seeded live in its constructor, so neither a long `bootSettleTime` nor an
  un-ready IMU keeps `qualityClass()` at `Uninitialized` past the old 1.5 s budget. Pinning it
  needs a cold-boot rig this chunk did not build. A comment in
  `test/motion_primitives_test.cpp` names the gap at the point a reader would look for the test.
- **`[~]` Independent adversarial verification covers 33 of 84 items.** Eight of the twelve
  verifier agents died on a session limit. The remaining 51 carry one triage pass plus my own
  reading — and, for every FIX, the build, the suite and a mutation, which is the stronger
  instrument. Where a ruling rests on reading alone it is a REJECT or a DEFER, never a silent
  drop.
- **`[~]` `A22` counts the dropped tags; it does not rank them.** The selection is still
  arrival-order. Ranking by sigma inside `poll()` would duplicate the estimator's own model,
  which is trap 1 wearing a different coat. The loss is now visible, which is what the class's
  own design demanded; choosing better is R2's.
- **`N1` is a VISIBILITY fix and is labelled as one.** The bound is dt-blind — `PilonsOdometry`
  holds no clock — so the phantom delta is **reported, not withheld**, on the same rule
  `maxTickRotation` has always followed. Preventing it needs a validity channel F4 does not have.
  Registered **HA-123**, invented, with its settling measurement written into the entry.
- **No item was silently dropped.** All 84 carry an outcome line in `DOCS2-API-DEFECTS.md`.

## Numbers

| Measure | Result |
|---|---|
| Items triaged | **84** — 59 FIX · 15 REJECT · 6 ARGUE · 4 DEFER |
| Suite | **1,151 cases / 1,523,877 assertions / 3 skipped** — green (was 1,121) |
| New test cases | **30**, including a new file for `diag/line_format.hpp`, which had none |
| Mutations | **21 run, 19 red and observed, 2 green** — both green ones were my own tests, both closed or reported |
| Fixes retracted after the suite disagreed | **1** (`I13`), plus one fix reworked after a SIGABRT (`A28`) |
| Hardware assumptions | **HA-123** added (invented) |
| Public surface changed | `MotionOutcome::Unset`; `TrapezoidProfile::isDone()` drops `noexcept`; `IMechanism` non-copyable; `EkfFusion::state`/`covariance` drop `noexcept`; `ProsDigitalOut`'s poison overload — all in the changelog |

## Handoffs

**To the release:** every gate green, both guards, ARM over all headers, `prepare_site.py`.
The published reference no longer describes behaviour we know is wrong — which was the point of
inserting this chunk before the merge.

**To R3:** `A29` (gear ratio) and `A15`/`A16` (atomic sensor reads) are bench-resolvable in
minutes with the brain on the desk. `HA-123`'s bound wants a real loop rate.

**To R4:** `A17`, `E4`, and `I13`'s `moved` flag — all three need measurement before a design.

**To F3:** `E3`. The seam is documented and its cold-start window is stated; the first consumer
owns the validity decision.

---

## Process observations — things that cost me time, and one opinion

Recorded because the team lead asked for them, and because three of these are protocol gaps
rather than mistakes anyone made.

**1. The doc-gate deadlock is worse than `RESUMING.md` says, and it bites hardest during a
mutation campaign.** `shulib_tests` DEPENDS on `shulib_doc_gates`, and one of those gates
(`briefing_status.py check`) derives the suite state by running the **existing binary**. So the
moment a mutation makes the suite red, the briefing is stale, the gate fails, and **nothing can
be recompiled** — including the restore. Every mutation in this chunk needed a
`briefing_status.py generate` *before* its build, and the briefing carried RED counts through
the middle of the campaign. The briefing's own note calls this "a from-scratch build deadlocks
once"; it is not once, it is once per mutation. **Suggested protocol line:** a mutation
campaign should `generate` before every build, and the chunk must re-`generate` at the end —
which is easy to forget when the last thing you did was restore a file.

**2. Mutations must be LINE-COUNT-NEUTRAL, and that is a new trap.** `docs/api/` records the
declaration line of every entity, so a mutation that adds or removes a line fails `check-fresh`,
the binary is never relinked, and running it reports the **old** result. My first mutation did
exactly this and reported GREEN off a stale binary — C4's trap with a new trigger that did not
exist before DOCS2 generated the reference. A mutation that changes a *rendered declaration*
(a default member initializer, a signature) additionally needs `api_doc_tool generate`, or the
same thing happens. **Neither is written down anywhere.**

**3. "Report, don't fix" created a second debt that nobody costed.** DOCS2's landmine L3 was the
right call — a chunk that both documents and changes behaviour cannot tell you which of the two
broke the suite. But it meant writing careful descriptions of defects into **published**
headers, and two consequences followed. The small one: every fix here paid a doc-rewrite cost,
which is fine and was budgeted. The large one: **some of those descriptions were wrong about
the mechanism.** `D6`'s caveat said a zeroed pod is what the ODO_STUCK cross-check cannot see;
the cross-check reads `IMotor` and works in deltas, so it never sees this seam at all, and the
same over-claim propagated into `DOCS2-COMPLETED.md`. A defect description needs the *same
evidence bar as a fix*, because it ships to the same readers — and it is harder to hold to,
because nothing compiles a sentence.

**4. My opinion, offered for review rather than applied: the reviewer's independent pass should
re-run the MUTATION CAMPAIGN, not just the suite.** `RESUMING.md` step 3 says re-run the build,
the guards and the ARM gate, then write your own oracle. That is good and it would not have
caught this chunk's worst moment. Two of my three green mutations were **my own tests failing
to test** — a reviewer re-running a green suite would have seen green and learned nothing. The
cheapest version: the reviewer picks two or three of the chunk's claimed fixes, reverts each in
a scratch copy, and checks the suite goes red. It is minutes, and it is the only check that
distinguishes "there is a test" from "there is a test that works".

**5. A smaller one: `MotionRig` seeds its localizer live in the constructor.** That is right for
almost every test and it makes boot-window behaviour untestable — which is why `D12`/`D14` ship
unpinned. A `MotionRig` variant that boots cold would have closed that, and would probably pay
for itself: the wait-for-live contract is load-bearing in five motions and is currently only
exercised incidentally.
