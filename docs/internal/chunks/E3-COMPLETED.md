# E3 — `AprilTagCorrector` — completion record

> Completion record for [`E3-apriltag-corrector.md`](E3-apriltag-corrector.md). Live log:
> [`E3-PROGRESS.md`](E3-PROGRESS.md). Closes chunk 18 of 40; **Phase E, chunk 3 of 4.**
>
> **Do not commit** was the instruction; everything below is in the working tree.

---

## 1. What this chunk actually did

E2 bounded *position* drift. This chunk is the first code in the project's history that can tell
the estimator **which way it is pointing**. A tag observation is different in kind from a GPS fix:
`TagObservation::poseInRobot` is a relative **pose**, so against a tag whose field pose is known it
yields an absolute heading, which no other source in the tree can provide.

| | |
|---|---|
| Suite before | 794 cases / 1,081,382 assertions |
| **Suite after** | **867 cases / 1,091,167 assertions**, 3 skipped (unchanged), all green |
| New cases | **73** |
| Mutations | **44 executed, 43 red, 3 holes found and closed, 1 recorded UNCLOSED** |
| CI guards | GUARD1 (PROS-free) PASS · GUARD2 (no `sim/` in core) PASS |
| ARM gate | PASS, **114 headers** (was 111) |
| Doc gates | self-test · check-coverage · check-fresh · check-examples · check-removability — all PASS |
| New invented constants | **HA-68 … HA-82** (15), all registered, reconciliation clean both ways |
| Freezes | **none** |

### Files

| File | What |
|---|---|
| `include/shulib/hal/vision_conversion.hpp` | **new** — the corners→pose PnP, a free pure function (T3) |
| `include/shulib/localization/tag_map.hpp` | **new** — the tag map + the rigid inversion (T2) |
| `include/shulib/localization/apriltag_corrector.hpp` | **new** — the corrector |
| `include/shulib/localization/correction.hpp` | *modified* — `FusionResult::headingNudge` + 3 flags, `AppliedCorrection::dtheta`, `providesHeading` promoted from RESERVED to live (all appended/trailing) |
| `include/shulib/localization/complementary_fusion.hpp` | *modified* — the heading gate, gain and per-tick bound |
| `include/shulib/localization/localizer.hpp` | *modified* — the persistent heading bias, the odom-delta re-expression, the E3 substitution rule |
| `include/shulib/diag/debug_record.hpp` | *modified* — three appended `GateReason` values |
| `include/shulib/motion/motion_scheduler.hpp` | *modified* — one line: `correctionDTheta` is stamped |
| `test/vision_conversion_test.cpp` | **new** — 15 cases, the PnP against an independent projector |
| `test/tag_map_test.cpp` | **new** — 11 cases, the inversion and the provenance contract |
| `test/apriltag_corrector_test.cpp` | **new** — 23 cases, the corrector's decisions in isolation |
| `test/apriltag_corrector_heading_test.cpp` | **new** — 15 cases, the keystone: yaw through the full stack |
| `test/apriltag_corrector_blackbox_test.cpp` | **new** — 6 cases, through a real Localizer to decoded bytes |
| `test/apriltag_corrector_cost_test.cpp` | **new** — 3 cases, the allocation pin |
| `test/debug_record_test.cpp`, `test/blackbox_format_test.cpp` | *modified* — wire pins for the three new reasons |
| `docs/internal/verify/verify-e3.sh` | **new** — the mutation harness (hardened, see §6.2) |

---

## 2. The four tensions, ruled — each with the alternative that was rejected

### T1 — E3 LANDS absolute yaw correction, via the documented additive path

**RULING: land it.** `FusionResult` gains a trailing `headingNudge`; the `Localizer` folds it into
a **persistent heading bias** and composes the published heading as `imu.heading() + bias` as the
last write of the tick. The frozen `IPoseSource` / `ICorrector` / `IFusionPolicy` signatures did
not move and no caller changed.

**Why the accumulator is the whole design, not an implementation detail.** The `Localizer`
re-reads `imu_.heading()` every tick and stamps it; there is no persistent heading state anywhere
in the tree (`PilonsOdometry` likewise sets `pose_.heading = imu.heading()` every tick). A
`headingNudge` that only decorated *this tick's* published heading would be discarded on the next
one — individually sane, collectively useless. That is verbatim the M2 red team's
*corrections-not-accumulating* failure. The accumulator is the same structure STEP 2 already uses
for position: `fusedX_` is persistent and advanced by odometry *deltas*, never reset to absolute
odometry.

**Why this does not violate decision #4.** The IMU remains the sole source of heading *change* —
every rotation still enters through `imu.heading()`, tick for tick, and `PilonsOdometry` is
untouched. What the corrector learns is a slowly-moving **bias**, moved by at most
`maxHeadingNudgeRate · dt` per tick. **There is no code path anywhere that assigns a heading**: the
policy can only return an increment. That is what makes never-snap *structural* for yaw rather
than a promise. The master plan is explicit that this is the intended shape:

> "The spec therefore makes **IMU-owned heading + absolute yaw correction (AprilTag/GPS)
> load-bearing, not optional**." — `shulib-v2-master-plan.md:250`

**Rejected alternative — defer to E4.** Defensible, and the safe choice. Rejected because the
*plumbing* — an accumulator, a bounded increment, a wire slot — is needed identically whether the
weight comes from a hand-written gain or a Kalman gain, so deferring would postpone none of E4's
real work while leaving the guide's promise outstanding. E4 replaces the *weighting*, not the path.

**Rejected alternative — let the corrector write heading / remove the re-stamp.** A source that
can assign heading can snap it, and a yaw snap poisons every field-relative command after it.

**Rejected alternative — widen `CorrectionProposal`'s meaning.** Not needed: `providesHeading`
already existed, RESERVED, for exactly this. E3 makes it live and the struct's shape is unchanged.

**Rejected alternative — cap the bias.** A cap looks prudent and is the wrong instinct: the nudge
is always *toward an absolute measurement*, so the loop is closed and cannot run away, whereas a
cap would silently lock out recovery from precisely the large real drift that makes the correction
worth having — the heading analogue of E2's D2 gate-lockout failure.

**A consequence the brief did not anticipate, ruled the same way.** If the bias corrects only the
*published* heading, `PilonsOdometry` keeps rotating its per-tick field delta by the **raw** IMU
heading, so the position prediction keeps accumulating a cross-track error of roughly
`bias × distance` — which is most of what heading drift actually costs over a 60 s run. So STEP 2
re-expresses the odometry delta under the learned bias. Six lines, `PilonsOdometry` still a pure
predictor, and an explicit zero-bias early-out so a tree with no heading-providing corrector is
**bit-identical to pre-E3 by construction** rather than by a floating-point argument. Proven by a
test that compares `pose().heading().radians()` with `==` on doubles.

### T2 — the tag map is INPUT, and shulib ships none

**RULING: `localization/tag_map.hpp` provides the type; the library ships NO built-in field
layout, and adding one would be a mistake until somebody can cite a published table.** Nobody on
this project has a game-manual table of AprilTag field poses in hand. A plausible-looking default
map would be invented geometry wearing the clothes of a specification, and every team that forgot
to override it would silently localize against fiction. An empty map makes every tag decline with
`RejectedNoTagMapEntry` — a loud, diagnosable state rather than a quiet wrong one.

**Provenance is enforced by precondition, not by convention.** `TagMap::add()` refuses an entry
whose `TagProvenance` is `Unspecified` or whose `source` citation is empty. `Invented` is a
legitimate answer — a guess *labeled* as a guess is what the A4 register exists to protect — but
"I did not say" is not, because the difference between a specified and an invented tag pose is
invisible in the arithmetic and total in the consequences. `anyInvented()` exists so a run anchored
to made-up field geometry cannot read the same as one anchored to a measured field. `add()` also
refuses a **duplicate id** — the mistake most likely to survive review, because the second entry
simply never wins a lookup and the map still "works".

**Rejected alternative — ship a plausible VEX layout and document it as provisional.** Convenient,
and it is how a wrong number becomes load-bearing. A map 2″ off yields a corrector that is
*confidently* 2″ wrong every time it sees that tag, with a small residual and a high confidence —
i.e. it looks exactly like a healthy fix. Sensor noise averages out; this does not, and no gate,
filter or amount of averaging can reveal it. Registered as **HA-68**, flagged in the register as
one of its three most dangerous entries.

### T3 — PnP is built here and does not live here

**RULING: `hal::tagCornersToRobotPose()`, a free pure function in
`include/shulib/hal/vision_conversion.hpp`** — named to sit beside `gps_conversion.hpp` and
`imu_conversion.hpp`, which is where the tree already puts its "convert exactly once, at the edge"
helpers. **`apriltag_corrector.hpp` does not include it**, which is the enforceable form of the
ruling: R2's adapter is the caller, exactly as `hal/vision.hpp:12-14` scopes it.

Algorithm: DLT homography (an 8×8 solve with partial pivoting, fixed storage) → `G = K⁻¹H` → scale
recovered from `|r1| = |r2| = 1` → orthonormalization → `r3 = r1 × r2` → planar reduction. `H` is
normalized with `h33 = 1`, which fixes `λ = 1/t_z` and makes the depth sign come out right by
construction — there is no "which of the two solutions is in front of the camera" branch to get
wrong.

**Rejected alternative — bearing + apparent-size range instead of a real PnP.** Much simpler, and
it cannot recover the tag's *orientation*, which is the entire prize of this chunk.

### T4 — two methods, and the cadence is the caller's

**RULING: `poll()` at vision cadence (the only method that touches `ITagSource::tags()`, therefore
the only one that allocates) and `propose()` every control tick (never touches the HAL).**

**Rejected alternative — call `tags()` from `propose()` only every Nth tick.** It does not remove
the heap allocation from the control loop; it makes it *intermittent*, which is strictly harder to
diagnose. A1's cost contract forbids the allocation, not its frequency.

The footgun this creates — a corrector nobody polls is silent forever — is answered three ways:
`pollCount()` is exposed, a never-polled corrector declines with `RejectedNoFix` from the very
first tick, and a poller that *was* running and stopped declines with the distinct
`RejectedObservationAge`.

---

## 3. The decision docket

**D1 — one tag, not an average of N.** `ICorrector` returns one proposal, so N visible tags must
become one answer. The tag with the smallest estimated σ wins, and `lastTagId()` says which.
*Rejected: average the N absolute poses.* Averaging headings needs a circular mean and both need a
weighting scheme that is exactly what an EKF derives and a complementary tier can only invent.
Worse, averaging **hides disagreement**: two tags implying poses 8 inches apart average to a
confident wrong answer with no residual to show for it, where "trust the best one" leaves the
second tag's disagreement visible as a future innovation.

**D2 — heading gets a trusted-range BAND, not its own noise model.** Planar-PnP *heading* degrades
faster with range than *position* does (the near-planar ambiguity). Modelling that properly needs a
second noise number on `CorrectionProposal` and a policy that uses it, which is E4's EKF. E3 uses
the blunter instrument that requires inventing no new relationship: a band outside which the
observation is not used at all (HA-73). Blunter, and honest about which of the two it is.

**D3 — latency is compensated in HEADING as well as position, and the rotation term comes from the
IMU.** A tag fix describes which way the robot was pointing ~80 ms ago; at 180°/s that is 14°,
fourteen times the entire heading budget. The history ring therefore carries an **unwrapped**
cumulative heading (interpolating raw wrapped headings across the ±π seam produces a garbage
rotation exactly once per revolution) — taken from `imu_.heading()`, not from `predicted.heading()`,
because since E3 the predicted heading contains the estimator's own bias learning and that is not
rotation. **See §6: this last point is argued from algebra, not from a measurement, and it is the
chunk's one unclosed mutation.**

**D4 — position and heading are gated INDEPENDENTLY.** They are different measurements with
different failure modes; a fix can be good enough to move position and not good enough to move
heading. `applied`/`gated` continue to mean exactly what they meant (position);
`headingApplied`/`headingGated` are new. `GateAudit::reason` stays position-primary, and a
heading-only rejection reads off the record as *a large `gateResidualHeading` beside a zero
`correctionDTheta`* — unambiguous, and it needed no twelfth enum value.

**D5 — `RejectedSensorQuality` is REUSED for a below-floor tag confidence.** Its documented meaning
("a sensor saying *I can see, badly*") is exactly the statement. Three new spellings were appended
for the three genuinely new statements; a fourth would have been vocabulary inflation.

**D6 — the E2 handoff answered: an EXCEPTIONAL verdict outranks a ROUTINE one.** E2 left the
one-gating-slot question open: "with two correctors, only the first silent source's verdict
survives — decide it deliberately." Ruled: routine means `RejectedStaleFix`; among equals the first
still wins. The reasoning is E2's own measurement — a healthy source spends the large majority of
its ticks stale (2520 against ~570 fresh in 30 s). Under plain first-wins, the GPS's *ordinary*
staleness would mask the tag corrector's *genuine* failure — an unmapped id, a dead vision task —
for essentially the whole run. One predicate, deterministic, and `AppliedCorrection::source` still
names whose verdict it is.

**D7 — `setPose()` keeps the learned bias.** A teleport says where the robot *is*, not which way
the IMU is wrong. Discarding a bias that took a second of tag sightings to learn, every time a
routine re-seeds its position, would throw the correction away at exactly the moments a routine
cares most. `p.heading()` is still ignored, as it always was; `setPose` now publishes the
*corrected* heading so it and `update()` cannot disagree about the same instant.

**D8 — the corrector consumes a frame once, before any later rejection.** A frame taken mid-spin is
skipped rather than folded once the spin ends: it describes a moment already judged untrustworthy
(E2's D8, same argument, pinned by its own test).

---

## 4. Test evidence

**73 new cases.** Every one names, in a comment, the bug it would catch.

### The PnP, and the failure that was *supposed* to recur here

`test/vision_conversion_test.cpp` is built in **three deliberate layers**, because a round-trip
through a projector that shares a camera model with the PnP under test would let an intrinsics or
sign error cancel exactly — the failure that bit C1, C3, C4 and E2.

1. **Hand-worked pixels.** Literal pixel coordinates computed on paper, the arithmetic written out
   in each comment, handed straight to the PnP. **No projector is involved at all**, so nothing can
   cancel. Five of them: the head-on anchor (a 150×150 px square → 24″ ahead facing us), a camera
   yawed 30°, a camera mounted off-centre, a tag turned 30° off head-on, and a tag mounted 15″
   above the camera (whose height must *not* leak into the horizontal position).
2. **The projector is pinned against those same hand-computed pixels before it is used as an
   oracle**, including one case whose expectation is reasoned in words ("a left-yawed camera pushes
   a straight-ahead tag to the image RIGHT"). It is written OUTWARD (field → body → camera →
   pixels) where the PnP works INWARD.
3. **Then** the sweep: 7 robot headings × 5 tag facings, robot at (−19.5, 31.25), camera at
   (4.5, −2.75) yawed 12°. **Neither heading 0 nor the origin appears anywhere.**

Plus: range scaling from 12″ to 120″; doubling the physical tag size must double the recovered
range; degenerate and impossible inputs; and the reprojection error being real rather than
decorative.

### `tag_map_test.cpp` — 11 cases

Two hand-worked inversion cases with all the arithmetic in the comment (one at heading 30° with
both `rx` and `ry` non-zero; one whose heading difference wraps past −180°), a 105-case sweep
against a forward composition written independently in the file, an axis-asymmetry probe, and the
provenance contract driven through its preconditions. **No case uses the origin or heading 0, and
every case uses a non-zero relative heading** — because the position term is rotated by the
*robot's* heading and the two coincide exactly when `rθ == 0`.

### `apriltag_corrector_test.cpp` — 23 cases

Five distinct silences, each with its own word. The map's verdicts and their documented priority.
The range band at both ends and just inside both. The confidence floor. Non-finite observations
declined and **not thrown**. Yaw-rate rejection at both signs, and the mid-spin frame not folded
after the spin. σ and confidence hand-computed to 16 digits. Multi-tag selection. The absolute
pose recovered exactly while the prediction is deliberately 2″ and 3° wrong. The gate boundary at
the hand-computed 8.944…, anti-lockout widening, and the accumulator reset. Latency in position, in
heading, **across the ±180° seam**, and zero for a stationary robot.

### `apriltag_corrector_heading_test.cpp` — 15 cases, the keystone

Four load-bearing properties, each with a named failure mode:

1. **It accumulates.** From a 4° IMU error: **0.50° at 3 s, 7.4e-05° at 15 s**, moving toward truth
   on **every one of 1500 ticks** and never past it.
2. **It nudges, never snaps.** With a 12° error and a maximally confident tag: **every one of 2000
   ticks** asserted against the per-tick bound, on the bias, on `AppliedCorrection::dtheta`, and on
   the published heading — plus a per-tick no-overshoot assertion.
3. **It survives the IMU re-stamp ordering.** After learning a bias, a 20° rotation arrives in
   full, with the bias riding on top and the published heading still exactly `raw + bias`.
4. **It costs nothing when absent.** `pose().heading().radians() == imu.heading().radians()`
   compared with `==` on doubles, across six headings including both sides of the ±180° seam.

Plus the boot guard (no bias is learned while the IMU is calibrating — a bias learned from
calibration garbage would be permanent), `setPose` retention, and **five two-corrector cases**:
GPS and tags agreeing, disagreeing, one silent, the exceptional-vs-routine rule, and the GPS's
pass-through heading proven not to dilute the tag's yaw fix.

### `apriltag_corrector_blackbox_test.cpp` — 6 cases

Every kind of silence as a different word; **bit-identical to having no corrector at all** when no
tags are in view (a stronger statement than "small effect"); all seven verdicts arriving in the
**decoded** file; the heading nudge on `correctionDTheta` with the never-snap bound audited **from
the bytes**; a gated heading read as "large residual, zero nudge"; and an accepted heading fix
whose nudge is provably a *fraction* of its residual — the numeric signature of a nudge rather than
a snap, re-derived from the decoded record alone.

### `apriltag_corrector_cost_test.cpp` — 3 cases: the cost is PINNED, not asserted

The file **replaces the global `operator new`/`delete`** and counts.

- **The instrument is proved first**: `poll()` must show a non-zero count. Without that, a silently
  broken counter would make every "zero allocations" assertion vacuously true — the exact shape of
  a test that looks like evidence and is not.
- `propose()`: **0 allocations across 20,000 ticks** (200 s of control loop) with tags visible
  throughout and >3500 fixes actually folded, so the branches are exercised, not early-outed.
- A full `Localizer::update()` with the tag corrector registered and the heading path **live**:
  **0 allocations across 5,000 ticks.**

---

## 5. Mutations — 44 executed, 43 red

`docs/internal/verify/verify-e3.sh`, gated on build success. The five the brief requires are first
and labelled as such: tag-map axes swapped · relative-pose inversion sign flipped · a tag fix snaps
instead of nudging · convergence accumulation broken · no-tags returns a low-confidence pull. All
five red.

The rest, all red: PnP camera→body axes swapped · tag normal not negated · camera mount offset
dropped · camera yaw ignored · tag size ignored · rotation columns not orthonormalized · scale from
one column · degenerate corner set accepted · TagMap rotated by the tag's heading · heading
inversion sign · provenance not required · duplicate id accepted · `anyInvented()` always false ·
`providesHeading` never set · heading latency dropped · position latency dropped · range band
removed · confidence floor removed · yaw-rate rejection removed · freshness guard removed ·
observation-age guard removed · unmapped id hidden · anti-lockout removed · σ ignores confidence ·
position gate inverted · worst-tag selection · **T4 violation (`propose()` calls `tags()`)** ·
heading gate removed · per-tick heading budget removed · `providesHeading` ignored by the policy ·
heading residual never audited · bias never published · odom delta not re-expressed ·
`dtheta` never reaches the audit record · E3's substitution rule reverted · E2's substitution guard
removed · E2's substitution overwrites a real verdict · `correctionDTheta` never stamped.

### THE HOLES — three found and closed, one recorded unclosed

**Hole 1 — the PnP scale taken from one in-plane axis instead of both.** Completely invisible on
noise-free input: a perfect square projects to columns of exactly equal norm, so `2/(n1+n2)` and
`1/n1` agree to the bit. Closed with an exact, hand-derivable probe — stretch the anchor image
horizontally by *f*, and the two in-plane axes then imply depths `24/f` and `24`; the correct scale
returns their **harmonic mean** (`f = 1.2` → 21.8181…, `f = 2.0` → exactly 16.0), the mutation
returns `24/f`. Fails alone.

**Hole 2 — the singular-system guard removed.** It *looks* redundant: for exactly collinear corners
the solve divides by zero and the downstream finiteness screens catch the infinities. **It is not
redundant.** Four corners collinear **to within 1e-13 of a pixel** leave the pivot small but
finite, the solve blows up to enormous finite numbers, and every downstream screen then passes:

```text
with the guard:     valid = false
without the guard:  valid = TRUE — "a tag 5.99 inches away at x = 5.73, facing us"
```

Plausible fiction, and nothing downstream — not the range band, not the gate, not the reprojection
error — can tell. Found by fuzzing **4,000,000 random corner sets** (which showed *zero*
difference) and then searching structured near-degenerate families. Closed by a subcase using
exactly that input.

**Hole 3 — `providesHeading` was not actually load-bearing, and the odometry delta was not
re-expressed.** Two related holes closed with two targeted tests. The pre-existing "a corrector
cannot move heading" test used a corrector claiming a 90°-wrong heading — which the *heading gate*
rejects anyway, so the property was being proved by the wrong mechanism. The new case keeps the lie
**inside** the gate (5°), where only `providesHeading` can stop it, and then shows the same
corrector moving the bias once it *does* claim to measure heading. The second: every heading test
until then kept the robot **stationary**, so nothing exercised the odom-delta rotation; the new
case learns a 4° bias, removes the tags, drives 20″, and asserts the fused position advanced along
35° and not along the raw IMU's 31°.

### THE HOLE I COULD NOT CLOSE — recorded, with its measurement

```text
apriltag_corrector.hpp:  const math::Angle imuHeading = imu_.heading();
                     →   const math::Angle imuHeading = predicted.heading();
```

**Measured both ways, same scenario** (12° IMU error, tag at 30″, 20 Hz vision against 100 Hz):

| rotation term from | max bias over the run | at 5 s | at 9 s | at 20 s |
|---|---|---|---|---|
| `imu_.heading()` (in the tree) | 12.000 | 9.83388 | 11.8852 | 12.000 |
| `predicted.heading()` (the mutation) | 12.000 | 9.85176 | 11.8987 | 12.000 |

**Neither overshoots. Ever.** The entire difference is ~0.1% in convergence rate. The change stays
because the algebra is unambiguous — the term means "how far has the robot *turned* since capture",
and taking it from a quantity that contains the estimator's own bias learning makes the *effective*
gain depend on the latency window and the loop rate — but **any test tight enough to separate
11.8852 from 11.8987 would be pinning an invented constant, which this chunk explicitly refuses to
do.** It is labelled KNOWN GREEN in the harness, with the measurement beside it.

---

## 6. Two process failures of mine, recorded because the recovery is the point

### 6.1 I OVERCLAIMED A BUG, in the live progress log, and had to strike it

At 00:12 I recorded as a headline finding that the convergence test had caught a **real bug**
producing a **12.1013° overshoot on a 12.0° correction**. **That was wrong.**

The failing assertion read `CHECK( std::abs(s.biasDeg() - 12.0) < 0.1 )  values: CHECK( 0.101325 <
0.1 )`. I read `|bias − 12| = 0.101` and **inferred the sign** — I wrote "12.1013, an overshoot"
without ever checking whether it was 12.1013 or 11.8987. It was **11.8987**: an *undershoot*, and
the tolerance was simply too tight for 900 ticks of an asymptotic approach. The same assertion
failed again *after* my "fix" (`0.110655 < 0.1`), which should have told me the diagnosis was
wrong; I extended the run instead of asking why.

**The tell I ignored, and the general lesson.** `std::abs()` in an assertion **destroys the sign**,
which is usually the most diagnostic bit in the number. I inferred it instead of measuring it, and
then wrote a confident narrative on top of the inference. The measurement that settles it took four
minutes. It is corrected in the progress log by **striking the claim rather than editing it away**,
because the process failure is the more useful record — and this document's §5 now carries the real
numbers.

The code change survives on its own merits (the algebra above), but its justification changed from
"fixes a measured overshoot" to "is the correct quantity", which is a much weaker claim and is
stated as such in the header.

### 6.2 The mutation harness reported two mutations GREEN that never ran

`grep -F` with a **multi-line** pattern matches if **any single line** matches, so the harness's
pre-flight check passed while python's exact whole-string `replace` found nothing. The file was
unchanged, the build succeeded, the suite passed — and it was scored as a hole in the suite.

This is precisely the class of fault E2's postmortem was written about ("the report itself looked
fine; the tell was the count"). Fixed **structurally** rather than by correcting the patterns: the
harness now **byte-compares the file against its backup after the edit** and treats "the edit
changed nothing" as a loud SKIP that exits non-zero. A mutation that did not modify the file can no
longer be reported as anything at all. E2's other two hardening measures (never `git checkout`,
trap on INT/TERM/PIPE) were carried over from the start and were not needed.

---

## 7. What we know for certain, and what we do not

**KNOWN**

- `AprilTagCorrector` implements `ICorrector` behind the unchanged signature; no caller changed.
- The PnP recovers the tag's relative pose exactly from hand-computed pixels at four independent
  geometries, and across a 35-case sweep at seven headings and five tag facings, none at the origin
  or heading 0.
- With no tags in view the corrector contributes **bit-identically nothing** — the estimate matches
  a Localizer with no correctors at all, digit for digit — while the record names both the reason
  and the source.
- With no heading-providing corrector, the published heading is the raw IMU reading **bit for bit**,
  and both new Localizer paths early-out on an exact `== 0.0` test.
- Against simulated truth with a simulated camera: a 4° heading error converges monotonically over
  1500 consecutive ticks and never overshoots; no tick of 2000 moved the heading by more than the
  documented per-tick bound, verified live *and* from decoded blackbox bytes.
- `propose()` allocates **zero** times across 20,000 ticks, measured by replacing the global
  allocator, with the instrument itself proved by a positive control.
- Every tag gating decision reaches a **decoded** blackbox file; a gated heading and an accepted
  heading are distinguishable from the bytes alone.
- A tag map entry cannot be added without stating where its numbers came from.

**NOT KNOWN, stated plainly**

- **Nothing here has seen a camera.** Every behaviour was proven against synthetic projections and
  a simulated tag source whose noise, cadence, confidence and latency are invented (HA-71…HA-79).
- **The `< 1°` heading requirement is NOT claimed to be met, and is not claimed to be on track.**
  The only thing E3 changes about that claim is that the specific reason it was listed as
  unachievable — nothing in the library could correct heading at all — no longer applies.
- **There is no tag map.** Not "a provisional one": none. The most dangerous input in the library
  is one the library refuses to guess at (HA-68).
- **A reversed corner winding is undetectable in software** — 180° of heading error with the
  reprojection residual at machine zero (HA-69). Only a physical tag settles it.
- **Fifteen new constants are guesses** (HA-68…HA-82). Every test asserts a *shape* — "farther is
  worse", "the gate widens with travel", "a spin rejects the fix" — never that a constant is right.
- **Whether folding tags is worth doing at all is unmeasured**, and depends on HA-20 (real IMU
  drift) versus real tag accuracy at range. Neither number exists.
- **Two disagreeing correctors are BOUNDED, not resolved.** That needs a covariance, i.e. E4.
- **`ComplementaryFusion`'s fixed 12-inch ceiling still binds** — E2's finding 2, unchanged and
  deliberately not patched from inside a corrector. It rejects tag fixes exactly as it rejects GPS
  fixes.
- **`gateMahalanobis` is still 0 on every real path**, and will be until E4.
- **No accuracy sweep against the A2 plant.** E2 ran 8 seeds of a 60 s hostile plant run; E3 did
  not, because the A3 hostile model has no tag source and inventing one would have meant tuning to
  my own invented noise — which the brief explicitly forbids. E3's claims are about *logic and
  bounds*, not about an error number.

---

## 8. Documentation contract — discharged

1. **Roadmap checkbox** — WS5 tier 2 `AprilTagCorrector` flipped `[x]` with files, test files, case
   and assertion counts, and **four** scope caveats attached to the checkbox itself. The
   "innovation-bounded, covariance-weighted gated nudge" line moved to `[~]` with the missing half
   ("covariance-weighted") named and assigned to E4.
2. **"You are here"** — E3 recorded with what it did *and* what it did not; next pointer moved to
   E4.
3. **Design notes in the headers** — `vision_conversion.hpp` carries the frames stated once
   explicitly, the corner-order contract with its *measured* sensitivity, the planar reduction's
   discards, the algorithm and its one structural assumption, and the near-ambiguity. `tag_map.hpp`
   explains why a wrong map does not average out and why shulib ships none. `apriltag_corrector.hpp`
   carries the two-method shape and its footgun, the order of operations, the one-tag ruling, the σ
   model, latency in both channels, and the deliberate non-responsibilities. `correction.hpp`,
   `complementary_fusion.hpp` and `localizer.hpp` each explain the heading path from their own side,
   including why the bias must persist and why it is uncapped.
4. **Test evidence** — §4, with the three-layer PnP argument spelled out and every mutation named.
5. **Decisions** — §2 (T1–T4) and §3 (D1–D8), each with its rejected alternative.
6. **Freeze Register** — **E3 freezes nothing.** It appends to one wire-stable vocabulary
   (`GateReason`, three values, re-pinned by test) and two value types (`FusionResult`,
   `AppliedCorrection`), all trailing and defaulted so every existing construction still compiles
   and means the same thing. F4's vision seam is untouched: `ITagSource` and `TagObservation` are
   byte-for-byte unchanged.

**Guide, per the named scope:**

- **Chapter 3** gains "The tag corrector: the first thing that can tell you which way you are
  facing" — written as a change to the *mental model* (heading error used to only ever grow; the
  IMU still owns rotation and what is learned is a *bias*), the six checks the corrector runs, four
  honest limits, and the point that matters most for this team: **the tags do not depend on the GPS
  strip, so in Driving Skills they are the only absolute source of anything.** The `setPose` and
  "design around drift" advice both updated.
- **Chapter 11** gains the three new `GateReason` rows, a new reading habit, and a
  "Reading the heading correction" subsection with a two-field table for `correctionDTheta` ×
  `gateResidualHeading` — including the explicit statement that a heading-only rejection has no
  word of its own and why.
- **Chapter 14**'s yaw section is rewritten as *"Correction now exists for position and heading —
  and what it is worth is unmeasured"*, structured as **measured → measured on what → still
  unmeasured**, and its "exactly one absolute position source" bullet is gone.
- **Chapter 15** gains seven terms: AprilTag, corrector, heading bias, PnP, provenance,
  snap/never-snap, tag map.
- **The README**, not named in the brief but corrected because E3 made one of its bullets
  actively FALSE: "No vision-based position correction yet (the correctors are planned work)".
  Rewritten to say the corrector exists **and** that no camera has ever been pointed at a tag and
  the library ships no map, pointing at chapter 14 for the scoped claim. The localization bullet
  now names both correctors. The suite counts were also refreshed (752 / 936,895 → 867 /
  1,091,167) — they had been stale since E2. **The measured heading-accuracy paragraph was
  deliberately NOT touched**: that number (worst 0.912° over 60 s across 10 seeded boots) comes
  from a test with no tag corrector in it, and letting E3 be read as having improved it would be
  exactly the overclaim this chunk exists to avoid.

No guide code block changed, so no example re-quoting was needed; `check-examples` passes.

### On the sentence that was hard to write

The brief said: *"If the sentence becomes hard to write honestly, that difficulty is the finding."*
It was, and here is the difficulty precisely.

Everything true about this chunk pulls toward optimism. The mechanism the `< 1°` spec depends on
now exists; it converges; it is bounded; it is audited to the byte. Every honest sentence about it
is a sentence about a *simulation*. The temptation is not to lie — it is to write "the mechanism
the spec needs now exists and works", let the reader supply "so the spec is probably met", and
never technically claim it.

The resolution was to make the chapter say what it is *not* claiming, explicitly, in the same
breath as what it is: **"this page does not claim the `< 1°` requirement is met, and does not claim
it is likely to be met. It claims exactly one thing — the specific reason it was previously listed
as unachievable no longer applies."** Followed by: *"If that reads as a smaller change than the
feature sounds like, that is the honest size of it."*

The structural device that made it writable was ordering: **what was measured → what it was
measured on → what remains unmeasured**, with the middle section (simulated truth, simulated
camera, invented map, invented noise) sitting physically between the good news and the conclusion,
where it cannot be skipped.

---

## 9. Verification (run, with output as observed)

```text
cmake --build build/test -j$(nproc) && ./build/test/shulib_tests | tail -6
  [doctest] test cases:     867 |     867 passed | 0 failed | 3 skipped
  [doctest] assertions: 1091167 | 1091167 passed | 0 failed |
  [doctest] Status: SUCCESS!

GUARD1 PASS          (no pros/ include anywhere in include/shulib)
GUARD2 PASS          (no shulib/sim/ include outside sim/)
ARM GATE PASS        (114 headers, arm-none-eabi-g++ -std=gnu++20 -Werror …)

doc gates:  self-test PASS · check-coverage PASS · check-fresh PASS
            check-examples PASS · check-removability PASS

docs/internal/verify/verify-e3.sh
  E3 mutations: 43 RED (good), 1 GREEN (HOLE — recorded, §5), 0 build-fail, 0 SKIPPED

register reconciliation, direction 1 (must print nothing):
  grep -rn "PROVISIONAL (A4" include/ test/ | grep -v "HA-[0-9]"   → empty
register reconciliation, direction 2 (every HA-nn in code has an entry):  → empty
```

---

## 10. Named handoffs

**→ E4 (the EKF)**

- **`R_heading` is yours.** E3 handles the fact that planar-PnP heading degrades faster with range
  than position with a **trusted-range band** (HA-73), which is a blunt instrument chosen because
  the alternative was inventing a second noise number. A filter with a real measurement covariance
  can use the whole range instead of a band.
- **`CorrectionProposal` will need a `headingStdDev`** to make that possible — appended, trailing,
  defaulted, exactly as `selfAudit` and `headingNudge` were.
- **Multi-tag triangulation is yours.** E3 deliberately uses the single best tag; averaging N
  absolute poses needs a weighting scheme only a covariance can justify, and averaging *hides*
  disagreement.
- **The 12-inch ceiling still binds**, on tags as on GPS (E2's finding 2, re-confirmed). Two
  correctors now disagree in real code, and the complementary tier can only bound the damage.
- **`HA-78` is a gain knob wearing the clothes of a covariance**, exactly as HA-66 is.

**→ R2 (the vision adapter)**

- **Read the corner-order contract at the top of `hal/vision_conversion.hpp`, and then read the
  three subcases in `test/vision_conversion_test.cpp` that measure it.** A cyclic rotation is
  harmless. **A reversed winding is catastrophic and completely silent** — 180° of heading error
  with the reprojection error at machine zero. Pin the detector's winding against a physical tag;
  no software self-check exists or can exist (HA-69).
- **`tagCornersToRobotPose()` is yours to call**, and it expects **undistorted** corners. Lens
  distortion is R2's to remove and R2's to prove.
- **The camera is assumed LEVEL** (HA-70). `CameraMount` carries position and yaw only, which
  encodes the assumption in the type; if the real mount pitches, that struct grows.
- **You own the vision task.** `AprilTagCorrector::poll()` must be called from it and **never** from
  the control loop. A corrector nobody polls is silent forever, and says so on every tick.

**→ R3 / R4**

- **HA-68 is the one to settle first.** Obtain or measure the tag layout, and record *how* in each
  `TagPlacement::source`. Everything this chunk does is downstream of that map being right.
- **HA-20 (real IMU drift) decides whether this chunk was worth building**, exactly as HA-26 and
  HA-20 together decided that for E2. Measure it early.
- **The range band (HA-73) needs one experiment**: park at a known pose, sweep range, plot recovered
  *position* error and recovered *heading* error separately. They will not degrade together.
