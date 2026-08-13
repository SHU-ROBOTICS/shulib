# E3 — `AprilTagCorrector` — live progress log

Appended as work happens. Timestamps from `date +%H:%M:%S`.

---

**23:11:11** — Start. Branch `shulib-v2`, HEAD `10007b2`, tree clean. Brief read in full
(`docs/internal/chunks/E3-apriltag-corrector.md`). Four rulings T1–T4 noted; documentation section
flagged as load-bearing as the code.

**23:11:30** — Beginning required reading in the prescribed order: `hal/vision.hpp` (full),
`localization/correction.hpp` (full), `gps_corrector.hpp` + `E2-COMPLETED.md`, `i_corrector.hpp`,
`localizer.hpp`, `docs/guide/14-what-it-cannot-do-yet.md`, `docs/internal/RESUMING.md`.
No code will be written until T1 is ruled.

**23:19:40** — Required reading complete. Structural notes that will decide T1:

- `Localizer::update()` STEP 5 stamps `heading = imu_.heading()` **fresh every tick**. There is no
  persistent heading state anywhere in the tree (`PilonsOdometry` likewise sets
  `pose_.heading = imu.heading()` every tick). So a `headingNudge` that only edits *this tick's*
  published heading **evaporates on the next tick** — that is precisely the M2 red-team failure mode
  ("corrections not accumulating"), and it is structural, not incidental.
- The wire slots for heading correction were **pre-declared and are already serialized**:
  `debug_record.hpp:151 correctionDTheta` ("heading nudge (0 at M2) — E3"), byte offset 340 in
  `blackbox_format.hpp`, and `GateAudit::residualHeading` ("E3 fills it"). The format is already
  pinned by `blackbox_format_test.cpp`. Nothing about the wire needs to move.
- `ComplementaryFusion`'s header says "Heading is NEVER touched"; that sentence is E3's to change.
- `hal/fake/fake_tag_source.hpp` exists (A-phase), `test/vision_test.cpp` already says the PnP
  reduction is "M3 ... red-teamed then". That is this chunk.

**23:20:05** — Registered constants currently run to **HA-67** (E2's tuning set). E3's new invented
constants therefore start at **HA-68**.

---

## 23:24:10 — **T1 RULED: E3 LANDS absolute yaw correction, via the documented additive path.**

**The ruling.** `FusionResult` gains a `headingNudge` (appended, trailing, defaulted). The
`Localizer` folds that nudge into a **persistent heading-bias accumulator** and composes the final
heading as `imu.heading() + bias` as the last write of the tick. The frozen
`IPoseSource` / `ICorrector` / `IFusionPolicy` signatures do not move.

**Why it must be an accumulator, not a per-tick edit.** This is the load-bearing part and it is not
optional. `Localizer` re-reads `imu_.heading()` every tick and stamps it; there is no persistent
heading state. A `headingNudge` that only decorated *this tick's* published heading would be
discarded on the next tick — individually sane, collectively useless. That is verbatim the M2 red
team's *corrections-not-accumulating* failure mode, and the brief holds it as a required test
(constraint 4). The accumulator is the same structure `Localizer` STEP 2 already uses for position:
`fusedX_` is persistent and advanced by odometry *deltas*, never reset to absolute odometry.

**Why it does not violate "heading is IMU-owned" (decision #4).** The IMU remains the sole source
of heading *change*: every rotation the robot makes still enters through `imu.heading()`, tick for
tick, and `PilonsOdometry` is untouched. What the corrector learns is a slowly-converging **bias**,
moved by at most `maxHeadingNudgeRate · dt` per tick. The master plan is explicit that this is the
intended shape, not a deviation:

> "The spec therefore makes **IMU-owned heading + absolute yaw correction (AprilTag/GPS)
> load-bearing, not optional** — Phase 3 yaw correction is promoted from 'nice-to-have' to
> **required to meet spec**." — `shulib-v2-master-plan.md:250`

and locked decision #4 (`:483`) says "GPS/AprilTag aggressiveness: innovation-bounded,
covariance-weighted, per-tick-clamped **gated nudge (never snap)**" without restricting it to
position.

**Never-snap for heading** is therefore structural, exactly as for position: the policy can only
ever *add a bounded increment* to the bias; there is no code path anywhere that assigns a heading.

**Rejected alternative — defer to E4.** It is defensible (the EKF is where a principled heading R
belongs) and it was the safe choice. Rejected because the *plumbing* — an accumulator, a bounded
nudge, a wire slot — is needed identically whether the weight comes from a hand-written gain or a
Kalman gain, so deferring would postpone none of E4's real work while leaving the chunk's central
prize unclaimed and the guide's promise outstanding. E4 replaces the *weighting*, not the path.

**Rejected alternative — let the corrector write the heading directly / remove the re-stamp.**
Forbidden by the brief and wrong on its own terms: a source that can assign heading can snap it,
and a yaw snap poisons every field-relative command that follows.

**Rejected alternative — widen `CorrectionProposal`'s meaning.** Not needed. `providesHeading`
already exists, RESERVED, for exactly this; E3 makes it live and nothing about the struct's shape
changes.

### The consequence I did not expect, ruled the same way

If the bias corrects only the *published* heading, `PilonsOdometry` keeps rotating its per-tick
field delta by the **uncorrected** IMU heading, so the position prediction keeps accumulating a
cross-track error of roughly `bias × distance` — which is most of what heading drift actually costs
you over a 60 s run. So `Localizer` STEP 2 rotates the odometry delta by the learned bias before
folding it. Six lines, exact identity at `bias == 0` (guarded by an explicit early-out, so
bit-identity is structural rather than a floating-point argument), and `PilonsOdometry` itself stays
a pure predictor — the fused belief stays the Localizer's.

**23:24:40** — Next: T2 (tag map), T3 (PnP home + geometry), T4 (cadence). Then code.

---

## 23:33:50 — **T3 done: PnP landed as a free pure function, and it found something.**

`include/shulib/hal/vision_conversion.hpp` — named to sit beside `gps_conversion.hpp` and
`imu_conversion.hpp`, which is where the tree already puts "convert exactly once, at the edge"
helpers. `tagCornersToRobotPose(corners, intrinsics, tagSize, mount)`. The corrector does **not
include this header at all** — R2's adapter is the caller, exactly as `hal/vision.hpp:12-14` scopes
it. Algorithm: DLT homography (8x8 solve, partial pivoting, fixed storage) -> `G = K^-1 H` ->
scale from `|r1| = |r2| = 1` -> orthonormalize -> `r3 = r1 x r2` -> planar reduction.

`test/vision_conversion_test.cpp`, **13 cases, 273 assertions, all green.** Three deliberate layers
so an intrinsics/sign error cannot cancel:

1. **Hand-worked pixels** — literal pixel coordinates computed on paper, arithmetic written out in
   each comment, handed straight to PnP. No projector involved, so nothing can cancel. Five of
   them: the head-on anchor, a camera yawed 30 deg, a camera mounted off-centre, a tag turned
   30 deg off head-on, and a tag mounted above the camera.
2. **The projector is pinned against those same hand-computed pixels** *before* it is used as an
   oracle — including a case whose expectation is reasoned in words ("a left-yawed camera pushes a
   straight-ahead tag to the image RIGHT").
3. **Then** the sweep: 7 robot headings x 5 tag facings, robot at (-19.5, 31.25), camera at
   (4.5, -2.75) yawed 12 deg. **Neither heading 0 nor the origin appears anywhere.**

### FINDING (pre-existing risk for R2, found by a test that failed for the right reason)

I wrote a test asserting the documented corner order is load-bearing — that rotating it corrupts
the pose. **It went red, and the code was right, not the test.** Measured:

| corner-order error | position | heading | reprojection error |
|---|---|---|---|
| cyclic rotation (any of 3) | **unchanged** | **unchanged** | ~5e-14 |
| **reversed winding** | **unchanged** | **flipped 180 deg** | **~1e-14** |
| adjacent swap | — | — | rejected, `valid = false` |

A rotation is harmless because it rotates the tag frame about its own face normal, which a planar
reduction discards. **A reversal is catastrophic and completely silent**: it mirrors the tag plane,
so the face normal points 180 deg wrong — and the mirrored pose reprojects onto *the very same four
pixels*, so the reprojection error stays at machine zero. **No self-check anywhere in the pipeline
can detect a reversed winding.** Only a physical tag can. Written into the header, pinned by three
subcases, and it is R2's to settle (HA-72).

**23:34:20** — Next: T2, the tag map.

---

**23:52:40** — **T2 done.** `include/shulib/localization/tag_map.hpp`.

Ruling: **shulib ships NO built-in VEX field tag map, and adding one would be a mistake.** Nobody
on this project has a citable game-manual table of AprilTag field poses in hand. A plausible-looking
default would be invented geometry wearing the clothes of a specification, and every team that
forgot to override it would be silently localizing against fiction. So: the map is empty until a
caller fills it, an empty map makes the corrector decline with `RejectedNoTagMapEntry`, and that is
a loud diagnosable state instead of a quiet wrong one. Obtaining the real layout is R3's (HA-68).

**Provenance is enforced by precondition, not by convention.** `TagMap::add()` refuses an entry
whose `TagProvenance` is `Unspecified` or whose `source` citation is empty. `Invented` is a
legitimate answer — a guess *labeled* as a guess is what the A4 register exists to protect —
but "I did not say" is not, because the difference between a specified and an invented tag pose is
invisible in the arithmetic and total in the consequences. `anyInvented()` exists so a run anchored
to made-up field geometry cannot read the same as one anchored to a measured field. `add()` also
rejects duplicate ids — the mistake most likely to survive review, because the second entry simply
never wins a lookup and the map still "works".

**23:53:10** — **T4 done.** Two-method shape: `poll()` at vision cadence (the only method that
touches `ITagSource::tags()`, i.e. the only one that allocates), `propose()` every tick (never
touches the HAL). Calling `tags()` "only every fifth tick" was rejected — that does not remove the
heap allocation from the control loop, it just makes it intermittent, which is harder to diagnose.
The footgun this creates (nobody polls ⇒ silent) is answered by `pollCount()`, by
`RejectedNoFix` from the very first tick, and by a distinct `RejectedObservationAge` when a poller
that *was* running stops.

**23:54:30** — Core plumbing landed and the whole suite is green with **nothing broken**:

```
[doctest] test cases:     807 |     807 passed | 0 failed | 3 skipped
[doctest] assertions: 1081649 | 1081649 passed | 0 failed |
```

794 -> 807 cases (the 13 new PnP cases). **Zero pre-existing tests changed behaviour**, which is the
evidence for the bit-identity claim: with no heading-providing corrector `headingBias_` is exactly
`0.0` and both new code paths early-out.

Three new `GateReason` values, appended (wire-stable, E2's discipline):
`RejectedNoTagMapEntry = 9`, `RejectedTagRange = 10`, `RejectedObservationAge = 11`.
`RejectedSensorQuality = 8` is *reused* for a below-floor tag confidence — same statement, so no
new spelling was invented for it.

### The E2 handoff question, answered deliberately

E2 handed E3: *"The record has ONE gating slot. With two correctors, only the first silent
source's verdict survives. Decide it deliberately rather than inheriting it."*

**Ruled: an EXCEPTIONAL verdict outranks a ROUTINE one, where routine means `RejectedStaleFix`;
among equals the first still wins.** The reasoning is E2's own measurement — a healthy source
spends the large majority of its ticks stale (E2 measured 2520 stale against ~570 fresh in 30 s).
Under plain first-wins, the GPS's *ordinary* staleness would mask the tag corrector's *genuine*
failure — an unmapped tag id, a dead vision task — for essentially the entire run. One predicate,
deterministic, and `AppliedCorrection::source` still names whose verdict it is.

---

**00:12:40** — Tests landing. `tag_map_test.cpp` (11 cases, 353 assertions),
`apriltag_corrector_test.cpp` (23 cases), `apriltag_corrector_heading_test.cpp` (13 cases,
6761 assertions) — all green.

### A REAL BUG, found by the convergence test (fixed in this chunk, not worked around)

The convergence test failed with the bias settling at **12.1013 degrees for a 12.0 degree
correction** — an overshoot, which a proportional nudge toward an absolute measurement should be
structurally incapable of.

**Cause.** The corrector's latency compensation advanced the tag-derived heading by the rotation
observed since capture, and it took that rotation from `predicted.heading()` — which, since E3, is
`imu + learned bias`. So the "rotation since capture" term silently included **the estimator's own
bias learning during the latency window**, feeding the correction back into itself and inflating
the residual by up to one window's worth of nudges (8 ticks x 0.1 deg = 0.8 deg).

**Fix, at the root:** the rotation history is now taken from `imu_.heading()`. The IMU is the
authority on ROTATION and its reading is bias-free by construction, which is exactly what a
"how far has the robot turned since capture" term needs. Now pinned by
`CHECK(s.biasDeg() <= 12.0 + 1e-9)` — approached from below, never overshot — in two tests.

This is the kind of thing that only shows up when the loop is actually closed, which is why the
brief made convergence a required test rather than an optional one.

### Measured convergence (simulated camera, invented noise — the honest framing)

From a 4.0 degree IMU error, tag at 30", detector confidence 0.9, vision at 20 Hz against a
100 Hz loop:

| elapsed | remaining heading error |
|---|---|
| 1 s | 2.17 deg |
| 3 s | 0.50 deg |
| 15 s | 7.4e-05 deg |

Monotone on **every one of 1500 ticks** — never once moved away from truth. The per-tick bound
holds on all 2000 ticks of the 12-degree never-snap case.

**00:30:10** — `apriltag_corrector_blackbox_test.cpp` (7 cases) and
`apriltag_corrector_cost_test.cpp` (3 cases) landed.

**The cost pin is a real instrument, not an assertion.** `apriltag_corrector_cost_test.cpp`
**replaces the global `operator new`/`delete`** and counts. Three cases:
1. **The instrument is proved first** — `poll()` MUST show a non-zero count (it calls the frozen
   by-value `tags()` seam). Without that, a silently-broken counter would make every
   "zero allocations" assertion below vacuously true, which is the exact shape of a test that
   looks like evidence and is not.
2. `propose()` — **0 allocations across 20,000 ticks** (200 s of control loop, longer than three
   matches), with tags visible throughout and >3500 fixes actually folded, so the branches are
   exercised rather than early-outed.
3. A full `Localizer::update()` with the tag corrector registered and the heading path **live** —
   **0 allocations across 5,000 ticks.**

Counting is paused around `poll()`, which is allowed to allocate by design and is the whole point
of the two-method split.

**A second test-design bug caught, worth recording** because it would have made two tests pass for
the wrong reason: to provoke a HEADING-ONLY rejection I first perturbed the tag's relative heading.
That does not work — the inversion rotates the position term by the *robot's* heading, so the
derived position moves too and the **position** gate fires first. The correct construction is the
observation a robot at the right position but facing 80 degrees away would produce. Both tests now
also assert `reason == Accepted` on the same tick, proving position and heading really are gated
independently.

---

## 00:58:20 — **CORRECTION TO MY OWN 00:12 ENTRY. I OVERCLAIMED A BUG.**

At 00:12 I recorded, as a headline finding, that the convergence test had caught a **real bug**
producing a **12.1013 degree overshoot on a 12.0 degree correction**. **That is wrong, and I am
striking it.**

**What actually happened.** The failing assertion read
`CHECK( std::abs(s.biasDeg() - 12.0) < 0.1 )  values: CHECK( 0.101325 < 0.1 )`. I read
`|bias − 12| = 0.101` and **inferred the sign** — I wrote "12.1013, an overshoot" without ever
checking whether it was 12.1013 or **11.8987**. It was 11.8987. An **undershoot**: the tolerance
was simply too tight for 900 ticks of an asymptotic approach. Exactly the same assertion failed
again *after* my "fix" (`0.110655 < 0.1`), which should have told me the diagnosis was wrong; I
extended the run to 2000 ticks instead of asking why.

**Measured properly, both ways, same scenario** (12 degree IMU error, tag at 30", 20 Hz vision):

| rotation term taken from | max bias, whole run | at 5 s | at 9 s | at 20 s |
|---|---|---|---|---|
| **`imu_.heading()`** (what is in the tree) | 12.000 | 9.83388 | **11.8852** | 12.000 |
| `predicted.heading()` (the "bug") | 12.000 | 9.85176 | **11.8987** | 12.000 |

**Neither overshoots. Ever.** The entire difference is a ~0.1% convergence-rate change, and 11.8987
is precisely the number I misread as 12.1013.

**What I am keeping, and on what grounds.** The change to `imu_.heading()` stays, but the
justification is now **algebra, not a measurement**. The compensation term means "how far has the
robot TURNED since the frame was captured". Taking it from `predicted.heading()` — which since E3
is `imu + learned bias` — folds the estimator's own bias learning into a term that is supposed to
be rotation, which makes the *effective* heading gain depend on the latency window and the loop
rate. That is a hidden coupling and it is wrong on its own terms; it is simply not a bug anyone
would have noticed.

**Consequence for the mutation campaign: this mutation is a GREEN I cannot honestly close.** Any
test tight enough to separate 11.8852 from 11.8987 would be pinning an invented constant, which
this chunk explicitly refuses to do ("E3 proves logic, not constants; R4 measures"). It is
recorded as an open hole with its measurement, not papered over. See §THE HOLES in the completion
record.

Header comment and test comments corrected to match. The false claim is struck rather than
silently edited, because the process failure is the more useful record.

---

**01:12:40** — **Mutation campaign complete: 44 mutations, 43 RED, 1 GREEN, 0 build-fail,
0 SKIPPED.** `docs/internal/verify/verify-e3.sh`.

### A HARNESS FAULT, found and fixed mid-campaign — and it is E2's lesson repeating

Two mutations reported **GREEN having never actually run**. `grep -F` with a **multi-line**
pattern matches if **any single line** matches, so the harness's pre-flight check passed while
python's exact whole-string `replace` found nothing — the file was unchanged, the build succeeded,
the suite passed, and it was scored as a hole in the suite.

This is precisely the class of fault E2's postmortem was written about ("the report itself looked
fine; the tell was the count"). Fixed structurally rather than by correcting the patterns: the
harness now **byte-compares the file against its backup after the edit** and treats "the edit
changed nothing" as a loud SKIP that exits non-zero. A mutation that did not modify the file can
no longer be reported as anything.

### Holes found and CLOSED (2)

1. **PnP: the scale is taken from one column only.** Invisible on noise-free input — a perfect
   square projects to columns of exactly equal norm, so `2/(n1+n2)` and `1/n1` agree to the bit.
   Closed with an exact, hand-derivable probe: stretch the anchor image horizontally by f, and
   the two in-plane axes then imply depths 24/f and 24; the correct scale returns their
   **harmonic mean** (f=1.2 -> 21.8181..., f=2.0 -> exactly 16), the mutation returns 24/f.
2. **PnP: the singular-system guard removed.** It looked redundant — for *exactly* collinear
   corners the solve divides by zero and the finiteness screens catch it. It is not redundant.
   Four corners collinear **to within 1e-13 of a pixel** leave the pivot small but finite, the
   solve blows up to enormous finite numbers, and every downstream screen passes:
   *with the guard `valid=false`; without it, "a tag 5.99 inches away at x=5.73, facing us"* —
   plausible fiction that nothing downstream could detect. Found by fuzzing 4,000,000 random
   corner sets (which showed **zero** difference) and then searching structured near-degenerate
   families. Closed by a subcase using exactly that input.

Two further holes had already been closed earlier by targeted tests: `providesHeading` not being
load-bearing (the pre-existing test proved it via the heading GATE, not the flag), and the
odometry delta not being re-expressed under the learned bias (every heading test until then kept
the robot stationary).

### The hole I could NOT close, recorded rather than papered over (1)

**`corrector: rotation history taken from the PREDICTED heading, not the IMU`.** See the 00:58
correction above. Measured both ways: max bias 12.000 in both, 11.8852 vs 11.8987 degrees at 9 s,
identical at 20 s. **Neither overshoots.** The change stays because the algebra says the term
means "rotation", and rotation is the IMU's; but any test tight enough to separate a 0.1%
convergence-rate difference would be pinning an invented constant, which this chunk refuses to do.
Labelled KNOWN GREEN in the harness with the measurement beside it.

**01:13:10** — HA register: **HA-68 … HA-82 added** (15 entries). Three of them — HA-68 (the tag
map), HA-69 (the corner winding), HA-70 (the camera being level) — are flagged as **the most
dangerous entries in the register**, because unlike a tuning constant they do not degrade
gracefully: each produces a *confidently* wrong fix with a small residual and a high confidence.
Bidirectional reconciliation clean in both directions.

---

**01:41:10** — Documentation complete. Guide chapters 3, 11, 14, 15; roadmap checkbox + "you are
here"; HA register; README (not in the brief's scope, but E3 made one of its bullets *actively
false* — "No vision-based position correction yet"). The README's measured heading-accuracy
paragraph was deliberately **left alone**: that number comes from a test with no tag corrector in
it, and letting E3 be read as having improved it is precisely the overclaim this chunk exists to
avoid.

**Chapter 14 was the hard one, and the difficulty is the finding** (recorded in
`E3-COMPLETED.md` §8). Everything true about this chunk pulls toward optimism, and every honest
sentence about it is a sentence about a *simulation*. The temptation is not to lie — it is to
write "the mechanism the spec needs now exists and works", let the reader supply "so the spec is
probably met", and never technically claim it. Resolved by making the chapter state what it is
**not** claiming in the same breath as what it is, and by ordering the section
**measured → measured on what → still unmeasured** so the simulated-camera/invented-map/invented-
noise paragraph sits physically between the good news and the conclusion.

**01:42:00** — FINAL VERIFICATION, all run, output as observed:

```
suite      867 cases / 1,091,167 assertions | 0 failed | 3 skipped     SUCCESS
GUARD1     PASS      GUARD2     PASS
ARM gate   PASS      114 headers (was 111)
doc gates  self-test PASS · coverage PASS · fresh PASS · examples PASS · removability PASS
mutations  44 executed — 43 RED, 1 GREEN (recorded unclosed with its measurement), 0 SKIPPED
register   reconciliation clean in BOTH directions
git        NOTHING COMMITTED — everything in the working tree, as instructed
```

**E3 closed.** `E3-COMPLETED.md` written at the depth of the C/D/E records.
