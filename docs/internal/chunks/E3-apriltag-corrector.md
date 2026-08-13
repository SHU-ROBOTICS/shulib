# Chunk E3 — `AprilTagCorrector`

> **Phase E, chunk 3 of 4.** Predecessor: E2 (`GpsCorrector`, the first real corrector).
> **This is the chunk that can change the project's headline accuracy claim — which is exactly
> why it must not overclaim.**

**Workstream:** WS5 (localization) · **Milestone:** M3 · **Freezes:** none

---

## Why this chunk exists, and why it is *here*

E2 bounded *position* drift when the GPS strip is in view. **Heading drift is still uncorrected**,
and heading is the harder half: `docs/guide/14-what-it-cannot-do-yet.md` currently tells readers, in
plain words, that the team's `< 1°` end-of-run requirement is **"not reliably achievable on a real
robot"** without absolute yaw correction, and names this chunk as what closes it.

A tag observation is different in kind from a GPS fix. `TagObservation::poseInRobot` is a relative
**pose** — position *and* orientation. Seen against a tag whose field pose is known, it yields an
**absolute heading**, which no other source in the tree can provide. That is the prize, and it is
also the risk: heading is currently **IMU-owned by construction**, and E3 is where that stops being
true.

It comes before the EKF (E4) because E4's job is to weigh correctors against each other, which
requires more than one to exist. E2 gave it a second; E3 gives it a second *kind*.

---

## What already exists — read before designing

| Thing | Where | The part that matters |
|---|---|---|
| **`ITagSource` / `TagObservation` — FROZEN (F4)** | `hal/vision.hpp` | `tags()` returns `std::vector<TagObservation>` **by value**; each tag is `{id, poseInRobot, confidence}` |
| The header's own scoping of PnP | `hal/vision.hpp:10-14` | corners→pose PnP is "a pure, host-testable function **built with the M3 AprilTagCorrector**" |
| `ICorrector` — the signature to implement, again | `localization/i_corrector.hpp` | Same contract E2 satisfied |
| **`CorrectionProposal::providesHeading` — RESERVED** | `localization/correction.hpp:54` | And the documented additive path: a `headingNudge` on `FusionResult`, applied **before** the IMU re-stamp |
| `GpsCorrector` — the pattern to follow | `localization/gps_corrector.hpp` | Latency ring, staleness guard, adaptive R, honest gate naming |
| **The two-corrector substitution guard** | `localization/localizer.hpp` | Dead code with one corrector, load-bearing with two — E2 closed it; **E3 is what makes it live** |
| The 12″ innovation ceiling | `ComplementaryFusion::innovationGate` | E2 recorded it for E4; it will reject tag fixes too |

**Read first:** `hal/vision.hpp` **in full**; `correction.hpp` **in full** (especially the
`providesHeading` comment and `FusionResult`'s deliberate absence of a heading field);
`E2-COMPLETED.md`'s rulings (T1's honest gate naming and T2's scoped accuracy claim are the models
to follow); `docs/guide/14-what-it-cannot-do-yet.md`'s yaw-correction section — **you are editing
that section, so read what it currently promises.**

---

## The four rulings

### T1 — does E3 land absolute yaw correction, and by which path?

This is the chunk's central decision and the one with the largest blast radius.

Today the Localizer **re-stamps heading from the IMU as the final write**, so no fusion policy can
own heading. That is a deliberate M2 structure, not an accident. `correction.hpp` documents the one
sanctioned way to change it: a `headingNudge` on `FusionResult`, applied before the IMU re-stamp,
described there as **additive — it does not change the frozen `IPoseSource`/`ICorrector`/
`IFusionPolicy` signatures callers depend on.**

If you land yaw correction, **use that path and no other.** Do not widen `CorrectionProposal`'s
meaning, do not let the corrector write pose directly, and do not remove the IMU re-stamp. If you
conclude yaw correction belongs in E4 with the EKF instead, that is a legitimate ruling — say so and
say why, because the guide currently promises a reader that E3 is what closes this.

**Whichever way it goes, the never-snap invariant (§13 #4) applies to heading exactly as it does to
position.** A hard yaw reset mid-match is worse than a position snap, because every subsequent
field-relative command inherits it.

### T2 — where do the tags live, and how true are their positions?

Converting "a tag is at relative pose P" into "the robot is at absolute pose X" requires knowing
**where that tag is on the field**. That map is new configuration and it is the corrector's input,
not its invention.

Decide where it lives and how a team supplies it. Then be honest about provenance: a tag's field
pose is either **specified** (from the game manual — cite it) or **measured** (nobody has measured
anything). Anything not traceable to a published spec is an invented constant and gets an `HA-nn`
entry, because a tag map that is 2″ off produces a corrector that is confidently 2″ wrong — and
unlike noise, that error does not average out.

### T3 — PnP is built here but does not live here

`hal/vision.hpp` scopes this precisely: the corners→pose PnP "is a pure, host-testable function
built with the M3 AprilTagCorrector", and the *adapter* is what calls it (R2). The seam already
hands the corrector a reduced `poseInRobot`.

So: **write PnP as a free, pure function** that R2's adapter will call — not as a private method of
the corrector. If it lives inside the corrector, R2 will either duplicate it or reach into the
corrector, and both are worse than the seam that already exists. The DoD's "PnP verified against
synthetically-projected tags of known pose" belongs to that free function.

### T4 — `tags()` allocates, and the hot path cannot

`ITagSource::tags()` returns a `std::vector` **by value**, and it is **frozen (F4)** — you cannot
change it. The header is explicit that vision runs *off* the 10 ms control path and the adapter
polls at a lower rate.

A corrector whose `propose()` calls `tags()` every tick heap-allocates on the control loop, which
A1's cost contract forbids. Decide how the corrector consumes tags at vision cadence rather than
tick cadence, and **pin the cost** — do not assert it.

---

## Scope

### In
1. **`AprilTagCorrector`** implementing `ICorrector`: tag→absolute-pose reduction against the tag
   map, latency/staleness handling, confidence→R, gating, and the honest `GateReason` vocabulary
   (append-only, following E2's naming discipline).
2. **The PnP pure function** (T3), verified against synthetically-projected tags of known pose.
3. **The tag map** (T2), with provenance stated per tag.
4. **Yaw correction via the documented additive path, or a reasoned deferral** (T1).
5. **Convergence evidence** — the M2 red team already caught the failure mode here (*corrections not
   accumulating*); prove nudges accumulate and converge rather than each being individually sane and
   collectively useless.
6. **Two-corrector behaviour** — GPS and tags together. This is the first time that path is real.

### Out
- The EKF, covariance, true Mahalanobis gating → **E4**
- The `pros`/coprocessor adapter and real camera intrinsics → **R2**
- Object/bearing manipulation targeting (`IVision`) → **M4/F′**
- Raising the 12″ innovation ceiling → **E4** (recorded by E2; do not patch it from inside a
  corrector)

### Explicitly rejected
- **A hard pose or yaw reset on a tag sighting.** Low R is not a licence to snap. §13 #4.
- **PnP inside the corrector** (T3).
- **Per-tick `tags()` calls** (T4).
- **Tuning to the simulated camera's invented noise.** E3 proves logic, not constants; R4 measures.

---

## Test requirements

Every test names, in a comment, the bug it would catch.

- **PnP against synthetic projection** — build tags at known poses, project them, run PnP, recover
  the pose. **Use an independent projection**: if the projector and PnP share a camera model, an
  intrinsics or sign error cancels and the test proves nothing. This is the C1/C3/C4/E2 failure, and
  it is the single most likely place for it to recur.
- **Tag map correctness** — a tag seen from a known robot pose yields that pose back, computed by
  hand at several headings (a frame error is invisible at heading 0 and at the origin — use neither).
- **Heading**, if T1 lands it: the corrected heading converges toward truth; **it is nudged, never
  snapped**; and it survives the IMU re-stamp ordering.
- **Convergence** — repeated corrections accumulate. The M2 red-team failure mode, held as a test.
- **Two correctors** — GPS and tags disagreeing, agreeing, and one silent. The substitution guard
  E2 closed is load-bearing here for the first time.
- **Off-camera / no tags** — degrades to whatever else is available, visibly, never a phantom pull.
- **Cost** — no per-tick allocation (T4), pinned.
- **Through the blackbox** — every tag gating decision reconstructable from the file alone.

### Mutations
Swap the tag-map axes; flip the sign of the relative-pose inversion; make a tag fix snap instead of
nudge; break convergence accumulation; make no-tags return a low-confidence pull.

**A mutation that stays GREEN is a hole — log it, close it with a test that fails alone, and give it
a prominent place in the record.** Every chunk so far has found one; D3 four, E1 two, E2 one.
Gate the runner on build success.

---

## Documentation — read this section as carefully as the code

This chunk edits the passage a sceptical outside reader is most likely to check, so the standard is
higher than usual.

**`docs/guide/14-what-it-cannot-do-yet.md`** currently says the `< 1°` requirement is *"not reliably
achievable on a real robot"* without absolute yaw correction. If E3 lands yaw correction, that
sentence changes — **and the honest change is narrow.** What E3 can establish is that heading drift
is reduced *against simulated truth, with a simulated camera, using invented noise magnitudes*. It
establishes nothing about a real robot, a real camera, or the real IMU drift the claim depends on
(HA-20, still folklore until measured).

**Do not write that `< 1°` is achieved.** Do not write that it is "on track". Write what was
measured, on what, and what remains unmeasured — in that order. If the sentence becomes hard to
write honestly, that difficulty *is* the finding.

**Also in scope:** chapter 3 (knowing where you are) gains the second corrector and the yaw story —
this is another change to the *mental model*, not just a feature; chapter 11 gains any new
`GateReason` spellings; chapter 14's "exactly one absolute position source" bullet falls; the
glossary gains any new term. Guide examples change in the test file first, then get re-quoted —
never hand-edit a markdown code block.

---

## Definition of Done

- [ ] `AprilTagCorrector` implements `ICorrector` behind the unchanged signature
- [ ] PnP is a free pure function, verified against **independently** projected synthetic tags
- [ ] Tag map exists with per-tag provenance; invented values registered `HA-nn`
- [ ] T1 ruled: yaw correction landed via the documented additive path, or deferred with reasoning
- [ ] Never-snap holds for position **and** heading; proven in the record
- [ ] Convergence proven (the M2 red-team failure mode)
- [ ] Two-corrector cases pass, including disagreement and one-silent
- [ ] No per-tick allocation; cost pinned
- [ ] Every gating decision reconstructable from the blackbox
- [ ] Guide updated per the documentation section — **accuracy claims scoped to simulation**
- [ ] Suite green; both guards; ARM gate; all four doc gates

---

## Live progress log — required

`docs/internal/chunks/E3-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`E3-COMPLETED.md`** at the depth of C1–C5 / D1–D3 / E1 / E2.

**Do not commit. Do not push.**

---

## Landmines

- **Don't snap — especially heading.** A yaw reset poisons every field-relative command after it.
- **Don't bypass the `headingNudge` path** (T1). The IMU re-stamp is structure, not an obstacle.
- **Don't put PnP in the corrector** (T3).
- **Don't call `tags()` per tick** (T4).
- **Don't share a camera model between the projector and PnP.** It cancels.
- **Don't let an invented tag position look measured** (T2).
- **Don't claim `< 1°`.** Simulated truth, simulated camera, invented noise. Say so.
