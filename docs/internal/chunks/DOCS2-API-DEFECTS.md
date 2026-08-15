# DOCS2 — API defects found while documenting

> **RESOLVED AT DEFECTS1 (2026-08-15).** Every item below now carries a
> `> **DEFECTS1 → …**` line directly under its headline giving its outcome, so this list is
> self-describing and no reader has to cross-reference it against a chunk record to know what
> happened. The triage that produced those outcomes is in
> [`DEFECTS1-api-defect-triage.md`](DEFECTS1-api-defect-triage.md); the execution record,
> including the honest partials, is in [`DEFECTS1-COMPLETED.md`](DEFECTS1-COMPLETED.md).
>
> **84 items: the 83 below, plus `N1`, which DEFECTS1's own triage found.** Outcomes:
> **59 FIX · 15 REJECT · 6 ARGUE · 4 DEFER.** The REJECTs matter as much as the fixes — six
> items were already fixed at DOCS2 itself, and several more do not hold: `A9`'s two asserted
> consequences are both refuted by a two-line probe, and `I12`'s load-bearing premise is false.
> **The original text of every item is UNCHANGED below.** A finding that turned out to be wrong
> is left standing with its evidence, because editing it would destroy the record of what a
> careful reader believed and why.

> **This chunk's landmine L3 is "do not fix code."** Documenting 1,625 public entities means
> reading every one of them against its implementation, which is a wide net for real defects —
> DOCS1 caught `hal/battery.hpp` teaching a battery model the code rejects, and that was **one
> header, read by accident**.
>
> Every item below was found that way and **left in place**. Each names a file and line, states
> the defect, and carries the evidence that convinced the reader. Nothing here has been changed;
> where a defect made a comment impossible to write honestly, the COMMENT states the real
> behaviour and the defect is listed here.
>
> **Severity is not assigned.** These were found by reading, not by measuring, and a confident
> severity ranking would be the same kind of unearned claim the list exists to catch. They are
> grouped by shape and left for a person to triage.
>
> *(That triage happened at DEFECTS1. The paragraph above is DOCS2's, kept as written.)*

**83 findings**, across 58 headers — 73 found while writing the documentation, and a
further 10 (section E) found while CORRECTING it, when a reviewer's objection sent someone back
to the code and the code turned out to be the thing at fault.

Every item has a stable ID (`D3`, `A17`, `I8`, `E5`) — **cite the ID, not the line number**,
because line numbers move the moment anyone edits a header.

| Section | IDs | Count | What it means |
|---|---|---:|---|
| **D** — doc-contradicts-code | `D1`–`D18` | 18 | a shipped comment states a behaviour the implementation does not have |
| **A** — api-smell | `A1`–`A32` | 32 | the code is self-consistent, but the surface invites a caller to get it wrong |
| **I** — inconsistency | `I1`–`I22` | 22 | two parts of the library are each defensible and disagree with each other |
| **O** — other | `O1` | 1 | does not fit the three shapes above |

---

## D — the documentation and the code disagree

*(items `D1`–`D18`)*

### D1. `include/shulib/control/feedforward.hpp:13`

**The banner attributes a behaviour to CompensatedVoltage::brownoutLimited that no code implements — nothing in the library reads the flag.**

> **DEFECTS1 → FIX.** Banner corrected. Nothing reads brownoutLimited: the pipeline uses only `voltage`, and the park is driven by F2's deadlines.

<details>
<summary>Evidence (9 lines)</summary>

```text
Banner claim (feedforward.hpp:13-16): "compensateForBattery() limits a desired voltage to what the battery can actually deliver (+/-battery) and flags when it saturated -- so the motion layer knows it is voltage-starved and the guaranteed end-of-run park still fires as the battery collapses (SS M2, SS 18)."

The only production caller binds the struct and then uses one member of it (motion/command_pipeline.hpp:148-151):
        const control::CompensatedVoltage cv =
            control::compensateForBattery(ff.calculate(wheels[i]), vb);
        motors[static_cast<std::size_t>(i)]->setVoltage(
            diag::recoverWheelVoltage(cv.voltage, vb, *deps.faults, "MOT"));

Repo-wide, `brownoutLimited` appears only in its own declaration and in test/feedforward_test.cpp (4 asserts). No motion-layer code reads it, so no park-on-brownout path is driven by it. I documented the member for what it is today, not for the banner's claim.
```

</details>

### D2. `include/shulib/diag/controller_display.hpp:21`

**The banner claims the fault-name column width is "checked by static math here", but the file contains no static_assert of any kind.**

> **DEFECTS1 → FIX.** And the arithmetic was already WRONG — MECHANISM_STALLED is 17 chars against a 15-char budget, stale since F1. The static math now exists and asserts the property that matters (a truncated row still names one code), self-extending via a second assert.

<details>
<summary>Evidence (1 lines)</summary>

```text
Banner line 20-22: "The longest fault spellings (GPS_GATE_REJECT, MOTOR_OVER_TEMP: 15 chars) fit row 1's 19 columns beside \"flt \" exactly — checked by static math here, pinned by test, and the seam truncates (never wraps) if a future code outgrows it."  `grep -n static_assert include/shulib/diag/controller_display.hpp` -> NONE. Nothing in this file relates hal::ILineDisplay::kCols to the longest faultCodeName() spelling, so a 16-char future code would silently truncate rather than fail to compile. Compounding it, line_display.hpp's kCols is itself marked unverified ("the vendored PROS header implies 15 columns, community practice says 19, and neither is a measurement") — the "fits exactly" arithmetic rests on the 19 that is in doubt.
```

</details>

### D3. `include/shulib/diag/loop_monitor.hpp:87`

**worstDt()'s comment claimed the value resets, four lines above reset()'s comment saying it does not. REWROTE (rule 5) — this is the one existing /// I changed.**

> **DEFECTS1 → REJECT.** Already fixed at DOCS2; worstDt() reads "since construction" at HEAD and the generated page carries it.

<details>
<summary>Evidence (12 lines)</summary>

```text
Before, at line 87:
    /// Largest dt observed since construction/reset (the §18.3 "worst loop dt" summary
    /// quantity, consumed at C5). Time{0} until two ticks have happened.

The code, at line 96-98:
    /// Re-baseline after a DELIBERATE gap (run boundary, pause): the next tick() only
    /// baselines, so the gap is not misreported as an overrun. Keeps worstDt/counts.
    void reset() noexcept { hasLast_ = false; }

reset() touches only hasLast_; worstDt_ and overrunCount_ survive it. So "since construction/reset" is false, and the header stated both halves of the contradiction nine lines apart. Since the generator reproduces /// text onto the published page verbatim, the reference would have shipped both sentences.

After: "Largest dt observed since construction (the §18.3 …" — the word "/reset" removed, nothing else touched. FIXED (doc only; no code changed).
```

</details>

### D4. `include/shulib/hal/imu.hpp:45`

**The existing /// on pitch() calls pitch and roll 'canonical', but the PROS adapter that produces them records the as-mounted sign convention as UNMEASURED and passes it through unnegated — so the published page promises a fixed sign convention the implementation explicitly does not yet provide. Not rewritten (it is not wrong about anything in this header).**

> **DEFECTS1 → FIX.** pitch() no longer calls the sign "canonical" and now states HA-110's open sign convention; fixed together with I6.

<details>
<summary>Evidence (5 lines)</summary>

```text
imu.hpp:45 — `/// Chassis pitch and roll (canonical, for tip detection).`

hal/pros/imu.hpp:50-53 — `// PITCH/ROLL: get_pitch()/get_roll() degrees (-180,180) → math::Angle, UNNEGATED — the as-mounted sign convention is unmeasured (HA-110); the tip detector consumes magnitudes first, and the bench settles the signs.` and, at the call sites, imu.hpp:136/146 `lastPitch_ = math::Angle::degrees(deg);  // sign as-mounted: HA-110`.

Everywhere else in this library 'canonical' names a settled convention (heading() is CCW-positive, +X = 0, and the adapter negates to get there). For pitch/roll it currently means only 'a math::Angle in radians'. A reader trusting the generated page would branch on the sign of pitch() for tip detection and get an as-mounted, unverified answer. Because roll() was undocumented I was free to say this at roll()'s own declaration, and did; pitch()'s existing comment is left for the orchestrator to rule on.
```

</details>

### D5. `include/shulib/hal/pros/gps.hpp:163`

**faultedReads() is documented as counting reads screened to no-fix, but the offset-unverified / offset-rejected path screens a read to no-fix WITHOUT incrementing the counter — so the one failure the class treats as permanent is the one it reports zero times.**

> **DEFECTS1 → FIX.** The offset-unverified screen now counts. Test note: the ONLY route to offsetRejected_ is unreadable-at-boot then readable-and-nonzero — verifyOffset short-circuits once the boot check passes.

<details>
<summary>Evidence (19 lines)</summary>

```text
The counter's own doc (gps.hpp:131-132):
    /// How many reads were screened to no-fix (T7 observability).
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

refresh() has two screen-to-no-fix paths and only one counts:
    void refresh() const {
        verifyOffset(/*bootPhase=*/false);
        if (!offsetVerified_) {
            hasFix_ = false;
            return;                      // <-- screened to no-fix, NOT counted
        }
        ...
        if (!std::isfinite(status.x) || ...) {
            faultedReads_ += 1;          // <-- counted
            hasFix_ = false;
            return;
        }

Once verifyOffset() sets offsetRejected_ = true (the deferred discovery of a configured firmware offset), the device is no-fix for the entire run and faultedReads() stays at 0 forever. An operator using the counter to answer "why is the GPS dead?" gets the least informative possible answer in precisely the case the header's HA-06 discussion cares most about.
```

</details>

### D6. `include/shulib/hal/pros/rotation.hpp:95`

**The banner promises a faulted rotation sensor is "never zero", but the last-good caches are initialized to 0.0, so a pod that faults from boot reports exactly zero forever.**

> **DEFECTS1 → FIX.** REJECTED by triage, OVERRULED by verification and by me. The harm story is refuted (OdoStallCheck reads IMotor and works in deltas; IRotation::velocity() has no consumer at all) — but the refuted story WAS the shipped text, and the F4 interface header still carried the unqualified claim its sibling hal/motor.hpp already caveats. Fixed as documentation.

<details>
<summary>Evidence (10 lines)</summary>

```text
Banner claim (rotation.hpp:20-23): "hold the last good value, never propagate, never zero (a zeroed tracking wheel reads as 'the robot stopped' — the exact dead-encoder runaway the loop's ODO_STUCK cross-check exists to catch)".

The code that contradicts it (rotation.hpp:95-96):
    mutable units::AngleDim lastPosition_{0.0};
    mutable units::AngularVelocity lastVelocity_{0.0};

Both readers return the cache on PROS_ERR:
    if (centideg == PROS_ERR) { faultedReads_ += 1; return lastPosition_; }

If the sensor is unplugged or dead at construction, NO read ever succeeds, so the cache is never primed and `position()` returns 0.0 rad and `velocity()` returns 0.0 rad/s for the whole run. `velocity()` is the sharp one: a constant 0.0 rad/s is literally the "the robot stopped" reading the banner names as dangerous, and it is indistinguishable from a genuinely stationary robot. `faultedReads()` is the only channel that says otherwise, and nothing in hal/ raises on it. The mitigation the banner relies on (a FROZEN value the ODO_STUCK cross-check can see) holds only once a good read has landed.
```

</details>

### D7. `include/shulib/localization/correction.hpp:103`

**Multi-line trailing ///< notes are mis-attributed by the generator: every continuation line becomes the NEXT member's documentation, so three published members carry another member's sentence and two lose their own. FusionResult::audit is currently documented with a sentence about appliedConfidence. Pre-existing; I did not touch these comments (my two additions are deliberately single-line for this reason).**

> **DEFECTS1 → REJECT.** Already fixed at DOCS2 — _strip_doc excludes ///< at HEAD; verified on the published page.

<details>
<summary>Evidence (20 lines)</summary>

```text
The code, correction.hpp:102-106 and 126-132:
    double appliedConfidence = 0.0;      ///< [0,1] confidence of the strongest applied fix (0 if none);
                                         ///< drives how much the drift accumulator is cleared.
    GateAudit audit{};                   ///< WHY this tick decided as it did (E1) — APPENDED, so every
                                         ///< existing positional construction of this struct still
                                         ///< compiles and means the same thing.

The generator disagrees. api_doc_tool.py's _strip_doc() matches any line starting with '///', so a continuation '///< text' line is swallowed as a PENDING doc block for the following declaration, and _read_declaration() prefers pending over the trailing note ('doc = list(pending) if pending else ([trailing] if trailing else [])'). Parsing the header in-process with the real tool prints:
  (field) appliedConfidence
      DOC: [0,1] confidence of the strongest applied fix (0 if none);      <- line 2 lost
  (field) audit
      DOC: < drives how much the drift accumulator is cleared.             <- appliedConfidence's sentence
  (field) headingNudge
      DOC: < existing positional construction of this struct still < compiles and means the same thing. The bounded heading INCREMENT to fold into the estimator's ...
  (field) AppliedCorrection::audit
      DOC: the gate's own account of this tick (E1) — this is the          <- line 2 lost
  (field) dtheta
      DOC: < value the record producer stamps into the §18.2 slots The NET heading change applied this tick, ...

So the published reference tells a reader that FusionResult::audit 'drives how much the drift accumulator is cleared' — which is false of audit and true of appliedConfidence. This passes check-coverage (the gate counts comments), which is exactly the hidden-gap failure the chunk exists to prevent. Two fixes are possible and both are outside a documentation pass: convert these trailing notes to /// blocks above their fields, or teach _strip_doc/_read_declaration that a '///<' line continues the previous trailing note. Same shape exists in other headers wherever a ///< wraps.
```

</details>

### D8. `include/shulib/localization/i_fusion_policy.hpp:5`

**The banner and fuse()'s own /// both say a fusion policy returns POSITION only; since E3 it also returns a bounded heading increment that the Localizer folds into a persistent heading bias.**

> **DEFECTS1 → FIX.** Banner and fuse() now describe the post-E3 contract (position AND a bounded heading increment).

<details>
<summary>Evidence (1 lines)</summary>

```text
Banner, line 5-6: "Given the predicted pose and the valid proposals, it returns the corrected POSITION only — heading is re-stamped from the IMU by the Localizer afterward, so a policy can never own heading at M2."  fuse()'s /// at line 37-38: "Fold the valid proposals into the predicted position and return the corrected (x, y) plus the audit flags."  But correction.hpp's FusionResult now carries `units::AngleDim headingNudge{}; bool headingApplied; bool headingGated; bool headingClamped;` and localizer.hpp:263-270 acts on them: `const double nudgeH = fr.headingNudge.value(); if (fr.headingApplied && std::isfinite(nudgeH)) { const double next = headingBias_ + nudgeH; ... headingBias_ = next; }`. complementary_fusion.hpp:261-262 fills them: `result.headingNudge = units::AngleDim{headingSum}; result.headingApplied = headingApplied;`. The "at M2" hedge does not save the flat clause "returns the corrected POSITION only", and this banner is reproduced verbatim onto the published API page. NOT FIXED — rule 5 (rewriting an existing /// only when factually wrong) plus rule 1 kept me out of the banner.
```

</details>

### D9. `include/shulib/localization/localizer.hpp:9`

**The banner's five-step numbering is off by one against the STEP labels in update() from step 3 onward, and the generator reproduces that banner verbatim onto the published page.**

> **DEFECTS1 → FIX.** Banner renumbered to match update()'s STEP labels, and the downstream self-inconsistency ("STEP 4 stamped") corrected with it.

<details>
<summary>Evidence (13 lines)</summary>

```text
Banner (lines 9-15) numbers the tick:
    3. fuse - ask each corrector ... fold the valid ones in
    4. heading - compose the fused heading from the IMU as the LAST write
    5. publish - recompute the quality scalar + categorical flags

The code labels it differently:
    line 214: // STEP 3 - gather VALID proposals
    line 260: // STEP 4 - fuse: an innovation-bounded, per-tick-clamped nudge
    line 267: // STEP 5 - heading composed from the IMU as the LAST write + publish

So the banner's "fuse" is STEP 4 in code, the banner's "heading" is STEP 5, and the banner's separate step 5 "publish" has no label of its own — it is folded into the code's STEP 5. Steps 1 and 2 agree; 3, 4 and 5 do not.

The banner is also inconsistent with itself downstream: line 18 says "so STEP 4 stamped the raw IMU reading", using the banner's numbering, while the code's STEP 4 is the fusion call. This costs more than usual here because api_doc_tool reproduces the header banner verbatim into docs/api/localizer.md, so a reader who arrives from the generated page and jumps to the source lands on the wrong step every time from 3 onward.
```

</details>

### D10. `include/shulib/manipulation/mechanism_outcome.hpp:54`

**Multi-line `///<` enumerator continuations are attributed to the NEXT enumerator, so the published page states confident wrong sentences about Unconfirmed and TimedOut. The coverage gate scores them as documented, making this a HIDDEN gap of exactly the class the tool exists to prevent.**

> **DEFECTS1 → REJECT.** Already fixed at DOCS2 — the published mechanism_outcome page carries each enumerator's own complete sentence.

<details>
<summary>Evidence (23 lines)</summary>

```text
Header (lines 45-61) uses a continuation style:
    Succeeded = 1,    ///< completed AND confirmed (where the operation defines a
                      ///< confirmation; completed, where it does not)
    Unconfirmed = 2,  ///< the operation ran to completion and the confirmation
                      ///< said the world did not change — healthy mechanism,
                      ///< failed task. Strategy, not pathology: NO fault.
    TimedOut = 3,     ///< the watchdog fired before the operation completed

LIVE in the published tree — docs/api/mechanism_outcome.md today renders:
    ### `MechanismOutcome::Unconfirmed`  ->  "< confirmation; completed, where it does not)"
    ### `MechanismOutcome::TimedOut`     ->  "< said the world did not change — healthy mechanism, < failed task. Strategy, not pathology: NO fault."

Root cause is in tools/api_doc_tool.py `_parse_enum_body`: `_strip_doc(raw)` matches ANY line starting with `///` (a `///<` continuation included) and is tested BEFORE the enumerator body is processed, so a continuation line lands in `pending` and is consumed as the doc for the next enumerator:
    doc = _strip_doc(raw)
    if doc is not None:
        pending.append(doc)   # a `///<` continuation ends up here
        i += 1; continue
    ...
    if k == 0 and pending: edoc = list(pending)

Second symptom, same cause: Stalled's second line (`///< FaultCode::MechanismStalled raised`, line 60) is never consumed by anything, so the page silently drops the half of that sentence that names the fault.

NOT FIXED, deliberately: the root cause is in tools/api_doc_tool.py, outside my assigned header list, and the enumerator comments themselves are not factually wrong — only the generator's attribution of them is. Fixing it in the header (restructuring five `///<` comments into `///` blocks above each enumerator) would paper over a parser bug that will re-fire on every other multi-line `///<` in the tree.
```

</details>

### D11. `include/shulib/math/twist2d.hpp:10`

**The file banner states ChassisSpeeds is FIELD frame, but the type has been frame-agnostic since the explicit math::Frame parameter was introduced, and the kinematics seam accepts only a BODY-frame one.**

> **DEFECTS1 → FIX.** ChassisSpeeds is frame-agnostic; the pre-C4 "FIELD frame" line is gone.

<details>
<summary>Evidence (11 lines)</summary>

```text
twist2d.hpp:9-10 (banner):
//   * ChassisSpeeds  — a commanded chassis velocity (what motion asks the
//                      drivetrain to do; FIELD frame until Chassis rotates it).

Against the code that consumes it:
  chassis.hpp:385   void drive(const math::ChassisSpeeds& speeds, math::Frame frame)   // frame explicit, no default
  kinematics.hpp:66 [[nodiscard]] virtual WheelSpeeds toWheels(const math::ChassisSpeeds& body) const = 0;  // "a BODY-frame commanded twist"
  command_pipeline.hpp:87-88  math::ChassisSpeeds body{};  ///< "The final achievable command in the BODY frame (post every clamp)"
  frame.hpp:42/53   fieldToRobot / robotToField both take AND return ChassisSpeeds

A reader who trusts the banner will assume any ChassisSpeeds they are handed is field-frame; roughly half of the ones in the tree are body-frame. Left as-is per this chunk's no-code-changes rule; the /// comments I added state the frame-agnostic truth, which now disagrees with the banner four lines above them.
```

</details>

### D12. `include/shulib/motion/hold_pose.hpp:39`

**HoldPose's only deadline is holdFor + 1.0 s from start(), and that same watchdog is what bounds the wait-for-live boot window — so a short hold started during IMU calibration exits TimedOut before its hold window ever begins, and the caller has no timeout parameter with which to budget for boot.**

> **DEFECTS1 → FIX.** Watchdog now armed with max(holdFor + slack, effective timeout). HONEST PARTIAL: no test pins it and mutation M21 stayed GREEN — MotionRig's localizer is seeded live, so the boot-window scenario needs a cold-boot rig this chunk did not build.

<details>
<summary>Evidence (8 lines)</summary>

```text
hold_pose.hpp:39-44 and 47-49 pass `timeout = 0.0`; move_to_pose.hpp:273-275 then sets
    watchdog_{options.holdFor > 0.0 ? options.holdFor + kHoldSlack : (timeout > 0.0 ? timeout : config.defaultTimeout), ...}
with `static constexpr double kHoldSlack = 1.0;` (move_to_pose.hpp:294). move_to_pose.hpp:114-119 checks that same watchdog inside the wait-for-live branch:
    if (!live && !everLive_) { ... if (watchdog_.expired()) return exitTimedOut("timed out waiting for a live estimate"); }

So HoldPose(deps, /*holdFor=*/0.5) has a total boot budget of 1.5 s. A V5 IMU calibration is ~2 s, and motion.hpp:82-84 states the contract as 'Callers budget timeouts to cover boot' — but HoldPose exposes no timeout knob at all, so this one cannot be budgeted.

Contradicted claim, move_to_pose.hpp:290-291: '/// The hold watchdog only backstops a clock pathology; hold-mode's own deadline (holdFor) is the real exit.' It is not only a clock backstop — it is the sole bound on the boot wait.
```

</details>

### D13. `include/shulib/motion/motion_config.hpp:141`

**validate() checks defaultTimeout > 0 but not finiteness, so an infinite defaultTimeout produces a watchdog that never expires — a motion that can hang, which is the one thing the watchdog exists to prevent. REPORTED, NOT FIXED.**

> **DEFECTS1 → FIX.** Finiteness added at BOTH layers — MotionConfig::validate()'s five scalars and Watchdog's own ctor, which is where the "a motion can never hang" claim lives.

<details>
<summary>Evidence (13 lines)</summary>

```text
motion_config.hpp:141 `SHULIB_PRECONDITION(defaultTimeout > 0.0, "MotionConfig: defaultTimeout must be > 0");` — inf passes. control/watchdog.hpp:16 `SHULIB_PRECONDITION(timeout > 0.0, "Watchdog: timeout must be > 0");` — inf passes there too. watchdog.hpp:5 claims "a motion can never hang" and "even a stalled control loop still exits (-> TimedOut)". The asymmetry is the tell: move_to_pose.hpp:279-280 guards the CALLER-supplied timeout with `SHULIB_PRECONDITION(std::isfinite(timeout) && timeout >= 0.0, ...)`, but timeout==0 selects config.defaultTimeout (move_to_pose.hpp:275), which never gets that check. Compiled and ran:

  motion::MotionConfig cfg;
  cfg.defaultTimeout = std::numeric_limits<double>::infinity();
  cfg.validate();                                 // does NOT raise
  control::Watchdog wd{cfg.defaultTimeout, clk};  // the timeout==0 path
  wd.start(); clk.advance(units::Time{1.0e9});

output:
  validate() accepted defaultTimeout = inf
  after 1e9 s: watchdog.expired() = 0

A MoveToPose/StrafeTo/TurnTo built with `timeout = 0` on that config runs forever with no TimedOut exit and no end-of-run park. Note validate() also omits std::isfinite on maxLinearSpeed / maxAngularSpeed / maxWheelSpeed / rotationRadius, all of which pass `> 0.0` at infinity.
```

</details>

### D14. `include/shulib/motion/move_to_pose.hpp:327`

**In hold mode the watchdog is never consulted after the first live tick, so the kHoldSlack watchdog documented as a "backstop" is armed but never read on the running path.**

> **DEFECTS1 → FIX.** Fixed with D12; the comment now says what the hold watchdog actually is (the boot bound plus a floor under holdFor), not the live backstop the code never read.

<details>
<summary>Evidence (15 lines)</summary>

```text
kHoldSlack's doc claims a live backstop (move_to_pose.hpp:327-328):
    /// The hold watchdog only backstops a clock pathology; hold-mode's own
    /// deadline (holdFor) is the real exit.
    static constexpr double kHoldSlack = 1.0;

and the ctor arms the watchdog with holdFor + kHoldSlack when holdFor > 0. But tick()'s exit verdict reads watchdog_ only in the non-hold branch:

    if (opts_.holdFor > 0.0) {
        if ((now.value() - holdStart_) >= opts_.holdFor) { ... }
    } else {
        if (transSettled && headSettled) { ... }
        if (watchdog_.expired()) { return exitTimedOut("target not reached"); }
    }

grep confirms watchdog_.expired() appears at only two sites: line 142 (the wait-for-live branch, before holdStart_ is set) and line 185 (the else/non-hold branch). HoldPose does not override tick(). So once a HoldPose goes live, holdStart_ + holdFor is the ONLY exit; the extra second of watchdog is unreachable, and motion.hpp's "a motion can NEVER hang: the watchdog is armed in start() and no code path disarms it" is true only in the letter — nothing disarms it, but in hold mode nothing reads it either.
```

</details>

### D15. `include/shulib/motion/odo_stall_check.hpp:37`

**The banner's "a single dead DRIVE encoder halves the mean" is exact only for a 2-motor drive; for the 4-wheel drive named in the same sentence it is 3/4, not 1/2.**

> **DEFECTS1 → FIX.** (n-1)/n, not "halves" — 1/2 at n=2, 3/4 on the X-drive named in the same sentence, 2/3 on the H-bot. The conclusion is stronger than the wrong figure implied.

<details>
<summary>Evidence (9 lines)</summary>

```text
Banner lines 37-39:
    //   * MEAN |Δshaft| over all drive wheels: a single dead DRIVE encoder halves
    //     the mean rather than zeroing it (still trips), while an X-drive strafe
    //     (all four wheels spinning) reads full spin travel.

The code, line 136:
    const double meanShaftDelta = (n > 0) ? sumAbsShaftDelta / static_cast<double>(n) : 0.0;

With n wheels, one dead encoder leaves (n-1)/n of the mean. The same bullet names an X-drive — four wheels — for which that is 0.75×, not 0.5×. The conclusion ("still trips") holds and is in fact stronger than claimed, so this is a wrong number in the reasoning rather than a wrong design. It matters because the generator reproduces each header banner verbatim into the published page, so the figure ships as documentation. NOT FIXED.
```

</details>

### D16. `include/shulib/sequence/run_guard.hpp:195`

**The banner says the end action is 'refused' once hardStopAt passes, but run() invokes the end-action callable unconditionally and reports endActionRan = true even when the hard floor already fired during scoring.**

> **DEFECTS1 → FIX.** Banner corrected: what the hard floor refuses is the end action's MOTIONS and WAITS, not its invocation — the guard cannot preempt caller code.

<details>
<summary>Evidence (1 lines)</summary>

```text
Banner (T2): 'hardStopAt — BE SAFE, unconditionally: every device is forced safe and everything, the end action included, is refused from here on.' run() has no floorFired_ guard: `cancelAll(); logActStart(); inEndAction_ = true; report.endActionRan = true; report.endActionSucceeded = invokeEndAction(...)`. Reachable exactly in the case the banner's own honesty section describes — scoring code that keeps the CPU past both deadlines (an unconditional retry loop) — after which logActStart() prints a NEGATIVE '(%.2fs to the hard stop)' from `floorDeadline_ - clock_->now()`. What is actually refused is the end action's MOTIONS (each pace()/waitFor re-fires the floor and cancels), not its invocation; the field doc at line 195 states the true behaviour, so the banner sentence is the one that misleads a reader budgeting a park.
```

</details>

### D17. `include/shulib/units/quantity.hpp:14`

**The design banner's canonical-unit list omits amperes, though the header itself declares the current dimension and the Current alias. The banner is reproduced verbatim onto the generated page, so the published contract is wrong.**

> **DEFECTS1 → FIX.** The ampere is in the canonical-unit list.

<details>
<summary>Evidence (8 lines)</summary>

```text
The banner declares five base dimensions and then lists four canonical units:

    //     L = length, A = angle, T = time, E = electric potential (voltage),
    //     I = electric current (amperes).
    ...
    //   * the stored value is ALWAYS canonical: inch, radian, second, volt.

but the header goes on to declare `using Current = Quantity<0, 0, 0, 0, 1>;   // amperes`, and hal/pros/battery.hpp stores `units::Current{ma / 1000.0}` — canonical amperes, which is not a derived combination of inch/radian/second/volt. NOT FIXED: editing the banner is outside "add /// comments", and the generator copies banners into published pages verbatim, so this needs a deliberate owner. I did correct the same omission in value()'s own /// (reported under rewrote_existing).
```

</details>

### D18. `tools/api_doc_tool.py:988`

**_parse_enum_body's docstring says a `///` run above a line documents the FIRST enumerator on it; for a one-line enum DECLARATION that is false — the run is consumed as the enum type's own doc and reaches no enumerator. This is exactly the case the docstring says the DOCS2 rewrite exists to handle.**

> **DEFECTS1 → FIX.** Docstring corrected, and it understated the damage: a /// run above an enum's declaration reaches NO enumerator, not just "the rest".

<details>
<summary>Evidence (16 lines)</summary>

```text
Docstring claim (line 988):
    a `///` run above a line documents the FIRST enumerator on it and a `///<`
    documents the LAST. On the house one-enumerator-per-line form those are the
    same enumerator; on a one-liner with several, the rest are undocumented and
    the gate says so

Code: _parse_namespace_scope collects `pending` from _strip_doc, hands it to
TypeDecl(...), THEN calls _parse_type_body -> _parse_enum_body(lines, start=i,...)
with `i` = the enum's own declaration line. _parse_enum_body initialises
`pending = []` and never sees the run above the declaration.

Measured (probe A above): with a /// run above `enum class LogLevel { Error, ... };`
the FIRST enumerator Error gets doc: [] — not the docstring's promise. Only the
last is reachable. The docstring understates the damage ('the rest are
undocumented' -> actually all but the last are), which will mislead the next
agent into thinking a /// above the line closes Error.
```

</details>

---

## A — the API itself has a sharp edge

*(items `A1`–`A32`)*

### A1. `include/shulib/chassis/robot_context.hpp:63`

**Nothing anywhere validates that driveMotors has at least as many motors as the installed IKinematics has wheels, and the commanding path indexes the span by wheel index with no bound check — a short motor list is silent out-of-bounds UB on every tick, not a precondition failure.**

> **DEFECTS1 → FIX.** Cross-check added to MotionDeps::validate(), the one bundle holding both the context and the kinematics. Mutation-proven.

<details>
<summary>Evidence (11 lines)</summary>

```text
RobotContext validates only emptiness and non-null-ness (robot_context.hpp:63): SHULIB_PRECONDITION(!cfg_.driveMotors.empty(), "RobotContext: driveMotors is empty"). MotionDeps::validate() (motion.hpp:184-190) checks five pointers for null and nothing else — it holds both ctx and kinematics and is the one place that could cross-check them.

The commanding path then indexes motors by WHEEL index, unguarded (command_pipeline.hpp:146-152):
    const auto motors = deps.ctx->driveMotors();
    for (int i = 0; i < wheels.size(); ++i) {
        ...
        motors[static_cast<std::size_t>(i)]->setVoltage(...);
    }
wheels.size() is kinematics->wheelCount(). Build a RobotContext with 3 motors and install XDrive (4 wheels) and motors[3] reads one past the end of the span: std::span::operator[] is unchecked, so this is UB with no diagnostic, on the hot path, on every tick.

The mismatch was clearly anticipated elsewhere — every RECORD-producing loop guards it (motion_scheduler.hpp:1041, turn_to.hpp:199, chassis.hpp:577, drive_brake.hpp:172, move_to_pose.hpp:201 all iterate 'i < motors.size() && ...'), and odo_stall_check.hpp:101 has a SHULIB_PRECONDITION on motors.size(). Only the path that actually drives the robot is unguarded. Reported, not fixed. I documented the caller's obligation honestly on both driveMotors entities rather than implying a check exists.
```

</details>

### A2. `include/shulib/control/feedforward.hpp:96`

**compensateForBattery() has no finiteness precondition on `desired`: a NaN passes straight through, unclamped AND unflagged, from the one function whose job is to bound the commanded voltage.**

> **DEFECTS1 → FIX.** The FLAG was the lie, not the value: `!(|d| <= b)` lands NaN on the true side. The NaN still passes through deliberately — recovery belongs at the motor edge, and a throw here would abort a motion an A3 hostile sensor should only degrade.

<details>
<summary>Evidence (7 lines)</summary>

```text
SHULIB_PRECONDITION(battery.value() >= 0.0, "compensateForBattery: battery voltage must be >= 0");
... return CompensatedVoltage{units::Voltage{std::clamp(d, -b, b)}, std::abs(d) > b};

std::clamp is `v < lo ? lo : hi < v ? hi : v` — both comparisons are false for NaN, so NaN is returned; and std::abs(NaN) > b is false, so the saturation flag reads clean. Confirmed by running it against the real header:
    voltage=nan isnan=1 brownoutLimited=0

Inconsistent with its own file: Feedforward's constructor (line 65) does check std::isfinite on all three gains. Mitigated downstream only by chance — diag::recoverWheelVoltage in command_pipeline.hpp happens to screen the value afterwards — so any other caller gets a silent NaN.
```

</details>

### A3. `include/shulib/diag/health_monitor.hpp:111`

**HealthMonitor's constructor is not explicit, so a FaultLatch implicitly converts to a HealthMonitor.**

> **DEFECTS1 → FIX.** explicit added.

<details>
<summary>Evidence (1 lines)</summary>

```text
HealthMonitor(FaultLatch& faults, const HealthMonitorConfig& config = {})  — the defaulted second parameter makes this a one-argument converting constructor, so `HealthMonitor m = someLatch;` compiles, and a FaultLatch& will silently convert at any call site taking HealthMonitor by value or const&. Sibling diag/ types mark their single-argument constructors explicit (tick_attribution.hpp:50: `explicit TickAttribution(hal::IClock& clock) noexcept`), so this reads as an oversight rather than a decision.
```

</details>

### A4. `include/shulib/diag/level_filter_sink.hpp:115`

**Override::tag() rebuilds a string_view from a NUL-terminated buffer, so a subsystem tag containing an embedded NUL is silently truncated and can collide with — or be made unreachable by — another tag.**

> **DEFECTS1 → FIX.** Tags matched by length, so an embedded NUL no longer makes a tag's own dial unreachable by its own name.

<details>
<summary>Evidence (8 lines)</summary>

```text
setLevel() stores the tag by length and then NUL-terminates (level_filter_sink.hpp:77-78):
    std::memcpy(slot.tagBuf, subsystem.data(), subsystem.size());
    slot.tagBuf[subsystem.size()] = '\0';

but it is read back through an implicit const char* -> string_view conversion, which stops at the first NUL (line 115):
    [[nodiscard]] std::string_view tag() const noexcept { return tagBuf; }

Given `setLevel("A\0B"sv, Debug)`, tag() reports "A". A later `setLevel("A"sv, Warn)` matches that slot and overwrites the "A\0B" entry, while passes("A\0B") compares "A\0B" == "A" and is false — so the original tag's own dial is unreachable by its own name and is silently steering a different channel. Low practical risk (tags are literals like "MOT"), but the header states the table is LOUD about every failure mode — "exceeding it is a LOUD precondition, never a silently ignored setLevel" — and this is a silent one it does not cover.
```

</details>

### A5. `include/shulib/diag/line_format.hpp:81`

**appendSanitized is documented as truncating at `cap`, but a truncated string emits cap+3 bytes, and the 3-byte ellipsis itself can be split mid-UTF-8 by appendRaw — the exact breakage the UTF-8 back-off exists to prevent.**

> **DEFECTS1 → FIX.** The 3-byte marker is emitted only when it fits. The first test for this MISSED it (mutation M13 stayed green): filling to kCapacity-2 leaves zero room, and appendRaw with zero room writes nothing. Needs 1-2 bytes free.

<details>
<summary>Evidence (1 lines)</summary>

```text
Contract in the banner: "truncation backs off UTF-8 continuation bytes and marks itself with '…'"; the /// on the member says "truncates at `cap` with '…' on a UTF-8 boundary". Code: the copy loop writes up to `take` (<= cap) bytes, then `if (truncated) { appendLiteral("…"); }` unconditionally appends 3 more, so a caller sizing a fixed column by `cap` gets cap+3. Worse, appendLiteral -> appendRaw computes `const std::size_t room = kCapacity - n; const std::size_t take = len < room ? len : room;` — with room == 1 or 2 it memcpy's a PARTIAL "…" (0xE2, or 0xE2 0x80), leaving a truncated multi-byte sequence at the end of the line. Unreachable at kCapacity 384 with today's renderers, but it is the one code path in this file that can emit invalid UTF-8.
```

</details>

### A6. `include/shulib/diag/line_format.hpp:122`

**appendNum's compaction trigger compares the rendered byte count against an absolute constant instead of against the requested `width`, so any column wider than 10 is unconditionally compacted away.**

> **DEFECTS1 → FIX.** Compaction now compares against the caller's width as well as the constant.

<details>
<summary>Evidence (1 lines)</summary>

```text
int len = std::snprintf(tmp, sizeof tmp, "%*.*f", width, prec, v);  if (len > kCompactThresholdBytes) { len = std::snprintf(tmp, sizeof tmp, "%.3g", v); }  with `inline constexpr int kCompactThresholdBytes = 10;`. A %*.*f rendering is at LEAST `width` bytes, so appendNum(line, 1.0, 12, 2) renders "        1.00" (12 bytes), trips the threshold, and emits "1" — destroying the very column the caller asked for, for a perfectly ordinary value. Latent today: the widest live caller is appendTimestamp's width 7 (term_sink/motion_result/triage/session_info all use 4-6), so nothing currently crosses it. The guard the banner describes is "a rendering longer than kCompactThresholdBytes is pathological", which is only true while width stays well under 10.
```

</details>

### A7. `include/shulib/diag/motion_result.hpp:85`

**MotionResult::outcome defaults to the SUCCESS value and MotionOutcome has no unknown/unset enumerator, so an unpopulated result line reports a settled motion.**

> **DEFECTS1 → FIX.** MotionOutcome::Unset = 5, append-only and value-pinned, and it is the new default.

<details>
<summary>Evidence (3 lines)</summary>

```text
MotionOutcome outcome = MotionOutcome::Settled;

emitResultLine then renders: line.appendLiteral(r.outcome == MotionOutcome::Settled ? " ✓" : " ✗"); i.e. a MotionResult whose producer forgot this field prints the checkmark and "SETTLED". This is the opposite polarity to the same struct's other truth guard — hasPathData defaults false precisely so over/drift render "n/a" rather than a fabricated 0.00 (banner: "the line NEVER fabricates a 0.00 it has no data behind"). The outcome field has no such pessimistic default and the enum offers no value to give it.
```

</details>

### A8. `include/shulib/diag/tick_attribution.hpp:79`

**PhaseScope's constructor is public, so the tick-open precondition that phase() enforces is trivially bypassable.**

> **DEFECTS1 → FIX.** Passkey idiom — a private ctor plus friend does NOT work, because std::optional does the in-place constructing and cannot be a friend. The scheduler was the bypass's only user and now goes through a checked phaseInPlace().

<details>
<summary>Evidence (1 lines)</summary>

```text
phase() guards the invariant: `SHULIB_PRECONDITION(tickOpen_, "TickAttribution::phase: no tick open"); return PhaseScope{*this, p};`. But PhaseScope declares `public:` and then `PhaseScope(TickAttribution& att, TickPhase phase) noexcept : att_{att}, phase_{phase}, start_{att.clock_.now()} {}` — a caller can write `TickAttribution::PhaseScope s{att, TickPhase::Motion};` with no tick open, and its destructor writes into att_.current_[idx] regardless of tickOpen_. Making the constructor private with `friend` (or private plus a factory) would make the checked path the only path; as written the check is advisory.
```

</details>

### A9. `include/shulib/hal/clock.hpp:30`

**IClock declares public copy/move construction and assignment on a polymorphic base, so slicing an implementation compiles silently.**

> **DEFECTS1 → REJECT.** Both asserted consequences are false, by probe: `IClock c = someProsClock;` does not compile (abstract type), and the assignment that does discards nothing (sizeof(IClock) is 8 — the vptr; a.now() and b.now() are unchanged). The header already carries the ruling with its reasons.

<details>
<summary>Evidence (10 lines)</summary>

```text
class IClock {
public:
    virtual ~IClock() = default;
    IClock() = default;
    IClock(const IClock&) = default;
    IClock(IClock&&) = default;
    IClock& operator=(const IClock&) = default;
    IClock& operator=(IClock&&) = default;

`IClock c = someProsClock;` and `base1 = base2` both compile and both discard the derived state. Protected (or deleted) copy/move is the usual guard. Reported once and low-severity because it is a HOUSE-WIDE pattern, not a one-off — IOptical, ICorrector and IKinematics all declare the same public set — so if it is deliberate it should be, and if it is not, it is six headers wide before it is one.
```

</details>

### A10. `include/shulib/hal/mechanism.hpp:132`

**IMechanism's defaulted copy/move duplicate the claim token and the claimant registration, so a copied mechanism arrives already claimed and pointing at an operation bound to the original. REPORTED, NOT FIXED.**

> **DEFECTS1 → FIX.** copy/move deleted on IMechanism — the seam matching a rule mechanism_op.hpp had already written down for the operations. Nothing in the tree copied one.

<details>
<summary>Evidence (13 lines)</summary>

```text
IMechanism holds the claim as VALUE state (`bool claimed_ = false; ICancellable* claimant_ = nullptr;`, mechanism.hpp:203-204) and re-defaults every copy/move member (mechanism.hpp:131-135). manipulation/mechanism_op.hpp:295-302 states the opposite rule for the other half of the same mechanism: "Non-copyable/non-movable (F2): the claim is a resource and the mechanism's registered claimant points at THIS object — a copy would double-release the claim and a move would leave the registration dangling" — and deletes copy/move on RunUntilConfirmed/ActuateAndConfirm. The mechanism side never got the same treatment. Compiled and ran against the real headers:

  hal::MotorMechanism orig{span, BrakeMode::Hold, "lift"};
  Op op; orig.tryClaim(op);
  hal::MotorMechanism copy = orig;   // implicit copy ctor: COMPILES
  copy.releaseClaim();

output:
  orig claimed=1 claimant=0x7fff53c9aa70 ok=1
  copy claimed=1 claimant=0x7fff53c9aa70 (points at op registered with orig)
  after copy.releaseClaim(): orig.claimed=1 copy.claimed=0

Consequences, both in the F2 guard's path: (a) a legitimate operation's tryClaim(copy) fails for no reason, because `copy` was born claimed by an operation that has never heard of it; (b) F2's end-of-run park guard walking a span<IMechanism*> that contains `copy` reaches claimant() and cancels an operation that is driving `orig`, while `orig`'s own claim is never released — which is exactly the "unreleased claim makes the END ACTION's own operation throw at start()" failure the banner at mechanism.hpp:53-68 says the claimant hook exists to close.
```

</details>

### A11. `include/shulib/hal/pros/block_sink.hpp:100`

**~ProsBlockSink discards std::fclose's return value, so a final flush failure on a full or dying card is silent — the exact invisible-drop failure this class's own banner says the blackbox exists to avoid.**

> **DEFECTS1 → FIX.** Documented rather than changed: flush() is bool for exactly this reason, so the honest close names the unreportable failure and points at the member that reports it.

<details>
<summary>Evidence (9 lines)</summary>

```text
Lines 98-102:

    ~ProsBlockSink() override {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

fclose flushes before closing and returns EOF if that write fails, which is precisely the card-full / card-yanked / dying-card case. Every other path in this class is bool-valued for that reason — write() is even [[nodiscard]] — and hal/block_sink.hpp justifies it: "a void write() would make it invisible. A caller that ignores the result cannot notice a truncated file." The last buffered block is the one most likely to be lost and is the only one whose loss nothing here can report; there is no counter, no isOpen() transition, and no return channel from a destructor. Noting it as an observation, not a proposed signature change: fixing it means deciding where a destructor-time failure should surface.
```

</details>

### A12. `include/shulib/hal/pros/char_sink.hpp:45`

**ProsCharSink's constructor accepts a std::FILE* with no null check, unlike every other pointer-taking constructor in the tree; a null (e.g. a failed fopen/tmpfile passed by a test) is undefined behaviour on the first write() rather than a loud precondition.**

> **DEFECTS1 → FIX.** Null check added, in the one class whose banner advertises injection so a test can pass tmpfile().

<details>
<summary>Evidence (1 lines)</summary>

```text
`explicit ProsCharSink(std::FILE* out = stdout) : out_{out} {}` — no SHULIB_PRECONDITION, and the header does not include "shulib/core/check.hpp" at all. write() then does `std::fwrite(text.data(), 1, text.size(), out_); std::fflush(out_);` on it. Compare the house rule applied everywhere else: MotorMechanism checks every IMotor* (mechanism.hpp:193-197), PneumaticMechanism checks every IDigitalOut*, MechanismDeps::validate() checks all three pointers, RunGuardConfig::validate() checks every IMechanism*. The banner advertises the FILE* as injectable precisely so a host test can hand it a tmpfile() — the one call site most likely to hand over a null on failure.
```

</details>

### A13. `include/shulib/hal/pros/controller.hpp:116`

**axis()/pressed()/isConnected() are const but mutate a `mutable` device object, so const-ness here carries no thread-safety guarantee and the header never says so.**

> **DEFECTS1 → REJECT.** pros::v5::Controller's only data member is the id fixed at construction; the const readers mutate nothing. `mutable` is there because PROS declared its getters non-const.

<details>
<summary>Evidence (1 lines)</summary>

```text
mutable ::pros::v5::Controller controller_;  // PROS's readers are non-const  — with `[[nodiscard]] double axis(ControllerAxis axis) const override { return controllerAxisToCanonical(static_cast<double>(controller_.get_analog(toProsAxis(axis)))); }`. Two tasks calling axis() concurrently on one adapter both mutate controller_, which is a data race despite both calls being const. The header documents the seam's threading nowhere, while diag/ headers state "Single-task by contract" explicitly — and this is the one seam a second driver's code is most likely to be read from off the motion task.
```

</details>

### A14. `include/shulib/hal/pros/digital_out.hpp:73`

**The "initialState cannot be forgotten" safety property — the header's central claim — is defeatable: a caller who writes the EXPANDER form and omits initialState silently gets the brain-ADI form with the line driven HIGH at boot.**

> **DEFECTS1 → FIX.** Deleted poison overload. Sharpened by verification: it compiled only through PARENTHESES — brace init already rejected it as a narrowing int -> bool.

<details>
<summary>Evidence (13 lines)</summary>

```text
Header lines 15-17 claim: "A safety step a caller can forget is a safety step that WILL be forgotten (the legacy escapeJSONString lesson); this one cannot be skipped, only stated."

But the two constructors are:
    ProsDigitalOut(std::uint8_t adiPort, bool initialState)                       // line 73
    ProsDigitalOut(std::uint8_t smartPort, std::uint8_t adiPort, bool initialState) // line 79

So `ProsDigitalOut oops(1, 2);` — a caller meaning {smartPort 1, adiPort 2} who forgot the boot state — is NOT a compile error. It selects the 2-arg constructor with adiPort=1 and initialState=(bool)2==true. Verified with the project's own flags from test/CMakeLists.txt:

    $ g++ -std=gnu++20 -Iinclude -Wall -Wextra -Wpedantic -Wshadow -Wconversion \
          -Wsign-conversion -Wdouble-promotion -fsyntax-only pdo.cpp
    COMPILES CLEAN -> 2-arg ctor selected, initialState = (bool)2 = true

Zero warnings. Because construction is a physical action (line 64: `line_{adiPort, initialState}`), that mistake fires the solenoid HIGH at boot, on the wrong port — exactly the failure mode the required argument exists to prevent. NOT FIXED.
```

</details>

### A15. `include/shulib/hal/pros/distance.hpp:98`

**confidence() issues its own second get_distance() call, so the documented `confidence() then distance()` idiom mixes two different device samples, and one logical read cycle can increment faultedReads_ twice.**

> **DEFECTS1 → ARGUE.** An atomic confidence()+distance() pair needs either a method on F4-LOCKED IDistance or an adapter-side sample window, which invents an unmeasured constant. See the ARGUE section of DEFECTS1-COMPLETED.md.

<details>
<summary>Evidence (11 lines)</summary>

```text
distance() reads the device at line 85 and confidence() reads it again at line 98:

    [[nodiscard]] units::Length distance() const override {
        const std::int32_t mm = sensor_.get_distance();     // line 85
        if (mm == PROS_ERR) { faultedReads_ += 1; return lastDistance_; }

    [[nodiscard]] double confidence() const override {
        const std::int32_t mm = sensor_.get_distance();     // line 98
        if (mm == PROS_ERR) { faultedReads_ += 1; return 0.0; }

IDistance's own header prescribes the paired usage: "Callers threshold confidence() for 'object present' — never trust distance() when confidence() is ~0" (include/shulib/hal/distance.hpp). That pairing is not atomic here. Two consequences: (a) an object crossing the 9999-sentinel or the 200 mm close-range boundary between the two calls yields a confidence describing one sample and a distance from another; (b) with the sensor unplugged, a caller doing the prescribed pair increments faultedReads_ by 2, so the accessor documented as "how many DEVICE-FAILURE reads were screened" cannot be read as a count of failed ticks.
```

</details>

### A16. `include/shulib/hal/pros/gps.hpp:126`

**Every IGps reader takes its own fresh device sample, so the contract-mandated hasFix()-then-pose() sequence spans TWO samples. A caller can be told "fix" and then handed a stale pose — which is exactly the straddled-read failure the class's single-atomic-read design exists to prevent.**

> **DEFECTS1 → ARGUE.** Same shape as A15 on F4-LOCKED IGps, and six device reads per corrector tick. Written up, not applied.

<details>
<summary>Evidence (14 lines)</summary>

```text
pose(), rmsError() and hasFix() each begin with `refresh();`, and refresh() unconditionally re-reads the device and rewrites hasFix_/lastPose_:

    [[nodiscard]] math::Pose2d pose() const override { refresh(); return lastPose_; }
    [[nodiscard]] bool hasFix() const override { refresh(); return hasFix_; }

    void refresh() const {
        ...
        const auto status = sensor_.get_position_and_orientation();
        ...
        if (!std::isfinite(status.x) || ...) { faultedReads_ += 1; hasFix_ = false; return; }   // lastPose_ untouched
        lastPose_ = gpsToRobotPose(...); hasFix_ = true;
    }

Sequence: hasFix() reads sample N -> true. pose() then reads sample N+1; if N+1 is off-strip, refresh() returns early leaving lastPose_ at sample N's value, and pose() hands back a pose the class itself has just decided is fix-less. The header banner claims the opposite guarantee: "ONE atomic status read carrying x/y ... so position and heading in a single IGps::pose() come from the SAME device sample rather than three reads that could straddle an update." That holds WITHIN one call and not across the three-accessor API, while hal/gps.hpp:29 requires the caller to use two of them: "Callers MUST check hasFix() before trusting pose()."
```

</details>

### A17. `include/shulib/hal/pros/imu.hpp:132`

**ProsImu::yawRate() in the DEFAULT DifferentiateRotation mode is a CONSUMING read that rebases the differentiation sample, but IImu::yawRate() is a plain const accessor called 2-4x per tick by independent consumers — so the value each caller gets depends on who read it first, and attaching a telemetry sink changes the yaw rate the Localizer sees.**

> **DEFECTS1 → DEFER.** R4. The consuming-read rebase needs measured call patterns and a real loop rate before a design can be chosen.

<details>
<summary>Evidence (7 lines)</summary>

```text
differentiatedRate() (imu.hpp:167-191) does `lastRotationDeg_ = degCw; lastSampleTime_ = now;` on every call with dt > 0, so each caller differentiates over the time since the PREVIOUS CALLER, not over the tick.

Callers per tick on a real robot: localization/localizer.hpp:292 `const double rawOmega = imu_.yawRate().value();`, localization/gps_corrector.hpp:259, localization/apriltag_corrector.hpp:306, and every motion's record builder — motion/drive_brake.hpp:210, motion/move_to_pose.hpp:368, motion/turn_to.hpp:233, motion/motion_scheduler.hpp:1142, chassis/chassis.hpp:584 (all `r.imuYawRate = ctx.imu().yawRate();`).

The record-builder calls sit inside hal::emitRecord's lazy lambda, which runs ONLY when a sink returns wantsRecord() == true (hal/telemetry_sink.hpp:122-125). So a 10 ms tick where the record builder reads at t=9 ms leaves the Localizer's next read a 1 ms window over a QUANTIZED angle — a ~10x noise amplification on the load-bearing <1 deg heading path — and that amplification appears and disappears with the telemetry sink. Observability perturbs the measurement.

This is the same shape the header itself forbids two lines up: 'PROS's get_digital_new_press() CONSUMES the event on read, so with two consumers one silently loses (HA-104)'. The value is not biased, only noisier, and the dt <= 0 guard (imu.hpp:182-184) saves the same-millisecond case — but populating a DebugRecord (N smart-port motor voltage/current reads) is exactly the work that straddles a millisecond.
```

</details>

### A18. `include/shulib/hal/pros/imu.hpp:172`

**faultedReads() counts screened READS, not failed ticks, and one dead sensor produces up to four counts per tick — so any threshold set on it is off by an unknown call-count factor.**

> **DEFECTS1 → REJECT.** The doc says "reads" and the code counts reads. HealthMonitor raises ImuLost from isReady(), never from this counter, and NOTHING in the library reads faultedReads() at all — so the asserted 4x-early IMU_LOST cannot happen.

<details>
<summary>Evidence (1 lines)</summary>

```text
faultedReads_ is incremented independently in heading() (imu.hpp:122), differentiatedRate() (171), gyroRate() (197), pitch() (152) and roll() (162). A single unplugged IMU on one 10 ms tick therefore adds 3-4 (heading + yawRate + pitch + roll), and yawRate() itself may be called several times per tick (see the first finding). The header banner says 'faultedReads() exposes the screen count; the loop's HealthMonitor owns raising IMU_LOST' — a consumer reading that as 'consecutive bad ticks' will trip IMU_LOST roughly 4x early, and the multiplier moves with the telemetry sink.
```

</details>

### A19. `include/shulib/hal/pros/motor.hpp:231`

**temperature()'s hold-last-good cache seeds at 20.0 C while the other three seed at 0, so a port that has never answered reports a plausible room-temperature motor forever — the same 'substitute a plausible value' failure mode the header's own T7 note rejects for encoders.**

> **DEFECTS1 → REJECT.** 0 degC reads exactly as healthy as 20 degC to the only consumer (a >= 55 degC threshold), so the symmetry fix changes no observable. A port dead at construction cannot produce a ProsMotor — the ctor read-back throws.

<details>
<summary>Evidence (17 lines)</summary>

```text
Seeds (lines 228-232):
    mutable units::AngleDim lastPosition_{0.0};
    mutable units::AngularVelocity lastVelocity_{0.0};
    mutable units::Current lastCurrent_{0.0};
    mutable double lastTemperature_ = 20.0;

Against the header's stated T7 rule (lines 31-40):
    // hold the last good value, never propagate ... never substitute zero (a
    // zeroed encoder reads as "the robot stopped" ... and a zero would make it
    // plausible instead of visible)

The reasoning cuts the same way here: the thermal monitor exists to notice a V5
motor throttling near 55 C (hal/motor.hpp:65-68), and 20 C is precisely the
reading that looks healthy. If the first read ever faults, temperature() reports
20 C with nothing in-band to say so. faultedReads() makes it *recoverable*, but
only for a caller who thinks to check. Defensible as a deliberate prior — I am
flagging the asymmetry, not asserting the fix.
```

</details>

### A20. `include/shulib/hal/vision.hpp:35`

**TagObservation and ObjectObservation are the only sensor value structs in the tree with no default member initializers, so id/classId/confidence are indeterminate on default construction.**

> **DEFECTS1 → FIX.** Default member initializers added; aggregate initialisation is unaffected.

<details>
<summary>Evidence (12 lines)</summary>

```text
vision.hpp:32-43 and 46-57 (declarations unchanged by this pass):
    struct TagObservation { int id; math::Pose2d poseInRobot; double confidence; };
    struct ObjectObservation { int classId; math::Angle bearing; double confidence; };

Every comparable struct in the tree does initialize:
    correction.hpp:50   bool valid = false;
    correction.hpp:52   double confidence = 0.0;
    vision_conversion.hpp:99  double fx = 0.0;   (CameraIntrinsics, TagPnpResult likewise)

`TagObservation t; t.poseInRobot = …;` leaves `id` and `confidence` indeterminate — and the corrector's screen cannot save it, because the screen itself is the read:
    apriltag_corrector.hpp:311   !std::isfinite(obs.confidence)
Evaluating isfinite() on an indeterminate double is already UB, so the guard runs after the damage. An indeterminate `id` that happens to hit a real map entry yields a confident fix against the wrong tag.
```

</details>

### A21. `include/shulib/kinematics/desaturate.hpp:29`

**A NaN wheel speed is invisible to WheelSpeeds::maxMagnitude() and passes straight through desaturateUniform unchanged, so the header's "last-line guarantee that no wheel is ever asked for more than it can give" does not hold for non-finite input.**

> **DEFECTS1 → FIX.** The CODE is right and the BANNER was over-claimed. desaturate is not the last line — recoverWheelVoltage at the motor edge is. Enforcing finiteness here would narrow LOCKED row F5 and turn an A3 pathology into an aborted motion.

<details>
<summary>Evidence (22 lines)</summary>

```text
Claim, desaturate.hpp banner lines 12-14:
    // This is deliberately separate from strafeAuthority() clamping (S13 #5): that
    // shapes the command upstream; this is the last-line guarantee that no wheel is
    // ever asked for more than it can give.

Mechanism, wheel_speeds.hpp:67 — std::max(m, NaN) returns m, so NaN never wins:
    for (int i = 0; i < n_; ++i) {
        m = std::max(m, std::abs(v_[static_cast<std::size_t>(i)].value()));
    }

Measured (compiled against the real headers, g++ -std=gnu++20 -Iinclude):
    maxMagnitude = 10.000000            <- set is {NaN, 10, 10, 10}
    out[0] = nan  (isnan=1)             <- returned unchanged, 'within budget'
    out[1] = 10.000000
    w2 scaled: o2[0]=nan o2[1]=20.000000  <- NaN survives the scaling branch too

Both branches leak it: the early return treats the set as within budget because
peak reads 10, and in the scaling branch NaN * scale = NaN. Note desaturateUniform
itself has no finiteness precondition even though its sibling checks do
(SHULIB_PRECONDITION on maxWheelSpeed only). ProsMotor::setVoltage does reject
non-finite at the motor edge, so this fails eventually — but as a crash at the
edge, not as the desaturation contract the banner advertises.
```

</details>

### A22. `include/shulib/localization/apriltag_corrector.hpp:225`

**poll() truncates an over-full frame by ARRIVAL ORDER, so the best-sigma tag can be discarded before the "pick the single best tag" selection ever sees it — and unlike every other rejection in this class, the discard has no counter and no GateReason.**

> **DEFECTS1 → FIX.** The drop is counted and exposed. The selection is deliberately still not sigma-ranked: ranking in poll() would duplicate the estimator's own model, which is the shared-model trap.

<details>
<summary>Evidence (7 lines)</summary>

```text
apriltag_corrector.hpp:225 —
    const std::size_t n = std::min(seen.size(), kMaxTagsPerFrame);
    for (std::size_t k = 0; k < n; ++k) { frame_[k] = seen[k]; }

This keeps the first 8 observations in ITagSource::tags()' vector order, which is the detector's order and carries no quality meaning. Step (7)'s selection loop then runs `for (k = 0; k < frameCount_; ++k)` over that prefix only, so if the source returns 9+ tags and the smallest-sigma one sits at index 8 or beyond, the class silently anchors to a worse tag than it had available.

That undercuts the header's stated design — "This class picks the tag with the SMALLEST estimated sigma and uses only that one" — which holds only within an arbitrary prefix. The contrast with the rest of the class is the sharp part: unmapped, out-of-range, low-confidence, high-yaw-rate and innovation rejects each get their own counter AND their own GateReason, precisely so silence is diagnosable. A dropped tag here increments nothing and is unobservable from the blackbox. The class doc reasons that >8 tags means a tag-rich field or a hallucinating detector "either way the best-sigma pick only needs a few" — true only if the kept few are chosen by sigma, which they are not.
```

</details>

### A23. `include/shulib/localization/ekf_fusion.hpp:494`

**state(i) and covariance(i, j) are public, noexcept, and index the fixed arrays with no bounds check, so an out-of-range index is silent UB — against the house precondition discipline used for every other public indexing accessor.**

> **DEFECTS1 → FIX.** Bounds-checked, and therefore no longer noexcept.

<details>
<summary>Evidence (13 lines)</summary>

```text
ekf_fusion.hpp:490-494:
    [[nodiscard]] double covariance(std::size_t i, std::size_t j) const noexcept {
        return at(P_, i, j);            // -> m[i * kN + j], unchecked
    }
    [[nodiscard]] double state(std::size_t i) const noexcept { return x_[i]; }

The documented contract is only a naming convention, not a guard — "One state entry, indexed by the kPx...kVy constants" — and nothing stops state(9).

The house pattern one directory over does check, on the same shape of accessor:
  wheel_speeds.hpp:53-55  /// The i-th wheel speed. Precondition: 0 <= i < size().
                          SHULIB_PRECONDITION(i >= 0 && i < n_, "WheelSpeeds: index out of range");

Mitigating: the file header says these are "observability (telemetry and tests; none of this is on the control path)". Still a public surface, still noexcept, still UB.
```

</details>

### A24. `include/shulib/manipulation/mechanism_op.hpp:379`

**cancel() on an ALREADY-FINISHED RunUntilConfirmed commands the mechanism unconditionally, with no check that this operation still holds the claim — so a stale operation object can safe a mechanism a DIFFERENT live operation now owns, defeating the one-operation-per-mechanism guarantee the claim token exists to provide.**

> **DEFECTS1 → FIX.** Guarded on `holdsClaim_ || !claimed()`, NOT on holdsClaim_ alone — an existing test asserts that a finished cancel() re-safes a mechanism nobody owns, and it is right. The disjunction closes the hole without overturning it.

<details>
<summary>Evidence (1 lines)</summary>

```text
RunUntilConfirmed::cancel(): `if (!started_) { return; } mech_->applySafeState();  // always — "make it safe NOW", idempotent  if (!finished_) { releaseClaim(); ... }`. finish() already ran releaseClaim() (holdsClaim_ = false), so after an exit this object holds no claim, yet applySafeState() runs anyway and IMechanism::applySafeState() has no claim check (mechanism.hpp:130, "callable at any time"). Sequence: op1 Succeeds and releases; op2.start() takes the claim and is running at 12 V; a later op1.cancel() (a retained operation object, a routine that cancels defensively) drops op2's mechanism to safe brake mode + 0 V, silently, with no fault and no Warn. op2 re-commands its voltage on its next tick but NOT the brake mode — which is exactly the half-safe `brake=Hold, V=9.0` state run_guard.hpp's banner (T6) names as the reason applySafeState() alone is never trusted. The banner's cancel contract says the safe state is 'STILL applied' on a finished operation for idempotence; that clause was written for the single-operation case and no code path checks holdsClaim_.
```

</details>

### A25. `include/shulib/manipulation/mechanism_op.hpp:605`

**Same unclaimed-command hole in ActuateAndConfirm::cancel(), and worse here: it drives a discrete actuator to the declared safe value, which UN-DOES an actuation a different, currently-claiming operation just performed.**

> **DEFECTS1 → FIX.** Same guard on the discrete-actuator side, where a stale cancel un-did a live operation's actuation.

<details>
<summary>Evidence (1 lines)</summary>

```text
ActuateAndConfirm::cancel(): `if (!started_) { return; } mech_->applySafeState();  // always — idempotent  if (!finished_) { releaseClaim(); ... }`. finish() (line 623 region) deliberately does NOT applySafeState and DOES releaseClaim(), so a Succeeded ActuateAndConfirm holds no claim. A cancel() on that finished object still calls PneumaticMechanism::applySafeState() → set(safe_) (mechanism.hpp:295). If the declared safe value is 'open' and a second operation has since re-closed the clamp, the stale cancel() opens it — the exact 'FLING ITS GOAL' failure the file banner cites as the reason success must not apply the safe state, reintroduced through the cancel path with no claim check.
```

</details>

### A26. `include/shulib/math/twist2d.hpp:8`

**Twist2d is used for both FIELD-frame and BODY-frame velocities with nothing in the type or the signature saying which, while the command-side type got an explicit Frame parameter for exactly that reason — the frame-confusion guard is one-sided.**

> **DEFECTS1 → ARGUE.** A frame discriminator on Twist2d; every type-level cure is a wide breaking change. Written up.

<details>
<summary>Evidence (8 lines)</summary>

```text
The same type, two frames, no discriminator:
  i_pose_source.hpp:27  /// Field-frame velocity estimate (the derivative of the published pose).
                        [[nodiscard]] virtual math::Twist2d twist() const noexcept = 0;
  kinematics.hpp:70     /// Forward kinematics: per-wheel surface speeds -> BODY-frame twist (for odometry).
                        [[nodiscard]] virtual math::Twist2d forward(const WheelSpeeds& wheels) const = 0;
  drive_plant.hpp:259   [[nodiscard]] math::Twist2d trueBodyTwist() const noexcept { return bodyTwist_; }

Compare the command path, where the library made the opposite choice on purpose (frame.hpp:29-32): "The caller must SAY which frame a command is in; the parameter has no default anywhere, so silently assuming the wrong frame — the classic bug class this rebuild exists to prevent — is a compile error." Handing IPoseSource::twist() to something expecting IKinematics::forward()'s output compiles silently and is wrong by a rotation of theta.
```

</details>

### A27. `include/shulib/motion/motion_scheduler.hpp:434`

**MotionStatsSink::beginMotion() clears every aggregate except target_ and startPose_, so the public targetPose() accessor returns the PREVIOUS motion's target between motions. Only the scheduler's own hasData() guard hides it.**

> **DEFECTS1 → FIX.** beginMotion() clears the poses with the scalars. Mutation-proven.

<details>
<summary>Evidence (16 lines)</summary>

```text
void beginMotion() noexcept {          // line 434
        sawRunning_ = false;
        maxProj_ = 0.0;
        maxDist_ = 0.0;
        lastAbsHeadErr_ = 0.0;
    }                                       // target_ and startPose_ are NOT reset

    math::Pose2d startPose_{};   // line 507
    math::Pose2d target_{};      // line 508

MotionStatsSink is a PUBLIC class with a public `targetPose()`. Its own class banner promises the honest-scope story is carried by hasData(), and the scheduler does guard correctly in finalize():
    .targetPose = hasData ? stats.targetPose() : math::Pose2d{},

FAILURE SCENARIO: motion 1 runs to a target of (48, 24); the scheduler arms motion 2, which calls beginMotion(). Before motion 2's first live tick, any direct reader of the sink calls targetPose() without checking hasData() and receives (48, 24) — motion 1's target, presented as motion 2's. It is not a default Pose2d and not detectable as stale from the returned value. overshoot() and drift() do not have this problem because maxProj_/maxDist_/lastAbsHeadErr_ ARE reset, so they read 0 regardless.

This one bit me while writing: my first draft of the targetPose() comment claimed "a default Pose2d otherwise", which is false. I corrected the comment to state the real hazard rather than ship a confident wrong sentence, and am reporting the underlying shape.
```

</details>

### A28. `include/shulib/motion/motion_scheduler.hpp:622`

**`~MotionScheduler() = default` reopens the exact hole F2 closed for the blocking waits: destroying a scheduler with a motion still armed leaves the drive motors at their last commanded voltage, silently.**

> **DEFECTS1 → FIX.** The destructor forces the drive safe — but does NOT call cancel(): the motion may already be destroyed (they are declared after the scheduler and die first), which the test caught as a SIGABRT. Mutation-proven; measured 8.4 V held before the fix.

<details>
<summary>Evidence (11 lines)</summary>

```text
~MotionScheduler() = default;   // line 622, no cancel, no safe state

This contradicts the emphasis the header itself places on the cancel path:
  * banner, "cancel(): the defined safe state": "commanded SYNCHRONOUSLY inside the call — no further tick is required for the drivetrain to be safe (a cancel that depends on someone continuing to tick is a cancel that can leave motors energized)";
  * banner, "Unwind safety of the blocking waits (F2)": "A throw through a blocking wait ... used to leave the active motion ARMED and the motors at their last command" — fixed by WaitUnwindGuard, whose destructor calls cancel(), with the stated dress code "belt plus braces ... for the panic path".

WaitUnwindGuard covers throws through waitUntilSettled()/waitUntil() only. The scheduler's own destructor is the remaining path with identical consequences.

FAILURE SCENARIO: a routine calls `sched.async(move);` and returns without waiting (or an exception is thrown from caller code between tick() calls in a hand-rolled non-blocking loop, i.e. outside any blocking wait). The MotionScheduler goes out of scope with `active_ != nullptr`; the defaulted destructor runs, applyCancelSafeState() is never called, and the drive motors hold their last commanded voltage with their last brake mode. Nothing raises, nothing logs, no boundary is recorded, and completedCount() never accounts for the motion. The robot drives into whatever is in front of it.

I documented the behaviour honestly at the special-member run rather than leave it implicit, but did not change it — this chunk's rule is report, do not fix.
```

</details>

### A29. `include/shulib/motion/odo_stall_check.hpp:79`

**There is no gear-ratio knob, so on any externally geared drivetrain `wheelRadius` must silently absorb the ratio — at which point the field's name and its own doc comment are both wrong.**

> **DEFECTS1 → DEFER.** R3/R4. There is no gear-ratio concept anywhere in the library and the A2 sim plant bakes 1:1 in too — larger than this chunk.

<details>
<summary>Evidence (10 lines)</summary>

```text
/// Drive wheel RADIUS (inches) — converts shaft radians to surface travel.
    /// Stand-in geometry (3.25″ wheel, 1:1 gearing — A4: HA-14).
    units::Length wheelRadius{3.25 / 2.0};

and line 134:
    const double spinTravel = meanShaftDelta * cfg_.wheelRadius.value();

where meanShaftDelta comes from IMotor::position(), documented in hal/motor.hpp:53 as "Cumulative output-shaft rotation (NOT wrapped)". Output-shaft radians times wheel radius is surface travel ONLY at 1:1 external gearing. OdoStallCheckConfig has five fields and none of them is a gear ratio, so a robot with, say, a 36:60 external reduction must set wheelRadius = trueRadius × ratio — a value that is not a wheel radius, in a field called wheelRadius, contradicting its own comment.

Grep confirms the library has no gear-ratio concept to borrow (`grep -rn "gearRatio|gear ratio|gearing" include/shulib` returns only the PROS cartridge read-back and prose), so this is a real gap rather than a local omission — but it is the one config field where the missing ratio silently produces a WRONG threshold rather than an unmeasured one. NOT FIXED.
```

</details>

### A30. `include/shulib/motion/odo_stall_check.hpp:118`

**update() dereferences every element of `motors` with no non-null and no non-empty precondition, unlike every sibling fan-out in the tree; an empty span makes the check silently un-trippable.**

> **DEFECTS1 → FIX.** Non-empty and non-null preconditions. The empty case was the dangerous half: it made the stall check report healthy forever.

<details>
<summary>Evidence (14 lines)</summary>

```text
update()'s only guard is a size ceiling:
    SHULIB_PRECONDITION(motors.size() <= static_cast<std::size_t>(kMaxWheels),
                        "OdoStallCheck: too many motors");
yet line 133 does:
    sumAbsShaftDelta += std::abs(motors[i]->position().value() - shaftBase_[i]);

Every sibling that takes the same shape DOES check both. hal/mechanism.hpp:207-210 (MotorMechanism) and :275-278 (PneumaticMechanism):
    SHULIB_PRECONDITION(!motors_.empty(), "MotorMechanism: motors is empty");
    SHULIB_PRECONDITION(m != nullptr, "MotorMechanism: a motor is null");
and chassis/robot_context.hpp:51-54 and localizer.hpp:145-147 do the same for their spans.

Second half of the same gap: with n == 0, line 116 gives meanShaftDelta = 0.0, so spinTravel = 0, so `spinTravel >= minSpinTravel` is false and stalled_ can never become true. A misconfigured empty span produces a check that reports "healthy" forever rather than failing loudly — the failure this library's precondition discipline exists to prevent.

Mitigation, stated honestly: the in-tree path (motion.hpp:166 `ctx.driveMotors()`) is already validated by RobotContext's constructor, so this bites a direct caller of OdoStallCheck, which the published reference invites. NOT FIXED.
```

</details>

### A31. `include/shulib/motion/strafe_to.hpp:34`

**StrafeTo inherits MoveToPose::setTarget(Pose2d) publicly, but the heading component of the pose the caller passes is silently discarded at the first live tick. REPORTED, NOT FIXED.**

> **DEFECTS1 → ARGUE.** Narrowing StrafeTo's inherited setTarget changes a public inherited signature. Written up.

<details>
<summary>Evidence (1 lines)</summary>

```text
strafe_to.hpp:41-42 constructs the base with `PoseMotionOptions{.captureHeadingAtLive = true}`. move_to_pose.hpp:95 then sets `captured_ = !(opts_.captureHeadingAtLive || opts_.capturePoseAtLive);` — false for StrafeTo — and move_to_pose.hpp:129-133 overwrites the heading at the first live tick: `target_ = math::Pose2d{target_.x(), target_.y(), p.heading()};`. Meanwhile move_to_pose.hpp:249 exposes `void setTarget(const math::Pose2d& target)` publicly, taking a full pose and validating only x/y finiteness. So `strafe.setTarget(Pose2d{x, y, someHeading}); strafe.start();` accepts someHeading, stores it, reports it from target() until the first live tick, and then throws it away with no diagnostic. StrafeTo's own constructor is honest about this (it takes only x and y and passes math::Pose2d{x, y, math::Angle{}}); the inherited setter is not. Documented in the class comment I added rather than silently left; a narrower override or a Length-pair setTarget would be the real fix.
```

</details>

### A32. `include/shulib/motion/turn_to.hpp:63`

**cfg_.validate() and the timeout precondition run in the constructor BODY, after the member-initializer list has already built pidH_, exit_ and stall_ from those same unvalidated fields — so a bad MotionConfig is reported by the wrong component's error message.**

> **DEFECTS1 → FIX.** A validatedConfig() helper mirroring validatedClock(), with cfg_ declared first so it runs before the PIDs and watchdog are built.

<details>
<summary>Evidence (10 lines)</summary>

```text
turn_to.hpp:55-60 build from `config` in the init list:
    pidH_{control::PidConfig{.kP = config.heading.kP, .kI = config.heading.kI, ...}, ...}
    exit_{config.headingSettle, timeout > 0.0 ? timeout : config.defaultTimeout, ...}
and only then, at line 63:
    cfg_.validate();
    SHULIB_PRECONDITION(std::isfinite(timeout) && timeout >= 0.0, "TurnTo: timeout must be finite and >= 0");

Both shipped precondition handlers throw (core/check.hpp: "a handler must NOT return"), so this is NOT a safety hole — construction unwinds either way. What is lost is the specific diagnostic. A MotionConfig with defaultTimeout <= 0, combined with the documented `timeout = 0` ("0 selects config.defaultTimeout"), trips watchdog.hpp:29 `"Watchdog: timeout must be > 0"` instead of motion_config.hpp:141 `"MotionConfig: defaultTimeout must be > 0"`. A non-finite heading gain trips pid.hpp:65 instead of `"MotionConfig: heading gains invalid"`. The user misconfigured MotionConfig; the message names a component they never touched.

Worth flagging because this repo demonstrably treats message specificity as load-bearing: MatrixKinematics keeps a precondition it documents as fully SUBSUMED by the next one, "kept for its message, not the load-bearing rejection". The same file already solves the ordering problem for the other half of the inputs — `deps.validatedClock()` exists so "a null pointer trips the precondition rather than being dereferenced" — but there is no `validatedConfig()` counterpart.
```

</details>

---

## I — two parts of the library disagree with each other

*(items `I1`–`I22`)*

### I1. `include/shulib/control/exit_group.hpp:31`

**ExitReason::Cancelled's documentation is TRUNCATED mid-clause in the generated reference: the second ///< continuation line is parsed as a doc run for the next enumerator (there is none) and discarded.**

> **DEFECTS1 → REJECT.** Already fixed at DOCS2 — the published page carries the full clause including "never returned by ExitGroup::check()".

<details>
<summary>Evidence (10 lines)</summary>

```text
Source:
    Cancelled,  ///< stopped from outside via IMotion::cancel() (chunk C2; never
                ///< returned by ExitGroup::check() — see header note)

Running the tool's own parser over the header prints:
    (enumerator) Cancelled: ['stopped from outside via IMotion::cancel() (chunk C2; never']

The published page therefore ends the sentence at 'never', dropping exactly the clause that says check() can never return it. Cause is api_doc_tool.py's _parse_enum_body: a ///< documents the LAST enumerator on its own line, and a bare ///< line becomes `pending` for the NEXT enumerator, which here is the closing brace. Fix is to move both lines to a /// run ABOVE `Cancelled,`. NOT changed, per this chunk's do-not-rewrite-existing-/// rule.

The identical shape exists outside my assignment at include/shulib/motion/motion.hpp:150-151 (MotionState::Cancelled) and should be swept with it.
```

</details>

### I2. `include/shulib/control/feedforward.hpp:90`

**CompensatedVoltage::brownoutLimited has no default member initializer while its sibling self-initializes, so a default-constructed CompensatedVoltage holds an indeterminate bool.**

> **DEFECTS1 → FIX.** brownoutLimited defaults to false.

<details>
<summary>Evidence (6 lines)</summary>

```text
struct CompensatedVoltage {
    units::Voltage voltage;   // Quantity has `double v_ = 0.0;` -> zero-initialized
    bool brownoutLimited;     // no initializer -> indeterminate under `CompensatedVoltage c;`
};

Every comparable struct in the tree defaults every field (PidConfig, GpsCorrectorConfig, GateAudit, CorrectionProposal, MotionResult). Latent today because the single producer aggregate-initializes both members, but `CompensatedVoltage cv;` in any future caller is a UB read on a safety flag.
```

</details>

### I3. `include/shulib/control/trapezoid_profile.hpp:51`

**TrapezoidProfile screens `distance` for finiteness but screens the two constraints only for `> 0`, so an infinite maxAcceleration is accepted and leaks a non-finite acceleration out of sample() — in a library whose F4 pillar is finiteness.**

> **DEFECTS1 → FIX.** Both constraints screened for finiteness. Mutation-proven.

<details>
<summary>Evidence (16 lines)</summary>

```text
The three preconditions, lines 51-53:

    SHULIB_PRECONDITION(c.maxVelocity > 0.0, "TrapezoidProfile: maxVelocity must be > 0");
    SHULIB_PRECONDITION(c.maxAcceleration > 0.0, "TrapezoidProfile: maxAcceleration must be > 0");
    SHULIB_PRECONDITION(std::isfinite(distance), "TrapezoidProfile: distance must be finite");

NaN is caught incidentally (NaN > 0.0 is false), but +inf passes `> 0.0`. `aMax_ = c.maxAcceleration` is then stored raw and returned by sample() for t <= 0:

    return scaled(0.0, 0.0, (duration_ > 0.0) ? aMax_ : 0.0);  // about to accelerate

Run against the real header (g++ -std=gnu++20 -Iinclude):

    ctor ACCEPTED maxAcceleration=inf; duration=5.000000
    sample(0.0): pos=0.000000 vel=0.000000 accel=inf  isfinite(accel)=0

A Feedforward consumer receives inf as its acceleration target on the first tick of the move. The header banner asserts "sample(t) clamps t to [0, duration]" but makes no finiteness claim about the constraints, and the class doc-comments now say the constraints must be `> 0` because that is all the code actually enforces.
```

</details>

### I4. `include/shulib/control/watchdog.hpp:28`

**Watchdog takes and returns bare doubles for seconds, bypassing units::Time in the one place a duration crosses an API — the exact "milliseconds into a seconds-based gain" bug class the units system exists to make impossible, and callers strip the typed value to feed it.**

> **DEFECTS1 → REJECT.** D2 ruled the typed/untyped boundary explicitly — typed at the facade, seconds-double inside the motion stack — and Watchdog sits inside the motion stack with three siblings shaped the same way. Not an outlier.

<details>
<summary>Evidence (13 lines)</summary>

```text
watchdog.hpp:28   Watchdog(double timeout, hal::IClock& clock)
watchdog.hpp:39   [[nodiscard]] double elapsed() const
...while the clock it reads is typed: clock.hpp:27  [[nodiscard]] virtual units::Time now() const = 0;

quantity.hpp:16-17 states the purpose: "This kills two bug classes at compile time: 'degrees into cos/sin' and 'milliseconds into a seconds-based gain'."

And the callers pay for it — mechanism_op.hpp:251 declares its budget typed and then unwraps it at the Watchdog boundary:
  /// Watchdog budget (finite, > 0). Typed time, the D2 discipline.
  units::Time timeout;
  ...
  mechanism_op.hpp:287  : watchdog_{validateConfig(config, deps).value(), deps.validatedClock()},
exit_group.hpp:46 propagates the untyped form outward: ExitGroup(const SettleConfig& settle, double timeout, hal::IClock& clock).
StallDetector, by contrast, takes units::Time persistence. Watchdog is the outlier.
```

</details>

### I5. `include/shulib/hal/gps.hpp:43`

**The IGps seam spells out pose()'s behaviour when hasFix() is false (finite, non-throwing, unspecified value) but says nothing about rmsError() or hasFix(), so implementers invent a contract and the consumer adds a backstop for a case the seam never forbade. REPORTED, NOT FIXED.**

> **DEFECTS1 → FIX.** rmsError() now states the finiteness, non-negativity and non-throwing rules pose() always had.

<details>
<summary>Evidence (1 lines)</summary>

```text
gps.hpp:36-39 for pose(): "When hasFix()==false the value is UNSPECIFIED but MUST be finite (no NaN/Inf) and MUST NOT throw". gps.hpp:42-43 for rmsError(): "RMS position error (canonical Length) — drives the corrector's R; large when off-strip." — no finiteness rule, no non-negativity rule, no no-fix rule, no non-throwing rule. Two places have already had to fill the gap independently: hal/pros/gps.hpp:99-104 invents one per-implementation ("Holds last-good while fix-less"), and localization/gps_corrector.hpp:220-224 adds its own guard for a shape the seam never ruled out — `const double rms = gps_.rmsError().value(); if (!std::isfinite(zx) || !std::isfinite(zy) || !std::isfinite(rms) || rms < 0.0) { ... RejectedNoFix; }` — under a comment calling it an "F4 finiteness backstop". A negative rmsError would flow into `sigma_meas = max(rmsTrustFactor * rmsError(), minPositionStdDev)` (gps_corrector.hpp:65) if that backstop were ever removed or a second consumer appeared without it. The narrow true statement: the guarantee is real for pose() and only conventional for rmsError().
```

</details>

### I6. `include/shulib/hal/imu.hpp:45`

**One /// comment above pitch() is written to cover two declarations, but the generator attaches it to pitch() alone — so pitch()'s reference entry reads 'Chassis pitch and roll' while roll() had no entry text at all. Minor, and now half-closed by roll() getting its own comment.**

> **DEFECTS1 → FIX.** pitch() has its own sentence; fixed with D4.

<details>
<summary>Evidence (6 lines)</summary>

```text
The source pairs them:
    /// Chassis pitch and roll (canonical, for tip detection).
    [[nodiscard]] virtual math::Angle pitch() const = 0;
    [[nodiscard]] virtual math::Angle roll() const = 0;

api_doc_tool.py only propagates one comment across a run when every declaration in the run matches _is_special_defaulted() — `=\s*(delete|default)\s*$`. A pure-virtual `= 0` does not match, so _parse_type_body() clears `pending` after pitch() and roll() parsed as UNDOCUMENTED (it was in this header's coverage list). The rendered result is a member entry whose text describes a member that is not it. The other half of the pattern is now fixed: roll() carries its own /// and pitch()'s text is untouched.
```

</details>

### I7. `include/shulib/hal/line_display.hpp:26`

**The seam says setLine() MUST NOT throw, but the shipped fake throws on an out-of-range row while the shipped PROS adapter silently returns — the same input, two behaviours, and one of them is the forbidden one.**

> **DEFECTS1 → FIX.** Neither implementation is wrong — the CONTRACT was. It now separates a device condition (never throw) from a caller precondition breach (each target answers in its own way, and both are right for their target).

<details>
<summary>Evidence (13 lines)</summary>

```text
The contract (line_display.hpp:26-27):
    // Contract: setLine() is synchronous on the caller's task, MUST NOT throw, and
    // row is in [0, kRows) — a bad row is the CALLER's precondition to keep.

fake/fake_line_display.hpp:22 throws (SHULIB_PRECONDITION routes to precondition_failed(), and the DEFAULT host handler throws PreconditionError — check.hpp:10-11):
    SHULIB_PRECONDITION(row >= 0 && row < kRows, "FakeLineDisplay::setLine: row out of range");

pros/line_display.hpp does the opposite — swallows it:
    if (row < 0 || row >= kRows) {
        return;  // caller's precondition (line_display.hpp:27); never throw here
    }

So a bad row is a red test on the host and a silent no-op on the robot. Both implementations cite the same contract line for opposite readings, which is the tell that the sentence needs to say which one it means: either the precondition-handler discipline overrides "MUST NOT throw" for contract breaches (and the PROS adapter is wrong to swallow), or it does not (and the fake is wrong to throw).
```

</details>

### I8. `include/shulib/hal/pros/distance.hpp:125`

**The T7 initial hold hardcodes 9999 and the mm-to-inch factor by hand instead of using kDistanceNoObjectMm and distanceMmToCanonical(), the named constant and converter this file already includes and uses two lines earlier.**

> **DEFECTS1 → FIX.** Through the named constant and the shared converter.

<details>
<summary>Evidence (5 lines)</summary>

```text
Line 125:

    mutable units::Length lastDistance_{9999.0 / 25.4};

But the same class screens the sentinel through the named constant at line 90 (`if (mm == kDistanceNoObjectMm)`) and converts through the shared function at line 77 (`distanceMmToCanonical(...)`), both from the already-included hal/distance_conversion.hpp, whose banner states the reason the constant exists: "kNoObjectMm below is that magic number, named" so "the adapter's screen reads as the rule it implements." The magic number and the 1/25.4 scale now live in two places; if HA-114's sentinel or the canonical unit ever changed, the screen would follow the constant and the initial hold would not.
```

</details>

### I9. `include/shulib/hal/pros/motor.hpp:127`

**brakeMode() screens a device read failure by holding the last commanded mode but, unlike every other reader in this class, does NOT increment faultedReads_ — so a port that fails only its brake-mode read is invisible to the very counter documented as the way to see a flaky port.**

> **DEFECTS1 → FIX.** brakeMode() counts its screen like the other four. Mutation-proven.

<details>
<summary>Evidence (20 lines)</summary>

```text
brakeMode() (line 127) screens without counting:
    const auto raw = motor_.get_brake_mode();
    if (raw == ::pros::v5::MotorBrake::invalid) {
        return brakeMode_;  // screened: hold last commanded
    }

Every other reader counts (faultedReads_ += 1 at lines 140, 154, 166, 181):
    const double deg = motor_.get_position();
    if (!std::isfinite(deg)) {
        faultedReads_ += 1;
        return lastPosition_;
    }

Against the documented purpose of the counter (line 187 region):
    /// How many reads were screened to last-good (T7 observability): telemetry
    /// and the loop's health policy can see a flaky port without this seam
    /// growing a validity channel F4 does not have.

'How many reads were screened' is false as written: one of the five screening
sites is uncounted.
```

</details>

### I10. `include/shulib/hal/pros/tick_pacer.hpp:16`

**The lazy first-call anchor prevents phantom catch-up ticks only at construction; an overrunning tick body reproduces exactly the same catch-up mid-run and nothing re-anchors.**

> **DEFECTS1 → FIX.** The pacer re-anchors after a tick body that overran a whole period, for the same reason the first call anchors lazily.

<details>
<summary>Evidence (1 lines)</summary>

```text
Banner lines 16-20: "prev is initialized from millis() lazily on the first pace() ... (a Robot constructed at t=0 but first paced at t=3000 must not 'catch up' 300 phantom ticks; FreeRTOS's catch-up semantics would run them back-to-back and the motion layer would see 300 zero-dt ticks)."  The project's own shim states and implements those semantics (test/pros_shim/pros/rtos.hpp:55-71): "If the wake instant is already in the past the call does not block, but *prev_time still advances — the FreeRTOS xTaskDelayUntil catch-up semantics" / `if (targetUs > s.nowUs) { s.nowUs = targetUs; } *prev_time = static_cast<std::uint32_t>(targetMs);`. So after a tick body that takes 50 ms, prevWakeMs_ is 40 ms stale and the next four pace() calls return instantly — four back-to-back near-zero-dt ticks, the identical hazard the header argues must not happen, arriving precisely when the loop is already in trouble. `anchored_` is set once and never cleared, so there is no mid-run re-anchor.
```

</details>

### I11. `include/shulib/hal/telemetry_sink.hpp:64`

**LogLevel is a one-line enum, so 4 of its 5 enumerators cannot be documented at all under the house comment-placement rules — this header can never pass check-coverage without a one-enumerator-per-line reformat, which this chunk forbids. NOT FIXED: reformatting is a code change and the decision belongs to whoever owns the enum.**

> **DEFECTS1 → REJECT.** Already fixed at HEAD — LogLevel is one enumerator per line with a /// on each, and check-coverage passes tree-wide, which it could not otherwise.

<details>
<summary>Evidence (21 lines)</summary>

```text
Code (unchanged by me):
    64: enum class LogLevel { Error, Warn, Info, Debug, Trace };

The gate's own advice claims two placements work: "write a /// comment directly
above the declaration (or a ///< comment on the same line)". I probed the parser
directly (tools/api_doc_tool.py, _parse_namespace_scope + _parse_enum_body) on
synthetic lines:

  A: '/// Severity, high to low.' above the one-liner
     Error -> doc: []   Warn -> []   Info -> []   Debug -> []   Trace -> []
  B: '///< only via SHULIB_TRACE' trailing on the one-liner
     Error -> doc: []   Warn -> []   Info -> []   Debug -> []
     Trace -> doc: ['only via SHULIB_TRACE']
  C: one enumerator per line
     all five carry their own doc

So exactly ONE of five (Trace, via ///<) is reachable without reformatting, and a
trailing ///< on a 5-enumerator line reads as if it describes the whole enum —
that is the 'looks complete, is not' failure this chunk exists to prevent, so I
wrote nothing rather than close one enumerator misleadingly. Same shape affects
BrakeMode (hal/motor.hpp:28), TrackingWheel::Role, Localizer::Quality.
```

</details>

### I12. `include/shulib/localization/complementary_fusion.hpp:110`

**A zero gain is a loud precondition failure but a zero nudge RATE — which disables correction identically — constructs silently, and then reports Accepted-but-never-applied on every tick forever.**

> **DEFECTS1 → REJECT.** The premise is false. maxNudgeRate == 0 is a USEFUL configuration (a heading-only corrector) reporting applied=0/conf=0; a near-zero GAIN reports applied=1/conf=0.9 for a fix that moved 1.8 attoinches. The ctor bans the one that lies and permits the one that works.

<details>
<summary>Evidence (14 lines)</summary>

```text
The ctor treats two equivalent misconfigurations differently (complementary_fusion.hpp:109-114, and the same pair for heading at 117-121):
    SHULIB_PRECONDITION(config.maxNudgeRate.value() >= 0.0,
                        "ComplementaryFusion: maxNudgeRate must be >= 0");
    ...
    SHULIB_PRECONDITION(config.maxGain > 0.0 && config.maxGain <= 1.0,
                        "ComplementaryFusion: maxGain must be in (0, 1]");

`maxGain == 0` and `maxNudgeRate == 0` produce the identical behaviour — every fix is accepted and nothing moves — yet one throws and the other is accepted.

It is worse than cosmetic because of what the audit then reports. With maxNudgeRate == 0, `maxNudge = 0 * dt == 0` on every tick, so:
    const bool applied = accepted && maxNudge > 0.0;        // line 267 — always false
    if (accepted) { audit.reason = diag::GateReason::Accepted; ... }   // line 291 — always Accepted

The blackbox therefore reads "gate Accepted, correction (0,0)" forever. The in-code comment at 265-266 attributes exactly that state to a transient "dt==0 stall", so a permanent misconfiguration is indistinguishable in telemetry from a one-tick timing hiccup.
```

</details>

### I13. `include/shulib/localization/ekf_fusion.hpp:837`

**With maxNudgeRate == 0 (which EkfFusion's own precondition explicitly allows), a zero-gain correction is still reported as applied with full appliedConfidence, so the Localizer clears its drift accumulator and the filter resets its process-noise accumulators for a fix that moved the estimate zero inches. ComplementaryFusion guards this exact case; EkfFusion does not.**

> **DEFECTS1 → DEFER.** R4, and this is a RETRACTION: I applied the obvious fix (gate `applied` on dPos > 0) and it reddened two E4 tests, which were right — an accepted zero-innovation fix moves no position while still shrinking P. The honest fix needs a `moved` flag where the clamp computes its scale. Defect, failed proxy and reason are now written into the branch.

<details>
<summary>Evidence (24 lines)</summary>

```text
ekf_fusion.hpp:374 permits it: SHULIB_PRECONDITION(config.maxNudgeRate.value() >= 0.0, "EkfFusion: maxNudgeRate must be >= 0");

ekf_fusion.hpp:836-844, with no budget test anywhere in the branch:
            if (o.accepted) {
                ++acceptedFixes_;
                out.applied = true;
                out.appliedConfidence = std::max(out.appliedConfidence,
                                                 std::clamp(p.confidence, 0.0, 1.0));
                posBudget = std::max(0.0, posBudget - o.dPos);
                lastAppliedPos_ += o.dPos;
                travelSinceFix_ = 0.0;  // the accumulated systematic bias was corrected
                timeSinceFix_ = 0.0;

complementary_fusion.hpp:265-267 rules the other way, in as many words:
        // "applied" means a fix was actually incorporated this tick: accepted by the gate AND the
        // per-tick budget allowed motion (maxNudge == 0 on a dt==0 stall => nothing could be applied).
        const bool applied = accepted && maxNudge > 0.0;

Measured, both tiers with maxNudgeRate = 0, same prediction and same proposal (2 in away, sigma 1.0, confidence 0.9):
  EKF   maxNudgeRate=0 -> applied=1 conf=0.900 x=0.099752339 accepted=1 trace=1152.003073
  COMP  maxNudgeRate=0 -> applied=0 conf=0.000 x=0.100000000
(x=0.0997 against a handed prediction of 0.1 is the velocity-filtering residual, not a correction.)

The consequence is not local: localizer.hpp consumes appliedConfidence as retain = 1.0 - clamp(fr.appliedConfidence, 0, 1) to shrink distanceSinceCorrection_, so quality() climbs 90% toward "recently corrected" on a fix that moved nothing. It also contradicts the file banner's claim at ekf_fusion.hpp:392-394 that the two tiers "agree about what a stalled tick means" — they agree for dt<=0, and disagree for a zero budget.
```

</details>

### I14. `include/shulib/localization/gps_corrector.hpp:206`

**The non-finite-input decline sets lastVerdict_ but increments no counter, so a tick it rejects is invisible in the per-source accounting this class exists to provide.**

> **DEFECTS1 → FIX.** The non-finite decline counts, so the per-source tally sums to the number of propose() calls again.

<details>
<summary>Evidence (5 lines)</summary>

```text
if (!std::isfinite(now) || !std::isfinite(px) || !std::isfinite(py)) {
    return decline(diag::GateReason::RejectedNoFix);  // nothing sane to reason from
}

Both other RejectedNoFix paths do count themselves (line 224 `++noFixTicks_;` for hasFix()==false, line 235 `++noFixTicks_;` for a non-finite GPS read). Here nothing moves, so accepted_ + noFixTicks_ + staleTicks_ + qualityRejects_ + yawRateRejects_ + innovationRejects_ stops equalling the number of propose() calls, and a run whose odometry went non-finite reads in the blackbox as a run in which this corrector was never asked — the exact confusion noFixTicks() is documented to prevent ("This is the number that says 'Driving Skills' out loud").
```

</details>

### I15. `include/shulib/localization/localizer.hpp:330`

**With two correctors proposing on the same tick, AppliedCorrection::source always names the FIRST-REGISTERED one regardless of which fix actually moved the estimate, and pairs that name with a confidence that may belong to the other corrector.**

> **DEFECTS1 → FIX.** FusionResult identifies no winner, so the fix is not a better guess: one proposer is attributed, several report "multiple". A faithful per-source split is an API change and is written up.

<details>
<summary>Evidence (7 lines)</summary>

```text
localizer.hpp:330 — `const char* source = (fr.applied && n > 0) ? names[0] : "none";`

`names[0]` is simply the first corrector that produced a valid proposal this tick; nothing here asks which one won. ComplementaryFusion::fuse folds EVERY in-gate proposal (`sumX += nudgeX; sumY += nudgeY;` inside `for (const CorrectionProposal& p : valid)`) and reports `appliedConfidence` from `maxConf = std::max(maxConf, conf)` — the STRONGEST proposal's confidence. FusionResult returns no index identifying it.

So on a tick where GPS (registered first) and AprilTag both pass the gate and the tag's fix dominates, one AppliedCorrection record carries corrector[0]'s NAME beside corrector[1]'s CONFIDENCE. The same value also drives Localizer's drift accounting: `retain = 1.0 - clamp(fr.appliedConfidence, 0, 1)`, so distanceSinceCorrection_ is cleared on the strength of a fix the record attributes to a different source.

This is invisible with one corrector, and E3 is precisely the chunk that makes two-corrector trees the point — the header spends a long note arbitrating WHICH silent source wins the single decline slot, but the accept path has no equivalent arbitration. The decline path is careful (selfAuditSource names who declined); the accept path is not.
```

</details>

### I16. `include/shulib/localization/pilons_odometry.hpp:59`

**PilonsOdometryConfig::maxTickRotation is a bare `double` in radians in a header that types every other physical quantity it touches, so the unit survives only in the doc comment and a caller can pass degrees without any diagnostic.**

> **DEFECTS1 → FIX.** Typed as units::AngleDim — and the retype broke two call sites at compile time, which is the point.

<details>
<summary>Evidence (1 lines)</summary>

```text
`double maxTickRotation = 0.5 * math::Angle::kPi;` — the only untyped physical quantity in the file, alongside `units::Length` travel and offsets throughout update() and a `units::Length`-typed Pose2d. units/quantity.hpp:81 defines `AngleDim` (radians) for exactly this 'rates/bookkeeping' role. The comparison it feeds is `std::abs(dTheta) > maxTickRotation_`, where dTheta comes from `h0.errorTo(h1)` in radians: passing 90.0 meaning degrees compiles, disables the trust gate entirely (nothing is ever implausible), and the constructor's only check is `config.maxTickRotation > 0.0`.
```

</details>

### I17. `include/shulib/motion/drive_brake.hpp:5`

**The design banner says the brake 'Commands ZERO volts to every drive motor each tick', but tick() stops commanding entirely once a verdict is reached — and the banner is reproduced verbatim onto the published reference page.**

> **DEFECTS1 → FIX.** "each ACTIVE tick".

<details>
<summary>Evidence (7 lines)</summary>

```text
Banner, drive_brake.hpp:5: '// Commands ZERO volts to every drive motor each tick (with BrakeMode::Brake so real hardware resists rather than coasts ...)'.

tick(), drive_brake.hpp:101-104:
    if (reason_ != control::ExitReason::Running) {
        return reason_;
    }
— returns before the motor loop at drive_brake.hpp:111-114. So after Settled/TimedOut/Cancelled nothing is re-commanded. That matches motion.hpp:34-35 ('further tick() calls are safe no-ops that ... leave the motors stopped') and is almost certainly the intended behaviour, but 'each tick' as published reads as an unconditional re-assertion the code does not perform. 'each ACTIVE tick' would be true.
```

</details>

### I18. `include/shulib/motion/motion_config.hpp:112`

**The same physical quantity — center-to-wheel distance — exists as two independently settable fields (MotionConfig::rotationRadius and MotionConfig::stall.rotationRadius), both defaulting to 7.0 and both citing HA-17, so setting one silently leaves the other stale. REPORTED, NOT FIXED.**

> **DEFECTS1 → FIX.** Documented as one quantity in two fields with the cross-check named; see DEFECTS1-COMPLETED.md for why validate() still does not descend into `stall`.

<details>
<summary>Evidence (3 lines)</summary>

```text
motion_config.hpp:110-112: `/// Center-to-wheel distance (in) — converts |omega| to an equivalent linear speed in DriveBrake's norm. Stand-in geometry (A4: HA-17/HA-52).` / `units::Length rotationRadius{7.0};` — consumed at drive_brake.hpp:119 `cfg_.rotationRadius.value() * std::abs(w / n)`.
odo_stall_check.hpp:75-77: `/// Converts |dheading| to equivalent wheel travel (≈ center-to-wheel distance). Stand-in geometry (A4: HA-17/HA-52).` / `units::Length rotationRadius{7.0};` — consumed at odo_stall_check.hpp:122, reached from MotionConfig only via the separate `stall` member (motion_config.hpp:115, wired at move_to_pose.hpp:277 `stall_{config.stall}`).
Same registered hardware claim (HA-17), same default (7.0), same meaning, two fields. When R3 measures the real geometry, `cfg.rotationRadius = measured;` updates DriveBrake's settle norm and leaves OdoStallCheck's rotation term at the 7.0 stand-in — a stall check calibrated against a number the rest of the config no longer believes. Nothing in validate() (motion_config.hpp:136-148) cross-checks them, and validate() explicitly declines to look inside `stall` at all.
```

</details>

### I19. `include/shulib/motion/motion_scheduler.hpp:785`

**lastExitReason() and lastCompleted().exit answer the same question with two different values on a scheduler that has never run a motion: Settled vs Running.**

> **DEFECTS1 → REJECT.** Both values are recorded rulings and every available change relocates the disagreement rather than removing it — setting lastExit_ = Running would make waitUntilSettled() return a value its own contract says it never returns, on an F6-frozen facade. The discriminator is already named in lastCompleted()'s and completedCount()'s comments.

<details>
<summary>Evidence (9 lines)</summary>

```text
control::ExitReason lastExit_ = control::ExitReason::Settled;  // line 1182, "vacuous default"
    CompletedMotion last_{};                                       // exit defaults to Running

CompletedMotion declares:
    control::ExitReason exit = control::ExitReason::Running;  ///< Running ⇒ "none yet"

while lastExitReason() is documented as "Settled before any motion has finished (the vacuous-wait default)".

FAILURE SCENARIO: on a virgin scheduler (or one whose caller checks status before issuing any motion), `sched.lastExitReason()` returns Settled — which reads as "the last motion succeeded" — while `sched.lastCompleted().exit` returns Running, the struct's documented "none yet". A caller that switches on one and logs the other reports a success that never happened. Each value is individually defensible (Settled makes waitUntilSettled() vacuously correct with nothing active; Running is the honest CompletedMotion default), and each is documented in isolation, which is precisely what makes the disagreement easy to trip on: neither comment is wrong, but together they describe two different truths about the same instant. completedCount() == 0 is the only reliable discriminator, and nothing points a reader at it. Low severity; noted because I had to state the discrepancy explicitly in lastCompleted()'s new comment to keep it honest.
```

</details>

### I20. `include/shulib/spec/accuracy.hpp:23`

**The heading targets are bare `double` values in DEGREES while the position targets in the same file are typed `units::Length` and the library's canonical angle unit is radians — the one file that is the single source of truth for the accuracy spec is also the one place the units discipline is not applied.**

> **DEFECTS1 → ARGUE.** spec/accuracy.hpp IS Freeze Register row F2, LOCKED. Written up.

<details>
<summary>Evidence (1 lines)</summary>

```text
`inline constexpr double kHeadingErrorMaxDeg = 1.0;` and `inline constexpr double kDockedHeadingTypicalDeg = 0.5;` sit three lines above `inline constexpr units::Length kPositionErrorEndOfRun{1.0};`. units/quantity.hpp:81 already provides `using AngleDim = Quantity<0,1,0,0,0>;   // radians`. Every consumer therefore has to convert by hand: test/accuracy_spec_test.cpp:107-111 computes `std::abs(...errorTo(...)) / (Angle::kPi / 180.0)` before comparing. A bare double whose unit lives only in the identifier suffix is the exact foot-gun the typed-quantity vocabulary exists to remove, and here it guards the one HARD requirement in the spec.
```

</details>

### I21. `include/shulib/units/literals.hpp:59`

**The angle literals are the only ones in the header that are not constexpr, so `90_deg` cannot initialize a constexpr variable while `24_in` can — and the blocking check can never fail on a literal.**

> **DEFECTS1 → ARGUE.** MEASURED, not argued. An identity-interval early-out makes std::remainder unreachable during constant evaluation and clang then accepts degrees(90.0) — 0 mismatches over 1,028,583 samples. But it buys a CLIFF (90_deg constexpr, 315_deg not, with no rule a caller can see) for ~6 lines inside LOCKED row F3. A decision, not an edit.

<details>
<summary>Evidence (11 lines)</summary>

```text
constexpr Length operator""_in(long double v) noexcept          // line 22 — constexpr
    inline ::shulib::math::Angle operator""_deg(unsigned long long v) // line 59 — not

Verified:
    $ cat lit2.cpp
    constexpr auto kAngle = 90_deg;
    $ g++ -std=gnu++20 -Iinclude -fsyntax-only lit2.cpp
    lit2.cpp:3:25: error: call to non-'constexpr' function
      'shulib::math::Angle shulib::units::literals::operator""_deg(long long unsigned int)'

while `constexpr auto k = 24_in;` compiles. The cause is math/angle.hpp:33-42, where radians()/degrees() run SHULIB_PRECONDITION(std::isfinite(...)) — a runtime check that cannot fail for a literal operand, since an out-of-range floating literal is already diagnosed by the compiler. The header's own banner advertises the two-forms symmetry ("so both `24_in` and `24.0_in` compile") and says nothing about this asymmetry, so a caller writing `static constexpr Angle kTarget = 90_deg;` in a config struct discovers it at the error message. I documented the behaviour at literals.hpp:59 rather than changing it. NOT FIXED.
```

</details>

### I22. `include/shulib/units/quantity.hpp:85`

**Scalar operators are asymmetric: `double * Quantity` is provided but `double / Quantity` is not, so `2.0 * dt` compiles and `2.0 / dt` does not — even though the latter has a well-defined derived dimension.**

> **DEFECTS1 → FIX.** The header now says WHY there is no double / Quantity: this block is dimension-preserving scaling, and an inverse belongs with the Quantity-by-Quantity operators.

<details>
<summary>Evidence (7 lines)</summary>

```text
Both multiplication orders exist:
    friend constexpr Quantity operator*(Quantity a, double s) noexcept { return Quantity{a.v_ * s}; }
    friend constexpr Quantity operator*(double s, Quantity a) noexcept { return Quantity{s * a.v_}; }
Division has only the quantity-on-the-left form:
    friend constexpr Quantity operator/(Quantity a, double s) noexcept { return Quantity{a.v_ / s}; }

There is no `operator/(double, Quantity)` and no namespace-scope equivalent, so building an inverse dimension from a plain number requires the workaround `Number{1.0} / dt` (which does work: Quantity<0,0,0,0,0> / Quantity<0,0,1,0,0> -> Quantity<0,0,-1,0,0>). This may well be deliberate — a scalar-over-quantity divide always changes dimension, unlike the scaling operators grouped around it — but nothing in the header says so, and the missing overload reads as an oversight next to the deliberately-doubled multiply. I documented the workaround on operator/ rather than assume intent.
```

</details>

---

## O — everything else

*(item `O1`)*

### O1. `include/shulib/localization/localizer.hpp:138`

**The coverage gate demands docs for 4 enumerators the parser CANNOT attach a doc to, because Localizer::Quality is declared on one line. Closing it requires reflowing the declaration, which this chunk forbids. This is the one item in batch 13 I could not close.**

> **DEFECTS1 → REJECT.** Already fixed at HEAD — Localizer::Quality is one enumerator per line with its own ///.

<details>
<summary>Evidence (15 lines)</summary>

```text
Line 138: `enum class Quality { Uninitialized, DeadReckon, Corrected, Degraded };`

api_doc_tool's _parse_enum_body starts AT the opener line with a fresh local `pending`, so a `///` above the line is consumed by _parse_type_body as the enum TYPE's doc and never reaches an enumerator; only a trailing `///<` binds, and only to the LAST item. I proved this rather than assuming it, by driving the real parser on three fixtures:

  A: one-liner, /// above (today's shape)
     enum class C::Quality: documented=True
        Uninitialized: documented=False   DeadReckon: documented=False
        Corrected:     documented=False   Degraded:   documented=False
  B: one-liner + trailing ///<
        Uninitialized/DeadReckon/Corrected: documented=False
        Degraded: documented=True  doc=['trailing text']
  C: expanded, one enumerator per line
        all four: documented=True

The tool's own docstring names this and calls it intended pressure: "on a one-liner with several, the rest are undocumented and the gate says so - which is the pressure to expand it, and is intended." So the tool's sanctioned fix is fixture C - reflowing line 138 to one enumerator per line with `///<` on each. That is a whitespace-inside-code change, which hard constraint 1 forbids, so I left it alone rather than reformat silently or paper over 1 of 4 with a trailing comment that would visually read as documenting the whole enum. TrackingWheel::Role (localization/tracking_wheel.hpp:39) is the tree's only other one-liner and has the identical problem, for whoever owns that batch. Remedy is 6 lines and mine to hand over, not to take.
```

</details>

---

---

## N — found by DEFECTS1's triage, not by DOCS2

*(item `N1`)*

> Added 2026-08-15 by DEFECTS1. Not one of the 83: this one surfaced while an adversarial
> reader was checking D6's mechanism, and it is filed here so the list stays the one place a
> reader looks. The 83 count above is deliberately unchanged — collapsing or inflating it would
> put this document at odds with DOCS2's completion record for no gain.

### N1. `include/shulib/localization/pilons_odometry.hpp:111`

**A tracking pod that enumerates LATE injects a one-tick phantom translation, and the odometry's own plausibility gate cannot see it: the gate checks |Δθ| and never |Δtravel|.**

> **DEFECTS1 → FIX.** NEW — not on DOCS2's list. Found by this chunk's triage, probe-verified at 28.4 in of phantom translation when a cold pod enumerates. Landed as a VISIBILITY fix and labelled as one: the bound is dt-blind (this class has no clock), so the delta is reported rather than withheld. HA-123.

<details>
<summary>Evidence (12 lines)</summary>

```text
A pod that is dead or not yet enumerated at construction reads 0, so TrackingWheel baselines
lastShaft_ at 0 (tracking_wheel.hpp:95). On the tick the pod finally answers, travelDelta()
differences its TRUE cumulative position against that 0.

Probe (g++ -std=gnu++20 -I include, FakeRotation + the real TrackingWheel):
  N1 baseline at construction: travelDelta=0.000000000
  N1 pod wakes at 1000 deg: NEXT travelDelta=28.361600 in  <-- phantom
  N1 following tick (steady): travelDelta=0.000000000

pilons_odometry.hpp:111 gates only rotation:
    implausible_ = !finite || std::abs(dTheta) > maxTickRotation_;
so a finite 28-inch translation passes the plausibility check unflagged and is integrated
straight into the pose. The scheduler's PoseDeltaGuard may notice the resulting jump, but that
guard is advisory by design and does not correct the estimate.
```

</details>

---

## E — found during the correction pass, after the adversarial review

*(items `E1`–`E10`)*

These ten were not found while writing the documentation. They were found while **fixing** it:
a reviewer flagged a sentence as wrong, the fixer went to the code to check, and the code turned
out to be the thing at fault. Each is a case where the honest comment could only be written by
first admitting a defect — which is the strongest argument in this chunk for why documenting a
surface finds bugs in it.

### E1. `include/shulib/hal/pros/motor.hpp:82`

**OBSERVABILITY GAP (reported, not fixed): brakeMode()'s sentinel screening is invisible to telemetry.**

> **DEFECTS1 → FIX.** Same defect as I9; landed with it.

<details>
<summary>Full finding (93 words)</summary>

```text
OBSERVABILITY GAP (reported, not fixed): brakeMode()'s sentinel screening is invisible to
telemetry. A motor whose get_brake_mode() returns MotorBrake::invalid on every tick is silently
served the last commanded mode and reports faultedReads() == 0, so the loop's health policy
cannot distinguish it from a healthy port. The other four readers all bump the counter on the
same class of failure. If the counter is meant to answer 'is this port flaky', brakeMode()
should increment it too (or expose a separate tally); if it is deliberately a sensor-path-only
signal, that is now documented as such.
```

</details>

### E2. `include/shulib/hal/pros/motor.hpp:118`

**REPORTED, NOT FIXED: the ProsMotor constructor guards the persistent-device-state trap for encoder units and gearing (it sets both explicitly and SHULIB_PRECONDITIONs on the read-back, per the banner's trap A / HA-98 reasoning) but leaves brake mode entirely inherited from whatever program last configured that port.**

> **DEFECTS1 → FIX.** Sharper than reported: brakeMode_ is ALSO the T7 fallback, so a port left in Hold and dying before any command reported Coast forever. Seeded from the device now. Mutation-proven.

<details>
<summary>Full finding (124 words)</summary>

```text
REPORTED, NOT FIXED: the ProsMotor constructor guards the persistent-device-state trap for
encoder units and gearing (it sets both explicitly and SHULIB_PRECONDITIONs on the read-back,
per the banner's trap A / HA-98 reasoning) but leaves brake mode entirely inherited from
whatever program last configured that port. That is the same trap, unguarded — a robot can boot
with a drivetrain in Hold because a previous session left it there, and nothing in this adapter
notices. The banner's own argument ('a motor that ignores configuration is a miswired robot, and
finding that at boot beats finding it mid-match') applies verbatim. Fixing it would mean either
a ctor-set default or a documented requirement to call setBrakeMode() at init; both are code
changes and out of scope here.
```

</details>

### E3. `include/shulib/hal/pros/optical.hpp:51`

**Cold-start hole in ProsOptical, not fixed per this chunk's standing rule: the last-good caches (optical.hpp:116-119) initialise to 0.0 and there is no first-successful-read flag, so a device that fails every read from construction serves hue()==0.0 (red), saturation()/brightness()/proximity()==0.0 indefinitely — indistinguishable at the seam from genuine readings.**

> **DEFECTS1 → DEFER.** F3. IOptical has zero consumers outside hal/, there is no honest finite seed for hue (NaN is forbidden at this seam by F4), and the chunk that writes the first consumer owns the validity decision.

<details>
<summary>Full finding (122 words)</summary>

```text
Cold-start hole in ProsOptical, not fixed per this chunk's standing rule: the last-good caches
(optical.hpp:116-119) initialise to 0.0 and there is no first-successful-read flag, so a device
that fails every read from construction serves hue()==0.0 (red),
saturation()/brightness()/proximity()==0.0 indefinitely — indistinguishable at the seam from
genuine readings. The T7 policy in the file banner (lines 22-24) says 'never zero' precisely to
avoid this, so the implementation misses its own stated policy in the cold-start window.
faultedReads() is the only tell, and it requires the caller to have sampled it before trusting a
first reading. A validity flag (or seeding the caches with NaN, which the F4 finiteness contract
forbids at this seam) would close it; the comment now documents the real behaviour instead.
```

</details>

### E4. `include/shulib/control/watchdog.hpp:14`

**Reported, not fixed (standing rule for this chunk): the §M2 guarantee 'a motion can never hang' is not achievable with the current design against a control task that stops running.**

> **DEFECTS1 → DEFER.** R4/T2. A polled watchdog cannot beat a stopped task without a supervisory task, and nothing in the library owns one.

<details>
<summary>Full finding (102 words)</summary>

```text
Reported, not fixed (standing rule for this chunk): the §M2 guarantee 'a motion can never hang'
is not achievable with the current design against a control task that stops running. Watchdog is
purely polled and every in-tree consumer (exit_group.hpp:60, move_to_pose.hpp:142/:185,
turn_to.hpp:108, mechanism_op.hpp:364) evaluates it from inside the very tick() that would be
blocked. Nothing in the library owns an independent task, so a tick() that blocks forever
(deadlocked mutex, blocking sensor read) produces neither a TimedOut nor the guaranteed end-of-
run park. Closing this needs a supervisory task or a pacer-side deadline, i.e. a code change; I
only documented the real boundary honestly.
```

</details>

### E5. `include/shulib/hal/pros/rotation.hpp:75`

**include/shulib/hal/pros/rotation.hpp:95-96 — ProsRotation's hold-last-good sentinel screen has no valid-yet flag and seeds both caches to zero (`mutable units::AngleDim lastPosition_{0.0}`, `mutable units::AngularVelocity lastVelocity_{0.0}`).**

> **DEFECTS1 → REJECT.** Duplicate report of D6, closed by its fix. E5's additional motor-half claim is separately refuted: ProsMotor's ctor read-back THROWS on a port that did not answer.

<details>
<summary>Full finding (210 words)</summary>

```text
include/shulib/hal/pros/rotation.hpp:95-96 — ProsRotation's hold-last-good sentinel screen has
no valid-yet flag and seeds both caches to zero (`mutable units::AngleDim lastPosition_{0.0}`,
`mutable units::AngularVelocity lastVelocity_{0.0}`). Consequence: a pod that returns PROS_ERR
on its very first read — unplugged, dead, or not yet enumerated at boot — publishes 0 rad and 0
rad/s through IRotation, which is the exact 'robot stopped' reading the screen is designed to
never emit. This defeats the stated rationale for the whole class: diag's ODO_STUCK /
odomStalled cross-check is built to notice a FROZEN NON-ZERO value, and a frozen ZERO makes a
dead pod look like a stationary robot instead of a visible fault.
localization/tracking_wheel.hpp reads deltas (line 74), so a stuck-at-zero position yields zero
travel forever rather than an obviously bogus one. Fix would be a have-a-value flag, or seeding
the caches to NaN/a sentinel so the first faulted read is distinguishable — NOT APPLIED, per
this chunk's report-don't-fix rule. Only observable workaround today is faultedReads() > 0
alongside a zero output, which is what I documented. Same pattern exists in the sibling
include/shulib/hal/pros/motor.hpp (lastPosition_/lastVelocity_/lastCurrent_ all {0.0} at lines
228-230); that file is outside this batch so I did not touch it, but the defect class is
identical there and only its temperature() seed (20.0) is currently documented.
```

</details>

### E6. `include/shulib/hal/pros/rotation.hpp:52`

**Same defect as finding 4 — include/shulib/hal/pros/rotation.hpp:95-96, zero-seeded last-good caches with no valid-yet flag, so the 'never zero' guarantee the class doc and the design banner both assert holds only after the first successful read.**

> **DEFECTS1 → REJECT.** Self-declared duplicate of E5, itself a duplicate of D6. Closed there.

<details>
<summary>Full finding (59 words)</summary>

```text
Same defect as finding 4 — include/shulib/hal/pros/rotation.hpp:95-96, zero-seeded last-good
caches with no valid-yet flag, so the 'never zero' guarantee the class doc and the design banner
both assert holds only after the first successful read. Reported, not fixed. Documented at both
published locations (class doc and header banner) since api_doc_tool.py reproduces the banner in
full on the generated page.
```

</details>

### E7. `include/shulib/localization/localizer.hpp:105`

**LocalizerConfig::minDt is the only config field with no constructor precondition, and there is no cross-field check that minDt <= maxDt.**

> **DEFECTS1 → FIX.** minDt > 0 and minDt < maxDt. Mutation-proven.

<details>
<summary>Full finding (107 words)</summary>

```text
LocalizerConfig::minDt is the only config field with no constructor precondition, and there is
no cross-field check that minDt <= maxDt. Two silent misconfigurations follow, neither of which
trips red-on-failure: (a) minDt > maxDt empties the trusted band, so every tick reports zero
linear velocity and Degraded quality for the life of the run; (b) minDt <= 0 disables the low-dt
guard entirely, letting a near-zero interval produce an unbounded velocity spike at
localizer.hpp:314-315. NOT FIXED per this chunk's standing rule — the fix would be two
SHULIB_PRECONDITIONs alongside the maxDt one at line 169 (minDt > 0 and minDt < maxDt).
Documented honestly in the comment instead.
```

</details>

### E8. `include/shulib/localization/apriltag_corrector.hpp:435`

**AppliedCorrection::source cannot attribute a fix once more than one corrector is registered — the exact configuration E3 exists to enable.**

> **DEFECTS1 → FIX.** Same defect as I15; landed with it.

<details>
<summary>Full finding (97 words)</summary>

```text
AppliedCorrection::source cannot attribute a fix once more than one corrector is registered —
the exact configuration E3 exists to enable. Localizer::update() (localizer.hpp:349) picks
names[0], the first VALID proposer in registration order, while ComplementaryFusion sums every
accepted proposal (complementary_fusion.hpp:208-209, 257-261), so a landed tag fix is
telemetered as "gps" whenever a GPS corrector is registered first and also proposed that tick.
NOT FIXED per the standing rule. A faithful record would need either the per-proposal
contribution back from FusionResult or a source list rather than one slot; both are API changes.
Documented as a limitation on name() instead.
```

</details>

### E9. `include/shulib/diag/health_monitor.hpp:109`

**HealthMonitor's constructor (include/shulib/diag/health_monitor.hpp:111-120) validates finiteness for brownoutVolts and maxMotorTempC but NOT for brownoutRecoverVolts, which is only ordered against brownoutVolts.**

> **DEFECTS1 → FIX.** isfinite added to brownoutRecoverVolts. Mutation-proven, with a second test pinning the payoff: two collapses are now two episodes.

<details>
<summary>Full finding (121 words)</summary>

```text
HealthMonitor's constructor (include/shulib/diag/health_monitor.hpp:111-120) validates
finiteness for brownoutVolts and maxMotorTempC but NOT for brownoutRecoverVolts, which is only
ordered against brownoutVolts. brownoutRecoverVolts = +Inf therefore passes construction, and
tick()'s re-arm branch (`else if (v >= cfg_.brownoutRecoverVolts.value())`, line 162) can never
fire, so brownoutActive_ latches on the first trip and every later brownout episode is silently
swallowed for the rest of the run — the anti-spam edge trigger turns into a permanent mute on
the one signal the E1 latched-brownout semantics exist to report. NaN is rejected only
incidentally, because NaN >= x is false. The fix would be one added
`std::isfinite(cfg_.brownoutRecoverVolts.value())` term in the second SHULIB_PRECONDITION. NOT
APPLIED — this chunk's standing rule is comments only; the comment now documents the real
behaviour.
```

</details>

### E10. `include/shulib/control/trapezoid_profile.hpp:91`

**REPORTED, NOT FIXED (no code was touched).**

> **DEFECTS1 → FIX.** sample() and isDone() reject a non-finite t. NOTE a signature change: isDone() drops noexcept, because the precondition handler throws and a noexcept frame would make a caller bug std::terminate. In the changelog.

<details>
<summary>Full finding (144 words)</summary>

```text
REPORTED, NOT FIXED (no code was touched). TrapezoidProfile::sample() has an unguarded non-
finite clock path that fails asymmetrically: sample(NaN) returns {position=NaN, velocity=NaN,
acceleration=-aMax}, a partially-finite state. A caller doing per-field finiteness validation on
the returned ProfileState will pass the acceleration field and can forward a plausible-looking
down-ramp acceleration downstream. Compounding it, isDone(NaN) returns false (probe output
`isDone(NaN)=0`), because it is `t >= duration_` and every NaN comparison is false — so a
follower loop that terminates on isDone() would spin forever on a NaN clock rather than failing
fast. The constructor guards distance with SHULIB_PRECONDITION(std::isfinite(distance));
sample() and isDone() apply no equivalent guard to t. This is latent today (nothing outside the
test calls either), but it is the kind of thing that bites when the profiled-motion work wires a
real clock in. The comment now documents the true behaviour instead of implying an all-NaN
sentinel.
```

</details>
