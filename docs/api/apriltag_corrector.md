<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/apriltag_corrector.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `apriltag_corrector.hpp`

AprilTagCorrector — the SECOND real corrector, and the FIRST source in the tree that can tell the estimator which way it is actually pointing.

This header declares **2** types (33 members).

Extracted from [`include/shulib/localization/apriltag_corrector.hpp`](../../include/shulib/localization/apriltag_corrector.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct AprilTagCorrectorConfig`](#struct-apriltagcorrectorconfig)
  - [`latency`](#apriltagcorrectorconfig-latency)
  - [`maxObservationAge`](#apriltagcorrectorconfig-maxobservationage)
  - [`minRange`](#apriltagcorrectorconfig-minrange)
  - [`maxRange`](#apriltagcorrectorconfig-maxrange)
  - [`minConfidence`](#apriltagcorrectorconfig-minconfidence)
  - [`maxYawRate`](#apriltagcorrectorconfig-maxyawrate)
  - [`baseStdDev`](#apriltagcorrectorconfig-basestddev)
  - [`stdDevPerInch`](#apriltagcorrectorconfig-stddevperinch)
  - [`gateSigma`](#apriltagcorrectorconfig-gatesigma)
  - [`postFixStdDev`](#apriltagcorrectorconfig-postfixstddev)
  - [`driftStdDevPerInch`](#apriltagcorrectorconfig-driftstddevperinch)
- [`class AprilTagCorrector`](#class-apriltagcorrector)
  - [`kHistory`](#apriltagcorrector-khistory)
  - [`kMaxTagsPerFrame`](#apriltagcorrector-kmaxtagsperframe)
  - [`kMinConfidenceFloor`](#apriltagcorrector-kminconfidencefloor)
  - [`AprilTagCorrector`](#apriltagcorrector-apriltagcorrector)
  - [`droppedTags`](#apriltagcorrector-droppedtags)
  - [`poll`](#apriltagcorrector-poll)
  - [`propose`](#apriltagcorrector-propose)
  - [`name`](#apriltagcorrector-name)
  - [`lastVerdict`](#apriltagcorrector-lastverdict)
  - [`lastTagId`](#apriltagcorrector-lasttagid)
  - [`pollCount`](#apriltagcorrector-pollcount)
  - [`acceptedFixes`](#apriltagcorrector-acceptedfixes)
  - [`noFrameTicks`](#apriltagcorrector-noframeticks)
  - [`staleFrameTicks`](#apriltagcorrector-staleframeticks)
  - [`staleTicks`](#apriltagcorrector-staleticks)
  - [`noTagTicks`](#apriltagcorrector-notagticks)
  - [`unmappedRejects`](#apriltagcorrector-unmappedrejects)
  - [`rangeRejects`](#apriltagcorrector-rangerejects)
  - [`qualityRejects`](#apriltagcorrector-qualityrejects)
  - [`yawRateRejects`](#apriltagcorrector-yawraterejects)
  - [`innovationRejects`](#apriltagcorrector-innovationrejects)
  - [`travelSinceFix`](#apriltagcorrector-travelsincefix)

<a id="struct-apriltagcorrectorconfig"></a>

## `struct AprilTagCorrectorConfig`

```cpp
struct AprilTagCorrectorConfig
```

Tuning for AprilTagCorrector. Every default is PROVISIONAL — there is no robot, no camera and no measured tag layout — and each carries its A4 Hardware Assumptions Register entry. E3 proves the corrector's LOGIC; R4 measures the constants. Nothing here was tuned to make the simulated camera look good, which is an explicit non-goal of this chunk.

*struct, declared at [`include/shulib/localization/apriltag_corrector.hpp:123`](../../include/shulib/localization/apriltag_corrector.hpp#L123).*

<a id="apriltagcorrectorconfig-latency"></a>

### `AprilTagCorrectorConfig::latency`

```cpp
units::Time latency{0.08}
```

End-to-end delay between the instant a frame describes and the instant its reduced tags can be read (exposure + detect + PnP + transport). Larger than the GPS's because a tag pipeline does more work per frame. PROVISIONAL (A4: HA-71) — invented, ≈80 ms.

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:127`](../../include/shulib/localization/apriltag_corrector.hpp#L127).*

<a id="apriltagcorrectorconfig-maxobservationage"></a>

### `AprilTagCorrectorConfig::maxObservationAge`

```cpp
units::Time maxObservationAge{0.25}
```

Decline once the newest snapshot is older than this: the vision task has stalled, died, or was never started. Distinct from "we looked and saw nothing" on purpose. PROVISIONAL (A4: HA-72).

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:131`](../../include/shulib/localization/apriltag_corrector.hpp#L131).*

<a id="apriltagcorrectorconfig-minrange"></a>

### `AprilTagCorrectorConfig::minRange`

```cpp
units::Length minRange{6.0}
```

Trusted range band, measured from the ROBOT CENTRE. Below `minRange` the tag overfills the frame and is likely clipped; above `maxRange` the planar-PnP heading ambiguity (hal/vision_conversion.hpp) makes the orientation untrustworthy well before the position is. PROVISIONAL (A4: HA-73).

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:136`](../../include/shulib/localization/apriltag_corrector.hpp#L136).*

<a id="apriltagcorrectorconfig-maxrange"></a>

### `AprilTagCorrectorConfig::maxRange`

```cpp
units::Length maxRange{72.0}
```

Upper edge of that band (inches, from the robot centre). An observation outside [minRange, maxRange] is DISCARDED, not down-weighted — the blunt instrument E3 chose over inventing a second noise number for heading. Precondition: maxRange > minRange.

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:140`](../../include/shulib/localization/apriltag_corrector.hpp#L140).*

<a id="apriltagcorrectorconfig-minconfidence"></a>

### `AprilTagCorrectorConfig::minConfidence`

```cpp
double minConfidence = 0.35
```

Detector confidence below this is not worth folding — the tag analogue of E2's sensor-quality ceiling (D7): without it, a 0.05-confidence detection is still folded with a microscopic pull, and the Localizer reports quality class Corrected on a run with no usable anchor. PROVISIONAL (A4: HA-74).

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:145`](../../include/shulib/localization/apriltag_corrector.hpp#L145).*

<a id="apriltagcorrectorconfig-maxyawrate"></a>

### `AprilTagCorrectorConfig::maxYawRate`

```cpp
units::AngularVelocity maxYawRate{2.0}
```

Decline any observation taken while the yaw rate exceeded this. A spinning robot smears the tag across the frame, and a rolling shutter skews it into a different quadrilateral — which PnP will happily solve, into a confidently wrong pose. PROVISIONAL (A4: HA-75).

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:149`](../../include/shulib/localization/apriltag_corrector.hpp#L149).*

<a id="apriltagcorrectorconfig-basestddev"></a>

### `AprilTagCorrectorConfig::baseStdDev`

```cpp
units::Length baseStdDev{1.0}
```

Position 1σ of a tag fix at zero range and confidence 1, and its growth per inch of range. PROVISIONAL (A4: HA-76).

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:152`](../../include/shulib/localization/apriltag_corrector.hpp#L152).*

<a id="apriltagcorrectorconfig-stddevperinch"></a>

### `AprilTagCorrectorConfig::stdDevPerInch`

```cpp
double stdDevPerInch = 0.02
```

Growth of that 1σ per inch of RANGE — inches of σ per inch, so 0 makes a fix's σ range-independent. Must be >= 0.

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:155`](../../include/shulib/localization/apriltag_corrector.hpp#L155).*

<a id="apriltagcorrectorconfig-gatesigma"></a>

### `AprilTagCorrectorConfig::gateSigma`

```cpp
double gateSigma = 4.0
```

Gate width in units of σ_eff, same meaning as E2's. PROVISIONAL (A4: HA-77).

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:157`](../../include/shulib/localization/apriltag_corrector.hpp#L157).*

<a id="apriltagcorrectorconfig-postfixstddev"></a>

### `AprilTagCorrectorConfig::postFixStdDev`

```cpp
units::Length postFixStdDev{1.0}
```

The estimate's position 1σ immediately after THIS source's fix is folded — the floor of σ_dr, so confidence is never 0. PROVISIONAL (A4: HA-78).

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:160`](../../include/shulib/localization/apriltag_corrector.hpp#L160).*

<a id="apriltagcorrectorconfig-driftstddevperinch"></a>

### `AprilTagCorrectorConfig::driftStdDevPerInch`

```cpp
double driftStdDevPerInch = 0.02
```

Growth of the dead-reckoning 1σ per inch travelled since this source's last fix — the anti-lockout term E2's D2 exists to explain. PROVISIONAL (A4: HA-79).

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:163`](../../include/shulib/localization/apriltag_corrector.hpp#L163).*

<a id="class-apriltagcorrector"></a>

## `class AprilTagCorrector`

```cpp
class AprilTagCorrector final : public ICorrector
```

The corrector that turns one tag sighting into an ABSOLUTE field pose — position AND heading, making it the first source in the tree that can tell the estimator which way it is actually pointing. THE TWO-METHOD SHAPE IS THE CONTRACT, and getting it wrong fails silently: poll() is the ONLY method that touches ITagSource, whose tags() returns a std::vector by value and so heap-allocates, which is why poll() belongs on a VISION-rate task and propose() can run every control tick allocating nothing. propose() is not sensor-free, though — it reads the injected clock and the IMU (heading AND yaw rate) on every call, so both must be live and wired before the control loop starts. A corrector nobody polls proposes nothing, forever — pollCount() and a RejectedNoFix verdict every tick are what make that diagnosable. It picks the single best-σ tag rather than averaging several, it computes no PnP (the seam hands it an already-reduced pose), it owns no tag map, and it never writes a pose or a heading: it only ever PROPOSES, and how far the estimate moves is the fusion policy's bounded nudge.

*class, declared at [`include/shulib/localization/apriltag_corrector.hpp:178`](../../include/shulib/localization/apriltag_corrector.hpp#L178).*

<a id="apriltagcorrector-khistory"></a>

### `AprilTagCorrector::kHistory`

```cpp
static constexpr std::size_t kHistory = 64
```

Ticks of predicted-pose history kept for latency compensation. 64 ticks is ~0.64 s at 100 Hz against an ~80 ms latency. Fixed capacity: the hot path never allocates.

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:182`](../../include/shulib/localization/apriltag_corrector.hpp#L182).*

<a id="apriltagcorrector-kmaxtagsperframe"></a>

### `AprilTagCorrector::kMaxTagsPerFrame`

```cpp
static constexpr std::size_t kMaxTagsPerFrame = 8
```

Tags kept from one poll. More than this in view at once means either a very tag-rich field or a detector hallucinating; either way the best-sigma pick only needs a few.

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:185`](../../include/shulib/localization/apriltag_corrector.hpp#L185).*

<a id="apriltagcorrector-kminconfidencefloor"></a>

### `AprilTagCorrector::kMinConfidenceFloor`

```cpp
static constexpr double kMinConfidenceFloor = 0.05
```

Floor under the divisor in σ_meas, so a zero-confidence detection cannot produce an infinite σ (and, through it, a NaN). Below `minConfidence` anyway, so it is a numerical guard rather than a tuning knob — which is why it is a constant and not a config field.

*field, declared at [`include/shulib/localization/apriltag_corrector.hpp:189`](../../include/shulib/localization/apriltag_corrector.hpp#L189).*

<a id="apriltagcorrector-apriltagcorrector"></a>

### `AprilTagCorrector::AprilTagCorrector`

```cpp
AprilTagCorrector(hal::IClock& clock, hal::ITagSource& tags, hal::IImu& imu, const TagMap& map, const AprilTagCorrectorConfig& config = {}, const char* name = "tags")
```

`clock`, `tags`, `imu` and `map` are non-owning references that must outlive this corrector. `name` is the stable telemetry id reported by name() and stamped into AppliedCorrection::source.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:194`](../../include/shulib/localization/apriltag_corrector.hpp#L194).*

<a id="apriltagcorrector-droppedtags"></a>

### `AprilTagCorrector::droppedTags`

```cpp
[[nodiscard]] int droppedTags() const noexcept
```

Observations discarded because a frame carried more than kMaxTagsPerFrame tags. Kept by ARRIVAL ORDER, so a dropped tag may have been the best one available: a nonzero count means the best-sigma pick was made over an arbitrary prefix rather than the whole frame.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:224`](../../include/shulib/localization/apriltag_corrector.hpp#L224).*

<a id="apriltagcorrector-poll"></a>

### `AprilTagCorrector::poll`

```cpp
void poll()
```

Take one frame from the tag source. **Call this from a vision-rate task, NEVER from the control loop** (header note, tension T4): this is the method that allocates.  A poll that sees NOTHING is still information — "we looked, the camera is alive, there was no tag" — and is recorded as such, which is how the off-camera path stays distinguishable from a dead vision task.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:232`](../../include/shulib/localization/apriltag_corrector.hpp#L232).*

<a id="apriltagcorrector-propose"></a>

### `AprilTagCorrector::propose`

```cpp
[[nodiscard]] CorrectionProposal propose(const math::Pose2d& predicted, units::Time /*dt*/) override
```

One tick of the sequence in the header note. Never throws, never allocates; `dt` is unused because this corrector timestamps from the injected clock (E2's D5).

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:258`](../../include/shulib/localization/apriltag_corrector.hpp#L258).*

<a id="apriltagcorrector-name"></a>

### `AprilTagCorrector::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

The stable telemetry id given at construction ("tags" unless overridden). Read it as an IDENTITY, not as attribution: the Localizer stamps AppliedCorrection::source with the FIRST corrector in registration order that returned a VALID proposal that tick, while the complementary policy folds the sum of every accepted proposal — so with two correctors registered the name tells you who was asked first, not whose fix moved the estimate. It also carries this name on the other path: when nothing reached the policy, source names the corrector whose DECLINE the record is reporting. Exact with one corrector only. The pointer is stored, NOT copied, so the caller's string must outlive this corrector.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:462`](../../include/shulib/localization/apriltag_corrector.hpp#L462).*

<a id="apriltagcorrector-lastverdict"></a>

### `AprilTagCorrector::lastVerdict`

```cpp
[[nodiscard]] diag::GateReason lastVerdict() const noexcept
```

What this corrector decided on the most recent propose() call.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:467`](../../include/shulib/localization/apriltag_corrector.hpp#L467).*

<a id="apriltagcorrector-lasttagid"></a>

### `AprilTagCorrector::lastTagId`

```cpp
[[nodiscard]] int lastTagId() const noexcept
```

The id of the tag most recently PROPOSED from, or -1 if none ever was. Names WHICH tag the estimate is anchored to, which is the first question when a fix looks wrong.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:470`](../../include/shulib/localization/apriltag_corrector.hpp#L470).*

<a id="apriltagcorrector-pollcount"></a>

### `AprilTagCorrector::pollCount`

```cpp
[[nodiscard]] std::uint32_t pollCount() const noexcept
```

Frames taken from the tag source since construction. Zero means nobody is polling.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:472`](../../include/shulib/localization/apriltag_corrector.hpp#L472).*

<a id="apriltagcorrector-acceptedfixes"></a>

### `AprilTagCorrector::acceptedFixes`

```cpp
[[nodiscard]] std::uint32_t acceptedFixes() const noexcept
```

Valid proposals returned since construction (the Localizer screens them again, and the fusion policy may still gate one, so this is not a count of estimate moves). At most ONE per polled frame — a frame is folded once — so it can never exceed pollCount().

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:476`](../../include/shulib/localization/apriltag_corrector.hpp#L476).*

<a id="apriltagcorrector-noframeticks"></a>

### `AprilTagCorrector::noFrameTicks`

```cpp
[[nodiscard]] std::uint32_t noFrameTicks() const noexcept
```

Ticks before the very first poll — the "nobody wired the vision task" number.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:478`](../../include/shulib/localization/apriltag_corrector.hpp#L478).*

<a id="apriltagcorrector-staleframeticks"></a>

### `AprilTagCorrector::staleFrameTicks`

```cpp
[[nodiscard]] std::uint32_t staleFrameTicks() const noexcept
```

Ticks whose newest frame was older than maxObservationAge — the poller stopped.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:480`](../../include/shulib/localization/apriltag_corrector.hpp#L480).*

<a id="apriltagcorrector-staleticks"></a>

### `AprilTagCorrector::staleTicks`

```cpp
[[nodiscard]] std::uint32_t staleTicks() const noexcept
```

Ticks that re-read a frame already folded (the normal steady state at 20 Hz vs 100 Hz).

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:482`](../../include/shulib/localization/apriltag_corrector.hpp#L482).*

<a id="apriltagcorrector-notagticks"></a>

### `AprilTagCorrector::noTagTicks`

```cpp
[[nodiscard]] std::uint32_t noTagTicks() const noexcept
```

Fresh frames with no tag in view at all — the off-camera path.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:484`](../../include/shulib/localization/apriltag_corrector.hpp#L484).*

<a id="apriltagcorrector-unmappedrejects"></a>

### `AprilTagCorrector::unmappedRejects`

```cpp
[[nodiscard]] std::uint32_t unmappedRejects() const noexcept
```

Fresh frames whose every tag was absent from the map. A configuration error, counted separately because it is the one the team can actually fix.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:487`](../../include/shulib/localization/apriltag_corrector.hpp#L487).*

<a id="apriltagcorrector-rangerejects"></a>

### `AprilTagCorrector::rangeRejects`

```cpp
[[nodiscard]] std::uint32_t rangeRejects() const noexcept
```

Fresh frames whose every tag was outside the trusted range band.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:489`](../../include/shulib/localization/apriltag_corrector.hpp#L489).*

<a id="apriltagcorrector-qualityrejects"></a>

### `AprilTagCorrector::qualityRejects`

```cpp
[[nodiscard]] std::uint32_t qualityRejects() const noexcept
```

Fresh frames whose every tag was below the confidence floor (or non-finite).

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:491`](../../include/shulib/localization/apriltag_corrector.hpp#L491).*

<a id="apriltagcorrector-yawraterejects"></a>

### `AprilTagCorrector::yawRateRejects`

```cpp
[[nodiscard]] std::uint32_t yawRateRejects() const noexcept
```

Fresh frames declined because the robot was spinning too fast.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:493`](../../include/shulib/localization/apriltag_corrector.hpp#L493).*

<a id="apriltagcorrector-innovationrejects"></a>

### `AprilTagCorrector::innovationRejects`

```cpp
[[nodiscard]] std::uint32_t innovationRejects() const noexcept
```

Fresh fixes declined by the normalized-innovation gate.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:495`](../../include/shulib/localization/apriltag_corrector.hpp#L495).*

<a id="apriltagcorrector-travelsincefix"></a>

### `AprilTagCorrector::travelSinceFix`

```cpp
[[nodiscard]] units::Length travelSinceFix() const noexcept
```

Distance the prediction has travelled since this source last proposed — the anti-lockout input, exposed so a test can prove the widening is real rather than asserted.

*function, declared at [`include/shulib/localization/apriltag_corrector.hpp:498`](../../include/shulib/localization/apriltag_corrector.hpp#L498).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 95 lines, click to expand</summary>

```text

 AprilTagCorrector — the SECOND real corrector, and the FIRST source in the tree that can tell
 the estimator which way it is actually pointing (master plan §6/§8; WS5, chunk E3).

 E2's GpsCorrector bounded POSITION drift. Heading was still IMU-only, and heading is the half
 the team's `< 1°` end-of-run spec actually turns on — the master plan says so in as many
 words: "IMU-owned heading + absolute yaw correction (AprilTag/GPS) load-bearing, not
 optional". A tag observation is different in KIND from a GPS fix: `TagObservation::poseInRobot`
 is a relative POSE — position AND orientation — so against a tag whose field pose is known it
 yields an ABSOLUTE HEADING. That is what this class is for.

 It implements ICorrector behind the exact signature the seam has carried since M2. No caller
 changed; the Localizer does not know there are two kinds of corrector.

 ── THE TWO-METHOD SHAPE, AND WHY IT IS NOT NEGOTIABLE (chunk tension T4) ───────────────────
   poll()     — call at VISION cadence, OFF the control loop. Reads ITagSource::tags().
   propose()  — call every control tick. Reads the snapshot poll() left, plus the clock and the
                IMU live. It NEVER touches ITagSource — that is what keeps it allocation-free.

 `ITagSource::tags()` returns `std::vector<TagObservation>` BY VALUE and is FROZEN (F4) — it
 cannot be changed, and it heap-allocates on every call. hal/vision.hpp is explicit that vision
 runs OFF the 10 ms control path and the adapter polls it at a lower rate. A propose() that
 called tags() would therefore put a heap allocation inside the control loop, which A1's cost
 contract forbids — and calling it "only every fifth tick" does not fix that, it just makes the
 allocation intermittent, which is worse to diagnose. So the cadence is the CALLER's, explicitly,
 and propose() is allocation-free on every path. Pinned, not asserted:
 test/apriltag_corrector_cost_test.cpp counts global allocations across 20,000 propose() calls.

 THE FOOTGUN THAT CREATES, stated plainly: a corrector nobody polls proposes nothing, forever,
 and silently. Three things make that diagnosable rather than mysterious — `pollCount()` is
 exposed, a never-polled corrector declines with `RejectedNoFix` (so the blackbox says so every
 tick), and a corrector whose poller STOPPED declines with `RejectedObservationAge`, which is a
 different word from every other kind of silence.

 ── WHAT IT DOES, AND IN WHAT ORDER ────────────────────────────────────────────────────────
   1. record the predicted pose in a history ring (needed for latency, below);
   2. never polled?          → decline, RejectedNoFix
   3. snapshot too old?      → decline, RejectedObservationAge   (the poller stopped)
   4. snapshot already used? → decline, RejectedStaleFix         (the double-count guard)
   5. polled, saw nothing?   → decline, RejectedNoFix            (the off-camera path)
   6. spinning fast?         → decline, RejectedHighYawRate      (motion blur / rolling shutter)
   7. pick the single best tag against the map (below); no survivor → decline with the reason
      that eliminated them, by a documented priority;
   8. invert against the tag map → an absolute field POSE (x, y, AND heading);
   9. compensate for pipeline latency, in position AND in heading;
  10. normalized-innovation gate on position → decline, RejectedNormalizedInnovation;
  11. otherwise propose, with providesHeading = true.

 ── WHY ONE TAG AND NOT ALL OF THEM ────────────────────────────────────────────────────────
 ICorrector returns ONE proposal, so N visible tags must become one answer. This class picks
 the tag with the SMALLEST estimated sigma and uses only that one. Rejected alternative:
 average the N absolute poses. Averaging positions is easy, averaging headings needs a circular
 mean, and both need a weighting scheme that is exactly the thing an EKF derives and a
 complementary tier can only invent. Worse, averaging HIDES disagreement: two tags implying
 poses 8 inches apart average to a confident, wrong answer with no residual to show for it,
 where "trust the best one" at least leaves the second tag's disagreement visible as a future
 innovation. Multi-tag triangulation is E4's, where a covariance makes it principled.

 ── SIGMA, AND WHY HEADING DOES NOT GET ITS OWN ────────────────────────────────────────────
     sigma_meas = (baseStdDev + stdDevPerInch · range) / max(confidence, kMinConfidenceFloor)
     sigma_dr   = hypot(postFixStdDev, driftStdDevPerInch · travelSinceFix)
     confidence = sigma_dr² / (sigma_dr² + sigma_meas²)        — the scalar Kalman gain (E2 D1)
 Physically, PnP HEADING degrades faster with range than PnP POSITION does (the near-planar
 ambiguity — see hal/vision_conversion.hpp). Modelling that properly needs a second noise
 number on CorrectionProposal and a policy that uses it, which is E4's EKF. E3 handles it with
 the blunter instrument that does not require inventing a number: a TRUSTED RANGE BAND
 (A4: HA-73) outside which the observation is not used at all. Blunter, but honest about which
 of the two it is.

 ── LATENCY IS COMPENSATED IN HEADING TOO, NOT JUST POSITION ───────────────────────────────
 E2 compensates a GPS fix's ~50 ms of staleness by carrying it forward along the odometry. A
 tag fix needs the same treatment for POSITION and — new here — for HEADING, because a tag fix
 describes which way the robot was pointing ~80 ms ago (HA-71). At a brisk 180 deg/s that is
 14 degrees, fourteen times the entire heading error budget. So the history ring carries an
 UNWRAPPED cumulative heading alongside the position, and the tag-derived heading is advanced
 by the rotation observed since capture. (Unwrapped, because interpolating raw wrapped headings
 across the ±π seam produces a garbage rotation exactly once per revolution.)

 That rotation is read from the IMU, NOT from `predicted.heading()` — see the note at step (1).
 Using the predicted heading feeds the correction back into itself and the bias overshoots.

 ── WHAT IT DELIBERATELY DOES NOT DO ───────────────────────────────────────────────────────
 * NO PnP. The corners→pose reduction is `hal::tagCornersToRobotPose`, a free pure function
   called by R2's ADAPTER, and this file does not include it (chunk tension T3). The seam hands
   this class an already-reduced `poseInRobot`.
 * NO SNAP — position or heading. This class only ever PROPOSES. How far the estimate moves is
   the fusion policy's bounded, per-tick-clamped nudge (§13 #4), and for heading the Localizer's
   bias accumulator moves by at most `maxHeadingNudgeRate · dt` per tick. There is no code path
   here that writes a pose or a heading.
 * NO TAG MAP OF ITS OWN. Where the tags are is INPUT (tension T2, see tag_map.hpp). An empty
   map makes every tag decline with RejectedNoTagMapEntry, loudly, rather than guessing.

 Pure w.r.t. its injected handles (clock, tag source, imu, map) and PROS-free. propose() never
 throws and never allocates; poll() allocates exactly as much as ITagSource::tags() does, off
 the control path, by design.
```

</details>
