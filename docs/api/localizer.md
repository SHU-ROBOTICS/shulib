<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/localizer.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `localizer.hpp`

Localizer — the fused field-frame estimate.

This header declares **3** types (22 members).

Extracted from [`include/shulib/localization/localizer.hpp`](../../include/shulib/localization/localizer.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct LocalizerConfig`](#struct-localizerconfig)
  - [`maxDt`](#localizerconfig-maxdt)
  - [`minDt`](#localizerconfig-mindt)
  - [`driftHorizon`](#localizerconfig-drifthorizon)
  - [`qFloor`](#localizerconfig-qfloor)
  - [`bootSettleTime`](#localizerconfig-bootsettletime)
- [`class Localizer`](#class-localizer)
  - [`kMaxCorrectors`](#localizer-kmaxcorrectors)
  - [`Localizer`](#localizer-localizer)
  - [`update`](#localizer-update)
  - [`pose`](#localizer-pose)
  - [`twist`](#localizer-twist)
  - [`quality`](#localizer-quality)
  - [`isDeadReckoning`](#localizer-isdeadreckoning)
  - [`qualityClass`](#localizer-qualityclass)
  - [`distanceSinceCorrection`](#localizer-distancesincecorrection)
  - [`lastCorrection`](#localizer-lastcorrection)
  - [`lastOdomDeltaImplausible`](#localizer-lastodomdeltaimplausible)
  - [`headingBias`](#localizer-headingbias)
  - [`setPose`](#localizer-setpose)
  - [`enum class Localizer::Quality`](#enum-class-localizer-quality)
    - [`Uninitialized`](#localizer-quality-uninitialized)
    - [`DeadReckon`](#localizer-quality-deadreckon)
    - [`Corrected`](#localizer-quality-corrected)
    - [`Degraded`](#localizer-quality-degraded)

<a id="struct-localizerconfig"></a>

## `struct LocalizerConfig`

```cpp
struct LocalizerConfig
```

Tuning for the fused estimate: the dt window the twist finite-difference is trusted over, how fast the quality scalar decays while dead-reckoning, and how long the boot settle window holds the fold closed. The Localizer constructor range-checks maxDt, driftHorizon, qFloor and bootSettleTime (red-on-failure); `minDt` is NOT checked, and nothing checks `minDt <= maxDt`, so the dt BAND is the caller's to keep sane: a floor above the ceiling empties it and every tick then silently reports zero linear velocity and Degraded quality, while a floor <= 0 disables the velocity-spike guard minDt exists to be. The drift-rate numbers are invented guesses until a real drivetrain is measured.

*struct, declared at [`include/shulib/localization/localizer.hpp:119`](../../include/shulib/localization/localizer.hpp#L119).*

<a id="localizerconfig-maxdt"></a>

### `LocalizerConfig::maxDt`

```cpp
double maxDt = 0.1
```

Above this tick dt (s), the linear-velocity finite-difference is not trusted (first tick after construction/teleport, or a loop stall) → zero linear velocity for that tick + a flagged tick.

*field, declared at [`include/shulib/localization/localizer.hpp:122`](../../include/shulib/localization/localizer.hpp#L122).*

<a id="localizerconfig-mindt"></a>

### `LocalizerConfig::minDt`

```cpp
double minDt = 1e-4
```

Below this tick dt (s), the finite-difference is likewise not trusted (a near-zero interval would otherwise blow up into an unphysical velocity spike).

*field, declared at [`include/shulib/localization/localizer.hpp:125`](../../include/shulib/localization/localizer.hpp#L125).*

<a id="localizerconfig-drifthorizon"></a>

### `LocalizerConfig::driftHorizon`

```cpp
units::Length driftHorizon{12.0}
```

distanceSinceCorrection at which the quality scalar decays to qFloor (drift erodes trust as we dead-reckon farther — process noise scales with travel). Default ~ one foot — an INVENTED drift-rate guess until R4 measures real dead-reckon drift (A4 register HA-36).

*field, declared at [`include/shulib/localization/localizer.hpp:129`](../../include/shulib/localization/localizer.hpp#L129).*

<a id="localizerconfig-qfloor"></a>

### `LocalizerConfig::qFloor`

```cpp
double qFloor = 0.2
```

Quality floor while dead-reckoning far from a fix, in [0,1).

*field, declared at [`include/shulib/localization/localizer.hpp:131`](../../include/shulib/localization/localizer.hpp#L131).*

<a id="localizerconfig-bootsettletime"></a>

### `LocalizerConfig::bootSettleTime`

```cpp
double bootSettleTime = 0.1
```

How long after a WITNESSED not-ready→ready transition the fold stays closed while the delayed sensor data path flushes its boot-boundary garbage (the settle window — header note). Applies ONLY when a not-ready phase was observed; a ready-from-construction boot takes no hold. Must cover the worst sensor data-path latency; 0.1 s clears the ~50 ms GPS-class delay with margin (adequacy vs. REAL latencies: A4 register HA-35, R4 measures).

*field, declared at [`include/shulib/localization/localizer.hpp:137`](../../include/shulib/localization/localizer.hpp#L137).*

<a id="class-localizer"></a>

## `class Localizer`

```cpp
class Localizer final : public IPoseSource
```

The fused field-frame estimate, and the IPoseSource every consumer above it reads: a deterministic five-step tick over an injected clock, IMU, PilonsOdometry and a non-owning list of correctors. Position is a PERSISTENT accumulator advanced by odometry DELTAS and nudged — never snapped — toward corrector proposals; heading is composed from the IMU as the LAST write of every tick, so nothing below can ASSIGN a heading, only move a bounded, persistent bias. It owns no loop and raises no faults: the caller calls update() once per control tick, and pose()/twist()/quality() then describe THAT tick until the next one.

*class, declared at [`include/shulib/localization/localizer.hpp:147`](../../include/shulib/localization/localizer.hpp#L147).*

<a id="localizer-kmaxcorrectors"></a>

### `Localizer::kMaxCorrectors`

```cpp
static constexpr std::size_t kMaxCorrectors = 4
```

At most this many correctors (GPS + AI-Vision tag + Pi tag + LIDAR today) — the valid-proposal buffer is fixed-capacity so the hot path never heap-allocates.

*field, declared at [`include/shulib/localization/localizer.hpp:173`](../../include/shulib/localization/localizer.hpp#L173).*

<a id="localizer-localizer"></a>

### `Localizer::Localizer`

```cpp
Localizer(hal::IClock& clock, hal::IImu& imu, PilonsOdometry& odom, IFusionPolicy& fusion, std::span<ICorrector* const> correctors = {}, const LocalizerConfig& config = {})
```

`correctors` is a NON-OWNING view: the backing array (and the correctors it points to) must outlive the Localizer. Empty at M2 (dead-reckon). All references are validated non-null.

*function, declared at [`include/shulib/localization/localizer.hpp:177`](../../include/shulib/localization/localizer.hpp#L177).*

<a id="localizer-update"></a>

### `Localizer::update`

```cpp
void update()
```

One fused tick (the five steps above).

*function, declared at [`include/shulib/localization/localizer.hpp:209`](../../include/shulib/localization/localizer.hpp#L209).*

<a id="localizer-pose"></a>

### `Localizer::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const noexcept override
```

The fused field-frame pose as of the last update(): x/y in INCHES from the persistent accumulator, heading in RADIANS as `imu.heading() + headingBias()`. While the IMU is still booting or settling the POSITION is frozen at its seed value (the fold is closed) while the heading keeps tracking the raw IMU, calibration garbage included — so check qualityClass() before believing this, rather than reading a plausible-looking pose that does not exist yet.

*function, declared at [`include/shulib/localization/localizer.hpp:415`](../../include/shulib/localization/localizer.hpp#L415).*

<a id="localizer-twist"></a>

### `Localizer::twist`

```cpp
[[nodiscard]] math::Twist2d twist() const noexcept override
```

Field-frame velocity: vx/vy in in/s, finite-differenced from the FUSED pose, and ω in rad/s taken straight from the IMU (0 when the IMU reads non-finite). A tick whose dt lands outside [minDt, maxDt] — a loop stall, or the tick after a teleport — reports ZERO linear velocity rather than a spike; the first tick, and any dt <= 0, keeps the previous linear velocity and refreshes only ω.

*function, declared at [`include/shulib/localization/localizer.hpp:421`](../../include/shulib/localization/localizer.hpp#L421).*

<a id="localizer-quality"></a>

### `Localizer::quality`

```cpp
[[nodiscard]] double quality() const noexcept override
```

Graded trust in [0,1], kept consistent with qualityClass(): EXACTLY 0 whenever the IMU has no heading authority (booting, settling, or lost mid-run), otherwise a drift term decaying linearly to qFloor over driftHorizon of dead-reckoned travel, halved for an unhealthy dt and halved again for an implausible odometry delta. An applied fix clears the drift term in PROPORTION to that fix's confidence, so a microscopic fix cannot spring this to 1.0.

*function, declared at [`include/shulib/localization/localizer.hpp:427`](../../include/shulib/localization/localizer.hpp#L427).*

<a id="localizer-isdeadreckoning"></a>

### `Localizer::isDeadReckoning`

```cpp
[[nodiscard]] bool isDeadReckoning() const noexcept override
```

True when no corrector proposal was applied on the most recent update(). A per-TICK answer, not a summary: it returns to true the moment a source goes quiet, and says nothing about how far the robot has dead-reckoned since (that is distanceSinceCorrection()). True before the first update().

*function, declared at [`include/shulib/localization/localizer.hpp:432`](../../include/shulib/localization/localizer.hpp#L432).*

<a id="localizer-qualityclass"></a>

### `Localizer::qualityClass`

```cpp
[[nodiscard]] Quality qualityClass() const noexcept
```

The categorical health a motion or skills gate branches on, carrying the distinction the [0,1] scalar cannot: Uninitialized means there is no live estimate YET and is what the motion layer's wait-for-live gate blocks on, while Degraded means an estimate exists and is decaying. Keeping those two apart is deliberate — a robot that had a fix and lost heading authority needs different recovery from one that is still booting.

*function, declared at [`include/shulib/localization/localizer.hpp:440`](../../include/shulib/localization/localizer.hpp#L440).*

<a id="localizer-distancesincecorrection"></a>

### `Localizer::distanceSinceCorrection`

```cpp
[[nodiscard]] units::Length distanceSinceCorrection() const noexcept
```

Inches of odometry travel accumulated since a fix was last applied — the input the quality decay is computed from. An applied fix does not zero it but SCALES it by (1 − the fix's confidence), so a weak fix barely dents it; setPose() clears it outright, and travel made while the boot fold is closed never enters it.

*function, declared at [`include/shulib/localization/localizer.hpp:445`](../../include/shulib/localization/localizer.hpp#L445).*

<a id="localizer-lastcorrection"></a>

### `Localizer::lastCorrection`

```cpp
[[nodiscard]] const AppliedCorrection& lastCorrection() const noexcept
```

The last tick's applied correction AND the gate's account of why (`audit`, added at E1) — the values a record producer stamps into the §18.2 gating slots.

*function, declared at [`include/shulib/localization/localizer.hpp:448`](../../include/shulib/localization/localizer.hpp#L448).*

<a id="localizer-lastodomdeltaimplausible"></a>

### `Localizer::lastOdomDeltaImplausible`

```cpp
[[nodiscard]] bool lastOdomDeltaImplausible() const noexcept
```

Forwarding accessor for PilonsOdometry::lastDeltaImplausible() — added at C1 (additive) so the motion loop can feed HealthMonitor's odomImplausible observable without holding the odometry itself. Raising stays POLICY: this only EXPOSES the flag; the Localizer still never raises faults (D3 at A3).

*function, declared at [`include/shulib/localization/localizer.hpp:453`](../../include/shulib/localization/localizer.hpp#L453).*

<a id="localizer-headingbias"></a>

### `Localizer::headingBias`

```cpp
[[nodiscard]] units::AngleDim headingBias() const noexcept
```

The learned heading bias, in radians: how far the published heading sits from the raw IMU reading (E3). Exposed so a test can prove the correction ACCUMULATES rather than evaporating each tick — the M2 red team's failure mode — and so telemetry can say how far the IMU has been found to have drifted. Zero on any tree with no heading-providing corrector, exactly.

*function, declared at [`include/shulib/localization/localizer.hpp:462`](../../include/shulib/localization/localizer.hpp#L462).*

<a id="localizer-setpose"></a>

### `Localizer::setPose`

```cpp
void setPose(const math::Pose2d& p)
```

Teleport the POSITION (x, y); heading stays IMU-owned. Forwards to PilonsOdometry::setPose so the predictor and the fused belief never diverge, and re-baselines twist + dt so the teleport injects no phantom velocity next tick.  E3: the learned heading bias is KEPT, deliberately. A teleport says where the robot IS, not which way the IMU is wrong; discarding a bias that took a second of tag sightings to learn, every time a routine re-seeds its position, would throw away the correction at exactly the moments a routine cares most. `p.heading()` is still ignored, as it always was.

*function, declared at [`include/shulib/localization/localizer.hpp:474`](../../include/shulib/localization/localizer.hpp#L474).*

<a id="enum-class-localizer-quality"></a>

## `enum class Localizer::Quality`

```cpp
enum class Quality
```

Categorical health for motion/skills gating (distinct from the [0,1] scalar). The order below is declaration order, NOT a ranking — `Degraded` is worse than `DeadReckon` despite sorting after it, so compare by enumerator and never by value.

*enum class, declared at [`include/shulib/localization/localizer.hpp:152`](../../include/shulib/localization/localizer.hpp#L152).*

<a id="localizer-quality-uninitialized"></a>

### `Localizer::Quality::Uninitialized`

```cpp
Uninitialized
```

No live estimate yet: update() has never run, or the boot settle window is still open. Distinct from Degraded on purpose — a consumer can tell "not started" from "started and lost it".

*enumerator, declared at [`include/shulib/localization/localizer.hpp:156`](../../include/shulib/localization/localizer.hpp#L156).*

<a id="localizer-quality-deadreckon"></a>

### `Localizer::Quality::DeadReckon`

```cpp
DeadReckon
```

Running on odometry alone, within the configured drift horizon. Healthy: no corrector has proposed recently, and the estimate has not yet dead-reckoned far enough for that to matter.

*enumerator, declared at [`include/shulib/localization/localizer.hpp:160`](../../include/shulib/localization/localizer.hpp#L160).*

<a id="localizer-quality-corrected"></a>

### `Localizer::Quality::Corrected`

```cpp
Corrected
```

The best state: a corrector proposal was folded in this tick and every health check passed. This is the only class that means an absolute reference is live.

*enumerator, declared at [`include/shulib/localization/localizer.hpp:163`](../../include/shulib/localization/localizer.hpp#L163).*

<a id="localizer-quality-degraded"></a>

### `Localizer::Quality::Degraded`

```cpp
Degraded
```

Trust the pose less. Reached four different ways, all of which mean the same thing to a caller: the IMU was ready and stopped being ready, the odometry reported an implausible delta, the tick's dt was outside the trusted band, or dead reckoning has run past `driftHorizon`.

*enumerator, declared at [`include/shulib/localization/localizer.hpp:168`](../../include/shulib/localization/localizer.hpp#L168).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 87 lines, click to expand</summary>

```text

 Localizer — the fused field-frame estimate (master plan §5/§6/§8; WS5). It is a thin, deterministic
 orchestrator over three already-tested pieces: PilonsOdometry (high-rate prediction), the IMU
 (heading authority), and a list of correctors (absolute fixes) behind an IFusionPolicy. It owns
 the four jobs the pieces below cannot, in a fixed five-step update():
   1. dt — source it from the injected IClock and turn the per-tick position change into a Twist2d.
   2. predict — advance PilonsOdometry; its pose is the dead-reckon prediction.
   3. gather — ask each corrector for an absolute proposal and keep the VALID ones.
   4. fuse — fold them in as an innovation-bounded, per-tick-clamped GATED NUDGE, via the
      IFusionPolicy.
   5. heading + publish — compose the fused heading from the IMU as the LAST write of the tick, so no
      corrector or policy can ever assign the robot a heading (decision #4). Since E3 the
      composition is `imu.heading() + headingBias_`, where the bias moves by at most a bounded
      nudge per tick — see the heading-bias note below.
      …then recompute the quality scalar + categorical flags from observable inputs.

 The numbering above matches the STEP labels in update() exactly. It used to be off by one
 from step 3 onward — the banner's "fuse" was the code's STEP 4, the banner's "heading" was
 STEP 5, and the banner's separate step 5 had no label at all — so a reader arriving from the
 generated page and jumping to the source landed on the wrong step every time. The banner was
 also inconsistent with itself downstream, where the heading-bias note cites "STEP 4" using
 the old numbering for what the code calls STEP 5.

 ── THE HEADING BIAS (added at E3, the absolute-yaw path M2 reserved) ──
 Until E3 nothing in the tree could tell the estimator its heading was wrong, so STEP 5 stamped
 the raw IMU reading and heading was IMU-owned in the strongest possible sense. That was correct
 while it lasted and is NOT what the accuracy spec needs: the master plan records that a raw V5
 IMU drifts ≈1°/min, so dead-reckoned heading alone will not hold the team's `< 1°` requirement
 across a 60-second run, and makes "IMU-owned heading + absolute yaw correction" load-bearing.
 AprilTagCorrector supplies the absolute heading; this is where it lands.

 The published heading is `imu.heading() + headingBias_`. The IMU remains the SOLE source of
 heading CHANGE — every rotation the robot makes still enters through imu.heading(), tick for
 tick, and PilonsOdometry is untouched — while the bias is a slowly-learned constant offset that
 moves by at most `maxHeadingNudgeRate · dt` per tick (ComplementaryFusion). There is no code
 path, here or anywhere below, that ASSIGNS a heading: the policy can only return an increment.
 That is what makes never-snap (§13 #4) structural for yaw rather than a promise.

 WHY THE BIAS MUST PERSIST, which is the whole design. The Localizer re-reads the IMU every tick,
 so a heading correction that only decorated THIS tick's published heading would be discarded on
 the next one — individually sane nudges, collectively useless. That is verbatim the M2 red
 team's "corrections not accumulating" failure mode. The accumulator is exactly the structure
 STEP 2 already uses for position (fusedX_ persists and is advanced by odom DELTAS, never reset
 to absolute odom). It is deliberately NOT capped: the nudge is always toward an absolute
 measurement, so the loop is closed and cannot run away, whereas a cap would silently lock out
 recovery from precisely the large real drift that makes the correction worth having — the same
 gate-lockout failure E2's D2 identified for position.

 AND THE CONSEQUENCE FOR POSITION. PilonsOdometry rotates its per-tick field delta by the RAW
 IMU heading. Left alone, a learned bias `b` would fix the reported heading while the position
 prediction kept accumulating a cross-track error of roughly `b × distance` — which is most of
 what heading drift actually costs over a run. So STEP 2 rotates the odometry delta by `b` before
 folding it. At `b == 0` this is skipped by an explicit early-out, so a tree with no
 heading-providing corrector is BIT-IDENTICAL to before E3 by construction, not by a
 floating-point argument.

 Swapping the complementary filter for an EKF later is a one-argument change (IFusionPolicy); the
 IPoseSource read seam and the ICorrector write seam never move. At M2 the corrector list is empty
 or holds only NullCorrector, so the default-tested path is dead-reckoning (odom + IMU).

 ── The IMU-readiness boot guard (added at A3, after the hostile fakes drew blood) ──
 A calibrating IMU emits GARBAGE THAT MOVES, and the odometry offset correction converts garbage
 heading swings into phantom translation (Δθ·offset per tick) — observed at A3: a robot sitting
 STILL through a 2 s calibration window ended 10.8 inches from where it started, permanently
 (no corrector at M2 can heal it), while quality honestly said 0. The flag was honest; the pose
 was not. So update() now distinguishes three IMU states:
   * NEVER been ready (boot): odometry still consumes its wheel deltas tick-by-tick (so the
     transition tick sees one tick's travel, not the whole boot's), but the fused position folds
     NOTHING — no odom deltas, no corrector proposals. Quality: Uninitialized, 0.
   * THE SETTLE WINDOW (the second A3 finding, caught only by the COMPOSED hostile model):
     isReady() is a status flag; the heading STREAM is a data path with its own latency. When
     calibration garbage and sensor latency are both live, ready flips true while the stream is
     still serving delayed garbage — observed: one post-transition fold differenced against a
     delayed-garbage prevHeading leaked 3.65 in. So after a WITNESSED not-ready phase, the fold
     stays closed for `bootSettleTime` past the first ready tick (quality stays Uninitialized —
     the estimate is not live yet). A boot that was ready from the very first update() has no
     boundary in its stream and takes no settle hold, so the normal path is byte-identical to
     before. Consequence, stated as the consumer contract: motion commanded before qualityClass
     leaves Uninitialized is unaccounted — C1's loop waits for a live estimate (real autons wait
     out calibration anyway).
   * ready: normal operation.
   * WAS ready, lost mid-run (dropout): deltas KEEP folding — the encoders are still good and a
     stale-heading estimate beats a frozen one — but quality reports Degraded (NOT Uninitialized:
     a robot that had a fix and lost its heading authority must be distinguishable from one still
     booting, or a skills gate applies the wrong recovery) with the scalar pinned to 0.
 PilonsOdometry itself stays readiness-blind on purpose — its header assigns recovery policy to
 this layer, and a second gate below would hide this one's absence from the tests.
```

</details>
