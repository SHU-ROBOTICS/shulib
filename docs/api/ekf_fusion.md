<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/ekf_fusion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `ekf_fusion.hpp`

EkfFusion — the M3 fusion policy: a 5-state SE(2) extended Kalman filter behind the SAME `IFusionPolicy` seam `ComplementaryFusion` has occupied since M2.

This header declares **2** types (40 members).

Extracted from [`include/shulib/localization/ekf_fusion.hpp`](../../include/shulib/localization/ekf_fusion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct EkfFusionConfig`](#struct-ekffusionconfig)
  - [`posNoisePerInch`](#ekffusionconfig-posnoiseperinch)
  - [`posNoiseRate`](#ekffusionconfig-posnoiserate)
  - [`headingNoisePerRad`](#ekffusionconfig-headingnoiseperrad)
  - [`headingDriftRate`](#ekffusionconfig-headingdriftrate)
  - [`velNoise`](#ekffusionconfig-velnoise)
  - [`odomStdDev`](#ekffusionconfig-odomstddev)
  - [`odomStdDevPerInch`](#ekffusionconfig-odomstddevperinch)
  - [`gateSigma`](#ekffusionconfig-gatesigma)
  - [`headingStdDev`](#ekffusionconfig-headingstddev)
  - [`initialPosStdDev`](#ekffusionconfig-initialposstddev)
  - [`initialHeadingStdDev`](#ekffusionconfig-initialheadingstddev)
  - [`initialVelStdDev`](#ekffusionconfig-initialvelstddev)
  - [`maxNudgeRate`](#ekffusionconfig-maxnudgerate)
  - [`maxHeadingNudgeRate`](#ekffusionconfig-maxheadingnudgerate)
  - [`reinitRejectCount`](#ekffusionconfig-reinitrejectcount)
  - [`reinitInnovation`](#ekffusionconfig-reinitinnovation)
  - [`reinitCooldown`](#ekffusionconfig-reinitcooldown)
  - [`maxDt`](#ekffusionconfig-maxdt)
- [`class EkfFusion`](#class-ekffusion)
  - [`kN`](#ekffusion-kn)
  - [`kPx`](#ekffusion-kpx)
  - [`kPy`](#ekffusion-kpy)
  - [`kTh`](#ekffusion-kth)
  - [`kVx`](#ekffusion-kvx)
  - [`kVy`](#ekffusion-kvy)
  - [`EkfFusion`](#ekffusion-ekffusion)
  - [`fuse`](#ekffusion-fuse)
  - [`positionCovarianceTrace`](#ekffusion-positioncovariancetrace)
  - [`covariance`](#ekffusion-covariance)
  - [`state`](#ekffusion-state)
  - [`velocityX`](#ekffusion-velocityx)
  - [`velocityY`](#ekffusion-velocityy)
  - [`reinitCount`](#ekffusion-reinitcount)
  - [`everReinit`](#ekffusion-everreinit)
  - [`consecutiveRejects`](#ekffusion-consecutiverejects)
  - [`resyncCount`](#ekffusion-resynccount)
  - [`numericGuardTrips`](#ekffusion-numericguardtrips)
  - [`acceptedFixes`](#ekffusion-acceptedfixes)
  - [`rejectedFixes`](#ekffusion-rejectedfixes)
  - [`lastCorrectionMagnitude`](#ekffusion-lastcorrectionmagnitude)
  - [`lastHeadingCorrectionMagnitude`](#ekffusion-lastheadingcorrectionmagnitude)

<a id="struct-ekffusionconfig"></a>

## `struct EkfFusionConfig`

```cpp
struct EkfFusionConfig
```

Tuning for `EkfFusion`. Every value is INVENTED and registered in the A4 hardware-assumptions register; R4 replaces them with measurements. The defaults are deliberately conservative (wide priors, a modest gate) so the filter's failure mode is "slow to trust" rather than "confidently wrong".

*struct, declared at [`include/shulib/localization/ekf_fusion.hpp:249`](../../include/shulib/localization/ekf_fusion.hpp#L249).*

<a id="ekffusionconfig-posnoiseperinch"></a>

### `EkfFusionConfig::posNoisePerInch`

```cpp
double posNoisePerInch = 0.02
```

1σ position error added per inch travelled (2% of travel). This is the term that makes the gate widen after a long blind stretch, which is what stops the E2/D2 gate lockout. PROVISIONAL (A4: HA-83).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:254`](../../include/shulib/localization/ekf_fusion.hpp#L254).*

<a id="ekffusionconfig-posnoiserate"></a>

### `EkfFusionConfig::posNoiseRate`

```cpp
units::Velocity posNoiseRate{0.5}
```

1σ position error added per second even when standing still — the floor that keeps `P` strictly positive-definite on a stationary tick. PROVISIONAL (A4: HA-83).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:257`](../../include/shulib/localization/ekf_fusion.hpp#L257).*

<a id="ekffusionconfig-headingnoiseperrad"></a>

### `EkfFusionConfig::headingNoisePerRad`

```cpp
double headingNoisePerRad = 0.01
```

1σ heading error added per radian actually rotated (1% of the rotation) — scale-factor error in the gyro. PROVISIONAL (A4: HA-84).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:260`](../../include/shulib/localization/ekf_fusion.hpp#L260).*

<a id="ekffusionconfig-headingdriftrate"></a>

### `EkfFusionConfig::headingDriftRate`

```cpp
units::AngularVelocity headingDriftRate{(1.0 / 60.0) * math::Angle::kPi / 180.0}
```

1σ heading error added per second at rest: HA-20's ≈1°/min of raw V5 IMU drift, which is the assumption the whole heading-correction story rests on. PROVISIONAL (A4: HA-84).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:263`](../../include/shulib/localization/ekf_fusion.hpp#L263).*

<a id="ekffusionconfig-velnoise"></a>

### `EkfFusionConfig::velNoise`

```cpp
units::Acceleration velNoise{200.0}
```

How much body velocity the drivetrain can gain or lose in one second — the process noise on the velocity states, i.e. how far the constant-velocity model is allowed to be wrong. 200 in/s² is roughly a hard VEX drive launch. PROVISIONAL (A4: HA-85).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:267`](../../include/shulib/localization/ekf_fusion.hpp#L267).*

<a id="ekffusionconfig-odomstddev"></a>

### `EkfFusionConfig::odomStdDev`

```cpp
units::Length odomStdDev{0.01}
```

1σ error on ONE TICK's odometry displacement, independent of distance — encoder quantization and tracking-wheel jitter. PROVISIONAL (A4: HA-86).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:272`](../../include/shulib/localization/ekf_fusion.hpp#L272).*

<a id="ekffusionconfig-odomstddevperinch"></a>

### `EkfFusionConfig::odomStdDevPerInch`

```cpp
double odomStdDevPerInch = 0.02
```

…plus this fraction of the tick's travel — slip, which scales with distance. PROVISIONAL (A4: HA-86).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:275`](../../include/shulib/localization/ekf_fusion.hpp#L275).*

<a id="ekffusionconfig-gatesigma"></a>

### `EkfFusionConfig::gateSigma`

```cpp
double gateSigma = 3.0
```

Reject a fix whose Mahalanobis distance exceeds this. 3.0 on a 2-degree-of-freedom position innovation is a ≈1.1% false-reject rate if the noise model is right. PROVISIONAL (A4: HA-87).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:281`](../../include/shulib/localization/ekf_fusion.hpp#L281).*

<a id="ekffusionconfig-headingstddev"></a>

### `EkfFusionConfig::headingStdDev`

```cpp
units::AngleDim headingStdDev{2.0 * math::Angle::kPi / 180.0}
```

1σ on an absolute heading measurement, flat: `CorrectionProposal` carries no heading σ, and inventing a per-proposal relationship would be worse than one honest constant. PROVISIONAL (A4: HA-88).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:285`](../../include/shulib/localization/ekf_fusion.hpp#L285).*

<a id="ekffusionconfig-initialposstddev"></a>

### `EkfFusionConfig::initialPosStdDev`

```cpp
units::Length initialPosStdDev{24.0}
```

"I could be anywhere within a tile." PROVISIONAL (A4: HA-89).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:289`](../../include/shulib/localization/ekf_fusion.hpp#L289).*

<a id="ekffusionconfig-initialheadingstddev"></a>

### `EkfFusionConfig::initialHeadingStdDev`

```cpp
units::AngleDim initialHeadingStdDev{30.0 * math::Angle::kPi / 180.0}
```

PROVISIONAL (A4: HA-89).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:291`](../../include/shulib/localization/ekf_fusion.hpp#L291).*

<a id="ekffusionconfig-initialvelstddev"></a>

### `EkfFusionConfig::initialVelStdDev`

```cpp
units::Velocity initialVelStdDev{24.0}
```

PROVISIONAL (A4: HA-89).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:293`](../../include/shulib/localization/ekf_fusion.hpp#L293).*

<a id="ekffusionconfig-maxnudgerate"></a>

### `EkfFusionConfig::maxNudgeRate`

```cpp
units::Velocity maxNudgeRate{12.0}
```

Max position correction per tick, as a RATE, so the bound is loop-rate independent. Matches `ComplementaryFusionConfig::maxNudgeRate` on purpose: never-snap must not change meaning when the tier is swapped.

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:299`](../../include/shulib/localization/ekf_fusion.hpp#L299).*

<a id="ekffusionconfig-maxheadingnudgerate"></a>

### `EkfFusionConfig::maxHeadingNudgeRate`

```cpp
units::AngularVelocity maxHeadingNudgeRate{10.0 * math::Angle::kPi / 180.0}
```

Max heading-bias change per tick, as a rate. Matches `maxHeadingNudgeRate` (A4: HA-82).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:301`](../../include/shulib/localization/ekf_fusion.hpp#L301).*

<a id="ekffusionconfig-reinitrejectcount"></a>

### `EkfFusionConfig::reinitRejectCount`

```cpp
int reinitRejectCount = 50
```

How many CONSECUTIVE gate rejections before the filter is willing to admit it is lost. At a ~20 Hz fix cadence this is ≈2.5 seconds of a sensor insisting the estimate is wrong. PROVISIONAL (A4: HA-90).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:307`](../../include/shulib/localization/ekf_fusion.hpp#L307).*

<a id="ekffusionconfig-reinitinnovation"></a>

### `EkfFusionConfig::reinitInnovation`

```cpp
units::Length reinitInnovation{6.0}
```

…and the mean rejected innovation over that run must exceed this, so a burst of borderline rejections while the filter is very confident cannot trigger it. PROVISIONAL (A4: HA-90).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:311`](../../include/shulib/localization/ekf_fusion.hpp#L311).*

<a id="ekffusionconfig-reinitcooldown"></a>

### `EkfFusionConfig::reinitCooldown`

```cpp
units::Time reinitCooldown{5.0}
```

Minimum time between re-inits — the rate limit. PROVISIONAL (A4: HA-91).

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:313`](../../include/shulib/localization/ekf_fusion.hpp#L313).*

<a id="ekffusionconfig-maxdt"></a>

### `EkfFusionConfig::maxDt`

```cpp
double maxDt = 0.1
```

Above this tick dt, the interval is not a usable prediction step (a loop stall, or the dt==0 tick the Localizer produces after construction and after `setPose`). The filter re-bases on the handed prediction instead of integrating garbage. Mirrors `LocalizerConfig::maxDt`; kept here because a policy cannot see the Localizer's config.

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:319`](../../include/shulib/localization/ekf_fusion.hpp#L319).*

<a id="class-ekffusion"></a>

## `class EkfFusion`

```cpp
class EkfFusion final : public IFusionPolicy
```

A 5-state SE(2) extended Kalman filter implementing `IFusionPolicy`. See the file header for the design and for the T1/T2/T4/T5 rulings.  STATEFUL, unlike `ComplementaryFusion`. `IFusionPolicy::fuse` never promised statelessness — an EKF cannot be stateless — but nothing said so either, so it is said here: ONE instance belongs to ONE Localizer, is mutated on the control task only, and must outlive it.

*class, declared at [`include/shulib/localization/ekf_fusion.hpp:328`](../../include/shulib/localization/ekf_fusion.hpp#L328).*

<a id="ekffusion-kn"></a>

### `EkfFusion::kN`

```cpp
static constexpr std::size_t kN = 5
```

State dimension. Indices are named below so no bare 0..4 appears in the algebra.

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:331`](../../include/shulib/localization/ekf_fusion.hpp#L331).*

<a id="ekffusion-kpx"></a>

### `EkfFusion::kPx`

```cpp
static constexpr std::size_t kPx = 0
```

field-frame x position, inches

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:332`](../../include/shulib/localization/ekf_fusion.hpp#L332).*

<a id="ekffusion-kpy"></a>

### `EkfFusion::kPy`

```cpp
static constexpr std::size_t kPy = 1
```

field-frame y position, inches

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:333`](../../include/shulib/localization/ekf_fusion.hpp#L333).*

<a id="ekffusion-kth"></a>

### `EkfFusion::kTh`

```cpp
static constexpr std::size_t kTh = 2
```

Heading θ, radians. Re-based to the IMU's answer at the top of every tick rather than integrated here: what this filter estimates is the ERROR in that heading, and it leaves as a bounded increment. There is no rival heading in the state.

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:337`](../../include/shulib/localization/ekf_fusion.hpp#L337).*

<a id="ekffusion-kvx"></a>

### `EkfFusion::kVx`

```cpp
static constexpr std::size_t kVx = 3
```

BODY-frame forward velocity, in/s

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:338`](../../include/shulib/localization/ekf_fusion.hpp#L338).*

<a id="ekffusion-kvy"></a>

### `EkfFusion::kVy`

```cpp
static constexpr std::size_t kVy = 4
```

BODY-frame left velocity, in/s

*field, declared at [`include/shulib/localization/ekf_fusion.hpp:339`](../../include/shulib/localization/ekf_fusion.hpp#L339).*

<a id="ekffusion-ekffusion"></a>

### `EkfFusion::EkfFusion`

```cpp
explicit EkfFusion(const EkfFusionConfig& config = {})
```

Validates every tuning value — each has its own precondition message — and COPIES the config, so mutating the caller's struct afterward changes nothing here. ALL preconditions live in this constructor deliberately: `fuse()` then has none left to raise, which is what lets it be non-throwing on the control path.  Construction does NOT initialize the filter. The first `fuse()` adopts the pose it is handed as the prior mean and the configured initial std devs as the prior covariance, so an EkfFusion never has to be told where the robot starts.  The default config is usable and deliberately conservative — wide priors, a modest gate, so the failure mode is "slow to trust" rather than "confidently wrong" — but every number in it is a guess until the hardware is measured.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:353`](../../include/shulib/localization/ekf_fusion.hpp#L353).*

<a id="ekffusion-fuse"></a>

### `EkfFusion::fuse`

```cpp
[[nodiscard]] FusionResult fuse(const math::Pose2d& predicted, std::span<const CorrectionProposal> valid, units::Time dt) override
```

One fusion tick. The file header walks the five steps; the CONTRACT is here.  `predicted` is the Localizer's already-INTEGRATED dead-reckoned pose (field frame, inches and radians), never a raw control input — and it must be the pose built on THIS policy's own previous answer, because the tick's odometry increment is recovered as `predicted.position` minus the position last returned. `valid` holds only proposals the Localizer has already screened, folded most-trusted (smallest `positionStdDev`) first. `dt` is the tick duration in seconds.  STATEFUL. It advances the state, the covariance and every counter, so calling it twice with identical arguments does not give the same answer twice, and a skipped tick loses the increment that tick carried. One instance belongs to one Localizer, on one task.  Returns the corrected field position, a bounded heading INCREMENT (never an absolute heading — the Localizer folds it into a persistent bias), and the gate audit. It never allocates and never throws: every runtime pathology is screened and counted instead.  Degenerate ticks, all of which apply no correction: the first call adopts `predicted` as the prior; `dt <= 0` (startup, or the tick after a `setPose` teleport) and `dt > maxDt` (a loop stall) re-base onto `predicted` and widen the covariance, counted in `resyncCount()`; a non-finite input returns `predicted` untouched, counted in `numericGuardTrips()`.  With NO proposals the answer is not bit-identical to `predicted` the way the complementary tier's is — it differs by one tick of velocity filtering, bounded by a fraction of one tick's travel and measured to be non-cumulative.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:410`](../../include/shulib/localization/ekf_fusion.hpp#L410).*

<a id="ekffusion-positioncovariancetrace"></a>

### `EkfFusion::positionCovarianceTrace`

```cpp
[[nodiscard]] double positionCovarianceTrace() const noexcept
```

`P[px][px] + P[py][py]`, square inches — the POSITION block only (header, T5). A 1σ radius is `sqrt(trace / 2)`.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:486`](../../include/shulib/localization/ekf_fusion.hpp#L486).*

<a id="ekffusion-covariance"></a>

### `EkfFusion::covariance`

```cpp
[[nodiscard]] double covariance(std::size_t i, std::size_t j) const
```

One covariance entry, for the invariant tests (symmetry, positive-definiteness). Both indices must be < kN. BOUNDS-CHECKED and therefore no longer noexcept: these are public, and the documented contract was only a naming convention ("indexed by the kPx…kVy constants"), not a guard — nothing stopped covariance(9, 0) from reading past a std::array<double, 25>. Every other public indexing accessor in the tree checks (wheel_speeds.hpp is the house pattern); these two did not, and "observability only, never on the control path" does not make out-of-range reads defined.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:496`](../../include/shulib/localization/ekf_fusion.hpp#L496).*

<a id="ekffusion-state"></a>

### `EkfFusion::state`

```cpp
[[nodiscard]] double state(std::size_t i) const
```

One state entry, indexed by the `kPx`…`kVy` constants; the index must be < kN. Bounds-checked, and not noexcept, for the reason above.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:502`](../../include/shulib/localization/ekf_fusion.hpp#L502).*

<a id="ekffusion-velocityx"></a>

### `EkfFusion::velocityX`

```cpp
[[nodiscard]] units::Velocity velocityX() const noexcept
```

Body-frame velocity estimate, in/s.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:507`](../../include/shulib/localization/ekf_fusion.hpp#L507).*

<a id="ekffusion-velocityy"></a>

### `EkfFusion::velocityY`

```cpp
[[nodiscard]] units::Velocity velocityY() const noexcept
```

The body-frame LEFT (+Y) component, in/s — the `kVy` state. Both velocity getters report the filter's own smoothed velocity STATE, which is not `IPoseSource::twist()`: that one is a FIELD-frame finite difference of the published pose.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:511`](../../include/shulib/localization/ekf_fusion.hpp#L511).*

<a id="ekffusion-reinitcount"></a>

### `EkfFusion::reinitCount`

```cpp
[[nodiscard]] std::uint32_t reinitCount() const noexcept
```

How many times the covariance has been re-initialised (T2). Latched for the run.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:514`](../../include/shulib/localization/ekf_fusion.hpp#L514).*

<a id="ekffusion-everreinit"></a>

### `EkfFusion::everReinit`

```cpp
[[nodiscard]] bool everReinit() const noexcept
```

Latched: has this filter ever declared itself lost? Never clears — a run in which the estimator gave up once is a different run from one in which it did not, forever.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:517`](../../include/shulib/localization/ekf_fusion.hpp#L517).*

<a id="ekffusion-consecutiverejects"></a>

### `EkfFusion::consecutiveRejects`

```cpp
[[nodiscard]] int consecutiveRejects() const noexcept
```

Consecutive gate rejections right now (resets on any accepted fix).

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:519`](../../include/shulib/localization/ekf_fusion.hpp#L519).*

<a id="ekffusion-resynccount"></a>

### `EkfFusion::resyncCount`

```cpp
[[nodiscard]] std::uint32_t resyncCount() const noexcept
```

Ticks on which the filter re-based onto the handed prediction instead of predicting: `dt <= 0` (the tick after a `setPose` teleport) and `dt > maxDt` (a loop stall). The FIRST tick is NOT counted here — it initialises and returns before this test — so a 0 does not rule out the filter having adopted `predicted` wholesale on tick one. Latched for the run.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:524`](../../include/shulib/localization/ekf_fusion.hpp#L524).*

<a id="ekffusion-numericguardtrips"></a>

### `EkfFusion::numericGuardTrips`

```cpp
[[nodiscard]] std::uint32_t numericGuardTrips() const noexcept
```

Times a non-finite intermediate was caught and the update abandoned. Should be 0.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:526`](../../include/shulib/localization/ekf_fusion.hpp#L526).*

<a id="ekffusion-acceptedfixes"></a>

### `EkfFusion::acceptedFixes`

```cpp
[[nodiscard]] std::uint32_t acceptedFixes() const noexcept
```

Fixes accepted by the Mahalanobis gate, and fixes rejected by it.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:528`](../../include/shulib/localization/ekf_fusion.hpp#L528).*

<a id="ekffusion-rejectedfixes"></a>

### `EkfFusion::rejectedFixes`

```cpp
[[nodiscard]] std::uint32_t rejectedFixes() const noexcept
```

…counted per PROPOSAL rather than per tick, and cumulative for the run (neither clears). A MALFORMED proposal — non-finite pose, or σ <= 0 — is counted here too, because it fails the same test: the gate accepts only a finite distance at or under `gateSigma`, and a NaN satisfies no inequality.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:533`](../../include/shulib/localization/ekf_fusion.hpp#L533).*

<a id="ekffusion-lastcorrectionmagnitude"></a>

### `EkfFusion::lastCorrectionMagnitude`

```cpp
[[nodiscard]] units::Length lastCorrectionMagnitude() const noexcept
```

How far the last tick's CORRECTIONS moved the position, summed over the proposals folded (so it upper-bounds the net move). This — not `AppliedCorrection::dx`, which under this tier also carries the small velocity-filtering residual from steps B/C — is the quantity `maxNudgeRate · dt` bounds, and it is what a never-snap test should assert on.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:539`](../../include/shulib/localization/ekf_fusion.hpp#L539).*

<a id="ekffusion-lastheadingcorrectionmagnitude"></a>

### `EkfFusion::lastHeadingCorrectionMagnitude`

```cpp
[[nodiscard]] units::AngleDim lastHeadingCorrectionMagnitude() const noexcept
```

…and the same for heading: |the increment emitted last tick|, bounded by `maxHeadingNudgeRate · dt`.

*function, declared at [`include/shulib/localization/ekf_fusion.hpp:544`](../../include/shulib/localization/ekf_fusion.hpp#L544).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 225 lines, click to expand</summary>

```text

 EkfFusion — the M3 fusion policy: a 5-state SE(2) extended Kalman filter behind the SAME
 `IFusionPolicy` seam `ComplementaryFusion` has occupied since M2 (master plan §8,
 "complementary → EKF"). It is the first thing in the library that can WEIGH two correctors
 against each other instead of merely bounding the damage when they disagree, and the first
 thing that carries an explicit statement of HOW WRONG IT MIGHT BE.

 `ComplementaryFusion` is NOT replaced. It stays as the shipped default and the fallback tier
 (the development plan's reason: "the simpler filter is easier to get right and to explain").
 Both tiers are selectable, both are tested, and the choice is one constructor argument.

 ── WHAT A COVARIANCE BUYS, IN ONE PARAGRAPH ───────────────────────────────────────────────
 The complementary tier answers "how far should I move toward this fix?" with a constant times
 a confidence. It has no way to answer "how wrong am I right now?", so it cannot tell a fix
 that disagrees with a CONFIDENT estimate (probably the sensor is lying) from one that
 disagrees with a LOST estimate (probably the sensor is right). The covariance `P` is that
 missing answer, carried as a number and updated every tick: it GROWS while dead-reckoning, in
 proportion to how far the robot travelled, and SHRINKS every time a fix is folded, in
 proportion to how good that fix claimed to be. Everything else here follows from having it —
 the gate scales with it, the gain is derived from it, two disagreeing sources are combined by
 it, and it is what the estimator publishes when asked how confident it is.

 ── THE STATE, AND WHY EACH PIECE IS THERE ────────────────────────────────────────────────
     x = [ px, py, θ, vx, vy ]
   px, py   field-frame position, inches
   θ        heading, radians — see T1 below; this is a BELIEF ABOUT the published heading,
            not a second, competing heading
   vx, vy   BODY-frame velocity, in/s (vx forward, vy left)
 Body-frame velocity is what makes the model genuinely nonlinear and what makes θ genuinely
 load-bearing: `p⁻ = p + R(θ)·v·dt` cannot be evaluated without a heading. A field-frame
 velocity state would have made θ decorative.

 ── T1 — THE SEAM SAYS A POLICY CANNOT OWN HEADING, AND IT STILL DOESN'T ───────────────────
 `i_fusion_policy.hpp` states that a policy returns the corrected POSITION only, because the
 Localizer re-stamps heading from the IMU afterward. E3 then opened the ONE sanctioned heading
 path: a bounded `FusionResult::headingNudge` that the Localizer folds into a persistent bias
 before composing `imu.heading() + bias` as the last write of the tick.

 This filter tracks θ and changes NEITHER of those things. What leaves here is still only
 `{x, y}` plus a bounded increment. Concretely:

   * The θ TIME UPDATE is the IMU's, by construction: `θ⁻ := predicted.heading()` at the top
     of every tick. The filter does not integrate its own heading and does not hold a rival
     one. What it estimates is the ERROR in the IMU's answer, and that estimate leaves as an
     increment, exactly as E3 designed.
   * That assignment is ALSO what makes the feedback loop safe. `predicted.heading()` already
     contains the bias built out of this filter's own past nudges. A filter that integrated
     `Δθ = predicted.heading() − previousPredictedHeading` would count its own correction a
     SECOND time and overshoot, with the overshoot growing with the gain. Re-basing on the
     handed heading makes the double-count structurally impossible rather than arithmetically
     avoided. (Δθ is still computed — with this filter's own last nudge subtracted off — but
     only to size the process noise on θ, where what is wanted is the PHYSICAL rotation.)
   * Only a proposal that sets `providesHeading` may move θ. For every other update the θ row
     of the Kalman gain is ZEROED, so a GPS cannot rotate the robot's idea of the field
     through a cross-covariance term. That preserves E2's T3 ruling under the swap, and it is
     the reason the published heading is still bit-identical to the raw IMU on a tree with no
     heading-providing corrector.

 REJECTED — let the EKF own the published heading and delete the re-stamp. Tidier algebra, and
 a structural change to both M2's decision #4 and E3's design: a policy that can return an
 absolute heading can snap one. REJECTED — drop θ and run a 4-state filter with field-frame
 velocity. Simpler, and it destroys the point: with no θ in the state there is no θ VARIANCE,
 so a heading fix cannot be weighed against the filter's own uncertainty, and heading
 arbitration becomes impossible. REJECTED — track θ but emit nothing, leaving heading to the
 complementary tier. Then swapping tiers would silently DELETE E3's heading correction, which
 is a regression wearing a feature's clothes.

 ── THE TICK, IN ORDER ────────────────────────────────────────────────────────────────────
 The Localizer hands this policy an already-INTEGRATED dead-reckoned prediction, not a raw
 control input. Both facts below fall out of that.

   A. Re-base θ to `predicted.heading()`; recover this tick's field-frame odometry increment
      `u = predicted.position − lastReturnedPosition` (exact: the Localizer assigns
      `fusedX_ = fr.x`, so the policy's own last answer is what the prediction was built on).
      Add the process noise Q for the interval.
   B. ODOMETRY UPDATE — `R(θ)ᵀ·u/dt` is a measurement of the BODY-frame velocity, which is
      literally what the wheels measured (`PilonsOdometry` rotated it out by the same heading
      θ was just re-based to, so rotating it back is exact). This is the channel through which
      the wheels enter the filter. It moves the VELOCITY states ONLY: the p and θ rows of its
      gain are zeroed.
      WHY, and it is the most important line in this file: the wheels measure how far they
      TURNED. They say nothing directly about where the robot IS. Folding a relative
      measurement as though it were an absolute one would shrink the position covariance every
      tick, and a position covariance that shrinks while dead-reckoning is a filter that
      becomes CERTAIN as it becomes WRONG — after which no absolute fix can ever pass the
      gate. That is E2's D2 gate-lockout failure, arrived at from the other side.
   C. PROPAGATE — `p ← p + R(θ)·v·dt` with the just-updated velocity, and `P ← F P Fᵀ`
      (all of Q for the interval was added once, in step A).
      Using the POSTERIOR velocity is deliberate: `u/dt` is the AVERAGE velocity over the
      interval just ended, so it is the right velocity to carry the position across that same
      interval. Doing it the other way round (propagate, then update) lags the odometry by a
      full tick at every change of speed.
   D. CORRECT — each valid proposal in ascending σ (most trusted first), gated on Mahalanobis
      distance, applied through a Joseph-form update with the never-snap budget enforced as a
      GAIN REDUCTION (see below).
   E. EMIT — the posterior position, the accumulated θ change as `headingNudge`, and the audit.

 Consequence worth stating plainly: with no proposals at all, this filter's answer is NOT
 bit-identical to the odometry's, the way the complementary tier's is. It differs by the one
 tick of velocity filtering in steps B/C — measured at under 0.7 inches of cumulative gap over
 two minutes of stop-start driving, bounded by about one tick's travel, and provably NOT
 cumulative (four times the run length does not widen it). `AppliedCorrection::dx/dy` therefore
 reads that small residual on a dead-reckoning tick under this tier, where it reads exactly
 zero under the complementary one. Measured and pinned by test rather than asserted here.

 On a CORRECTING tick that residual is charged against the per-tick budget BEFORE any proposal
 is folded (see `foldProposals`), so the tick's total departure is `max(budget, residual)`
 rather than `budget + residual` — without that charge, a persistent correction stream was
 measured inflating the published move to 0.16 inches against a 0.12 inch budget, because a
 position fix teaches the velocity states and that comes back as motion on the next tick.

 WHAT THAT MEANS FOR THE §18.2 AUDIT, STATED EXACTLY, BECAUSE IT IS NOT QUITE THE SAME UNDER
 THE TWO TIERS. The never-snap budget bounds the CORRECTION exactly, under both tiers, always
 — `lastCorrectionMagnitude()` is the quantity, and it never exceeds `maxNudgeRate · dt`.
 `AppliedCorrection::dx/dy` is the tick's TOTAL departure from the dead-reckoned prediction,
 which under the complementary tier is the correction and nothing else, and under this tier is
 the correction OR the filtering residual, whichever is larger. Measured over eight seeds of a
 60-second hostile plant run, the residual reached **0.133 inches against a 0.12 inch budget**
 — 11% over, and only during the hardest direction changes in the script. That excess is not a
 correction: it is the estimate tracking the robot's real motion through a one-tick filter, and
 it is bounded by a fraction of one tick's travel. A reader auditing never-snap from a blackbox
 written under this tier should read `dx/dy` with that 11% allowance, and chapter 11 says so.

 ── THE NEVER-SNAP BOUND IS A GAIN REDUCTION, WHICH IS WHY THE JOSEPH FORM IS LOAD-BEARING ─
 Decision #4 says a correction is a bounded nudge, never a snap, and it does not stop applying
 because the filter got cleverer. The obvious implementation — take the Kalman step, then clip
 the state move — makes the filter LIE: `P` would shrink as though the full correction had
 been applied while the state still sat where the clip left it, i.e. it would become confident
 precisely because it was prevented from correcting.

 So the bound is applied to the GAIN instead. If the optimal step would move the position by
 more than `maxNudgeRate·dt` (or the heading by more than `maxHeadingNudgeRate·dt`), the gain
 is scaled by the ratio, and the covariance is then updated with THAT gain. The Joseph form
     P⁺ = (I − K H) P⁻ (I − K H)ᵀ + K R Kᵀ
 is exactly correct for ANY gain, optimal or not — that is its actual virtue, and it is the
 reason it is used here rather than the shorter `(I − K H) P⁻`, which is only valid at the
 optimal gain and silently loses symmetry and positive-definiteness away from it. Every
 deliberately suboptimal gain in this file (the rate clamp, the zeroed heading row, the
 velocity-only odometry update) depends on that property.

 ── T4 — THE 12-INCH CEILING IS REPLACED HERE, AND SURVIVES IN THE OTHER TIER ──────────────
 `ComplementaryFusion::innovationGate` is a fixed 12 inches, and E2 recorded live that an
 estimate 29 inches out never recovered with a perfectly good GPS in view. A fixed distance is
 exactly what a covariance replaces. The test here is
     ν = √( rᵀ S⁻¹ r ),   S = H P Hᵀ + R,   reject if ν > gateSigma
 so the SAME 29-inch fix is REJECTED when the filter is confident (it is far more likely to be
 a reflection than the truth) and ACCEPTED when the filter knows it is badly lost. The
 complementary tier keeps its 12-inch gate unchanged — it has no `P` to normalise by, and E2's
 T1 already ruled that a distance normalised by an assumed constant must not be called a
 Mahalanobis distance.

 ── T5 — `gateMahalanobis` BECOMES REAL ───────────────────────────────────────────────────
 `GateAudit::mahalanobis` and `DebugRecord::gateMahalanobis` have been declared and empty since
 A1, and E2 refused to fill them with the ratio it had, because normalising by an ASSUMED
 constant makes the assumption the entire content. The number written here comes from `S`,
 which is `P` (estimated by this filter, tick by tick) plus `R` (the fix's own stated σ), so
 `RejectedMahalanobis` is finally raisable by something that earned it.
 `GateAudit::covarianceTrace` carries `P[px][px] + P[py][py]`, in square inches — the POSITION
 block only, because a trace over a state vector mixing inches, radians and inches-per-second
 is a number with no unit and no meaning. A reader wanting a 1σ radius takes `√(trace/2)`.

 ── T2 — "CONSECUTIVE-REJECT RE-INIT" vs §13 #4 "NEVER SNAP" ───────────────────────────────
 The development plan asks for a consecutive-reject re-init; §13 #4 forbids snapping; and a
 re-init that teleports the estimate IS a snap. The conflict is real as written, and it
 dissolves the moment
 "re-init" is read as re-initialising the belief's UNCERTAINTY rather than its VALUE.

 RULING: on the trigger, this filter does not move the estimate by so much as a thousandth of
 an inch. It resets the POSITION and VELOCITY covariance to the initial prior and nothing else.
   * Never-snap holds bit-for-bit — the per-tick rate clamp is untouched and still binds on the
     re-init tick and every tick after it. No existing never-snap test changed.
   * The estimator nevertheless RECOVERS, which is the entire reason re-init was asked for:
     with a large P the Mahalanobis gate opens, the following fixes are accepted with a large
     gain, and the estimate walks home at up to `maxNudgeRate` instead of never arriving.
   * It is DECLARED: `GateReason::CovarianceReinit` on that tick, so the event is a word in the
     decoded blackbox rather than an inference, and `covarianceTrace` jumps on the same tick as
     an independent numeric witness of the same event.
   * It is LATCHED and counted (`reinitCount()`, `everReinit()`), and RATE-LIMITED by a
     cooldown, and it takes a high bar: N CONSECUTIVE gate rejections AND a mean rejected
     innovation above a floor. Ticks where nothing was proposed neither count nor reset.
 REJECTED — teleport the state onto the rejected fix. That is the snap §13 #4 forbids, and it
 contains a plain self-contradiction: the trigger is the filter saying N times that it does NOT
 trust this fix, so jumping onto it trusts it completely on the strength of having distrusted
 it. REJECTED — no re-init at all: that preserves E2's finding 2 forever, and a robot shoved by
 an opponent never recovers with a perfect fix in view. REJECTED — permanently widen the gate
 after N rejections: the outlier protection is then spent once and gone, where covariance
 inflation self-heals as fixes are folded.

 ── HOW PROPOSALS ARE WEIGHED (the capability that justifies the chunk) ────────────────────
 Proposals are folded as SEQUENTIAL Kalman updates in ascending `positionStdDev`, which for
 independent measurements is equivalent to a batch update while still letting each one be gated
 on its own merits (a batch update cannot reject one row). Most-trusted-first is deliberate: a
 good fix tightens P before a doubtful one is tested against it. Two sources that disagree
 therefore settle at the inverse-variance-weighted point between them —
     x* = (z_A/σ_A² + z_B/σ_B²) / (1/σ_A² + 1/σ_B²)
 — rather than at whichever arrived first, or at the midpoint.

 **`confidence` is NOT used as a weight, deliberately.** E2 DERIVES its confidence from σ (it
 is the scalar Kalman gain σ_dr²/(σ_dr² + σ_meas²)), so weighting by both would count the same
 information twice. This tier weights by the stated σ and by nothing else — which is precisely
 the difference between a covariance filter and the gain knob that HA-66 and HA-78 have been
 wearing a covariance's clothes as. `confidence` is still read for ONE purpose: it is passed
 back out as `appliedConfidence`, which is Localizer bookkeeping (how much of the drift
 accumulator an applied fix clears), not fusion weighting.

 Heading has no per-proposal σ on `CorrectionProposal`, so heading measurements use one
 configured σ. REJECTED — append a `headingStdDev` field now (E3's handoff suggested it): no
 corrector in the tree can state one, and "nothing reads it today" is exactly how a field
 becomes load-bearing by accident (E2's T3, in reverse). It stays a named handoff for whoever
 adds a corrector that can measure it. REJECTED — scale the heading σ by `confidence`: that
 invents a relationship between a [0,1] trust scalar and a variance, and it would be
 inconsistent with the position channel, which ignores confidence.

 ── COST ──────────────────────────────────────────────────────────────────────────────────
 Everything is fixed-size `std::array` on the stack: 5 states, a 5×5 covariance, at most
 `Localizer::kMaxCorrectors` proposals. `fuse()` never allocates and never throws (all
 preconditions are in the constructor; every runtime pathology is screened and counted rather
 than raised). Pinned by test with a replaced global allocator, not asserted here.

 ── WHAT IS INVENTED ──────────────────────────────────────────────────────────────────────
 Every noise number below is a GUESS until R4 measures the hardware. They are registered
 HA-83…HA-91 and each carries its tag. The STRUCTURE is what this chunk proves; the NUMBERS
 are fitted on a robot that does not exist yet. No test in this chunk asserts that a constant
 is right — only that a shape is (farther travel ⇒ more uncertainty, a tighter σ ⇒ more pull,
 a confident filter rejects what a lost one accepts).
```

</details>
