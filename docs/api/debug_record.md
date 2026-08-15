<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/debug_record.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `debug_record.hpp`

DebugRecord — the per-tick snapshot schema.

This header declares **3** types (55 members) and **1** constant.

Extracted from [`include/shulib/diag/debug_record.hpp`](../../include/shulib/diag/debug_record.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class GateReason`](#enum-class-gatereason)
  - [`None`](#gatereason-none)
  - [`Accepted`](#gatereason-accepted)
  - [`RejectedInnovation`](#gatereason-rejectedinnovation)
  - [`RejectedMahalanobis`](#gatereason-rejectedmahalanobis)
  - [`RejectedNoFix`](#gatereason-rejectednofix)
  - [`RejectedHighYawRate`](#gatereason-rejectedhighyawrate)
  - [`RejectedNormalizedInnovation`](#gatereason-rejectednormalizedinnovation)
  - [`RejectedStaleFix`](#gatereason-rejectedstalefix)
  - [`RejectedSensorQuality`](#gatereason-rejectedsensorquality)
  - [`RejectedNoTagMapEntry`](#gatereason-rejectednotagmapentry)
  - [`RejectedTagRange`](#gatereason-rejectedtagrange)
  - [`RejectedObservationAge`](#gatereason-rejectedobservationage)
  - [`CovarianceReinit`](#gatereason-covariancereinit)
- [`enum class TickPhase`](#enum-class-tickphase)
  - [`Localization`](#tickphase-localization)
  - [`Motion`](#tickphase-motion)
  - [`Health`](#tickphase-health)
  - [`Telemetry`](#tickphase-telemetry)
  - [`Scheduler`](#tickphase-scheduler)
  - [`User`](#tickphase-user)
- [`kTickPhaseSlots`](#ktickphaseslots) — *constant*
- [`struct DebugRecord`](#struct-debugrecord)
  - [`kMaxWheels`](#debugrecord-kmaxwheels)
  - [`t`](#debugrecord-t)
  - [`dt`](#debugrecord-dt)
  - [`targetPose`](#debugrecord-targetpose)
  - [`measuredPose`](#debugrecord-measuredpose)
  - [`errorX`](#debugrecord-errorx)
  - [`errorY`](#debugrecord-errory)
  - [`errorHeading`](#debugrecord-errorheading)
  - [`commanded`](#debugrecord-commanded)
  - [`wheelCount`](#debugrecord-wheelcount)
  - [`wheelVoltage`](#debugrecord-wheelvoltage)
  - [`wheelCurrent`](#debugrecord-wheelcurrent)
  - [`imuYaw`](#debugrecord-imuyaw)
  - [`imuYawRate`](#debugrecord-imuyawrate)
  - [`activeCommandId`](#debugrecord-activecommandid)
  - [`activeCommandState`](#debugrecord-activecommandstate)
  - [`deadReckoning`](#debugrecord-deadreckoning)
  - [`qualityClass`](#debugrecord-qualityclass)
  - [`quality`](#debugrecord-quality)
  - [`covarianceTrace`](#debugrecord-covariancetrace)
  - [`gateResidualX`](#debugrecord-gateresidualx)
  - [`gateResidualY`](#debugrecord-gateresidualy)
  - [`gateResidualHeading`](#debugrecord-gateresidualheading)
  - [`gateMahalanobis`](#debugrecord-gatemahalanobis)
  - [`gateReason`](#debugrecord-gatereason)
  - [`correctionDx`](#debugrecord-correctiondx)
  - [`correctionDy`](#debugrecord-correctiondy)
  - [`correctionDTheta`](#debugrecord-correctiondtheta)
  - [`clampedThisTick`](#debugrecord-clampedthistick)
  - [`strafeFallbackActive`](#debugrecord-strafefallbackactive)
  - [`fault`](#debugrecord-fault)
  - [`batteryVoltage`](#debugrecord-batteryvoltage)
  - [`batteryCurrent`](#debugrecord-batterycurrent)
  - [`droppedRecords`](#debugrecord-droppedrecords)
  - [`droppedLines`](#debugrecord-droppedlines)
  - [`tickPhase`](#debugrecord-tickphase)

<a id="enum-class-gatereason"></a>

## `enum class GateReason`

```cpp
enum class GateReason : std::uint8_t
```

Why the fusion gate accepted/rejected this tick's correction (§18.2 "gating reason"). WIRE-STABLE: explicit values, append-only, pinned by test (these go on the F9 wire). A1 defines the vocabulary; E2/E3 (correctors) and E4 (EKF) populate it.

*enum class, declared at [`include/shulib/diag/debug_record.hpp:48`](../../include/shulib/diag/debug_record.hpp#L48).*

<a id="gatereason-none"></a>

### `GateReason::None`

```cpp
None = 0
```

no correction proposal this tick (pure dead-reckoning)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:49`](../../include/shulib/diag/debug_record.hpp#L49).*

<a id="gatereason-accepted"></a>

### `GateReason::Accepted`

```cpp
Accepted = 1
```

a proposal passed the gate and was (nudge-)applied

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:50`](../../include/shulib/diag/debug_record.hpp#L50).*

<a id="gatereason-rejectedinnovation"></a>

### `GateReason::RejectedInnovation`

```cpp
RejectedInnovation = 2
```

outside the innovation bound (complementary tier)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:51`](../../include/shulib/diag/debug_record.hpp#L51).*

<a id="gatereason-rejectedmahalanobis"></a>

### `GateReason::RejectedMahalanobis`

```cpp
RejectedMahalanobis = 3
```

failed the Mahalanobis gate (EKF tier, E4)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:52`](../../include/shulib/diag/debug_record.hpp#L52).*

<a id="gatereason-rejectednofix"></a>

### `GateReason::RejectedNoFix`

```cpp
RejectedNoFix = 4
```

source had no usable fix (off-strip GPS / no tag, E2/E3)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:53`](../../include/shulib/diag/debug_record.hpp#L53).*

<a id="gatereason-rejectedhighyawrate"></a>

### `GateReason::RejectedHighYawRate`

```cpp
RejectedHighYawRate = 5
```

spinning too fast to trust the fix (E2)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:54`](../../include/shulib/diag/debug_record.hpp#L54).*

<a id="gatereason-rejectednormalizedinnovation"></a>

### `GateReason::RejectedNormalizedInnovation`

```cpp
RejectedNormalizedInnovation = 6
```

Failed the complementary tier's NORMALIZED-INNOVATION gate: |residual| exceeded `gateSigma` times the fix's own 1σ (measurement σ from the device's reported error, widened by how far the estimate has dead-reckoned since its last fix). Appended at E2 rather than reusing `RejectedMahalanobis`, which needs a filter-estimated covariance the complementary tier does not have — see gps_corrector.hpp. (E2)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:60`](../../include/shulib/diag/debug_record.hpp#L60).*

<a id="gatereason-rejectedstalefix"></a>

### `GateReason::RejectedStaleFix`

```cpp
RejectedStaleFix = 7
```

The source re-reported a sample it has already folded, so there is no new information this tick. The V5 GPS camera produces a fix every ~50 ms while the control loop runs at ~100 Hz, so a corrector that folds every read counts one measurement five times. (E2)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:65`](../../include/shulib/diag/debug_record.hpp#L65).*

<a id="gatereason-rejectedsensorquality"></a>

### `GateReason::RejectedSensorQuality`

```cpp
RejectedSensorQuality = 8
```

The source claims a fix but reports a self-error too large to be worth folding — a sensor saying "I can see, badly" rather than "I cannot see". (E2) — reused at E3 for a tag detection below the confidence floor, which is the same statement.

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:69`](../../include/shulib/diag/debug_record.hpp#L69).*

<a id="gatereason-rejectednotagmapentry"></a>

### `GateReason::RejectedNoTagMapEntry`

```cpp
RejectedNoTagMapEntry = 9
```

A tag was SEEN but the tag map does not know where it is, so no absolute pose can be derived from it. Distinct from RejectedNoFix on purpose: this is a CONFIGURATION error the team can fix (an id missing from the map, or an empty map), not the field being the field, and it is the one worth shouting about. (E3)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:74`](../../include/shulib/diag/debug_record.hpp#L74).*

<a id="gatereason-rejectedtagrange"></a>

### `GateReason::RejectedTagRange`

```cpp
RejectedTagRange = 10
```

Every visible tag was outside the corrector's trusted range band — too close to fit in the frame, or far enough that planar-PnP's heading ambiguity makes the orientation untrustworthy (localization/apriltag_corrector.hpp). (E3)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:78`](../../include/shulib/diag/debug_record.hpp#L78).*

<a id="gatereason-rejectedobservationage"></a>

### `GateReason::RejectedObservationAge`

```cpp
RejectedObservationAge = 11
```

The newest vision frame is older than the corrector's freshness horizon: the vision task has stalled, died, or was never started. Distinct from RejectedStaleFix (a frame already folded — the normal steady state) and from RejectedNoFix (looked, saw nothing), because "the camera stopped talking" calls for a different response than either. (E3)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:83`](../../include/shulib/diag/debug_record.hpp#L83).*

<a id="gatereason-covariancereinit"></a>

### `GateReason::CovarianceReinit`

```cpp
CovarianceReinit = 12
```

The EKF tier gave up on its own confidence and re-initialised its covariance: N consecutive Mahalanobis rejections with a persistently large innovation means the filter's belief about how wrong it might be is itself wrong. **The estimate is NOT moved** — only the uncertainty is reset — so §13 #4's never-snap bound still holds on this tick and every tick after it (E4's T2 ruling, localization/ekf_fusion.hpp). It is a WORD in the record rather than an inference because an estimator that quietly changes its mind about how much to trust the world is the hardest kind of run to debug; `covarianceTrace` jumping on the same tick is the independent numeric witness. (E4)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:92`](../../include/shulib/diag/debug_record.hpp#L92).*

<a id="enum-class-tickphase"></a>

## `enum class TickPhase`

```cpp
enum class TickPhase : std::uint8_t
```

Index vocabulary for DebugRecord::tickPhase — WHO consumed the loop budget this tick (diagnostics-plan D-3: LoopMonitor detects an overrun but cannot attribute it; these slots turn "the loop is slow" into a name). WIRE-STABLE: explicit values, append-only, pinned by test (F9 serializes the array these index). Defined at C5 — BEFORE the H1 freeze — precisely so the slots exist even where the producer does not yet (the one genuinely time-sensitive act in diagnostics-plan.md).

*enum class, declared at [`include/shulib/diag/debug_record.hpp:102`](../../include/shulib/diag/debug_record.hpp#L102).*

<a id="tickphase-localization"></a>

### `TickPhase::Localization`

```cpp
Localization = 0
```

Localizer::update() — producer: C5 (MotionScheduler)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:103`](../../include/shulib/diag/debug_record.hpp#L103).*

<a id="tickphase-motion"></a>

### `TickPhase::Motion`

```cpp
Motion = 1
```

the active motion's tick / the idle work — producer: C5

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:104`](../../include/shulib/diag/debug_record.hpp#L104).*

<a id="tickphase-health"></a>

### `TickPhase::Health`

```cpp
Health = 2
```

health observables, where separable — RESERVED (E1+)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:105`](../../include/shulib/diag/debug_record.hpp#L105).*

<a id="tickphase-telemetry"></a>

### `TickPhase::Telemetry`

```cpp
Telemetry = 3
```

sink formatting/IO, where separable — RESERVED (E1+)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:106`](../../include/shulib/diag/debug_record.hpp#L106).*

<a id="tickphase-scheduler"></a>

### `TickPhase::Scheduler`

```cpp
Scheduler = 4
```

scheduler bookkeeping, where separable — RESERVED (E1+)

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:107`](../../include/shulib/diag/debug_record.hpp#L107).*

<a id="tickphase-user"></a>

### `TickPhase::User`

```cpp
User = 5
```

caller-owned work (G2 markers, mechanisms) — RESERVED, still no producer. F1 RULED why it stayed empty rather than filling it: the only place caller work is visible today is a waitUntil predicate, which runs OUTSIDE the attribution bracket, and crediting it would break the pinned sum contract (attributed phases never exceed the tick total — tick_attribution.hpp). The named producer is F2's sequencer loop / G2's marker dispatch, which own a loop and can bracket user work properly.

*enumerator, declared at [`include/shulib/diag/debug_record.hpp:108`](../../include/shulib/diag/debug_record.hpp#L108).*

<a id="ktickphaseslots"></a>

## `kTickPhaseSlots`

```cpp
inline constexpr int kTickPhaseSlots = 8
```

Capacity of DebugRecord::tickPhase. STRICTLY GREATER than the defined phases on purpose: slots 6..7 are spare, reserved before the F9 freeze so a new phase is a vocabulary append, not a wire reshape. Pinned by test.

*constant, declared at [`include/shulib/diag/debug_record.hpp:121`](../../include/shulib/diag/debug_record.hpp#L121).*

<a id="struct-debugrecord"></a>

## `struct DebugRecord`

```cpp
struct DebugRecord
```

The per-tick snapshot (§18.2), captured each control tick and rate-budgeted by the producer. Plain struct on purpose: it is a snapshot, not an invariant-bearing type — the invariants live in the systems that populate it.

*struct, declared at [`include/shulib/diag/debug_record.hpp:126`](../../include/shulib/diag/debug_record.hpp#L126).*

<a id="debugrecord-kmaxwheels"></a>

### `DebugRecord::kMaxWheels`

```cpp
static constexpr int kMaxWheels = kinematics::WheelSpeeds::kMaxWheels
```

Per-wheel capacity, tied to the kinematics contract so they can never diverge.

*field, declared at [`include/shulib/diag/debug_record.hpp:128`](../../include/shulib/diag/debug_record.hpp#L128).*

<a id="debugrecord-t"></a>

### `DebugRecord::t`

```cpp
units::Time t{}
```

seconds since the run epoch (the [t=…] stamp) — producer: C1

*field, declared at [`include/shulib/diag/debug_record.hpp:131`](../../include/shulib/diag/debug_record.hpp#L131).*

<a id="debugrecord-dt"></a>

### `DebugRecord::dt`

```cpp
units::Time dt{}
```

this tick's measured dt — producer: C1 (via LoopMonitor)

*field, declared at [`include/shulib/diag/debug_record.hpp:132`](../../include/shulib/diag/debug_record.hpp#L132).*

<a id="debugrecord-targetpose"></a>

### `DebugRecord::targetPose`

```cpp
math::Pose2d targetPose{}
```

where the active motion wants the robot — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:135`](../../include/shulib/diag/debug_record.hpp#L135).*

<a id="debugrecord-measuredpose"></a>

### `DebugRecord::measuredPose`

```cpp
math::Pose2d measuredPose{}
```

the fused estimate (Localizer::pose()) — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:136`](../../include/shulib/diag/debug_record.hpp#L136).*

<a id="debugrecord-errorx"></a>

### `DebugRecord::errorX`

```cpp
units::Length errorX{}
```

target − measured, field x — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:137`](../../include/shulib/diag/debug_record.hpp#L137).*

<a id="debugrecord-errory"></a>

### `DebugRecord::errorY`

```cpp
units::Length errorY{}
```

target − measured, field y — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:138`](../../include/shulib/diag/debug_record.hpp#L138).*

<a id="debugrecord-errorheading"></a>

### `DebugRecord::errorHeading`

```cpp
units::AngleDim errorHeading{}
```

shortest signed heading error (radians) — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:139`](../../include/shulib/diag/debug_record.hpp#L139).*

<a id="debugrecord-commanded"></a>

### `DebugRecord::commanded`

```cpp
math::ChassisSpeeds commanded{}
```

commanded (vx, vy, ω) this tick — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:140`](../../include/shulib/diag/debug_record.hpp#L140).*

<a id="debugrecord-wheelcount"></a>

### `DebugRecord::wheelCount`

```cpp
int wheelCount = 0
```

valid entries in the arrays below, [0, kMaxWheels] — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:143`](../../include/shulib/diag/debug_record.hpp#L143).*

<a id="debugrecord-wheelvoltage"></a>

### `DebugRecord::wheelVoltage`

```cpp
std::array<units::Voltage, static_cast<std::size_t>(kMaxWheels)> wheelVoltage{}
```

— C1

*field, declared at [`include/shulib/diag/debug_record.hpp:144`](../../include/shulib/diag/debug_record.hpp#L144).*

<a id="debugrecord-wheelcurrent"></a>

### `DebugRecord::wheelCurrent`

```cpp
std::array<units::Current, static_cast<std::size_t>(kMaxWheels)> wheelCurrent{}
```

— C1

*field, declared at [`include/shulib/diag/debug_record.hpp:145`](../../include/shulib/diag/debug_record.hpp#L145).*

<a id="debugrecord-imuyaw"></a>

### `DebugRecord::imuYaw`

```cpp
math::Angle imuYaw{}
```

canonical IMU heading — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:148`](../../include/shulib/diag/debug_record.hpp#L148).*

<a id="debugrecord-imuyawrate"></a>

### `DebugRecord::imuYawRate`

```cpp
units::AngularVelocity imuYawRate{}
```

canonical yaw rate — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:149`](../../include/shulib/diag/debug_record.hpp#L149).*

<a id="debugrecord-activecommandid"></a>

### `DebugRecord::activeCommandId`

```cpp
std::uint32_t activeCommandId = 0
```

0 = no active command. Ids are assigned by the motion scheduler (C2); the value is wire-stable as a plain integer regardless of what the ids come to mean.

*field, declared at [`include/shulib/diag/debug_record.hpp:154`](../../include/shulib/diag/debug_record.hpp#L154).*

<a id="debugrecord-activecommandstate"></a>

### `DebugRecord::activeCommandState`

```cpp
std::uint8_t activeCommandState = 0
```

Motion-layer state (run/settling/…). 0 = idle. The VOCABULARY is owned by the motion layer (C1/C2); once assigned, values are wire-stable like FaultCode's.

*field, declared at [`include/shulib/diag/debug_record.hpp:157`](../../include/shulib/diag/debug_record.hpp#L157).*

<a id="debugrecord-deadreckoning"></a>

### `DebugRecord::deadReckoning`

```cpp
bool deadReckoning = false
```

Localizer::isDeadReckoning() — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:160`](../../include/shulib/diag/debug_record.hpp#L160).*

<a id="debugrecord-qualityclass"></a>

### `DebugRecord::qualityClass`

```cpp
std::uint8_t qualityClass = 0
```

Categorical quality, mirroring localization::Localizer::Quality numerically (0=Uninitialized 1=DeadReckon 2=Corrected 3=Degraded — the mapping is pinned by test so a reorder of either enum is caught). Kept as a raw byte so diag/ stays a leaf that localization/ may depend on, never the reverse. — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:165`](../../include/shulib/diag/debug_record.hpp#L165).*

<a id="debugrecord-quality"></a>

### `DebugRecord::quality`

```cpp
double quality = 0.0
```

the [0,1] scalar (Localizer::quality()) — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:166`](../../include/shulib/diag/debug_record.hpp#L166).*

<a id="debugrecord-covariancetrace"></a>

### `DebugRecord::covarianceTrace`

```cpp
double covarianceTrace = 0.0
```

EKF covariance trace once E4 lands; the complementary tier may surface its scalar trust weight here until then. One slot, semantics per active fusion policy (§18.2 "covariance trace / filter trust weights"). — E4

*field, declared at [`include/shulib/diag/debug_record.hpp:170`](../../include/shulib/diag/debug_record.hpp#L170).*

<a id="debugrecord-gateresidualx"></a>

### `DebugRecord::gateResidualX`

```cpp
units::Length gateResidualX{}
```

innovation, field x — E2/E3

*field, declared at [`include/shulib/diag/debug_record.hpp:173`](../../include/shulib/diag/debug_record.hpp#L173).*

<a id="debugrecord-gateresidualy"></a>

### `DebugRecord::gateResidualY`

```cpp
units::Length gateResidualY{}
```

innovation, field y — E2/E3

*field, declared at [`include/shulib/diag/debug_record.hpp:174`](../../include/shulib/diag/debug_record.hpp#L174).*

<a id="debugrecord-gateresidualheading"></a>

### `DebugRecord::gateResidualHeading`

```cpp
units::AngleDim gateResidualHeading{}
```

innovation, heading (radians) — E3

*field, declared at [`include/shulib/diag/debug_record.hpp:175`](../../include/shulib/diag/debug_record.hpp#L175).*

<a id="debugrecord-gatemahalanobis"></a>

### `DebugRecord::gateMahalanobis`

```cpp
double gateMahalanobis = 0.0
```

Mahalanobis distance of the fix — E4

*field, declared at [`include/shulib/diag/debug_record.hpp:176`](../../include/shulib/diag/debug_record.hpp#L176).*

<a id="debugrecord-gatereason"></a>

### `DebugRecord::gateReason`

```cpp
GateReason gateReason = GateReason::None
```

why accepted/rejected — E2/E3/E4

*field, declared at [`include/shulib/diag/debug_record.hpp:177`](../../include/shulib/diag/debug_record.hpp#L177).*

<a id="debugrecord-correctiondx"></a>

### `DebugRecord::correctionDx`

```cpp
units::Length correctionDx{}
```

net position nudge applied this tick — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:180`](../../include/shulib/diag/debug_record.hpp#L180).*

<a id="debugrecord-correctiondy"></a>

### `DebugRecord::correctionDy`

```cpp
units::Length correctionDy{}
```

— C1

*field, declared at [`include/shulib/diag/debug_record.hpp:181`](../../include/shulib/diag/debug_record.hpp#L181).*

<a id="debugrecord-correctiondtheta"></a>

### `DebugRecord::correctionDTheta`

```cpp
units::AngleDim correctionDTheta{}
```

heading nudge (0 at M2: heading is IMU-owned) — E3

*field, declared at [`include/shulib/diag/debug_record.hpp:182`](../../include/shulib/diag/debug_record.hpp#L182).*

<a id="debugrecord-clampedthistick"></a>

### `DebugRecord::clampedThisTick`

```cpp
bool clampedThisTick = false
```

the per-tick nudge budget bound the correction — C1

*field, declared at [`include/shulib/diag/debug_record.hpp:183`](../../include/shulib/diag/debug_record.hpp#L183).*

<a id="debugrecord-strafefallbackactive"></a>

### `DebugRecord::strafeFallbackActive`

```cpp
bool strafeFallbackActive = false
```

H-drive turn-then-drive fallback engaged (§13 #5) — C3

*field, declared at [`include/shulib/diag/debug_record.hpp:184`](../../include/shulib/diag/debug_record.hpp#L184).*

<a id="debugrecord-fault"></a>

### `DebugRecord::fault`

```cpp
FaultCode fault = FaultCode::None
```

fault raised THIS tick (the latch keeps the first)

*field, declared at [`include/shulib/diag/debug_record.hpp:187`](../../include/shulib/diag/debug_record.hpp#L187).*

<a id="debugrecord-batteryvoltage"></a>

### `DebugRecord::batteryVoltage`

```cpp
units::Voltage batteryVoltage{}
```

— C1

*field, declared at [`include/shulib/diag/debug_record.hpp:188`](../../include/shulib/diag/debug_record.hpp#L188).*

<a id="debugrecord-batterycurrent"></a>

### `DebugRecord::batteryCurrent`

```cpp
units::Current batteryCurrent{}
```

— C1

*field, declared at [`include/shulib/diag/debug_record.hpp:189`](../../include/shulib/diag/debug_record.hpp#L189).*

<a id="debugrecord-droppedrecords"></a>

### `DebugRecord::droppedRecords`

```cpp
std::uint32_t droppedRecords = 0
```

emit()-channel records dropped so far — C5

*field, declared at [`include/shulib/diag/debug_record.hpp:196`](../../include/shulib/diag/debug_record.hpp#L196).*

<a id="debugrecord-droppedlines"></a>

### `DebugRecord::droppedLines`

```cpp
std::uint32_t droppedLines = 0
```

log()-channel lines dropped so far — C5

*field, declared at [`include/shulib/diag/debug_record.hpp:197`](../../include/shulib/diag/debug_record.hpp#L197).*

<a id="debugrecord-tickphase"></a>

### `DebugRecord::tickPhase`

```cpp
std::array<units::Time, static_cast<std::size_t>(kTickPhaseSlots)> tickPhase{}
```

D-3: per-subsystem tick-time attribution in canonical seconds, indexed by TickPhase (top of file). Slots for phases marked RESERVED hold 0 until their producer exists; slots 6..7 are spare capacity (kTickPhaseSlots note). The values describe the most recently COMPLETED tick (the stamping sink cannot know the current tick's total mid-tick; one-tick lag, documented at the producer). All-zero when attribution is off, which is indistinguishable here from a tick that spent no time anywhere — read it with that in mind. Do NOT audit the sum against `dt`: tick_attribution.hpp's "attributed never exceeds the total" contract is qualified ON THE SAME CLOCK, and the attribution clock is injected separately from the loop clock `dt` is measured on — this record carries no attribution total of its own to compare against. Even on that one clock the relation is soft enough that TickAttribution floors its own remainder at 0, because a clock that jumps mid-phase can push the phases past the total. So read a shortfall as un-instrumented work rather than a missing phase, and read a sum above `dt` as a statement about two clocks rather than a broken record. — C5 (scheduler)

*field, declared at [`include/shulib/diag/debug_record.hpp:212`](../../include/shulib/diag/debug_record.hpp#L212).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 30 lines</summary>

```text

 DebugRecord — the per-tick snapshot schema (master plan §18.2; WS13, chunk A1).

 ONE record, MANY sinks (§18.1): this struct is the single source of truth every sink
 merely FORMATS — TermSink pretty-prints it (A1), SdSink writes it to the blackbox (E1),
 Shul2Sink serializes it onto the wire (H1). Because every sink shares this one schema,
 bench / terminal / field / sim traces are directly comparable.

 ── FREEZE REGISTER NOTE (read before touching a field) ─────────────────────────────
 F9 (frozen at chunk H1) is the wire serialization of EXACTLY this record. From A1
 onward, every field change carries the cost of reshaping that wire, every sink, the
 SdSink blackbox layout, and the VexBuilder overlay AT ONCE. That is why the COMPLETE
 §18.2 field set is defined NOW — including fields for systems that do not exist yet
 (gating residuals at E2/E3, covariance trace at E4, strafe fallback at C3). Later
 chunks POPULATE fields; they must never RESHAPE them. Additions are append-only.
 ────────────────────────────────────────────────────────────────────────────────────

 Populated-when: each field notes the chunk that first writes it. Until then it holds
 its documented default — a default-constructed record is a valid "quiet" record (no
 command, no fault, zero quality), pinned by test/debug_record_test.cpp.

 Typed units by design (F3): every field with a dimension is a units::Quantity /
 math value type, so a milliseconds-vs-seconds or degrees-vs-radians confusion cannot
 enter the record. Angular ERRORS/deltas are units::AngleDim (radians, non-wrapping —
 a difference is not a heading); absolute headings are math::Angle (wrapping).

 Fixed capacity, no heap: the per-wheel arrays reuse kinematics::WheelSpeeds::kMaxWheels
 so the two capacities can never diverge; the record is a flat value type produced every
 ~10ms tick, so it must never allocate (see hal::emitRecord for how a NullSink build
 skips even POPULATING it).
```

</details>
