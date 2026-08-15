<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/gps_corrector.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `gps_corrector.hpp`

GpsCorrector — the FIRST REAL corrector.

This header declares **2** types (20 members).

Extracted from [`include/shulib/localization/gps_corrector.hpp`](../../include/shulib/localization/gps_corrector.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct GpsCorrectorConfig`](#struct-gpscorrectorconfig)
  - [`latency`](#gpscorrectorconfig-latency)
  - [`rmsTrustFactor`](#gpscorrectorconfig-rmstrustfactor)
  - [`minPositionStdDev`](#gpscorrectorconfig-minpositionstddev)
  - [`maxReportedRms`](#gpscorrectorconfig-maxreportedrms)
  - [`maxYawRate`](#gpscorrectorconfig-maxyawrate)
  - [`gateSigma`](#gpscorrectorconfig-gatesigma)
  - [`postFixStdDev`](#gpscorrectorconfig-postfixstddev)
  - [`driftStdDevPerInch`](#gpscorrectorconfig-driftstddevperinch)
- [`class GpsCorrector`](#class-gpscorrector)
  - [`kHistory`](#gpscorrector-khistory)
  - [`GpsCorrector`](#gpscorrector-gpscorrector)
  - [`propose`](#gpscorrector-propose)
  - [`name`](#gpscorrector-name)
  - [`lastVerdict`](#gpscorrector-lastverdict)
  - [`acceptedFixes`](#gpscorrector-acceptedfixes)
  - [`noFixTicks`](#gpscorrector-nofixticks)
  - [`staleTicks`](#gpscorrector-staleticks)
  - [`qualityRejects`](#gpscorrector-qualityrejects)
  - [`yawRateRejects`](#gpscorrector-yawraterejects)
  - [`innovationRejects`](#gpscorrector-innovationrejects)
  - [`travelSinceFix`](#gpscorrector-travelsincefix)

<a id="struct-gpscorrectorconfig"></a>

## `struct GpsCorrectorConfig`

```cpp
struct GpsCorrectorConfig
```

Tuning for GpsCorrector. Every default is PROVISIONAL — there is no robot yet, and each one carries its A4 Hardware Assumptions Register entry. E2 proves the corrector's LOGIC against A3's hostile GPS; the magnitudes in that model are themselves guesses, and R4 measures both.

*struct, declared at [`include/shulib/localization/gps_corrector.hpp:119`](../../include/shulib/localization/gps_corrector.hpp#L119).*

<a id="gpscorrectorconfig-latency"></a>

### `GpsCorrectorConfig::latency`

```cpp
units::Time latency{0.05}
```

End-to-end delay between the moment a fix describes and the moment it can be read (camera exposure + solve + transport). PROVISIONAL (A4: HA-30) — invented, ≈50 ms.

*field, declared at [`include/shulib/localization/gps_corrector.hpp:122`](../../include/shulib/localization/gps_corrector.hpp#L122).*

<a id="gpscorrectorconfig-rmstrustfactor"></a>

### `GpsCorrectorConfig::rmsTrustFactor`

```cpp
double rmsTrustFactor = 2.0
```

Multiplier applied to the device's self-reported rms to get the 1σ actually used. > 1 because A4 register HA-29 records that a sensor's self-estimate and its real error are different numbers and the gap must be survived. PROVISIONAL (A4: HA-61).

*field, declared at [`include/shulib/localization/gps_corrector.hpp:126`](../../include/shulib/localization/gps_corrector.hpp#L126).*

<a id="gpscorrectorconfig-minpositionstddev"></a>

### `GpsCorrectorConfig::minPositionStdDev`

```cpp
units::Length minPositionStdDev{0.5}
```

Floor on the measurement 1σ. Without it, a device reporting ~0 error produces an arbitrarily tight gate that rejects everything including itself. PROVISIONAL (A4: HA-62).

*field, declared at [`include/shulib/localization/gps_corrector.hpp:129`](../../include/shulib/localization/gps_corrector.hpp#L129).*

<a id="gpscorrectorconfig-maxreportedrms"></a>

### `GpsCorrectorConfig::maxReportedRms`

```cpp
units::Length maxReportedRms{6.0}
```

Decline a fix whose REPORTED rms exceeds this — the sensor saying "I can see, badly". Without it a fix claiming 99" of error is still folded: the gate widens to accept it and the confidence shrinks to almost nothing, so the estimate barely moves, but the Localizer still reports quality class "Corrected" and the run looks anchored when it is not. PROVISIONAL (A4: HA-63).

*field, declared at [`include/shulib/localization/gps_corrector.hpp:135`](../../include/shulib/localization/gps_corrector.hpp#L135).*

<a id="gpscorrectorconfig-maxyawrate"></a>

### `GpsCorrectorConfig::maxYawRate`

```cpp
units::AngularVelocity maxYawRate{3.0}
```

Decline any fix taken while the yaw rate exceeds this. During a fast spin the lever-arm removal done at the HAL edge is at its most wrong (the sensor is swinging through an arc at ω·r, and its heading and position are sampled at slightly different instants), and the latency compensation cannot recover a rotation it did not see. PROVISIONAL (A4: HA-64).

*field, declared at [`include/shulib/localization/gps_corrector.hpp:140`](../../include/shulib/localization/gps_corrector.hpp#L140).*

<a id="gpscorrectorconfig-gatesigma"></a>

### `GpsCorrectorConfig::gateSigma`

```cpp
double gateSigma = 4.0
```

Gate width in units of σ_eff. PROVISIONAL (A4: HA-65).

*field, declared at [`include/shulib/localization/gps_corrector.hpp:142`](../../include/shulib/localization/gps_corrector.hpp#L142).*

<a id="gpscorrectorconfig-postfixstddev"></a>

### `GpsCorrectorConfig::postFixStdDev`

```cpp
units::Length postFixStdDev{1.0}
```

The estimate's position 1σ immediately after this source's fix is folded — the floor of σ_dr, so confidence is never 0 (a 0-confidence proposal is screened out by the Localizer and would read as "no proposal at all"). PROVISIONAL (A4: HA-66).

*field, declared at [`include/shulib/localization/gps_corrector.hpp:146`](../../include/shulib/localization/gps_corrector.hpp#L146).*

<a id="gpscorrectorconfig-driftstddevperinch"></a>

### `GpsCorrectorConfig::driftStdDevPerInch`

```cpp
double driftStdDevPerInch = 0.02
```

Growth of the dead-reckoning 1σ per inch travelled since this source's last fix. This is the anti-lockout term (header note). PROVISIONAL (A4: HA-67).

*field, declared at [`include/shulib/localization/gps_corrector.hpp:149`](../../include/shulib/localization/gps_corrector.hpp#L149).*

<a id="class-gpscorrector"></a>

## `class GpsCorrector`

```cpp
class GpsCorrector final : public ICorrector
```

The V5 GPS as an ICorrector — the first thing in the library that can tell the estimate it is wrong. Each propose() either offers an ABSOLUTE field POSITION with a confidence, or declines and says why on CorrectionProposal::selfAudit, so a run with no strip under it (Driving Skills) reads as a diagnosable state rather than an idle estimator. Three boundaries it holds to: it never returns a heading (the PREDICTED IMU heading rides back out unchanged and providesHeading stays false); it never snaps, because it only ever proposes and the fusion policy owns how far the estimate actually moves; and it does no frame conversion and no lever-arm removal, both of which the HAL edge has already done.  STATEFUL, and on EVERY tick: the predicted-position history ring and the dead-reckon distance that widens the gate advance even on ticks with no fix — that is precisely when they must. A given fix is folded exactly once; re-reading it declines as stale. PROPOSE() never throws and never allocates — the history ring is fixed-capacity and the tick path carries no checks. The CONSTRUCTOR is the opposite: it validates every GpsCorrectorConfig field with SHULIB_PRECONDITION, which throws PreconditionError under both shipped policies, so a config assembled from tuning input has to be guarded where it is built, not on the tick.

*class, declared at [`include/shulib/localization/gps_corrector.hpp:168`](../../include/shulib/localization/gps_corrector.hpp#L168).*

<a id="gpscorrector-khistory"></a>

### `GpsCorrector::kHistory`

```cpp
static constexpr std::size_t kHistory = 64
```

Ticks of predicted-position history kept for latency compensation. 64 ticks is ~0.64 s at 100 Hz against a ~50 ms latency — deep enough that a stalled loop or a slower control rate still finds the capture instant inside the ring. Fixed capacity: the hot path never allocates.

*field, declared at [`include/shulib/localization/gps_corrector.hpp:174`](../../include/shulib/localization/gps_corrector.hpp#L174).*

<a id="gpscorrector-gpscorrector"></a>

### `GpsCorrector::GpsCorrector`

```cpp
GpsCorrector(hal::IClock& clock, hal::IGps& gps, hal::IImu& imu, const GpsCorrectorConfig& config = {}, const char* name = "gps")
```

`clock`, `gps` and `imu` are non-owning references that must outlive this corrector. `name` is the stable telemetry id reported by name() and stamped into AppliedCorrection::source, so per-source dead-reckon accounting can say WHICH source went quiet.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:180`](../../include/shulib/localization/gps_corrector.hpp#L180).*

<a id="gpscorrector-propose"></a>

### `GpsCorrector::propose`

```cpp
[[nodiscard]] CorrectionProposal propose(const math::Pose2d& predicted, units::Time /*dt*/) override
```

One tick of the sequence in the header note. Never throws, never allocates; `dt` is unused because this corrector timestamps from the injected clock, which is authoritative and monotonic where a per-tick dt is a difference the Localizer already took.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:203`](../../include/shulib/localization/gps_corrector.hpp#L203).*

<a id="gpscorrector-name"></a>

### `GpsCorrector::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

Stable telemetry id — also what AppliedCorrection::source reports when this corrector is the reason the tick dead-reckoned.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:324`](../../include/shulib/localization/gps_corrector.hpp#L324).*

<a id="gpscorrector-lastverdict"></a>

### `GpsCorrector::lastVerdict`

```cpp
[[nodiscard]] diag::GateReason lastVerdict() const noexcept
```

What this corrector decided on the most recent propose() call.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:329`](../../include/shulib/localization/gps_corrector.hpp#L329).*

<a id="gpscorrector-acceptedfixes"></a>

### `GpsCorrector::acceptedFixes`

```cpp
[[nodiscard]] std::uint32_t acceptedFixes() const noexcept
```

Fixes proposed to the fusion policy since construction.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:331`](../../include/shulib/localization/gps_corrector.hpp#L331).*

<a id="gpscorrector-nofixticks"></a>

### `GpsCorrector::noFixTicks`

```cpp
[[nodiscard]] std::uint32_t noFixTicks() const noexcept
```

Ticks the source had no usable fix at all — off the strip, disconnected, or serving a non-finite read. This is the number that says "Driving Skills" out loud.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:334`](../../include/shulib/localization/gps_corrector.hpp#L334).*

<a id="gpscorrector-staleticks"></a>

### `GpsCorrector::staleTicks`

```cpp
[[nodiscard]] std::uint32_t staleTicks() const noexcept
```

Ticks that re-read a sample already folded (the ~50 ms camera cadence against a ~100 Hz loop, so a healthy run spends MOST of its ticks here).

*function, declared at [`include/shulib/localization/gps_corrector.hpp:337`](../../include/shulib/localization/gps_corrector.hpp#L337).*

<a id="gpscorrector-qualityrejects"></a>

### `GpsCorrector::qualityRejects`

```cpp
[[nodiscard]] std::uint32_t qualityRejects() const noexcept
```

Fresh fixes declined because the device's own reported error was too large.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:339`](../../include/shulib/localization/gps_corrector.hpp#L339).*

<a id="gpscorrector-yawraterejects"></a>

### `GpsCorrector::yawRateRejects`

```cpp
[[nodiscard]] std::uint32_t yawRateRejects() const noexcept
```

Fresh fixes declined because the robot was spinning too fast to trust them.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:341`](../../include/shulib/localization/gps_corrector.hpp#L341).*

<a id="gpscorrector-innovationrejects"></a>

### `GpsCorrector::innovationRejects`

```cpp
[[nodiscard]] std::uint32_t innovationRejects() const noexcept
```

Fresh fixes declined by the normalized-innovation gate.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:343`](../../include/shulib/localization/gps_corrector.hpp#L343).*

<a id="gpscorrector-travelsincefix"></a>

### `GpsCorrector::travelSinceFix`

```cpp
[[nodiscard]] units::Length travelSinceFix() const noexcept
```

Distance the prediction has travelled since this source last proposed a fix — the input to the anti-lockout term, exposed so a test can prove the widening is real.

*function, declared at [`include/shulib/localization/gps_corrector.hpp:346`](../../include/shulib/localization/gps_corrector.hpp#L346).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 95 lines, click to expand</summary>

```text

 GpsCorrector — the FIRST REAL corrector (master plan §6/§8; WS5, chunk E2). Everything
 before this chunk dead-reckoned: odometry and the IMU counted the robot's own motion and
 nothing in the library could ever tell the estimate it was wrong. This is the code that can.

 It implements ICorrector behind the exact signature the seam has had since M2 — PULL, not
 push: the Localizer calls propose(predicted, dt) each tick with the odometry-predicted pose,
 and this class answers with an absolute field position and how much to trust it, or with
 nothing at all.

 ── WHAT IT DOES, AND IN WHAT ORDER ────────────────────────────────────────────────────────
   1. record the predicted position in a short history ring (needed for latency, below);
   2. no fix?              → decline, RejectedNoFix        (the Driving-Skills path)
   3. non-finite read?     → decline, RejectedNoFix        (F4 backstop; never trust a NaN)
   4. sample unchanged?    → decline, RejectedStaleFix     (the double-count guard)
   5. claimed error huge?  → decline, RejectedSensorQuality
   6. spinning fast?       → decline, RejectedHighYawRate
   7. compensate for sensor latency using the odometry history;
   8. normalized-innovation gate → decline, RejectedNormalizedInnovation
   9. otherwise propose, with an adaptive σ and a confidence derived from it.
 Every decline carries its reason out on CorrectionProposal::selfAudit, which is how an
 off-strip run becomes visible in the blackbox instead of looking like an idle estimator.

 ── WHAT IT DELIBERATELY DOES NOT DO ───────────────────────────────────────────────────────
 * NO FRAME CONVERSION and NO LEVER-ARM REMOVAL. Both belong to the HAL edge and are already
   done by the time a pose reaches here: hal/gps.hpp documents IGps::pose() as "the
   robot-CENTER pose (lever-arm corrected)", and hal/gps_conversion.hpp calls itself "the ONE
   place the VEX GPS frame becomes shulib's canonical frame" and says ONE owner for the lever
   arm, naming double-subtraction as the failure. Doing either here would be invisible in host
   tests (FakeGps stores a centre pose, so a second removal would just be wrong by the arm)
   and silently wrong on the robot, where the R1 adapter is contractually obliged to have done
   it already. E2's obligation to those two conversions is PROOF — independent oracles at
   several headings in test/gps_conversion_test.cpp — not a second implementation.
 * NO HEADING. The V5 GPS reports one; this class never lets it out. CorrectionProposal::
   fieldPose carries the PREDICTED (IMU) heading, so even a future policy that read
   fieldPose.heading() would read the IMU's answer, and providesHeading stays false. Heading
   is IMU-owned by decision #4 and the Localizer re-stamps it as the last write; a GPS-heading
   path is E3/E4's deliberate additive decision (a headingNudge on FusionResult), not a side
   effect of E2.
 * NO SNAP, ever. This class only ever PROPOSES an absolute pose; how far the estimate moves
   toward it is the fusion policy's bounded, per-tick-clamped nudge (§13 #4). There is no code
   path here that writes a pose.

 ── THE GATE IS A NORMALIZED INNOVATION, NOT A MAHALANOBIS DISTANCE (chunk tension T1) ──────
 The gate computes
        ν = |z − predicted| / σ_eff,      σ_eff = hypot(σ_meas, σ_dr)
 and declines when ν > gateSigma. A Mahalanobis distance normalises by the innovation
 covariance S = H·P·Hᵀ + R with P ESTIMATED BY A FILTER; the complementary tier estimates no P
 and σ_dr below is a hand-written heuristic over an invented growth rate. So this class never
 raises GateReason::RejectedMahalanobis and never writes GateAudit::mahalanobis — E4's EKF
 earns that number, and one field holding both an earned and an asserted quantity would make
 the difference between them invisible. GateReason::RejectedNormalizedInnovation says what
 actually happened.

 WHY σ_dr EXISTS (and why a fixed inch threshold was rejected). σ_meas alone gives a gate a
 couple of inches wide. After twenty feet of dead-reckoning the estimate can be further off
 than that, so a truthful fix would look outrageous and be rejected — and every subsequent
 one too, because nothing else can fix the estimate. That is gate lockout: the GPS goes
 silently dead exactly when it is worth the most. Widening the gate with the distance travelled
 since this source last had a fix is what prevents it, and it is tested directly (a fix that a
 zero-growth build rejects is accepted after a long dead-reckon).

 ── ADAPTIVE R, AND WHY THE DEVICE'S CLAIM IS NOT TAKEN AT FACE VALUE ───────────────────────
   σ_meas     = max(rmsTrustFactor · rmsError(), minPositionStdDev)
   confidence = σ_dr² / (σ_dr² + σ_meas²)
 The confidence is the scalar Kalman gain of a one-dimensional update, which is the right
 shape for the complementary tier's pull weight: trust the fix more when the estimate is
 uncertain, less when the sensor says it is uncertain. rmsTrustFactor exists because A4
 register HA-29 records that the device's self-estimate and its real error are different
 numbers; minPositionStdDev exists so a device claiming ~0 error cannot produce an arbitrarily
 tight gate. Both are provisional (see the register entries on each field) — E2 proves the
 LOGIC; R4 measures the constants.

 ── THE DOUBLE-COUNT GUARD ─────────────────────────────────────────────────────────────────
 The GPS camera produces a new fix every ~50 ms (HA-28) while the control loop runs at ~100 Hz,
 so a corrector that folds every read treats one measurement as five independent ones — the
 hazard sim/hostile/gps_hostility.hpp names in its header. This class folds a sample once: a
 read whose position and reported error are unchanged is declined as RejectedStaleFix.
 HONEST LIMITATION: with a NOISELESS gps (the identity degradation model, or a FakeGps whose
 pose is set once) every read after the first is byte-identical, so correction happens once and
 then stops. That is the contract behaving as written — an unchanged number carries no new
 information — but it surprises anyone who expects continuous correction from a static fake.
 A real device jitters (HA-26) and a hostile-model run produces a fresh sample every cadence.

 ── LATENCY ────────────────────────────────────────────────────────────────────────────────
 A fix describes where the robot WAS, roughly 50 ms ago (HA-30). Applied as if it described
 where the robot IS, it drags the estimate backwards along the direction of travel — at 40 in/s
 that is a systematic 2-inch lag, larger than the sensor's own noise. So the fix is carried
 forward by the odometry travelled since it was captured: z = gpsPose + (P(now) − P(capture)),
 read out of the history ring. Odometry is excellent over 50 ms even when it is poor over 50
 seconds, which is exactly what this needs.

 Pure w.r.t. its injected handles (clock, gps, imu) and PROS-free: it is built against the HAL
 seam, so the same code runs against FakeGps on the host and the R1 pros::Gps adapter on the
 robot. propose() never throws and never allocates.
```

</details>
