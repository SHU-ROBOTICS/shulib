<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/drive_characterization.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drive_characterization.hpp`

DriveCharacterization — what the GAINS and REST steps of shulib Calibrate MEASURE, as data.

This header declares **4** types (40 members).

Extracted from [`include/shulib/calibration/drive_characterization.hpp`](../../include/shulib/calibration/drive_characterization.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class CharacterizationSource`](#enum-class-characterizationsource)
  - [`Unset`](#characterizationsource-unset)
  - [`Measured`](#characterizationsource-measured)
- [`struct SideCharacterization`](#struct-sidecharacterization)
  - [`gains`](#sidecharacterization-gains)
  - [`residualRms`](#sidecharacterization-residualrms)
  - [`alpha`](#sidecharacterization-alpha)
  - [`voltsMin`](#sidecharacterization-voltsmin)
  - [`voltsMax`](#sidecharacterization-voltsmax)
  - [`samples`](#sidecharacterization-samples)
  - [`members`](#sidecharacterization-members)
  - [`kABelowResolution`](#sidecharacterization-kabelowresolution)
- [`struct RestSummary`](#struct-restsummary)
  - [`driftRadPerS`](#restsummary-driftradpers)
  - [`driftSigmaWhite`](#restsummary-driftsigmawhite)
  - [`driftSigmaWalk`](#restsummary-driftsigmawalk)
  - [`structureRatio`](#restsummary-structureratio)
  - [`headingSigmaRad`](#restsummary-headingsigmarad)
  - [`stepRateSigma`](#restsummary-stepratesigma)
  - [`yawRateMean`](#restsummary-yawratemean)
  - [`yawRateSigma`](#restsummary-yawratesigma)
  - [`maxHeadingStepRad`](#restsummary-maxheadingsteprad)
  - [`durationS`](#restsummary-durations)
  - [`speedSigma`](#restsummary-speedsigma)
  - [`encoderTurnRad`](#restsummary-encoderturnrad)
  - [`encoderTurnMaxRad`](#restsummary-encoderturnmaxrad)
  - [`encoderMoveMaxIn`](#restsummary-encodermovemaxin)
  - [`samples`](#restsummary-samples)
  - [`timing`](#restsummary-timing)
  - [`driftDegPerMin`](#restsummary-driftdegpermin)
  - [`driftTurnBiasBound`](#restsummary-driftturnbiasbound)
  - [`structureRatioDefined`](#restsummary-structureratiodefined)
- [`struct DriveCharacterization`](#struct-drivecharacterization)
  - [`source`](#drivecharacterization-source)
  - [`hasGains`](#drivecharacterization-hasgains)
  - [`hasRest`](#drivecharacterization-hasrest)
  - [`left`](#drivecharacterization-left)
  - [`right`](#drivecharacterization-right)
  - [`rampTiming`](#drivecharacterization-ramptiming)
  - [`rest`](#drivecharacterization-rest)
  - [`provenance`](#drivecharacterization-provenance)
  - [`combined`](#drivecharacterization-combined)
  - [`kVAsymmetry`](#drivecharacterization-kvasymmetry)
  - [`valid`](#drivecharacterization-valid)

<a id="enum-class-characterizationsource"></a>

## `enum class CharacterizationSource`

```cpp
enum class CharacterizationSource : std::uint8_t
```

Where a characterization came from. APPEND-ONLY and persisted: a new source goes at the end.

*enum class, declared at [`include/shulib/calibration/drive_characterization.hpp:51`](../../include/shulib/calibration/drive_characterization.hpp#L51).*

<a id="characterizationsource-unset"></a>

### `CharacterizationSource::Unset`

```cpp
Unset = 0
```

nothing measured — the placeholders (HA-45) are in force

*enumerator, declared at [`include/shulib/calibration/drive_characterization.hpp:52`](../../include/shulib/calibration/drive_characterization.hpp#L52).*

<a id="characterizationsource-measured"></a>

### `CharacterizationSource::Measured`

```cpp
Measured = 1
```

measured by the robot, in shulib Calibrate (GAINS and/or REST)

*enumerator, declared at [`include/shulib/calibration/drive_characterization.hpp:53`](../../include/shulib/calibration/drive_characterization.hpp#L53).*

<a id="struct-sidecharacterization"></a>

## `struct SideCharacterization`

```cpp
struct SideCharacterization
```

One drive side's fitted gains and the facts behind them (calibration::FitResult, kept).

*struct, declared at [`include/shulib/calibration/drive_characterization.hpp:57`](../../include/shulib/calibration/drive_characterization.hpp#L57).*

<a id="sidecharacterization-gains"></a>

### `SideCharacterization::gains`

```cpp
control::FeedforwardGains gains{}
```

kS (V), kV (V·s/in, per in/s of the side's SURFACE speed in the drivetrain record's scale), kA (V·s²/in — RECORDED, NOT APPLIED; see the header).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:60`](../../include/shulib/calibration/drive_characterization.hpp#L60).*

<a id="sidecharacterization-residualrms"></a>

### `SideCharacterization::residualRms`

```cpp
double residualRms = 0.0
```

The fit's residual RMS, in/s.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:62`](../../include/shulib/calibration/drive_characterization.hpp#L62).*

<a id="sidecharacterization-alpha"></a>

### `SideCharacterization::alpha`

```cpp
double alpha = 0.0
```

The fitted α (the discrete law's decay per tick; feedforward_fit.hpp).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:64`](../../include/shulib/calibration/drive_characterization.hpp#L64).*

<a id="sidecharacterization-voltsmin"></a>

### `SideCharacterization::voltsMin`

```cpp
double voltsMin = 0.0
```

The smallest |volts| the fit admitted — the low end of the range the gains were fitted over.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:66`](../../include/shulib/calibration/drive_characterization.hpp#L66).*

<a id="sidecharacterization-voltsmax"></a>

### `SideCharacterization::voltsMax`

```cpp
double voltsMax = 0.0
```

The largest |volts| the fit admitted — the high end (A4 register HA-171: they must hold beyond it).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:69`](../../include/shulib/calibration/drive_characterization.hpp#L69).*

<a id="sidecharacterization-samples"></a>

### `SideCharacterization::samples`

```cpp
std::uint32_t samples = 0
```

How many samples the fit admitted.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:71`](../../include/shulib/calibration/drive_characterization.hpp#L71).*

<a id="sidecharacterization-members"></a>

### `SideCharacterization::members`

```cpp
std::uint8_t members = 0
```

How many members the side had when it was measured: a later boot with fewer present raises kS roughly as 1/(members present) (A4 register HA-174), which Part B's gains line notes.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:74`](../../include/shulib/calibration/drive_characterization.hpp#L74).*

<a id="sidecharacterization-kabelowresolution"></a>

### `SideCharacterization::kABelowResolution`

```cpp
bool kABelowResolution = false
```

True when kA was below the fit's resolution and is therefore exactly 0.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:76`](../../include/shulib/calibration/drive_characterization.hpp#L76).*

<a id="struct-restsummary"></a>

## `struct RestSummary`

```cpp
struct RestSummary
```

The REST step's summary (rest_capture.hpp computes it; the header there derives every formula). Every angle is in RADIANS, every rate in rad/s, every speed in in/s, every time in seconds.

*struct, declared at [`include/shulib/calibration/drive_characterization.hpp:81`](../../include/shulib/calibration/drive_characterization.hpp#L81).*

<a id="restsummary-driftradpers"></a>

### `RestSummary::driftRadPerS`

```cpp
double driftRadPerS = 0.0
```

The heading's drift: the least-squares SLOPE of the unwrapped heading against time, rad/s. A PER-BOOT number — never applied as a correction (A4 register HA-173).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:84`](../../include/shulib/calibration/drive_characterization.hpp#L84).*

<a id="restsummary-driftsigmawhite"></a>

### `RestSummary::driftSigmaWhite`

```cpp
double driftSigmaWhite = 0.0
```

The slope's standard error IF the heading noise is WHITE: σ_line / √(Σ(t − t̄)²), which is σ_line·√12/(√N·T) for evenly spaced ticks. rad/s.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:87`](../../include/shulib/calibration/drive_characterization.hpp#L87).*

<a id="restsummary-driftsigmawalk"></a>

### `RestSummary::driftSigmaWalk`

```cpp
double driftSigmaWalk = 0.0
```

The slope's standard error IF the heading is a RANDOM WALK (integrated rate noise): σ_Δ·√(1.2/(dt·T)). rad/s. Neither is picked automatically: each overstates the other case badly (rest_capture.hpp; HA-173).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:91`](../../include/shulib/calibration/drive_characterization.hpp#L91).*

<a id="restsummary-structureratio"></a>

### `RestSummary::structureRatio`

```cpp
double structureRatio = 0.0
```

The structure ratio ρ = σ_Δ/(√2·σ_line): ≈ 1 for white heading noise, ≪ 1 for a random walk. UNDEFINED when the heading has no noise about its line (σ_line = 0 — the quiet host test plant, or an IMU that reports a constant): it is then reported as 0, which is NOT random-walk evidence — read structureRatioDefined() first. A rotation the encoders saw (encoderTurnMaxRad) also pulls ρ toward 0: the step it puts in the heading inflates σ_line and not σ_Δ (A4 register HA-173).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:98`](../../include/shulib/calibration/drive_characterization.hpp#L98).*

<a id="restsummary-headingsigmarad"></a>

### `RestSummary::headingSigmaRad`

```cpp
double headingSigmaRad = 0.0
```

Heading noise: σ_line, the standard deviation of the residual about the fitted LINE (never about the mean — a drift inflates that), rad.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:101`](../../include/shulib/calibration/drive_characterization.hpp#L101).*

<a id="restsummary-stepratesigma"></a>

### `RestSummary::stepRateSigma`

```cpp
double stepRateSigma = 0.0
```

σ_Δ/dt: the standard deviation of the per-tick heading step about its mean, over the mean dt — the rate noise SettledUtil actually differentiates. rad/s.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:104`](../../include/shulib/calibration/drive_characterization.hpp#L104).*

<a id="restsummary-yawratemean"></a>

### `RestSummary::yawRateMean`

```cpp
double yawRateMean = 0.0
```

The IMU's yaw-rate channel: its mean, rad/s — a second, independent drift estimate.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:106`](../../include/shulib/calibration/drive_characterization.hpp#L106).*

<a id="restsummary-yawratesigma"></a>

### `RestSummary::yawRateSigma`

```cpp
double yawRateSigma = 0.0
```

The yaw-rate channel's standard deviation, rad/s.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:108`](../../include/shulib/calibration/drive_characterization.hpp#L108).*

<a id="restsummary-maxheadingsteprad"></a>

### `RestSummary::maxHeadingStepRad`

```cpp
double maxHeadingStepRad = 0.0
```

The largest |tick-to-tick heading step| seen, rad (A4 register HA-160 evidence).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:110`](../../include/shulib/calibration/drive_characterization.hpp#L110).*

<a id="restsummary-durations"></a>

### `RestSummary::durationS`

```cpp
double durationS = 0.0
```

How long the capture ran, seconds (first sample to last).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:112`](../../include/shulib/calibration/drive_characterization.hpp#L112).*

<a id="restsummary-speedsigma"></a>

### `RestSummary::speedSigma`

```cpp
double speedSigma[2] = {0.0, 0.0}
```

Each side's median encoder speed at rest: its standard deviation, in/s. [0] LEFT, [1] RIGHT.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:114`](../../include/shulib/calibration/drive_characterization.hpp#L114).*

<a id="restsummary-encoderturnrad"></a>

### `RestSummary::encoderTurnRad`

```cpp
double encoderTurnRad = 0.0
```

The encoders' rotation (dR − dL)/W at the capture's LAST sample, from its first, rad (positive = CCW). A robot that really turned — a hand on it — turned the IMU's heading too, and the drift's slope cannot tell that from drift (rest_capture.hpp's header).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:118`](../../include/shulib/calibration/drive_characterization.hpp#L118).*

<a id="restsummary-encoderturnmaxrad"></a>

### `RestSummary::encoderTurnMaxRad`

```cpp
double encoderTurnMaxRad = 0.0
```

The largest |encoder rotation| from the first sample, over the whole capture, rad — under RestCaptureConfig::turnBound, or the capture would have been refused as Turned. What it can have done to the drift is driftTurnBiasBound().

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:122`](../../include/shulib/calibration/drive_characterization.hpp#L122).*

<a id="restsummary-encodermovemaxin"></a>

### `RestSummary::encoderMoveMaxIn`

```cpp
double encoderMoveMaxIn = 0.0
```

The largest |encoder translation| — the centre's travel (dL + dR)/2 — from the first sample, over the whole capture, inches (under moveBound, or it was refused as Moved).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:125`](../../include/shulib/calibration/drive_characterization.hpp#L125).*

<a id="restsummary-samples"></a>

### `RestSummary::samples`

```cpp
std::uint32_t samples = 0
```

How many samples the capture took.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:127`](../../include/shulib/calibration/drive_characterization.hpp#L127).*

<a id="restsummary-timing"></a>

### `RestSummary::timing`

```cpp
diag::TickTimingSummary timing{}
```

The capture's loop timing.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:129`](../../include/shulib/calibration/drive_characterization.hpp#L129).*

<a id="restsummary-driftdegpermin"></a>

### `RestSummary::driftDegPerMin`

```cpp
[[nodiscard]] double driftDegPerMin() const noexcept
```

The drift in degrees per minute — the unit HA-20 states its bound in.

*function, declared at [`include/shulib/calibration/drive_characterization.hpp:132`](../../include/shulib/calibration/drive_characterization.hpp#L132).*

<a id="restsummary-driftturnbiasbound"></a>

### `RestSummary::driftTurnBiasBound`

```cpp
[[nodiscard]] double driftTurnBiasBound() const noexcept
```

The most the rotation the ENCODERS saw can have moved driftRadPerS, rad/s: 3·encoderTurnMaxRad/durationS. A heading offset bounded by M over [0, T] moves a least-squares slope by at most M·Σ|t − t̄|/Σ(t − t̄)², which is 3·M/T for evenly spaced ticks (rest_capture.hpp's header derives it; one step of size A at mid-capture moves it by 1.5·A/T). A capture is not refused for a rotation under turnBound, so this is how a reader tells a measured drift from a nudge: a drift not well clear of this bound says nothing (A4 register HA-170, HA-173). 0 for a robot that never turned; 0 when durationS is 0.

*function, declared at [`include/shulib/calibration/drive_characterization.hpp:143`](../../include/shulib/calibration/drive_characterization.hpp#L143).*

<a id="restsummary-structureratiodefined"></a>

### `RestSummary::structureRatioDefined`

```cpp
[[nodiscard]] bool structureRatioDefined() const noexcept
```

True when structureRatio is defined — the heading had SOME noise about its line (headingSigmaRad > 0). When false, structureRatio is reported as 0 and means nothing.

*function, declared at [`include/shulib/calibration/drive_characterization.hpp:149`](../../include/shulib/calibration/drive_characterization.hpp#L149).*

<a id="struct-drivecharacterization"></a>

## `struct DriveCharacterization`

```cpp
struct DriveCharacterization
```

Everything shulib Calibrate's GAINS and REST steps measured about one drivetrain (header). Part B's record persists it; Part B's builder consumes `combined()`.

*struct, declared at [`include/shulib/calibration/drive_characterization.hpp:154`](../../include/shulib/calibration/drive_characterization.hpp#L154).*

<a id="drivecharacterization-source"></a>

### `DriveCharacterization::source`

```cpp
CharacterizationSource source = CharacterizationSource::Unset
```

Where the numbers came from. Unset (the default) is "nothing measured".

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:156`](../../include/shulib/calibration/drive_characterization.hpp#L156).*

<a id="drivecharacterization-hasgains"></a>

### `DriveCharacterization::hasGains`

```cpp
bool hasGains = false
```

True when the GAINS section (left, right, rampTiming) holds a measurement.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:158`](../../include/shulib/calibration/drive_characterization.hpp#L158).*

<a id="drivecharacterization-hasrest"></a>

### `DriveCharacterization::hasRest`

```cpp
bool hasRest = false
```

True when the REST section (rest) holds a measurement.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:160`](../../include/shulib/calibration/drive_characterization.hpp#L160).*

<a id="drivecharacterization-left"></a>

### `DriveCharacterization::left`

```cpp
SideCharacterization left{}
```

The LEFT side's gains.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:162`](../../include/shulib/calibration/drive_characterization.hpp#L162).*

<a id="drivecharacterization-right"></a>

### `DriveCharacterization::right`

```cpp
SideCharacterization right{}
```

The RIGHT side's gains.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:164`](../../include/shulib/calibration/drive_characterization.hpp#L164).*

<a id="drivecharacterization-ramptiming"></a>

### `DriveCharacterization::rampTiming`

```cpp
diag::TickTimingSummary rampTiming{}
```

The GAINS run's loop timing, every powered and stopping tick of it.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:166`](../../include/shulib/calibration/drive_characterization.hpp#L166).*

<a id="drivecharacterization-rest"></a>

### `DriveCharacterization::rest`

```cpp
RestSummary rest{}
```

The REST step's summary.

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:168`](../../include/shulib/calibration/drive_characterization.hpp#L168).*

<a id="drivecharacterization-provenance"></a>

### `DriveCharacterization::provenance`

```cpp
char provenance[96]{}
```

The build stamp or session note, NUL-terminated (a fixed buffer, not a pointer — the whole struct is written to a card as it stands).

*field, declared at [`include/shulib/calibration/drive_characterization.hpp:171`](../../include/shulib/calibration/drive_characterization.hpp#L171).*

<a id="drivecharacterization-combined"></a>

### `DriveCharacterization::combined`

```cpp
[[nodiscard]] constexpr control::FeedforwardGains combined() const noexcept
```

D2's combination: the field-by-field MEAN of the two sides' gains. Its kA is carried, and no consumer applies it (header). Meaningful only when hasGains.

*function, declared at [`include/shulib/calibration/drive_characterization.hpp:175`](../../include/shulib/calibration/drive_characterization.hpp#L175).*

<a id="drivecharacterization-kvasymmetry"></a>

### `DriveCharacterization::kVAsymmetry`

```cpp
[[nodiscard]] double kVAsymmetry() const noexcept
```

|kV_left − kV_right| / their mean — how far apart the sides are. 0 when both are 0.

*function, declared at [`include/shulib/calibration/drive_characterization.hpp:182`](../../include/shulib/calibration/drive_characterization.hpp#L182).*

<a id="drivecharacterization-valid"></a>

### `DriveCharacterization::valid`

```cpp
[[nodiscard]] bool valid(char* why, std::size_t cap) const
```

True when the characterization holds together, else false with the first offending FIELD named in `why` (NUL-terminated, cut to `cap`; may be null) — ASCII, no '%': * `source` is a known value; Unset carries no section, Measured at least one; * a GAINS section: per side every number finite, kV > 0, kS >= 0, kA >= 0, residual >= 0, 0 <= voltsMin <= voltsMax <= 12, samples > 0, members in 1..21, and a kA below resolution is exactly 0; the ramp's timing summary valid; * a REST section: every number finite, every sigma, ratio, step and largest encoder motion >= 0, the final encoder rotation no larger than the largest, the duration > 0, samples > 0, its timing summary valid; * provenance NUL-terminated inside its buffer. It does not judge whether the numbers are PLAUSIBLE — FitConfig's ranges did that when they were measured, and a record must not be refused later because a range moved.

*function, declared at [`include/shulib/calibration/drive_characterization.hpp:199`](../../include/shulib/calibration/drive_characterization.hpp#L199).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 34 lines</summary>

```text

 DriveCharacterization — what the GAINS and REST steps of shulib Calibrate MEASURE, as data (chunk
 R5a Part A, 2026-09-29). drive_characterizer.hpp (the GAINS step: each side's kS/kV/kA) and
 rest_capture.hpp (the REST step: the IMU's drift and noise, the encoders' noise at rest, the loop's
 timing) produce it; Part B of R5a persists it on the SD card beside the drivetrain record, bound
 to that record, and teaches the robot builder to consume the gains. NOTHING CONSUMES IT YET: until
 Part B, no program reads it and every drive runs on the placeholder gains (A4 register HA-45).

 NOTHING IN IT HAS BEEN MEASURED ON A ROBOT. The host test plant fills it with the plant's own
 constants; a robot session (S2 gains, S3 noise) is what will put real numbers here.

 ── Why TWO sides, and what "combined" means (R5a's decision D2) ───────────────────────
 The motion layer applies ONE control::Feedforward to every wheel (feedforward.hpp; the command
 pipeline's step 7). The two sides are fitted and kept SEPARATELY anyway — a dragging side or a
 slipping member is a mechanical finding, and averaging it away before anyone saw it would hide
 it — and combined() is their field-by-field MEAN, which is what Part B hands the pipeline.
 kVAsymmetry() says how far apart the sides are; the characterizer warns past 15 %.
 OF THE MEAN, ONLY kS AND kV REACH A MOTOR: the pipeline calls the cruise overload
 Feedforward::calculate(v) and WheelSpeeds has no acceleration channel, so kA is RECORDED (for the
 S2 log and R6's plant back-fit, where τ = kA/kV) and NOT APPLIED (D2, D10).

 ── Laid out to be SERIALIZED (Part B's record) ─────────────────────────────────────
 Fixed-width fields, bounded arrays, no pointers, no std::string, no allocation, constexpr-
 constructible — config/robot_config.hpp's rule for everything a RobotConfig carries, because Part
 B embeds this struct in RobotConfig BY VALUE (that header's ruling B1: one representation, never a
 mirror). The section flags say which halves hold data, so a REST run can be merged into a record
 that already holds a GAINS run without erasing it (Part B's mergeSections). `source` and every
 enum here are APPEND-ONLY: their values are persisted.

 ── The one thing that must never be done with `rest` (A4 register HA-173) ───────────
 The drift is a PER-BOOT bias of the IMU (HA-20): one boot's number is wrong at the next boot. It is
 a sample for the register, reported and kept — NEVER applied as a heading correction.

 PROS-free, sim-free, allocation-free. valid() writes ASCII with no '%' (a panel may paint it).
```

</details>
