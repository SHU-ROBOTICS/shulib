<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/drive_characterizer.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drive_characterizer.hpp`

DriveCharacterizer — shulib Calibrate's GAINS step: the robot drives itself through four short, bounded segments while each drive side's kS, kV and kA are fitted on the brain.

This header declares **10** types (106 members), **3** free functions, and **1** constant.

Extracted from [`include/shulib/calibration/drive_characterizer.hpp`](../../include/shulib/calibration/drive_characterizer.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class SegmentKind`](#enum-class-segmentkind)
  - [`QuasiForward`](#segmentkind-quasiforward)
  - [`QuasiBackward`](#segmentkind-quasibackward)
  - [`StepForward`](#segmentkind-stepforward)
  - [`StepBackward`](#segmentkind-stepbackward)
- [`kSegmentCount`](#ksegmentcount) — *constant*
- [`segmentKindName`](#segmentkindname) — *free function*
- [`enum class SegmentEnd`](#enum-class-segmentend)
  - [`None`](#segmentend-none)
  - [`Distance`](#segmentend-distance)
  - [`RampTop`](#segmentend-ramptop)
  - [`Timeout`](#segmentend-timeout)
- [`segmentEndName`](#segmentendname) — *free function*
- [`enum class CharacterizerPhase`](#enum-class-characterizerphase)
  - [`Idle`](#characterizerphase-idle)
  - [`WaitImu`](#characterizerphase-waitimu)
  - [`Rest`](#characterizerphase-rest)
  - [`Driving`](#characterizerphase-driving)
  - [`Stopping`](#characterizerphase-stopping)
  - [`Done`](#characterizerphase-done)
  - [`Aborted`](#characterizerphase-aborted)
- [`enum class CharacterizerRefusal`](#enum-class-characterizerrefusal)
  - [`None`](#characterizerrefusal-none)
  - [`Aborted`](#characterizerrefusal-aborted)
  - [`ImuNotReady`](#characterizerrefusal-imunotready)
  - [`RestTimeout`](#characterizerrefusal-resttimeout)
  - [`CurveExceeded`](#characterizerrefusal-curveexceeded)
  - [`Fight`](#characterizerrefusal-fight)
  - [`MissedTick`](#characterizerrefusal-missedtick)
  - [`TooHot`](#characterizerrefusal-toohot)
  - [`StopTooLong`](#characterizerrefusal-stoptoolong)
  - [`FitRefused`](#characterizerrefusal-fitrefused)
- [`struct CharacterizerConfig`](#struct-characterizerconfig)
  - [`rampRate`](#characterizerconfig-ramprate)
  - [`rampMax`](#characterizerconfig-rampmax)
  - [`stepVolts`](#characterizerconfig-stepvolts)
  - [`segmentDistance`](#characterizerconfig-segmentdistance)
  - [`segmentTimeout`](#characterizerconfig-segmenttimeout)
  - [`asymRefuse`](#characterizerconfig-asymrefuse)
  - [`asymWarn`](#characterizerconfig-asymwarn)
  - [`stopBound`](#characterizerconfig-stopbound)
  - [`tempGuard`](#characterizerconfig-tempguard)
  - [`restFloor`](#characterizerconfig-restfloor)
  - [`restYawRateFloor`](#characterizerconfig-restyawratefloor)
  - [`restSettleTime`](#characterizerconfig-restsettletime)
  - [`restTimeout`](#characterizerconfig-resttimeout)
  - [`imuReadyTimeout`](#characterizerconfig-imureadytimeout)
  - [`commandTimeout`](#characterizerconfig-commandtimeout)
  - [`monitor`](#characterizerconfig-monitor)
  - [`fit`](#characterizerconfig-fit)
  - [`timing`](#characterizerconfig-timing)
  - [`curveGuard`](#characterizerconfig-curveguard)
  - [`sidewaysExcursion`](#characterizerconfig-sidewaysexcursion)
  - [`clearAheadAndBehind`](#characterizerconfig-clearaheadandbehind)
  - [`valid`](#characterizerconfig-valid)
- [`struct SegmentRecord`](#struct-segmentrecord)
  - [`kind`](#segmentrecord-kind)
  - [`end`](#segmentrecord-end)
  - [`stopped`](#segmentrecord-stopped)
  - [`seconds`](#segmentrecord-seconds)
  - [`lastVolts`](#segmentrecord-lastvolts)
  - [`admitted`](#segmentrecord-admitted)
  - [`excluded`](#segmentrecord-excluded)
  - [`travelIn`](#segmentrecord-travelin)
  - [`stopIn`](#segmentrecord-stopin)
  - [`batteryMin`](#segmentrecord-batterymin)
  - [`batteryMean`](#segmentrecord-batterymean)
  - [`maxTemperature`](#segmentrecord-maxtemperature)
  - [`encoderCurveRad`](#segmentrecord-encodercurverad)
  - [`imuYawRad`](#segmentrecord-imuyawrad)
  - [`maxHeadingStepRad`](#segmentrecord-maxheadingsteprad)
  - [`timing`](#segmentrecord-timing)
  - [`imuMinusEncoderRad`](#segmentrecord-imuminusencoderrad)
- [`struct CharacterizerReport`](#struct-characterizerreport)
  - [`segments`](#characterizerreport-segments)
  - [`segmentCount`](#characterizerreport-segmentcount)
  - [`fit`](#characterizerreport-fit)
  - [`combined`](#characterizerreport-combined)
  - [`kVAsymmetry`](#characterizerreport-kvasymmetry)
  - [`asymmetryWarning`](#characterizerreport-asymmetrywarning)
  - [`emptySegmentWarning`](#characterizerreport-emptysegmentwarning)
  - [`curveGuardRad`](#characterizerreport-curveguardrad)
  - [`timing`](#characterizerreport-timing)
- [`struct CharacterizerSample`](#struct-characterizersample)
  - [`valid`](#characterizersample-valid)
  - [`t`](#characterizersample-t)
  - [`segment`](#characterizersample-segment)
  - [`powered`](#characterizersample-powered)
  - [`volts`](#characterizersample-volts)
  - [`speed`](#characterizersample-speed)
  - [`position`](#characterizersample-position)
  - [`battery`](#characterizersample-battery)
  - [`maxCurrent`](#characterizersample-maxcurrent)
  - [`headingRad`](#characterizersample-headingrad)
- [`struct CharacterizerStatus`](#struct-characterizerstatus)
  - [`phase`](#characterizerstatus-phase)
  - [`segment`](#characterizerstatus-segment)
  - [`volts`](#characterizerstatus-volts)
  - [`travelIn`](#characterizerstatus-travelin)
  - [`curveRad`](#characterizerstatus-curverad)
  - [`stopIn`](#characterizerstatus-stopin)
  - [`atRest`](#characterizerstatus-atrest)
- [`class DriveCharacterizer`](#class-drivecharacterizer)
  - [`DriveCharacterizer`](#drivecharacterizer-drivecharacterizer)
  - [`DriveCharacterizer (overload 2)`](#drivecharacterizer-drivecharacterizer-2)
  - [`operator=`](#drivecharacterizer-operator-eq)
  - [`start`](#drivecharacterizer-start)
  - [`tick`](#drivecharacterizer-tick)
  - [`abort`](#drivecharacterizer-abort)
  - [`phase`](#drivecharacterizer-phase)
  - [`refusal`](#drivecharacterizer-refusal)
  - [`refusalPort`](#drivecharacterizer-refusalport)
  - [`reason`](#drivecharacterizer-reason)
  - [`warning`](#drivecharacterizer-warning)
  - [`report`](#drivecharacterizer-report)
  - [`status`](#drivecharacterizer-status)
  - [`lastSample`](#drivecharacterizer-lastsample)
  - [`config`](#drivecharacterizer-config)
  - [`result`](#drivecharacterizer-result)
- [`characterizerRefusalName`](#characterizerrefusalname) — *free function*

<a id="enum-class-segmentkind"></a>

## `enum class SegmentKind`

```cpp
enum class SegmentKind : std::uint8_t
```

The four segments, in the order they run. APPEND-ONLY (the value is printed and recorded).

*enum class, declared at [`include/shulib/calibration/drive_characterizer.hpp:157`](../../include/shulib/calibration/drive_characterizer.hpp#L157).*

<a id="segmentkind-quasiforward"></a>

### `SegmentKind::QuasiForward`

```cpp
QuasiForward = 0
```

a ramp from 0 V at rampRate, forward

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:158`](../../include/shulib/calibration/drive_characterizer.hpp#L158).*

<a id="segmentkind-quasibackward"></a>

### `SegmentKind::QuasiBackward`

```cpp
QuasiBackward = 1
```

the same ramp, backward

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:159`](../../include/shulib/calibration/drive_characterizer.hpp#L159).*

<a id="segmentkind-stepforward"></a>

### `SegmentKind::StepForward`

```cpp
StepForward = 2
```

stepVolts held, forward

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:160`](../../include/shulib/calibration/drive_characterizer.hpp#L160).*

<a id="segmentkind-stepbackward"></a>

### `SegmentKind::StepBackward`

```cpp
StepBackward = 3
```

stepVolts held, backward — the hard reverse (HA-160)

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:161`](../../include/shulib/calibration/drive_characterizer.hpp#L161).*

<a id="ksegmentcount"></a>

## `kSegmentCount`

```cpp
inline constexpr std::size_t kSegmentCount = 4
```

How many segments a run has.

*constant, declared at [`include/shulib/calibration/drive_characterizer.hpp:165`](../../include/shulib/calibration/drive_characterizer.hpp#L165).*

<a id="segmentkindname"></a>

## `segmentKindName`

```cpp
[[nodiscard]] constexpr const char* segmentKindName(SegmentKind k) noexcept
```

A short, stable, ASCII name for a segment — the serial record's word and every message's.

*free function, declared at [`include/shulib/calibration/drive_characterizer.hpp:168`](../../include/shulib/calibration/drive_characterizer.hpp#L168).*

<a id="enum-class-segmentend"></a>

## `enum class SegmentEnd`

```cpp
enum class SegmentEnd : std::uint8_t
```

How a segment's powered part ended. None = it did not end normally (a refusal, or still running). APPEND-ONLY.

*enum class, declared at [`include/shulib/calibration/drive_characterizer.hpp:180`](../../include/shulib/calibration/drive_characterizer.hpp#L180).*

<a id="segmentend-none"></a>

### `SegmentEnd::None`

```cpp
None = 0
```

not ended normally

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:181`](../../include/shulib/calibration/drive_characterizer.hpp#L181).*

<a id="segmentend-distance"></a>

### `SegmentEnd::Distance`

```cpp
Distance = 1
```

a side's travel reached segmentDistance

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:182`](../../include/shulib/calibration/drive_characterizer.hpp#L182).*

<a id="segmentend-ramptop"></a>

### `SegmentEnd::RampTop`

```cpp
RampTop = 2
```

the ramp reached rampMax

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:183`](../../include/shulib/calibration/drive_characterizer.hpp#L183).*

<a id="segmentend-timeout"></a>

### `SegmentEnd::Timeout`

```cpp
Timeout = 3
```

segmentTimeout passed

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:184`](../../include/shulib/calibration/drive_characterizer.hpp#L184).*

<a id="segmentendname"></a>

## `segmentEndName`

```cpp
[[nodiscard]] constexpr const char* segmentEndName(SegmentEnd e) noexcept
```

A short, stable, ASCII word for how a segment ended (the serial record's word and the warning's).

*free function, declared at [`include/shulib/calibration/drive_characterizer.hpp:188`](../../include/shulib/calibration/drive_characterizer.hpp#L188).*

<a id="enum-class-characterizerphase"></a>

## `enum class CharacterizerPhase`

```cpp
enum class CharacterizerPhase : std::uint8_t
```

Where a run has got to. A run ends in exactly one of Done or Aborted.

*enum class, declared at [`include/shulib/calibration/drive_characterizer.hpp:199`](../../include/shulib/calibration/drive_characterizer.hpp#L199).*

<a id="characterizerphase-idle"></a>

### `CharacterizerPhase::Idle`

```cpp
Idle
```

constructed, start() not yet called

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:200`](../../include/shulib/calibration/drive_characterizer.hpp#L200).*

<a id="characterizerphase-waitimu"></a>

### `CharacterizerPhase::WaitImu`

```cpp
WaitImu
```

waiting for the IMU's boot calibration

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:201`](../../include/shulib/calibration/drive_characterizer.hpp#L201).*

<a id="characterizerphase-rest"></a>

### `CharacterizerPhase::Rest`

```cpp
Rest
```

unpowered, waiting for the rest definition before the next segment

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:202`](../../include/shulib/calibration/drive_characterizer.hpp#L202).*

<a id="characterizerphase-driving"></a>

### `CharacterizerPhase::Driving`

```cpp
Driving
```

POWERED: a segment's volts, re-commanded every tick

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:203`](../../include/shulib/calibration/drive_characterizer.hpp#L203).*

<a id="characterizerphase-stopping"></a>

### `CharacterizerPhase::Stopping`

```cpp
Stopping
```

0 V + Brake until rest; the stopping distance is being measured

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:204`](../../include/shulib/calibration/drive_characterizer.hpp#L204).*

<a id="characterizerphase-done"></a>

### `CharacterizerPhase::Done`

```cpp
Done
```

result() is valid

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:205`](../../include/shulib/calibration/drive_characterizer.hpp#L205).*

<a id="characterizerphase-aborted"></a>

### `CharacterizerPhase::Aborted`

```cpp
Aborted
```

refused or aborted; reason() says why; every member at 0 V + Coast

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:206`](../../include/shulib/calibration/drive_characterizer.hpp#L206).*

<a id="enum-class-characterizerrefusal"></a>

## `enum class CharacterizerRefusal`

```cpp
enum class CharacterizerRefusal : std::uint8_t
```

WHY a run ended in Aborted — a code a panel words and a test asserts. APPEND-ONLY.

*enum class, declared at [`include/shulib/calibration/drive_characterizer.hpp:210`](../../include/shulib/calibration/drive_characterizer.hpp#L210).*

<a id="characterizerrefusal-none"></a>

### `CharacterizerRefusal::None`

```cpp
None
```

not refused

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:211`](../../include/shulib/calibration/drive_characterizer.hpp#L211).*

<a id="characterizerrefusal-aborted"></a>

### `CharacterizerRefusal::Aborted`

```cpp
Aborted
```

abort() — the operator, or the program on the operator's behalf

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:212`](../../include/shulib/calibration/drive_characterizer.hpp#L212).*

<a id="characterizerrefusal-imunotready"></a>

### `CharacterizerRefusal::ImuNotReady`

```cpp
ImuNotReady
```

the IMU never became ready within imuReadyTimeout

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:213`](../../include/shulib/calibration/drive_characterizer.hpp#L213).*

<a id="characterizerrefusal-resttimeout"></a>

### `CharacterizerRefusal::RestTimeout`

```cpp
RestTimeout
```

nothing came to rest within restTimeout (before a segment, or after one)

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:214`](../../include/shulib/calibration/drive_characterizer.hpp#L214).*

<a id="characterizerrefusal-curveexceeded"></a>

### `CharacterizerRefusal::CurveExceeded`

```cpp
CurveExceeded
```

the encoders' curve passed ψ_g while driving (rule 3)

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:215`](../../include/shulib/calibration/drive_characterizer.hpp#L215).*

<a id="characterizerrefusal-fight"></a>

### `CharacterizerRefusal::Fight`

```cpp
Fight
```

the coupled-side monitor saw a member fight its side while driving

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:216`](../../include/shulib/calibration/drive_characterizer.hpp#L216).*

<a id="characterizerrefusal-missedtick"></a>

### `CharacterizerRefusal::MissedTick`

```cpp
MissedTick
```

tick() was not called within commandTimeout while driving

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:217`](../../include/shulib/calibration/drive_characterizer.hpp#L217).*

<a id="characterizerrefusal-toohot"></a>

### `CharacterizerRefusal::TooHot`

```cpp
TooHot
```

a member was at or over tempGuard before a segment

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:218`](../../include/shulib/calibration/drive_characterizer.hpp#L218).*

<a id="characterizerrefusal-stoptoolong"></a>

### `CharacterizerRefusal::StopTooLong`

```cpp
StopTooLong
```

a side took more than stopBound to stop after a segment

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:219`](../../include/shulib/calibration/drive_characterizer.hpp#L219).*

<a id="characterizerrefusal-fitrefused"></a>

### `CharacterizerRefusal::FitRefused`

```cpp
FitRefused
```

a side's fit did not come out Ok (the report names the side and verdict)

*enumerator, declared at [`include/shulib/calibration/drive_characterizer.hpp:220`](../../include/shulib/calibration/drive_characterizer.hpp#L220).*

<a id="struct-characterizerconfig"></a>

## `struct CharacterizerConfig`

```cpp
struct CharacterizerConfig
```

Every number the GAINS step uses. Its own are INVENTED for a first run and carried as fields so an S2 serial log can move them with a logged line — PROVISIONAL, every one (A4: HA-170). The rest definition, the IMU wait, the missed-tick ceiling and the fight monitor are the drivetrain calibrator's, copied from CalibratorConfig's defaults so the two steps agree (A4: HA-136; the monitor's HA-132/133).

*struct, declared at [`include/shulib/calibration/drive_characterizer.hpp:228`](../../include/shulib/calibration/drive_characterizer.hpp#L228).*

<a id="characterizerconfig-ramprate"></a>

### `CharacterizerConfig::rampRate`

```cpp
double rampRate = 2.0
```

The quasi-static ramp's rate, V/s. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:230`](../../include/shulib/calibration/drive_characterizer.hpp#L230).*

<a id="characterizerconfig-rampmax"></a>

### `CharacterizerConfig::rampMax`

```cpp
double rampMax = 10.0
```

The ramp ends when it reaches this, V. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:232`](../../include/shulib/calibration/drive_characterizer.hpp#L232).*

<a id="characterizerconfig-stepvolts"></a>

### `CharacterizerConfig::stepVolts`

```cpp
double stepVolts = 6.0
```

The step segments' held volts. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:234`](../../include/shulib/calibration/drive_characterizer.hpp#L234).*

<a id="characterizerconfig-segmentdistance"></a>

### `CharacterizerConfig::segmentDistance`

```cpp
units::Length segmentDistance{36.0}
```

A segment ends when EITHER side has travelled this far. 36 in — a CHOICE that sets the lane's length; whether it is long enough is S2's question. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:237`](../../include/shulib/calibration/drive_characterizer.hpp#L237).*

<a id="characterizerconfig-segmenttimeout"></a>

### `CharacterizerConfig::segmentTimeout`

```cpp
units::Time segmentTimeout{8.0}
```

A segment ends after this long, whatever it has done. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:239`](../../include/shulib/calibration/drive_characterizer.hpp#L239).*

<a id="characterizerconfig-asymrefuse"></a>

### `CharacterizerConfig::asymRefuse`

```cpp
double asymRefuse = 0.30
```

The curve guard, as the side asymmetry it tolerates: ψ_g = segmentDistance × asymRefuse / W (rule 3). 0.30 — twice asymWarn, so a robot that warns still finishes. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:243`](../../include/shulib/calibration/drive_characterizer.hpp#L243).*

<a id="characterizerconfig-asymwarn"></a>

### `CharacterizerConfig::asymWarn`

```cpp
double asymWarn = 0.15
```

The sides' kV differing by more than this fraction of their mean is WARNED (a dragging side or a slipping member — a mechanical finding, not a tuning; R5a's decision D2). PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:247`](../../include/shulib/calibration/drive_characterizer.hpp#L247).*

<a id="characterizerconfig-stopbound"></a>

### `CharacterizerConfig::stopBound`

```cpp
units::Length stopBound{12.0}
```

A stopping distance longer than this refuses (StopTooLong) — and, with segmentDistance, sets the clear space the lane needs. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:250`](../../include/shulib/calibration/drive_characterizer.hpp#L250).*

<a id="characterizerconfig-tempguard"></a>

### `CharacterizerConfig::tempGuard`

```cpp
double tempGuard = 45.0
```

A member at or over this temperature (°C) refuses a segment (TooHot). 45 °C — under HA-44's first throttle step at 55 °C. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:253`](../../include/shulib/calibration/drive_characterizer.hpp#L253).*

<a id="characterizerconfig-restfloor"></a>

### `CharacterizerConfig::restFloor`

```cpp
units::AngularVelocity restFloor{CalibratorConfig{}.restFloor}
```

|velocity| under this is a still motor (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:255`](../../include/shulib/calibration/drive_characterizer.hpp#L255).*

<a id="characterizerconfig-restyawratefloor"></a>

### `CharacterizerConfig::restYawRateFloor`

```cpp
units::AngularVelocity restYawRateFloor{CalibratorConfig{}.restYawRateFloor}
```

|IMU yaw rate| under this is a still robot (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:257`](../../include/shulib/calibration/drive_characterizer.hpp#L257).*

<a id="characterizerconfig-restsettletime"></a>

### `CharacterizerConfig::restSettleTime`

```cpp
units::Time restSettleTime{CalibratorConfig{}.restSettleTime}
```

How long stillness must hold (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:259`](../../include/shulib/calibration/drive_characterizer.hpp#L259).*

<a id="characterizerconfig-resttimeout"></a>

### `CharacterizerConfig::restTimeout`

```cpp
units::Time restTimeout{CalibratorConfig{}.restTimeout}
```

How long to wait for stillness — before a segment, and for a stop — before refusing (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:262`](../../include/shulib/calibration/drive_characterizer.hpp#L262).*

<a id="characterizerconfig-imureadytimeout"></a>

### `CharacterizerConfig::imuReadyTimeout`

```cpp
units::Time imuReadyTimeout{CalibratorConfig{}.imuReadyTimeout}
```

How long to wait for the IMU's boot calibration (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:264`](../../include/shulib/calibration/drive_characterizer.hpp#L264).*

<a id="characterizerconfig-commandtimeout"></a>

### `CharacterizerConfig::commandTimeout`

```cpp
units::Time commandTimeout{CalibratorConfig{}.commandTimeout}
```

A clock gap past this while Driving is a missed tick (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:266`](../../include/shulib/calibration/drive_characterizer.hpp#L266).*

<a id="characterizerconfig-monitor"></a>

### `CharacterizerConfig::monitor`

```cpp
teleop::SideMonitorConfig monitor{}
```

The coupled-side fight detector's thresholds — the drive program's, hal::MotorGroup's and the calibrator's (A4: HA-132/133).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:269`](../../include/shulib/calibration/drive_characterizer.hpp#L269).*

<a id="characterizerconfig-fit"></a>

### `CharacterizerConfig::fit`

```cpp
FitConfig fit{}
```

The fit's admission rules, verdicts and ranges (A4: HA-170).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:271`](../../include/shulib/calibration/drive_characterizer.hpp#L271).*

<a id="characterizerconfig-timing"></a>

### `CharacterizerConfig::timing`

```cpp
diag::TickTimingConfig timing{}
```

The loop timing's nominal period and late budget; the nominal must equal fit.nominalDt.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:273`](../../include/shulib/calibration/drive_characterizer.hpp#L273).*

<a id="characterizerconfig-curveguard"></a>

### `CharacterizerConfig::curveGuard`

```cpp
[[nodiscard]] double curveGuard(units::Length trackWidth) const noexcept
```

ψ_g, radians: segmentDistance × asymRefuse / trackWidth (rule 3).

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:276`](../../include/shulib/calibration/drive_characterizer.hpp#L276).*

<a id="characterizerconfig-sidewaysexcursion"></a>

### `CharacterizerConfig::sidewaysExcursion`

```cpp
[[nodiscard]] double sidewaysExcursion(units::Length trackWidth) const noexcept
```

The clear space to EACH SIDE of a segment's start line, inches — the powered arc of a segment curving right up to the guard, segmentDistance·(1 − cos ψ_g)/ψ_g (R5a's D9), PLUS the whole stopBound: the stop is not curve-guarded (once the brake is on nothing can change its path) and may curve far more sharply than the powered part — one side braking short, the other rolling on — so its sideways share is bounded only by its length. 14.6 + 12 = 26.6 in at W = 12.5 in. The header's "The clear space" says what this bound assumes.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:286`](../../include/shulib/calibration/drive_characterizer.hpp#L286).*

<a id="characterizerconfig-clearaheadandbehind"></a>

### `CharacterizerConfig::clearAheadAndBehind`

```cpp
[[nodiscard]] double clearAheadAndBehind() const noexcept
```

The clear space ahead AND behind, inches: segmentDistance + stopBound (R5a's decision D11) — the most a segment's centre travels, powered and stopping (the header's "The clear space").

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:294`](../../include/shulib/calibration/drive_characterizer.hpp#L294).*

<a id="characterizerconfig-valid"></a>

### `CharacterizerConfig::valid`

```cpp
[[nodiscard]] bool valid(units::Length trackWidth, char* why, std::size_t cap) const
```

True when the config can run on a drivetrain of effective width `trackWidth`: every field finite and positive where it must be; rampMax and stepVolts in (0, 12]; asymWarn in (0, 1) and asymRefuse ABOVE it (a curve guard that fired before the warning could be reached would refuse every robot that warns); ψ_g at most π (a bigger guard bounds nothing a segment can do); tempGuard in (0, 100); restTimeout > restSettleTime; the fit's config valid; the timing's nominal equal to fit.nominalDt and its budget above it. Else false naming the field in `why` (may be null).

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:305`](../../include/shulib/calibration/drive_characterizer.hpp#L305).*

<a id="struct-segmentrecord"></a>

## `struct SegmentRecord`

```cpp
struct SegmentRecord
```

Everything one segment did — one serial line each.

*struct, declared at [`include/shulib/calibration/drive_characterizer.hpp:365`](../../include/shulib/calibration/drive_characterizer.hpp#L365).*

<a id="segmentrecord-kind"></a>

### `SegmentRecord::kind`

```cpp
SegmentKind kind = SegmentKind::QuasiForward
```

Which segment.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:367`](../../include/shulib/calibration/drive_characterizer.hpp#L367).*

<a id="segmentrecord-end"></a>

### `SegmentRecord::end`

```cpp
SegmentEnd end = SegmentEnd::None
```

How its powered part ended (None if a refusal ended it).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:369`](../../include/shulib/calibration/drive_characterizer.hpp#L369).*

<a id="segmentrecord-stopped"></a>

### `SegmentRecord::stopped`

```cpp
bool stopped = false
```

True once its stop reached rest — stopIn is then final.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:371`](../../include/shulib/calibration/drive_characterizer.hpp#L371).*

<a id="segmentrecord-seconds"></a>

### `SegmentRecord::seconds`

```cpp
double seconds = 0.0
```

Seconds from the segment's first powered tick to its end tick — or to the tick a refusal ended it on.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:374`](../../include/shulib/calibration/drive_characterizer.hpp#L374).*

<a id="segmentrecord-lastvolts"></a>

### `SegmentRecord::lastVolts`

```cpp
double lastVolts = 0.0
```

The last volts commanded before the end: within one tick's ramp under rampMax for a ramp that reached its top (the end tick commands nothing), the step's level for a step.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:377`](../../include/shulib/calibration/drive_characterizer.hpp#L377).*

<a id="segmentrecord-admitted"></a>

### `SegmentRecord::admitted`

```cpp
std::uint32_t admitted[2] = {0, 0}
```

Samples the fit admitted, per side ([0] LEFT, [1] RIGHT) — driving and stopping.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:379`](../../include/shulib/calibration/drive_characterizer.hpp#L379).*

<a id="segmentrecord-excluded"></a>

### `SegmentRecord::excluded`

```cpp
std::array<std::uint32_t, kFitExclusionReasons> excluded[2]{}
```

Samples excluded, per side, by reason (index = FitExclusion value − 1).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:381`](../../include/shulib/calibration/drive_characterizer.hpp#L381).*

<a id="segmentrecord-travelin"></a>

### `SegmentRecord::travelIn`

```cpp
double travelIn[2] = {0.0, 0.0}
```

Each side's powered travel, inches (median, encoders; negative backward) — as last measured when a refusal ended the segment while Driving.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:384`](../../include/shulib/calibration/drive_characterizer.hpp#L384).*

<a id="segmentrecord-stopin"></a>

### `SegmentRecord::stopIn`

```cpp
double stopIn[2] = {0.0, 0.0}
```

Each side's STOPPING distance, inches: from the end tick to rest (negative backward) — so far while Stopping, and the last measured when a refusal ended the stop.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:387`](../../include/shulib/calibration/drive_characterizer.hpp#L387).*

<a id="segmentrecord-batterymin"></a>

### `SegmentRecord::batteryMin`

```cpp
double batteryMin = 0.0
```

The battery's lowest reading over the segment, volts — over the SAME readings as the mean: one per tick from the segment's first powered tick through its stop.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:390`](../../include/shulib/calibration/drive_characterizer.hpp#L390).*

<a id="segmentrecord-batterymean"></a>

### `SegmentRecord::batteryMean`

```cpp
double batteryMean = 0.0
```

The battery's mean over the segment, volts (the readings batteryMin is the least of).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:392`](../../include/shulib/calibration/drive_characterizer.hpp#L392).*

<a id="segmentrecord-maxtemperature"></a>

### `SegmentRecord::maxTemperature`

```cpp
double maxTemperature = 0.0
```

The hottest member reading over the segment, °C.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:394`](../../include/shulib/calibration/drive_characterizer.hpp#L394).*

<a id="segmentrecord-encodercurverad"></a>

### `SegmentRecord::encoderCurveRad`

```cpp
double encoderCurveRad = 0.0
```

The encoders' curve ψ = (dR − dL)/W at the powered end, radians (positive = CCW).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:396`](../../include/shulib/calibration/drive_characterizer.hpp#L396).*

<a id="segmentrecord-imuyawrad"></a>

### `SegmentRecord::imuYawRad`

```cpp
double imuYawRad = 0.0
```

The IMU's accumulated yaw over the same span, radians (recorded only — rule 3).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:398`](../../include/shulib/calibration/drive_characterizer.hpp#L398).*

<a id="segmentrecord-maxheadingsteprad"></a>

### `SegmentRecord::maxHeadingStepRad`

```cpp
double maxHeadingStepRad = 0.0
```

The largest |tick-to-tick heading step| over the segment, radians (HA-160 evidence).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:400`](../../include/shulib/calibration/drive_characterizer.hpp#L400).*

<a id="segmentrecord-timing"></a>

### `SegmentRecord::timing`

```cpp
diag::TickTimingSummary timing{}
```

The segment's tick timing (driving and stopping) — so far, when a refusal ended it.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:402`](../../include/shulib/calibration/drive_characterizer.hpp#L402).*

<a id="segmentrecord-imuminusencoderrad"></a>

### `SegmentRecord::imuMinusEncoderRad`

```cpp
[[nodiscard]] double imuMinusEncoderRad() const noexcept
```

imuYawRad − encoderCurveRad: how far the IMU and the encoders disagree about the curve (HA-160 evidence; never a refusal).

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:406`](../../include/shulib/calibration/drive_characterizer.hpp#L406).*

<a id="struct-characterizerreport"></a>

## `struct CharacterizerReport`

```cpp
struct CharacterizerReport
```

Every number behind a run, filled as far as the run got — worth reading after a refusal too.

*struct, declared at [`include/shulib/calibration/drive_characterizer.hpp:410`](../../include/shulib/calibration/drive_characterizer.hpp#L410).*

<a id="characterizerreport-segments"></a>

### `CharacterizerReport::segments`

```cpp
SegmentRecord segments[kSegmentCount]{}
```

One record per segment started, in order.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:412`](../../include/shulib/calibration/drive_characterizer.hpp#L412).*

<a id="characterizerreport-segmentcount"></a>

### `CharacterizerReport::segmentCount`

```cpp
std::uint8_t segmentCount = 0
```

How many entries of `segments` are real.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:414`](../../include/shulib/calibration/drive_characterizer.hpp#L414).*

<a id="characterizerreport-fit"></a>

### `CharacterizerReport::fit`

```cpp
FitResult fit[2]{}
```

Each side's fit ([0] LEFT, [1] RIGHT) — filled when the last segment's stop reaches rest.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:416`](../../include/shulib/calibration/drive_characterizer.hpp#L416).*

<a id="characterizerreport-combined"></a>

### `CharacterizerReport::combined`

```cpp
control::FeedforwardGains combined{}
```

The combined gains: the field-by-field mean of the two fits (D2). Its kA is RECORDED, NOT APPLIED — the pipeline calls the cruise overload (header).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:419`](../../include/shulib/calibration/drive_characterizer.hpp#L419).*

<a id="characterizerreport-kvasymmetry"></a>

### `CharacterizerReport::kVAsymmetry`

```cpp
double kVAsymmetry = 0.0
```

|kV_L − kV_R| / their mean.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:421`](../../include/shulib/calibration/drive_characterizer.hpp#L421).*

<a id="characterizerreport-asymmetrywarning"></a>

### `CharacterizerReport::asymmetryWarning`

```cpp
bool asymmetryWarning = false
```

kVAsymmetry over CharacterizerConfig::asymWarn.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:423`](../../include/shulib/calibration/drive_characterizer.hpp#L423).*

<a id="characterizerreport-emptysegmentwarning"></a>

### `CharacterizerReport::emptySegmentWarning`

```cpp
bool emptySegmentWarning = false
```

True when some segment admitted NO sample on a side (its `admitted[side]` is 0): that side's gains come from the other segments alone — a segment that could not move (something in the way, a pinned robot) or a side held at its current limit the whole segment. Warned, not refused (the header's "What it does NOT refuse"). Set on a finished run only.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:428`](../../include/shulib/calibration/drive_characterizer.hpp#L428).*

<a id="characterizerreport-curveguardrad"></a>

### `CharacterizerReport::curveGuardRad`

```cpp
double curveGuardRad = 0.0
```

ψ_g, radians (rule 3), from the description's track width.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:430`](../../include/shulib/calibration/drive_characterizer.hpp#L430).*

<a id="characterizerreport-timing"></a>

### `CharacterizerReport::timing`

```cpp
diag::TickTimingSummary timing{}
```

The whole run's tick timing (every driving and stopping tick).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:432`](../../include/shulib/calibration/drive_characterizer.hpp#L432).*

<a id="struct-characterizersample"></a>

## `struct CharacterizerSample`

```cpp
struct CharacterizerSample
```

This tick's raw reading, for a program to record (Part C's CSV) without the library allocating.

*struct, declared at [`include/shulib/calibration/drive_characterizer.hpp:436`](../../include/shulib/calibration/drive_characterizer.hpp#L436).*

<a id="characterizersample-valid"></a>

### `CharacterizerSample::valid`

```cpp
bool valid = false
```

False until the first driving tick; then true for every driving and stopping tick.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:438`](../../include/shulib/calibration/drive_characterizer.hpp#L438).*

<a id="characterizersample-t"></a>

### `CharacterizerSample::t`

```cpp
double t = 0.0
```

The clock, seconds.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:440`](../../include/shulib/calibration/drive_characterizer.hpp#L440).*

<a id="characterizersample-segment"></a>

### `CharacterizerSample::segment`

```cpp
std::uint8_t segment = 0
```

The segment (SegmentKind's value).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:442`](../../include/shulib/calibration/drive_characterizer.hpp#L442).*

<a id="characterizersample-powered"></a>

### `CharacterizerSample::powered`

```cpp
bool powered = false
```

True while Driving, false while Stopping.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:444`](../../include/shulib/calibration/drive_characterizer.hpp#L444).*

<a id="characterizersample-volts"></a>

### `CharacterizerSample::volts`

```cpp
double volts = 0.0
```

The volts commanded this tick (0 while stopping).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:446`](../../include/shulib/calibration/drive_characterizer.hpp#L446).*

<a id="characterizersample-speed"></a>

### `CharacterizerSample::speed`

```cpp
double speed[2] = {0.0, 0.0}
```

Each side's median surface speed, in/s.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:448`](../../include/shulib/calibration/drive_characterizer.hpp#L448).*

<a id="characterizersample-position"></a>

### `CharacterizerSample::position`

```cpp
double position[2] = {0.0, 0.0}
```

Each side's median position, inches (cumulative — sidePosition()).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:450`](../../include/shulib/calibration/drive_characterizer.hpp#L450).*

<a id="characterizersample-battery"></a>

### `CharacterizerSample::battery`

```cpp
double battery = 0.0
```

The battery, volts.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:452`](../../include/shulib/calibration/drive_characterizer.hpp#L452).*

<a id="characterizersample-maxcurrent"></a>

### `CharacterizerSample::maxCurrent`

```cpp
double maxCurrent[2] = {0.0, 0.0}
```

Each side's largest |member current|, amperes.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:454`](../../include/shulib/calibration/drive_characterizer.hpp#L454).*

<a id="characterizersample-headingrad"></a>

### `CharacterizerSample::headingRad`

```cpp
double headingRad = 0.0
```

The IMU's heading as read, radians (wrapped).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:456`](../../include/shulib/calibration/drive_characterizer.hpp#L456).*

<a id="struct-characterizerstatus"></a>

## `struct CharacterizerStatus`

```cpp
struct CharacterizerStatus
```

What a panel paints this tick.

*struct, declared at [`include/shulib/calibration/drive_characterizer.hpp:460`](../../include/shulib/calibration/drive_characterizer.hpp#L460).*

<a id="characterizerstatus-phase"></a>

### `CharacterizerStatus::phase`

```cpp
CharacterizerPhase phase = CharacterizerPhase::Idle
```

The phase.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:462`](../../include/shulib/calibration/drive_characterizer.hpp#L462).*

<a id="characterizerstatus-segment"></a>

### `CharacterizerStatus::segment`

```cpp
std::uint8_t segment = 0
```

The segment under way or about to start (0-based; kSegmentCount once finished).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:464`](../../include/shulib/calibration/drive_characterizer.hpp#L464).*

<a id="characterizerstatus-volts"></a>

### `CharacterizerStatus::volts`

```cpp
double volts = 0.0
```

The volts in force (0 when not driving).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:466`](../../include/shulib/calibration/drive_characterizer.hpp#L466).*

<a id="characterizerstatus-travelin"></a>

### `CharacterizerStatus::travelIn`

```cpp
double travelIn[2] = {0.0, 0.0}
```

Each side's travel in the current segment, inches.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:468`](../../include/shulib/calibration/drive_characterizer.hpp#L468).*

<a id="characterizerstatus-curverad"></a>

### `CharacterizerStatus::curveRad`

```cpp
double curveRad = 0.0
```

The encoders' curve so far in the current segment, radians.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:470`](../../include/shulib/calibration/drive_characterizer.hpp#L470).*

<a id="characterizerstatus-stopin"></a>

### `CharacterizerStatus::stopIn`

```cpp
double stopIn[2] = {0.0, 0.0}
```

Each side's stopping distance so far, inches (Stopping only).

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:472`](../../include/shulib/calibration/drive_characterizer.hpp#L472).*

<a id="characterizerstatus-atrest"></a>

### `CharacterizerStatus::atRest`

```cpp
bool atRest = false
```

True when the rest definition holds this tick.

*field, declared at [`include/shulib/calibration/drive_characterizer.hpp:474`](../../include/shulib/calibration/drive_characterizer.hpp#L474).*

<a id="class-drivecharacterizer"></a>

## `class DriveCharacterizer`

```cpp
class DriveCharacterizer
```

The tick-driven GAINS step (header). Non-owning: the motors (through the sides' spans), the IMU, the clock and the battery must outlive it. Not copyable. RUN AGAIN is a new object — nothing is carried over by construction (the calibrator's rule).

*class, declared at [`include/shulib/calibration/drive_characterizer.hpp:480`](../../include/shulib/calibration/drive_characterizer.hpp#L480).*

<a id="drivecharacterizer-drivecharacterizer"></a>

### `DriveCharacterizer::DriveCharacterizer`

```cpp
DriveCharacterizer(DriveSide left, DriveSide right, units::Length trackWidth, hal::IImu& imu, hal::IClock& clock, hal::IBattery& battery, const CharacterizerConfig& config = {})
```

`left`/`right`: each side's members and scale, from the drivetrain record (rule 1: the sign lives in SideMember). `trackWidth`: the record's EFFECTIVE width — ψ and ψ_g are computed with it (rule 3). Preconditions: both sides driveSideValid(), no port listed twice across them, config.valid(trackWidth).

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:486`](../../include/shulib/calibration/drive_characterizer.hpp#L486).*

<a id="drivecharacterizer-drivecharacterizer-2"></a>

### `DriveCharacterizer::DriveCharacterizer (overload 2)`

```cpp
DriveCharacterizer(const DriveCharacterizer&) = delete
```

Not copyable: it holds references to hardware it does not own, and a copy would be a second state machine commanding the same motors.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:513`](../../include/shulib/calibration/drive_characterizer.hpp#L513).*

<a id="drivecharacterizer-operator-eq"></a>

### `DriveCharacterizer::operator=`

```cpp
DriveCharacterizer& operator=(const DriveCharacterizer&) = delete
```

Not copyable: it holds references to hardware it does not own, and a copy would be a second state machine commanding the same motors. Not copy-assignable, for the same reason.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:515`](../../include/shulib/calibration/drive_characterizer.hpp#L515).*

<a id="drivecharacterizer-start"></a>

### `DriveCharacterizer::start`

```cpp
void start(const char* provenance = "")
```

Begin a run: every member to 0 V + Coast, then WaitImu. `provenance` (the build stamp or a session note) is copied into the result's fixed buffer, cut to fit; no pointer is kept. Precondition: the run is Idle.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:520`](../../include/shulib/calibration/drive_characterizer.hpp#L520).*

<a id="drivecharacterizer-tick"></a>

### `DriveCharacterizer::tick`

```cpp
void tick()
```

One control tick. Cheap and non-blocking; the caller paces it (10 ms). A no-op before start() and after the run ends.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:531`](../../include/shulib/calibration/drive_characterizer.hpp#L531).*

<a id="drivecharacterizer-abort"></a>

### `DriveCharacterizer::abort`

```cpp
void abort(const char* why = "aborted by the operator")
```

Stop now: every member to 0 V + Coast (never Brake — rule 4), Aborted with CharacterizerRefusal::Aborted and `why` as the reason. Allowed in any phase, including Done (it then discards the result).

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:563`](../../include/shulib/calibration/drive_characterizer.hpp#L563).*

<a id="drivecharacterizer-phase"></a>

### `DriveCharacterizer::phase`

```cpp
[[nodiscard]] CharacterizerPhase phase() const noexcept
```

Where the run has got to.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:568`](../../include/shulib/calibration/drive_characterizer.hpp#L568).*

<a id="drivecharacterizer-refusal"></a>

### `DriveCharacterizer::refusal`

```cpp
[[nodiscard]] CharacterizerRefusal refusal() const noexcept
```

Which refusal ended the run (None while it runs or once it is Done).

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:570`](../../include/shulib/calibration/drive_characterizer.hpp#L570).*

<a id="drivecharacterizer-refusalport"></a>

### `DriveCharacterizer::refusalPort`

```cpp
[[nodiscard]] int refusalPort() const noexcept
```

The port the refusal names (a hot motor, a fighting member, one that would not rest), or 0.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:572`](../../include/shulib/calibration/drive_characterizer.hpp#L572).*

<a id="drivecharacterizer-reason"></a>

### `DriveCharacterizer::reason`

```cpp
[[nodiscard]] const char* reason() const noexcept
```

Why the run was refused — empty while it runs or once it is Done. ASCII, no '%'.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:574`](../../include/shulib/calibration/drive_characterizer.hpp#L574).*

<a id="drivecharacterizer-warning"></a>

### `DriveCharacterizer::warning`

```cpp
[[nodiscard]] const char* warning() const noexcept
```

The warnings a FINISHED run still owes the operator — a segment that admitted no sample on a side (CharacterizerReport::emptySegmentWarning) first, then the sides' kV asymmetry past asymWarn, joined by "; ". Empty when there is none. ASCII, no '%'.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:578`](../../include/shulib/calibration/drive_characterizer.hpp#L578).*

<a id="drivecharacterizer-report"></a>

### `DriveCharacterizer::report`

```cpp
[[nodiscard]] const CharacterizerReport& report() const noexcept
```

Every number behind the run, filled as far as it got.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:580`](../../include/shulib/calibration/drive_characterizer.hpp#L580).*

<a id="drivecharacterizer-status"></a>

### `DriveCharacterizer::status`

```cpp
[[nodiscard]] const CharacterizerStatus& status() const noexcept
```

What a panel paints this tick.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:582`](../../include/shulib/calibration/drive_characterizer.hpp#L582).*

<a id="drivecharacterizer-lastsample"></a>

### `DriveCharacterizer::lastSample`

```cpp
[[nodiscard]] const CharacterizerSample& lastSample() const noexcept
```

This tick's raw reading (CharacterizerSample).

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:584`](../../include/shulib/calibration/drive_characterizer.hpp#L584).*

<a id="drivecharacterizer-config"></a>

### `DriveCharacterizer::config`

```cpp
[[nodiscard]] const CharacterizerConfig& config() const noexcept
```

The config in force.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:586`](../../include/shulib/calibration/drive_characterizer.hpp#L586).*

<a id="drivecharacterizer-result"></a>

### `DriveCharacterizer::result`

```cpp
[[nodiscard]] const DriveCharacterization& result() const
```

The measured characterization: source Measured, the GAINS section filled (each side's fit and member count, the run's timing), no REST section. Precondition: phase() == Done.

*function, declared at [`include/shulib/calibration/drive_characterizer.hpp:590`](../../include/shulib/calibration/drive_characterizer.hpp#L590).*

<a id="characterizerrefusalname"></a>

## `characterizerRefusalName`

```cpp
[[nodiscard]] constexpr const char* characterizerRefusalName(CharacterizerRefusal r) noexcept
```

A short, stable, ASCII name for a refusal — the serial record's code.

*free function, declared at [`include/shulib/calibration/drive_characterizer.hpp:1220`](../../include/shulib/calibration/drive_characterizer.hpp#L1220).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 128 lines, click to expand</summary>

```text

 DriveCharacterizer — shulib Calibrate's GAINS step: the robot drives itself through four short,
 bounded segments while each drive side's kS, kV and kA are fitted on the brain (chunk R5a Part A,
 2026-09-29 — the roadmap's R5 ramp routine and fit, moved onto the robot by the team lead's ruling
 of 2026-09-26: gains are measured by the robot and kept on the SD card, never typed or committed).

 ── PROVEN ON THE HOST TEST PLANT; IT HAS NEVER DRIVEN A ROBOT ──────────────────────
 Every property below is shown against the A2 host test plant, whose motor model IS the law being
 fitted (sim/motor_model.hpp's honesty boundary) — so recovering the plant's gains proves the
 ARITHMETIC and every safety rule, not that a V5 drive side follows the law (A4 register HA-171).
 The plant models neither a V5 Coast nor a V5 Brake at 0 V (only its τ = kA/kV decay), so how far
 a real robot rolls or brakes after a segment is unknown until S2 (HA-53, HA-143). Every number in
 CharacterizerConfig is INVENTED (HA-170); the rest floors it shares with the drivetrain calibrator
 keep their HA-136. No program runs this yet: Part C of R5a wires it into shulib Calibrate.

 ── The phases ──────────────────────────────────────────────────────────────────────
   Idle → WaitImu → [Rest → Driving → Stopping] × 4 → Done | Aborted
   WaitImu   unpowered   wait for imu.isReady(), bounded.
   Rest      unpowered   the drivetrain calibrator's rest definition — every member under restFloor
                         AND the IMU's yaw rate under restYawRateFloor — held restSettleTime, bounded
                         by restTimeout. Then every member's temperature must be UNDER tempGuard,
                         else TooHot, naming the port and the temperature, and nothing is powered.
   Driving   POWERED     one segment. BOTH sides get the SAME volts every tick (each member
                         sign × volts):
                           QuasiForward   a ramp of rampRate V/s from 0
                           QuasiBackward  the same, backward
                           StepForward    stepVolts held
                           StepBackward   the same, backward — a hard reverse from rest, the
                                          condition of 2026-09-25's IMU episode (HA-160)
                         It ENDS — normally, never a refusal — on the FIRST of: either side's travel
                         reaching segmentDistance (judged PER SIDE, never on the mean: the faster side
                         is the one that runs out of room); the ramp reaching rampMax; segmentTimeout.
   Stopping  0 V + BRAKE the end tick commands 0 V + Brake on every member — which reaches the motor
                         since DEFECTS2 (HA-142), and which HA-53 claims is prompt — and holds it,
                         re-commanded every tick, until the rest definition holds; then 0 V + Coast.
                         Each side's STOPPING DISTANCE (encoders, from the end tick to rest) is
                         measured and reported; one longer than stopBound ends the run as StopTooLong
                         before any further segment (R5a's decision D11).
   Done      result() is the characterization; report() every number behind it.
 The two STEP segments exist because a ramp alone cannot identify the law: once its start transient
 decays, speed is affine in volts and the fit's columns are collinear (feedforward_fit.hpp; R5a's
 re-check R2 and decision D10).

 ── The rules, each with the mistake it prevents ───────────────────────────────────
  1. ONE PLACE FOR A SIGN. Each member's `sign` multiplies its command and its readings HERE, once.
     shulib Calibrate's adapters carry POSITIVE ports (drivetrain_calibrator.hpp's rule 1); a
     program whose adapters are already signed passes +1. Applying it twice is a five-against-four
     fight on the robot and invisible in a symmetric test (R5a's landmine 6).
  2. THE MEDIAN, NOT THE MEAN, for a side's speed and travel — hal::medianInPlace, the statistic
     hal::MotorGroup reports, so a slipping member (robot two's port 18, HA-130) cannot drag a side.
  3. THE CURVE GUARD READS THE ENCODERS, NEVER THE IMU (R5a's decision D9). Equal volts on sides whose
     gains differ curve the robot by about segmentDistance × asymmetry / W. The guard is the
     encoders' ψ = (dR − dL)/W from each side's median travel, refused (CurveExceeded) past
     ψ_g = segmentDistance × asymRefuse / W, computed at construction from the DESCRIPTION's
     effective track width. asymRefuse (0.30) is twice the 15 % asymmetry warning, so a robot that
     warns still finishes. The IMU's accumulated yaw is RECORDED beside ψ for every segment — their
     disagreement is HA-160 evidence — and never refuses: an IMU that jumps 37° in a tick (as on
     2026-09-25) would otherwise end the run it was meant to be recorded by. Never loosen the
     encoder guard to make a run pass.
  4. BRAKE AT A NORMAL SEGMENT END ONLY, UNTIL REST; COAST EVERYWHERE ELSE. Every refusal and
     every abort leaves every member at 0 V + Coast ON THE SAME TICK — a person may be reaching for
     the robot (the calibrator's rule 6), and a braked robot cannot be pushed.
  5. A MISSED TICK WHILE DRIVING IS A STOP: a clock gap past commandTimeout while Driving commands
     0 V + Coast and refuses (MissedTick). Every powered volt is re-commanded every tick. A gap
     while at rest or stopping powers nothing, and is not refused.
  6. THE IMU'S YAW IS ACCUMULATED PER TICK through Angle::errorTo (the calibrator's rule 3), and
     RECORDED ONLY — with the largest tick-to-tick step of every segment.
  7. HOT MOTORS DO NOT START A SEGMENT: every member under tempGuard (45 °C, below HA-44's first
     throttle step), else TooHot before anything is powered. The largest member temperature of
     every segment is recorded.
  8. THE FIT IS FED EVERY TICK, driving AND stopping, and ITS admission decides (feedforward_fit.hpp):
     the stop's samples are excluded there as u = 0, which is what makes that exclusion
     load-bearing on the plant (admitting them moves kS from 1.00 to 0.53).

 ── What it reports ─────────────────────────────────────────────────────────────────
 Per segment: how it ended; the samples admitted and excluded by reason, per side; each side's
 powered travel and STOPPING DISTANCE; the battery's minimum and mean; the largest member
 temperature; the encoders' curve ψ and the IMU's accumulated yaw at the powered end, and their
 difference; the largest tick-to-tick heading step; the segment's tick timing. Per side: the fit
 (FitResult). The combined gains (kA marked RECORDED, NOT APPLIED — the pipeline calls the cruise
 overload), the sides' kV asymmetry and its warning, and a warning for any segment that admitted
 no sample on a side (below, "What it does NOT refuse"). lastSample() exposes this tick's raw
 reading (time, volts, each side's median speed AND median position, battery, current, IMU
 heading), so a program can record the raw data (Part C's CSV) without this class allocating. A
 segment a refusal ended keeps how far it got — its seconds, last volts, travel, curve, IMU yaw and
 timing as last measured (its `end` stays None) — so the record after a refusal shows the numbers
 behind it.

 ── The clear space, and what it assumes (R5a's D9 and D11, corrected 2026-10-04) ───────
 clearAheadAndBehind() = segmentDistance + stopBound (48 in) and sidewaysExcursion(W) = the powered
 arc to the guard, segmentDistance·(1 − cos ψ_g)/ψ_g, PLUS stopBound (26.6 in at W = 12.5 in). The
 design's 14.6 in left the stop out: the curve guard is judged only while Driving, and a stop that
 keeps curving — on the host test plant a Done run at 46° a segment with stops near 11 in swept
 19.7 in sideways — or curves harder than the powered part (one side braking short: a 13.1 in /
 1.9 in stop after a 6° segment) is not bounded by the arc. What the two numbers assume:
   * PER SEGMENT, from that segment's start pose. Each backward segment drives back along roughly
     the path its forward segment took, so the robot stays near its start line; on the host test
     plant the run's own excursion from its start pose stayed inside both bounds in every case
     measured (development log, shulib-v2 branch, 2026-10-04). A robot that drives very
     differently backward than forward could drift further over the run — nothing here tracks the
     run's pose (an open question for the team lead, beside the one below).
   * The powered part curves at a roughly CONSTANT rate (equal volts on fixed gains). The guard
     bounds |ψ| on every tick, not the path's shape: a robot that turned ψ_g at once and then ran
     straight would go segmentDistance·sin ψ_g (27.4 in) sideways — in practice the guard refuses
     that early turn first (a kS of 2.2 V against 1.0 is refused CurveExceeded within the first
     segment on the host test plant).
   * Every stop stays inside stopBound. A stop past it is refused (StopTooLong) only once it has
     happened: the robot has ALREADY rolled that far, so the lane's ends need room beyond the bound
     for the one stop that exceeds it. The plant cannot say how far a V5 rolls or brakes (HA-53).

 ── What it does NOT refuse (open, for the team lead) ───────────────────────────────
 A segment that does not MOVE — something in front of the robot, a foot on it, a side whose
 motors cannot turn it — is not refused: it ends normally on RampTop (the ramp at
 rampMax, after up to rampMax/rampRate = 5 s of pushing) or on Timeout (a step after
 segmentTimeout, 8 s at stepVolts), with every sample in the dead band and so none admitted. The
 run then finishes on the segments that did move — a robot blocked in one direction finishes Done
 on the other direction's samples alone — and says so: any segment that admitted NO sample on a
 side sets CharacterizerReport::emptySegmentWarning and leads warning() with the segment, the side,
 its travel and how it ended (R5a's review of 2026-10-04 found such a run finishing with an empty
 warning; a side held at its current limit through a whole segment warns the same way). The
 drivetrain calibrator REFUSES the same situation (Refusal::SpinNoStart — "is it pinned, or on
 blocks?"); whether the GAINS step should too, and with what thresholds, is the team lead's
 decision before shulib Calibrate runs it (R5a Part C). The exposure until then is bounded:
 segmentTimeout per segment, every refusal and every normal end through the rules above.

 Tick-driven and non-blocking (10 ms on the robot); allocation-free; PROS-free and sim-free;
 raises no faults (what to do about a refusal is the program's policy); every runtime string is
 ASCII with no '%' ("pct") — PROS re-reads screen text as a format.
```

</details>
