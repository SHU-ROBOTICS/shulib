<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/drivetrain_calibrator.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drivetrain_calibrator.hpp`

DrivetrainCalibrator — the robot describes its own drivetrain.

This header declares **8** types (71 members) and **3** constants.

Extracted from [`include/shulib/calibration/drivetrain_calibrator.hpp`](../../include/shulib/calibration/drivetrain_calibrator.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kDegreesToRadians`](#kdegreestoradians) — *constant*
- [`kReproScaleTolerance`](#kreproscaletolerance) — *constant*
- [`kReproWidthTolerance`](#kreprowidthtolerance) — *constant*
- [`enum class Phase`](#enum-class-phase)
  - [`Idle`](#phase-idle)
  - [`WaitImu`](#phase-waitimu)
  - [`PushCapture`](#phase-pushcapture)
  - [`SideAssign`](#phase-sideassign)
  - [`ScalePush`](#phase-scalepush)
  - [`SpinTest`](#phase-spintest)
  - [`Done`](#phase-done)
  - [`Aborted`](#phase-aborted)
- [`enum class Answer`](#enum-class-answer)
  - [`Done`](#answer-done)
  - [`FrontFirst`](#answer-frontfirst)
  - [`BackFirst`](#answer-backfirst)
  - [`Distance24`](#answer-distance24)
  - [`Distance48`](#answer-distance48)
  - [`Distance72`](#answer-distance72)
  - [`Abort`](#answer-abort)
- [`struct HumanFacts`](#struct-humanfacts)
  - [`kind`](#humanfacts-kind)
  - [`cartridge`](#humanfacts-cartridge)
  - [`imuPort`](#humanfacts-imuport)
  - [`provenance`](#humanfacts-provenance)
- [`struct CalibratorConfig`](#struct-calibratorconfig)
  - [`movedThreshold`](#calibratorconfig-movedthreshold)
  - [`restFloor`](#calibratorconfig-restfloor)
  - [`restSettleTime`](#calibratorconfig-restsettletime)
  - [`restTimeout`](#calibratorconfig-resttimeout)
  - [`pulseVolts`](#calibratorconfig-pulsevolts)
  - [`pulseTime`](#calibratorconfig-pulsetime)
  - [`pulseYawFloor`](#calibratorconfig-pulseyawfloor)
  - [`straightBound`](#calibratorconfig-straightbound)
  - [`minPushRad`](#calibratorconfig-minpushrad)
  - [`spinVolts`](#calibratorconfig-spinvolts)
  - [`spinTurns`](#calibratorconfig-spinturns)
  - [`spinTimeout`](#calibratorconfig-spintimeout)
  - [`spinAgreement`](#calibratorconfig-spinagreement)
  - [`imuReadyTimeout`](#calibratorconfig-imureadytimeout)
  - [`commandTimeout`](#calibratorconfig-commandtimeout)
  - [`monitor`](#calibratorconfig-monitor)
- [`struct PortObservation`](#struct-portobservation)
  - [`port`](#portobservation-port)
  - [`sign`](#portobservation-sign)
  - [`side`](#portobservation-side)
  - [`pushDeltaRad`](#portobservation-pushdeltarad)
  - [`tapeDeltaRad`](#portobservation-tapedeltarad)
  - [`pulseYawRad`](#portobservation-pulseyawrad)
  - [`pulseShaftRad`](#portobservation-pulseshaftrad)
- [`struct CalibrationReport`](#struct-calibrationreport)
  - [`ports`](#calibrationreport-ports)
  - [`portCount`](#calibrationreport-portcount)
  - [`drivePortCount`](#calibrationreport-driveportcount)
  - [`backFirst`](#calibrationreport-backfirst)
  - [`pushDistanceIn`](#calibrationreport-pushdistancein)
  - [`pushStraightnessRad`](#calibrationreport-pushstraightnessrad)
  - [`leftMedianRad`](#calibrationreport-leftmedianrad)
  - [`rightMedianRad`](#calibrationreport-rightmedianrad)
  - [`leftInchesPerRadian`](#calibrationreport-leftinchesperradian)
  - [`rightInchesPerRadian`](#calibrationreport-rightinchesperradian)
  - [`leftSpreadRad`](#calibrationreport-leftspreadrad)
  - [`rightSpreadRad`](#calibrationreport-rightspreadrad)
  - [`widthCcwIn`](#calibrationreport-widthccwin)
  - [`widthCwIn`](#calibrationreport-widthcwin)
  - [`widthDisagreement`](#calibrationreport-widthdisagreement)
- [`struct CalibratorStatus`](#struct-calibratorstatus)
  - [`phase`](#calibratorstatus-phase)
  - [`prompt`](#calibratorstatus-prompt)
  - [`progress`](#calibratorstatus-progress)
- [`class DrivetrainCalibrator`](#class-drivetraincalibrator)
  - [`DrivetrainCalibrator`](#drivetraincalibrator-drivetraincalibrator)
  - [`start`](#drivetraincalibrator-start)
  - [`tick`](#drivetraincalibrator-tick)
  - [`answer`](#drivetraincalibrator-answer)
  - [`abort`](#drivetraincalibrator-abort)
  - [`phase`](#drivetraincalibrator-phase)
  - [`status`](#drivetraincalibrator-status)
  - [`reason`](#drivetraincalibrator-reason)
  - [`result`](#drivetraincalibrator-result)
  - [`report`](#drivetraincalibrator-report)
  - [`config`](#drivetraincalibrator-config)

<a id="kdegreestoradians"></a>

## `kDegreesToRadians`

```cpp
inline constexpr double kDegreesToRadians = math::Angle::kPi / 180.0
```

Radians per degree — the thresholds below are stated in the degrees a person can picture on a panel and converted once, here, rather than written as decimal radians nobody can check.

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:100`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L100).*

<a id="kreproscaletolerance"></a>

## `kReproScaleTolerance`

```cpp
inline constexpr double kReproScaleTolerance = 0.02
```

The reproducibility the calibrator CLAIMS for a side's scale: two complete runs of the same procedure on the same drivetrain must agree within this fraction. PROVISIONAL (A4: HA-136) — pinned against the A2 plant under encoder and IMU hostility; a bench session decides whether a real drivetrain holds it.

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:106`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L106).*

<a id="kreprowidthtolerance"></a>

## `kReproWidthTolerance`

```cpp
inline constexpr double kReproWidthTolerance = 0.03
```

The reproducibility the calibrator CLAIMS for the effective track width, as a fraction. Looser than the scale's because the spin measurement runs through the floor's scrub, which is the one thing the host plant does not model. PROVISIONAL (A4: HA-136).

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:111`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L111).*

<a id="enum-class-phase"></a>

## `enum class Phase`

```cpp
enum class Phase : std::uint8_t
```

Where a run has got to. A run ends in exactly one of Done or Aborted.

*enum class, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:114`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L114).*

<a id="phase-idle"></a>

### `Phase::Idle`

```cpp
Idle
```

constructed, start() not yet called

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:115`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L115).*

<a id="phase-waitimu"></a>

### `Phase::WaitImu`

```cpp
WaitImu
```

waiting for the IMU to finish its boot calibration

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:116`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L116).*

<a id="phase-pushcapture"></a>

### `Phase::PushCapture`

```cpp
PushCapture
```

UNPOWERED: watching a whole-robot push for drive ports and their signs

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:117`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L117).*

<a id="phase-sideassign"></a>

### `Phase::SideAssign`

```cpp
SideAssign
```

powered: one brief pulse per drive port, naming its side from the yaw sign

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:118`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L118).*

<a id="phase-scalepush"></a>

### `Phase::ScalePush`

```cpp
ScalePush
```

UNPOWERED: watching a push along a tape for each side's inches per radian

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:119`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L119).*

<a id="phase-spintest"></a>

### `Phase::SpinTest`

```cpp
SpinTest
```

powered: a spin each way for the effective track width

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:120`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L120).*

<a id="phase-done"></a>

### `Phase::Done`

```cpp
Done
```

result() is valid

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:121`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L121).*

<a id="phase-aborted"></a>

### `Phase::Aborted`

```cpp
Aborted
```

refused or aborted; reason() says why, every motor is at 0 V + Coast

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:122`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L122).*

<a id="enum-class-answer"></a>

## `enum class Answer`

```cpp
enum class Answer : std::uint8_t
```

What the operator's touch means. Everything a person contributes to a calibration arrives through exactly this enum — there is no blocking prompt anywhere in the class.

*enum class, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:127`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L127).*

<a id="answer-done"></a>

### `Answer::Done`

```cpp
Done
```

"I have finished pushing"

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:128`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L128).*

<a id="answer-frontfirst"></a>

### `Answer::FrontFirst`

```cpp
FrontFirst
```

the push led with the end the build team calls the FRONT

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:129`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L129).*

<a id="answer-backfirst"></a>

### `Answer::BackFirst`

```cpp
BackFirst
```

the push led with the other end — every captured sign inverts

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:130`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L130).*

<a id="answer-distance24"></a>

### `Answer::Distance24`

```cpp
Distance24
```

the tape push covered 24 in

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:131`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L131).*

<a id="answer-distance48"></a>

### `Answer::Distance48`

```cpp
Distance48
```

... 48 in (the default offer: two field tiles)

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:132`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L132).*

<a id="answer-distance72"></a>

### `Answer::Distance72`

```cpp
Distance72
```

... 72 in

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:133`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L133).*

<a id="answer-abort"></a>

### `Answer::Abort`

```cpp
Abort
```

stop now

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:134`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L134).*

<a id="struct-humanfacts"></a>

## `struct HumanFacts`

```cpp
struct HumanFacts
```

The three facts no measurement can supply, stated by a person before the run.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:138`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L138).*

<a id="humanfacts-kind"></a>

### `HumanFacts::kind`

```cpp
hal::DrivetrainKind kind = hal::DrivetrainKind::Tank
```

Which drivetrain this is. Only Tank is supported: a holonomic drivetrain needs a strafe scale this procedure does not measure, and the calibrator refuses rather than guess.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:141`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L141).*

<a id="humanfacts-cartridge"></a>

### `HumanFacts::cartridge`

```cpp
hal::MotorCartridge cartridge = hal::MotorCartridge::Blue
```

The cartridge in the drive motors — read off a motor, not inferred (see the header).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:143`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L143).*

<a id="humanfacts-imuport"></a>

### `HumanFacts::imuPort`

```cpp
std::uint8_t imuPort = 0
```

The IMU's smart port, 1..hal::kMaxSmartPorts. Carried into the description so a later boot can check the device is still there.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:146`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L146).*

<a id="humanfacts-provenance"></a>

### `HumanFacts::provenance`

```cpp
const char* provenance = ""
```

The build stamp or session note to record as the description's provenance. Copied into the description's fixed buffer (truncated to fit) — the calibrator keeps no pointer to it past start().

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:150`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L150).*

<a id="struct-calibratorconfig"></a>

## `struct CalibratorConfig`

```cpp
struct CalibratorConfig
```

Every number the procedure uses. NOT ONE OF THEM IS A MEASUREMENT: all are INVENTED for a first run and carried as fields so a bench session can move them without touching code. PROVISIONAL, all of them (A4: HA-136).

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:156`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L156).*

<a id="calibratorconfig-movedthreshold"></a>

### `CalibratorConfig::movedThreshold`

```cpp
units::AngleDim movedThreshold{5.0 * kDegreesToRadians}
```

Shaft travel over the push above which a candidate counts as a DRIVE port. 5 degrees — the bench tester's MOTOR WATCH number, invented there and carried here unchanged so the two tools call the same push the same thing. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:160`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L160).*

<a id="calibratorconfig-restfloor"></a>

### `CalibratorConfig::restFloor`

```cpp
units::AngularVelocity restFloor{0.05}
```

|velocity| below which a motor counts as stopped, for the rest windows between pulses. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:163`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L163).*

<a id="calibratorconfig-restsettletime"></a>

### `CalibratorConfig::restSettleTime`

```cpp
units::Time restSettleTime{0.5}
```

How long everything must stay below restFloor before a pulse may start — so one pulse's coast-down cannot be read as the next pulse's motion. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:166`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L166).*

<a id="calibratorconfig-resttimeout"></a>

### `CalibratorConfig::restTimeout`

```cpp
units::Time restTimeout{3.0}
```

How long to wait for rest before refusing. Without a bound this wait is the one unbounded loop in the procedure. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:169`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L169).*

<a id="calibratorconfig-pulsevolts"></a>

### `CalibratorConfig::pulseVolts`

```cpp
units::Voltage pulseVolts{3.0}
```

The ceiling for a single-member pulse — the bench tester's DRIVE station's first ceiling. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:172`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L172).*

<a id="calibratorconfig-pulsetime"></a>

### `CalibratorConfig::pulseTime`

```cpp
units::Time pulseTime{0.4}
```

How long one member is pulsed. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:174`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L174).*

<a id="calibratorconfig-pulseyawfloor"></a>

### `CalibratorConfig::pulseYawFloor`

```cpp
units::AngleDim pulseYawFloor{2.0 * kDegreesToRadians}
```

The |dtheta| a pulse must produce before its sign may be read as a side. Below it the robot is not on the floor (wheels up on blocks turn nothing) and the run refuses saying so. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:178`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L178).*

<a id="calibratorconfig-straightbound"></a>

### `CalibratorConfig::straightBound`

```cpp
units::AngleDim straightBound{3.0 * kDegreesToRadians}
```

The |dtheta| a tape push may drift by before it is not a straight push. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:181`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L181).*

<a id="calibratorconfig-minpushrad"></a>

### `CalibratorConfig::minPushRad`

```cpp
units::AngleDim minPushRad{2.0 * math::Angle::kPi}
```

The least median shaft travel a tape push must produce per side — one full wheel turn. A shorter push divides the tape distance by a number too small to trust. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:185`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L185).*

<a id="calibratorconfig-spinvolts"></a>

### `CalibratorConfig::spinVolts`

```cpp
units::Voltage spinVolts{3.0}
```

The ceiling for the in-place spin. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:187`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L187).*

<a id="calibratorconfig-spinturns"></a>

### `CalibratorConfig::spinTurns`

```cpp
double spinTurns = 2.0
```

How many full turns each direction of the spin must cover. More turns average more of the floor and dilute a single scuff; two is the compromise with how long a person will stand and watch. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:191`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L191).*

<a id="calibratorconfig-spintimeout"></a>

### `CalibratorConfig::spinTimeout`

```cpp
units::Time spinTimeout{20.0}
```

How long one direction of the spin may take before the run refuses. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:194`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L194).*

<a id="calibratorconfig-spinagreement"></a>

### `CalibratorConfig::spinAgreement`

```cpp
double spinAgreement = 0.05
```

The largest relative disagreement tolerated between the two directions' track widths. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:197`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L197).*

<a id="calibratorconfig-imureadytimeout"></a>

### `CalibratorConfig::imuReadyTimeout`

```cpp
units::Time imuReadyTimeout{5.0}
```

How long to wait for the IMU's boot calibration. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:199`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L199).*

<a id="calibratorconfig-commandtimeout"></a>

### `CalibratorConfig::commandTimeout`

```cpp
units::Time commandTimeout{0.1}
```

How far the clock may advance between two tick() calls before a powered phase treats the gap as a stalled caller, commands 0 V and refuses. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:202`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L202).*

<a id="calibratorconfig-monitor"></a>

### `CalibratorConfig::monitor`

```cpp
teleop::SideMonitorConfig monitor{}
```

The coupled-side fight detector's thresholds for the spin. Defaults are the drive program's and hal::MotorGroup's — one evaluator, one set of numbers (A4: HA-132/133).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:205`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L205).*

<a id="struct-portobservation"></a>

## `struct PortObservation`

```cpp
struct PortObservation
```

What one run concluded about ONE candidate — the panel's table and the log's record.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:209`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L209).*

<a id="portobservation-port"></a>

### `PortObservation::port`

```cpp
std::int8_t port = 0
```

The candidate's port as handed in: always POSITIVE (the sign is captured, not given).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:211`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L211).*

<a id="portobservation-sign"></a>

### `PortObservation::sign`

```cpp
std::int8_t sign = 0
```

The sign captured from the push, already inverted if the push was BACK-first: +1, −1, or 0 for a candidate that did not move and is therefore not a drive motor.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:214`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L214).*

<a id="portobservation-side"></a>

### `PortObservation::side`

```cpp
std::int8_t side = 0
```

−1 LEFT, +1 RIGHT, 0 = not a drive port or not yet pulsed.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:216`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L216).*

<a id="portobservation-pushdeltarad"></a>

### `PortObservation::pushDeltaRad`

```cpp
double pushDeltaRad = 0.0
```

Shaft travel over the SIGN-CAPTURE push, radians, as READ (before the sign is applied) — the number the sign was read off.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:219`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L219).*

<a id="portobservation-tapedeltarad"></a>

### `PortObservation::tapeDeltaRad`

```cpp
double tapeDeltaRad = 0.0
```

Shaft travel over the TAPE push, radians, sign-corrected. Kept separate from pushDeltaRad so a panel can show both pushes; 0 for a port that is not a drive motor.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:222`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L222).*

<a id="portobservation-pulseyawrad"></a>

### `PortObservation::pulseYawRad`

```cpp
double pulseYawRad = 0.0
```

The IMU's accumulated dtheta over this port's own pulse, radians. 0 if it was not pulsed.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:224`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L224).*

<a id="portobservation-pulseshaftrad"></a>

### `PortObservation::pulseShaftRad`

```cpp
double pulseShaftRad = 0.0
```

This port's own sign-corrected shaft travel over its pulse, radians. Must be > 0.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:226`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L226).*

<a id="struct-calibrationreport"></a>

## `struct CalibrationReport`

```cpp
struct CalibrationReport
```

Every number behind a result — for the panel, for the log, and for a human deciding whether to believe the run. Populated as far as the run got, so it is worth reading after a REFUSAL too.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:231`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L231).*

<a id="calibrationreport-ports"></a>

### `CalibrationReport::ports`

```cpp
PortObservation ports[hal::kMaxSmartPorts]{}
```

One row per candidate, in the order they were handed in.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:233`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L233).*

<a id="calibrationreport-portcount"></a>

### `CalibrationReport::portCount`

```cpp
std::uint8_t portCount = 0
```

How many rows of `ports` are real.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:235`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L235).*

<a id="calibrationreport-driveportcount"></a>

### `CalibrationReport::drivePortCount`

```cpp
std::uint8_t drivePortCount = 0
```

How many of them moved on the push and are therefore drive motors.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:237`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L237).*

<a id="calibrationreport-backfirst"></a>

### `CalibrationReport::backFirst`

```cpp
bool backFirst = false
```

True when the operator answered BACK FIRST and every captured sign was inverted.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:239`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L239).*

<a id="calibrationreport-pushdistancein"></a>

### `CalibrationReport::pushDistanceIn`

```cpp
double pushDistanceIn = 0.0
```

The tape distance the operator chose, inches.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:241`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L241).*

<a id="calibrationreport-pushstraightnessrad"></a>

### `CalibrationReport::pushStraightnessRad`

```cpp
double pushStraightnessRad = 0.0
```

|dtheta| the IMU saw over the tape push, radians — how straight the push actually was.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:243`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L243).*

<a id="calibrationreport-leftmedianrad"></a>

### `CalibrationReport::leftMedianRad`

```cpp
double leftMedianRad = 0.0
```

The LEFT side's median shaft travel over the tape push, radians (the scale's denominator).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:245`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L245).*

<a id="calibrationreport-rightmedianrad"></a>

### `CalibrationReport::rightMedianRad`

```cpp
double rightMedianRad = 0.0
```

The RIGHT side's median shaft travel over the tape push, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:247`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L247).*

<a id="calibrationreport-leftinchesperradian"></a>

### `CalibrationReport::leftInchesPerRadian`

```cpp
double leftInchesPerRadian = 0.0
```

The LEFT side's measured scale, inches per radian — available as soon as the tape push is answered, so a panel (and a host test) can read it without waiting for the spin.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:250`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L250).*

<a id="calibrationreport-rightinchesperradian"></a>

### `CalibrationReport::rightInchesPerRadian`

```cpp
double rightInchesPerRadian = 0.0
```

The RIGHT side's measured scale, inches per radian.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:252`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L252).*

<a id="calibrationreport-leftspreadrad"></a>

### `CalibrationReport::leftSpreadRad`

```cpp
double leftSpreadRad = 0.0
```

max − min of the LEFT side's members' travel over the tape push, radians: how far apart a side's motors read. Robot two's port 18 is expected to show here (A4 register HA-130).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:255`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L255).*

<a id="calibrationreport-rightspreadrad"></a>

### `CalibrationReport::rightSpreadRad`

```cpp
double rightSpreadRad = 0.0
```

max − min of the RIGHT side's members' travel over the tape push, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:257`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L257).*

<a id="calibrationreport-widthccwin"></a>

### `CalibrationReport::widthCcwIn`

```cpp
double widthCcwIn = 0.0
```

The effective track width from the CCW spin, inches.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:259`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L259).*

<a id="calibrationreport-widthcwin"></a>

### `CalibrationReport::widthCwIn`

```cpp
double widthCwIn = 0.0
```

The effective track width from the CW spin, inches.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:261`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L261).*

<a id="calibrationreport-widthdisagreement"></a>

### `CalibrationReport::widthDisagreement`

```cpp
double widthDisagreement = 0.0
```

|Wccw − Wcw| / their mean — the agreement the run was judged on.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:263`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L263).*

<a id="struct-calibratorstatus"></a>

## `struct CalibratorStatus`

```cpp
struct CalibratorStatus
```

What a panel needs to paint this tick: where the run is, what the operator must DO, and a live line of numbers.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:268`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L268).*

<a id="calibratorstatus-phase"></a>

### `CalibratorStatus::phase`

```cpp
Phase phase = Phase::Idle
```

The phase as of the last tick(), answer() or abort().

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:270`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L270).*

<a id="calibratorstatus-prompt"></a>

### `CalibratorStatus::prompt`

```cpp
const char* prompt = "not started"
```

The instruction for the person, as a stable string literal — or the refusal, once aborted.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:272`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L272).*

<a id="calibratorstatus-progress"></a>

### `CalibratorStatus::progress`

```cpp
char progress[96]{}
```

A live line of this phase's numbers, NUL-terminated and bounded.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:274`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L274).*

<a id="class-drivetraincalibrator"></a>

## `class DrivetrainCalibrator`

```cpp
class DrivetrainCalibrator
```

The tick-driven state machine described in the header. Non-owning: the motors, the IMU, the clock and the arrays the spans view must all outlive it. Owns no loop, allocates nothing, blocks nowhere, raises no faults.

*class, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:280`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L280).*

<a id="drivetraincalibrator-drivetraincalibrator"></a>

### `DrivetrainCalibrator::DrivetrainCalibrator`

```cpp
DrivetrainCalibrator(std::span<hal::IMotor* const> candidates, std::span<const std::int8_t> ports, hal::IImu& imu, hal::IClock& clock, const CalibratorConfig& config = {})
```

`candidates`: every motor the brain found, as adapters built with POSITIVE ports (rule 1 in the header). `ports`: their physical port numbers, parallel to `candidates`, each in 1..hal::kMaxSmartPorts and POSITIVE. Preconditions, all of them the caller's contract: the two spans are the same length; there are at least two candidates and at most hal::kMaxSmartPorts; no candidate is null; every port is positive, in range, and listed once.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:288`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L288).*

<a id="drivetraincalibrator-start"></a>

### `DrivetrainCalibrator::start`

```cpp
void start(const HumanFacts& facts)
```

Begin a run from the three stated facts. Preconditions (caller contract, not run-time conditions): the calibrator is Idle, the kind is Tank, the cartridge is a real cartridge, and the IMU port is in range and is not one of the candidate ports. Leaves every candidate at 0 V + Coast so the robot can be pushed, and enters WaitImu.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:335`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L335).*

<a id="drivetraincalibrator-tick"></a>

### `DrivetrainCalibrator::tick`

```cpp
void tick()
```

One control tick. Cheap and non-blocking; the caller paces it (10 ms on the robot). A no-op once the run has ended.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:361`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L361).*

<a id="drivetraincalibrator-answer"></a>

### `DrivetrainCalibrator::answer`

```cpp
void answer(Answer a)
```

The operator's touch. Abort is accepted in any phase; every other answer is accepted only by the step that is waiting for it, and an answer that arrives out of order REFUSES the run naming both — a touch that silently did nothing would leave the operator believing a step had been taken. Precondition: a run is under way (anything but Idle/Done/Aborted), except for Abort, which is always allowed.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:398`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L398).*

<a id="drivetraincalibrator-abort"></a>

### `DrivetrainCalibrator::abort`

```cpp
void abort(const char* why = "aborted by the operator")
```

Stop now: every candidate to 0 V + Coast (never Brake — rule 6), phase Aborted, `why` recorded as the reason. Allowed from ANY phase, including Done, where it discards a result the caller has not copied — call it only when abandoning the run is what is meant.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:449`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L449).*

<a id="drivetraincalibrator-phase"></a>

### `DrivetrainCalibrator::phase`

```cpp
[[nodiscard]] Phase phase() const noexcept
```

Where the run has got to.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:458`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L458).*

<a id="drivetraincalibrator-status"></a>

### `DrivetrainCalibrator::status`

```cpp
[[nodiscard]] const CalibratorStatus& status() const noexcept
```

What a panel paints this tick (see CalibratorStatus).

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:461`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L461).*

<a id="drivetraincalibrator-reason"></a>

### `DrivetrainCalibrator::reason`

```cpp
[[nodiscard]] const char* reason() const noexcept
```

Why the run was refused or aborted — empty while it is still going or if it finished. Every refusal names its cause, and the ones that can name a PORT do.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:465`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L465).*

<a id="drivetraincalibrator-result"></a>

### `DrivetrainCalibrator::result`

```cpp
[[nodiscard]] const hal::DrivetrainDescription& result() const
```

The measured description. Precondition: phase() == Done — a refused run has no result, and asking for one is a caller bug, not a value to be handed back.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:469`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L469).*

<a id="drivetraincalibrator-report"></a>

### `DrivetrainCalibrator::report`

```cpp
[[nodiscard]] const CalibrationReport& report() const noexcept
```

Every number behind the run, filled as far as the run got — worth reading after a refusal.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:477`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L477).*

<a id="drivetraincalibrator-config"></a>

### `DrivetrainCalibrator::config`

```cpp
[[nodiscard]] const CalibratorConfig& config() const noexcept
```

The config in force.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:480`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L480).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 77 lines, click to expand</summary>

```text

 DrivetrainCalibrator — the robot describes its own drivetrain (chunk R3d Part 1, 2026-09-17).

 ── What it is for ──────────────────────────────────────────────────────────────────
 Almost every number a team is made to type about a drivetrain is discoverable on the robot.
 This state machine discovers them. It is handed every motor the brain found, an IMU and a
 clock, and it produces a hal::DrivetrainDescription: which ports drive, which way each one is
 wired, which side each belongs to, each side's inches per radian, and the effective track
 width. The three facts it CANNOT discover — the cartridge (physically unobservable: the rotor
 turns the same for every insert and the firmware already divides by whatever it was told), the
 drivetrain KIND, and which end is the FRONT — are stated by a person, and the front is stated
 PHYSICALLY, by which end they pushed first.

 ── PROVEN ON THE PLANT; NOT YET RUN ON A ROBOT ─────────────────────────────────────
 Every property below is validated against the A2 host plant (sim/drive_plant.hpp) and nothing
 else. No physical drivetrain has ever moved under this class. Where this header says the
 calibrator "measures" something it means "is designed to measure, and does measure on the
 plant". The numbers in CalibratorConfig are INVENTED for a first run — every one of them
 (A4 register HA-136) — and a bench session is what will say which must move.

 ── The phases ──────────────────────────────────────────────────────────────────────
   WaitImu      unpowered   wait for imu.isReady(), bounded.
   PushCapture  UNPOWERED   the operator pushes the whole robot along its length. Every
                            candidate whose shaft moved by more than movedThreshold is a DRIVE
                            port, and the DIRECTION it moved is its SIGN. Then the operator
                            says which end went first; BACK-first inverts every sign. Ports
                            that did not move are not drive motors and are never touched again.
   SideAssign   powered     one member at a time, briefly, at a low ceiling. On a tank, driving
                            ONE side forward pivots the robot about the other: LEFT forward
                            turns it CLOCKWISE, which is NEGATIVE yaw in shulib's CCW-positive
                            frame (hal/imu.hpp), and RIGHT forward is positive — reasoned from
                            rigid-body kinematics, unmeasured on a robot (A4 register HA-134). The pulse also
                            CONFIRMS the captured sign — the pulsed member's own encoder must
                            advance POSITIVE under sign x V — and a contradiction refuses the
                            run naming the port rather than being averaged away.
   ScalePush    UNPOWERED   the operator pushes the robot a chosen distance along a tape. Each
                            side's scale is that distance divided by the MEDIAN of its members'
                            shaft travel. The push's straightness is checked against the IMU.
   SpinTest     powered     spin in place both ways; W = (dR - dL) / dtheta_imu per direction,
                            averaged. On a floor this measures the EFFECTIVE width, which is
                            LARGER than a taped one because a turning tank scrubs (A4 register
                            HA-135). The coupled-side monitor watches both sides throughout.
   Done                     result() is the description; report() is every number behind it.

 ── The rules that make it correct, each with the mistake it prevents ───────────────
 1. ONE PLACE FOR A SIGN, and it moves exactly once. During calibration NO sign is known, so
    the candidates' adapters are constructed with POSITIVE ports and THIS class applies the
    captured sign to a member's command and to its reading. The description it produces carries
    the sign INSIDE the port number, and every later program constructs its adapter from that
    signed port and never negates again. A calibrator that negated a member whose adapter was
    ALSO built from a negative port would capture every sign backwards, consistently — invisible
    on the host, a five-against-four fight on the robot. Hence the precondition: every candidate
    port handed in must be POSITIVE.
 2. NO WRAPPED ANGLE TOUCHES SHAFT ARITHMETIC. IMotor::position() is CUMULATIVE (hal/motor.hpp);
    a math::Angle anywhere in a delta would wrap a two-turn spin to nothing. Shaft travel is
    units::AngleDim and plain doubles throughout.
 3. THE IMU'S dtheta IS ACCUMULATED PER TICK through Angle::errorTo, never taken as
    end-minus-start: a single difference of two wrapped headings cannot see two full turns.
 4. THE MEDIAN, NOT THE MEAN, for each side's scale — and it is hal::medianInPlace, the SAME
    function hal::MotorGroup reports position() with. A member that travels short (robot two's
    port 18, ~20 %, A4 register HA-130) therefore cancels EXACTLY between calibration and run
    time rather than poisoning the scale or being accepted as the truth of the side.
 5. THE PUSH IS POWERLESS. Polarity is found with nothing driven, because a pulse on a coupled
    train whose signs are unknown IS the fight the coupled-side monitor exists to prevent. Only
    members whose sign is already captured are ever pulsed, one at a time.
 6. COAST, NEVER BRAKE, on every refusal and every abort. A calibration that stops with a person
    standing next to the robot must leave the robot pushable.
 7. A MISSED TICK IS A STOP. Every powered phase re-commands the same volts every tick, so a
    caller that stops ticking would leave the motors running. If the clock has advanced by more
    than commandTimeout since the last tick, the calibrator commands 0 V and refuses. (The C2
    stalled-pacer lesson, applied here.)

 Tick-driven and NON-BLOCKING: the caller owns the loop and paces it (10 ms on the robot); the
 operator's answers arrive through answer(), never through a blocking read. Allocation-free
 (every array is sized by hal::kMaxSmartPorts). Raises NO faults — what to do about a refusal is
 the program's policy, exactly as the HAL adapters leave raising to the loop layer. PROS-free
 and sim-free.
```

</details>
