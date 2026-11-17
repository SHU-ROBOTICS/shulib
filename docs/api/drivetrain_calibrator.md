<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/drivetrain_calibrator.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drivetrain_calibrator.hpp`

DrivetrainCalibrator — the robot describes its own drivetrain.

This header declares **12** types (151 members), **5** free functions, and **6** constants.

Extracted from [`include/shulib/calibration/drivetrain_calibrator.hpp`](../../include/shulib/calibration/drivetrain_calibrator.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kDegreesToRadians`](#kdegreestoradians) — *constant*
- [`kReproScaleTolerance`](#kreproscaletolerance) — *constant*
- [`kReproWidthTolerance`](#kreprowidthtolerance) — *constant*
- [`kMaxPulseLevels`](#kmaxpulselevels) — *constant*
- [`kMaxSpinLevels`](#kmaxspinlevels) — *constant*
- [`kMaxPulseRecords`](#kmaxpulserecords) — *constant*
- [`enum class Phase`](#enum-class-phase)
  - [`Idle`](#phase-idle)
  - [`WaitImu`](#phase-waitimu)
  - [`PushCapture`](#phase-pushcapture)
  - [`SideAssign`](#phase-sideassign)
  - [`SpinTest`](#phase-spintest)
  - [`Done`](#phase-done)
  - [`Aborted`](#phase-aborted)
- [`enum class Answer`](#enum-class-answer)
  - [`Done`](#answer-done)
  - [`Abort`](#answer-abort)
- [`enum class Refusal`](#enum-class-refusal)
  - [`None`](#refusal-none)
  - [`Aborted`](#refusal-aborted)
  - [`ImuNotReady`](#refusal-imunotready)
  - [`PushTooFew`](#refusal-pushtoofew)
  - [`PushCurved`](#refusal-pushcurved)
  - [`PushTooShort`](#refusal-pushtooshort)
  - [`RestTimeout`](#refusal-resttimeout)
  - [`NonDriveMoved`](#refusal-nondrivemoved)
  - [`PulseContradiction`](#refusal-pulsecontradiction)
  - [`PulseRolled`](#refusal-pulserolled)
  - [`PulseStill`](#refusal-pulsestill)
  - [`OneSided`](#refusal-onesided)
  - [`SpinWrongWay`](#refusal-spinwrongway)
  - [`SpinNoStart`](#refusal-spinnostart)
  - [`SpinTimeout`](#refusal-spintimeout)
  - [`SpinFight`](#refusal-spinfight)
  - [`SpinWidthInvalid`](#refusal-spinwidthinvalid)
  - [`SpinDisagree`](#refusal-spindisagree)
  - [`MissedTick`](#refusal-missedtick)
  - [`OutOfOrder`](#refusal-outoforder)
  - [`InvalidDescription`](#refusal-invaliddescription)
- [`enum class PulseVerdict`](#enum-class-pulseverdict)
  - [`None`](#pulseverdict-none)
  - [`Weak`](#pulseverdict-weak)
  - [`Left`](#pulseverdict-left)
  - [`Right`](#pulseverdict-right)
  - [`Contradiction`](#pulseverdict-contradiction)
  - [`Decoupled`](#pulseverdict-decoupled)
  - [`Rolled`](#pulseverdict-rolled)
  - [`Still`](#pulseverdict-still)
- [`struct HumanFacts`](#struct-humanfacts)
  - [`kind`](#humanfacts-kind)
  - [`cartridge`](#humanfacts-cartridge)
  - [`imuPort`](#humanfacts-imuport)
  - [`provenance`](#humanfacts-provenance)
- [`struct CalibratorConfig`](#struct-calibratorconfig)
  - [`movedThreshold`](#calibratorconfig-movedthreshold)
  - [`restFloor`](#calibratorconfig-restfloor)
  - [`restYawRateFloor`](#calibratorconfig-restyawratefloor)
  - [`restSettleTime`](#calibratorconfig-restsettletime)
  - [`restTimeout`](#calibratorconfig-resttimeout)
  - [`pulseLevels`](#calibratorconfig-pulselevels)
  - [`pulseLevelCount`](#calibratorconfig-pulselevelcount)
  - [`pulseTime`](#calibratorconfig-pulsetime)
  - [`pulseYawFloor`](#calibratorconfig-pulseyawfloor)
  - [`decoupledOwnMin`](#calibratorconfig-decoupledownmin)
  - [`pushDistance`](#calibratorconfig-pushdistance)
  - [`straightBound`](#calibratorconfig-straightbound)
  - [`minPushRad`](#calibratorconfig-minpushrad)
  - [`spinLevels`](#calibratorconfig-spinlevels)
  - [`spinLevelCount`](#calibratorconfig-spinlevelcount)
  - [`spinStartTime`](#calibratorconfig-spinstarttime)
  - [`spinStartYaw`](#calibratorconfig-spinstartyaw)
  - [`spinWrongWay`](#calibratorconfig-spinwrongway)
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
  - [`pushTravelRad`](#portobservation-pushtravelrad)
  - [`pulseYawRad`](#portobservation-pulseyawrad)
  - [`pulseShaftRad`](#portobservation-pulseshaftrad)
  - [`pulseLevelVolts`](#portobservation-pulselevelvolts)
  - [`pulseOthersRad`](#portobservation-pulseothersrad)
  - [`verdict`](#portobservation-verdict)
  - [`excluded`](#portobservation-excluded)
- [`struct PulseRecord`](#struct-pulserecord)
  - [`port`](#pulserecord-port)
  - [`ordinal`](#pulserecord-ordinal)
  - [`levelVolts`](#pulserecord-levelvolts)
  - [`yawRad`](#pulserecord-yawrad)
  - [`ownRad`](#pulserecord-ownrad)
  - [`othersMaxRad`](#pulserecord-othersmaxrad)
  - [`verdict`](#pulserecord-verdict)
- [`struct SpinRecord`](#struct-spinrecord)
  - [`direction`](#spinrecord-direction)
  - [`levelVolts`](#spinrecord-levelvolts)
  - [`yawRad`](#spinrecord-yawrad)
  - [`leftTravelIn`](#spinrecord-lefttravelin)
  - [`rightTravelIn`](#spinrecord-righttravelin)
  - [`widthIn`](#spinrecord-widthin)
  - [`seconds`](#spinrecord-seconds)
- [`struct CalibrationReport`](#struct-calibrationreport)
  - [`ports`](#calibrationreport-ports)
  - [`portCount`](#calibrationreport-portcount)
  - [`drivePortCount`](#calibrationreport-driveportcount)
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
  - [`pulses`](#calibrationreport-pulses)
  - [`pulseCount`](#calibrationreport-pulsecount)
  - [`spins`](#calibrationreport-spins)
  - [`spinCount`](#calibrationreport-spincount)
  - [`excludedPorts`](#calibrationreport-excludedports)
  - [`excludedCount`](#calibrationreport-excludedcount)
- [`struct CalibratorStatus`](#struct-calibratorstatus)
  - [`phase`](#calibratorstatus-phase)
  - [`prompt`](#calibratorstatus-prompt)
  - [`progress`](#calibratorstatus-progress)
  - [`phaseSeconds`](#calibratorstatus-phaseseconds)
  - [`candidates`](#calibratorstatus-candidates)
  - [`movedNow`](#calibratorstatus-movednow)
  - [`pushTravelRad`](#calibratorstatus-pushtravelrad)
  - [`pushFarEnough`](#calibratorstatus-pushfarenough)
  - [`atRest`](#calibratorstatus-atrest)
  - [`driftRad`](#calibratorstatus-driftrad)
  - [`pulseOrdinal`](#calibratorstatus-pulseordinal)
  - [`pulseTotal`](#calibratorstatus-pulsetotal)
  - [`pulsePort`](#calibratorstatus-pulseport)
  - [`pulseLevelVolts`](#calibratorstatus-pulselevelvolts)
  - [`pulsing`](#calibratorstatus-pulsing)
  - [`liveYawRad`](#calibratorstatus-liveyawrad)
  - [`spinDirection`](#calibratorstatus-spindirection)
  - [`spinTurns`](#calibratorstatus-spinturns)
  - [`spinLevelVolts`](#calibratorstatus-spinlevelvolts)
  - [`leftCount`](#calibratorstatus-leftcount)
  - [`rightCount`](#calibratorstatus-rightcount)
  - [`excludedCount`](#calibratorstatus-excludedcount)
- [`class DrivetrainCalibrator`](#class-drivetraincalibrator)
  - [`DrivetrainCalibrator`](#drivetraincalibrator-drivetraincalibrator)
  - [`DrivetrainCalibrator (overload 2)`](#drivetraincalibrator-drivetraincalibrator-2)
  - [`operator=`](#drivetraincalibrator-operator-eq)
  - [`start`](#drivetraincalibrator-start)
  - [`tick`](#drivetraincalibrator-tick)
  - [`answer`](#drivetraincalibrator-answer)
  - [`abort`](#drivetraincalibrator-abort)
  - [`phase`](#drivetraincalibrator-phase)
  - [`status`](#drivetraincalibrator-status)
  - [`reason`](#drivetraincalibrator-reason)
  - [`refusal`](#drivetraincalibrator-refusal)
  - [`refusalPort`](#drivetraincalibrator-refusalport)
  - [`warning`](#drivetraincalibrator-warning)
  - [`result`](#drivetraincalibrator-result)
  - [`report`](#drivetraincalibrator-report)
  - [`config`](#drivetraincalibrator-config)
  - [`accepts`](#drivetraincalibrator-accepts)
  - [`pushReady`](#drivetraincalibrator-pushready)
- [`refusalName`](#refusalname) — *free function*
- [`pulseVerdictWord`](#pulseverdictword) — *free function*
- [`formatPushRecord`](#formatpushrecord) — *free function*
- [`formatPulseRecord`](#formatpulserecord) — *free function*
- [`formatSpinRecord`](#formatspinrecord) — *free function*

<a id="kdegreestoradians"></a>

## `kDegreesToRadians`

```cpp
inline constexpr double kDegreesToRadians = math::Angle::kPi / 180.0
```

Radians per degree — the thresholds below are stated in the degrees a person can picture on a panel and converted once, here, rather than written as decimal radians nobody can check.

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:170`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L170).*

<a id="kreproscaletolerance"></a>

## `kReproScaleTolerance`

```cpp
inline constexpr double kReproScaleTolerance = 0.02
```

The reproducibility the calibrator CLAIMS for a side's scale: two complete runs of the same procedure on the same drivetrain must agree within this fraction. PROVISIONAL (A4: HA-136) — pinned against the A2 plant under encoder and IMU hostility; a bench session decides whether a real drivetrain holds it.

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:176`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L176).*

<a id="kreprowidthtolerance"></a>

## `kReproWidthTolerance`

```cpp
inline constexpr double kReproWidthTolerance = 0.03
```

The reproducibility the calibrator CLAIMS for the effective track width, as a fraction. Looser than the scale's because the spin measurement runs through the floor's scrub, which is the one thing the host plant does not model. PROVISIONAL (A4: HA-136).

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:181`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L181).*

<a id="kmaxpulselevels"></a>

## `kMaxPulseLevels`

```cpp
inline constexpr std::size_t kMaxPulseLevels = 4
```

The most pulse levels a ramp may have. Four: the default ramp uses three, and a fourth leaves room to try one more without an array growing.

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:185`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L185).*

<a id="kmaxspinlevels"></a>

## `kMaxSpinLevels`

```cpp
inline constexpr std::size_t kMaxSpinLevels = 4
```

The most spin levels the start check may step through.

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:188`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L188).*

<a id="kmaxpulserecords"></a>

## `kMaxPulseRecords`

```cpp
inline constexpr std::size_t kMaxPulseRecords = hal::kMaxSmartPorts * kMaxPulseLevels
```

The most pulse records one run can produce: every port, at every level.

*constant, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:191`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L191).*

<a id="enum-class-phase"></a>

## `enum class Phase`

```cpp
enum class Phase : std::uint8_t
```

Where a run has got to. A run ends in exactly one of Done or Aborted.

*enum class, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:194`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L194).*

<a id="phase-idle"></a>

### `Phase::Idle`

```cpp
Idle
```

constructed, start() not yet called

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:195`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L195).*

<a id="phase-waitimu"></a>

### `Phase::WaitImu`

```cpp
WaitImu
```

waiting for the IMU to finish its boot calibration

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:196`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L196).*

<a id="phase-pushcapture"></a>

### `Phase::PushCapture`

```cpp
PushCapture
```

UNPOWERED: watching the one push for drive ports, their signs and travel

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:197`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L197).*

<a id="phase-sideassign"></a>

### `Phase::SideAssign`

```cpp
SideAssign
```

powered: a ramped pulse per drive port, naming its side from the yaw sign

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:198`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L198).*

<a id="phase-spintest"></a>

### `Phase::SpinTest`

```cpp
SpinTest
```

powered: a spin each way for the effective track width

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:199`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L199).*

<a id="phase-done"></a>

### `Phase::Done`

```cpp
Done
```

result() is valid

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:200`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L200).*

<a id="phase-aborted"></a>

### `Phase::Aborted`

```cpp
Aborted
```

refused or aborted; reason() says why, every motor is at 0 V + Coast

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:201`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L201).*

<a id="enum-class-answer"></a>

## `enum class Answer`

```cpp
enum class Answer : std::uint8_t
```

What the operator's touch means. Everything a person contributes to a calibration arrives through exactly this enum — there is no blocking prompt anywhere in the class. Never persisted (the record stores no answer), so CAL1 Part B's removal of FrontFirst and BackFirst renumbered Abort safely: nothing on a card changed.

*enum class, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:208`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L208).*

<a id="answer-done"></a>

### `Answer::Done`

```cpp
Done
```

"I have finished pushing" — the end that led the push is the FRONT (CAL1)

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:209`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L209).*

<a id="answer-abort"></a>

### `Answer::Abort`

```cpp
Abort
```

stop now

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:210`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L210).*

<a id="enum-class-refusal"></a>

## `enum class Refusal`

```cpp
enum class Refusal : std::uint8_t
```

WHY a run ended in Aborted — a CODE, so a panel can say it in plain words keyed on a value rather than on prose (prose is not an API), and a test can assert it. reason() carries the sentence with the numbers; refusalPort() carries the port where the refusal can name one.

*enum class, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:216`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L216).*

<a id="refusal-none"></a>

### `Refusal::None`

```cpp
None
```

not refused (still running, or Done)

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:217`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L217).*

<a id="refusal-aborted"></a>

### `Refusal::Aborted`

```cpp
Aborted
```

abort() or Answer::Abort — the operator, or the program on the operator's behalf

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:218`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L218).*

<a id="refusal-imunotready"></a>

### `Refusal::ImuNotReady`

```cpp
ImuNotReady
```

the IMU never finished calibrating within imuReadyTimeout

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:219`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L219).*

<a id="refusal-pushtoofew"></a>

### `Refusal::PushTooFew`

```cpp
PushTooFew
```

DONE with fewer than two ports moved by the push

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:220`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L220).*

<a id="refusal-pushcurved"></a>

### `Refusal::PushCurved`

```cpp
PushCurved
```

the push turned the robot by more than straightBound

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:221`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L221).*

<a id="refusal-pushtooshort"></a>

### `Refusal::PushTooShort`

```cpp
PushTooShort
```

the push (or one side of it) turned the shafts less than minPushRad

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:222`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L222).*

<a id="refusal-resttimeout"></a>

### `Refusal::RestTimeout`

```cpp
RestTimeout
```

nothing came to rest within restTimeout (a motor still turning, or the robot still rotating)

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:223`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L223).*

<a id="refusal-nondrivemoved"></a>

### `Refusal::NonDriveMoved`

```cpp
NonDriveMoved
```

a motor that did not move on the push (or gave no reading at DONE) moved while one drive port was pulsed

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:224`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L224).*

<a id="refusal-pulsecontradiction"></a>

### `Refusal::PulseContradiction`

```cpp
PulseContradiction
```

a pulsed member's own encoder went against its captured sign

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:225`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L225).*

<a id="refusal-pulserolled"></a>

### `Refusal::PulseRolled`

```cpp
PulseRolled
```

its train moved at the top level but the robot did not turn (it rolled, or is off the floor)

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:226`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L226).*

<a id="refusal-pulsestill"></a>

### `Refusal::PulseStill`

```cpp
PulseStill
```

nothing moved at the top level

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:227`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L227).*

<a id="refusal-onesided"></a>

### `Refusal::OneSided`

```cpp
OneSided
```

after the pulses, a side has no member

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:228`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L228).*

<a id="refusal-spinwrongway"></a>

### `Refusal::SpinWrongWay`

```cpp
SpinWrongWay
```

the spin turned the wrong way past spinWrongWay — the sides are swapped

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:229`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L229).*

<a id="refusal-spinnostart"></a>

### `Refusal::SpinNoStart`

```cpp
SpinNoStart
```

the spin could not start turning even at the top spin level

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:230`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L230).*

<a id="refusal-spintimeout"></a>

### `Refusal::SpinTimeout`

```cpp
SpinTimeout
```

the spin started but did not finish its turns within spinTimeout

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:231`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L231).*

<a id="refusal-spinfight"></a>

### `Refusal::SpinFight`

```cpp
SpinFight
```

the coupled-side monitor saw a member fight its side during the spin

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:232`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L232).*

<a id="refusal-spinwidthinvalid"></a>

### `Refusal::SpinWidthInvalid`

```cpp
SpinWidthInvalid
```

a spin direction measured a width that is not a positive number

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:233`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L233).*

<a id="refusal-spindisagree"></a>

### `Refusal::SpinDisagree`

```cpp
SpinDisagree
```

the two spin directions disagree by more than spinAgreement

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:234`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L234).*

<a id="refusal-missedtick"></a>

### `Refusal::MissedTick`

```cpp
MissedTick
```

tick() was not called within commandTimeout while powered

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:235`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L235).*

<a id="refusal-outoforder"></a>

### `Refusal::OutOfOrder`

```cpp
OutOfOrder
```

an answer arrived that no step was waiting for

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:236`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L236).*

<a id="refusal-invaliddescription"></a>

### `Refusal::InvalidDescription`

```cpp
InvalidDescription
```

the measured description failed its own validity rules

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:237`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L237).*

<a id="enum-class-pulseverdict"></a>

## `enum class PulseVerdict`

```cpp
enum class PulseVerdict : std::uint8_t
```

What one pulse concluded about the port it drove — one per PulseRecord, and the deciding one on the port's PortObservation.

*enum class, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:242`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L242).*

<a id="pulseverdict-none"></a>

### `PulseVerdict::None`

```cpp
None
```

not pulsed

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:243`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L243).*

<a id="pulseverdict-weak"></a>

### `PulseVerdict::Weak`

```cpp
Weak
```

the yaw stayed under the floor at this level; the next level is tried

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:244`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L244).*

<a id="pulseverdict-left"></a>

### `PulseVerdict::Left`

```cpp
Left
```

negative yaw at or past the floor: a LEFT member (HA-134)

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:245`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L245).*

<a id="pulseverdict-right"></a>

### `PulseVerdict::Right`

```cpp
Right
```

positive yaw at or past the floor: a RIGHT member

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:246`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L246).*

<a id="pulseverdict-contradiction"></a>

### `PulseVerdict::Contradiction`

```cpp
Contradiction
```

its own encoder went against its captured sign — refused

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:247`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L247).*

<a id="pulseverdict-decoupled"></a>

### `PulseVerdict::Decoupled`

```cpp
Decoupled
```

top level: its shaft turned, nothing else moved — a slipping gear, LEFT OUT

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:248`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L248).*

<a id="pulseverdict-rolled"></a>

### `PulseVerdict::Rolled`

```cpp
Rolled
```

top level: other drive ports moved, the robot did not turn — refused

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:249`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L249).*

<a id="pulseverdict-still"></a>

### `PulseVerdict::Still`

```cpp
Still
```

top level: nothing moved — refused

*enumerator, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:250`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L250).*

<a id="struct-humanfacts"></a>

## `struct HumanFacts`

```cpp
struct HumanFacts
```

The three facts no measurement can supply, stated by a person before the run.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:254`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L254).*

<a id="humanfacts-kind"></a>

### `HumanFacts::kind`

```cpp
hal::DrivetrainKind kind = hal::DrivetrainKind::Tank
```

Which drivetrain this is. Only Tank is supported: a holonomic drivetrain needs a strafe scale this procedure does not measure, and the calibrator refuses rather than guess.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:257`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L257).*

<a id="humanfacts-cartridge"></a>

### `HumanFacts::cartridge`

```cpp
hal::MotorCartridge cartridge = hal::MotorCartridge::Blue
```

The cartridge in the drive motors — read off a motor, not inferred (see the header).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:259`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L259).*

<a id="humanfacts-imuport"></a>

### `HumanFacts::imuPort`

```cpp
std::uint8_t imuPort = 0
```

The IMU's smart port, 1..hal::kMaxSmartPorts. Carried into the description so a later boot can check the device is still there.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:262`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L262).*

<a id="humanfacts-provenance"></a>

### `HumanFacts::provenance`

```cpp
const char* provenance = ""
```

The build stamp or session note to record as the description's provenance. Copied into the description's fixed buffer (truncated to fit) — the calibrator keeps no pointer to it past start().

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:266`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L266).*

<a id="struct-calibratorconfig"></a>

## `struct CalibratorConfig`

```cpp
struct CalibratorConfig
```

Every number the procedure uses. NOT ONE OF THEM IS A MEASUREMENT: all are INVENTED for a first run and carried as fields so a bench session can move them without touching code. PROVISIONAL, all of them (A4: HA-136) — `pushDistance` is a CHOICE rather than a belief.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:272`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L272).*

<a id="calibratorconfig-movedthreshold"></a>

### `CalibratorConfig::movedThreshold`

```cpp
units::AngleDim movedThreshold{5.0 * kDegreesToRadians}
```

Shaft travel over the push above which a candidate counts as a DRIVE port — a FINITE travel: a push reading that is not finite is no reading, and never a drive port (DIAG1 Part B). 5 degrees — the bench tester's MOTOR WATCH number, invented there and carried here unchanged so the two tools call the same push the same thing. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:278`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L278).*

<a id="calibratorconfig-restfloor"></a>

### `CalibratorConfig::restFloor`

```cpp
units::AngularVelocity restFloor{0.05}
```

|velocity| below which a motor counts as stopped, for the rest windows. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:281`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L281).*

<a id="calibratorconfig-restyawratefloor"></a>

### `CalibratorConfig::restYawRateFloor`

```cpp
units::AngularVelocity restYawRateFloor{2.0 * kDegreesToRadians}
```

|IMU yaw rate| below which the ROBOT counts as still, for the same rest windows (added at 2F-1: the design always defined rest as "yaw rate and every velocity under a floor", and the code checked the velocities only, so a robot being turned by hand with its motors stationary counted as at rest). 2 deg/s. Bounded BELOW by the IMU's rest noise (unmeasured on a V5; the host's hostile IMU uses 0.5 deg/s, HA-22, so this is 4 sigma there), and ABOVE by the precondition restYawRateFloor x pulseTime < pulseYawFloor — a drift slow enough to pass the rest check then adds less than the floor to a pulse, and a side can only come out WRONG through a leak at least as large as the floor, so a slow drift can never flip a side on its own. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:291`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L291).*

<a id="calibratorconfig-restsettletime"></a>

### `CalibratorConfig::restSettleTime`

```cpp
units::Time restSettleTime{0.5}
```

How long everything must stay at rest before a pulse or a spin may start — so one pulse's coast-down cannot be read as the next pulse's motion. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:294`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L294).*

<a id="calibratorconfig-resttimeout"></a>

### `CalibratorConfig::restTimeout`

```cpp
units::Time restTimeout{3.0}
```

How long to wait for rest before refusing. Without a bound this wait is the one unbounded loop in the procedure. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:297`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L297).*

<a id="calibratorconfig-pulselevels"></a>

### `CalibratorConfig::pulseLevels`

```cpp
std::array<units::Voltage, kMaxPulseLevels> pulseLevels{units::Voltage{3.0}, units::Voltage{6.0}, units::Voltage{9.0}, units::Voltage{0.0}}
```

The pulse ramp: one member is pulsed at pulseLevels[0], then — only if the robot's yaw stayed under pulseYawFloor — at the next level, and so on (added at 2F-1, replacing the single 3 V `pulseVolts`: one member of five delivers a fifth of its pulse to a coupled train under the plant's model, and robot two's port 11 turned the robot 0.14-0.17 deg at 3 V). 3 / 6 / 9 V: the first is the old pulse; 9 V for 0.4 s on ONE motor is a visible jerk, not a drive. Strictly increasing, each in (0, 12] V. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:304`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L304).*

<a id="calibratorconfig-pulselevelcount"></a>

### `CalibratorConfig::pulseLevelCount`

```cpp
std::size_t pulseLevelCount = 3
```

How many entries of pulseLevels are used, 1..kMaxPulseLevels. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:307`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L307).*

<a id="calibratorconfig-pulsetime"></a>

### `CalibratorConfig::pulseTime`

```cpp
units::Time pulseTime{0.4}
```

How long one member is pulsed, at each level. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:309`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L309).*

<a id="calibratorconfig-pulseyawfloor"></a>

### `CalibratorConfig::pulseYawFloor`

```cpp
units::AngleDim pulseYawFloor{2.0 * kDegreesToRadians}
```

The |dtheta| a pulse must produce before its sign may be read as a side. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:312`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L312).*

<a id="calibratorconfig-decoupledownmin"></a>

### `CalibratorConfig::decoupledOwnMin`

```cpp
units::AngleDim decoupledOwnMin{20.0 * kDegreesToRadians}
```

At the top level, a port whose OWN shaft turned at least this far while no other drive port moved movedThreshold is DECOUPLED — its motor turns but does not drive its gear train (a slipping or stripped gear) — and is LEFT OUT rather than refused. 20 degrees of shaft: well past gear backlash, which a train that does not break free still shows. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:318`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L318).*

<a id="calibratorconfig-pushdistance"></a>

### `CalibratorConfig::pushDistance`

```cpp
units::Length pushDistance{48.0}
```

The one push's length along the tape: 48 in, two field tiles. A CHOICE, not a belief — stated here and on the screen, so a wrong distance cannot be answered (there is no distance question any more). Each side's scale is this divided by the side's median travel.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:323`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L323).*

<a id="calibratorconfig-straightbound"></a>

### `CalibratorConfig::straightBound`

```cpp
units::AngleDim straightBound{3.0 * kDegreesToRadians}
```

The |dtheta| the push may drift by before it is not a straight push. Also the bound that keeps a twist from inverting one side's captured signs. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:326`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L326).*

<a id="calibratorconfig-minpushrad"></a>

### `CalibratorConfig::minPushRad`

```cpp
units::AngleDim minPushRad{2.0 * math::Angle::kPi}
```

The least median shaft travel the push must produce — over the moved ports at DONE, and over each side once the sides are known — one full wheel turn. A shorter push divides the distance by a number too small to trust. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:330`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L330).*

<a id="calibratorconfig-spinlevels"></a>

### `CalibratorConfig::spinLevels`

```cpp
std::array<units::Voltage, kMaxSpinLevels> spinLevels{units::Voltage{3.0}, units::Voltage{6.0}, units::Voltage{0.0}, units::Voltage{0.0}}
```

The spin's levels: it starts at spinLevels[0] and steps to the next only when the robot has not turned spinStartYaw within spinStartTime at the current one (added at 2F-1, replacing the single 3 V `spinVolts`). 3 / 6 V. Strictly increasing, each in (0, 12] V. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:335`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L335).*

<a id="calibratorconfig-spinlevelcount"></a>

### `CalibratorConfig::spinLevelCount`

```cpp
std::size_t spinLevelCount = 2
```

How many entries of spinLevels are used, 1..kMaxSpinLevels. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:338`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L338).*

<a id="calibratorconfig-spinstarttime"></a>

### `CalibratorConfig::spinStartTime`

```cpp
units::Time spinStartTime{2.0}
```

How long the spin may run at one level without turning the robot spinStartYaw before it steps up — or, at the top level, refuses: a spin that cannot turn the robot used to stall for the whole spinTimeout. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:342`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L342).*

<a id="calibratorconfig-spinstartyaw"></a>

### `CalibratorConfig::spinStartYaw`

```cpp
units::AngleDim spinStartYaw{5.0 * kDegreesToRadians}
```

The |yaw| a spin level must reach within spinStartTime to count as turning. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:345`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L345).*

<a id="calibratorconfig-spinwrongway"></a>

### `CalibratorConfig::spinWrongWay`

```cpp
units::AngleDim spinWrongWay{15.0 * kDegreesToRadians}
```

How far the spin may turn the WRONG way before it refuses — at once, mid-spin: sides that are swapped consistently cause no fight inside either side, so the coupled-side monitor is silent and nothing else would stop the robot spinning the wrong way under power. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:350`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L350).*

<a id="calibratorconfig-spinturns"></a>

### `CalibratorConfig::spinTurns`

```cpp
double spinTurns = 2.0
```

How many full turns each direction of the spin must cover. More turns average more of the floor and dilute a single scuff; two is the compromise with how long a person will stand and watch. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:354`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L354).*

<a id="calibratorconfig-spintimeout"></a>

### `CalibratorConfig::spinTimeout`

```cpp
units::Time spinTimeout{20.0}
```

How long one direction of the spin may take before the run refuses. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:357`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L357).*

<a id="calibratorconfig-spinagreement"></a>

### `CalibratorConfig::spinAgreement`

```cpp
double spinAgreement = 0.05
```

The largest relative disagreement tolerated between the two directions' track widths. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:360`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L360).*

<a id="calibratorconfig-imureadytimeout"></a>

### `CalibratorConfig::imuReadyTimeout`

```cpp
units::Time imuReadyTimeout{5.0}
```

How long to wait for the IMU's boot calibration. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:362`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L362).*

<a id="calibratorconfig-commandtimeout"></a>

### `CalibratorConfig::commandTimeout`

```cpp
units::Time commandTimeout{0.1}
```

How far the clock may advance between two tick() calls before a powered phase treats the gap as a stalled caller, commands 0 V and refuses. PROVISIONAL (A4: HA-136).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:365`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L365).*

<a id="calibratorconfig-monitor"></a>

### `CalibratorConfig::monitor`

```cpp
teleop::SideMonitorConfig monitor{}
```

The coupled-side fight detector's thresholds for the spin. Defaults are the drive program's and hal::MotorGroup's — one evaluator, one set of numbers (A4: HA-132/133).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:368`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L368).*

<a id="struct-portobservation"></a>

## `struct PortObservation`

```cpp
struct PortObservation
```

What one run concluded about ONE candidate — the panel's table and the log's record.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:372`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L372).*

<a id="portobservation-port"></a>

### `PortObservation::port`

```cpp
std::int8_t port = 0
```

The candidate's port as handed in: always POSITIVE (the sign is captured, not given).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:374`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L374).*

<a id="portobservation-sign"></a>

### `PortObservation::sign`

```cpp
std::int8_t sign = 0
```

The sign captured from the push (the end that led it is the FRONT): +1, −1, or 0 for a candidate that did not move and is therefore not a drive motor — or whose push reading was not finite, which is no reading and no sign (the push record says NO READING).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:378`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L378).*

<a id="portobservation-side"></a>

### `PortObservation::side`

```cpp
std::int8_t side = 0
```

−1 LEFT, +1 RIGHT, 0 = not a drive port, not yet decided, or LEFT OUT.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:380`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L380).*

<a id="portobservation-pushdeltarad"></a>

### `PortObservation::pushDeltaRad`

```cpp
double pushDeltaRad = 0.0
```

Shaft travel over the push, radians, as READ (before the sign is applied) — live while the push is watched, then the number the sign was read off.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:383`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L383).*

<a id="portobservation-pushtravelrad"></a>

### `PortObservation::pushTravelRad`

```cpp
double pushTravelRad = 0.0
```

The same travel sign-corrected (sign × pushDeltaRad): the port's travel in the robot's forward direction — positive, since the end that led the push is the front. Exactly +0.0 for a port that is not a drive motor — a still one, and (since DIAG1 Part B) one whose push reading was not finite (NO READING), where sign × pushDeltaRad would be 0 × ±Inf or 0 × NaN, a NaN.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:389`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L389).*

<a id="portobservation-pulseyawrad"></a>

### `PortObservation::pulseYawRad`

```cpp
double pulseYawRad = 0.0
```

The IMU's accumulated dtheta over this port's DECIDING pulse (its last), radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:391`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L391).*

<a id="portobservation-pulseshaftrad"></a>

### `PortObservation::pulseShaftRad`

```cpp
double pulseShaftRad = 0.0
```

This port's own sign-corrected shaft travel over that pulse, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:393`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L393).*

<a id="portobservation-pulselevelvolts"></a>

### `PortObservation::pulseLevelVolts`

```cpp
double pulseLevelVolts = 0.0
```

The level that pulse ran at, volts (0 if the port was not pulsed).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:395`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L395).*

<a id="portobservation-pulseothersrad"></a>

### `PortObservation::pulseOthersRad`

```cpp
double pulseOthersRad = 0.0
```

The largest |travel| any OTHER drive port showed over that pulse, radians — its side-mates through the gear train, and the other side if the robot rolled.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:398`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L398).*

<a id="portobservation-verdict"></a>

### `PortObservation::verdict`

```cpp
PulseVerdict verdict = PulseVerdict::None
```

What that pulse concluded (None if the port was not pulsed).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:400`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L400).*

<a id="portobservation-excluded"></a>

### `PortObservation::excluded`

```cpp
bool excluded = false
```

True when the port was LEFT OUT as decoupled: it moved on the push, it is on the drivetrain, and it is in no side of the description.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:403`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L403).*

<a id="struct-pulserecord"></a>

## `struct PulseRecord`

```cpp
struct PulseRecord
```

One pulse, as the serial record prints it: which port, at which level, and what it did.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:407`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L407).*

<a id="pulserecord-port"></a>

### `PulseRecord::port`

```cpp
std::int8_t port = 0
```

The pulsed port (positive, as handed in).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:409`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L409).*

<a id="pulserecord-ordinal"></a>

### `PulseRecord::ordinal`

```cpp
std::uint8_t ordinal = 0
```

Which drive port of the run this was, 1-based.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:411`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L411).*

<a id="pulserecord-levelvolts"></a>

### `PulseRecord::levelVolts`

```cpp
double levelVolts = 0.0
```

The level, volts.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:413`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L413).*

<a id="pulserecord-yawrad"></a>

### `PulseRecord::yawRad`

```cpp
double yawRad = 0.0
```

The IMU's accumulated dtheta over the pulse, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:415`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L415).*

<a id="pulserecord-ownrad"></a>

### `PulseRecord::ownRad`

```cpp
double ownRad = 0.0
```

The pulsed member's own sign-corrected shaft travel, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:417`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L417).*

<a id="pulserecord-othersmaxrad"></a>

### `PulseRecord::othersMaxRad`

```cpp
double othersMaxRad = 0.0
```

The largest |travel| any other drive port showed, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:419`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L419).*

<a id="pulserecord-verdict"></a>

### `PulseRecord::verdict`

```cpp
PulseVerdict verdict = PulseVerdict::None
```

What this pulse concluded.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:421`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L421).*

<a id="struct-spinrecord"></a>

## `struct SpinRecord`

```cpp
struct SpinRecord
```

One direction of the spin, as the serial record prints it.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:425`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L425).*

<a id="spinrecord-direction"></a>

### `SpinRecord::direction`

```cpp
double direction = 0.0
```

+1 counter-clockwise, −1 clockwise.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:427`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L427).*

<a id="spinrecord-levelvolts"></a>

### `SpinRecord::levelVolts`

```cpp
double levelVolts = 0.0
```

The level the spin finished at, volts (the start check may have stepped it up).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:429`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L429).*

<a id="spinrecord-yawrad"></a>

### `SpinRecord::yawRad`

```cpp
double yawRad = 0.0
```

The IMU's accumulated dtheta over the direction, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:431`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L431).*

<a id="spinrecord-lefttravelin"></a>

### `SpinRecord::leftTravelIn`

```cpp
double leftTravelIn = 0.0
```

The LEFT side's travel, inches (its median shaft travel times its scale).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:433`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L433).*

<a id="spinrecord-righttravelin"></a>

### `SpinRecord::rightTravelIn`

```cpp
double rightTravelIn = 0.0
```

The RIGHT side's travel, inches.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:435`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L435).*

<a id="spinrecord-widthin"></a>

### `SpinRecord::widthIn`

```cpp
double widthIn = 0.0
```

This direction's effective track width, inches.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:437`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L437).*

<a id="spinrecord-seconds"></a>

### `SpinRecord::seconds`

```cpp
double seconds = 0.0
```

How long the direction took, seconds.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:439`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L439).*

<a id="struct-calibrationreport"></a>

## `struct CalibrationReport`

```cpp
struct CalibrationReport
```

Every number behind a result — for the panel, for the log, and for a human deciding whether to believe the run. Populated as far as the run got, so it is worth reading after a REFUSAL too.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:444`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L444).*

<a id="calibrationreport-ports"></a>

### `CalibrationReport::ports`

```cpp
PortObservation ports[hal::kMaxSmartPorts]{}
```

One row per candidate, in the order they were handed in.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:446`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L446).*

<a id="calibrationreport-portcount"></a>

### `CalibrationReport::portCount`

```cpp
std::uint8_t portCount = 0
```

How many rows of `ports` are real.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:448`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L448).*

<a id="calibrationreport-driveportcount"></a>

### `CalibrationReport::drivePortCount`

```cpp
std::uint8_t drivePortCount = 0
```

How many of them moved on the push and are therefore drive motors.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:450`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L450).*

<a id="calibrationreport-pushdistancein"></a>

### `CalibrationReport::pushDistanceIn`

```cpp
double pushDistanceIn = 0.0
```

The push's length, inches — CalibratorConfig::pushDistance, recorded once the push is taken.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:452`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L452).*

<a id="calibrationreport-pushstraightnessrad"></a>

### `CalibrationReport::pushStraightnessRad`

```cpp
double pushStraightnessRad = 0.0
```

The IMU's dtheta over the push, radians, signed — how straight the push actually was.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:454`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L454).*

<a id="calibrationreport-leftmedianrad"></a>

### `CalibrationReport::leftMedianRad`

```cpp
double leftMedianRad = 0.0
```

The LEFT side's median push travel, radians (the scale's denominator).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:456`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L456).*

<a id="calibrationreport-rightmedianrad"></a>

### `CalibrationReport::rightMedianRad`

```cpp
double rightMedianRad = 0.0
```

The RIGHT side's median push travel, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:458`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L458).*

<a id="calibrationreport-leftinchesperradian"></a>

### `CalibrationReport::leftInchesPerRadian`

```cpp
double leftInchesPerRadian = 0.0
```

The LEFT side's measured scale, inches per radian — available once the sides are known, so a panel (and a host test) can read it without waiting for the spin.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:461`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L461).*

<a id="calibrationreport-rightinchesperradian"></a>

### `CalibrationReport::rightInchesPerRadian`

```cpp
double rightInchesPerRadian = 0.0
```

The RIGHT side's measured scale, inches per radian.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:463`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L463).*

<a id="calibrationreport-leftspreadrad"></a>

### `CalibrationReport::leftSpreadRad`

```cpp
double leftSpreadRad = 0.0
```

max − min of the LEFT side's members' push travel, radians: how far apart a side's motors read. Robot two's port 18 is expected to show here (A4 register HA-130).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:466`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L466).*

<a id="calibrationreport-rightspreadrad"></a>

### `CalibrationReport::rightSpreadRad`

```cpp
double rightSpreadRad = 0.0
```

max − min of the RIGHT side's members' push travel, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:468`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L468).*

<a id="calibrationreport-widthccwin"></a>

### `CalibrationReport::widthCcwIn`

```cpp
double widthCcwIn = 0.0
```

The effective track width from the CCW spin, inches.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:470`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L470).*

<a id="calibrationreport-widthcwin"></a>

### `CalibrationReport::widthCwIn`

```cpp
double widthCwIn = 0.0
```

The effective track width from the CW spin, inches.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:472`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L472).*

<a id="calibrationreport-widthdisagreement"></a>

### `CalibrationReport::widthDisagreement`

```cpp
double widthDisagreement = 0.0
```

|Wccw − Wcw| / their mean — the agreement the run was judged on.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:474`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L474).*

<a id="calibrationreport-pulses"></a>

### `CalibrationReport::pulses`

```cpp
PulseRecord pulses[kMaxPulseRecords]{}
```

Every pulse the run drove, in order, at every level — one serial line each.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:476`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L476).*

<a id="calibrationreport-pulsecount"></a>

### `CalibrationReport::pulseCount`

```cpp
std::uint8_t pulseCount = 0
```

How many entries of `pulses` are real. A program printing the record prints the ones added since it last looked.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:479`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L479).*

<a id="calibrationreport-spins"></a>

### `CalibrationReport::spins`

```cpp
SpinRecord spins[2]{}
```

The spin, one record per finished direction (0 = CCW, 1 = CW).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:481`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L481).*

<a id="calibrationreport-spincount"></a>

### `CalibrationReport::spinCount`

```cpp
std::uint8_t spinCount = 0
```

How many entries of `spins` are real.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:483`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L483).*

<a id="calibrationreport-excludedports"></a>

### `CalibrationReport::excludedPorts`

```cpp
std::int8_t excludedPorts[hal::kMaxSmartPorts]{}
```

The ports LEFT OUT as decoupled, in the order they were found.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:485`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L485).*

<a id="calibrationreport-excludedcount"></a>

### `CalibrationReport::excludedCount`

```cpp
std::uint8_t excludedCount = 0
```

How many entries of `excludedPorts` are real.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:487`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L487).*

<a id="struct-calibratorstatus"></a>

## `struct CalibratorStatus`

```cpp
struct CalibratorStatus
```

What a panel needs to paint this tick: where the run is, what the operator must DO, a live line of prose for a log, and — since 2F-1 — the NUMBERS behind that prose, so a screen can show them large without parsing text.

*struct, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:493`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L493).*

<a id="calibratorstatus-phase"></a>

### `CalibratorStatus::phase`

```cpp
Phase phase = Phase::Idle
```

The phase as of the last tick(), answer() or abort().

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:495`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L495).*

<a id="calibratorstatus-prompt"></a>

### `CalibratorStatus::prompt`

```cpp
const char* prompt = "not started"
```

The instruction for the person — or the refusal's one-line summary, once aborted. Owned by the calibrator and valid until its next tick(), answer() or abort(); at most 44 characters (the width the Part 2 program printed it at — the Part 2F-2 program paints calibrate_panel.hpp's words instead and does not show this; a caller that does has 44 to allow for).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:501`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L501).*

<a id="calibratorstatus-progress"></a>

### `CalibratorStatus::progress`

```cpp
char progress[96]{}
```

A live line of this phase's numbers, NUL-terminated and bounded — for a serial log.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:503`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L503).*

<a id="calibratorstatus-phaseseconds"></a>

### `CalibratorStatus::phaseSeconds`

```cpp
double phaseSeconds = 0.0
```

Seconds since the current phase began, while a run is under way (the IMU wait's live number); once the run has ended, since the last running phase began.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:506`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L506).*

<a id="calibratorstatus-candidates"></a>

### `CalibratorStatus::candidates`

```cpp
std::uint8_t candidates = 0
```

How many candidates were handed in.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:508`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L508).*

<a id="calibratorstatus-movednow"></a>

### `CalibratorStatus::movedNow`

```cpp
std::uint8_t movedNow = 0
```

Candidates whose push travel is past movedThreshold (a finite travel — the same predicate the finish and pushReady() use) — live while the push is watched, the captured count after it.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:512`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L512).*

<a id="calibratorstatus-pushtravelrad"></a>

### `CalibratorStatus::pushTravelRad`

```cpp
double pushTravelRad = 0.0
```

The median |push travel| of the moved candidates, radians (0 with none moved).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:514`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L514).*

<a id="calibratorstatus-pushfarenough"></a>

### `CalibratorStatus::pushFarEnough`

```cpp
bool pushFarEnough = false
```

pushTravelRad ≥ minPushRad — the push's "far enough", the SAME predicate DONE is judged on.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:516`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L516).*

<a id="calibratorstatus-atrest"></a>

### `CalibratorStatus::atRest`

```cpp
bool atRest = false
```

Every motor under restFloor AND the IMU's yaw rate under restYawRateFloor, this tick.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:518`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L518).*

<a id="calibratorstatus-driftrad"></a>

### `CalibratorStatus::driftRad`

```cpp
double driftRad = 0.0
```

The yaw accumulated over the push, radians — live while it is watched, then as captured.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:520`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L520).*

<a id="calibratorstatus-pulseordinal"></a>

### `CalibratorStatus::pulseOrdinal`

```cpp
std::uint8_t pulseOrdinal = 0
```

Which drive port of pulseTotal is being tested, 1-based (0 before the pulses start).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:522`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L522).*

<a id="calibratorstatus-pulsetotal"></a>

### `CalibratorStatus::pulseTotal`

```cpp
std::uint8_t pulseTotal = 0
```

How many drive ports the pulses will test.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:524`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L524).*

<a id="calibratorstatus-pulseport"></a>

### `CalibratorStatus::pulsePort`

```cpp
std::int8_t pulsePort = 0
```

The port under test (0 when none).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:526`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L526).*

<a id="calibratorstatus-pulselevelvolts"></a>

### `CalibratorStatus::pulseLevelVolts`

```cpp
double pulseLevelVolts = 0.0
```

The ramp level being tried, volts (0 when none).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:528`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L528).*

<a id="calibratorstatus-pulsing"></a>

### `CalibratorStatus::pulsing`

```cpp
bool pulsing = false
```

True while a pulse is being DRIVEN; false in the rest windows around it.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:530`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L530).*

<a id="calibratorstatus-liveyawrad"></a>

### `CalibratorStatus::liveYawRad`

```cpp
double liveYawRad = 0.0
```

The yaw since the current pulse, spin direction or rest window began, radians.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:532`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L532).*

<a id="calibratorstatus-spindirection"></a>

### `CalibratorStatus::spinDirection`

```cpp
double spinDirection = 0.0
```

+1 while spinning counter-clockwise, −1 clockwise, 0 when not spinning.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:534`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L534).*

<a id="calibratorstatus-spinturns"></a>

### `CalibratorStatus::spinTurns`

```cpp
double spinTurns = 0.0
```

Turns covered in the current spin direction, signed by that direction (negative = the wrong way).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:537`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L537).*

<a id="calibratorstatus-spinlevelvolts"></a>

### `CalibratorStatus::spinLevelVolts`

```cpp
double spinLevelVolts = 0.0
```

The spin level in force, volts (0 when not spinning).

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:539`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L539).*

<a id="calibratorstatus-leftcount"></a>

### `CalibratorStatus::leftCount`

```cpp
std::uint8_t leftCount = 0
```

Drive ports decided LEFT so far.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:541`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L541).*

<a id="calibratorstatus-rightcount"></a>

### `CalibratorStatus::rightCount`

```cpp
std::uint8_t rightCount = 0
```

Drive ports decided RIGHT so far.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:543`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L543).*

<a id="calibratorstatus-excludedcount"></a>

### `CalibratorStatus::excludedCount`

```cpp
std::uint8_t excludedCount = 0
```

Drive ports LEFT OUT as decoupled so far.

*field, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:545`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L545).*

<a id="class-drivetraincalibrator"></a>

## `class DrivetrainCalibrator`

```cpp
class DrivetrainCalibrator
```

The tick-driven state machine described in the header. Non-owning: the motors, the IMU, the clock and the arrays the spans view must all outlive it. Owns no loop, allocates nothing, blocks nowhere, raises no faults. Not copyable or movable (it holds references); RUN AGAIN is a new object (the header).

*class, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:552`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L552).*

<a id="drivetraincalibrator-drivetraincalibrator"></a>

### `DrivetrainCalibrator::DrivetrainCalibrator`

```cpp
DrivetrainCalibrator(std::span<hal::IMotor* const> candidates, std::span<const std::int8_t> ports, hal::IImu& imu, hal::IClock& clock, const CalibratorConfig& config = {})
```

`candidates`: every motor the brain found, as adapters built with POSITIVE ports (rule 1 in the header). `ports`: their physical port numbers, parallel to `candidates`, each in 1..hal::kMaxSmartPorts and POSITIVE. Preconditions, all of them the caller's contract: the two spans are the same length; there are at least two candidates and at most hal::kMaxSmartPorts; no candidate is null; every port is positive, in range, and listed once; every config threshold is positive, the level ramps are strictly increasing within (0, 12] V, and restYawRateFloor x pulseTime < pulseYawFloor (CalibratorConfig).

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:561`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L561).*

<a id="drivetraincalibrator-drivetraincalibrator-2"></a>

### `DrivetrainCalibrator::DrivetrainCalibrator (overload 2)`

```cpp
DrivetrainCalibrator(const DrivetrainCalibrator&) = delete
```

Not copyable: it holds references to hardware it does not own, and a copy would be a second state machine commanding the same motors.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:628`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L628).*

<a id="drivetraincalibrator-operator-eq"></a>

### `DrivetrainCalibrator::operator=`

```cpp
DrivetrainCalibrator& operator=(const DrivetrainCalibrator&) = delete
```

Not copyable: it holds references to hardware it does not own, and a copy would be a second state machine commanding the same motors. Not copy-assignable, for the same reason.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:630`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L630).*

<a id="drivetraincalibrator-start"></a>

### `DrivetrainCalibrator::start`

```cpp
void start(const HumanFacts& facts)
```

Begin a run from the three stated facts. Preconditions (caller contract, not run-time conditions): the calibrator is Idle, the kind is Tank, the cartridge is a real cartridge, and the IMU port is in range and is not one of the candidate ports. Leaves every candidate at 0 V + Coast so the robot can be pushed, and enters WaitImu.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:636`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L636).*

<a id="drivetraincalibrator-tick"></a>

### `DrivetrainCalibrator::tick`

```cpp
void tick()
```

One control tick. Cheap and non-blocking; the caller paces it (10 ms on the robot). A no-op once the run has ended.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:662`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L662).*

<a id="drivetraincalibrator-answer"></a>

### `DrivetrainCalibrator::answer`

```cpp
void answer(Answer a)
```

The operator's touch. Abort is accepted in any phase; every other answer is accepted only by the step that is waiting for it, and an answer that arrives out of order REFUSES the run naming both — a touch that silently did nothing would leave the operator believing a step had been taken. Precondition: a run is under way (anything but Idle/Done/Aborted), except for Abort, which is always allowed. Unchanged at 2F-1: DONE before the push is ready is still ACCEPTED here and judged by the finish (which refuses too few ports, a curved push or a short one); a panel keeps DONE off the screen until pushReady() — the gate is a readiness query, not a new refusal.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:701`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L701).*

<a id="drivetraincalibrator-abort"></a>

### `DrivetrainCalibrator::abort`

```cpp
void abort(const char* why = "aborted by the operator")
```

Stop now: every candidate to 0 V + Coast (never Brake — rule 6), phase Aborted with Refusal::Aborted, `why` recorded as the reason. Allowed from ANY phase, including Done, where it discards a result the caller has not copied — call it only when abandoning the run is what is meant.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:730`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L730).*

<a id="drivetraincalibrator-phase"></a>

### `DrivetrainCalibrator::phase`

```cpp
[[nodiscard]] Phase phase() const noexcept
```

Where the run has got to.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:741`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L741).*

<a id="drivetraincalibrator-status"></a>

### `DrivetrainCalibrator::status`

```cpp
[[nodiscard]] const CalibratorStatus& status() const noexcept
```

What a panel paints this tick (see CalibratorStatus).

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:744`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L744).*

<a id="drivetraincalibrator-reason"></a>

### `DrivetrainCalibrator::reason`

```cpp
[[nodiscard]] const char* reason() const noexcept
```

Why the run was refused or aborted — empty while it is still going or if it finished. Every refusal names its cause, the ones that can name a PORT do, and none contains a '%' (the header). The code is refusal().

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:749`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L749).*

<a id="drivetraincalibrator-refusal"></a>

### `DrivetrainCalibrator::refusal`

```cpp
[[nodiscard]] Refusal refusal() const noexcept
```

Which refusal ended the run — Refusal::None while it runs or once it is Done.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:752`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L752).*

<a id="drivetraincalibrator-refusalport"></a>

### `DrivetrainCalibrator::refusalPort`

```cpp
[[nodiscard]] int refusalPort() const noexcept
```

The port the refusal names, positive — or 0 when it names none (a push too short, a spin that disagreed, an abort).

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:756`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L756).*

<a id="drivetraincalibrator-warning"></a>

### `DrivetrainCalibrator::warning`

```cpp
[[nodiscard]] const char* warning() const noexcept
```

The warnings a run that is NOT refused still owes the operator, joined by " | " — one sentence per port LEFT OUT as decoupled ("port 18 turns but does not move the robot -- its gear is slipping; it is left out"), and (since DIAG1 Part B) one per candidate whose push reading was not finite at DONE ("port 13 gave no reading at DONE -- not counted as a drive port; if it drives the robot, check it and run again"). Empty when there is none. Bounded: a sentence that does not fit is cut, and the report still names every port. ASCII, no '%'.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:764`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L764).*

<a id="drivetraincalibrator-result"></a>

### `DrivetrainCalibrator::result`

```cpp
[[nodiscard]] const hal::DrivetrainDescription& result() const
```

The measured description. Precondition: phase() == Done — a refused run has no result, and asking for one is a caller bug, not a value to be handed back.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:768`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L768).*

<a id="drivetraincalibrator-report"></a>

### `DrivetrainCalibrator::report`

```cpp
[[nodiscard]] const CalibrationReport& report() const noexcept
```

Every number behind the run, filled as far as the run got — worth reading after a refusal.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:776`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L776).*

<a id="drivetraincalibrator-config"></a>

### `DrivetrainCalibrator::config`

```cpp
[[nodiscard]] const CalibratorConfig& config() const noexcept
```

The config in force.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:779`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L779).*

<a id="drivetraincalibrator-accepts"></a>

### `DrivetrainCalibrator::accepts`

```cpp
[[nodiscard]] bool accepts(Answer a) const noexcept
```

True when answer(`a`) would be ACCEPTED right now — i.e. when a panel may OFFER it. Abort is always accepted; everything else only by the step that is waiting for it.  ── WHY THIS EXISTS (added at R3d Part 2, for the calibrate program's button row) ── answer() REFUSES an out-of-order answer and ends the run, which is right (a touch that silently did nothing would leave the operator believing a step had been taken) — but it makes a wrong BUTTON destructive. A panel therefore has to know which answers are live, and Phase alone cannot tell it: PushCapture is two steps (watching, then awaiting FRONT/BACK). Before this query the only public signal was the PROSE of status().prompt, and prose is not an API.  It is a PURE QUERY over the same step answer() switches on. It duplicates that switch, so the anti-drift device is a test rather than a comment: drivetrain_calibrator_test.cpp case 21 drives the run to each waiting step and asserts, for EVERY Answer, that accepts() agrees with what answer() actually does on a fresh run. accepts() says whether an answer is LEGAL; pushReady() says whether DONE is WISE.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:797`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L797).*

<a id="drivetraincalibrator-pushready"></a>

### `DrivetrainCalibrator::pushReady`

```cpp
[[nodiscard]] bool pushReady() const
```

True when the push is PLAUSIBLY COMPLETE, so a panel may OFFER DONE (added at 2F-1 — two of robot two's five starts ended because DONE was touched before any push, on a screen that offered it with nothing moved). All three of: at least two candidates past movedThreshold (a FINITE travel — the finish's own movedOnPush() predicate, so a candidate with no reading is counted by neither); their median |travel| at least minPushRad (the push's "far enough"); and everything at rest — every motor under restFloor AND the IMU's yaw rate under restYawRateFloor. Read LIVE from the motors and the IMU, through the SAME helpers the finish uses (pushSummary(), atRest()), so the gate cannot drift from the finish: DONE with the first two false is refused by the finish (PushTooFew, PushTooShort) under the same predicate. At-rest is the one clause the finish does NOT refuse on (the brief's "a readiness query, not a new refusal"): the gate is STRICTER than the finish, never looser. False outside the push.

*function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:819`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L819).*

<a id="refusalname"></a>

## `refusalName`

```cpp
[[nodiscard]] constexpr const char* refusalName(Refusal r) noexcept
```

A short, stable, ASCII name for a refusal code — for the serial record ("[PULSE_ROLLED port 11]") and for a test's failure message. Never shown as the operator's explanation: the panel's words are calibrate_panel.hpp's.

*free function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:1878`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L1878).*

<a id="pulseverdictword"></a>

## `pulseVerdictWord`

```cpp
[[nodiscard]] constexpr const char* pulseVerdictWord(PulseVerdict v) noexcept
```

A short, stable, ASCII word for a pulse verdict — the last column of a pulse's serial line.

*free function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:1906`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L1906).*

<a id="formatpushrecord"></a>

## `formatPushRecord`

```cpp
inline int formatPushRecord(const PortObservation& o, char* out, std::size_t cap)
```

The three record formatters below add 0.0 to every signed number they print: a travel of exactly nothing times a captured sign of −1 is IEEE negative zero, and "own -0.0 deg" in a log reads as a motor that moved backward. (x + 0.0 is +0.0 for x = −0.0 and x for everything else.)  One line of the PUSH TABLE for the serial record (F1.5): the candidate's port, its travel as read, whether it moved and the sign captured — `push port 11 travel -373.2 deg MOVED sign -1`. A candidate whose push reading was not finite keeps its port and its reading, and its word is `NO READING` instead of `still` (DIAG1 Part B: it is not a drive port, and unlike a still one it is a motor the run could not read — the number is spelled the C library's way, e.g. `+inf`). Written into `out` (NUL-terminated, cut to `cap`); returns what std::snprintf returns. ASCII, no '%'.

*free function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:1931`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L1931).*

<a id="formatpulserecord"></a>

## `formatPulseRecord`

```cpp
inline int formatPulseRecord(const PulseRecord& r, unsigned total, char* out, std::size_t cap)
```

One line per PULSE for the serial record (F1.5) — the numbers that would have explained robot two's start B: `pulse port 12 (3 of 9) 6.0 V yaw -2.31 deg own +41.2 deg others 38.9 deg: LEFT`. `total` is the run's drive-port count (the report's drivePortCount). ASCII, no '%'.

*free function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:1943`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L1943).*

<a id="formatspinrecord"></a>

## `formatSpinRecord`

```cpp
inline int formatSpinRecord(const SpinRecord& r, char* out, std::size_t cap)
```

One line per SPIN DIRECTION for the serial record (F1.5): `spin CCW 3.0 V yaw +720.0 deg L -78.54 in R +78.54 in W 12.500 in 7.41 s`. ASCII, no '%'.

*free function, declared at [`include/shulib/calibration/drivetrain_calibrator.hpp:1955`](../../include/shulib/calibration/drivetrain_calibrator.hpp#L1955).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 144 lines, click to expand</summary>

```text

 DrivetrainCalibrator — the robot describes its own drivetrain (chunk R3d Part 1, 2026-09-17;
 REBUILT at R3d Part 2F-1, 2026-09-24, after its first night on robot two).

 ── What it is for ──────────────────────────────────────────────────────────────────
 Almost every number a team is made to type about a drivetrain is discoverable on the robot.
 This state machine discovers them. It is handed every motor the brain found, an IMU and a
 clock, and it produces a hal::DrivetrainDescription: which ports drive, which way each one is
 wired, which side each belongs to, each side's inches per radian, and the effective track
 width. The three facts it CANNOT discover — the cartridge (physically unobservable: the rotor
 turns the same for every insert and the firmware already divides by whatever it was told), the
 drivetrain KIND, and which end is the FRONT — are stated by a person, and the front is stated
 PHYSICALLY: the end that LEADS the push is the FRONT (the team lead's ruling, 2026-10-06; until
 CAL1 Part B, 2026-11-05, the operator was asked afterwards which end went first — that question,
 Answer::FrontFirst / BackFirst and CalibrationReport::backFirst are gone). A push led with the
 other end yields every sign and both sides the other way round, which nothing in this class can
 see; a program compares the result with the card's record (calibration/front_check.hpp).

 ── PROVEN ON THE PLANT; IT HAS NEVER FINISHED ON A ROBOT ───────────────────────────
 Every property below is validated against the A2 host plant (sim/drive_plant.hpp) and nothing
 else. The Part 1 version of this class ran five times on robot two on 2026-09-24 and never
 finished — every run stopped at its side-assignment pulse or before it — and THIS version has
 never run on a robot at all. No physical drivetrain has ever been described by it. Where this
 header says the calibrator "measures" something it means "is designed to measure, and does
 measure on the plant". The numbers in CalibratorConfig are INVENTED — every one of them (A4
 register HA-136) — and the serial record this class exposes is what will say which must move.

 ── What Part 2F-1 changed, and why (each a finding of that first night) ────────────
  * ONE PUSH, not two. The operator pushes the robot forward CalibratorConfig::pushDistance
    (48 in) along a tape, once. It yields the drive ports, their signs AND each port's travel;
    each side's scale is computed from that travel once the pulses have named the sides. The
    distance question is gone: only one distance exists, so a wrong one cannot be picked.
  * The side pulse RAMPS (3 → 6 → 9 V) and stops at the first level that turns the robot past
    the yaw floor. On the host plant at the tree's usual static friction, one member of five at
    the old single 3 V pulse turns the robot by EXACTLY nothing — the refusal robot two gave —
    because the old test rig ran with no static friction at all. When no level turns the robot,
    the pulse is CLASSIFIED instead of refused blind: a motor that turns but moves nothing
    (a slipping gear) is LEFT OUT with a warning and the run goes on; a robot that rolled instead
    of turning, or that nothing moved, is refused naming the port and saying which.
  * REST includes the IMU's yaw rate, as the design always said and the code did not: hand
    motion before a pulse used to leak straight into the pulse's yaw.
  * The spin gained two SAFETY guards: it refuses at once if it turns the WRONG way (a side set
    swapped consistently causes no fight, so nothing used to stop it for 20 s), and it steps up
    once and then refuses if it cannot START turning (instead of stalling for 20 s).
  * Numbers for a panel, not prose: CalibratorStatus carries the live counts and angles, a
    Refusal code names every refusal, pushReady() is the DONE gate built from the finish's own
    predicates, and the report carries one record per push port, per pulse and per spin
    direction for the serial log (calibrate_panel.hpp is the pure screen model built on them).
  * Part 2F-2 (2026-09-24) removed the three names 2F-1 had kept only so the Part 2 program
    still compiled: Phase::ScalePush (the separate tape push), Answer::Distance24/48/72 (the
    distance question) and PortObservation::tapeDeltaRad. The scale's input is the one push's
    pushTravelRad; there is no distance to answer.

 ── The phases ──────────────────────────────────────────────────────────────────────
   WaitImu      unpowered   wait for imu.isReady(), bounded.
   PushCapture  UNPOWERED   the operator pushes the whole robot FORWARD pushDistance along a
                            tape, straight. Every candidate whose shaft moved by more than
                            movedThreshold is a DRIVE port and the DIRECTION it moved is its
                            SIGN; its travel is kept for the scale. The push must be straight
                            (straightBound) and long enough (minPushRad). The end that LED the
                            push is the FRONT: the signs stand as captured. Ports that
                            did not move are not drive motors and are never touched again.
                            A candidate whose push reading is NOT FINITE (NaN, ±Inf) did not
                            "move" either — it gave no reading — so it is not a drive port;
                            unlike a still one it is NAMED, in the push record (NO READING)
                            and in warning() (DIAG1 Part B; one predicate, movedOnPush()).
                            Proven on the host test plant only: on a V5 no push reading is
                            expected to be non-finite, because ProsMotor screens a failed read
                            to the last good one first (A4 register HA-08).
   SideAssign   powered     one member at a time, a brief pulse at a RAMP of levels. On a tank,
                            driving ONE side forward turns the robot toward the other: LEFT
                            forward turns it CLOCKWISE, which is NEGATIVE yaw in shulib's
                            CCW-positive frame (hal/imu.hpp), and RIGHT forward is positive —
                            reasoned from rigid-body kinematics, unmeasured on a robot (A4
                            register HA-134). The pulse also CONFIRMS the captured sign — the
                            pulsed member's own encoder must advance POSITIVE under sign x V —
                            and a contradiction refuses the run naming the port. A port no level
                            can turn the robot with is classified (PulseVerdict). Once every
                            port is decided, each side's scale is pushDistance divided by the
                            MEDIAN of its members' push travel.
   SpinTest     powered     spin in place both ways; W = (dR - dL) / dtheta_imu per direction,
                            averaged. On a floor this measures the EFFECTIVE width, which is
                            LARGER than a taped one because a turning tank scrubs (A4 register
                            HA-135). The coupled-side monitor watches both sides throughout; the
                            wrong-way guard and the start check watch the yaw.
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
    time rather than poisoning the scale or being accepted as the truth of the side. Since
    DIAG1 Part B that median leaves out a value that is not finite, as the group does; the
    push's own medians only ever see finite values (a non-finite push reading is not a drive
    port, rule of the PushCapture phase above), so the one site that can meet one is the
    spin's side travel, which reads the shafts live.
 5. THE PUSH IS POWERLESS. Polarity is found with nothing driven, because a pulse on a coupled
    train whose signs are unknown IS the fight the coupled-side monitor exists to prevent. Only
    members whose sign is already captured are ever pulsed, one at a time, and a member left
    out as slipping is never commanded again.
 6. COAST, NEVER BRAKE, on every refusal, every abort and every pulse's idle members. A
    calibration that stops with a person standing next to the robot must leave the robot
    pushable.
 7. A MISSED TICK IS A STOP. Every powered phase re-commands the same volts every tick, so a
    caller that stops ticking would leave the motors running. If the clock has advanced by more
    than commandTimeout since the last tick, the calibrator commands 0 V and refuses. (The C2
    stalled-pacer lesson, applied here.) The ramp's levels and the spin's step-up are
    re-commanded every tick like everything else.

 Tick-driven and NON-BLOCKING: the caller owns the loop and paces it (10 ms on the robot); the
 operator's answers arrive through answer(), never through a blocking read. Allocation-free
 (every array is sized by hal::kMaxSmartPorts). Raises NO faults — what to do about a refusal is
 the program's policy, exactly as the HAL adapters leave raising to the loop layer. PROS-free
 and sim-free.

 ── RUN AGAIN: construct a new one (Part 2F-1) ──────────────────────────────────────
 There is deliberately no reset(). A program offering RUN AGAIN rebuilds the calibrator in
 place — std::optional<DrivetrainCalibrator>::emplace(candidates, ports, imu, clock, config),
 then start() — which carries NOTHING over by construction: no sign, no side, no report row.
 A reset() would have to remember every field added later, and the one it forgot would leak
 between runs. The IMU is already calibrated by then, so WaitImu passes on its first tick.

 ── EVERY RUNTIME STRING IN THIS FILE IS ASCII, AND NONE CONTAINS A PERCENT SIGN ────
 Prompts, progress lines, refusals and records are painted on the V5's screen and read over
 USB serial. PROS 4.2.2 draws text through VEXos's own display routines (not LVGL, as this note
 said until 2F-1 — a read of the vendored libpros.a corrected it), whose fonts carry ASCII only,
 so a UTF-8 em dash renders as a missing glyph and as mojibake in a raw `cat` of the user port.
 And PROS hands the already-formatted text to VEXos AS A FORMAT STRING, so a '%' that reaches
 the screen is re-read as a conversion with no argument behind it: the spin-disagreement
 refusal printed two of them until 2F-1 ("pct" now). src/ has used "--" since R3a. The COMMENTS
 in this file keep their typography: they are read in an editor.
```

</details>
