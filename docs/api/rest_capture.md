<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/rest_capture.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `rest_capture.hpp`

RestCapture — shulib Calibrate's REST step: the robot sits still, unpowered, for a minute while the IMU's drift and noise, the encoders' noise at rest and the loop's timing are measured.

This header declares **7** types (50 members) and **6** free functions.

Extracted from [`include/shulib/calibration/rest_capture.hpp`](../../include/shulib/calibration/rest_capture.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct SideMember`](#struct-sidemember)
  - [`motor`](#sidemember-motor)
  - [`sign`](#sidemember-sign)
  - [`port`](#sidemember-port)
- [`struct DriveSide`](#struct-driveside)
  - [`members`](#driveside-members)
  - [`inchesPerRadian`](#driveside-inchesperradian)
- [`sideSpeed`](#sidespeed) — *free function*
- [`sidePosition`](#sideposition) — *free function*
- [`sideTravel`](#sidetravel) — *free function*
- [`driveSideValid`](#drivesidevalid) — *free function*
- [`struct RestCaptureConfig`](#struct-restcaptureconfig)
  - [`duration`](#restcaptureconfig-duration)
  - [`moveBound`](#restcaptureconfig-movebound)
  - [`turnBound`](#restcaptureconfig-turnbound)
  - [`settleMarginWarn`](#restcaptureconfig-settlemarginwarn)
  - [`restFloor`](#restcaptureconfig-restfloor)
  - [`restYawRateFloor`](#restcaptureconfig-restyawratefloor)
  - [`restSettleTime`](#restcaptureconfig-restsettletime)
  - [`restTimeout`](#restcaptureconfig-resttimeout)
  - [`imuReadyTimeout`](#restcaptureconfig-imureadytimeout)
  - [`timing`](#restcaptureconfig-timing)
  - [`valid`](#restcaptureconfig-valid)
- [`enum class RestPhase`](#enum-class-restphase)
  - [`Idle`](#restphase-idle)
  - [`WaitImu`](#restphase-waitimu)
  - [`Settle`](#restphase-settle)
  - [`Capture`](#restphase-capture)
  - [`Done`](#restphase-done)
  - [`Aborted`](#restphase-aborted)
- [`enum class RestRefusal`](#enum-class-restrefusal)
  - [`None`](#restrefusal-none)
  - [`Aborted`](#restrefusal-aborted)
  - [`ImuNotReady`](#restrefusal-imunotready)
  - [`RestTimeout`](#restrefusal-resttimeout)
  - [`Moved`](#restrefusal-moved)
  - [`Turned`](#restrefusal-turned)
- [`class RestCapture`](#class-restcapture)
  - [`RestCapture`](#restcapture-restcapture)
  - [`RestCapture (overload 2)`](#restcapture-restcapture-2)
  - [`operator=`](#restcapture-operator-eq)
  - [`start`](#restcapture-start)
  - [`tick`](#restcapture-tick)
  - [`abort`](#restcapture-abort)
  - [`phase`](#restcapture-phase)
  - [`refusal`](#restcapture-refusal)
  - [`reason`](#restcapture-reason)
  - [`refusalPort`](#restcapture-refusalport)
  - [`captureSeconds`](#restcapture-captureseconds)
  - [`unreadSides`](#restcapture-unreadsides)
  - [`summary`](#restcapture-summary)
  - [`config`](#restcapture-config)
- [`struct SettleFloorReport`](#struct-settlefloorreport)
  - [`headingRateNoise`](#settlefloorreport-headingratenoise)
  - [`headingRateFloor`](#settlefloorreport-headingratefloor)
  - [`headingMargin`](#settlefloorreport-headingmargin)
  - [`headingWarn`](#settlefloorreport-headingwarn)
  - [`speedNoise`](#settlefloorreport-speednoise)
  - [`speedFloor`](#settlefloorreport-speedfloor)
  - [`speedMargin`](#settlefloorreport-speedmargin)
  - [`speedWarn`](#settlefloorreport-speedwarn)
- [`settleFloorReport`](#settlefloorreport) — *free function*
- [`restRefusalName`](#restrefusalname) — *free function*

<a id="struct-sidemember"></a>

## `struct SideMember`

```cpp
struct SideMember
```

One motor of a drive side, as the GAINS and REST steps read and command it.

*struct, declared at [`include/shulib/calibration/rest_capture.hpp:131`](../../include/shulib/calibration/rest_capture.hpp#L131).*

<a id="sidemember-motor"></a>

### `SideMember::motor`

```cpp
hal::IMotor* motor = nullptr
```

The motor — non-owning, must outlive the step. In shulib Calibrate its adapter is built with a POSITIVE port and `sign` is applied HERE, once (drivetrain_calibrator.hpp's rule 1); a program whose adapters already carry the sign in the port passes sign +1.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:135`](../../include/shulib/calibration/rest_capture.hpp#L135).*

<a id="sidemember-sign"></a>

### `SideMember::sign`

```cpp
std::int8_t sign = 1
```

+1 or −1: multiplies every command sent to the motor and every reading taken from it.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:137`](../../include/shulib/calibration/rest_capture.hpp#L137).*

<a id="sidemember-port"></a>

### `SideMember::port`

```cpp
std::int8_t port = 0
```

The physical port, 1..21 — for the messages that name a port. Positive.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:139`](../../include/shulib/calibration/rest_capture.hpp#L139).*

<a id="struct-driveside"></a>

## `struct DriveSide`

```cpp
struct DriveSide
```

One drive side: its members and its scale (inches of surface travel per radian of shaft) — both from the drivetrain record (the description's DriveSideDescription).

*struct, declared at [`include/shulib/calibration/rest_capture.hpp:144`](../../include/shulib/calibration/rest_capture.hpp#L144).*

<a id="driveside-members"></a>

### `DriveSide::members`

```cpp
std::span<const SideMember> members{}
```

The side's members, 1..hal::kMaxSmartPorts of them.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:146`](../../include/shulib/calibration/rest_capture.hpp#L146).*

<a id="driveside-inchesperradian"></a>

### `DriveSide::inchesPerRadian`

```cpp
units::Length inchesPerRadian{0.0}
```

The side's inches per radian, > 0 — the description's measured scale. Every speed and travel the steps report, and so every gain the GAINS step fits, is in THIS scale's inches (R5a's landmine 7: a re-calibrated drivetrain changes every gain's units).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:150`](../../include/shulib/calibration/rest_capture.hpp#L150).*

<a id="sidespeed"></a>

## `sideSpeed`

```cpp
[[nodiscard]] inline double sideSpeed(const DriveSide& side)
```

The side's surface speed now, in/s: the MEDIAN over its members of sign × velocity() — the statistic hal::MotorGroup reports, through the same hal::medianInPlace — times its scale.

*free function, declared at [`include/shulib/calibration/rest_capture.hpp:155`](../../include/shulib/calibration/rest_capture.hpp#L155).*

<a id="sideposition"></a>

## `sidePosition`

```cpp
[[nodiscard]] inline double sidePosition(const DriveSide& side)
```

The side's position now, in: the MEDIAN over its members of sign × position(), times its scale — cumulative, as IMotor::position() is (the raw CSV's column; a travel is sideTravel()).

*free function, declared at [`include/shulib/calibration/rest_capture.hpp:167`](../../include/shulib/calibration/rest_capture.hpp#L167).*

<a id="sidetravel"></a>

## `sideTravel`

```cpp
[[nodiscard]] inline double sideTravel(const DriveSide& side, const double* base)
```

The side's travel since `base` (each member's position() at the baseline, parallel to side.members), in: the MEDIAN of the members' sign-corrected DELTAS, times the scale — the calibrator's sideTravel(): members whose encoders started at different counts still agree.

*free function, declared at [`include/shulib/calibration/rest_capture.hpp:180`](../../include/shulib/calibration/rest_capture.hpp#L180).*

<a id="drivesidevalid"></a>

## `driveSideValid`

```cpp
[[nodiscard]] inline bool driveSideValid(const DriveSide& side) noexcept
```

Check one side's shape — the constructors' shared precondition: 1..21 members, none null, sign ±1, port 1..21, scale finite and > 0.

*free function, declared at [`include/shulib/calibration/rest_capture.hpp:192`](../../include/shulib/calibration/rest_capture.hpp#L192).*

<a id="struct-restcaptureconfig"></a>

## `struct RestCaptureConfig`

```cpp
struct RestCaptureConfig
```

Every number the REST step uses. The capture's own are INVENTED for a first run (PROVISIONAL, A4: HA-170); the rest definition and the IMU wait are the drivetrain calibrator's, copied from CalibratorConfig's defaults so the two steps call the same stillness the same thing (A4: HA-136).

*struct, declared at [`include/shulib/calibration/rest_capture.hpp:209`](../../include/shulib/calibration/rest_capture.hpp#L209).*

<a id="restcaptureconfig-duration"></a>

### `RestCaptureConfig::duration`

```cpp
units::Time duration{60.0}
```

How long the capture samples. 60 s — a CHOICE: under a random-walk heading a longer capture helps only as 1/√T (HA-173). PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:212`](../../include/shulib/calibration/rest_capture.hpp#L212).*

<a id="restcaptureconfig-movebound"></a>

### `RestCaptureConfig::moveBound`

```cpp
units::Length moveBound{0.25}
```

The encoders' TRANSLATION (the centre's travel) past this refuses as Moved. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:215`](../../include/shulib/calibration/rest_capture.hpp#L215).*

<a id="restcaptureconfig-turnbound"></a>

### `RestCaptureConfig::turnBound`

```cpp
units::AngleDim turnBound{3.0 * kDegreesToRadians}
```

The encoders' ROTATION past this refuses as Turned. 3°. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:217`](../../include/shulib/calibration/rest_capture.hpp#L217).*

<a id="restcaptureconfig-settlemarginwarn"></a>

### `RestCaptureConfig::settleMarginWarn`

```cpp
double settleMarginWarn = 2.0
```

settleFloorReport() warns when a floor clears the measured noise by less than this factor. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:220`](../../include/shulib/calibration/rest_capture.hpp#L220).*

<a id="restcaptureconfig-restfloor"></a>

### `RestCaptureConfig::restFloor`

```cpp
units::AngularVelocity restFloor{CalibratorConfig{}.restFloor}
```

|velocity| under this is a still motor (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:222`](../../include/shulib/calibration/rest_capture.hpp#L222).*

<a id="restcaptureconfig-restyawratefloor"></a>

### `RestCaptureConfig::restYawRateFloor`

```cpp
units::AngularVelocity restYawRateFloor{CalibratorConfig{}.restYawRateFloor}
```

|IMU yaw rate| under this is a still robot (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:224`](../../include/shulib/calibration/rest_capture.hpp#L224).*

<a id="restcaptureconfig-restsettletime"></a>

### `RestCaptureConfig::restSettleTime`

```cpp
units::Time restSettleTime{CalibratorConfig{}.restSettleTime}
```

How long stillness must hold before the capture starts (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:226`](../../include/shulib/calibration/rest_capture.hpp#L226).*

<a id="restcaptureconfig-resttimeout"></a>

### `RestCaptureConfig::restTimeout`

```cpp
units::Time restTimeout{CalibratorConfig{}.restTimeout}
```

How long to wait for stillness before refusing (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:228`](../../include/shulib/calibration/rest_capture.hpp#L228).*

<a id="restcaptureconfig-imureadytimeout"></a>

### `RestCaptureConfig::imuReadyTimeout`

```cpp
units::Time imuReadyTimeout{CalibratorConfig{}.imuReadyTimeout}
```

How long to wait for the IMU's boot calibration (the calibrator's; A4: HA-136).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:230`](../../include/shulib/calibration/rest_capture.hpp#L230).*

<a id="restcaptureconfig-timing"></a>

### `RestCaptureConfig::timing`

```cpp
diag::TickTimingConfig timing{}
```

The loop timing's nominal period and late budget (HA-32; LoopMonitor's 15 ms).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:232`](../../include/shulib/calibration/rest_capture.hpp#L232).*

<a id="restcaptureconfig-valid"></a>

### `RestCaptureConfig::valid`

```cpp
[[nodiscard]] bool valid(char* why, std::size_t cap) const
```

True when every field is finite and positive where it must be (settleMarginWarn >= 1, restTimeout > restSettleTime), else false naming the field in `why` (may be null).

*function, declared at [`include/shulib/calibration/rest_capture.hpp:236`](../../include/shulib/calibration/rest_capture.hpp#L236).*

<a id="enum-class-restphase"></a>

## `enum class RestPhase`

```cpp
enum class RestPhase : std::uint8_t
```

Where a capture has got to. A capture ends in exactly one of Done or Aborted.

*enum class, declared at [`include/shulib/calibration/rest_capture.hpp:270`](../../include/shulib/calibration/rest_capture.hpp#L270).*

<a id="restphase-idle"></a>

### `RestPhase::Idle`

```cpp
Idle
```

constructed, start() not yet called

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:271`](../../include/shulib/calibration/rest_capture.hpp#L271).*

<a id="restphase-waitimu"></a>

### `RestPhase::WaitImu`

```cpp
WaitImu
```

waiting for the IMU's boot calibration

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:272`](../../include/shulib/calibration/rest_capture.hpp#L272).*

<a id="restphase-settle"></a>

### `RestPhase::Settle`

```cpp
Settle
```

waiting for the rest definition to hold restSettleTime

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:273`](../../include/shulib/calibration/rest_capture.hpp#L273).*

<a id="restphase-capture"></a>

### `RestPhase::Capture`

```cpp
Capture
```

sampling for `duration`

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:274`](../../include/shulib/calibration/rest_capture.hpp#L274).*

<a id="restphase-done"></a>

### `RestPhase::Done`

```cpp
Done
```

summary() is the capture's

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:275`](../../include/shulib/calibration/rest_capture.hpp#L275).*

<a id="restphase-aborted"></a>

### `RestPhase::Aborted`

```cpp
Aborted
```

refused or aborted; reason() says why; every member at 0 V + Coast

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:276`](../../include/shulib/calibration/rest_capture.hpp#L276).*

<a id="enum-class-restrefusal"></a>

## `enum class RestRefusal`

```cpp
enum class RestRefusal : std::uint8_t
```

WHY a capture ended in Aborted — a code a panel words and a test asserts. APPEND-ONLY.

*enum class, declared at [`include/shulib/calibration/rest_capture.hpp:280`](../../include/shulib/calibration/rest_capture.hpp#L280).*

<a id="restrefusal-none"></a>

### `RestRefusal::None`

```cpp
None
```

not refused

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:281`](../../include/shulib/calibration/rest_capture.hpp#L281).*

<a id="restrefusal-aborted"></a>

### `RestRefusal::Aborted`

```cpp
Aborted
```

abort() — the operator, or the program on the operator's behalf

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:282`](../../include/shulib/calibration/rest_capture.hpp#L282).*

<a id="restrefusal-imunotready"></a>

### `RestRefusal::ImuNotReady`

```cpp
ImuNotReady
```

the IMU never became ready, or stopped being ready after it was

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:283`](../../include/shulib/calibration/rest_capture.hpp#L283).*

<a id="restrefusal-resttimeout"></a>

### `RestRefusal::RestTimeout`

```cpp
RestTimeout
```

the robot never came to rest within restTimeout

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:284`](../../include/shulib/calibration/rest_capture.hpp#L284).*

<a id="restrefusal-moved"></a>

### `RestRefusal::Moved`

```cpp
Moved
```

the encoders' translation passed moveBound during the capture

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:285`](../../include/shulib/calibration/rest_capture.hpp#L285).*

<a id="restrefusal-turned"></a>

### `RestRefusal::Turned`

```cpp
Turned
```

the encoders' rotation passed turnBound during the capture

*enumerator, declared at [`include/shulib/calibration/rest_capture.hpp:286`](../../include/shulib/calibration/rest_capture.hpp#L286).*

<a id="class-restcapture"></a>

## `class RestCapture`

```cpp
class RestCapture
```

The stationary capture (header). Non-owning: the motors (through the sides' spans), the IMU and the clock must outlive it. Not copyable (it holds references). Construct a new one to capture again (the calibrator's RUN AGAIN rule: nothing carried over by construction).

*class, declared at [`include/shulib/calibration/rest_capture.hpp:292`](../../include/shulib/calibration/rest_capture.hpp#L292).*

<a id="restcapture-restcapture"></a>

### `RestCapture::RestCapture`

```cpp
RestCapture(DriveSide left, DriveSide right, units::Length trackWidth, hal::IImu& imu, hal::IClock& clock, const RestCaptureConfig& config = {})
```

Preconditions: both sides valid (driveSideValid), no port listed twice across them, the track width finite and > 0 (the description's effective width), config.valid().

*function, declared at [`include/shulib/calibration/rest_capture.hpp:296`](../../include/shulib/calibration/rest_capture.hpp#L296).*

<a id="restcapture-restcapture-2"></a>

### `RestCapture::RestCapture (overload 2)`

```cpp
RestCapture(const RestCapture&) = delete
```

Not copyable: it holds references to hardware it does not own.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:310`](../../include/shulib/calibration/rest_capture.hpp#L310).*

<a id="restcapture-operator-eq"></a>

### `RestCapture::operator=`

```cpp
RestCapture& operator=(const RestCapture&) = delete
```

Not copyable: it holds references to hardware it does not own. Not copy-assignable, for the same reason.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:312`](../../include/shulib/calibration/rest_capture.hpp#L312).*

<a id="restcapture-start"></a>

### `RestCapture::start`

```cpp
void start()
```

Begin: every member to 0 V + Coast (the robot stays pushable), then WaitImu. Precondition: the capture is Idle.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:316`](../../include/shulib/calibration/rest_capture.hpp#L316).*

<a id="restcapture-tick"></a>

### `RestCapture::tick`

```cpp
void tick()
```

One tick. Cheap and non-blocking; the caller paces it (10 ms). A no-op before start() and after the capture ends.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:324`](../../include/shulib/calibration/rest_capture.hpp#L324).*

<a id="restcapture-abort"></a>

### `RestCapture::abort`

```cpp
void abort(const char* why = "aborted by the operator")
```

Stop now: every member to 0 V + Coast, Aborted with RestRefusal::Aborted and `why` as the reason. Allowed in any phase, including Done (it then discards the summary).

*function, declared at [`include/shulib/calibration/rest_capture.hpp:339`](../../include/shulib/calibration/rest_capture.hpp#L339).*

<a id="restcapture-phase"></a>

### `RestCapture::phase`

```cpp
[[nodiscard]] RestPhase phase() const noexcept
```

Where the capture has got to.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:344`](../../include/shulib/calibration/rest_capture.hpp#L344).*

<a id="restcapture-refusal"></a>

### `RestCapture::refusal`

```cpp
[[nodiscard]] RestRefusal refusal() const noexcept
```

Which refusal ended it (None while it runs or once it is Done).

*function, declared at [`include/shulib/calibration/rest_capture.hpp:346`](../../include/shulib/calibration/rest_capture.hpp#L346).*

<a id="restcapture-reason"></a>

### `RestCapture::reason`

```cpp
[[nodiscard]] const char* reason() const noexcept
```

Why it was refused — empty while it runs or once it is Done. ASCII, no '%'.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:348`](../../include/shulib/calibration/rest_capture.hpp#L348).*

<a id="restcapture-refusalport"></a>

### `RestCapture::refusalPort`

```cpp
[[nodiscard]] int refusalPort() const noexcept
```

The port the refusal names (a motor that would not come to rest), or 0.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:350`](../../include/shulib/calibration/rest_capture.hpp#L350).*

<a id="restcapture-captureseconds"></a>

### `RestCapture::captureSeconds`

```cpp
[[nodiscard]] double captureSeconds() const noexcept
```

Seconds of capture so far (0 before Capture) — a panel's live number.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:352`](../../include/shulib/calibration/rest_capture.hpp#L352).*

<a id="restcapture-unreadsides"></a>

### `RestCapture::unreadSides`

```cpp
[[nodiscard]] std::uint32_t unreadSides() const noexcept
```

How many (tick, side) readings the capture LEFT OUT because a side had no finite reading — its speed (then left out of that side's speed σ) or its travel (then that tick judges and records no encoder motion). The header's "A side that gives no reading". 0 on a V5, where the motor adapter screens a failed read first (A4 register HA-08); a program prints it beside the summary when it is not.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:358`](../../include/shulib/calibration/rest_capture.hpp#L358).*

<a id="restcapture-summary"></a>

### `RestCapture::summary`

```cpp
[[nodiscard]] const RestSummary& summary() const
```

The capture's summary. Precondition: phase() == Done.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:360`](../../include/shulib/calibration/rest_capture.hpp#L360).*

<a id="restcapture-config"></a>

### `RestCapture::config`

```cpp
[[nodiscard]] const RestCaptureConfig& config() const noexcept
```

The config in force.

*function, declared at [`include/shulib/calibration/rest_capture.hpp:366`](../../include/shulib/calibration/rest_capture.hpp#L366).*

<a id="struct-settlefloorreport"></a>

## `struct SettleFloorReport`

```cpp
struct SettleFloorReport
```

How far each settle floor of a MotionConfig clears what a REST capture measured.

*struct, declared at [`include/shulib/calibration/rest_capture.hpp:655`](../../include/shulib/calibration/rest_capture.hpp#L655).*

<a id="settlefloorreport-headingratenoise"></a>

### `SettleFloorReport::headingRateNoise`

```cpp
double headingRateNoise = 0.0
```

The MEASURED heading-rate noise SettledUtil sees: the summary's σ_Δ/dt, rad/s.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:657`](../../include/shulib/calibration/rest_capture.hpp#L657).*

<a id="settlefloorreport-headingratefloor"></a>

### `SettleFloorReport::headingRateFloor`

```cpp
double headingRateFloor = 0.0
```

MotionConfig::headingSettle.maxErrorRate, rad/s.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:659`](../../include/shulib/calibration/rest_capture.hpp#L659).*

<a id="settlefloorreport-headingmargin"></a>

### `SettleFloorReport::headingMargin`

```cpp
double headingMargin = 0.0
```

floor / noise — how many times over the floor clears it (+infinity for zero noise).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:661`](../../include/shulib/calibration/rest_capture.hpp#L661).*

<a id="settlefloorreport-headingwarn"></a>

### `SettleFloorReport::headingWarn`

```cpp
bool headingWarn = false
```

headingMargin under the warning factor: a heading settle may flap on noise alone.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:663`](../../include/shulib/calibration/rest_capture.hpp#L663).*

<a id="settlefloorreport-speednoise"></a>

### `SettleFloorReport::speedNoise`

```cpp
double speedNoise = 0.0
```

The larger side's encoder speed σ at rest, in/s.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:665`](../../include/shulib/calibration/rest_capture.hpp#L665).*

<a id="settlefloorreport-speedfloor"></a>

### `SettleFloorReport::speedFloor`

```cpp
double speedFloor = 0.0
```

MotionConfig::brakeSettle.maxError, in/s.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:667`](../../include/shulib/calibration/rest_capture.hpp#L667).*

<a id="settlefloorreport-speedmargin"></a>

### `SettleFloorReport::speedMargin`

```cpp
double speedMargin = 0.0
```

floor / noise (+infinity for zero noise).

*field, declared at [`include/shulib/calibration/rest_capture.hpp:669`](../../include/shulib/calibration/rest_capture.hpp#L669).*

<a id="settlefloorreport-speedwarn"></a>

### `SettleFloorReport::speedWarn`

```cpp
bool speedWarn = false
```

speedMargin under the warning factor.

*field, declared at [`include/shulib/calibration/rest_capture.hpp:671`](../../include/shulib/calibration/rest_capture.hpp#L671).*

<a id="settlefloorreport"></a>

## `settleFloorReport`

```cpp
[[nodiscard]] inline SettleFloorReport settleFloorReport( const RestSummary& summary, const motion::MotionConfig& config, double marginWarn = RestCaptureConfig{}.settleMarginWarn)
```

Compare MotionConfig's settle floors with a REST capture's MEASURED noise (A4 register HA-51's settle path): the heading-rate floor against σ_Δ/dt — the per-tick step SettledUtil actually differentiates — and the brake floor against the larger side's encoder speed σ. Each margin is floor/noise; a margin under `marginWarn` (RestCaptureConfig::settleMarginWarn, 2×) warns. It NEVER uses √2·σ_line/dt, motion_config.hpp's derivation of 0.3 rad/s: that formula holds only for white heading noise, and under a random walk (HA-22's numbers) it reads ~0.24 rad/s against a true ~0.009 — a false warning against 0.30 (A4 register HA-173). Pure; changes nothing.

*free function, declared at [`include/shulib/calibration/rest_capture.hpp:681`](../../include/shulib/calibration/rest_capture.hpp#L681).*

<a id="restrefusalname"></a>

## `restRefusalName`

```cpp
[[nodiscard]] constexpr const char* restRefusalName(RestRefusal r) noexcept
```

A short, stable, ASCII name for a REST refusal — the serial record's code.

*free function, declared at [`include/shulib/calibration/rest_capture.hpp:700`](../../include/shulib/calibration/rest_capture.hpp#L700).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 104 lines, click to expand</summary>

```text

 RestCapture — shulib Calibrate's REST step: the robot sits still, unpowered, for a minute while
 the IMU's drift and noise, the encoders' noise at rest and the loop's timing are measured (chunk
 R5a Part A, 2026-09-29 — the roadmap's R4 stationary capture, moved onto the robot). Also the
 input types and the side readings the GAINS step (drive_characterizer.hpp) shares with it, and
 settleFloorReport(), which says how far each settle floor of MotionConfig clears what was measured.

 ── PROVEN ON THE HOST TEST PLANT; IT HAS NEVER RUN ON A ROBOT ──────────────────────
 Every property below is shown against the A2 host test plant with the A3 hostile IMU (white
 heading noise, a drawn per-boot rate bias) and a test-local random-walk heading. The plant CANNOT
 show the V5 IMU's noise STRUCTURE — white heading noise or integrated rate noise — nor its real
 noise, drift or continuity (A4 register HA-20/21/22/160/173). Every number in RestCaptureConfig
 is INVENTED (HA-170); the rest floors it shares with the drivetrain calibrator keep their HA-136.

 ── What it measures, and the formula behind each (rad, rad/s, in/s, s) ──────────────
 Per tick of the capture: the heading, UNWRAPPED — accumulated through Angle::errorTo, never taken
 as end minus start (drivetrain_calibrator.hpp's rule 3: a wrapped heading crossing ±π would jump
 by 2π); the per-tick heading step Δh (the same errorTo); the IMU's yaw rate; each side's median
 encoder speed and travel; the time.
   * DRIFT = the least-squares SLOPE of the unwrapped heading against time (online, Welford's
     centred sums — no Σt² − (Σt)²/n cancellation). Its standard error is reported under BOTH noise
     models, because which one holds is exactly what the host cannot show:
       - WHITE heading noise (the host's hostile IMU; HA-21): σ_b = σ_line / √(Σ(t − t̄)²), which
         for N evenly spaced ticks over T is σ_line·√12/(√N·T) — ≈ 0.002 °/min at 60 s and
         100 Hz with HA-21's 0.05°;
       - a RANDOM WALK (integrated rate noise — a gyro's heading is the integral of its rate;
         HA-22): σ_b = σ_Δ·√(1.2/(dt·T)) — the slope of a Wiener process of diffusion σ_Δ²/dt
         over T has variance 1.2·(σ_Δ²/dt)/T — ≈ 0.42 °/min at 60 s, 0.19 °/min at 300 s with
         HA-22's 0.5 °/s per 10 ms sample. That is HA-20's typical drift itself: under that
         structure one boot cannot separate its bias from its noise, and only the spread across
         boots can (A4 register HA-173).
     Neither is picked automatically: each overstates the other case badly.
   * the STRUCTURE RATIO ρ = σ_Δ/(√2·σ_line): white heading noise gives Δ = n[k] − n[k−1], so
     σ_Δ = √2·σ and ρ ≈ 1; a random walk has a small step and a large wander about any line, so
     ρ ≪ 1 (about 0.04 at HA-22's numbers over 60 s). S3's evidence for HA-173's second half.
     UNDEFINED when σ_line is 0 (no heading noise at all — the quiet host plant): reported as 0,
     which is not random-walk evidence, so RestSummary::structureRatioDefined() says which.
   * HEADING NOISE σ_line: the residual about the fitted LINE, never about the mean — a drift of
     10 °/min inflates the latter to degrees.
   * σ_Δ/dt, with σ_Δ the step's standard deviation about ITS mean and dt the mean tick: the rate
     noise SettledUtil actually differentiates (control/settled_util.hpp: |error − previous| / dt).
   * the yaw-rate channel's mean (a second drift estimate) and its σ;
   * each side's median encoder speed σ; the largest |Δh| (HA-160 evidence); the tick timing;
   * the encoders' rotation at the end and at its largest, and their largest translation (below).

 THE DRIFT IS NEVER APPLIED as a heading correction: it is a per-boot bias (HA-20), so one boot's
 number is wrong at the next boot (A4 register HA-173). Nothing here feeds a settle tolerance, the
 EKF or the heading either — recorded and reported only (R5a's decision D5).

 ── The phases ──────────────────────────────────────────────────────────────────────
   WaitImu   wait for imu.isReady(), bounded by imuReadyTimeout.
   Settle    the drivetrain calibrator's REST definition: every member under restFloor AND the
             IMU's yaw rate under restYawRateFloor, held restSettleTime, bounded by restTimeout.
   Capture   `duration` of samples. Refused if the robot MOVES or TURNS, judged from the
             ENCODERS — never from the IMU, whose apparent rotation at rest IS the drift being
             measured (a 10 °/min bias "turns" it 10° in a minute).
   Done | Aborted.
 UNPOWERED throughout: start() and every refusal leave every member at 0 V + Coast, so the robot
 stays pushable (the calibrator's rule 6), and nothing here ever commands a volt.

 ── Moved and Turned, and why they are judged on the decomposed motion ──────────────
 Turned: the encoders' rotation (dR − dL)/W past turnBound. Moved: the encoders' TRANSLATION, the
 centre's travel (dL + dR)/2, past moveBound. Both are measured from the capture's FIRST sample,
 and both are judged in either direction (|·|). R5a's design wrote "either side's travel" for
 Moved; with its own numbers (W = 12.5 in, 0.25 in, 3°) a turn in place moves each side 0.25 in
 at 2.3° of turn, so Moved would always fire first and Turned — the refusal that names a turn —
 could never name one. Split into translation and rotation, each refusal says what happened.
 THE COST OF THAT SPLIT, kept on purpose (R5a's review of 2026-10-04): the rotation a capture
 ADMITS rose — a turn in place from 2.29° (refused as Moved under the design's rule) to 3.0°, a
 pivot about one wheel from 1.15° to 2.29° (it is still refused as Moved, at 0.25 in of centre
 travel). The bound that should decide this is turnBound itself, sized on purpose — not an
 accident of the other bound's wording; its size is the team lead's (A4 register HA-170).

 ── What a rotation UNDER turnBound does to the drift — not refused, but RECORDED ────────
 A robot that really turns turns the IMU with it, and the drift's slope cannot tell that from
 drift. A heading offset ψ(t) with |ψ| <= M over [0, T] moves the least-squares slope by
   Σ(t − t̄)·ψ(t) / Σ(t − t̄)²  <=  M·Σ|t − t̄| / Σ(t − t̄)²  =  3·M·(N − 1)/(N·T)  <  3·M/T
 for N evenly spaced ticks (Σ|t − t̄| = h(N² − 1)/4 and Σ(t − t̄)² = h²N(N² − 1)/12 at spacing h,
 odd N; even N gives 3·M/((1 + 1/N)·T)) — approached by a robot turned to −M just after the
 capture starts and to +M at mid-capture. ONE step of size A at time t0 moves it by
 6·A·t0·(T − t0)/T³: 1.5·A/T at mid-capture. At the defaults (3°, 60 s) that is up to 4.5 °/min
 for one nudge and 9 °/min worst case — 4.5 to 9 times HA-20's whole 1 °/min bound, with no
 refusal (the host test plant reproduces 1.5·A/T within 0.03 %: development log, shulib-v2
 branch, 2026-10-04). So the capture RECORDS what the encoders saw — RestSummary::encoderTurnRad
 (at the end), encoderTurnMaxRad and encoderMoveMaxIn — and RestSummary::driftTurnBiasBound() is
 3·encoderTurnMaxRad/T: a drift not well clear of it says nothing. A nudge also drags the
 structure ratio toward 0 (its step inflates σ_line, not σ_Δ), so it can read as random-walk
 evidence (HA-173). Whether to shrink turnBound, or to regress the IMU heading MINUS the encoder
 rotation, is an open decision for the team lead; neither is done here.

 ── A side that gives no reading ─────────────────────────────────────────────────────
 A side's speed and travel are the median over its members (hal::medianInPlace). When EVERY
 member's reading is not finite on a tick, that median is NaN — and one NaN added to a running
 σ, or judged against a bound, would have finished a 60 s capture Done with a summary its own
 valid() refuses (found 2026-10-04, R5a's review: a whole side's velocity() NaN on ONE tick did
 exactly that). So such a reading is LEFT OUT and counted (unreadSides()): a non-finite speed is
 not added to that side's σ; a non-finite travel judges and records no Moved/Turned on that tick
 — the travel is measured from the capture's first sample, so a push made during the gap still
 shows at the next tick that reads. The characterizer's fit does the same with a non-finite sample
 (FitExclusion::NonFinite). The IMU's heading is not screened here: a non-finite heading is the
 IMU adapter's to screen (A4 register HA-08), as it is for every other heading consumer.

 Tick-driven and non-blocking (10 ms on the robot); allocation-free; PROS-free and sim-free;
 raises no faults; every runtime string is ASCII with no '%' (a panel paints them).
```

</details>
