<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/tune_program.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tune_program.hpp`

mechanism/tune_program.hpp — "shulib Tune", the program that tunes a robot's MECHANISMS on the brain (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's D7 (a) and D8 (a), with its re-check amendments #3, #7, #8, #10 and #14).

This header declares **6** types (62 members), **4** free functions, and **24** constants.

Extracted from [`include/shulib/mechanism/tune_program.hpp`](../../include/shulib/mechanism/tune_program.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kTuneCompetitionDisabled`](#ktunecompetitiondisabled) — *constant*
- [`kTuneCompetitionAutonomous`](#ktunecompetitionautonomous) — *constant*
- [`kTuneCompetitionConnected`](#ktunecompetitionconnected) — *constant*
- [`kTuneTestMostSeconds`](#ktunetestmostseconds) — *constant*
- [`kTuneTestPlanFactor`](#ktunetestplanfactor) — *constant*
- [`kTuneTestExtraSeconds`](#ktunetestextraseconds) — *constant*
- [`kTuneHoldSeconds`](#ktuneholdseconds) — *constant*
- [`kTuneHomeSeconds`](#ktunehomeseconds) — *constant*
- [`kTunePowerTestSeconds`](#ktunepowertestseconds) — *constant*
- [`kTuneSpeedTestSeconds`](#ktunespeedtestseconds) — *constant*
- [`kTuneJogMostVolts`](#ktunejogmostvolts) — *constant*
- [`kTuneJogDeadband`](#ktunejogdeadband) — *constant*
- [`kTuneMaxSamples`](#ktunemaxsamples) — *constant*
- [`kTuneTag`](#ktunetag) — *constant*
- [`kTuneTargetNameBytes`](#ktunetargetnamebytes) — *constant*
- [`struct TuneTargets`](#struct-tunetargets)
  - [`a`](#tunetargets-a)
  - [`b`](#tunetargets-b)
  - [`aName`](#tunetargets-aname)
  - [`bName`](#tunetargets-bname)
- [`tuneTargets`](#tunetargets) — *free function*
- [`tunePositionBound`](#tunepositionbound) — *free function*
- [`struct TuneSample`](#struct-tunesample)
  - [`t`](#tunesample-t)
  - [`x`](#tunesample-x)
  - [`amps`](#tunesample-amps)
- [`struct TuneMoveFacts`](#struct-tunemovefacts)
  - [`test`](#tunemovefacts-test)
  - [`kind`](#tunemovefacts-kind)
  - [`unit`](#tunemovefacts-unit)
  - [`target`](#tunemovefacts-target)
  - [`start`](#tunemovefacts-start)
  - [`within`](#tunemovefacts-within)
  - [`outcome`](#tunemovefacts-outcome)
  - [`seconds`](#tunemovefacts-seconds)
  - [`bound`](#tunemovefacts-bound)
  - [`stallAmps`](#tunemovefacts-stallamps)
  - [`capAmps`](#tunemovefacts-capamps)
  - [`stoppedBy`](#tunemovefacts-stoppedby)
- [`tuneReportFrom`](#tunereportfrom) — *free function*
- [`tuneTestLabel`](#tunetestlabel) — *free function*
- [`class TuneIo`](#class-tuneio)
  - [`~TuneIo`](#tuneio-destructor-tuneio)
  - [`TuneIo`](#tuneio-tuneio)
  - [`TuneIo (overload 2)`](#tuneio-tuneio-2)
  - [`TuneIo (overload 3)`](#tuneio-tuneio-3)
  - [`operator=`](#tuneio-operator-eq)
  - [`operator= (overload 2)`](#tuneio-operator-eq-2)
  - [`competitionStatus`](#tuneio-competitionstatus)
- [`struct TuneDevices`](#struct-tunedevices)
  - [`devices`](#tunedevices-devices)
  - [`screen`](#tunedevices-screen)
  - [`controller`](#tunedevices-controller)
  - [`io`](#tunedevices-io)
- [`kGround`](#kground) — *constant*
- [`kText`](#ktext) — *constant*
- [`kGrey`](#kgrey) — *constant*
- [`kGreyFill`](#kgreyfill) — *constant*
- [`kBlue`](#kblue) — *constant*
- [`kLightBlue`](#klightblue) — *constant*
- [`kAmber`](#kamber) — *constant*
- [`kGreen`](#kgreen) — *constant*
- [`kRed`](#kred) — *constant*
- [`class TuneProgram`](#class-tuneprogram)
  - [`TuneProgram`](#tuneprogram-tuneprogram)
  - [`TuneProgram (overload 2)`](#tuneprogram-tuneprogram-2)
  - [`operator=`](#tuneprogram-operator-eq)
  - [`TuneProgram (overload 3)`](#tuneprogram-tuneprogram-3)
  - [`operator= (overload 2)`](#tuneprogram-operator-eq-2)
  - [`~TuneProgram`](#tuneprogram-destructor-tuneprogram)
  - [`bootBanner`](#tuneprogram-bootbanner)
  - [`run`](#tuneprogram-run)
  - [`step`](#tuneprogram-step)
  - [`enter`](#tuneprogram-enter)
  - [`copy`](#tuneprogram-copy)
  - [`copyChanged`](#tuneprogram-copychanged)
  - [`uncopied`](#tuneprogram-uncopied)
  - [`page`](#tuneprogram-page)
  - [`made`](#tuneprogram-made)
  - [`shown`](#tuneprogram-shown)
  - [`selected`](#tuneprogram-selected)
  - [`drawn`](#tuneprogram-drawn)
  - [`report`](#tuneprogram-report)
  - [`moveSamples`](#tuneprogram-movesamples)
  - [`holdSamples`](#tuneprogram-holdsamples)
  - [`lastBound`](#tuneprogram-lastbound)
  - [`status`](#tuneprogram-status)
  - [`busy`](#tuneprogram-busy)
  - [`tuning`](#tuneprogram-tuning)
  - [`service`](#tuneprogram-service)
  - [`parts`](#tuneprogram-parts)
  - [`confirmed`](#tuneprogram-confirmed)
  - [`capsWrittenBack`](#tuneprogram-capswrittenback)
  - [`copiedAtExit`](#tuneprogram-copiedatexit)
  - [`resetAtExit`](#tuneprogram-resetatexit)
  - [`undoTrackedDepth`](#tuneprogram-undotrackeddepth)

<a id="ktunecompetitiondisabled"></a>

## `kTuneCompetitionDisabled`

```cpp
inline constexpr std::uint8_t kTuneCompetitionDisabled = 0x01
```

The competition status bit "disabled" (PROS's competition_get_status(); hal/pros/tune.hpp checks the three against PROS's own values at compile time). Hex: the API reference tool reads `<<` as a bracket.

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:138`](../../include/shulib/mechanism/tune_program.hpp#L138).*

<a id="ktunecompetitionautonomous"></a>

## `kTuneCompetitionAutonomous`

```cpp
inline constexpr std::uint8_t kTuneCompetitionAutonomous = 0x02
```

The competition status bit "autonomous".

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:140`](../../include/shulib/mechanism/tune_program.hpp#L140).*

<a id="ktunecompetitionconnected"></a>

## `kTuneCompetitionConnected`

```cpp
inline constexpr std::uint8_t kTuneCompetitionConnected = 0x04
```

The competition status bit "a field or switch is connected".

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:142`](../../include/shulib/mechanism/tune_program.hpp#L142).*

<a id="ktunetestmostseconds"></a>

## `kTuneTestMostSeconds`

```cpp
inline constexpr double kTuneTestMostSeconds = 10.0
```

A TEST move's time limit at most, seconds (the brief's re-check #7).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:144`](../../include/shulib/mechanism/tune_program.hpp#L144).*

<a id="ktunetestplanfactor"></a>

## `kTuneTestPlanFactor`

```cpp
inline constexpr double kTuneTestPlanFactor = 2.0
```

A TEST move's time limit: the plan's time times this (re-check #7) ...

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:146`](../../include/shulib/mechanism/tune_program.hpp#L146).*

<a id="ktunetestextraseconds"></a>

## `kTuneTestExtraSeconds`

```cpp
inline constexpr double kTuneTestExtraSeconds = 0.5
```

... plus the settle time plus this, seconds (re-check #7).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:148`](../../include/shulib/mechanism/tune_program.hpp#L148).*

<a id="ktuneholdseconds"></a>

## `kTuneHoldSeconds`

```cpp
inline constexpr double kTuneHoldSeconds = 1.0
```

How long after a position or speed move SUCCEEDS the program records its hold (re-check #3: "the HOLD current over the second after it settles"), seconds.

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:151`](../../include/shulib/mechanism/tune_program.hpp#L151).*

<a id="ktunehomeseconds"></a>

## `kTuneHomeSeconds`

```cpp
inline constexpr double kTuneHomeSeconds = 10.0
```

HOME's time limit, seconds — INVENTED (a library choice: the TEST cap reused).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:153`](../../include/shulib/mechanism/tune_program.hpp#L153).*

<a id="ktunepowertestseconds"></a>

## `kTunePowerTestSeconds`

```cpp
inline constexpr double kTunePowerTestSeconds = 0.5
```

A power mechanism's TEST run, seconds — INVENTED (a library choice).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:155`](../../include/shulib/mechanism/tune_program.hpp#L155).*

<a id="ktunespeedtestseconds"></a>

## `kTuneSpeedTestSeconds`

```cpp
inline constexpr double kTuneSpeedTestSeconds = 5.0
```

A flywheel's TEST spin-up time limit, seconds — INVENTED (a library choice; at most the TEST cap).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:157`](../../include/shulib/mechanism/tune_program.hpp#L157).*

<a id="ktunejogmostvolts"></a>

## `kTuneJogMostVolts`

```cpp
inline constexpr double kTuneJogMostVolts = 6.0
```

The stick's most volts on the shown mechanism — INVENTED (a library choice, half of full power).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:159`](../../include/shulib/mechanism/tune_program.hpp#L159).*

<a id="ktunejogdeadband"></a>

## `kTuneJogDeadband`

```cpp
inline constexpr double kTuneJogDeadband = 0.15
```

The stick's deadband, of full deflection — INVENTED (a resting thumb or drift moves nothing).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:161`](../../include/shulib/mechanism/tune_program.hpp#L161).*

<a id="ktunemaxsamples"></a>

## `kTuneMaxSamples`

```cpp
inline constexpr std::size_t kTuneMaxSamples = 1200
```

The most samples one test keeps: a 10 s move and its second at 10 ms, with room.

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:163`](../../include/shulib/mechanism/tune_program.hpp#L163).*

<a id="ktunetag"></a>

## `kTuneTag`

```cpp
inline constexpr const char* kTuneTag = "TUNEPG"
```

The log tag of the program's own lines (the COPY block's is LiveTuning's "TUNE").

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:165`](../../include/shulib/mechanism/tune_program.hpp#L165).*

<a id="ktunetargetnamebytes"></a>

## `kTuneTargetNameBytes`

```cpp
inline constexpr std::size_t kTuneTargetNameBytes = sizeof(Preset::name)
```

The longest test-target name, NUL included (a preset's name is at most 15 characters).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:170`](../../include/shulib/mechanism/tune_program.hpp#L170).*

<a id="struct-tunetargets"></a>

## `struct TuneTargets`

```cpp
struct TuneTargets
```

TEST A's and TEST B's targets for one mechanism (header): a position's or a flywheel's first and last preset, or (fewer than two) a quarter and three quarters of the range / half and nine tenths of the top speed; a power mechanism's +maxVolts and −maxVolts. A pneumatic has none (both 0).

*struct, declared at [`include/shulib/mechanism/tune_program.hpp:175`](../../include/shulib/mechanism/tune_program.hpp#L175).*

<a id="tunetargets-a"></a>

### `TuneTargets::a`

```cpp
double a = 0.0
```

TEST A's target (the mechanism's unit; volts for a power mechanism).

*field, declared at [`include/shulib/mechanism/tune_program.hpp:177`](../../include/shulib/mechanism/tune_program.hpp#L177).*

<a id="tunetargets-b"></a>

### `TuneTargets::b`

```cpp
double b = 0.0
```

TEST B's target.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:179`](../../include/shulib/mechanism/tune_program.hpp#L179).*

<a id="tunetargets-aname"></a>

### `TuneTargets::aName`

```cpp
char aName[kTuneTargetNameBytes]{}
```

TEST A's name ("floor", "1/4 range", "forward").

*field, declared at [`include/shulib/mechanism/tune_program.hpp:181`](../../include/shulib/mechanism/tune_program.hpp#L181).*

<a id="tunetargets-bname"></a>

### `TuneTargets::bName`

```cpp
char bName[kTuneTargetNameBytes]{}
```

TEST B's name.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:183`](../../include/shulib/mechanism/tune_program.hpp#L183).*

<a id="tunetargets"></a>

## `tuneTargets`

```cpp
[[nodiscard]] inline TuneTargets tuneTargets(const MechanismSpec& s) noexcept
```

The targets of `s` (TuneTargets says how).

*free function, declared at [`include/shulib/mechanism/tune_program.hpp:187`](../../include/shulib/mechanism/tune_program.hpp#L187).*

<a id="tunepositionbound"></a>

## `tunePositionBound`

```cpp
[[nodiscard]] inline double tunePositionBound(const MechanismSpec& s, double fromDeg, double target)
```

A position TEST move's TIME LIMIT (re-check #7): the plan's own time at the LIVE speed and acceleration — the trapezoid the service plans for that move, in output degrees from `fromDeg` (where it is now) to `target` clamped inside the live travel limits, by the service's one scale (the steepest units per degree, detail::unitsPerOutputDeg) — × kTuneTestPlanFactor, + the settle time + kTuneTestExtraSeconds, at most kTuneTestMostSeconds. PRECONDITION (the profile's): the live top speed and acceleration are finite and above 0 (live tuning never sets them to 0).

*free function, declared at [`include/shulib/mechanism/tune_program.hpp:241`](../../include/shulib/mechanism/tune_program.hpp#L241).*

<a id="struct-tunesample"></a>

## `struct TuneSample`

```cpp
struct TuneSample
```

One sample of a test: after a tick, the clock, where it is (its unit) or how fast (output rpm), and the most current any of its motors draws.

*struct, declared at [`include/shulib/mechanism/tune_program.hpp:257`](../../include/shulib/mechanism/tune_program.hpp#L257).*

<a id="tunesample-t"></a>

### `TuneSample::t`

```cpp
double t = 0.0
```

The clock, s.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:259`](../../include/shulib/mechanism/tune_program.hpp#L259).*

<a id="tunesample-x"></a>

### `TuneSample::x`

```cpp
double x = 0.0
```

Where it is, or how fast.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:261`](../../include/shulib/mechanism/tune_program.hpp#L261).*

<a id="tunesample-amps"></a>

### `TuneSample::amps`

```cpp
double amps = 0.0
```

The most current any of its motors draws, A.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:263`](../../include/shulib/mechanism/tune_program.hpp#L263).*

<a id="struct-tunemovefacts"></a>

## `struct TuneMoveFacts`

```cpp
struct TuneMoveFacts
```

What a test was, for tuneReportFrom().

*struct, declared at [`include/shulib/mechanism/tune_program.hpp:267`](../../include/shulib/mechanism/tune_program.hpp#L267).*

<a id="tunemovefacts-test"></a>

### `TuneMoveFacts::test`

```cpp
TuneTest test = TuneTest::None
```

Which test.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:269`](../../include/shulib/mechanism/tune_program.hpp#L269).*

<a id="tunemovefacts-kind"></a>

### `TuneMoveFacts::kind`

```cpp
Kind kind = Kind::Position
```

The mechanism's kind.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:271`](../../include/shulib/mechanism/tune_program.hpp#L271).*

<a id="tunemovefacts-unit"></a>

### `TuneMoveFacts::unit`

```cpp
const char* unit = ""
```

Its unit word.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:273`](../../include/shulib/mechanism/tune_program.hpp#L273).*

<a id="tunemovefacts-target"></a>

### `TuneMoveFacts::target`

```cpp
double target = 0.0
```

The target.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:275`](../../include/shulib/mechanism/tune_program.hpp#L275).*

<a id="tunemovefacts-start"></a>

### `TuneMoveFacts::start`

```cpp
double start = 0.0
```

Where it was (or how fast) when the test began.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:277`](../../include/shulib/mechanism/tune_program.hpp#L277).*

<a id="tunemovefacts-within"></a>

### `TuneMoveFacts::within`

```cpp
double within = 0.0
```

"Reached" (a position's within, units) or the at-speed band (a flywheel's, rpm).

*field, declared at [`include/shulib/mechanism/tune_program.hpp:279`](../../include/shulib/mechanism/tune_program.hpp#L279).*

<a id="tunemovefacts-outcome"></a>

### `TuneMoveFacts::outcome`

```cpp
manipulation::MechanismOutcome outcome = manipulation::MechanismOutcome::Running
```

The verdict.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:281`](../../include/shulib/mechanism/tune_program.hpp#L281).*

<a id="tunemovefacts-seconds"></a>

### `TuneMoveFacts::seconds`

```cpp
double seconds = 0.0
```

Seconds from the command to the verdict.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:283`](../../include/shulib/mechanism/tune_program.hpp#L283).*

<a id="tunemovefacts-bound"></a>

### `TuneMoveFacts::bound`

```cpp
double bound = 0.0
```

The time limit.

*field, declared at [`include/shulib/mechanism/tune_program.hpp:285`](../../include/shulib/mechanism/tune_program.hpp#L285).*

<a id="tunemovefacts-stallamps"></a>

### `TuneMoveFacts::stallAmps`

```cpp
double stallAmps = 0.0
```

Its stall signature's amps (0: none).

*field, declared at [`include/shulib/mechanism/tune_program.hpp:287`](../../include/shulib/mechanism/tune_program.hpp#L287).*

<a id="tunemovefacts-capamps"></a>

### `TuneMoveFacts::capAmps`

```cpp
double capAmps = 0.0
```

Its current cap (0: none).

*field, declared at [`include/shulib/mechanism/tune_program.hpp:289`](../../include/shulib/mechanism/tune_program.hpp#L289).*

<a id="tunemovefacts-stoppedby"></a>

### `TuneMoveFacts::stoppedBy`

```cpp
const char* stoppedBy = ""
```

What stopped it (CANCELLED), else "".

*field, declared at [`include/shulib/mechanism/tune_program.hpp:291`](../../include/shulib/mechanism/tune_program.hpp#L291).*

<a id="tunereportfrom"></a>

## `tuneReportFrom`

```cpp
[[nodiscard]] inline TuneReport tuneReportFrom(std::span<const TuneSample> move, std::span<const TuneSample> hold, const TuneMoveFacts& f) noexcept
```

THE REPORT (header) worked out of the samples: `move` from the first tick after the command to the tick that reached the verdict (inclusive), `hold` the second after a SUCCEEDED position or speed move. The direction of travel is the sign of (target − start) (+ when equal). Overshoot: the most any move sample is past the target in that direction, at least 0 (n/a for a power run). Settle: from the first move sample inside `within` of the target to the last move sample — SUCCEEDED only. Peak: the most `amps` over the move. Final error: the last move sample minus the target. Hold: the mean `amps` over `hold` (SUCCEEDED and at least one hold sample; else n/a). The warnings: a stated stall signature not above the hold; a stated cap not above the peak.

*free function, declared at [`include/shulib/mechanism/tune_program.hpp:302`](../../include/shulib/mechanism/tune_program.hpp#L302).*

<a id="tunetestlabel"></a>

## `tuneTestLabel`

```cpp
inline void tuneTestLabel(char* out, std::size_t cap, char which, Kind k, double target) noexcept
```

The label of a TEST button: "A 22", "B -1.5", or a power run's "A FWD" / "B REV" — at most 7 characters (a target is shown clamped to ±9999).

*free function, declared at [`include/shulib/mechanism/tune_program.hpp:351`](../../include/shulib/mechanism/tune_program.hpp#L351).*

<a id="class-tuneio"></a>

## `class TuneIo`

```cpp
class TuneIo
```

The one thing the program reads that is on no HAL seam: the competition status byte. On the brain: hal::pros::ProsTuneIo (hal/pros/tune.hpp); on the host: a test's fake.

*class, declared at [`include/shulib/mechanism/tune_program.hpp:368`](../../include/shulib/mechanism/tune_program.hpp#L368).*

<a id="tuneio-destructor-tuneio"></a>

### `TuneIo::~TuneIo`

```cpp
virtual ~TuneIo() = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:371`](../../include/shulib/mechanism/tune_program.hpp#L371).*

<a id="tuneio-tuneio"></a>

### `TuneIo::TuneIo`

```cpp
TuneIo() = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted. Default-constructible.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:373`](../../include/shulib/mechanism/tune_program.hpp#L373).*

<a id="tuneio-tuneio-2"></a>

### `TuneIo::TuneIo (overload 2)`

```cpp
TuneIo(const TuneIo&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted. Default-constructible. Copyable at the base.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:375`](../../include/shulib/mechanism/tune_program.hpp#L375).*

<a id="tuneio-tuneio-3"></a>

### `TuneIo::TuneIo (overload 3)`

```cpp
TuneIo(TuneIo&&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted. Default-constructible. Copyable at the base. Movable at the base.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:377`](../../include/shulib/mechanism/tune_program.hpp#L377).*

<a id="tuneio-operator-eq"></a>

### `TuneIo::operator=`

```cpp
TuneIo& operator=(const TuneIo&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted. Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:379`](../../include/shulib/mechanism/tune_program.hpp#L379).*

<a id="tuneio-operator-eq-2"></a>

### `TuneIo::operator= (overload 2)`

```cpp
TuneIo& operator=(TuneIo&&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted. Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base. Move-assignable at the base.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:381`](../../include/shulib/mechanism/tune_program.hpp#L381).*

<a id="tuneio-competitionstatus"></a>

### `TuneIo::competitionStatus`

```cpp
[[nodiscard]] virtual std::uint8_t competitionStatus() = 0
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted. Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base. Move-assignable at the base. The competition status byte (the kTuneCompetition* bits).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:383`](../../include/shulib/mechanism/tune_program.hpp#L383).*

<a id="struct-tunedevices"></a>

## `struct TuneDevices`

```cpp
struct TuneDevices
```

The devices the program runs on, all BORROWED: each must outlive the program.

*struct, declared at [`include/shulib/mechanism/tune_program.hpp:387`](../../include/shulib/mechanism/tune_program.hpp#L387).*

<a id="tunedevices-devices"></a>

### `TuneDevices::devices`

```cpp
hal::IDeviceProvider& devices
```

the mechanisms' devices by port; the clock; the pacer; the sink

*field, declared at [`include/shulib/mechanism/tune_program.hpp:388`](../../include/shulib/mechanism/tune_program.hpp#L388).*

<a id="tunedevices-screen"></a>

### `TuneDevices::screen`

```cpp
hal::IBrainScreen& screen
```

the brain's screen and touch

*field, declared at [`include/shulib/mechanism/tune_program.hpp:389`](../../include/shulib/mechanism/tune_program.hpp#L389).*

<a id="tunedevices-controller"></a>

### `TuneDevices::controller`

```cpp
hal::IController& controller
```

the master controller: every button a STOP; the stick

*field, declared at [`include/shulib/mechanism/tune_program.hpp:390`](../../include/shulib/mechanism/tune_program.hpp#L390).*

<a id="tunedevices-io"></a>

### `TuneDevices::io`

```cpp
TuneIo& io
```

the competition status

*field, declared at [`include/shulib/mechanism/tune_program.hpp:391`](../../include/shulib/mechanism/tune_program.hpp#L391).*

<a id="kground"></a>

## `kGround`

```cpp
inline constexpr std::uint32_t kGround = 0x000000
```

The ground.

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:397`](../../include/shulib/mechanism/tune_program.hpp#L397).*

<a id="ktext"></a>

## `kText`

```cpp
inline constexpr std::uint32_t kText = 0xFFFFFF
```

Ordinary text.

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:399`](../../include/shulib/mechanism/tune_program.hpp#L399).*

<a id="kgrey"></a>

## `kGrey`

```cpp
inline constexpr std::uint32_t kGrey = 0x808080
```

Grey (a quiet line, a greyed button's label).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:401`](../../include/shulib/mechanism/tune_program.hpp#L401).*

<a id="kgreyfill"></a>

## `kGreyFill`

```cpp
inline constexpr std::uint32_t kGreyFill = 0x303030
```

A greyed button's fill.

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:403`](../../include/shulib/mechanism/tune_program.hpp#L403).*

<a id="kblue"></a>

## `kBlue`

```cpp
inline constexpr std::uint32_t kBlue = 0x1E5AA8
```

Blue (a header, a button).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:405`](../../include/shulib/mechanism/tune_program.hpp#L405).*

<a id="klightblue"></a>

## `kLightBlue`

```cpp
inline constexpr std::uint32_t kLightBlue = 0x7FB2FF
```

Light blue (a title's text).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:407`](../../include/shulib/mechanism/tune_program.hpp#L407).*

<a id="kamber"></a>

## `kAmber`

```cpp
inline constexpr std::uint32_t kAmber = 0xF0C000
```

Amber (attention).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:409`](../../include/shulib/mechanism/tune_program.hpp#L409).*

<a id="kgreen"></a>

## `kGreen`

```cpp
inline constexpr std::uint32_t kGreen = 0x30C030
```

Green (good, GO).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:411`](../../include/shulib/mechanism/tune_program.hpp#L411).*

<a id="kred"></a>

## `kRed`

```cpp
inline constexpr std::uint32_t kRed = 0xE03030
```

Red (stopped, refused, STOP).

*constant, declared at [`include/shulib/mechanism/tune_program.hpp:413`](../../include/shulib/mechanism/tune_program.hpp#L413).*

<a id="class-tuneprogram"></a>

## `class TuneProgram`

```cpp
class TuneProgram
```

"SHULIB TUNE" (header). Construct it — in STATIC storage (PROS deletes the opcontrol task on a mode change; the service's slots point into this object) — over the robot's mechanism table and its devices; call bootBanner() from initialize() and run() from opcontrol(). Not copyable or movable.

*class, declared at [`include/shulib/mechanism/tune_program.hpp:419`](../../include/shulib/mechanism/tune_program.hpp#L419).*

<a id="tuneprogram-tuneprogram"></a>

### `TuneProgram::TuneProgram`

```cpp
TuneProgram(const MechanismTable& declared, TuneDevices d, const char* buildStamp)
```

`declared` is the robot file's table (a constant that outlives the program); `d` and `buildStamp` are borrowed and must outlive it. Constructs NOTHING on the robot (no device is asked for).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:423`](../../include/shulib/mechanism/tune_program.hpp#L423).*

<a id="tuneprogram-tuneprogram-2"></a>

### `TuneProgram::TuneProgram (overload 2)`

```cpp
TuneProgram(const TuneProgram&) = delete
```

Not copyable: it borrows the robot's devices, and the service's slots point into it.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:431`](../../include/shulib/mechanism/tune_program.hpp#L431).*

<a id="tuneprogram-operator-eq"></a>

### `TuneProgram::operator=`

```cpp
TuneProgram& operator=(const TuneProgram&) = delete
```

Not copyable: it borrows the robot's devices, and the service's slots point into it. Not copy-assignable.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:433`](../../include/shulib/mechanism/tune_program.hpp#L433).*

<a id="tuneprogram-tuneprogram-3"></a>

### `TuneProgram::TuneProgram (overload 3)`

```cpp
TuneProgram(TuneProgram&&) = delete
```

Not copyable: it borrows the robot's devices, and the service's slots point into it. Not copy-assignable. Not movable.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:435`](../../include/shulib/mechanism/tune_program.hpp#L435).*

<a id="tuneprogram-operator-eq-2"></a>

### `TuneProgram::operator= (overload 2)`

```cpp
TuneProgram& operator=(TuneProgram&&) = delete
```

Not copyable: it borrows the robot's devices, and the service's slots point into it. Not copy-assignable. Not movable. Not move-assignable.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:437`](../../include/shulib/mechanism/tune_program.hpp#L437).*

<a id="tuneprogram-destructor-tuneprogram"></a>

### `TuneProgram::~TuneProgram`

```cpp
~TuneProgram() = default
```

Not copyable: it borrows the robot's devices, and the service's slots point into it. Not copy-assignable. Not movable. Not move-assignable. The service ends any slot still holding its claim (its destructor); nothing else to release.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:439`](../../include/shulib/mechanism/tune_program.hpp#L439).*

<a id="tuneprogram-bootbanner"></a>

### `TuneProgram::bootBanner`

```cpp
void bootBanner(const char* label, const char* statusWords)
```

initialize(): the boot lines (constructs nothing, powers nothing). `label` is the build's beacon (or null), `statusWords` the competition status at boot in words (or null).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:443`](../../include/shulib/mechanism/tune_program.hpp#L443).*

<a id="tuneprogram-run"></a>

### `TuneProgram::run`

```cpp
void run()
```

opcontrol(): the loop, one pass every control tick, until VEXos reports the robot DISABLED or autonomous starting — every mechanism released first — or forever. Re-entered by PROS after a disable: the object carries on (the made mechanisms, the live numbers, the page).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:473`](../../include/shulib/mechanism/tune_program.hpp#L473).*

<a id="tuneprogram-step"></a>

### `TuneProgram::step`

```cpp
bool step()
```

ONE pass of the loop (header): read the status, the controller and the screen; stop on any abort cause; act on a touch or the stick; paint; tick the service once (it then advances the world); record a test's sample. False when driver control ended (everything released): the caller returns. Exposed for the host tests, which drive the loop pass by pass.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:483`](../../include/shulib/mechanism/tune_program.hpp#L483).*

<a id="tuneprogram-enter"></a>

### `TuneProgram::enter`

```cpp
void enter()
```

What run()/step() do when (re-)entered: the service's driver-control entry (a step left running by a deleted task CANCELLED, every motor mechanism made safe), a full repaint. Called by run().

*function, declared at [`include/shulib/mechanism/tune_program.hpp:496`](../../include/shulib/mechanism/tune_program.hpp#L496).*

<a id="tuneprogram-copy"></a>

### `TuneProgram::copy`

```cpp
int copy(std::size_t i)
```

COPY mechanism `i`'s block (LiveTuning::copyBlock) into the provider's sink — on the brain the terminal and the SD copy — and say how many lines on the page. Returns the count. PRECONDITION: `i` below the table's count.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:508`](../../include/shulib/mechanism/tune_program.hpp#L508).*

<a id="tuneprogram-copychanged"></a>

### `TuneProgram::copyChanged`

```cpp
int copyChanged()
```

The COPY button (D9 (a): "for each mechanism changed, ONE block"): a block for EVERY mechanism whose numbers differ from the robot file's — or, when none does, the shown one's (0 lines, which says so). Returns the lines copied in all.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:528`](../../include/shulib/mechanism/tune_program.hpp#L528).*

<a id="tuneprogram-uncopied"></a>

### `TuneProgram::uncopied`

```cpp
[[nodiscard]] bool uncopied(std::size_t i) const
```

True when mechanism `i`'s numbers differ from the robot file's AND from what its last COPY wrote (a change no COPY carries yet: what EXIT, or a cap going back, COPIES first).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:554`](../../include/shulib/mechanism/tune_program.hpp#L554).*

<a id="tuneprogram-page"></a>

### `TuneProgram::page`

```cpp
[[nodiscard]] TunePage page() const noexcept
```

The page on the screen.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:560`](../../include/shulib/mechanism/tune_program.hpp#L560).*

<a id="tuneprogram-made"></a>

### `TuneProgram::made`

```cpp
[[nodiscard]] bool made() const noexcept
```

True once READY made the mechanisms.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:562`](../../include/shulib/mechanism/tune_program.hpp#L562).*

<a id="tuneprogram-shown"></a>

### `TuneProgram::shown`

```cpp
[[nodiscard]] std::size_t shown() const noexcept
```

The shown mechanism's index.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:564`](../../include/shulib/mechanism/tune_program.hpp#L564).*

<a id="tuneprogram-selected"></a>

### `TuneProgram::selected`

```cpp
[[nodiscard]] std::size_t selected() const noexcept
```

The selected number's position in the shown mechanism's list.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:566`](../../include/shulib/mechanism/tune_program.hpp#L566).*

<a id="tuneprogram-drawn"></a>

### `TuneProgram::drawn`

```cpp
[[nodiscard]] const TuneView& drawn() const noexcept
```

The view last painted (what a touch is tested against).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:568`](../../include/shulib/mechanism/tune_program.hpp#L568).*

<a id="tuneprogram-report"></a>

### `TuneProgram::report`

```cpp
[[nodiscard]] const TuneReport& report() const noexcept
```

The last TEST's report.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:570`](../../include/shulib/mechanism/tune_program.hpp#L570).*

<a id="tuneprogram-movesamples"></a>

### `TuneProgram::moveSamples`

```cpp
[[nodiscard]] std::span<const TuneSample> moveSamples() const noexcept
```

The last TEST's samples: the move's.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:572`](../../include/shulib/mechanism/tune_program.hpp#L572).*

<a id="tuneprogram-holdsamples"></a>

### `TuneProgram::holdSamples`

```cpp
[[nodiscard]] std::span<const TuneSample> holdSamples() const noexcept
```

... and its hold second's.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:576`](../../include/shulib/mechanism/tune_program.hpp#L576).*

<a id="tuneprogram-lastbound"></a>

### `TuneProgram::lastBound`

```cpp
[[nodiscard]] double lastBound() const noexcept
```

The last TEST's time limit, s.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:580`](../../include/shulib/mechanism/tune_program.hpp#L580).*

<a id="tuneprogram-status"></a>

### `TuneProgram::status`

```cpp
[[nodiscard]] const char* status() const noexcept
```

The status sentence on the page.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:582`](../../include/shulib/mechanism/tune_program.hpp#L582).*

<a id="tuneprogram-busy"></a>

### `TuneProgram::busy`

```cpp
[[nodiscard]] bool busy() const noexcept
```

A test, a homing or a test's hold second is running.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:584`](../../include/shulib/mechanism/tune_program.hpp#L584).*

<a id="tuneprogram-tuning"></a>

### `TuneProgram::tuning`

```cpp
[[nodiscard]] LiveTuning& tuning() noexcept
```

Live tuning (the table the service runs).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:586`](../../include/shulib/mechanism/tune_program.hpp#L586).*

<a id="tuneprogram-service"></a>

### `TuneProgram::service`

```cpp
[[nodiscard]] MechanismService& service() noexcept
```

The service (bound once made).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:588`](../../include/shulib/mechanism/tune_program.hpp#L588).*

<a id="tuneprogram-parts"></a>

### `TuneProgram::parts`

```cpp
[[nodiscard]] const robot::MechanismParts& parts() const noexcept
```

The built parts.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:590`](../../include/shulib/mechanism/tune_program.hpp#L590).*

<a id="tuneprogram-confirmed"></a>

### `TuneProgram::confirmed`

```cpp
[[nodiscard]] bool confirmed(std::size_t i) const noexcept
```

Whether mechanism `i` passed stage 2 of the clear confirm this session.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:592`](../../include/shulib/mechanism/tune_program.hpp#L592).*

<a id="tuneprogram-capswrittenback"></a>

### `TuneProgram::capsWrittenBack`

```cpp
[[nodiscard]] int capsWrittenBack() const noexcept
```

How many current caps EXIT (or leaving a page) wrote back, in all.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:594`](../../include/shulib/mechanism/tune_program.hpp#L594).*

<a id="tuneprogram-copiedatexit"></a>

### `TuneProgram::copiedAtExit`

```cpp
[[nodiscard]] int copiedAtExit() const noexcept
```

How many mechanisms the last EXIT COPIED (changed, never copied) and RESET (a cap that could not go back alone).

*function, declared at [`include/shulib/mechanism/tune_program.hpp:597`](../../include/shulib/mechanism/tune_program.hpp#L597).*

<a id="tuneprogram-resetatexit"></a>

### `TuneProgram::resetAtExit`

```cpp
[[nodiscard]] int resetAtExit() const noexcept
```

... and RESET.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:599`](../../include/shulib/mechanism/tune_program.hpp#L599).*

<a id="tuneprogram-undotrackeddepth"></a>

### `TuneProgram::undoTrackedDepth`

```cpp
[[nodiscard]] std::size_t undoTrackedDepth() const noexcept
```

How many undo records the program tracks (which mechanism each is): always LiveTuning's undoDepth() — every change goes through the tracking.

*function, declared at [`include/shulib/mechanism/tune_program.hpp:602`](../../include/shulib/mechanism/tune_program.hpp#L602).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 95 lines, click to expand</summary>

```text

 mechanism/tune_program.hpp — "shulib Tune", the program that tunes a robot's MECHANISMS on the brain
 (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's D7 (a) and D8 (a), with its re-check amendments
 #3, #7, #8, #10 and #14). PROS-free: hal/pros/tune.hpp is its thin shell on a V5.

 ── Why its own program, and why in the library ─────────────────────────────────────
 A mechanism's numbers are found ON THE ROBOT (the team lead, 2026-10-29: "things need to be able to
 be tuned"): a string cascade's gains, top speed, homing push and current cap. The match programs run
 EXACTLY the robot file's numbers, so tuning lives in a program of its own, uploaded beside them
 (`make ROBOT=<any robot with mechanisms> PROGRAM=tune`, slot 7) — never a page of shulib Drive, where
 one stray tap in practice would leave the robot driving on numbers nobody wrote down (D7 (b)). It is
 LIBRARY code over the HAL's seams — the standard Calibrate's pattern (calibration/calibrate_program.hpp)
 — so every robot, robot one's cascade and a robot made in the panel later, runs the same program:
 nothing in THIS file names a robot. (The BUILD names each robot once, as it does for every program:
 a robot that registers a mechanism table adds its ROBOT value to the Makefile's MECHANISM_ROBOTS, a
 `<robot> && SHULIB_PROGRAM_TUNE` branch to src/build_selection.hpp's one chain and its define to that
 file's Tune guard, and a `<robot>-tune` build to tools/src_build_gate.py's tables — the src build gate
 fails a robot with mechanisms that is missing from any of them: its step 0, or the guard's #error.)

 ── What it builds, and what it never builds ────────────────────────────────────────
 ONLY the robot's mechanisms: each one's config projected from the robot file
 (mechanism::toMechanismConfig, kept in this object beside the parts it builds), robot::MechanismParts
 over the provider, ONE MechanismService bound over a LiveTuning's COPY of the table (live_tuning.hpp).
 It NEVER constructs a chassis, a drive motor or a Robot (landmine 17: a robot on blocks is not
 assumed), and NOTHING is made before stage 1 of the clear confirm (READY): making a 3-wire line
 drives it to its start level (HA-119 — robot two's claw moves at boot).

 ── Its safety rules: the Auton Tester's (re-check #7) ──────────────────────────────
   * EVERY mechanism is released to its declared safe state (MechanismService::leaveDriverControl():
     a running step CANCELLED, every motor mechanism inert in its safe brake mode, a pneumatic keeping
     its level — HA-157) on ANY of the controller's twelve buttons, on the controller DISCONNECTING, on
     a field or switch being plugged in, on the page's STOP, and when VEXos reports the robot DISABLED
     or autonomous starting (then run() returns: PROS calls opcontrol() again later, and this object,
     in the shell's static storage, carries on). A stop is applied BEFORE the tick that follows the
     read that saw it — never a tick later.
   * READY (which makes the mechanisms — a pneumatic moves), TEST, HOME, STRING CHECK and the stick are
     REFUSED, each with one sentence, unless a controller is connected (it carries the stop), no
     button is held, VEXos reports no field or switch, and (but READY) the mechanism is built (and,
     for a position TEST, homed).
   * The stick starts a jog only on a FRESH push (centred, then pushed): one held over a change of
     mechanism, a stop, GO or another page moves nothing until it is let go and pushed again.
   * STAGE 2 of the clear confirm: a mechanism's FIRST motion in the session waits for GO.
   * A TEST move's time limit is the plan's own time at the live speed and acceleration, × 2, plus the
     settle time and 0.5 s, at most 10 s (tunePositionBound()); the service reports TIMED_OUT there.
   * A change is applied only BETWEEN steps: a running test greys every button but STOP, and live
     tuning itself refuses a change while its mechanism's step runs (LiveTuning, rule 3).

 ── The current cap stays in the motor (re-check #8, D12 (b); A4 register HA-220) ─────
 A cap changed live is WRITTEN into each motor and a motor keeps a limit after the program ends, and
 the next program, stating no cap, writes nothing. So this program puts back EVERY limit it changed —
 the robot file's cap, or the motor's 2.5 A default when the file states none — on RESET (LiveTuning
 re-derives it), on LEAVING that mechanism's page (‹ ›) and on EXIT (restoreCap(): LiveTuning::apply of
 the file's cap; on EXIT, a RESET when that is refused). The tuned cap then leaves the live numbers, so
 restoreCap() first COPIES the mechanism's numbers to the log when no COPY carries them yet; a page
 whose cap cannot go back (a live signature at or above the file's cap) is not left — the page says
 "lower it, or RESET". ONLY the shown mechanism carries a live cap: LiveTuning's UNDO is global, so an
 UNDO whose next undo would change ANOTHER mechanism's cap (the cap put back when its page was left, or
 a RESET of a raised cap) is checked BEFORE it is made and waits — nothing changes, the page says "UNDO
 it on its page" (undoWaitsOn(); every change is tracked to know whose each undo is). Stopping the program from the brain runs no
 code: the CLEAR page and the TUNE page's safety line (any mechanism's cap changed) say "EXIT before you
 stop".

 ── A TEST move and its report (re-check #3, #7) ────────────────────────────────────
 TEST A / TEST B: a position mechanism moves to its first and last preset (with fewer than two, a
 quarter and three quarters of its live range); a flywheel spins up to its first and last preset (with
 fewer, half and nine tenths of its top speed — INVENTED); a power mechanism runs at +maxVolts / −maxVolts
 for kTunePowerTestSeconds (INVENTED). Each is an ordinary service step. After every tick the program
 records one sample (the clock, where it is or how fast, the most current any of its motors draws);
 when a position or speed move SUCCEEDS it records ONE MORE SECOND (its hold, or its spin at speed)
 with every button but STOP greyed. Then tuneReportFrom() works the report out of the samples — the
 time to the verdict, the farthest past the target, the settle (the first sample inside "reached" to
 the verdict), the most current, the final error, the HOLD current (the mean over that second) — and
 a WARNING when the stall signature is not above that hold (an ordinary hold could trip it: V1) or the
 cap not above the peak (the cap limited the move: landmine 18). A flywheel is stopped after its
 second; a position mechanism goes on holding. The report is on the REPORT page and in the log (tag
 TUNEPG, the terminal and the SD copy).

 ── COPY ────────────────────────────────────────────────────────────────────────────
 LiveTuning::copyBlock() into the provider's sink: on the brain the terminal AND /usd/tune_log.txt,
 APPENDED across boots (re-check #14 — hal/pros/tune.hpp). The COPY button writes ONE block for EVERY
 mechanism changed (D9 (a)); EXIT writes one for every mechanism changed since its last COPY, so the
 live numbers — which are not kept — are in the log before the program can be stopped. Until the
 panel's import exists (TUNE1-B2, after the panel), a student copies the block's lines into the robot
 file BY HAND.

 ── The stick (a delta from the brief, logged: D8 (a) says "a student jogs it to the right height and
 reads the number off", and nothing else on the page can) ───────────────────────────
 The LEFT stick's up/down moves the shown POSITION or POWER mechanism by hold-to-run (the service's
 jog() / drive()) at up to kTuneJogMostVolts (INVENTED) beyond kTuneJogDeadband, and holds where it is
 (or stops) when let go. Never a button: every button is a STOP. A position mechanism before homing
 moves only by its declaration's `.unhomedVolts` (the service's rule) — 0 means not at all.

 The live numbers never persist: leaving the program, or a reboot, runs the robot file's numbers again
 (every program but this one always does). NOTHING HERE HAS RUN ON A ROBOT: its tests drive it over
 FakeDeviceProvider and the host test plant (its constants invented — HA-183).
```

</details>
