<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/live_tuning.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `live_tuning.hpp`

mechanism/live_tuning.hpp — LIVE TUNING's library half: a mechanism's numbers changed WHILE THE ROBOT RUNS, inside the same rules as the build, undone, and copied back out as the lines its robot file needs (chunk TUNE1-A, 2026-11-10; the MECH-S/TUNE1 brief'…

This header declares **7** types (81 members), **10** free functions, and **5** constants.

Extracted from [`include/shulib/mechanism/live_tuning.hpp`](../../include/shulib/mechanism/live_tuning.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kLiveUndoDepth`](#kliveundodepth) — *constant*
- [`kLiveKvMostTimesStarting`](#klivekvmosttimesstarting) — *constant*
- [`kLiveFirstKaSeconds`](#klivefirstkaseconds) — *constant*
- [`kCopyNumberChars`](#kcopynumberchars) — *constant*
- [`enum class Tunable`](#enum-class-tunable)
  - [`KP`](#tunable-kp)
  - [`KI`](#tunable-ki)
  - [`KD`](#tunable-kd)
  - [`KS`](#tunable-ks)
  - [`KV`](#tunable-kv)
  - [`KA`](#tunable-ka)
  - [`MaxSpeed`](#tunable-maxspeed)
  - [`MaxAcceleration`](#tunable-maxacceleration)
  - [`Within`](#tunable-within)
  - [`Settle`](#tunable-settle)
  - [`GravityHold`](#tunable-gravityhold)
  - [`StallAmps`](#tunable-stallamps)
  - [`StallRpm`](#tunable-stallrpm)
  - [`StallLasting`](#tunable-stalllasting)
  - [`HomeVolts`](#tunable-homevolts)
  - [`HomeStallAmps`](#tunable-homestallamps)
  - [`HomeStallRpm`](#tunable-homestallrpm)
  - [`HomeStallLasting`](#tunable-homestalllasting)
  - [`CurrentCap`](#tunable-currentcap)
  - [`RangeMin`](#tunable-rangemin)
  - [`RangeMax`](#tunable-rangemax)
  - [`AtSpeedBand`](#tunable-atspeedband)
  - [`AtSpeedTime`](#tunable-atspeedtime)
  - [`MaxVolts`](#tunable-maxvolts)
- [`kTunableCount`](#ktunablecount) — *constant*
- [`enum class StepRule`](#enum-class-steprule)
  - [`Gain`](#steprule-gain)
  - [`ZeroGain`](#steprule-zerogain)
  - [`Volts`](#steprule-volts)
  - [`Amps`](#steprule-amps)
  - [`Percent`](#steprule-percent)
  - [`Reached`](#steprule-reached)
  - [`Seconds`](#steprule-seconds)
  - [`Limit`](#steprule-limit)
- [`struct Parameter`](#struct-parameter)
  - [`id`](#parameter-id)
  - [`name`](#parameter-name)
  - [`rule`](#parameter-rule)
  - [`member`](#parameter-member)
- [`struct ParameterList`](#struct-parameterlist)
  - [`items`](#parameterlist-items)
  - [`count`](#parameterlist-count)
- [`enum class LiveRefusal`](#enum-class-liverefusal)
  - [`None`](#liverefusal-none)
  - [`NotLive`](#liverefusal-notlive)
  - [`BuildRefuses`](#liverefusal-buildrefuses)
  - [`NotANumber`](#liverefusal-notanumber)
  - [`GainNotAbove0`](#liverefusal-gainnotabove0)
  - [`FeedforwardNegative`](#liverefusal-feedforwardnegative)
  - [`SettleIs0`](#liverefusal-settleis0)
  - [`KvPastTwiceStarting`](#liverefusal-kvpasttwicestarting)
  - [`TopSpeedPastFree`](#liverefusal-topspeedpastfree)
  - [`RangeWidened`](#liverefusal-rangewidened)
  - [`HomingPushTurned`](#liverefusal-homingpushturned)
  - [`CapRemoved`](#liverefusal-capremoved)
  - [`StepRunning`](#liverefusal-steprunning)
  - [`InsideATick`](#liverefusal-insideatick)
  - [`LeavesOutHeldTarget`](#liverefusal-leavesoutheldtarget)
  - [`LeavesOutWhereItIs`](#liverefusal-leavesoutwhereitis)
  - [`HeldButtonMoving`](#liverefusal-heldbuttonmoving)
  - [`NothingToUndo`](#liverefusal-nothingtoundo)
- [`liveRefusalText`](#liverefusaltext) — *free function*
- [`offered`](#offered) — *free function*
- [`tunableValue`](#tunablevalue) — *free function*
- [`setTunable`](#settunable) — *free function*
- [`tunableUnit`](#tunableunit) — *free function*
- [`parameterOf`](#parameterof) — *free function*
- [`parametersOf`](#parametersof) — *free function*
- [`tuningRefusal`](#tuningrefusal) — *free function*
- [`tuningProblem`](#tuningproblem) — *free function*
- [`steppedValue`](#steppedvalue) — *free function*
- [`struct Applied`](#struct-applied)
  - [`ok`](#applied-ok)
  - [`why`](#applied-why)
  - [`build`](#applied-build)
  - [`sentence`](#applied-sentence)
- [`class LiveTuning`](#class-livetuning)
  - [`LiveTuning`](#livetuning-livetuning)
  - [`LiveTuning (overload 2)`](#livetuning-livetuning-2)
  - [`operator=`](#livetuning-operator-eq)
  - [`LiveTuning (overload 3)`](#livetuning-livetuning-3)
  - [`operator= (overload 2)`](#livetuning-operator-eq-2)
  - [`~LiveTuning`](#livetuning-destructor-livetuning)
  - [`table`](#livetuning-table)
  - [`declared`](#livetuning-declared)
  - [`count`](#livetuning-count)
  - [`spec`](#livetuning-spec)
  - [`attach`](#livetuning-attach)
  - [`attached`](#livetuning-attached)
  - [`changed`](#livetuning-changed)
  - [`value`](#livetuning-value)
  - [`stepped`](#livetuning-stepped)
  - [`undoDepth`](#livetuning-undodepth)
  - [`apply`](#livetuning-apply)
  - [`step`](#livetuning-step)
  - [`undo`](#livetuning-undo)
  - [`reset`](#livetuning-reset)
  - [`copyBlock`](#livetuning-copyblock)

<a id="kliveundodepth"></a>

## `kLiveUndoDepth`

```cpp
inline constexpr std::size_t kLiveUndoDepth = 16
```

How many changes UNDO reaches back (D8 (a)): the 17th undo in a row is refused.

*constant, declared at [`include/shulib/mechanism/live_tuning.hpp:142`](../../include/shulib/mechanism/live_tuning.hpp#L142).*

<a id="klivekvmosttimesstarting"></a>

## `kLiveKvMostTimesStarting`

```cpp
inline constexpr double kLiveKvMostTimesStarting = 2.0
```

The most a LIVE kV may be, as a multiple of its starting value (12 V over the output's free speed) — an INVENTED library bound (TUNE1-A): it refuses P9's kV = 1e9 and leaves a real motor's slower-than- nominal kV room.

*constant, declared at [`include/shulib/mechanism/live_tuning.hpp:146`](../../include/shulib/mechanism/live_tuning.hpp#L146).*

<a id="klivefirstkaseconds"></a>

## `kLiveFirstKaSeconds`

```cpp
inline constexpr double kLiveFirstKaSeconds = 0.01
```

kA's first [+] from 0, as a lag on kV: kA = kV × this, seconds (re-check #14 — a 10 ms lag).

*constant, declared at [`include/shulib/mechanism/live_tuning.hpp:148`](../../include/shulib/mechanism/live_tuning.hpp#L148).*

<a id="kcopynumberchars"></a>

## `kCopyNumberChars`

```cpp
inline constexpr std::size_t kCopyNumberChars = 24
```

The longest number the copy block and a sentence print: std::to_chars' shortest round-trip form of any double is at most 24 characters ("-2.2250738585072014e-308").

*constant, declared at [`include/shulib/mechanism/live_tuning.hpp:151`](../../include/shulib/mechanism/live_tuning.hpp#L151).*

<a id="enum-class-tunable"></a>

## `enum class Tunable`

```cpp
enum class Tunable : std::uint8_t
```

ONE LIVE NUMBER — every number some kind lets live tuning change (parametersOf() says which a given mechanism offers). The value is in the DECLARATION's unit: a stall's rpm at the motor (the spec keeps rad/s), a travel limit in degrees or inches.

*enum class, declared at [`include/shulib/mechanism/live_tuning.hpp:156`](../../include/shulib/mechanism/live_tuning.hpp#L156).*

<a id="tunable-kp"></a>

### `Tunable::KP`

```cpp
KP
```

kP — a position's or a flywheel's

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:157`](../../include/shulib/mechanism/live_tuning.hpp#L157).*

<a id="tunable-ki"></a>

### `Tunable::KI`

```cpp
KI
```

kI — the hold loop's (position) or the speed loop's (flywheel)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:158`](../../include/shulib/mechanism/live_tuning.hpp#L158).*

<a id="tunable-kd"></a>

### `Tunable::KD`

```cpp
KD
```

kD (position)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:159`](../../include/shulib/mechanism/live_tuning.hpp#L159).*

<a id="tunable-ks"></a>

### `Tunable::KS`

```cpp
KS
```

kS, V (position, flywheel)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:160`](../../include/shulib/mechanism/live_tuning.hpp#L160).*

<a id="tunable-kv"></a>

### `Tunable::KV`

```cpp
KV
```

kV (position, flywheel)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:161`](../../include/shulib/mechanism/live_tuning.hpp#L161).*

<a id="tunable-ka"></a>

### `Tunable::KA`

```cpp
KA
```

kA (position ONLY: a flywheel's cannot be stated)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:162`](../../include/shulib/mechanism/live_tuning.hpp#L162).*

<a id="tunable-maxspeed"></a>

### `Tunable::MaxSpeed`

```cpp
MaxSpeed
```

the smooth move's top speed, units/s (position)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:163`](../../include/shulib/mechanism/live_tuning.hpp#L163).*

<a id="tunable-maxacceleration"></a>

### `Tunable::MaxAcceleration`

```cpp
MaxAcceleration
```

its acceleration, units/s² (position)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:164`](../../include/shulib/mechanism/live_tuning.hpp#L164).*

<a id="tunable-within"></a>

### `Tunable::Within`

```cpp
Within
```

"reached" within this, units (position)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:165`](../../include/shulib/mechanism/live_tuning.hpp#L165).*

<a id="tunable-settle"></a>

### `Tunable::Settle`

```cpp
Settle
```

... for this long, s (position)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:166`](../../include/shulib/mechanism/live_tuning.hpp#L166).*

<a id="tunable-gravityhold"></a>

### `Tunable::GravityHold`

```cpp
GravityHold
```

kG, V (a position mechanism with gravity)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:167`](../../include/shulib/mechanism/live_tuning.hpp#L167).*

<a id="tunable-stallamps"></a>

### `Tunable::StallAmps`

```cpp
StallAmps
```

the stall signature's amps

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:168`](../../include/shulib/mechanism/live_tuning.hpp#L168).*

<a id="tunable-stallrpm"></a>

### `Tunable::StallRpm`

```cpp
StallRpm
```

its "below this motor rpm"

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:169`](../../include/shulib/mechanism/live_tuning.hpp#L169).*

<a id="tunable-stalllasting"></a>

### `Tunable::StallLasting`

```cpp
StallLasting
```

its lasting, s

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:170`](../../include/shulib/mechanism/live_tuning.hpp#L170).*

<a id="tunable-homevolts"></a>

### `Tunable::HomeVolts`

```cpp
HomeVolts
```

the homing push, V (homing by switch or by stall)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:171`](../../include/shulib/mechanism/live_tuning.hpp#L171).*

<a id="tunable-homestallamps"></a>

### `Tunable::HomeStallAmps`

```cpp
HomeStallAmps
```

the homing signature's amps (homing by stall)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:172`](../../include/shulib/mechanism/live_tuning.hpp#L172).*

<a id="tunable-homestallrpm"></a>

### `Tunable::HomeStallRpm`

```cpp
HomeStallRpm
```

its "below this motor rpm"

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:173`](../../include/shulib/mechanism/live_tuning.hpp#L173).*

<a id="tunable-homestalllasting"></a>

### `Tunable::HomeStallLasting`

```cpp
HomeStallLasting
```

its lasting, s

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:174`](../../include/shulib/mechanism/live_tuning.hpp#L174).*

<a id="tunable-currentcap"></a>

### `Tunable::CurrentCap`

```cpp
CurrentCap
```

the current cap, A per motor (0: none)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:175`](../../include/shulib/mechanism/live_tuning.hpp#L175).*

<a id="tunable-rangemin"></a>

### `Tunable::RangeMin`

```cpp
RangeMin
```

the lower travel limit (narrowed only)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:176`](../../include/shulib/mechanism/live_tuning.hpp#L176).*

<a id="tunable-rangemax"></a>

### `Tunable::RangeMax`

```cpp
RangeMax
```

the upper travel limit (narrowed only)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:177`](../../include/shulib/mechanism/live_tuning.hpp#L177).*

<a id="tunable-atspeedband"></a>

### `Tunable::AtSpeedBand`

```cpp
AtSpeedBand
```

a flywheel's "at speed" band, rpm

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:178`](../../include/shulib/mechanism/live_tuning.hpp#L178).*

<a id="tunable-atspeedtime"></a>

### `Tunable::AtSpeedTime`

```cpp
AtSpeedTime
```

... and how long, s

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:179`](../../include/shulib/mechanism/live_tuning.hpp#L179).*

<a id="tunable-maxvolts"></a>

### `Tunable::MaxVolts`

```cpp
MaxVolts
```

a power mechanism's most volts

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:180`](../../include/shulib/mechanism/live_tuning.hpp#L180).*

<a id="ktunablecount"></a>

## `kTunableCount`

```cpp
inline constexpr std::size_t kTunableCount = 24
```

How many Tunable values there are.

*constant, declared at [`include/shulib/mechanism/live_tuning.hpp:183`](../../include/shulib/mechanism/live_tuning.hpp#L183).*

<a id="enum-class-steprule"></a>

## `enum class StepRule`

```cpp
enum class StepRule : std::uint8_t
```

How one [−] / [+] press moves a number (D8 (a); header).

*enum class, declared at [`include/shulib/mechanism/live_tuning.hpp:186`](../../include/shulib/mechanism/live_tuning.hpp#L186).*

<a id="steprule-gain"></a>

### `StepRule::Gain`

```cpp
Gain
```

×/÷ 1.25, FINE ×/÷ 1.05

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:187`](../../include/shulib/mechanism/live_tuning.hpp#L187).*

<a id="steprule-zerogain"></a>

### `StepRule::ZeroGain`

```cpp
ZeroGain
```

kA: from 0 to kV × 0.01 s, then ×/÷ 1.25; below that first step, 0

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:188`](../../include/shulib/mechanism/live_tuning.hpp#L188).*

<a id="steprule-volts"></a>

### `StepRule::Volts`

```cpp
Volts
```

±0.05 V

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:189`](../../include/shulib/mechanism/live_tuning.hpp#L189).*

<a id="steprule-amps"></a>

### `StepRule::Amps`

```cpp
Amps
```

±0.05 A

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:190`](../../include/shulib/mechanism/live_tuning.hpp#L190).*

<a id="steprule-percent"></a>

### `StepRule::Percent`

```cpp
Percent
```

×/÷ 1.05 (about 5 %): a speed, an acceleration, a stall's rpm, the at-speed band

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:191`](../../include/shulib/mechanism/live_tuning.hpp#L191).*

<a id="steprule-reached"></a>

### `StepRule::Reached`

```cpp
Reached
```

±0.05 in, or ±0.5 deg

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:192`](../../include/shulib/mechanism/live_tuning.hpp#L192).*

<a id="steprule-seconds"></a>

### `StepRule::Seconds`

```cpp
Seconds
```

±0.05 s

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:193`](../../include/shulib/mechanism/live_tuning.hpp#L193).*

<a id="steprule-limit"></a>

### `StepRule::Limit`

```cpp
Limit
```

±0.25 in, or ±1 deg

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:194`](../../include/shulib/mechanism/live_tuning.hpp#L194).*

<a id="struct-parameter"></a>

## `struct Parameter`

```cpp
struct Parameter
```

ONE live number as the page shows it and the robot file says it.

*struct, declared at [`include/shulib/mechanism/live_tuning.hpp:198`](../../include/shulib/mechanism/live_tuning.hpp#L198).*

<a id="parameter-id"></a>

### `Parameter::id`

```cpp
Tunable id = Tunable::KP
```

Which number.

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:200`](../../include/shulib/mechanism/live_tuning.hpp#L200).*

<a id="parameter-name"></a>

### `Parameter::name`

```cpp
const char* name = ""
```

Its name on the page ("kP", "top speed", "stall amps").

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:202`](../../include/shulib/mechanism/live_tuning.hpp#L202).*

<a id="parameter-rule"></a>

### `Parameter::rule`

```cpp
StepRule rule = StepRule::Gain
```

How a press moves it.

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:204`](../../include/shulib/mechanism/live_tuning.hpp#L204).*

<a id="parameter-member"></a>

### `Parameter::member`

```cpp
const char* member = ""
```

How the robot file names it (".tuning.kP", ".stall", ".home").

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:206`](../../include/shulib/mechanism/live_tuning.hpp#L206).*

<a id="struct-parameterlist"></a>

## `struct ParameterList`

```cpp
struct ParameterList
```

A mechanism's live numbers, in the page's order (gains first).

*struct, declared at [`include/shulib/mechanism/live_tuning.hpp:210`](../../include/shulib/mechanism/live_tuning.hpp#L210).*

<a id="parameterlist-items"></a>

### `ParameterList::items`

```cpp
std::array<Parameter, kTunableCount> items{}
```

The numbers.

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:212`](../../include/shulib/mechanism/live_tuning.hpp#L212).*

<a id="parameterlist-count"></a>

### `ParameterList::count`

```cpp
std::size_t count = 0
```

How many are real.

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:214`](../../include/shulib/mechanism/live_tuning.hpp#L214).*

<a id="enum-class-liverefusal"></a>

## `enum class LiveRefusal`

```cpp
enum class LiveRefusal : std::uint8_t
```

Why live tuning refused a change (each with its one sentence, liveRefusalText()).

*enum class, declared at [`include/shulib/mechanism/live_tuning.hpp:218`](../../include/shulib/mechanism/live_tuning.hpp#L218).*

<a id="liverefusal-none"></a>

### `LiveRefusal::None`

```cpp
None = 0
```

applied

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:219`](../../include/shulib/mechanism/live_tuning.hpp#L219).*

<a id="liverefusal-notlive"></a>

### `LiveRefusal::NotLive`

```cpp
NotLive
```

not a live number of this mechanism

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:220`](../../include/shulib/mechanism/live_tuning.hpp#L220).*

<a id="liverefusal-buildrefuses"></a>

### `LiveRefusal::BuildRefuses`

```cpp
BuildRefuses
```

the build's own check refuses the changed declaration (its sentence)

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:221`](../../include/shulib/mechanism/live_tuning.hpp#L221).*

<a id="liverefusal-notanumber"></a>

### `LiveRefusal::NotANumber`

```cpp
NotANumber
```

a number that is not finite

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:222`](../../include/shulib/mechanism/live_tuning.hpp#L222).*

<a id="liverefusal-gainnotabove0"></a>

### `LiveRefusal::GainNotAbove0`

```cpp
GainNotAbove0
```

kP, kI, kD or kV at or below 0

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:223`](../../include/shulib/mechanism/live_tuning.hpp#L223).*

<a id="liverefusal-feedforwardnegative"></a>

### `LiveRefusal::FeedforwardNegative`

```cpp
FeedforwardNegative
```

kS or kA below 0

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:224`](../../include/shulib/mechanism/live_tuning.hpp#L224).*

<a id="liverefusal-settleis0"></a>

### `LiveRefusal::SettleIs0`

```cpp
SettleIs0
```

a settle time of exactly 0

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:225`](../../include/shulib/mechanism/live_tuning.hpp#L225).*

<a id="liverefusal-kvpasttwicestarting"></a>

### `LiveRefusal::KvPastTwiceStarting`

```cpp
KvPastTwiceStarting
```

kV past kLiveKvMostTimesStarting × its starting value

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:226`](../../include/shulib/mechanism/live_tuning.hpp#L226).*

<a id="liverefusal-topspeedpastfree"></a>

### `LiveRefusal::TopSpeedPastFree`

```cpp
TopSpeedPastFree
```

the top speed past the output's free speed

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:227`](../../include/shulib/mechanism/live_tuning.hpp#L227).*

<a id="liverefusal-rangewidened"></a>

### `LiveRefusal::RangeWidened`

```cpp
RangeWidened
```

a travel limit wider than the robot file's

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:228`](../../include/shulib/mechanism/live_tuning.hpp#L228).*

<a id="liverefusal-homingpushturned"></a>

### `LiveRefusal::HomingPushTurned`

```cpp
HomingPushTurned
```

the homing push turned around

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:229`](../../include/shulib/mechanism/live_tuning.hpp#L229).*

<a id="liverefusal-capremoved"></a>

### `LiveRefusal::CapRemoved`

```cpp
CapRemoved
```

a stated cap removed by a change

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:230`](../../include/shulib/mechanism/live_tuning.hpp#L230).*

<a id="liverefusal-steprunning"></a>

### `LiveRefusal::StepRunning`

```cpp
StepRunning
```

a step runs on it: between steps only

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:231`](../../include/shulib/mechanism/live_tuning.hpp#L231).*

<a id="liverefusal-insideatick"></a>

### `LiveRefusal::InsideATick`

```cpp
InsideATick
```

the service is ticking: between ticks only

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:232`](../../include/shulib/mechanism/live_tuning.hpp#L232).*

<a id="liverefusal-leavesoutheldtarget"></a>

### `LiveRefusal::LeavesOutHeldTarget`

```cpp
LeavesOutHeldTarget
```

the new limits leave out the target it holds

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:233`](../../include/shulib/mechanism/live_tuning.hpp#L233).*

<a id="liverefusal-leavesoutwhereitis"></a>

### `LiveRefusal::LeavesOutWhereItIs`

```cpp
LeavesOutWhereItIs
```

holding nothing, the new limits leave where it is further outside them

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:234`](../../include/shulib/mechanism/live_tuning.hpp#L234).*

<a id="liverefusal-heldbuttonmoving"></a>

### `LiveRefusal::HeldButtonMoving`

```cpp
HeldButtonMoving
```

a held button moves it while the limits change

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:235`](../../include/shulib/mechanism/live_tuning.hpp#L235).*

<a id="liverefusal-nothingtoundo"></a>

### `LiveRefusal::NothingToUndo`

```cpp
NothingToUndo
```

undo() with no change left to undo

*enumerator, declared at [`include/shulib/mechanism/live_tuning.hpp:236`](../../include/shulib/mechanism/live_tuning.hpp#L236).*

<a id="liverefusaltext"></a>

## `liveRefusalText`

```cpp
[[nodiscard]] constexpr const char* liveRefusalText(LiveRefusal r) noexcept
```

The SENTENCE for a live refusal (for BuildRefuses: refusalText(specRefusal(...)) says it); nullptr for None.

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:241`](../../include/shulib/mechanism/live_tuning.hpp#L241).*

<a id="offered"></a>

## `offered`

```cpp
[[nodiscard]] inline bool offered(const MechanismSpec& s, Tunable t) noexcept
```

Whether mechanism `s` lets live tuning change `t` (header: what is live, per kind).

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:448`](../../include/shulib/mechanism/live_tuning.hpp#L448).*

<a id="tunablevalue"></a>

## `tunableValue`

```cpp
[[nodiscard]] inline double tunableValue(const MechanismSpec& s, Tunable t) noexcept
```

The number `t` of `s`, in the declaration's unit (a stall's rpm at the motor — the shortest one that is exactly the spec's rad/s); 0 where `s`'s kind has none.

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:498`](../../include/shulib/mechanism/live_tuning.hpp#L498).*

<a id="settunable"></a>

## `setTunable`

```cpp
inline void setTunable(MechanismSpec& s, Tunable t, double v) noexcept
```

Set the number `t` of `s` to `v` (the declaration's unit; a stall's rpm as kinds.hpp turns it into rad/s). Nothing for a number `s`'s kind does not have. No rule is checked here (tuningRefusal()).

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:511`](../../include/shulib/mechanism/live_tuning.hpp#L511).*

<a id="tunableunit"></a>

## `tunableUnit`

```cpp
[[nodiscard]] inline const char* tunableUnit(const MechanismSpec& s, Tunable t) noexcept
```

The unit `t` of `s` is shown in ("V/deg", "in/s", "A per motor").

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:530`](../../include/shulib/mechanism/live_tuning.hpp#L530).*

<a id="parameterof"></a>

## `parameterOf`

```cpp
[[nodiscard]] inline const Parameter& parameterOf(Tunable t) noexcept
```

The descriptor of `t` (its page name, step rule and member).

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:607`](../../include/shulib/mechanism/live_tuning.hpp#L607).*

<a id="parametersof"></a>

## `parametersOf`

```cpp
[[nodiscard]] inline ParameterList parametersOf(const MechanismSpec& s) noexcept
```

The live numbers mechanism `s` offers, in the page's order (header: what is live, per kind; a pneumatic offers none).

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:617`](../../include/shulib/mechanism/live_tuning.hpp#L617).*

<a id="tuningrefusal"></a>

## `tuningRefusal`

```cpp
[[nodiscard]] inline LiveRefusal tuningRefusal(const MechanismSpec& declared, const MechanismSpec& tuned) noexcept
```

THE LIVE RULES on a whole spec (header, rules 1 and 2): the build's own check FIRST (BuildRefuses — specRefusal(tuned) names which), then the live bounds the build lacks, each applied to a number that differs from `declared` (the robot file's) — and kS, kA, kV and the top speed only when that number is further past its bound than `declared`'s own (a file's number past one may step back toward it). Rule 3, the slot's state, is LiveTuning::apply()'s. NotLive when the two are not one mechanism's kind.

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:633`](../../include/shulib/mechanism/live_tuning.hpp#L633).*

<a id="tuningproblem"></a>

## `tuningProblem`

```cpp
[[nodiscard]] inline const char* tuningProblem(const MechanismSpec& declared, const MechanismSpec& tuned) noexcept
```

tuningRefusal() as a SENTENCE (nullptr: allowed) — for BuildRefuses, the build's own words.

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:700`](../../include/shulib/mechanism/live_tuning.hpp#L700).*

<a id="steppedvalue"></a>

## `steppedValue`

```cpp
[[nodiscard]] inline double steppedValue(const MechanismSpec& s, Tunable t, int direction, bool fine = false) noexcept
```

ONE [−] (`direction` < 0) or [+] (> 0) press on number `t` of `s` (header: the step rules; `fine` makes a gain's factor 1.05). The NEXT value — nothing is applied. Unchanged for a number `s` does not offer, or for `direction` 0.

*free function, declared at [`include/shulib/mechanism/live_tuning.hpp:708`](../../include/shulib/mechanism/live_tuning.hpp#L708).*

<a id="struct-applied"></a>

## `struct Applied`

```cpp
struct Applied
```

What one change, undo or reset did.

*struct, declared at [`include/shulib/mechanism/live_tuning.hpp:750`](../../include/shulib/mechanism/live_tuning.hpp#L750).*

<a id="applied-ok"></a>

### `Applied::ok`

```cpp
bool ok = false
```

True when applied (or when nothing needed to change).

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:752`](../../include/shulib/mechanism/live_tuning.hpp#L752).*

<a id="applied-why"></a>

### `Applied::why`

```cpp
LiveRefusal why = LiveRefusal::None
```

Why it was refused (None when ok).

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:754`](../../include/shulib/mechanism/live_tuning.hpp#L754).*

<a id="applied-build"></a>

### `Applied::build`

```cpp
Refusal build = Refusal::None
```

For BuildRefuses: the build's own reason.

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:756`](../../include/shulib/mechanism/live_tuning.hpp#L756).*

<a id="applied-sentence"></a>

### `Applied::sentence`

```cpp
char sentence[320]{}
```

One sentence for the page: "'lift': kP 0.25 -> 0.3125", or the refusal and why.

*field, declared at [`include/shulib/mechanism/live_tuning.hpp:758`](../../include/shulib/mechanism/live_tuning.hpp#L758).*

<a id="class-livetuning"></a>

## `class LiveTuning`

```cpp
class LiveTuning
```

LIVE TUNING (header): the robot's mechanism table as a mutable COPY the service runs, the robot file's own table for "narrowed only" and RESET, the changes, undo and the copy block. Lives in the program's static storage (the service's slots point into it); non-copyable and non-movable.

*class, declared at [`include/shulib/mechanism/live_tuning.hpp:764`](../../include/shulib/mechanism/live_tuning.hpp#L764).*

<a id="livetuning-livetuning"></a>

### `LiveTuning::LiveTuning`

```cpp
explicit LiveTuning(const MechanismTable& declared) noexcept
```

A copy of `declared` (the robot file's table — a constant that outlives this object).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:767`](../../include/shulib/mechanism/live_tuning.hpp#L767).*

<a id="livetuning-livetuning-2"></a>

### `LiveTuning::LiveTuning (overload 2)`

```cpp
LiveTuning(const LiveTuning&) = delete
```

Pinned: the service's slots point into its table.

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:771`](../../include/shulib/mechanism/live_tuning.hpp#L771).*

<a id="livetuning-operator-eq"></a>

### `LiveTuning::operator=`

```cpp
LiveTuning& operator=(const LiveTuning&) = delete
```

Pinned: the service's slots point into its table. Non-copy-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:773`](../../include/shulib/mechanism/live_tuning.hpp#L773).*

<a id="livetuning-livetuning-3"></a>

### `LiveTuning::LiveTuning (overload 3)`

```cpp
LiveTuning(LiveTuning&&) = delete
```

Pinned: the service's slots point into its table. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:775`](../../include/shulib/mechanism/live_tuning.hpp#L775).*

<a id="livetuning-operator-eq-2"></a>

### `LiveTuning::operator= (overload 2)`

```cpp
LiveTuning& operator=(LiveTuning&&) = delete
```

Pinned: the service's slots point into its table. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:777`](../../include/shulib/mechanism/live_tuning.hpp#L777).*

<a id="livetuning-destructor-livetuning"></a>

### `LiveTuning::~LiveTuning`

```cpp
~LiveTuning() = default
```

Pinned: the service's slots point into its table. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason). Nothing to release (the service it is attached to is the program's).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:779`](../../include/shulib/mechanism/live_tuning.hpp#L779).*

<a id="livetuning-table"></a>

### `LiveTuning::table`

```cpp
[[nodiscard]] const MechanismTable& table() const noexcept
```

THE TABLE THE SERVICE RUNS — bind the service over this, then attach() it.

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:782`](../../include/shulib/mechanism/live_tuning.hpp#L782).*

<a id="livetuning-declared"></a>

### `LiveTuning::declared`

```cpp
[[nodiscard]] const MechanismTable& declared() const noexcept
```

The robot file's table (never written).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:784`](../../include/shulib/mechanism/live_tuning.hpp#L784).*

<a id="livetuning-count"></a>

### `LiveTuning::count`

```cpp
[[nodiscard]] std::size_t count() const noexcept
```

How many mechanisms.

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:786`](../../include/shulib/mechanism/live_tuning.hpp#L786).*

<a id="livetuning-spec"></a>

### `LiveTuning::spec`

```cpp
[[nodiscard]] const MechanismSpec& spec(std::size_t i) const
```

Mechanism `i`'s live spec. PRECONDITION: `i` below count().

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:788`](../../include/shulib/mechanism/live_tuning.hpp#L788).*

<a id="livetuning-attach"></a>

### `LiveTuning::attach`

```cpp
void attach(MechanismService& service)
```

The service that runs this table. PRECONDITIONS: `service` bound over table() (each of its slots' spec IS this table's — landmine 10: a change here must reach the slots); attached once.

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:794`](../../include/shulib/mechanism/live_tuning.hpp#L794).*

<a id="livetuning-attached"></a>

### `LiveTuning::attached`

```cpp
[[nodiscard]] bool attached() const noexcept
```

True once attach() has run.

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:806`](../../include/shulib/mechanism/live_tuning.hpp#L806).*

<a id="livetuning-changed"></a>

### `LiveTuning::changed`

```cpp
[[nodiscard]] bool changed(std::size_t i) const
```

True when mechanism `i`'s live numbers differ from the robot file's (byte for byte).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:808`](../../include/shulib/mechanism/live_tuning.hpp#L808).*

<a id="livetuning-value"></a>

### `LiveTuning::value`

```cpp
[[nodiscard]] double value(std::size_t i, Tunable t) const
```

Mechanism `i`'s live number `t` (tunableValue()).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:813`](../../include/shulib/mechanism/live_tuning.hpp#L813).*

<a id="livetuning-stepped"></a>

### `LiveTuning::stepped`

```cpp
[[nodiscard]] double stepped(std::size_t i, Tunable t, int direction, bool fine = false) const
```

The value one press would set (steppedValue()); nothing applied.

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:815`](../../include/shulib/mechanism/live_tuning.hpp#L815).*

<a id="livetuning-undodepth"></a>

### `LiveTuning::undoDepth`

```cpp
[[nodiscard]] std::size_t undoDepth() const noexcept
```

How many changes undo() can still reach (at most kLiveUndoDepth).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:819`](../../include/shulib/mechanism/live_tuning.hpp#L819).*

<a id="livetuning-apply"></a>

### `LiveTuning::apply`

```cpp
Applied apply(std::size_t i, Tunable t, double v)
```

CHANGE mechanism `i`'s number `t` to `v` (the declaration's unit), inside the rules (header, rules 1–3, checked BEFORE the spec is written): applied, re-derived by the service (its one Info line) and undoable — or refused with its sentence and NOTHING changed. Setting a number to what it is already is ok and changes nothing. PRECONDITIONS: attached; `i` below count().

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:825`](../../include/shulib/mechanism/live_tuning.hpp#L825).*

<a id="livetuning-step"></a>

### `LiveTuning::step`

```cpp
Applied step(std::size_t i, Tunable t, int direction, bool fine = false)
```

One [−] (`direction` < 0) or [+] (> 0) press: apply(i, t, stepped(i, t, direction, fine)).

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:857`](../../include/shulib/mechanism/live_tuning.hpp#L857).*

<a id="livetuning-undo"></a>

### `LiveTuning::undo`

```cpp
Applied undo()
```

UNDO the last change (any mechanism's — the sentence names it), up to kLiveUndoDepth back: that mechanism's numbers before it, bit for bit, re-derived. Refused — and kept, to undo later — while that mechanism's step runs, inside a tick, or when it would narrow a range past a hold (rule 3); refused with nothing left to undo. PRECONDITION: attached.

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:864`](../../include/shulib/mechanism/live_tuning.hpp#L864).*

<a id="livetuning-reset"></a>

### `LiveTuning::reset`

```cpp
Applied reset(std::size_t i)
```

RESET mechanism `i` to the robot file's numbers, bit for bit, re-derived (one undoable change). Refused while its step runs or inside a tick (rule 3; the robot file's range — never narrower than the live one — always holds a hold the live range held, and never leaves where it is further out). PRECONDITIONS: attached; `i` below count().

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:899`](../../include/shulib/mechanism/live_tuning.hpp#L899).*

<a id="livetuning-copyblock"></a>

### `LiveTuning::copyBlock`

```cpp
int copyBlock(std::size_t i, hal::ITelemetrySink& sink) const
```

COPY (D9 (a), header): mechanism `i`'s block of `[TUNE]` lines into `sink` (tag "TUNE", Info) — a begin line, the SMALLEST set of members whose declaration builds to exactly the live spec, and an end line with their count. Returns that count (0 for an untuned one: the block says so with no member line). PRECONDITION: `i` below count().

*function, declared at [`include/shulib/mechanism/live_tuning.hpp:924`](../../include/shulib/mechanism/live_tuning.hpp#L924).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 113 lines, click to expand</summary>

```text

 mechanism/live_tuning.hpp — LIVE TUNING's library half: a mechanism's numbers changed WHILE THE
 ROBOT RUNS, inside the same rules as the build, undone, and copied back out as the lines its robot
 file needs (chunk TUNE1-A, 2026-11-10; the MECH-S/TUNE1 brief's D8 (a) and D9 (a), with its
 re-check amendments #7 and #14).

 ── Why ─────────────────────────────────────────────────────────────────────────────
 The team lead (2026-10-29): "things need to be able to be tuned" — a string lift's gains, speeds and
 limits found ON THE ROBOT, its response watched, the numbers then written into the robot file. Every
 number a mechanism runs with is its spec's (mechanism/spec.hpp), and the service's slots read it —
 but not all of it every tick: the PIDs COPY their gains at bind, the stall detectors their
 signatures, the soft stop its limits in degrees (the brief's P6, landmine 10). A spec changed under a
 slot without telling it changes nothing, or worse, half of the numbers. So live tuning owns the
 table the service runs, changes it ONLY through apply() / undo() / reset(), and every change is
 followed by MechanismService::rederive() — the slot re-reads every copy.

 ── How it is wired (the Tune program, TUNE1-B1, is the one caller) ──────────────────
     static mechanism::LiveTuning tuning{kRobotTwoMechanisms};   // a COPY of the robot file's table
     service.bind(tuning.table(), devices, clock, faults, sink); // the service runs THE COPY
     tuning.attach(service);                                     // checked: the service is over it
 The robot file's table is never written (it is a constant): RESET puts its numbers back, bit for bit,
 and no number persists in the robot file — leaving the program or a reboot runs the robot file's
 numbers again; every program but Tune always does. ONE EXCEPTION, in the MOTORS: a current cap
 changed live is written into each motor (MECH-S-A's boot path), and a motor keeps a limit after the
 program that set it ends (believed — HA-220); the next program reads it and says so but, stating no
 cap, writes nothing (D12 (b)). So RESET — or UNDO back to the file's cap — every mechanism whose cap
 changed BEFORE leaving: that writes the robot file's cap back, or the motor's 2.5 A default when it
 states none. Nothing here does it at the program's end (a static object's destructor is no place to
 drive a motor). The match programs never include this header (TUNE1-B1's build gate checks their
 linked symbols).

 ── The rules (D8 (a)) ───────────────────────────────────────────────────────────────
 What is live, per kind (parametersOf()): a POSITION mechanism (Arm, Lift) — its ten `.tuning`
 numbers (kP, kI, kD, kS, kV, kA, top speed, acceleration, within, settle), the gravity hold (kG, when
 it has gravity), the stall signature, the homing push and its signature (when it homes by switch or
 by stall), the current cap and the travel limits; a FLYWHEEL — kS, kV, kP, kI, the at-speed band and
 time, the stall signature, the cap (no kA: SpeedTuning cannot say one, so the page cannot either —
 re-check #14); a POWER mechanism — maxVolts, the stall signature (when it states one) and the cap.
 NOT live: presets (a goal finds its preset BY VALUE — P7: change "high" and `find(lift["high"])`
 answers nobody), the gearing, the shape, the homing method and position, a pneumatic's anything.
 A change is APPLIED only when every one of these holds, checked BEFORE the spec is written:
   1. the changed declaration passes THE BUILD'S OWN CHECK (specRefusal — its sentence, word for word);
   2. THE LIVE BOUNDS the build lacks (tuningRefusal — P9: the build accepts a NaN kP, a negative kS,
      kV = 1e9 and a 500 in/s top speed on a lift that turns at most 7.85 in/s): every number finite;
      kP, kI, kD and kV above 0 and the settle time never 0 — never a value the robot file CANNOT SAY
      (a `.tuning` 0 means "the starting value"); kS and kA not negative; kV at most twice its starting
      value (12 V over the output's free speed — kLiveKvMostTimesStarting, an INVENTED library bound);
      the top speed at most the output's free speed (the starting profile's own scale); a travel limit
      only NARROWED (never wider than the robot file's — and the build already refuses one past a
      preset or the home position); the homing push never turned around; a stated cap never removed
      by a change (UNDO and RESET put it back). Each bound applies to a number that DIFFERS from the
      robot file's — a number as the file states it is the build's to judge, so a file the build
      accepted can always be tuned — and a number the file states PAST a bound (a 10 in/s top speed
      over a 7.85 in/s output) may step back TOWARD it: kS, kA, kV and the top speed are refused only
      when further past than the file's own number;
   3. THE SLOT'S STATE: not while the service ticks (from a predicate, a sink — anything a tick calls),
      not while that mechanism's STEP runs (a change is applied BETWEEN steps), and for the travel
      limits never past the target it is HOLDING now, never while a held button moves it, and —
      holding nothing — never past WHERE IT IS (its next hold, a HOME or a let-go, begins where it is
      clamped inside the limits: a jump) (PositionSlot::takesRange — re-check #7: a hold is never moved,
      now or at the next hold; the draft's re-clamp asked 153 V) — each refusal one sentence, and nothing
      changed.
 Each [−] / [+] press is stepped(): a gain ×/÷ 1.25 (FINE ×/÷ 1.05); kA, which starts at 0 and cannot
 leave it by a factor, first steps to kV × 0.01 s (a 10 ms lag's feedforward), then ×/÷ 1.25, and a
 [−] below that first step goes back to 0 (which the file can say); volts and amps ±0.05; a speed, an
 acceleration, a stall's rpm or the at-speed band ×/÷ 1.05 (about 5 %); "within" ±0.05 in or ±0.5 deg;
 a time ±0.05 s; a travel limit ±0.25 in or ±1 deg. A factor's result is rounded to six significant
 figures and a step's to the micro-unit, so the copy's numbers stay short; a step never removes a cap
 (0 A would be NO cap — the motor's full 2.5 A).

 ── Undone (D8 (a)) ──────────────────────────────────────────────────────────────────
 undo(): the LAST change (any mechanism's — the sentence names it), up to kLiveUndoDepth (16) back; the
 17th is refused. reset(i): mechanism `i` back to the robot file's numbers, bit for bit (itself one
 undoable change). Both obey rule 3 (an undo that would narrow a range past a hold, or past where it
 is, waits). Every copy of a spec here is a byte copy, so "bit for bit" is exactly that.

 ── Copied back out (D9 (a), re-check #14) ───────────────────────────────────────────
 copyBlock(i, sink) logs ONE block of `[TUNE]` lines (tag "TUNE"): a begin line naming the
 declaration's own file and line, then THE SMALLEST SET of members whose declaration builds to exactly
 the live spec, then an end line with the count. The cascade's (mechanism_live_tuning_rig.hpp's, a file
 with no `.tuning`), after one [+] on kP and a top speed of 2 in/s:
     'cascade': copy begins -- write these into its declaration (tuned live): <its file>:<its line>
     .tuning.kP = 0.3125
     .tuning.maxSpeed = 2
     .tuning.maxAcceleration = 23.561944901923447
     'cascade': copy ends -- 3 lines
 Which members — a PATCH, member by member (TUNE1-B2's import must MERGE it, never replace the
 declaration): every member changed live, written WHOLE as the declaration writes it (`.stall = {...}`,
 `.home = homeByStall(...)`, `.gravity = constantGravity(...)`), and no other (`.stall`, `.range`,
 `.home`, `.gravity`, `.currentCap` as the file states them stay as written); and — when any `.tuning`
 number changed live — the WHOLE `.tuning` the declaration then needs (the student replaces
 `.tuning = {...}` with it): every number changed live, then, one at a time in the derivation's order,
 every number whose derivation from the set so far would differ from the live one — found by running
 the file's own derivation (detail::positionStart / speedStart) on the candidate and comparing. So a
 changed top speed carries the acceleration it was tuned with (landmine 12), a number the file states
 that the library would work out differently stays, and an untouched kV stays derived (a stated kV
 would go stale on a cartridge change). A number the file states AT the value the library would work
 out anyway cannot be told from a derived one (the spec holds values, not statements): it is LEFT OUT
 — the block rebuilds the same spec today, but after a cartridge or gearing change that number follows
 the library's derivation. With no `.tuning` number changed, no `.tuning` line: the file's own `.tuning`
 stays as written (no other live number moves a derived one — they hang on the gearing, the cartridge
 and the shape). The `.tuning` lines name each number the C++ way the student writes them
 into `.tuning = {...}`, in its member order (C++ needs designated initializers in that order). Every
 number is printed SHORTEST-ROUND-TRIP (std::to_chars), so it reads back as the same double. Every line
 is at most 200 bytes (TermSink cuts a message there — landmine 16), by construction: a number is at
 most kCopyNumberChars (24) characters, and the longest line — a stall homing one — is 77 fixed bytes
 and five numbers, 197 at most. The cost the team carries (D9): a number a COPY states stays as tuned
 if the cartridge, gearing or spool changes later — RESET and re-tune after such a change. On the
 host the `_s`/`_in`/`_volt` literals read through long double (x87, 80 bits) and then round to
 double; on the V5's ARM a long double IS a double, so a robot file reads each number exactly.

 PROS-free, allocation-free. NOTHING HERE HAS RUN ON A ROBOT: the tests drive it over the host test
 plant (its constants invented — HA-183).
```

</details>
