<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/kinds.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `kinds.hpp`

mechanism/kinds.hpp — WHAT A MECHANISM IS, the way a team writes it.

This header declares **30** types (178 members), **18** free functions, **15** constants, and **31** type aliass.

Extracted from [`include/shulib/mechanism/kinds.hpp`](../../include/shulib/mechanism/kinds.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct Name`](#struct-name)
  - [`text`](#name-text)
  - [`file`](#name-file)
  - [`line`](#name-line)
  - [`Name`](#name-name)
  - [`Name (overload 2)`](#name-name-2)
- [`struct Motor`](#struct-motor)
  - [`present`](#motor-present)
  - [`reversed`](#motor-reversed)
  - [`changeAt`](#motor-changeat)
- [`Forward`](#forward) — *constant*
- [`Reversed`](#reversed) — *constant*
- [`struct Motors`](#struct-motors)
  - [`m`](#motors-m)
  - [`count`](#motors-count)
  - [`tooMany`](#motors-toomany)
  - [`file`](#motors-file)
  - [`line`](#motors-line)
  - [`Motors`](#motors-motors)
  - [`Motors (overload 2)`](#motors-motors-2)
- [`struct Sensor`](#struct-sensor)
  - [`kind`](#sensor-kind)
  - [`reversed`](#sensor-reversed)
  - [`pressedLevel`](#sensor-pressedlevel)
  - [`seesWithinIn`](#sensor-seeswithinin)
  - [`seesProximity`](#sensor-seesproximity)
  - [`changeAt`](#sensor-changeat)
  - [`file`](#sensor-file)
  - [`line`](#sensor-line)
- [`rotationSensor`](#rotationsensor) — *free function*
- [`distanceSensor`](#distancesensor) — *free function*
- [`opticalSensor`](#opticalsensor) — *free function*
- [`limitSwitch`](#limitswitch) — *free function*
- [`struct Stall`](#struct-stall)
  - [`amps`](#stall-amps)
  - [`belowMotorRpm`](#stall-belowmotorrpm)
  - [`lasting`](#stall-lasting)
- [`struct Gearing`](#struct-gearing)
  - [`motorTurnsPerTurn`](#gearing-motorturnsperturn)
  - [`Gearing`](#gearing-gearing)
  - [`Gearing (overload 2)`](#gearing-gearing-2)
  - [`operator*`](#gearing-operator-star)
- [`gears`](#gears) — *free function*
- [`direct`](#direct) — *constant*
- [`struct GravityTerm`](#struct-gravityterm)
  - [`kind`](#gravityterm-kind)
  - [`volts`](#gravityterm-volts)
  - [`levelAtDeg`](#gravityterm-levelatdeg)
- [`cosine`](#cosine) — *free function*
- [`constantGravity`](#constantgravity) — *free function*
- [`noGravity`](#nogravity) — *constant*
- [`struct Homing`](#struct-homing)
  - [`method`](#homing-method)
  - [`at`](#homing-at)
  - [`sensorAtZeroDeg`](#homing-sensoratzerodeg)
  - [`volts`](#homing-volts)
  - [`stall`](#homing-stall)
- [`restsAtStop`](#restsatstop) — *free function*
- [`restsAtStop (overload 2)`](#restsatstop-2) — *free function*
- [`homeBySwitch`](#homebyswitch) — *free function*
- [`homeBySwitch (overload 2)`](#homebyswitch-2) — *free function*
- [`homeByStall`](#homebystall) — *free function*
- [`homeByStall (overload 2)`](#homebystall-2) — *free function*
- [`absoluteSensor`](#absolutesensor) — *free function*
- [`neverHomed`](#neverhomed) — *constant*
- [`struct DegreeRange`](#struct-degreerange)
  - [`min`](#degreerange-min)
  - [`max`](#degreerange-max)
- [`struct InchRange`](#struct-inchrange)
  - [`min`](#inchrange-min)
  - [`max`](#inchrange-max)
- [`struct DegreePreset`](#struct-degreepreset)
  - [`name`](#degreepreset-name)
  - [`value`](#degreepreset-value)
- [`struct InchPreset`](#struct-inchpreset)
  - [`name`](#inchpreset-name)
  - [`value`](#inchpreset-value)
- [`struct RpmPreset`](#struct-rpmpreset)
  - [`name`](#rpmpreset-name)
  - [`rpm`](#rpmpreset-rpm)
- [`struct LiftShape`](#struct-liftshape)
  - [`geometry`](#liftshape-geometry)
  - [`spoolDiameterIn`](#liftshape-spooldiameterin)
  - [`armLengthIn`](#liftshape-armlengthin)
  - [`pivotHeightIn`](#liftshape-pivotheightin)
  - [`stages`](#liftshape-stages)
  - [`table`](#liftshape-table)
  - [`points`](#liftshape-points)
- [`spool`](#spool) — *free function*
- [`linkage`](#linkage) — *free function*
- [`struct HeightAt`](#struct-heightat)
  - [`height`](#heightat-height)
  - [`degrees`](#heightat-degrees)
- [`measured`](#measured) — *free function*
- [`struct PositionTuning`](#struct-positiontuning)
  - [`kP`](#positiontuning-kp)
  - [`kI`](#positiontuning-ki)
  - [`kD`](#positiontuning-kd)
  - [`kS`](#positiontuning-ks)
  - [`kV`](#positiontuning-kv)
  - [`kA`](#positiontuning-ka)
  - [`maxSpeed`](#positiontuning-maxspeed)
  - [`maxAcceleration`](#positiontuning-maxacceleration)
  - [`within`](#positiontuning-within)
  - [`settle`](#positiontuning-settle)
- [`struct SpeedTuning`](#struct-speedtuning)
  - [`kS`](#speedtuning-ks)
  - [`kV`](#speedtuning-kv)
  - [`kP`](#speedtuning-kp)
  - [`kI`](#speedtuning-ki)
- [`struct PositionGoal`](#struct-positiongoal)
  - [`mechanism`](#positiongoal-mechanism)
  - [`mechanismName`](#positiongoal-mechanismname)
  - [`presetName`](#positiongoal-presetname)
  - [`value`](#positiongoal-value)
  - [`computed`](#positiongoal-computed)
- [`struct SpeedGoal`](#struct-speedgoal)
  - [`mechanism`](#speedgoal-mechanism)
  - [`mechanismName`](#speedgoal-mechanismname)
  - [`presetName`](#speedgoal-presetname)
  - [`rpm`](#speedgoal-rpm)
  - [`computed`](#speedgoal-computed)
- [`struct PneumaticGoal`](#struct-pneumaticgoal)
  - [`mechanism`](#pneumaticgoal-mechanism)
  - [`mechanismName`](#pneumaticgoal-mechanismname)
  - [`position`](#pneumaticgoal-position)
  - [`positionName`](#pneumaticgoal-positionname)
- [`struct Power`](#struct-power)
  - [`name`](#power-name)
  - [`motors`](#power-motors)
  - [`cartridge`](#power-cartridge)
  - [`pieceSensor`](#power-piecesensor)
  - [`safe`](#power-safe)
  - [`stall`](#power-stall)
  - [`maxVolts`](#power-maxvolts)
  - [`operator[]`](#power-operator-index)
  - [`spec`](#power-spec)
- [`struct Arm`](#struct-arm)
  - [`name`](#arm-name)
  - [`motors`](#arm-motors)
  - [`cartridge`](#arm-cartridge)
  - [`gearing`](#arm-gearing)
  - [`angleSensor`](#arm-anglesensor)
  - [`safe`](#arm-safe)
  - [`stall`](#arm-stall)
  - [`range`](#arm-range)
  - [`gravity`](#arm-gravity)
  - [`home`](#arm-home)
  - [`presets`](#arm-presets)
  - [`unhomedVolts`](#arm-unhomedvolts)
  - [`tuning`](#arm-tuning)
  - [`operator[]`](#arm-operator-index)
  - [`to`](#arm-to)
  - [`requires`](#arm-requires)
  - [`clampedTo`](#arm-clampedto)
  - [`spec`](#arm-spec)
- [`struct Lift`](#struct-lift)
  - [`name`](#lift-name)
  - [`motors`](#lift-motors)
  - [`cartridge`](#lift-cartridge)
  - [`gearing`](#lift-gearing)
  - [`shape`](#lift-shape)
  - [`angleSensor`](#lift-anglesensor)
  - [`safe`](#lift-safe)
  - [`stall`](#lift-stall)
  - [`range`](#lift-range)
  - [`gravity`](#lift-gravity)
  - [`home`](#lift-home)
  - [`presets`](#lift-presets)
  - [`unhomedVolts`](#lift-unhomedvolts)
  - [`tuning`](#lift-tuning)
  - [`operator[]`](#lift-operator-index)
  - [`to`](#lift-to)
  - [`to (overload 2)`](#lift-to-2)
  - [`clampedTo`](#lift-clampedto)
  - [`spec`](#lift-spec)
- [`struct AtSpeed`](#struct-atspeed)
  - [`rpm`](#atspeed-rpm)
  - [`lasting`](#atspeed-lasting)
- [`within`](#within) — *free function*
- [`struct Flywheel`](#struct-flywheel)
  - [`name`](#flywheel-name)
  - [`motors`](#flywheel-motors)
  - [`cartridge`](#flywheel-cartridge)
  - [`gearing`](#flywheel-gearing)
  - [`angleSensor`](#flywheel-anglesensor)
  - [`safe`](#flywheel-safe)
  - [`stall`](#flywheel-stall)
  - [`atSpeed`](#flywheel-atspeed)
  - [`presets`](#flywheel-presets)
  - [`tuning`](#flywheel-tuning)
  - [`freeRpm`](#flywheel-freerpm)
  - [`operator[]`](#flywheel-operator-index)
  - [`at`](#flywheel-at)
  - [`clampedTo`](#flywheel-clampedto)
  - [`spec`](#flywheel-spec)
- [`struct Levels`](#struct-levels)
  - [`l`](#levels-l)
  - [`count`](#levels-count)
  - [`tooMany`](#levels-toomany)
  - [`Levels`](#levels-levels)
  - [`Levels (overload 2)`](#levels-levels-2)
- [`struct NamedPosition`](#struct-namedposition)
  - [`name`](#namedposition-name)
  - [`levels`](#namedposition-levels)
  - [`confirm`](#namedposition-confirm)
- [`struct Positions`](#struct-positions)
  - [`p`](#positions-p)
  - [`count`](#positions-count)
  - [`tooMany`](#positions-toomany)
  - [`file`](#positions-file)
  - [`line`](#positions-line)
  - [`Positions`](#positions-positions)
  - [`Positions (overload 2)`](#positions-positions-2)
- [`High`](#high) — *constant*
- [`Low`](#low) — *constant*
- [`grabs`](#grabs) — *constant*
- [`releases`](#releases) — *constant*
- [`struct Pneumatic`](#struct-pneumatic)
  - [`name`](#pneumatic-name)
  - [`positions`](#pneumatic-positions)
  - [`start`](#pneumatic-start)
  - [`safe`](#pneumatic-safe)
  - [`moveTime`](#pneumatic-movetime)
  - [`pieceSensor`](#pneumatic-piecesensor)
  - [`confirmWithin`](#pneumatic-confirmwithin)
  - [`changeAt`](#pneumatic-changeat)
  - [`to`](#pneumatic-to)
  - [`operator[]`](#pneumatic-operator-index)
  - [`spec`](#pneumatic-spec)
- [`Arm`](#arm) — *type alias*
- [`Lift`](#lift) — *type alias*
- [`Pneumatic`](#pneumatic) — *type alias*
- [`Power`](#power) — *type alias*
- [`Flywheel`](#flywheel) — *type alias*
- [`Forward (overload 2)`](#forward-2) — *type alias*
- [`Reversed (overload 2)`](#reversed-2) — *type alias*
- [`rotationSensor (overload 2)`](#rotationsensor-2) — *type alias*
- [`distanceSensor (overload 2)`](#distancesensor-2) — *type alias*
- [`opticalSensor (overload 2)`](#opticalsensor-2) — *type alias*
- [`limitSwitch (overload 2)`](#limitswitch-2) — *type alias*
- [`Stall`](#stall) — *type alias*
- [`cosine (overload 2)`](#cosine-2) — *type alias*
- [`constantGravity (overload 2)`](#constantgravity-2) — *type alias*
- [`noGravity (overload 2)`](#nogravity-2) — *type alias*
- [`restsAtStop (overload 3)`](#restsatstop-3) — *type alias*
- [`homeBySwitch (overload 3)`](#homebyswitch-3) — *type alias*
- [`homeByStall (overload 3)`](#homebystall-3) — *type alias*
- [`absoluteSensor (overload 2)`](#absolutesensor-2) — *type alias*
- [`neverHomed (overload 2)`](#neverhomed-2) — *type alias*
- [`spool (overload 2)`](#spool-2) — *type alias*
- [`linkage (overload 2)`](#linkage-2) — *type alias*
- [`measured (overload 2)`](#measured-2) — *type alias*
- [`HeightAt`](#heightat) — *type alias*
- [`within (overload 2)`](#within-2) — *type alias*
- [`High (overload 2)`](#high-2) — *type alias*
- [`Low (overload 2)`](#low-2) — *type alias*
- [`grabs (overload 2)`](#grabs-2) — *type alias*
- [`releases (overload 2)`](#releases-2) — *type alias*
- [`gears (overload 2)`](#gears-2) — *type alias*
- [`direct (overload 2)`](#direct-2) — *type alias*
- [`Red`](#red) — *constant*
- [`Green`](#green) — *constant*
- [`Blue`](#blue) — *constant*
- [`struct BrakeWord`](#struct-brakeword)
  - [`safe`](#brakeword-safe)
  - [`drive`](#brakeword-drive)
  - [`config::SafeBrakeMode`](#brakeword-config-safebrakemode)
  - [`hal::BrakeMode`](#brakeword-hal-brakemode)
- [`Hold`](#hold) — *constant*
- [`Brake`](#brake) — *constant*
- [`Coast`](#coast) — *constant*

<a id="struct-name"></a>

## `struct Name`

```cpp
struct Name
```

A mechanism's name ("claw pivot 6": boot lines, logs, the route ids' stem, the run-time lookup). Written as a plain string (`.name = "arm"`); it also records the FILE and LINE it was written on, which is how the direction check and the boot lines point at the declaration (header).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:90`](../../include/shulib/mechanism/kinds.hpp#L90).*

<a id="name-text"></a>

### `Name::text`

```cpp
const char* text = nullptr
```

The name's text, borrowed from the team's literal (null = UNSET).

*field, declared at [`include/shulib/mechanism/kinds.hpp:92`](../../include/shulib/mechanism/kinds.hpp#L92).*

<a id="name-file"></a>

### `Name::file`

```cpp
const char* file = nullptr
```

The file the `.name = ...` was written in (the path the compiler opened).

*field, declared at [`include/shulib/mechanism/kinds.hpp:94`](../../include/shulib/mechanism/kinds.hpp#L94).*

<a id="name-line"></a>

### `Name::line`

```cpp
std::uint32_t line = 0
```

The line it was written on.

*field, declared at [`include/shulib/mechanism/kinds.hpp:96`](../../include/shulib/mechanism/kinds.hpp#L96).*

<a id="name-name"></a>

### `Name::Name`

```cpp
constexpr Name() noexcept = default
```

UNSET.

*function, declared at [`include/shulib/mechanism/kinds.hpp:98`](../../include/shulib/mechanism/kinds.hpp#L98).*

<a id="name-name-2"></a>

### `Name::Name (overload 2)`

```cpp
consteval Name(const char* s, std::source_location at = std::source_location::current()) noexcept
```

UNSET. `.name = "arm"`: the text, and where it was written (the defaulted `at` is the team's line).

*function, declared at [`include/shulib/mechanism/kinds.hpp:100`](../../include/shulib/mechanism/kinds.hpp#L100).*

<a id="struct-motor"></a>

## `struct Motor`

```cpp
struct Motor
```

One MOTOR role: how the motor is MOUNTED (which way + turns the mechanism), never its port.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:107`](../../include/shulib/mechanism/kinds.hpp#L107).*

<a id="motor-present"></a>

### `Motor::present`

```cpp
bool present = false
```

Set by Forward / Reversed: a role the mechanism has.

*field, declared at [`include/shulib/mechanism/kinds.hpp:109`](../../include/shulib/mechanism/kinds.hpp#L109).*

<a id="motor-reversed"></a>

### `Motor::reversed`

```cpp
bool reversed = false
```

Mounted so that + turns the mechanism the other way.

*field, declared at [`include/shulib/mechanism/kinds.hpp:111`](../../include/shulib/mechanism/kinds.hpp#L111).*

<a id="motor-changeat"></a>

### `Motor::changeAt`

```cpp
const char* changeAt = nullptr
```

Generated robot files only: this role's JSON path (the direction check's "change it HERE").

*field, declared at [`include/shulib/mechanism/kinds.hpp:113`](../../include/shulib/mechanism/kinds.hpp#L113).*

<a id="forward"></a>

## `Forward`

```cpp
inline constexpr Motor Forward{.present = true}
```

A motor mounted so that + moves the mechanism its "+" way (up, in, out — the team's word).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:116`](../../include/shulib/mechanism/kinds.hpp#L116).*

<a id="reversed"></a>

## `Reversed`

```cpp
inline constexpr Motor Reversed{.present = true, .reversed = true}
```

A motor mounted the other way round (the mirror-image motor of a two-motor lift).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:118`](../../include/shulib/mechanism/kinds.hpp#L118).*

<a id="struct-motors"></a>

## `struct Motors`

```cpp
struct Motors
```

A mechanism's MOTOR roles, in order: `.motors = {Forward, Reversed}` (one to four; a fifth is recorded as "too many" and refused by the list).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:122`](../../include/shulib/mechanism/kinds.hpp#L122).*

<a id="motors-m"></a>

### `Motors::m`

```cpp
Motor m[kMaxMotorRoles]{}
```

The roles, in the order written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:124`](../../include/shulib/mechanism/kinds.hpp#L124).*

<a id="motors-count"></a>

### `Motors::count`

```cpp
std::uint8_t count = 0
```

How many were written (at most kMaxMotorRoles).

*field, declared at [`include/shulib/mechanism/kinds.hpp:126`](../../include/shulib/mechanism/kinds.hpp#L126).*

<a id="motors-toomany"></a>

### `Motors::tooMany`

```cpp
bool tooMany = false
```

More than kMaxMotorRoles were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:128`](../../include/shulib/mechanism/kinds.hpp#L128).*

<a id="motors-file"></a>

### `Motors::file`

```cpp
const char* file = nullptr
```

Where the list was written (GCC 13 captures an initializer list at the declaration's CLOSING line — measured — so the spec points every role at the `.name` line instead).

*field, declared at [`include/shulib/mechanism/kinds.hpp:131`](../../include/shulib/mechanism/kinds.hpp#L131).*

<a id="motors-line"></a>

### `Motors::line`

```cpp
std::uint32_t line = 0
```

That line.

*field, declared at [`include/shulib/mechanism/kinds.hpp:133`](../../include/shulib/mechanism/kinds.hpp#L133).*

<a id="motors-motors"></a>

### `Motors::Motors`

```cpp
constexpr Motors() noexcept = default
```

UNSET: no motors.

*function, declared at [`include/shulib/mechanism/kinds.hpp:135`](../../include/shulib/mechanism/kinds.hpp#L135).*

<a id="motors-motors-2"></a>

### `Motors::Motors (overload 2)`

```cpp
consteval Motors(std::initializer_list<Motor> list, std::source_location at = std::source_location::current()) noexcept
```

UNSET: no motors. `.motors = {Forward, Reversed}`.

*function, declared at [`include/shulib/mechanism/kinds.hpp:137`](../../include/shulib/mechanism/kinds.hpp#L137).*

<a id="struct-sensor"></a>

## `struct Sensor`

```cpp
struct Sensor
```

One SENSOR role (an angle sensor, a piece sensor). Made by the words below, never by hand.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:151`](../../include/shulib/mechanism/kinds.hpp#L151).*

<a id="sensor-kind"></a>

### `Sensor::kind`

```cpp
SensorKind kind = SensorKind::None
```

What it is; None = no such role.

*field, declared at [`include/shulib/mechanism/kinds.hpp:153`](../../include/shulib/mechanism/kinds.hpp#L153).*

<a id="sensor-reversed"></a>

### `Sensor::reversed`

```cpp
bool reversed = false
```

Rotation: mounted so that + reads the other way.

*field, declared at [`include/shulib/mechanism/kinds.hpp:155`](../../include/shulib/mechanism/kinds.hpp#L155).*

<a id="sensor-pressedlevel"></a>

### `Sensor::pressedLevel`

```cpp
bool pressedLevel = true
```

Switch: the level that means PRESSED — `true` (HA-184's claim).

*field, declared at [`include/shulib/mechanism/kinds.hpp:157`](../../include/shulib/mechanism/kinds.hpp#L157).*

<a id="sensor-seeswithinin"></a>

### `Sensor::seesWithinIn`

```cpp
double seesWithinIn = 0.0
```

Distance: closer than this many inches = "sees a piece".

*field, declared at [`include/shulib/mechanism/kinds.hpp:159`](../../include/shulib/mechanism/kinds.hpp#L159).*

<a id="sensor-seesproximity"></a>

### `Sensor::seesProximity`

```cpp
double seesProximity = 0.0
```

Optical: proximity at or above this = "sees a piece", (0, 1].

*field, declared at [`include/shulib/mechanism/kinds.hpp:161`](../../include/shulib/mechanism/kinds.hpp#L161).*

<a id="sensor-changeat"></a>

### `Sensor::changeAt`

```cpp
const char* changeAt = nullptr
```

Generated robot files only: this role's JSON path.

*field, declared at [`include/shulib/mechanism/kinds.hpp:163`](../../include/shulib/mechanism/kinds.hpp#L163).*

<a id="sensor-file"></a>

### `Sensor::file`

```cpp
const char* file = nullptr
```

Where it was written (captured).

*field, declared at [`include/shulib/mechanism/kinds.hpp:165`](../../include/shulib/mechanism/kinds.hpp#L165).*

<a id="sensor-line"></a>

### `Sensor::line`

```cpp
std::uint32_t line = 0
```

That line.

*field, declared at [`include/shulib/mechanism/kinds.hpp:167`](../../include/shulib/mechanism/kinds.hpp#L167).*

<a id="rotationsensor"></a>

## `rotationSensor`

```cpp
[[nodiscard]] constexpr Sensor rotationSensor(Motor mounted = Forward, std::source_location at = std::source_location::current()) noexcept
```

A V5 ROTATION sensor on the mechanism's output shaft — its angle sensor. `mounted` Reversed when + reads the other way.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:171`](../../include/shulib/mechanism/kinds.hpp#L171).*

<a id="distancesensor"></a>

## `distanceSensor`

```cpp
[[nodiscard]] constexpr Sensor distanceSensor(units::Length within, std::source_location at = std::source_location::current()) noexcept
```

A DISTANCE sensor as a piece sensor: "sees a piece" when something is closer than `within`.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:176`](../../include/shulib/mechanism/kinds.hpp#L176).*

<a id="opticalsensor"></a>

## `opticalSensor`

```cpp
[[nodiscard]] constexpr Sensor opticalSensor(double proximity, std::source_location at = std::source_location::current()) noexcept
```

An OPTICAL sensor as a piece sensor: "sees a piece" at proximity >= `proximity`, in (0, 1].

*free function, declared at [`include/shulib/mechanism/kinds.hpp:181`](../../include/shulib/mechanism/kinds.hpp#L181).*

<a id="limitswitch"></a>

## `limitSwitch`

```cpp
[[nodiscard]] constexpr Sensor limitSwitch(std::source_location at = std::source_location::current()) noexcept
```

A LIMIT SWITCH (or bumper) on a 3-wire port as a piece sensor: pressed = sees a piece.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:186`](../../include/shulib/mechanism/kinds.hpp#L186).*

<a id="struct-stall"></a>

## `struct Stall`

```cpp
struct Stall
```

A STALL SIGNATURE: the motors draw at least `amps` while turning slower than `belowMotorRpm` (at the MOTOR shaft — every other rpm a team writes is the output's) for `lasting` (manipulation::StallDetector's three numbers). The team's — never a default. REQUIRED on an Arm, a Lift and a Flywheel (N10: their behaviour goes on after the step — a hold, a spin — and a blocked one must never sit at stall current, P4); optional on a Power mechanism. A stated one must be able to trip: amps below the motor's 2.5 A limit, belowMotorRpm above 0, lasting above 0.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:196`](../../include/shulib/mechanism/kinds.hpp#L196).*

<a id="stall-amps"></a>

### `Stall::amps`

```cpp
double amps = 0.0
```

At least this current, A (below 2.5).

*field, declared at [`include/shulib/mechanism/kinds.hpp:198`](../../include/shulib/mechanism/kinds.hpp#L198).*

<a id="stall-belowmotorrpm"></a>

### `Stall::belowMotorRpm`

```cpp
double belowMotorRpm = 0.0
```

While the MOTOR shaft turns slower than this, rpm (above 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:200`](../../include/shulib/mechanism/kinds.hpp#L200).*

<a id="stall-lasting"></a>

### `Stall::lasting`

```cpp
units::Time lasting{0.0}
```

Continuously for this long (above 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:202`](../../include/shulib/mechanism/kinds.hpp#L202).*

<a id="struct-gearing"></a>

## `struct Gearing`

```cpp
struct Gearing
```

THE GEARING, as tooth counts (header): `gears(12, 60)` = a 12-tooth gear on the motor drives a 60; `gears(60, 12)` = geared UP; stages multiply, `gears(12, 36) * gears(12, 60)`; `direct` = none. Sprockets the same way (teeth). A bare number does not compile, and names the fix.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:213`](../../include/shulib/mechanism/kinds.hpp#L213).*

<a id="gearing-motorturnsperturn"></a>

### `Gearing::motorTurnsPerTurn`

```cpp
double motorTurnsPerTurn = 0.0
```

Motor turns per output turn (> 0); 0 = UNSET.

*field, declared at [`include/shulib/mechanism/kinds.hpp:215`](../../include/shulib/mechanism/kinds.hpp#L215).*

<a id="gearing-gearing"></a>

### `Gearing::Gearing`

```cpp
constexpr Gearing() noexcept = default
```

UNSET.

*function, declared at [`include/shulib/mechanism/kinds.hpp:217`](../../include/shulib/mechanism/kinds.hpp#L217).*

<a id="gearing-gearing-2"></a>

### `Gearing::Gearing (overload 2)`

```cpp
template <class Write_the_gearing_as_tooth_counts_like_gears_12_60> requires std::is_arithmetic_v<Write_the_gearing_as_tooth_counts_like_gears_12_60> Gearing(Write_the_gearing_as_tooth_counts_like_gears_12_60) = delete
```

UNSET. `.gearing = 5`: refused. The template parameter's NAME is the reason GCC prints ("use of deleted function ... [with Write_the_gearing_as_tooth_counts_like_gears_12_60 = int]") — a consteval refusal inside a converting constructor printed no reason (measured, n41).

*function, declared at [`include/shulib/mechanism/kinds.hpp:222`](../../include/shulib/mechanism/kinds.hpp#L222).*

<a id="gearing-operator-star"></a>

### `Gearing::operator*`

```cpp
[[nodiscard]] constexpr Gearing operator*(Gearing o) const noexcept
```

UNSET. `.gearing = 5`: refused. The template parameter's NAME is the reason GCC prints ("use of deleted function ... [with Write_the_gearing_as_tooth_counts_like_gears_12_60 = int]") — a consteval refusal inside a converting constructor printed no reason (measured, n41). Two stages in series: their reductions multiply.

*function, declared at [`include/shulib/mechanism/kinds.hpp:225`](../../include/shulib/mechanism/kinds.hpp#L225).*

<a id="gears"></a>

## `gears`

```cpp
[[nodiscard]] consteval Gearing gears(int driving, int driven)
```

A gear pair, DRIVING (on the motor's side) then DRIVEN, in teeth. Refuses a gear with no tooth.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:232`](../../include/shulib/mechanism/kinds.hpp#L232).*

<a id="direct"></a>

## `direct`

```cpp
inline constexpr Gearing direct = [] { Gearing g; g.motorTurnsPerTurn = 1.0; return g; }()
```

No gearing: the output turns with the motor.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:241`](../../include/shulib/mechanism/kinds.hpp#L241).*

<a id="struct-gravityterm"></a>

## `struct GravityTerm`

```cpp
struct GravityTerm
```

The holding term for gravity — `volts` is what holds the mechanism still where gravity pulls hardest (an arm held LEVEL; a lift anywhere). Made by cosine(), constantGravity(), noGravity.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:251`](../../include/shulib/mechanism/kinds.hpp#L251).*

<a id="gravityterm-kind"></a>

### `GravityTerm::kind`

```cpp
Gravity kind = static_cast<Gravity>(0)
```

Which shape. UNSET = 0, refused.

*field, declared at [`include/shulib/mechanism/kinds.hpp:253`](../../include/shulib/mechanism/kinds.hpp#L253).*

<a id="gravityterm-volts"></a>

### `GravityTerm::volts`

```cpp
double volts = 0.0
```

The hold, V.

*field, declared at [`include/shulib/mechanism/kinds.hpp:255`](../../include/shulib/mechanism/kinds.hpp#L255).*

<a id="gravityterm-levelatdeg"></a>

### `GravityTerm::levelAtDeg`

```cpp
double levelAtDeg = 0.0
```

Cosine: the output angle (deg) at which the arm is level.

*field, declared at [`include/shulib/mechanism/kinds.hpp:257`](../../include/shulib/mechanism/kinds.hpp#L257).*

<a id="cosine"></a>

## `cosine`

```cpp
[[nodiscard]] constexpr GravityTerm cosine(units::Voltage holdLevel, double levelAt) noexcept
```

An ARM: `holdLevel` holds it out level, less as it points up (cosine); level at `levelAt` degrees.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:260`](../../include/shulib/mechanism/kinds.hpp#L260).*

<a id="constantgravity"></a>

## `constantGravity`

```cpp
[[nodiscard]] constexpr GravityTerm constantGravity(units::Voltage volts) noexcept
```

A LIFT: the same `volts` everywhere along its travel.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:264`](../../include/shulib/mechanism/kinds.hpp#L264).*

<a id="nogravity"></a>

## `noGravity`

```cpp
inline constexpr GravityTerm noGravity{Gravity::None, 0.0, 0.0}
```

Nothing to hold (a turntable, a counterbalanced arm).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:268`](../../include/shulib/mechanism/kinds.hpp#L268).*

<a id="struct-homing"></a>

## `struct Homing`

```cpp
struct Homing
```

How a position mechanism finds its zero — every moment the requirements list: at program start with no motion (restsAtStop, absoluteSensor, a home switch already pressed), or by a homing MOVE (homeBySwitch, homeByStall) at auton start, at driver start or on request, which the auton or the driver file says. Made by the words below.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:274`](../../include/shulib/mechanism/kinds.hpp#L274).*

<a id="homing-method"></a>

### `Homing::method`

```cpp
HomeMethod method = static_cast<HomeMethod>(0)
```

How. UNSET = 0, refused.

*field, declared at [`include/shulib/mechanism/kinds.hpp:276`](../../include/shulib/mechanism/kinds.hpp#L276).*

<a id="homing-at"></a>

### `Homing::at`

```cpp
double at = 0.0
```

Where zero puts it (degrees / inches).

*field, declared at [`include/shulib/mechanism/kinds.hpp:278`](../../include/shulib/mechanism/kinds.hpp#L278).*

<a id="homing-sensoratzerodeg"></a>

### `Homing::sensorAtZeroDeg`

```cpp
double sensorAtZeroDeg = 0.0
```

absoluteSensor only: the sensor's reading (deg) when the mechanism is at `at`.

*field, declared at [`include/shulib/mechanism/kinds.hpp:280`](../../include/shulib/mechanism/kinds.hpp#L280).*

<a id="homing-volts"></a>

### `Homing::volts`

```cpp
double volts = 0.0
```

A homing move's gentle push toward its stop, V, signed (it must point at the home end).

*field, declared at [`include/shulib/mechanism/kinds.hpp:282`](../../include/shulib/mechanism/kinds.hpp#L282).*

<a id="homing-stall"></a>

### `Homing::stall`

```cpp
Stall stall{}
```

homeByStall only: the signature that means "at the stop".

*field, declared at [`include/shulib/mechanism/kinds.hpp:284`](../../include/shulib/mechanism/kinds.hpp#L284).*

<a id="restsatstop"></a>

## `restsAtStop`

```cpp
[[nodiscard]] constexpr Homing restsAtStop(double at) noexcept
```

It RESTS on its stop when the program starts (an arm down on its hard stop): that is `at`.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:287`](../../include/shulib/mechanism/kinds.hpp#L287).*

<a id="restsatstop-2"></a>

## `restsAtStop (overload 2)`

```cpp
[[nodiscard]] constexpr Homing restsAtStop(units::Length at) noexcept
```

The same for a lift: `at` in inches.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:289`](../../include/shulib/mechanism/kinds.hpp#L289).*

<a id="homebyswitch"></a>

## `homeBySwitch`

```cpp
[[nodiscard]] constexpr Homing homeBySwitch(double at, units::Voltage toward) noexcept
```

Push at `toward` until its HOME SWITCH is pressed; that is `at`. Wire it with homeSwitch('B').

*free function, declared at [`include/shulib/mechanism/kinds.hpp:291`](../../include/shulib/mechanism/kinds.hpp#L291).*

<a id="homebyswitch-2"></a>

## `homeBySwitch (overload 2)`

```cpp
[[nodiscard]] constexpr Homing homeBySwitch(units::Length at, units::Voltage toward) noexcept
```

The same for a lift: `at` in inches.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:295`](../../include/shulib/mechanism/kinds.hpp#L295).*

<a id="homebystall"></a>

## `homeByStall`

```cpp
[[nodiscard]] constexpr Homing homeByStall(double at, units::Voltage toward, Stall signature) noexcept
```

Push at `toward` until it STALLS against its stop (the team's `signature`); that is `at`.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:299`](../../include/shulib/mechanism/kinds.hpp#L299).*

<a id="homebystall-2"></a>

## `homeByStall (overload 2)`

```cpp
[[nodiscard]] constexpr Homing homeByStall(units::Length at, units::Voltage toward, Stall signature) noexcept
```

The same for a lift: `at` in inches.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:303`](../../include/shulib/mechanism/kinds.hpp#L303).*

<a id="absolutesensor"></a>

## `absoluteSensor`

```cpp
[[nodiscard]] constexpr Homing absoluteSensor(double at, double sensorReadsDeg) noexcept
```

An angle sensor that knows where it is at power-up: the mechanism is at `at` when the sensor reads `sensorReadsDeg` — a FIXED offset, never "zero wherever it stands" (HA-186: that the V5 rotation sensor's cumulative reading starts at its absolute angle). Needs `.angleSensor`.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:309`](../../include/shulib/mechanism/kinds.hpp#L309).*

<a id="neverhomed"></a>

## `neverHomed`

```cpp
inline constexpr Homing neverHomed{.method = HomeMethod::None}
```

Never homed: position moves are refused (NOT_HOMED); a held button jogs it at `.unhomedVolts`.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:313`](../../include/shulib/mechanism/kinds.hpp#L313).*

<a id="struct-degreerange"></a>

## `struct DegreeRange`

```cpp
struct DegreeRange
```

An Arm's travel limits, degrees. REQUIRED (min < max).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:316`](../../include/shulib/mechanism/kinds.hpp#L316).*

<a id="degreerange-min"></a>

### `DegreeRange::min`

```cpp
double min = 0.0
```

The lower limit.

*field, declared at [`include/shulib/mechanism/kinds.hpp:318`](../../include/shulib/mechanism/kinds.hpp#L318).*

<a id="degreerange-max"></a>

### `DegreeRange::max`

```cpp
double max = 0.0
```

The upper limit.

*field, declared at [`include/shulib/mechanism/kinds.hpp:320`](../../include/shulib/mechanism/kinds.hpp#L320).*

<a id="struct-inchrange"></a>

## `struct InchRange`

```cpp
struct InchRange
```

A Lift's travel limits, inches. REQUIRED (min < max).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:323`](../../include/shulib/mechanism/kinds.hpp#L323).*

<a id="inchrange-min"></a>

### `InchRange::min`

```cpp
units::Length min{0.0}
```

The lower limit.

*field, declared at [`include/shulib/mechanism/kinds.hpp:325`](../../include/shulib/mechanism/kinds.hpp#L325).*

<a id="inchrange-max"></a>

### `InchRange::max`

```cpp
units::Length max{0.0}
```

The upper limit.

*field, declared at [`include/shulib/mechanism/kinds.hpp:327`](../../include/shulib/mechanism/kinds.hpp#L327).*

<a id="struct-degreepreset"></a>

## `struct DegreePreset`

```cpp
struct DegreePreset
```

An Arm's named preset: `{"score", 95}`.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:330`](../../include/shulib/mechanism/kinds.hpp#L330).*

<a id="degreepreset-name"></a>

### `DegreePreset::name`

```cpp
const char* name = nullptr
```

Its name (15 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:332`](../../include/shulib/mechanism/kinds.hpp#L332).*

<a id="degreepreset-value"></a>

### `DegreePreset::value`

```cpp
double value = 0.0
```

Its angle, degrees.

*field, declared at [`include/shulib/mechanism/kinds.hpp:334`](../../include/shulib/mechanism/kinds.hpp#L334).*

<a id="struct-inchpreset"></a>

## `struct InchPreset`

```cpp
struct InchPreset
```

A Lift's named preset: `{"high", 22_in}`.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:337`](../../include/shulib/mechanism/kinds.hpp#L337).*

<a id="inchpreset-name"></a>

### `InchPreset::name`

```cpp
const char* name = nullptr
```

Its name (15 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:339`](../../include/shulib/mechanism/kinds.hpp#L339).*

<a id="inchpreset-value"></a>

### `InchPreset::value`

```cpp
units::Length value{0.0}
```

Its height.

*field, declared at [`include/shulib/mechanism/kinds.hpp:341`](../../include/shulib/mechanism/kinds.hpp#L341).*

<a id="struct-rpmpreset"></a>

## `struct RpmPreset`

```cpp
struct RpmPreset
```

A Flywheel's named speed: `{"shoot", 3000}`.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:344`](../../include/shulib/mechanism/kinds.hpp#L344).*

<a id="rpmpreset-name"></a>

### `RpmPreset::name`

```cpp
const char* name = nullptr
```

Its name (15 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:346`](../../include/shulib/mechanism/kinds.hpp#L346).*

<a id="rpmpreset-rpm"></a>

### `RpmPreset::rpm`

```cpp
double rpm = 0.0
```

Its output rpm.

*field, declared at [`include/shulib/mechanism/kinds.hpp:348`](../../include/shulib/mechanism/kinds.hpp#L348).*

<a id="struct-liftshape"></a>

## `struct LiftShape`

```cpp
struct LiftShape
```

A lift's SHAPE: how its inches relate to its output shaft's degrees. Made by spool(), linkage(), measured().

*struct, declared at [`include/shulib/mechanism/kinds.hpp:353`](../../include/shulib/mechanism/kinds.hpp#L353).*

<a id="liftshape-geometry"></a>

### `LiftShape::geometry`

```cpp
Geometry geometry = static_cast<Geometry>(0)
```

Which shape. UNSET = 0, refused.

*field, declared at [`include/shulib/mechanism/kinds.hpp:355`](../../include/shulib/mechanism/kinds.hpp#L355).*

<a id="liftshape-spooldiameterin"></a>

### `LiftShape::spoolDiameterIn`

```cpp
double spoolDiameterIn = 0.0
```

spool(): its diameter, inches.

*field, declared at [`include/shulib/mechanism/kinds.hpp:357`](../../include/shulib/mechanism/kinds.hpp#L357).*

<a id="liftshape-armlengthin"></a>

### `LiftShape::armLengthIn`

```cpp
double armLengthIn = 0.0
```

linkage(): one stage's arm length, inches.

*field, declared at [`include/shulib/mechanism/kinds.hpp:359`](../../include/shulib/mechanism/kinds.hpp#L359).*

<a id="liftshape-pivotheightin"></a>

### `LiftShape::pivotHeightIn`

```cpp
double pivotHeightIn = 0.0
```

linkage(): the first pivot's height, inches.

*field, declared at [`include/shulib/mechanism/kinds.hpp:361`](../../include/shulib/mechanism/kinds.hpp#L361).*

<a id="liftshape-stages"></a>

### `LiftShape::stages`

```cpp
std::uint8_t stages = 0
```

linkage(): 1 = a four-bar, 2 = a double reverse four-bar.

*field, declared at [`include/shulib/mechanism/kinds.hpp:363`](../../include/shulib/mechanism/kinds.hpp#L363).*

<a id="liftshape-table"></a>

### `LiftShape::table`

```cpp
double table[kMaxTablePoints][2]{}
```

measured(): {height in, output angle deg} rows.

*field, declared at [`include/shulib/mechanism/kinds.hpp:365`](../../include/shulib/mechanism/kinds.hpp#L365).*

<a id="liftshape-points"></a>

### `LiftShape::points`

```cpp
std::uint8_t points = 0
```

measured(): how many rows were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:367`](../../include/shulib/mechanism/kinds.hpp#L367).*

<a id="spool"></a>

## `spool`

```cpp
[[nodiscard]] constexpr LiftShape spool(units::Length diameter) noexcept
```

A string or chain on a spool of this diameter on the output shaft.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:370`](../../include/shulib/mechanism/kinds.hpp#L370).*

<a id="linkage"></a>

## `linkage`

```cpp
[[nodiscard]] constexpr LiftShape linkage(units::Length armLength, units::Length pivotHeight, int stages) noexcept
```

A four-bar (1 stage) or a double reverse four-bar (2 stages): each stage `armLength` long, the first pivot `pivotHeight` up; its height follows the sine of the arm's angle from level.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:375`](../../include/shulib/mechanism/kinds.hpp#L375).*

<a id="struct-heightat"></a>

## `struct HeightAt`

```cpp
struct HeightAt
```

One measured pair of a lift's table: its height at an output angle.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:380`](../../include/shulib/mechanism/kinds.hpp#L380).*

<a id="heightat-height"></a>

### `HeightAt::height`

```cpp
units::Length height{0.0}
```

The lift's height there.

*field, declared at [`include/shulib/mechanism/kinds.hpp:382`](../../include/shulib/mechanism/kinds.hpp#L382).*

<a id="heightat-degrees"></a>

### `HeightAt::degrees`

```cpp
double degrees = 0.0
```

The output shaft's angle there, degrees.

*field, declared at [`include/shulib/mechanism/kinds.hpp:384`](../../include/shulib/mechanism/kinds.hpp#L384).*

<a id="measured"></a>

## `measured`

```cpp
template <std::size_t N> [[nodiscard]] constexpr LiftShape measured(const HeightAt (&pairs)[N]) noexcept
```

Any other shape: measured pairs {height, output angle}, at least two, strictly increasing — interpolated between them. More than kMaxTablePoints pairs is recorded and refused by the list.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:389`](../../include/shulib/mechanism/kinds.hpp#L389).*

<a id="struct-positiontuning"></a>

## `struct PositionTuning`

```cpp
struct PositionTuning
```

A position mechanism's overrides of the library's STARTING values (header; N7), one by one — 0 keeps the starting value. Gains per OUTPUT-SHAFT degree; speed and acceleration in the mechanism's units per second.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:402`](../../include/shulib/mechanism/kinds.hpp#L402).*

<a id="positiontuning-kp"></a>

### `PositionTuning::kP`

```cpp
double kP = 0.0
```

Proportional gain, V/deg (starting value 0.25).

*field, declared at [`include/shulib/mechanism/kinds.hpp:404`](../../include/shulib/mechanism/kinds.hpp#L404).*

<a id="positiontuning-ki"></a>

### `PositionTuning::kI`

```cpp
double kI = 0.0
```

The hold loop's integral gain, V/(deg*s) (starting value 0.3).

*field, declared at [`include/shulib/mechanism/kinds.hpp:406`](../../include/shulib/mechanism/kinds.hpp#L406).*

<a id="positiontuning-kd"></a>

### `PositionTuning::kD`

```cpp
double kD = 0.0
```

Derivative gain, V/(deg/s) (starting value 0.004).

*field, declared at [`include/shulib/mechanism/kinds.hpp:408`](../../include/shulib/mechanism/kinds.hpp#L408).*

<a id="positiontuning-ks"></a>

### `PositionTuning::kS`

```cpp
double kS = 0.0
```

Static feedforward, V (starting value 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:410`](../../include/shulib/mechanism/kinds.hpp#L410).*

<a id="positiontuning-kv"></a>

### `PositionTuning::kV`

```cpp
double kV = 0.0
```

Velocity feedforward, V/(deg/s) (starting value 12 V over the output's free speed).

*field, declared at [`include/shulib/mechanism/kinds.hpp:412`](../../include/shulib/mechanism/kinds.hpp#L412).*

<a id="positiontuning-ka"></a>

### `PositionTuning::kA`

```cpp
double kA = 0.0
```

Acceleration feedforward, V/(deg/s^2) (starting value 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:414`](../../include/shulib/mechanism/kinds.hpp#L414).*

<a id="positiontuning-maxspeed"></a>

### `PositionTuning::maxSpeed`

```cpp
double maxSpeed = 0.0
```

The smooth move's top speed, units/s (starting value 75 % of the free speed).

*field, declared at [`include/shulib/mechanism/kinds.hpp:416`](../../include/shulib/mechanism/kinds.hpp#L416).*

<a id="positiontuning-maxacceleration"></a>

### `PositionTuning::maxAcceleration`

```cpp
double maxAcceleration = 0.0
```

Its acceleration, units/s^2 (starting value: the top speed in 0.25 s).

*field, declared at [`include/shulib/mechanism/kinds.hpp:418`](../../include/shulib/mechanism/kinds.hpp#L418).*

<a id="positiontuning-within"></a>

### `PositionTuning::within`

```cpp
double within = 0.0
```

"Reached" when within this of the target (starting value 1.5 deg, or 0.25 in) ...

*field, declared at [`include/shulib/mechanism/kinds.hpp:420`](../../include/shulib/mechanism/kinds.hpp#L420).*

<a id="positiontuning-settle"></a>

### `PositionTuning::settle`

```cpp
units::Time settle{0.0}
```

... for this long (starting value 0.1 s).

*field, declared at [`include/shulib/mechanism/kinds.hpp:422`](../../include/shulib/mechanism/kinds.hpp#L422).*

<a id="struct-speedtuning"></a>

## `struct SpeedTuning`

```cpp
struct SpeedTuning
```

A flywheel's overrides of its starting values — 0 keeps the starting value.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:425`](../../include/shulib/mechanism/kinds.hpp#L425).*

<a id="speedtuning-ks"></a>

### `SpeedTuning::kS`

```cpp
double kS = 0.0
```

Static feedforward, V (starting value 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:427`](../../include/shulib/mechanism/kinds.hpp#L427).*

<a id="speedtuning-kv"></a>

### `SpeedTuning::kV`

```cpp
double kV = 0.0
```

Velocity feedforward, V per output rad/s (starting value 12 V over the free speed).

*field, declared at [`include/shulib/mechanism/kinds.hpp:429`](../../include/shulib/mechanism/kinds.hpp#L429).*

<a id="speedtuning-kp"></a>

### `SpeedTuning::kP`

```cpp
double kP = 0.0
```

Proportional gain, V per rad/s (starting value 0.05).

*field, declared at [`include/shulib/mechanism/kinds.hpp:431`](../../include/shulib/mechanism/kinds.hpp#L431).*

<a id="speedtuning-ki"></a>

### `SpeedTuning::kI`

```cpp
double kI = 0.0
```

Integral gain, V per rad (starting value 0.2).

*field, declared at [`include/shulib/mechanism/kinds.hpp:433`](../../include/shulib/mechanism/kinds.hpp#L433).*

<a id="struct-positiongoal"></a>

## `struct PositionGoal`

```cpp
struct PositionGoal
```

A position goal: an Arm's or a Lift's preset or constant target.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:443`](../../include/shulib/mechanism/kinds.hpp#L443).*

<a id="positiongoal-mechanism"></a>

### `PositionGoal::mechanism`

```cpp
const void* mechanism
```

The declaration's address (what a driver control's build check compares).

*field, declared at [`include/shulib/mechanism/kinds.hpp:445`](../../include/shulib/mechanism/kinds.hpp#L445).*

<a id="positiongoal-mechanismname"></a>

### `PositionGoal::mechanismName`

```cpp
const char* mechanismName
```

The declaration's `.name` (what the run-time lookup compares).

*field, declared at [`include/shulib/mechanism/kinds.hpp:447`](../../include/shulib/mechanism/kinds.hpp#L447).*

<a id="positiongoal-presetname"></a>

### `PositionGoal::presetName`

```cpp
const char* presetName
```

The preset's name, or nullptr for a number.

*field, declared at [`include/shulib/mechanism/kinds.hpp:449`](../../include/shulib/mechanism/kinds.hpp#L449).*

<a id="positiongoal-value"></a>

### `PositionGoal::value`

```cpp
double value
```

The target, degrees or inches.

*field, declared at [`include/shulib/mechanism/kinds.hpp:451`](../../include/shulib/mechanism/kinds.hpp#L451).*

<a id="positiongoal-computed"></a>

### `PositionGoal::computed`

```cpp
bool computed
```

A run-time number (`clampedTo`): clamped at the limits by the soft stop, logged.

*field, declared at [`include/shulib/mechanism/kinds.hpp:453`](../../include/shulib/mechanism/kinds.hpp#L453).*

<a id="struct-speedgoal"></a>

## `struct SpeedGoal`

```cpp
struct SpeedGoal
```

A speed goal: a Flywheel's named speed or constant target.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:456`](../../include/shulib/mechanism/kinds.hpp#L456).*

<a id="speedgoal-mechanism"></a>

### `SpeedGoal::mechanism`

```cpp
const void* mechanism
```

The declaration's address.

*field, declared at [`include/shulib/mechanism/kinds.hpp:458`](../../include/shulib/mechanism/kinds.hpp#L458).*

<a id="speedgoal-mechanismname"></a>

### `SpeedGoal::mechanismName`

```cpp
const char* mechanismName
```

The declaration's `.name`.

*field, declared at [`include/shulib/mechanism/kinds.hpp:460`](../../include/shulib/mechanism/kinds.hpp#L460).*

<a id="speedgoal-presetname"></a>

### `SpeedGoal::presetName`

```cpp
const char* presetName
```

The preset's name, or nullptr for a number.

*field, declared at [`include/shulib/mechanism/kinds.hpp:462`](../../include/shulib/mechanism/kinds.hpp#L462).*

<a id="speedgoal-rpm"></a>

### `SpeedGoal::rpm`

```cpp
double rpm
```

The target, output rpm.

*field, declared at [`include/shulib/mechanism/kinds.hpp:464`](../../include/shulib/mechanism/kinds.hpp#L464).*

<a id="speedgoal-computed"></a>

### `SpeedGoal::computed`

```cpp
bool computed
```

A run-time number (`clampedTo`).

*field, declared at [`include/shulib/mechanism/kinds.hpp:466`](../../include/shulib/mechanism/kinds.hpp#L466).*

<a id="struct-pneumaticgoal"></a>

## `struct PneumaticGoal`

```cpp
struct PneumaticGoal
```

A pneumatic goal: one of its named positions.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:469`](../../include/shulib/mechanism/kinds.hpp#L469).*

<a id="pneumaticgoal-mechanism"></a>

### `PneumaticGoal::mechanism`

```cpp
const void* mechanism
```

The declaration's address.

*field, declared at [`include/shulib/mechanism/kinds.hpp:471`](../../include/shulib/mechanism/kinds.hpp#L471).*

<a id="pneumaticgoal-mechanismname"></a>

### `PneumaticGoal::mechanismName`

```cpp
const char* mechanismName
```

The declaration's `.name`.

*field, declared at [`include/shulib/mechanism/kinds.hpp:473`](../../include/shulib/mechanism/kinds.hpp#L473).*

<a id="pneumaticgoal-position"></a>

### `PneumaticGoal::position`

```cpp
std::uint8_t position
```

The position's index in the declaration.

*field, declared at [`include/shulib/mechanism/kinds.hpp:475`](../../include/shulib/mechanism/kinds.hpp#L475).*

<a id="pneumaticgoal-positionname"></a>

### `PneumaticGoal::positionName`

```cpp
const char* positionName
```

The position's name (the lookup checks it at its index).

*field, declared at [`include/shulib/mechanism/kinds.hpp:477`](../../include/shulib/mechanism/kinds.hpp#L477).*

<a id="struct-power"></a>

## `struct Power`

```cpp
struct Power
```

POWER — run at a voltage (intakes, rollers, conveyors, robot two's claw pivots). Members in the order C++ requires them written (header).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:616`](../../include/shulib/mechanism/kinds.hpp#L616).*

<a id="power-name"></a>

### `Power::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:618`](../../include/shulib/mechanism/kinds.hpp#L618).*

<a id="power-motors"></a>

### `Power::motors`

```cpp
Motors motors{}
```

Its motor roles.

*field, declared at [`include/shulib/mechanism/kinds.hpp:620`](../../include/shulib/mechanism/kinds.hpp#L620).*

<a id="power-cartridge"></a>

### `Power::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

Its motors' cartridge. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:622`](../../include/shulib/mechanism/kinds.hpp#L622).*

<a id="power-piecesensor"></a>

### `Power::pieceSensor`

```cpp
Sensor pieceSensor{}
```

Optional: distanceSensor(2_in), opticalSensor(0.5), limitSwitch().

*field, declared at [`include/shulib/mechanism/kinds.hpp:624`](../../include/shulib/mechanism/kinds.hpp#L624).*

<a id="power-safe"></a>

### `Power::safe`

```cpp
config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0)
```

What every stop lands in: Coast, Brake or Hold. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:626`](../../include/shulib/mechanism/kinds.hpp#L626).*

<a id="power-stall"></a>

### `Power::stall`

```cpp
Stall stall{}
```

Optional: the jam signature (`stalls`, the STALLED verdict); with none, the boot says a held button is cut only by its release (HA-154).

*field, declared at [`include/shulib/mechanism/kinds.hpp:629`](../../include/shulib/mechanism/kinds.hpp#L629).*

<a id="power-maxvolts"></a>

### `Power::maxVolts`

```cpp
units::Voltage maxVolts{12.0}
```

The most any command may ask (the motor's own ceiling unless lowered), (0, 12] V.

*field, declared at [`include/shulib/mechanism/kinds.hpp:631`](../../include/shulib/mechanism/kinds.hpp#L631).*

<a id="power-operator-index"></a>

### `Power::operator[]`

```cpp
template <class T> [[nodiscard]] consteval int operator[](T) const
```

`intake["fast"]`: refused BY NAME — a power mechanism has no presets (run it: `r.run(intake, 12_volt, ...)`).

*function, declared at [`include/shulib/mechanism/kinds.hpp:636`](../../include/shulib/mechanism/kinds.hpp#L636).*

<a id="power-spec"></a>

### `Power::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec() const noexcept
```

This declaration as data.

*function, declared at [`include/shulib/mechanism/kinds.hpp:642`](../../include/shulib/mechanism/kinds.hpp#L642).*

<a id="struct-arm"></a>

## `struct Arm`

```cpp
struct Arm
```

ARM — go to an angle in DEGREES and hold it (arms, tilters, a claw pivot with presets).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:654`](../../include/shulib/mechanism/kinds.hpp#L654).*

<a id="arm-name"></a>

### `Arm::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:656`](../../include/shulib/mechanism/kinds.hpp#L656).*

<a id="arm-motors"></a>

### `Arm::motors`

```cpp
Motors motors{}
```

Its motor roles.

*field, declared at [`include/shulib/mechanism/kinds.hpp:658`](../../include/shulib/mechanism/kinds.hpp#L658).*

<a id="arm-cartridge"></a>

### `Arm::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

Its motors' cartridge. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:660`](../../include/shulib/mechanism/kinds.hpp#L660).*

<a id="arm-gearing"></a>

### `Arm::gearing`

```cpp
Gearing gearing{}
```

REQUIRED: gears(12, 60) — tooth counts, the motor's gear first.

*field, declared at [`include/shulib/mechanism/kinds.hpp:662`](../../include/shulib/mechanism/kinds.hpp#L662).*

<a id="arm-anglesensor"></a>

### `Arm::angleSensor`

```cpp
Sensor angleSensor{}
```

Optional: rotationSensor() — then it, not the motor encoders, is the angle.

*field, declared at [`include/shulib/mechanism/kinds.hpp:664`](../../include/shulib/mechanism/kinds.hpp#L664).*

<a id="arm-safe"></a>

### `Arm::safe`

```cpp
config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0)
```

What every stop lands in. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:666`](../../include/shulib/mechanism/kinds.hpp#L666).*

<a id="arm-stall"></a>

### `Arm::stall`

```cpp
Stall stall{}
```

REQUIRED: a blocked move's or hold's signature (P4, N10).

*field, declared at [`include/shulib/mechanism/kinds.hpp:668`](../../include/shulib/mechanism/kinds.hpp#L668).*

<a id="arm-range"></a>

### `Arm::range`

```cpp
DegreeRange range{}
```

The travel limits, degrees. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:670`](../../include/shulib/mechanism/kinds.hpp#L670).*

<a id="arm-gravity"></a>

### `Arm::gravity`

```cpp
GravityTerm gravity{}
```

cosine(...) or noGravity. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:672`](../../include/shulib/mechanism/kinds.hpp#L672).*

<a id="arm-home"></a>

### `Arm::home`

```cpp
Homing home{}
```

restsAtStop / homeBySwitch / homeByStall / absoluteSensor / neverHomed. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:674`](../../include/shulib/mechanism/kinds.hpp#L674).*

<a id="arm-presets"></a>

### `Arm::presets`

```cpp
DegreePreset presets[kMaxPresets]{}
```

Optional: {"score", 95}, ... (six at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:676`](../../include/shulib/mechanism/kinds.hpp#L676).*

<a id="arm-unhomedvolts"></a>

### `Arm::unhomedVolts`

```cpp
units::Voltage unhomedVolts{0.0}
```

Optional: a held button's most before homing (0 = nothing moves it before homing).

*field, declared at [`include/shulib/mechanism/kinds.hpp:678`](../../include/shulib/mechanism/kinds.hpp#L678).*

<a id="arm-tuning"></a>

### `Arm::tuning`

```cpp
PositionTuning tuning{}
```

Optional: overrides of the starting values (header).

*field, declared at [`include/shulib/mechanism/kinds.hpp:680`](../../include/shulib/mechanism/kinds.hpp#L680).*

<a id="arm-operator-index"></a>

### `Arm::operator[]`

```cpp
[[nodiscard]] consteval PositionGoal operator[](const char* preset) const
```

A named preset: `arm["score"]`. A name the arm does not have FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:683`](../../include/shulib/mechanism/kinds.hpp#L683).*

<a id="arm-to"></a>

### `Arm::to`

```cpp
[[nodiscard]] consteval PositionGoal to(double degrees) const
```

A constant target in degrees: `arm.to(120)`. Past the travel limits FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:693`](../../include/shulib/mechanism/kinds.hpp#L693).*

<a id="arm-requires"></a>

### `Arm::requires`

```cpp
template <class T> requires(!std::is_arithmetic_v<T>) [[nodiscard]] consteval PositionGoal to(const T&) const
```

`arm.to(90_deg)`: refused BY NAME (`_deg` is a wrapping heading — 270 reads -90).

*function, declared at [`include/shulib/mechanism/kinds.hpp:701`](../../include/shulib/mechanism/kinds.hpp#L701).*

<a id="arm-clampedto"></a>

### `Arm::clampedTo`

```cpp
[[nodiscard]] constexpr PositionGoal clampedTo(double degrees) const noexcept
```

A target computed while the auton runs: clamped to the limits by the soft stop, and logged.

*function, declared at [`include/shulib/mechanism/kinds.hpp:707`](../../include/shulib/mechanism/kinds.hpp#L707).*

<a id="arm-spec"></a>

### `Arm::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec() const noexcept
```

This declaration as data.

*function, declared at [`include/shulib/mechanism/kinds.hpp:712`](../../include/shulib/mechanism/kinds.hpp#L712).*

<a id="struct-lift"></a>

## `struct Lift`

```cpp
struct Lift
```

LIFT — go to a height in INCHES and hold it (a spool lift, a four-bar, a DR4B, any measured shape). Members in the Arm's order, its shape after the gearing.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:748`](../../include/shulib/mechanism/kinds.hpp#L748).*

<a id="lift-name"></a>

### `Lift::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:750`](../../include/shulib/mechanism/kinds.hpp#L750).*

<a id="lift-motors"></a>

### `Lift::motors`

```cpp
Motors motors{}
```

Its motor roles.

*field, declared at [`include/shulib/mechanism/kinds.hpp:752`](../../include/shulib/mechanism/kinds.hpp#L752).*

<a id="lift-cartridge"></a>

### `Lift::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

Its motors' cartridge. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:754`](../../include/shulib/mechanism/kinds.hpp#L754).*

<a id="lift-gearing"></a>

### `Lift::gearing`

```cpp
Gearing gearing{}
```

REQUIRED: gears(12, 36), or direct.

*field, declared at [`include/shulib/mechanism/kinds.hpp:756`](../../include/shulib/mechanism/kinds.hpp#L756).*

<a id="lift-shape"></a>

### `Lift::shape`

```cpp
LiftShape shape{}
```

REQUIRED: spool(...), linkage(...) or measured({...}).

*field, declared at [`include/shulib/mechanism/kinds.hpp:758`](../../include/shulib/mechanism/kinds.hpp#L758).*

<a id="lift-anglesensor"></a>

### `Lift::angleSensor`

```cpp
Sensor angleSensor{}
```

Optional: rotationSensor() on the output shaft.

*field, declared at [`include/shulib/mechanism/kinds.hpp:760`](../../include/shulib/mechanism/kinds.hpp#L760).*

<a id="lift-safe"></a>

### `Lift::safe`

```cpp
config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0)
```

What every stop lands in. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:762`](../../include/shulib/mechanism/kinds.hpp#L762).*

<a id="lift-stall"></a>

### `Lift::stall`

```cpp
Stall stall{}
```

REQUIRED (P4, N10).

*field, declared at [`include/shulib/mechanism/kinds.hpp:764`](../../include/shulib/mechanism/kinds.hpp#L764).*

<a id="lift-range"></a>

### `Lift::range`

```cpp
InchRange range{}
```

The travel limits, inches. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:766`](../../include/shulib/mechanism/kinds.hpp#L766).*

<a id="lift-gravity"></a>

### `Lift::gravity`

```cpp
GravityTerm gravity{}
```

constantGravity(...) (or cosine for a linkage's arm, or noGravity). REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:768`](../../include/shulib/mechanism/kinds.hpp#L768).*

<a id="lift-home"></a>

### `Lift::home`

```cpp
Homing home{}
```

How it finds zero. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:770`](../../include/shulib/mechanism/kinds.hpp#L770).*

<a id="lift-presets"></a>

### `Lift::presets`

```cpp
InchPreset presets[kMaxPresets]{}
```

Optional: {"high", 22_in}, ... (six at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:772`](../../include/shulib/mechanism/kinds.hpp#L772).*

<a id="lift-unhomedvolts"></a>

### `Lift::unhomedVolts`

```cpp
units::Voltage unhomedVolts{0.0}
```

Optional: a held button's most before homing.

*field, declared at [`include/shulib/mechanism/kinds.hpp:774`](../../include/shulib/mechanism/kinds.hpp#L774).*

<a id="lift-tuning"></a>

### `Lift::tuning`

```cpp
PositionTuning tuning{}
```

Optional: overrides of the starting values.

*field, declared at [`include/shulib/mechanism/kinds.hpp:776`](../../include/shulib/mechanism/kinds.hpp#L776).*

<a id="lift-operator-index"></a>

### `Lift::operator[]`

```cpp
[[nodiscard]] consteval PositionGoal operator[](const char* preset) const
```

A named preset: `lift["high"]`. A name the lift does not have FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:779`](../../include/shulib/mechanism/kinds.hpp#L779).*

<a id="lift-to"></a>

### `Lift::to`

```cpp
[[nodiscard]] consteval PositionGoal to(units::Length height) const
```

A constant target height: `lift.to(18_in)`. Past the travel limits FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:789`](../../include/shulib/mechanism/kinds.hpp#L789).*

<a id="lift-to-2"></a>

### `Lift::to (overload 2)`

```cpp
template <class T> requires std::is_arithmetic_v<T> [[nodiscard]] consteval PositionGoal to(T) const
```

`lift.to(18)`: refused BY NAME (a height is inches: `lift.to(18_in)`).

*function, declared at [`include/shulib/mechanism/kinds.hpp:797`](../../include/shulib/mechanism/kinds.hpp#L797).*

<a id="lift-clampedto"></a>

### `Lift::clampedTo`

```cpp
[[nodiscard]] constexpr PositionGoal clampedTo(units::Length height) const noexcept
```

A height computed while the auton runs: clamped to the limits by the soft stop, and logged.

*function, declared at [`include/shulib/mechanism/kinds.hpp:803`](../../include/shulib/mechanism/kinds.hpp#L803).*

<a id="lift-spec"></a>

### `Lift::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec() const noexcept
```

This declaration as data.

*function, declared at [`include/shulib/mechanism/kinds.hpp:808`](../../include/shulib/mechanism/kinds.hpp#L808).*

<a id="struct-atspeed"></a>

## `struct AtSpeed`

```cpp
struct AtSpeed
```

"At speed": within `rpm` of the target for `lasting` (both the team's, P6). Made by within().

*struct, declared at [`include/shulib/mechanism/kinds.hpp:864`](../../include/shulib/mechanism/kinds.hpp#L864).*

<a id="atspeed-rpm"></a>

### `AtSpeed::rpm`

```cpp
double rpm = 0.0
```

The band, output rpm.

*field, declared at [`include/shulib/mechanism/kinds.hpp:866`](../../include/shulib/mechanism/kinds.hpp#L866).*

<a id="atspeed-lasting"></a>

### `AtSpeed::lasting`

```cpp
units::Time lasting{0.0}
```

How long it must stay inside it.

*field, declared at [`include/shulib/mechanism/kinds.hpp:868`](../../include/shulib/mechanism/kinds.hpp#L868).*

<a id="within"></a>

## `within`

```cpp
[[nodiscard]] constexpr AtSpeed within(double rpm, units::Time lasting) noexcept
```

`.atSpeed = within(60, 0.1_s)`: within 60 rpm of the target for 0.1 s.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:871`](../../include/shulib/mechanism/kinds.hpp#L871).*

<a id="struct-flywheel"></a>

## `struct Flywheel`

```cpp
struct Flywheel
```

FLYWHEEL — spin at an RPM of its output and say when it is up to speed (the SPEED kind; re-check N11: the declaration's word is what a team builds).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:875`](../../include/shulib/mechanism/kinds.hpp#L875).*

<a id="flywheel-name"></a>

### `Flywheel::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:877`](../../include/shulib/mechanism/kinds.hpp#L877).*

<a id="flywheel-motors"></a>

### `Flywheel::motors`

```cpp
Motors motors{}
```

Its motor roles.

*field, declared at [`include/shulib/mechanism/kinds.hpp:879`](../../include/shulib/mechanism/kinds.hpp#L879).*

<a id="flywheel-cartridge"></a>

### `Flywheel::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

Its motors' cartridge. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:881`](../../include/shulib/mechanism/kinds.hpp#L881).*

<a id="flywheel-gearing"></a>

### `Flywheel::gearing`

```cpp
Gearing gearing{}
```

REQUIRED: gears(72, 12) = geared UP 6:1.

*field, declared at [`include/shulib/mechanism/kinds.hpp:883`](../../include/shulib/mechanism/kinds.hpp#L883).*

<a id="flywheel-anglesensor"></a>

### `Flywheel::angleSensor`

```cpp
Sensor angleSensor{}
```

Optional: rotationSensor() on the wheel — then it, not the motor encoders, is the speed.

*field, declared at [`include/shulib/mechanism/kinds.hpp:885`](../../include/shulib/mechanism/kinds.hpp#L885).*

<a id="flywheel-safe"></a>

### `Flywheel::safe`

```cpp
config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0)
```

What every stop lands in. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:887`](../../include/shulib/mechanism/kinds.hpp#L887).*

<a id="flywheel-stall"></a>

### `Flywheel::stall`

```cpp
Stall stall{}
```

REQUIRED: a jammed wheel's signature (P4, N10).

*field, declared at [`include/shulib/mechanism/kinds.hpp:889`](../../include/shulib/mechanism/kinds.hpp#L889).*

<a id="flywheel-atspeed"></a>

### `Flywheel::atSpeed`

```cpp
AtSpeed atSpeed{}
```

REQUIRED: within(60, 0.1_s).

*field, declared at [`include/shulib/mechanism/kinds.hpp:891`](../../include/shulib/mechanism/kinds.hpp#L891).*

<a id="flywheel-presets"></a>

### `Flywheel::presets`

```cpp
RpmPreset presets[kMaxPresets]{}
```

Its named speeds: {"shoot", 3000}, ... (six at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:893`](../../include/shulib/mechanism/kinds.hpp#L893).*

<a id="flywheel-tuning"></a>

### `Flywheel::tuning`

```cpp
SpeedTuning tuning{}
```

Optional: overrides of the starting values.

*field, declared at [`include/shulib/mechanism/kinds.hpp:895`](../../include/shulib/mechanism/kinds.hpp#L895).*

<a id="flywheel-freerpm"></a>

### `Flywheel::freeRpm`

```cpp
[[nodiscard]] constexpr double freeRpm() const noexcept
```

The fastest it can spin: the cartridge's free speed through the gearing, output rpm.

*function, declared at [`include/shulib/mechanism/kinds.hpp:898`](../../include/shulib/mechanism/kinds.hpp#L898).*

<a id="flywheel-operator-index"></a>

### `Flywheel::operator[]`

```cpp
[[nodiscard]] consteval SpeedGoal operator[](const char* preset) const
```

A named speed: `flywheel["shoot"]`. A name it does not have FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:902`](../../include/shulib/mechanism/kinds.hpp#L902).*

<a id="flywheel-at"></a>

### `Flywheel::at`

```cpp
[[nodiscard]] consteval SpeedGoal at(double rpm) const
```

A constant speed: `flywheel.at(3000)`. Past its free speed FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:912`](../../include/shulib/mechanism/kinds.hpp#L912).*

<a id="flywheel-clampedto"></a>

### `Flywheel::clampedTo`

```cpp
[[nodiscard]] constexpr SpeedGoal clampedTo(double rpm) const noexcept
```

A speed computed while the auton runs: clamped to the free speed, and logged.

*function, declared at [`include/shulib/mechanism/kinds.hpp:919`](../../include/shulib/mechanism/kinds.hpp#L919).*

<a id="flywheel-spec"></a>

### `Flywheel::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec() const noexcept
```

This declaration as data (its free speed derived, its starting values filled).

*function, declared at [`include/shulib/mechanism/kinds.hpp:924`](../../include/shulib/mechanism/kinds.hpp#L924).*

<a id="struct-levels"></a>

## `struct Levels`

```cpp
struct Levels
```

One position's line levels, as written: `{Low}` (one line) or `{High, Low}` (two). Records HOW MANY were written, so the wiring can refuse a position stating a different number of levels than it has lines (U2: a bool array cannot tell an unstated level from Low).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:956`](../../include/shulib/mechanism/kinds.hpp#L956).*

<a id="levels-l"></a>

### `Levels::l`

```cpp
bool l[kMaxLineRoles]{}
```

The levels, line 1 first.

*field, declared at [`include/shulib/mechanism/kinds.hpp:958`](../../include/shulib/mechanism/kinds.hpp#L958).*

<a id="levels-count"></a>

### `Levels::count`

```cpp
std::uint8_t count = 0
```

How many were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:960`](../../include/shulib/mechanism/kinds.hpp#L960).*

<a id="levels-toomany"></a>

### `Levels::tooMany`

```cpp
bool tooMany = false
```

More than kMaxLineRoles were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:962`](../../include/shulib/mechanism/kinds.hpp#L962).*

<a id="levels-levels"></a>

### `Levels::Levels`

```cpp
constexpr Levels() noexcept = default
```

UNSET: no levels.

*function, declared at [`include/shulib/mechanism/kinds.hpp:964`](../../include/shulib/mechanism/kinds.hpp#L964).*

<a id="levels-levels-2"></a>

### `Levels::Levels (overload 2)`

```cpp
consteval Levels(std::initializer_list<bool> list) noexcept
```

UNSET: no levels. `{High, Low}`.

*function, declared at [`include/shulib/mechanism/kinds.hpp:966`](../../include/shulib/mechanism/kinds.hpp#L966).*

<a id="struct-namedposition"></a>

## `struct NamedPosition`

```cpp
struct NamedPosition
```

A pneumatic POSITION: its name, each line's level in it, and whether reaching it is a GRAB (or a RELEASE) a piece sensor confirms (Q2: with no sensor it is counted done after `moveTime`, reported UNCONFIRMED).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:979`](../../include/shulib/mechanism/kinds.hpp#L979).*

<a id="namedposition-name"></a>

### `NamedPosition::name`

```cpp
const char* name = nullptr
```

Its name (15 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:981`](../../include/shulib/mechanism/kinds.hpp#L981).*

<a id="namedposition-levels"></a>

### `NamedPosition::levels`

```cpp
Levels levels{}
```

Its line levels.

*field, declared at [`include/shulib/mechanism/kinds.hpp:983`](../../include/shulib/mechanism/kinds.hpp#L983).*

<a id="namedposition-confirm"></a>

### `NamedPosition::confirm`

```cpp
Confirm confirm = Confirm::None
```

`grabs`, `releases`, or nothing.

*field, declared at [`include/shulib/mechanism/kinds.hpp:985`](../../include/shulib/mechanism/kinds.hpp#L985).*

<a id="struct-positions"></a>

## `struct Positions`

```cpp
struct Positions
```

A pneumatic mechanism's positions, in order (two or three): `.positions = {{"closed", {Low}, grabs}, {"open", {High}}}`.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:989`](../../include/shulib/mechanism/kinds.hpp#L989).*

<a id="positions-p"></a>

### `Positions::p`

```cpp
NamedPosition p[kMaxPneumaticPositions]{}
```

The positions, in the order written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:991`](../../include/shulib/mechanism/kinds.hpp#L991).*

<a id="positions-count"></a>

### `Positions::count`

```cpp
std::uint8_t count = 0
```

How many were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:993`](../../include/shulib/mechanism/kinds.hpp#L993).*

<a id="positions-toomany"></a>

### `Positions::tooMany`

```cpp
bool tooMany = false
```

More than kMaxPneumaticPositions were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:995`](../../include/shulib/mechanism/kinds.hpp#L995).*

<a id="positions-file"></a>

### `Positions::file`

```cpp
const char* file = nullptr
```

Where the list was written (an initializer list — see Motors::file; the spec uses `.name`'s).

*field, declared at [`include/shulib/mechanism/kinds.hpp:997`](../../include/shulib/mechanism/kinds.hpp#L997).*

<a id="positions-line"></a>

### `Positions::line`

```cpp
std::uint32_t line = 0
```

That line.

*field, declared at [`include/shulib/mechanism/kinds.hpp:999`](../../include/shulib/mechanism/kinds.hpp#L999).*

<a id="positions-positions"></a>

### `Positions::Positions`

```cpp
constexpr Positions() noexcept = default
```

UNSET: no positions.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1001`](../../include/shulib/mechanism/kinds.hpp#L1001).*

<a id="positions-positions-2"></a>

### `Positions::Positions (overload 2)`

```cpp
consteval Positions(std::initializer_list<NamedPosition> list, std::source_location at = std::source_location::current()) noexcept
```

UNSET: no positions. `{{"closed", {Low}, grabs}, {"open", {High}}}`.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1003`](../../include/shulib/mechanism/kinds.hpp#L1003).*

<a id="high"></a>

## `High`

```cpp
inline constexpr bool High = true
```

A line driven HIGH (on, `true` — the solenoid powered).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1016`](../../include/shulib/mechanism/kinds.hpp#L1016).*

<a id="low"></a>

## `Low`

```cpp
inline constexpr bool Low = false
```

A line driven LOW (off, `false` — the solenoid unpowered).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1018`](../../include/shulib/mechanism/kinds.hpp#L1018).*

<a id="grabs"></a>

## `grabs`

```cpp
inline constexpr Confirm grabs = Confirm::Sees
```

Reaching this position GRABS a piece: a piece sensor confirms it; with none, UNCONFIRMED and counted done (Q2 (c)).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1021`](../../include/shulib/mechanism/kinds.hpp#L1021).*

<a id="releases"></a>

## `releases`

```cpp
inline constexpr Confirm releases = Confirm::StopsSeeing
```

Reaching this position lets a piece GO: a piece sensor confirms it is gone.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1023`](../../include/shulib/mechanism/kinds.hpp#L1023).*

<a id="struct-pneumatic"></a>

## `struct Pneumatic`

```cpp
struct Pneumatic
```

PNEUMATIC — two or three named positions, each the team's line levels (Q13 (a)).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:1026`](../../include/shulib/mechanism/kinds.hpp#L1026).*

<a id="pneumatic-name"></a>

### `Pneumatic::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1028`](../../include/shulib/mechanism/kinds.hpp#L1028).*

<a id="pneumatic-positions"></a>

### `Pneumatic::positions`

```cpp
Positions positions{}
```

REQUIRED: {{"closed", {Low}, grabs}, {"open", {High}}} — one level per wired line.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1030`](../../include/shulib/mechanism/kinds.hpp#L1030).*

<a id="pneumatic-start"></a>

### `Pneumatic::start`

```cpp
const char* start = nullptr
```

REQUIRED: the position at program start (a position's name).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1032`](../../include/shulib/mechanism/kinds.hpp#L1032).*

<a id="pneumatic-safe"></a>

### `Pneumatic::safe`

```cpp
const char* safe = nullptr
```

REQUIRED: the position the end-of-run guard puts it in.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1034`](../../include/shulib/mechanism/kinds.hpp#L1034).*

<a id="pneumatic-movetime"></a>

### `Pneumatic::moveTime`

```cpp
units::Time moveTime{0.0}
```

REQUIRED: how long the cylinder takes (no default, P6).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1036`](../../include/shulib/mechanism/kinds.hpp#L1036).*

<a id="pneumatic-piecesensor"></a>

### `Pneumatic::pieceSensor`

```cpp
Sensor pieceSensor{}
```

Optional: confirms a `grabs` / `releases` position.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1038`](../../include/shulib/mechanism/kinds.hpp#L1038).*

<a id="pneumatic-confirmwithin"></a>

### `Pneumatic::confirmWithin`

```cpp
units::Time confirmWithin{0.0}
```

With a piece sensor: how long after moveTime the confirmation may take.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1040`](../../include/shulib/mechanism/kinds.hpp#L1040).*

<a id="pneumatic-changeat"></a>

### `Pneumatic::changeAt`

```cpp
const char* changeAt = nullptr
```

Generated robot files only: the positions' JSON path (G3a).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1042`](../../include/shulib/mechanism/kinds.hpp#L1042).*

<a id="pneumatic-to"></a>

### `Pneumatic::to`

```cpp
template <class T> [[nodiscard]] consteval PneumaticGoal to(const T&) const
```

`claw.to(...)`: refused BY NAME (a position is named: claw["open"]).

*function, declared at [`include/shulib/mechanism/kinds.hpp:1046`](../../include/shulib/mechanism/kinds.hpp#L1046).*

<a id="pneumatic-operator-index"></a>

### `Pneumatic::operator[]`

```cpp
[[nodiscard]] consteval PneumaticGoal operator[](const char* position) const
```

A named position: `claw["closed"]`. A name it does not have FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1051`](../../include/shulib/mechanism/kinds.hpp#L1051).*

<a id="pneumatic-spec"></a>

### `Pneumatic::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec(std::uint8_t lines = 1) const noexcept
```

This declaration as data, with `lines` 3-wire lines — the WIRING says how many (lines('A') or lines('B', 'C')).

*function, declared at [`include/shulib/mechanism/kinds.hpp:1063`](../../include/shulib/mechanism/kinds.hpp#L1063).*

<a id="arm"></a>

## `Arm`

```cpp
using mechanism::Arm
```

The ARM declaration (degrees, held).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1106`](../../include/shulib/mechanism/kinds.hpp#L1106).*

<a id="lift"></a>

## `Lift`

```cpp
using mechanism::Lift
```

The LIFT declaration (inches, held).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1108`](../../include/shulib/mechanism/kinds.hpp#L1108).*

<a id="pneumatic"></a>

## `Pneumatic`

```cpp
using mechanism::Pneumatic
```

The PNEUMATIC declaration (named positions).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1110`](../../include/shulib/mechanism/kinds.hpp#L1110).*

<a id="power"></a>

## `Power`

```cpp
using mechanism::Power
```

The POWER declaration (volts).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1112`](../../include/shulib/mechanism/kinds.hpp#L1112).*

<a id="flywheel"></a>

## `Flywheel`

```cpp
using mechanism::Flywheel
```

The FLYWHEEL declaration (output rpm, "at speed").

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1114`](../../include/shulib/mechanism/kinds.hpp#L1114).*

<a id="forward-2"></a>

## `Forward (overload 2)`

```cpp
using mechanism::Forward
```

A motor mounted the + way.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1116`](../../include/shulib/mechanism/kinds.hpp#L1116).*

<a id="reversed-2"></a>

## `Reversed (overload 2)`

```cpp
using mechanism::Reversed
```

A motor mounted the other way.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1118`](../../include/shulib/mechanism/kinds.hpp#L1118).*

<a id="rotationsensor-2"></a>

## `rotationSensor (overload 2)`

```cpp
using mechanism::rotationSensor
```

An angle sensor: rotationSensor() / rotationSensor(Reversed).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1120`](../../include/shulib/mechanism/kinds.hpp#L1120).*

<a id="distancesensor-2"></a>

## `distanceSensor (overload 2)`

```cpp
using mechanism::distanceSensor
```

A distance piece sensor: distanceSensor(2_in).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1122`](../../include/shulib/mechanism/kinds.hpp#L1122).*

<a id="opticalsensor-2"></a>

## `opticalSensor (overload 2)`

```cpp
using mechanism::opticalSensor
```

An optical piece sensor: opticalSensor(0.5).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1124`](../../include/shulib/mechanism/kinds.hpp#L1124).*

<a id="limitswitch-2"></a>

## `limitSwitch (overload 2)`

```cpp
using mechanism::limitSwitch
```

A limit-switch piece sensor: limitSwitch().

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1126`](../../include/shulib/mechanism/kinds.hpp#L1126).*

<a id="stall"></a>

## `Stall`

```cpp
using mechanism::Stall
```

A stall signature: {.amps, .belowMotorRpm, .lasting}.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1128`](../../include/shulib/mechanism/kinds.hpp#L1128).*

<a id="cosine-2"></a>

## `cosine (overload 2)`

```cpp
using mechanism::cosine
```

An arm's gravity: cosine(1.6_volt, 45).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1130`](../../include/shulib/mechanism/kinds.hpp#L1130).*

<a id="constantgravity-2"></a>

## `constantGravity (overload 2)`

```cpp
using mechanism::constantGravity
```

A lift's gravity: constantGravity(1.0_volt).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1132`](../../include/shulib/mechanism/kinds.hpp#L1132).*

<a id="nogravity-2"></a>

## `noGravity (overload 2)`

```cpp
using mechanism::noGravity
```

No gravity to hold.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1134`](../../include/shulib/mechanism/kinds.hpp#L1134).*

<a id="restsatstop-3"></a>

## `restsAtStop (overload 3)`

```cpp
using mechanism::restsAtStop
```

Homing: it rests on its stop at program start.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1136`](../../include/shulib/mechanism/kinds.hpp#L1136).*

<a id="homebyswitch-3"></a>

## `homeBySwitch (overload 3)`

```cpp
using mechanism::homeBySwitch
```

Homing: push until its switch is pressed.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1138`](../../include/shulib/mechanism/kinds.hpp#L1138).*

<a id="homebystall-3"></a>

## `homeByStall (overload 3)`

```cpp
using mechanism::homeByStall
```

Homing: push until it stalls.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1140`](../../include/shulib/mechanism/kinds.hpp#L1140).*

<a id="absolutesensor-2"></a>

## `absoluteSensor (overload 2)`

```cpp
using mechanism::absoluteSensor
```

Homing: an absolute angle sensor (HA-186).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1142`](../../include/shulib/mechanism/kinds.hpp#L1142).*

<a id="neverhomed-2"></a>

## `neverHomed (overload 2)`

```cpp
using mechanism::neverHomed
```

Never homed.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1144`](../../include/shulib/mechanism/kinds.hpp#L1144).*

<a id="spool-2"></a>

## `spool (overload 2)`

```cpp
using mechanism::spool
```

A lift's shape: a spool.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1146`](../../include/shulib/mechanism/kinds.hpp#L1146).*

<a id="linkage-2"></a>

## `linkage (overload 2)`

```cpp
using mechanism::linkage
```

A lift's shape: a four-bar or a DR4B.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1148`](../../include/shulib/mechanism/kinds.hpp#L1148).*

<a id="measured-2"></a>

## `measured (overload 2)`

```cpp
using mechanism::measured
```

A lift's shape: measured pairs.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1150`](../../include/shulib/mechanism/kinds.hpp#L1150).*

<a id="heightat"></a>

## `HeightAt`

```cpp
using mechanism::HeightAt
```

One measured pair.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1152`](../../include/shulib/mechanism/kinds.hpp#L1152).*

<a id="within-2"></a>

## `within (overload 2)`

```cpp
using mechanism::within
```

A flywheel's "at speed" band.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1154`](../../include/shulib/mechanism/kinds.hpp#L1154).*

<a id="high-2"></a>

## `High (overload 2)`

```cpp
using mechanism::High
```

A line driven high.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1156`](../../include/shulib/mechanism/kinds.hpp#L1156).*

<a id="low-2"></a>

## `Low (overload 2)`

```cpp
using mechanism::Low
```

A line driven low.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1158`](../../include/shulib/mechanism/kinds.hpp#L1158).*

<a id="grabs-2"></a>

## `grabs (overload 2)`

```cpp
using mechanism::grabs
```

A position that grabs.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1160`](../../include/shulib/mechanism/kinds.hpp#L1160).*

<a id="releases-2"></a>

## `releases (overload 2)`

```cpp
using mechanism::releases
```

A position that releases.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1162`](../../include/shulib/mechanism/kinds.hpp#L1162).*

<a id="gears-2"></a>

## `gears (overload 2)`

```cpp
using mechanism::gears
```

A gear pair, driving then driven.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1164`](../../include/shulib/mechanism/kinds.hpp#L1164).*

<a id="direct-2"></a>

## `direct (overload 2)`

```cpp
using mechanism::direct
```

No gearing.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1166`](../../include/shulib/mechanism/kinds.hpp#L1166).*

<a id="red"></a>

## `Red`

```cpp
inline constexpr hal::MotorCartridge Red = hal::MotorCartridge::Red
```

The RED cartridge (100 rpm).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1168`](../../include/shulib/mechanism/kinds.hpp#L1168).*

<a id="green"></a>

## `Green`

```cpp
inline constexpr hal::MotorCartridge Green = hal::MotorCartridge::Green
```

The GREEN cartridge (200 rpm).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1170`](../../include/shulib/mechanism/kinds.hpp#L1170).*

<a id="blue"></a>

## `Blue`

```cpp
inline constexpr hal::MotorCartridge Blue = hal::MotorCartridge::Blue
```

The BLUE cartridge (600 rpm).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1172`](../../include/shulib/mechanism/kinds.hpp#L1172).*

<a id="struct-brakeword"></a>

## `struct BrakeWord`

```cpp
struct BrakeWord
```

ONE word per brake mode for both places a team writes one: a mechanism's `.safe` (a config::SafeBrakeMode) and the driver loop's `.releaseBrakeMode` (a hal::BrakeMode) — so a driver file needs no `hal::BrakeMode::Coast` beside the robot file's `Coast` (N11).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:1176`](../../include/shulib/mechanism/kinds.hpp#L1176).*

<a id="brakeword-safe"></a>

### `BrakeWord::safe`

```cpp
config::SafeBrakeMode safe
```

As a mechanism's declared safe state.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1178`](../../include/shulib/mechanism/kinds.hpp#L1178).*

<a id="brakeword-drive"></a>

### `BrakeWord::drive`

```cpp
hal::BrakeMode drive
```

As the drive's brake mode.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1180`](../../include/shulib/mechanism/kinds.hpp#L1180).*

<a id="brakeword-config-safebrakemode"></a>

### `BrakeWord::config::SafeBrakeMode`

```cpp
constexpr operator config::SafeBrakeMode() const noexcept
```

Read as a mechanism's `.safe`.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1182`](../../include/shulib/mechanism/kinds.hpp#L1182).*

<a id="brakeword-hal-brakemode"></a>

### `BrakeWord::hal::BrakeMode`

```cpp
constexpr operator hal::BrakeMode() const noexcept
```

Read as a drive brake mode.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1184`](../../include/shulib/mechanism/kinds.hpp#L1184).*

<a id="hold"></a>

## `Hold`

```cpp
inline constexpr BrakeWord Hold{config::SafeBrakeMode::Hold, hal::BrakeMode::Hold}
```

HOLD: drives back to where it stopped — the one that keeps a load up.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1187`](../../include/shulib/mechanism/kinds.hpp#L1187).*

<a id="brake"></a>

## `Brake`

```cpp
inline constexpr BrakeWord Brake{config::SafeBrakeMode::Brake, hal::BrakeMode::Brake}
```

BRAKE: resists, but does not return to where it stopped.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1189`](../../include/shulib/mechanism/kinds.hpp#L1189).*

<a id="coast"></a>

## `Coast`

```cpp
inline constexpr BrakeWord Coast{config::SafeBrakeMode::Coast, hal::BrakeMode::Coast}
```

COAST: undriven — for a mechanism that must never sit at stall current.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1191`](../../include/shulib/mechanism/kinds.hpp#L1191).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 66 lines, click to expand</summary>

```text

 mechanism/kinds.hpp — WHAT A MECHANISM IS, the way a team writes it (the mechanism core, chunk
 MECH-A1, 2026-10-08).

 ONE declaration per mechanism, in the team's own words — a name, its motors and which way each is
 mounted, its cartridge, its gearing as tooth counts, what it does when stopped and what a jam
 looks like, its limits and presets — with NO port anywhere: the ports are the wiring
 (mechanism/wiring.hpp), a separate block of the robot file (P2; the team lead's Q1 (a): the
 binding lives only there).

     inline constexpr Arm arm{
         .name = "arm",
         .motors = {Forward},
         .cartridge = Green,
         .gearing = gears(12, 60),         // a 12-tooth gear on the motor drives a 60 on the arm
         .safe = Hold,
         .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
         .range = {0, 160},                // degrees: down on its stop .. just past vertical
         .gravity = cosine(1.6_volt, 45),  // 1.6 V holds it out level; it is level at 45 degrees
         .home = restsAtStop(0),           // it rests on its stop at 0 when the program starts
         .presets = {{"stow", 0}, {"score", 95}, {"high", 140}},
     };

 ── THE MEMBERS ARE WRITTEN IN ONE ORDER PER KIND — C++ requires it ──────────────────
 A designated initializer must follow the declaration order, so each kind's members are ordered
 the way a student reads them: what it IS (name, motors, cartridge, gearing, shape, angle or piece
 sensor), what it does STOPPED (safe, stall), where it GOES (range, gravity, home, presets — or
 at-speed and presets), the extras (unhomedVolts, tuning). Out of order is a plain C++ error
 ("designator order ... does not match declaration order"); the guide prints each kind's order.

 ── THE DECLARATION IS THE HANDLE (Q18 (a): "checked at build") ──────────────────────
 An auton names `arm`, a preset `arm["score"]`, a constant target `arm.to(120)`. A misspelt or
 removed mechanism is "'arm' was not declared in this scope"; a misspelt preset, a constant past
 the travel limits, a speed past the free speed, a pneumatic position the mechanism does not have
 — each is a consteval refusal at the auton's own line, NAMED (detail::no_such_preset_on_this_
 mechanism, ...); and the type-shaped slips are named too: `arm.to(90_deg)`, `lift.to(18)`,
 `intake["fast"]`, `claw.to(...)`, `.gearing = 5`. Nothing is generated in the core: G3a's robot
 panel writes these same declarations.

 ── EVERY FIELD HAS AN UNSET VALUE, and a number the team owns is never invented (P6) ─
 A field left out is UNSET, and the robot's list (mechanisms(...), wiring.hpp) refuses it AT BUILD,
 naming it (spec.hpp's refusals). Exactly three kinds of value are NOT the team's and come from the
 library, each a STARTING value, PROVISIONAL: the controllers' gains, the smooth move's speed and
 acceleration (a fraction of the cartridge's free speed through the stated gearing), and the
 "reached" tolerance — the requirements' "gains ship with starting values and are MEASURED on the
 robot later" (N7). They are the hardware assumption the brief holds as HA-182 (MECH-A3 registers
 it with the behaviours that run these numbers; nothing in this run commands a motor). Each can be
 stated instead, one by one (`.tuning`). No time limit is ever a default: every step states its own.

 ── UNITS ────────────────────────────────────────────────────────────────────────────
 An Arm is in DEGREES of the arm — plain numbers: `_deg` is a WRAPPING heading (270 reads -90), so
 `arm.to(90_deg)` does not compile. A Lift is in INCHES (`_in`). A Flywheel is in RPM of its output.
 Gains are per OUTPUT-SHAFT DEGREE for every position mechanism (a lift's per degree of its spool).
 The GEARING is tooth counts, driving then driven: gears(12, 60) is a 5:1 reduction (the output
 turns once per 5 motor turns). Never a bare number — the drive geometry states its ratio the other
 way round (hal/drive_geometry.hpp `motorToWheelRatio`, wheel turns per motor turn), and a bare
 5 or 1.0/5 would compile either way and be wrong by 25x (re-check U1).

 ── WHERE TO CHANGE IT ───────────────────────────────────────────────────────────────
 Every declaration records the file and line of its `.name = ...` (std::source_location, a
 defaulted argument nobody writes), so a boot line or the direction check can name the robot
 file's line with no string typed by hand; a GENERATED robot file states each role's JSON path in
 `changeAt` instead (G3a). The path is the one the compiler opened — it differs between the host
 and the brain, so a test pins its suffix.

 PROS-free, allocation-free; everything here is constexpr or consteval.
```

</details>
