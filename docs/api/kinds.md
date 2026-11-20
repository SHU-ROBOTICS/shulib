<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/kinds.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `kinds.hpp`

mechanism/kinds.hpp — WHAT A MECHANISM IS, the way a team writes it.

This header declares **31** types (186 members), **20** free functions, **15** constants, and **31** type aliass.

Extracted from [`include/shulib/mechanism/kinds.hpp`](../../include/shulib/mechanism/kinds.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

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
- [`struct CurrentCap`](#struct-currentcap)
  - [`amps`](#currentcap-amps)
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
  - [`units`](#positiongoal-units)
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
- [`positionStartingValues`](#positionstartingvalues) — *free function*
- [`speedStartingValues`](#speedstartingvalues) — *free function*
- [`struct Power`](#struct-power)
  - [`name`](#power-name)
  - [`motors`](#power-motors)
  - [`cartridge`](#power-cartridge)
  - [`pieceSensor`](#power-piecesensor)
  - [`safe`](#power-safe)
  - [`stall`](#power-stall)
  - [`currentCap`](#power-currentcap)
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
  - [`currentCap`](#arm-currentcap)
  - [`range`](#arm-range)
  - [`gravity`](#arm-gravity)
  - [`home`](#arm-home)
  - [`presets`](#arm-presets)
  - [`unhomedVolts`](#arm-unhomedvolts)
  - [`maxDownVolts`](#arm-maxdownvolts)
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
  - [`currentCap`](#lift-currentcap)
  - [`range`](#lift-range)
  - [`gravity`](#lift-gravity)
  - [`home`](#lift-home)
  - [`presets`](#lift-presets)
  - [`unhomedVolts`](#lift-unhomedvolts)
  - [`maxDownVolts`](#lift-maxdownvolts)
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
  - [`currentCap`](#flywheel-currentcap)
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
  - [`operator config::SafeBrakeMode`](#brakeword-operator-config-safebrakemode)
  - [`operator hal::BrakeMode`](#brakeword-operator-hal-brakemode)
- [`Hold`](#hold) — *constant*
- [`Brake`](#brake) — *constant*
- [`Coast`](#coast) — *constant*

<a id="struct-name"></a>

## `struct Name`

```cpp
struct Name
```

A mechanism's name ("claw pivot 6": boot lines, logs, the route ids' stem, the run-time lookup). Written as a plain string (`.name = "arm"`); it also records the FILE and LINE it was written on, which is how the direction check and the boot lines point at the declaration (header).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:95`](../../include/shulib/mechanism/kinds.hpp#L95).*

<a id="name-text"></a>

### `Name::text`

```cpp
const char* text = nullptr
```

The name's text, borrowed from the team's literal (null = UNSET).

*field, declared at [`include/shulib/mechanism/kinds.hpp:97`](../../include/shulib/mechanism/kinds.hpp#L97).*

<a id="name-file"></a>

### `Name::file`

```cpp
const char* file = nullptr
```

The file the `.name = ...` was written in (the path the compiler opened).

*field, declared at [`include/shulib/mechanism/kinds.hpp:99`](../../include/shulib/mechanism/kinds.hpp#L99).*

<a id="name-line"></a>

### `Name::line`

```cpp
std::uint32_t line = 0
```

The line it was written on.

*field, declared at [`include/shulib/mechanism/kinds.hpp:101`](../../include/shulib/mechanism/kinds.hpp#L101).*

<a id="name-name"></a>

### `Name::Name`

```cpp
constexpr Name() noexcept = default
```

UNSET.

*function, declared at [`include/shulib/mechanism/kinds.hpp:103`](../../include/shulib/mechanism/kinds.hpp#L103).*

<a id="name-name-2"></a>

### `Name::Name (overload 2)`

```cpp
consteval Name(const char* s, std::source_location at = std::source_location::current()) noexcept
```

UNSET. `.name = "arm"`: the text, and where it was written (the defaulted `at` is the team's line).

*function, declared at [`include/shulib/mechanism/kinds.hpp:105`](../../include/shulib/mechanism/kinds.hpp#L105).*

<a id="struct-motor"></a>

## `struct Motor`

```cpp
struct Motor
```

One MOTOR role: how the motor is MOUNTED (which way + turns the mechanism), never its port.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:112`](../../include/shulib/mechanism/kinds.hpp#L112).*

<a id="motor-present"></a>

### `Motor::present`

```cpp
bool present = false
```

Set by Forward / Reversed: a role the mechanism has.

*field, declared at [`include/shulib/mechanism/kinds.hpp:114`](../../include/shulib/mechanism/kinds.hpp#L114).*

<a id="motor-reversed"></a>

### `Motor::reversed`

```cpp
bool reversed = false
```

Mounted so that + turns the mechanism the other way.

*field, declared at [`include/shulib/mechanism/kinds.hpp:116`](../../include/shulib/mechanism/kinds.hpp#L116).*

<a id="motor-changeat"></a>

### `Motor::changeAt`

```cpp
const char* changeAt = nullptr
```

Generated robot files only: this role's JSON path (the direction check's "change it HERE").

*field, declared at [`include/shulib/mechanism/kinds.hpp:118`](../../include/shulib/mechanism/kinds.hpp#L118).*

<a id="forward"></a>

## `Forward`

```cpp
inline constexpr Motor Forward{.present = true}
```

A motor mounted so that + moves the mechanism its "+" way (up, in, out — the team's word).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:121`](../../include/shulib/mechanism/kinds.hpp#L121).*

<a id="reversed"></a>

## `Reversed`

```cpp
inline constexpr Motor Reversed{.present = true, .reversed = true}
```

A motor mounted the other way round (the mirror-image motor of a two-motor lift).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:123`](../../include/shulib/mechanism/kinds.hpp#L123).*

<a id="struct-motors"></a>

## `struct Motors`

```cpp
struct Motors
```

A mechanism's MOTOR roles, in order: `.motors = {Forward, Reversed}` (one to four; a fifth is recorded as "too many" and refused by the list).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:127`](../../include/shulib/mechanism/kinds.hpp#L127).*

<a id="motors-m"></a>

### `Motors::m`

```cpp
Motor m[kMaxMotorRoles]{}
```

The roles, in the order written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:129`](../../include/shulib/mechanism/kinds.hpp#L129).*

<a id="motors-count"></a>

### `Motors::count`

```cpp
std::uint8_t count = 0
```

How many were written (at most kMaxMotorRoles).

*field, declared at [`include/shulib/mechanism/kinds.hpp:131`](../../include/shulib/mechanism/kinds.hpp#L131).*

<a id="motors-toomany"></a>

### `Motors::tooMany`

```cpp
bool tooMany = false
```

More than kMaxMotorRoles were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:133`](../../include/shulib/mechanism/kinds.hpp#L133).*

<a id="motors-file"></a>

### `Motors::file`

```cpp
const char* file = nullptr
```

Where the list was written (GCC 13 captures an initializer list at the declaration's CLOSING line — measured — so the spec points every role at the `.name` line instead).

*field, declared at [`include/shulib/mechanism/kinds.hpp:136`](../../include/shulib/mechanism/kinds.hpp#L136).*

<a id="motors-line"></a>

### `Motors::line`

```cpp
std::uint32_t line = 0
```

That line.

*field, declared at [`include/shulib/mechanism/kinds.hpp:138`](../../include/shulib/mechanism/kinds.hpp#L138).*

<a id="motors-motors"></a>

### `Motors::Motors`

```cpp
constexpr Motors() noexcept = default
```

UNSET: no motors.

*function, declared at [`include/shulib/mechanism/kinds.hpp:140`](../../include/shulib/mechanism/kinds.hpp#L140).*

<a id="motors-motors-2"></a>

### `Motors::Motors (overload 2)`

```cpp
consteval Motors(std::initializer_list<Motor> list, std::source_location at = std::source_location::current()) noexcept
```

UNSET: no motors. `.motors = {Forward, Reversed}`.

*function, declared at [`include/shulib/mechanism/kinds.hpp:142`](../../include/shulib/mechanism/kinds.hpp#L142).*

<a id="struct-sensor"></a>

## `struct Sensor`

```cpp
struct Sensor
```

One SENSOR role (an angle sensor, a piece sensor). Made by the words below, never by hand.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:156`](../../include/shulib/mechanism/kinds.hpp#L156).*

<a id="sensor-kind"></a>

### `Sensor::kind`

```cpp
SensorKind kind = SensorKind::None
```

What it is; None = no such role.

*field, declared at [`include/shulib/mechanism/kinds.hpp:158`](../../include/shulib/mechanism/kinds.hpp#L158).*

<a id="sensor-reversed"></a>

### `Sensor::reversed`

```cpp
bool reversed = false
```

Rotation: mounted so that + reads the other way.

*field, declared at [`include/shulib/mechanism/kinds.hpp:160`](../../include/shulib/mechanism/kinds.hpp#L160).*

<a id="sensor-pressedlevel"></a>

### `Sensor::pressedLevel`

```cpp
bool pressedLevel = true
```

Switch: the level that means PRESSED — `true` (HA-184's claim).

*field, declared at [`include/shulib/mechanism/kinds.hpp:162`](../../include/shulib/mechanism/kinds.hpp#L162).*

<a id="sensor-seeswithinin"></a>

### `Sensor::seesWithinIn`

```cpp
double seesWithinIn = 0.0
```

Distance: closer than this many inches = "sees a piece".

*field, declared at [`include/shulib/mechanism/kinds.hpp:164`](../../include/shulib/mechanism/kinds.hpp#L164).*

<a id="sensor-seesproximity"></a>

### `Sensor::seesProximity`

```cpp
double seesProximity = 0.0
```

Optical: proximity at or above this = "sees a piece", (0, 1].

*field, declared at [`include/shulib/mechanism/kinds.hpp:166`](../../include/shulib/mechanism/kinds.hpp#L166).*

<a id="sensor-changeat"></a>

### `Sensor::changeAt`

```cpp
const char* changeAt = nullptr
```

Generated robot files only: this role's JSON path.

*field, declared at [`include/shulib/mechanism/kinds.hpp:168`](../../include/shulib/mechanism/kinds.hpp#L168).*

<a id="sensor-file"></a>

### `Sensor::file`

```cpp
const char* file = nullptr
```

Where it was written (captured).

*field, declared at [`include/shulib/mechanism/kinds.hpp:170`](../../include/shulib/mechanism/kinds.hpp#L170).*

<a id="sensor-line"></a>

### `Sensor::line`

```cpp
std::uint32_t line = 0
```

That line.

*field, declared at [`include/shulib/mechanism/kinds.hpp:172`](../../include/shulib/mechanism/kinds.hpp#L172).*

<a id="rotationsensor"></a>

## `rotationSensor`

```cpp
[[nodiscard]] constexpr Sensor rotationSensor(Motor mounted = Forward, std::source_location at = std::source_location::current()) noexcept
```

A V5 ROTATION sensor on the mechanism's output shaft — its angle sensor. `mounted` Reversed when + reads the other way.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:176`](../../include/shulib/mechanism/kinds.hpp#L176).*

<a id="distancesensor"></a>

## `distanceSensor`

```cpp
[[nodiscard]] constexpr Sensor distanceSensor(units::Length within, std::source_location at = std::source_location::current()) noexcept
```

A DISTANCE sensor as a piece sensor: "sees a piece" when something is closer than `within`.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:181`](../../include/shulib/mechanism/kinds.hpp#L181).*

<a id="opticalsensor"></a>

## `opticalSensor`

```cpp
[[nodiscard]] constexpr Sensor opticalSensor(double proximity, std::source_location at = std::source_location::current()) noexcept
```

An OPTICAL sensor as a piece sensor: "sees a piece" at proximity >= `proximity`, in (0, 1].

*free function, declared at [`include/shulib/mechanism/kinds.hpp:186`](../../include/shulib/mechanism/kinds.hpp#L186).*

<a id="limitswitch"></a>

## `limitSwitch`

```cpp
[[nodiscard]] constexpr Sensor limitSwitch(std::source_location at = std::source_location::current()) noexcept
```

A LIMIT SWITCH (or bumper) on a 3-wire port as a piece sensor: pressed = sees a piece.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:191`](../../include/shulib/mechanism/kinds.hpp#L191).*

<a id="struct-stall"></a>

## `struct Stall`

```cpp
struct Stall
```

A STALL SIGNATURE: the motors draw at least `amps` while turning slower than `belowMotorRpm` (at the MOTOR shaft — every other rpm a team writes is the output's) for `lasting` (manipulation::StallDetector's three numbers). The team's — never a default. REQUIRED on an Arm, a Lift and a Flywheel (N10: their behaviour goes on after the step — a hold, a spin — and a blocked one must never sit at stall current, P4); optional on a Power mechanism. A stated one must be able to trip: amps below the motor's 2.5 A limit, belowMotorRpm above 0, lasting above 0.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:201`](../../include/shulib/mechanism/kinds.hpp#L201).*

<a id="stall-amps"></a>

### `Stall::amps`

```cpp
double amps = 0.0
```

At least this current, A (below 2.5).

*field, declared at [`include/shulib/mechanism/kinds.hpp:203`](../../include/shulib/mechanism/kinds.hpp#L203).*

<a id="stall-belowmotorrpm"></a>

### `Stall::belowMotorRpm`

```cpp
double belowMotorRpm = 0.0
```

While the MOTOR shaft turns slower than this, rpm (above 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:205`](../../include/shulib/mechanism/kinds.hpp#L205).*

<a id="stall-lasting"></a>

### `Stall::lasting`

```cpp
units::Time lasting{0.0}
```

Continuously for this long (above 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:207`](../../include/shulib/mechanism/kinds.hpp#L207).*

<a id="struct-currentcap"></a>

## `struct CurrentCap`

```cpp
struct CurrentCap
```

A CURRENT CAP (chunk MECH-S-A, 2026-11-05): the most current EACH motor of the mechanism may draw, amperes — `.currentCap = {.amps = 1.2}`. Optional on a Power, an Arm, a Lift and a Flywheel; the team's number, never a default (none written = the motor's own 2.5 A). Set in every motor at boot and enforced by the MOTOR itself from then on — every command, the Hold brake, a homing push, a blocking loop that starves the service (A4 register HA-219: how fast, unmeasured). PER MOTOR: two motors on one spool pull with twice it. The build refuses one below 0 or above 2.5 A, one under 0.001 A (a V5 limit is whole milliamps), and any stated stall signature at or above it (a capped motor never draws more: the signature could never trip) — a cap written as 0 is no cap, the same as leaving it out; the boot warns when a signature is not MORE than 0.2 A below it, as the motor reads it in whole milliamps (HA-223). Choosing it — the ORDER four currents must keep: the heaviest HOLD (with a game piece) < the stall signature < the cap − 0.2 A, the heaviest MOVE's peak < the cap, and the cap ≤ what the string can take (about 10 lbf per amp per motor for a red cartridge on a 1.5 in spool — HA-222, NOT measured). It does not limit the carriage's momentum when it hits a stop (HA-222): push gently.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:223`](../../include/shulib/mechanism/kinds.hpp#L223).*

<a id="currentcap-amps"></a>

### `CurrentCap::amps`

```cpp
double amps = 0.0
```

At most this current per motor, A: from 0.001 (one milliamp) to 2.5. 0 = no cap stated.

*field, declared at [`include/shulib/mechanism/kinds.hpp:225`](../../include/shulib/mechanism/kinds.hpp#L225).*

<a id="struct-gearing"></a>

## `struct Gearing`

```cpp
struct Gearing
```

THE GEARING, as tooth counts (header): `gears(12, 60)` = a 12-tooth gear on the motor drives a 60; `gears(60, 12)` = geared UP; stages multiply, `gears(12, 36) * gears(12, 60)`; `direct` = none. Sprockets the same way (teeth). A bare number does not compile, and names the fix.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:236`](../../include/shulib/mechanism/kinds.hpp#L236).*

<a id="gearing-motorturnsperturn"></a>

### `Gearing::motorTurnsPerTurn`

```cpp
double motorTurnsPerTurn = 0.0
```

Motor turns per output turn (> 0); 0 = UNSET.

*field, declared at [`include/shulib/mechanism/kinds.hpp:238`](../../include/shulib/mechanism/kinds.hpp#L238).*

<a id="gearing-gearing"></a>

### `Gearing::Gearing`

```cpp
constexpr Gearing() noexcept = default
```

UNSET.

*function, declared at [`include/shulib/mechanism/kinds.hpp:240`](../../include/shulib/mechanism/kinds.hpp#L240).*

<a id="gearing-gearing-2"></a>

### `Gearing::Gearing (overload 2)`

```cpp
template <class Write_the_gearing_as_tooth_counts_like_gears_12_60> requires std::is_arithmetic_v<Write_the_gearing_as_tooth_counts_like_gears_12_60> Gearing(Write_the_gearing_as_tooth_counts_like_gears_12_60) = delete
```

UNSET. `.gearing = 5`: refused. The template parameter's NAME is the reason GCC prints ("use of deleted function ... [with Write_the_gearing_as_tooth_counts_like_gears_12_60 = int]") — a consteval refusal inside a converting constructor printed no reason (measured, n41).

*function, declared at [`include/shulib/mechanism/kinds.hpp:245`](../../include/shulib/mechanism/kinds.hpp#L245).*

<a id="gearing-operator-star"></a>

### `Gearing::operator*`

```cpp
[[nodiscard]] constexpr Gearing operator*(Gearing o) const noexcept
```

UNSET. `.gearing = 5`: refused. The template parameter's NAME is the reason GCC prints ("use of deleted function ... [with Write_the_gearing_as_tooth_counts_like_gears_12_60 = int]") — a consteval refusal inside a converting constructor printed no reason (measured, n41). Two stages in series: their reductions multiply.

*function, declared at [`include/shulib/mechanism/kinds.hpp:248`](../../include/shulib/mechanism/kinds.hpp#L248).*

<a id="gears"></a>

## `gears`

```cpp
[[nodiscard]] consteval Gearing gears(int driving, int driven)
```

A gear pair, DRIVING (on the motor's side) then DRIVEN, in teeth. Refuses a gear with no tooth.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:255`](../../include/shulib/mechanism/kinds.hpp#L255).*

<a id="direct"></a>

## `direct`

```cpp
inline constexpr Gearing direct = [] { Gearing g; g.motorTurnsPerTurn = 1.0; return g; }()
```

No gearing: the output turns with the motor.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:264`](../../include/shulib/mechanism/kinds.hpp#L264).*

<a id="struct-gravityterm"></a>

## `struct GravityTerm`

```cpp
struct GravityTerm
```

The holding term for gravity — `volts` is what holds the mechanism still where gravity pulls hardest (an arm held LEVEL; a lift anywhere). Made by cosine(), constantGravity(), noGravity.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:274`](../../include/shulib/mechanism/kinds.hpp#L274).*

<a id="gravityterm-kind"></a>

### `GravityTerm::kind`

```cpp
Gravity kind = static_cast<Gravity>(0)
```

Which shape. UNSET = 0, refused.

*field, declared at [`include/shulib/mechanism/kinds.hpp:276`](../../include/shulib/mechanism/kinds.hpp#L276).*

<a id="gravityterm-volts"></a>

### `GravityTerm::volts`

```cpp
double volts = 0.0
```

The hold, V.

*field, declared at [`include/shulib/mechanism/kinds.hpp:278`](../../include/shulib/mechanism/kinds.hpp#L278).*

<a id="gravityterm-levelatdeg"></a>

### `GravityTerm::levelAtDeg`

```cpp
double levelAtDeg = 0.0
```

Cosine: the output angle (deg) at which the arm is level.

*field, declared at [`include/shulib/mechanism/kinds.hpp:280`](../../include/shulib/mechanism/kinds.hpp#L280).*

<a id="cosine"></a>

## `cosine`

```cpp
[[nodiscard]] constexpr GravityTerm cosine(units::Voltage holdLevel, double levelAt) noexcept
```

An ARM: `holdLevel` holds it out level, less as it points up (cosine); level at `levelAt` degrees.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:283`](../../include/shulib/mechanism/kinds.hpp#L283).*

<a id="constantgravity"></a>

## `constantGravity`

```cpp
[[nodiscard]] constexpr GravityTerm constantGravity(units::Voltage volts) noexcept
```

A LIFT: the same `volts` everywhere along its travel.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:287`](../../include/shulib/mechanism/kinds.hpp#L287).*

<a id="nogravity"></a>

## `noGravity`

```cpp
inline constexpr GravityTerm noGravity{Gravity::None, 0.0, 0.0}
```

Nothing to hold (a turntable, a counterbalanced arm).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:291`](../../include/shulib/mechanism/kinds.hpp#L291).*

<a id="struct-homing"></a>

## `struct Homing`

```cpp
struct Homing
```

How a position mechanism finds its zero — every moment the requirements list: at program start with no motion (restsAtStop, absoluteSensor, a home switch already pressed), or by a homing MOVE (homeBySwitch, homeByStall) at auton start, at driver start or on request, which the auton or the driver file says. Made by the words below.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:297`](../../include/shulib/mechanism/kinds.hpp#L297).*

<a id="homing-method"></a>

### `Homing::method`

```cpp
HomeMethod method = static_cast<HomeMethod>(0)
```

How. UNSET = 0, refused.

*field, declared at [`include/shulib/mechanism/kinds.hpp:299`](../../include/shulib/mechanism/kinds.hpp#L299).*

<a id="homing-at"></a>

### `Homing::at`

```cpp
double at = 0.0
```

Where zero puts it (degrees / inches).

*field, declared at [`include/shulib/mechanism/kinds.hpp:301`](../../include/shulib/mechanism/kinds.hpp#L301).*

<a id="homing-sensoratzerodeg"></a>

### `Homing::sensorAtZeroDeg`

```cpp
double sensorAtZeroDeg = 0.0
```

absoluteSensor only: the sensor's reading (deg) when the mechanism is at `at`.

*field, declared at [`include/shulib/mechanism/kinds.hpp:303`](../../include/shulib/mechanism/kinds.hpp#L303).*

<a id="homing-volts"></a>

### `Homing::volts`

```cpp
double volts = 0.0
```

A homing move's gentle push toward its stop, V, signed (it must point at the home end).

*field, declared at [`include/shulib/mechanism/kinds.hpp:305`](../../include/shulib/mechanism/kinds.hpp#L305).*

<a id="homing-stall"></a>

### `Homing::stall`

```cpp
Stall stall{}
```

homeByStall only: the signature that means "at the stop".

*field, declared at [`include/shulib/mechanism/kinds.hpp:307`](../../include/shulib/mechanism/kinds.hpp#L307).*

<a id="restsatstop"></a>

## `restsAtStop`

```cpp
[[nodiscard]] constexpr Homing restsAtStop(double at) noexcept
```

It RESTS on its stop when the program starts (an arm down on its hard stop): that is `at`.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:310`](../../include/shulib/mechanism/kinds.hpp#L310).*

<a id="restsatstop-2"></a>

## `restsAtStop (overload 2)`

```cpp
[[nodiscard]] constexpr Homing restsAtStop(units::Length at) noexcept
```

The same for a lift: `at` in inches.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:312`](../../include/shulib/mechanism/kinds.hpp#L312).*

<a id="homebyswitch"></a>

## `homeBySwitch`

```cpp
[[nodiscard]] constexpr Homing homeBySwitch(double at, units::Voltage toward) noexcept
```

Push at `toward` until its HOME SWITCH is pressed; that is `at`. Wire it with homeSwitch('B').

*free function, declared at [`include/shulib/mechanism/kinds.hpp:314`](../../include/shulib/mechanism/kinds.hpp#L314).*

<a id="homebyswitch-2"></a>

## `homeBySwitch (overload 2)`

```cpp
[[nodiscard]] constexpr Homing homeBySwitch(units::Length at, units::Voltage toward) noexcept
```

The same for a lift: `at` in inches.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:318`](../../include/shulib/mechanism/kinds.hpp#L318).*

<a id="homebystall"></a>

## `homeByStall`

```cpp
[[nodiscard]] constexpr Homing homeByStall(double at, units::Voltage toward, Stall signature) noexcept
```

Push at `toward` until it STALLS against its stop (the team's `signature`); that is `at`.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:322`](../../include/shulib/mechanism/kinds.hpp#L322).*

<a id="homebystall-2"></a>

## `homeByStall (overload 2)`

```cpp
[[nodiscard]] constexpr Homing homeByStall(units::Length at, units::Voltage toward, Stall signature) noexcept
```

The same for a lift: `at` in inches.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:326`](../../include/shulib/mechanism/kinds.hpp#L326).*

<a id="absolutesensor"></a>

## `absoluteSensor`

```cpp
[[nodiscard]] constexpr Homing absoluteSensor(double at, double sensorReadsDeg) noexcept
```

An angle sensor that knows where it is at power-up: the mechanism is at `at` when the sensor reads `sensorReadsDeg` — a FIXED offset, never "zero wherever it stands" (HA-186: that the V5 rotation sensor's cumulative reading starts at its absolute angle). Needs `.angleSensor`.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:332`](../../include/shulib/mechanism/kinds.hpp#L332).*

<a id="neverhomed"></a>

## `neverHomed`

```cpp
inline constexpr Homing neverHomed{.method = HomeMethod::None}
```

Never homed: position moves are refused (NOT_HOMED); a held button jogs it at `.unhomedVolts`.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:336`](../../include/shulib/mechanism/kinds.hpp#L336).*

<a id="struct-degreerange"></a>

## `struct DegreeRange`

```cpp
struct DegreeRange
```

An Arm's travel limits, degrees. REQUIRED (min < max).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:339`](../../include/shulib/mechanism/kinds.hpp#L339).*

<a id="degreerange-min"></a>

### `DegreeRange::min`

```cpp
double min = 0.0
```

The lower limit.

*field, declared at [`include/shulib/mechanism/kinds.hpp:341`](../../include/shulib/mechanism/kinds.hpp#L341).*

<a id="degreerange-max"></a>

### `DegreeRange::max`

```cpp
double max = 0.0
```

The upper limit.

*field, declared at [`include/shulib/mechanism/kinds.hpp:343`](../../include/shulib/mechanism/kinds.hpp#L343).*

<a id="struct-inchrange"></a>

## `struct InchRange`

```cpp
struct InchRange
```

A Lift's travel limits, inches. REQUIRED (min < max).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:346`](../../include/shulib/mechanism/kinds.hpp#L346).*

<a id="inchrange-min"></a>

### `InchRange::min`

```cpp
units::Length min{0.0}
```

The lower limit.

*field, declared at [`include/shulib/mechanism/kinds.hpp:348`](../../include/shulib/mechanism/kinds.hpp#L348).*

<a id="inchrange-max"></a>

### `InchRange::max`

```cpp
units::Length max{0.0}
```

The upper limit.

*field, declared at [`include/shulib/mechanism/kinds.hpp:350`](../../include/shulib/mechanism/kinds.hpp#L350).*

<a id="struct-degreepreset"></a>

## `struct DegreePreset`

```cpp
struct DegreePreset
```

An Arm's named preset: `{"score", 95}`.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:353`](../../include/shulib/mechanism/kinds.hpp#L353).*

<a id="degreepreset-name"></a>

### `DegreePreset::name`

```cpp
const char* name = nullptr
```

Its name (15 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:355`](../../include/shulib/mechanism/kinds.hpp#L355).*

<a id="degreepreset-value"></a>

### `DegreePreset::value`

```cpp
double value = 0.0
```

Its angle, degrees.

*field, declared at [`include/shulib/mechanism/kinds.hpp:357`](../../include/shulib/mechanism/kinds.hpp#L357).*

<a id="struct-inchpreset"></a>

## `struct InchPreset`

```cpp
struct InchPreset
```

A Lift's named preset: `{"high", 22_in}`.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:360`](../../include/shulib/mechanism/kinds.hpp#L360).*

<a id="inchpreset-name"></a>

### `InchPreset::name`

```cpp
const char* name = nullptr
```

Its name (15 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:362`](../../include/shulib/mechanism/kinds.hpp#L362).*

<a id="inchpreset-value"></a>

### `InchPreset::value`

```cpp
units::Length value{0.0}
```

Its height.

*field, declared at [`include/shulib/mechanism/kinds.hpp:364`](../../include/shulib/mechanism/kinds.hpp#L364).*

<a id="struct-rpmpreset"></a>

## `struct RpmPreset`

```cpp
struct RpmPreset
```

A Flywheel's named speed: `{"shoot", 3000}`.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:367`](../../include/shulib/mechanism/kinds.hpp#L367).*

<a id="rpmpreset-name"></a>

### `RpmPreset::name`

```cpp
const char* name = nullptr
```

Its name (15 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:369`](../../include/shulib/mechanism/kinds.hpp#L369).*

<a id="rpmpreset-rpm"></a>

### `RpmPreset::rpm`

```cpp
double rpm = 0.0
```

Its output rpm.

*field, declared at [`include/shulib/mechanism/kinds.hpp:371`](../../include/shulib/mechanism/kinds.hpp#L371).*

<a id="struct-liftshape"></a>

## `struct LiftShape`

```cpp
struct LiftShape
```

A lift's SHAPE: how its inches relate to its output shaft's degrees. Made by spool(), linkage(), measured().

*struct, declared at [`include/shulib/mechanism/kinds.hpp:376`](../../include/shulib/mechanism/kinds.hpp#L376).*

<a id="liftshape-geometry"></a>

### `LiftShape::geometry`

```cpp
Geometry geometry = static_cast<Geometry>(0)
```

Which shape. UNSET = 0, refused.

*field, declared at [`include/shulib/mechanism/kinds.hpp:378`](../../include/shulib/mechanism/kinds.hpp#L378).*

<a id="liftshape-spooldiameterin"></a>

### `LiftShape::spoolDiameterIn`

```cpp
double spoolDiameterIn = 0.0
```

spool(): its diameter, inches.

*field, declared at [`include/shulib/mechanism/kinds.hpp:380`](../../include/shulib/mechanism/kinds.hpp#L380).*

<a id="liftshape-armlengthin"></a>

### `LiftShape::armLengthIn`

```cpp
double armLengthIn = 0.0
```

linkage(): one stage's arm length, inches.

*field, declared at [`include/shulib/mechanism/kinds.hpp:382`](../../include/shulib/mechanism/kinds.hpp#L382).*

<a id="liftshape-pivotheightin"></a>

### `LiftShape::pivotHeightIn`

```cpp
double pivotHeightIn = 0.0
```

linkage(): the first pivot's height, inches.

*field, declared at [`include/shulib/mechanism/kinds.hpp:384`](../../include/shulib/mechanism/kinds.hpp#L384).*

<a id="liftshape-stages"></a>

### `LiftShape::stages`

```cpp
std::uint8_t stages = 0
```

linkage(): 1 = a four-bar, 2 = a double reverse four-bar.

*field, declared at [`include/shulib/mechanism/kinds.hpp:386`](../../include/shulib/mechanism/kinds.hpp#L386).*

<a id="liftshape-table"></a>

### `LiftShape::table`

```cpp
double table[kMaxTablePoints][2]{}
```

measured(): {height in, output angle deg} rows.

*field, declared at [`include/shulib/mechanism/kinds.hpp:388`](../../include/shulib/mechanism/kinds.hpp#L388).*

<a id="liftshape-points"></a>

### `LiftShape::points`

```cpp
std::uint8_t points = 0
```

measured(): how many rows were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:390`](../../include/shulib/mechanism/kinds.hpp#L390).*

<a id="spool"></a>

## `spool`

```cpp
[[nodiscard]] constexpr LiftShape spool(units::Length diameter) noexcept
```

A string or chain on a spool of this diameter on the output shaft.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:393`](../../include/shulib/mechanism/kinds.hpp#L393).*

<a id="linkage"></a>

## `linkage`

```cpp
[[nodiscard]] constexpr LiftShape linkage(units::Length armLength, units::Length pivotHeight, int stages) noexcept
```

A four-bar (1 stage) or a double reverse four-bar (2 stages): each stage `armLength` long, the first pivot `pivotHeight` up; its height follows the sine of the arm's angle from level.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:398`](../../include/shulib/mechanism/kinds.hpp#L398).*

<a id="struct-heightat"></a>

## `struct HeightAt`

```cpp
struct HeightAt
```

One measured pair of a lift's table: its height at an output angle.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:403`](../../include/shulib/mechanism/kinds.hpp#L403).*

<a id="heightat-height"></a>

### `HeightAt::height`

```cpp
units::Length height{0.0}
```

The lift's height there.

*field, declared at [`include/shulib/mechanism/kinds.hpp:405`](../../include/shulib/mechanism/kinds.hpp#L405).*

<a id="heightat-degrees"></a>

### `HeightAt::degrees`

```cpp
double degrees = 0.0
```

The output shaft's angle there, degrees.

*field, declared at [`include/shulib/mechanism/kinds.hpp:407`](../../include/shulib/mechanism/kinds.hpp#L407).*

<a id="measured"></a>

## `measured`

```cpp
template <std::size_t N> [[nodiscard]] constexpr LiftShape measured(const HeightAt (&pairs)[N]) noexcept
```

Any other shape: measured pairs {height, output angle}, at least two, strictly increasing — interpolated between them. More than kMaxTablePoints pairs is recorded and refused by the list.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:412`](../../include/shulib/mechanism/kinds.hpp#L412).*

<a id="struct-positiontuning"></a>

## `struct PositionTuning`

```cpp
struct PositionTuning
```

A position mechanism's overrides of the library's STARTING values (header; N7), one by one — 0 keeps the starting value. Gains per OUTPUT-SHAFT degree; speed and acceleration in the mechanism's units per second.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:425`](../../include/shulib/mechanism/kinds.hpp#L425).*

<a id="positiontuning-kp"></a>

### `PositionTuning::kP`

```cpp
double kP = 0.0
```

Proportional gain, V/deg (starting value 0.25).

*field, declared at [`include/shulib/mechanism/kinds.hpp:427`](../../include/shulib/mechanism/kinds.hpp#L427).*

<a id="positiontuning-ki"></a>

### `PositionTuning::kI`

```cpp
double kI = 0.0
```

The hold loop's integral gain, V/(deg*s) (starting value 0.3).

*field, declared at [`include/shulib/mechanism/kinds.hpp:429`](../../include/shulib/mechanism/kinds.hpp#L429).*

<a id="positiontuning-kd"></a>

### `PositionTuning::kD`

```cpp
double kD = 0.0
```

Derivative gain, V/(deg/s) (starting value 0.004).

*field, declared at [`include/shulib/mechanism/kinds.hpp:431`](../../include/shulib/mechanism/kinds.hpp#L431).*

<a id="positiontuning-ks"></a>

### `PositionTuning::kS`

```cpp
double kS = 0.0
```

Static feedforward, V (starting value 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:433`](../../include/shulib/mechanism/kinds.hpp#L433).*

<a id="positiontuning-kv"></a>

### `PositionTuning::kV`

```cpp
double kV = 0.0
```

Velocity feedforward, V/(deg/s) (starting value 12 V over the output's free speed).

*field, declared at [`include/shulib/mechanism/kinds.hpp:435`](../../include/shulib/mechanism/kinds.hpp#L435).*

<a id="positiontuning-ka"></a>

### `PositionTuning::kA`

```cpp
double kA = 0.0
```

Acceleration feedforward, V/(deg/s^2) (starting value 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:437`](../../include/shulib/mechanism/kinds.hpp#L437).*

<a id="positiontuning-maxspeed"></a>

### `PositionTuning::maxSpeed`

```cpp
double maxSpeed = 0.0
```

The smooth move's top speed, units/s (starting value 75 % of the free speed).

*field, declared at [`include/shulib/mechanism/kinds.hpp:439`](../../include/shulib/mechanism/kinds.hpp#L439).*

<a id="positiontuning-maxacceleration"></a>

### `PositionTuning::maxAcceleration`

```cpp
double maxAcceleration = 0.0
```

Its acceleration, units/s^2 (starting value: the top speed in 0.25 s).

*field, declared at [`include/shulib/mechanism/kinds.hpp:441`](../../include/shulib/mechanism/kinds.hpp#L441).*

<a id="positiontuning-within"></a>

### `PositionTuning::within`

```cpp
double within = 0.0
```

"Reached" when within this of the target (starting value 1.5 deg, or 0.25 in) ...

*field, declared at [`include/shulib/mechanism/kinds.hpp:443`](../../include/shulib/mechanism/kinds.hpp#L443).*

<a id="positiontuning-settle"></a>

### `PositionTuning::settle`

```cpp
units::Time settle{0.0}
```

... for this long (starting value 0.1 s).

*field, declared at [`include/shulib/mechanism/kinds.hpp:445`](../../include/shulib/mechanism/kinds.hpp#L445).*

<a id="struct-speedtuning"></a>

## `struct SpeedTuning`

```cpp
struct SpeedTuning
```

A flywheel's overrides of its starting values — 0 keeps the starting value.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:448`](../../include/shulib/mechanism/kinds.hpp#L448).*

<a id="speedtuning-ks"></a>

### `SpeedTuning::kS`

```cpp
double kS = 0.0
```

Static feedforward, V (starting value 0).

*field, declared at [`include/shulib/mechanism/kinds.hpp:450`](../../include/shulib/mechanism/kinds.hpp#L450).*

<a id="speedtuning-kv"></a>

### `SpeedTuning::kV`

```cpp
double kV = 0.0
```

Velocity feedforward, V per output rad/s (starting value 12 V over the free speed).

*field, declared at [`include/shulib/mechanism/kinds.hpp:452`](../../include/shulib/mechanism/kinds.hpp#L452).*

<a id="speedtuning-kp"></a>

### `SpeedTuning::kP`

```cpp
double kP = 0.0
```

Proportional gain, V per rad/s (starting value 0.05).

*field, declared at [`include/shulib/mechanism/kinds.hpp:454`](../../include/shulib/mechanism/kinds.hpp#L454).*

<a id="speedtuning-ki"></a>

### `SpeedTuning::kI`

```cpp
double kI = 0.0
```

Integral gain, V per rad (starting value 0.2).

*field, declared at [`include/shulib/mechanism/kinds.hpp:456`](../../include/shulib/mechanism/kinds.hpp#L456).*

<a id="struct-positiongoal"></a>

## `struct PositionGoal`

```cpp
struct PositionGoal
```

A position goal: an Arm's or a Lift's preset or constant target.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:466`](../../include/shulib/mechanism/kinds.hpp#L466).*

<a id="positiongoal-mechanism"></a>

### `PositionGoal::mechanism`

```cpp
const void* mechanism
```

The declaration's address (what a driver control's build check compares).

*field, declared at [`include/shulib/mechanism/kinds.hpp:468`](../../include/shulib/mechanism/kinds.hpp#L468).*

<a id="positiongoal-mechanismname"></a>

### `PositionGoal::mechanismName`

```cpp
const char* mechanismName
```

The declaration's `.name` (what the run-time lookup compares).

*field, declared at [`include/shulib/mechanism/kinds.hpp:470`](../../include/shulib/mechanism/kinds.hpp#L470).*

<a id="positiongoal-presetname"></a>

### `PositionGoal::presetName`

```cpp
const char* presetName
```

The preset's name, or nullptr for a number.

*field, declared at [`include/shulib/mechanism/kinds.hpp:472`](../../include/shulib/mechanism/kinds.hpp#L472).*

<a id="positiongoal-value"></a>

### `PositionGoal::value`

```cpp
double value
```

The target, degrees or inches.

*field, declared at [`include/shulib/mechanism/kinds.hpp:474`](../../include/shulib/mechanism/kinds.hpp#L474).*

<a id="positiongoal-computed"></a>

### `PositionGoal::computed`

```cpp
bool computed
```

A run-time number (`clampedTo`): clamped at the limits by the soft stop, logged.

*field, declared at [`include/shulib/mechanism/kinds.hpp:476`](../../include/shulib/mechanism/kinds.hpp#L476).*

<a id="positiongoal-units"></a>

### `PositionGoal::units`

```cpp
Units units
```

Degrees (an Arm's goal) or inches (a Lift's): the service finds only a mechanism of these units (UNSET, the byte 0, finds none).

*field, declared at [`include/shulib/mechanism/kinds.hpp:479`](../../include/shulib/mechanism/kinds.hpp#L479).*

<a id="struct-speedgoal"></a>

## `struct SpeedGoal`

```cpp
struct SpeedGoal
```

A speed goal: a Flywheel's named speed or constant target.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:482`](../../include/shulib/mechanism/kinds.hpp#L482).*

<a id="speedgoal-mechanism"></a>

### `SpeedGoal::mechanism`

```cpp
const void* mechanism
```

The declaration's address.

*field, declared at [`include/shulib/mechanism/kinds.hpp:484`](../../include/shulib/mechanism/kinds.hpp#L484).*

<a id="speedgoal-mechanismname"></a>

### `SpeedGoal::mechanismName`

```cpp
const char* mechanismName
```

The declaration's `.name`.

*field, declared at [`include/shulib/mechanism/kinds.hpp:486`](../../include/shulib/mechanism/kinds.hpp#L486).*

<a id="speedgoal-presetname"></a>

### `SpeedGoal::presetName`

```cpp
const char* presetName
```

The preset's name, or nullptr for a number.

*field, declared at [`include/shulib/mechanism/kinds.hpp:488`](../../include/shulib/mechanism/kinds.hpp#L488).*

<a id="speedgoal-rpm"></a>

### `SpeedGoal::rpm`

```cpp
double rpm
```

The target, output rpm.

*field, declared at [`include/shulib/mechanism/kinds.hpp:490`](../../include/shulib/mechanism/kinds.hpp#L490).*

<a id="speedgoal-computed"></a>

### `SpeedGoal::computed`

```cpp
bool computed
```

A run-time number (`clampedTo`).

*field, declared at [`include/shulib/mechanism/kinds.hpp:492`](../../include/shulib/mechanism/kinds.hpp#L492).*

<a id="struct-pneumaticgoal"></a>

## `struct PneumaticGoal`

```cpp
struct PneumaticGoal
```

A pneumatic goal: one of its named positions.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:495`](../../include/shulib/mechanism/kinds.hpp#L495).*

<a id="pneumaticgoal-mechanism"></a>

### `PneumaticGoal::mechanism`

```cpp
const void* mechanism
```

The declaration's address.

*field, declared at [`include/shulib/mechanism/kinds.hpp:497`](../../include/shulib/mechanism/kinds.hpp#L497).*

<a id="pneumaticgoal-mechanismname"></a>

### `PneumaticGoal::mechanismName`

```cpp
const char* mechanismName
```

The declaration's `.name`.

*field, declared at [`include/shulib/mechanism/kinds.hpp:499`](../../include/shulib/mechanism/kinds.hpp#L499).*

<a id="pneumaticgoal-position"></a>

### `PneumaticGoal::position`

```cpp
std::uint8_t position
```

The position's index in the declaration.

*field, declared at [`include/shulib/mechanism/kinds.hpp:501`](../../include/shulib/mechanism/kinds.hpp#L501).*

<a id="pneumaticgoal-positionname"></a>

### `PneumaticGoal::positionName`

```cpp
const char* positionName
```

The position's name (the lookup checks it at its index).

*field, declared at [`include/shulib/mechanism/kinds.hpp:503`](../../include/shulib/mechanism/kinds.hpp#L503).*

<a id="positionstartingvalues"></a>

## `positionStartingValues`

```cpp
[[nodiscard]] constexpr PositionSpec positionStartingValues(const MechanismSpec& s) noexcept
```

The numbers a POSITION spec would run with NO `.tuning` — the library's starting values (HA-182) for its cartridge, gearing and shape. What startingValuesLine() (wiring.hpp) compares a spec against to say which of its numbers are the library's and which the team's.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:679`](../../include/shulib/mechanism/kinds.hpp#L679).*

<a id="speedstartingvalues"></a>

## `speedStartingValues`

```cpp
[[nodiscard]] constexpr SpeedSpec speedStartingValues(const MechanismSpec& s) noexcept
```

The numbers a SPEED spec would run with NO `.tuning` (HA-182), from its cartridge through its gearing.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:687`](../../include/shulib/mechanism/kinds.hpp#L687).*

<a id="struct-power"></a>

## `struct Power`

```cpp
struct Power
```

POWER — run at a voltage (intakes, rollers, conveyors, robot two's claw pivots). Members in the order C++ requires them written (header).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:698`](../../include/shulib/mechanism/kinds.hpp#L698).*

<a id="power-name"></a>

### `Power::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:700`](../../include/shulib/mechanism/kinds.hpp#L700).*

<a id="power-motors"></a>

### `Power::motors`

```cpp
Motors motors{}
```

Its motor roles.

*field, declared at [`include/shulib/mechanism/kinds.hpp:702`](../../include/shulib/mechanism/kinds.hpp#L702).*

<a id="power-cartridge"></a>

### `Power::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

Its motors' cartridge. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:704`](../../include/shulib/mechanism/kinds.hpp#L704).*

<a id="power-piecesensor"></a>

### `Power::pieceSensor`

```cpp
Sensor pieceSensor{}
```

Optional: distanceSensor(2_in), opticalSensor(0.5), limitSwitch().

*field, declared at [`include/shulib/mechanism/kinds.hpp:706`](../../include/shulib/mechanism/kinds.hpp#L706).*

<a id="power-safe"></a>

### `Power::safe`

```cpp
config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0)
```

What every stop lands in: Coast, Brake or Hold. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:708`](../../include/shulib/mechanism/kinds.hpp#L708).*

<a id="power-stall"></a>

### `Power::stall`

```cpp
Stall stall{}
```

Optional: the jam signature (`stalls`, the STALLED verdict); with none, the boot says a held button is cut only by its release (HA-154).

*field, declared at [`include/shulib/mechanism/kinds.hpp:711`](../../include/shulib/mechanism/kinds.hpp#L711).*

<a id="power-currentcap"></a>

### `Power::currentCap`

```cpp
CurrentCap currentCap{}
```

Optional: the most current each motor may draw, set in the motor at boot (CurrentCap; MECH-S-A).

*field, declared at [`include/shulib/mechanism/kinds.hpp:713`](../../include/shulib/mechanism/kinds.hpp#L713).*

<a id="power-maxvolts"></a>

### `Power::maxVolts`

```cpp
units::Voltage maxVolts{12.0}
```

The most any command may ask (the motor's own ceiling unless lowered), (0, 12] V.

*field, declared at [`include/shulib/mechanism/kinds.hpp:715`](../../include/shulib/mechanism/kinds.hpp#L715).*

<a id="power-operator-index"></a>

### `Power::operator[]`

```cpp
template <class T> [[nodiscard]] consteval int operator[](T) const
```

`intake["fast"]`: refused BY NAME — a power mechanism has no presets (run it: `r.run(intake, 12_volt, ...)`).

*function, declared at [`include/shulib/mechanism/kinds.hpp:720`](../../include/shulib/mechanism/kinds.hpp#L720).*

<a id="power-spec"></a>

### `Power::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec() const noexcept
```

This declaration as data.

*function, declared at [`include/shulib/mechanism/kinds.hpp:726`](../../include/shulib/mechanism/kinds.hpp#L726).*

<a id="struct-arm"></a>

## `struct Arm`

```cpp
struct Arm
```

ARM — go to an angle in DEGREES and hold it (arms, tilters, a claw pivot with presets).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:739`](../../include/shulib/mechanism/kinds.hpp#L739).*

<a id="arm-name"></a>

### `Arm::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:741`](../../include/shulib/mechanism/kinds.hpp#L741).*

<a id="arm-motors"></a>

### `Arm::motors`

```cpp
Motors motors{}
```

Its motor roles.

*field, declared at [`include/shulib/mechanism/kinds.hpp:743`](../../include/shulib/mechanism/kinds.hpp#L743).*

<a id="arm-cartridge"></a>

### `Arm::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

Its motors' cartridge. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:745`](../../include/shulib/mechanism/kinds.hpp#L745).*

<a id="arm-gearing"></a>

### `Arm::gearing`

```cpp
Gearing gearing{}
```

REQUIRED: gears(12, 60) — tooth counts, the motor's gear first.

*field, declared at [`include/shulib/mechanism/kinds.hpp:747`](../../include/shulib/mechanism/kinds.hpp#L747).*

<a id="arm-anglesensor"></a>

### `Arm::angleSensor`

```cpp
Sensor angleSensor{}
```

Optional: rotationSensor() — then it, not the motor encoders, is the angle.

*field, declared at [`include/shulib/mechanism/kinds.hpp:749`](../../include/shulib/mechanism/kinds.hpp#L749).*

<a id="arm-safe"></a>

### `Arm::safe`

```cpp
config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0)
```

What every stop lands in. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:751`](../../include/shulib/mechanism/kinds.hpp#L751).*

<a id="arm-stall"></a>

### `Arm::stall`

```cpp
Stall stall{}
```

REQUIRED: a blocked move's or hold's signature (P4, N10).

*field, declared at [`include/shulib/mechanism/kinds.hpp:753`](../../include/shulib/mechanism/kinds.hpp#L753).*

<a id="arm-currentcap"></a>

### `Arm::currentCap`

```cpp
CurrentCap currentCap{}
```

Optional: the most current each motor may draw, set in the motor at boot (CurrentCap; MECH-S-A).

*field, declared at [`include/shulib/mechanism/kinds.hpp:755`](../../include/shulib/mechanism/kinds.hpp#L755).*

<a id="arm-range"></a>

### `Arm::range`

```cpp
DegreeRange range{}
```

The travel limits, degrees. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:757`](../../include/shulib/mechanism/kinds.hpp#L757).*

<a id="arm-gravity"></a>

### `Arm::gravity`

```cpp
GravityTerm gravity{}
```

cosine(...) or noGravity. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:759`](../../include/shulib/mechanism/kinds.hpp#L759).*

<a id="arm-home"></a>

### `Arm::home`

```cpp
Homing home{}
```

restsAtStop / homeBySwitch / homeByStall / absoluteSensor / neverHomed. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:761`](../../include/shulib/mechanism/kinds.hpp#L761).*

<a id="arm-presets"></a>

### `Arm::presets`

```cpp
DegreePreset presets[kMaxPresets]{}
```

Optional: {"score", 95}, ... (six at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:763`](../../include/shulib/mechanism/kinds.hpp#L763).*

<a id="arm-unhomedvolts"></a>

### `Arm::unhomedVolts`

```cpp
units::Voltage unhomedVolts{0.0}
```

Optional: a held button's most before homing (0 = nothing moves it before homing).

*field, declared at [`include/shulib/mechanism/kinds.hpp:765`](../../include/shulib/mechanism/kinds.hpp#L765).*

<a id="arm-maxdownvolts"></a>

### `Arm::maxDownVolts`

```cpp
units::Voltage maxDownVolts{0.0}
```

Optional: the most volts anything drives it DOWN with (0 = the motor's 12 V; API 2.10).

*field, declared at [`include/shulib/mechanism/kinds.hpp:767`](../../include/shulib/mechanism/kinds.hpp#L767).*

<a id="arm-tuning"></a>

### `Arm::tuning`

```cpp
PositionTuning tuning{}
```

Optional: overrides of the starting values (header).

*field, declared at [`include/shulib/mechanism/kinds.hpp:769`](../../include/shulib/mechanism/kinds.hpp#L769).*

<a id="arm-operator-index"></a>

### `Arm::operator[]`

```cpp
[[nodiscard]] consteval PositionGoal operator[](const char* preset) const
```

A named preset: `arm["score"]`. A name the arm does not have FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:772`](../../include/shulib/mechanism/kinds.hpp#L772).*

<a id="arm-to"></a>

### `Arm::to`

```cpp
[[nodiscard]] consteval PositionGoal to(double degrees) const
```

A constant target in degrees: `arm.to(120)`. Past the travel limits FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:782`](../../include/shulib/mechanism/kinds.hpp#L782).*

<a id="arm-requires"></a>

### `Arm::requires`

```cpp
template <class T> requires(!std::is_arithmetic_v<T>) [[nodiscard]] consteval PositionGoal to(const T&) const
```

`arm.to(90_deg)`: refused BY NAME (`_deg` is a wrapping heading — 270 reads -90).

*function, declared at [`include/shulib/mechanism/kinds.hpp:790`](../../include/shulib/mechanism/kinds.hpp#L790).*

<a id="arm-clampedto"></a>

### `Arm::clampedTo`

```cpp
[[nodiscard]] constexpr PositionGoal clampedTo(double degrees) const noexcept
```

A target computed while the auton runs: clamped to the limits by the soft stop, and logged.

*function, declared at [`include/shulib/mechanism/kinds.hpp:796`](../../include/shulib/mechanism/kinds.hpp#L796).*

<a id="arm-spec"></a>

### `Arm::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec() const noexcept
```

This declaration as data.

*function, declared at [`include/shulib/mechanism/kinds.hpp:801`](../../include/shulib/mechanism/kinds.hpp#L801).*

<a id="struct-lift"></a>

## `struct Lift`

```cpp
struct Lift
```

LIFT — go to a height in INCHES and hold it (a spool lift, a four-bar, a DR4B, any measured shape). Members in the Arm's order, its shape after the gearing.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:839`](../../include/shulib/mechanism/kinds.hpp#L839).*

<a id="lift-name"></a>

### `Lift::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:841`](../../include/shulib/mechanism/kinds.hpp#L841).*

<a id="lift-motors"></a>

### `Lift::motors`

```cpp
Motors motors{}
```

Its motor roles.

*field, declared at [`include/shulib/mechanism/kinds.hpp:843`](../../include/shulib/mechanism/kinds.hpp#L843).*

<a id="lift-cartridge"></a>

### `Lift::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

Its motors' cartridge. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:845`](../../include/shulib/mechanism/kinds.hpp#L845).*

<a id="lift-gearing"></a>

### `Lift::gearing`

```cpp
Gearing gearing{}
```

REQUIRED: gears(12, 36), or direct.

*field, declared at [`include/shulib/mechanism/kinds.hpp:847`](../../include/shulib/mechanism/kinds.hpp#L847).*

<a id="lift-shape"></a>

### `Lift::shape`

```cpp
LiftShape shape{}
```

REQUIRED: spool(...), linkage(...) or measured({...}).

*field, declared at [`include/shulib/mechanism/kinds.hpp:849`](../../include/shulib/mechanism/kinds.hpp#L849).*

<a id="lift-anglesensor"></a>

### `Lift::angleSensor`

```cpp
Sensor angleSensor{}
```

Optional: rotationSensor() on the output shaft.

*field, declared at [`include/shulib/mechanism/kinds.hpp:851`](../../include/shulib/mechanism/kinds.hpp#L851).*

<a id="lift-safe"></a>

### `Lift::safe`

```cpp
config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0)
```

What every stop lands in. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:853`](../../include/shulib/mechanism/kinds.hpp#L853).*

<a id="lift-stall"></a>

### `Lift::stall`

```cpp
Stall stall{}
```

REQUIRED (P4, N10).

*field, declared at [`include/shulib/mechanism/kinds.hpp:855`](../../include/shulib/mechanism/kinds.hpp#L855).*

<a id="lift-currentcap"></a>

### `Lift::currentCap`

```cpp
CurrentCap currentCap{}
```

Optional: the most current each motor may draw, set in the motor at boot (CurrentCap; MECH-S-A) — how hard a string lift's string is pulled.

*field, declared at [`include/shulib/mechanism/kinds.hpp:858`](../../include/shulib/mechanism/kinds.hpp#L858).*

<a id="lift-range"></a>

### `Lift::range`

```cpp
InchRange range{}
```

The travel limits, inches. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:860`](../../include/shulib/mechanism/kinds.hpp#L860).*

<a id="lift-gravity"></a>

### `Lift::gravity`

```cpp
GravityTerm gravity{}
```

constantGravity(...) (or cosine for a linkage's arm, or noGravity). REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:862`](../../include/shulib/mechanism/kinds.hpp#L862).*

<a id="lift-home"></a>

### `Lift::home`

```cpp
Homing home{}
```

How it finds zero. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:864`](../../include/shulib/mechanism/kinds.hpp#L864).*

<a id="lift-presets"></a>

### `Lift::presets`

```cpp
InchPreset presets[kMaxPresets]{}
```

Optional: {"high", 22_in}, ... (six at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:866`](../../include/shulib/mechanism/kinds.hpp#L866).*

<a id="lift-unhomedvolts"></a>

### `Lift::unhomedVolts`

```cpp
units::Voltage unhomedVolts{0.0}
```

Optional: a held button's most before homing.

*field, declared at [`include/shulib/mechanism/kinds.hpp:868`](../../include/shulib/mechanism/kinds.hpp#L868).*

<a id="lift-maxdownvolts"></a>

### `Lift::maxDownVolts`

```cpp
units::Voltage maxDownVolts{0.0}
```

Optional: the most volts anything drives it DOWN with (0 = the motor's 12 V; API 2.10).

*field, declared at [`include/shulib/mechanism/kinds.hpp:870`](../../include/shulib/mechanism/kinds.hpp#L870).*

<a id="lift-tuning"></a>

### `Lift::tuning`

```cpp
PositionTuning tuning{}
```

Optional: overrides of the starting values.

*field, declared at [`include/shulib/mechanism/kinds.hpp:872`](../../include/shulib/mechanism/kinds.hpp#L872).*

<a id="lift-operator-index"></a>

### `Lift::operator[]`

```cpp
[[nodiscard]] consteval PositionGoal operator[](const char* preset) const
```

A named preset: `lift["high"]`. A name the lift does not have FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:875`](../../include/shulib/mechanism/kinds.hpp#L875).*

<a id="lift-to"></a>

### `Lift::to`

```cpp
[[nodiscard]] consteval PositionGoal to(units::Length height) const
```

A constant target height: `lift.to(18_in)`. Past the travel limits FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:885`](../../include/shulib/mechanism/kinds.hpp#L885).*

<a id="lift-to-2"></a>

### `Lift::to (overload 2)`

```cpp
template <class T> requires std::is_arithmetic_v<T> [[nodiscard]] consteval PositionGoal to(T) const
```

`lift.to(18)`: refused BY NAME (a height is inches: `lift.to(18_in)`).

*function, declared at [`include/shulib/mechanism/kinds.hpp:893`](../../include/shulib/mechanism/kinds.hpp#L893).*

<a id="lift-clampedto"></a>

### `Lift::clampedTo`

```cpp
[[nodiscard]] constexpr PositionGoal clampedTo(units::Length height) const noexcept
```

A height computed while the auton runs: clamped to the limits by the soft stop, and logged.

*function, declared at [`include/shulib/mechanism/kinds.hpp:899`](../../include/shulib/mechanism/kinds.hpp#L899).*

<a id="lift-spec"></a>

### `Lift::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec() const noexcept
```

This declaration as data.

*function, declared at [`include/shulib/mechanism/kinds.hpp:904`](../../include/shulib/mechanism/kinds.hpp#L904).*

<a id="struct-atspeed"></a>

## `struct AtSpeed`

```cpp
struct AtSpeed
```

"At speed": within `rpm` of the target for `lasting` (both the team's, P6). Made by within().

*struct, declared at [`include/shulib/mechanism/kinds.hpp:952`](../../include/shulib/mechanism/kinds.hpp#L952).*

<a id="atspeed-rpm"></a>

### `AtSpeed::rpm`

```cpp
double rpm = 0.0
```

The band, output rpm.

*field, declared at [`include/shulib/mechanism/kinds.hpp:954`](../../include/shulib/mechanism/kinds.hpp#L954).*

<a id="atspeed-lasting"></a>

### `AtSpeed::lasting`

```cpp
units::Time lasting{0.0}
```

How long it must stay inside it.

*field, declared at [`include/shulib/mechanism/kinds.hpp:956`](../../include/shulib/mechanism/kinds.hpp#L956).*

<a id="within"></a>

## `within`

```cpp
[[nodiscard]] constexpr AtSpeed within(double rpm, units::Time lasting) noexcept
```

`.atSpeed = within(60, 0.1_s)`: within 60 rpm of the target for 0.1 s.

*free function, declared at [`include/shulib/mechanism/kinds.hpp:959`](../../include/shulib/mechanism/kinds.hpp#L959).*

<a id="struct-flywheel"></a>

## `struct Flywheel`

```cpp
struct Flywheel
```

FLYWHEEL — spin at an RPM of its output and say when it is up to speed (the SPEED kind; re-check N11: the declaration's word is what a team builds).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:963`](../../include/shulib/mechanism/kinds.hpp#L963).*

<a id="flywheel-name"></a>

### `Flywheel::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:965`](../../include/shulib/mechanism/kinds.hpp#L965).*

<a id="flywheel-motors"></a>

### `Flywheel::motors`

```cpp
Motors motors{}
```

Its motor roles.

*field, declared at [`include/shulib/mechanism/kinds.hpp:967`](../../include/shulib/mechanism/kinds.hpp#L967).*

<a id="flywheel-cartridge"></a>

### `Flywheel::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

Its motors' cartridge. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:969`](../../include/shulib/mechanism/kinds.hpp#L969).*

<a id="flywheel-gearing"></a>

### `Flywheel::gearing`

```cpp
Gearing gearing{}
```

REQUIRED: gears(72, 12) = geared UP 6:1.

*field, declared at [`include/shulib/mechanism/kinds.hpp:971`](../../include/shulib/mechanism/kinds.hpp#L971).*

<a id="flywheel-anglesensor"></a>

### `Flywheel::angleSensor`

```cpp
Sensor angleSensor{}
```

Optional: rotationSensor() on the wheel — then it, not the motor encoders, is the speed.

*field, declared at [`include/shulib/mechanism/kinds.hpp:973`](../../include/shulib/mechanism/kinds.hpp#L973).*

<a id="flywheel-safe"></a>

### `Flywheel::safe`

```cpp
config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0)
```

What every stop lands in. REQUIRED.

*field, declared at [`include/shulib/mechanism/kinds.hpp:975`](../../include/shulib/mechanism/kinds.hpp#L975).*

<a id="flywheel-stall"></a>

### `Flywheel::stall`

```cpp
Stall stall{}
```

REQUIRED: a jammed wheel's signature (P4, N10).

*field, declared at [`include/shulib/mechanism/kinds.hpp:977`](../../include/shulib/mechanism/kinds.hpp#L977).*

<a id="flywheel-currentcap"></a>

### `Flywheel::currentCap`

```cpp
CurrentCap currentCap{}
```

Optional: the most current each motor may draw, set in the motor at boot (CurrentCap; MECH-S-A).

*field, declared at [`include/shulib/mechanism/kinds.hpp:979`](../../include/shulib/mechanism/kinds.hpp#L979).*

<a id="flywheel-atspeed"></a>

### `Flywheel::atSpeed`

```cpp
AtSpeed atSpeed{}
```

REQUIRED: within(60, 0.1_s).

*field, declared at [`include/shulib/mechanism/kinds.hpp:981`](../../include/shulib/mechanism/kinds.hpp#L981).*

<a id="flywheel-presets"></a>

### `Flywheel::presets`

```cpp
RpmPreset presets[kMaxPresets]{}
```

Its named speeds: {"shoot", 3000}, ... (six at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:983`](../../include/shulib/mechanism/kinds.hpp#L983).*

<a id="flywheel-tuning"></a>

### `Flywheel::tuning`

```cpp
SpeedTuning tuning{}
```

Optional: overrides of the starting values.

*field, declared at [`include/shulib/mechanism/kinds.hpp:985`](../../include/shulib/mechanism/kinds.hpp#L985).*

<a id="flywheel-freerpm"></a>

### `Flywheel::freeRpm`

```cpp
[[nodiscard]] constexpr double freeRpm() const noexcept
```

The fastest it can spin: the cartridge's free speed through the gearing, output rpm.

*function, declared at [`include/shulib/mechanism/kinds.hpp:988`](../../include/shulib/mechanism/kinds.hpp#L988).*

<a id="flywheel-operator-index"></a>

### `Flywheel::operator[]`

```cpp
[[nodiscard]] consteval SpeedGoal operator[](const char* preset) const
```

A named speed: `flywheel["shoot"]`. A name it does not have FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:992`](../../include/shulib/mechanism/kinds.hpp#L992).*

<a id="flywheel-at"></a>

### `Flywheel::at`

```cpp
[[nodiscard]] consteval SpeedGoal at(double rpm) const
```

A constant speed: `flywheel.at(3000)`. Past its free speed FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1002`](../../include/shulib/mechanism/kinds.hpp#L1002).*

<a id="flywheel-clampedto"></a>

### `Flywheel::clampedTo`

```cpp
[[nodiscard]] constexpr SpeedGoal clampedTo(double rpm) const noexcept
```

A speed computed while the auton runs: clamped to the free speed, and logged.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1009`](../../include/shulib/mechanism/kinds.hpp#L1009).*

<a id="flywheel-spec"></a>

### `Flywheel::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec() const noexcept
```

This declaration as data (its free speed derived, its starting values filled).

*function, declared at [`include/shulib/mechanism/kinds.hpp:1014`](../../include/shulib/mechanism/kinds.hpp#L1014).*

<a id="struct-levels"></a>

## `struct Levels`

```cpp
struct Levels
```

One position's line levels, as written: `{Low}` (one line) or `{High, Low}` (two). Records HOW MANY were written, so the wiring can refuse a position stating a different number of levels than it has lines (U2: a bool array cannot tell an unstated level from Low).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:1043`](../../include/shulib/mechanism/kinds.hpp#L1043).*

<a id="levels-l"></a>

### `Levels::l`

```cpp
bool l[kMaxLineRoles]{}
```

The levels, line 1 first.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1045`](../../include/shulib/mechanism/kinds.hpp#L1045).*

<a id="levels-count"></a>

### `Levels::count`

```cpp
std::uint8_t count = 0
```

How many were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1047`](../../include/shulib/mechanism/kinds.hpp#L1047).*

<a id="levels-toomany"></a>

### `Levels::tooMany`

```cpp
bool tooMany = false
```

More than kMaxLineRoles were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1049`](../../include/shulib/mechanism/kinds.hpp#L1049).*

<a id="levels-levels"></a>

### `Levels::Levels`

```cpp
constexpr Levels() noexcept = default
```

UNSET: no levels.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1051`](../../include/shulib/mechanism/kinds.hpp#L1051).*

<a id="levels-levels-2"></a>

### `Levels::Levels (overload 2)`

```cpp
consteval Levels(std::initializer_list<bool> list) noexcept
```

UNSET: no levels. `{High, Low}`.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1053`](../../include/shulib/mechanism/kinds.hpp#L1053).*

<a id="struct-namedposition"></a>

## `struct NamedPosition`

```cpp
struct NamedPosition
```

A pneumatic POSITION: its name, each line's level in it, and whether reaching it is a GRAB (or a RELEASE) a piece sensor confirms (Q2: with no sensor it is counted done after `moveTime`, reported UNCONFIRMED).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:1066`](../../include/shulib/mechanism/kinds.hpp#L1066).*

<a id="namedposition-name"></a>

### `NamedPosition::name`

```cpp
const char* name = nullptr
```

Its name (15 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1068`](../../include/shulib/mechanism/kinds.hpp#L1068).*

<a id="namedposition-levels"></a>

### `NamedPosition::levels`

```cpp
Levels levels{}
```

Its line levels.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1070`](../../include/shulib/mechanism/kinds.hpp#L1070).*

<a id="namedposition-confirm"></a>

### `NamedPosition::confirm`

```cpp
Confirm confirm = Confirm::None
```

`grabs`, `releases`, or nothing.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1072`](../../include/shulib/mechanism/kinds.hpp#L1072).*

<a id="struct-positions"></a>

## `struct Positions`

```cpp
struct Positions
```

A pneumatic mechanism's positions, in order (two or three): `.positions = {{"closed", {Low}, grabs}, {"open", {High}}}`.

*struct, declared at [`include/shulib/mechanism/kinds.hpp:1076`](../../include/shulib/mechanism/kinds.hpp#L1076).*

<a id="positions-p"></a>

### `Positions::p`

```cpp
NamedPosition p[kMaxPneumaticPositions]{}
```

The positions, in the order written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1078`](../../include/shulib/mechanism/kinds.hpp#L1078).*

<a id="positions-count"></a>

### `Positions::count`

```cpp
std::uint8_t count = 0
```

How many were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1080`](../../include/shulib/mechanism/kinds.hpp#L1080).*

<a id="positions-toomany"></a>

### `Positions::tooMany`

```cpp
bool tooMany = false
```

More than kMaxPneumaticPositions were written.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1082`](../../include/shulib/mechanism/kinds.hpp#L1082).*

<a id="positions-file"></a>

### `Positions::file`

```cpp
const char* file = nullptr
```

Where the list was written (an initializer list — see Motors::file; the spec uses `.name`'s).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1084`](../../include/shulib/mechanism/kinds.hpp#L1084).*

<a id="positions-line"></a>

### `Positions::line`

```cpp
std::uint32_t line = 0
```

That line.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1086`](../../include/shulib/mechanism/kinds.hpp#L1086).*

<a id="positions-positions"></a>

### `Positions::Positions`

```cpp
constexpr Positions() noexcept = default
```

UNSET: no positions.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1088`](../../include/shulib/mechanism/kinds.hpp#L1088).*

<a id="positions-positions-2"></a>

### `Positions::Positions (overload 2)`

```cpp
consteval Positions(std::initializer_list<NamedPosition> list, std::source_location at = std::source_location::current()) noexcept
```

UNSET: no positions. `{{"closed", {Low}, grabs}, {"open", {High}}}`.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1090`](../../include/shulib/mechanism/kinds.hpp#L1090).*

<a id="high"></a>

## `High`

```cpp
inline constexpr bool High = true
```

A line driven HIGH (on, `true` — the solenoid powered).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1103`](../../include/shulib/mechanism/kinds.hpp#L1103).*

<a id="low"></a>

## `Low`

```cpp
inline constexpr bool Low = false
```

A line driven LOW (off, `false` — the solenoid unpowered).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1105`](../../include/shulib/mechanism/kinds.hpp#L1105).*

<a id="grabs"></a>

## `grabs`

```cpp
inline constexpr Confirm grabs = Confirm::Sees
```

Reaching this position GRABS a piece: a piece sensor confirms it; with none, UNCONFIRMED and counted done (Q2 (c)).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1108`](../../include/shulib/mechanism/kinds.hpp#L1108).*

<a id="releases"></a>

## `releases`

```cpp
inline constexpr Confirm releases = Confirm::StopsSeeing
```

Reaching this position lets a piece GO: a piece sensor confirms it is gone.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1110`](../../include/shulib/mechanism/kinds.hpp#L1110).*

<a id="struct-pneumatic"></a>

## `struct Pneumatic`

```cpp
struct Pneumatic
```

PNEUMATIC — two or three named positions, each the team's line levels (Q13 (a)).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:1113`](../../include/shulib/mechanism/kinds.hpp#L1113).*

<a id="pneumatic-name"></a>

### `Pneumatic::name`

```cpp
Name name{}
```

Its name (23 characters at most).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1115`](../../include/shulib/mechanism/kinds.hpp#L1115).*

<a id="pneumatic-positions"></a>

### `Pneumatic::positions`

```cpp
Positions positions{}
```

REQUIRED: {{"closed", {Low}, grabs}, {"open", {High}}} — one level per wired line.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1117`](../../include/shulib/mechanism/kinds.hpp#L1117).*

<a id="pneumatic-start"></a>

### `Pneumatic::start`

```cpp
const char* start = nullptr
```

REQUIRED: the position at program start (a position's name).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1119`](../../include/shulib/mechanism/kinds.hpp#L1119).*

<a id="pneumatic-safe"></a>

### `Pneumatic::safe`

```cpp
const char* safe = nullptr
```

REQUIRED: the position the end-of-run guard puts it in.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1121`](../../include/shulib/mechanism/kinds.hpp#L1121).*

<a id="pneumatic-movetime"></a>

### `Pneumatic::moveTime`

```cpp
units::Time moveTime{0.0}
```

REQUIRED: how long the cylinder takes (no default, P6).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1123`](../../include/shulib/mechanism/kinds.hpp#L1123).*

<a id="pneumatic-piecesensor"></a>

### `Pneumatic::pieceSensor`

```cpp
Sensor pieceSensor{}
```

Optional: confirms a `grabs` / `releases` position.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1125`](../../include/shulib/mechanism/kinds.hpp#L1125).*

<a id="pneumatic-confirmwithin"></a>

### `Pneumatic::confirmWithin`

```cpp
units::Time confirmWithin{0.0}
```

With a piece sensor: how long after moveTime the confirmation may take.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1127`](../../include/shulib/mechanism/kinds.hpp#L1127).*

<a id="pneumatic-changeat"></a>

### `Pneumatic::changeAt`

```cpp
const char* changeAt = nullptr
```

Generated robot files only: the positions' JSON path (G3a).

*field, declared at [`include/shulib/mechanism/kinds.hpp:1129`](../../include/shulib/mechanism/kinds.hpp#L1129).*

<a id="pneumatic-to"></a>

### `Pneumatic::to`

```cpp
template <class T> [[nodiscard]] consteval PneumaticGoal to(const T&) const
```

`claw.to(...)`: refused BY NAME (a position is named: claw["open"]).

*function, declared at [`include/shulib/mechanism/kinds.hpp:1133`](../../include/shulib/mechanism/kinds.hpp#L1133).*

<a id="pneumatic-operator-index"></a>

### `Pneumatic::operator[]`

```cpp
[[nodiscard]] consteval PneumaticGoal operator[](const char* position) const
```

A named position: `claw["closed"]`. A name it does not have FAILS THE BUILD.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1138`](../../include/shulib/mechanism/kinds.hpp#L1138).*

<a id="pneumatic-spec"></a>

### `Pneumatic::spec`

```cpp
[[nodiscard]] constexpr MechanismSpec spec(std::uint8_t lines = 1) const noexcept
```

This declaration as data, with `lines` 3-wire lines — the WIRING says how many (lines('A') or lines('B', 'C')).

*function, declared at [`include/shulib/mechanism/kinds.hpp:1150`](../../include/shulib/mechanism/kinds.hpp#L1150).*

<a id="arm"></a>

## `Arm`

```cpp
using mechanism::Arm
```

The ARM declaration (degrees, held).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1193`](../../include/shulib/mechanism/kinds.hpp#L1193).*

<a id="lift"></a>

## `Lift`

```cpp
using mechanism::Lift
```

The LIFT declaration (inches, held).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1195`](../../include/shulib/mechanism/kinds.hpp#L1195).*

<a id="pneumatic"></a>

## `Pneumatic`

```cpp
using mechanism::Pneumatic
```

The PNEUMATIC declaration (named positions).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1197`](../../include/shulib/mechanism/kinds.hpp#L1197).*

<a id="power"></a>

## `Power`

```cpp
using mechanism::Power
```

The POWER declaration (volts).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1199`](../../include/shulib/mechanism/kinds.hpp#L1199).*

<a id="flywheel"></a>

## `Flywheel`

```cpp
using mechanism::Flywheel
```

The FLYWHEEL declaration (output rpm, "at speed").

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1201`](../../include/shulib/mechanism/kinds.hpp#L1201).*

<a id="forward-2"></a>

## `Forward (overload 2)`

```cpp
using mechanism::Forward
```

A motor mounted the + way.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1203`](../../include/shulib/mechanism/kinds.hpp#L1203).*

<a id="reversed-2"></a>

## `Reversed (overload 2)`

```cpp
using mechanism::Reversed
```

A motor mounted the other way.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1205`](../../include/shulib/mechanism/kinds.hpp#L1205).*

<a id="rotationsensor-2"></a>

## `rotationSensor (overload 2)`

```cpp
using mechanism::rotationSensor
```

An angle sensor: rotationSensor() / rotationSensor(Reversed).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1207`](../../include/shulib/mechanism/kinds.hpp#L1207).*

<a id="distancesensor-2"></a>

## `distanceSensor (overload 2)`

```cpp
using mechanism::distanceSensor
```

A distance piece sensor: distanceSensor(2_in).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1209`](../../include/shulib/mechanism/kinds.hpp#L1209).*

<a id="opticalsensor-2"></a>

## `opticalSensor (overload 2)`

```cpp
using mechanism::opticalSensor
```

An optical piece sensor: opticalSensor(0.5).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1211`](../../include/shulib/mechanism/kinds.hpp#L1211).*

<a id="limitswitch-2"></a>

## `limitSwitch (overload 2)`

```cpp
using mechanism::limitSwitch
```

A limit-switch piece sensor: limitSwitch().

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1213`](../../include/shulib/mechanism/kinds.hpp#L1213).*

<a id="stall"></a>

## `Stall`

```cpp
using mechanism::Stall
```

A stall signature: {.amps, .belowMotorRpm, .lasting}.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1215`](../../include/shulib/mechanism/kinds.hpp#L1215).*

<a id="cosine-2"></a>

## `cosine (overload 2)`

```cpp
using mechanism::cosine
```

An arm's gravity: cosine(1.6_volt, 45).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1217`](../../include/shulib/mechanism/kinds.hpp#L1217).*

<a id="constantgravity-2"></a>

## `constantGravity (overload 2)`

```cpp
using mechanism::constantGravity
```

A lift's gravity: constantGravity(1.0_volt).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1219`](../../include/shulib/mechanism/kinds.hpp#L1219).*

<a id="nogravity-2"></a>

## `noGravity (overload 2)`

```cpp
using mechanism::noGravity
```

No gravity to hold.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1221`](../../include/shulib/mechanism/kinds.hpp#L1221).*

<a id="restsatstop-3"></a>

## `restsAtStop (overload 3)`

```cpp
using mechanism::restsAtStop
```

Homing: it rests on its stop at program start.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1223`](../../include/shulib/mechanism/kinds.hpp#L1223).*

<a id="homebyswitch-3"></a>

## `homeBySwitch (overload 3)`

```cpp
using mechanism::homeBySwitch
```

Homing: push until its switch is pressed.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1225`](../../include/shulib/mechanism/kinds.hpp#L1225).*

<a id="homebystall-3"></a>

## `homeByStall (overload 3)`

```cpp
using mechanism::homeByStall
```

Homing: push until it stalls.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1227`](../../include/shulib/mechanism/kinds.hpp#L1227).*

<a id="absolutesensor-2"></a>

## `absoluteSensor (overload 2)`

```cpp
using mechanism::absoluteSensor
```

Homing: an absolute angle sensor (HA-186).

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1229`](../../include/shulib/mechanism/kinds.hpp#L1229).*

<a id="neverhomed-2"></a>

## `neverHomed (overload 2)`

```cpp
using mechanism::neverHomed
```

Never homed.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1231`](../../include/shulib/mechanism/kinds.hpp#L1231).*

<a id="spool-2"></a>

## `spool (overload 2)`

```cpp
using mechanism::spool
```

A lift's shape: a spool.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1233`](../../include/shulib/mechanism/kinds.hpp#L1233).*

<a id="linkage-2"></a>

## `linkage (overload 2)`

```cpp
using mechanism::linkage
```

A lift's shape: a four-bar or a DR4B.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1235`](../../include/shulib/mechanism/kinds.hpp#L1235).*

<a id="measured-2"></a>

## `measured (overload 2)`

```cpp
using mechanism::measured
```

A lift's shape: measured pairs.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1237`](../../include/shulib/mechanism/kinds.hpp#L1237).*

<a id="heightat"></a>

## `HeightAt`

```cpp
using mechanism::HeightAt
```

One measured pair.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1239`](../../include/shulib/mechanism/kinds.hpp#L1239).*

<a id="within-2"></a>

## `within (overload 2)`

```cpp
using mechanism::within
```

A flywheel's "at speed" band.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1241`](../../include/shulib/mechanism/kinds.hpp#L1241).*

<a id="high-2"></a>

## `High (overload 2)`

```cpp
using mechanism::High
```

A line driven high.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1243`](../../include/shulib/mechanism/kinds.hpp#L1243).*

<a id="low-2"></a>

## `Low (overload 2)`

```cpp
using mechanism::Low
```

A line driven low.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1245`](../../include/shulib/mechanism/kinds.hpp#L1245).*

<a id="grabs-2"></a>

## `grabs (overload 2)`

```cpp
using mechanism::grabs
```

A position that grabs.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1247`](../../include/shulib/mechanism/kinds.hpp#L1247).*

<a id="releases-2"></a>

## `releases (overload 2)`

```cpp
using mechanism::releases
```

A position that releases.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1249`](../../include/shulib/mechanism/kinds.hpp#L1249).*

<a id="gears-2"></a>

## `gears (overload 2)`

```cpp
using mechanism::gears
```

A gear pair, driving then driven.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1251`](../../include/shulib/mechanism/kinds.hpp#L1251).*

<a id="direct-2"></a>

## `direct (overload 2)`

```cpp
using mechanism::direct
```

No gearing.

*type alias, declared at [`include/shulib/mechanism/kinds.hpp:1253`](../../include/shulib/mechanism/kinds.hpp#L1253).*

<a id="red"></a>

## `Red`

```cpp
inline constexpr hal::MotorCartridge Red = hal::MotorCartridge::Red
```

The RED cartridge (100 rpm).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1255`](../../include/shulib/mechanism/kinds.hpp#L1255).*

<a id="green"></a>

## `Green`

```cpp
inline constexpr hal::MotorCartridge Green = hal::MotorCartridge::Green
```

The GREEN cartridge (200 rpm).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1257`](../../include/shulib/mechanism/kinds.hpp#L1257).*

<a id="blue"></a>

## `Blue`

```cpp
inline constexpr hal::MotorCartridge Blue = hal::MotorCartridge::Blue
```

The BLUE cartridge (600 rpm).

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1259`](../../include/shulib/mechanism/kinds.hpp#L1259).*

<a id="struct-brakeword"></a>

## `struct BrakeWord`

```cpp
struct BrakeWord
```

ONE word per brake mode for both places a team writes one: a mechanism's `.safe` (a config::SafeBrakeMode) and the driver loop's `.releaseBrakeMode` (a hal::BrakeMode) — so a driver file needs no `hal::BrakeMode::Coast` beside the robot file's `Coast` (N11).

*struct, declared at [`include/shulib/mechanism/kinds.hpp:1263`](../../include/shulib/mechanism/kinds.hpp#L1263).*

<a id="brakeword-safe"></a>

### `BrakeWord::safe`

```cpp
config::SafeBrakeMode safe
```

As a mechanism's declared safe state.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1265`](../../include/shulib/mechanism/kinds.hpp#L1265).*

<a id="brakeword-drive"></a>

### `BrakeWord::drive`

```cpp
hal::BrakeMode drive
```

As the drive's brake mode.

*field, declared at [`include/shulib/mechanism/kinds.hpp:1267`](../../include/shulib/mechanism/kinds.hpp#L1267).*

<a id="brakeword-operator-config-safebrakemode"></a>

### `BrakeWord::operator config::SafeBrakeMode`

```cpp
constexpr operator config::SafeBrakeMode() const noexcept
```

Read as a mechanism's `.safe`.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1269`](../../include/shulib/mechanism/kinds.hpp#L1269).*

<a id="brakeword-operator-hal-brakemode"></a>

### `BrakeWord::operator hal::BrakeMode`

```cpp
constexpr operator hal::BrakeMode() const noexcept
```

Read as a drive brake mode.

*function, declared at [`include/shulib/mechanism/kinds.hpp:1271`](../../include/shulib/mechanism/kinds.hpp#L1271).*

<a id="hold"></a>

## `Hold`

```cpp
inline constexpr BrakeWord Hold{config::SafeBrakeMode::Hold, hal::BrakeMode::Hold}
```

HOLD: drives back to where it stopped — the one that keeps a load up.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1274`](../../include/shulib/mechanism/kinds.hpp#L1274).*

<a id="brake"></a>

## `Brake`

```cpp
inline constexpr BrakeWord Brake{config::SafeBrakeMode::Brake, hal::BrakeMode::Brake}
```

BRAKE: resists, but does not return to where it stopped.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1276`](../../include/shulib/mechanism/kinds.hpp#L1276).*

<a id="coast"></a>

## `Coast`

```cpp
inline constexpr BrakeWord Coast{config::SafeBrakeMode::Coast, hal::BrakeMode::Coast}
```

COAST: undriven — for a mechanism that must never sit at stall current.

*constant, declared at [`include/shulib/mechanism/kinds.hpp:1278`](../../include/shulib/mechanism/kinds.hpp#L1278).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 71 lines, click to expand</summary>

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
 sensor), what it does STOPPED (safe, stall, and — since MECH-S-A — currentCap), where it GOES
 (range, gravity, home, presets — or at-speed and presets), the extras (unhomedVolts, tuning). Power's
 maxVolts comes after its currentCap. Out of order is a plain C++ error
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
 robot later" (N7). They are HA-182 (registered by MECH-A3, 2026-10-15, with the behaviours that run
 them — mechanism/service.hpp), INVENTED until each mechanism is tuned on the robot;
 startingValuesLine() (wiring.hpp) words them per mechanism for a boot line — the programs print it
 since the core's fifth part (MECH-A5) — so nobody mistakes them for measurements.
 Each can be stated instead, one by one (`.tuning`). (The service's own integral clamps and window
 are HA-182 too, but LIBRARY constants: no `.tuning` field, not on that line — service.hpp.) No time
 limit is ever a default: every step states its own.

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
