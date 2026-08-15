<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/motor.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motor.hpp`

ProsMotor — IMotor over pros::Motor.

This header declares **2** types (13 members).

Extracted from [`include/shulib/hal/pros/motor.hpp`](../../include/shulib/hal/pros/motor.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class MotorGearset`](#enum-class-motorgearset)
  - [`Red`](#motorgearset-red)
  - [`Green`](#motorgearset-green)
  - [`Blue`](#motorgearset-blue)
- [`class ProsMotor`](#class-prosmotor)
  - [`ProsMotor`](#prosmotor-prosmotor)
  - [`setVoltage`](#prosmotor-setvoltage)
  - [`commandedVoltage`](#prosmotor-commandedvoltage)
  - [`setBrakeMode`](#prosmotor-setbrakemode)
  - [`brakeMode`](#prosmotor-brakemode)
  - [`position`](#prosmotor-position)
  - [`velocity`](#prosmotor-velocity)
  - [`current`](#prosmotor-current)
  - [`temperature`](#prosmotor-temperature)
  - [`faultedReads`](#prosmotor-faultedreads)

<a id="enum-class-motorgearset"></a>

## `enum class MotorGearset`

```cpp
enum class MotorGearset
```

The drive cartridge in the physical motor — a construction fact the caller must state (see header: velocity scaling is gearset-dependent, so there is deliberately no default).

*enum class, declared at [`include/shulib/hal/pros/motor.hpp:69`](../../include/shulib/hal/pros/motor.hpp#L69).*

<a id="motorgearset-red"></a>

### `MotorGearset::Red`

```cpp
Red
```

36:1, 100 RPM

*enumerator, declared at [`include/shulib/hal/pros/motor.hpp:70`](../../include/shulib/hal/pros/motor.hpp#L70).*

<a id="motorgearset-green"></a>

### `MotorGearset::Green`

```cpp
Green
```

18:1, 200 RPM

*enumerator, declared at [`include/shulib/hal/pros/motor.hpp:71`](../../include/shulib/hal/pros/motor.hpp#L71).*

<a id="motorgearset-blue"></a>

### `MotorGearset::Blue`

```cpp
Blue
```

6:1, 600 RPM

*enumerator, declared at [`include/shulib/hal/pros/motor.hpp:72`](../../include/shulib/hal/pros/motor.hpp#L72).*

<a id="class-prosmotor"></a>

## `class ProsMotor`

```cpp
class ProsMotor final : public IMotor
```

IMotor over a real pros::Motor — the adapter that converts V5 units to canonical ones exactly once, at the edge. Three contracts a caller should know: the constructor CONFIGURES the device (degrees + the stated gearset) and reads both back, so a miswired port fails at boot rather than mid-match; a NEGATIVE port reverses the motor inside PROS and this adapter never negates on top of that; and the four MEASUREMENT reads — position(), velocity(), current(), temperature() — screen the PROS_ERR / PROS_ERR_F sentinels by HOLDING THE LAST GOOD VALUE rather than propagating a sentinel or substituting zero, because a frozen reading is what the loop's odometry cross-checks are built to notice while a zeroed encoder would look exactly like a robot that stopped. faultedReads() counts those four and only those four. The other two accessors sit OUTSIDE that scheme: commandedVoltage() is a local mirror of the last applied command and touches no device, and brakeMode() does screen the device's `invalid` — to the last COMMANDED mode — but without counting it, so a port whose brake-mode read fails on every tick still reports faultedReads() == 0. Read the counter as a signal about the sensor path, not about the port as a whole. This adapter does not raise faults; raising is the loop layer's policy.

*class, declared at [`include/shulib/hal/pros/motor.hpp:90`](../../include/shulib/hal/pros/motor.hpp#L90).*

<a id="prosmotor-prosmotor"></a>

### `ProsMotor::ProsMotor`

```cpp
ProsMotor(std::int8_t port, MotorGearset gearset)
```

`port`: 1..21, NEGATIVE to reverse (PROS applies the reversal — once). `gearset`: the physical cartridge color. The ctor configures the device (degrees + gearset) and READS BOTH BACK — a disagreeing device raises SHULIB_PRECONDITION (header: why boot-loud beats match-silent).

*function, declared at [`include/shulib/hal/pros/motor.hpp:96`](../../include/shulib/hal/pros/motor.hpp#L96).*

<a id="prosmotor-setvoltage"></a>

### `ProsMotor::setVoltage`

```cpp
void setVoltage(units::Voltage volts) override
```

Clamp to ±kMaxMotorVoltage, REJECT non-finite (never coerce — L4), send as millivolts (HA-94). commandedVoltage() reflects the value APPLIED.

*function, declared at [`include/shulib/hal/pros/motor.hpp:122`](../../include/shulib/hal/pros/motor.hpp#L122).*

<a id="prosmotor-commandedvoltage"></a>

### `ProsMotor::commandedVoltage`

```cpp
[[nodiscard]] units::Voltage commandedVoltage() const override
```

The voltage this adapter last APPLIED, after the ±kMaxMotorVoltage clamp. A locally mirrored value computed by the same clamp as the millivolt command, NOT a read-back from the device — so it can never disagree with what went on the wire, and it never faults. 0 V until the first setVoltage().

*function, declared at [`include/shulib/hal/pros/motor.hpp:133`](../../include/shulib/hal/pros/motor.hpp#L133).*

<a id="prosmotor-setbrakemode"></a>

### `ProsMotor::setBrakeMode`

```cpp
void setBrakeMode(BrakeMode mode) override
```

Send the brake mode to the device AND remember it: the remembered value is what brakeMode() falls back to when the device's own read comes back invalid (T7). Only that FALLBACK starts at Coast: the ctor configures encoder units and gearing but never a brake mode, and brakeMode() reads the DEVICE first — so until this is called at least once, the value you observe is whatever mode the motor is still holding from an earlier program, which is the persistent-device-state trap the header describes for encoder units. Call this during setup if the mode has to be known rather than inherited.

*function, declared at [`include/shulib/hal/pros/motor.hpp:142`](../../include/shulib/hal/pros/motor.hpp#L142).*

<a id="prosmotor-brakemode"></a>

### `ProsMotor::brakeMode`

```cpp
[[nodiscard]] BrakeMode brakeMode() const override
```

Reads the DEVICE back and maps to the canonical enum; an unreadable device (MotorBrake::invalid) holds the last commanded mode (T7).

*function, declared at [`include/shulib/hal/pros/motor.hpp:149`](../../include/shulib/hal/pros/motor.hpp#L149).*

<a id="prosmotor-position"></a>

### `ProsMotor::position`

```cpp
[[nodiscard]] units::AngleDim position() const override
```

Cumulative output-shaft radians (never wrapped). Sentinel-screened: PROS_ERR_F holds the last good value (header, T7).

*function, declared at [`include/shulib/hal/pros/motor.hpp:160`](../../include/shulib/hal/pros/motor.hpp#L160).*

<a id="prosmotor-velocity"></a>

### `ProsMotor::velocity`

```cpp
[[nodiscard]] units::AngularVelocity velocity() const override
```

Measured output-shaft angular velocity, canonical rad/s, converted from the device's RPM (HA-96). This is the DEVICE's reported velocity — this adapter never differentiates position() to get it. Sentinel-screened like position(): PROS_ERR_F holds the last good value and counts a faulted read; 0 rad/s before the first successful read.

*function, declared at [`include/shulib/hal/pros/motor.hpp:174`](../../include/shulib/hal/pros/motor.hpp#L174).*

<a id="prosmotor-current"></a>

### `ProsMotor::current`

```cpp
[[nodiscard]] units::Current current() const override
```

Canonical amperes. PROS_ERR (INT32_MAX) is IN-BAND for the int32 mA read — only this adapter can screen it (motor_conversion.hpp note).

*function, declared at [`include/shulib/hal/pros/motor.hpp:186`](../../include/shulib/hal/pros/motor.hpp#L186).*

<a id="prosmotor-temperature"></a>

### `ProsMotor::temperature`

```cpp
[[nodiscard]] double temperature() const override
```

Motor temperature in degrees Celsius — already canonical, so this is the one reader that applies no conversion. Sentinel-screened like the others (non-finite holds the last good value and counts a faulted read), but note the seed: before ANY successful read this returns 20 °C, not 0, so a port that has never answered reports a room-temperature motor. Check faultedReads() before trusting it as a thermal signal.

*function, declared at [`include/shulib/hal/pros/motor.hpp:201`](../../include/shulib/hal/pros/motor.hpp#L201).*

<a id="prosmotor-faultedreads"></a>

### `ProsMotor::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many of the four MEASUREMENT reads (position/velocity/current/temperature) were screened to last-good — cumulative for the life of the object, never reset. T7 observability: telemetry and the loop's health policy can see a flaky port without this seam growing a validity channel F4 does not have. brakeMode()'s own screening is deliberately NOT tallied here, so 0 does not mean the port is healthy.

*function, declared at [`include/shulib/hal/pros/motor.hpp:216`](../../include/shulib/hal/pros/motor.hpp#L216).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 46 lines, click to expand</summary>

```text

 ProsMotor — IMotor over pros::Motor (chunk R1a).

 BINDS:
  * move_voltage(mV)          ← setVoltage()   (motor_conversion.hpp, HA-94)
  * get_position() [degrees]  → position()     (motorPositionDegToCanonical, HA-95)
  * get_actual_velocity() RPM → velocity()     (motorRpmToCanonical, HA-96)
  * get_current_draw() mA     → current()      (motorMilliampsToCanonical, HA-97)
  * get_temperature() °C      → temperature()  (identity — already canonical)
  * set/get_brake_mode        ↔ setBrakeMode()/brakeMode() (mapped both ways below)

 WHY THE CTOR DEMANDS A GEARSET AND SETS UNITS EXPLICITLY (trap A, HA-98):
 pros::Motor's ctor defaults are MotorGears::invalid / MotorUnits::invalid,
 which mean "leave the device as it is" — get_position() then returns
 whatever encoder units that motor's firmware was last told, possibly by a
 different program on a different day. A motor left in rotations returns
 1/360 of what this adapter believes, odometry is silently wrong by 360×,
 and nothing crashes. So this ctor:
   1. passes an EXPLICIT gearset (no default parameter here — the cartridge
      color is a physical fact of the built robot; velocity scaling depends
      on it, so the adapter must not guess), and
   2. explicitly sets MotorUnits::degrees, and
   3. READS BOTH BACK and raises SHULIB_PRECONDITION if the device disagrees
      — a motor that ignores configuration is a miswired robot, and finding
      that at boot beats finding it mid-match.

 REVERSAL: a NEGATIVE port reverses the motor, applied by PROS itself —
 exactly once, there. This adapter never negates on top.

 SENTINELS (T7): IMotor has no validity channel, so PROS_ERR / PROS_ERR_F
 from a read is screened HERE: hold the last good value, never propagate
 (breaks the F4 finiteness contract A3's hostility models as a bug), never
 substitute zero (a zeroed encoder reads as "the robot stopped" — exactly
 the dead-encoder runaway the loop's ODO_STUCK cross-check exists to catch,
 and a zero would make it plausible instead of visible). A frozen position
 is what the loop's wheels-spin-but-no-motion cross-check is DESIGNED to
 see (health_monitor.hpp odomStalled); faultedReads() is exposed for
 telemetry. Raising the fault itself stays with the loop layer — hal/ is
 below diag/ and raising is policy (health_monitor.hpp header).

 NON-FINITE COMMAND (L4): rejected via SHULIB_PRECONDITION, exactly like
 FakeMotor (fake_motor.hpp:18-24) — NOT coerced to 0. The measurement
 prototype for this brief made that mistake; it compiles, passes a clamp
 test, and turns a programming error into a robot that coasts.

 HA register: HA-94..HA-98 (docs/hardware-assumptions.md).
```

</details>
