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

*enum class, declared at [`include/shulib/hal/pros/motor.hpp:114`](../../include/shulib/hal/pros/motor.hpp#L114).*

<a id="motorgearset-red"></a>

### `MotorGearset::Red`

```cpp
Red
```

36:1, 100 RPM

*enumerator, declared at [`include/shulib/hal/pros/motor.hpp:115`](../../include/shulib/hal/pros/motor.hpp#L115).*

<a id="motorgearset-green"></a>

### `MotorGearset::Green`

```cpp
Green
```

18:1, 200 RPM

*enumerator, declared at [`include/shulib/hal/pros/motor.hpp:116`](../../include/shulib/hal/pros/motor.hpp#L116).*

<a id="motorgearset-blue"></a>

### `MotorGearset::Blue`

```cpp
Blue
```

6:1, 600 RPM

*enumerator, declared at [`include/shulib/hal/pros/motor.hpp:117`](../../include/shulib/hal/pros/motor.hpp#L117).*

<a id="class-prosmotor"></a>

## `class ProsMotor`

```cpp
class ProsMotor final : public IMotor
```

IMotor over a real pros::Motor — the adapter that converts V5 units to canonical ones exactly once, at the edge. Four contracts a caller should know: the constructor CONFIGURES the device (degrees + the stated gearset) and reads both back, so a miswired port fails at boot rather than mid-match; a NEGATIVE port reverses the motor inside PROS and this adapter never negates on top of that; a ZERO command under a Brake or Hold mode that this program commanded is sent as PROS brake() — the one call that applies a mode — while Coast, or no commanded mode, stays a 0 V command (header: THE STOP; before DEFECTS2 every zero was 0 V and no mode ever reached the motor); and the five screened READS — position(), velocity(), current(), temperature() and brakeMode() — handle the PROS_ERR / PROS_ERR_F / `invalid` sentinels by HOLDING THE LAST GOOD VALUE (for brakeMode(), the last mode this adapter commanded or seeded) rather than propagating a sentinel or substituting zero, because a frozen reading is what the loop's odometry cross-checks are built to notice while a zeroed encoder would look exactly like a robot that stopped. faultedReads() counts every one of those five screens. commandedVoltage() sits outside the scheme: a local mirror of the last applied command that touches no device. This adapter does not raise faults; raising is the loop layer's policy.

*class, declared at [`include/shulib/hal/pros/motor.hpp:136`](../../include/shulib/hal/pros/motor.hpp#L136).*

<a id="prosmotor-prosmotor"></a>

### `ProsMotor::ProsMotor`

```cpp
ProsMotor(std::int8_t port, MotorGearset gearset)
```

`port`: 1..21, NEGATIVE to reverse (PROS applies the reversal — once). `gearset`: the physical cartridge color. The ctor configures the device (degrees + gearset) and READS BOTH BACK — a disagreeing device raises SHULIB_PRECONDITION (header: why boot-loud beats match-silent).

*function, declared at [`include/shulib/hal/pros/motor.hpp:142`](../../include/shulib/hal/pros/motor.hpp#L142).*

<a id="prosmotor-setvoltage"></a>

### `ProsMotor::setVoltage`

```cpp
void setVoltage(units::Voltage volts) override
```

Clamp to ±kMaxMotorVoltage, REJECT non-finite (never coerce — L4), send as millivolts (HA-94). commandedVoltage() is the clamped volts, UNROUNDED, however the command was sent — so a sub-millivolt command reads back as itself (0.0004 V) although the wire sent a stop. When the millivolts SENT are 0 and this program's last setBrakeMode() was Brake or Hold, the stop goes out as PROS brake() instead of move_voltage(0) — once, on the way INTO the stop; a zero repeated under the same mode sends nothing more (header: THE STOP, HA-142/HA-144). Coast, or no mode commanded yet, sends move_voltage(0) every time, as before.

*function, declared at [`include/shulib/hal/pros/motor.hpp:174`](../../include/shulib/hal/pros/motor.hpp#L174).*

<a id="prosmotor-commandedvoltage"></a>

### `ProsMotor::commandedVoltage`

```cpp
[[nodiscard]] units::Voltage commandedVoltage() const override
```

The voltage this adapter last APPLIED, after the ±kMaxMotorVoltage clamp. A locally mirrored value computed by the same clamp as the millivolt command, NOT a read-back from the device — so it can never disagree with what went on the wire, and it never faults. 0 V until the first setVoltage().

*function, declared at [`include/shulib/hal/pros/motor.hpp:192`](../../include/shulib/hal/pros/motor.hpp#L192).*

<a id="prosmotor-setbrakemode"></a>

### `ProsMotor::setBrakeMode`

```cpp
void setBrakeMode(BrakeMode mode) override
```

Send the brake mode to the device AND remember it twice: as the value brakeMode() falls back to when the device's own read comes back invalid (T7), and as the mode THIS PROGRAM commanded, which is what decides how a zero command is sent (header: THE STOP). If the last command was a zero, the stop is RE-ISSUED under the new mode — brake() for Brake or Hold (unless already stopped under that very mode), move_voltage(0) for Coast when the motor was in a brake() stop — so mode-then-zero and zero-then-mode end the same way. Whether the port ACCEPTED the write is kept as well: a refused one is re-sent before the next brake(), which stops under the device's register (header: THE STOP). The ctor configures encoder units and gearing but never a brake mode, and brakeMode() reads the DEVICE first — so until this is called at least once, the value you observe is whatever mode the motor is still holding from an earlier program (the persistent-device- state trap the header describes for encoder units), and a zero command is sent as 0 V whatever that inherited mode is. Call this during setup if the mode has to be applied rather than inherited.

*function, declared at [`include/shulib/hal/pros/motor.hpp:208`](../../include/shulib/hal/pros/motor.hpp#L208).*

<a id="prosmotor-brakemode"></a>

### `ProsMotor::brakeMode`

```cpp
[[nodiscard]] BrakeMode brakeMode() const override
```

Reads the DEVICE back and maps to the canonical enum; an unreadable device (MotorBrake::invalid) holds the last commanded mode — or, before any command, the mode the ctor read from the device (T7) — and COUNTS a faulted read, like the other four screens. This reports the device's configured mode; it says nothing about whether the motor is currently stopped under it (header: THE STOP).

*function, declared at [`include/shulib/hal/pros/motor.hpp:222`](../../include/shulib/hal/pros/motor.hpp#L222).*

<a id="prosmotor-position"></a>

### `ProsMotor::position`

```cpp
[[nodiscard]] units::AngleDim position() const override
```

Cumulative output-shaft radians (never wrapped). Sentinel-screened: PROS_ERR_F holds the last good value (header, T7).

*function, declared at [`include/shulib/hal/pros/motor.hpp:233`](../../include/shulib/hal/pros/motor.hpp#L233).*

<a id="prosmotor-velocity"></a>

### `ProsMotor::velocity`

```cpp
[[nodiscard]] units::AngularVelocity velocity() const override
```

Measured output-shaft angular velocity, canonical rad/s, converted from the device's RPM (HA-96). This is the DEVICE's reported velocity — this adapter never differentiates position() to get it. Sentinel-screened like position(): PROS_ERR_F holds the last good value and counts a faulted read; 0 rad/s before the first successful read.

*function, declared at [`include/shulib/hal/pros/motor.hpp:247`](../../include/shulib/hal/pros/motor.hpp#L247).*

<a id="prosmotor-current"></a>

### `ProsMotor::current`

```cpp
[[nodiscard]] units::Current current() const override
```

Canonical amperes. PROS_ERR (INT32_MAX) is IN-BAND for the int32 mA read — only this adapter can screen it (motor_conversion.hpp note).

*function, declared at [`include/shulib/hal/pros/motor.hpp:259`](../../include/shulib/hal/pros/motor.hpp#L259).*

<a id="prosmotor-temperature"></a>

### `ProsMotor::temperature`

```cpp
[[nodiscard]] double temperature() const override
```

Motor temperature in degrees Celsius — already canonical, so this is the one reader that applies no conversion. Sentinel-screened like the others (non-finite holds the last good value and counts a faulted read), but note the seed: before ANY successful read this returns 20 °C, not 0, so a port that has never answered reports a room-temperature motor. Check faultedReads() before trusting it as a thermal signal.

*function, declared at [`include/shulib/hal/pros/motor.hpp:274`](../../include/shulib/hal/pros/motor.hpp#L274).*

<a id="prosmotor-faultedreads"></a>

### `ProsMotor::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many of the five screened READS (position/velocity/current/temperature/brakeMode) were screened to last-good — cumulative for the life of the object, never reset. T7 observability: telemetry and the loop's health policy can see a flaky port without this seam growing a validity channel F4 does not have. brakeMode() has been counted since DEFECTS1 (I9/E1); this comment said otherwise until DEFECTS2. COMMANDS are not counted: a move_voltage() or brake() the port refuses leaves this number unchanged.

*function, declared at [`include/shulib/hal/pros/motor.hpp:290`](../../include/shulib/hal/pros/motor.hpp#L290).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 90 lines, click to expand</summary>

```text

 ProsMotor — IMotor over pros::Motor (chunk R1a).

 BINDS:
  * move_voltage(mV)          ← setVoltage()   (motor_conversion.hpp, HA-94)
  * brake()                   ← setVoltage() at 0 mV under a commanded Brake/Hold, and
                                setBrakeMode() while stopped (DEFECTS2 — below)
  * get_position() [degrees]  → position()     (motorPositionDegToCanonical, HA-95)
  * get_actual_velocity() RPM → velocity()     (motorRpmToCanonical, HA-96)
  * get_current_draw() mA     → current()      (motorMilliampsToCanonical, HA-97)
  * get_temperature() °C      → temperature()  (identity — already canonical)
  * set/get_brake_mode        ↔ setBrakeMode()/brakeMode() (mapped both ways below)

 THE STOP — HOW A BRAKE MODE REACHES THE MOTOR (chunk DEFECTS2, 2026-09-26;
 A4 register HA-142): PROS applies a brake mode ONLY through brake(). move_voltage()
 "will not respect brake modes, and simply sets the voltage" (vendored motors.h:253-254;
 the same note on motor_move, :52); brake() "stops the motor using the currently
 configured brake mode ... sets motor velocity to zero, which will cause it to act
 according to the set brake mode" (motors.hpp:260-284). Until DEFECTS2 this adapter only
 ever called move_voltage(), so every stop in the library — a mechanism's declared Hold, the
 drive's "0 V + Brake" cancel — was a 0 V command whose mode, by PROS's own documentation, the
 motor ignored: on a V5, Hold went limp and Brake coasted (reasoned from that text — never
 observed), while every host test passed (the shim had no brake(), the fakes echo the mode,
 and the A2 plant pins a wheel at 0 V). The rule now:
   * setVoltage() whose MILLIVOLTS round to 0 (the value SENT — a 0.4 mV command is a stop,
     never keyed on the raw double), under a mode THIS PROGRAM commanded through
     setBrakeMode() that is Brake or Hold ⇒ brake(). Issued on the TRANSITION into that stop
     only: a zero command repeated under the same mode (a live operation re-sends its volts
     every tick; DriveBrake re-commands Brake + 0 V every tick) sends nothing more, because a
     repeated brake() under Hold may re-capture the held position and let a loaded shaft creep
     (A4 register HA-144, invented).
   * Coast, or NO mode commanded yet ⇒ move_voltage(0) on every call, exactly as before
     DEFECTS2. Calibrate, shulib Drive and the tester's DRIVE station set Coast and their
     measured behaviour rests on this 0 V path (what a motor physically does at 0 V is
     A4 register HA-143 — unknown).
   * setBrakeMode() while the last command was 0 mV RE-ISSUES the stop under the new mode, so
     the order of the two calls never matters: Hold-then-0 V and 0 V-then-Hold both hold, and
     Hold-then-0 V-then-Coast leaves the motor at 0 V as Coast always meant.
   * Any non-zero command leaves the stop (move_voltage replaces it — A4 register HA-146).
   * A call the port REFUSES (PROS_ERR — a dropout) is never believed. brake() stops under
     whatever the device's REGISTER holds, so a set_brake_mode() the port refused is re-sent
     before the next brake(), and the stop is recorded only when BOTH were accepted; until
     then every zero command retries. A caller that sends its zero ONCE (applySafeState() at
     an operation's exit, T3's binding) gets no retry if the port is down at that moment —
     the limit every refused command has had, move_voltage(0) included.
 KEYED ON THE COMMANDED MODE, NEVER ON brakeMode_: the ctor SEEDS brakeMode_ from the device
 (the T7 fallback, below), and a drive motor an earlier program left in Hold would otherwise
 brake() at centred sticks in a program that never asked for Hold. A motor this program never
 gave a mode stays on move_voltage(0) at zero, whatever the device remembers (HA-98).
 The stop record is per ADAPTER OBJECT: two ProsMotors on one port (IMotor's copy semantics)
 each believe only their own history, and nothing here can see a VEXos disable
 (A4 register HA-145): after a re-enable, a zero repeated under the same mode sends nothing new.
 The fix lives HERE, below every wrapper (MotorGroup, the hostile fakes, test wrappers), so
 each inherits it; IMotor's signature is unchanged (F4) — no stop verb was added above.

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

 HA register: HA-94..HA-98, HA-142..HA-146 (docs/hardware-assumptions.md).
```

</details>
