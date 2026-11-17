<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/motor_conversion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motor_conversion.hpp`

Motor canonical conversions — the ONE place the V5 smart motor's units become shulib's canonical units (§7: "convert exactly once, at the edge").

This header declares **6** free functions.

Extracted from [`include/shulib/hal/motor_conversion.hpp`](../../include/shulib/hal/motor_conversion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`motorVoltageToMillivolts`](#motorvoltagetomillivolts) — *free function*
- [`motorVoltageApplied`](#motorvoltageapplied) — *free function*
- [`motorPositionDegToCanonical`](#motorpositiondegtocanonical) — *free function*
- [`motorRpmToCanonical`](#motorrpmtocanonical) — *free function*
- [`motorMilliampsToCanonical`](#motormilliampstocanonical) — *free function*
- [`motorCurrentLimitToMilliamps`](#motorcurrentlimittomilliamps) — *free function*

<a id="motorvoltagetomillivolts"></a>

## `motorVoltageToMillivolts`

```cpp
[[nodiscard]] inline std::int32_t motorVoltageToMillivolts(units::Voltage volts)
```

Canonical volts → the V5's move_voltage() millivolt command, clamped to the ±12 V hardware limit (kMaxMotorVoltage) and rounded to the nearest integer millivolt. This is the ONE scale factor between shulib's motor command and the wire (HA-94): drop it and every command is 1/1000 of the intended torque — the robot hums instead of driving, and no test that checks "a command was sent" can see it.  Non-finite input throws (fail-loud backstop) — but the ADAPTER must have rejected it already; see the binding contract above.

*free function, declared at [`include/shulib/hal/motor_conversion.hpp:64`](../../include/shulib/hal/motor_conversion.hpp#L64).*

<a id="motorvoltageapplied"></a>

## `motorVoltageApplied`

```cpp
[[nodiscard]] inline units::Voltage motorVoltageApplied(units::Voltage volts)
```

The voltage actually APPLIED after the ±12 V clamp (canonical volts) — the value IMotor::commandedVoltage() must report, computed by the same clamp as the millivolt command so telemetry can never disagree with the wire.

*free function, declared at [`include/shulib/hal/motor_conversion.hpp:75`](../../include/shulib/hal/motor_conversion.hpp#L75).*

<a id="motorpositiondegtocanonical"></a>

## `motorPositionDegToCanonical`

```cpp
[[nodiscard]] inline units::AngleDim motorPositionDegToCanonical(double positionDeg)
```

Output-shaft degrees (get_position() AFTER the adapter set MotorUnits::degrees, HA-95) → canonical cumulative radians. Deliberately NOT math::Angle: odometry integrates total travel, so this must never wrap (motor.hpp:12-14).

*free function, declared at [`include/shulib/hal/motor_conversion.hpp:85`](../../include/shulib/hal/motor_conversion.hpp#L85).*

<a id="motorrpmtocanonical"></a>

## `motorRpmToCanonical`

```cpp
[[nodiscard]] inline units::AngularVelocity motorRpmToCanonical(double rpm)
```

Output-shaft RPM (get_actual_velocity(), HA-96) → canonical rad/s. 1 rev/min = 2π rad / 60 s.

*free function, declared at [`include/shulib/hal/motor_conversion.hpp:94`](../../include/shulib/hal/motor_conversion.hpp#L94).*

<a id="motormilliampstocanonical"></a>

## `motorMilliampsToCanonical`

```cpp
[[nodiscard]] inline units::Current motorMilliampsToCanonical(double milliamps)
```

Milliamps (get_current_draw(), HA-97) → canonical amperes. The mA→A divide happens exactly once, here (motor.hpp:61-62). Skip it and every current is 1000× — stall detection fires instantly on a free-spinning motor; apply it twice and it never fires at all. Takes a double because the adapter has already screened the integer sentinel (PROS_ERR is in-band for int32 — only the adapter can tell 2147483647 mA from a reading).

*free function, declared at [`include/shulib/hal/motor_conversion.hpp:107`](../../include/shulib/hal/motor_conversion.hpp#L107).*

<a id="motorcurrentlimittomilliamps"></a>

## `motorCurrentLimitToMilliamps`

```cpp
[[nodiscard]] inline std::int32_t motorCurrentLimitToMilliamps(units::Current limit)
```

Canonical amperes → the V5's set_current_limit() milliamp argument, rounded to the nearest integer milliamp (chunk MECH-S-A; HA-219). The A→mA multiply happens exactly once, here; the read-back comes home through motorMilliampsToCanonical() above. Get it wrong by 1000× and a 1.2 A cap is sent as 1 mA (the motor barely turns) or 1 200 000 mA (no cap at all) — and the read-back would agree with the mistake, so only this function's own test can see it. PRECONDITION: `limit` finite, at least 0.001 A (one milliamp — anything smaller would round to 0 mA, a motor with no torque) and at most 2.5 A (hal::kDefaultMotorCurrentLimit — the build refuses any other cap first: mechanism/spec.hpp's refusals 71 and 73).

*free function, declared at [`include/shulib/hal/motor_conversion.hpp:122`](../../include/shulib/hal/motor_conversion.hpp#L122).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 41 lines</summary>

```text

 Motor canonical conversions — the ONE place the V5 smart motor's units become
 shulib's canonical units (§7: "convert exactly once, at the edge"). Pure,
 PROS-free functions, following the exact pattern of imu_conversion.hpp /
 gps_conversion.hpp: the unit arithmetic is host- and mutation-testable in
 isolation, and the hal/pros IMotor adapter is thin glue that CALLS these
 (never re-derives them — E2's HA-07 lesson: an obligation that lives in prose
 is an obligation nothing executes).

 V5 motor conventions (as read from the vendored PROS headers — every row is a
 registered belief, docs/hardware-assumptions.md):
   * move_voltage() takes MILLIVOLTS in [-12000, 12000]  (motors.hpp:234; HA-94)
   * get_position() returns degrees ONLY once the adapter has explicitly set
     MotorUnits::degrees — the ctor default leaves whatever the firmware was
     last told, possibly by a different program on a different day (HA-95/HA-98)
   * get_actual_velocity() returns RPM                    (motors.hpp:404; HA-96)
   * get_current_draw() returns mA                        (motors.hpp:426; HA-97)
   * set_current_limit() takes, and get_current_limit() returns, mA; the
     default is 2500 (motors.hpp:974, :1237; motors.h; MECH-S-A, HA-219)
   * get_temperature() returns °C — identity, no conversion function needed

 shulib canonical (F3): volts, radians (cumulative, non-wrapping AngleDim),
 rad/s, amperes.

 ADAPTER BINDING CONTRACT (load-bearing, like the IMU's and GPS's):
  * setVoltage() REJECTS a non-finite command (IMotor contract, motor.hpp:8-11)
    BEFORE calling motorVoltageToMillivolts() — the conversion's own finiteness
    precondition is the fail-loud backstop, NOT the rejection path. Silently
    coercing non-finite to 0 is the exact mistake the M5 measurement prototype
    made: it compiles, passes a clamp test, and turns a programming error into
    a robot that coasts (brief L4).
  * The adapter MUST set encoder units (degrees) AND gearing explicitly at
    construction and READ BOTH BACK — pros::Motor's ctor defaults mean "leave
    the device as it was", and a motor left in rotations reports 1/360 of what
    this conversion assumes. Odometry is then silently wrong by 360× and
    nothing crashes. (HA-98.)
  * Sentinel screening (PROS_ERR / PROS_ERR_F) happens in the adapter BEFORE
    these are called (T7: hold last good, never propagate, never zero). The
    finiteness preconditions here catch PROS_ERR_F (= +inf) as a backstop;
    PROS_ERR (= INT32_MAX) is in-band for integers and ONLY the adapter can
    screen it.
```

</details>
