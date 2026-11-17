<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/motor_current_limit.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motor_current_limit.hpp`

IMotorCurrentLimit — a V5 motor's OWN current limit, set and read back (chunk MECH-S-A, 2026-11-05; the MECH-S brief's D1 (a)).

This header declares **1** type (8 members) and **1** constant.

Extracted from [`include/shulib/hal/motor_current_limit.hpp`](../../include/shulib/hal/motor_current_limit.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kDefaultMotorCurrentLimit`](#kdefaultmotorcurrentlimit) — *constant*
- [`class IMotorCurrentLimit`](#class-imotorcurrentlimit)
  - [`~IMotorCurrentLimit`](#imotorcurrentlimit-destructor-imotorcurrentlimit)
  - [`IMotorCurrentLimit`](#imotorcurrentlimit-imotorcurrentlimit)
  - [`IMotorCurrentLimit (overload 2)`](#imotorcurrentlimit-imotorcurrentlimit-2)
  - [`IMotorCurrentLimit (overload 3)`](#imotorcurrentlimit-imotorcurrentlimit-3)
  - [`operator=`](#imotorcurrentlimit-operator-eq)
  - [`operator= (overload 2)`](#imotorcurrentlimit-operator-eq-2)
  - [`setCurrentLimit`](#imotorcurrentlimit-setcurrentlimit)
  - [`currentLimit`](#imotorcurrentlimit-currentlimit)

<a id="kdefaultmotorcurrentlimit"></a>

## `kDefaultMotorCurrentLimit`

```cpp
inline constexpr units::Current kDefaultMotorCurrentLimit{2.5}
```

A V5 motor's documented DEFAULT current limit, 2.5 A ("The default value is 2500 mA" — vendored pros/motors.h). The most any limit may be set to here, the value a motor that never had one reads, and what one that lost power comes back at (A4 register HA-220).

*constant, declared at [`include/shulib/hal/motor_current_limit.hpp:46`](../../include/shulib/hal/motor_current_limit.hpp#L46).*

<a id="class-imotorcurrentlimit"></a>

## `class IMotorCurrentLimit`

```cpp
class IMotorCurrentLimit
```

A motor's own current limit, set and read back (header). Implemented BESIDE IMotor by an adapter whose device enforces a limit; reached through IDeviceProvider::currentLimit(port).

*class, declared at [`include/shulib/hal/motor_current_limit.hpp:50`](../../include/shulib/hal/motor_current_limit.hpp#L50).*

<a id="imotorcurrentlimit-destructor-imotorcurrentlimit"></a>

### `IMotorCurrentLimit::~IMotorCurrentLimit`

```cpp
virtual ~IMotorCurrentLimit() = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IMotorCurrentLimit*` well-defined. A consumer holds one by pointer and never owns it (the provider does).

*function, declared at [`include/shulib/hal/motor_current_limit.hpp:54`](../../include/shulib/hal/motor_current_limit.hpp#L54).*

<a id="imotorcurrentlimit-imotorcurrentlimit"></a>

### `IMotorCurrentLimit::IMotorCurrentLimit`

```cpp
IMotorCurrentLimit() = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IMotorCurrentLimit*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state.

*function, declared at [`include/shulib/hal/motor_current_limit.hpp:56`](../../include/shulib/hal/motor_current_limit.hpp#L56).*

<a id="imotorcurrentlimit-imotorcurrentlimit-2"></a>

### `IMotorCurrentLimit::IMotorCurrentLimit (overload 2)`

```cpp
IMotorCurrentLimit(const IMotorCurrentLimit&) = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IMotorCurrentLimit*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state. Copyable at the base, because the base holds no state; an adapter decides for itself.

*function, declared at [`include/shulib/hal/motor_current_limit.hpp:58`](../../include/shulib/hal/motor_current_limit.hpp#L58).*

<a id="imotorcurrentlimit-imotorcurrentlimit-3"></a>

### `IMotorCurrentLimit::IMotorCurrentLimit (overload 3)`

```cpp
IMotorCurrentLimit(IMotorCurrentLimit&&) = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IMotorCurrentLimit*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state. Copyable at the base, because the base holds no state; an adapter decides for itself. Movable, same reasoning as the copy constructor.

*function, declared at [`include/shulib/hal/motor_current_limit.hpp:60`](../../include/shulib/hal/motor_current_limit.hpp#L60).*

<a id="imotorcurrentlimit-operator-eq"></a>

### `IMotorCurrentLimit::operator=`

```cpp
IMotorCurrentLimit& operator=(const IMotorCurrentLimit&) = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IMotorCurrentLimit*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state. Copyable at the base, because the base holds no state; an adapter decides for itself. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning.

*function, declared at [`include/shulib/hal/motor_current_limit.hpp:62`](../../include/shulib/hal/motor_current_limit.hpp#L62).*

<a id="imotorcurrentlimit-operator-eq-2"></a>

### `IMotorCurrentLimit::operator= (overload 2)`

```cpp
IMotorCurrentLimit& operator=(IMotorCurrentLimit&&) = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IMotorCurrentLimit*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state. Copyable at the base, because the base holds no state; an adapter decides for itself. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning. Move-assignable, same reasoning.

*function, declared at [`include/shulib/hal/motor_current_limit.hpp:64`](../../include/shulib/hal/motor_current_limit.hpp#L64).*

<a id="imotorcurrentlimit-setcurrentlimit"></a>

### `IMotorCurrentLimit::setCurrentLimit`

```cpp
[[nodiscard]] virtual bool setCurrentLimit(units::Current limit) = 0
```

Tell the motor never to draw more than `limit` from now on (the motor enforces it — header). True when the DEVICE accepted the write; false when it refused it (a port that did not answer). True is not proof the motor now holds `limit`: read it back with currentLimit() (VEXos may hold its own share — HA-221). A V5 motor holds its limit in WHOLE milliamps: an adapter over one rounds to the nearest mA (hal::motorCurrentLimitToMilliamps), so compare a read-back to the mA, never to the double asked. PRECONDITION: `limit` finite, from 0.001 A (one milliamp) to kDefaultMotorCurrentLimit (the build refuses any other cap first — mechanism/spec.hpp).

*function, declared at [`include/shulib/hal/motor_current_limit.hpp:73`](../../include/shulib/hal/motor_current_limit.hpp#L73).*

<a id="imotorcurrentlimit-currentlimit"></a>

### `IMotorCurrentLimit::currentLimit`

```cpp
[[nodiscard]] virtual units::Current currentLimit() const = 0
```

The limit the MOTOR reports now — READ BACK from the device, never the last request's echo (IMotor::brakeMode()'s rule). An adapter over real hardware that cannot read the device answers its last good reading and counts the screen (hal::pros::ProsMotor's faultedReads()).

*function, declared at [`include/shulib/hal/motor_current_limit.hpp:78`](../../include/shulib/hal/motor_current_limit.hpp#L78).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 36 lines</summary>

```text

 IMotorCurrentLimit — a V5 motor's OWN current limit, set and read back (chunk MECH-S-A, 2026-11-05;
 the MECH-S brief's D1 (a)).

 ── An F4-ADDITIVE SIBLING, not a change to F4 ──────────────────────────────────────
 Exactly as IDigitalOut (F11), IController (F13), IDigitalIn (F14) and IReadFaults (MECH-A1) were
 added beside the locked F4 seams, this one is added beside IMotor: hal/motor.hpp is untouched, byte
 for byte (MECH-PANEL §6 planned exactly this — "a NEW sibling seam … so F4's row is untouched and
 there is no F4 bump"). An adapter whose motor can limit its own current implements this interface AS
 WELL as IMotor (hal::pros::ProsMotor; the host's hal::fake::FakeMotor); a consumer asks the device
 provider for it by port (IDeviceProvider::currentLimit), never by casting a motor.

 ── WHY THE MOTOR, NOT THE LIBRARY (the brief's D1) ─────────────────────────────────
 A string pulled too hard snaps, and a V5 motor's current IS the string's pull (its torque through
 the spool — A4 register HA-222). The library could clamp each tick's volts so the expected current
 stays under a cap — but only while something ticks it: a student's blocking loop starves the
 mechanism service, and a starved service leaves a motor at its last command (the service's own
 "honest limit", measured at 11.74 V held for 1.5 s). The MOTOR's limit (PROS
 `motor_set_current_limit`, "The motor will reduce its output at 1000 mA instead of the default
 2500 mA" — vendored pros/motors.h) is enforced by the motor itself from the moment it is set: every
 command, the Hold brake at 0 V, a homing push, a disable, the end of an auton, a starved service.
 HOW FAST the motor enforces it, and that it does at all with nothing ticking, is A4 register
 HA-219 — reasoned from that one sentence, never measured.

 ── What a limit set here CANNOT be trusted to do (each an A4 register entry) ────────
  * stay set: a limit lives in the DEVICE, so one set by an earlier program may still be there at
    boot, and a motor that loses power comes back at its 2500 mA default (HA-220) — which is why the
    mechanism service READS every mechanism motor's limit at boot and re-checks a capped one each
    second (mechanism/service.hpp);
  * be the number read back: with more than 8 motors plugged in VEXos may lower each motor's limit
    on its own (HA-221 — robot two runs 10), so the read-back is the device's answer, never the
    request's echo;
  * limit a shock: the rotor's and the carriage's MOMENTUM when it hits a stop pull on the string
    with no current at all (HA-222).

 PROS-free, allocation-free, no state of its own.
```

</details>
