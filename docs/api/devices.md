<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/devices.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `devices.hpp`

mechanism/devices.hpp — the DEVICES one mechanism was built over, and what the build found for each of its ROLES.

This header declares **3** types (21 members) and **1** constant.

Extracted from [`include/shulib/mechanism/devices.hpp`](../../include/shulib/mechanism/devices.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kDevicesMaxMotors`](#kdevicesmaxmotors) — *constant*
- [`struct Devices`](#struct-devices)
  - [`motors`](#devices-motors)
  - [`pneumatic`](#devices-pneumatic)
  - [`angleSensor`](#devices-anglesensor)
  - [`homeSwitch`](#devices-homeswitch)
  - [`pieceDistance`](#devices-piecedistance)
  - [`pieceOptical`](#devices-pieceoptical)
  - [`pieceSwitch`](#devices-pieceswitch)
  - [`pieceFaults`](#devices-piecefaults)
  - [`homeSwitchFaults`](#devices-homeswitchfaults)
  - [`absence`](#devices-absence)
  - [`currentLimits`](#devices-currentlimits)
- [`enum class RoleState`](#enum-class-rolestate)
  - [`Present`](#rolestate-present)
  - [`Refused`](#rolestate-refused)
  - [`NotCheckable`](#rolestate-notcheckable)
  - [`NotAsked`](#rolestate-notasked)
- [`struct RoleFact`](#struct-rolefact)
  - [`role`](#rolefact-role)
  - [`port`](#rolefact-port)
  - [`line`](#rolefact-line)
  - [`state`](#rolefact-state)
  - [`degradeOnly`](#rolefact-degradeonly)
  - [`refusal`](#rolefact-refusal)

<a id="kdevicesmaxmotors"></a>

## `kDevicesMaxMotors`

```cpp
inline constexpr std::size_t kDevicesMaxMotors = 4
```

How many motors one mechanism's `Devices` has room for: config::kMaxMotorsPerMechanism, stated here because this header includes nothing above hal/ (robot/mechanism_parts.hpp and mechanism/service.hpp each static_assert that the two agree).

*constant, declared at [`include/shulib/mechanism/devices.hpp:41`](../../include/shulib/mechanism/devices.hpp#L41).*

<a id="struct-devices"></a>

## `struct Devices`

```cpp
struct Devices
```

The devices one mechanism was built over (header).

*struct, declared at [`include/shulib/mechanism/devices.hpp:44`](../../include/shulib/mechanism/devices.hpp#L44).*

<a id="devices-motors"></a>

### `Devices::motors`

```cpp
hal::MotorMechanism* motors = nullptr
```

A motor kind's actuator (power, position, speed); nullptr for a pneumatic or when ABSENT.

*field, declared at [`include/shulib/mechanism/devices.hpp:46`](../../include/shulib/mechanism/devices.hpp#L46).*

<a id="devices-pneumatic"></a>

### `Devices::pneumatic`

```cpp
hal::PneumaticMechanism* pneumatic = nullptr
```

A pneumatic kind's actuator; nullptr for a motor kind or when ABSENT.

*field, declared at [`include/shulib/mechanism/devices.hpp:48`](../../include/shulib/mechanism/devices.hpp#L48).*

<a id="devices-anglesensor"></a>

### `Devices::angleSensor`

```cpp
hal::IRotation* angleSensor = nullptr
```

Its angle sensor (a Rotation role).

*field, declared at [`include/shulib/mechanism/devices.hpp:50`](../../include/shulib/mechanism/devices.hpp#L50).*

<a id="devices-homeswitch"></a>

### `Devices::homeSwitch`

```cpp
hal::IDigitalIn* homeSwitch = nullptr
```

Its home switch (a limit switch on a 3-wire line).

*field, declared at [`include/shulib/mechanism/devices.hpp:52`](../../include/shulib/mechanism/devices.hpp#L52).*

<a id="devices-piecedistance"></a>

### `Devices::pieceDistance`

```cpp
hal::IDistance* pieceDistance = nullptr
```

Its piece sensor when it is a distance sensor (at most one of the three piece pointers).

*field, declared at [`include/shulib/mechanism/devices.hpp:54`](../../include/shulib/mechanism/devices.hpp#L54).*

<a id="devices-pieceoptical"></a>

### `Devices::pieceOptical`

```cpp
hal::IOptical* pieceOptical = nullptr
```

Its piece sensor when it is an optical sensor.

*field, declared at [`include/shulib/mechanism/devices.hpp:56`](../../include/shulib/mechanism/devices.hpp#L56).*

<a id="devices-pieceswitch"></a>

### `Devices::pieceSwitch`

```cpp
hal::IDigitalIn* pieceSwitch = nullptr
```

Its piece sensor when it is a limit switch.

*field, declared at [`include/shulib/mechanism/devices.hpp:58`](../../include/shulib/mechanism/devices.hpp#L58).*

<a id="devices-piecefaults"></a>

### `Devices::pieceFaults`

```cpp
const hal::IReadFaults* pieceFaults = nullptr
```

The piece sensor's READ-FAULT count (the provider's readFaults(port) / lineReadFaults(line)): a sample taken while it moved was a HELD value, never evidence. nullptr = no piece sensor, or one that never screens a read.

*field, declared at [`include/shulib/mechanism/devices.hpp:62`](../../include/shulib/mechanism/devices.hpp#L62).*

<a id="devices-homeswitchfaults"></a>

### `Devices::homeSwitchFaults`

```cpp
const hal::IReadFaults* homeSwitchFaults = nullptr
```

The home switch's read-fault count (lineReadFaults(line)).

*field, declared at [`include/shulib/mechanism/devices.hpp:64`](../../include/shulib/mechanism/devices.hpp#L64).*

<a id="devices-absence"></a>

### `Devices::absence`

```cpp
const char* absence = ""
```

Why the mechanism was NOT built — the provider's sentence naming the port; "" when it was.

*field, declared at [`include/shulib/mechanism/devices.hpp:66`](../../include/shulib/mechanism/devices.hpp#L66).*

<a id="devices-currentlimits"></a>

### `Devices::currentLimits`

```cpp
hal::IMotorCurrentLimit* currentLimits[kDevicesMaxMotors]{}
```

Each motor's own current limit, in the motors' order (MECH-S-A; the provider's currentLimit(port)); nullptr past the mechanism's motors, for a pneumatic or ABSENT one, and where the provider's motor cannot limit its current.

*field, declared at [`include/shulib/mechanism/devices.hpp:70`](../../include/shulib/mechanism/devices.hpp#L70).*

<a id="enum-class-rolestate"></a>

## `enum class RoleState`

```cpp
enum class RoleState : std::uint8_t
```

What the build found for ONE ROLE — the facts its boot line words (mechanism::roleLine, in mechanism/wiring.hpp).

*enum class, declared at [`include/shulib/mechanism/devices.hpp:75`](../../include/shulib/mechanism/devices.hpp#L75).*

<a id="rolestate-present"></a>

### `RoleState::Present`

```cpp
Present
```

a smart device of the expected kind answered on its port

*enumerator, declared at [`include/shulib/mechanism/devices.hpp:76`](../../include/shulib/mechanism/devices.hpp#L76).*

<a id="rolestate-refused"></a>

### `RoleState::Refused`

```cpp
Refused
```

the provider refused it (its sentence names the port and what was found)

*enumerator, declared at [`include/shulib/mechanism/devices.hpp:77`](../../include/shulib/mechanism/devices.hpp#L77).*

<a id="rolestate-notcheckable"></a>

### `RoleState::NotCheckable`

```cpp
NotCheckable
```

a 3-wire role: made, but the brain cannot see a 3-wire device (HA-158)

*enumerator, declared at [`include/shulib/mechanism/devices.hpp:78`](../../include/shulib/mechanism/devices.hpp#L78).*

<a id="rolestate-notasked"></a>

### `RoleState::NotAsked`

```cpp
NotAsked
```

not requested: the mechanism was already ABSENT

*enumerator, declared at [`include/shulib/mechanism/devices.hpp:79`](../../include/shulib/mechanism/devices.hpp#L79).*

<a id="struct-rolefact"></a>

## `struct RoleFact`

```cpp
struct RoleFact
```

One role of one mechanism, as the build left it — what a boot line needs.

*struct, declared at [`include/shulib/mechanism/devices.hpp:83`](../../include/shulib/mechanism/devices.hpp#L83).*

<a id="rolefact-role"></a>

### `RoleFact::role`

```cpp
const char* role = ""
```

The role in the boot line's words: "motor 1".."motor 4", "line 1", "line 2", "angle sensor", "home switch", "piece sensor".

*field, declared at [`include/shulib/mechanism/devices.hpp:86`](../../include/shulib/mechanism/devices.hpp#L86).*

<a id="rolefact-port"></a>

### `RoleFact::port`

```cpp
std::uint8_t port = 0
```

Its smart port, physical (1..21); 0 for a 3-wire role.

*field, declared at [`include/shulib/mechanism/devices.hpp:88`](../../include/shulib/mechanism/devices.hpp#L88).*

<a id="rolefact-line"></a>

### `RoleFact::line`

```cpp
std::uint8_t line = 0
```

Its 3-wire line, 1..8 = A..H; 0 for a smart role.

*field, declared at [`include/shulib/mechanism/devices.hpp:90`](../../include/shulib/mechanism/devices.hpp#L90).*

<a id="rolefact-state"></a>

### `RoleFact::state`

```cpp
RoleState state = RoleState::NotAsked
```

What the build found.

*field, declared at [`include/shulib/mechanism/devices.hpp:92`](../../include/shulib/mechanism/devices.hpp#L92).*

<a id="rolefact-degradeonly"></a>

### `RoleFact::degradeOnly`

```cpp
bool degradeOnly = false
```

A refusal that leaves the mechanism RUNNING WITHOUT this role (a piece sensor), rather than ABSENT.

*field, declared at [`include/shulib/mechanism/devices.hpp:95`](../../include/shulib/mechanism/devices.hpp#L95).*

<a id="rolefact-refusal"></a>

### `RoleFact::refusal`

```cpp
const char* refusal = ""
```

The provider's refusal sentence when `state` is Refused (stable for the parts' lifetime); "" otherwise.

*field, declared at [`include/shulib/mechanism/devices.hpp:98`](../../include/shulib/mechanism/devices.hpp#L98).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 22 lines</summary>

```text

 mechanism/devices.hpp — the DEVICES one mechanism was built over, and what the build found for
 each of its ROLES (the mechanism core, chunk MECH-A1, 2026-10-08).

 robot::MechanismParts (robot/mechanism_parts.hpp) builds every role a mechanism declares — its
 motors or lines, its angle sensor, its home switch, its piece sensor — and hands the result to
 the mechanism service (MECH-A2) as one `Devices` per mechanism, and to the boot lines as one
 `RoleFact` per role. HAL POINTERS ONLY, and no other mechanism/ header included, so robot/ may
 produce these without depending on the vocabulary above it (mechanism/kinds.hpp, wiring.hpp).

 nullptr in a `Devices` = that role is not on this mechanism, or the mechanism is ABSENT, or (a
 piece sensor) it was refused and the mechanism runs WITHOUT one. The read-fault pointers are the
 provider's IDeviceProvider::readFaults / lineReadFaults for that sensor (hal/read_faults.hpp):
 the service reads a sample taken while one moved as HELD, never as evidence (MECH-A2's rule).

 Since MECH-S-A (2026-11-05) a motor mechanism's `Devices` also carries each motor's own CURRENT
 LIMIT (the provider's IDeviceProvider::currentLimit(port) — hal/motor_current_limit.hpp), in the
 motors' order: the service writes a stated cap through it at boot and reads every motor's limit
 back (mechanism/service.hpp). nullptr = no such motor, or a provider whose motor cannot limit its
 current (the service says so when a cap is stated).

 PROS-free, allocation-free.
```

</details>
