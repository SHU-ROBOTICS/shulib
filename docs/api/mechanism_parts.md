<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/robot/mechanism_parts.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `mechanism_parts.hpp`

robot::MechanismParts — every ROLE of every mechanism, built from the config over a device provider.

This header declares **1** type (19 members).

Extracted from [`include/shulib/robot/mechanism_parts.hpp`](../../include/shulib/robot/mechanism_parts.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class MechanismParts`](#class-mechanismparts)
  - [`kReasonBytes`](#mechanismparts-kreasonbytes)
  - [`kMaxRoles`](#mechanismparts-kmaxroles)
  - [`MechanismParts`](#mechanismparts-mechanismparts)
  - [`MechanismParts (overload 2)`](#mechanismparts-mechanismparts-2)
  - [`MechanismParts (overload 3)`](#mechanismparts-mechanismparts-3)
  - [`operator=`](#mechanismparts-operator-eq)
  - [`operator= (overload 2)`](#mechanismparts-operator-eq-2)
  - [`~MechanismParts`](#mechanismparts-destructor-mechanismparts)
  - [`build`](#mechanismparts-build)
  - [`size`](#mechanismparts-size)
  - [`devices`](#mechanismparts-devices)
  - [`built`](#mechanismparts-built)
  - [`builtAt`](#mechanismparts-builtat)
  - [`motorAt`](#mechanismparts-motorat)
  - [`pneumaticAt`](#mechanismparts-pneumaticat)
  - [`absence`](#mechanismparts-absence)
  - [`roles`](#mechanismparts-roles)
  - [`config`](#mechanismparts-config)
  - [`indexOf`](#mechanismparts-indexof)

<a id="class-mechanismparts"></a>

## `class MechanismParts`

```cpp
class MechanismParts
```

Every role of every mechanism a config declares, built over a device provider (header). Built ONCE, by build(); non-copyable and non-movable — the built mechanisms hold spans into its storage.

*class, declared at [`include/shulib/robot/mechanism_parts.hpp:67`](../../include/shulib/robot/mechanism_parts.hpp#L67).*

<a id="mechanismparts-kreasonbytes"></a>

### `MechanismParts::kReasonBytes`

```cpp
static constexpr std::size_t kReasonBytes = 160
```

The longest reason a mechanism was not built (or a piece sensor was refused), NUL included — sized to the on-robot provider's refusal buffer (hal::pros::ProsDeviceProvider::kRefusalBytes) so the whole sentence it wrote survives the copy.

*field, declared at [`include/shulib/robot/mechanism_parts.hpp:72`](../../include/shulib/robot/mechanism_parts.hpp#L72).*

<a id="mechanismparts-kmaxroles"></a>

### `MechanismParts::kMaxRoles`

```cpp
static constexpr std::size_t kMaxRoles = config::kMaxMotorsPerMechanism + config::kMaxPneumaticsPerMechanism + 3
```

The most roles one mechanism has: its motors (or lines), an angle sensor, a home switch and a piece sensor.

*field, declared at [`include/shulib/robot/mechanism_parts.hpp:75`](../../include/shulib/robot/mechanism_parts.hpp#L75).*

<a id="mechanismparts-mechanismparts"></a>

### `MechanismParts::MechanismParts`

```cpp
MechanismParts() = default
```

Empty: nothing built.

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:79`](../../include/shulib/robot/mechanism_parts.hpp#L79).*

<a id="mechanismparts-mechanismparts-2"></a>

### `MechanismParts::MechanismParts (overload 2)`

```cpp
MechanismParts(const MechanismParts&) = delete
```

Empty: nothing built. Non-copyable: the built mechanisms hold spans into this object's storage.

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:81`](../../include/shulib/robot/mechanism_parts.hpp#L81).*

<a id="mechanismparts-mechanismparts-3"></a>

### `MechanismParts::MechanismParts (overload 3)`

```cpp
MechanismParts(MechanismParts&&) = delete
```

Empty: nothing built. Non-copyable: the built mechanisms hold spans into this object's storage. Non-movable, same reasoning.

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:83`](../../include/shulib/robot/mechanism_parts.hpp#L83).*

<a id="mechanismparts-operator-eq"></a>

### `MechanismParts::operator=`

```cpp
MechanismParts& operator=(const MechanismParts&) = delete
```

Empty: nothing built. Non-copyable: the built mechanisms hold spans into this object's storage. Non-movable, same reasoning. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:85`](../../include/shulib/robot/mechanism_parts.hpp#L85).*

<a id="mechanismparts-operator-eq-2"></a>

### `MechanismParts::operator= (overload 2)`

```cpp
MechanismParts& operator=(MechanismParts&&) = delete
```

Empty: nothing built. Non-copyable: the built mechanisms hold spans into this object's storage. Non-movable, same reasoning. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:87`](../../include/shulib/robot/mechanism_parts.hpp#L87).*

<a id="mechanismparts-destructor-mechanismparts"></a>

### `MechanismParts::~MechanismParts`

```cpp
~MechanismParts() = default
```

Empty: nothing built. Non-copyable: the built mechanisms hold spans into this object's storage. Non-movable, same reasoning. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Destroys the mechanisms it built (never the devices: the provider owns those).

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:89`](../../include/shulib/robot/mechanism_parts.hpp#L89).*

<a id="mechanismparts-build"></a>

### `MechanismParts::build`

```cpp
void build(std::span<const config::MechanismConfig> configs, hal::IDeviceProvider& devices)
```

Build every mechanism of `configs`, in order (header: THE ORDER). The configs are BORROWED, not copied: each built mechanism's name() is the pointer into them, exactly as the Robot's mechanisms have always borrowed the Robot's own config copy (robot_builder_test.cpp pins that pointer) — so `configs` must OUTLIVE this object. The Robot hands in its own copy (made for exactly that reason: it never keeps a caller's reference); a caller with no Robot keeps its configs in static storage beside the parts. `devices` is asked for every role and must outlive this object too. PRECONDITIONS: called once; at most config::kMaxMechanisms configs; the configs passed RobotConfig::valid() (a provider precondition — one port asked twice with two configurations, one 3-wire line asked as an output and an input — would otherwise throw out of here, loudly).

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:101`](../../include/shulib/robot/mechanism_parts.hpp#L101).*

<a id="mechanismparts-size"></a>

### `MechanismParts::size`

```cpp
[[nodiscard]] std::size_t size() const noexcept
```

How many mechanisms were declared (built or not).

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:114`](../../include/shulib/robot/mechanism_parts.hpp#L114).*

<a id="mechanismparts-devices"></a>

### `MechanismParts::devices`

```cpp
[[nodiscard]] std::span<const mechanism::Devices> devices() const noexcept
```

Every declared mechanism's devices, in config order — what the mechanism service binds over (MECH-A2). An ABSENT one has every pointer null and its `absence` set.

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:118`](../../include/shulib/robot/mechanism_parts.hpp#L118).*

<a id="mechanismparts-built"></a>

### `MechanismParts::built`

```cpp
[[nodiscard]] std::span<hal::IMechanism* const> built() const noexcept
```

Every BUILT mechanism of both kinds, in config order — the `hal::IMechanism*` span an end-of-run guard walks. An ABSENT one is not in it.

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:124`](../../include/shulib/robot/mechanism_parts.hpp#L124).*

<a id="mechanismparts-builtat"></a>

### `MechanismParts::builtAt`

```cpp
[[nodiscard]] hal::IMechanism* builtAt(std::size_t i) const noexcept
```

The `i`-th declared mechanism if it was built, of either kind; nullptr when ABSENT, when it has nothing to build, or when `i` is not below size().

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:130`](../../include/shulib/robot/mechanism_parts.hpp#L130).*

<a id="mechanismparts-motorat"></a>

### `MechanismParts::motorAt`

```cpp
[[nodiscard]] hal::MotorMechanism* motorAt(std::size_t i) const noexcept
```

The `i`-th declared mechanism as a built MOTOR mechanism; nullptr when ABSENT, pneumatic, or `i` is not below size().

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:142`](../../include/shulib/robot/mechanism_parts.hpp#L142).*

<a id="mechanismparts-pneumaticat"></a>

### `MechanismParts::pneumaticAt`

```cpp
[[nodiscard]] hal::PneumaticMechanism* pneumaticAt(std::size_t i) const noexcept
```

The `i`-th declared mechanism as a built PNEUMATIC mechanism; nullptr when ABSENT, a motor mechanism, or `i` is not below size().

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:148`](../../include/shulib/robot/mechanism_parts.hpp#L148).*

<a id="mechanismparts-absence"></a>

### `MechanismParts::absence`

```cpp
[[nodiscard]] const char* absence(std::size_t i) const noexcept
```

Why the `i`-th declared mechanism was NOT built: the provider's sentence naming the port, or a fixed one for a mechanism with nothing to build. "" EXACTLY when builtAt(i) is non-null. "no mechanism is declared at this index" for an `i` not below size().

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:155`](../../include/shulib/robot/mechanism_parts.hpp#L155).*

<a id="mechanismparts-roles"></a>

### `MechanismParts::roles`

```cpp
[[nodiscard]] std::span<const mechanism::RoleFact> roles(std::size_t i) const noexcept
```

The `i`-th mechanism's ROLES, in boot-line order (motors or lines, angle sensor, home switch, piece sensor) — what each one's boot line words. Empty for an `i` not below size().

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:161`](../../include/shulib/robot/mechanism_parts.hpp#L161).*

<a id="mechanismparts-config"></a>

### `MechanismParts::config`

```cpp
[[nodiscard]] const config::MechanismConfig& config(std::size_t i) const
```

The `i`-th mechanism's config (the borrowed one). PRECONDITION: `i` below size().

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:169`](../../include/shulib/robot/mechanism_parts.hpp#L169).*

<a id="mechanismparts-indexof"></a>

### `MechanismParts::indexOf`

```cpp
[[nodiscard]] std::size_t indexOf(const char* name) const noexcept
```

The index of the mechanism whose config name is `name`, or size() when none is (or `name` is null). Compared over the name field's 24 bytes.

*function, declared at [`include/shulib/robot/mechanism_parts.hpp:176`](../../include/shulib/robot/mechanism_parts.hpp#L176).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 47 lines, click to expand</summary>

```text

 robot::MechanismParts — every ROLE of every mechanism, built from the config over a device
 provider (the mechanism core, chunk MECH-A1, 2026-10-08).

 ── What it is, and why it is its own class ─────────────────────────────────────────
 Until MECH-A1, robot::Robot built its mechanisms itself (chunk T3 and its addendum B2): a
 hal::MotorMechanism over each mechanism's motors, a hal::PneumaticMechanism over its 3-wire lines,
 ABSENT with the provider's sentence when one would not be made. The mechanism core adds SENSOR
 roles — an angle sensor, a home switch, a piece sensor — and two consumers of the result that are
 not a Robot: the mechanism service (MECH-A2), which runs each mechanism over its `Devices`, and
 Bench Tests' station 13 (MECH-A6), which builds no Robot and no drivetrain but must build the same
 parts to check a mechanism's direction before it is homed. So the builder became this class: the
 storage robot.hpp held (the motors and lines borrowed from the provider, the mechanisms built over
 them, the reason when one was not) MOVED here, and the sensor roles added. The Robot OWNS one and
 forwards every accessor it had to it, unchanged (robot.hpp's MECHANISMS section).

 ── The ORDER, per mechanism ────────────────────────────────────────────────────────
   1. its ACTUATORS — the motors, each with the mechanism's own signed port and cartridge, or the
      3-wire lines, each made at its START level (on a brain making a line DRIVES it, HA-119; the
      per-line levels when the config states them). A refusal: the mechanism is ABSENT.
   2. its ANGLE sensor (a rotation sensor): a refusal makes the mechanism ABSENT — its angle is how
      it knows where it is.
   3. its HOME SWITCH (a 3-wire input): always made on a brain (the registry cannot see a 3-wire
      device — HA-158), so its boot line says "not checkable"; a provider that refuses it anyway
      (only outside 1..8 on a brain, which RobotConfig::valid() refuses first) makes it ABSENT.
   4. its PIECE sensor (a distance or optical sensor, or a limit switch): a refusal leaves it
      RUNNING WITHOUT one — the mechanism is built, and its sensor steps say so (MECH-A2, N8).
 Once a mechanism is ABSENT the rest of its roles are NOT ASKED, and the mechanism is not in
 built(): there is nothing there for a guard to make safe, and nothing a step may command. A
 motor made before the refusal stays made — the provider owns it; nothing commands it.

 Beside each piece sensor and home switch it records the provider's READ-FAULT count for it
 (IDeviceProvider::readFaults / lineReadFaults — hal/read_faults.hpp), into that mechanism's
 `Devices`: the service's rule that a sample taken while it moved is HELD, never evidence.

 One `RoleFact` per role — what was asked, on which port or line, and what the provider said — is
 what a program's boot lines word (mechanism::roleLine, mechanism/wiring.hpp): "arm: motor 1 ->
 port 1 ok", "wrist: angle sensor -> port 10 ABSENT: port 10: a distance sensor (expected a
 rotation sensor)", "lift: home switch -> 3-wire B not checkable (...)".

 ── What it does NOT do ─────────────────────────────────────────────────────────────
 It never ticks and never commands a motor (the Robot's rule, unchanged): a mechanism is first
 made safe by whatever drives it. It does not check the config — RobotConfig::valid() is the
 owner of every port collision, and a caller builds only a config valid() accepted. It allocates
 nothing: every array is fixed, sized by the config's capacities.

 PROS-free, sim-free, allocation-free.
```

</details>
