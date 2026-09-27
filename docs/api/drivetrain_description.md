<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/drivetrain_description.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drivetrain_description.hpp`

DrivetrainDescription — the ONE representation of a drivetrain's discovered facts.

This header declares **5** types (24 members), **1** free function, and **1** constant.

Extracted from [`include/shulib/hal/drivetrain_description.hpp`](../../include/shulib/hal/drivetrain_description.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxSmartPorts`](#kmaxsmartports) — *constant*
- [`enum class DrivetrainKind`](#enum-class-drivetrainkind)
  - [`Tank`](#drivetrainkind-tank)
- [`enum class MotorCartridge`](#enum-class-motorcartridge)
  - [`Red`](#motorcartridge-red)
  - [`Green`](#motorcartridge-green)
  - [`Blue`](#motorcartridge-blue)
- [`cartridgeWord`](#cartridgeword) — *free function*
- [`enum class DescriptionSource`](#enum-class-descriptionsource)
  - [`TypedTable`](#descriptionsource-typedtable)
  - [`RobotMeasured`](#descriptionsource-robotmeasured)
- [`struct DriveSideDescription`](#struct-drivesidedescription)
  - [`signedPorts`](#drivesidedescription-signedports)
  - [`count`](#drivesidedescription-count)
  - [`inchesPerRadian`](#drivesidedescription-inchesperradian)
- [`struct DrivetrainDescription`](#struct-drivetraindescription)
  - [`kind`](#drivetraindescription-kind)
  - [`cartridge`](#drivetraindescription-cartridge)
  - [`imuPort`](#drivetraindescription-imuport)
  - [`left`](#drivetraindescription-left)
  - [`right`](#drivetraindescription-right)
  - [`trackWidth`](#drivetraindescription-trackwidth)
  - [`source`](#drivetraindescription-source)
  - [`provenance`](#drivetraindescription-provenance)
  - [`valid`](#drivetraindescription-valid)
  - [`leftGeometry`](#drivetraindescription-leftgeometry)
  - [`rightGeometry`](#drivetraindescription-rightgeometry)
  - [`sideOf`](#drivetraindescription-sideof)
  - [`signedPortOf`](#drivetraindescription-signedportof)
  - [`describe`](#drivetraindescription-describe)
  - [`portMagnitude`](#drivetraindescription-portmagnitude)

<a id="kmaxsmartports"></a>

## `kMaxSmartPorts`

```cpp
inline constexpr std::size_t kMaxSmartPorts = 21
```

The most smart ports a V5 brain has, and therefore the most drive motors any side or any description can carry. Physical ports are numbered 1..kMaxSmartPorts — 0 is "no port".

*constant, declared at [`include/shulib/hal/drivetrain_description.hpp:56`](../../include/shulib/hal/drivetrain_description.hpp#L56).*

<a id="enum-class-drivetrainkind"></a>

## `enum class DrivetrainKind`

```cpp
enum class DrivetrainKind : std::uint8_t
```

Which drivetrain the description describes. APPEND-ONLY: the numeric values are written into a persisted record, so an existing name's value may never change. Holonomic kinds arrive with the X-drive in spring 2027 (R3c), and the calibrator refuses anything but Tank because a strafe scale needs measurements it does not yet take.

*enum class, declared at [`include/shulib/hal/drivetrain_description.hpp:62`](../../include/shulib/hal/drivetrain_description.hpp#L62).*

<a id="drivetrainkind-tank"></a>

### `DrivetrainKind::Tank`

```cpp
Tank = 1
```

two sides, no lateral authority — kinematics/tank.hpp

*enumerator, declared at [`include/shulib/hal/drivetrain_description.hpp:63`](../../include/shulib/hal/drivetrain_description.hpp#L63).*

<a id="enum-class-motorcartridge"></a>

## `enum class MotorCartridge`

```cpp
enum class MotorCartridge : std::uint8_t
```

The cartridge in the drive motors. HUMAN-STATED and unverifiable from inside: every cartridge spins the rotor at the same speed and the firmware divides by the gearset it was configured with, so a wrong belief is invisible to any reading the robot can take. Append-only values, for the same persisted-record reason as DrivetrainKind. There is deliberately no `Unset` member: a description carrying an unstated cartridge must be INVALID, not merely incomplete, and valid() refuses any byte outside this set.

*enum class, declared at [`include/shulib/hal/drivetrain_description.hpp:72`](../../include/shulib/hal/drivetrain_description.hpp#L72).*

<a id="motorcartridge-red"></a>

### `MotorCartridge::Red`

```cpp
Red = 1
```

100 rpm

*enumerator, declared at [`include/shulib/hal/drivetrain_description.hpp:73`](../../include/shulib/hal/drivetrain_description.hpp#L73).*

<a id="motorcartridge-green"></a>

### `MotorCartridge::Green`

```cpp
Green = 2
```

200 rpm

*enumerator, declared at [`include/shulib/hal/drivetrain_description.hpp:74`](../../include/shulib/hal/drivetrain_description.hpp#L74).*

<a id="motorcartridge-blue"></a>

### `MotorCartridge::Blue`

```cpp
Blue = 3
```

600 rpm

*enumerator, declared at [`include/shulib/hal/drivetrain_description.hpp:75`](../../include/shulib/hal/drivetrain_description.hpp#L75).*

<a id="cartridgeword"></a>

## `cartridgeWord`

```cpp
[[nodiscard]] inline const char* cartridgeWord(MotorCartridge c) noexcept
```

The cartridge as words, for banners — "UNSET" for any byte outside the enum, which is what a default-constructed or corrupted description carries.

*free function, declared at [`include/shulib/hal/drivetrain_description.hpp:80`](../../include/shulib/hal/drivetrain_description.hpp#L80).*

<a id="enum-class-descriptionsource"></a>

## `enum class DescriptionSource`

```cpp
enum class DescriptionSource : std::uint8_t
```

Where a description's numbers came from — printed in every banner, because "typed by a human from a tape measure" and "measured by the robot" are not the same claim.

*enum class, declared at [`include/shulib/hal/drivetrain_description.hpp:91`](../../include/shulib/hal/drivetrain_description.hpp#L91).*

<a id="descriptionsource-typedtable"></a>

### `DescriptionSource::TypedTable`

```cpp
TypedTable = 1
```

src/robots/chassis_table.hpp, via tableToDescription()

*enumerator, declared at [`include/shulib/hal/drivetrain_description.hpp:92`](../../include/shulib/hal/drivetrain_description.hpp#L92).*

<a id="descriptionsource-robotmeasured"></a>

### `DescriptionSource::RobotMeasured`

```cpp
RobotMeasured = 2
```

calibration::DrivetrainCalibrator, on a robot that moved

*enumerator, declared at [`include/shulib/hal/drivetrain_description.hpp:93`](../../include/shulib/hal/drivetrain_description.hpp#L93).*

<a id="struct-drivesidedescription"></a>

## `struct DriveSideDescription`

```cpp
struct DriveSideDescription
```

One side of a tank drivetrain: its drive ports WITH THEIR SIGNS, and the one number that converts that side's shaft radians into inches of travel.

*struct, declared at [`include/shulib/hal/drivetrain_description.hpp:98`](../../include/shulib/hal/drivetrain_description.hpp#L98).*

<a id="drivesidedescription-signedports"></a>

### `DriveSideDescription::signedPorts`

```cpp
std::int8_t signedPorts[kMaxSmartPorts]{}
```

The side's drive ports, |p| in 1..kMaxSmartPorts, SIGN = the PROS reversal: `-11` is port 11 constructed with a negative port number, which PROS reverses exactly once, in the adapter. THE SIGN LIVES HERE AND NOWHERE ELSE — a consumer hands the entry to its adapter AS STORED and never negates again (hal/pros/motor.hpp; hal/motor_group.hpp's "one place for a sign, or the sign gets applied twice"). Order is fixed but informational; a side is a set. Entries beyond `count` are 0 and are not read.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:105`](../../include/shulib/hal/drivetrain_description.hpp#L105).*

<a id="drivesidedescription-count"></a>

### `DriveSideDescription::count`

```cpp
std::uint8_t count = 0
```

How many entries of `signedPorts` are real. 0 = NO SIDE, which is invalid: a drivetrain with a side nobody found is not a drivetrain.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:108`](../../include/shulib/hal/drivetrain_description.hpp#L108).*

<a id="drivesidedescription-inchesperradian"></a>

### `DriveSideDescription::inchesPerRadian`

```cpp
units::Length inchesPerRadian{0.0}
```

Inches of wheel travel per radian of this side's motor output-shaft rotation — the whole conversion, as one measured number (hal::DriveGeometry::fromInchesPerRadian). Must be finite and > 0; 0 is UNSET and refuses rather than silently scaling travel by nothing.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:112`](../../include/shulib/hal/drivetrain_description.hpp#L112).*

<a id="struct-drivetraindescription"></a>

## `struct DrivetrainDescription`

```cpp
struct DrivetrainDescription
```

Everything a composition root needs to build a drivetrain's object graph, from whichever of the two sources produced it. Plain data (designated initializers at the call site), fixed width, allocation-free, serializable byte by byte. Nothing validates itself on construction — a caller builds one and then asks valid(), which is what lets a host test build a contradictory one by hand and watch it be refused.

*struct, declared at [`include/shulib/hal/drivetrain_description.hpp:120`](../../include/shulib/hal/drivetrain_description.hpp#L120).*

<a id="drivetraindescription-kind"></a>

### `DrivetrainDescription::kind`

```cpp
DrivetrainKind kind = DrivetrainKind::Tank
```

Which drivetrain this is. HUMAN-STATED.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:122`](../../include/shulib/hal/drivetrain_description.hpp#L122).*

<a id="drivetraindescription-cartridge"></a>

### `DrivetrainDescription::cartridge`

```cpp
MotorCartridge cartridge = static_cast<MotorCartridge>(0)
```

The cartridge in the drive motors. HUMAN-STATED (see MotorCartridge). Default-constructed as the byte 0, which is NOT a member of the enum and which valid() refuses on purpose: a description nobody filled in must never look filled in.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:126`](../../include/shulib/hal/drivetrain_description.hpp#L126).*

<a id="drivetraindescription-imuport"></a>

### `DrivetrainDescription::imuPort`

```cpp
std::uint8_t imuPort = 0
```

The IMU's smart port, 1..kMaxSmartPorts. 0 = UNSET, and invalid: every measurement this type carries was taken against an IMU, and a description without one cannot be re-checked.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:129`](../../include/shulib/hal/drivetrain_description.hpp#L129).*

<a id="drivetraindescription-left"></a>

### `DrivetrainDescription::left`

```cpp
DriveSideDescription left{}
```

The LEFT side — kinematic wheel 0 (kinematics/tank.hpp's wheel order).

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:131`](../../include/shulib/hal/drivetrain_description.hpp#L131).*

<a id="drivetraindescription-right"></a>

### `DrivetrainDescription::right`

```cpp
DriveSideDescription right{}
```

The RIGHT side — kinematic wheel 1.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:133`](../../include/shulib/hal/drivetrain_description.hpp#L133).*

<a id="drivetraindescription-trackwidth"></a>

### `DrivetrainDescription::trackWidth`

```cpp
units::Length trackWidth{0.0}
```

Contact line to contact line, inches, > 0. From a tape (TypedTable) or EFFECTIVE, from a powered spin compared against the IMU (RobotMeasured; A4 register HA-135) — and those are not the same number: a turning tank scrubs on foam, so the encoders over-count the turn and the measured value comes out LARGER than the tape's. The effective one is what TankKinematics needs to command a turn that lands, and what the odometry's heading cross-check reads as zero.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:140`](../../include/shulib/hal/drivetrain_description.hpp#L140).*

<a id="drivetraindescription-source"></a>

### `DrivetrainDescription::source`

```cpp
DescriptionSource source = DescriptionSource::TypedTable
```

Which producer filled this in.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:142`](../../include/shulib/hal/drivetrain_description.hpp#L142).*

<a id="drivetraindescription-provenance"></a>

### `DrivetrainDescription::provenance`

```cpp
char provenance[96]{}
```

Who measured what, how, and when — printed in every banner. A fixed-width, NUL-terminated buffer rather than a pointer, so the whole struct can be written to a card as it stands.

*field, declared at [`include/shulib/hal/drivetrain_description.hpp:145`](../../include/shulib/hal/drivetrain_description.hpp#L145).*

<a id="drivetraindescription-valid"></a>

### `DrivetrainDescription::valid`

```cpp
[[nodiscard]] bool valid(char* why = nullptr, std::size_t cap = 0) const noexcept
```

True when nothing in the description contradicts itself. `why` (optional; pass nullptr and 0 to skip) receives the FIRST contradiction found, naming the field or the port. Each check writes its own reason with a LITERAL format string rather than through a shared helper: the on-robot build gate compiles src/ under -Wformat-nonliteral, and a helper taking `const char* format` defeats the compiler's argument checking exactly where a wrong specifier would corrupt the one message a person gets when the robot refuses to boot. The checks are exactly the mistakes a typo, a mis-wire or a corrupted record produce: an unknown kind or cartridge byte; an IMU port outside 1..21; a side with no ports or with more than the brain has; an entry that is not a port in 1..21; a port listed twice on one side or once on each; the IMU port doubling as a drive port; a non-finite or non-positive inches-per-radian on either side; a non-finite or non-positive track width.

*function, declared at [`include/shulib/hal/drivetrain_description.hpp:158`](../../include/shulib/hal/drivetrain_description.hpp#L158).*

<a id="drivetraindescription-leftgeometry"></a>

### `DrivetrainDescription::leftGeometry`

```cpp
[[nodiscard]] constexpr DriveGeometry leftGeometry() const noexcept
```

The LEFT side's geometry for the odometry and the stall check — one object, built from the one measured number (DriveGeometry::fromInchesPerRadian, so `wheelRadius` carries the product). Only meaningful on a valid() description.

*function, declared at [`include/shulib/hal/drivetrain_description.hpp:290`](../../include/shulib/hal/drivetrain_description.hpp#L290).*

<a id="drivetraindescription-rightgeometry"></a>

### `DrivetrainDescription::rightGeometry`

```cpp
[[nodiscard]] constexpr DriveGeometry rightGeometry() const noexcept
```

The RIGHT side's geometry — a separate object, because an asymmetric drivetrain is representable and has been seen.

*function, declared at [`include/shulib/hal/drivetrain_description.hpp:296`](../../include/shulib/hal/drivetrain_description.hpp#L296).*

<a id="drivetraindescription-sideof"></a>

### `DrivetrainDescription::sideOf`

```cpp
[[nodiscard]] constexpr int sideOf(int port) const noexcept
```

−1 LEFT, +1 RIGHT, 0 = not a drive port in this description. Matches on |port|, so a signed entry and a physical port number find each other.

*function, declared at [`include/shulib/hal/drivetrain_description.hpp:302`](../../include/shulib/hal/drivetrain_description.hpp#L302).*

<a id="drivetraindescription-signedportof"></a>

### `DrivetrainDescription::signedPortOf`

```cpp
[[nodiscard]] constexpr std::int8_t signedPortOf(int port) const noexcept
```

This description's entry for physical `port` AS STORED — signed — or 0 when the port is not a drive port here. THIS is the value a composition root hands to its motor adapter: the one place a sign lives.

*function, declared at [`include/shulib/hal/drivetrain_description.hpp:319`](../../include/shulib/hal/drivetrain_description.hpp#L319).*

<a id="drivetraindescription-describe"></a>

### `DrivetrainDescription::describe`

```cpp
void describe(char* buf, std::size_t cap) const
```

The one-line panel text: source, kind, cartridge, port counts, both scales, the width. Truncates rather than overflowing; writes at least a NUL whenever cap > 0.

*function, declared at [`include/shulib/hal/drivetrain_description.hpp:335`](../../include/shulib/hal/drivetrain_description.hpp#L335).*

<a id="drivetraindescription-portmagnitude"></a>

### `DrivetrainDescription::portMagnitude`

```cpp
[[nodiscard]] static constexpr int portMagnitude(std::int8_t p) noexcept
```

|p| for a (possibly signed) port entry, as an int.

*function, declared at [`include/shulib/hal/drivetrain_description.hpp:348`](../../include/shulib/hal/drivetrain_description.hpp#L348).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 41 lines</summary>

```text

 DrivetrainDescription — the ONE representation of a drivetrain's discovered facts
 (chunk R3d Part 1, 2026-09-17).

 ── Why this type exists ────────────────────────────────────────────────────────────
 Almost every number a human is asked to type about a drivetrain is discoverable on the
 robot: which ports drive, which way each one is wired, which side each belongs to, how
 far the chassis travels per shaft radian, how wide the track effectively is. Only three
 facts must be STATED — the cartridge (physically unobservable: the rotor turns the same
 for every insert and the firmware already divides by whatever it was told), the
 drivetrain KIND, and which end is the FRONT. Everything else is a measurement, and this
 struct is where a measurement lands.

 It has more than one producer on purpose:
   * calibration::DrivetrainCalibrator fills it from a push, a set of pulses, a push
     along a tape and a powered spin      (source = RobotMeasured);
   * src/robots/chassis_table.hpp's tableToDescription() fills it from the typed table, which
     stays as the fallback                (source = TypedTable).
 One representation, two sources — so a composition root has exactly ONE input type.

 ── Laid out to be SERIALIZED, before anything serializes it ────────────────────────
 Fixed-width fields, bounded arrays, no pointers, no std::string, no allocation. That is
 not tidiness: a calibration record is read ONCE per boot, and a flipped bit in a sign or
 a scale drives the robot wrong SILENTLY — so the record that carries this will be
 written byte by byte under a CRC-32 and refused on mismatch. This header only defines
 the shape that makes that possible; nothing here writes a byte.

 ── One number lives here: kMaxSmartPorts ───────────────────────────────────────────
 A V5 brain has 21 smart ports. src/robots/chassis_table.hpp's kMaxPort now CITES the constant
 below rather than repeating it. teleop::kMaxSideMembers deliberately keeps its own copy:
 hal::MotorGroup already depends on teleop/coupled_side_monitor.hpp, so making the monitor
 depend on this header would close a directory-level cycle, and a pure fight detector has
 no business knowing what a drivetrain description is. The two constants are pinned EQUAL
 by a static_assert in the host test instead — a pin with no dependency.

 NO HARDWARE CLAIM IS MADE ANYWHERE IN THIS FILE. No description has ever been produced
 on a robot; the calibrator that fills one has never moved a physical drivetrain. Every
 property here is proven against the A2 host plant and nothing else.

 PROS-free, sim-free, allocation-free; safe to include from any layer that may include
 hal/.
```

</details>
