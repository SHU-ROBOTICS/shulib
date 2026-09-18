<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/config/robot_config.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `robot_config.hpp`

RobotConfig — a whole robot, described as DATA.

This header declares **17** types (75 members), **3** free functions, **3** constants, and **1** type alias.

Extracted from [`include/shulib/config/robot_config.hpp`](../../include/shulib/config/robot_config.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxMechanisms`](#kmaxmechanisms) — *constant*
- [`kMaxMotorsPerMechanism`](#kmaxmotorspermechanism) — *constant*
- [`kMaxPneumaticsPerMechanism`](#kmaxpneumaticspermechanism) — *constant*
- [`enum class DrivetrainKind`](#enum-class-drivetrainkind)
  - [`Tank`](#drivetrainkind-tank)
  - [`XDrive`](#drivetrainkind-xdrive)
  - [`HDrive`](#drivetrainkind-hdrive)
- [`drivetrainKindWord`](#drivetrainkindword) — *free function*
- [`struct RobotIdentity`](#struct-robotidentity)
  - [`name`](#robotidentity-name)
  - [`team`](#robotidentity-team)
  - [`season`](#robotidentity-season)
- [`struct TankDrivetrain`](#struct-tankdrivetrain)
  - [`description`](#tankdrivetrain-description)
- [`struct XDriveDrivetrain`](#struct-xdrivedrivetrain)
  - [`ports`](#xdrivedrivetrain-ports)
  - [`cartridge`](#xdrivedrivetrain-cartridge)
  - [`imuPort`](#xdrivedrivetrain-imuport)
  - [`driveRadius`](#xdrivedrivetrain-driveradius)
- [`struct HDriveDrivetrain`](#struct-hdrivedrivetrain)
  - [`ports`](#hdrivedrivetrain-ports)
  - [`cartridge`](#hdrivedrivetrain-cartridge)
  - [`imuPort`](#hdrivedrivetrain-imuport)
  - [`trackWidth`](#hdrivedrivetrain-trackwidth)
  - [`strafeWheelOffset`](#hdrivedrivetrain-strafewheeloffset)
- [`struct DrivetrainConfig`](#struct-drivetrainconfig)
  - [`kind`](#drivetrainconfig-kind)
  - [`tank`](#drivetrainconfig-tank)
  - [`xdrive`](#drivetrainconfig-xdrive)
  - [`hdrive`](#drivetrainconfig-hdrive)
  - [`kindOf`](#drivetrainconfig-kindof)
  - [`imuPort`](#drivetrainconfig-imuport)
  - [`cartridge`](#drivetrainconfig-cartridge)
  - [`wheelCount`](#drivetrainconfig-wheelcount)
  - [`driveMotorCount`](#drivetrainconfig-drivemotorcount)
- [`enum class OdometryKind`](#enum-class-odometrykind)
  - [`DriveEncoders`](#odometrykind-driveencoders)
  - [`TrackingWheels`](#odometrykind-trackingwheels)
- [`odometryKindWord`](#odometrykindword) — *free function*
- [`struct TrackingWheelConfig`](#struct-trackingwheelconfig)
  - [`port`](#trackingwheelconfig-port)
  - [`wheelDiameter`](#trackingwheelconfig-wheeldiameter)
  - [`offset`](#trackingwheelconfig-offset)
- [`struct TrackingWheelsOdometry`](#struct-trackingwheelsodometry)
  - [`forward`](#trackingwheelsodometry-forward)
  - [`lateral`](#trackingwheelsodometry-lateral)
- [`struct OdometryConfig`](#struct-odometryconfig)
  - [`kind`](#odometryconfig-kind)
  - [`wheels`](#odometryconfig-wheels)
- [`struct GpsConfig`](#struct-gpsconfig)
  - [`present`](#gpsconfig-present)
  - [`port`](#gpsconfig-port)
  - [`leverArmForward`](#gpsconfig-leverarmforward)
  - [`leverArmLeft`](#gpsconfig-leverarmleft)
- [`struct SensorsConfig`](#struct-sensorsconfig)
  - [`gps`](#sensorsconfig-gps)
- [`struct MechanismConfig`](#struct-mechanismconfig)
  - [`name`](#mechanismconfig-name)
  - [`motorPorts`](#mechanismconfig-motorports)
  - [`motorCount`](#mechanismconfig-motorcount)
  - [`pneumaticPorts`](#mechanismconfig-pneumaticports)
  - [`pneumaticCount`](#mechanismconfig-pneumaticcount)
- [`TuningConfig`](#tuningconfig) — *type alias*
- [`enum class PortUse`](#enum-class-portuse)
  - [`Free`](#portuse-free)
  - [`DriveLeft`](#portuse-driveleft)
  - [`DriveRight`](#portuse-driveright)
  - [`DriveWheel`](#portuse-drivewheel)
  - [`Imu`](#portuse-imu)
  - [`PodForward`](#portuse-podforward)
  - [`PodLateral`](#portuse-podlateral)
  - [`Gps`](#portuse-gps)
  - [`Mechanism`](#portuse-mechanism)
- [`portUseWord`](#portuseword) — *free function*
- [`struct RobotConfig`](#struct-robotconfig)
  - [`identity`](#robotconfig-identity)
  - [`drivetrain`](#robotconfig-drivetrain)
  - [`odometry`](#robotconfig-odometry)
  - [`sensors`](#robotconfig-sensors)
  - [`mechanisms`](#robotconfig-mechanisms)
  - [`mechanismCount`](#robotconfig-mechanismcount)
  - [`tuning`](#robotconfig-tuning)
  - [`valid`](#robotconfig-valid)
  - [`describe`](#robotconfig-describe)
  - [`validCartridge`](#robotconfig-validcartridge)
  - [`validImuPort`](#robotconfig-validimuport)
  - [`validWheelPort`](#robotconfig-validwheelport)
  - [`validPod`](#robotconfig-validpod)
- [`class IRobotConfig`](#class-irobotconfig)
  - [`~IRobotConfig`](#irobotconfig-destructor-irobotconfig)
  - [`IRobotConfig`](#irobotconfig-irobotconfig)
  - [`IRobotConfig (overload 2)`](#irobotconfig-irobotconfig-2)
  - [`IRobotConfig (overload 3)`](#irobotconfig-irobotconfig-3)
  - [`operator=`](#irobotconfig-operator-eq)
  - [`operator= (overload 2)`](#irobotconfig-operator-eq-2)
  - [`config`](#irobotconfig-config)
- [`class StaticRobotConfig`](#class-staticrobotconfig)
  - [`StaticRobotConfig`](#staticrobotconfig-staticrobotconfig)
  - [`config`](#staticrobotconfig-config)

<a id="kmaxmechanisms"></a>

## `kMaxMechanisms`

```cpp
inline constexpr std::size_t kMaxMechanisms = 8
```

The most mechanisms one robot may declare. A DESIGN constant (ruling B6), not a hardware belief: nothing measured says eight, and a robot with more mechanisms than this is asking for a bigger number here, not for a different data structure.

*constant, declared at [`include/shulib/config/robot_config.hpp:58`](../../include/shulib/config/robot_config.hpp#L58).*

<a id="kmaxmotorspermechanism"></a>

## `kMaxMotorsPerMechanism`

```cpp
inline constexpr std::size_t kMaxMotorsPerMechanism = 4
```

The most motors one mechanism may drive. Design constant (B6), same reasoning as above.

*constant, declared at [`include/shulib/config/robot_config.hpp:61`](../../include/shulib/config/robot_config.hpp#L61).*

<a id="kmaxpneumaticspermechanism"></a>

## `kMaxPneumaticsPerMechanism`

```cpp
inline constexpr std::size_t kMaxPneumaticsPerMechanism = 2
```

The most pneumatic (ADI digital-out) lines one mechanism may actuate. Design constant (B6).

*constant, declared at [`include/shulib/config/robot_config.hpp:64`](../../include/shulib/config/robot_config.hpp#L64).*

<a id="enum-class-drivetrainkind"></a>

## `enum class DrivetrainKind`

```cpp
enum class DrivetrainKind : std::uint8_t
```

Which drivetrain a RobotConfig describes.  DELIBERATELY NOT `hal::DrivetrainKind`, and the distinction matters: that enum is the one written into a persisted calibration record, it is append-only for that reason, and it knows only `Tank` because the calibrator can only measure a tank today. THIS enum is the CONFIG vocabulary and already carries the two holonomic kinds, because src/main.cpp's X-drive graph has existed since R1a and must be expressible as data (Part 2). A tank config carries a `hal::DrivetrainDescription`, whose own `kind` field is the persisted one; the two agree by construction because valid() checks them against each other.

*enum class, declared at [`include/shulib/config/robot_config.hpp:75`](../../include/shulib/config/robot_config.hpp#L75).*

<a id="drivetrainkind-tank"></a>

### `DrivetrainKind::Tank`

```cpp
Tank = 1
```

two sides, no lateral authority — kinematics/tank.hpp

*enumerator, declared at [`include/shulib/config/robot_config.hpp:76`](../../include/shulib/config/robot_config.hpp#L76).*

<a id="drivetrainkind-xdrive"></a>

### `DrivetrainKind::XDrive`

```cpp
XDrive = 2
```

four 45° omnis — kinematics/x_drive.hpp

*enumerator, declared at [`include/shulib/config/robot_config.hpp:77`](../../include/shulib/config/robot_config.hpp#L77).*

<a id="drivetrainkind-hdrive"></a>

### `DrivetrainKind::HDrive`

```cpp
HDrive = 3
```

tank base + one transverse strafe wheel — kinematics/h_drive.hpp

*enumerator, declared at [`include/shulib/config/robot_config.hpp:78`](../../include/shulib/config/robot_config.hpp#L78).*

<a id="drivetrainkindword"></a>

## `drivetrainKindWord`

```cpp
[[nodiscard]] inline const char* drivetrainKindWord(DrivetrainKind k) noexcept
```

The drivetrain kind as a word, for banners and refusal messages. "UNSET" for any byte outside the enum, which is what a default-constructed config carries.

*free function, declared at [`include/shulib/config/robot_config.hpp:83`](../../include/shulib/config/robot_config.hpp#L83).*

<a id="struct-robotidentity"></a>

## `struct RobotIdentity`

```cpp
struct RobotIdentity
```

Who this robot is, in words. Banner material only — nothing in the library branches on it. Fixed-width and NUL-terminated so the whole config can be written to a card as it stands.

*struct, declared at [`include/shulib/config/robot_config.hpp:94`](../../include/shulib/config/robot_config.hpp#L94).*

<a id="robotidentity-name"></a>

### `RobotIdentity::name`

```cpp
char name[32]{}
```

The robot's name, e.g. "Robot Two". Empty is legal (an unnamed robot still drives).

*field, declared at [`include/shulib/config/robot_config.hpp:96`](../../include/shulib/config/robot_config.hpp#L96).*

<a id="robotidentity-team"></a>

### `RobotIdentity::team`

```cpp
char team[16]{}
```

The team number, e.g. "1234A".

*field, declared at [`include/shulib/config/robot_config.hpp:98`](../../include/shulib/config/robot_config.hpp#L98).*

<a id="robotidentity-season"></a>

### `RobotIdentity::season`

```cpp
char season[16]{}
```

The season, e.g. "Push Back".

*field, declared at [`include/shulib/config/robot_config.hpp:100`](../../include/shulib/config/robot_config.hpp#L100).*

<a id="struct-tankdrivetrain"></a>

## `struct TankDrivetrain`

```cpp
struct TankDrivetrain
```

A tank drivetrain: R3d's `hal::DrivetrainDescription`, EMBEDDED BY VALUE and unchanged (ruling B1). Everything a tank needs is already in it — signed ports per side, inches per radian per side, effective track width, cartridge, IMU port, source and provenance — and it is the type the calibrator produces and the record persists, so a config built from a card is a field copy and nothing else.

*struct, declared at [`include/shulib/config/robot_config.hpp:108`](../../include/shulib/config/robot_config.hpp#L108).*

<a id="tankdrivetrain-description"></a>

### `TankDrivetrain::description`

```cpp
hal::DrivetrainDescription description{}
```

The description, exactly as the calibrator or the typed table produced it. Its own `valid()` is called by RobotConfig::valid(), and its message surfaces verbatim.

*field, declared at [`include/shulib/config/robot_config.hpp:111`](../../include/shulib/config/robot_config.hpp#L111).*

<a id="struct-xdrivedrivetrain"></a>

## `struct XDriveDrivetrain`

```cpp
struct XDriveDrivetrain
```

A symmetric 45° X-drive, typed by hand (no calibrator measures one yet).

*struct, declared at [`include/shulib/config/robot_config.hpp:115`](../../include/shulib/config/robot_config.hpp#L115).*

<a id="xdrivedrivetrain-ports"></a>

### `XDriveDrivetrain::ports`

```cpp
std::int8_t ports[4]{}
```

The four wheel ports in x_drive.hpp's canonical F5 wheel order — 0 front-left, 1 back-left, 2 back-right, 3 front-right — WITH THEIR SIGNS, exactly as `hal::DriveSideDescription::signedPorts` carries a tank's: `-11` is port 11 handed to the adapter negative, which PROS reverses exactly once. The sign lives here and nowhere else. A `0` entry is "no motor on that wheel", which is representable and refused by `robot::validate()`'s wheel-count cross-check, not here (see RobotConfig::valid()).

*field, declared at [`include/shulib/config/robot_config.hpp:122`](../../include/shulib/config/robot_config.hpp#L122).*

<a id="xdrivedrivetrain-cartridge"></a>

### `XDriveDrivetrain::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

The cartridge in the drive motors. HUMAN-STATED and unverifiable from inside, exactly as `hal::MotorCartridge` documents; `valid()` refuses any byte outside the enum.

*field, declared at [`include/shulib/config/robot_config.hpp:125`](../../include/shulib/config/robot_config.hpp#L125).*

<a id="xdrivedrivetrain-imuport"></a>

### `XDriveDrivetrain::imuPort`

```cpp
std::uint8_t imuPort = 0
```

The IMU's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid — every drivetrain here is heading-owned by an IMU.

*field, declared at [`include/shulib/config/robot_config.hpp:128`](../../include/shulib/config/robot_config.hpp#L128).*

<a id="xdrivedrivetrain-driveradius"></a>

### `XDriveDrivetrain::driveRadius`

```cpp
units::Length driveRadius{0.0}
```

Centre-to-wheel distance in inches, the ONE geometry input `kinematics::xDrive()` takes. Must be finite and > 0.

*field, declared at [`include/shulib/config/robot_config.hpp:131`](../../include/shulib/config/robot_config.hpp#L131).*

<a id="struct-hdrivedrivetrain"></a>

## `struct HDriveDrivetrain`

```cpp
struct HDriveDrivetrain
```

An H-drive: a tank base plus one transverse strafe wheel, typed by hand.

*struct, declared at [`include/shulib/config/robot_config.hpp:135`](../../include/shulib/config/robot_config.hpp#L135).*

<a id="hdrivedrivetrain-ports"></a>

### `HDriveDrivetrain::ports`

```cpp
std::int8_t ports[3]{}
```

The three wheel ports in h_drive.hpp's wheel order — 0 LEFT drive line, 1 RIGHT drive line, 2 STRAFE — signed, with the same one-place-for-a-sign rule as the X-drive's. Spelling the strafe wheel as `ports[2]` rather than a separate field is deliberate: the kinematics numbers it as wheel 2, and a separate field would need a second rule saying where it goes in the motor array.

*field, declared at [`include/shulib/config/robot_config.hpp:141`](../../include/shulib/config/robot_config.hpp#L141).*

<a id="hdrivedrivetrain-cartridge"></a>

### `HDriveDrivetrain::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

The cartridge in the drive motors. HUMAN-STATED; `valid()` refuses a byte outside the enum.

*field, declared at [`include/shulib/config/robot_config.hpp:143`](../../include/shulib/config/robot_config.hpp#L143).*

<a id="hdrivedrivetrain-imuport"></a>

### `HDriveDrivetrain::imuPort`

```cpp
std::uint8_t imuPort = 0
```

The IMU's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid.

*field, declared at [`include/shulib/config/robot_config.hpp:145`](../../include/shulib/config/robot_config.hpp#L145).*

<a id="hdrivedrivetrain-trackwidth"></a>

### `HDriveDrivetrain::trackWidth`

```cpp
units::Length trackWidth{0.0}
```

Contact line to contact line, inches, finite and > 0 — `hDrive()`'s `trackWidth`.

*field, declared at [`include/shulib/config/robot_config.hpp:147`](../../include/shulib/config/robot_config.hpp#L147).*

<a id="hdrivedrivetrain-strafewheeloffset"></a>

### `HDriveDrivetrain::strafeWheelOffset`

```cpp
units::Length strafeWheelOffset{0.0}
```

The strafe wheel's +FORWARD body coordinate, SIGNED and finite (negative = behind centre) — `hDrive()`'s `strafeWheelOffset`.

*field, declared at [`include/shulib/config/robot_config.hpp:150`](../../include/shulib/config/robot_config.hpp#L150).*

<a id="struct-drivetrainconfig"></a>

## `struct DrivetrainConfig`

```cpp
struct DrivetrainConfig
```

Which drivetrain this robot has, and its numbers.  A TAGGED STRUCT rather than a union: all three alternatives are stored and `kind` says which one is read. The alternative — `std::variant` or a real union — was rejected because it costs this type its aggregate-ness, and being an aggregate is precisely what lets a person (and G3's generator) write `{.kind = …, .tank = {…}}` with designated initializers and have the compiler check it. The price is about thirty bytes of static storage on a brain that has megabytes, paid once.

*struct, declared at [`include/shulib/config/robot_config.hpp:161`](../../include/shulib/config/robot_config.hpp#L161).*

<a id="drivetrainconfig-kind"></a>

### `DrivetrainConfig::kind`

```cpp
DrivetrainKind kind = DrivetrainKind::Tank
```

Which of the three members below is the real one. Everything else is ignored — not validated, not read, not built.

*field, declared at [`include/shulib/config/robot_config.hpp:164`](../../include/shulib/config/robot_config.hpp#L164).*

<a id="drivetrainconfig-tank"></a>

### `DrivetrainConfig::tank`

```cpp
TankDrivetrain tank{}
```

The tank alternative, read only when `kind == Tank`.

*field, declared at [`include/shulib/config/robot_config.hpp:166`](../../include/shulib/config/robot_config.hpp#L166).*

<a id="drivetrainconfig-xdrive"></a>

### `DrivetrainConfig::xdrive`

```cpp
XDriveDrivetrain xdrive{}
```

The X-drive alternative, read only when `kind == XDrive`.

*field, declared at [`include/shulib/config/robot_config.hpp:168`](../../include/shulib/config/robot_config.hpp#L168).*

<a id="drivetrainconfig-hdrive"></a>

### `DrivetrainConfig::hdrive`

```cpp
HDriveDrivetrain hdrive{}
```

The H-drive alternative, read only when `kind == HDrive`.

*field, declared at [`include/shulib/config/robot_config.hpp:170`](../../include/shulib/config/robot_config.hpp#L170).*

<a id="drivetrainconfig-kindof"></a>

### `DrivetrainConfig::kindOf`

```cpp
[[nodiscard]] constexpr DrivetrainKind kindOf() const noexcept
```

Which alternative is live — the accessor the brief names, so call sites read `drivetrain.kind()`-shaped code even though the tag is a plain field.

*function, declared at [`include/shulib/config/robot_config.hpp:174`](../../include/shulib/config/robot_config.hpp#L174).*

<a id="drivetrainconfig-imuport"></a>

### `DrivetrainConfig::imuPort`

```cpp
[[nodiscard]] constexpr std::uint8_t imuPort() const noexcept
```

The IMU port this drivetrain's heading comes from, whichever alternative is live. 0 when `kind` is not a known kind.

*function, declared at [`include/shulib/config/robot_config.hpp:178`](../../include/shulib/config/robot_config.hpp#L178).*

<a id="drivetrainconfig-cartridge"></a>

### `DrivetrainConfig::cartridge`

```cpp
[[nodiscard]] constexpr hal::MotorCartridge cartridge() const noexcept
```

The cartridge in the drive motors, whichever alternative is live. The byte 0 (UNSET) when `kind` is not a known kind.

*function, declared at [`include/shulib/config/robot_config.hpp:189`](../../include/shulib/config/robot_config.hpp#L189).*

<a id="drivetrainconfig-wheelcount"></a>

### `DrivetrainConfig::wheelCount`

```cpp
[[nodiscard]] constexpr int wheelCount() const noexcept
```

How many KINEMATIC WHEELS the installed drivetrain has — 2 tank, 4 X-drive, 3 H-drive. This is the number `kinematics::IKinematics::wheelCount()` will report, derived from the config so `robot::validate()` can make the F5 cross-check before any object exists. 0 when `kind` is not a known kind.

*function, declared at [`include/shulib/config/robot_config.hpp:202`](../../include/shulib/config/robot_config.hpp#L202).*

<a id="drivetrainconfig-drivemotorcount"></a>

### `DrivetrainConfig::driveMotorCount`

```cpp
[[nodiscard]] constexpr int driveMotorCount() const noexcept
```

How many `hal::IMotor` handles the builder will place in the RobotContext — ONE per kinematic wheel when the config is complete. A tank always contributes 2 (one MotorGroup per side, and a side with no ports is already refused by the description's own `valid()`); a holonomic drivetrain contributes one per NON-ZERO port slot, so a wheel whose port was left at 0 makes this number smaller than `wheelCount()` — which is exactly the disagreement `robot::validate()` exists to name.

*function, declared at [`include/shulib/config/robot_config.hpp:217`](../../include/shulib/config/robot_config.hpp#L217).*

<a id="enum-class-odometrykind"></a>

## `enum class OdometryKind`

```cpp
enum class OdometryKind : std::uint8_t
```

Which estimator feeds the Localizer.

*enum class, declared at [`include/shulib/config/robot_config.hpp:244`](../../include/shulib/config/robot_config.hpp#L244).*

<a id="odometrykind-driveencoders"></a>

### `OdometryKind::DriveEncoders`

```cpp
DriveEncoders = 1
```

The drive encoders themselves — `localization::DriveEncoderOdometry` over the two side groups. Needs no extra hardware and is the default for a tank. NOT available on a holonomic drivetrain: that estimator reads exactly two sides and an X-drive has none.

*enumerator, declared at [`include/shulib/config/robot_config.hpp:248`](../../include/shulib/config/robot_config.hpp#L248).*

<a id="odometrykind-trackingwheels"></a>

### `OdometryKind::TrackingWheels`

```cpp
TrackingWheels = 2
```

Two unpowered tracking wheels — `localization::PilonsOdometry`. The only choice for a holonomic drivetrain, and the better one for a tank that has the pods.

*enumerator, declared at [`include/shulib/config/robot_config.hpp:251`](../../include/shulib/config/robot_config.hpp#L251).*

<a id="odometrykindword"></a>

## `odometryKindWord`

```cpp
[[nodiscard]] inline const char* odometryKindWord(OdometryKind k) noexcept
```

The odometry kind as a word, for banners. "UNSET" for a byte outside the enum.

*free function, declared at [`include/shulib/config/robot_config.hpp:255`](../../include/shulib/config/robot_config.hpp#L255).*

<a id="struct-trackingwheelconfig"></a>

## `struct TrackingWheelConfig`

```cpp
struct TrackingWheelConfig
```

One unpowered tracking wheel: where it is plugged in, how big it is, and where it sits. The role (forward or lateral) is NOT a field — it is fixed by which slot of `TrackingWheelsOdometry` the wheel occupies, exactly as `localization::TrackingWheel`'s private constructor and two factories fix it, so a pair handed over in the wrong order cannot be represented rather than merely being rejected.

*struct, declared at [`include/shulib/config/robot_config.hpp:268`](../../include/shulib/config/robot_config.hpp#L268).*

<a id="trackingwheelconfig-port"></a>

### `TrackingWheelConfig::port`

```cpp
std::uint8_t port = 0
```

The rotation sensor's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid. Unsigned: a rotation sensor's direction is a mounting fact expressed by the wheel's offset sign and the sensor's own reading, not by a negative port.

*field, declared at [`include/shulib/config/robot_config.hpp:272`](../../include/shulib/config/robot_config.hpp#L272).*

<a id="trackingwheelconfig-wheeldiameter"></a>

### `TrackingWheelConfig::wheelDiameter`

```cpp
units::Length wheelDiameter{0.0}
```

Wheel diameter in inches, finite and > 0.

*field, declared at [`include/shulib/config/robot_config.hpp:274`](../../include/shulib/config/robot_config.hpp#L274).*

<a id="trackingwheelconfig-offset"></a>

### `TrackingWheelConfig::offset`

```cpp
units::Length offset{0.0}
```

The wheel's signed offset from the tracking centre, inches, finite — the +LEFT coordinate for the FORWARD wheel and the +FORWARD coordinate for the LATERAL one (tracking_wheel.hpp).

*field, declared at [`include/shulib/config/robot_config.hpp:277`](../../include/shulib/config/robot_config.hpp#L277).*

<a id="struct-trackingwheelsodometry"></a>

## `struct TrackingWheelsOdometry`

```cpp
struct TrackingWheelsOdometry
```

The two-pod (Pilons) odometry's geometry.

*struct, declared at [`include/shulib/config/robot_config.hpp:281`](../../include/shulib/config/robot_config.hpp#L281).*

<a id="trackingwheelsodometry-forward"></a>

### `TrackingWheelsOdometry::forward`

```cpp
TrackingWheelConfig forward{}
```

The FORWARD-rolling wheel (body +X); its `offset` is its +LEFT coordinate.

*field, declared at [`include/shulib/config/robot_config.hpp:283`](../../include/shulib/config/robot_config.hpp#L283).*

<a id="trackingwheelsodometry-lateral"></a>

### `TrackingWheelsOdometry::lateral`

```cpp
TrackingWheelConfig lateral{}
```

The LATERAL-rolling wheel (body +Y); its `offset` is its +FORWARD coordinate.

*field, declared at [`include/shulib/config/robot_config.hpp:285`](../../include/shulib/config/robot_config.hpp#L285).*

<a id="struct-odometryconfig"></a>

## `struct OdometryConfig`

```cpp
struct OdometryConfig
```

Which estimator this robot runs, and its geometry.  Same tagged-struct shape and the same reasoning as DrivetrainConfig. Note what this means for a tank that HAS pods but is configured `DriveEncoders`: the `wheels` member is simply not read — not validated, not port-claimed, not built. Declaring pods and then choosing drive encoders is therefore a no-op, not an error; the brief calls that a warning rather than a refusal, and the honest place for a warning is the boot banner, not a validity rule that would make an over-complete config invalid.

*struct, declared at [`include/shulib/config/robot_config.hpp:296`](../../include/shulib/config/robot_config.hpp#L296).*

<a id="odometryconfig-kind"></a>

### `OdometryConfig::kind`

```cpp
OdometryKind kind = OdometryKind::DriveEncoders
```

Which estimator to build.

*field, declared at [`include/shulib/config/robot_config.hpp:298`](../../include/shulib/config/robot_config.hpp#L298).*

<a id="odometryconfig-wheels"></a>

### `OdometryConfig::wheels`

```cpp
TrackingWheelsOdometry wheels{}
```

The tracking-wheel geometry, read only when `kind == TrackingWheels`.

*field, declared at [`include/shulib/config/robot_config.hpp:300`](../../include/shulib/config/robot_config.hpp#L300).*

<a id="struct-gpsconfig"></a>

## `struct GpsConfig`

```cpp
struct GpsConfig
```

The GPS, if this robot has one.  ABSENCE IS A FIELD, NOT A ZERO PORT. `present = false` means the robot has no GPS, and robot/robot.hpp answers that with the explicit `hal::AbsentGps` in the context and NO corrector installed — never a null and never a fake (landmine 5). A GPS-less robot is the common case here: robot two has none, and Driving Skills has no field strip at all.

*struct, declared at [`include/shulib/config/robot_config.hpp:309`](../../include/shulib/config/robot_config.hpp#L309).*

<a id="gpsconfig-present"></a>

### `GpsConfig::present`

```cpp
bool present = false
```

Whether a GPS is installed. When false every other field is ignored.

*field, declared at [`include/shulib/config/robot_config.hpp:311`](../../include/shulib/config/robot_config.hpp#L311).*

<a id="gpsconfig-port"></a>

### `GpsConfig::port`

```cpp
std::uint8_t port = 0
```

The GPS's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid when `present`.

*field, declared at [`include/shulib/config/robot_config.hpp:313`](../../include/shulib/config/robot_config.hpp#L313).*

<a id="gpsconfig-leverarmforward"></a>

### `GpsConfig::leverArmForward`

```cpp
units::Length leverArmForward{0.0}
```

The sensor's +FORWARD body coordinate from the robot centre, inches, finite.

*field, declared at [`include/shulib/config/robot_config.hpp:315`](../../include/shulib/config/robot_config.hpp#L315).*

<a id="gpsconfig-leverarmleft"></a>

### `GpsConfig::leverArmLeft`

```cpp
units::Length leverArmLeft{0.0}
```

The sensor's +LEFT body coordinate from the robot centre, inches, finite.

*field, declared at [`include/shulib/config/robot_config.hpp:317`](../../include/shulib/config/robot_config.hpp#L317).*

<a id="struct-sensorsconfig"></a>

## `struct SensorsConfig`

```cpp
struct SensorsConfig
```

Every sensor this robot has that is not part of the drivetrain or the odometry.  TAGS AND VISION ARE DELIBERATELY ABSENT IN PART 1, and that is a statement about the tree rather than about the schema: no camera adapter exists (R2 owns it), so a field here would describe hardware the library cannot open. `robot::Robot` installs `hal::AbsentTagSource` and `hal::AbsentVision` unconditionally, exactly as both src/main.cpp graphs do today.

*struct, declared at [`include/shulib/config/robot_config.hpp:326`](../../include/shulib/config/robot_config.hpp#L326).*

<a id="sensorsconfig-gps"></a>

### `SensorsConfig::gps`

```cpp
GpsConfig gps{}
```

The GPS, present or absent.

*field, declared at [`include/shulib/config/robot_config.hpp:328`](../../include/shulib/config/robot_config.hpp#L328).*

<a id="struct-mechanismconfig"></a>

## `struct MechanismConfig`

```cpp
struct MechanismConfig
```

One mechanism — an intake, a lift, a clamp — as data.  TYPES ONLY IN PART 1, and this is the one place in the header where a field exists that nothing reads yet. It is here so F7's schema is complete for G3's codegen and so a `.vexbot` round-trip does not lose the mechanisms it carries; `robot::Robot` constructs NO mechanism from it, and `valid()` checks only that the ports do not collide with anything else on the robot. G2/T3 consume it, with the command registry that binds a mechanism to a marker id.

*struct, declared at [`include/shulib/config/robot_config.hpp:338`](../../include/shulib/config/robot_config.hpp#L338).*

<a id="mechanismconfig-name"></a>

### `MechanismConfig::name`

```cpp
char name[24]{}
```

The mechanism's name — also the key a command-id registry will bind against. Non-empty for any mechanism inside `RobotConfig::mechanismCount`.

*field, declared at [`include/shulib/config/robot_config.hpp:341`](../../include/shulib/config/robot_config.hpp#L341).*

<a id="mechanismconfig-motorports"></a>

### `MechanismConfig::motorPorts`

```cpp
std::int8_t motorPorts[kMaxMotorsPerMechanism]{}
```

The mechanism's motor ports WITH THEIR SIGNS, same one-place-for-a-sign rule as the drivetrain's. Entries beyond `motorCount` are 0 and are not read.

*field, declared at [`include/shulib/config/robot_config.hpp:344`](../../include/shulib/config/robot_config.hpp#L344).*

<a id="mechanismconfig-motorcount"></a>

### `MechanismConfig::motorCount`

```cpp
std::uint8_t motorCount = 0
```

How many entries of `motorPorts` are real. 0 is legal — a purely pneumatic mechanism.

*field, declared at [`include/shulib/config/robot_config.hpp:346`](../../include/shulib/config/robot_config.hpp#L346).*

<a id="mechanismconfig-pneumaticports"></a>

### `MechanismConfig::pneumaticPorts`

```cpp
std::uint8_t pneumaticPorts[kMaxPneumaticsPerMechanism]{}
```

The mechanism's pneumatic (ADI digital-out) lines, 1..8 on the brain's three-wire port. Entries beyond `pneumaticCount` are 0 and are not read.

*field, declared at [`include/shulib/config/robot_config.hpp:349`](../../include/shulib/config/robot_config.hpp#L349).*

<a id="mechanismconfig-pneumaticcount"></a>

### `MechanismConfig::pneumaticCount`

```cpp
std::uint8_t pneumaticCount = 0
```

How many entries of `pneumaticPorts` are real. 0 is legal — a purely motorized mechanism.

*field, declared at [`include/shulib/config/robot_config.hpp:351`](../../include/shulib/config/robot_config.hpp#L351).*

<a id="tuningconfig"></a>

## `TuningConfig`

```cpp
using TuningConfig = chassis::ChassisConfig
```

The motion layer's gains, budgets and fault policy for this robot — `chassis::ChassisConfig` passed through WHOLE, not mirrored, for exactly the reason ruling B1 gives for the drivetrain description: a second struct with the same fields drifts. Every value in it is PROVISIONAL until R5 tunes on hardware (HA-45/50/51/52).

*type alias, declared at [`include/shulib/config/robot_config.hpp:358`](../../include/shulib/config/robot_config.hpp#L358).*

<a id="enum-class-portuse"></a>

## `enum class PortUse`

```cpp
enum class PortUse : std::uint8_t
```

What claims a smart port, for the one-owner-per-port check. Ordered by nothing; the values are interior to `RobotConfig::valid()` and are not persisted anywhere.

*enum class, declared at [`include/shulib/config/robot_config.hpp:362`](../../include/shulib/config/robot_config.hpp#L362).*

<a id="portuse-free"></a>

### `PortUse::Free`

```cpp
Free = 0
```

nothing claims this port

*enumerator, declared at [`include/shulib/config/robot_config.hpp:363`](../../include/shulib/config/robot_config.hpp#L363).*

<a id="portuse-driveleft"></a>

### `PortUse::DriveLeft`

```cpp
DriveLeft
```

a tank LEFT-side drive motor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:364`](../../include/shulib/config/robot_config.hpp#L364).*

<a id="portuse-driveright"></a>

### `PortUse::DriveRight`

```cpp
DriveRight
```

a tank RIGHT-side drive motor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:365`](../../include/shulib/config/robot_config.hpp#L365).*

<a id="portuse-drivewheel"></a>

### `PortUse::DriveWheel`

```cpp
DriveWheel
```

a holonomic drive motor (X-drive or H-drive wheel)

*enumerator, declared at [`include/shulib/config/robot_config.hpp:366`](../../include/shulib/config/robot_config.hpp#L366).*

<a id="portuse-imu"></a>

### `PortUse::Imu`

```cpp
Imu
```

the drivetrain's inertial sensor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:367`](../../include/shulib/config/robot_config.hpp#L367).*

<a id="portuse-podforward"></a>

### `PortUse::PodForward`

```cpp
PodForward
```

the forward tracking wheel's rotation sensor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:368`](../../include/shulib/config/robot_config.hpp#L368).*

<a id="portuse-podlateral"></a>

### `PortUse::PodLateral`

```cpp
PodLateral
```

the lateral tracking wheel's rotation sensor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:369`](../../include/shulib/config/robot_config.hpp#L369).*

<a id="portuse-gps"></a>

### `PortUse::Gps`

```cpp
Gps
```

the GPS

*enumerator, declared at [`include/shulib/config/robot_config.hpp:370`](../../include/shulib/config/robot_config.hpp#L370).*

<a id="portuse-mechanism"></a>

### `PortUse::Mechanism`

```cpp
Mechanism
```

a mechanism motor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:371`](../../include/shulib/config/robot_config.hpp#L371).*

<a id="portuseword"></a>

## `portUseWord`

```cpp
[[nodiscard]] inline const char* portUseWord(PortUse u) noexcept
```

A port claim as words, for the collision message — "port 7 is claimed by the LEFT drive and by a mechanism motor" reads as an instruction to go and fix one of them.

*free function, declared at [`include/shulib/config/robot_config.hpp:376`](../../include/shulib/config/robot_config.hpp#L376).*

<a id="struct-robotconfig"></a>

## `struct RobotConfig`

```cpp
struct RobotConfig
```

EVERYTHING about one robot, as one value.  Plain data with designated initializers at the call site, like every other config type in this tree. Nothing validates itself on construction: a caller builds one and then asks `valid()`, which is what lets a host test build a deliberately contradictory config by hand and watch it be refused by name.

*struct, declared at [`include/shulib/config/robot_config.hpp:397`](../../include/shulib/config/robot_config.hpp#L397).*

<a id="robotconfig-identity"></a>

### `RobotConfig::identity`

```cpp
RobotIdentity identity{}
```

Who this robot is — banner material.

*field, declared at [`include/shulib/config/robot_config.hpp:399`](../../include/shulib/config/robot_config.hpp#L399).*

<a id="robotconfig-drivetrain"></a>

### `RobotConfig::drivetrain`

```cpp
DrivetrainConfig drivetrain{}
```

Which drivetrain it has, and its numbers.

*field, declared at [`include/shulib/config/robot_config.hpp:401`](../../include/shulib/config/robot_config.hpp#L401).*

<a id="robotconfig-odometry"></a>

### `RobotConfig::odometry`

```cpp
OdometryConfig odometry{}
```

Which estimator it runs.

*field, declared at [`include/shulib/config/robot_config.hpp:403`](../../include/shulib/config/robot_config.hpp#L403).*

<a id="robotconfig-sensors"></a>

### `RobotConfig::sensors`

```cpp
SensorsConfig sensors{}
```

Which other sensors it carries.

*field, declared at [`include/shulib/config/robot_config.hpp:405`](../../include/shulib/config/robot_config.hpp#L405).*

<a id="robotconfig-mechanisms"></a>

### `RobotConfig::mechanisms`

```cpp
MechanismConfig mechanisms[kMaxMechanisms]{}
```

Its mechanisms. Entries beyond `mechanismCount` are ignored entirely.

*field, declared at [`include/shulib/config/robot_config.hpp:407`](../../include/shulib/config/robot_config.hpp#L407).*

<a id="robotconfig-mechanismcount"></a>

### `RobotConfig::mechanismCount`

```cpp
std::uint8_t mechanismCount = 0
```

How many entries of `mechanisms` are real. Must be <= kMaxMechanisms.

*field, declared at [`include/shulib/config/robot_config.hpp:409`](../../include/shulib/config/robot_config.hpp#L409).*

<a id="robotconfig-tuning"></a>

### `RobotConfig::tuning`

```cpp
TuningConfig tuning{}
```

The motion layer's gains, budgets and fault policy. Defaults are the library's PROVISIONAL ones.

*field, declared at [`include/shulib/config/robot_config.hpp:412`](../../include/shulib/config/robot_config.hpp#L412).*

<a id="robotconfig-valid"></a>

### `RobotConfig::valid`

```cpp
[[nodiscard]] bool valid(char* why = nullptr, std::size_t cap = 0) const noexcept
```

True when nothing in the config contradicts itself. `why` (optional; pass nullptr and 0 to skip) receives the FIRST contradiction found, naming the field or the port.  WHAT IT CHECKS — exactly the mistakes a typo, a mis-wire or a mis-generated file produce: an unknown drivetrain or odometry kind byte; a tank whose `hal::DrivetrainDescription` is invalid (ITS OWN message surfaces verbatim — one representation, one refusal); a tank whose description disagrees with the config tag; an unknown cartridge byte; an IMU port outside 1..21; a holonomic port outside -21..21; the same holonomic wheel port twice; a non-finite or non-positive drive radius / track width; a non-finite strafe offset; `DriveEncoders` on a holonomic drivetrain (that estimator reads two sides and a holonomic drivetrain has none); a tracking wheel with an unset port, a non-positive diameter or a non-finite offset; a GPS declared present with an unset port or a non-finite lever arm; more mechanisms than capacity, a mechanism with an empty name, too many motors or pneumatics, a motor port out of range or a pneumatic line outside 1..8; and — across ALL of the above at once — any smart port claimed by two different things, naming both.  WHAT IT DELIBERATELY DOES NOT CHECK, and why the omission is a design decision rather than a hole: that every kinematic wheel actually HAS a motor. A holonomic port left at 0 is representable ("no port" is the tree's meaning for 0 everywhere else), and the refusal it deserves is the F5 cross-check `robot::validate()` already has to make — the same wheel-count-equals-motor-count check `motion::MotionDeps::validate()` makes at the bottom of the stack. Making it here as WELL would be two owners for one rule, which is how two checks come to disagree; one owner, named in both places, is the choice.  noexcept and total: it allocates nothing, throws nothing, and returns for every input.

*function, declared at [`include/shulib/config/robot_config.hpp:439`](../../include/shulib/config/robot_config.hpp#L439).*

<a id="robotconfig-describe"></a>

### `RobotConfig::describe`

```cpp
void describe(char* buf, std::size_t cap) const
```

The one-line banner: identity, drivetrain kind and source, the geometry that governs, the estimator, and which optional devices are present. Truncates rather than overflowing; writes at least a NUL whenever cap > 0. Only meaningful on a `valid()` config.

*function, declared at [`include/shulib/config/robot_config.hpp:698`](../../include/shulib/config/robot_config.hpp#L698).*

<a id="robotconfig-validcartridge"></a>

### `RobotConfig::validCartridge`

```cpp
[[nodiscard]] static bool validCartridge(hal::MotorCartridge c, char* why, std::size_t cap) noexcept
```

The cartridge check, shared by the two holonomic branches (the tank's lives inside `hal::DrivetrainDescription::valid()`, where it already was).

*function, declared at [`include/shulib/config/robot_config.hpp:745`](../../include/shulib/config/robot_config.hpp#L745).*

<a id="robotconfig-validimuport"></a>

### `RobotConfig::validImuPort`

```cpp
[[nodiscard]] static bool validImuPort(std::uint8_t port, char* why, std::size_t cap) noexcept
```

The IMU-port range check, shared by the two holonomic branches.

*function, declared at [`include/shulib/config/robot_config.hpp:759`](../../include/shulib/config/robot_config.hpp#L759).*

<a id="robotconfig-validwheelport"></a>

### `RobotConfig::validWheelPort`

```cpp
[[nodiscard]] static bool validWheelPort(std::int8_t port, const char* kindWord, std::size_t wheel, char* why, std::size_t cap) noexcept
```

One holonomic wheel port's range check. `0` (no port) passes HERE on purpose — see the "what it deliberately does not check" note on valid(); only a number that is not a port at all is refused. `kindWord` and `wheel` are what make the message point at a field.

*function, declared at [`include/shulib/config/robot_config.hpp:773`](../../include/shulib/config/robot_config.hpp#L773).*

<a id="robotconfig-validpod"></a>

### `RobotConfig::validPod`

```cpp
[[nodiscard]] static bool validPod(const TrackingWheelConfig& pod, const char* role, char* why, std::size_t cap) noexcept
```

One tracking wheel's own checks (port set and in range, diameter finite and > 0, offset finite). `role` names the slot in the message.

*function, declared at [`include/shulib/config/robot_config.hpp:792`](../../include/shulib/config/robot_config.hpp#L792).*

<a id="class-irobotconfig"></a>

## `class IRobotConfig`

```cpp
class IRobotConfig
```

The config seam (master plan decision #10, name kept): where a robot's description comes from. ONE virtual call, made once at boot — a generated header, a card read at boot, or a constant typed by hand all look the same to `robot::Robot` through this.

*class, declared at [`include/shulib/config/robot_config.hpp:826`](../../include/shulib/config/robot_config.hpp#L826).*

<a id="irobotconfig-destructor-irobotconfig"></a>

### `IRobotConfig::~IRobotConfig`

```cpp
virtual ~IRobotConfig() = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state.

*function, declared at [`include/shulib/config/robot_config.hpp:831`](../../include/shulib/config/robot_config.hpp#L831).*

<a id="irobotconfig-irobotconfig"></a>

### `IRobotConfig::IRobotConfig`

```cpp
IRobotConfig() = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize.

*function, declared at [`include/shulib/config/robot_config.hpp:833`](../../include/shulib/config/robot_config.hpp#L833).*

<a id="irobotconfig-irobotconfig-2"></a>

### `IRobotConfig::IRobotConfig (overload 2)`

```cpp
IRobotConfig(const IRobotConfig&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means.

*function, declared at [`include/shulib/config/robot_config.hpp:835`](../../include/shulib/config/robot_config.hpp#L835).*

<a id="irobotconfig-irobotconfig-3"></a>

### `IRobotConfig::IRobotConfig (overload 3)`

```cpp
IRobotConfig(IRobotConfig&&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor.

*function, declared at [`include/shulib/config/robot_config.hpp:837`](../../include/shulib/config/robot_config.hpp#L837).*

<a id="irobotconfig-operator-eq"></a>

### `IRobotConfig::operator=`

```cpp
IRobotConfig& operator=(const IRobotConfig&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning.

*function, declared at [`include/shulib/config/robot_config.hpp:839`](../../include/shulib/config/robot_config.hpp#L839).*

<a id="irobotconfig-operator-eq-2"></a>

### `IRobotConfig::operator= (overload 2)`

```cpp
IRobotConfig& operator=(IRobotConfig&&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning. Move-assignable, same reasoning.

*function, declared at [`include/shulib/config/robot_config.hpp:841`](../../include/shulib/config/robot_config.hpp#L841).*

<a id="irobotconfig-config"></a>

### `IRobotConfig::config`

```cpp
[[nodiscard]] virtual const RobotConfig& config() const noexcept = 0
```

This robot's description. The reference must stay valid for as long as the caller uses it — every implementation in the tree returns a reference to storage it owns, never a temporary.

*function, declared at [`include/shulib/config/robot_config.hpp:845`](../../include/shulib/config/robot_config.hpp#L845).*

<a id="class-staticrobotconfig"></a>

## `class StaticRobotConfig`

```cpp
class StaticRobotConfig final : public IRobotConfig
```

An `IRobotConfig` over one config held BY VALUE — the implementation a generated header, a hand-typed constant or a card read at boot all end up using. Holding by value rather than by reference is deliberate: a boot sequence that decodes a record into a local and then hands out a reference to it is the dangling-reference bug this type exists to make impossible.

*class, declared at [`include/shulib/config/robot_config.hpp:852`](../../include/shulib/config/robot_config.hpp#L852).*

<a id="staticrobotconfig-staticrobotconfig"></a>

### `StaticRobotConfig::StaticRobotConfig`

```cpp
explicit StaticRobotConfig(const RobotConfig& cfg) noexcept
```

Copy `cfg` into this source. The copy is the point; nothing is borrowed.

*function, declared at [`include/shulib/config/robot_config.hpp:855`](../../include/shulib/config/robot_config.hpp#L855).*

<a id="staticrobotconfig-config"></a>

### `StaticRobotConfig::config`

```cpp
[[nodiscard]] const RobotConfig& config() const noexcept override
```

The held config.

*function, declared at [`include/shulib/config/robot_config.hpp:858`](../../include/shulib/config/robot_config.hpp#L858).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 41 lines</summary>

```text

 RobotConfig — a whole robot, described as DATA (chunk G1 Part 1, 2026-09-18).

 ── Why this type exists ────────────────────────────────────────────────────────────
 Everything a composition root says about a robot — which ports drive, which way, how
 the chassis is measured, which sensors are on it, which mechanisms it carries, what the
 motion layer is tuned to — is today spelled as C++ object construction in one place
 (src/main.cpp's two graphs). That is a fine thing for a code-fluent team and a closed
 door for everyone else. This header is the OTHER door: the same facts as a plain value
 a person can type, a calibrator can fill in, or a generator can emit, and which
 robot/robot.hpp turns into the identical object graph.

   ONE representation, THREE producers:
     * a human typing a `constexpr RobotConfig` (today; the standalone path);
     * calibration::DrivetrainCalibrator, whose DrivetrainDescription is EMBEDDED here
       by value rather than mirrored (ruling B1 — a second struct with the same fields is
       exactly how a calibration record and a config come to disagree about one robot);
     * G3's codegen from a `.vexbot` file (master plan §16.2). That is why every field is
       fixed-width and the whole type is constexpr-constructible: a generator emits
       `inline constexpr RobotConfig kRobot{…};` and the compiler checks it.

 ── What this header is NOT ─────────────────────────────────────────────────────────
 It is not a replacement for longhand construction. chassis.hpp's standalone promise is
 unchanged and still pinned by its own file-free test: a builder is "an additional
 on-ramp, never the only path". G1 ADDS; it removes nothing.

 It also does not build anything. This file holds data and the rules that say when the
 data contradicts itself. Turning it into a robot is robot/robot.hpp's job, and the one
 cross-check that needs both a config and a drivetrain — every kinematic wheel has a
 motor — deliberately lives THERE and only there (see RobotConfig::valid()'s note).

 ── Laid out to be GENERATED and to be SERIALIZED ───────────────────────────────────
 Fixed-width fields, bounded arrays, no pointers, no std::string, no std::function, no
 allocation, no virtual anything in the data types. Capacities are named constants
 (ruling B6): a route or a robot that does not fit is two of them, not a reallocation.

 NO HARDWARE CLAIM IS MADE ANYWHERE IN THIS FILE. No RobotConfig has ever been used to
 build a graph that drove a physical robot; the only thing any config here has driven is
 the A2 host plant.

 PROS-free, sim-free, allocation-free.
```

</details>
