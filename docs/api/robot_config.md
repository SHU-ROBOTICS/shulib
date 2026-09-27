<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/config/robot_config.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `robot_config.hpp`

RobotConfig — a whole robot, described as DATA.

This header declares **19** types (86 members), **9** free functions, **3** constants, and **1** type alias.

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
- [`enum class SafeBrakeMode`](#enum-class-safebrakemode)
  - [`Coast`](#safebrakemode-coast)
  - [`Brake`](#safebrakemode-brake)
  - [`Hold`](#safebrakemode-hold)
- [`safeBrakeModeWord`](#safebrakemodeword) — *free function*
- [`isSafeBrakeMode`](#issafebrakemode) — *free function*
- [`toBrakeMode`](#tobrakemode) — *free function*
- [`enum class PneumaticLevel`](#enum-class-pneumaticlevel)
  - [`Low`](#pneumaticlevel-low)
  - [`High`](#pneumaticlevel-high)
- [`pneumaticLevelWord`](#pneumaticlevelword) — *free function*
- [`isPneumaticLevel`](#ispneumaticlevel) — *free function*
- [`toLineValue`](#tolinevalue) — *free function*
- [`struct MechanismConfig`](#struct-mechanismconfig)
  - [`name`](#mechanismconfig-name)
  - [`motorPorts`](#mechanismconfig-motorports)
  - [`motorCount`](#mechanismconfig-motorcount)
  - [`cartridge`](#mechanismconfig-cartridge)
  - [`safeMode`](#mechanismconfig-safemode)
  - [`pneumaticPorts`](#mechanismconfig-pneumaticports)
  - [`pneumaticCount`](#mechanismconfig-pneumaticcount)
  - [`pneumaticSafe`](#mechanismconfig-pneumaticsafe)
  - [`pneumaticStart`](#mechanismconfig-pneumaticstart)
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
  - [`terminatedName`](#robotconfig-terminatedname)
  - [`sameName`](#robotconfig-samename)
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

*constant, declared at [`include/shulib/config/robot_config.hpp:60`](../../include/shulib/config/robot_config.hpp#L60).*

<a id="kmaxmotorspermechanism"></a>

## `kMaxMotorsPerMechanism`

```cpp
inline constexpr std::size_t kMaxMotorsPerMechanism = 4
```

The most motors one mechanism may drive. Design constant (B6), same reasoning as above.

*constant, declared at [`include/shulib/config/robot_config.hpp:63`](../../include/shulib/config/robot_config.hpp#L63).*

<a id="kmaxpneumaticspermechanism"></a>

## `kMaxPneumaticsPerMechanism`

```cpp
inline constexpr std::size_t kMaxPneumaticsPerMechanism = 2
```

The most pneumatic (ADI digital-out) lines one mechanism may actuate. Design constant (B6).

*constant, declared at [`include/shulib/config/robot_config.hpp:66`](../../include/shulib/config/robot_config.hpp#L66).*

<a id="enum-class-drivetrainkind"></a>

## `enum class DrivetrainKind`

```cpp
enum class DrivetrainKind : std::uint8_t
```

Which drivetrain a RobotConfig describes.  DELIBERATELY NOT `hal::DrivetrainKind`, and the distinction matters: that enum is the one written into a persisted calibration record, it is append-only for that reason, and it knows only `Tank` because the calibrator can only measure a tank today. THIS enum is the CONFIG vocabulary and already carries the two holonomic kinds, because src/main.cpp's X-drive graph existed since R1a and had to be expressible as data — since G1 Part 2c it is (src/xdrive_config.hpp, the tree's first `constexpr RobotConfig`). A tank config carries a `hal::DrivetrainDescription`, whose own `kind` field is the persisted one; the two agree by construction because valid() checks them against each other.

*enum class, declared at [`include/shulib/config/robot_config.hpp:78`](../../include/shulib/config/robot_config.hpp#L78).*

<a id="drivetrainkind-tank"></a>

### `DrivetrainKind::Tank`

```cpp
Tank = 1
```

two sides, no lateral authority — kinematics/tank.hpp

*enumerator, declared at [`include/shulib/config/robot_config.hpp:79`](../../include/shulib/config/robot_config.hpp#L79).*

<a id="drivetrainkind-xdrive"></a>

### `DrivetrainKind::XDrive`

```cpp
XDrive = 2
```

four 45° omnis — kinematics/x_drive.hpp

*enumerator, declared at [`include/shulib/config/robot_config.hpp:80`](../../include/shulib/config/robot_config.hpp#L80).*

<a id="drivetrainkind-hdrive"></a>

### `DrivetrainKind::HDrive`

```cpp
HDrive = 3
```

tank base + one transverse strafe wheel — kinematics/h_drive.hpp

*enumerator, declared at [`include/shulib/config/robot_config.hpp:81`](../../include/shulib/config/robot_config.hpp#L81).*

<a id="drivetrainkindword"></a>

## `drivetrainKindWord`

```cpp
[[nodiscard]] inline const char* drivetrainKindWord(DrivetrainKind k) noexcept
```

The drivetrain kind as a word, for banners and refusal messages. "UNSET" for any byte outside the enum, which is what a default-constructed config carries.

*free function, declared at [`include/shulib/config/robot_config.hpp:86`](../../include/shulib/config/robot_config.hpp#L86).*

<a id="struct-robotidentity"></a>

## `struct RobotIdentity`

```cpp
struct RobotIdentity
```

Who this robot is, in words. Banner material only — nothing in the library branches on it. Fixed-width and NUL-terminated so the whole config can be written to a card as it stands.

*struct, declared at [`include/shulib/config/robot_config.hpp:97`](../../include/shulib/config/robot_config.hpp#L97).*

<a id="robotidentity-name"></a>

### `RobotIdentity::name`

```cpp
char name[32]{}
```

The robot's name, e.g. "Robot Two". Empty is legal (an unnamed robot still drives).

*field, declared at [`include/shulib/config/robot_config.hpp:99`](../../include/shulib/config/robot_config.hpp#L99).*

<a id="robotidentity-team"></a>

### `RobotIdentity::team`

```cpp
char team[16]{}
```

The team number, e.g. "1234A".

*field, declared at [`include/shulib/config/robot_config.hpp:101`](../../include/shulib/config/robot_config.hpp#L101).*

<a id="robotidentity-season"></a>

### `RobotIdentity::season`

```cpp
char season[16]{}
```

The season, e.g. "Push Back".

*field, declared at [`include/shulib/config/robot_config.hpp:103`](../../include/shulib/config/robot_config.hpp#L103).*

<a id="struct-tankdrivetrain"></a>

## `struct TankDrivetrain`

```cpp
struct TankDrivetrain
```

A tank drivetrain: R3d's `hal::DrivetrainDescription`, EMBEDDED BY VALUE and unchanged (ruling B1). Everything a tank needs is already in it — signed ports per side, inches per radian per side, effective track width, cartridge, IMU port, source and provenance — and it is the type the calibrator produces and the record persists, so a config built from a card is a field copy and nothing else.

*struct, declared at [`include/shulib/config/robot_config.hpp:111`](../../include/shulib/config/robot_config.hpp#L111).*

<a id="tankdrivetrain-description"></a>

### `TankDrivetrain::description`

```cpp
hal::DrivetrainDescription description{}
```

The description, exactly as the calibrator or the typed table produced it. Its own `valid()` is called by RobotConfig::valid(), and its message surfaces verbatim.

*field, declared at [`include/shulib/config/robot_config.hpp:114`](../../include/shulib/config/robot_config.hpp#L114).*

<a id="struct-xdrivedrivetrain"></a>

## `struct XDriveDrivetrain`

```cpp
struct XDriveDrivetrain
```

A symmetric 45° X-drive, typed by hand (no calibrator measures one yet).

*struct, declared at [`include/shulib/config/robot_config.hpp:118`](../../include/shulib/config/robot_config.hpp#L118).*

<a id="xdrivedrivetrain-ports"></a>

### `XDriveDrivetrain::ports`

```cpp
std::int8_t ports[4]{}
```

The four wheel ports in x_drive.hpp's canonical F5 wheel order — 0 front-left, 1 back-left, 2 back-right, 3 front-right — WITH THEIR SIGNS, exactly as `hal::DriveSideDescription::signedPorts` carries a tank's: `-11` is port 11 handed to the adapter negative, which PROS reverses exactly once. The sign lives here and nowhere else. A `0` entry is "no motor on that wheel", which is representable and refused by `robot::validate()`'s wheel-count cross-check, not here (see RobotConfig::valid()).

*field, declared at [`include/shulib/config/robot_config.hpp:125`](../../include/shulib/config/robot_config.hpp#L125).*

<a id="xdrivedrivetrain-cartridge"></a>

### `XDriveDrivetrain::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

The cartridge in the drive motors. HUMAN-STATED and unverifiable from inside, exactly as `hal::MotorCartridge` documents; `valid()` refuses any byte outside the enum.

*field, declared at [`include/shulib/config/robot_config.hpp:128`](../../include/shulib/config/robot_config.hpp#L128).*

<a id="xdrivedrivetrain-imuport"></a>

### `XDriveDrivetrain::imuPort`

```cpp
std::uint8_t imuPort = 0
```

The IMU's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid — every drivetrain here is heading-owned by an IMU.

*field, declared at [`include/shulib/config/robot_config.hpp:131`](../../include/shulib/config/robot_config.hpp#L131).*

<a id="xdrivedrivetrain-driveradius"></a>

### `XDriveDrivetrain::driveRadius`

```cpp
units::Length driveRadius{0.0}
```

Centre-to-wheel distance in inches, the ONE geometry input `kinematics::xDrive()` takes. Must be finite and > 0.

*field, declared at [`include/shulib/config/robot_config.hpp:134`](../../include/shulib/config/robot_config.hpp#L134).*

<a id="struct-hdrivedrivetrain"></a>

## `struct HDriveDrivetrain`

```cpp
struct HDriveDrivetrain
```

An H-drive: a tank base plus one transverse strafe wheel, typed by hand.

*struct, declared at [`include/shulib/config/robot_config.hpp:138`](../../include/shulib/config/robot_config.hpp#L138).*

<a id="hdrivedrivetrain-ports"></a>

### `HDriveDrivetrain::ports`

```cpp
std::int8_t ports[3]{}
```

The three wheel ports in h_drive.hpp's wheel order — 0 LEFT drive line, 1 RIGHT drive line, 2 STRAFE — signed, with the same one-place-for-a-sign rule as the X-drive's. Spelling the strafe wheel as `ports[2]` rather than a separate field is deliberate: the kinematics numbers it as wheel 2, and a separate field would need a second rule saying where it goes in the motor array.

*field, declared at [`include/shulib/config/robot_config.hpp:144`](../../include/shulib/config/robot_config.hpp#L144).*

<a id="hdrivedrivetrain-cartridge"></a>

### `HDriveDrivetrain::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

The cartridge in the drive motors. HUMAN-STATED; `valid()` refuses a byte outside the enum.

*field, declared at [`include/shulib/config/robot_config.hpp:146`](../../include/shulib/config/robot_config.hpp#L146).*

<a id="hdrivedrivetrain-imuport"></a>

### `HDriveDrivetrain::imuPort`

```cpp
std::uint8_t imuPort = 0
```

The IMU's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid.

*field, declared at [`include/shulib/config/robot_config.hpp:148`](../../include/shulib/config/robot_config.hpp#L148).*

<a id="hdrivedrivetrain-trackwidth"></a>

### `HDriveDrivetrain::trackWidth`

```cpp
units::Length trackWidth{0.0}
```

Contact line to contact line, inches, finite and > 0 — `hDrive()`'s `trackWidth`.

*field, declared at [`include/shulib/config/robot_config.hpp:150`](../../include/shulib/config/robot_config.hpp#L150).*

<a id="hdrivedrivetrain-strafewheeloffset"></a>

### `HDriveDrivetrain::strafeWheelOffset`

```cpp
units::Length strafeWheelOffset{0.0}
```

The strafe wheel's +FORWARD body coordinate, SIGNED and finite (negative = behind centre) — `hDrive()`'s `strafeWheelOffset`.

*field, declared at [`include/shulib/config/robot_config.hpp:153`](../../include/shulib/config/robot_config.hpp#L153).*

<a id="struct-drivetrainconfig"></a>

## `struct DrivetrainConfig`

```cpp
struct DrivetrainConfig
```

Which drivetrain this robot has, and its numbers.  A TAGGED STRUCT rather than a union: all three alternatives are stored and `kind` says which one is read. The alternative — `std::variant` or a real union — was rejected because it costs this type its aggregate-ness, and being an aggregate is precisely what lets a person (and G3's generator) write `{.kind = …, .tank = {…}}` with designated initializers and have the compiler check it. The price is about thirty bytes of static storage on a brain that has megabytes, paid once.

*struct, declared at [`include/shulib/config/robot_config.hpp:164`](../../include/shulib/config/robot_config.hpp#L164).*

<a id="drivetrainconfig-kind"></a>

### `DrivetrainConfig::kind`

```cpp
DrivetrainKind kind = DrivetrainKind::Tank
```

Which of the three members below is the real one. Everything else is ignored — not validated, not read, not built.

*field, declared at [`include/shulib/config/robot_config.hpp:167`](../../include/shulib/config/robot_config.hpp#L167).*

<a id="drivetrainconfig-tank"></a>

### `DrivetrainConfig::tank`

```cpp
TankDrivetrain tank{}
```

The tank alternative, read only when `kind == Tank`.

*field, declared at [`include/shulib/config/robot_config.hpp:169`](../../include/shulib/config/robot_config.hpp#L169).*

<a id="drivetrainconfig-xdrive"></a>

### `DrivetrainConfig::xdrive`

```cpp
XDriveDrivetrain xdrive{}
```

The X-drive alternative, read only when `kind == XDrive`.

*field, declared at [`include/shulib/config/robot_config.hpp:171`](../../include/shulib/config/robot_config.hpp#L171).*

<a id="drivetrainconfig-hdrive"></a>

### `DrivetrainConfig::hdrive`

```cpp
HDriveDrivetrain hdrive{}
```

The H-drive alternative, read only when `kind == HDrive`.

*field, declared at [`include/shulib/config/robot_config.hpp:173`](../../include/shulib/config/robot_config.hpp#L173).*

<a id="drivetrainconfig-kindof"></a>

### `DrivetrainConfig::kindOf`

```cpp
[[nodiscard]] constexpr DrivetrainKind kindOf() const noexcept
```

Which alternative is live — the accessor the brief names, so call sites read `drivetrain.kind()`-shaped code even though the tag is a plain field.

*function, declared at [`include/shulib/config/robot_config.hpp:177`](../../include/shulib/config/robot_config.hpp#L177).*

<a id="drivetrainconfig-imuport"></a>

### `DrivetrainConfig::imuPort`

```cpp
[[nodiscard]] constexpr std::uint8_t imuPort() const noexcept
```

The IMU port this drivetrain's heading comes from, whichever alternative is live. 0 when `kind` is not a known kind.

*function, declared at [`include/shulib/config/robot_config.hpp:181`](../../include/shulib/config/robot_config.hpp#L181).*

<a id="drivetrainconfig-cartridge"></a>

### `DrivetrainConfig::cartridge`

```cpp
[[nodiscard]] constexpr hal::MotorCartridge cartridge() const noexcept
```

The cartridge in the drive motors, whichever alternative is live. The byte 0 (UNSET) when `kind` is not a known kind.

*function, declared at [`include/shulib/config/robot_config.hpp:192`](../../include/shulib/config/robot_config.hpp#L192).*

<a id="drivetrainconfig-wheelcount"></a>

### `DrivetrainConfig::wheelCount`

```cpp
[[nodiscard]] constexpr int wheelCount() const noexcept
```

How many KINEMATIC WHEELS the installed drivetrain has — 2 tank, 4 X-drive, 3 H-drive. This is the number `kinematics::IKinematics::wheelCount()` will report, derived from the config so `robot::validate()` can make the F5 cross-check before any object exists. 0 when `kind` is not a known kind.

*function, declared at [`include/shulib/config/robot_config.hpp:205`](../../include/shulib/config/robot_config.hpp#L205).*

<a id="drivetrainconfig-drivemotorcount"></a>

### `DrivetrainConfig::driveMotorCount`

```cpp
[[nodiscard]] constexpr int driveMotorCount() const noexcept
```

How many `hal::IMotor` handles the builder will place in the RobotContext — ONE per kinematic wheel when the config is complete. A tank always contributes 2 (one MotorGroup per side, and a side with no ports is already refused by the description's own `valid()`); a holonomic drivetrain contributes one per NON-ZERO port slot, so a wheel whose port was left at 0 makes this number smaller than `wheelCount()` — which is exactly the disagreement `robot::validate()` exists to name.

*function, declared at [`include/shulib/config/robot_config.hpp:220`](../../include/shulib/config/robot_config.hpp#L220).*

<a id="enum-class-odometrykind"></a>

## `enum class OdometryKind`

```cpp
enum class OdometryKind : std::uint8_t
```

Which estimator feeds the Localizer.

*enum class, declared at [`include/shulib/config/robot_config.hpp:247`](../../include/shulib/config/robot_config.hpp#L247).*

<a id="odometrykind-driveencoders"></a>

### `OdometryKind::DriveEncoders`

```cpp
DriveEncoders = 1
```

The drive encoders themselves — `localization::DriveEncoderOdometry` over the two side groups. Needs no extra hardware and is the default for a tank. NOT available on a holonomic drivetrain: that estimator reads exactly two sides and an X-drive has none.

*enumerator, declared at [`include/shulib/config/robot_config.hpp:251`](../../include/shulib/config/robot_config.hpp#L251).*

<a id="odometrykind-trackingwheels"></a>

### `OdometryKind::TrackingWheels`

```cpp
TrackingWheels = 2
```

Two unpowered tracking wheels — `localization::PilonsOdometry`. The only choice for a holonomic drivetrain, and the better one for a tank that has the pods.

*enumerator, declared at [`include/shulib/config/robot_config.hpp:254`](../../include/shulib/config/robot_config.hpp#L254).*

<a id="odometrykindword"></a>

## `odometryKindWord`

```cpp
[[nodiscard]] inline const char* odometryKindWord(OdometryKind k) noexcept
```

The odometry kind as a word, for banners. "UNSET" for a byte outside the enum.

*free function, declared at [`include/shulib/config/robot_config.hpp:258`](../../include/shulib/config/robot_config.hpp#L258).*

<a id="struct-trackingwheelconfig"></a>

## `struct TrackingWheelConfig`

```cpp
struct TrackingWheelConfig
```

One unpowered tracking wheel: where it is plugged in, how big it is, and where it sits. The role (forward or lateral) is NOT a field — it is fixed by which slot of `TrackingWheelsOdometry` the wheel occupies, exactly as `localization::TrackingWheel`'s private constructor and two factories fix it, so a pair handed over in the wrong order cannot be represented rather than merely being rejected.

*struct, declared at [`include/shulib/config/robot_config.hpp:271`](../../include/shulib/config/robot_config.hpp#L271).*

<a id="trackingwheelconfig-port"></a>

### `TrackingWheelConfig::port`

```cpp
std::uint8_t port = 0
```

The rotation sensor's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid. Unsigned: a rotation sensor's direction is a mounting fact expressed by the wheel's offset sign and the sensor's own reading, not by a negative port.

*field, declared at [`include/shulib/config/robot_config.hpp:275`](../../include/shulib/config/robot_config.hpp#L275).*

<a id="trackingwheelconfig-wheeldiameter"></a>

### `TrackingWheelConfig::wheelDiameter`

```cpp
units::Length wheelDiameter{0.0}
```

Wheel diameter in inches, finite and > 0.

*field, declared at [`include/shulib/config/robot_config.hpp:277`](../../include/shulib/config/robot_config.hpp#L277).*

<a id="trackingwheelconfig-offset"></a>

### `TrackingWheelConfig::offset`

```cpp
units::Length offset{0.0}
```

The wheel's signed offset from the tracking centre, inches, finite — the +LEFT coordinate for the FORWARD wheel and the +FORWARD coordinate for the LATERAL one (tracking_wheel.hpp).

*field, declared at [`include/shulib/config/robot_config.hpp:280`](../../include/shulib/config/robot_config.hpp#L280).*

<a id="struct-trackingwheelsodometry"></a>

## `struct TrackingWheelsOdometry`

```cpp
struct TrackingWheelsOdometry
```

The two-pod (Pilons) odometry's geometry.

*struct, declared at [`include/shulib/config/robot_config.hpp:284`](../../include/shulib/config/robot_config.hpp#L284).*

<a id="trackingwheelsodometry-forward"></a>

### `TrackingWheelsOdometry::forward`

```cpp
TrackingWheelConfig forward{}
```

The FORWARD-rolling wheel (body +X); its `offset` is its +LEFT coordinate.

*field, declared at [`include/shulib/config/robot_config.hpp:286`](../../include/shulib/config/robot_config.hpp#L286).*

<a id="trackingwheelsodometry-lateral"></a>

### `TrackingWheelsOdometry::lateral`

```cpp
TrackingWheelConfig lateral{}
```

The LATERAL-rolling wheel (body +Y); its `offset` is its +FORWARD coordinate.

*field, declared at [`include/shulib/config/robot_config.hpp:288`](../../include/shulib/config/robot_config.hpp#L288).*

<a id="struct-odometryconfig"></a>

## `struct OdometryConfig`

```cpp
struct OdometryConfig
```

Which estimator this robot runs, and its geometry.  Same tagged-struct shape and the same reasoning as DrivetrainConfig. Note what this means for a tank that HAS pods but is configured `DriveEncoders`: the `wheels` member is simply not read — not validated, not port-claimed, not built. Declaring pods and then choosing drive encoders is therefore a no-op, not an error; the brief calls that a warning rather than a refusal, and the honest place for a warning is the boot banner, not a validity rule that would make an over-complete config invalid.

*struct, declared at [`include/shulib/config/robot_config.hpp:299`](../../include/shulib/config/robot_config.hpp#L299).*

<a id="odometryconfig-kind"></a>

### `OdometryConfig::kind`

```cpp
OdometryKind kind = OdometryKind::DriveEncoders
```

Which estimator to build.

*field, declared at [`include/shulib/config/robot_config.hpp:301`](../../include/shulib/config/robot_config.hpp#L301).*

<a id="odometryconfig-wheels"></a>

### `OdometryConfig::wheels`

```cpp
TrackingWheelsOdometry wheels{}
```

The tracking-wheel geometry, read only when `kind == TrackingWheels`.

*field, declared at [`include/shulib/config/robot_config.hpp:303`](../../include/shulib/config/robot_config.hpp#L303).*

<a id="struct-gpsconfig"></a>

## `struct GpsConfig`

```cpp
struct GpsConfig
```

The GPS, if this robot has one.  ABSENCE IS A FIELD, NOT A ZERO PORT. `present = false` means the robot has no GPS, and robot/robot.hpp answers that with the explicit `hal::AbsentGps` in the context and NO corrector installed — never a null and never a fake (landmine 5). A GPS-less robot is the common case here: robot two has none, and Driving Skills has no field strip at all.

*struct, declared at [`include/shulib/config/robot_config.hpp:312`](../../include/shulib/config/robot_config.hpp#L312).*

<a id="gpsconfig-present"></a>

### `GpsConfig::present`

```cpp
bool present = false
```

Whether a GPS is installed. When false every other field is ignored.

*field, declared at [`include/shulib/config/robot_config.hpp:314`](../../include/shulib/config/robot_config.hpp#L314).*

<a id="gpsconfig-port"></a>

### `GpsConfig::port`

```cpp
std::uint8_t port = 0
```

The GPS's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid when `present`.

*field, declared at [`include/shulib/config/robot_config.hpp:316`](../../include/shulib/config/robot_config.hpp#L316).*

<a id="gpsconfig-leverarmforward"></a>

### `GpsConfig::leverArmForward`

```cpp
units::Length leverArmForward{0.0}
```

The sensor's +FORWARD body coordinate from the robot centre, inches, finite.

*field, declared at [`include/shulib/config/robot_config.hpp:318`](../../include/shulib/config/robot_config.hpp#L318).*

<a id="gpsconfig-leverarmleft"></a>

### `GpsConfig::leverArmLeft`

```cpp
units::Length leverArmLeft{0.0}
```

The sensor's +LEFT body coordinate from the robot centre, inches, finite.

*field, declared at [`include/shulib/config/robot_config.hpp:320`](../../include/shulib/config/robot_config.hpp#L320).*

<a id="struct-sensorsconfig"></a>

## `struct SensorsConfig`

```cpp
struct SensorsConfig
```

Every sensor this robot has that is not part of the drivetrain or the odometry.  TAGS AND VISION ARE DELIBERATELY ABSENT IN PART 1, and that is a statement about the tree rather than about the schema: no camera adapter exists (R2 owns it), so a field here would describe hardware the library cannot open. `robot::Robot` installs `hal::AbsentTagSource` and `hal::AbsentVision` unconditionally, exactly as src/main.cpp's hand-wired graphs did.

*struct, declared at [`include/shulib/config/robot_config.hpp:329`](../../include/shulib/config/robot_config.hpp#L329).*

<a id="sensorsconfig-gps"></a>

### `SensorsConfig::gps`

```cpp
GpsConfig gps{}
```

The GPS, present or absent.

*field, declared at [`include/shulib/config/robot_config.hpp:331`](../../include/shulib/config/robot_config.hpp#L331).*

<a id="enum-class-safebrakemode"></a>

## `enum class SafeBrakeMode`

```cpp
enum class SafeBrakeMode : std::uint8_t
```

A motor mechanism's DECLARED safe brake mode, as config data — what `robot::Robot` hands to `hal::MotorMechanism`'s constructor, and so what every stop of that mechanism lands in (`applySafeState()`: this mode, then 0 V).  DELIBERATELY NOT `hal::BrakeMode`, and the reason is landmine-sized: `hal::BrakeMode`'s first enumerator is `Coast`, so a value-initialised field of that type would silently declare COAST — and a loaded arm or claw at 0 V and Coast drops its load. There is no safe default for a mechanism (hal/mechanism.hpp's banner: Hold for a loaded lift, Coast or Brake for a jammed intake), so the config refuses to have one: like `hal::MotorCartridge`, this enum has NO `Unset` member, the byte 0 is what a config nobody filled in carries, and `RobotConfig::valid()` refuses it for any mechanism that has motors. Fixed-width, so a generator can emit it and a card can store it.

*enum class, declared at [`include/shulib/config/robot_config.hpp:346`](../../include/shulib/config/robot_config.hpp#L346).*

<a id="safebrakemode-coast"></a>

### `SafeBrakeMode::Coast`

```cpp
Coast = 1
```

undriven — for an intake that must never sit at stall current

*enumerator, declared at [`include/shulib/config/robot_config.hpp:347`](../../include/shulib/config/robot_config.hpp#L347).*

<a id="safebrakemode-brake"></a>

### `SafeBrakeMode::Brake`

```cpp
Brake = 2
```

brakes and resists, but does not return to where it stopped

*enumerator, declared at [`include/shulib/config/robot_config.hpp:348`](../../include/shulib/config/robot_config.hpp#L348).*

<a id="safebrakemode-hold"></a>

### `SafeBrakeMode::Hold`

```cpp
Hold = 3
```

drives back to the angle it stopped at — the one that keeps a load up

*enumerator, declared at [`include/shulib/config/robot_config.hpp:349`](../../include/shulib/config/robot_config.hpp#L349).*

<a id="safebrakemodeword"></a>

## `safeBrakeModeWord`

```cpp
[[nodiscard]] inline const char* safeBrakeModeWord(SafeBrakeMode m) noexcept
```

The safe brake mode as a word, for banners and refusal messages. "UNSET" for any byte outside the enum, which is what a default-constructed `MechanismConfig` carries.

*free function, declared at [`include/shulib/config/robot_config.hpp:354`](../../include/shulib/config/robot_config.hpp#L354).*

<a id="issafebrakemode"></a>

## `isSafeBrakeMode`

```cpp
[[nodiscard]] constexpr bool isSafeBrakeMode(SafeBrakeMode m) noexcept
```

True when `m` is one of the three modes — false for the UNSET byte 0 and any other byte.

*free function, declared at [`include/shulib/config/robot_config.hpp:364`](../../include/shulib/config/robot_config.hpp#L364).*

<a id="tobrakemode"></a>

## `toBrakeMode`

```cpp
[[nodiscard]] constexpr hal::BrakeMode toBrakeMode(SafeBrakeMode m) noexcept
```

The HAL brake mode a `SafeBrakeMode` stands for. Total: a byte outside the enum converts to `Hold` — the mode that drops nothing — and never to Coast. That fallback is a floor, not a default anyone relies on: `robot::Robot` only converts a mode `RobotConfig::valid()` accepted.

*free function, declared at [`include/shulib/config/robot_config.hpp:371`](../../include/shulib/config/robot_config.hpp#L371).*

<a id="enum-class-pneumaticlevel"></a>

## `enum class PneumaticLevel`

```cpp
enum class PneumaticLevel : std::uint8_t
```

A pneumatic mechanism's line LEVEL, as config data — its declared SAFE level and its START level (`MechanismConfig::pneumaticSafe`, `pneumaticStart`). `Low` is the line off (`false`: the solenoid unpowered), `High` the line on (`true`: powered).  DELIBERATELY NOT `bool`, for the reason `SafeBrakeMode` is not a `hal::BrakeMode`: a value-initialised bool is `false`, so a field of that type would silently declare "off" for a config nobody filled in — and whether off leaves a claw OPEN (dropping what it holds) or CLOSED depends on how its valve is plumbed, which only the team knows. So, like `hal::MotorCartridge`, this enum has NO `Unset` member, the byte 0 is what a config nobody filled in carries, and `RobotConfig::valid()` refuses it for any mechanism that has 3-wire lines (T3's addendum B2).

*enum class, declared at [`include/shulib/config/robot_config.hpp:390`](../../include/shulib/config/robot_config.hpp#L390).*

<a id="pneumaticlevel-low"></a>

### `PneumaticLevel::Low`

```cpp
Low = 1
```

the line off (`false`) — the solenoid unpowered

*enumerator, declared at [`include/shulib/config/robot_config.hpp:391`](../../include/shulib/config/robot_config.hpp#L391).*

<a id="pneumaticlevel-high"></a>

### `PneumaticLevel::High`

```cpp
High = 2
```

the line on (`true`) — the solenoid powered

*enumerator, declared at [`include/shulib/config/robot_config.hpp:392`](../../include/shulib/config/robot_config.hpp#L392).*

<a id="pneumaticlevelword"></a>

## `pneumaticLevelWord`

```cpp
[[nodiscard]] inline const char* pneumaticLevelWord(PneumaticLevel l) noexcept
```

The level as a word, for banners and refusal messages: "low", "high", or "UNSET" for any byte outside the enum, which is what a default-constructed `MechanismConfig` carries.

*free function, declared at [`include/shulib/config/robot_config.hpp:397`](../../include/shulib/config/robot_config.hpp#L397).*

<a id="ispneumaticlevel"></a>

## `isPneumaticLevel`

```cpp
[[nodiscard]] constexpr bool isPneumaticLevel(PneumaticLevel l) noexcept
```

True when `l` is `Low` or `High` — false for the UNSET byte 0 and any other byte.

*free function, declared at [`include/shulib/config/robot_config.hpp:406`](../../include/shulib/config/robot_config.hpp#L406).*

<a id="tolinevalue"></a>

## `toLineValue`

```cpp
[[nodiscard]] constexpr bool toLineValue(PneumaticLevel l) noexcept
```

The line value a level stands for: `High` is `true`, `Low` is `false`. Only for a level `RobotConfig::valid()` accepted — `robot::Robot` converts nothing else — because there is no safe floor to fall back to here (unlike `toBrakeMode()`'s Hold): either value moves a cylinder.

*free function, declared at [`include/shulib/config/robot_config.hpp:413`](../../include/shulib/config/robot_config.hpp#L413).*

<a id="struct-mechanismconfig"></a>

## `struct MechanismConfig`

```cpp
struct MechanismConfig
```

One mechanism — an intake, a lift, a clamp, a claw pivot, a pneumatic claw — as data.  BUILT since chunk T3 (2026-09-26): `robot::Robot` builds ONE `hal::MotorMechanism` per mechanism that has motors, asking the provider for each motor with THIS mechanism's cartridge (not the drivetrain's), names it with `name`, and exposes it by name (`Robot::mechanism()`) and as the `hal::IMechanism*` span an end-of-run guard walks (`Robot::mechanisms()`). Since T3's addendum B2 (the same day) it also builds ONE `hal::PneumaticMechanism` per mechanism that has 3-wire lines and no motors — each line made at `pneumaticStart` (on a brain, making a line DRIVES it), the mechanism's safe level `pneumaticSafe` — found by `Robot::pneumaticMechanism()` and in the same span. A device the provider will not make leaves that mechanism ABSENT, reported with the provider's sentence — never the whole robot refused. A mechanism with BOTH motors and lines is refused by `RobotConfig::valid()`: one name is one object, and a voltage and a level do not fan through one. What a mechanism MEANS — which buttons drive it, what confirms it — stays with the program (teleop/mechanism_binding.hpp, manipulation/).  Two mechanisms that must move independently are TWO entries, never one entry with two motors: a `MotorMechanism` fans ONE voltage to all its motors, so two claws in one entry would move together on every button. The same holds for lines: a `PneumaticMechanism` fans ONE level to every line, so its two lines are two valves fired together — NOT the two coils of a double-coil solenoid, which need opposite levels and are not built by this library. How one WOULD be, once a robot is known to have one (said here, deliberately not built — T3's addendum B2): its two lines stay ONE mechanism (one name, one toggle button), and each line gains a stated polarity; for an inverted line the builder asks the provider at the OPPOSITE start level and hands the mechanism a one-line `hal::IDigitalOut` adapter that writes (and reports) the opposite of what it is told — so `PneumaticMechanism` still fans one level, each coil gets its own, and nothing else changes: not the toggle, not the guard's span.

*struct, declared at [`include/shulib/config/robot_config.hpp:443`](../../include/shulib/config/robot_config.hpp#L443).*

<a id="mechanismconfig-name"></a>

### `MechanismConfig::name`

```cpp
char name[24]{}
```

The mechanism's name — the key `robot::Robot::mechanism()` looks it up by, and the key a command-id registry will bind against. Non-empty, NUL-terminated inside these 24 bytes, and unique among the robot's mechanisms (all three refused by `RobotConfig::valid()`). The Robot BORROWS this text for the mechanism's `name()` from its own copy of the config.

*field, declared at [`include/shulib/config/robot_config.hpp:448`](../../include/shulib/config/robot_config.hpp#L448).*

<a id="mechanismconfig-motorports"></a>

### `MechanismConfig::motorPorts`

```cpp
std::int8_t motorPorts[kMaxMotorsPerMechanism]{}
```

The mechanism's motor ports WITH THEIR SIGNS, same one-place-for-a-sign rule as the drivetrain's: a pivot that moves the wrong way on its forward button is fixed by flipping the sign HERE, nowhere else. Entries beyond `motorCount` are 0 and are not read.

*field, declared at [`include/shulib/config/robot_config.hpp:452`](../../include/shulib/config/robot_config.hpp#L452).*

<a id="mechanismconfig-motorcount"></a>

### `MechanismConfig::motorCount`

```cpp
std::uint8_t motorCount = 0
```

How many entries of `motorPorts` are real. 0 is legal — a purely pneumatic mechanism (a mechanism with motors AND lines is refused).

*field, declared at [`include/shulib/config/robot_config.hpp:455`](../../include/shulib/config/robot_config.hpp#L455).*

<a id="mechanismconfig-cartridge"></a>

### `MechanismConfig::cartridge`

```cpp
hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0)
```

The cartridge in THIS mechanism's motors — its own, never the drivetrain's (a RED claw on a BLUE drive is the normal case). HUMAN-STATED and unverifiable from inside, exactly as `hal::MotorCartridge` documents. The byte 0 is UNSET, and `RobotConfig::valid()` refuses it (or any byte outside the enum) whenever `motorCount > 0`; not read when `motorCount == 0`.

*field, declared at [`include/shulib/config/robot_config.hpp:460`](../../include/shulib/config/robot_config.hpp#L460).*

<a id="mechanismconfig-safemode"></a>

### `MechanismConfig::safeMode`

```cpp
SafeBrakeMode safeMode = static_cast<SafeBrakeMode>(0)
```

The DECLARED safe brake mode every stop of this mechanism lands in. The byte 0 is UNSET, and `RobotConfig::valid()` refuses it whenever `motorCount > 0` — see `SafeBrakeMode` for why there is no default. Not read when `motorCount == 0`.

*field, declared at [`include/shulib/config/robot_config.hpp:464`](../../include/shulib/config/robot_config.hpp#L464).*

<a id="mechanismconfig-pneumaticports"></a>

### `MechanismConfig::pneumaticPorts`

```cpp
std::uint8_t pneumaticPorts[kMaxPneumaticsPerMechanism]{}
```

The mechanism's pneumatic (ADI digital-out) lines: the brain's three-wire ports as numbers, 1..8 for A..H. Entries beyond `pneumaticCount` are 0 and are not read. No line may appear twice — in one mechanism or across two (refused by `RobotConfig::valid()`). BUILT since T3's addendum B2, as one `hal::PneumaticMechanism` (see above).

*field, declared at [`include/shulib/config/robot_config.hpp:469`](../../include/shulib/config/robot_config.hpp#L469).*

<a id="mechanismconfig-pneumaticcount"></a>

### `MechanismConfig::pneumaticCount`

```cpp
std::uint8_t pneumaticCount = 0
```

How many entries of `pneumaticPorts` are real. 0 is legal — a purely motorized mechanism.

*field, declared at [`include/shulib/config/robot_config.hpp:471`](../../include/shulib/config/robot_config.hpp#L471).*

<a id="mechanismconfig-pneumaticsafe"></a>

### `MechanismConfig::pneumaticSafe`

```cpp
PneumaticLevel pneumaticSafe = static_cast<PneumaticLevel>(0)
```

The level every line is driven to whenever anything STOPS this mechanism — its `applySafeState()`, which an end-of-run guard calls. (A driver's toggle button never commands it: a claw keeps its level through a disable.) The byte 0 is UNSET, and `RobotConfig::valid()` refuses it whenever `pneumaticCount > 0` — see `PneumaticLevel` for why there is no default. Not read when `pneumaticCount == 0`.

*field, declared at [`include/shulib/config/robot_config.hpp:477`](../../include/shulib/config/robot_config.hpp#L477).*

<a id="mechanismconfig-pneumaticstart"></a>

### `MechanismConfig::pneumaticStart`

```cpp
PneumaticLevel pneumaticStart = static_cast<PneumaticLevel>(0)
```

The level every line is driven to when the Robot is BUILT — the program-start state (a claw that starts closed). On a brain, making a 3-wire line drives it (A4 register HA-119), so this is the one level the builder itself puts on the robot. The byte 0 is UNSET, refused by `RobotConfig::valid()` whenever `pneumaticCount > 0`. Not read when `pneumaticCount == 0`.

*field, declared at [`include/shulib/config/robot_config.hpp:482`](../../include/shulib/config/robot_config.hpp#L482).*

<a id="tuningconfig"></a>

## `TuningConfig`

```cpp
using TuningConfig = chassis::ChassisConfig
```

The motion layer's gains, budgets and fault policy for this robot — `chassis::ChassisConfig` passed through WHOLE, not mirrored, for exactly the reason ruling B1 gives for the drivetrain description: a second struct with the same fields drifts. Every value in it is PROVISIONAL until R5 tunes on hardware (HA-45/50/51/52).

*type alias, declared at [`include/shulib/config/robot_config.hpp:489`](../../include/shulib/config/robot_config.hpp#L489).*

<a id="enum-class-portuse"></a>

## `enum class PortUse`

```cpp
enum class PortUse : std::uint8_t
```

What claims a smart port, for the one-owner-per-port check. Ordered by nothing; the values are interior to `RobotConfig::valid()` and are not persisted anywhere.

*enum class, declared at [`include/shulib/config/robot_config.hpp:493`](../../include/shulib/config/robot_config.hpp#L493).*

<a id="portuse-free"></a>

### `PortUse::Free`

```cpp
Free = 0
```

nothing claims this port

*enumerator, declared at [`include/shulib/config/robot_config.hpp:494`](../../include/shulib/config/robot_config.hpp#L494).*

<a id="portuse-driveleft"></a>

### `PortUse::DriveLeft`

```cpp
DriveLeft
```

a tank LEFT-side drive motor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:495`](../../include/shulib/config/robot_config.hpp#L495).*

<a id="portuse-driveright"></a>

### `PortUse::DriveRight`

```cpp
DriveRight
```

a tank RIGHT-side drive motor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:496`](../../include/shulib/config/robot_config.hpp#L496).*

<a id="portuse-drivewheel"></a>

### `PortUse::DriveWheel`

```cpp
DriveWheel
```

a holonomic drive motor (X-drive or H-drive wheel)

*enumerator, declared at [`include/shulib/config/robot_config.hpp:497`](../../include/shulib/config/robot_config.hpp#L497).*

<a id="portuse-imu"></a>

### `PortUse::Imu`

```cpp
Imu
```

the drivetrain's inertial sensor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:498`](../../include/shulib/config/robot_config.hpp#L498).*

<a id="portuse-podforward"></a>

### `PortUse::PodForward`

```cpp
PodForward
```

the forward tracking wheel's rotation sensor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:499`](../../include/shulib/config/robot_config.hpp#L499).*

<a id="portuse-podlateral"></a>

### `PortUse::PodLateral`

```cpp
PodLateral
```

the lateral tracking wheel's rotation sensor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:500`](../../include/shulib/config/robot_config.hpp#L500).*

<a id="portuse-gps"></a>

### `PortUse::Gps`

```cpp
Gps
```

the GPS

*enumerator, declared at [`include/shulib/config/robot_config.hpp:501`](../../include/shulib/config/robot_config.hpp#L501).*

<a id="portuse-mechanism"></a>

### `PortUse::Mechanism`

```cpp
Mechanism
```

a mechanism motor

*enumerator, declared at [`include/shulib/config/robot_config.hpp:502`](../../include/shulib/config/robot_config.hpp#L502).*

<a id="portuseword"></a>

## `portUseWord`

```cpp
[[nodiscard]] inline const char* portUseWord(PortUse u) noexcept
```

A port claim as words, for the collision message — "port 7 is claimed by the LEFT drive and by a mechanism motor" reads as an instruction to go and fix one of them.

*free function, declared at [`include/shulib/config/robot_config.hpp:507`](../../include/shulib/config/robot_config.hpp#L507).*

<a id="struct-robotconfig"></a>

## `struct RobotConfig`

```cpp
struct RobotConfig
```

EVERYTHING about one robot, as one value.  Plain data with designated initializers at the call site, like every other config type in this tree. Nothing validates itself on construction: a caller builds one and then asks `valid()`, which is what lets a host test build a deliberately contradictory config by hand and watch it be refused by name.

*struct, declared at [`include/shulib/config/robot_config.hpp:528`](../../include/shulib/config/robot_config.hpp#L528).*

<a id="robotconfig-identity"></a>

### `RobotConfig::identity`

```cpp
RobotIdentity identity{}
```

Who this robot is — banner material.

*field, declared at [`include/shulib/config/robot_config.hpp:530`](../../include/shulib/config/robot_config.hpp#L530).*

<a id="robotconfig-drivetrain"></a>

### `RobotConfig::drivetrain`

```cpp
DrivetrainConfig drivetrain{}
```

Which drivetrain it has, and its numbers.

*field, declared at [`include/shulib/config/robot_config.hpp:532`](../../include/shulib/config/robot_config.hpp#L532).*

<a id="robotconfig-odometry"></a>

### `RobotConfig::odometry`

```cpp
OdometryConfig odometry{}
```

Which estimator it runs.

*field, declared at [`include/shulib/config/robot_config.hpp:534`](../../include/shulib/config/robot_config.hpp#L534).*

<a id="robotconfig-sensors"></a>

### `RobotConfig::sensors`

```cpp
SensorsConfig sensors{}
```

Which other sensors it carries.

*field, declared at [`include/shulib/config/robot_config.hpp:536`](../../include/shulib/config/robot_config.hpp#L536).*

<a id="robotconfig-mechanisms"></a>

### `RobotConfig::mechanisms`

```cpp
MechanismConfig mechanisms[kMaxMechanisms]{}
```

Its mechanisms. Entries beyond `mechanismCount` are ignored entirely.

*field, declared at [`include/shulib/config/robot_config.hpp:538`](../../include/shulib/config/robot_config.hpp#L538).*

<a id="robotconfig-mechanismcount"></a>

### `RobotConfig::mechanismCount`

```cpp
std::uint8_t mechanismCount = 0
```

How many entries of `mechanisms` are real. Must be <= kMaxMechanisms.

*field, declared at [`include/shulib/config/robot_config.hpp:540`](../../include/shulib/config/robot_config.hpp#L540).*

<a id="robotconfig-tuning"></a>

### `RobotConfig::tuning`

```cpp
TuningConfig tuning{}
```

The motion layer's gains, budgets and fault policy. Defaults are the library's PROVISIONAL ones.

*field, declared at [`include/shulib/config/robot_config.hpp:543`](../../include/shulib/config/robot_config.hpp#L543).*

<a id="robotconfig-valid"></a>

### `RobotConfig::valid`

```cpp
[[nodiscard]] bool valid(char* why = nullptr, std::size_t cap = 0) const noexcept
```

True when nothing in the config contradicts itself. `why` (optional; pass nullptr and 0 to skip) receives the FIRST contradiction found, naming the field or the port.  WHAT IT CHECKS — exactly the mistakes a typo, a mis-wire or a mis-generated file produce: an unknown drivetrain or odometry kind byte; a tank whose `hal::DrivetrainDescription` is invalid (ITS OWN message surfaces verbatim — one representation, one refusal); a tank whose description disagrees with the config tag; an unknown cartridge byte; an IMU port outside 1..21; a holonomic port outside -21..21; the same holonomic wheel port twice; a non-finite or non-positive drive radius / track width; a non-finite strafe offset; `DriveEncoders` on a holonomic drivetrain (that estimator reads two sides and a holonomic drivetrain has none); a tracking wheel with an unset port, a non-positive diameter or a non-finite offset; a GPS declared present with an unset port or a non-finite lever arm; more mechanisms than capacity, a mechanism with an empty name or a name with no NUL inside its 24 bytes, too many motors or pneumatics, a motor port out of range or a pneumatic line outside 1..8, a 3-wire line listed twice (by two mechanisms or by one — T3's addendum B2); and — across ALL of the above at once — any smart port claimed by two different things, naming both (so a mechanism port that is also a drive port is refused here, before any device is asked for with either cartridge). THEN, in a second pass once every port is claimed (chunk T3 — so each collision above keeps its own message): a mechanism with BOTH motors and 3-wire lines (addendum B2); for every mechanism with motors, an UNSET or unknown cartridge byte, an UNSET or unknown safe brake mode byte; for every mechanism with lines (addendum B2), an UNSET or unknown safe level byte, an UNSET or unknown start level byte; and, for all mechanisms, two with the same name.  WHAT IT DELIBERATELY DOES NOT CHECK, and why the omission is a design decision rather than a hole: that every kinematic wheel actually HAS a motor. A holonomic port left at 0 is representable ("no port" is the tree's meaning for 0 everywhere else), and the refusal it deserves is the F5 cross-check `robot::validate()` already has to make — the same wheel-count-equals-motor-count check `motion::MotionDeps::validate()` makes at the bottom of the stack. Making it here as WELL would be two owners for one rule, which is how two checks come to disagree; one owner, named in both places, is the choice.  noexcept and total: it allocates nothing, throws nothing, and returns for every input.

*function, declared at [`include/shulib/config/robot_config.hpp:578`](../../include/shulib/config/robot_config.hpp#L578).*

<a id="robotconfig-terminatedname"></a>

### `RobotConfig::terminatedName`

```cpp
[[nodiscard]] static constexpr bool terminatedName(const char (&name)[24]) noexcept
```

True when `name` has a NUL inside its 24 bytes — the precondition for reading it as a C string at all.

*function, declared at [`include/shulib/config/robot_config.hpp:952`](../../include/shulib/config/robot_config.hpp#L952).*

<a id="robotconfig-samename"></a>

### `RobotConfig::sameName`

```cpp
[[nodiscard]] static constexpr bool sameName(const char (&a)[24], const char (&b)[24]) noexcept
```

Two TERMINATED mechanism names, compared as C strings (bytes after the NUL are ignored, so a name typed into a reused buffer still compares by what it says).

*function, declared at [`include/shulib/config/robot_config.hpp:963`](../../include/shulib/config/robot_config.hpp#L963).*

<a id="robotconfig-describe"></a>

### `RobotConfig::describe`

```cpp
void describe(char* buf, std::size_t cap) const
```

The one-line banner: identity, drivetrain kind and source, the geometry that governs, the estimator, and which optional devices are present. Truncates rather than overflowing; writes at least a NUL whenever cap > 0. Only meaningful on a `valid()` config.

*function, declared at [`include/shulib/config/robot_config.hpp:978`](../../include/shulib/config/robot_config.hpp#L978).*

<a id="robotconfig-validcartridge"></a>

### `RobotConfig::validCartridge`

```cpp
[[nodiscard]] static bool validCartridge(hal::MotorCartridge c, char* why, std::size_t cap) noexcept
```

The cartridge check, shared by the two holonomic branches (the tank's lives inside `hal::DrivetrainDescription::valid()`, where it already was).

*function, declared at [`include/shulib/config/robot_config.hpp:1025`](../../include/shulib/config/robot_config.hpp#L1025).*

<a id="robotconfig-validimuport"></a>

### `RobotConfig::validImuPort`

```cpp
[[nodiscard]] static bool validImuPort(std::uint8_t port, char* why, std::size_t cap) noexcept
```

The IMU-port range check, shared by the two holonomic branches.

*function, declared at [`include/shulib/config/robot_config.hpp:1039`](../../include/shulib/config/robot_config.hpp#L1039).*

<a id="robotconfig-validwheelport"></a>

### `RobotConfig::validWheelPort`

```cpp
[[nodiscard]] static bool validWheelPort(std::int8_t port, const char* kindWord, std::size_t wheel, char* why, std::size_t cap) noexcept
```

One holonomic wheel port's range check. `0` (no port) passes HERE on purpose — see the "what it deliberately does not check" note on valid(); only a number that is not a port at all is refused. `kindWord` and `wheel` are what make the message point at a field.

*function, declared at [`include/shulib/config/robot_config.hpp:1053`](../../include/shulib/config/robot_config.hpp#L1053).*

<a id="robotconfig-validpod"></a>

### `RobotConfig::validPod`

```cpp
[[nodiscard]] static bool validPod(const TrackingWheelConfig& pod, const char* role, char* why, std::size_t cap) noexcept
```

One tracking wheel's own checks (port set and in range, diameter finite and > 0, offset finite). `role` names the slot in the message.

*function, declared at [`include/shulib/config/robot_config.hpp:1072`](../../include/shulib/config/robot_config.hpp#L1072).*

<a id="class-irobotconfig"></a>

## `class IRobotConfig`

```cpp
class IRobotConfig
```

The config seam (master plan decision #10, name kept): where a robot's description comes from. ONE virtual call, made once at boot — a generated header, a card read at boot, or a constant typed by hand all look the same to `robot::Robot` through this.

*class, declared at [`include/shulib/config/robot_config.hpp:1106`](../../include/shulib/config/robot_config.hpp#L1106).*

<a id="irobotconfig-destructor-irobotconfig"></a>

### `IRobotConfig::~IRobotConfig`

```cpp
virtual ~IRobotConfig() = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state.

*function, declared at [`include/shulib/config/robot_config.hpp:1111`](../../include/shulib/config/robot_config.hpp#L1111).*

<a id="irobotconfig-irobotconfig"></a>

### `IRobotConfig::IRobotConfig`

```cpp
IRobotConfig() = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize.

*function, declared at [`include/shulib/config/robot_config.hpp:1113`](../../include/shulib/config/robot_config.hpp#L1113).*

<a id="irobotconfig-irobotconfig-2"></a>

### `IRobotConfig::IRobotConfig (overload 2)`

```cpp
IRobotConfig(const IRobotConfig&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means.

*function, declared at [`include/shulib/config/robot_config.hpp:1115`](../../include/shulib/config/robot_config.hpp#L1115).*

<a id="irobotconfig-irobotconfig-3"></a>

### `IRobotConfig::IRobotConfig (overload 3)`

```cpp
IRobotConfig(IRobotConfig&&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor.

*function, declared at [`include/shulib/config/robot_config.hpp:1117`](../../include/shulib/config/robot_config.hpp#L1117).*

<a id="irobotconfig-operator-eq"></a>

### `IRobotConfig::operator=`

```cpp
IRobotConfig& operator=(const IRobotConfig&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning.

*function, declared at [`include/shulib/config/robot_config.hpp:1119`](../../include/shulib/config/robot_config.hpp#L1119).*

<a id="irobotconfig-operator-eq-2"></a>

### `IRobotConfig::operator= (overload 2)`

```cpp
IRobotConfig& operator=(IRobotConfig&&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so a source held as `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move, which are re-defaulted below. The base carries no state. Default-constructible: the base has nothing to initialize. Copyable: the base holds no state, so a derived type decides what copying means. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning. Move-assignable, same reasoning.

*function, declared at [`include/shulib/config/robot_config.hpp:1121`](../../include/shulib/config/robot_config.hpp#L1121).*

<a id="irobotconfig-config"></a>

### `IRobotConfig::config`

```cpp
[[nodiscard]] virtual const RobotConfig& config() const noexcept = 0
```

This robot's description. The reference must stay valid for as long as the caller uses it — every implementation in the tree returns a reference to storage it owns, never a temporary.

*function, declared at [`include/shulib/config/robot_config.hpp:1125`](../../include/shulib/config/robot_config.hpp#L1125).*

<a id="class-staticrobotconfig"></a>

## `class StaticRobotConfig`

```cpp
class StaticRobotConfig final : public IRobotConfig
```

An `IRobotConfig` over one config held BY VALUE — the implementation a generated header, a hand-typed constant or a card read at boot all end up using. Holding by value rather than by reference is deliberate: a boot sequence that decodes a record into a local and then hands out a reference to it is the dangling-reference bug this type exists to make impossible.

*class, declared at [`include/shulib/config/robot_config.hpp:1132`](../../include/shulib/config/robot_config.hpp#L1132).*

<a id="staticrobotconfig-staticrobotconfig"></a>

### `StaticRobotConfig::StaticRobotConfig`

```cpp
explicit StaticRobotConfig(const RobotConfig& cfg) noexcept
```

Copy `cfg` into this source. The copy is the point; nothing is borrowed.

*function, declared at [`include/shulib/config/robot_config.hpp:1135`](../../include/shulib/config/robot_config.hpp#L1135).*

<a id="staticrobotconfig-config"></a>

### `StaticRobotConfig::config`

```cpp
[[nodiscard]] const RobotConfig& config() const noexcept override
```

The held config.

*function, declared at [`include/shulib/config/robot_config.hpp:1138`](../../include/shulib/config/robot_config.hpp#L1138).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 42 lines</summary>

```text

 RobotConfig — a whole robot, described as DATA (chunk G1 Part 1, 2026-09-18).

 ── Why this type exists ────────────────────────────────────────────────────────────
 Everything a composition root says about a robot — which ports drive, which way, how
 the chassis is measured, which sensors are on it, which mechanisms it carries, what the
 motion layer is tuned to — was spelled only as C++ object construction in one place
 (src/main.cpp's hand-wired graphs, until G1 Part 2b built "shulib Teleop" from a
 RobotConfig instead). That is a fine thing for a code-fluent team and a closed
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
