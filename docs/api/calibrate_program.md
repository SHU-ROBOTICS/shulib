<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/calibrate_program.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `calibrate_program.hpp`

calibrate_program — THE STANDARD "shulib Calibrate" (chunk CAL1, Part A2a, 2026-10-18): the robot measures its own drivetrain, for ANY team, from the few facts only a person can state.

This header declares **8** types (40 members), **8** free functions, and **4** constants.

Extracted from [`include/shulib/calibration/calibrate_program.hpp`](../../include/shulib/calibration/calibrate_program.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct MechanismPort`](#struct-mechanismport)
  - [`port`](#mechanismport-port)
  - [`name`](#mechanismport-name)
- [`kMaxMechanismPorts`](#kmaxmechanismports) — *constant*
- [`struct TypedDrivePorts`](#struct-typeddriveports)
  - [`left`](#typeddriveports-left)
  - [`right`](#typeddriveports-right)
- [`struct CalibrateFacts`](#struct-calibratefacts)
  - [`drive`](#calibratefacts-drive)
  - [`cartridge`](#calibratefacts-cartridge)
  - [`mechanisms`](#calibratefacts-mechanisms)
  - [`robot`](#calibratefacts-robot)
  - [`imuPortStated`](#calibratefacts-imuportstated)
  - [`typedDrive`](#calibratefacts-typeddrive)
  - [`calibrator`](#calibratefacts-calibrator)
  - [`recordName`](#calibratefacts-recordname)
- [`enum class FactsList`](#enum-class-factslist)
  - [`Mechanisms`](#factslist-mechanisms)
  - [`TypedLeft`](#factslist-typedleft)
  - [`TypedRight`](#factslist-typedright)
  - [`RecordName`](#factslist-recordname)
- [`struct FactsProblem`](#struct-factsproblem)
  - [`why`](#factsproblem-why)
  - [`entry`](#factsproblem-entry)
  - [`list`](#factsproblem-list)
- [`hasTypedDrive`](#hastypeddrive) — *free function*
- [`factsProblem`](#factsproblem) — *free function*
- [`typedDriveDescription`](#typeddrivedescription) — *free function*
- [`typedDrivePortsFrom`](#typeddriveportsfrom) — *free function*
- [`declaredMechanism`](#declaredmechanism) — *free function*
- [`kCompetitionDisabled`](#kcompetitiondisabled) — *constant*
- [`kCompetitionAutonomous`](#kcompetitionautonomous) — *constant*
- [`kCompetitionConnected`](#kcompetitionconnected) — *constant*
- [`class CalibrateIo`](#class-calibrateio)
  - [`~CalibrateIo`](#calibrateio-destructor-calibrateio)
  - [`CalibrateIo`](#calibrateio-calibrateio)
  - [`CalibrateIo (overload 2)`](#calibrateio-calibrateio-2)
  - [`CalibrateIo (overload 3)`](#calibrateio-calibrateio-3)
  - [`operator=`](#calibrateio-operator-eq)
  - [`operator= (overload 2)`](#calibrateio-operator-eq-2)
  - [`line`](#calibrateio-line)
  - [`competitionStatus`](#calibrateio-competitionstatus)
  - [`delayMs`](#calibrateio-delayms)
  - [`readShaftDegrees`](#calibrateio-readshaftdegrees)
- [`competitionWords`](#competitionwords) — *free function*
- [`struct CalibrateDevices`](#struct-calibratedevices)
  - [`devices`](#calibratedevices-devices)
  - [`registry`](#calibratedevices-registry)
  - [`card`](#calibratedevices-card)
  - [`screen`](#calibratedevices-screen)
  - [`io`](#calibratedevices-io)
- [`class CalibrateProgram`](#class-calibrateprogram)
  - [`kPushLogMinSeconds`](#calibrateprogram-kpushlogminseconds)
  - [`kPushLogKeepaliveSeconds`](#calibrateprogram-kpushlogkeepaliveseconds)
  - [`CalibrateProgram`](#calibrateprogram-calibrateprogram)
  - [`CalibrateProgram (overload 2)`](#calibrateprogram-calibrateprogram-2)
  - [`operator=`](#calibrateprogram-operator-eq)
  - [`run`](#calibrateprogram-run)
- [`calibrateBootBanner`](#calibratebootbanner) — *free function*
- [`calibrateStateScreen`](#calibratestatescreen) — *free function*

<a id="struct-mechanismport"></a>

## `struct MechanismPort`

```cpp
struct MechanismPort
```

A motor Calibrate must never power: a mechanism's smart port, and what to call it in the log. LIST EVERY MOTOR THAT IS NOT A DRIVE MOTOR: a motor not listed is a drive CANDIDATE, and one that turns with the push is POWERED (3, then 6, then 9 V) like a drive motor. A 3-wire solenoid is not a motor and is never a candidate.

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:142`](../../include/shulib/calibration/calibrate_program.hpp#L142).*

<a id="mechanismport-port"></a>

### `MechanismPort::port`

```cpp
std::uint8_t port = 0
```

1..21; 0 = an unused entry (only if `name` is null too)

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:143`](../../include/shulib/calibration/calibrate_program.hpp#L143).*

<a id="mechanismport-name"></a>

### `MechanismPort::name`

```cpp
const char* name = nullptr
```

what the log and the screen call it; null prints "a mechanism"

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:144`](../../include/shulib/calibration/calibrate_program.hpp#L144).*

<a id="kmaxmechanismports"></a>

## `kMaxMechanismPorts`

```cpp
inline constexpr std::size_t kMaxMechanismPorts = hal::kMaxSmartPorts
```

The most mechanism motors a team can declare: every smart port, so a robot is never refused for counting.

*constant, declared at [`include/shulib/calibration/calibrate_program.hpp:149`](../../include/shulib/calibration/calibrate_program.hpp#L149).*

<a id="struct-typeddriveports"></a>

## `struct TypedDrivePorts`

```cpp
struct TypedDrivePorts
```

The drive ports a team ALREADY TYPES for its drive program (CAL1 D6 (a), the team lead 2026-10-08) — OPTIONAL: leave them out and nothing changes. When the card holds no usable record, the front check compares this run with THESE instead of with nothing (header). The spelling is the one a typed table already uses (src/robots/chassis_table.hpp's `.left = {-11, 12, -13, 14, -15}`, carried unchanged into hal::DriveSideDescription::signedPorts by the boot's tableToDescription()): a SIGNED port per motor on the side it drives, `-11` = port 11 reversed once in the adapter. So "SAME FRONT as the typed ports" means exactly "a drive built from them turns every motor the way this push measured". 0 = an unused slot (skipped, the order kept). They are compared, never calibrated from.

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:160`](../../include/shulib/calibration/calibrate_program.hpp#L160).*

<a id="typeddriveports-left"></a>

### `TypedDrivePorts::left`

```cpp
std::int8_t left[hal::kMaxSmartPorts]{}
```

the LEFT side's drive ports, signed; 0 = unused

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:161`](../../include/shulib/calibration/calibrate_program.hpp#L161).*

<a id="typeddriveports-right"></a>

### `TypedDrivePorts::right`

```cpp
std::int8_t right[hal::kMaxSmartPorts]{}
```

the RIGHT side's drive ports, signed; 0 = unused

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:162`](../../include/shulib/calibration/calibrate_program.hpp#L162).*

<a id="struct-calibratefacts"></a>

## `struct CalibrateFacts`

```cpp
struct CalibrateFacts
```

The facts only a person can state (header). The optional members are UNSET until stated: a value is never invented, and an unset kind or cartridge REFUSES before anything is made or powered. Every member has a default, so a team names only what it states (designated initializers, in this order — `typedDrive` is LAST, so a file that names the others still compiles).

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:170`](../../include/shulib/calibration/calibrate_program.hpp#L170).*

<a id="calibratefacts-drive"></a>

### `CalibrateFacts::drive`

```cpp
std::optional<config::DrivetrainKind> drive{}
```

the kind the team HAS; only Tank is measured

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:171`](../../include/shulib/calibration/calibrate_program.hpp#L171).*

<a id="calibratefacts-cartridge"></a>

### `CalibrateFacts::cartridge`

```cpp
std::optional<hal::MotorCartridge> cartridge{}
```

the DRIVE motors' cartridge (insert colour)

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:172`](../../include/shulib/calibration/calibrate_program.hpp#L172).*

<a id="calibratefacts-mechanisms"></a>

### `CalibrateFacts::mechanisms`

```cpp
MechanismPort mechanisms[kMaxMechanismPorts]{}
```

EVERY motor that is not a drive motor

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:173`](../../include/shulib/calibration/calibrate_program.hpp#L173).*

<a id="calibratefacts-robot"></a>

### `CalibrateFacts::robot`

```cpp
const char* robot = nullptr
```

optional: the robot's name, for the log

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:174`](../../include/shulib/calibration/calibrate_program.hpp#L174).*

<a id="calibratefacts-imuportstated"></a>

### `CalibrateFacts::imuPortStated`

```cpp
std::uint8_t imuPortStated = 0
```

optional cross-check (the registry decides); 0 = none

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:175`](../../include/shulib/calibration/calibrate_program.hpp#L175).*

<a id="calibratefacts-typeddrive"></a>

### `CalibrateFacts::typedDrive`

```cpp
TypedDrivePorts typedDrive{}
```

optional (D6 (a)): the drive ports the team types — the front check's reference when the card holds no usable record; empty = none

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:178`](../../include/shulib/calibration/calibrate_program.hpp#L178).*

<a id="calibratefacts-calibrator"></a>

### `CalibrateFacts::calibrator`

```cpp
CalibratorConfig calibrator{}
```

optional: the calibrator's thresholds for THIS robot (default: CalibratorConfig's). A drive with many motors per side turns the robot less per pulsed member (robot one, 2026-11-16: one member of five turned it ~1.0 deg at 9 V), so its robot file may lower `pulseYawFloor` — and with it `restYawRateFloor`, so a drift during a pulse stays under the floor (factsProblem() checks).

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:183`](../../include/shulib/calibration/calibrate_program.hpp#L183).*

<a id="calibratefacts-recordname"></a>

### `CalibrateFacts::recordName`

```cpp
const char* recordName = nullptr
```

optional (chunk ROB1-BT): THIS robot's name for its record (calibration::isRecordName). Stated, the record's provenance starts `robot=<name> <stamp>`, so the boot can refuse a card from its twin; unset (nullptr), it is "shulib Calibrate <stamp>" as before. LAST, so older facts still compile.

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:187`](../../include/shulib/calibration/calibrate_program.hpp#L187).*

<a id="enum-class-factslist"></a>

## `enum class FactsList`

```cpp
enum class FactsList : std::uint8_t
```

Which list of CalibrateFacts a FactsProblem's `entry` indexes.

*enum class, declared at [`include/shulib/calibration/calibrate_program.hpp:191`](../../include/shulib/calibration/calibrate_program.hpp#L191).*

<a id="factslist-mechanisms"></a>

### `FactsList::Mechanisms`

```cpp
Mechanisms
```

CalibrateFacts::mechanisms

*enumerator, declared at [`include/shulib/calibration/calibrate_program.hpp:192`](../../include/shulib/calibration/calibrate_program.hpp#L192).*

<a id="factslist-typedleft"></a>

### `FactsList::TypedLeft`

```cpp
TypedLeft
```

CalibrateFacts::typedDrive.left

*enumerator, declared at [`include/shulib/calibration/calibrate_program.hpp:193`](../../include/shulib/calibration/calibrate_program.hpp#L193).*

<a id="factslist-typedright"></a>

### `FactsList::TypedRight`

```cpp
TypedRight
```

CalibrateFacts::typedDrive.right

*enumerator, declared at [`include/shulib/calibration/calibrate_program.hpp:194`](../../include/shulib/calibration/calibrate_program.hpp#L194).*

<a id="factslist-recordname"></a>

### `FactsList::RecordName`

```cpp
RecordName
```

CalibrateFacts::recordName (no list: `entry` is 0)

*enumerator, declared at [`include/shulib/calibration/calibrate_program.hpp:195`](../../include/shulib/calibration/calibrate_program.hpp#L195).*

<a id="struct-factsproblem"></a>

## `struct FactsProblem`

```cpp
struct FactsProblem
```

Why a team's facts cannot be used: `why` is null when they can; `entry` is the index of the entry it names, in the list `list` names.

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:200`](../../include/shulib/calibration/calibrate_program.hpp#L200).*

<a id="factsproblem-why"></a>

### `FactsProblem::why`

```cpp
const char* why = nullptr
```

null = usable

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:201`](../../include/shulib/calibration/calibrate_program.hpp#L201).*

<a id="factsproblem-entry"></a>

### `FactsProblem::entry`

```cpp
std::size_t entry = 0
```

the entry it is about

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:202`](../../include/shulib/calibration/calibrate_program.hpp#L202).*

<a id="factsproblem-list"></a>

### `FactsProblem::list`

```cpp
FactsList list = FactsList::Mechanisms
```

the list `entry` indexes

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:203`](../../include/shulib/calibration/calibrate_program.hpp#L203).*

<a id="hastypeddrive"></a>

## `hasTypedDrive`

```cpp
[[nodiscard]] constexpr bool hasTypedDrive(const CalibrateFacts& f) noexcept
```

True when `f` states typed drive ports: any non-zero entry on EITHER side (a list with one side only is then refused by factsProblem(), never run as untyped).

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:208`](../../include/shulib/calibration/calibrate_program.hpp#L208).*

<a id="factsproblem"></a>

## `factsProblem`

```cpp
[[nodiscard]] constexpr FactsProblem factsProblem(const CalibrateFacts& f) noexcept
```

The first reason `f` cannot be used. A stated record name that is not one (isRecordName) first; then the mechanism list: an entry NAMED but with no port (a placeholder `{0, "intake"}` would silently leave that motor a drive candidate), a port outside 1..21, or a port listed twice (the SECOND entry is named). Then, when it states typed drive ports (D6 (a)): a port outside 1..21, a port listed twice — on one side, or once on each, by |port| (the description's own rule) — or only ONE side typed. constexpr, so a build can static_assert its own facts.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:224`](../../include/shulib/calibration/calibrate_program.hpp#L224).*

<a id="typeddrivedescription"></a>

## `typedDriveDescription`

```cpp
[[nodiscard]] constexpr hal::DrivetrainDescription typedDriveDescription( const CalibrateFacts& f) noexcept
```

The typed drive ports as a description's two sides — the reference compareFronts() reads when the card holds no usable record (header). Only the two sides are filled (the check reads nothing else); zero slots are skipped and the order kept. Assumes factsProblem(f) passed.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:293`](../../include/shulib/calibration/calibrate_program.hpp#L293).*

<a id="typeddriveportsfrom"></a>

## `typedDrivePortsFrom`

```cpp
[[nodiscard]] constexpr TypedDrivePorts typedDrivePortsFrom( const hal::DrivetrainDescription& d) noexcept
```

A description's two sides as typed drive ports — for a team whose drive program is a config::RobotConfig: `.typedDrive = typedDrivePortsFrom(kRobot.drivetrain.tank.description)` (the inverse of typedDriveDescription(); the `kRobot` must be a constant expression when the facts are `constexpr`).

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:313`](../../include/shulib/calibration/calibrate_program.hpp#L313).*

<a id="declaredmechanism"></a>

## `declaredMechanism`

```cpp
[[nodiscard]] constexpr const MechanismPort* declaredMechanism(const CalibrateFacts& f, int port) noexcept
```

The declared mechanism entry for physical `port`, or null.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:326`](../../include/shulib/calibration/calibrate_program.hpp#L326).*

<a id="kcompetitiondisabled"></a>

## `kCompetitionDisabled`

```cpp
inline constexpr std::uint8_t kCompetitionDisabled = 0x01
```

The competition status bit "disabled" (PROS's competition_get_status(); hal/pros/calibrate.hpp checks the three against PROS's own values at compile time). Written in hex: the API reference tool reads `<<` as a bracket.

*constant, declared at [`include/shulib/calibration/calibrate_program.hpp:341`](../../include/shulib/calibration/calibrate_program.hpp#L341).*

<a id="kcompetitionautonomous"></a>

## `kCompetitionAutonomous`

```cpp
inline constexpr std::uint8_t kCompetitionAutonomous = 0x02
```

The competition status bit "autonomous" — see kCompetitionDisabled.

*constant, declared at [`include/shulib/calibration/calibrate_program.hpp:343`](../../include/shulib/calibration/calibrate_program.hpp#L343).*

<a id="kcompetitionconnected"></a>

## `kCompetitionConnected`

```cpp
inline constexpr std::uint8_t kCompetitionConnected = 0x04
```

The competition status bit "field or switch connected" — see kCompetitionDisabled.

*constant, declared at [`include/shulib/calibration/calibrate_program.hpp:345`](../../include/shulib/calibration/calibrate_program.hpp#L345).*

<a id="class-calibrateio"></a>

## `class CalibrateIo`

```cpp
class CalibrateIo
```

Everything the program reaches outside the library's seams — four calls. On the brain: hal::pros::ProsCalibrateIo (hal/pros/calibrate.hpp); on the host: a test's fake.

*class, declared at [`include/shulib/calibration/calibrate_program.hpp:349`](../../include/shulib/calibration/calibrate_program.hpp#L349).*

<a id="calibrateio-destructor-calibrateio"></a>

### `CalibrateIo::~CalibrateIo`

```cpp
virtual ~CalibrateIo() = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:353`](../../include/shulib/calibration/calibrate_program.hpp#L353).*

<a id="calibrateio-calibrateio"></a>

### `CalibrateIo::CalibrateIo`

```cpp
CalibrateIo() = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:355`](../../include/shulib/calibration/calibrate_program.hpp#L355).*

<a id="calibrateio-calibrateio-2"></a>

### `CalibrateIo::CalibrateIo (overload 2)`

```cpp
CalibrateIo(const CalibrateIo&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:357`](../../include/shulib/calibration/calibrate_program.hpp#L357).*

<a id="calibrateio-calibrateio-3"></a>

### `CalibrateIo::CalibrateIo (overload 3)`

```cpp
CalibrateIo(CalibrateIo&&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base. Movable at the base.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:359`](../../include/shulib/calibration/calibrate_program.hpp#L359).*

<a id="calibrateio-operator-eq"></a>

### `CalibrateIo::operator=`

```cpp
CalibrateIo& operator=(const CalibrateIo&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:361`](../../include/shulib/calibration/calibrate_program.hpp#L361).*

<a id="calibrateio-operator-eq-2"></a>

### `CalibrateIo::operator= (overload 2)`

```cpp
CalibrateIo& operator=(CalibrateIo&&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base. Move-assignable at the base.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:363`](../../include/shulib/calibration/calibrate_program.hpp#L363).*

<a id="calibrateio-line"></a>

### `CalibrateIo::line`

```cpp
virtual void line(const char* text) = 0
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base. Move-assignable at the base. One serial line, WITHOUT its "[R3D] " tag and newline (the platform adds them). On the brain it reaches the USB serial as ONE write() per line (SER1).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:366`](../../include/shulib/calibration/calibrate_program.hpp#L366).*

<a id="calibrateio-competitionstatus"></a>

### `CalibrateIo::competitionStatus`

```cpp
[[nodiscard]] virtual std::uint8_t competitionStatus() = 0
```

The competition status byte (the kCompetition* bits).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:368`](../../include/shulib/calibration/calibrate_program.hpp#L368).*

<a id="calibrateio-delayms"></a>

### `CalibrateIo::delayMs`

```cpp
virtual void delayMs(std::uint32_t ms) = 0
```

Wait `ms` milliseconds — a plain wait (the touch baseline's retry, the fault's idle).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:370`](../../include/shulib/calibration/calibrate_program.hpp#L370).*

<a id="calibrateio-readshaftdegrees"></a>

### `CalibrateIo::readShaftDegrees`

```cpp
[[nodiscard]] virtual bool readShaftDegrees(int port, double& degrees) = 0
```

THE WATCH'S READ: the DECLARED mechanism motor on `port`'s shaft position, degrees, read WITHOUT configuring or commanding it. False when it cannot be read (then nothing is judged).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:373`](../../include/shulib/calibration/calibrate_program.hpp#L373).*

<a id="competitionwords"></a>

## `competitionWords`

```cpp
inline const char* competitionWords(std::uint8_t s, char* buf, std::size_t n)
```

The status byte in words, into `buf`: "0x04 field=yes disabled=no auton=no". Returns `buf`.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:377`](../../include/shulib/calibration/calibrate_program.hpp#L377).*

<a id="struct-calibratedevices"></a>

## `struct CalibrateDevices`

```cpp
struct CalibrateDevices
```

The devices the program runs on, all BORROWED: each must outlive the program.

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:386`](../../include/shulib/calibration/calibrate_program.hpp#L386).*

<a id="calibratedevices-devices"></a>

### `CalibrateDevices::devices`

```cpp
hal::IDeviceProvider& devices
```

motors and the IMU by port; the clock; the pacer

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:387`](../../include/shulib/calibration/calibrate_program.hpp#L387).*

<a id="calibratedevices-registry"></a>

### `CalibrateDevices::registry`

```cpp
hal::IDeviceRegistry& registry
```

what is plugged in, read without opening anything

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:388`](../../include/shulib/calibration/calibrate_program.hpp#L388).*

<a id="calibratedevices-card"></a>

### `CalibrateDevices::card`

```cpp
hal::IFileStore& card
```

the SD card (the record and its backup)

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:389`](../../include/shulib/calibration/calibrate_program.hpp#L389).*

<a id="calibratedevices-screen"></a>

### `CalibrateDevices::screen`

```cpp
hal::IBrainScreen& screen
```

the brain's screen and touch

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:390`](../../include/shulib/calibration/calibrate_program.hpp#L390).*

<a id="calibratedevices-io"></a>

### `CalibrateDevices::io`

```cpp
CalibrateIo& io
```

the serial line, the competition status, a wait, the watch

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:391`](../../include/shulib/calibration/calibrate_program.hpp#L391).*

<a id="class-calibrateprogram"></a>

## `class CalibrateProgram`

```cpp
class CalibrateProgram
```

THE STANDARD CALIBRATE (header). Construct it over the robot's devices and the team's facts and call run() from opcontrol(): it owns its own 10 ms loop, and returns when the field disables the robot or starts autonomous (cleanly, every motor at 0 V + Coast) or when no run can start (a refusal painted and printed before anything is made). Not copyable; one per program.

*class, declared at [`include/shulib/calibration/calibrate_program.hpp:506`](../../include/shulib/calibration/calibrate_program.hpp#L506).*

<a id="calibrateprogram-kpushlogminseconds"></a>

### `CalibrateProgram::kPushLogMinSeconds`

```cpp
static constexpr double kPushLogMinSeconds = 0.25
```

The least time between two `push live:` serial lines while the count changes, seconds.

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:509`](../../include/shulib/calibration/calibrate_program.hpp#L509).*

<a id="calibrateprogram-kpushlogkeepaliveseconds"></a>

### `CalibrateProgram::kPushLogKeepaliveSeconds`

```cpp
static constexpr double kPushLogKeepaliveSeconds = 5.0
```

The longest time between two `push live:` lines while nothing changes, seconds.

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:511`](../../include/shulib/calibration/calibrate_program.hpp#L511).*

<a id="calibrateprogram-calibrateprogram"></a>

### `CalibrateProgram::CalibrateProgram`

```cpp
CalibrateProgram(const CalibrateFacts& facts, CalibrateDevices d, const char* buildStamp)
```

`buildStamp` is "<date> <time>" of the build (the caller's __DATE__ " " __TIME__): the banner prints it whole and the screen's header its time. `facts` is copied; `d` and the stamp are borrowed and must outlive the program.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:516`](../../include/shulib/calibration/calibrate_program.hpp#L516).*

<a id="calibrateprogram-calibrateprogram-2"></a>

### `CalibrateProgram::CalibrateProgram (overload 2)`

```cpp
CalibrateProgram(const CalibrateProgram&) = delete
```

Not copyable: it borrows the robot's devices, and a copy would be a second program commanding the same motors.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:522`](../../include/shulib/calibration/calibrate_program.hpp#L522).*

<a id="calibrateprogram-operator-eq"></a>

### `CalibrateProgram::operator=`

```cpp
CalibrateProgram& operator=(const CalibrateProgram&) = delete
```

Not copyable: it borrows the robot's devices, and a copy would be a second program commanding the same motors. Not copy-assignable, for the same reason.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:524`](../../include/shulib/calibration/calibrate_program.hpp#L524).*

<a id="calibrateprogram-run"></a>

### `CalibrateProgram::run`

```cpp
void run()
```

Run calibrations until driver control ends — the field's disable, or autonomous starting — or return at once when no run can start (a refusal, painted and printed) (header).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:528`](../../include/shulib/calibration/calibrate_program.hpp#L528).*

<a id="calibratebootbanner"></a>

## `calibrateBootBanner`

```cpp
inline void calibrateBootBanner(CalibrateIo& io, const char* label, std::uint8_t status, const char* buildStamp)
```

initialize()'s banner for the standard Calibrate, through `io`: `label` (a build's beacon, or null), the competition status at boot, the build stamp. Constructs nothing and powers nothing — the motors are found when driver control starts.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:1860`](../../include/shulib/calibration/calibrate_program.hpp#L1860).*

<a id="calibratestatescreen"></a>

## `calibrateStateScreen`

```cpp
inline void calibrateStateScreen(hal::IBrainScreen& screen, CalibrateIo& io, const char* state, const char* why)
```

disabled() / competition_initialize() / autonomous() for the standard Calibrate: its competition-state screen on `screen` (it runs ONLY in driver control) and one serial line through `io`. Pass calibrate_screen.hpp's words — screen_text::kStateName* for `state` and kStateWhy* for `why` — which are checked at compile time to fit; any other text is cut, and the screen says so. A free function: those entry points run on their own PROS tasks, before and after run()'s program exists.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:1893`](../../include/shulib/calibration/calibrate_program.hpp#L1893).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 101 lines, click to expand</summary>

```text

 calibrate_program — THE STANDARD "shulib Calibrate" (chunk CAL1, Part A2a, 2026-10-18): the robot
 measures its own drivetrain, for ANY team, from the few facts only a person can state.

 ── What a team states, and nothing else ────────────────────────────────────────────
 CalibrateFacts: the drive KIND (config::DrivetrainKind — only a tank is measured; anything else
 is refused by name), the drive motors' CARTRIDGE (no reading can tell it: the rotor turns the same
 for every insert), and the port of EVERY motor that is NOT a drive motor (its MECHANISMS — the
 registry reports a motor's kind, never its job). Everything else is discovered on the robot: which
 ports drive, which way each is wired, which side, each side's inches per radian, the effective
 track width, the IMU's port — and THE FRONT, which is the end that LEADS the push (the team lead,
 2026-10-06). A motor the facts do not list is a drive CANDIDATE: one that turns with the push is
 POWERED (3, then 6, then 9 V) like a drive motor. The program says so wherever it can (its banner
 when nothing is declared, a line per candidate that did not turn, the LEFT OUT and pulse-refusal
 lines), because the facts are the only guard.

 ── Where it comes from ─────────────────────────────────────────────────────────────
 This is "shulib Calibrate" — src/programs/calibrate_program.cpp (R3d Part 2F-2, 2026-09-24; SER1's
 one-call serial lines, 2026-10-15) — MOVED: the census, the declared-port drop, the refusals, the
 calibrator, the touch, the serial record and RUN AGAIN are its code, in its order. What changed is
 the CAL1 brief's §1.3 list and nothing else: the words that named robot two are the facts'; the
 front is the push's (CAL1 Part B, 2026-11-05, removed the calibrator's WHICH END question, which
 this program had answered itself the tick DONE was accepted); the save is SAFE-SAVE B1's
 (calibration/record_store.hpp); and three things are new — the FRONT CHECK, the WATCH of the
 declared ports, and the two HELD screens. Since CAL1 Part B robot two runs THIS program: its
 src/programs/calibrate_program.cpp is a 27-line glue over hal/pros/calibrate.hpp, with facts
 derived from its tables (src/robots/calibrate_facts.hpp).

 ── It owns NO pixel and NO PROS call ───────────────────────────────────────────────
 It paints through calibrate_screen.hpp onto an hal::IBrainScreen, builds its motors and its IMU
 through an hal::IDeviceProvider (the provider asks the registry before it constructs, and catches
 an adapter's throw per device), reads the census from an hal::IDeviceRegistry, reads and saves the
 record through an hal::IFileStore, and reaches the rest — a serial line, the competition status, a
 plain wait, and the watch's read of a declared motor — through CalibrateIo, four calls. So the
 whole program runs on the host test plant (hal::fake), and hal/pros/calibrate.hpp is its thin robot
 shell.

 ── The FRONT CHECK, before anything is saved ───────────────────────────────────────
 At DONE (after the MEASURED lines, before any write) the card's record is read the way the boot
 reads it (loadRecord(): the record file, else its backup) and compared with this run's port by port
 (front_check.hpp's compareFronts()). No card: no check (nothing can be saved). A card holding no
 record (neither file opened): the FIRST record, saved with a NOTE (painted only over a save that
 SUCCEEDED: under NOT SAVED it would name a record the card does not hold). Files that opened and do
 not decode: NOT DONE, a line per file naming it and its reason, saved with a NOTE. SAME FRONT: saved.
 CANNOT TELL: saved with a NOTE. FRONT FLIPPED: the save WAITS on the FRONT FLIPPED screen — USE NEW
 (save as pushed: right after a LEFT/RIGHT cable swap or a deliberate change of front) or RUN AGAIN
 (save nothing: right after a wrong push). The records cannot tell those apart, so the screen states
 what the RECORDS say and never what the operator did, and nothing is ever turned to another front:
 every saved record's front is the end that led its push.
 THE RECORD'S NAME (chunk ROB1-BT): when the facts name this robot and the card's record names another
 robot or none, the card line and the front check each add a line saying so. The front is still checked
 against it (the only front the card holds; twins are wired the same, HA-256), and the save replaces it.
 THE TYPED DRIVE PORTS (CAL1 Part A2b, 2026-10-30 — the team lead's D6 (a), 2026-10-08): a team
 whose drive program already TYPES its drive ports may state them (CalibrateFacts::typedDrive,
 optional). With a usable record on the card they change nothing — not a line, not a draw, not a
 byte. With NO usable record on a present card (neither file opened, or files that opened and do
 not decode: the boot treats a record that does not decode as absent, and drives from a valid typed
 table) they are the reference instead: the same verdicts in their own words, FRONT FLIPPED held the
 same way (USE NEW, then fix the typed ports; or RUN AGAIN), and SAME noted as an EXACT match
 (every port in either list agrees) or a partial one (the serial log names each port that differs:
 a typed port with a wrong sign fights its side-mates in anything that drives from them). They are
 COMPARED, never copied into a record: Calibrate never calibrates from them. With no card there is
 no check, typed or not. The words that say what a discarded run leaves follow the CARD's contents,
 never the reference.
 THE LIMIT: the check and the save read the card separately (one program, one task: no race). A
 card that misreads a GOOD record as absent at the check makes this run a FIRST record, saved with
 no flip check — SAFE-SAVE's two-fault limit (HA-139, the belief that a read returns what the card
 holds; record_store.hpp, THE LIMIT). No third read is added to hide it: a third read can misread
 too. So a FIRST RECORD note means "the check READ no record", never "the card held none". (With
 typed drive ports in the facts, such a run is checked against THEM instead — still not against the
 record the card holds.)

 ── The WATCH, and the second hold ──────────────────────────────────────────────────
 Every declared mechanism port is READ when the push begins and again at DONE — never commanded,
 never configured: CalibrateIo::readShaftDegrees(), which on the brain is PROS's C reads by port and
 no motor object (hal/pros/calibrate.hpp; its belief is HA-187 in docs/hardware-assumptions.md). One
 that turned as far as the push's own movedThreshold — the calibrator's own test for a drive port,
 so no new threshold — HOLDS the save on MECHANISM PORT TURNED: SAVE (a mechanism the push turns:
 keep it declared) or RUN AGAIN (a drive motor declared by mistake: fix the facts). After a USE NEW
 it follows FRONT FLIPPED: one question per screen. EVERY declared port that turned is its own
 question, on its own screen, in the facts' order — each with ITS turn and whether the card's record
 (or, when they were the reference, the typed drive ports) drives with IT, its boxes greyed again as
 it arrives — and nothing is saved until every one is
 answered SAVE: SAVE is a judgment about one port, so one port's evidence never stands for another's
 (an intake the push legitimately turns must not hide a drive motor declared by mistake). Only an
 ACCEPTED push is judged — a push the calibrator refuses at DONE saves nothing, so the watch says
 nothing about it.

 ── Nothing is written while HELD, and every exit leaves the motors at 0 V + Coast ──
 A hold is a SaveOutcome, not a calibrator phase: nothing is powered, and RUN AGAIN, the end of
 driver control or a power-off ends it with nothing written — the words then say what the card
 holds (its record, no usable record, or no card at all), and what ended it (the field's disable,
 or autonomous starting). An RAII guard runs the all-stop on every C++ exit; the competition status
 is polled every tick, and EITHER bit — disabled, or autonomous — exits the loop cleanly (PROS
 deletes the opcontrol task at either change, and a deleted task runs no destructors); a
 PreconditionError inside the loop is caught once at its edge (PROGRAM FAULT: everything stopped,
 said, idled until driver control ends the same way).

 NO HARDWARE CLAIM: this program has not run on a robot in this form. The calibrator inside it ran
 on robot two in its R3d Part 2F form on 2026-09-24 (one run stopped, one finished unrecorded).
 Every threshold is INVENTED (A4 register HA-136). PROS-free, sim-free; allocation-free.
```

</details>
