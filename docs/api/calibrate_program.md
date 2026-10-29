<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/calibrate_program.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `calibrate_program.hpp`

calibrate_program — THE STANDARD "shulib Calibrate" (chunk CAL1, Part A2a, 2026-10-18): the robot measures its own drivetrain, for ANY team, from the few facts only a person can state.

This header declares **6** types (30 members), **5** free functions, and **4** constants.

Extracted from [`include/shulib/calibration/calibrate_program.hpp`](../../include/shulib/calibration/calibrate_program.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct MechanismPort`](#struct-mechanismport)
  - [`port`](#mechanismport-port)
  - [`name`](#mechanismport-name)
- [`kMaxMechanismPorts`](#kmaxmechanismports) — *constant*
- [`struct CalibrateFacts`](#struct-calibratefacts)
  - [`drive`](#calibratefacts-drive)
  - [`cartridge`](#calibratefacts-cartridge)
  - [`mechanisms`](#calibratefacts-mechanisms)
  - [`robot`](#calibratefacts-robot)
  - [`imuPortStated`](#calibratefacts-imuportstated)
- [`struct FactsProblem`](#struct-factsproblem)
  - [`why`](#factsproblem-why)
  - [`entry`](#factsproblem-entry)
- [`factsProblem`](#factsproblem) — *free function*
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

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:124`](../../include/shulib/calibration/calibrate_program.hpp#L124).*

<a id="mechanismport-port"></a>

### `MechanismPort::port`

```cpp
std::uint8_t port = 0
```

1..21; 0 = an unused entry (only if `name` is null too)

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:125`](../../include/shulib/calibration/calibrate_program.hpp#L125).*

<a id="mechanismport-name"></a>

### `MechanismPort::name`

```cpp
const char* name = nullptr
```

what the log and the screen call it; null prints "a mechanism"

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:126`](../../include/shulib/calibration/calibrate_program.hpp#L126).*

<a id="kmaxmechanismports"></a>

## `kMaxMechanismPorts`

```cpp
inline constexpr std::size_t kMaxMechanismPorts = hal::kMaxSmartPorts
```

The most mechanism motors a team can declare: every smart port, so a robot is never refused for counting.

*constant, declared at [`include/shulib/calibration/calibrate_program.hpp:131`](../../include/shulib/calibration/calibrate_program.hpp#L131).*

<a id="struct-calibratefacts"></a>

## `struct CalibrateFacts`

```cpp
struct CalibrateFacts
```

The facts only a person can state (header). The two optional members are UNSET until stated: a value is never invented, and an unset one REFUSES before anything is made or powered. Every member has a default, so a team names only what it states (designated initializers).

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:136`](../../include/shulib/calibration/calibrate_program.hpp#L136).*

<a id="calibratefacts-drive"></a>

### `CalibrateFacts::drive`

```cpp
std::optional<config::DrivetrainKind> drive{}
```

the kind the team HAS; only Tank is measured

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:137`](../../include/shulib/calibration/calibrate_program.hpp#L137).*

<a id="calibratefacts-cartridge"></a>

### `CalibrateFacts::cartridge`

```cpp
std::optional<hal::MotorCartridge> cartridge{}
```

the DRIVE motors' cartridge (insert colour)

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:138`](../../include/shulib/calibration/calibrate_program.hpp#L138).*

<a id="calibratefacts-mechanisms"></a>

### `CalibrateFacts::mechanisms`

```cpp
MechanismPort mechanisms[kMaxMechanismPorts]{}
```

EVERY motor that is not a drive motor

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:139`](../../include/shulib/calibration/calibrate_program.hpp#L139).*

<a id="calibratefacts-robot"></a>

### `CalibrateFacts::robot`

```cpp
const char* robot = nullptr
```

optional: the robot's name, for the log

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:140`](../../include/shulib/calibration/calibrate_program.hpp#L140).*

<a id="calibratefacts-imuportstated"></a>

### `CalibrateFacts::imuPortStated`

```cpp
std::uint8_t imuPortStated = 0
```

optional cross-check (the registry decides); 0 = none

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:141`](../../include/shulib/calibration/calibrate_program.hpp#L141).*

<a id="struct-factsproblem"></a>

## `struct FactsProblem`

```cpp
struct FactsProblem
```

Why a team's facts cannot be used: `why` is null when they can; `entry` is the index into CalibrateFacts::mechanisms of the entry it names.

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:146`](../../include/shulib/calibration/calibrate_program.hpp#L146).*

<a id="factsproblem-why"></a>

### `FactsProblem::why`

```cpp
const char* why = nullptr
```

null = usable

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:147`](../../include/shulib/calibration/calibrate_program.hpp#L147).*

<a id="factsproblem-entry"></a>

### `FactsProblem::entry`

```cpp
std::size_t entry = 0
```

the mechanisms[] entry it is about

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:148`](../../include/shulib/calibration/calibrate_program.hpp#L148).*

<a id="factsproblem"></a>

## `factsProblem`

```cpp
[[nodiscard]] constexpr FactsProblem factsProblem(const CalibrateFacts& f) noexcept
```

The first reason `f`'s mechanism list cannot be used: an entry NAMED but with no port (a placeholder `{0, "intake"}` would silently leave that motor a drive candidate), a port outside 1..21, or a port listed twice (the SECOND entry is named). constexpr, so a build can static_assert its own facts.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:155`](../../include/shulib/calibration/calibrate_program.hpp#L155).*

<a id="declaredmechanism"></a>

## `declaredMechanism`

```cpp
[[nodiscard]] constexpr const MechanismPort* declaredMechanism(const CalibrateFacts& f, int port) noexcept
```

The declared mechanism entry for physical `port`, or null.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:177`](../../include/shulib/calibration/calibrate_program.hpp#L177).*

<a id="kcompetitiondisabled"></a>

## `kCompetitionDisabled`

```cpp
inline constexpr std::uint8_t kCompetitionDisabled = 0x01
```

The competition status bit "disabled" (PROS's competition_get_status(); hal/pros/calibrate.hpp checks the three against PROS's own values at compile time). Written in hex: the API reference tool reads `<<` as a bracket.

*constant, declared at [`include/shulib/calibration/calibrate_program.hpp:192`](../../include/shulib/calibration/calibrate_program.hpp#L192).*

<a id="kcompetitionautonomous"></a>

## `kCompetitionAutonomous`

```cpp
inline constexpr std::uint8_t kCompetitionAutonomous = 0x02
```

The competition status bit "autonomous" — see kCompetitionDisabled.

*constant, declared at [`include/shulib/calibration/calibrate_program.hpp:194`](../../include/shulib/calibration/calibrate_program.hpp#L194).*

<a id="kcompetitionconnected"></a>

## `kCompetitionConnected`

```cpp
inline constexpr std::uint8_t kCompetitionConnected = 0x04
```

The competition status bit "field or switch connected" — see kCompetitionDisabled.

*constant, declared at [`include/shulib/calibration/calibrate_program.hpp:196`](../../include/shulib/calibration/calibrate_program.hpp#L196).*

<a id="class-calibrateio"></a>

## `class CalibrateIo`

```cpp
class CalibrateIo
```

Everything the program reaches outside the library's seams — four calls. On the brain: hal::pros::ProsCalibrateIo (hal/pros/calibrate.hpp); on the host: a test's fake.

*class, declared at [`include/shulib/calibration/calibrate_program.hpp:200`](../../include/shulib/calibration/calibrate_program.hpp#L200).*

<a id="calibrateio-destructor-calibrateio"></a>

### `CalibrateIo::~CalibrateIo`

```cpp
virtual ~CalibrateIo() = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:204`](../../include/shulib/calibration/calibrate_program.hpp#L204).*

<a id="calibrateio-calibrateio"></a>

### `CalibrateIo::CalibrateIo`

```cpp
CalibrateIo() = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:206`](../../include/shulib/calibration/calibrate_program.hpp#L206).*

<a id="calibrateio-calibrateio-2"></a>

### `CalibrateIo::CalibrateIo (overload 2)`

```cpp
CalibrateIo(const CalibrateIo&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:208`](../../include/shulib/calibration/calibrate_program.hpp#L208).*

<a id="calibrateio-calibrateio-3"></a>

### `CalibrateIo::CalibrateIo (overload 3)`

```cpp
CalibrateIo(CalibrateIo&&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base. Movable at the base.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:210`](../../include/shulib/calibration/calibrate_program.hpp#L210).*

<a id="calibrateio-operator-eq"></a>

### `CalibrateIo::operator=`

```cpp
CalibrateIo& operator=(const CalibrateIo&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:212`](../../include/shulib/calibration/calibrate_program.hpp#L212).*

<a id="calibrateio-operator-eq-2"></a>

### `CalibrateIo::operator= (overload 2)`

```cpp
CalibrateIo& operator=(CalibrateIo&&) = default
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base. Move-assignable at the base.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:214`](../../include/shulib/calibration/calibrate_program.hpp#L214).*

<a id="calibrateio-line"></a>

### `CalibrateIo::line`

```cpp
virtual void line(const char* text) = 0
```

Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base holds no state). Default-constructible. Copyable at the base. Movable at the base. Copy-assignable at the base. Move-assignable at the base. One serial line, WITHOUT its "[R3D] " tag and newline (the platform adds them). On the brain it reaches the USB serial as ONE write() per line (SER1).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:217`](../../include/shulib/calibration/calibrate_program.hpp#L217).*

<a id="calibrateio-competitionstatus"></a>

### `CalibrateIo::competitionStatus`

```cpp
[[nodiscard]] virtual std::uint8_t competitionStatus() = 0
```

The competition status byte (the kCompetition* bits).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:219`](../../include/shulib/calibration/calibrate_program.hpp#L219).*

<a id="calibrateio-delayms"></a>

### `CalibrateIo::delayMs`

```cpp
virtual void delayMs(std::uint32_t ms) = 0
```

Wait `ms` milliseconds — a plain wait (the touch baseline's retry, the fault's idle).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:221`](../../include/shulib/calibration/calibrate_program.hpp#L221).*

<a id="calibrateio-readshaftdegrees"></a>

### `CalibrateIo::readShaftDegrees`

```cpp
[[nodiscard]] virtual bool readShaftDegrees(int port, double& degrees) = 0
```

THE WATCH'S READ: the DECLARED mechanism motor on `port`'s shaft position, degrees, read WITHOUT configuring or commanding it. False when it cannot be read (then nothing is judged).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:224`](../../include/shulib/calibration/calibrate_program.hpp#L224).*

<a id="competitionwords"></a>

## `competitionWords`

```cpp
inline const char* competitionWords(std::uint8_t s, char* buf, std::size_t n)
```

The status byte in words, into `buf`: "0x04 field=yes disabled=no auton=no". Returns `buf`.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:228`](../../include/shulib/calibration/calibrate_program.hpp#L228).*

<a id="struct-calibratedevices"></a>

## `struct CalibrateDevices`

```cpp
struct CalibrateDevices
```

The devices the program runs on, all BORROWED: each must outlive the program.

*struct, declared at [`include/shulib/calibration/calibrate_program.hpp:237`](../../include/shulib/calibration/calibrate_program.hpp#L237).*

<a id="calibratedevices-devices"></a>

### `CalibrateDevices::devices`

```cpp
hal::IDeviceProvider& devices
```

motors and the IMU by port; the clock; the pacer

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:238`](../../include/shulib/calibration/calibrate_program.hpp#L238).*

<a id="calibratedevices-registry"></a>

### `CalibrateDevices::registry`

```cpp
hal::IDeviceRegistry& registry
```

what is plugged in, read without opening anything

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:239`](../../include/shulib/calibration/calibrate_program.hpp#L239).*

<a id="calibratedevices-card"></a>

### `CalibrateDevices::card`

```cpp
hal::IFileStore& card
```

the SD card (the record and its backup)

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:240`](../../include/shulib/calibration/calibrate_program.hpp#L240).*

<a id="calibratedevices-screen"></a>

### `CalibrateDevices::screen`

```cpp
hal::IBrainScreen& screen
```

the brain's screen and touch

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:241`](../../include/shulib/calibration/calibrate_program.hpp#L241).*

<a id="calibratedevices-io"></a>

### `CalibrateDevices::io`

```cpp
CalibrateIo& io
```

the serial line, the competition status, a wait, the watch

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:242`](../../include/shulib/calibration/calibrate_program.hpp#L242).*

<a id="class-calibrateprogram"></a>

## `class CalibrateProgram`

```cpp
class CalibrateProgram
```

THE STANDARD CALIBRATE (header). Construct it over the robot's devices and the team's facts and call run() from opcontrol(): it owns its own 10 ms loop, and returns when the field disables the robot or starts autonomous (cleanly, every motor at 0 V + Coast) or when no run can start (a refusal painted and printed before anything is made). Not copyable; one per program.

*class, declared at [`include/shulib/calibration/calibrate_program.hpp:359`](../../include/shulib/calibration/calibrate_program.hpp#L359).*

<a id="calibrateprogram-kpushlogminseconds"></a>

### `CalibrateProgram::kPushLogMinSeconds`

```cpp
static constexpr double kPushLogMinSeconds = 0.25
```

The least time between two `push live:` serial lines while the count changes, seconds.

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:362`](../../include/shulib/calibration/calibrate_program.hpp#L362).*

<a id="calibrateprogram-kpushlogkeepaliveseconds"></a>

### `CalibrateProgram::kPushLogKeepaliveSeconds`

```cpp
static constexpr double kPushLogKeepaliveSeconds = 5.0
```

The longest time between two `push live:` lines while nothing changes, seconds.

*field, declared at [`include/shulib/calibration/calibrate_program.hpp:364`](../../include/shulib/calibration/calibrate_program.hpp#L364).*

<a id="calibrateprogram-calibrateprogram"></a>

### `CalibrateProgram::CalibrateProgram`

```cpp
CalibrateProgram(const CalibrateFacts& facts, CalibrateDevices d, const char* buildStamp)
```

`buildStamp` is "<date> <time>" of the build (the caller's __DATE__ " " __TIME__): the banner prints it whole and the screen's header its time. `facts` is copied; `d` and the stamp are borrowed and must outlive the program.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:369`](../../include/shulib/calibration/calibrate_program.hpp#L369).*

<a id="calibrateprogram-calibrateprogram-2"></a>

### `CalibrateProgram::CalibrateProgram (overload 2)`

```cpp
CalibrateProgram(const CalibrateProgram&) = delete
```

Not copyable: it borrows the robot's devices, and a copy would be a second program commanding the same motors.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:375`](../../include/shulib/calibration/calibrate_program.hpp#L375).*

<a id="calibrateprogram-operator-eq"></a>

### `CalibrateProgram::operator=`

```cpp
CalibrateProgram& operator=(const CalibrateProgram&) = delete
```

Not copyable: it borrows the robot's devices, and a copy would be a second program commanding the same motors. Not copy-assignable, for the same reason.

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:377`](../../include/shulib/calibration/calibrate_program.hpp#L377).*

<a id="calibrateprogram-run"></a>

### `CalibrateProgram::run`

```cpp
void run()
```

Run calibrations until driver control ends — the field's disable, or autonomous starting — or return at once when no run can start (a refusal, painted and printed) (header).

*function, declared at [`include/shulib/calibration/calibrate_program.hpp:381`](../../include/shulib/calibration/calibrate_program.hpp#L381).*

<a id="calibratebootbanner"></a>

## `calibrateBootBanner`

```cpp
inline void calibrateBootBanner(CalibrateIo& io, const char* label, std::uint8_t status, const char* buildStamp)
```

initialize()'s banner for the standard Calibrate, through `io`: `label` (a build's beacon, or null), the competition status at boot, the build stamp. Constructs nothing and powers nothing — the motors are found when driver control starts.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:1552`](../../include/shulib/calibration/calibrate_program.hpp#L1552).*

<a id="calibratestatescreen"></a>

## `calibrateStateScreen`

```cpp
inline void calibrateStateScreen(hal::IBrainScreen& screen, CalibrateIo& io, const char* state, const char* why)
```

disabled() / competition_initialize() / autonomous() for the standard Calibrate: its competition-state screen on `screen` (it runs ONLY in driver control) and one serial line through `io`. Pass calibrate_screen.hpp's words — screen_text::kStateName* for `state` and kStateWhy* for `why` — which are checked at compile time to fit; any other text is cut, and the screen says so. A free function: those entry points run on their own PROS tasks, before and after run()'s program exists.

*free function, declared at [`include/shulib/calibration/calibrate_program.hpp:1585`](../../include/shulib/calibration/calibrate_program.hpp#L1585).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 83 lines, click to expand</summary>

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
 front is the push's (the program answers the calibrator's WHICH END question itself, the tick DONE
 is accepted, so that screen is never painted — CAL1 Part B deletes the question); the save is
 SAFE-SAVE B1's (calibration/record_store.hpp); and three things are new — the FRONT CHECK, the
 WATCH of the declared ports, and the two HELD screens. That program stays robot two's until CAL1
 Part B moves robot two onto this one.

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
 every saved record's front is the end that led its push. (CAL1 Part A2b adds the team's TYPED drive
 ports as the reference when the card holds no usable record — the team lead's D6 (a).)
 THE LIMIT: the check and the save read the card separately (one program, one task: no race). A
 card that misreads a GOOD record as absent at the check makes this run a FIRST record, saved with
 no flip check — SAFE-SAVE's two-fault limit (HA-139, the belief that a read returns what the card
 holds; record_store.hpp, THE LIMIT). No third read is added to hide it: a third read can misread
 too. So a FIRST RECORD note means "the check READ no record", never "the card held none".

 ── The WATCH, and the second hold ──────────────────────────────────────────────────
 Every declared mechanism port is READ when the push begins and again at DONE — never commanded,
 never configured: CalibrateIo::readShaftDegrees(), which on the brain is PROS's C reads by port and
 no motor object (hal/pros/calibrate.hpp; the belief CAL1 registers as HA-187, in Part A2b). One
 that turned as far as the push's own movedThreshold — the calibrator's own test for a drive port,
 so no new threshold — HOLDS the save on MECHANISM PORT TURNED: SAVE (a mechanism the push turns:
 keep it declared) or RUN AGAIN (a drive motor declared by mistake: fix the facts). After a USE NEW
 it follows FRONT FLIPPED: one question per screen. EVERY declared port that turned is its own
 question, on its own screen, in the facts' order — each with ITS turn and whether the card's record
 drives with IT, its boxes greyed again as it arrives — and nothing is saved until every one is
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
