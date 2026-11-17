<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/robot/robot.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `robot.hpp`

robot::Robot — a config::RobotConfig, wired.

This header declares **3** types (48 members) and **2** free functions.

Extracted from [`include/shulib/robot/robot.hpp`](../../include/shulib/robot/robot.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct BuildVerdict`](#struct-buildverdict)
  - [`ok`](#buildverdict-ok)
  - [`why`](#buildverdict-why)
- [`struct BuildOptions`](#struct-buildoptions)
  - [`absentDrivePorts`](#buildoptions-absentdriveports)
  - [`pacer`](#buildoptions-pacer)
  - [`isAbsent`](#buildoptions-isabsent)
- [`validate`](#validate) — *free function*
- [`writePortMap`](#writeportmap) — *free function*
- [`class Robot`](#class-robot)
  - [`Robot`](#robot-robot)
  - [`Robot (overload 2)`](#robot-robot-2)
  - [`Robot (overload 3)`](#robot-robot-3)
  - [`operator=`](#robot-operator-eq)
  - [`operator= (overload 2)`](#robot-operator-eq-2)
  - [`~Robot`](#robot-destructor-robot)
  - [`chassis`](#robot-chassis)
  - [`chassis (overload 2)`](#robot-chassis-2)
  - [`localizer`](#robot-localizer)
  - [`localizer (overload 2)`](#robot-localizer-2)
  - [`faults`](#robot-faults)
  - [`faults (overload 2)`](#robot-faults-2)
  - [`health`](#robot-health)
  - [`health (overload 2)`](#robot-health-2)
  - [`context`](#robot-context)
  - [`context (overload 2)`](#robot-context-2)
  - [`config`](#robot-config)
  - [`hasGpsCorrector`](#robot-hasgpscorrector)
  - [`motorGroupCount`](#robot-motorgroupcount)
  - [`driveOdometry`](#robot-driveodometry)
  - [`mechanismCount`](#robot-mechanismcount)
  - [`builtMechanismAt`](#robot-builtmechanismat)
  - [`builtMechanismAt (overload 2)`](#robot-builtmechanismat-2)
  - [`mechanismAt`](#robot-mechanismat)
  - [`mechanismAt (overload 2)`](#robot-mechanismat-2)
  - [`pneumaticMechanismAt`](#robot-pneumaticmechanismat)
  - [`pneumaticMechanismAt (overload 2)`](#robot-pneumaticmechanismat-2)
  - [`mechanismAbsence`](#robot-mechanismabsence)
  - [`mechanism`](#robot-mechanism)
  - [`mechanism (overload 2)`](#robot-mechanism-2)
  - [`pneumaticMechanism`](#robot-pneumaticmechanism)
  - [`pneumaticMechanism (overload 2)`](#robot-pneumaticmechanism-2)
  - [`mechanisms`](#robot-mechanisms)
  - [`mechanismParts`](#robot-mechanismparts)
  - [`camera`](#robot-camera)
  - [`camera (overload 2)`](#robot-camera-2)
  - [`cameraAbsence`](#robot-cameraabsence)
  - [`wallSensors`](#robot-wallsensors)
  - [`wallSensor`](#robot-wallsensor)
  - [`wallSensor (overload 2)`](#robot-wallsensor-2)
  - [`wallSensorAbsence`](#robot-wallsensorabsence)
  - [`portMap`](#robot-portmap)
  - [`describe`](#robot-describe)

<a id="struct-buildverdict"></a>

## `struct BuildVerdict`

```cpp
struct BuildVerdict
```

The answer to "can this robot be built, on this target, right now?" — one bool and one sentence. The sentence is what a person reads off the brain screen when the robot refuses to boot, so it names the PORT or the FIELD and nothing else; 192 bytes is enough for the longest message any check below can produce and small enough to sit on a boot stack.

*struct, declared at [`include/shulib/robot/robot.hpp:223`](../../include/shulib/robot/robot.hpp#L223).*

<a id="buildverdict-ok"></a>

### `BuildVerdict::ok`

```cpp
bool ok = false
```

True when every check passed and `Robot` may be constructed from the same pair.

*field, declared at [`include/shulib/robot/robot.hpp:225`](../../include/shulib/robot/robot.hpp#L225).*

<a id="buildverdict-why"></a>

### `BuildVerdict::why`

```cpp
char why[192]{}
```

The FIRST thing that was wrong, naming the port or the config field. Empty when `ok`.

*field, declared at [`include/shulib/robot/robot.hpp:227`](../../include/shulib/robot/robot.hpp#L227).*

<a id="struct-buildoptions"></a>

## `struct BuildOptions`

```cpp
struct BuildOptions
```

What the builder is told about THIS boot that the config cannot say: which tank drive ports the boot check found dead, and (since COMP-a) a pacer to put between the chassis and the provider's. Default-constructed, it changes nothing — every Part 1 caller's graph is byte-for-byte what it was. The two fields have DIFFERENT lifetimes, and that is the thing to read before passing one: `absentDrivePorts` is read during construction only; `pacer` is kept for the Robot's whole life.

*struct, declared at [`include/shulib/robot/robot.hpp:235`](../../include/shulib/robot/robot.hpp#L235).*

<a id="buildoptions-absentdriveports"></a>

### `BuildOptions::absentDrivePorts`

```cpp
std::span<const int> absentDrivePorts = {}
```

Tank drive ports the boot check reported MISSING, as port MAGNITUDES 1..21 — exactly the numbers `calibration::BootVerdict::isMissing()` answers for. Each becomes an ABSENT (`nullptr`) member of its side's MotorGroup and is never requested from the provider. A repeated entry is harmless (membership, not a count). BORROWED: read during `validate()` and during `Robot`'s construction only, never kept — the Robot recovers what it needs from its own groups.

*field, declared at [`include/shulib/robot/robot.hpp:242`](../../include/shulib/robot/robot.hpp#L242).*

<a id="buildoptions-pacer"></a>

### `BuildOptions::pacer`

```cpp
motion::ITickPacer* pacer = nullptr
```

The pacer the Chassis is built with, or `nullptr` (the default) for `devices.pacer()` — exactly the graph every earlier caller got. A competition program passes its `sequence::RunGuard` here (constructed around `devices.pacer()` BEFORE the Robot), because the guard's deadline reaches a running motion only through the pacer seam (run_guard.hpp's banner) and a Chassis takes its pacer once, at construction.  BORROWED FOR THE ROBOT'S WHOLE LIFE — unlike `absentDrivePorts`, this is NOT read during construction only: the Chassis keeps the reference (chassis.hpp's constructor), so the pacer must outlive the Robot. Declared after `absentDrivePorts` so the designated spelling reads in declaration order: `robot::BuildOptions{.absentDrivePorts = ..., .pacer = &guard}`. `validate()` does not read it (a pacer is not a device and cannot be refused).

*field, declared at [`include/shulib/robot/robot.hpp:255`](../../include/shulib/robot/robot.hpp#L255).*

<a id="buildoptions-isabsent"></a>

### `BuildOptions::isAbsent`

```cpp
[[nodiscard]] constexpr bool isAbsent(int port) const noexcept
```

True when physical port `port` is in `absentDrivePorts`.

*function, declared at [`include/shulib/robot/robot.hpp:258`](../../include/shulib/robot/robot.hpp#L258).*

<a id="validate"></a>

## `validate`

```cpp
[[nodiscard]] inline BuildVerdict validate(const config::RobotConfig& cfg, hal::IDeviceProvider& devices, const BuildOptions& options = {})
```

Can `cfg` be built with `devices`? PURE, TOTAL, and the precondition `Robot`'s constructor runs (ruling B3). Four passes, in this order:  1. **The config's own rules** — `config::RobotConfig::valid()`, whose message (including a tank description's own refusal, verbatim) surfaces unchanged. 2. **The F5 cross-check** — the number of KINEMATIC WHEELS the drivetrain has must equal the number of `hal::IMotor` handles the builder will put in the context. This is the same check `motion::MotionDeps::validate()` makes at the bottom of the stack, and it is the ONLY thing standing between a config typo and an unchecked `std::span::operator[]` on the hot path — so it is made HERE too, before the deps exist, where it can name the config field instead of saying "3 != 4". (It lives in exactly these two places and nowhere else; `RobotConfig::valid()` deliberately does not duplicate it — see its documentation.) 3. **The absent ports** (`options.absentDrivePorts`, G1 Part 2a) — refused, naming the port, when the list is given for a drivetrain that is not a tank (only a tank side is a group that can carry a hole), when a port in it is not one of the description's drive ports, or when it would leave a side with NO present member. Nothing else: the dead-port POLICY was applied by the boot check that produced the list (header, ruling M1). 4. **Every device obtainable** — each drive port that is not absent, the IMU, the GPS if one is configured, and each tracking-wheel encoder is requested from the provider. The first `nullptr` becomes the verdict, carrying the provider's own `lastRefusal()` so the port that is empty or dead is named by the layer that actually looked. An absent port is never asked for, so a dead port is named once — by the boot check — and never twice. MECHANISM motors are deliberately not asked for here (chunk T3): one the provider will not make leaves that mechanism ABSENT when the Robot is built, and never refuses the robot. Nor are a pneumatic mechanism's 3-wire lines (addendum B2): asking for one DRIVES it, and the only refusal the on-robot provider can make — a line outside 1..8 — is pass 1's. Their CONFIG is checked in pass 1 (cartridge, safe brake mode, safe and start levels, ports and lines that collide with the drive or each other).  It is not `const` on the provider because obtaining a device may construct one; it is idempotent, so calling it and then constructing a `Robot` asks for the same objects twice and gets the same addresses both times.

*free function, declared at [`include/shulib/robot/robot.hpp:300`](../../include/shulib/robot/robot.hpp#L300).*

<a id="writeportmap"></a>

## `writePortMap`

```cpp
inline void writePortMap(const config::RobotConfig& cfg, char* out, std::size_t cap) noexcept
```

The §18.5 session header's PORT MAP for `cfg` — the text `diag::SessionInfo::portMap` carries — generated from the config, so it cannot drift from the wiring (the "G1 generates" promise in diag/session_info.hpp). `Robot::portMap()` is this over the Robot's own config; it is a free function as well so the formatting can be tested on a description no valid Robot could carry (a 21-port side).  The formats are the ones src/main.cpp printed by hand, so moving a program onto the builder changed no session header (G1 Part 2b moved "shulib Teleop" — retired at T4 Part 3 — and G1 Part 2c the invented X-drive):  * **tank** — the retired `tankPortMapString()`'s: the description's source word, both sides' SIGNED ports as stored (`%+d`, space-separated, each side capped at 39 characters), the IMU, then — only when configured — `GPS<n>` and the two pods `ROTF<n> ROTL<n>`, then both sides' inches per radian and the track width. For a drive-encoder tank without a GPS (robot two) the text is byte-for-byte what `tankPortMapString()` printed (pinned against a transcription of it in test/robot_builder_test.cpp, and against robot two's 2026-09-25 capture in test/boot_source_test.cpp). * **X-drive** — `portMapString()`'s: `FL BL BR FR` (the F5 wheel order), the two pods, the GPS when present, the IMU. * **H-drive** — the same shape: `L R S` (left line, right line, strafe wheel), pods, GPS, IMU.  Deliberately NOT marked for absent ports: this is what the robot is WIRED to be; a dead port is `describe()`'s and the panel's to announce. Bounded BY HAND — assembled in a generous local buffer and copied into `out` with an explicit bound, never a moving `snprintf` (the on-robot build gate treats `-Wformat-truncation` as an error) — so it truncates rather than overflowing and writes a NUL whenever `cap > 0`. Does not validate: it formats what is stored.

*free function, declared at [`include/shulib/robot/robot.hpp:467`](../../include/shulib/robot/robot.hpp#L467).*

<a id="class-robot"></a>

## `class Robot`

```cpp
class Robot
```

A whole robot: the PROS-free object graph a `config::RobotConfig` describes, owned, with its devices borrowed from an `hal::IDeviceProvider`.  **The provider must outlive the Robot**, and so must everything the provider borrows (on the host: the harness whose clock, battery, sink and pacer it was given). The Robot owns the graph; it owns no hardware, by ruling B2.  Neither copyable nor movable, for the same structural reason `chassis::Chassis` is not: the graph is a web of references into this object's own storage, so a copy or a move would leave half of it pointing at the wrong place. Construct it where it will live.

*class, declared at [`include/shulib/robot/robot.hpp:546`](../../include/shulib/robot/robot.hpp#L546).*

<a id="robot-robot"></a>

### `Robot::Robot`

```cpp
Robot(const config::RobotConfig& cfg, hal::IDeviceProvider& devices, const BuildOptions& options = {})
```

Build the whole graph. PRECONDITION: `validate(cfg, devices, options)` passes — and this runs it, with the SAME options, so a config that cannot be built throws `PreconditionError` carrying the verdict's own sentence, before any member exists. `cfg` is COPIED (a boot sequence that decodes a record into a local and hands out a reference to it is the dangling-reference bug this copy makes impossible); `devices` is borrowed and must outlive the Robot; `options.absentDrivePorts` is read during construction only (its absent ports become `nullptr` group members and are never requested — G1 Part 2a, ruling M1), so that span may die as soon as the constructor returns — but `options.pacer`, when set, is KEPT by the Chassis and must outlive the Robot (COMP-a; `BuildOptions::pacer`). Default options build exactly the graph the two-argument form always built. LAST, after the whole drive graph, each configured mechanism with motors, or with 3-wire lines, is built — or recorded ABSENT with the provider's sentence (chunk T3 and its addendum B2; the file header's MECHANISMS section); then the camera (VIS1-A1), then each declared side distance sensor (L6-A2) — each ABSENT, never refused, when the provider will not make it.

*function, declared at [`include/shulib/robot/robot.hpp:562`](../../include/shulib/robot/robot.hpp#L562).*

<a id="robot-robot-2"></a>

### `Robot::Robot (overload 2)`

```cpp
Robot(const Robot&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage.

*function, declared at [`include/shulib/robot/robot.hpp:630`](../../include/shulib/robot/robot.hpp#L630).*

<a id="robot-robot-3"></a>

### `Robot::Robot (overload 3)`

```cpp
Robot(Robot&&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case.

*function, declared at [`include/shulib/robot/robot.hpp:632`](../../include/shulib/robot/robot.hpp#L632).*

<a id="robot-operator-eq"></a>

### `Robot::operator=`

```cpp
Robot& operator=(const Robot&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/robot/robot.hpp:634`](../../include/shulib/robot/robot.hpp#L634).*

<a id="robot-operator-eq-2"></a>

### `Robot::operator= (overload 2)`

```cpp
Robot& operator=(Robot&&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/robot/robot.hpp:636`](../../include/shulib/robot/robot.hpp#L636).*

<a id="robot-destructor-robot"></a>

### `Robot::~Robot`

```cpp
~Robot() = default
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Destroys the graph. The provider and everything it borrows must still be alive.

*function, declared at [`include/shulib/robot/robot.hpp:638`](../../include/shulib/robot/robot.hpp#L638).*

<a id="robot-chassis"></a>

### `Robot::chassis`

```cpp
[[nodiscard]] chassis::Chassis& chassis() noexcept
```

The public facade every auton is written against — the whole point of the class.

*function, declared at [`include/shulib/robot/robot.hpp:641`](../../include/shulib/robot/robot.hpp#L641).*

<a id="robot-chassis-2"></a>

### `Robot::chassis (overload 2)`

```cpp
[[nodiscard]] const chassis::Chassis& chassis() const noexcept
```

The facade, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:643`](../../include/shulib/robot/robot.hpp#L643).*

<a id="robot-localizer"></a>

### `Robot::localizer`

```cpp
[[nodiscard]] localization::Localizer& localizer() noexcept
```

The fused estimator, for a caller that wants the quality class or a corrector's record rather than just the pose.

*function, declared at [`include/shulib/robot/robot.hpp:647`](../../include/shulib/robot/robot.hpp#L647).*

<a id="robot-localizer-2"></a>

### `Robot::localizer (overload 2)`

```cpp
[[nodiscard]] const localization::Localizer& localizer() const noexcept
```

The estimator, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:649`](../../include/shulib/robot/robot.hpp#L649).*

<a id="robot-faults"></a>

### `Robot::faults`

```cpp
[[nodiscard]] diag::FaultLatch& faults() noexcept
```

The run-scoped fault latch — what the run summary and the boot banner read.

*function, declared at [`include/shulib/robot/robot.hpp:652`](../../include/shulib/robot/robot.hpp#L652).*

<a id="robot-faults-2"></a>

### `Robot::faults (overload 2)`

```cpp
[[nodiscard]] const diag::FaultLatch& faults() const noexcept
```

The fault latch, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:654`](../../include/shulib/robot/robot.hpp#L654).*

<a id="robot-health"></a>

### `Robot::health`

```cpp
[[nodiscard]] diag::HealthMonitor& health() noexcept
```

The health monitor (the A3 pathology → fault policy).

*function, declared at [`include/shulib/robot/robot.hpp:657`](../../include/shulib/robot/robot.hpp#L657).*

<a id="robot-health-2"></a>

### `Robot::health (overload 2)`

```cpp
[[nodiscard]] const diag::HealthMonitor& health() const noexcept
```

The health monitor, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:659`](../../include/shulib/robot/robot.hpp#L659).*

<a id="robot-context"></a>

### `Robot::context`

```cpp
[[nodiscard]] chassis::RobotContext& context() noexcept
```

The one object every layer reads hardware through — the Tier-3 seam for code that wants a device this facade does not expose.

*function, declared at [`include/shulib/robot/robot.hpp:663`](../../include/shulib/robot/robot.hpp#L663).*

<a id="robot-context-2"></a>

### `Robot::context (overload 2)`

```cpp
[[nodiscard]] const chassis::RobotContext& context() const noexcept
```

The context, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:665`](../../include/shulib/robot/robot.hpp#L665).*

<a id="robot-config"></a>

### `Robot::config`

```cpp
[[nodiscard]] const config::RobotConfig& config() const noexcept
```

The config this robot was built from, as stored.

*function, declared at [`include/shulib/robot/robot.hpp:668`](../../include/shulib/robot/robot.hpp#L668).*

<a id="robot-hasgpscorrector"></a>

### `Robot::hasGpsCorrector`

```cpp
[[nodiscard]] bool hasGpsCorrector() const noexcept
```

Whether this robot has a GPS corrector installed — the observable half of "absent is explicit": false means `hal::AbsentGps` is in the context and the Localizer's corrector span is EMPTY, not that a corrector is present and quiet.

*function, declared at [`include/shulib/robot/robot.hpp:673`](../../include/shulib/robot/robot.hpp#L673).*

<a id="robot-motorgroupcount"></a>

### `Robot::motorGroupCount`

```cpp
[[nodiscard]] std::size_t motorGroupCount() const noexcept
```

How many `hal::MotorGroup`s this robot has — 2 on a tank (one per side), 0 on a holonomic drivetrain, whose wheels are single motors.

*function, declared at [`include/shulib/robot/robot.hpp:677`](../../include/shulib/robot/robot.hpp#L677).*

<a id="robot-driveodometry"></a>

### `Robot::driveOdometry`

```cpp
[[nodiscard]] const localization::DriveEncoderOdometry* driveOdometry() const noexcept
```

The drive-encoder odometry this robot runs, for a panel that wants its cross-check (the per-side distances beside the fused estimate) — or `nullptr` when this robot runs tracking wheels instead. Read-only: the Localizer owns the updating.

*function, declared at [`include/shulib/robot/robot.hpp:682`](../../include/shulib/robot/robot.hpp#L682).*

<a id="robot-mechanismcount"></a>

### `Robot::mechanismCount`

```cpp
[[nodiscard]] std::size_t mechanismCount() const noexcept
```

How many mechanisms the config declares, built or not, of both kinds — `config().mechanismCount`. `builtMechanismAt()`, `mechanismAt()`, `pneumaticMechanismAt()` and `mechanismAbsence()` take an index below this.

*function, declared at [`include/shulib/robot/robot.hpp:691`](../../include/shulib/robot/robot.hpp#L691).*

<a id="robot-builtmechanismat"></a>

### `Robot::builtMechanismAt`

```cpp
[[nodiscard]] hal::IMechanism* builtMechanismAt(std::size_t i) noexcept
```

The `i`-th declared mechanism if it was BUILT, of EITHER kind — nullptr when it is ABSENT, has nothing to build, or `i` is not below `mechanismCount()`. This is THE "is this one built?" question: a boot line or a panel row that reports each declared mechanism asks this (and prints its `name()`), never `mechanismAt()`, which is nullptr for a built PNEUMATIC mechanism too and would report a working claw as absent. `mechanismAbsence(i)` is empty exactly when this is non-null.

*function, declared at [`include/shulib/robot/robot.hpp:699`](../../include/shulib/robot/robot.hpp#L699).*

<a id="robot-builtmechanismat-2"></a>

### `Robot::builtMechanismAt (overload 2)`

```cpp
[[nodiscard]] const hal::IMechanism* builtMechanismAt(std::size_t i) const noexcept
```

The `i`-th declared mechanism if it was built, read-only (same rules).

*function, declared at [`include/shulib/robot/robot.hpp:703`](../../include/shulib/robot/robot.hpp#L703).*

<a id="robot-mechanismat"></a>

### `Robot::mechanismAt`

```cpp
[[nodiscard]] hal::MotorMechanism* mechanismAt(std::size_t i) noexcept
```

The `i`-th declared mechanism as a BUILT MOTOR mechanism — or nullptr when it is ABSENT (the provider refused one of its motors), is a pneumatic mechanism (see `pneumaticMechanismAt()`), has nothing to build, or `i` is not below `mechanismCount()`. A TYPED lookup, for code that needs the motor type (a `teleop::MechanismBinding`): null here does NOT mean "absent" — `builtMechanismAt()` answers that.

*function, declared at [`include/shulib/robot/robot.hpp:712`](../../include/shulib/robot/robot.hpp#L712).*

<a id="robot-mechanismat-2"></a>

### `Robot::mechanismAt (overload 2)`

```cpp
[[nodiscard]] const hal::MotorMechanism* mechanismAt(std::size_t i) const noexcept
```

The `i`-th declared mechanism, read-only (same rules).

*function, declared at [`include/shulib/robot/robot.hpp:716`](../../include/shulib/robot/robot.hpp#L716).*

<a id="robot-pneumaticmechanismat"></a>

### `Robot::pneumaticMechanismAt`

```cpp
[[nodiscard]] hal::PneumaticMechanism* pneumaticMechanismAt(std::size_t i) noexcept
```

The `i`-th declared mechanism as a BUILT PNEUMATIC mechanism (addendum B2) — or nullptr when it is ABSENT (the provider refused one of its lines), is a motor mechanism (see `mechanismAt()`), has nothing to build, or `i` is not below `mechanismCount()`. Typed, like `mechanismAt()`: null here does not mean "absent" either.

*function, declared at [`include/shulib/robot/robot.hpp:724`](../../include/shulib/robot/robot.hpp#L724).*

<a id="robot-pneumaticmechanismat-2"></a>

### `Robot::pneumaticMechanismAt (overload 2)`

```cpp
[[nodiscard]] const hal::PneumaticMechanism* pneumaticMechanismAt(std::size_t i) const noexcept
```

The `i`-th declared pneumatic mechanism, read-only (same rules).

*function, declared at [`include/shulib/robot/robot.hpp:728`](../../include/shulib/robot/robot.hpp#L728).*

<a id="robot-mechanismabsence"></a>

### `Robot::mechanismAbsence`

```cpp
[[nodiscard]] const char* mechanismAbsence(std::size_t i) const noexcept
```

Why the `i`-th declared mechanism was NOT built: the provider's own sentence (naming the port or the 3-wire line) for an ABSENT one, a fixed sentence for one with nothing to build, and "no mechanism is declared at this index" for an `i` not below `mechanismCount()`. The empty string EXACTLY when `builtMechanismAt(i)` is non-null — a built mechanism of either kind, and nothing else. Stable for the Robot's lifetime.

*function, declared at [`include/shulib/robot/robot.hpp:737`](../../include/shulib/robot/robot.hpp#L737).*

<a id="robot-mechanism"></a>

### `Robot::mechanism`

```cpp
[[nodiscard]] hal::MotorMechanism* mechanism(const char* name) noexcept
```

The BUILT MOTOR mechanism whose config name is `name` — nullptr when no declared mechanism has that name (or `name` is null), when that one was not built, or when it is a pneumatic mechanism (`pneumaticMechanism()` finds those: the lookup is typed because a binding needs the concrete type, and the library does not downcast). Names are unique: the config's `valid()` refuses two mechanisms with one name.

*function, declared at [`include/shulib/robot/robot.hpp:746`](../../include/shulib/robot/robot.hpp#L746).*

<a id="robot-mechanism-2"></a>

### `Robot::mechanism (overload 2)`

```cpp
[[nodiscard]] const hal::MotorMechanism* mechanism(const char* name) const noexcept
```

The named mechanism, read-only (same rules).

*function, declared at [`include/shulib/robot/robot.hpp:750`](../../include/shulib/robot/robot.hpp#L750).*

<a id="robot-pneumaticmechanism"></a>

### `Robot::pneumaticMechanism`

```cpp
[[nodiscard]] hal::PneumaticMechanism* pneumaticMechanism(const char* name) noexcept
```

The BUILT PNEUMATIC mechanism whose config name is `name` (addendum B2) — what a `teleop::ToggleBinding` is bound to. nullptr when no declared mechanism has that name (or `name` is null), when that one was not built, or when it is a motor mechanism.

*function, declared at [`include/shulib/robot/robot.hpp:757`](../../include/shulib/robot/robot.hpp#L757).*

<a id="robot-pneumaticmechanism-2"></a>

### `Robot::pneumaticMechanism (overload 2)`

```cpp
[[nodiscard]] const hal::PneumaticMechanism* pneumaticMechanism(const char* name) const noexcept
```

The named pneumatic mechanism, read-only (same rules).

*function, declared at [`include/shulib/robot/robot.hpp:761`](../../include/shulib/robot/robot.hpp#L761).*

<a id="robot-mechanisms"></a>

### `Robot::mechanisms`

```cpp
[[nodiscard]] std::span<hal::IMechanism* const> mechanisms() noexcept
```

Every BUILT mechanism of BOTH kinds, in config order, as the `hal::IMechanism*` span an end-of-run guard walks (`sequence::RunGuardConfig::mechanisms`) — an ABSENT one is not in it, because there is nothing there to make safe. Empty on a robot with none. The storage is this Robot's, so the span is valid for the Robot's lifetime.

*function, declared at [`include/shulib/robot/robot.hpp:769`](../../include/shulib/robot/robot.hpp#L769).*

<a id="robot-mechanismparts"></a>

### `Robot::mechanismParts`

```cpp
[[nodiscard]] const MechanismParts& mechanismParts() const noexcept
```

Every role of every mechanism, as this Robot built them (MECH-A1; the file header's MECHANISMS section): `devices()` — what the mechanism service binds over, the SAME objects mechanisms() lists for the guard —, `roles(i)` — each role's boot-line facts —, and the per-index lookups the accessors above forward to. Valid for the Robot's lifetime.

*function, declared at [`include/shulib/robot/robot.hpp:777`](../../include/shulib/robot/robot.hpp#L777).*

<a id="robot-camera"></a>

### `Robot::camera`

```cpp
[[nodiscard]] hal::IVisionFrames* camera() noexcept
```

The robot's AI Vision camera if it was BUILT — nullptr when the config declares none (port 0) or the provider refused it (`cameraAbsence()` says why). Owned by the provider; valid for the Robot's lifetime. Reading frames from it is the consumer's (one read per period, from its own loop); the Robot never reads one.

*function, declared at [`include/shulib/robot/robot.hpp:785`](../../include/shulib/robot/robot.hpp#L785).*

<a id="robot-camera-2"></a>

### `Robot::camera (overload 2)`

```cpp
[[nodiscard]] const hal::IVisionFrames* camera() const noexcept
```

The camera, read-only (its status() is const; reading a frame is not).

*function, declared at [`include/shulib/robot/robot.hpp:787`](../../include/shulib/robot/robot.hpp#L787).*

<a id="robot-cameraabsence"></a>

### `Robot::cameraAbsence`

```cpp
[[nodiscard]] const char* cameraAbsence() const noexcept
```

Why `camera()` is nullptr: "no camera is declared" for port 0, the provider's own sentence naming the port for a refused one ("port 3: nothing plugged in (expected an AI Vision sensor)"), or a fixed sentence when the provider gave none. The empty string EXACTLY when `camera()` is non-null. Stable for the Robot's lifetime.

*function, declared at [`include/shulib/robot/robot.hpp:793`](../../include/shulib/robot/robot.hpp#L793).*

<a id="robot-wallsensors"></a>

### `Robot::wallSensors`

```cpp
[[nodiscard]] const localization::WallSensors& wallSensors() const noexcept
```

Both sides as the wall reset reads them — each side's sensor (nullptr when absent), its read-fault count, its MEASURED mount and why it is absent — the robot's limits and its reset log (seated and refused counts, the last report). The object the context's OPTIONAL `walls` slot points at when the config declares a side (with none declared the slot is nullptr — the reset refuses NO SENSOR — and this reads "not declared" on both sides). Valid for the Robot's lifetime.

*function, declared at [`include/shulib/robot/robot.hpp:803`](../../include/shulib/robot/robot.hpp#L803).*

<a id="robot-wallsensor"></a>

### `Robot::wallSensor`

```cpp
[[nodiscard]] hal::IDistance* wallSensor(localization::WallSide side) noexcept
```

The `side` distance sensor if it was BUILT — nullptr when the config declares none on that side (port 0) or the provider refused it (`wallSensorAbsence(side)` says why). Owned by the provider; valid for the Robot's lifetime. The Robot never reads it.

*function, declared at [`include/shulib/robot/robot.hpp:808`](../../include/shulib/robot/robot.hpp#L808).*

<a id="robot-wallsensor-2"></a>

### `Robot::wallSensor (overload 2)`

```cpp
[[nodiscard]] const hal::IDistance* wallSensor(localization::WallSide side) const noexcept
```

The `side` distance sensor, read-only (its readings are const).

*function, declared at [`include/shulib/robot/robot.hpp:812`](../../include/shulib/robot/robot.hpp#L812).*

<a id="robot-wallsensorabsence"></a>

### `Robot::wallSensorAbsence`

```cpp
[[nodiscard]] const char* wallSensorAbsence(localization::WallSide side) const noexcept
```

Why `wallSensor(side)` is nullptr: "not declared" for port 0, the provider's own sentence for a refused one (the host fake's: "port 5 has no distance sensor"), or a fixed sentence when the provider gave none. The empty string EXACTLY when `wallSensor(side)` is non-null. Stable for the Robot's lifetime.

*function, declared at [`include/shulib/robot/robot.hpp:820`](../../include/shulib/robot/robot.hpp#L820).*

<a id="robot-portmap"></a>

### `Robot::portMap`

```cpp
void portMap(char* out, std::size_t cap) const noexcept
```

The §18.5 session header's port map for this robot, generated from the config it was built with — `writePortMap(config(), out, cap)`, whose documentation gives the formats. Truncates rather than overflowing (the header field is `diag::kMaxPortMapBytes`, 96 bytes); writes a NUL whenever cap > 0. Use this, never `describe()`, for the header: the banner is longer than the field.

*function, declared at [`include/shulib/robot/robot.hpp:830`](../../include/shulib/robot/robot.hpp#L830).*

<a id="robot-describe"></a>

### `Robot::describe`

```cpp
void describe(char* buf, std::size_t cap) const
```

The BANNER LINE: identity, the drivetrain kind and where its numbers came from, both sides' ports AS STORED (signs included — the one place a sign lives), any drive port ABSENT (`DEGRADED: absent 3,13` — only when there is one: built absent at boot, or marked absent mid-run by the driver loop's protections, T4 Part 2), the geometry that governs, the estimator, and each optional device as present or ABSENT — the side distance sensors (L6-A2) after the tags: "walls L4 R5", "walls L4 R5(ABSENT)" for a refused side, " MOUNT UNSET" after them when a declared side's mount is unmeasured, "walls ABSENT" with no side declared — short and fixed-width, so the camera's reason stays the part a short buffer cuts; LAST the camera (VIS1-A1): "camera ABSENT" with none declared, "camera port 3 ABSENT (<why>)" refused, or "camera port 3 " + `hal::cameraStatusWords(camera()->status())`, the adapter's reason after it in parentheses when it has one ("camera port 3 set up, model DIFFERENT (class 0 is 'blueBall', the robot file expects 'redBluePin')") — ASCII only, and LAST so a short buffer cuts the camera's reason, never the drive. Until VIS1-A1 the line ended "vision ABSENT". Truncates rather than overflowing; writes at least a NUL whenever cap > 0.  It is built from the CONFIG this robot was constructed with, not from a second copy of the numbers, so it cannot drift from the wiring — the same property src/main.cpp's retired `portMapString()` had and for the same reason. The absent ports are read back from the two groups' own slots, which were built from the same description in the same order.

*function, declared at [`include/shulib/robot/robot.hpp:851`](../../include/shulib/robot/robot.hpp#L851).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 174 lines, click to expand</summary>

```text

 robot::Robot — a config::RobotConfig, wired (chunk G1 Part 1, 2026-09-18).

 ── What this class is ──────────────────────────────────────────────────────────────
 The composition root, written once, for every robot this library will ever run. It owns
 the whole PROS-free object graph — kinematics, motor groups, geometry, odometry, fusion,
 the GPS corrector when there is a GPS, the Localizer, the fault latch, the health
 monitor, the RobotContext, the MotionDeps and the Chassis — and it BORROWS the devices
 from an hal::IDeviceProvider, which is the one object that knows what a port is on this
 target. Two lines at the composition root:

     hal::pros::ProsDeviceProvider devices;      // the adapters, made on demand, by port
     robot::Robot robot{kRobotConfig, devices};  // the whole graph, wired

 and `robot.chassis()` is the same facade a hand-wired graph produces. The provider on the first
 line exists since G1 Part 2a (hal/pros/device_provider.hpp, host-tested through the PROS shim);
 since G1 Part 2b the programs that drive robot two are built from these two pieces ("shulib
 Teleop" first, which T4 Part 3 retired; "shulib Drive", src/programs/drive_program.cpp, since
 then). On a brain the second line is spelled the way those programs spell it: `robot::validate()`
 first (it names the port or the field before anything is constructed), then
 `std::optional<Robot>::emplace` into storage that lives for the whole program, once, from
 `initialize()` (below: a Robot is constructed where it will live).

 ── IT ADDS A WAY TO WIRE; IT NEVER CHANGES THE PHYSICS (ruling B4) ─────────────────
 The graph below is the SAME objects, in the SAME order, with the SAME arguments as the
 hand-written graphs src/main.cpp carried (robot two's until G1 Part 2b; the invented X-drive's
 until 2c) and as the host rigs. That is not an aspiration:
 it is the acceptance test. test/robot_builder_test.cpp drives a builder-made Chassis and
 a hand-wired one across the A2 plant and requires the per-tick ground-truth samples to be
 memcmp-EQUAL, the commanded volts equal, and the exit reason and tick count equal —
 through a clean and a hostile scenario. If the builder and the longhand disagree by one
 ulp, the builder has changed something, and a rounding note is not an acceptable answer.

 THE BUILDER CONSTRUCTS AND NEVER TICKS. No localizer update, no motion tick, no motor
 write happens anywhere in this file. That is what makes the ordering above a discipline
 rather than a hazard — nothing here has an observable time-ordering, because no time
 passes and no device is written during construction. ONE exception, and it is not a
 command the builder invents (T3's addendum B2): a 3-wire line cannot be MADE without being
 DRIVEN (A4 register HA-119), so a pneumatic mechanism's lines are asked for at the config's
 stated START level, and on a brain that is the moment they take it — last, after the whole
 drive graph (the MECHANISMS section below).

 ── validate() is PURE and TOTAL; construction PRECONDITIONS on it (ruling B3) ──────
 A robot that cannot be built says WHICH PORT OR FIELD before a single object exists. That
 is the R3b refusal-at-boot rule, made library: `validate(cfg, devices)` checks the config
 against its own rules, makes the F5 wheel-count cross-check, and asks the provider for
 every device the config names — returning a verdict with a sentence. The constructor runs
 exactly that check FIRST and throws PreconditionError carrying the same sentence.

 Rejected: a factory returning std::optional<Robot>. Chassis is non-copyable and
 non-movable (its scheduler is pinned by a self-referential command-id stamp), so Robot is
 too, and returning one by value would depend on elision games the V5 toolchain should not
 be asked to play. A Robot is constructed WHERE IT WILL LIVE: on a brain, an EMPTY
 `std::optional<Robot>` that lives for the program, emplaced ONCE from `initialize()` —
 src/programs/drive_program.cpp's pattern. Never a `static Robot r{…}`: at namespace scope it is built
 before `initialize()` has installed the on-robot precondition handler, in an order across
 translation units nobody controls; inside a function, one whose constructor THROWS is
 constructed again on the next call, and PROS calls opcontrol() again after every disable. In a
 test, `std::optional<Robot>::emplace` too.

 ── ABSENT IS NOT NULL ──────────────────────────────────────────────────────────────
 A robot with no GPS gets hal::AbsentGps in its context and NO corrector in its Localizer.
 It never gets a nullptr (RobotContext's preconditions throw on one, and they are right to)
 and it never gets a fake — a test double inside a competition binary erases the difference
 between "no device is installed" and "a test injects here". Tags and the frozen `hal::IVision`
 are ABSENT unconditionally (hal::AbsentTagSource, hal::AbsentVision): no AprilTag adapter exists
 yet (R2b's tag part), and `IVision` has no consumer — the AI Vision camera is read a FRAME at a
 time through `hal::IVisionFrames` instead (the CAMERA section below; VIS1's D11).

 ── ABSENT DRIVE PORTS: the boot check decides, the builder obeys (G1 Part 2a, M1) ──
 A tank drive port the boot check found DEAD (calibration/boot_check.hpp — the registry is read
 BEFORE any adapter exists, and the dead-port policy, teleop::evaluateDegradation, is applied
 there, once) is handed to the builder in `BuildOptions::absentDrivePorts`. Such a port is
 NEVER requested from the provider and becomes a `nullptr` slot in its side's MotorGroup — the
 group's ABSENT member (hal/motor_group.hpp), which it skips when commanding and excludes from
 every read. The builder applies NO policy of its own: re-running the policy over provider
 refusals was the rejected alternative, because it would report a dead port twice, silently
 tolerate a motor that ANSWERS the registry but rejects its configuration, and refuse every
 robot with fewer than three motors a side. So the builder refuses only what is structurally
 wrong with the list — a port that is not one of the description's drive ports, a list on a
 drivetrain that is not a tank, a side left with no present member — and everything the
 provider will not make still refuses the build, as before. With no absent ports (the
 default) every byte of the graph is what it was: Part 1's bit-identity cases are the pin.
 A port that dies AFTER boot is not the builder's either: the driver loop's protections
 (teleop/drive_protection.hpp, T4 Part 2) mark it ABSENT in its group mid-run
 (MotorGroup::markAbsent — out of every read, still commanded), and describe() names it
 from then on; nothing is ever rebuilt. Each group is labelled with its slots' port numbers,
 so those lines name the port.

 ── MECHANISMS: built from the config, ABSENT rather than refused (chunk T3) ────────
 Every `config::MechanismConfig` with motors becomes ONE hal::MotorMechanism, built in the
 constructor's BODY — after the whole drive graph, so the drive's construction order, its
 provider requests and its behaviour are exactly what they were (the bit-identity cases are
 still the pin, and one with mechanisms configured has joined them). Each motor is asked for
 with the MECHANISM's own signed port and cartridge; the mechanism gets its DECLARED safe brake
 mode and its config name (borrowed from this Robot's own copy of the config). validate() does
 NOT ask for mechanism motors: a motor the provider will not make leaves THAT mechanism ABSENT —
 mechanismAt() is nullptr, mechanismAbsence() carries the provider's own sentence naming the
 port — and the rest of the robot builds and drives, because an unplugged claw must not stop a
 robot from driving (the dead-port rule's spirit). A port a mechanism shares with the drive
 never gets this far: RobotConfig::valid() refuses the collision before anything is asked for.
 Since T3's addendum B2, a mechanism with 3-wire lines and no motors becomes ONE
 hal::PneumaticMechanism: each line asked for with the config's START level (making a line
 drives it — the one device write above), the mechanism's DECLARED safe level, its config name;
 pneumaticMechanism(name) finds it, and mechanisms() carries it beside the motor ones. A line the
 provider will not make leaves that mechanism ABSENT, like a motor. A mechanism with both motors
 and lines never gets here (valid() refuses it), and one with neither is reported as not built.
 The lookups by name and index are TYPED (a binding needs the concrete type), so a null from
 one of them does not mean "absent": builtMechanismAt(i) is the kind-neutral "is it built?", and
 mechanismAbsence(i) is empty exactly when it is non-null. A boot or panel line that reports
 each declared mechanism asks those two — never mechanismAt(), which is null for a built claw.
 On a field, a Robot built in initialize() is built while the brain is DISABLED, so whether a
 line's start level reaches the valve then, or only at the enable, is A4 register HA-157's too.
 The builder still never ticks and never commands a motor: a mechanism is first made safe by
 whatever drives it (a teleop::MechanismBinding at the driver-control loop's entry, an
 operation's exit), and a pneumatic one's first level after the start level is the driver's
 (a teleop::ToggleBinding) or an operation's.
 SINCE THE MECHANISM CORE (MECH-A1, 2026-10-08) the building is robot::MechanismParts'
 (robot/mechanism_parts.hpp): the storage and the code that were here moved there, unchanged in
 what they build for every config before MECH-A1, and it builds the SENSOR roles a config now
 states too — an angle sensor (refused = the mechanism ABSENT), a home switch, a piece sensor
 (refused = it runs WITHOUT one) — with one fact per role for the boot lines. The Robot owns one,
 built LAST as before, and every accessor below forwards to it; mechanismParts() hands it out
 whole (its devices() are what the mechanism service binds over — MECH-A2).

 ── THE AI VISION CAMERA: built from the config, ABSENT rather than refused (VIS1-A1) ──
 A config whose `sensors.camera.port` is not 0 gets its camera built LAST, in the constructor's
 body after the mechanisms — so every provider request before it, and so every graph built before
 VIS1-A1, is exactly what it was. `devices.aiVision(port, settings)` writes the camera's settings
 (the provider's banner); validate() does NOT ask for it. A camera the provider will not make — an
 empty port, the OLDER Vision sensor on it, a cable out — leaves `camera()` nullptr and
 `cameraAbsence()` carrying the provider's own sentence, and the robot builds and drives: a dead
 camera must never stop a robot from driving a match (VIS1's D5; a mechanism's refused piece sensor
 is the same rule) — and a provider that THROWS from aiVision() (a breach of its interface) is caught
 and named the same way. `cameraAbsence()` is "" EXACTLY when `camera()` is non-null, "no camera is
 declared" for port 0. The builder never reads a frame (no tick, no poll): the boot line's camera
 words come from `IVisionFrames::status()`, which touches nothing — "camera port 3 set up, model OK"
 — never from a downcast to the PROS adapter. Who reads frames, and how often, is the consumer's: one
 read per period, from its own loop (VIS1's D10; the shared camera feed is vision::CameraFeed,
 vision/camera_feed.hpp). The frozen `IVision` slot in the context stays hal::AbsentVision (D11).

 ── THE SIDE DISTANCE SENSORS: built from the config, ABSENT rather than refused (L6-A2) ──
 For each side of `sensors.walls` whose port is not 0, the constructor's body asks the provider for
 the distance sensor (`devices.distance(port)`) and its read-fault count (`devices.readFaults(port)`)
 — LAST, after the camera, so every provider request before them, and so every graph built before
 L6-A2, is exactly what it was. validate() does NOT ask for them. A sensor the provider will not make
 leaves that side ABSENT — `wallSensor(side)` nullptr, `wallSensorAbsence(side)` the provider's own
 sentence, COPIED (the provider's pointer is valid only until its next refusal) — and the robot
 builds and drives: a cable out must never cost a match (the L6 brief's D5; the camera's rule), and a
 provider that THROWS from distance() is caught and named the same way. `wallSensorAbsence(side)` is
 "" EXACTLY when `wallSensor(side)` is non-null, "not declared" for port 0. Each side carries its
 MEASURED mount from the config (an UNSET one is built all the same; every reset with it refuses
 MOUNT UNSET), and the robot's limits ride beside them. The whole block — both sides, the limits and
 the reset log — is the context's OPTIONAL `walls` slot (chassis/robot_context.hpp), which is how
 the everyday wall reset (competition/auton_wall_reset.hpp) finds them through `chassis.deps().ctx`
 — handed in only when the config declares a side; with none declared the slot is nullptr and the
 reset's word is the true one, NO SENSOR (add the `walls` block). Filling the block in the body is
 safe: the context only stores the pointer, and nothing reads the slot before the first reset. The boot line says which sides exist: `walls L4 R5`, `walls L4 R5(ABSENT)` for a
 refused side (its reason is in wallSensorAbsence() and in the reset's refusal line, never on the
 boot line), ` MOUNT UNSET` after them when a declared mount is unmeasured, `walls ABSENT` when no
 side is declared.

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 A Robot built from a config has driven the A2 host plant, through fakes, and nothing else.
 No config has ever been used on a physical robot. Since T4 Part 3 "shulib Drive" builds its
 graph with this class over the on-robot provider (src/programs/drive_program.cpp), as "shulib
 Teleop" did from G1 Part 2b until T4 Part 3 retired it; THIS version of the program has not run
 on hardware. Since G1 Part 2c the invented X-drive is built the same way
 (src/robots/xdrive_config.hpp, src/programs/xdrive_program.cpp) and no program wires a graph by hand; its ports
 are invented, so it refuses at boot on every robot that exists.

 PROS-free, sim-free, allocation-free in the control path (the graph is one in-place
 aggregate, sized by hal::kMaxSmartPorts — no unique_ptr anywhere, as with every other
 graph in this tree).
```

</details>
