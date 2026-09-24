<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/robot/robot.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `robot.hpp`

robot::Robot — a config::RobotConfig, wired.

This header declares **3** types (26 members) and **2** free functions.

Extracted from [`include/shulib/robot/robot.hpp`](../../include/shulib/robot/robot.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct BuildVerdict`](#struct-buildverdict)
  - [`ok`](#buildverdict-ok)
  - [`why`](#buildverdict-why)
- [`struct BuildOptions`](#struct-buildoptions)
  - [`absentDrivePorts`](#buildoptions-absentdriveports)
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
  - [`portMap`](#robot-portmap)
  - [`describe`](#robot-describe)

<a id="struct-buildverdict"></a>

## `struct BuildVerdict`

```cpp
struct BuildVerdict
```

The answer to "can this robot be built, on this target, right now?" — one bool and one sentence. The sentence is what a person reads off the brain screen when the robot refuses to boot, so it names the PORT or the FIELD and nothing else; 192 bytes is enough for the longest message any check below can produce and small enough to sit on a boot stack.

*struct, declared at [`include/shulib/robot/robot.hpp:122`](../../include/shulib/robot/robot.hpp#L122).*

<a id="buildverdict-ok"></a>

### `BuildVerdict::ok`

```cpp
bool ok = false
```

True when every check passed and `Robot` may be constructed from the same pair.

*field, declared at [`include/shulib/robot/robot.hpp:124`](../../include/shulib/robot/robot.hpp#L124).*

<a id="buildverdict-why"></a>

### `BuildVerdict::why`

```cpp
char why[192]{}
```

The FIRST thing that was wrong, naming the port or the config field. Empty when `ok`.

*field, declared at [`include/shulib/robot/robot.hpp:126`](../../include/shulib/robot/robot.hpp#L126).*

<a id="struct-buildoptions"></a>

## `struct BuildOptions`

```cpp
struct BuildOptions
```

What the builder is told about THIS boot that the config cannot say. Today one thing: which tank drive ports the boot check found dead. Default-constructed, it changes nothing — every Part 1 caller's graph is byte-for-byte what it was.

*struct, declared at [`include/shulib/robot/robot.hpp:132`](../../include/shulib/robot/robot.hpp#L132).*

<a id="buildoptions-absentdriveports"></a>

### `BuildOptions::absentDrivePorts`

```cpp
std::span<const int> absentDrivePorts = {}
```

Tank drive ports the boot check reported MISSING, as port MAGNITUDES 1..21 — exactly the numbers `calibration::BootVerdict::isMissing()` answers for. Each becomes an ABSENT (`nullptr`) member of its side's MotorGroup and is never requested from the provider. A repeated entry is harmless (membership, not a count). BORROWED: read during `validate()` and during `Robot`'s construction only, never kept — the Robot recovers what it needs from its own groups.

*field, declared at [`include/shulib/robot/robot.hpp:139`](../../include/shulib/robot/robot.hpp#L139).*

<a id="buildoptions-isabsent"></a>

### `BuildOptions::isAbsent`

```cpp
[[nodiscard]] constexpr bool isAbsent(int port) const noexcept
```

True when physical port `port` is in `absentDrivePorts`.

*function, declared at [`include/shulib/robot/robot.hpp:142`](../../include/shulib/robot/robot.hpp#L142).*

<a id="validate"></a>

## `validate`

```cpp
[[nodiscard]] inline BuildVerdict validate(const config::RobotConfig& cfg, hal::IDeviceProvider& devices, const BuildOptions& options = {})
```

Can `cfg` be built with `devices`? PURE, TOTAL, and the precondition `Robot`'s constructor runs (ruling B3). Four passes, in this order:  1. **The config's own rules** — `config::RobotConfig::valid()`, whose message (including a tank description's own refusal, verbatim) surfaces unchanged. 2. **The F5 cross-check** — the number of KINEMATIC WHEELS the drivetrain has must equal the number of `hal::IMotor` handles the builder will put in the context. This is the same check `motion::MotionDeps::validate()` makes at the bottom of the stack, and it is the ONLY thing standing between a config typo and an unchecked `std::span::operator[]` on the hot path — so it is made HERE too, before the deps exist, where it can name the config field instead of saying "3 != 4". (It lives in exactly these two places and nowhere else; `RobotConfig::valid()` deliberately does not duplicate it — see its documentation.) 3. **The absent ports** (`options.absentDrivePorts`, G1 Part 2a) — refused, naming the port, when the list is given for a drivetrain that is not a tank (only a tank side is a group that can carry a hole), when a port in it is not one of the description's drive ports, or when it would leave a side with NO present member. Nothing else: the dead-port POLICY was applied by the boot check that produced the list (header, ruling M1). 4. **Every device obtainable** — each drive port that is not absent, the IMU, the GPS if one is configured, and each tracking-wheel encoder is requested from the provider. The first `nullptr` becomes the verdict, carrying the provider's own `lastRefusal()` so the port that is empty or dead is named by the layer that actually looked. An absent port is never asked for, so a dead port is named once — by the boot check — and never twice.  It is not `const` on the provider because obtaining a device may construct one; it is idempotent, so calling it and then constructing a `Robot` asks for the same objects twice and gets the same addresses both times.

*free function, declared at [`include/shulib/robot/robot.hpp:178`](../../include/shulib/robot/robot.hpp#L178).*

<a id="writeportmap"></a>

## `writePortMap`

```cpp
inline void writePortMap(const config::RobotConfig& cfg, char* out, std::size_t cap) noexcept
```

The §18.5 session header's PORT MAP for `cfg` — the text `diag::SessionInfo::portMap` carries — generated from the config, so it cannot drift from the wiring (the "G1 generates" promise in diag/session_info.hpp). `Robot::portMap()` is this over the Robot's own config; it is a free function as well so the formatting can be tested on a description no valid Robot could carry (a 21-port side).  The formats are the ones src/main.cpp prints by hand today, so moving a program onto the builder changes no session header:  * **tank** — `tankPortMapString()`'s: the description's source word, both sides' SIGNED ports as stored (`%+d`, space-separated, each side capped at 39 characters), the IMU, then — only when configured — `GPS<n>` and the two pods `ROTF<n> ROTL<n>`, then both sides' inches per radian and the track width. For a drive-encoder tank without a GPS (robot two) the text is byte-for-byte `tankPortMapString()`'s. * **X-drive** — `portMapString()`'s: `FL BL BR FR` (the F5 wheel order), the two pods, the GPS when present, the IMU. * **H-drive** — the same shape: `L R S` (left line, right line, strafe wheel), pods, GPS, IMU.  Deliberately NOT marked for absent ports: this is what the robot is WIRED to be; a dead port is `describe()`'s and the panel's to announce. Bounded BY HAND — assembled in a generous local buffer and copied into `out` with an explicit bound, never a moving `snprintf` (the on-robot build gate treats `-Wformat-truncation` as an error) — so it truncates rather than overflowing and writes a NUL whenever `cap > 0`. Does not validate: it formats what is stored.

*free function, declared at [`include/shulib/robot/robot.hpp:342`](../../include/shulib/robot/robot.hpp#L342).*

<a id="class-robot"></a>

## `class Robot`

```cpp
class Robot
```

A whole robot: the PROS-free object graph a `config::RobotConfig` describes, owned, with its devices borrowed from an `hal::IDeviceProvider`.  **The provider must outlive the Robot**, and so must everything the provider borrows (on the host: the harness whose clock, battery, sink and pacer it was given). The Robot owns the graph; it owns no hardware, by ruling B2.  Neither copyable nor movable, for the same structural reason `chassis::Chassis` is not: the graph is a web of references into this object's own storage, so a copy or a move would leave half of it pointing at the wrong place. Construct it where it will live.

*class, declared at [`include/shulib/robot/robot.hpp:420`](../../include/shulib/robot/robot.hpp#L420).*

<a id="robot-robot"></a>

### `Robot::Robot`

```cpp
Robot(const config::RobotConfig& cfg, hal::IDeviceProvider& devices, const BuildOptions& options = {})
```

Build the whole graph. PRECONDITION: `validate(cfg, devices, options)` passes — and this runs it, with the SAME options, so a config that cannot be built throws `PreconditionError` carrying the verdict's own sentence, before any member exists. `cfg` is COPIED (a boot sequence that decodes a record into a local and hands out a reference to it is the dangling-reference bug this copy makes impossible); `devices` is borrowed and must outlive the Robot; `options` is read during construction only (its absent ports become `nullptr` group members and are never requested — G1 Part 2a, ruling M1), so its span may die as soon as the constructor returns. Default options build exactly the graph the two-argument form always built.

*function, declared at [`include/shulib/robot/robot.hpp:431`](../../include/shulib/robot/robot.hpp#L431).*

<a id="robot-robot-2"></a>

### `Robot::Robot (overload 2)`

```cpp
Robot(const Robot&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage.

*function, declared at [`include/shulib/robot/robot.hpp:491`](../../include/shulib/robot/robot.hpp#L491).*

<a id="robot-robot-3"></a>

### `Robot::Robot (overload 3)`

```cpp
Robot(Robot&&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case.

*function, declared at [`include/shulib/robot/robot.hpp:493`](../../include/shulib/robot/robot.hpp#L493).*

<a id="robot-operator-eq"></a>

### `Robot::operator=`

```cpp
Robot& operator=(const Robot&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/robot/robot.hpp:495`](../../include/shulib/robot/robot.hpp#L495).*

<a id="robot-operator-eq-2"></a>

### `Robot::operator= (overload 2)`

```cpp
Robot& operator=(Robot&&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/robot/robot.hpp:497`](../../include/shulib/robot/robot.hpp#L497).*

<a id="robot-destructor-robot"></a>

### `Robot::~Robot`

```cpp
~Robot() = default
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Destroys the graph. The provider and everything it borrows must still be alive.

*function, declared at [`include/shulib/robot/robot.hpp:499`](../../include/shulib/robot/robot.hpp#L499).*

<a id="robot-chassis"></a>

### `Robot::chassis`

```cpp
[[nodiscard]] chassis::Chassis& chassis() noexcept
```

The public facade every auton is written against — the whole point of the class.

*function, declared at [`include/shulib/robot/robot.hpp:502`](../../include/shulib/robot/robot.hpp#L502).*

<a id="robot-chassis-2"></a>

### `Robot::chassis (overload 2)`

```cpp
[[nodiscard]] const chassis::Chassis& chassis() const noexcept
```

The facade, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:504`](../../include/shulib/robot/robot.hpp#L504).*

<a id="robot-localizer"></a>

### `Robot::localizer`

```cpp
[[nodiscard]] localization::Localizer& localizer() noexcept
```

The fused estimator, for a caller that wants the quality class or a corrector's record rather than just the pose.

*function, declared at [`include/shulib/robot/robot.hpp:508`](../../include/shulib/robot/robot.hpp#L508).*

<a id="robot-localizer-2"></a>

### `Robot::localizer (overload 2)`

```cpp
[[nodiscard]] const localization::Localizer& localizer() const noexcept
```

The estimator, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:510`](../../include/shulib/robot/robot.hpp#L510).*

<a id="robot-faults"></a>

### `Robot::faults`

```cpp
[[nodiscard]] diag::FaultLatch& faults() noexcept
```

The run-scoped fault latch — what the run summary and the boot banner read.

*function, declared at [`include/shulib/robot/robot.hpp:513`](../../include/shulib/robot/robot.hpp#L513).*

<a id="robot-faults-2"></a>

### `Robot::faults (overload 2)`

```cpp
[[nodiscard]] const diag::FaultLatch& faults() const noexcept
```

The fault latch, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:515`](../../include/shulib/robot/robot.hpp#L515).*

<a id="robot-health"></a>

### `Robot::health`

```cpp
[[nodiscard]] diag::HealthMonitor& health() noexcept
```

The health monitor (the A3 pathology → fault policy).

*function, declared at [`include/shulib/robot/robot.hpp:518`](../../include/shulib/robot/robot.hpp#L518).*

<a id="robot-health-2"></a>

### `Robot::health (overload 2)`

```cpp
[[nodiscard]] const diag::HealthMonitor& health() const noexcept
```

The health monitor, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:520`](../../include/shulib/robot/robot.hpp#L520).*

<a id="robot-context"></a>

### `Robot::context`

```cpp
[[nodiscard]] chassis::RobotContext& context() noexcept
```

The one object every layer reads hardware through — the Tier-3 seam for code that wants a device this facade does not expose.

*function, declared at [`include/shulib/robot/robot.hpp:524`](../../include/shulib/robot/robot.hpp#L524).*

<a id="robot-context-2"></a>

### `Robot::context (overload 2)`

```cpp
[[nodiscard]] const chassis::RobotContext& context() const noexcept
```

The context, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:526`](../../include/shulib/robot/robot.hpp#L526).*

<a id="robot-config"></a>

### `Robot::config`

```cpp
[[nodiscard]] const config::RobotConfig& config() const noexcept
```

The config this robot was built from, as stored.

*function, declared at [`include/shulib/robot/robot.hpp:529`](../../include/shulib/robot/robot.hpp#L529).*

<a id="robot-hasgpscorrector"></a>

### `Robot::hasGpsCorrector`

```cpp
[[nodiscard]] bool hasGpsCorrector() const noexcept
```

Whether this robot has a GPS corrector installed — the observable half of "absent is explicit": false means `hal::AbsentGps` is in the context and the Localizer's corrector span is EMPTY, not that a corrector is present and quiet.

*function, declared at [`include/shulib/robot/robot.hpp:534`](../../include/shulib/robot/robot.hpp#L534).*

<a id="robot-motorgroupcount"></a>

### `Robot::motorGroupCount`

```cpp
[[nodiscard]] std::size_t motorGroupCount() const noexcept
```

How many `hal::MotorGroup`s this robot has — 2 on a tank (one per side), 0 on a holonomic drivetrain, whose wheels are single motors.

*function, declared at [`include/shulib/robot/robot.hpp:538`](../../include/shulib/robot/robot.hpp#L538).*

<a id="robot-driveodometry"></a>

### `Robot::driveOdometry`

```cpp
[[nodiscard]] const localization::DriveEncoderOdometry* driveOdometry() const noexcept
```

The drive-encoder odometry this robot runs, for a panel that wants its cross-check (the per-side distances beside the fused estimate) — or `nullptr` when this robot runs tracking wheels instead. Read-only: the Localizer owns the updating.

*function, declared at [`include/shulib/robot/robot.hpp:543`](../../include/shulib/robot/robot.hpp#L543).*

<a id="robot-portmap"></a>

### `Robot::portMap`

```cpp
void portMap(char* out, std::size_t cap) const noexcept
```

The §18.5 session header's port map for this robot, generated from the config it was built with — `writePortMap(config(), out, cap)`, whose documentation gives the formats. Truncates rather than overflowing (the header field is `diag::kMaxPortMapBytes`, 96 bytes); writes a NUL whenever cap > 0. Use this, never `describe()`, for the header: the banner is longer than the field.

*function, declared at [`include/shulib/robot/robot.hpp:552`](../../include/shulib/robot/robot.hpp#L552).*

<a id="robot-describe"></a>

### `Robot::describe`

```cpp
void describe(char* buf, std::size_t cap) const
```

The BANNER LINE: identity, the drivetrain kind and where its numbers came from, both sides' ports AS STORED (signs included — the one place a sign lives), any drive port built ABSENT (`DEGRADED: absent 3,13` — only when there is one), the geometry that governs, the estimator, and each optional device as present or ABSENT. Truncates rather than overflowing; writes at least a NUL whenever cap > 0.  It is built from the CONFIG this robot was constructed with, not from a second copy of the numbers, so it cannot drift from the wiring — the same property src/main.cpp's `portMapString()` has and for the same reason. The absent ports are read back from the two groups' own slots, which were built from the same description in the same order.

*function, declared at [`include/shulib/robot/robot.hpp:564`](../../include/shulib/robot/robot.hpp#L564).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 78 lines, click to expand</summary>

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

 and `robot.chassis()` is the same facade a hand-wired graph produces. The provider on the
 first line exists since G1 Part 2a (hal/pros/device_provider.hpp, host-tested through the
 PROS shim); no program in src/ uses either line yet — "shulib Teleop" moves onto them in
 Part 2b.

 ── IT ADDS A WAY TO WIRE; IT NEVER CHANGES THE PHYSICS (ruling B4) ─────────────────
 The graph below is the SAME objects, in the SAME order, with the SAME arguments as
 src/main.cpp's two hand-written graphs and as the host rigs. That is not an aspiration:
 it is the acceptance test. test/robot_builder_test.cpp drives a builder-made Chassis and
 a hand-wired one across the A2 plant and requires the per-tick ground-truth samples to be
 memcmp-EQUAL, the commanded volts equal, and the exit reason and tick count equal —
 through a clean and a hostile scenario. If the builder and the longhand disagree by one
 ulp, the builder has changed something, and a rounding note is not an acceptable answer.

 THE BUILDER CONSTRUCTS AND NEVER TICKS. No localizer update, no motion tick, no motor
 write happens anywhere in this file. That is what makes the ordering above a discipline
 rather than a hazard — nothing here has an observable time-ordering, because no time
 passes and no device is written during construction.

 ── validate() is PURE and TOTAL; construction PRECONDITIONS on it (ruling B3) ──────
 A robot that cannot be built says WHICH PORT OR FIELD before a single object exists. That
 is the R3b refusal-at-boot rule, made library: `validate(cfg, devices)` checks the config
 against its own rules, makes the F5 wheel-count cross-check, and asks the provider for
 every device the config names — returning a verdict with a sentence. The constructor runs
 exactly that check FIRST and throws PreconditionError carrying the same sentence.

 Rejected: a factory returning std::optional<Robot>. Chassis is non-copyable and
 non-movable (its scheduler is pinned by a self-referential command-id stamp), so Robot is
 too, and returning one by value would depend on elision games the V5 toolchain should not
 be asked to play. A Robot is constructed WHERE IT WILL LIVE: `static Robot r{…}` in a
 main, `std::optional<Robot>::emplace` in a test.

 ── ABSENT IS NOT NULL ──────────────────────────────────────────────────────────────
 A robot with no GPS gets hal::AbsentGps in its context and NO corrector in its Localizer.
 It never gets a nullptr (RobotContext's preconditions throw on one, and they are right to)
 and it never gets a fake — a test double inside a competition binary erases the difference
 between "no device is installed" and "a test injects here". Tags and vision are ABSENT
 unconditionally in Part 1, because no camera adapter exists yet (R2 owns it) and a field
 for hardware the library cannot open would be a promise rather than a description.

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

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 A Robot built from a config has driven the A2 host plant, through fakes, and nothing else.
 No config has ever been used on a physical robot. src/main.cpp is unchanged and still wires
 both of its graphs by hand: the on-robot provider exists (Part 2a) but no program uses it
 yet — "shulib Teleop" moves onto it in Part 2b, and main.cpp becomes a dispatcher in 2c.

 PROS-free, sim-free, allocation-free in the control path (the graph is one in-place
 aggregate, sized by hal::kMaxSmartPorts — no unique_ptr anywhere, as with every other
 graph in this tree).
```

</details>
