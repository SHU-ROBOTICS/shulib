<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/robot/robot.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `robot.hpp`

robot::Robot — a config::RobotConfig, wired.

This header declares **2** types (22 members) and **1** free function.

Extracted from [`include/shulib/robot/robot.hpp`](../../include/shulib/robot/robot.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct BuildVerdict`](#struct-buildverdict)
  - [`ok`](#buildverdict-ok)
  - [`why`](#buildverdict-why)
- [`validate`](#validate) — *free function*
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
  - [`describe`](#robot-describe)

<a id="struct-buildverdict"></a>

## `struct BuildVerdict`

```cpp
struct BuildVerdict
```

The answer to "can this robot be built, on this target, right now?" — one bool and one sentence. The sentence is what a person reads off the brain screen when the robot refuses to boot, so it names the PORT or the FIELD and nothing else; 192 bytes is enough for the longest message any check below can produce and small enough to sit on a boot stack.

*struct, declared at [`include/shulib/robot/robot.hpp:103`](../../include/shulib/robot/robot.hpp#L103).*

<a id="buildverdict-ok"></a>

### `BuildVerdict::ok`

```cpp
bool ok = false
```

True when every check passed and `Robot` may be constructed from the same pair.

*field, declared at [`include/shulib/robot/robot.hpp:105`](../../include/shulib/robot/robot.hpp#L105).*

<a id="buildverdict-why"></a>

### `BuildVerdict::why`

```cpp
char why[192]{}
```

The FIRST thing that was wrong, naming the port or the config field. Empty when `ok`.

*field, declared at [`include/shulib/robot/robot.hpp:107`](../../include/shulib/robot/robot.hpp#L107).*

<a id="validate"></a>

## `validate`

```cpp
[[nodiscard]] inline BuildVerdict validate(const config::RobotConfig& cfg, hal::IDeviceProvider& devices)
```

Can `cfg` be built with `devices`? PURE, TOTAL, and the precondition `Robot`'s constructor runs (ruling B3). Three passes, in this order:  1. **The config's own rules** — `config::RobotConfig::valid()`, whose message (including a tank description's own refusal, verbatim) surfaces unchanged. 2. **The F5 cross-check** — the number of KINEMATIC WHEELS the drivetrain has must equal the number of `hal::IMotor` handles the builder will put in the context. This is the same check `motion::MotionDeps::validate()` makes at the bottom of the stack, and it is the ONLY thing standing between a config typo and an unchecked `std::span::operator[]` on the hot path — so it is made HERE too, before the deps exist, where it can name the config field instead of saying "3 != 4". (It lives in exactly these two places and nowhere else; `RobotConfig::valid()` deliberately does not duplicate it — see its documentation.) 3. **Every device obtainable** — each drive port, the IMU, the GPS if one is configured, and each tracking-wheel encoder is requested from the provider. The first `nullptr` becomes the verdict, carrying the provider's own `lastRefusal()` so the port that is empty or dead is named by the layer that actually looked.  It is not `const` on the provider because obtaining a device may construct one; it is idempotent, so calling it and then constructing a `Robot` asks for the same objects twice and gets the same addresses both times.

*free function, declared at [`include/shulib/robot/robot.hpp:130`](../../include/shulib/robot/robot.hpp#L130).*

<a id="class-robot"></a>

## `class Robot`

```cpp
class Robot
```

A whole robot: the PROS-free object graph a `config::RobotConfig` describes, owned, with its devices borrowed from an `hal::IDeviceProvider`.  **The provider must outlive the Robot**, and so must everything the provider borrows (on the host: the harness whose clock, battery, sink and pacer it was given). The Robot owns the graph; it owns no hardware, by ruling B2.  Neither copyable nor movable, for the same structural reason `chassis::Chassis` is not: the graph is a web of references into this object's own storage, so a copy or a move would leave half of it pointing at the wrong place. Construct it where it will live.

*class, declared at [`include/shulib/robot/robot.hpp:223`](../../include/shulib/robot/robot.hpp#L223).*

<a id="robot-robot"></a>

### `Robot::Robot`

```cpp
Robot(const config::RobotConfig& cfg, hal::IDeviceProvider& devices)
```

Build the whole graph. PRECONDITION: `validate(cfg, devices)` passes — and this runs it, so a config that cannot be built throws `PreconditionError` carrying the verdict's own sentence, before any member exists. `cfg` is COPIED (a boot sequence that decodes a record into a local and hands out a reference to it is the dangling-reference bug this copy makes impossible); `devices` is borrowed and must outlive the Robot.

*function, declared at [`include/shulib/robot/robot.hpp:230`](../../include/shulib/robot/robot.hpp#L230).*

<a id="robot-robot-2"></a>

### `Robot::Robot (overload 2)`

```cpp
Robot(const Robot&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage.

*function, declared at [`include/shulib/robot/robot.hpp:289`](../../include/shulib/robot/robot.hpp#L289).*

<a id="robot-robot-3"></a>

### `Robot::Robot (overload 3)`

```cpp
Robot(Robot&&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case.

*function, declared at [`include/shulib/robot/robot.hpp:291`](../../include/shulib/robot/robot.hpp#L291).*

<a id="robot-operator-eq"></a>

### `Robot::operator=`

```cpp
Robot& operator=(const Robot&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/robot/robot.hpp:293`](../../include/shulib/robot/robot.hpp#L293).*

<a id="robot-operator-eq-2"></a>

### `Robot::operator= (overload 2)`

```cpp
Robot& operator=(Robot&&) = delete
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/robot/robot.hpp:295`](../../include/shulib/robot/robot.hpp#L295).*

<a id="robot-destructor-robot"></a>

### `Robot::~Robot`

```cpp
~Robot() = default
```

Non-copyable: the graph is a web of references into this object's own storage. Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Destroys the graph. The provider and everything it borrows must still be alive.

*function, declared at [`include/shulib/robot/robot.hpp:297`](../../include/shulib/robot/robot.hpp#L297).*

<a id="robot-chassis"></a>

### `Robot::chassis`

```cpp
[[nodiscard]] chassis::Chassis& chassis() noexcept
```

The public facade every auton is written against — the whole point of the class.

*function, declared at [`include/shulib/robot/robot.hpp:300`](../../include/shulib/robot/robot.hpp#L300).*

<a id="robot-chassis-2"></a>

### `Robot::chassis (overload 2)`

```cpp
[[nodiscard]] const chassis::Chassis& chassis() const noexcept
```

The facade, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:302`](../../include/shulib/robot/robot.hpp#L302).*

<a id="robot-localizer"></a>

### `Robot::localizer`

```cpp
[[nodiscard]] localization::Localizer& localizer() noexcept
```

The fused estimator, for a caller that wants the quality class or a corrector's record rather than just the pose.

*function, declared at [`include/shulib/robot/robot.hpp:306`](../../include/shulib/robot/robot.hpp#L306).*

<a id="robot-localizer-2"></a>

### `Robot::localizer (overload 2)`

```cpp
[[nodiscard]] const localization::Localizer& localizer() const noexcept
```

The estimator, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:308`](../../include/shulib/robot/robot.hpp#L308).*

<a id="robot-faults"></a>

### `Robot::faults`

```cpp
[[nodiscard]] diag::FaultLatch& faults() noexcept
```

The run-scoped fault latch — what the run summary and the boot banner read.

*function, declared at [`include/shulib/robot/robot.hpp:311`](../../include/shulib/robot/robot.hpp#L311).*

<a id="robot-faults-2"></a>

### `Robot::faults (overload 2)`

```cpp
[[nodiscard]] const diag::FaultLatch& faults() const noexcept
```

The fault latch, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:313`](../../include/shulib/robot/robot.hpp#L313).*

<a id="robot-health"></a>

### `Robot::health`

```cpp
[[nodiscard]] diag::HealthMonitor& health() noexcept
```

The health monitor (the A3 pathology → fault policy).

*function, declared at [`include/shulib/robot/robot.hpp:316`](../../include/shulib/robot/robot.hpp#L316).*

<a id="robot-health-2"></a>

### `Robot::health (overload 2)`

```cpp
[[nodiscard]] const diag::HealthMonitor& health() const noexcept
```

The health monitor, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:318`](../../include/shulib/robot/robot.hpp#L318).*

<a id="robot-context"></a>

### `Robot::context`

```cpp
[[nodiscard]] chassis::RobotContext& context() noexcept
```

The one object every layer reads hardware through — the Tier-3 seam for code that wants a device this facade does not expose.

*function, declared at [`include/shulib/robot/robot.hpp:322`](../../include/shulib/robot/robot.hpp#L322).*

<a id="robot-context-2"></a>

### `Robot::context (overload 2)`

```cpp
[[nodiscard]] const chassis::RobotContext& context() const noexcept
```

The context, read-only.

*function, declared at [`include/shulib/robot/robot.hpp:324`](../../include/shulib/robot/robot.hpp#L324).*

<a id="robot-config"></a>

### `Robot::config`

```cpp
[[nodiscard]] const config::RobotConfig& config() const noexcept
```

The config this robot was built from, as stored.

*function, declared at [`include/shulib/robot/robot.hpp:327`](../../include/shulib/robot/robot.hpp#L327).*

<a id="robot-hasgpscorrector"></a>

### `Robot::hasGpsCorrector`

```cpp
[[nodiscard]] bool hasGpsCorrector() const noexcept
```

Whether this robot has a GPS corrector installed — the observable half of "absent is explicit": false means `hal::AbsentGps` is in the context and the Localizer's corrector span is EMPTY, not that a corrector is present and quiet.

*function, declared at [`include/shulib/robot/robot.hpp:332`](../../include/shulib/robot/robot.hpp#L332).*

<a id="robot-motorgroupcount"></a>

### `Robot::motorGroupCount`

```cpp
[[nodiscard]] std::size_t motorGroupCount() const noexcept
```

How many `hal::MotorGroup`s this robot has — 2 on a tank (one per side), 0 on a holonomic drivetrain, whose wheels are single motors.

*function, declared at [`include/shulib/robot/robot.hpp:336`](../../include/shulib/robot/robot.hpp#L336).*

<a id="robot-describe"></a>

### `Robot::describe`

```cpp
void describe(char* buf, std::size_t cap) const
```

The BANNER LINE: identity, the drivetrain kind and where its numbers came from, both sides' ports AS STORED (signs included — the one place a sign lives), the geometry that governs, the estimator, and each optional device as present or ABSENT. Truncates rather than overflowing; writes at least a NUL whenever cap > 0.  It is built from the CONFIG this robot was constructed with, not from a second copy of the numbers, so it cannot drift from the wiring — the same property src/main.cpp's `portMapString()` has and for the same reason.

*function, declared at [`include/shulib/robot/robot.hpp:346`](../../include/shulib/robot/robot.hpp#L346).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 59 lines, click to expand</summary>

```text

 robot::Robot — a config::RobotConfig, wired (chunk G1 Part 1, 2026-09-18).

 ── What this class is ──────────────────────────────────────────────────────────────
 The composition root, written once, for every robot this library will ever run. It owns
 the whole PROS-free object graph — kinematics, motor groups, geometry, odometry, fusion,
 the GPS corrector when there is a GPS, the Localizer, the fault latch, the health
 monitor, the RobotContext, the MotionDeps and the Chassis — and it BORROWS the devices
 from an hal::IDeviceProvider, which is the one object that knows what a port is on this
 target. Two lines at the composition root:

     ProsDeviceProvider devices;                 // the adapters, made on demand  (Part 2)
     robot::Robot robot{kRobotConfig, devices};  // the whole graph, wired

 and `robot.chassis()` is the same facade a hand-wired graph produces.

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

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 A Robot built from a config has driven the A2 host plant, through fakes, and nothing else.
 No config has ever been used on a physical robot; src/main.cpp is unchanged and still
 wires both of its graphs by hand — collapsing it is Part 2.

 PROS-free, sim-free, allocation-free in the control path (the graph is one in-place
 aggregate, sized by hal::kMaxSmartPorts — no unique_ptr anywhere, as with every other
 graph in this tree).
```

</details>
