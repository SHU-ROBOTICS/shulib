<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/boot_check.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `boot_check.hpp`

THE BOOT CHECK — is this description still true of THIS robot?

This header declares **7** types (34 members) and **5** free functions.

Extracted from [`include/shulib/calibration/boot_check.hpp`](../../include/shulib/calibration/boot_check.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class PortClass`](#enum-class-portclass)
  - [`Present`](#portclass-present)
  - [`Missing`](#portclass-missing)
  - [`Swapped`](#portclass-swapped)
- [`portClassWord`](#portclassword) — *free function*
- [`enum class BootDecision`](#enum-class-bootdecision)
  - [`Ok`](#bootdecision-ok)
  - [`Degraded`](#bootdecision-degraded)
  - [`Refuse`](#bootdecision-refuse)
- [`bootDecisionWord`](#bootdecisionword) — *free function*
- [`struct BootPortStatus`](#struct-bootportstatus)
  - [`signedPort`](#bootportstatus-signedport)
  - [`port`](#bootportstatus-port)
  - [`side`](#bootportstatus-side)
  - [`state`](#bootportstatus-state)
  - [`found`](#bootportstatus-found)
- [`struct BootVerdict`](#struct-bootverdict)
  - [`decision`](#bootverdict-decision)
  - [`ports`](#bootverdict-ports)
  - [`portCount`](#bootverdict-portcount)
  - [`missingCount`](#bootverdict-missingcount)
  - [`swappedCount`](#bootverdict-swappedcount)
  - [`newPorts`](#bootverdict-newports)
  - [`newCount`](#bootverdict-newcount)
  - [`declaredPorts`](#bootverdict-declaredports)
  - [`declaredCount`](#bootverdict-declaredcount)
  - [`imuPort`](#bootverdict-imuport)
  - [`imuFound`](#bootverdict-imufound)
  - [`imuOk`](#bootverdict-imuok)
  - [`degradation`](#bootverdict-degradation)
  - [`reason`](#bootverdict-reason)
  - [`usable`](#bootverdict-usable)
  - [`isMissing`](#bootverdict-ismissing)
- [`struct DeclaredMotorPorts`](#struct-declaredmotorports)
  - [`ports`](#declaredmotorports-ports)
  - [`label`](#declaredmotorports-label)
- [`verifyAgainstRegistry`](#verifyagainstregistry) — *free function*
- [`verifyAgainstRegistry (overload 2)`](#verifyagainstregistry-2) — *free function*
- [`enum class BootSource`](#enum-class-bootsource)
  - [`UseRecord`](#bootsource-userecord)
  - [`UseTable`](#bootsource-usetable)
  - [`Refuse`](#bootsource-refuse)
- [`struct BootSourceChoice`](#struct-bootsourcechoice)
  - [`source`](#bootsourcechoice-source)
  - [`why`](#bootsourcechoice-why)
- [`chooseBootSource`](#choosebootsource) — *free function*

<a id="enum-class-portclass"></a>

## `enum class PortClass`

```cpp
enum class PortClass : std::uint8_t
```

What the registry says about ONE port the description lists as a drive port.

*enum class, declared at [`include/shulib/calibration/boot_check.hpp:71`](../../include/shulib/calibration/boot_check.hpp#L71).*

<a id="portclass-present"></a>

### `PortClass::Present`

```cpp
Present
```

a MOTOR is there, as the description says

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:72`](../../include/shulib/calibration/boot_check.hpp#L72).*

<a id="portclass-missing"></a>

### `PortClass::Missing`

```cpp
Missing
```

the port is EMPTY — a dead cable or a dead motor (policy decides)

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:73`](../../include/shulib/calibration/boot_check.hpp#L73).*

<a id="portclass-swapped"></a>

### `PortClass::Swapped`

```cpp
Swapped
```

the port carries a DIFFERENT KIND of device — a rewired robot (always refuses)

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:74`](../../include/shulib/calibration/boot_check.hpp#L74).*

<a id="portclassword"></a>

## `portClassWord`

```cpp
[[nodiscard]] inline const char* portClassWord(PortClass c) noexcept
```

The class as one word, for the panel and the refusal line.

*free function, declared at [`include/shulib/calibration/boot_check.hpp:78`](../../include/shulib/calibration/boot_check.hpp#L78).*

<a id="enum-class-bootdecision"></a>

## `enum class BootDecision`

```cpp
enum class BootDecision : std::uint8_t
```

What the boot may do with this description on this robot.

*enum class, declared at [`include/shulib/calibration/boot_check.hpp:88`](../../include/shulib/calibration/boot_check.hpp#L88).*

<a id="bootdecision-ok"></a>

### `BootDecision::Ok`

```cpp
Ok
```

every listed port carries its motor and the IMU is where it should be

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:89`](../../include/shulib/calibration/boot_check.hpp#L89).*

<a id="bootdecision-degraded"></a>

### `BootDecision::Degraded`

```cpp
Degraded
```

some ports are MISSING, but teleop::evaluateDegradation still says drive

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:90`](../../include/shulib/calibration/boot_check.hpp#L90).*

<a id="bootdecision-refuse"></a>

### `BootDecision::Refuse`

```cpp
Refuse
```

a SWAPPED port, a bad IMU port, an unusable census, or the policy says no

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:91`](../../include/shulib/calibration/boot_check.hpp#L91).*

<a id="bootdecisionword"></a>

## `bootDecisionWord`

```cpp
[[nodiscard]] inline const char* bootDecisionWord(BootDecision d) noexcept
```

The decision as one word, for a banner.

*free function, declared at [`include/shulib/calibration/boot_check.hpp:95`](../../include/shulib/calibration/boot_check.hpp#L95).*

<a id="struct-bootportstatus"></a>

## `struct BootPortStatus`

```cpp
struct BootPortStatus
```

One row of the comparison: what the description said about a port, and what the brain says.

*struct, declared at [`include/shulib/calibration/boot_check.hpp:105`](../../include/shulib/calibration/boot_check.hpp#L105).*

<a id="bootportstatus-signedport"></a>

### `BootPortStatus::signedPort`

```cpp
std::int8_t signedPort = 0
```

The description's entry AS STORED — signed, because that is what the adapter gets.

*field, declared at [`include/shulib/calibration/boot_check.hpp:107`](../../include/shulib/calibration/boot_check.hpp#L107).*

<a id="bootportstatus-port"></a>

### `BootPortStatus::port`

```cpp
std::uint8_t port = 0
```

|signedPort| — the physical port a person reads off the brain.

*field, declared at [`include/shulib/calibration/boot_check.hpp:109`](../../include/shulib/calibration/boot_check.hpp#L109).*

<a id="bootportstatus-side"></a>

### `BootPortStatus::side`

```cpp
std::int8_t side = 0
```

−1 LEFT, +1 RIGHT (the description's sides).

*field, declared at [`include/shulib/calibration/boot_check.hpp:111`](../../include/shulib/calibration/boot_check.hpp#L111).*

<a id="bootportstatus-state"></a>

### `BootPortStatus::state`

```cpp
PortClass state = PortClass::Present
```

What the census made of it.

*field, declared at [`include/shulib/calibration/boot_check.hpp:113`](../../include/shulib/calibration/boot_check.hpp#L113).*

<a id="bootportstatus-found"></a>

### `BootPortStatus::found`

```cpp
hal::DeviceKind found = hal::DeviceKind::None
```

What the registry actually answered — so a SWAPPED row can say WHAT is there now.

*field, declared at [`include/shulib/calibration/boot_check.hpp:115`](../../include/shulib/calibration/boot_check.hpp#L115).*

<a id="struct-bootverdict"></a>

## `struct BootVerdict`

```cpp
struct BootVerdict
```

The whole verdict: the decision, every port's row, the new motors, the IMU's own row, the degradation verdict the decision was taken from, and one bounded line of prose naming every port in every class. Fixed width and allocation-free — a composition root builds one on the stack before any adapter exists.

*struct, declared at [`include/shulib/calibration/boot_check.hpp:122`](../../include/shulib/calibration/boot_check.hpp#L122).*

<a id="bootverdict-decision"></a>

### `BootVerdict::decision`

```cpp
BootDecision decision = BootDecision::Refuse
```

What the boot may do. Defaults to Refuse: a verdict nobody computed must never read "Ok".

*field, declared at [`include/shulib/calibration/boot_check.hpp:124`](../../include/shulib/calibration/boot_check.hpp#L124).*

<a id="bootverdict-ports"></a>

### `BootVerdict::ports`

```cpp
BootPortStatus ports[hal::kMaxSmartPorts]{}
```

One row per drive port the description lists, LEFT side first then RIGHT, in the description's own order.

*field, declared at [`include/shulib/calibration/boot_check.hpp:127`](../../include/shulib/calibration/boot_check.hpp#L127).*

<a id="bootverdict-portcount"></a>

### `BootVerdict::portCount`

```cpp
std::uint8_t portCount = 0
```

How many rows of `ports` are real.

*field, declared at [`include/shulib/calibration/boot_check.hpp:129`](../../include/shulib/calibration/boot_check.hpp#L129).*

<a id="bootverdict-missingcount"></a>

### `BootVerdict::missingCount`

```cpp
std::uint8_t missingCount = 0
```

How many of them are MISSING.

*field, declared at [`include/shulib/calibration/boot_check.hpp:131`](../../include/shulib/calibration/boot_check.hpp#L131).*

<a id="bootverdict-swappedcount"></a>

### `BootVerdict::swappedCount`

```cpp
std::uint8_t swappedCount = 0
```

How many of them are SWAPPED.

*field, declared at [`include/shulib/calibration/boot_check.hpp:133`](../../include/shulib/calibration/boot_check.hpp#L133).*

<a id="bootverdict-newports"></a>

### `BootVerdict::newPorts`

```cpp
std::uint8_t newPorts[hal::kMaxSmartPorts]{}
```

Physical ports carrying a MOTOR that the description does not list — always a warning. A port the caller DECLARED as a mechanism motor is not here: it is in `declaredPorts`.

*field, declared at [`include/shulib/calibration/boot_check.hpp:136`](../../include/shulib/calibration/boot_check.hpp#L136).*

<a id="bootverdict-newcount"></a>

### `BootVerdict::newCount`

```cpp
std::uint8_t newCount = 0
```

How many entries of `newPorts` are real.

*field, declared at [`include/shulib/calibration/boot_check.hpp:138`](../../include/shulib/calibration/boot_check.hpp#L138).*

<a id="bootverdict-declaredports"></a>

### `BootVerdict::declaredPorts`

```cpp
std::uint8_t declaredPorts[hal::kMaxSmartPorts]{}
```

Physical ports carrying a MOTOR that the description does not list AND the caller declared as a mechanism motor (`DeclaredMotorPorts` — a claw pivot, T3 Part 2): expected, named with the caller's label, never counted as NEW. Never changes the decision.

*field, declared at [`include/shulib/calibration/boot_check.hpp:142`](../../include/shulib/calibration/boot_check.hpp#L142).*

<a id="bootverdict-declaredcount"></a>

### `BootVerdict::declaredCount`

```cpp
std::uint8_t declaredCount = 0
```

How many entries of `declaredPorts` are real.

*field, declared at [`include/shulib/calibration/boot_check.hpp:144`](../../include/shulib/calibration/boot_check.hpp#L144).*

<a id="bootverdict-imuport"></a>

### `BootVerdict::imuPort`

```cpp
std::uint8_t imuPort = 0
```

The description's IMU port.

*field, declared at [`include/shulib/calibration/boot_check.hpp:146`](../../include/shulib/calibration/boot_check.hpp#L146).*

<a id="bootverdict-imufound"></a>

### `BootVerdict::imuFound`

```cpp
hal::DeviceKind imuFound = hal::DeviceKind::None
```

What the registry says is on the IMU port.

*field, declared at [`include/shulib/calibration/boot_check.hpp:148`](../../include/shulib/calibration/boot_check.hpp#L148).*

<a id="bootverdict-imuok"></a>

### `BootVerdict::imuOk`

```cpp
bool imuOk = false
```

True when the IMU port carries an IMU.

*field, declared at [`include/shulib/calibration/boot_check.hpp:150`](../../include/shulib/calibration/boot_check.hpp#L150).*

<a id="bootverdict-degradation"></a>

### `BootVerdict::degradation`

```cpp
teleop::DegradationVerdict degradation{}
```

The dead-port policy's own verdict over the PRESENT/MISSING counts — one policy, two programs (teleop/drivetrain_degradation.hpp).

*field, declared at [`include/shulib/calibration/boot_check.hpp:153`](../../include/shulib/calibration/boot_check.hpp#L153).*

<a id="bootverdict-reason"></a>

### `BootVerdict::reason`

```cpp
char reason[320]{}
```

One line naming every port in every class, for the screen and the log. Bounded and NUL-terminated; truncates rather than overflowing. 320 bytes because the Ok line carries the honest limit as well as the counts, and the host build compiles this header under -Wformat-truncation: a buffer that COULD truncate a literal is a compile error here, which is the right place to find out rather than on a brain's screen. ASCII only, deliberately -- the V5's screen font and the USB serial log both render a UTF-8 em dash as mojibake, and every other on-screen string in src/ uses "--".

*field, declared at [`include/shulib/calibration/boot_check.hpp:161`](../../include/shulib/calibration/boot_check.hpp#L161).*

<a id="bootverdict-usable"></a>

### `BootVerdict::usable`

```cpp
[[nodiscard]] bool usable() const noexcept
```

True when this verdict permits building a drivetrain at all (Ok or Degraded).

*function, declared at [`include/shulib/calibration/boot_check.hpp:164`](../../include/shulib/calibration/boot_check.hpp#L164).*

<a id="bootverdict-ismissing"></a>

### `BootVerdict::isMissing`

```cpp
[[nodiscard]] bool isMissing(int port) const noexcept
```

True when physical `port` is one this verdict classed MISSING — the question the composition root asks per port when it decides which group members are ABSENT.

*function, declared at [`include/shulib/calibration/boot_check.hpp:168`](../../include/shulib/calibration/boot_check.hpp#L168).*

<a id="struct-declaredmotorports"></a>

## `struct DeclaredMotorPorts`

```cpp
struct DeclaredMotorPorts
```

Motor ports the PROGRAM declares as something other than drive motors — robot two's claw pivots on 6 and 7 (T3 Part 2) — and the words the verdict calls them by. A registry MOTOR on one of these ports that the description does not list is EXPECTED: it is named in its own clause (`<label>: 6 7`) instead of as a NEW motor, so a boot line never reports a declared claw as an unknown motor. Borrowed for the call. The default is none, which is what the two-argument `verifyAgainstRegistry()` passes.

*struct, declared at [`include/shulib/calibration/boot_check.hpp:184`](../../include/shulib/calibration/boot_check.hpp#L184).*

<a id="declaredmotorports-ports"></a>

### `DeclaredMotorPorts::ports`

```cpp
std::span<const int> ports{}
```

Physical ports 1..21 (magnitudes); a port outside that range matches nothing.

*field, declared at [`include/shulib/calibration/boot_check.hpp:186`](../../include/shulib/calibration/boot_check.hpp#L186).*

<a id="declaredmotorports-label"></a>

### `DeclaredMotorPorts::label`

```cpp
const char* label = "declared mechanism motors"
```

What the verdict calls them, e.g. "claw pivots (declared, not drive motors)". Printed with a precision of 48 characters; ASCII only (it reaches a screen). Null prints nothing.

*field, declared at [`include/shulib/calibration/boot_check.hpp:189`](../../include/shulib/calibration/boot_check.hpp#L189).*

<a id="verifyagainstregistry"></a>

## `verifyAgainstRegistry`

```cpp
[[nodiscard]] inline BootVerdict verifyAgainstRegistry(const hal::DrivetrainDescription& desc, const hal::IDeviceRegistry& registry, const DeclaredMotorPorts& declared)
```

Compare `desc` against what `registry` says is plugged in, WITHOUT constructing anything.  The decision, in this order (each earlier one wins, and `reason` names it): 1. the description is not `valid()` — Refuse (defensive: every caller checks first, and a refusal here is still better than a graph built from a contradictory description); 2. any SWAPPED port — Refuse, naming each one and what is there instead (ruling P2); 3. the IMU port missing or carrying something else — Refuse, naming what is there; 4. whatever `teleop::evaluateDegradation` says about the PRESENT counts — Refuse, Degraded (listing the missing ports) or Ok. One policy, shared with "shulib Drive". NEW motors are always listed, and never change the decision. A motor on a port in `declared` that the description does not list is not NEW: it is listed after them under the caller's label, and never changes the decision either. With no declared port (or none carrying a motor) the prose is exactly the two-argument form's.  The honest limit is in the header and is repeated in `reason` on every non-Ok verdict: two drive cables swapped WITH EACH OTHER are invisible to a port census and are caught at the first drive by the coupled-side monitor.

*free function, declared at [`include/shulib/calibration/boot_check.hpp:209`](../../include/shulib/calibration/boot_check.hpp#L209).*

<a id="verifyagainstregistry-2"></a>

## `verifyAgainstRegistry (overload 2)`

```cpp
[[nodiscard]] inline BootVerdict verifyAgainstRegistry(const hal::DrivetrainDescription& desc, const hal::IDeviceRegistry& registry)
```

`verifyAgainstRegistry` with NO declared mechanism motors — every motor the description does not list is NEW (the R3d Part 2 form; its prose unchanged).

*free function, declared at [`include/shulib/calibration/boot_check.hpp:376`](../../include/shulib/calibration/boot_check.hpp#L376).*

<a id="enum-class-bootsource"></a>

## `enum class BootSource`

```cpp
enum class BootSource : std::uint8_t
```

Which of the two description sources a boot should use.

*enum class, declared at [`include/shulib/calibration/boot_check.hpp:382`](../../include/shulib/calibration/boot_check.hpp#L382).*

<a id="bootsource-userecord"></a>

### `BootSource::UseRecord`

```cpp
UseRecord
```

the card's calibration record — measured on THIS robot

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:383`](../../include/shulib/calibration/boot_check.hpp#L383).*

<a id="bootsource-usetable"></a>

### `BootSource::UseTable`

```cpp
UseTable
```

src/robots/chassis_table.hpp's typed table — a human's tape measure

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:384`](../../include/shulib/calibration/boot_check.hpp#L384).*

<a id="bootsource-refuse"></a>

### `BootSource::Refuse`

```cpp
Refuse
```

neither can be trusted; nothing is constructed

*enumerator, declared at [`include/shulib/calibration/boot_check.hpp:385`](../../include/shulib/calibration/boot_check.hpp#L385).*

<a id="struct-bootsourcechoice"></a>

## `struct BootSourceChoice`

```cpp
struct BootSourceChoice
```

The chosen source with the sentence that explains it. `why` is always a string literal, so the struct is copyable, allocation-free and safe to keep.

*struct, declared at [`include/shulib/calibration/boot_check.hpp:390`](../../include/shulib/calibration/boot_check.hpp#L390).*

<a id="bootsourcechoice-source"></a>

### `BootSourceChoice::source`

```cpp
BootSource source = BootSource::Refuse
```

What to build the drivetrain from.

*field, declared at [`include/shulib/calibration/boot_check.hpp:392`](../../include/shulib/calibration/boot_check.hpp#L392).*

<a id="bootsourcechoice-why"></a>

### `BootSourceChoice::why`

```cpp
const char* why = ""
```

One sentence for the banner and the refusal screen — never empty.

*field, declared at [`include/shulib/calibration/boot_check.hpp:394`](../../include/shulib/calibration/boot_check.hpp#L394).*

<a id="choosebootsource"></a>

## `chooseBootSource`

```cpp
[[nodiscard]] inline constexpr BootSourceChoice chooseBootSource(bool recordDecoded, BootDecision recordVerdict, bool tableValid) noexcept
```

THE BOOT DECISION TABLE, as data rather than as an `if` chain nobody tested (ruling P3).  `recordDecoded` — a record was on the card and passed every format check. `recordVerdict` — what verifyAgainstRegistry() made of it (meaningless when !recordDecoded). `tableValid`    — `tableToDescription(kChassis).valid()`.  | record   | verdict  | table   | → | |----------|----------|---------|---| | decoded  | Ok       | any     | UseRecord | | decoded  | Degraded | any     | UseRecord (with dead ports named by the verdict) | | decoded  | Refuse   | any     | **Refuse** — NO fallback to the table | | absent   | —        | valid   | UseTable, with a warning that it is unverified | | absent   | —        | invalid | Refuse, naming BOTH reasons |  The row that carries the ruling is the third. A record that FAILED its port check means the robot was rewired since the calibration — and the typed table is OLDER than the record, so falling back to it would answer "this robot changed" with "then use the more out-of-date description". Rejected alternatives: falling back to the table on a refused record (above); preferring a VALID table over a DEGRADED record (a table is typed from a build that may no longer exist; a degraded record was measured on this robot and merely lost a motor); a screen chooser (the team lead's ruling against choosers inside programs, R3b Part 0b).

*free function, declared at [`include/shulib/calibration/boot_check.hpp:418`](../../include/shulib/calibration/boot_check.hpp#L418).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 55 lines, click to expand</summary>

```text

 THE BOOT CHECK — is this description still true of THIS robot? (chunk R3d Part 2)

 ── Why a description is not enough ─────────────────────────────────────────────────
 A calibration record is a claim about a robot made on some earlier day. Between then and
 now: a cable can be re-seated into the next port, a motor can die, a card can be carried
 from robot one to robot two, a sensor can be swapped for a different kind. None of those
 is visible from inside the record — the CRC will be perfect, `valid()` will pass, and the
 description will describe a robot that no longer exists. So before "shulib Teleop"
 constructs a single adapter, it asks the brain what is actually plugged in and compares.

 BEFORE ANY ADAPTER EXISTS, deliberately (ruling P2). Constructing ten ProsMotors to find
 out which ones answer is how HA-111's invented port map turned into ten throwing
 constructors at boot; the registry read is free and happens first.

 ── The four classes, and what each one means ───────────────────────────────────────
   PRESENT  the port carries a MOTOR, as the description says. Nothing to report.
   MISSING  the port is EMPTY. A dead cable, a dead motor, a port bank that browned out.
            This is TOLERATED by policy, not by this file: teleop::evaluateDegradation
            (the team lead's rule — "1–2 dead ports shouldn't stop driving") decides, so
            the drive program and the library program cannot disagree about it.
   SWAPPED  the port carries a DIFFERENT KIND of device. This always REFUSES (ruling P2).
            A port that lost its motor may be a dead cable; a port that now carries a
            rotation sensor is a REWIRED ROBOT, and a rewired robot needs a new
            calibration, not a best effort.
   NEW      a motor the registry sees on a port the description does not list. Always a
            WARNING, never a refusal: an extra motor is not a drivetrain fault (an
            intake, a lift), but a drive motor added since the calibration would sit
            uncommanded and fighting its side-mates, so nobody gets to not be told.
            A motor on a port the PROGRAM declares as a mechanism motor (a claw pivot —
            `DeclaredMotorPorts`, T3 Part 2) is not NEW: it is expected, and named in its
            own clause with the program's words, never as an unknown motor.
 The IMU port must carry an IMU. Missing or wrong kind ⇒ REFUSE: every number in the
 description was measured against an IMU, and without one nothing can be re-checked.

 ═══ THE HONEST LIMIT — READ THIS, AND SAY IT ON THE SCREEN ═════════════════════════
 TWO DRIVE CABLES SWAPPED WITH EACH OTHER ARE INVISIBLE HERE. Both ports still carry
 motors, so both classify PRESENT, and the verdict is Ok. The description then drives port
 12's motor with port 13's sign — two members of one side fighting each other on the first
 command. The same is true of a RE-GEARING that keeps the ports: the registry cannot see a
 gear.

 That is not a hole to be patched here; it is the boundary of what a port census can know.
 What catches it is the FIRST DRIVE: teleop::CoupledSideMonitor sees a member opposite its
 side's command and raises MOTOR_GROUP_DISAGREE within 250 ms, and the panel names the
 port. A limit nobody has pinned is a limit somebody will forget, so
 test/boot_check_test.cpp asserts it — two drive ports exchanged ⇒ Ok — and the refusal
 screen's wording tells the operator what the check did NOT check.

 ── Pure, allocation-free, testable against every row ──────────────────────────────
 Numbers and an interface in, a verdict out; no clock, no state, no adapters. Everything
 here is exercised through hal::fake::FakeDeviceRegistry, which is how "a port that now
 carries a rotation sensor" becomes a test case instead of an argument.

 NO HARDWARE CLAIM: no verdict in this file has ever been computed on a robot.
```

</details>
