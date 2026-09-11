<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/drivetrain_degradation.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drivetrain_degradation.hpp`

Drivetrain degradation policy — what a drive program does when some of the motors it was built for do not answer.

This header declares **3** types (10 members), **1** free function, and **2** constants.

Extracted from [`include/shulib/teleop/drivetrain_degradation.hpp`](../../include/shulib/teleop/drivetrain_degradation.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMinAnsweringPerSide`](#kminansweringperside) — *constant*
- [`kMaxDeadTotal`](#kmaxdeadtotal) — *constant*
- [`struct SideCount`](#struct-sidecount)
  - [`expected`](#sidecount-expected)
  - [`answering`](#sidecount-answering)
- [`enum class DriveVerdict`](#enum-class-driveverdict)
  - [`Drive`](#driveverdict-drive)
  - [`DriveDegraded`](#driveverdict-drivedegraded)
  - [`Refuse`](#driveverdict-refuse)
- [`struct DegradationVerdict`](#struct-degradationverdict)
  - [`verdict`](#degradationverdict-verdict)
  - [`leftDead`](#degradationverdict-leftdead)
  - [`rightDead`](#degradationverdict-rightdead)
  - [`totalDead`](#degradationverdict-totaldead)
  - [`reason`](#degradationverdict-reason)
- [`evaluateDegradation`](#evaluatedegradation) — *free function*

<a id="kminansweringperside"></a>

## `kMinAnsweringPerSide`

```cpp
inline constexpr int kMinAnsweringPerSide = 3
```

The per-side floor: a side with fewer answering motors than this cannot pull its share (header: two of five drags the robot into a curve; three drives).

*constant, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:46`](../../include/shulib/teleop/drivetrain_degradation.hpp#L46).*

<a id="kmaxdeadtotal"></a>

## `kMaxDeadTotal`

```cpp
inline constexpr int kMaxDeadTotal = 2
```

The whole-drivetrain cap: more dead motors than this, anywhere, is a systemic problem (harness, port bank, battery), not a port, and the drivetrain is refused.

*constant, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:50`](../../include/shulib/teleop/drivetrain_degradation.hpp#L50).*

<a id="struct-sidecount"></a>

## `struct SideCount`

```cpp
struct SideCount
```

One side's census: how many motors the chassis table lists for it, and how many of those actually answered (adapter constructed, and not since marked ABSENT).

*struct, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:54`](../../include/shulib/teleop/drivetrain_degradation.hpp#L54).*

<a id="sidecount-expected"></a>

### `SideCount::expected`

```cpp
int expected = 0
```

Motors the chassis table lists on this side.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:55`](../../include/shulib/teleop/drivetrain_degradation.hpp#L55).*

<a id="sidecount-answering"></a>

### `SideCount::answering`

```cpp
int answering = 0
```

Of those, the ones that answer right now (0 <= answering <= expected).

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:56`](../../include/shulib/teleop/drivetrain_degradation.hpp#L56).*

<a id="enum-class-driveverdict"></a>

## `enum class DriveVerdict`

```cpp
enum class DriveVerdict
```

What the drive program is allowed to do with the motors it has.

*enum class, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:60`](../../include/shulib/teleop/drivetrain_degradation.hpp#L60).*

<a id="driveverdict-drive"></a>

### `DriveVerdict::Drive`

```cpp
Drive
```

Every listed motor answers: drive normally.

*enumerator, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:61`](../../include/shulib/teleop/drivetrain_degradation.hpp#L61).*

<a id="driveverdict-drivedegraded"></a>

### `DriveVerdict::DriveDegraded`

```cpp
DriveDegraded
```

One or two dead, every side still at the floor or above: drive, warn, log.

*enumerator, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:62`](../../include/shulib/teleop/drivetrain_degradation.hpp#L62).*

<a id="driveverdict-refuse"></a>

### `DriveVerdict::Refuse`

```cpp
Refuse
```

A side below the floor, the total over the cap, or nonsense counts: 0 V.

*enumerator, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:63`](../../include/shulib/teleop/drivetrain_degradation.hpp#L63).*

<a id="struct-degradationverdict"></a>

## `struct DegradationVerdict`

```cpp
struct DegradationVerdict
```

The verdict with the counts behind it, so the panel, the LCD and the log can say WHICH side is down and by how much rather than just "degraded".

*struct, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:68`](../../include/shulib/teleop/drivetrain_degradation.hpp#L68).*

<a id="degradationverdict-verdict"></a>

### `DegradationVerdict::verdict`

```cpp
DriveVerdict verdict = DriveVerdict::Refuse
```

The decision.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:69`](../../include/shulib/teleop/drivetrain_degradation.hpp#L69).*

<a id="degradationverdict-leftdead"></a>

### `DegradationVerdict::leftDead`

```cpp
int leftDead = 0
```

expected − answering on the left.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:70`](../../include/shulib/teleop/drivetrain_degradation.hpp#L70).*

<a id="degradationverdict-rightdead"></a>

### `DegradationVerdict::rightDead`

```cpp
int rightDead = 0
```

expected − answering on the right.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:71`](../../include/shulib/teleop/drivetrain_degradation.hpp#L71).*

<a id="degradationverdict-totaldead"></a>

### `DegradationVerdict::totalDead`

```cpp
int totalDead = 0
```

leftDead + rightDead.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:72`](../../include/shulib/teleop/drivetrain_degradation.hpp#L72).*

<a id="degradationverdict-reason"></a>

### `DegradationVerdict::reason`

```cpp
const char* reason = ""
```

One short phrase for the log; "" when Drive.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:73`](../../include/shulib/teleop/drivetrain_degradation.hpp#L73).*

<a id="evaluatedegradation"></a>

## `evaluateDegradation`

```cpp
[[nodiscard]] constexpr DegradationVerdict evaluateDegradation(SideCount left, SideCount right) noexcept
```

The policy: refuse iff any side answers with fewer than kMinAnsweringPerSide, or the total dead exceeds kMaxDeadTotal; DriveDegraded when anything is dead but neither limit is hit; Drive when nothing is. Counts that make no sense — a negative, or more answering than expected, or an empty side — are REFUSED with a reason, because the safe direction for a nonsense census is 0 V, never "drive anyway".

*free function, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:81`](../../include/shulib/teleop/drivetrain_degradation.hpp#L81).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 39 lines</summary>

```text

 Drivetrain degradation policy — what a drive program does when some of the
 motors it was built for do not answer (chunk R3b Part 0b, 2026-09-10).

 WHY THIS EXISTS: the team lead's instruction for the drive program was "some
 ports might die during comp; 1–2 dead ports shouldn't stop driving, just let
 it be a warning that gets logged." A dead port is therefore NOT a fault that
 ends the program (the tester's rule, right for diagnosis) and NOT a silent
 zero (the worst outcome — a side quietly down a motor and nobody told). It is
 a WARNING with a policy behind it, and the policy is this pure function so it
 can be host-tested against every boundary row rather than argued about on
 the field.

 THE RULE (brief §3, decided): refuse to drive only if ANY side has fewer than
 three answering motors, or the TOTAL dead exceeds two. Otherwise drive —
 degraded, with the counts shown.

 THE REASONING, stated so the numbers are not magic: robot two has five
 coupled motors per side on one gear train, four wheels per side. Two motors
 of five is a side that cannot pull its share — under load it lags, and the
 robot drags into a curve at speed that the driver cannot steer out of; that
 side is also carrying the mechanical drag of three unpowered rotors. Three of
 five drives: down 40 % of the side's torque, straight enough at reduced
 speed, and the driver is told. The total cap of two is the second guard: two
 dead on ONE side is 3/5 (drives); one dead on each is 4/5 + 4/5 (drives);
 three dead anywhere is a drivetrain with a systemic problem — a cable
 harness, a brain port bank, a battery sag — not a port, and driving it is
 the wrong answer even where the per-side floor would pass.

 DOMAIN, stated honestly: the floor is written for a five-per-side train and
 the bench bot's four-per-side (3/4 drives, 2/4 refuses). A drivetrain with
 fewer than three motors per side is outside this policy's domain — it would
 be refused at every boot — and that is deliberate: nobody has thought about
 such a robot here, and a policy that silently generalised to it would be an
 invented answer. Widen it, with a reason, when such a robot exists.

 PURE and PROS-free: numbers in, a verdict out, no state, no clock. The
 caller (src/drive_program.cpp) counts the motors whose adapter constructed —
 and re-counts when a member goes ABSENT at runtime — and acts on the verdict.
```

</details>
