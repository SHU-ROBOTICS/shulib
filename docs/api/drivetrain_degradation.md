<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/drivetrain_degradation.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drivetrain_degradation.hpp`

Drivetrain degradation policy — what a drive program does when some of the motors it was built for do not answer.

This header declares **3** types (10 members), **2** free functions, and **2** constants.

Extracted from [`include/shulib/teleop/drivetrain_degradation.hpp`](../../include/shulib/teleop/drivetrain_degradation.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

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
- [`evaluateRuntimeDegradation`](#evaluateruntimedegradation) — *free function*

<a id="kminansweringperside"></a>

## `kMinAnsweringPerSide`

```cpp
inline constexpr int kMinAnsweringPerSide = 3
```

The per-side floor: a side with fewer answering motors than this cannot pull its share (header: reasoned for five a side — two of five drags the robot into a curve, three drives; on robot two's four a side, 2/4 is refused). Applied at BOOT only by the library (evaluateDegradation); mid-run only the total cap applies (evaluateRuntimeDegradation).

*constant, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:65`](../../include/shulib/teleop/drivetrain_degradation.hpp#L65).*

<a id="kmaxdeadtotal"></a>

## `kMaxDeadTotal`

```cpp
inline constexpr int kMaxDeadTotal = 2
```

The whole-drivetrain cap: more dead motors than this, anywhere, is a systemic problem (harness, port bank, battery), not a port, and the drivetrain is refused.

*constant, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:69`](../../include/shulib/teleop/drivetrain_degradation.hpp#L69).*

<a id="struct-sidecount"></a>

## `struct SideCount`

```cpp
struct SideCount
```

One side's census: how many motors the drivetrain description lists for it, and how many of those actually answer (present at boot, and not since marked ABSENT).

*struct, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:73`](../../include/shulib/teleop/drivetrain_degradation.hpp#L73).*

<a id="sidecount-expected"></a>

### `SideCount::expected`

```cpp
int expected = 0
```

Motors listed on this side (the description's ports).

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:74`](../../include/shulib/teleop/drivetrain_degradation.hpp#L74).*

<a id="sidecount-answering"></a>

### `SideCount::answering`

```cpp
int answering = 0
```

Of those, the ones that answer right now (0 <= answering <= expected).

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:75`](../../include/shulib/teleop/drivetrain_degradation.hpp#L75).*

<a id="enum-class-driveverdict"></a>

## `enum class DriveVerdict`

```cpp
enum class DriveVerdict
```

What a drive is allowed to do with the motors it has — at boot (evaluateDegradation) or mid-run (evaluateRuntimeDegradation).

*enum class, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:80`](../../include/shulib/teleop/drivetrain_degradation.hpp#L80).*

<a id="driveverdict-drive"></a>

### `DriveVerdict::Drive`

```cpp
Drive
```

Every listed motor answers: drive normally.

*enumerator, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:81`](../../include/shulib/teleop/drivetrain_degradation.hpp#L81).*

<a id="driveverdict-drivedegraded"></a>

### `DriveVerdict::DriveDegraded`

```cpp
DriveDegraded
```

One or two dead: drive, warn, log. At boot every side is also still at the floor or above; mid-run only the total is judged, so a side below the floor drives on.

*enumerator, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:82`](../../include/shulib/teleop/drivetrain_degradation.hpp#L82).*

<a id="driveverdict-refuse"></a>

### `DriveVerdict::Refuse`

```cpp
Refuse
```

0 V. At boot: a side below the floor, the total over the cap, or nonsense counts. Mid-run: the total over the cap, or nonsense counts.

*enumerator, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:85`](../../include/shulib/teleop/drivetrain_degradation.hpp#L85).*

<a id="struct-degradationverdict"></a>

## `struct DegradationVerdict`

```cpp
struct DegradationVerdict
```

The verdict with the counts behind it, so the panel, the LCD and the log can say WHICH side is down and by how much rather than just "degraded".

*struct, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:91`](../../include/shulib/teleop/drivetrain_degradation.hpp#L91).*

<a id="degradationverdict-verdict"></a>

### `DegradationVerdict::verdict`

```cpp
DriveVerdict verdict = DriveVerdict::Refuse
```

The decision.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:92`](../../include/shulib/teleop/drivetrain_degradation.hpp#L92).*

<a id="degradationverdict-leftdead"></a>

### `DegradationVerdict::leftDead`

```cpp
int leftDead = 0
```

expected − answering on the left.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:93`](../../include/shulib/teleop/drivetrain_degradation.hpp#L93).*

<a id="degradationverdict-rightdead"></a>

### `DegradationVerdict::rightDead`

```cpp
int rightDead = 0
```

expected − answering on the right.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:94`](../../include/shulib/teleop/drivetrain_degradation.hpp#L94).*

<a id="degradationverdict-totaldead"></a>

### `DegradationVerdict::totalDead`

```cpp
int totalDead = 0
```

leftDead + rightDead.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:95`](../../include/shulib/teleop/drivetrain_degradation.hpp#L95).*

<a id="degradationverdict-reason"></a>

### `DegradationVerdict::reason`

```cpp
const char* reason = ""
```

One short phrase for the log; "" when Drive.

*field, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:96`](../../include/shulib/teleop/drivetrain_degradation.hpp#L96).*

<a id="evaluatedegradation"></a>

## `evaluateDegradation`

```cpp
[[nodiscard]] constexpr DegradationVerdict evaluateDegradation(SideCount left, SideCount right) noexcept
```

The policy: refuse iff any side answers with fewer than kMinAnsweringPerSide, or the total dead exceeds kMaxDeadTotal; DriveDegraded when anything is dead but neither limit is hit; Drive when nothing is. Counts that make no sense — a negative, or more answering than expected, or an empty side — are REFUSED with a reason, because the safe direction for a nonsense census is 0 V, never "drive anyway".

*free function, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:104`](../../include/shulib/teleop/drivetrain_degradation.hpp#L104).*

<a id="evaluateruntimedegradation"></a>

## `evaluateRuntimeDegradation`

```cpp
[[nodiscard]] constexpr DegradationVerdict evaluateRuntimeDegradation(SideCount left, SideCount right) noexcept
```

The MID-RUN policy (chunk T4 Part 2; the team lead's ruling N7 (c), 2026-09-27: "Only when 3+ are dead in total"): refuse iff the total dead exceeds kMaxDeadTotal — the per-side floor is NOT applied once driving has started (header: on robot two's four a side it would stop the robot at the second death on one side, against his rule of 2026-09-10). DriveDegraded when anything is dead, Drive when nothing is, with the same counts as evaluateDegradation(). Because only the total is judged, an EMPTY side (expected 0) is not nonsense here — a chassis with one group passes it as (that group, {}) — but a negative count, or more answering than expected, is REFUSED with a reason (0 V is the safe direction for a census that makes no sense). evaluateDegradation() is unchanged: it is the boot check's rule.

*free function, declared at [`include/shulib/teleop/drivetrain_degradation.hpp:149`](../../include/shulib/teleop/drivetrain_degradation.hpp#L149).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 56 lines, click to expand</summary>

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

 THE REASONING, stated so the numbers are not magic — and stated as it was
 reasoned, for FIVE coupled motors per side on one gear train (robot two's typed
 table of 2026-09-10). Two motors of five is a side that cannot pull its share —
 under load it lags, and the robot drags into a curve at speed that the driver
 cannot steer out of; that side is also carrying the mechanical drag of three
 unpowered rotors. Three of five drives: down 40 % of the side's torque, straight
 enough at reduced speed, and the driver is told. The total cap of two is the
 second guard: two dead on ONE side is 3/5 (drives); one dead on each is 4/5 + 4/5
 (drives); three dead anywhere is a drivetrain with a systemic problem — a cable
 harness, a brain port bank, a battery sag — not a port, and driving it is the
 wrong answer even where the per-side floor would pass.

 ROBOT TWO IS NOW FOUR PER SIDE (its calibration record of 2026-09-25), and the
 same numbers read differently there: 3/4 drives, and 2/4 — the SECOND dead port
 on one side — is REFUSED by the per-side floor. At BOOT that is the rule as it
 stands (the boot check applies evaluateDegradation(), below, once). MID-RUN it is
 not: the team lead's ruling of 2026-09-27 (N7 (c), "Only when 3+ are dead in
 total") keeps only the TOTAL cap once driving has started —
 evaluateRuntimeDegradation(), below — so two dead on one side of robot two drive
 on with a warning, which is his rule of 2026-09-10 exactly ("1-2 dead ports
 shouldn't stop driving"), and the third anywhere stops the drive. Whether the boot
 should match is his question, not this header's.

 DOMAIN, stated honestly: the floor is written for a five-per-side train and
 the four-per-side ones (robot two since 2026-09-25, the bench bot: 3/4 drives,
 2/4 refuses at boot). A drivetrain with fewer than three motors per side is
 outside this policy's domain — it would be refused at every boot — and that is
 deliberate: nobody has thought about such a robot here, and a policy that
 silently generalised to it would be an invented answer. Widen it, with a reason,
 when such a robot exists.

 PURE and PROS-free: numbers in, a verdict out, no state, no clock. The callers
 count and act: the boot check (calibration/boot_check.hpp) applies
 evaluateDegradation() once, before any adapter exists; the library's driver loop
 (teleop/drive_protection.hpp, since T4 Part 2) re-counts each group's present
 members whenever one is marked ABSENT mid-run, and at every entry, and applies
 evaluateRuntimeDegradation(). The old "shulib Drive" applied evaluateDegradation()
 both at boot and mid-run itself, until T4 Part 3 replaced it with a program that
 drives through that loop.
```

</details>
