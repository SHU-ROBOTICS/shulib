<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/direction_check.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `direction_check.hpp`

mechanism/direction_check.hpp — THE DIRECTION CHECK's judgement.

This header declares **4** types (22 members), **4** free functions, and **1** constant.

Extracted from [`include/shulib/mechanism/direction_check.hpp`](../../include/shulib/mechanism/direction_check.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class PersonSaw`](#enum-class-personsaw)
  - [`NotAsked`](#personsaw-notasked)
  - [`ItsPlusWay`](#personsaw-itsplusway)
  - [`TheOtherWay`](#personsaw-theotherway)
  - [`NothingOrSomethingElseMoved`](#personsaw-nothingorsomethingelsemoved)
- [`struct DirectionObservation`](#struct-directionobservation)
  - [`commandVolts`](#directionobservation-commandvolts)
  - [`motorDeltaRad`](#directionobservation-motordeltarad)
  - [`motorPeakAmps`](#directionobservation-motorpeakamps)
  - [`hasAngleSensor`](#directionobservation-hasanglesensor)
  - [`sensorDeltaRad`](#directionobservation-sensordeltarad)
  - [`person`](#directionobservation-person)
- [`enum class DirectionFinding`](#enum-class-directionfinding)
  - [`Ok`](#directionfinding-ok)
  - [`DidNotMove`](#directionfinding-didnotmove)
  - [`MotorFights`](#directionfinding-motorfights)
  - [`FightOrJam`](#directionfinding-fightorjam)
  - [`SensorReversed`](#directionfinding-sensorreversed)
  - [`MechanismReversed`](#directionfinding-mechanismreversed)
  - [`CableOnAnotherMechanism`](#directionfinding-cableonanothermechanism)
  - [`AwaitingPerson`](#directionfinding-awaitingperson)
  - [`BackDriven`](#directionfinding-backdriven)
- [`changeHere`](#changehere) — *free function*
- [`wireHere`](#wirehere) — *free function*
- [`struct DirectionVerdict`](#struct-directionverdict)
  - [`finding`](#directionverdict-finding)
  - [`role`](#directionverdict-role)
  - [`fix`](#directionverdict-fix)
- [`judgeDirection`](#judgedirection) — *free function*
- [`kNothingOrSomethingElseMoved`](#knothingorsomethingelsemoved) — *constant*
- [`judgePneumatic`](#judgepneumatic) — *free function*

<a id="enum-class-personsaw"></a>

## `enum class PersonSaw`

```cpp
enum class PersonSaw : std::uint8_t
```

What the person answered after the jog — the page's THREE buttons, plus "not asked yet".

*enum class, declared at [`include/shulib/mechanism/direction_check.hpp:61`](../../include/shulib/mechanism/direction_check.hpp#L61).*

<a id="personsaw-notasked"></a>

### `PersonSaw::NotAsked`

```cpp
NotAsked = 0
```

the question has not been answered yet

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:62`](../../include/shulib/mechanism/direction_check.hpp#L62).*

<a id="personsaw-itsplusway"></a>

### `PersonSaw::ItsPlusWay`

```cpp
ItsPlusWay
```

THIS mechanism moved, the way its "+" should

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:63`](../../include/shulib/mechanism/direction_check.hpp#L63).*

<a id="personsaw-theotherway"></a>

### `PersonSaw::TheOtherWay`

```cpp
TheOtherWay
```

THIS mechanism moved, the OTHER way

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:64`](../../include/shulib/mechanism/direction_check.hpp#L64).*

<a id="personsaw-nothingorsomethingelsemoved"></a>

### `PersonSaw::NothingOrSomethingElseMoved`

```cpp
NothingOrSomethingElseMoved
```

it did NOT move — or ANOTHER mechanism moved

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:65`](../../include/shulib/mechanism/direction_check.hpp#L65).*

<a id="struct-directionobservation"></a>

## `struct DirectionObservation`

```cpp
struct DirectionObservation
```

What the page recorded over one jog of ONE motor mechanism.

*struct, declared at [`include/shulib/mechanism/direction_check.hpp:69`](../../include/shulib/mechanism/direction_check.hpp#L69).*

<a id="directionobservation-commandvolts"></a>

### `DirectionObservation::commandVolts`

```cpp
double commandVolts = 0.0
```

The jog's signed command, volts — the robot file's "+" way when positive. Finite, not 0.

*field, declared at [`include/shulib/mechanism/direction_check.hpp:71`](../../include/shulib/mechanism/direction_check.hpp#L71).*

<a id="directionobservation-motordeltarad"></a>

### `DirectionObservation::motorDeltaRad`

```cpp
double motorDeltaRad[kMaxMotorRoles]{}
```

Each motor role's encoder change over the jog, radians (role order: the declaration's).

*field, declared at [`include/shulib/mechanism/direction_check.hpp:73`](../../include/shulib/mechanism/direction_check.hpp#L73).*

<a id="directionobservation-motorpeakamps"></a>

### `DirectionObservation::motorPeakAmps`

```cpp
double motorPeakAmps[kMaxMotorRoles]{}
```

Each motor role's highest current during the jog, amps.

*field, declared at [`include/shulib/mechanism/direction_check.hpp:75`](../../include/shulib/mechanism/direction_check.hpp#L75).*

<a id="directionobservation-hasanglesensor"></a>

### `DirectionObservation::hasAngleSensor`

```cpp
bool hasAngleSensor = false
```

The mechanism has an angle sensor and the page read it.

*field, declared at [`include/shulib/mechanism/direction_check.hpp:77`](../../include/shulib/mechanism/direction_check.hpp#L77).*

<a id="directionobservation-sensordeltarad"></a>

### `DirectionObservation::sensorDeltaRad`

```cpp
double sensorDeltaRad = 0.0
```

The angle sensor's change over the jog, radians (as the declaration's mounting reads it).

*field, declared at [`include/shulib/mechanism/direction_check.hpp:79`](../../include/shulib/mechanism/direction_check.hpp#L79).*

<a id="directionobservation-person"></a>

### `DirectionObservation::person`

```cpp
PersonSaw person = PersonSaw::NotAsked
```

The person's answer.

*field, declared at [`include/shulib/mechanism/direction_check.hpp:81`](../../include/shulib/mechanism/direction_check.hpp#L81).*

<a id="enum-class-directionfinding"></a>

## `enum class DirectionFinding`

```cpp
enum class DirectionFinding : std::uint8_t
```

The judgement's finding.

*enum class, declared at [`include/shulib/mechanism/direction_check.hpp:85`](../../include/shulib/mechanism/direction_check.hpp#L85).*

<a id="directionfinding-ok"></a>

### `DirectionFinding::Ok`

```cpp
Ok
```

it moved its + way (or a pneumatic went where it was told)

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:86`](../../include/shulib/mechanism/direction_check.hpp#L86).*

<a id="directionfinding-didnotmove"></a>

### `DirectionFinding::DidNotMove`

```cpp
DidNotMove
```

a motor neither moved nor drew current: its cable, or its port

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:87`](../../include/shulib/mechanism/direction_check.hpp#L87).*

<a id="directionfinding-motorfights"></a>

### `DirectionFinding::MotorFights`

```cpp
MotorFights
```

a motor turned AGAINST its command while a mate turned WITH its own: the mate drags it

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:88`](../../include/shulib/mechanism/direction_check.hpp#L88).*

<a id="directionfinding-fightorjam"></a>

### `DirectionFinding::FightOrJam`

```cpp
FightOrJam
```

every motor still, every one drawing: they fight, or it is jammed

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:89`](../../include/shulib/mechanism/direction_check.hpp#L89).*

<a id="directionfinding-sensorreversed"></a>

### `DirectionFinding::SensorReversed`

```cpp
SensorReversed
```

the angle sensor reads against the motors

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:90`](../../include/shulib/mechanism/direction_check.hpp#L90).*

<a id="directionfinding-mechanismreversed"></a>

### `DirectionFinding::MechanismReversed`

```cpp
MechanismReversed
```

it moved the OTHER way (or a pneumatic went to another position)

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:91`](../../include/shulib/mechanism/direction_check.hpp#L91).*

<a id="directionfinding-cableonanothermechanism"></a>

### `DirectionFinding::CableOnAnotherMechanism`

```cpp
CableOnAnotherMechanism
```

it did not move although commanded: its cable or line is elsewhere

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:92`](../../include/shulib/mechanism/direction_check.hpp#L92).*

<a id="directionfinding-awaitingperson"></a>

### `DirectionFinding::AwaitingPerson`

```cpp
AwaitingPerson
```

the encoders found nothing: ask "ONLY '<name>' should move: which way?"

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:93`](../../include/shulib/mechanism/direction_check.hpp#L93).*

<a id="directionfinding-backdriven"></a>

### `DirectionFinding::BackDriven`

```cpp
BackDriven
```

every motor that moved went AGAINST its command: gravity or a load beat the jog

*enumerator, declared at [`include/shulib/mechanism/direction_check.hpp:94`](../../include/shulib/mechanism/direction_check.hpp#L94).*

<a id="changehere"></a>

## `changeHere`

```cpp
inline void changeHere(char* out, std::size_t cap, const char* source, const char* file, std::uint32_t line, const MechanismSpec& s)
```

"Change it HERE" (file header): `source` when stated (a generated robot file's JSON path), else `file`:`line` (where the team wrote it), else the declaration's own line, else the robot file by name. Writes at most `cap` bytes, NUL-terminated.

*free function, declared at [`include/shulib/mechanism/direction_check.hpp:100`](../../include/shulib/mechanism/direction_check.hpp#L100).*

<a id="wirehere"></a>

## `wireHere`

```cpp
inline void wireHere(char* out, std::size_t cap, const MechanismWiring& w, const MechanismSpec& s)
```

The mechanism's WIRING line (its `wire(...)`), where a cable on the wrong port is fixed; the wiring list by name when the line was not captured. Writes at most `cap` bytes, NUL-terminated.

*free function, declared at [`include/shulib/mechanism/direction_check.hpp:114`](../../include/shulib/mechanism/direction_check.hpp#L114).*

<a id="struct-directionverdict"></a>

## `struct DirectionVerdict`

```cpp
struct DirectionVerdict
```

The judgement: a finding, the motor role it names (where it names one), and the sentence — the robot-file line to change, or what to check.

*struct, declared at [`include/shulib/mechanism/direction_check.hpp:124`](../../include/shulib/mechanism/direction_check.hpp#L124).*

<a id="directionverdict-finding"></a>

### `DirectionVerdict::finding`

```cpp
DirectionFinding finding = DirectionFinding::AwaitingPerson
```

What was found.

*field, declared at [`include/shulib/mechanism/direction_check.hpp:126`](../../include/shulib/mechanism/direction_check.hpp#L126).*

<a id="directionverdict-role"></a>

### `DirectionVerdict::role`

```cpp
std::uint8_t role = 0
```

The motor role the finding names (0-based; 0 where it names none).

*field, declared at [`include/shulib/mechanism/direction_check.hpp:128`](../../include/shulib/mechanism/direction_check.hpp#L128).*

<a id="directionverdict-fix"></a>

### `DirectionVerdict::fix`

```cpp
char fix[384]{}
```

The sentence (empty while awaiting the person).

*field, declared at [`include/shulib/mechanism/direction_check.hpp:130`](../../include/shulib/mechanism/direction_check.hpp#L130).*

<a id="judgedirection"></a>

## `judgeDirection`

```cpp
[[nodiscard]] inline DirectionVerdict judgeDirection(const MechanismSpec& s, const MechanismWiring& w, const DirectionObservation& o, double minMoveRad, double drawAmps)
```

THE JUDGEMENT for a MOTOR mechanism (file header), in this order: every motor still and every one drawing → FightOrJam; a motor that neither moved nor drew → DidNotMove (its cable, its wire line); a motor that moved AGAINST the command while another moved WITH it → MotorFights (that role's Forward/Reversed); motors that moved, every one AGAINST the command → BackDriven (no line named: jog harder or from another pose); the angle sensor against the command → SensorReversed; then the person's answer — NotAsked → AwaitingPerson, TheOtherWay → MechanismReversed (swap EVERY Forward and Reversed), NothingOrSomethingElseMoved → CableOnAnotherMechanism (its wire line), ItsPlusWay → Ok. PRECONDITIONS: `s` is a motor kind (one to four motor roles); `o.commandVolts` finite and not 0; `minMoveRad` and `drawAmps` finite and above 0 (the page's stated numbers).

*free function, declared at [`include/shulib/mechanism/direction_check.hpp:143`](../../include/shulib/mechanism/direction_check.hpp#L143).*

<a id="knothingorsomethingelsemoved"></a>

## `kNothingOrSomethingElseMoved`

```cpp
inline constexpr std::uint8_t kNothingOrSomethingElseMoved = 0xFF
```

A pneumatic's answer "it did NOT move, or ANOTHER mechanism moved" (judgePneumatic's `sawPosition`).

*constant, declared at [`include/shulib/mechanism/direction_check.hpp:291`](../../include/shulib/mechanism/direction_check.hpp#L291).*

<a id="judgepneumatic"></a>

## `judgePneumatic`

```cpp
[[nodiscard]] inline DirectionVerdict judgePneumatic(const MechanismSpec& s, const MechanismWiring& w, std::uint8_t commanded, std::uint8_t sawPosition)
```

THE JUDGEMENT for a PNEUMATIC mechanism: the page commanded position `commanded` (after the person confirmed the robot was safe) and the person says which named position it went to — or kNothingOrSomethingElseMoved. The commanded one → Ok; another → MechanismReversed (its line levels are swapped, named where they are stated); nothing or something else → CableOnAnotherMechanism (its valve is not on the line the wiring says, or is unplugged, or has no air — its wire line). PRECONDITIONS: `s` is a pneumatic; `commanded` one of its positions; `sawPosition` one of its positions or kNothingOrSomethingElseMoved.

*free function, declared at [`include/shulib/mechanism/direction_check.hpp:300`](../../include/shulib/mechanism/direction_check.hpp#L300).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 46 lines, click to expand</summary>

```text

 mechanism/direction_check.hpp — THE DIRECTION CHECK's judgement (the mechanism core, chunk MECH-A4,
 2026-10-22; its bench page is MECH-A6's station 13). Pure, host-tested, PROS-free.

 ── What it is for ─────────────────────────────────────────────────────────────────────
 The boot names every role's port and sees a cable on the WRONG KIND of device, but two cables of
 the SAME kind swapped between mechanisms, a motor declared the wrong way, or an angle sensor
 mounted backwards all look fine at boot. The direction check jogs ONE mechanism a little (the page
 commands it, under its own gates), records what the encoders saw, and asks the person ONE question
 — "ONLY '<name>' should have moved: did it go its + way, the OTHER way, or did it NOT move (or
 something ELSE moved)?" — and this file turns the two halves into a finding and the exact place to
 change: the declaration's line for a direction, the `wire(...)` line for a cable on the wrong
 mechanism. It REPORTS; it cannot save a flip (the team lead's Q1 (a): the robot file is the only
 place a direction lives — edit it, rebuild, upload).

 ── What the encoders can see, and what only a person can ──────────────────────────
 A V5 motor reverses its command AND its encoder together, so a motor's own encoder cannot see that
 motor declared the wrong way (the brief's landmine 17). What the encoders CAN see:
   * a motor whose encoder moves AGAINST its command WHILE A MATE MOVES WITH ITS OWN — the mate
     drags it: a FIGHT, and THAT motor's Forward/Reversed is the one to flip;
   * every motor that moved went AGAINST its command (one motor, or all of them alike) — no mate
     dragged it: the mechanism was BACK-DRIVEN by something stronger than the jog (gravity, a
     load). No declaration is wrong, and none is named (the review's finding: "make motor 1
     Reversed" for a sagging one-motor arm would break a correct declaration);
   * every motor still while every one draws current — two motors of equals declared against each
     other stall TOGETHER, or the mechanism is JAMMED / at a stop (never "check motor 1's cable");
   * a motor that neither moved nor drew — its cable, or its port in the wiring;
   * an angle sensor reading against the motors — the sensor's mounting is declared backwards.
 What only the person can say: which way the MECHANISM moved, and whether it was THIS mechanism at
 all — a same-kind cable swap moves ANOTHER mechanism, whose encoder agrees with its command (the
 re-check's D6: the THIRD answer names the wiring line, not the declaration).

 ── "Change it HERE" ─────────────────────────────────────────────────────────────────
 A role's place, in order: its stated `changeAt` (a GENERATED robot file's JSON path — G3a), else
 the file and line the declaration was written on (captured by the `.name` word — no string typed
 by hand, so it cannot go stale when a line moves), else the declaration's own line. A cable on the
 wrong mechanism is fixed at its `wire(...)` line (captured where motors(...) / lines(...) is
 written). A captured path is the one the compiler opened, so host and brain differ: a test pins
 its SUFFIX.

 ── The thresholds are the PAGE's ───────────────────────────────────────────────────
 `minMoveRad` (an encoder change below it is "did not move") and `drawAmps` (a still motor drawing
 at least it is pushing on something) are the bench page's STATED numbers (HA-182's page entries,
 MECH-A6) — never invented here.

 Allocation-free; every sentence bounded and NUL-terminated in the verdict's own buffer.
```

</details>
