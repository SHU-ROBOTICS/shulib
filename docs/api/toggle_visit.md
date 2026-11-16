<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/toggle_visit.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `toggle_visit.hpp`

competition::checkAndFlipToggle — the Toggle flip for a SKILLS run, as ONE route step.

This header declares **7** types (96 members), **14** free functions, and **2** constants.

Extracted from [`include/shulib/competition/toggle_visit.hpp`](../../include/shulib/competition/toggle_visit.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kToggleVisitTickSeconds`](#ktogglevisittickseconds) — *constant*
- [`kToggleVisitWaitSlackSeconds`](#ktogglevisitwaitslackseconds) — *constant*
- [`struct FlipperGoal`](#struct-flippergoal)
  - [`kind`](#flippergoal-kind)
  - [`position`](#flippergoal-position)
  - [`pneumatic`](#flippergoal-pneumatic)
  - [`FlipperGoal`](#flippergoal-flippergoal)
  - [`FlipperGoal (overload 2)`](#flippergoal-flippergoal-2)
  - [`FlipperGoal (overload 3)`](#flippergoal-flippergoal-3)
  - [`passive`](#flippergoal-passive)
  - [`mechanismName`](#flippergoal-mechanismname)
  - [`enum class FlipperGoal::Kind`](#enum-class-flippergoal-kind)
    - [`Passive`](#flippergoal-kind-passive)
    - [`Position`](#flippergoal-kind-position)
    - [`Pneumatic`](#flippergoal-kind-pneumatic)
- [`struct ToggleVisitRequest`](#struct-togglevisitrequest)
  - [`toggle`](#togglevisitrequest-toggle)
  - [`flipper`](#togglevisitrequest-flipper)
  - [`back`](#togglevisitrequest-back)
  - [`lookFrom`](#togglevisitrequest-lookfrom)
  - [`offBy`](#togglevisitrequest-offby)
  - [`pushFrom`](#togglevisitrequest-pushfrom)
  - [`lookWithin`](#togglevisitrequest-lookwithin)
  - [`flipBound`](#togglevisitrequest-flipbound)
  - [`backBound`](#togglevisitrequest-backbound)
  - [`settle`](#togglevisitrequest-settle)
  - [`driveBound`](#togglevisitrequest-drivebound)
  - [`margin`](#togglevisitrequest-margin)
  - [`maxImageAge`](#togglevisitrequest-maximageage)
  - [`maxPushes`](#togglevisitrequest-maxpushes)
- [`enum class ToggleVisitProblem`](#enum-class-togglevisitproblem)
  - [`None`](#togglevisitproblem-none)
  - [`ToggleUnset`](#togglevisitproblem-toggleunset)
  - [`NotFinite`](#togglevisitproblem-notfinite)
  - [`LookFromUnset`](#togglevisitproblem-lookfromunset)
  - [`OffByUnset`](#togglevisitproblem-offbyunset)
  - [`PushFromUnset`](#togglevisitproblem-pushfromunset)
  - [`PushNotNearer`](#togglevisitproblem-pushnotnearer)
  - [`OffByTooLarge`](#togglevisitproblem-offbytoolarge)
  - [`LookWithinUnset`](#togglevisitproblem-lookwithinunset)
  - [`FlipBoundUnset`](#togglevisitproblem-flipboundunset)
  - [`BackBoundUnset`](#togglevisitproblem-backboundunset)
  - [`SettleUnset`](#togglevisitproblem-settleunset)
  - [`DriveBoundUnset`](#togglevisitproblem-driveboundunset)
  - [`MarginUnset`](#togglevisitproblem-marginunset)
  - [`MaxImageAgeUnset`](#togglevisitproblem-maximageageunset)
  - [`MaxPushes`](#togglevisitproblem-maxpushes)
  - [`FlipperKindsDiffer`](#togglevisitproblem-flipperkindsdiffer)
  - [`TwoMechanisms`](#togglevisitproblem-twomechanisms)
  - [`SameGoalTwice`](#togglevisitproblem-samegoaltwice)
- [`toggleVisitProblemOf`](#togglevisitproblemof) — *free function*
- [`toggleVisitProblemSentence`](#togglevisitproblemsentence) — *free function*
- [`toggleVisitRequestProblem`](#togglevisitrequestproblem) — *free function*
- [`toggleVisit`](#togglevisit) — *free function*
- [`toggleVisitCeilTick`](#togglevisitceiltick) — *free function*
- [`toggleVisitNeedSeconds`](#togglevisitneedseconds) — *free function*
- [`worstCaseSeconds`](#worstcaseseconds) — *free function*
- [`enum class ToggleVisitVerdict`](#enum-class-togglevisitverdict)
  - [`Unset`](#togglevisitverdict-unset)
  - [`AlreadyOurs`](#togglevisitverdict-alreadyours)
  - [`Flipped`](#togglevisitverdict-flipped)
  - [`NotSkills`](#togglevisitverdict-notskills)
  - [`NoCamera`](#togglevisitverdict-nocamera)
  - [`FlipperNotReady`](#togglevisitverdict-flippernotready)
  - [`WrongWall`](#togglevisitverdict-wrongwall)
  - [`UnknownBefore`](#togglevisitverdict-unknownbefore)
  - [`NotAtSpot`](#togglevisitverdict-notatspot)
  - [`NotEnoughTime`](#togglevisitverdict-notenoughtime)
  - [`NoChange`](#togglevisitverdict-nochange)
  - [`StillNotOurs`](#togglevisitverdict-stillnotours)
  - [`UnknownAfter`](#togglevisitverdict-unknownafter)
  - [`NoTimeForSecondPush`](#togglevisitverdict-notimeforsecondpush)
  - [`FlipperStuck`](#togglevisitverdict-flipperstuck)
  - [`Cut`](#togglevisitverdict-cut)
  - [`OursAfterShortStroke`](#togglevisitverdict-oursaftershortstroke)
  - [`RetractedForTime`](#togglevisitverdict-retractedfortime)
  - [`DriveFailed`](#togglevisitverdict-drivefailed)
  - [`SeenNotOurs`](#togglevisitverdict-seennotours)
  - [`FrozenBeforePush`](#togglevisitverdict-frozenbeforepush)
- [`toggleVisitWord`](#togglevisitword) — *free function*
- [`toggleColourOf`](#togglecolourof) — *free function*
- [`oursFor`](#oursfor) — *free function*
- [`struct ToggleVisitReport`](#struct-togglevisitreport)
  - [`verdict`](#togglevisitreport-verdict)
  - [`toggle`](#togglevisitreport-toggle)
  - [`ours`](#togglevisitreport-ours)
  - [`before`](#togglevisitreport-before)
  - [`after`](#togglevisitreport-after)
  - [`whyBefore`](#togglevisitreport-whybefore)
  - [`whyAfter`](#togglevisitreport-whyafter)
  - [`afterTooEarly`](#togglevisitreport-aftertooearly)
  - [`pushes`](#togglevisitreport-pushes)
  - [`fullStroke`](#togglevisitreport-fullstroke)
  - [`range`](#togglevisitreport-range)
  - [`bearing`](#togglevisitreport-bearing)
  - [`positionChecked`](#togglevisitreport-positionchecked)
  - [`lensFromWallIn`](#togglevisitreport-lensfromwallin)
  - [`wallChecked`](#togglevisitreport-wallchecked)
  - [`secondsLeftAtStart`](#togglevisitreport-secondsleftatstart)
  - [`secondsLeftAtCheck`](#togglevisitreport-secondsleftatcheck)
  - [`secondsNeeded`](#togglevisitreport-secondsneeded)
  - [`seconds`](#togglevisitreport-seconds)
  - [`push`](#togglevisitreport-push)
  - [`back`](#togglevisitreport-back)
  - [`driveIn`](#togglevisitreport-drivein)
  - [`driveOut`](#togglevisitreport-driveout)
  - [`detail`](#togglevisitreport-detail)
  - [`ok`](#togglevisitreport-ok)
  - [`outcome`](#togglevisitreport-outcome)
- [`struct ExplicitToggleCamera`](#struct-explicittogglecamera)
  - [`feed`](#explicittogglecamera-feed)
  - [`model`](#explicittogglecamera-model)
  - [`tracker`](#explicittogglecamera-tracker)
  - [`poseTolerance`](#explicittogglecamera-posetolerance)
  - [`headingToleranceDeg`](#explicittogglecamera-headingtolerancedeg)
- [`bandAtLookSpotProblem`](#bandatlookspotproblem) — *free function*
- [`towardWallRadians`](#towardwallradians) — *free function*
- [`checkAndFlipToggle`](#checkandfliptoggle) — *free function*
- [`lookAtToggle`](#lookattoggle) — *free function*

<a id="ktogglevisittickseconds"></a>

## `kToggleVisitTickSeconds`

```cpp
inline constexpr double kToggleVisitTickSeconds = 0.010
```

The control tick the time check rounds every bound UP to, seconds: the brain's 10-ms tick (hal::pros::ProsTickPacer::kTickMs, A4 register HA-32) — the grid every wait ends on.

*constant, declared at [`include/shulib/competition/toggle_visit.hpp:183`](../../include/shulib/competition/toggle_visit.hpp#L183).*

<a id="ktogglevisitwaitslackseconds"></a>

## `kToggleVisitWaitSlackSeconds`

```cpp
inline constexpr double kToggleVisitWaitSlackSeconds = 0.25
```

Slack past a wait's own bound given to its backstop timeout, seconds. Unreachable by construction (each wait's predicate, the mechanism service's deadline or the motion's watchdog ends it first); code-level belt and braces, not an HA entry (Routine's kMechanismWaitSlackSeconds is the same idea).

*constant, declared at [`include/shulib/competition/toggle_visit.hpp:188`](../../include/shulib/competition/toggle_visit.hpp#L188).*

<a id="struct-flippergoal"></a>

## `struct FlipperGoal`

```cpp
struct FlipperGoal
```

WHICH flipper and which of its goals — a preset of ONE Arm or Lift, a position of ONE Pneumatic, or (default-constructed) a PASSIVE flipper: a fixed bar the robot's own drive in and out strokes. It converts IMPLICITLY from a mechanism goal, so `.flipper = flipper["flip"]`.

*struct, declared at [`include/shulib/competition/toggle_visit.hpp:193`](../../include/shulib/competition/toggle_visit.hpp#L193).*

<a id="flippergoal-kind"></a>

### `FlipperGoal::kind`

```cpp
Kind kind = Kind::Passive
```

The kind (Passive when default-constructed).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:201`](../../include/shulib/competition/toggle_visit.hpp#L201).*

<a id="flippergoal-position"></a>

### `FlipperGoal::position`

```cpp
mechanism::PositionGoal position{}
```

The goal when Position.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:203`](../../include/shulib/competition/toggle_visit.hpp#L203).*

<a id="flippergoal-pneumatic"></a>

### `FlipperGoal::pneumatic`

```cpp
mechanism::PneumaticGoal pneumatic{}
```

The goal when Pneumatic.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:205`](../../include/shulib/competition/toggle_visit.hpp#L205).*

<a id="flippergoal-flippergoal"></a>

### `FlipperGoal::FlipperGoal`

```cpp
constexpr FlipperGoal() noexcept = default
```

A PASSIVE flipper.

*function, declared at [`include/shulib/competition/toggle_visit.hpp:208`](../../include/shulib/competition/toggle_visit.hpp#L208).*

<a id="flippergoal-flippergoal-2"></a>

### `FlipperGoal::FlipperGoal (overload 2)`

```cpp
constexpr FlipperGoal(const mechanism::PositionGoal& g) noexcept
```

A PASSIVE flipper. An Arm's or a Lift's goal (`arm["flip"]`, `lift.to(6_in)`).

*function, declared at [`include/shulib/competition/toggle_visit.hpp:210`](../../include/shulib/competition/toggle_visit.hpp#L210).*

<a id="flippergoal-flippergoal-3"></a>

### `FlipperGoal::FlipperGoal (overload 3)`

```cpp
constexpr FlipperGoal(const mechanism::PneumaticGoal& g) noexcept
```

A Pneumatic's position (`flipper["out"]`).

*function, declared at [`include/shulib/competition/toggle_visit.hpp:213`](../../include/shulib/competition/toggle_visit.hpp#L213).*

<a id="flippergoal-passive"></a>

### `FlipperGoal::passive`

```cpp
[[nodiscard]] constexpr bool passive() const noexcept
```

True for a passive flipper.

*function, declared at [`include/shulib/competition/toggle_visit.hpp:217`](../../include/shulib/competition/toggle_visit.hpp#L217).*

<a id="flippergoal-mechanismname"></a>

### `FlipperGoal::mechanismName`

```cpp
[[nodiscard]] constexpr const char* mechanismName() const noexcept
```

The mechanism's name ("" for a passive one).

*function, declared at [`include/shulib/competition/toggle_visit.hpp:219`](../../include/shulib/competition/toggle_visit.hpp#L219).*

<a id="enum-class-flippergoal-kind"></a>

## `enum class FlipperGoal::Kind`

```cpp
enum class Kind : std::uint8_t
```

Which kind of flipper the goal is for.

*enum class, declared at [`include/shulib/competition/toggle_visit.hpp:195`](../../include/shulib/competition/toggle_visit.hpp#L195).*

<a id="flippergoal-kind-passive"></a>

### `FlipperGoal::Kind::Passive`

```cpp
Passive = 0
```

no mechanism: the drive in and out is the stroke (default)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:196`](../../include/shulib/competition/toggle_visit.hpp#L196).*

<a id="flippergoal-kind-position"></a>

### `FlipperGoal::Kind::Position`

```cpp
Position
```

an Arm's or a Lift's preset or constant

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:197`](../../include/shulib/competition/toggle_visit.hpp#L197).*

<a id="flippergoal-kind-pneumatic"></a>

### `FlipperGoal::Kind::Pneumatic`

```cpp
Pneumatic
```

a Pneumatic's named position

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:198`](../../include/shulib/competition/toggle_visit.hpp#L198).*

<a id="struct-togglevisitrequest"></a>

## `struct ToggleVisitRequest`

```cpp
struct ToggleVisitRequest
```

One visit's numbers — every one the TEAM's, from Step 0 at the robot; 0 = UNSET and refused by name (the core's rule: no library default). `flipBound` and `backBound` may stay 0 for a PASSIVE flipper (nothing is commanded); any value given is still counted in the time check.

*struct, declared at [`include/shulib/competition/toggle_visit.hpp:230`](../../include/shulib/competition/toggle_visit.hpp#L230).*

<a id="togglevisitrequest-toggle"></a>

### `ToggleVisitRequest::toggle`

```cpp
vision::override2026::ToggleId toggle{}
```

WHICH Toggle (`toggles::kRedWall`). "Ours" is its Quadrant's colour (HA-243).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:232`](../../include/shulib/competition/toggle_visit.hpp#L232).*

<a id="togglevisitrequest-flipper"></a>

### `ToggleVisitRequest::flipper`

```cpp
FlipperGoal flipper{}
```

The push: a preset of ONE Arm or Lift, a position of ONE Pneumatic, or default = PASSIVE.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:234`](../../include/shulib/competition/toggle_visit.hpp#L234).*

<a id="togglevisitrequest-back"></a>

### `ToggleVisitRequest::back`

```cpp
FlipperGoal back{}
```

The retract: another goal of the SAME mechanism (default = PASSIVE, with a passive `flipper`).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:236`](../../include/shulib/competition/toggle_visit.hpp#L236).*

<a id="togglevisitrequest-lookfrom"></a>

### `ToggleVisitRequest::lookFrom`

```cpp
units::Length lookFrom{}
```

The LOOK SPOT: lens to the Toggle's face, horizontal, inches (Step 0.4). That the camera names the colour from there on a real field is A4 register HA-245 (the host camera model's prediction, unmeasured). With a FIELD start it is ALSO compared with the estimate's lens distance from the wall's INNER face — the face taken to sit at that inner face (A4 register HA-201: UNVERIFIED).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:241`](../../include/shulib/competition/toggle_visit.hpp#L241).*

<a id="togglevisitrequest-offby"></a>

### `ToggleVisitRequest::offBy`

```cpp
units::Length offBy{}
```

How far off the look spot is accepted: the larger half-width of the range interval the bench page prints AT the look spot, plus the route's arrival allowance, inches (Step 0.4 — no default). A COARSE check: where the flipper lands is the route's driving (A4 register HA-244).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:245`](../../include/shulib/competition/toggle_visit.hpp#L245).*

<a id="togglevisitrequest-pushfrom"></a>

### `ToggleVisitRequest::pushFrom`

```cpp
units::Length pushFrom{}
```

The PUSH SPOT: lens to the face where the flipper (or the bar) rolls the Toggle, inches (Step 0.5; HA-244).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:247`](../../include/shulib/competition/toggle_visit.hpp#L247).*

<a id="togglevisitrequest-lookwithin"></a>

### `ToggleVisitRequest::lookWithin`

```cpp
units::Time lookWithin{}
```

The longest a look waits for a sure colour, seconds (Step 0.3).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:249`](../../include/shulib/competition/toggle_visit.hpp#L249).*

<a id="togglevisitrequest-flipbound"></a>

### `ToggleVisitRequest::flipBound`

```cpp
units::Time flipBound{}
```

The push's own time limit, seconds (Step 0.1: the worst of five). It, or the flipper's stall signature, ends a push a sticky Toggle stops before it damages the Toggle — A4 register HA-235. For a pneumatic it must be at least the cylinder's move time plus its confirm window (else FLIPPER NOT READY).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:253`](../../include/shulib/competition/toggle_visit.hpp#L253).*

<a id="togglevisitrequest-backbound"></a>

### `ToggleVisitRequest::backBound`

```cpp
units::Time backBound{}
```

The retract's own time limit, seconds (Step 0.1). For a pneumatic, at least its move time plus its confirm window, as `flipBound`.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:256`](../../include/shulib/competition/toggle_visit.hpp#L256).*

<a id="togglevisitrequest-settle"></a>

### `ToggleVisitRequest::settle`

```cpp
units::Time settle{}
```

How long the Toggle takes to sit flat after the retract, seconds (Step 0.1; HA-232).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:258`](../../include/shulib/competition/toggle_visit.hpp#L258).*

<a id="togglevisitrequest-drivebound"></a>

### `ToggleVisitRequest::driveBound`

```cpp
units::Time driveBound{}
```

The drive in's — and the drive out's — own time limit, seconds (Step 0.6; HA-244).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:260`](../../include/shulib/competition/toggle_visit.hpp#L260).*

<a id="togglevisitrequest-margin"></a>

### `ToggleVisitRequest::margin`

```cpp
units::Time margin{}
```

Spare time kept at the end of the visit, seconds.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:262`](../../include/shulib/competition/toggle_visit.hpp#L262).*

<a id="togglevisitrequest-maximageage"></a>

### `ToggleVisitRequest::maxImageAge`

```cpp
units::Time maxImageAge{}
```

The camera's image age at its read, seconds (Step 0.2; HA-192) — rule 2's pose bound, rule 5's frozen check (how long the drive in may HOLD to tell a frozen camera from a lagging one — counted in rule 4) and rule 6's "a picture from after the retract".

*field, declared at [`include/shulib/competition/toggle_visit.hpp:266`](../../include/shulib/competition/toggle_visit.hpp#L266).*

<a id="togglevisitrequest-maxpushes"></a>

### `ToggleVisitRequest::maxPushes`

```cpp
std::uint8_t maxPushes = 2
```

At most this many pushes: 1 or 2 (a 3 is refused). On the research's reading every skills Toggle needs TWO from yellow, through the other colour (A4 register HA-231) — the rule holds either way.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:269`](../../include/shulib/competition/toggle_visit.hpp#L269).*

<a id="enum-class-togglevisitproblem"></a>

## `enum class ToggleVisitProblem`

```cpp
enum class ToggleVisitProblem : std::uint8_t
```

What is wrong with a request, if anything (each a refusal, at build through toggleVisit(...) or at run time as a precondition).

*enum class, declared at [`include/shulib/competition/toggle_visit.hpp:274`](../../include/shulib/competition/toggle_visit.hpp#L274).*

<a id="togglevisitproblem-none"></a>

### `ToggleVisitProblem::None`

```cpp
None = 0
```

nothing

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:275`](../../include/shulib/competition/toggle_visit.hpp#L275).*

<a id="togglevisitproblem-toggleunset"></a>

### `ToggleVisitProblem::ToggleUnset`

```cpp
ToggleUnset
```

no Toggle named

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:276`](../../include/shulib/competition/toggle_visit.hpp#L276).*

<a id="togglevisitproblem-notfinite"></a>

### `ToggleVisitProblem::NotFinite`

```cpp
NotFinite
```

a number is NaN or infinite

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:277`](../../include/shulib/competition/toggle_visit.hpp#L277).*

<a id="togglevisitproblem-lookfromunset"></a>

### `ToggleVisitProblem::LookFromUnset`

```cpp
LookFromUnset
```

lookFrom UNSET

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:278`](../../include/shulib/competition/toggle_visit.hpp#L278).*

<a id="togglevisitproblem-offbyunset"></a>

### `ToggleVisitProblem::OffByUnset`

```cpp
OffByUnset
```

offBy UNSET

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:279`](../../include/shulib/competition/toggle_visit.hpp#L279).*

<a id="togglevisitproblem-pushfromunset"></a>

### `ToggleVisitProblem::PushFromUnset`

```cpp
PushFromUnset
```

pushFrom UNSET

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:280`](../../include/shulib/competition/toggle_visit.hpp#L280).*

<a id="togglevisitproblem-pushnotnearer"></a>

### `ToggleVisitProblem::PushNotNearer`

```cpp
PushNotNearer
```

pushFrom >= lookFrom: the push spot must be nearer than the look spot

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:281`](../../include/shulib/competition/toggle_visit.hpp#L281).*

<a id="togglevisitproblem-offbytoolarge"></a>

### `ToggleVisitProblem::OffByTooLarge`

```cpp
OffByTooLarge
```

offBy >= lookFrom: the window would reach the face

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:282`](../../include/shulib/competition/toggle_visit.hpp#L282).*

<a id="togglevisitproblem-lookwithinunset"></a>

### `ToggleVisitProblem::LookWithinUnset`

```cpp
LookWithinUnset
```

lookWithin UNSET

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:283`](../../include/shulib/competition/toggle_visit.hpp#L283).*

<a id="togglevisitproblem-flipboundunset"></a>

### `ToggleVisitProblem::FlipBoundUnset`

```cpp
FlipBoundUnset
```

flipBound UNSET for a mechanism flipper (or negative)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:284`](../../include/shulib/competition/toggle_visit.hpp#L284).*

<a id="togglevisitproblem-backboundunset"></a>

### `ToggleVisitProblem::BackBoundUnset`

```cpp
BackBoundUnset
```

backBound UNSET for a mechanism flipper (or negative)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:285`](../../include/shulib/competition/toggle_visit.hpp#L285).*

<a id="togglevisitproblem-settleunset"></a>

### `ToggleVisitProblem::SettleUnset`

```cpp
SettleUnset
```

settle UNSET

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:286`](../../include/shulib/competition/toggle_visit.hpp#L286).*

<a id="togglevisitproblem-driveboundunset"></a>

### `ToggleVisitProblem::DriveBoundUnset`

```cpp
DriveBoundUnset
```

driveBound UNSET

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:287`](../../include/shulib/competition/toggle_visit.hpp#L287).*

<a id="togglevisitproblem-marginunset"></a>

### `ToggleVisitProblem::MarginUnset`

```cpp
MarginUnset
```

margin UNSET

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:288`](../../include/shulib/competition/toggle_visit.hpp#L288).*

<a id="togglevisitproblem-maximageageunset"></a>

### `ToggleVisitProblem::MaxImageAgeUnset`

```cpp
MaxImageAgeUnset
```

maxImageAge UNSET

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:289`](../../include/shulib/competition/toggle_visit.hpp#L289).*

<a id="togglevisitproblem-maxpushes"></a>

### `ToggleVisitProblem::MaxPushes`

```cpp
MaxPushes
```

maxPushes not 1 or 2

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:290`](../../include/shulib/competition/toggle_visit.hpp#L290).*

<a id="togglevisitproblem-flipperkindsdiffer"></a>

### `ToggleVisitProblem::FlipperKindsDiffer`

```cpp
FlipperKindsDiffer
```

the flipper and the back are of different kinds (one passive, or arm vs pneumatic)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:291`](../../include/shulib/competition/toggle_visit.hpp#L291).*

<a id="togglevisitproblem-twomechanisms"></a>

### `ToggleVisitProblem::TwoMechanisms`

```cpp
TwoMechanisms
```

the flipper and the back name two mechanisms

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:292`](../../include/shulib/competition/toggle_visit.hpp#L292).*

<a id="togglevisitproblem-samegoaltwice"></a>

### `ToggleVisitProblem::SameGoalTwice`

```cpp
SameGoalTwice
```

the flipper and the back are the same goal: no stroke

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:293`](../../include/shulib/competition/toggle_visit.hpp#L293).*

<a id="togglevisitproblemof"></a>

## `toggleVisitProblemOf`

```cpp
[[nodiscard]] constexpr ToggleVisitProblem toggleVisitProblemOf(const ToggleVisitRequest& r) noexcept
```

The first problem with `r`, or None. Pure, total, constexpr.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:346`](../../include/shulib/competition/toggle_visit.hpp#L346).*

<a id="togglevisitproblemsentence"></a>

## `toggleVisitProblemSentence`

```cpp
[[nodiscard]] constexpr const char* toggleVisitProblemSentence(ToggleVisitProblem p) noexcept
```

The problem as a sentence for a person, or nullptr for None.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:402`](../../include/shulib/competition/toggle_visit.hpp#L402).*

<a id="togglevisitrequestproblem"></a>

## `toggleVisitRequestProblem`

```cpp
[[nodiscard]] constexpr const char* toggleVisitRequestProblem(const ToggleVisitRequest& r) noexcept
```

The first problem with `r` as a sentence, or nullptr.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:428`](../../include/shulib/competition/toggle_visit.hpp#L428).*

<a id="togglevisit"></a>

## `toggleVisit`

```cpp
[[nodiscard]] consteval ToggleVisitRequest toggleVisit(ToggleVisitRequest r)
```

A request checked AT BUILD: `inline constexpr auto kRedWallVisit = competition::toggleVisit({...});` — a problem stops the build, naming it (the core's build-check idiom). A request built while the auton runs is checked by the step itself, as a precondition.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:435`](../../include/shulib/competition/toggle_visit.hpp#L435).*

<a id="togglevisitceiltick"></a>

## `toggleVisitCeilTick`

```cpp
[[nodiscard]] inline double toggleVisitCeilTick(double x) noexcept
```

`x` seconds rounded UP to the 10-ms tick (a value already on the grid stays).

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:463`](../../include/shulib/competition/toggle_visit.hpp#L463).*

<a id="togglevisitneedseconds"></a>

## `toggleVisitNeedSeconds`

```cpp
[[nodiscard]] inline double toggleVisitNeedSeconds(const ToggleVisitRequest& r, int pushes) noexcept
```

Rule 4's need(n), seconds: n x (ceil(driveBound) + ceil(maxImageAge) + ceil(flipBound) + ceil(backBound) + ceil(settle) + ceil(driveBound) + ceil(lookWithin) + 2 ticks) + margin — every bound rounded UP to the tick; ceil(maxImageAge) is the most the drive in may HOLD on "camera frozen?" (rule 5).

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:470`](../../include/shulib/competition/toggle_visit.hpp#L470).*

<a id="worstcaseseconds"></a>

## `worstCaseSeconds`

```cpp
[[nodiscard]] inline double worstCaseSeconds(const ToggleVisitRequest& r) noexcept
```

The time check's own sum for the whole visit: need(maxPushes), seconds — what a route's budget (and the route preview, RPV1) should allow for this step after the look before it (itself at most `lookWithin`). The ideal host plant flips in less; the robot will not.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:482`](../../include/shulib/competition/toggle_visit.hpp#L482).*

<a id="enum-class-togglevisitverdict"></a>

## `enum class ToggleVisitVerdict`

```cpp
enum class ToggleVisitVerdict : std::uint8_t
```

How a visit ended (appended-only, like every enum a route shares).

*enum class, declared at [`include/shulib/competition/toggle_visit.hpp:487`](../../include/shulib/competition/toggle_visit.hpp#L487).*

<a id="togglevisitverdict-unset"></a>

### `ToggleVisitVerdict::Unset`

```cpp
Unset = 0
```

a default report — no visit ran (never returned by the step)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:488`](../../include/shulib/competition/toggle_visit.hpp#L488).*

<a id="togglevisitverdict-alreadyours"></a>

### `ToggleVisitVerdict::AlreadyOurs`

```cpp
AlreadyOurs
```

SUCCEEDS: it showed ours before; nothing pushed (seating not checked)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:489`](../../include/shulib/competition/toggle_visit.hpp#L489).*

<a id="togglevisitverdict-flipped"></a>

### `ToggleVisitVerdict::Flipped`

```cpp
Flipped
```

SUCCEEDS: it showed ours after a FULL stroke (seating not checked)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:490`](../../include/shulib/competition/toggle_visit.hpp#L490).*

<a id="togglevisitverdict-notskills"></a>

### `ToggleVisitVerdict::NotSkills`

```cpp
NotSkills
```

not a SKILLS auton: no "ours" (the alliance's comes with ALLY1)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:491`](../../include/shulib/competition/toggle_visit.hpp#L491).*

<a id="togglevisitverdict-nocamera"></a>

### `ToggleVisitVerdict::NoCamera`

```cpp
NoCamera
```

no usable camera for this step

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:492`](../../include/shulib/competition/toggle_visit.hpp#L492).*

<a id="togglevisitverdict-flippernotready"></a>

### `ToggleVisitVerdict::FlipperNotReady`

```cpp
FlipperNotReady
```

the flipper absent, not homed, busy, or not at `back` (never moved there)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:493`](../../include/shulib/competition/toggle_visit.hpp#L493).*

<a id="togglevisitverdict-wrongwall"></a>

### `ToggleVisitVerdict::WrongWall`

```cpp
WrongWall
```

a FIELD start, and the robot does not face the named Toggle's wall

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:494`](../../include/shulib/competition/toggle_visit.hpp#L494).*

<a id="togglevisitverdict-unknownbefore"></a>

### `ToggleVisitVerdict::UnknownBefore`

```cpp
UnknownBefore
```

no sure colour at the look (the tracker's reason): nothing pushed

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:495`](../../include/shulib/competition/toggle_visit.hpp#L495).*

<a id="togglevisitverdict-notatspot"></a>

### `ToggleVisitVerdict::NotAtSpot`

```cpp
NotAtSpot
```

the band is not wholly inside the look spot's window: nothing pushed or driven

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:496`](../../include/shulib/competition/toggle_visit.hpp#L496).*

<a id="togglevisitverdict-notenoughtime"></a>

### `ToggleVisitVerdict::NotEnoughTime`

```cpp
NotEnoughTime
```

the time left does not cover need(maxPushes): nothing pushed or driven

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:497`](../../include/shulib/competition/toggle_visit.hpp#L497).*

<a id="togglevisitverdict-nochange"></a>

### `ToggleVisitVerdict::NoChange`

```cpp
NoChange
```

after push 1 it shows the colour it had before (sticky, or a frozen camera)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:498`](../../include/shulib/competition/toggle_visit.hpp#L498).*

<a id="togglevisitverdict-stillnotours"></a>

### `ToggleVisitVerdict::StillNotOurs`

```cpp
StillNotOurs
```

after the last allowed push it is still not ours

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:499`](../../include/shulib/competition/toggle_visit.hpp#L499).*

<a id="togglevisitverdict-unknownafter"></a>

### `ToggleVisitVerdict::UnknownAfter`

```cpp
UnknownAfter
```

no sure colour after the push (the reason): it stops

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:500`](../../include/shulib/competition/toggle_visit.hpp#L500).*

<a id="togglevisitverdict-notimeforsecondpush"></a>

### `ToggleVisitVerdict::NoTimeForSecondPush`

```cpp
NoTimeForSecondPush
```

push 1 turned it to the third colour; need(1) does not fit

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:501`](../../include/shulib/competition/toggle_visit.hpp#L501).*

<a id="togglevisitverdict-flipperstuck"></a>

### `ToggleVisitVerdict::FlipperStuck`

```cpp
FlipperStuck
```

the retract did not reach `back`: the robot stays (it does not drag the Toggle)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:502`](../../include/shulib/competition/toggle_visit.hpp#L502).*

<a id="togglevisitverdict-cut"></a>

### `ToggleVisitVerdict::Cut`

```cpp
Cut
```

the run's deadline or a disable cut the step

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:503`](../../include/shulib/competition/toggle_visit.hpp#L503).*

<a id="togglevisitverdict-oursaftershortstroke"></a>

### `ToggleVisitVerdict::OursAfterShortStroke`

```cpp
OursAfterShortStroke
```

ours after a stroke that stopped short — maybe stuck part-way: NOT counted

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:504`](../../include/shulib/competition/toggle_visit.hpp#L504).*

<a id="togglevisitverdict-retractedfortime"></a>

### `ToggleVisitVerdict::RetractedForTime`

```cpp
RetractedForTime
```

the retract-now rule fired: back at once, out, and done

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:505`](../../include/shulib/competition/toggle_visit.hpp#L505).*

<a id="togglevisitverdict-drivefailed"></a>

### `ToggleVisitVerdict::DriveFailed`

```cpp
DriveFailed
```

the drive in or out did not arrive within driveBound

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:507`](../../include/shulib/competition/toggle_visit.hpp#L507).*

<a id="togglevisitverdict-seennotours"></a>

### `ToggleVisitVerdict::SeenNotOurs`

```cpp
SeenNotOurs
```

lookAtToggle only: a sure colour at the spot, not ours (the look call never pushes)

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:508`](../../include/shulib/competition/toggle_visit.hpp#L508).*

<a id="togglevisitverdict-frozenbeforepush"></a>

### `ToggleVisitVerdict::FrozenBeforePush`

```cpp
FrozenBeforePush
```

the camera seen frozen on the drive in: nothing pushed, the robot drove out

*enumerator, declared at [`include/shulib/competition/toggle_visit.hpp:509`](../../include/shulib/competition/toggle_visit.hpp#L509).*

<a id="togglevisitword"></a>

## `toggleVisitWord`

```cpp
[[nodiscard]] constexpr const char* toggleVisitWord(ToggleVisitVerdict v) noexcept
```

The verdict as words for a log line, a bench page or the route preview.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:513`](../../include/shulib/competition/toggle_visit.hpp#L513).*

<a id="togglecolourof"></a>

## `toggleColourOf`

```cpp
[[nodiscard]] constexpr vision::ToggleColour toggleColourOf(vision::override2026::QuadrantColour q) noexcept
```

A Quadrant's colour as the camera's colour (RED, BLUE; Unknown for Unset).

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:541`](../../include/shulib/competition/toggle_visit.hpp#L541).*

<a id="oursfor"></a>

## `oursFor`

```cpp
[[nodiscard]] constexpr vision::ToggleColour oursFor(vision::override2026::ToggleId id) noexcept
```

"Ours" for `id` in a skills run: its Quadrant's colour as the camera names it (Unknown for Unset).

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:551`](../../include/shulib/competition/toggle_visit.hpp#L551).*

<a id="struct-togglevisitreport"></a>

## `struct ToggleVisitReport`

```cpp
struct ToggleVisitReport
```

One visit, as a VALUE (rule 7's line says the same in words).

*struct, declared at [`include/shulib/competition/toggle_visit.hpp:556`](../../include/shulib/competition/toggle_visit.hpp#L556).*

<a id="togglevisitreport-verdict"></a>

### `ToggleVisitReport::verdict`

```cpp
ToggleVisitVerdict verdict = ToggleVisitVerdict::Unset
```

How it ended.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:558`](../../include/shulib/competition/toggle_visit.hpp#L558).*

<a id="togglevisitreport-toggle"></a>

### `ToggleVisitReport::toggle`

```cpp
vision::override2026::ToggleId toggle{}
```

Which Toggle.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:560`](../../include/shulib/competition/toggle_visit.hpp#L560).*

<a id="togglevisitreport-ours"></a>

### `ToggleVisitReport::ours`

```cpp
vision::ToggleColour ours = vision::ToggleColour::Unknown
```

The target: its Quadrant's colour.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:562`](../../include/shulib/competition/toggle_visit.hpp#L562).*

<a id="togglevisitreport-before"></a>

### `ToggleVisitReport::before`

```cpp
vision::ToggleColour before = vision::ToggleColour::Unknown
```

The colour at the look before (Unknown when not sure).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:564`](../../include/shulib/competition/toggle_visit.hpp#L564).*

<a id="togglevisitreport-after"></a>

### `ToggleVisitReport::after`

```cpp
vision::ToggleColour after = vision::ToggleColour::Unknown
```

The colour at the last look after (Unknown when not sure or none was made).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:566`](../../include/shulib/competition/toggle_visit.hpp#L566).*

<a id="togglevisitreport-whybefore"></a>

### `ToggleVisitReport::whyBefore`

```cpp
vision::ToggleWhy whyBefore = vision::ToggleWhy::NoRead
```

The tracker's reason at the end of the look before.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:568`](../../include/shulib/competition/toggle_visit.hpp#L568).*

<a id="togglevisitreport-whyafter"></a>

### `ToggleVisitReport::whyAfter`

```cpp
vision::ToggleWhy whyAfter = vision::ToggleWhy::NoRead
```

The tracker's reason at the end of the last look after.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:570`](../../include/shulib/competition/toggle_visit.hpp#L570).*

<a id="togglevisitreport-aftertooearly"></a>

### `ToggleVisitReport::afterTooEarly`

```cpp
bool afterTooEarly = false
```

True when the last look after saw a sure colour it could not count: no picture from after the retract, or the lens not yet moved past `frozenMoveIn` since it.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:573`](../../include/shulib/competition/toggle_visit.hpp#L573).*

<a id="togglevisitreport-pushes"></a>

### `ToggleVisitReport::pushes`

```cpp
std::uint8_t pushes = 0
```

Pushes made (a passive stroke counts as one).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:575`](../../include/shulib/competition/toggle_visit.hpp#L575).*

<a id="togglevisitreport-fullstroke"></a>

### `ToggleVisitReport::fullStroke`

```cpp
bool fullStroke[2]{false, false}
```

Whether each push's stroke was FULL (the push SUCCEEDED after a drive in that arrived; a passive drive that reached the push spot).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:578`](../../include/shulib/competition/toggle_visit.hpp#L578).*

<a id="togglevisitreport-range"></a>

### `ToggleVisitReport::range`

```cpp
vision::Interval range{}
```

The band's range interval at the look before, inches (valid when a band passed).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:580`](../../include/shulib/competition/toggle_visit.hpp#L580).*

<a id="togglevisitreport-bearing"></a>

### `ToggleVisitReport::bearing`

```cpp
vision::Interval bearing{}
```

The band's bearing interval at the look before, radians (valid when a band passed).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:582`](../../include/shulib/competition/toggle_visit.hpp#L582).*

<a id="togglevisitreport-positionchecked"></a>

### `ToggleVisitReport::positionChecked`

```cpp
bool positionChecked = false
```

True when the tracker's POSITION gate ran on the look before's sure view (the tracker's config turns it on, and the camera handles stated a pose bound it accepts).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:585`](../../include/shulib/competition/toggle_visit.hpp#L585).*

<a id="togglevisitreport-lensfromwallin"></a>

### `ToggleVisitReport::lensFromWallIn`

```cpp
double lensFromWallIn = 0.0
```

With a FIELD start: the estimate's lens distance from the Toggle's wall at the look, inches.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:587`](../../include/shulib/competition/toggle_visit.hpp#L587).*

<a id="togglevisitreport-wallchecked"></a>

### `ToggleVisitReport::wallChecked`

```cpp
bool wallChecked = false
```

True when the wall and the field distance were checked (a FIELD start).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:589`](../../include/shulib/competition/toggle_visit.hpp#L589).*

<a id="togglevisitreport-secondsleftatstart"></a>

### `ToggleVisitReport::secondsLeftAtStart`

```cpp
double secondsLeftAtStart = 0.0
```

The guard's remaining() when the step began, seconds.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:591`](../../include/shulib/competition/toggle_visit.hpp#L591).*

<a id="togglevisitreport-secondsleftatcheck"></a>

### `ToggleVisitReport::secondsLeftAtCheck`

```cpp
double secondsLeftAtCheck = 0.0
```

The guard's remaining() at the last time check, seconds (0 when none was made).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:593`](../../include/shulib/competition/toggle_visit.hpp#L593).*

<a id="togglevisitreport-secondsneeded"></a>

### `ToggleVisitReport::secondsNeeded`

```cpp
double secondsNeeded = 0.0
```

What that check needed, seconds (0 when none was made).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:595`](../../include/shulib/competition/toggle_visit.hpp#L595).*

<a id="togglevisitreport-seconds"></a>

### `ToggleVisitReport::seconds`

```cpp
double seconds = 0.0
```

The step's own time, seconds.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:597`](../../include/shulib/competition/toggle_visit.hpp#L597).*

<a id="togglevisitreport-push"></a>

### `ToggleVisitReport::push`

```cpp
mechanism::StepReport push[2]{}
```

Each push's StepReport (a default one for a passive stroke or a push never made).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:599`](../../include/shulib/competition/toggle_visit.hpp#L599).*

<a id="togglevisitreport-back"></a>

### `ToggleVisitReport::back`

```cpp
mechanism::StepReport back[2]{}
```

Each retract's StepReport (likewise).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:601`](../../include/shulib/competition/toggle_visit.hpp#L601).*

<a id="togglevisitreport-drivein"></a>

### `ToggleVisitReport::driveIn`

```cpp
control::ExitReason driveIn = control::ExitReason::Running
```

The drive in's and out's exit for the last stroke (Running when none was made).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:603`](../../include/shulib/competition/toggle_visit.hpp#L603).*

<a id="togglevisitreport-driveout"></a>

### `ToggleVisitReport::driveOut`

```cpp
control::ExitReason driveOut = control::ExitReason::Running
```

Likewise, the drive out.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:605`](../../include/shulib/competition/toggle_visit.hpp#L605).*

<a id="togglevisitreport-detail"></a>

### `ToggleVisitReport::detail`

```cpp
const char* detail = ""
```

A short reason beside some verdicts (FLIPPER NOT READY's, NO CAMERA's, NOT AT THE SPOT's) — a library literal or the flipper's absence sentence; "" when none.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:608`](../../include/shulib/competition/toggle_visit.hpp#L608).*

<a id="togglevisitreport-ok"></a>

### `ToggleVisitReport::ok`

```cpp
[[nodiscard]] constexpr bool ok() const noexcept
```

True for the two successes: ALREADY OURS and FLIPPED.

*function, declared at [`include/shulib/competition/toggle_visit.hpp:611`](../../include/shulib/competition/toggle_visit.hpp#L611).*

<a id="togglevisitreport-outcome"></a>

### `ToggleVisitReport::outcome`

```cpp
[[nodiscard]] constexpr manipulation::MechanismOutcome outcome() const noexcept
```

What a route's then() takes: SUCCEEDED for ok(); CANCELLED for CUT (the route stops whatever its on-failure choice); STALLED for FLIPPER STUCK, or a visit that ended short of ours (NO CHANGE, STILL NOT OURS, UNKNOWN AFTER) after a push that STALLED; TIMED_OUT for NOT ENOUGH TIME, NO TIME FOR PUSH 2 and RETRACTED FOR TIME; UNCONFIRMED for every other doubt, OURS AFTER A SHORT STROKE included.

*function, declared at [`include/shulib/competition/toggle_visit.hpp:619`](../../include/shulib/competition/toggle_visit.hpp#L619).*

<a id="struct-explicittogglecamera"></a>

## `struct ExplicitToggleCamera`

```cpp
struct ExplicitToggleCamera
```

The camera handles the explicit form reads — the robot's ONE camera feed and the mount the library believes, the TEAM's tracker numbers, and the pose bound the caller states. The step builds its OWN tracker from `tracker` and `model` for each visit, so the hold that counts always began inside it.

*struct, declared at [`include/shulib/competition/toggle_visit.hpp:642`](../../include/shulib/competition/toggle_visit.hpp#L642).*

<a id="explicittogglecamera-feed"></a>

### `ExplicitToggleCamera::feed`

```cpp
vision::CameraFeed* feed = nullptr
```

The camera's feed — ticked inside the step's waits (nullptr: no camera).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:644`](../../include/shulib/competition/toggle_visit.hpp#L644).*

<a id="explicittogglecamera-model"></a>

### `ExplicitToggleCamera::model`

```cpp
const vision::CameraModel* model = nullptr
```

The mount the library believes, built from a placement cameraModelProblem() accepted (nullptr: no camera).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:647`](../../include/shulib/competition/toggle_visit.hpp#L647).*

<a id="explicittogglecamera-tracker"></a>

### `ExplicitToggleCamera::tracker`

```cpp
vision::ToggleTrackerConfig tracker{}
```

The team's tracker numbers (toggleTrackerConfigProblem() must accept them).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:649`](../../include/shulib/competition/toggle_visit.hpp#L649).*

<a id="explicittogglecamera-posetolerance"></a>

### `ExplicitToggleCamera::poseTolerance`

```cpp
units::Length poseTolerance{}
```

The pose bound the caller states, inches: how far the estimate may be off. 0 = NONE stated — the tracker's position gate is then skipped (its view says unchecked), never run on a pose taken as exact.

*field, declared at [`include/shulib/competition/toggle_visit.hpp:653`](../../include/shulib/competition/toggle_visit.hpp#L653).*

<a id="explicittogglecamera-headingtolerancedeg"></a>

### `ExplicitToggleCamera::headingToleranceDeg`

```cpp
double headingToleranceDeg = 0.0
```

The heading bound the caller states, degrees (used only with `poseTolerance`).

*field, declared at [`include/shulib/competition/toggle_visit.hpp:655`](../../include/shulib/competition/toggle_visit.hpp#L655).*

<a id="bandatlookspotproblem"></a>

## `bandAtLookSpotProblem`

```cpp
[[nodiscard]] inline const char* bandAtLookSpotProblem(const vision::ToggleView& v, const ToggleVisitRequest& r, double bearingWindowDeg) noexcept
```

The band at the look spot? nullptr when its WHOLE range interval lies inside [lookFrom - offBy, lookFrom + offBy] (both ends included) and its WHOLE bearing interval inside +-bearingWindowDeg; otherwise what is wrong. Containment, never overlap (file banner, rule 2). Pure.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:661`](../../include/shulib/competition/toggle_visit.hpp#L661).*

<a id="towardwallradians"></a>

## `towardWallRadians`

```cpp
[[nodiscard]] inline double towardWallRadians(vision::override2026::ToggleId id) noexcept
```

The direction TOWARD `id`'s wall, F1 radians (minus kTogglePlaces' inward normal): right 0, blue +pi/2, left pi, red -pi/2. PRECONDITION-free: an Unset id answers 0.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:678`](../../include/shulib/competition/toggle_visit.hpp#L678).*

<a id="checkandfliptoggle"></a>

## `checkAndFlipToggle`

```cpp
[[nodiscard]] inline ToggleVisitReport checkAndFlipToggle(ExplicitToggleCamera& camera, chassis::Chassis& chassis, sequence::RunGuard& guard, mechanism::MechanismService* service, const AutonEntry& entry, const ToggleVisitRequest& request, hal::ITelemetrySink& sink)
```

THE STEP, explicit form (file banner): look at `request.toggle` from the look spot the route parked the robot on; nothing more if it already shows its Quadrant's colour; otherwise drive in, push, retract, drive out, look again — at most `maxPushes` pushes, the second only on the third colour. Blocking, through `guard`'s deadline-aware waits only; at most the look before plus worstCaseSeconds(request). Returns the report, a VALUE; `outcome()` is what a then() returns. ONE [TGL] line to `sink`. `service` may be nullptr for a PASSIVE flipper. PRECONDITIONS (a programming error, thrown): `guard` is running; toggleVisitRequestProblem(request) is nullptr; and, with a usable camera, lookFrom - pushFrom is more than twice the tracker's frozenMoveIn.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:1511`](../../include/shulib/competition/toggle_visit.hpp#L1511).*

<a id="lookattoggle"></a>

## `lookAtToggle`

```cpp
[[nodiscard]] inline ToggleVisitReport lookAtToggle(ExplicitToggleCamera& camera, chassis::Chassis& chassis, sequence::RunGuard& guard, mechanism::MechanismService* service, const AutonEntry& entry, const ToggleVisitRequest& request, hal::ITelemetrySink& sink)
```

Rules 1–3 ONLY — it never moves anything: is it allowed, what colour is the Toggle, is the robot at the look spot. ALREADY OURS, SEEN NOT OURS (a sure colour at the spot, not ours), or the refusal that stopped it — the building block for a team that writes the flip itself (the brief's D1 (c); the checked step is the recommended path). ONE [TGL] line. The preconditions of checkAndFlipToggle.

*free function, declared at [`include/shulib/competition/toggle_visit.hpp:1524`](../../include/shulib/competition/toggle_visit.hpp#L1524).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 150 lines, click to expand</summary>

```text

 competition::checkAndFlipToggle — the Toggle flip for a SKILLS run, as ONE route step (chunk AF1S-A1,
 2026-11-11; the AF1S brief's D1–D11 and its re-check R1–R12, the team lead's answers of 2026-11-10).
 At a LOOK SPOT the route parked the robot on, square in front of a named Toggle, it looks at the
 Toggle with the front AI Vision camera, does NOTHING if the Toggle already shows its Quadrant's
 colour, and otherwise drives straight in, pushes with the flipper, retracts, drives straight back
 out and looks again — a second push only when the first turned the Toggle to the THIRD colour,
 never a third. Every doubt does nothing more and is reported; the route's own choice after a failed
 step (its onFail, or the default stop) decides what comes next. The form this run gives is the
 EXPLICIT one, over handles (the host tests and a hand-wired Tier-3 program); the everyday form over
 an auton's context is AF1S-A2's.

     r.then([&] {
         return competition::checkAndFlipToggle(cam, ctx.chassis, ctx.guard, ctx.mechanismService,
                                                ctx.entry, kRedWallVisit, ctx.telemetry).outcome();
     }, "red wall Toggle");

 ── "Ours" in a skills run ──────────────────────────────────────────────────────────────────────
 Each Toggle's QUADRANT colour (vision/override_elements.hpp's table, A4 register HA-243 — a reading
 of the manual's drawings, v2.0 <RSC3>a-iii-1: "Toggles only confer Ownership of Pins when the Toggle
 matches the color of its Quadrant"; v2.1's change log keeps it in substance). The route names WHICH
 Toggle; "ours" is never typed, so it cannot be typed wrong. A MATCH auton is refused (NOT SKILLS):
 its "ours" is the alliance's, which comes after the 21st (ALLY1).

 ── What "FLIPPED" means — and what it cannot (A4 register HA-232) ──────────────────────────────
 It means "the Toggle SHOWS our colour after a FULL stroke" — never "it is set". The game counts a
 Toggle as set only when it is fully seated and no robot touches it (v2.0 <SC4>); a Toggle stuck
 part-way through its roll shows the face it is rolling TO, and the camera reads that colour "sure"
 (VIS1-A2's banner; the AF1S brief's probe P5(c): 58 of 108 stuck positions). So a success line
 always ENDS "seating not checked", and a stroke that stopped short — a push that STALLED, TIMED OUT
 or was CANCELLED, or a passive drive that did not reach the push spot — and then reads ours is NOT a
 success: OURS AFTER SHORT STROKE. That the flipper pushes on the camera's word ALONE is the team
 lead's ruling (2026-11-03, "just the ai vision sensor", confirmed for this step on 2026-11-10 — the
 brief's D11): no sensor sees colour at the push point.

 ── The rules, in order (each one a test) ───────────────────────────────────────────────────────
   1. ALLOWED — reading NOTHING, commanding nothing: a SKILLS auton (else NOT SKILLS); a usable camera
      (the feed, a measured mount, a tracker config the tracker accepts — else NO CAMERA); the flipper
      found on this robot, built, homed, idle and AT `back` within its "reached" tolerance — a
      pneumatic's `flipBound` and `backBound` each at least its move time plus its confirm window,
      the most its slot waits for a verdict, so rule 4 never counts less than the step will wait
      (else FLIPPER NOT READY — the step NEVER moves it there itself: the route does); with a FIELD start,
      the estimate's heading within +-45 degrees of the direction TOWARD the named Toggle's wall —
      minus kTogglePlaces' inward normal: right wall 0, blue wall +90, left wall 180, red wall -90
      (else WRONG WALL). A RELATIVE start skips the wall check and the line says so.
   2. LOOK — a FRESH tracker, fed from the look's start on every read of the feed (ticked inside the
      run guard's deadline-aware wait, so the mechanism service and any motion keep ticking and a
      deadline cuts it), with the estimate and the caller's pose bound widened for the image's age
      (vision::boundAtImage, `maxImageAge`) — up to `lookWithin` for a SURE colour, else UNKNOWN
      BEFORE with the tracker's reason. Sure, the band must be AT THE SPOT: its WHOLE range interval
      inside [lookFrom - offBy, lookFrom + offBy] — CONTAINMENT, never overlap (the interval is 5–18
      in wide on the host camera model: an overlap test accepted a robot 8–11 in too far) — and its
      whole bearing interval inside the tracker's window; with a FIELD start, also the estimate's lens
      within `offBy` of `lookFrom` from the wall's inner face, along its normal — which takes the
      Toggle's face to sit AT that inner face (where it sits is UNVERIFIED, A4 register HA-201: a real
      set-back shifts this window by that much; Step 0.4 measures it). Else NOT AT THE SPOT. This check is
      COARSE: the camera catches only gross arrival errors; where the flipper lands is the route's
      odometry (A4 register HA-244).
   3. OURS ALREADY — ALREADY OURS, nothing pushed.
   4. TIME — need(n) = n x (ceil(driveBound) + ceil(maxImageAge) + ceil(flipBound) + ceil(backBound) +
      ceil(settle) + ceil(driveBound) + ceil(lookWithin) + 2 ticks) + margin, ceil() up to the 10-ms
      tick (ceil(maxImageAge): the most the drive in HOLDS on "camera frozen?", rule 5). Push 1
      only when the guard's remaining() >= need(maxPushes) (a visit that may need two pushes is not
      begun unless both fit), push 2 only when >= need(1). Else NOT ENOUGH TIME / NO TIME FOR PUSH 2.
      worstCaseSeconds(request) is need(maxPushes) — the look before it adds at most `lookWithin`.
   5. FLIP, every push PAIRED — drive straight IN by lookFrom - pushFrom along the current heading
      (NEVER a blocking Chassis verb: a motion::MoveToPose started with scheduler().async and waited
      through the guard, its predicate ticking the feed and the tracker on every read, cancelled on
      every exit); command `flipper`; on ANY verdict, `back` at once; wait for it; `settle`; drive
      straight back OUT to the look spot. The camera SEEN FROZEN on the drive in (the lens moved past
      `frozenMoveIn` and the band's box did not move, still so on a picture taken after that was first
      seen — readAt - maxImageAge, HA-192: a live camera's lagging pictures catch up within
      maxImageAge — so the look the push would rest on may be a stale picture): the drive in stops
      there, nothing is pushed, the robot drives out (FROZEN BEFORE PUSH). Raised but not yet that old
      (a maxImageAge longer than the rest of the drive in), the step HOLDS before anything reaches the
      Toggle — an active flipper's drive in arrives and nothing is pushed yet; a PASSIVE bar's drive
      in (the stroke itself) stops where the question is raised — and keeps reading, at most
      ceil(maxImageAge) a drive in (rule 4 counts it): the evidence turning stale, or the question
      still open when that runs out, is FROZEN BEFORE PUSH; the question clearing (a live camera's
      lagging pictures caught up) lets the push go on (a passive bar drives on, within what is left of
      driveBound). With a pose bound stated, this check (and rule 6's "camera frozen?") reads an
      UNGATED twin of the tracker: the position gate drops a frozen picture's band as "no band" once
      the lens has moved it off the Toggle's place. A drive in that did not arrive: nothing pushed, the robot drives out (DRIVE FAILED); a
      drive OUT that did not arrive: DRIVE FAILED, no look after (it would not be from the look
      spot). A retract that does not reach `back`: FLIPPER STUCK and the robot does
      NOT drive out (a flipper caught on the Toggle would drag it — <GG9>). If, while the flipper is
      OUT (from the push's command until `back`'s; a passive bar: from the end of the drive in until
      the drive out), the time left falls to backBound + settle + driveBound + margin, `back` is
      commanded AT ONCE (the push is replaced), there is no settle, the robot drives out as soon as it
      is back, and the visit ends RETRACTED FOR TIME — the robot must be OFF the Toggle at 0:00
      (<SC4>b). Rule 4 makes that unreachable while every stage keeps to its bound; it is for time lost
      OUTSIDE them (a brain's loop that stalled). A PASSIVE flipper (a fixed bar): the drive in and out
      IS the stroke; no mechanism is commanded.
   6. LOOK AGAIN — moving, and only after the retract (the brief's re-check R3, R4, R8). The tracker
      has been fed on EVERY read since the look before (so a camera that froze anywhere — at the look
      spot, on the way in, at the push spot, on the way out — has been moved past `frozenMoveIn` and
      reads "camera frozen?"); the after-colour counts only from a SURE view on a read whose IMAGE is
      after the retract's end (readAt - maxImageAge >= the retract's end — the read's time is not the
      picture's, HA-192) AND after the lens has moved more than `frozenMoveIn` from where it was at
      the retract's end (belt and braces: the look after begins only once the drive out SETTLED at
      the look spot, and the excursion is more than twice `frozenMoveIn`, so with any settle
      tolerance under the excursion less `frozenMoveIn` this clause cannot refuse a read — the one
      host test that reaches it settles within 11 in of a 12-in excursion) — up to `lookWithin` after
      reaching the look spot. Ours after a FULL stroke:
      FLIPPED. Ours after a short one: OURS AFTER SHORT STROKE. The colour it had before: NO CHANGE (a
      sticky Toggle, or a camera frozen since before the push — both stop). Not sure: UNKNOWN AFTER.
      Push 1 and the THIRD colour: rule 4's push-2 check, then 5 and 6 once more; push 2 not ours:
      STILL NOT OURS. Never a third push.
   7. ONE [TGL] line per visit through the caller's sink — Info for the two successes, Warn otherwise;
      ASCII, at most 160 bytes; every success line ends "seating not checked". The service's own
      [MECH] lines for each push and retract appear beside it.

 ── What it cannot catch (stated, not hidden) ───────────────────────────────────────────────────
 A Toggle stuck part-way that reads as its destination (above); a camera frozen while the robot is
 parked, IF the step were changed to look from where it pushed (rule 6 is why it cannot); a camera
 frozen at the look spot whose stale picture shows the Toggle is ALREADY ours — nothing is driven,
 so nothing can show it frozen (a stale not-ours picture IS caught: the drive in moves the lens, rule
 5); a PASSIVE bar that reaches the Toggle before the lens has moved `frozenMoveIn` on the drive in
 (before a frozen camera CAN look frozen) strokes it on the stale look, and the verdict is still
 FROZEN BEFORE PUSH (host test plant: a bar touching 2 in into a 16-in drive in); a Toggle that changes after the look, while the camera is still live on the drive in (too close
 to name a colour at most mounts); for a PNEUMATIC flipper, whether the cylinder truly reached the
 end of its stroke or truly went back — its slot reports SUCCEEDED once the commanded level has been
 held for its move time (nothing senses the rod), so for a pneumatic "a FULL stroke" and "reached
 `back`" mean "the move time passed": a cylinder blocked part-way by a jammed Toggle still reads as
 FLIPPED's full stroke, and one caught on the Toggle on the way back is driven out with it — OURS
 AFTER A SHORT STROKE and FLIPPER STUCK cannot come from a pneumatic (A4 register HA-235); a lookalike
 band the tracker's gates cannot tell from a face (A4 register HA-199 — the position gate, with a pose
 the team trusts, is what turns those away); a robot touching the Toggle after the step ends; the
 pieces the robot holds (<SG6>: holding more than one Pin or one Cup, it may not touch a Toggle — the
 route's to respect).

 A programming error is LOUD (a precondition): the guard not running; a request that names no Toggle,
 leaves a number UNSET, puts the push spot at or beyond the look spot, an `offBy` of `lookFrom` or
 more, a flipper and a back of two mechanisms or the same goal twice, or `maxPushes` other than 1 or
 2 — refused AT BUILD when the request is built through toggleVisit(...) as a constant; and an
 excursion lookFrom - pushFrom of twice the tracker's `frozenMoveIn` or less (the look after could
 not see a frozen camera).

 ── A MIRRORED auton (chunk MIRROR1-A, 2026-11-13; competition/field_mirror.hpp) ─────────────────
 The route names robot one's Toggle; `entry` says the run is robot two's, mirrored. The step reads the
 localizer and drives the scheduler — below the Chassis's mirror, the REAL field — so it swaps for
 itself: the request's Toggle becomes its IMAGE under the entry's mirror (override2026::mirroredToggle,
 found from the season's places: robot one's red wall Toggle is robot two's right wall Toggle, left
 <-> blue wall), whose Quadrant colour is the same (checked at compile time in override_mirror.hpp), so
 "ours" does not change. Rule 1's WRONG WALL check, the look spot, the [TGL] line — every rule — then
 reads the real Toggle; every other field of the request is a size or a mechanism and stays.

 PROS-free, sim-free, allocation-free; single-task. It never runs in the background and never reads
 the camera outside its own waits. NOTHING HERE HAS RUN ON A ROBOT: every rule has run against the
 host test plant (the drive, the arm and the host Toggle and camera models) and nothing else.
```

</details>
