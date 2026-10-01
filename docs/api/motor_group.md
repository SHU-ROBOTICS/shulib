<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/motor_group.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motor_group.hpp`

MotorGroup — N physically COUPLED motors behind ONE IMotor.

This header declares **1** type (23 members) and **2** free functions.

Extracted from [`include/shulib/hal/motor_group.hpp`](../../include/shulib/hal/motor_group.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`isUsableReading`](#isusablereading) — *free function*
- [`medianInPlace`](#medianinplace) — *free function*
- [`class MotorGroup`](#class-motorgroup)
  - [`kMaxMembers`](#motorgroup-kmaxmembers)
  - [`MotorGroup`](#motorgroup-motorgroup)
  - [`setVoltage`](#motorgroup-setvoltage)
  - [`commandedVoltage`](#motorgroup-commandedvoltage)
  - [`setBrakeMode`](#motorgroup-setbrakemode)
  - [`brakeMode`](#motorgroup-brakemode)
  - [`position`](#motorgroup-position)
  - [`velocity`](#motorgroup-velocity)
  - [`current`](#motorgroup-current)
  - [`temperature`](#motorgroup-temperature)
  - [`totalCurrent`](#motorgroup-totalcurrent)
  - [`nonFiniteReads`](#motorgroup-nonfinitereads)
  - [`evaluateDisagreement`](#motorgroup-evaluatedisagreement)
  - [`disagreeingMembers`](#motorgroup-disagreeingmembers)
  - [`disagreeingMask`](#motorgroup-disagreeingmask)
  - [`lastVerdict`](#motorgroup-lastverdict)
  - [`resetDisagreement`](#motorgroup-resetdisagreement)
  - [`memberCount`](#motorgroup-membercount)
  - [`presentCount`](#motorgroup-presentcount)
  - [`absentMask`](#motorgroup-absentmask)
  - [`isPresent`](#motorgroup-ispresent)
  - [`member`](#motorgroup-member)
  - [`thresholds`](#motorgroup-thresholds)

<a id="isusablereading"></a>

## `isUsableReading`

```cpp
[[nodiscard]] inline bool isUsableReading(double v) noexcept
```

True when `v` is a READING the group's aggregates count: a finite number. NaN (of either sign), +Inf and -Inf are not readings — PROS's failure sentinel is +Inf (A4 register HA-08) — and every aggregate in this header leaves them out, the way it leaves out an ABSENT member (the header's "A READING THAT IS NOT A NUMBER"). This is the ONE test: medianInPlace() excludes with it, MotorGroup::nonFiniteReads() counts with it, and the drivetrain calibrator's "moved on the push" rule is built on it, so the three cannot disagree about what a reading is (DIAG1 Part B). It needs a build that keeps NaN and infinity: a build with -ffinite-math-only or -ffast-math is refused by an #error just above it.

*free function, declared at [`include/shulib/hal/motor_group.hpp:207`](../../include/shulib/hal/motor_group.hpp#L207).*

<a id="medianinplace"></a>

## `medianInPlace`

```cpp
[[nodiscard]] inline double medianInPlace(double* v, std::size_t n)
```

THE median of the FINITE values among the `n` in `v` — the one definition in the tree, so nothing that has to agree with a hal::MotorGroup's reading can drift from it. A value that is not finite (isUsableReading() false: NaN of either sign, ±Inf) is LEFT OUT, the way the group leaves out an absent member; when none is finite the result is a fresh quiet NaN, never one of the inputs. Over the k finite values: odd k, the middle value, no arithmetic at all (k == 1 is the bare value, bit for bit); even k, std::midpoint of the two middle values — exact when they agree, and it cannot overflow where 0.5 * (a + b) could — which one dead member still cannot reach when k >= 4 (the header's "why not the mean"). REORDERS `v`: its contents afterwards are unspecified, which is why the caller passes a scratch array it owns and never reads it again. Precondition: v non-null, n >= 1.  Extracted from MotorGroup at chunk R3d Part 1 because the drivetrain calibrator measures each side's distance scale through the SAME statistic the group will later report to the odometry: a slow member (robot two's port 18, A4 register HA-130) then cancels exactly instead of approximately. A second, privately written median would have made that cancellation an accident rather than a guarantee — and two medians that treated a NaN differently would break it exactly when a reading is bad, which is why the exclusion lives HERE and not in the group.  DIAG1 Part B (2026-09-29) — how, and what it pins. The finite values are copied FORWARD in their input order before the sort, never compacted with std::partition or swap-with-last: std::sort is not stable, so among EQUAL values (+0.0 and -0.0) which one lands in the middle depends on the order it is handed, and a compaction that reordered the finite values would let {+0.0, NaN, -0.0, +0.0} answer differently from the same group with that member absent, {+0.0, -0.0, +0.0}. With every value finite the answer is bit-identical to the version that sorted everything and halved the sum — except where the two middle values' SUM overflowed (possible only when at least one of them is past DBL_MAX/2 in size, so reachable only in host tests), which gave +Inf or -Inf and now gives the finite midpoint. So the median is non-finite exactly when no value is finite.

*free function, declared at [`include/shulib/hal/motor_group.hpp:237`](../../include/shulib/hal/motor_group.hpp#L237).*

<a id="class-motorgroup"></a>

## `class MotorGroup`

```cpp
class MotorGroup final : public IMotor
```

N coupled motors presented as ONE IMotor: commands fan out to every member unchanged; position and velocity read back as the MEDIAN (a dead port's frozen reading cannot drag them until a majority is dead), current as the per-member mean (the sum is totalCurrent()), temperature as the maximum, brake mode as the first member's device read-back. Every aggregate is over the PRESENT members whose reading is FINITE: a member reading NaN or ±Inf is left out of that read (never latched), counted by nonFiniteReads(), and a read with no finite member returns NaN (the header's "A READING THAT IS NOT A NUMBER"). The coupled-side monitor rides inside as the disagreement observable — one evaluation per tick through evaluateDisagreement(), which motion::tickHealthObservables performs for every group it is handed. Raises nothing itself; never negates a member. Non-owning: members and the array the span views must outlive the group.

*class, declared at [`include/shulib/hal/motor_group.hpp:264`](../../include/shulib/hal/motor_group.hpp#L264).*

<a id="motorgroup-kmaxmembers"></a>

### `MotorGroup::kMaxMembers`

```cpp
static constexpr std::size_t kMaxMembers = teleop::kMaxSideMembers
```

The most members one group may carry — a V5 brain has 21 smart ports, and the monitor's bit masks are sized to the same bound.

*field, declared at [`include/shulib/hal/motor_group.hpp:268`](../../include/shulib/hal/motor_group.hpp#L268).*

<a id="motorgroup-motorgroup"></a>

### `MotorGroup::MotorGroup`

```cpp
explicit MotorGroup(std::span<IMotor* const> members, teleop::SideMonitorConfig thresholds = {})
```

`members`: N >= 1 slots in a FIXED order (the bit positions of disagreeingMask() and absentMask() index this span). A `nullptr` slot is an ABSENT member — a drive port found dead at boot (header). `thresholds`: the disagreement detector's knobs — the defaults are the drive program's (INVENTED for a first run; R4 measures).  Precondition: at least ONE member is PRESENT. A group of nothing but holes has no position, no velocity and no temperature to report, and every read below would have to invent one — so it is refused at construction, where a composition root can still paint a refusal, rather than at the first read, mid-match.

*function, declared at [`include/shulib/hal/motor_group.hpp:279`](../../include/shulib/hal/motor_group.hpp#L279).*

<a id="motorgroup-setvoltage"></a>

### `MotorGroup::setVoltage`

```cpp
void setVoltage(units::Voltage volts) override
```

Fan the SAME command to every member (each clamps for itself); the group mirrors the clamped value for commandedVoltage(). Non-finite is rejected here, before any member sees it, so a bad command cannot half-apply.

*function, declared at [`include/shulib/hal/motor_group.hpp:299`](../../include/shulib/hal/motor_group.hpp#L299).*

<a id="motorgroup-commandedvoltage"></a>

### `MotorGroup::commandedVoltage`

```cpp
[[nodiscard]] units::Voltage commandedVoltage() const override
```

The group's last applied command after the ±kMaxMotorVoltage clamp — a local mirror (the ProsMotor convention), never a member read-back. 0 V until the first setVoltage().

*function, declared at [`include/shulib/hal/motor_group.hpp:313`](../../include/shulib/hal/motor_group.hpp#L313).*

<a id="motorgroup-setbrakemode"></a>

### `MotorGroup::setBrakeMode`

```cpp
void setBrakeMode(BrakeMode mode) override
```

Fan the mode to every PRESENT member.

*function, declared at [`include/shulib/hal/motor_group.hpp:316`](../../include/shulib/hal/motor_group.hpp#L316).*

<a id="motorgroup-brakemode"></a>

### `MotorGroup::brakeMode`

```cpp
[[nodiscard]] BrakeMode brakeMode() const override
```

The FIRST PRESENT member's read-back (the device's answer, per the IMotor contract).

*function, declared at [`include/shulib/hal/motor_group.hpp:325`](../../include/shulib/hal/motor_group.hpp#L325).*

<a id="motorgroup-position"></a>

### `MotorGroup::position`

```cpp
[[nodiscard]] units::AngleDim position() const override
```

MEDIAN of the PRESENT members' cumulative shaft rotation (header: why not the mean), over the members whose reading is FINITE; NaN if none is finite. A member left out is counted by nonFiniteReads().

*function, declared at [`include/shulib/hal/motor_group.hpp:330`](../../include/shulib/hal/motor_group.hpp#L330).*

<a id="motorgroup-velocity"></a>

### `MotorGroup::velocity`

```cpp
[[nodiscard]] units::AngularVelocity velocity() const override
```

MEDIAN of the PRESENT members' measured angular velocity, over the members whose reading is FINITE; NaN if none is finite.

*function, declared at [`include/shulib/hal/motor_group.hpp:336`](../../include/shulib/hal/motor_group.hpp#L336).*

<a id="motorgroup-current"></a>

### `MotorGroup::current`

```cpp
[[nodiscard]] units::Current current() const override
```

MEAN current PER PRESENT MEMBER, so per-motor thresholds keep their meaning. An ABSENT member is not a member drawing 0 A: dividing by the SLOT count would make a side with a dead port read cooler than it is. Neither is a member whose reading is not finite: it is left out of the sum AND of the count it is divided by (header: "A READING THAT IS NOT A NUMBER"). NaN if no present member's reading is finite. The sum is totalCurrent().

*function, declared at [`include/shulib/hal/motor_group.hpp:346`](../../include/shulib/hal/motor_group.hpp#L346).*

<a id="motorgroup-temperature"></a>

### `MotorGroup::temperature`

```cpp
[[nodiscard]] double temperature() const override
```

MAX temperature (°C) over the PRESENT members whose reading is FINITE: the hottest member throttles first. A member reading +Inf °C is left out, not taken as the hottest — so on its own it no longer raises MOTOR_OVER_TEMP — and counted by nonFiniteReads(); a NaN member can no longer win by standing first (std::max keeps its first argument against a NaN, so the old answer depended on member order). NaN if no present member's reading is finite. Each member is read ONCE (until DIAG1 Part B the first present member was read twice, once as the seed).

*function, declared at [`include/shulib/hal/motor_group.hpp:359`](../../include/shulib/hal/motor_group.hpp#L359).*

<a id="motorgroup-totalcurrent"></a>

### `MotorGroup::totalCurrent`

```cpp
[[nodiscard]] units::Current totalCurrent() const
```

The SUM of the current draw (amperes) of every PRESENT member whose reading is FINITE — the side's total, exposed outside the IMotor surface so current() can stay per-member. NaN if no present member's reading is finite: an empty sum would read 0 A, a side drawing nothing, which is a claim no member made.

*function, declared at [`include/shulib/hal/motor_group.hpp:381`](../../include/shulib/hal/motor_group.hpp#L381).*

<a id="motorgroup-nonfinitereads"></a>

### `MotorGroup::nonFiniteReads`

```cpp
[[nodiscard]] int nonFiniteReads() const noexcept
```

How many member READINGS this group has left out for not being finite (NaN of either sign, +Inf, -Inf) — one per member per read, over position(), velocity(), current(), totalCurrent() and temperature() alike; cumulative for the group's life, never reset. It makes the exclusion OBSERVABLE, as absentMask() does a hole: a member left out in silence would be a reading nobody knows is missing. Counted with the exclusion's own test (isUsableReading()), so the two cannot drift. Mirrors hal::pros::ProsMotor::faultedReads(), including its `mutable` counter bumped by a const read. evaluateDisagreement()'s velocity samples are NOT counted: what the monitor does with a non-finite member velocity is its own, later change. The group raises nothing on this number (raising is the loop layer's policy, header) and nothing in the library reads it yet. On a V5 it is expected to stay 0 — ProsMotor screens a non-finite read to the last good one first (A4 register HA-08).

*function, declared at [`include/shulib/hal/motor_group.hpp:397`](../../include/shulib/hal/motor_group.hpp#L397).*

<a id="motorgroup-evaluatedisagreement"></a>

### `MotorGroup::evaluateDisagreement`

```cpp
const teleop::SideVerdict& evaluateDisagreement()
```

One tick of the disagreement observable: samples every member's velocity() against the group's commanded voltage through the shared coupled-side monitor, advancing its persistence counters ONCE. Call exactly once per control tick — the health tick (motion::tickHealthObservables) does. Returns the verdict it also stores.

*function, declared at [`include/shulib/hal/motor_group.hpp:403`](../../include/shulib/hal/motor_group.hpp#L403).*

<a id="motorgroup-disagreeingmembers"></a>

### `MotorGroup::disagreeingMembers`

```cpp
[[nodiscard]] int disagreeingMembers() const noexcept
```

Members that have disagreed with the group for the full persistence window, as of the last evaluateDisagreement() (0 = healthy, or never evaluated).

*function, declared at [`include/shulib/hal/motor_group.hpp:421`](../../include/shulib/hal/motor_group.hpp#L421).*

<a id="motorgroup-disagreeingmask"></a>

### `MotorGroup::disagreeingMask`

```cpp
[[nodiscard]] std::uint32_t disagreeingMask() const noexcept
```

Bit i set = member i (this span's index) is PERSISTENTLY disagreeing.

*function, declared at [`include/shulib/hal/motor_group.hpp:424`](../../include/shulib/hal/motor_group.hpp#L424).*

<a id="motorgroup-lastverdict"></a>

### `MotorGroup::lastVerdict`

```cpp
[[nodiscard]] const teleop::SideVerdict& lastVerdict() const noexcept
```

The whole verdict of the last evaluateDisagreement() — for a panel that wants the this-tick mask, the fastest member and the present count as well.

*function, declared at [`include/shulib/hal/motor_group.hpp:428`](../../include/shulib/hal/motor_group.hpp#L428).*

<a id="motorgroup-resetdisagreement"></a>

### `MotorGroup::resetDisagreement`

```cpp
void resetDisagreement() noexcept
```

Clear the monitor's streaks and the stored verdict (a re-arm after a cut, or a new-run boundary).

*function, declared at [`include/shulib/hal/motor_group.hpp:432`](../../include/shulib/hal/motor_group.hpp#L432).*

<a id="motorgroup-membercount"></a>

### `MotorGroup::memberCount`

```cpp
[[nodiscard]] std::size_t memberCount() const noexcept
```

How many SLOTS the group has — present and absent together, so a panel's port list keeps its indices when a member goes missing. presentCount() is how many answer.

*function, declared at [`include/shulib/hal/motor_group.hpp:439`](../../include/shulib/hal/motor_group.hpp#L439).*

<a id="motorgroup-presentcount"></a>

### `MotorGroup::presentCount`

```cpp
[[nodiscard]] std::size_t presentCount() const noexcept
```

How many members are PRESENT (not a nullptr slot). Always >= 1.

*function, declared at [`include/shulib/hal/motor_group.hpp:442`](../../include/shulib/hal/motor_group.hpp#L442).*

<a id="motorgroup-absentmask"></a>

### `MotorGroup::absentMask`

```cpp
[[nodiscard]] std::uint32_t absentMask() const noexcept
```

Bit i set = slot i is ABSENT. 0 = a complete group — the state every graph built from a healthy robot is in, and the one the whole pre-existing suite pins.

*function, declared at [`include/shulib/hal/motor_group.hpp:446`](../../include/shulib/hal/motor_group.hpp#L446).*

<a id="motorgroup-ispresent"></a>

### `MotorGroup::isPresent`

```cpp
[[nodiscard]] bool isPresent(std::size_t i) const noexcept
```

True when slot `i` is present. Out of range is false, so a panel walking a fixed-size array cannot fall off the end.

*function, declared at [`include/shulib/hal/motor_group.hpp:450`](../../include/shulib/hal/motor_group.hpp#L450).*

<a id="motorgroup-member"></a>

### `MotorGroup::member`

```cpp
[[nodiscard]] IMotor& member(std::size_t i) const
```

Member `i` (0-based, the span's order). Preconditions: i < memberCount() AND slot i is PRESENT. Asking an absent slot for its motor is a caller bug — there is no object to return and no sentinel that would not be a lie — so it is a precondition violation rather than a reference to nothing.

*function, declared at [`include/shulib/hal/motor_group.hpp:458`](../../include/shulib/hal/motor_group.hpp#L458).*

<a id="motorgroup-thresholds"></a>

### `MotorGroup::thresholds`

```cpp
[[nodiscard]] const teleop::SideMonitorConfig& thresholds() const noexcept
```

The thresholds the disagreement observable runs under.

*function, declared at [`include/shulib/hal/motor_group.hpp:467`](../../include/shulib/hal/motor_group.hpp#L467).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 166 lines, click to expand</summary>

```text

 MotorGroup — N physically COUPLED motors behind ONE IMotor (chunk R3b Part 1, 2026-09-14).

 ── Why this exists ─────────────────────────────────────────────────────────────────
 A real VEX drivetrain has several motors per side on one gear train. The kinematics
 layer sees ONE wheel per side (kinematics/tank.hpp:80: "how many physical motors sit
 on each side is the HAL's business"), and the command pipeline maps kinematic wheel →
 motor 1:1 (motion/command_pipeline.hpp step 7). Until this class existed, no HAL
 facility answered the kinematics' delegation: a context built with N motors per side
 was ACCEPTED by MotionDeps::validate()'s old `>=` guard, two motors were commanded and
 the rest were never given a voltage or a brake mode, silently (roadmap: "What R3b must
 BUILD", item 1; measured on the bench bot's then-7-motor drive). This class is that
 facility, and MotionDeps::validate() now demands EQUALITY so the defect cannot recur:
 with `driveMotors = {&leftGroup, &rightGroup}` the 1:1 mapping is CORRECT by
 construction, which is the whole point of the group.

 ── Command path: fan-out, nothing else ─────────────────────────────────────────────
 setVoltage(v) and setBrakeMode(m) go to every member, the SAME value. Clamping is each
 member's own; commandedVoltage() is the group's local mirror of the last applied value
 (the ProsMotor convention). THE GROUP NEVER NEGATES: a member that must turn the other
 way is constructed with a NEGATIVE port, and PROS applies that reversal exactly once,
 in the adapter (hal/pros/motor.hpp header). One place for a sign, or the sign gets
 applied twice (R3b Session 2 §2.3, landmine 4).

 ── Read path: MEDIAN, not mean — and why ───────────────────────────────────────────
 position() and velocity() are the MEDIAN across members. Members are coupled, so they
 agree to within noise when healthy; the question is what happens when one is not. A
 member whose port stopped answering reports its LAST GOOD position forever (ProsMotor's
 sentinel screen — the IMotor class doc in hal/motor.hpp, ~:60-67). A MEAN would then drift
 by 1/N of all further travel — silently wrong odometry, exactly the failure the odometry
 cross-checks cannot see because the number keeps moving. A median is untouched until a
 MAJORITY fails.
 Rejected: mean (above); "first member" (one dead port kills the side with no signal at
 all). For an even N the median is the mean of the two middle values, which one dead
 member still cannot reach when N >= 4; for N == 2 no aggregate can tell the live member
 from the dead one, and the median degrades to a half-drag — stated, not hidden.
 (Robot two is N = 5 per side; the bench bot N = 4.)

 current() is the MEAN per member, so the tree's per-motor stall/capture thresholds keep
 their meaning (a member at 2.4 A reads as a member at 2.4 A); the SUM is exposed as the
 separate, non-IMotor totalCurrent(). temperature() is the MAX: the hottest member
 throttles first, and the thermal monitor must see the worst one. brakeMode() is the
 FIRST member's read-back (a device answer, not the library's memory — the IMotor
 contract), with a member that disagrees with it counted by the observable below.

 ── A READING THAT IS NOT A NUMBER (DIAG1 Part B, 2026-09-29) ───────────────────────
 A member whose reading is not finite — NaN of either sign, +Inf or -Inf — is LEFT OUT of that
 read, the way an ABSENT member (below) is left out of every read: position() and velocity()
 are the median of the present members whose reading is finite, current() the mean and
 totalCurrent() the sum over them, temperature() their max. The rule lives in ONE place,
 isUsableReading(): medianInPlace() excludes with it and nonFiniteReads() counts with it.
  * PER READ, NEVER A LATCH. The same member's next finite reading is counted again;
    presentCount() and absentMask() never change for it. A member that is ALWAYS unreadable is
    a dead port, and that is the absent member's business, decided at boot.
  * NaN WHEN NONE IS FINITE. When no present member gives a finite reading the read returns a
    fresh quiet NaN (std::numeric_limits<double>::quiet_NaN()), never one of its inputs, so the
    answer does not depend on which non-finite values arrived ({+Inf, -Inf} used to give a
    NEGATIVE NaN). The drive-encoder odometry already handles a NaN side: that tick freezes
    position and the next finite tick integrates across it. motion::OdoStallCheck, which reads
    the drive motors when a window closes, gets a NaN spin travel from such a side and gives NO
    verdict for that window, nor for the next (its new baseline is the NaN) — where a side all
    +Inf or all -Inf used to read as an infinite travel and, on a drivetrain with an independent
    motion source, a stall (ODO_STUCK). What the check should do with a travel it cannot read
    is a follow-up's decision.
  * WHY ±Inf TOO, NOT ONLY NaN. +Inf is PROS's own failure sentinel (PROS_ERR_F, A4 register
    HA-08): it means "no reading", not a position. Kept in, one infinite member of three or
    more still moves the median by half a neighbour's gap, and on a side of one or two members
    it makes the whole side non-finite.
  * WHY NOT A LAST-GOOD CACHE HERE. The adapter already holds last-good (ProsMotor's screen);
    a second cache in the group would be a frozen member feeding the median — the drag this
    median exists to survive — and a latch the absent-member rule does not have.
  * WHY NOT A MAJORITY RULE (NaN once half or more are non-finite — the DIAG1 research's other
    option). The recommendation the team lead approved (2026-09-26) chose exclusion: one rule
    for "no reading", whether the port died at boot or one read failed.
  * THE MEMBERSHIP STEP (A4 register HA-165). Leaving members out changes WHICH values the
    median is taken over, so the side's reading can step on the tick a member drops out, and
    step back on the tick it returns, with nothing moving. For ONE member left out: of an odd
    count (three or more), the median moves to the midpoint of the two values then in the
    middle — the middle value and a neighbour, or the middle value's two neighbours when the
    member left out IS the middle one — by at most HALF the larger of the two gaps beside the
    middle value; of an even count, it moves from the midpoint to one of the two middle
    values — by half the gap between them (for a two-member side, half the gap between the
    two). With SEVERAL left out, the median of the rest can land anywhere between the lowest
    and the highest member position, so the step is bounded only by the members' SPREAD
    (highest minus lowest); the worst case is the one member farthest from the full median,
    read alone. Coupled members sit close together, so the step is small — unless their
    CUMULATIVE positions have drifted apart: robot two's port 18 travelled ~20 % short on a
    hand push (HA-130; unmeasured under power), so its offset would grow with the distance
    driven. The odometry reads the step as travel; its trust gate bounds one tick at
    maxTickTravel (HA-123), and the return tick carries the step back. A side whose infinite
    members used to reach the middle — two +Inf of three, say — read non-finite and froze its
    tick as NONFINITE+TRAVEL; it now reads the rest's median and integrates, so a step under
    maxTickTravel (36 in by default) moves the pose SILENTLY (DIAG1 N7 pins that, and TRAVEL
    past the bound).
  * THE UNDEFINED BEHAVIOUR IT REMOVES. The median used to std::sort every value, and a NaN
    breaks the strict weak ordering std::sort requires. Probed against this header before the
    change (host x86-64, g++ 13.3.0 libstdc++, -O2; the robot's library was not probed):
    {3, NaN, 1, 2} gave NaN; {NaN, 1, 2, 3, 4} gave 2 where the finite median is 2.5;
    {1, 2, NaN, 3, 4} gave NaN while {1, 2, 3, 4, NaN} gave 3; 21 members with 1-3 NaN gave a
    WRONG FINITE value on 18,030 of 20,000 draws and NaN on 1,350 — a plausible wrong number,
    decided by which slot was bad.
  * NOT A ROBOT PATH TODAY. ProsMotor screens a non-finite read to the last good one before any
    group sees it (HA-08), so on a V5 this exclusion is expected never to fire. It defines what
    the library does when a reading does get through — a future adapter, a nested group, a host
    test. Nothing here has run on a robot.

 ── The disagreement observable: ONE evaluator, shared with the drive program ───────
 The fighting-motor / frozen-member detector is teleop::CoupledSideMonitor — the pure,
 host-tested evaluator "shulib Drive" and the tester's DRIVE station already run (R3b
 Part 0b). This class REUSES it rather than writing a second one: same thresholds, same
 persistence window, same meaning of "disagreeing" (a member opposite in sign to the
 command above 0.5 rad/s, or under 25 % of the group's fastest, while the group is
 commanded above 1 V and turning above 1 rad/s, for 25 consecutive ticks). Because the
 monitor COUNTS PERSISTENCE, it must be evaluated exactly ONCE per control tick, not on
 every read: evaluateDisagreement() is that tick, and motion::tickHealthObservables()
 calls it for every group in MotionDeps::motorGroups — so it runs in Chassis::drive()
 and in every motion with no extra caller wiring. (The Session 2 brief said "evaluated
 on each read"; a stateful persistence counter cannot be, and a read-side evaluation
 would advance the window once per position()/velocity()/temperature() call. Recorded
 in the Parts 1–3 log.)

 The group RAISES NO FAULT — raising is the loop layer's policy, exactly as
 hal/pros/motor.hpp says of the adapter. diag::HealthMonitor turns a persisted
 disagreement into FaultCode::MotorGroupDisagree (appended, wire-stable), and the
 scheduler's default abort mask leaves it on the CONTINUE side: the drive still works
 with one bad member, and the driver must be TOLD, not stopped. The alternative — abort
 on a fight — was rejected because a match must not end on a member that is merely
 slow (robot two's port 18 travels ~20 % short of its side-mates on every push and is
 NOT a fight — A4 register HA-130; test 16 pins that it is tolerated and that an opposing
 member is not — the disagreement FLOOR, A4 register HA-133, invented with the rest of the
 thresholds, HA-132). Configure the mask to abort if a season's data says otherwise.

 ── THE ABSENT MEMBER (chunk R3d Part 2, ruling P4) ─────────────────────────────────
 A `nullptr` in the members span is an ABSENT member: a drive port the composition root
 found DEAD at boot. The group's contract was "every member is PRESENT"; it is now "AT
 LEAST ONE member is present", and every read below says which members it is over.

 WHY A HOLE IN THE SPAN, AND NOT A NullMotor OBJECT: a fake in a competition binary is
 test scaffolding (R3b §6's rule), and a NullMotor would have to answer position() and
 velocity() with SOMETHING — and every answer is a lie the median would then average in.
 A hole cannot be read by accident.

 WHY BOOT, AND NOT RUNTIME: a dead port is known before the graph exists. The drive program
 finds one by catching hal/pros/motor.hpp's constructor throw per port (its HA-98 read-back,
 kept as std::optional<ProsMotor> in its `Member` shape) and drives the rest itself, without
 this class. The graphs that DO use this class — robot::Robot (`BuildOptions::absentDrivePorts`,
 since G1 Part 2a), which "shulib Teleop" is built on since G1 Part 2b (until then its
 hand-wired `TankMotors` in src/main.cpp did the same) — take the MISSING set
 calibration::boot_check read from the brain's own registry, never construct those ports, and
 hand this class a span with a hole where each one would be. Runtime death INSIDE the library
 graph — promoting the drive program's MemberAbsenceDetector in here — is a later chunk.

 WHAT CHANGES, AND WHAT DOES NOT: setVoltage/setBrakeMode SKIP the holes; position(),
 velocity(), current() and temperature() are over the PRESENT members only; the monitor is
 told `present = false` for a hole and never flags it; memberCount() still reports how many
 SLOTS the group has (so the panel's port list keeps its indices) and presentCount() how
 many answer. ALL-PRESENT GROUPS ARE BIT-IDENTICAL — not "equivalent": the whole
 pre-existing suite is the pin, and its assertion count was recorded before and after this
 change, with `MotorGroup 8b` and the plant's case 7 as the local half of it. A PRESENT
 member whose reading is not finite (above) is left out of that one read and moves neither
 count: it is still a slot, still present, still commanded.

 Non-owning: the pointer ARRAY the span views, and the motors it points to, must
 outlive the group — the tree's ownership convention (the IMotor destructor's doc in
 hal/motor.hpp, ~:70-78).
 PROS-free; host-tested against FakeMotors and through the A2 plant's coupled members.
```

</details>
