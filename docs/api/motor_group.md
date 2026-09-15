<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/motor_group.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motor_group.hpp`

MotorGroup — N physically COUPLED motors behind ONE IMotor.

This header declares **1** type (19 members).

Extracted from [`include/shulib/hal/motor_group.hpp`](../../include/shulib/hal/motor_group.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

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
  - [`evaluateDisagreement`](#motorgroup-evaluatedisagreement)
  - [`disagreeingMembers`](#motorgroup-disagreeingmembers)
  - [`disagreeingMask`](#motorgroup-disagreeingmask)
  - [`lastVerdict`](#motorgroup-lastverdict)
  - [`resetDisagreement`](#motorgroup-resetdisagreement)
  - [`memberCount`](#motorgroup-membercount)
  - [`member`](#motorgroup-member)
  - [`thresholds`](#motorgroup-thresholds)

<a id="class-motorgroup"></a>

## `class MotorGroup`

```cpp
class MotorGroup final : public IMotor
```

N coupled motors presented as ONE IMotor: commands fan out to every member unchanged; position and velocity read back as the MEDIAN (a dead port's frozen reading cannot drag them until a majority is dead), current as the per-member mean (the sum is totalCurrent()), temperature as the maximum, brake mode as the first member's device read-back. The coupled-side monitor rides inside as the disagreement observable — one evaluation per tick through evaluateDisagreement(), which motion::tickHealthObservables performs for every group it is handed. Raises nothing itself; never negates a member. Non-owning: members and the array the span views must outlive the group.

*class, declared at [`include/shulib/hal/motor_group.hpp:101`](../../include/shulib/hal/motor_group.hpp#L101).*

<a id="motorgroup-kmaxmembers"></a>

### `MotorGroup::kMaxMembers`

```cpp
static constexpr std::size_t kMaxMembers = teleop::kMaxSideMembers
```

The most members one group may carry — a V5 brain has 21 smart ports, and the monitor's bit masks are sized to the same bound.

*field, declared at [`include/shulib/hal/motor_group.hpp:105`](../../include/shulib/hal/motor_group.hpp#L105).*

<a id="motorgroup-motorgroup"></a>

### `MotorGroup::MotorGroup`

```cpp
explicit MotorGroup(std::span<IMotor* const> members, teleop::SideMonitorConfig thresholds = {})
```

`members`: N >= 1 present adapters, all non-null, in a FIXED order (the bit positions of disagreeingMask() index this span). `thresholds`: the disagreement detector's knobs — the defaults are the drive program's (INVENTED for a first run; R4 measures).

*function, declared at [`include/shulib/hal/motor_group.hpp:110`](../../include/shulib/hal/motor_group.hpp#L110).*

<a id="motorgroup-setvoltage"></a>

### `MotorGroup::setVoltage`

```cpp
void setVoltage(units::Voltage volts) override
```

Fan the SAME command to every member (each clamps for itself); the group mirrors the clamped value for commandedVoltage(). Non-finite is rejected here, before any member sees it, so a bad command cannot half-apply.

*function, declared at [`include/shulib/hal/motor_group.hpp:123`](../../include/shulib/hal/motor_group.hpp#L123).*

<a id="motorgroup-commandedvoltage"></a>

### `MotorGroup::commandedVoltage`

```cpp
[[nodiscard]] units::Voltage commandedVoltage() const override
```

The group's last applied command after the ±kMaxMotorVoltage clamp — a local mirror (the ProsMotor convention), never a member read-back. 0 V until the first setVoltage().

*function, declared at [`include/shulib/hal/motor_group.hpp:135`](../../include/shulib/hal/motor_group.hpp#L135).*

<a id="motorgroup-setbrakemode"></a>

### `MotorGroup::setBrakeMode`

```cpp
void setBrakeMode(BrakeMode mode) override
```

Fan the mode to every member.

*function, declared at [`include/shulib/hal/motor_group.hpp:138`](../../include/shulib/hal/motor_group.hpp#L138).*

<a id="motorgroup-brakemode"></a>

### `MotorGroup::brakeMode`

```cpp
[[nodiscard]] BrakeMode brakeMode() const override
```

The FIRST member's read-back (the device's answer, per the IMotor contract).

*function, declared at [`include/shulib/hal/motor_group.hpp:145`](../../include/shulib/hal/motor_group.hpp#L145).*

<a id="motorgroup-position"></a>

### `MotorGroup::position`

```cpp
[[nodiscard]] units::AngleDim position() const override
```

MEDIAN of the members' cumulative shaft rotation (header: why not the mean).

*function, declared at [`include/shulib/hal/motor_group.hpp:148`](../../include/shulib/hal/motor_group.hpp#L148).*

<a id="motorgroup-velocity"></a>

### `MotorGroup::velocity`

```cpp
[[nodiscard]] units::AngularVelocity velocity() const override
```

MEDIAN of the members' measured angular velocity.

*function, declared at [`include/shulib/hal/motor_group.hpp:153`](../../include/shulib/hal/motor_group.hpp#L153).*

<a id="motorgroup-current"></a>

### `MotorGroup::current`

```cpp
[[nodiscard]] units::Current current() const override
```

MEAN current PER MEMBER, so per-motor thresholds keep their meaning; the sum is totalCurrent().

*function, declared at [`include/shulib/hal/motor_group.hpp:160`](../../include/shulib/hal/motor_group.hpp#L160).*

<a id="motorgroup-temperature"></a>

### `MotorGroup::temperature`

```cpp
[[nodiscard]] double temperature() const override
```

MAX member temperature (°C): the hottest member throttles first.

*function, declared at [`include/shulib/hal/motor_group.hpp:165`](../../include/shulib/hal/motor_group.hpp#L165).*

<a id="motorgroup-totalcurrent"></a>

### `MotorGroup::totalCurrent`

```cpp
[[nodiscard]] units::Current totalCurrent() const
```

The SUM of every member's current draw (amperes) — the side's total, exposed outside the IMotor surface so current() can stay per-member.

*function, declared at [`include/shulib/hal/motor_group.hpp:175`](../../include/shulib/hal/motor_group.hpp#L175).*

<a id="motorgroup-evaluatedisagreement"></a>

### `MotorGroup::evaluateDisagreement`

```cpp
const teleop::SideVerdict& evaluateDisagreement()
```

One tick of the disagreement observable: samples every member's velocity() against the group's commanded voltage through the shared coupled-side monitor, advancing its persistence counters ONCE. Call exactly once per control tick — the health tick (motion::tickHealthObservables) does. Returns the verdict it also stores.

*function, declared at [`include/shulib/hal/motor_group.hpp:181`](../../include/shulib/hal/motor_group.hpp#L181).*

<a id="motorgroup-disagreeingmembers"></a>

### `MotorGroup::disagreeingMembers`

```cpp
[[nodiscard]] int disagreeingMembers() const noexcept
```

Members that have disagreed with the group for the full persistence window, as of the last evaluateDisagreement() (0 = healthy, or never evaluated).

*function, declared at [`include/shulib/hal/motor_group.hpp:194`](../../include/shulib/hal/motor_group.hpp#L194).*

<a id="motorgroup-disagreeingmask"></a>

### `MotorGroup::disagreeingMask`

```cpp
[[nodiscard]] std::uint32_t disagreeingMask() const noexcept
```

Bit i set = member i (this span's index) is PERSISTENTLY disagreeing.

*function, declared at [`include/shulib/hal/motor_group.hpp:197`](../../include/shulib/hal/motor_group.hpp#L197).*

<a id="motorgroup-lastverdict"></a>

### `MotorGroup::lastVerdict`

```cpp
[[nodiscard]] const teleop::SideVerdict& lastVerdict() const noexcept
```

The whole verdict of the last evaluateDisagreement() — for a panel that wants the this-tick mask, the fastest member and the present count as well.

*function, declared at [`include/shulib/hal/motor_group.hpp:201`](../../include/shulib/hal/motor_group.hpp#L201).*

<a id="motorgroup-resetdisagreement"></a>

### `MotorGroup::resetDisagreement`

```cpp
void resetDisagreement() noexcept
```

Clear the monitor's streaks and the stored verdict (a re-arm after a cut, or a new-run boundary).

*function, declared at [`include/shulib/hal/motor_group.hpp:205`](../../include/shulib/hal/motor_group.hpp#L205).*

<a id="motorgroup-membercount"></a>

### `MotorGroup::memberCount`

```cpp
[[nodiscard]] std::size_t memberCount() const noexcept
```

How many members the group commands.

*function, declared at [`include/shulib/hal/motor_group.hpp:211`](../../include/shulib/hal/motor_group.hpp#L211).*

<a id="motorgroup-member"></a>

### `MotorGroup::member`

```cpp
[[nodiscard]] IMotor& member(std::size_t i) const
```

Member `i` (0-based, the span's order). Precondition: i < memberCount().

*function, declared at [`include/shulib/hal/motor_group.hpp:214`](../../include/shulib/hal/motor_group.hpp#L214).*

<a id="motorgroup-thresholds"></a>

### `MotorGroup::thresholds`

```cpp
[[nodiscard]] const teleop::SideMonitorConfig& thresholds() const noexcept
```

The thresholds the disagreement observable runs under.

*function, declared at [`include/shulib/hal/motor_group.hpp:220`](../../include/shulib/hal/motor_group.hpp#L220).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 77 lines, click to expand</summary>

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
 sentinel screen, hal/motor.hpp:47-54). A MEAN would then drift by 1/N of all further
 travel — silently wrong odometry, exactly the failure the odometry cross-checks cannot
 see because the number keeps moving. A median is untouched until a MAJORITY fails.
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

 Every member is PRESENT by contract: the library graph requires N >= 1 constructed
 adapters and has no dead-port tolerance (that is "shulib Drive"'s job until R3d), so
 the monitor's `present` flag is always true here.

 Non-owning: the pointer ARRAY the span views, and the motors it points to, must
 outlive the group — the tree's ownership convention (hal/motor.hpp:57-65).
 PROS-free; host-tested against FakeMotors and through the A2 plant's coupled members.
```

</details>
