<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/motor_group.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motor_group.hpp`

MotorGroup — N physically COUPLED motors behind ONE IMotor.

This header declares **1** type (22 members) and **1** free function.

Extracted from [`include/shulib/hal/motor_group.hpp`](../../include/shulib/hal/motor_group.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

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

<a id="medianinplace"></a>

## `medianInPlace`

```cpp
[[nodiscard]] inline double medianInPlace(double* v, std::size_t n)
```

THE median of the `n` values in `v` — the one definition in the tree, so nothing that has to agree with a hal::MotorGroup's reading can drift from it. Odd n: the middle value, no arithmetic at all (n == 1 is the bare value, bit for bit). Even n: the mean of the two middle values (exact when they agree), which one dead member still cannot reach when n >= 4 — see the header's "why not the mean". SORTS `v` IN PLACE, which is why the caller passes a scratch array it owns. Precondition: v non-null, n >= 1.  Extracted from MotorGroup at chunk R3d Part 1 because the drivetrain calibrator measures each side's distance scale through the SAME statistic the group will later report to the odometry: a slow member (robot two's port 18, A4 register HA-130) then cancels exactly instead of approximately. A second, privately written median would have made that cancellation an accident rather than a guarantee.

*free function, declared at [`include/shulib/hal/motor_group.hpp:128`](../../include/shulib/hal/motor_group.hpp#L128).*

<a id="class-motorgroup"></a>

## `class MotorGroup`

```cpp
class MotorGroup final : public IMotor
```

N coupled motors presented as ONE IMotor: commands fan out to every member unchanged; position and velocity read back as the MEDIAN (a dead port's frozen reading cannot drag them until a majority is dead), current as the per-member mean (the sum is totalCurrent()), temperature as the maximum, brake mode as the first member's device read-back. The coupled-side monitor rides inside as the disagreement observable — one evaluation per tick through evaluateDisagreement(), which motion::tickHealthObservables performs for every group it is handed. Raises nothing itself; never negates a member. Non-owning: members and the array the span views must outlive the group.

*class, declared at [`include/shulib/hal/motor_group.hpp:143`](../../include/shulib/hal/motor_group.hpp#L143).*

<a id="motorgroup-kmaxmembers"></a>

### `MotorGroup::kMaxMembers`

```cpp
static constexpr std::size_t kMaxMembers = teleop::kMaxSideMembers
```

The most members one group may carry — a V5 brain has 21 smart ports, and the monitor's bit masks are sized to the same bound.

*field, declared at [`include/shulib/hal/motor_group.hpp:147`](../../include/shulib/hal/motor_group.hpp#L147).*

<a id="motorgroup-motorgroup"></a>

### `MotorGroup::MotorGroup`

```cpp
explicit MotorGroup(std::span<IMotor* const> members, teleop::SideMonitorConfig thresholds = {})
```

`members`: N >= 1 slots in a FIXED order (the bit positions of disagreeingMask() and absentMask() index this span). A `nullptr` slot is an ABSENT member — a drive port found dead at boot (header). `thresholds`: the disagreement detector's knobs — the defaults are the drive program's (INVENTED for a first run; R4 measures).  Precondition: at least ONE member is PRESENT. A group of nothing but holes has no position, no velocity and no temperature to report, and every read below would have to invent one — so it is refused at construction, where a composition root can still paint a refusal, rather than at the first read, mid-match.

*function, declared at [`include/shulib/hal/motor_group.hpp:158`](../../include/shulib/hal/motor_group.hpp#L158).*

<a id="motorgroup-setvoltage"></a>

### `MotorGroup::setVoltage`

```cpp
void setVoltage(units::Voltage volts) override
```

Fan the SAME command to every member (each clamps for itself); the group mirrors the clamped value for commandedVoltage(). Non-finite is rejected here, before any member sees it, so a bad command cannot half-apply.

*function, declared at [`include/shulib/hal/motor_group.hpp:178`](../../include/shulib/hal/motor_group.hpp#L178).*

<a id="motorgroup-commandedvoltage"></a>

### `MotorGroup::commandedVoltage`

```cpp
[[nodiscard]] units::Voltage commandedVoltage() const override
```

The group's last applied command after the ±kMaxMotorVoltage clamp — a local mirror (the ProsMotor convention), never a member read-back. 0 V until the first setVoltage().

*function, declared at [`include/shulib/hal/motor_group.hpp:192`](../../include/shulib/hal/motor_group.hpp#L192).*

<a id="motorgroup-setbrakemode"></a>

### `MotorGroup::setBrakeMode`

```cpp
void setBrakeMode(BrakeMode mode) override
```

Fan the mode to every PRESENT member.

*function, declared at [`include/shulib/hal/motor_group.hpp:195`](../../include/shulib/hal/motor_group.hpp#L195).*

<a id="motorgroup-brakemode"></a>

### `MotorGroup::brakeMode`

```cpp
[[nodiscard]] BrakeMode brakeMode() const override
```

The FIRST PRESENT member's read-back (the device's answer, per the IMotor contract).

*function, declared at [`include/shulib/hal/motor_group.hpp:204`](../../include/shulib/hal/motor_group.hpp#L204).*

<a id="motorgroup-position"></a>

### `MotorGroup::position`

```cpp
[[nodiscard]] units::AngleDim position() const override
```

MEDIAN of the PRESENT members' cumulative shaft rotation (header: why not the mean).

*function, declared at [`include/shulib/hal/motor_group.hpp:207`](../../include/shulib/hal/motor_group.hpp#L207).*

<a id="motorgroup-velocity"></a>

### `MotorGroup::velocity`

```cpp
[[nodiscard]] units::AngularVelocity velocity() const override
```

MEDIAN of the PRESENT members' measured angular velocity.

*function, declared at [`include/shulib/hal/motor_group.hpp:212`](../../include/shulib/hal/motor_group.hpp#L212).*

<a id="motorgroup-current"></a>

### `MotorGroup::current`

```cpp
[[nodiscard]] units::Current current() const override
```

MEAN current PER PRESENT MEMBER, so per-motor thresholds keep their meaning. An ABSENT member is not a member drawing 0 A: dividing by the SLOT count would make a side with a dead port read cooler than it is. The sum is totalCurrent().

*function, declared at [`include/shulib/hal/motor_group.hpp:220`](../../include/shulib/hal/motor_group.hpp#L220).*

<a id="motorgroup-temperature"></a>

### `MotorGroup::temperature`

```cpp
[[nodiscard]] double temperature() const override
```

MAX PRESENT member temperature (°C): the hottest member throttles first.

*function, declared at [`include/shulib/hal/motor_group.hpp:225`](../../include/shulib/hal/motor_group.hpp#L225).*

<a id="motorgroup-totalcurrent"></a>

### `MotorGroup::totalCurrent`

```cpp
[[nodiscard]] units::Current totalCurrent() const
```

The SUM of every PRESENT member's current draw (amperes) — the side's total, exposed outside the IMotor surface so current() can stay per-member.

*function, declared at [`include/shulib/hal/motor_group.hpp:237`](../../include/shulib/hal/motor_group.hpp#L237).*

<a id="motorgroup-evaluatedisagreement"></a>

### `MotorGroup::evaluateDisagreement`

```cpp
const teleop::SideVerdict& evaluateDisagreement()
```

One tick of the disagreement observable: samples every member's velocity() against the group's commanded voltage through the shared coupled-side monitor, advancing its persistence counters ONCE. Call exactly once per control tick — the health tick (motion::tickHealthObservables) does. Returns the verdict it also stores.

*function, declared at [`include/shulib/hal/motor_group.hpp:243`](../../include/shulib/hal/motor_group.hpp#L243).*

<a id="motorgroup-disagreeingmembers"></a>

### `MotorGroup::disagreeingMembers`

```cpp
[[nodiscard]] int disagreeingMembers() const noexcept
```

Members that have disagreed with the group for the full persistence window, as of the last evaluateDisagreement() (0 = healthy, or never evaluated).

*function, declared at [`include/shulib/hal/motor_group.hpp:261`](../../include/shulib/hal/motor_group.hpp#L261).*

<a id="motorgroup-disagreeingmask"></a>

### `MotorGroup::disagreeingMask`

```cpp
[[nodiscard]] std::uint32_t disagreeingMask() const noexcept
```

Bit i set = member i (this span's index) is PERSISTENTLY disagreeing.

*function, declared at [`include/shulib/hal/motor_group.hpp:264`](../../include/shulib/hal/motor_group.hpp#L264).*

<a id="motorgroup-lastverdict"></a>

### `MotorGroup::lastVerdict`

```cpp
[[nodiscard]] const teleop::SideVerdict& lastVerdict() const noexcept
```

The whole verdict of the last evaluateDisagreement() — for a panel that wants the this-tick mask, the fastest member and the present count as well.

*function, declared at [`include/shulib/hal/motor_group.hpp:268`](../../include/shulib/hal/motor_group.hpp#L268).*

<a id="motorgroup-resetdisagreement"></a>

### `MotorGroup::resetDisagreement`

```cpp
void resetDisagreement() noexcept
```

Clear the monitor's streaks and the stored verdict (a re-arm after a cut, or a new-run boundary).

*function, declared at [`include/shulib/hal/motor_group.hpp:272`](../../include/shulib/hal/motor_group.hpp#L272).*

<a id="motorgroup-membercount"></a>

### `MotorGroup::memberCount`

```cpp
[[nodiscard]] std::size_t memberCount() const noexcept
```

How many SLOTS the group has — present and absent together, so a panel's port list keeps its indices when a member goes missing. presentCount() is how many answer.

*function, declared at [`include/shulib/hal/motor_group.hpp:279`](../../include/shulib/hal/motor_group.hpp#L279).*

<a id="motorgroup-presentcount"></a>

### `MotorGroup::presentCount`

```cpp
[[nodiscard]] std::size_t presentCount() const noexcept
```

How many members are PRESENT (not a nullptr slot). Always >= 1.

*function, declared at [`include/shulib/hal/motor_group.hpp:282`](../../include/shulib/hal/motor_group.hpp#L282).*

<a id="motorgroup-absentmask"></a>

### `MotorGroup::absentMask`

```cpp
[[nodiscard]] std::uint32_t absentMask() const noexcept
```

Bit i set = slot i is ABSENT. 0 = a complete group — the state every graph built from a healthy robot is in, and the one the whole pre-existing suite pins.

*function, declared at [`include/shulib/hal/motor_group.hpp:286`](../../include/shulib/hal/motor_group.hpp#L286).*

<a id="motorgroup-ispresent"></a>

### `MotorGroup::isPresent`

```cpp
[[nodiscard]] bool isPresent(std::size_t i) const noexcept
```

True when slot `i` is present. Out of range is false, so a panel walking a fixed-size array cannot fall off the end.

*function, declared at [`include/shulib/hal/motor_group.hpp:290`](../../include/shulib/hal/motor_group.hpp#L290).*

<a id="motorgroup-member"></a>

### `MotorGroup::member`

```cpp
[[nodiscard]] IMotor& member(std::size_t i) const
```

Member `i` (0-based, the span's order). Preconditions: i < memberCount() AND slot i is PRESENT. Asking an absent slot for its motor is a caller bug — there is no object to return and no sentinel that would not be a lie — so it is a precondition violation rather than a reference to nothing.

*function, declared at [`include/shulib/hal/motor_group.hpp:298`](../../include/shulib/hal/motor_group.hpp#L298).*

<a id="motorgroup-thresholds"></a>

### `MotorGroup::thresholds`

```cpp
[[nodiscard]] const teleop::SideMonitorConfig& thresholds() const noexcept
```

The thresholds the disagreement observable runs under.

*function, declared at [`include/shulib/hal/motor_group.hpp:307`](../../include/shulib/hal/motor_group.hpp#L307).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 100 lines, click to expand</summary>

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
 this class. The graphs that DO use this class — "shulib Teleop" (src/main.cpp's `TankMotors`)
 and, since G1 Part 2a, robot::Robot (`BuildOptions::absentDrivePorts`) — take the MISSING set
 calibration::boot_check read from the brain's own registry, never construct those ports, and
 hand this class a span with a hole where each one would be. Runtime death INSIDE the library
 graph — promoting the drive program's MemberAbsenceDetector in here — is a later chunk.

 WHAT CHANGES, AND WHAT DOES NOT: setVoltage/setBrakeMode SKIP the holes; position(),
 velocity(), current() and temperature() are over the PRESENT members only; the monitor is
 told `present = false` for a hole and never flags it; memberCount() still reports how many
 SLOTS the group has (so the panel's port list keeps its indices) and presentCount() how
 many answer. ALL-PRESENT GROUPS ARE BIT-IDENTICAL — not "equivalent": the whole
 pre-existing suite is the pin, and its assertion count was recorded before and after this
 change, with `MotorGroup 8b` and the plant's case 7 as the local half of it.

 Non-owning: the pointer ARRAY the span views, and the motors it points to, must
 outlive the group — the tree's ownership convention (hal/motor.hpp:57-65).
 PROS-free; host-tested against FakeMotors and through the A2 plant's coupled members.
```

</details>
