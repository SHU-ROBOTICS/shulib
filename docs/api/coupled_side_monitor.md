<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/coupled_side_monitor.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `coupled_side_monitor.hpp`

Coupled-side monitor — the fighting-motor detector for a drivetrain side whose motors are mechanically coupled through one gear train (chunk R3b Part 0b, 2026-09-10), plus the small pure pieces the drive loops need: the per-side arcade arithmetic, and the t…

This header declares **9** types (40 members), **2** free functions, and **1** constant.

Extracted from [`include/shulib/teleop/coupled_side_monitor.hpp`](../../include/shulib/teleop/coupled_side_monitor.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`magnitudeOf`](#magnitudeof) — *free function*
- [`kMaxSideMembers`](#kmaxsidemembers) — *constant*
- [`struct SideMonitorConfig`](#struct-sidemonitorconfig)
  - [`commandFloorV`](#sidemonitorconfig-commandfloorv)
  - [`movingFloorRadS`](#sidemonitorconfig-movingfloorrads)
  - [`oppositeFloorRadS`](#sidemonitorconfig-oppositefloorrads)
  - [`nearZeroFraction`](#sidemonitorconfig-nearzerofraction)
  - [`persistTicks`](#sidemonitorconfig-persistticks)
- [`struct MemberSample`](#struct-membersample)
  - [`velocityRadS`](#membersample-velocityrads)
  - [`present`](#membersample-present)
- [`struct SideVerdict`](#struct-sideverdict)
  - [`evaluated`](#sideverdict-evaluated)
  - [`fastestRadS`](#sideverdict-fastestrads)
  - [`presentCount`](#sideverdict-presentcount)
  - [`disagreeingMask`](#sideverdict-disagreeingmask)
  - [`persistedMask`](#sideverdict-persistedmask)
  - [`persistedCount`](#sideverdict-persistedcount)
- [`class CoupledSideMonitor`](#class-coupledsidemonitor)
  - [`CoupledSideMonitor`](#coupledsidemonitor-coupledsidemonitor)
  - [`CoupledSideMonitor (overload 2)`](#coupledsidemonitor-coupledsidemonitor-2)
  - [`update`](#coupledsidemonitor-update)
  - [`disagreeTicks`](#coupledsidemonitor-disagreeticks)
  - [`reset`](#coupledsidemonitor-reset)
  - [`config`](#coupledsidemonitor-config)
- [`struct SideVolts`](#struct-sidevolts)
  - [`left`](#sidevolts-left)
  - [`right`](#sidevolts-right)
- [`tankSideVolts`](#tanksidevolts) — *free function*
- [`class MemberAbsenceDetector`](#class-memberabsencedetector)
  - [`MemberAbsenceDetector`](#memberabsencedetector-memberabsencedetector)
  - [`MemberAbsenceDetector (overload 2)`](#memberabsencedetector-memberabsencedetector-2)
  - [`update`](#memberabsencedetector-update)
  - [`absent`](#memberabsencedetector-absent)
  - [`reason`](#memberabsencedetector-reason)
- [`struct MemberReading`](#struct-memberreading)
  - [`positionRad`](#memberreading-positionrad)
  - [`velocityRadS`](#memberreading-velocityrads)
  - [`present`](#memberreading-present)
- [`struct FrozenVerdict`](#struct-frozenverdict)
  - [`suspectMask`](#frozenverdict-suspectmask)
  - [`frozenMask`](#frozenverdict-frozenmask)
  - [`latchedMask`](#frozenverdict-latchedmask)
- [`class FrozenReadingDetector`](#class-frozenreadingdetector)
  - [`FrozenReadingDetector`](#frozenreadingdetector-frozenreadingdetector)
  - [`update`](#frozenreadingdetector-update)
  - [`reset`](#frozenreadingdetector-reset)
  - [`repeatReads`](#frozenreadingdetector-repeatreads)
  - [`frozenTicks`](#frozenreadingdetector-frozenticks)
  - [`latchedMask`](#frozenreadingdetector-latchedmask)
  - [`suspectReads`](#frozenreadingdetector-suspectreads)
  - [`floors`](#frozenreadingdetector-floors)

<a id="magnitudeof"></a>

## `magnitudeOf`

```cpp
[[nodiscard]] constexpr double magnitudeOf(double x) noexcept
```

|x| as a constexpr-safe helper (the standard's abs(double) is only guaranteed constexpr from C++23, and this tree builds as gnu++20 on both the host and the V5).

*free function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:66`](../../include/shulib/teleop/coupled_side_monitor.hpp#L66).*

<a id="kmaxsidemembers"></a>

## `kMaxSideMembers`

```cpp
inline constexpr std::size_t kMaxSideMembers = 21
```

The most members one side may carry. A V5 brain has 21 smart ports, so no side can have more; the counters are sized to this so the monitor never allocates.

*constant, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:70`](../../include/shulib/teleop/coupled_side_monitor.hpp#L70).*

<a id="struct-sidemonitorconfig"></a>

## `struct SideMonitorConfig`

```cpp
struct SideMonitorConfig
```

The thresholds, every one of them INVENTED for a first hardware run (R3b Session 2 gate 5) and carried here unchanged so the old "shulib Drive", the tester, hal::MotorGroup and the driver loop's protections agree; R3d/R4 measure. PROVISIONAL (A4: HA-132); the near-zero fraction is also HA-133. The protections' FROZEN rule reuses two of them as its floors (FrozenReadingDetector), so it has no threshold of its own but its read count.

*struct, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:77`](../../include/shulib/teleop/coupled_side_monitor.hpp#L77).*

<a id="sidemonitorconfig-commandfloorv"></a>

### `SideMonitorConfig::commandFloorV`

```cpp
double commandFloorV = 1.0
```

Evaluate only while |side command| exceeds this.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:78`](../../include/shulib/teleop/coupled_side_monitor.hpp#L78).*

<a id="sidemonitorconfig-movingfloorrads"></a>

### `SideMonitorConfig::movingFloorRadS`

```cpp
double movingFloorRadS = 1.0
```

... and only while the side's fastest present member exceeds this.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:79`](../../include/shulib/teleop/coupled_side_monitor.hpp#L79).*

<a id="sidemonitorconfig-oppositefloorrads"></a>

### `SideMonitorConfig::oppositeFloorRadS`

```cpp
double oppositeFloorRadS = 0.5
```

An opposite-sign member counts only above this |velocity|.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:80`](../../include/shulib/teleop/coupled_side_monitor.hpp#L80).*

<a id="sidemonitorconfig-nearzerofraction"></a>

### `SideMonitorConfig::nearZeroFraction`

```cpp
double nearZeroFraction = 0.25
```

Under this fraction of the side's fastest = stalled/frozen.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:81`](../../include/shulib/teleop/coupled_side_monitor.hpp#L81).*

<a id="sidemonitorconfig-persistticks"></a>

### `SideMonitorConfig::persistTicks`

```cpp
int persistTicks = 25
```

Consecutive disagreeing ticks before a member is PERSISTED (250 ms at 10 ms).

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:82`](../../include/shulib/teleop/coupled_side_monitor.hpp#L82).*

<a id="struct-membersample"></a>

## `struct MemberSample`

```cpp
struct MemberSample
```

One member's reading this tick, as the monitor needs it.

*struct, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:86`](../../include/shulib/teleop/coupled_side_monitor.hpp#L86).*

<a id="membersample-velocityrads"></a>

### `MemberSample::velocityRadS`

```cpp
double velocityRadS = 0.0
```

The adapter's velocity(), canonical rad/s (sign already reversed by PROS for a negative port).

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:87`](../../include/shulib/teleop/coupled_side_monitor.hpp#L87).*

<a id="membersample-present"></a>

### `MemberSample::present`

```cpp
bool present = true
```

false = ABSENT (adapter never constructed, or marked dead at runtime): never flagged, never the fastest.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:88`](../../include/shulib/teleop/coupled_side_monitor.hpp#L88).*

<a id="struct-sideverdict"></a>

## `struct SideVerdict`

```cpp
struct SideVerdict
```

What one update() concluded about the side.

*struct, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:92`](../../include/shulib/teleop/coupled_side_monitor.hpp#L92).*

<a id="sideverdict-evaluated"></a>

### `SideVerdict::evaluated`

```cpp
bool evaluated = false
```

The side was above both floors this tick, so members were judged.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:93`](../../include/shulib/teleop/coupled_side_monitor.hpp#L93).*

<a id="sideverdict-fastestrads"></a>

### `SideVerdict::fastestRadS`

```cpp
double fastestRadS = 0.0
```

The largest |velocity| among PRESENT members this tick.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:94`](../../include/shulib/teleop/coupled_side_monitor.hpp#L94).*

<a id="sideverdict-presentcount"></a>

### `SideVerdict::presentCount`

```cpp
int presentCount = 0
```

Members with present == true this tick.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:95`](../../include/shulib/teleop/coupled_side_monitor.hpp#L95).*

<a id="sideverdict-disagreeingmask"></a>

### `SideVerdict::disagreeingMask`

```cpp
std::uint32_t disagreeingMask = 0
```

Bit i set = member i disagreed THIS tick (any streak length).

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:96`](../../include/shulib/teleop/coupled_side_monitor.hpp#L96).*

<a id="sideverdict-persistedmask"></a>

### `SideVerdict::persistedMask`

```cpp
std::uint32_t persistedMask = 0
```

Bit i set = member i has disagreed for persistTicks consecutive ticks or more.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:97`](../../include/shulib/teleop/coupled_side_monitor.hpp#L97).*

<a id="sideverdict-persistedcount"></a>

### `SideVerdict::persistedCount`

```cpp
int persistedCount = 0
```

Number of bits set in persistedMask.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:98`](../../include/shulib/teleop/coupled_side_monitor.hpp#L98).*

<a id="class-coupledsidemonitor"></a>

## `class CoupledSideMonitor`

```cpp
class CoupledSideMonitor
```

The per-side evaluator. Construct one per side, call update() every tick with that side's command and its members in a FIXED order (the bit positions in the verdict are indices into that span), and act on persistedMask. Members beyond kMaxSideMembers are ignored (the caller's precondition, not an allocation).

*class, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:105`](../../include/shulib/teleop/coupled_side_monitor.hpp#L105).*

<a id="coupledsidemonitor-coupledsidemonitor"></a>

### `CoupledSideMonitor::CoupledSideMonitor`

```cpp
constexpr CoupledSideMonitor() noexcept = default
```

Default thresholds are the station's INVENTED ones (SideMonitorConfig).

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:108`](../../include/shulib/teleop/coupled_side_monitor.hpp#L108).*

<a id="coupledsidemonitor-coupledsidemonitor-2"></a>

### `CoupledSideMonitor::CoupledSideMonitor (overload 2)`

```cpp
constexpr explicit CoupledSideMonitor(SideMonitorConfig thresholds) noexcept
```

Thresholds stated by the caller.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:111`](../../include/shulib/teleop/coupled_side_monitor.hpp#L111).*

<a id="coupledsidemonitor-update"></a>

### `CoupledSideMonitor::update`

```cpp
[[nodiscard]] constexpr SideVerdict update(double commandVolts, std::span<const MemberSample> members) noexcept
```

One tick: `commandVolts` is the volts this side was commanded (its SIGN is the expected velocity sign); `members` are this side's readings in the fixed order. Counters advance for members disagreeing this tick, reset for members that agree, and reset for EVERY member on a tick the side is not evaluated (below a floor) — so a side that stops being driven forgets its streaks, exactly as the station did.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:118`](../../include/shulib/teleop/coupled_side_monitor.hpp#L118).*

<a id="coupledsidemonitor-disagreeticks"></a>

### `CoupledSideMonitor::disagreeTicks`

```cpp
[[nodiscard]] constexpr int disagreeTicks(std::size_t i) const noexcept
```

Consecutive disagreeing ticks for member `i` right now (0 = agreeing or not evaluated). The panel colours a member amber on any streak and red on a persisted one.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:157`](../../include/shulib/teleop/coupled_side_monitor.hpp#L157).*

<a id="coupledsidemonitor-reset"></a>

### `CoupledSideMonitor::reset`

```cpp
constexpr void reset() noexcept
```

Clear every streak — what a re-arm after a cut does, so the same fight has to persist again for the full window before it cuts again.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:163`](../../include/shulib/teleop/coupled_side_monitor.hpp#L163).*

<a id="coupledsidemonitor-config"></a>

### `CoupledSideMonitor::config`

```cpp
[[nodiscard]] constexpr const SideMonitorConfig& config() const noexcept
```

The thresholds in force.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:168`](../../include/shulib/teleop/coupled_side_monitor.hpp#L168).*

<a id="struct-sidevolts"></a>

## `struct SideVolts`

```cpp
struct SideVolts
```

The two side commands of a tank drive, in volts.

*struct, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:176`](../../include/shulib/teleop/coupled_side_monitor.hpp#L176).*

<a id="sidevolts-left"></a>

### `SideVolts::left`

```cpp
double left = 0.0
```

Volts for every member of the LEFT side.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:177`](../../include/shulib/teleop/coupled_side_monitor.hpp#L177).*

<a id="sidevolts-right"></a>

### `SideVolts::right`

```cpp
double right = 0.0
```

Volts for every member of the RIGHT side.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:178`](../../include/shulib/teleop/coupled_side_monitor.hpp#L178).*

<a id="tanksidevolts"></a>

## `tankSideVolts`

```cpp
[[nodiscard]] constexpr SideVolts tankSideVolts(const DriveRequest& request, double maxVolts) noexcept
```

The arcade arithmetic both the drive program and the tester's DRIVE station use — ONE definition, so the driver feels one robot: left = maxV × (forward − yawCcw), right = maxV × (forward + yawCcw), each clamped to ±maxV. A CCW (left) turn is a positive yawCcw, which slows the left side and speeds the right — the sign convention of the locked body frame (stick_mapping.hpp). Every member of a side gets the same volts (a group is a voltage fan-out).

*free function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:187`](../../include/shulib/teleop/coupled_side_monitor.hpp#L187).*

<a id="class-memberabsencedetector"></a>

## `class MemberAbsenceDetector`

```cpp
class MemberAbsenceDetector
```

Runtime dead-port detection for ONE member, from the ADAPTER's faulted-read count (R3b Part 0b brief §3 item 4): a port whose adapter constructed at boot can still die in a match. Two signatures, either one for `persistTicks` consecutive ticks marks the member ABSENT — LATCHED for the rest of the run, because a port that answered again for a moment is not one to hand the fight detector back to: * its adapter's faultedReads() counter ADVANCED on every one of those ticks (every read screened to last-good — the port is not answering), or * its velocity read EXACTLY 0.0 while the side was commanded above the command floor and its side-mates' fastest exceeded the moving floor (the train is turning, this encoder is not: a port that reports nothing, not a stalled motor — a stall reads a small nonzero velocity and a large current, and is the monitor's business). Its caller — the old "shulib Drive" (src/programs/drive_program.cpp), which holds each port as a hal::pros::ProsMotor and so HAS the count — keeps commanding the absent member's voltage (harmless) and drops it from the monitor's agreement set by passing present = false. The library's driver loop does NOT run this class: it holds its drive motors as hal::IMotor, which has no faulted-read count (F4, locked), and it does not run the velocity-exactly-0 signature either (the team lead's ruling of 2026-09-28, decision C: through IMotor a dead port reads exactly 0 only when it died at rest, where FrozenReadingDetector below latches on the same tick, and alone the signature marks live members — a stalled side beside a slipping mate). It stays for a caller that has the count; after T4 Part 3 retires the old program, no src/ caller does.

*class, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:216`](../../include/shulib/teleop/coupled_side_monitor.hpp#L216).*

<a id="memberabsencedetector-memberabsencedetector"></a>

### `MemberAbsenceDetector::MemberAbsenceDetector`

```cpp
constexpr MemberAbsenceDetector() noexcept = default
```

Default thresholds are the monitor's.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:219`](../../include/shulib/teleop/coupled_side_monitor.hpp#L219).*

<a id="memberabsencedetector-memberabsencedetector-2"></a>

### `MemberAbsenceDetector::MemberAbsenceDetector (overload 2)`

```cpp
constexpr explicit MemberAbsenceDetector(SideMonitorConfig thresholds) noexcept
```

Thresholds stated by the caller.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:222`](../../include/shulib/teleop/coupled_side_monitor.hpp#L222).*

<a id="memberabsencedetector-update"></a>

### `MemberAbsenceDetector::update`

```cpp
[[nodiscard]] constexpr bool update(int faultedReadsNow, double velocityRadS, double sideCommandVolts, double matesFastestRadS) noexcept
```

One tick. `faultedReadsNow` is the adapter's cumulative faultedReads(); `velocityRadS` its velocity(); `sideCommandVolts` the side's command; `matesFastestRadS` the largest |velocity| among the OTHER present members of the side. Returns true once the member is absent (and stays true: latched).

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:229`](../../include/shulib/teleop/coupled_side_monitor.hpp#L229).*

<a id="memberabsencedetector-absent"></a>

### `MemberAbsenceDetector::absent`

```cpp
[[nodiscard]] constexpr bool absent() const noexcept
```

True once update() has latched the member absent.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:258`](../../include/shulib/teleop/coupled_side_monitor.hpp#L258).*

<a id="memberabsencedetector-reason"></a>

### `MemberAbsenceDetector::reason`

```cpp
[[nodiscard]] constexpr const char* reason() const noexcept
```

Which signature latched it ("" while present) — for the one-time log line.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:261`](../../include/shulib/teleop/coupled_side_monitor.hpp#L261).*

<a id="struct-memberreading"></a>

## `struct MemberReading`

```cpp
struct MemberReading
```

One member's readings this tick, in the side's fixed order, as FrozenReadingDetector needs them.

*struct, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:361`](../../include/shulib/teleop/coupled_side_monitor.hpp#L361).*

<a id="memberreading-positionrad"></a>

### `MemberReading::positionRad`

```cpp
double positionRad = 0.0
```

The member's position(), radians — compared bit for bit.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:362`](../../include/shulib/teleop/coupled_side_monitor.hpp#L362).*

<a id="memberreading-velocityrads"></a>

### `MemberReading::velocityRadS`

```cpp
double velocityRadS = 0.0
```

Its velocity(), rad/s — compared bit for bit, and its magnitude against the floors.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:363`](../../include/shulib/teleop/coupled_side_monitor.hpp#L363).*

<a id="memberreading-present"></a>

### `MemberReading::present`

```cpp
bool present = true
```

false = not read this tick (a boot hole, or a member already marked ABSENT): its counters reset and it takes part in no rule.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:364`](../../include/shulib/teleop/coupled_side_monitor.hpp#L364).*

<a id="struct-frozenverdict"></a>

## `struct FrozenVerdict`

```cpp
struct FrozenVerdict
```

What one FrozenReadingDetector::update() concluded, as member bit masks (bit i = member i).

*struct, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:368`](../../include/shulib/teleop/coupled_side_monitor.hpp#L368).*

<a id="frozenverdict-suspectmask"></a>

### `FrozenVerdict::suspectMask`

```cpp
std::uint32_t suspectMask = 0
```

SUSPECT this tick: repeated for suspectReads reads while it, or a mate whose readings still change, says the train turns.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:369`](../../include/shulib/teleop/coupled_side_monitor.hpp#L369).*

<a id="frozenverdict-frozenmask"></a>

### `FrozenVerdict::frozenMask`

```cpp
std::uint32_t frozenMask = 0
```

FROZEN this tick — the members a caller keeps out of the fight monitor and the over-current watch.

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:370`](../../include/shulib/teleop/coupled_side_monitor.hpp#L370).*

<a id="frozenverdict-latchedmask"></a>

### `FrozenVerdict::latchedMask`

```cpp
std::uint32_t latchedMask = 0
```

NEWLY latched ABSENT this tick (each member latches once in the detector's life).

*field, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:371`](../../include/shulib/teleop/coupled_side_monitor.hpp#L371).*

<a id="class-frozenreadingdetector"></a>

## `class FrozenReadingDetector`

```cpp
class FrozenReadingDetector
```

The frozen-reading rule for ONE side (the header section above): which members read FROZEN this tick, and which have been FROZEN long enough to be marked ABSENT. Construct one per side with that side's monitor thresholds (its floors) and the read count; call update() once per tick with the side's members in a FIXED order. Members beyond kMaxSideMembers are ignored.

*class, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:378`](../../include/shulib/teleop/coupled_side_monitor.hpp#L378).*

<a id="frozenreadingdetector-frozenreadingdetector"></a>

### `FrozenReadingDetector::FrozenReadingDetector`

```cpp
FrozenReadingDetector(SideMonitorConfig floors, int suspectReads)
```

`floors`: the side's own monitor thresholds (`oppositeFloorRadS`, `movingFloorRadS`, `persistTicks`). `suspectReads`: the consecutive repeated reads that make a member SUSPECT (DriveProtectionConfig::frozenSuspectReads, 5, A4 register HA-175). PRECONDITION: `0 < suspectReads < floors.persistTicks` — the rule must decide before the monitor can.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:384`](../../include/shulib/teleop/coupled_side_monitor.hpp#L384).*

<a id="frozenreadingdetector-update"></a>

### `FrozenReadingDetector::update`

```cpp
[[nodiscard]] FrozenVerdict update(std::span<const MemberReading> members) noexcept
```

One tick: the side's members' readings, in the fixed order. Returns this tick's verdict.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:395`](../../include/shulib/teleop/coupled_side_monitor.hpp#L395).*

<a id="frozenreadingdetector-reset"></a>

### `FrozenReadingDetector::reset`

```cpp
void reset() noexcept
```

Forget every counter and every latch — a new detector in all but its thresholds.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:465`](../../include/shulib/teleop/coupled_side_monitor.hpp#L465).*

<a id="frozenreadingdetector-repeatreads"></a>

### `FrozenReadingDetector::repeatReads`

```cpp
[[nodiscard]] int repeatReads(std::size_t i) const noexcept
```

Member `i`'s consecutive repeated reads right now (0 out of range) — for a panel or a test.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:477`](../../include/shulib/teleop/coupled_side_monitor.hpp#L477).*

<a id="frozenreadingdetector-frozenticks"></a>

### `FrozenReadingDetector::frozenTicks`

```cpp
[[nodiscard]] int frozenTicks(std::size_t i) const noexcept
```

Member `i`'s consecutive FROZEN ticks right now (0 out of range, or not frozen).

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:482`](../../include/shulib/teleop/coupled_side_monitor.hpp#L482).*

<a id="frozenreadingdetector-latchedmask"></a>

### `FrozenReadingDetector::latchedMask`

```cpp
[[nodiscard]] std::uint32_t latchedMask() const noexcept
```

Every member latched ABSENT so far, as a bit mask.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:487`](../../include/shulib/teleop/coupled_side_monitor.hpp#L487).*

<a id="frozenreadingdetector-suspectreads"></a>

### `FrozenReadingDetector::suspectReads`

```cpp
[[nodiscard]] int suspectReads() const noexcept
```

The read count this detector was built with.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:490`](../../include/shulib/teleop/coupled_side_monitor.hpp#L490).*

<a id="frozenreadingdetector-floors"></a>

### `FrozenReadingDetector::floors`

```cpp
[[nodiscard]] const SideMonitorConfig& floors() const noexcept
```

The side's thresholds this detector reads its floors and its window from.

*function, declared at [`include/shulib/teleop/coupled_side_monitor.hpp:493`](../../include/shulib/teleop/coupled_side_monitor.hpp#L493).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 50 lines, click to expand</summary>

```text

 Coupled-side monitor — the fighting-motor detector for a drivetrain side whose
 motors are mechanically coupled through one gear train (chunk R3b Part 0b,
 2026-09-10), plus the small pure pieces the drive loops need: the per-side arcade
 arithmetic, and the two runtime dead-port detectors (the faulted-read one of 2026-09-10,
 and T4 Part 2's frozen-reading one, which is the one the library's driver loop runs).

 THE HAZARD, so the thresholds have a reason: robot two has several motors per
 side on ONE gear train (five in its typed table of 2026-09-10, four on its
 calibration record of 2026-09-25 — the reasoning holds for either). A wrong sign
 on one of them stalls it against the others — every motor sits at stall current,
 the train hums and twitches, nothing moves, and gears strip. The bench tester's
 DRIVE station carries this logic as its gate 5, and that station has run on robot
 two's brain (2026-09-10, ten adapters constructed; no motor has yet turned under
 it); this header is that logic EXTRACTED into a pure, host-tested evaluator so the
 old "shulib Drive", the station, hal::MotorGroup and — since T4 Part 2 — the
 library's driver-loop protections (teleop/drive_protection.hpp) share one detector
 and one set of numbers.

 WHAT COUNTS AS DISAGREEING (the station's gate 5, verbatim in meaning):
   * evaluated only while the side is COMMANDED above `commandFloorV` and its
     fastest PRESENT member turns above `movingFloorRadS` — below either, a
     side at rest or a reversing stick would flag everything;
   * a present member whose velocity has the OPPOSITE sign to the command and
     magnitude above `oppositeFloorRadS` — it is being driven backwards
     against its mates;
   * or a present member whose |velocity| is under `nearZeroFraction` of the
     side's fastest — it is stalled (or its encoder is not turning with the
     train: a frozen reading, which ProsMotor holds on a sentinel).
   * an ABSENT member is never flagged and never counts as the side's fastest.
     A dead port is a WARNING elsewhere (drivetrain_degradation.hpp); if it
     were also a disagreement it would cut the drive on every tick forever.

 PERSISTENCE IS COUNTED INSIDE: a member must disagree for `persistTicks`
 CONSECUTIVE ticks (250 ms at 10 ms) before it is reported as persisted. That
 window survives a stick reversal — all members coast the old way for a few
 ticks after the command flips, which is a transient, not a fight — and it is
 the same window as the over-current cut. The 25th consecutive tick flags;
 the 24th does not (the tests pin the boundary exactly).

 The monitor does not command anything. What to DO about a persisted
 disagreement is the caller's policy: the tester cuts permanently (diagnosis);
 the driver loop's protections (teleop/drive_protection.hpp, since T4 Part 2 —
 the old "shulib Drive" did the same) cut for one second, log, count and re-arm
 (a match must not end on a transient — brief §3); an autonomous motion raises
 MOTOR_GROUP_DISAGREE and continues. Every threshold is INVENTED for a first
 run (R3b Session 2's own words) and is a config field, not a magic number.

 PURE and PROS-free: numbers in, a verdict out; the only state is the per-member
 tick counters, and reset() clears them. Host-tested in isolation.
```

</details>
