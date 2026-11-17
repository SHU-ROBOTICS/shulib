<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/drive_protection.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drive_protection.hpp`

DriveProtection — the old "shulib Drive"'s drive protections, in the library's driver loop, for every program that drives through it.

This header declares **4** types (33 members), **1** free function, and **2** constants.

Extracted from [`include/shulib/teleop/drive_protection.hpp`](../../include/shulib/teleop/drive_protection.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct DriveProtectionConfig`](#struct-driveprotectionconfig)
  - [`overCurrentAmps`](#driveprotectionconfig-overcurrentamps)
  - [`overCurrentTicks`](#driveprotectionconfig-overcurrentticks)
  - [`cutTicks`](#driveprotectionconfig-cutticks)
  - [`maxCutsLogged`](#driveprotectionconfig-maxcutslogged)
  - [`overCurrentCuts`](#driveprotectionconfig-overcurrentcuts)
  - [`frozenSuspectReads`](#driveprotectionconfig-frozensuspectreads)
- [`enum class ProtectionMode`](#enum-class-protectionmode)
  - [`Driving`](#protectionmode-driving)
  - [`Degraded`](#protectionmode-degraded)
  - [`Cut`](#protectionmode-cut)
  - [`Refused`](#protectionmode-refused)
- [`kMaxProtectedGroups`](#kmaxprotectedgroups) — *constant*
- [`kProtectionTextBytes`](#kprotectiontextbytes) — *constant*
- [`struct DriveProtectionState`](#struct-driveprotectionstate)
  - [`mode`](#driveprotectionstate-mode)
  - [`cutTicksLeft`](#driveprotectionstate-cutticksleft)
  - [`cutCount`](#driveprotectionstate-cutcount)
  - [`overCurrentWarnings`](#driveprotectionstate-overcurrentwarnings)
  - [`lastCut`](#driveprotectionstate-lastcut)
  - [`dead`](#driveprotectionstate-dead)
  - [`frozen`](#driveprotectionstate-frozen)
  - [`verdict`](#driveprotectionstate-verdict)
  - [`groupCount`](#driveprotectionstate-groupcount)
  - [`peakAmps`](#driveprotectionstate-peakamps)
- [`validProtectionConfig`](#validprotectionconfig) — *free function*
- [`class DriveProtection`](#class-driveprotection)
  - [`DriveProtection`](#driveprotection-driveprotection)
  - [`~DriveProtection`](#driveprotection-destructor-driveprotection)
  - [`DriveProtection (overload 2)`](#driveprotection-driveprotection-2)
  - [`DriveProtection (overload 3)`](#driveprotection-driveprotection-3)
  - [`operator=`](#driveprotection-operator-eq)
  - [`operator= (overload 2)`](#driveprotection-operator-eq-2)
  - [`enter`](#driveprotection-enter)
  - [`beforeDrive`](#driveprotection-beforedrive)
  - [`commandAllowed`](#driveprotection-commandallowed)
  - [`afterDrive`](#driveprotection-afterdrive)
  - [`onDisable`](#driveprotection-ondisable)
  - [`state`](#driveprotection-state)
  - [`config`](#driveprotection-config)

<a id="struct-driveprotectionconfig"></a>

## `struct DriveProtectionConfig`

```cpp
struct DriveProtectionConfig
```

The drive protections' numbers — the old "shulib Drive"'s (R3b Part 0b; T4 Part 3 replaced it), each with its source, and ONE new one, `frozenSuspectReads`. A program leaves this at its defaults by naming no `protection` in its DriverLoopConfig. The fight's floors are NOT here: they are each group's SideMonitorConfig (HA-132/133), one set of numbers as before. runDriverControl checks it before anything moves (validProtectionConfig).

*struct, declared at [`include/shulib/teleop/drive_protection.hpp:132`](../../include/shulib/teleop/drive_protection.hpp#L132).*

<a id="driveprotectionconfig-overcurrentamps"></a>

### `DriveProtectionConfig::overCurrentAmps`

```cpp
double overCurrentAmps = 2.4
```

A drive motor drawing MORE than this, in amperes, is over-current — the old program's `kCurrentLimitA`, INVENTED for a first run (A4 register HA-176). Finite and > 0.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:135`](../../include/shulib/teleop/drive_protection.hpp#L135).*

<a id="driveprotectionconfig-overcurrentticks"></a>

### `DriveProtectionConfig::overCurrentTicks`

```cpp
int overCurrentTicks = 25
```

Consecutive ticks over `overCurrentAmps` before an over-current counts (25 = 250 ms at 10 ms) — EQUAL to the fight monitor's `persistTicks` by choice, as the old program pinned it (HA-132). > 0.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:139`](../../include/shulib/teleop/drive_protection.hpp#L139).*

<a id="driveprotectionconfig-cutticks"></a>

### `DriveProtectionConfig::cutTicks`

```cpp
int cutTicks = 100
```

How long a cut holds the drive at 0 V, in ticks (100 = the old program's 1 s). > 0.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:141`](../../include/shulib/teleop/drive_protection.hpp#L141).*

<a id="driveprotectionconfig-maxcutslogged"></a>

### `DriveProtectionConfig::maxCutsLogged`

```cpp
int maxCutsLogged = 50
```

Cuts logged in full — the cut line and one `at the cut:` line per member; later cuts are counted but no longer logged per port, so a permanent fight cannot fill a log (the old program's 50). Over-current WARNINGS are logged within the same number, counted apart. > 0.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:145`](../../include/shulib/teleop/drive_protection.hpp#L145).*

<a id="driveprotectionconfig-overcurrentcuts"></a>

### `DriveProtectionConfig::overCurrentCuts`

```cpp
bool overCurrentCuts = true
```

true (the default, the team lead's N1 (a)): an over-current CUTS the drive. false: it is logged and counted as a WARNING (DriveProtectionState::overCurrentWarnings) and nothing is cut — the one switch for a robot that must push. The fight cut is not switched by this.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:149`](../../include/shulib/teleop/drive_protection.hpp#L149).*

<a id="driveprotectionconfig-frozensuspectreads"></a>

### `DriveProtectionConfig::frozenSuspectReads`

```cpp
int frozenSuspectReads = 5
```

The consecutive repeated reads — position AND velocity bit-identical — that make a member SUSPECT (FrozenReadingDetector; A4 register HA-175, invented). > 0 and below EVERY group's `persistTicks`: the frozen-reading rule must decide before the fight monitor can cut.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:153`](../../include/shulib/teleop/drive_protection.hpp#L153).*

<a id="enum-class-protectionmode"></a>

## `enum class ProtectionMode`

```cpp
enum class ProtectionMode
```

What the drive protections are doing to the drive right now.

*enum class, declared at [`include/shulib/teleop/drive_protection.hpp:164`](../../include/shulib/teleop/drive_protection.hpp#L164).*

<a id="protectionmode-driving"></a>

### `ProtectionMode::Driving`

```cpp
Driving
```

Every drive motor answers; the stick drives.

*enumerator, declared at [`include/shulib/teleop/drive_protection.hpp:165`](../../include/shulib/teleop/drive_protection.hpp#L165).*

<a id="protectionmode-degraded"></a>

### `ProtectionMode::Degraded`

```cpp
Degraded
```

Some are dead (at boot, or marked mid-run) but no more than two: the stick drives, and the driver is told.

*enumerator, declared at [`include/shulib/teleop/drive_protection.hpp:166`](../../include/shulib/teleop/drive_protection.hpp#L166).*

<a id="protectionmode-cut"></a>

### `ProtectionMode::Cut`

```cpp
Cut
```

A 1 s cut is in force (a fight, or an over-current): the drive is at 0 V, and re-arms by itself.

*enumerator, declared at [`include/shulib/teleop/drive_protection.hpp:167`](../../include/shulib/teleop/drive_protection.hpp#L167).*

<a id="protectionmode-refused"></a>

### `ProtectionMode::Refused`

```cpp
Refused
```

More than two drive motors are dead in total: 0 V for the rest of the run.

*enumerator, declared at [`include/shulib/teleop/drive_protection.hpp:168`](../../include/shulib/teleop/drive_protection.hpp#L168).*

<a id="kmaxprotectedgroups"></a>

## `kMaxProtectedGroups`

```cpp
inline constexpr std::size_t kMaxProtectedGroups = 4
```

The most motor groups one DriveProtection covers — a tank's two, or a four-wheel holonomic with a group per wheel. runDriverControl refuses a chassis with more before anything moves.

*constant, declared at [`include/shulib/teleop/drive_protection.hpp:173`](../../include/shulib/teleop/drive_protection.hpp#L173).*

<a id="kprotectiontextbytes"></a>

## `kProtectionTextBytes`

```cpp
inline constexpr std::size_t kProtectionTextBytes = 96
```

The size of each text field of DriveProtectionState, terminating NUL included.

*constant, declared at [`include/shulib/teleop/drive_protection.hpp:176`](../../include/shulib/teleop/drive_protection.hpp#L176).*

<a id="struct-driveprotectionstate"></a>

## `struct DriveProtectionState`

```cpp
struct DriveProtectionState
```

What the protections are doing, for a panel — handed to a `perTick` that takes a fourth argument (runDriverControl). Trivially copyable; every text is NUL-terminated, cut to fit.

*struct, declared at [`include/shulib/teleop/drive_protection.hpp:180`](../../include/shulib/teleop/drive_protection.hpp#L180).*

<a id="driveprotectionstate-mode"></a>

### `DriveProtectionState::mode`

```cpp
ProtectionMode mode = ProtectionMode::Driving
```

Driving, degraded, cut, or refused.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:182`](../../include/shulib/teleop/drive_protection.hpp#L182).*

<a id="driveprotectionstate-cutticksleft"></a>

### `DriveProtectionState::cutTicksLeft`

```cpp
int cutTicksLeft = 0
```

Ticks of the cut in force still to run (0 when no cut is in force).

*field, declared at [`include/shulib/teleop/drive_protection.hpp:184`](../../include/shulib/teleop/drive_protection.hpp#L184).*

<a id="driveprotectionstate-cutcount"></a>

### `DriveProtectionState::cutCount`

```cpp
int cutCount = 0
```

Cuts this driver-control entry — logged ones and unlogged ones alike.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:186`](../../include/shulib/teleop/drive_protection.hpp#L186).*

<a id="driveprotectionstate-overcurrentwarnings"></a>

### `DriveProtectionState::overCurrentWarnings`

```cpp
int overCurrentWarnings = 0
```

Over-current WARNINGS this entry (DriveProtectionConfig::overCurrentCuts false).

*field, declared at [`include/shulib/teleop/drive_protection.hpp:188`](../../include/shulib/teleop/drive_protection.hpp#L188).*

<a id="driveprotectionstate-lastcut"></a>

### `DriveProtectionState::lastCut`

```cpp
char lastCut[kProtectionTextBytes]{}
```

The last cut's words, as its log line gives them ("CUT: FIGHT on LEFT -- port(s) 18"); empty before the first cut.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:191`](../../include/shulib/teleop/drive_protection.hpp#L191).*

<a id="driveprotectionstate-dead"></a>

### `DriveProtectionState::dead`

```cpp
char dead[kProtectionTextBytes]{}
```

Every DEAD drive port — a boot hole or a runtime mark — by label, in group order (LEFT's, then RIGHT's: "18 13"); empty when all answer. An unlabelled member is "m<index>".

*field, declared at [`include/shulib/teleop/drive_protection.hpp:194`](../../include/shulib/teleop/drive_protection.hpp#L194).*

<a id="driveprotectionstate-frozen"></a>

### `DriveProtectionState::frozen`

```cpp
char frozen[kProtectionTextBytes]{}
```

The members FROZEN this tick, the same way ("13"). Empty almost always: a dead member passes through it for 250 ms on its way to `dead`, and a LIVE member may show for a few ticks as its train stops (the probes: at most 3 at a coast-down; at a plain wall none, since the encoder check of 2026-10-01 — a mate whose encoder has stopped is no evidence that the train turns, however its lagging report reads). The exception is the rule's known gap (A4 register HA-175): BESIDE A SLIPPING side-mate, a stalled LIVE member can stay here the full 250 ms and then be marked dead (a tie, or a report settling sooner).

*field, declared at [`include/shulib/teleop/drive_protection.hpp:202`](../../include/shulib/teleop/drive_protection.hpp#L202).*

<a id="driveprotectionstate-verdict"></a>

### `DriveProtectionState::verdict`

```cpp
DegradationVerdict verdict{.verdict = DriveVerdict::Drive}
```

The degradation verdict in force, by the MID-RUN rule (evaluateRuntimeDegradation).

*field, declared at [`include/shulib/teleop/drive_protection.hpp:204`](../../include/shulib/teleop/drive_protection.hpp#L204).*

<a id="driveprotectionstate-groupcount"></a>

### `DriveProtectionState::groupCount`

```cpp
std::size_t groupCount = 0
```

How many groups (sides) `peakAmps` holds.

*field, declared at [`include/shulib/teleop/drive_protection.hpp:206`](../../include/shulib/teleop/drive_protection.hpp#L206).*

<a id="driveprotectionstate-peakamps"></a>

### `DriveProtectionState::peakAmps`

```cpp
double peakAmps[kMaxProtectedGroups]{}
```

Each group's highest member current this entry, amperes, over its present members that were not FROZEN (a frozen current is a stale one). Index = the chassis's motorGroups order (a tank: 0 LEFT, 1 RIGHT).

*field, declared at [`include/shulib/teleop/drive_protection.hpp:210`](../../include/shulib/teleop/drive_protection.hpp#L210).*

<a id="validprotectionconfig"></a>

## `validProtectionConfig`

```cpp
[[nodiscard]] inline bool validProtectionConfig(const DriveProtectionConfig& cfg, std::span<hal::MotorGroup* const> groups, char* why, std::size_t cap) noexcept
```

The entry check runDriverControl makes before anything moves (the brief's P2-20): every number of `cfg` finite and > 0, no more than kMaxProtectedGroups groups, and `frozenSuspectReads` below every group's `persistTicks`. False, with the reason in `why` (cut to `cap`, always NUL-terminated when cap > 0; `why` may be null), when one is not.

*free function, declared at [`include/shulib/teleop/drive_protection.hpp:220`](../../include/shulib/teleop/drive_protection.hpp#L220).*

<a id="class-driveprotection"></a>

## `class DriveProtection`

```cpp
class DriveProtection
```

The drive protections for ONE driver-control entry (the file header): the frozen-reading rule and runtime absence, the fight and over-current cuts, the mid-run refusal, and their log lines. runDriverControl builds one per call and drives it — enter() before tick 1, then beforeDrive() / commandAllowed() / afterDrive() every tick, onDisable() on the disable — so a program never calls it; it is public so a test, or a program with its own loop, can. Non-copyable, non-movable. The chassis, its groups, its drive motors and its sink must outlive it; its destructor clears every group's exclusion mask.

*class, declared at [`include/shulib/teleop/drive_protection.hpp:268`](../../include/shulib/teleop/drive_protection.hpp#L268).*

<a id="driveprotection-driveprotection"></a>

### `DriveProtection::DriveProtection`

```cpp
DriveProtection(chassis::Chassis& chassis, const DriveProtectionConfig& cfg, units::Time tickPeriod)
```

Over `chassis`'s drive motors, motor groups and telemetry sink, with `cfg` and the loop's tick period (for the durations its lines print). PRECONDITIONS: validProtectionConfig(cfg, the chassis's motorGroups) holds, and `tickPeriod` is finite and > 0 — exactly the two runDriverControl checks before it commands anything, so the loop's DriveProtection can never refuse after the loop's entry has moved something (any tick the loop accepts prints: "12.5 ms" at a 0.5 ms tick).

*function, declared at [`include/shulib/teleop/drive_protection.hpp:276`](../../include/shulib/teleop/drive_protection.hpp#L276).*

<a id="driveprotection-destructor-driveprotection"></a>

### `DriveProtection::~DriveProtection`

```cpp
~DriveProtection()
```

Clears every group's exclusion mask, so the disable return and a throw out of the loop both leave the groups as motion code expects (the file header). The ABSENT marks stay.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:300`](../../include/shulib/teleop/drive_protection.hpp#L300).*

<a id="driveprotection-driveprotection-2"></a>

### `DriveProtection::DriveProtection (overload 2)`

```cpp
DriveProtection(const DriveProtection&) = delete
```

Non-copyable: it holds per-member state for groups it does not own.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:303`](../../include/shulib/teleop/drive_protection.hpp#L303).*

<a id="driveprotection-driveprotection-3"></a>

### `DriveProtection::DriveProtection (overload 3)`

```cpp
DriveProtection(DriveProtection&&) = delete
```

Non-copyable: it holds per-member state for groups it does not own. Non-movable, same reasoning.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:305`](../../include/shulib/teleop/drive_protection.hpp#L305).*

<a id="driveprotection-operator-eq"></a>

### `DriveProtection::operator=`

```cpp
DriveProtection& operator=(const DriveProtection&) = delete
```

Non-copyable: it holds per-member state for groups it does not own. Non-movable, same reasoning. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:307`](../../include/shulib/teleop/drive_protection.hpp#L307).*

<a id="driveprotection-operator-eq-2"></a>

### `DriveProtection::operator= (overload 2)`

```cpp
DriveProtection& operator=(DriveProtection&&) = delete
```

Non-copyable: it holds per-member state for groups it does not own. Non-movable, same reasoning. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:309`](../../include/shulib/teleop/drive_protection.hpp#L309).*

<a id="driveprotection-enter"></a>

### `DriveProtection::enter`

```cpp
void enter()
```

Before tick 1: clear every group's exclusion mask and judge the drive by the MID-RUN rule from the groups as they stand — runtime marks from an earlier entry included. More than two dead in total REFUSES here, with one line, and the drive gets 0 V from tick 1; anything dead is said in one DEGRADED line; a complete drive says nothing.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:315`](../../include/shulib/teleop/drive_protection.hpp#L315).*

<a id="driveprotection-beforedrive"></a>

### `DriveProtection::beforeDrive`

```cpp
void beforeDrive()
```

Every tick, BEFORE drive(): the frozen-reading rule over every present member, the FROZEN members kept out of their group's monitor for this tick, and a member FROZEN for the monitor's window marked ABSENT (one line, and the degradation re-judged — which may REFUSE).

*function, declared at [`include/shulib/teleop/drive_protection.hpp:323`](../../include/shulib/teleop/drive_protection.hpp#L323).*

<a id="driveprotection-commandallowed"></a>

### `DriveProtection::commandAllowed`

```cpp
[[nodiscard]] bool commandAllowed() const noexcept
```

Whether this tick's drive() may take the stick's command: false while a cut or a refusal is in force, when the loop hands drive() a zero command instead.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:374`](../../include/shulib/teleop/drive_protection.hpp#L374).*

<a id="driveprotection-afterdrive"></a>

### `DriveProtection::afterDrive`

```cpp
void afterDrive()
```

Every tick, AFTER drive(): this tick's currents and peaks, the driven stretches (the peak line), then a cut in force counting down (and re-arming at zero) — or, with none in force and no refusal, the fight and over-current checks, either of which may CUT now.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:379`](../../include/shulib/teleop/drive_protection.hpp#L379).*

<a id="driveprotection-ondisable"></a>

### `DriveProtection::onDisable`

```cpp
void onDisable()
```

The loop's step 1, AFTER cancel(): one summary line per group (each member's peak current and temperature this entry, and the cut count). Bare drive motors (no group) get none — they have no stretch or peak record either, only the over-current cut.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:426`](../../include/shulib/teleop/drive_protection.hpp#L426).*

<a id="driveprotection-state"></a>

### `DriveProtection::state`

```cpp
[[nodiscard]] const DriveProtectionState& state() const noexcept
```

What the protections are doing right now (for a panel).

*function, declared at [`include/shulib/teleop/drive_protection.hpp:456`](../../include/shulib/teleop/drive_protection.hpp#L456).*

<a id="driveprotection-config"></a>

### `DriveProtection::config`

```cpp
[[nodiscard]] const DriveProtectionConfig& config() const noexcept
```

The numbers this entry runs under.

*function, declared at [`include/shulib/teleop/drive_protection.hpp:459`](../../include/shulib/teleop/drive_protection.hpp#L459).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 102 lines, click to expand</summary>

```text

 DriveProtection — the old "shulib Drive"'s drive protections, in the library's driver loop,
 for every program that drives through it (chunk T4 Part 2, 2026-09-29).

 ── Why this exists ─────────────────────────────────────────────────────────────────
 The team lead's ruling R4 (2026-09-27): "Moved into the library, so the competition program's
 opcontrol() and Teleop get them too, not just this program: the mid-match absence rule and the
 two re-arming cuts." Until this file only the old "shulib Drive" (src/programs/drive_program.cpp
 until T4 Part 3 replaced it) had them, and it drove through bare adapters, never through the
 library. (The "Teleop" of the ruling is "shulib Teleop", which T4 Part 3 retired.) Now
 teleop::runDriverControl (teleop_loop.hpp) builds ONE DriveProtection per call and runs it on
 every tick, so every program that drives through the library's loop is protected with no wiring
 of its own — and no program can forget to be. The numbers are DriverLoopConfig::protection, a
 field whose defaults are the old program's, so a program that states nothing gets them.

 ── ONE TICK, in the loop's order ───────────────────────────────────────────────────
   beforeDrive()  THE FROZEN RULE, first. Every group's FrozenReadingDetector
                  (coupled_side_monitor.hpp) is handed each PRESENT member's fresh position()
                  and velocity(). A member whose readings are FROZEN — repeated bit for bit for
                  5 reads while it, or a side-mate whose ENCODER still moves, says the train
                  turns, with at least as many of its side answering as not — is kept out of its
                  group's fight monitor for this tick (MotorGroup::setMonitorExcluded) and out
                  of the over-current watch. One FROZEN for the monitor's 250 ms is marked ABSENT
                  (MotorGroup::markAbsent: out of every read, still commanded), logged once, and
                  the degradation re-judged by the MID-RUN rule, evaluateRuntimeDegradation:
                  more than two drive motors dead in total, anywhere, REFUSES — 0 V for the
                  rest of the run (the team lead's N7 (c), "Only when 3+ are dead in total").
                  It runs BEFORE drive() because drive() is where the group monitors are judged:
                  a member found FROZEN here is already out of the monitor THIS tick, so a dead
                  member's stale velocity can never start the fight streak that would cut
                  (the 2026-09-27 re-check's race; the brief's landmine 16).
   drive()        the loop's own call — the stick's command, or, while a cut or a refusal is in
                  force (commandAllowed() false), a ZERO command, so the Localizer and the health
                  tick keep running. The cruise feedforward makes a zero command exactly 0 V.
   afterDrive()   a cut in force counts down and, at zero, RE-ARMS: every over-current streak
                  cleared, every group's monitor reset, one line. Otherwise two checks, each of
                  which CUTS:
                  * the FIGHT: a group's own monitor, judged once in this tick's drive() with its
                    ABSENT and FROZEN members already out, persisted — a member opposite its
                    side's command above 0.5 rad/s, or under 25 % of the side's fastest, while the
                    side is commanded above 1 V and turning above 1 rad/s, for 250 ms
                    (HA-132/133). There is no second monitor.
                  * the OVER-CURRENT: one member above `overCurrentAmps` (2.4 A) for
                    `overCurrentTicks` (250 ms) CONSECUTIVE ticks, counted only while its side
                    is commanded above the monitor's 1 V floor and never for an ABSENT or FROZEN
                    member (a dead port's current is its last good reading, and one that died at
                    the current limit would otherwise re-cut a standing robot every 1.25 s),
                    judged per member, never on the side's total (HA-176). A full-stick
                    REVERSAL at speed, or a full-stick launch from rest, is expected to hold
                    every member near its current limit (believed to be the PROS default
                    2.5 A, HA-154 — unmeasured) while the train turns round or spins up; whether
                    that lasts under 250 ms on robot two is unmeasured too, so a healthy drive
                    may cut mid-manoeuvre (HA-176's settle has the robot check; the host test
                    plant synthesizes no current and cannot show it).
                  A cut writes 0 V to every drive motor IN THIS TICK — drive() has already sent
                  the stick's volts, and a zero command alone would start the cut a tick late —
                  then holds the drive at 0 V for `cutTicks` (1 s), counts the cut and logs it
                  with one `at the cut:` line per member, and re-arms. With `overCurrentCuts`
                  false an over-current is logged and counted as a WARNING and nothing is cut
                  (the team lead's N1 (a): the cut is ON, with one switch to turn it off, and a
                  push check on the robot is required before any match).
                  THE PEAK LINE: the tick a side's command falls back to 1 V or less, one line
                  if that driven stretch's highest member current is a new high for this entry
                  — the record a push check reads, which does not depend on the loop ever
                  seeing the disable (HA-161; the team has no competition switch).
   onDisable()    the loop's step 1, after cancel(): one summary line per group — each member's
                  peak current and temperature this entry, and the cut count. A bonus: it prints
                  only if the loop polls disabled() before PROS deletes the task. (A drive with
                  no groups prints none.)
 Every line goes through the chassis's own telemetry sink with the loop's tag, `TEL`, in the old
 program's words, re-pointed from a bare port to the group's label ("port 13") and its side.

 ── What persists, and what does not ────────────────────────────────────────────────
 The ABSENT marks live in the GROUPS, which live in the program's static storage: they persist
 for the rest of the run — into the next driver-control entry, which re-judges them at entry (so
 a robot with 3+ dead is REFUSED from tick 1, every later start included, until the program is
 started again), and into a later autonomous (not acted on there: the scheduler's fault policy
 governs a motion). The cut, its count and the FROZEN exclusion do NOT persist: this object is
 built per call, and both enter() and its destructor clear every group's exclusion mask, so the
 disable return AND a throw out of the loop leave the groups as motion code expects
 (MOTOR_GROUP_DISAGREE in an autonomous motion is unchanged). If PROS deletes the task before the
 loop polls the disable, no destructor runs: the mask then holds the last tick's FROZEN members
 until the next driver-control entry clears it — stated, not solved.

 ── A drive motor that is not in a group ────────────────────────────────────────────
 (the invented X-drive's four wheels; a hand-built chassis) gets the over-current cut only,
 named by its wheel index and counted while that motor is commanded above 1 V. It has no side to
 be compared against, so there is no FROZEN rule and no fight for it — and so a bare motor's
 frozen current is NOT screened out of its over-current streak: stated, not solved. The X-drive
 refuses at boot on every robot that exists.

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 Host-tested on the host test plant's member faults and on hand-stepped readings
 (test/drive_protection_test.cpp) — a train with inertia, a wall, a coast-down and a slipping
 member, which the plant cannot shape. NOTHING here has run on a robot. Every number is invented
 or reasoned: the 2.4 A over 250 ms (HA-176), the 5 reads and the 250 ms latch (HA-175), the
 fight's floors (HA-132/133). The per-tick reads — each present member's position, velocity,
 current and temperature, about 32 adapter reads per 10 ms tick on robot two — are what the old
 program read, and their cost on the brain is unmeasured (HA-32). So is this object's size on
 the driver-control task's stack (about 5 KB at kMaxProtectedGroups groups).

 PROS-free, sim-free, allocation-free.
```

</details>
