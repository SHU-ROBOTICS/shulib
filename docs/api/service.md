<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/service.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `service.hpp`

mechanism/service.hpp — THE MECHANISM SERVICE: one object, ticked once per control tick, that runs every mechanism's behaviour.

This header declares **8** types (106 members), **5** constants, and **1** type alias.

Extracted from [`include/shulib/mechanism/service.hpp`](../../include/shulib/mechanism/service.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kCapSignatureMarginAmps`](#kcapsignaturemarginamps) — *constant*
- [`kCapRecheckSeconds`](#kcaprecheckseconds) — *constant*
- [`MechanismOutcome`](#mechanismoutcome) — *type alias*
- [`kHoldIntegralLimitVolts`](#kholdintegrallimitvolts) — *constant*
- [`kSpeedIntegralWindow`](#kspeedintegralwindow) — *constant*
- [`kSpeedIntegralLimitVolts`](#kspeedintegrallimitvolts) — *constant*
- [`struct StepReport`](#struct-stepreport)
  - [`outcome`](#stepreport-outcome)
  - [`doneBy`](#stepreport-doneby)
  - [`countedDone`](#stepreport-counteddone)
  - [`elapsed`](#stepreport-elapsed)
- [`class Slot`](#class-slot)
  - [`Slot`](#slot-slot)
  - [`operator=`](#slot-operator-eq)
  - [`Slot (overload 2)`](#slot-slot-2)
  - [`operator= (overload 2)`](#slot-operator-eq-2)
  - [`~Slot`](#slot-destructor-slot)
  - [`kind`](#slot-kind)
  - [`spec`](#slot-spec)
  - [`name`](#slot-name)
  - [`index`](#slot-index)
  - [`built`](#slot-built)
  - [`absence`](#slot-absence)
  - [`lastStep`](#slot-laststep)
  - [`running`](#slot-running)
  - [`stepsBegun`](#slot-stepsbegun)
  - [`commandSeq`](#slot-commandseq)
  - [`holdsClaim`](#slot-holdsclaim)
  - [`seen`](#slot-seen)
  - [`sees`](#slot-sees)
  - [`homeSwitchPressed`](#slot-homeswitchpressed)
  - [`declaresPieceSensor`](#slot-declarespiecesensor)
  - [`hasPieceSensor`](#slot-haspiecesensor)
  - [`pieceSensorMissing`](#slot-piecesensormissing)
  - [`pieceSensorPlace`](#slot-piecesensorplace)
  - [`cancel`](#slot-cancel)
  - [`enum class Slot::Seen`](#enum-class-slot-seen)
    - [`Yes`](#slot-seen-yes)
    - [`No`](#slot-seen-no)
    - [`Unknown`](#slot-seen-unknown)
- [`class PowerSlot`](#class-powerslot)
  - [`PowerSlot`](#powerslot-powerslot)
  - [`checkRun`](#powerslot-checkrun)
  - [`run`](#powerslot-run)
  - [`stop`](#powerslot-stop)
  - [`drive`](#powerslot-drive)
  - [`commandedVolts`](#powerslot-commandedvolts)
- [`class PositionSlot`](#class-positionslot)
  - [`PositionSlot`](#positionslot-positionslot)
  - [`checkMoveTo`](#positionslot-checkmoveto)
  - [`moveTo`](#positionslot-moveto)
  - [`moveToPreset`](#positionslot-movetopreset)
  - [`holdHere`](#positionslot-holdhere)
  - [`checkHome`](#positionslot-checkhome)
  - [`home`](#positionslot-home)
  - [`jog`](#positionslot-jog)
  - [`release`](#positionslot-release)
  - [`homed`](#positionslot-homed)
  - [`position`](#positionslot-position)
  - [`target`](#positionslot-target)
  - [`softStops`](#positionslot-softstops)
  - [`jointDeg`](#positionslot-jointdeg)
  - [`holding`](#positionslot-holding)
  - [`heldButtonMoving`](#positionslot-heldbuttonmoving)
  - [`takesRange`](#positionslot-takesrange)
- [`class SpeedSlot`](#class-speedslot)
  - [`SpeedSlot`](#speedslot-speedslot)
  - [`checkSpinUp`](#speedslot-checkspinup)
  - [`spinUp`](#speedslot-spinup)
  - [`spinUpPreset`](#speedslot-spinuppreset)
  - [`stop`](#speedslot-stop)
  - [`spinning`](#speedslot-spinning)
  - [`rpm`](#speedslot-rpm)
  - [`targetRpm`](#speedslot-targetrpm)
  - [`atSpeed`](#speedslot-atspeed)
- [`class PneumaticSlot`](#class-pneumaticslot)
  - [`kNoPosition`](#pneumaticslot-knoposition)
  - [`PneumaticSlot`](#pneumaticslot-pneumaticslot)
  - [`set`](#pneumaticslot-set)
  - [`commandedPosition`](#pneumaticslot-commandedposition)
  - [`lastStepBound`](#pneumaticslot-laststepbound)
- [`class MechanismService`](#class-mechanismservice)
  - [`MechanismService`](#mechanismservice-mechanismservice)
  - [`MechanismService (overload 2)`](#mechanismservice-mechanismservice-2)
  - [`operator=`](#mechanismservice-operator-eq)
  - [`MechanismService (overload 3)`](#mechanismservice-mechanismservice-3)
  - [`operator= (overload 2)`](#mechanismservice-operator-eq-2)
  - [`~MechanismService`](#mechanismservice-destructor-mechanismservice)
  - [`bind`](#mechanismservice-bind)
  - [`bind (overload 2)`](#mechanismservice-bind-2)
  - [`bound`](#mechanismservice-bound)
  - [`pace`](#mechanismservice-pace)
  - [`tickFromDriverLoop`](#mechanismservice-tickfromdriverloop)
  - [`enterDriverControl`](#mechanismservice-enterdrivercontrol)
  - [`enterAutonomous`](#mechanismservice-enterautonomous)
  - [`leaveDriverControl`](#mechanismservice-leavedrivercontrol)
  - [`size`](#mechanismservice-size)
  - [`slot`](#mechanismservice-slot)
  - [`slotPtr`](#mechanismservice-slotptr)
  - [`power`](#mechanismservice-power)
  - [`position`](#mechanismservice-position)
  - [`speed`](#mechanismservice-speed)
  - [`pneumatic`](#mechanismservice-pneumatic)
  - [`find`](#mechanismservice-find)
  - [`find (overload 2)`](#mechanismservice-find-2)
  - [`find (overload 3)`](#mechanismservice-find-3)
  - [`find (overload 4)`](#mechanismservice-find-4)
  - [`find (overload 5)`](#mechanismservice-find-5)
  - [`find (overload 6)`](#mechanismservice-find-6)
  - [`find (overload 7)`](#mechanismservice-find-7)
  - [`find (overload 8)`](#mechanismservice-find-8)
  - [`slotFor`](#mechanismservice-slotfor)
  - [`mechanisms`](#mechanismservice-mechanisms)
  - [`ticks`](#mechanismservice-ticks)
  - [`leftoverClaimsReleased`](#mechanismservice-leftoverclaimsreleased)
  - [`clock`](#mechanismservice-clock)
  - [`faults`](#mechanismservice-faults)
  - [`telemetry`](#mechanismservice-telemetry)
  - [`rederive`](#mechanismservice-rederive)
  - [`canRederive`](#mechanismservice-canrederive)

<a id="kcapsignaturemarginamps"></a>

## `kCapSignatureMarginAmps`

```cpp
inline constexpr double kCapSignatureMarginAmps = 0.2
```

How close (A) a stated stall or homing signature may come to the CURRENT CAP before the boot warns (MECH-S-A; A4 register HA-223 — invented: a stalled capped motor is believed to read within ~0.1 A of its cap). A LIBRARY constant, like the integral clamps below.

*constant, declared at [`include/shulib/mechanism/service.hpp:187`](../../include/shulib/mechanism/service.hpp#L187).*

<a id="kcaprecheckseconds"></a>

## `kCapRecheckSeconds`

```cpp
inline constexpr double kCapRecheckSeconds = 1.0
```

How often (s, on the service's clock, inside a tick) each capped motor's limit is read again (MECH-S-A; HA-220 — a motor that lost power comes back at its default). A library constant.

*constant, declared at [`include/shulib/mechanism/service.hpp:190`](../../include/shulib/mechanism/service.hpp#L190).*

<a id="mechanismoutcome"></a>

## `MechanismOutcome`

```cpp
using manipulation::MechanismOutcome
```

A step's verdict vocabulary (manipulation/mechanism_outcome.hpp), named here for the service's readers: SUCCEEDED, UNCONFIRMED, TIMED_OUT, CANCELLED, STALLED, ABSENT, NOT_HOMED, REFUSED.

*type alias, declared at [`include/shulib/mechanism/service.hpp:194`](../../include/shulib/mechanism/service.hpp#L194).*

<a id="kholdintegrallimitvolts"></a>

## `kHoldIntegralLimitVolts`

```cpp
inline constexpr double kHoldIntegralLimitVolts = 3.0
```

The position HOLD loop's integral clamp, volts: the I-term never adds more than this either way (HA-182 — invented, PROVISIONAL; a LIBRARY constant for every mechanism, not a `.tuning` field and not on the boot line). Enough to carry a gravity mismatch of a few volts; small enough that a hold pushed off its target cannot wind up to full power.

*constant, declared at [`include/shulib/mechanism/service.hpp:200`](../../include/shulib/mechanism/service.hpp#L200).*

<a id="kspeedintegralwindow"></a>

## `kSpeedIntegralWindow`

```cpp
inline constexpr double kSpeedIntegralWindow = 0.10
```

The SPEED loop's integral acts only while the speed error is within this FRACTION of the target (HA-182; a library constant, like the clamps). Accumulated through the whole spin-up it winds up and overshoots (measured on the host test plant: +219 rpm at 3000 with a matched plant, +467 with a lighter wheel).

*constant, declared at [`include/shulib/mechanism/service.hpp:205`](../../include/shulib/mechanism/service.hpp#L205).*

<a id="kspeedintegrallimitvolts"></a>

## `kSpeedIntegralLimitVolts`

```cpp
inline constexpr double kSpeedIntegralLimitVolts = 3.0
```

The SPEED loop's integral clamp, volts (HA-182; a library constant).

*constant, declared at [`include/shulib/mechanism/service.hpp:207`](../../include/shulib/mechanism/service.hpp#L207).*

<a id="struct-stepreport"></a>

## `struct StepReport`

```cpp
struct StepReport
```

What the most recent step on a mechanism did — what a step reads and logs.

*struct, declared at [`include/shulib/mechanism/service.hpp:210`](../../include/shulib/mechanism/service.hpp#L210).*

<a id="stepreport-outcome"></a>

### `StepReport::outcome`

```cpp
MechanismOutcome outcome = MechanismOutcome::Running
```

The verdict; `Running` while the step is still working.

*field, declared at [`include/shulib/mechanism/service.hpp:212`](../../include/shulib/mechanism/service.hpp#L212).*

<a id="stepreport-doneby"></a>

### `StepReport::doneBy`

```cpp
DoneBy doneBy = DoneBy::None
```

Which condition ended it (`None`: the bound, a refusal or a cancel).

*field, declared at [`include/shulib/mechanism/service.hpp:214`](../../include/shulib/mechanism/service.hpp#L214).*

<a id="stepreport-counteddone"></a>

### `StepReport::countedDone`

```cpp
bool countedDone = false
```

UNCONFIRMED only because no sensor exists to confirm it — none declared (a sensorless grab, the team lead's Q2 (c)), or the declared one MISSING this boot (N8 (a)): the step is counted DONE and the chain goes on.

*field, declared at [`include/shulib/mechanism/service.hpp:218`](../../include/shulib/mechanism/service.hpp#L218).*

<a id="stepreport-elapsed"></a>

### `StepReport::elapsed`

```cpp
double elapsed = 0.0
```

Seconds from the command to the verdict.

*field, declared at [`include/shulib/mechanism/service.hpp:220`](../../include/shulib/mechanism/service.hpp#L220).*

<a id="class-slot"></a>

## `class Slot`

```cpp
class Slot : public hal::ICancellable
```

ONE mechanism's slot (header): its behaviour, its claim, its last verdict, its sensors' evidence. The per-kind slots below derive from it. Lives in the service's storage; non-copyable and non-movable, because the mechanism's claim registers it BY ADDRESS.

*class, declared at [`include/shulib/mechanism/service.hpp:243`](../../include/shulib/mechanism/service.hpp#L243).*

<a id="slot-slot"></a>

### `Slot::Slot`

```cpp
Slot(const Slot&) = delete
```

Non-copyable: a copy would leave the mechanism's claim registered to the original.

*function, declared at [`include/shulib/mechanism/service.hpp:246`](../../include/shulib/mechanism/service.hpp#L246).*

<a id="slot-operator-eq"></a>

### `Slot::operator=`

```cpp
Slot& operator=(const Slot&) = delete
```

Non-copyable: a copy would leave the mechanism's claim registered to the original. Non-copy-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:248`](../../include/shulib/mechanism/service.hpp#L248).*

<a id="slot-slot-2"></a>

### `Slot::Slot (overload 2)`

```cpp
Slot(Slot&&) = delete
```

Non-copyable: a copy would leave the mechanism's claim registered to the original. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:250`](../../include/shulib/mechanism/service.hpp#L250).*

<a id="slot-operator-eq-2"></a>

### `Slot::operator= (overload 2)`

```cpp
Slot& operator=(Slot&&) = delete
```

Non-copyable: a copy would leave the mechanism's claim registered to the original. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:252`](../../include/shulib/mechanism/service.hpp#L252).*

<a id="slot-destructor-slot"></a>

### `Slot::~Slot`

```cpp
~Slot() override = default
```

Non-copyable: a copy would leave the mechanism's claim registered to the original. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason). Releases nothing itself: the service that owns the slot ends one still holding its claim before the slot is destroyed (~MechanismService), while the slot is whole.

*function, declared at [`include/shulib/mechanism/service.hpp:255`](../../include/shulib/mechanism/service.hpp#L255).*

<a id="slot-kind"></a>

### `Slot::kind`

```cpp
[[nodiscard]] Kind kind() const noexcept
```

Which behaviour runs it.

*function, declared at [`include/shulib/mechanism/service.hpp:258`](../../include/shulib/mechanism/service.hpp#L258).*

<a id="slot-spec"></a>

### `Slot::spec`

```cpp
[[nodiscard]] const MechanismSpec& spec() const noexcept
```

The mechanism as data (the robot's list's entry, borrowed for the service's lifetime).

*function, declared at [`include/shulib/mechanism/service.hpp:260`](../../include/shulib/mechanism/service.hpp#L260).*

<a id="slot-name"></a>

### `Slot::name`

```cpp
[[nodiscard]] const char* name() const noexcept
```

Its name ("claw pivot 6").

*function, declared at [`include/shulib/mechanism/service.hpp:262`](../../include/shulib/mechanism/service.hpp#L262).*

<a id="slot-index"></a>

### `Slot::index`

```cpp
[[nodiscard]] std::uint8_t index() const noexcept
```

Its index in the robot's list.

*function, declared at [`include/shulib/mechanism/service.hpp:264`](../../include/shulib/mechanism/service.hpp#L264).*

<a id="slot-built"></a>

### `Slot::built`

```cpp
[[nodiscard]] bool built() const noexcept
```

True when its actuator was built at boot (false: ABSENT — every step answers ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:266`](../../include/shulib/mechanism/service.hpp#L266).*

<a id="slot-absence"></a>

### `Slot::absence`

```cpp
[[nodiscard]] const char* absence() const noexcept
```

Why it was not built — the boot's sentence naming the port; "" when it was.

*function, declared at [`include/shulib/mechanism/service.hpp:268`](../../include/shulib/mechanism/service.hpp#L268).*

<a id="slot-laststep"></a>

### `Slot::lastStep`

```cpp
[[nodiscard]] const StepReport& lastStep() const noexcept
```

The most recent verdict: a step's, or the STALLED that ended a hold-to-run (drive() — not a step, so stepsBegun() does not count it). Before either, a default report (`Running`, with running() false).

*function, declared at [`include/shulib/mechanism/service.hpp:272`](../../include/shulib/mechanism/service.hpp#L272).*

<a id="slot-running"></a>

### `Slot::running`

```cpp
[[nodiscard]] bool running() const noexcept
```

True while a STEP is still working (a behaviour that goes on after its step is not a step).

*function, declared at [`include/shulib/mechanism/service.hpp:274`](../../include/shulib/mechanism/service.hpp#L274).*

<a id="slot-stepsbegun"></a>

### `Slot::stepsBegun`

```cpp
[[nodiscard]] std::uint32_t stepsBegun() const noexcept
```

How many steps have begun on this slot (refused and absent ones included; a hold-to-run is not a step). 0 with lastStep() still `Running`: no verdict yet — the default report.

*function, declared at [`include/shulib/mechanism/service.hpp:279`](../../include/shulib/mechanism/service.hpp#L279).*

<a id="slot-commandseq"></a>

### `Slot::commandSeq`

```cpp
[[nodiscard]] std::uint32_t commandSeq() const noexcept
```

Bumped by EVERY command (a step, a hold-to-run, a stop) and by a stall that ends a hold-to-run: "the most recent input wins" (Q8) — an input that sees a number it did not write has been superseded.

*function, declared at [`include/shulib/mechanism/service.hpp:283`](../../include/shulib/mechanism/service.hpp#L283).*

<a id="slot-holdsclaim"></a>

### `Slot::holdsClaim`

```cpp
[[nodiscard]] bool holdsClaim() const noexcept
```

True while THIS slot holds its mechanism's claim (it is the registered claimant).

*function, declared at [`include/shulib/mechanism/service.hpp:285`](../../include/shulib/mechanism/service.hpp#L285).*

<a id="slot-seen"></a>

### `Slot::seen`

```cpp
[[nodiscard]] Seen seen() const
```

What the piece sensor says NOW — one sample, read from the device (header: positive evidence, the read-fault rule). UNKNOWN for a mechanism with no piece sensor built, and for a reading that is not a number (the F4 sensor seams promise finite readings; an implementer that broke that promise must not turn "not a number" into "no piece" — a comparison with NaN is false).

*function, declared at [`include/shulib/mechanism/service.hpp:299`](../../include/shulib/mechanism/service.hpp#L299).*

<a id="slot-sees"></a>

### `Slot::sees`

```cpp
[[nodiscard]] bool sees() const
```

True only on positive evidence of a piece: `seen() == Seen::Yes`.

*function, declared at [`include/shulib/mechanism/service.hpp:329`](../../include/shulib/mechanism/service.hpp#L329).*

<a id="slot-homeswitchpressed"></a>

### `Slot::homeSwitchPressed`

```cpp
[[nodiscard]] bool homeSwitchPressed() const
```

The HOME switch reads PRESSED now, on a FRESH read: false with no home switch, and false for a sample taken while the switch's read-fault count moved since the end of the last tick (the read-fault rule — a held level is never a home: a switch that failed, or that was dead from boot, holds its last level, and with HA-184 wrong that level could read "pressed" and home the mechanism wherever it stands). `pressedLevel` is the level that means pressed (HA-184 — unmeasured). What it cannot see: an UNPLUGGED 3-wire switch reads a level, not a failed read.

*function, declared at [`include/shulib/mechanism/service.hpp:336`](../../include/shulib/mechanism/service.hpp#L336).*

<a id="slot-declarespiecesensor"></a>

### `Slot::declaresPieceSensor`

```cpp
[[nodiscard]] bool declaresPieceSensor() const noexcept
```

True when the DECLARATION has a piece sensor, whatever this boot found.

*function, declared at [`include/shulib/mechanism/service.hpp:347`](../../include/shulib/mechanism/service.hpp#L347).*

<a id="slot-haspiecesensor"></a>

### `Slot::hasPieceSensor`

```cpp
[[nodiscard]] bool hasPieceSensor() const noexcept
```

True when the piece sensor was built this boot.

*function, declared at [`include/shulib/mechanism/service.hpp:351`](../../include/shulib/mechanism/service.hpp#L351).*

<a id="slot-piecesensormissing"></a>

### `Slot::pieceSensorMissing`

```cpp
[[nodiscard]] bool pieceSensorMissing() const noexcept
```

Declared, but MISSING this boot: its conditions run their bound and end UNCONFIRMED, counted done (N8 (a)).

*function, declared at [`include/shulib/mechanism/service.hpp:357`](../../include/shulib/mechanism/service.hpp#L357).*

<a id="slot-piecesensorplace"></a>

### `Slot::pieceSensorPlace`

```cpp
void pieceSensorPlace(char* out, std::size_t cap) const
```

Where its piece sensor is wired, as a log line says it — "port 9", "3-wire E" — from the robot's list; "its port unknown to the service" when it was bound from bare specs (or declares none). Writes at most `cap` bytes, NUL-terminated.

*function, declared at [`include/shulib/mechanism/service.hpp:361`](../../include/shulib/mechanism/service.hpp#L361).*

<a id="slot-cancel"></a>

### `Slot::cancel`

```cpp
void cancel() override
```

THE GUARD'S REACH (hal::ICancellable): the behaviour goes inert, the declared safe state is applied, the claim is released, a running step ends CANCELLED. Idempotent. A mechanism ANOTHER owner has claimed is left alone (it is not this slot's to make safe — the guard reaches that owner through its own registration).

*function, declared at [`include/shulib/mechanism/service.hpp:375`](../../include/shulib/mechanism/service.hpp#L375).*

<a id="enum-class-slot-seen"></a>

## `enum class Slot::Seen`

```cpp
enum class Seen : std::uint8_t
```

The piece sensor's evidence (header): YES and NO only on a fresh reading, UNKNOWN otherwise.

*enum class, declared at [`include/shulib/mechanism/service.hpp:290`](../../include/shulib/mechanism/service.hpp#L290).*

<a id="slot-seen-yes"></a>

### `Slot::Seen::Yes`

```cpp
Yes
```

a fresh reading that sees a piece

*enumerator, declared at [`include/shulib/mechanism/service.hpp:291`](../../include/shulib/mechanism/service.hpp#L291).*

<a id="slot-seen-no"></a>

### `Slot::Seen::No`

```cpp
No
```

a fresh reading that does not

*enumerator, declared at [`include/shulib/mechanism/service.hpp:292`](../../include/shulib/mechanism/service.hpp#L292).*

<a id="slot-seen-unknown"></a>

### `Slot::Seen::Unknown`

```cpp
Unknown
```

no evidence: no sensor this boot, a near reading with no confidence, a read fault

*enumerator, declared at [`include/shulib/mechanism/service.hpp:293`](../../include/shulib/mechanism/service.hpp#L293).*

<a id="class-powerslot"></a>

## `class PowerSlot`

```cpp
class PowerSlot final : public Slot
```

POWER: run at a voltage until a done-condition, within a bound; the driver's hold-to-run.

*class, declared at [`include/shulib/mechanism/service.hpp:820`](../../include/shulib/mechanism/service.hpp#L820).*

<a id="powerslot-powerslot"></a>

### `PowerSlot::PowerSlot`

```cpp
PowerSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i, const Devices& d)
```

Built by the service's bind(); `d.motors` is the actuator (nullptr when ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:823`](../../include/shulib/mechanism/service.hpp#L823).*

<a id="powerslot-checkrun"></a>

### `PowerSlot::checkRun`

```cpp
[[nodiscard]] const Slot* checkRun(units::Voltage volts, const Until& until, units::Time bound) const
```

STEP1-B (the STEP1 brief's D8 (a)): run()'s preconditions, in run()'s order with run()'s messages — every one of them, and nothing else. run() calls it first; a Routine's partway trigger (`r.atPercent(60).startRun(...)`) calls it where the trigger is WRITTEN, so a trigger written wrong throws on its own line, every run, whether or not the drive ever reaches its mark. It reads only its arguments, this slot's declaration and the robot's list (peer()) — never run state — so the answer at the trigger's line is the answer when it fires. A check a later chunk adds to run() goes HERE, and triggers get it too. Returns the slot whose piece sensor the condition reads (this one, or the named peer).

*function, declared at [`include/shulib/mechanism/service.hpp:840`](../../include/shulib/mechanism/service.hpp#L840).*

<a id="powerslot-run"></a>

### `PowerSlot::run`

```cpp
void run(units::Voltage volts, Until until, units::Time bound)
```

Start running at `volts` (clamped to the declaration's maxVolts) until `until`, bounded by `bound`. Returns at once; the service ticks it, and EVERY exit applies the declared safe state and releases the claim. PRECONDITIONS (nothing is commanded when one fails — a condition that can never be met is the team's mistake, said where it is made): `volts` finite; `bound` finite and > 0; a time condition finite, > 0 and at most `bound` (shorter than it beside another condition: `orAfter`); `seesPiece` / `stopsSeeingPiece` only on a mechanism whose DECLARATION has a piece sensor, named on this robot; `stalls` only with a stated stall signature; one mechanism's sensor (`!until.conflicting()`, and the step's own sensor beside a named one only when the name is this mechanism's — `until.readsOwnSensorToo()`); a named mechanism has a name.

*function, declared at [`include/shulib/mechanism/service.hpp:887`](../../include/shulib/mechanism/service.hpp#L887).*

<a id="powerslot-stop"></a>

### `PowerSlot::stop`

```cpp
void stop()
```

Stop: a running step CANCELLED, the declared safe state, the claim released. Does nothing to a mechanism another owner holds, or one that is ABSENT.

*function, declared at [`include/shulib/mechanism/service.hpp:907`](../../include/shulib/mechanism/service.hpp#L907).*

<a id="powerslot-drive"></a>

### `PowerSlot::drive`

```cpp
void drive(units::Voltage volts)
```

The DRIVER's hold-to-run: `volts` (clamped) every tick until another command — open-ended; the button's release is the caller's stop(). Call it once or every tick (each call restates the volts). A stated stall signature watches it (N10): a trip is STALLED, MECHANISM_STALLED, the safe state, released, and the command number bumped (a held button is superseded until it is released). With NO signature (allowed on a power mechanism) nothing but the release cuts it — held against its stop, it sits at the motor's current limit (A4 register HA-154). Does nothing on a mechanism another owner holds (the T3 rule), or one that is ABSENT. PRECONDITION: `volts` finite.

*function, declared at [`include/shulib/mechanism/service.hpp:923`](../../include/shulib/mechanism/service.hpp#L923).*

<a id="powerslot-commandedvolts"></a>

### `PowerSlot::commandedVolts`

```cpp
[[nodiscard]] double commandedVolts() const
```

The voltage its motors were last commanded — the DEVICE readback (0 when ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:948`](../../include/shulib/mechanism/service.hpp#L948).*

<a id="class-positionslot"></a>

## `class PositionSlot`

```cpp
class PositionSlot final : public Slot
```

POSITION (an Arm or a Lift): go to a position smoothly and HOLD it; find its zero; keep inside its travel limits (file header: the position rules). Positions are in the mechanism's units (degrees or inches); the loops run on the OUTPUT angle in degrees, where gravity's cosine lives.

*class, declared at [`include/shulib/mechanism/service.hpp:1080`](../../include/shulib/mechanism/service.hpp#L1080).*

<a id="positionslot-positionslot"></a>

### `PositionSlot::PositionSlot`

```cpp
PositionSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i, const Devices& d)
```

Built by the service's bind(); `d.motors` is the actuator (nullptr when ABSENT). Homing that needs NO motion happens here, at boot, each logged: rests at its stop; an absolute angle sensor; a home switch already pressed on a fresh read. The service's clock must be set.

*function, declared at [`include/shulib/mechanism/service.hpp:1085`](../../include/shulib/mechanism/service.hpp#L1085).*

<a id="positionslot-checkmoveto"></a>

### `PositionSlot::checkMoveTo`

```cpp
void checkMoveTo(double pos, units::Time bound, bool computed = true) const
```

STEP1-B (the STEP1 brief's D8 (a)): moveTo()'s preconditions, in moveTo()'s order with its messages — every one of them, and nothing else (checkRun()'s rule: moveTo() calls it first, a partway trigger calls it where it is written; it reads only its arguments and the declaration).

*function, declared at [`include/shulib/mechanism/service.hpp:1129`](../../include/shulib/mechanism/service.hpp#L1129).*

<a id="positionslot-moveto"></a>

### `PositionSlot::moveTo`

```cpp
void moveTo(double pos, units::Time bound, bool computed = true)
```

Move to `pos` (the mechanism's units) within `bound`, then HOLD there (file header). Before it has found its zero: NOT_HOMED, the safe state, nothing moves. A COMPUTED target past a travel limit is CLAMPED to it and logged once ("target 175.00 clamped to the travel limit 160.00 (soft stop)"). PRECONDITIONS (checked before anything changes): `pos` finite; `bound` finite and > 0; a target that is not `computed` (a preset, `arm.to(95)` — the build checked it) inside the travel limits.

*function, declared at [`include/shulib/mechanism/service.hpp:1144`](../../include/shulib/mechanism/service.hpp#L1144).*

<a id="positionslot-movetopreset"></a>

### `PositionSlot::moveToPreset`

```cpp
void moveToPreset(std::uint8_t i, units::Time bound)
```

Move to preset `i` (a named value the build checked against the limits). PRECONDITION: `i` is one of its presets.

*function, declared at [`include/shulib/mechanism/service.hpp:1186`](../../include/shulib/mechanism/service.hpp#L1186).*

<a id="positionslot-holdhere"></a>

### `PositionSlot::holdHere`

```cpp
void holdHere()
```

HOLD WHERE IT IS (gravity included; the target clamped to the travel limits) — a driver's release of a hold-to-run. Not a step; a running step ends CANCELLED (the most recent command wins). Before homing there is no position to hold: the declared safe state instead. Does nothing on a mechanism another owner holds, or one that is ABSENT.

*function, declared at [`include/shulib/mechanism/service.hpp:1194`](../../include/shulib/mechanism/service.hpp#L1194).*

<a id="positionslot-checkhome"></a>

### `PositionSlot::checkHome`

```cpp
void checkHome(units::Time bound) const
```

STEP1-B (the STEP1 brief's D8 (a)): home()'s preconditions, in home()'s order with its messages — every one of them, and nothing else (checkRun()'s rule).

*function, declared at [`include/shulib/mechanism/service.hpp:1206`](../../include/shulib/mechanism/service.hpp#L1206).*

<a id="positionslot-home"></a>

### `PositionSlot::home`

```cpp
void home(units::Time bound)
```

FIND ITS ZERO by the declaration's method, within `bound`: a homing push (`homeBySwitch`: until its switch is pressed on a fresh read; `homeByStall`: until its homing signature trips) — then SUCCEEDED, holding there, its zero logged; out of time TIMED_OUT, the safe state, the zero it had (if any) kept — a stall-home's line then says the most current its push drew when stopped (no faster than its homing signature's speed; the number that signature is sized by), or that it never got that slow. An absolute sensor is READ AGAIN (no motion; SUCCEEDED; a moved zero is logged). A mechanism that RESTS AT ITS STOP is not re-zeroed: it trusts the pose it had at boot, never the one it has now — SUCCEEDED with no change, held where it is. A homing push by switch is watched by the stated stall signature (STALLED: pushed into something with the switch never pressed). PRECONDITIONS: `bound` finite and > 0; the declaration has a homing method (`neverHomed` has no way to find its zero).

*function, declared at [`include/shulib/mechanism/service.hpp:1225`](../../include/shulib/mechanism/service.hpp#L1225).*

<a id="positionslot-jog"></a>

### `PositionSlot::jog`

```cpp
void jog(units::Voltage volts)
```

The DRIVER's hold-to-run (`whileHeld`): `volts` every tick until another command — open-ended; the button's release is the caller's holdHere(). Once homed, a push toward a travel limit is taken over and HELD AT THE LIMIT for the rest of that push (file header: the soft stop); before homing it runs at most `.unhomedVolts` either way, and with none it does nothing (the safe state). 0 V holds where it is (holdHere()). The stated stall signature watches it. Call it once or every tick (each call restates the volts). Does nothing on a mechanism another owner holds, or one that is ABSENT. PRECONDITION: `volts` finite.

*function, declared at [`include/shulib/mechanism/service.hpp:1261`](../../include/shulib/mechanism/service.hpp#L1261).*

<a id="positionslot-release"></a>

### `PositionSlot::release`

```cpp
void release()
```

LET GO: the declared safe state, the claim released, a running step CANCELLED (a hold is dropped — under Hold the motor's own brake keeps what it can). Does nothing on a mechanism another owner holds, or one that is ABSENT.

*function, declared at [`include/shulib/mechanism/service.hpp:1294`](../../include/shulib/mechanism/service.hpp#L1294).*

<a id="positionslot-homed"></a>

### `PositionSlot::homed`

```cpp
[[nodiscard]] bool homed() const noexcept
```

True once it has found its zero (at boot or by a homing step).

*function, declared at [`include/shulib/mechanism/service.hpp:1303`](../../include/shulib/mechanism/service.hpp#L1303).*

<a id="positionslot-position"></a>

### `PositionSlot::position`

```cpp
[[nodiscard]] double position() const
```

Where it is now, in its units — meaningful once homed (before, it is the encoders' — or the angle sensor's — own count through the gearing, with no zero applied). 0 when ABSENT.

*function, declared at [`include/shulib/mechanism/service.hpp:1306`](../../include/shulib/mechanism/service.hpp#L1306).*

<a id="positionslot-target"></a>

### `PositionSlot::target`

```cpp
[[nodiscard]] double target() const noexcept
```

The position the move or the hold aims at, in its units (inside the travel limits).

*function, declared at [`include/shulib/mechanism/service.hpp:1310`](../../include/shulib/mechanism/service.hpp#L1310).*

<a id="positionslot-softstops"></a>

### `PositionSlot::softStops`

```cpp
[[nodiscard]] int softStops() const noexcept
```

How many times the soft stop acted: a computed target clamped, or a push taken over and held at a limit (once per push, however long it is held there).

*function, declared at [`include/shulib/mechanism/service.hpp:1313`](../../include/shulib/mechanism/service.hpp#L1313).*

<a id="positionslot-jointdeg"></a>

### `PositionSlot::jointDeg`

```cpp
[[nodiscard]] double jointDeg() const
```

The OUTPUT angle in degrees (an angle sensor's reading when one is wired, else the motor encoders through the gearing), with its zero applied once homed. 0 when ABSENT.

*function, declared at [`include/shulib/mechanism/service.hpp:1316`](../../include/shulib/mechanism/service.hpp#L1316).*

<a id="positionslot-holding"></a>

### `PositionSlot::holding`

```cpp
[[nodiscard]] bool holding() const noexcept
```

True while it HOLDS a target (TUNE1-A — what live tuning's "never narrowed past where it is holding" reads): homed and active in a hold (after a move, a release of a hold-to-run, a homing) or a held button's soft stop at a limit. Its target is target().

*function, declared at [`include/shulib/mechanism/service.hpp:1320`](../../include/shulib/mechanism/service.hpp#L1320).*

<a id="positionslot-heldbuttonmoving"></a>

### `PositionSlot::heldButtonMoving`

```cpp
[[nodiscard]] bool heldButtonMoving() const noexcept
```

True while a HELD BUTTON moves it (TUNE1-A): a hold-to-run in progress that the soft stop has not taken over at a limit (once it has, holding() is true instead).

*function, declared at [`include/shulib/mechanism/service.hpp:1325`](../../include/shulib/mechanism/service.hpp#L1325).*

<a id="positionslot-takesrange"></a>

### `PositionSlot::takesRange`

```cpp
[[nodiscard]] bool takesRange(const PositionSpec& candidate) const noexcept
```

Whether a live re-derive may give it `candidate`'s travel limits NOW (TUNE1-A) — live tuning's check BEFORE it writes the spec (for its reason and sentence) and rederive()'s refusal AFTER (the decision that counts), through this one function. The where-it-is clause READS THE SENSOR, so the two calls can disagree when a reading moves between them (a coasting or drifting lift, jitter at the limit): rederive() then REFUSES — nothing re-derived, nothing said — and live tuning puts the old spec back and records nothing (TUNE1-A's verification round). False while it HOLDS a target the new limits leave out (the brief's re-check #7: a hold is never moved — refused instead); while a held button moves it and the limits change (the soft stop would meet a push against a limit it was not braking for); and, homed and neither, when the new limits leave WHERE IT IS further outside them than the limits in force do (TUNE1-A's review: the next hold — a HOME of a mechanism that rests at its stop, a holdHere(), a timed-out move — begins at where it is CLAMPED inside the limits, so a limit narrowed past it there would make that hold a jump with no smooth move: 8 in narrowed away asks −11 V at once; a widening, or a change that leaves it as far outside as it was, never does). Compared in OUTPUT DEGREES, the hold's and the soft stop's own unit, so a target set by a move to exactly the new limit is inside it.

*function, declared at [`include/shulib/mechanism/service.hpp:1344`](../../include/shulib/mechanism/service.hpp#L1344).*

<a id="class-speedslot"></a>

## `class SpeedSlot`

```cpp
class SpeedSlot final : public Slot
```

SPEED (a Flywheel): spin at an RPM of its output and say when it is "at speed" (file header). The step SUCCEEDS at speed; the wheel keeps spinning until stopped.

*class, declared at [`include/shulib/mechanism/service.hpp:1787`](../../include/shulib/mechanism/service.hpp#L1787).*

<a id="speedslot-speedslot"></a>

### `SpeedSlot::SpeedSlot`

```cpp
SpeedSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i, const Devices& d)
```

Built by the service's bind(); `d.motors` is the actuator (nullptr when ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:1790`](../../include/shulib/mechanism/service.hpp#L1790).*

<a id="speedslot-checkspinup"></a>

### `SpeedSlot::checkSpinUp`

```cpp
void checkSpinUp(double rpmGoal, units::Time bound, bool computed = true) const
```

STEP1-B (the STEP1 brief's D8 (a)): spinUp()'s preconditions, in spinUp()'s order with its messages — every one of them, and nothing else (checkRun()'s rule).

*function, declared at [`include/shulib/mechanism/service.hpp:1801`](../../include/shulib/mechanism/service.hpp#L1801).*

<a id="speedslot-spinup"></a>

### `SpeedSlot::spinUp`

```cpp
void spinUp(double rpmTarget, units::Time bound, bool computed = true)
```

Spin up to `rpm` (output rpm) within `bound`: SUCCEEDED once "at speed" (within the band for its time), and it KEEPS spinning; out of time TIMED_OUT, and it still spins (an unreachable speed is not a fault; a jam trips the stall signature). A COMPUTED speed past the free speed is CLAMPED to it and logged once. PRECONDITIONS (checked before anything changes): `rpm` finite; `bound` finite and > 0; a speed that is not `computed` within the free speed.

*function, declared at [`include/shulib/mechanism/service.hpp:1815`](../../include/shulib/mechanism/service.hpp#L1815).*

<a id="speedslot-spinuppreset"></a>

### `SpeedSlot::spinUpPreset`

```cpp
void spinUpPreset(std::uint8_t i, units::Time bound)
```

Spin up to preset `i`. PRECONDITION: `i` is one of its presets.

*function, declared at [`include/shulib/mechanism/service.hpp:1838`](../../include/shulib/mechanism/service.hpp#L1838).*

<a id="speedslot-stop"></a>

### `SpeedSlot::stop`

```cpp
void stop()
```

Stop: a running step CANCELLED, the declared safe state, the claim released. Does nothing to a mechanism another owner holds, or one that is ABSENT.

*function, declared at [`include/shulib/mechanism/service.hpp:1844`](../../include/shulib/mechanism/service.hpp#L1844).*

<a id="speedslot-spinning"></a>

### `SpeedSlot::spinning`

```cpp
[[nodiscard]] bool spinning() const noexcept
```

Commanded to spin (a toggle's state): true from spinUp() until a stop, a stall or a cancel.

*function, declared at [`include/shulib/mechanism/service.hpp:1853`](../../include/shulib/mechanism/service.hpp#L1853).*

<a id="speedslot-rpm"></a>

### `SpeedSlot::rpm`

```cpp
[[nodiscard]] double rpm() const
```

The output speed now, rpm: the angle sensor's when one is wired, else the motors' through the gearing. 0 when ABSENT.

*function, declared at [`include/shulib/mechanism/service.hpp:1856`](../../include/shulib/mechanism/service.hpp#L1856).*

<a id="speedslot-targetrpm"></a>

### `SpeedSlot::targetRpm`

```cpp
[[nodiscard]] double targetRpm() const noexcept
```

The speed it aims at, rpm (0 when it is not spinning).

*function, declared at [`include/shulib/mechanism/service.hpp:1866`](../../include/shulib/mechanism/service.hpp#L1866).*

<a id="speedslot-atspeed"></a>

### `SpeedSlot::atSpeed`

```cpp
[[nodiscard]] bool atSpeed() const noexcept
```

Within the band for its time, now.

*function, declared at [`include/shulib/mechanism/service.hpp:1868`](../../include/shulib/mechanism/service.hpp#L1868).*

<a id="class-pneumaticslot"></a>

## `class PneumaticSlot`

```cpp
class PneumaticSlot final : public Slot
```

PNEUMATIC: two or three named positions, each the team's level per line (Q13 (a)).

*class, declared at [`include/shulib/mechanism/service.hpp:1947`](../../include/shulib/mechanism/service.hpp#L1947).*

<a id="pneumaticslot-knoposition"></a>

### `PneumaticSlot::kNoPosition`

```cpp
static constexpr std::uint8_t kNoPosition = 0xFF
```

`commandedPosition()` when the lines match none of the positions.

*field, declared at [`include/shulib/mechanism/service.hpp:1950`](../../include/shulib/mechanism/service.hpp#L1950).*

<a id="pneumaticslot-pneumaticslot"></a>

### `PneumaticSlot::PneumaticSlot`

```cpp
PneumaticSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i, const Devices& d)
```

Built by the service's bind(); `d.pneumatic` is the actuator (nullptr when ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:1953`](../../include/shulib/mechanism/service.hpp#L1953).*

<a id="pneumaticslot-set"></a>

### `PneumaticSlot::set`

```cpp
void set(std::uint8_t p)
```

Go to position `p`: EACH line gets the position's level NOW; the step's verdict comes after the move time — SUCCEEDED when the position confirms nothing; a `grabs` / `releases` position with a piece sensor SUCCEEDS when the sensor agrees within the confirm window, else UNCONFIRMED (not counted); with NO sensor (none declared, or the declared one missing this boot) it is UNCONFIRMED and COUNTED done (Q2 (c)). The claim is held until the verdict; the level persists after it. PRECONDITION: `p` is one of its positions.

*function, declared at [`include/shulib/mechanism/service.hpp:1962`](../../include/shulib/mechanism/service.hpp#L1962).*

<a id="pneumaticslot-commandedposition"></a>

### `PneumaticSlot::commandedPosition`

```cpp
[[nodiscard]] std::uint8_t commandedPosition() const
```

The position its lines are at NOW — read back from the DEVICES (so the end-of-run guard's safe state shows here too), or kNoPosition when they match none of its positions (or it is ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:1974`](../../include/shulib/mechanism/service.hpp#L1974).*

<a id="pneumaticslot-laststepbound"></a>

### `PneumaticSlot::lastStepBound`

```cpp
[[nodiscard]] double lastStepBound() const noexcept
```

The longest a set() takes to reach its verdict: the move time plus the confirm window, seconds.

*function, declared at [`include/shulib/mechanism/service.hpp:1991`](../../include/shulib/mechanism/service.hpp#L1991).*

<a id="class-mechanismservice"></a>

## `class MechanismService`

```cpp
class MechanismService final : public motion::ITickPacer
```

THE SERVICE (file header): one slot per mechanism of the robot's list, ticked exactly once per control tick through the pacer seam or the driver loop's stated place.

*class, declared at [`include/shulib/mechanism/service.hpp:2079`](../../include/shulib/mechanism/service.hpp#L2079).*

<a id="mechanismservice-mechanismservice"></a>

### `MechanismService::MechanismService`

```cpp
explicit MechanismService(motion::ITickPacer& inner) noexcept
```

`inner` advances the world (the program's watch, or the tick pacer) and must outlive the service. Unbound, pace() ticks nothing and forwards — a program builds the pacer chain before the Robot and binds the service after it (landmine 1).

*function, declared at [`include/shulib/mechanism/service.hpp:2084`](../../include/shulib/mechanism/service.hpp#L2084).*

<a id="mechanismservice-mechanismservice-2"></a>

### `MechanismService::MechanismService (overload 2)`

```cpp
MechanismService(const MechanismService&) = delete
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it.

*function, declared at [`include/shulib/mechanism/service.hpp:2087`](../../include/shulib/mechanism/service.hpp#L2087).*

<a id="mechanismservice-operator-eq"></a>

### `MechanismService::operator=`

```cpp
MechanismService& operator=(const MechanismService&) = delete
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it. Non-copy-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:2089`](../../include/shulib/mechanism/service.hpp#L2089).*

<a id="mechanismservice-mechanismservice-3"></a>

### `MechanismService::MechanismService (overload 3)`

```cpp
MechanismService(MechanismService&&) = delete
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:2091`](../../include/shulib/mechanism/service.hpp#L2091).*

<a id="mechanismservice-operator-eq-2"></a>

### `MechanismService::operator= (overload 2)`

```cpp
MechanismService& operator=(MechanismService&&) = delete
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:2093`](../../include/shulib/mechanism/service.hpp#L2093).*

<a id="mechanismservice-destructor-mechanismservice"></a>

### `MechanismService::~MechanismService`

```cpp
~MechanismService() override
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason). A slot still holding its mechanism's claim — a step or a hold-to-run mid-flight — is ended first: CANCELLED ("its service was destroyed"), the declared safe state, the claim released. Otherwise the claim would stay registered to a destroyed slot (any later cancel-all would call into it) and the motor would stay at its last volts — the F1 operations' destructor rule (manipulation/mechanism_op.hpp). A slot holding no claim is untouched (another owner's claim is that owner's; a pneumatic's level persists). This is where bind()'s rule matters: the devices, the clock, the latch and the sink must outlive the service. Nothing escapes: a device that throws is left where it is, its claim is STILL released (no claimant may outlive the slot it points at), and the other slots are still ended.

*function, declared at [`include/shulib/mechanism/service.hpp:2103`](../../include/shulib/mechanism/service.hpp#L2103).*

<a id="mechanismservice-bind"></a>

### `MechanismService::bind`

```cpp
void bind(std::span<const MechanismSpec> specs, std::span<const Devices> devices, hal::IClock& clock, diag::FaultLatch& faults, hal::ITelemetrySink& telemetry)
```

BOOT: one slot per spec, over its devices — `devices[i]` is `specs[i]`'s, as robot::MechanismParts built them (the SAME devices the guard's span lists, landmine 20). A mechanism that was not built is a slot that answers ABSENT. Nothing is commanded — but a STATED current cap is WRITTEN into its motors here (a limit, not a command), and every motor kind's limits are read (the header's CURRENT CAP section). The specs, the devices' objects, the clock, the latch and the sink must outlive the service. PRECONDITIONS, each checked before anything is stored: bound ONCE; one Devices per spec, at most 8; each spec valid (specRefusal) and its name unique; a built mechanism's Devices carries the actuator of its kind, with the declared number of motors or lines, and an ABSENT one none; a built position or speed mechanism carries its ANGLE sensor exactly when it declares one, and a built position mechanism its HOME switch exactly when its homing is by switch (a refused angle sensor or home switch makes the mechanism ABSENT — robot::MechanismParts); a power or pneumatic one neither; a piece sensor only as declared, of the declared kind; a current limit only for a built motor mechanism's own motors (MECH-S-A). Homing that needs no motion happens here (PositionSlot): each such home is logged.

*function, declared at [`include/shulib/mechanism/service.hpp:2135`](../../include/shulib/mechanism/service.hpp#L2135).*

<a id="mechanismservice-bind-2"></a>

### `MechanismService::bind (overload 2)`

```cpp
void bind(const MechanismTable& robot, std::span<const Devices> devices, hal::IClock& clock, diag::FaultLatch& faults, hal::ITelemetrySink& telemetry)
```

BOOT from the ROBOT'S LIST (wiring.hpp's mechanisms(...)): the same, and each slot also knows its wiring (a missing sensor's line names its port). `robot` must outlive the service.

*function, declared at [`include/shulib/mechanism/service.hpp:2141`](../../include/shulib/mechanism/service.hpp#L2141).*

<a id="mechanismservice-bound"></a>

### `MechanismService::bound`

```cpp
[[nodiscard]] bool bound() const noexcept
```

True once bind() has run.

*function, declared at [`include/shulib/mechanism/service.hpp:2146`](../../include/shulib/mechanism/service.hpp#L2146).*

<a id="mechanismservice-pace"></a>

### `MechanismService::pace`

```cpp
void pace() override
```

THE PACER SEAM (autons): tick every mechanism once, then advance the world. A pace() right after tickFromDriverLoop() does NOT tick again (exactly once per tick). Unbound: forwards only.

*function, declared at [`include/shulib/mechanism/service.hpp:2151`](../../include/shulib/mechanism/service.hpp#L2151).*

<a id="mechanismservice-tickfromdriverloop"></a>

### `MechanismService::tickFromDriverLoop`

```cpp
void tickFromDriverLoop()
```

THE DRIVER LOOP'S STATED PLACE (after the driver's inputs, before perTick): tick every mechanism once; the next pace() then only advances the world.

*function, declared at [`include/shulib/mechanism/service.hpp:2160`](../../include/shulib/mechanism/service.hpp#L2160).*

<a id="mechanismservice-enterdrivercontrol"></a>

### `MechanismService::enterDriverControl`

```cpp
void enterDriverControl()
```

DRIVER CONTROL BEGINS: every slot drops its autonomous-era behaviour. A claim this service holds is released with the behaviour; a claim ANOTHER owner left (an operation on a task PROS deleted) is released WITHOUT calling its cancel() — it may have lived on the deleted task's stack — counted, and said (the team lead's ruling of 2026-09-26, T3 Part 2). A running step ends CANCELLED. Motor mechanisms are put in their declared safe state; a pneumatic one is sent its START position on the FIRST entry only (T4's ruling: the claw starts closed — whatever autonomous left it at — and afterwards keeps what it holds). Clears a re-entrancy flag a deleted task left.

*function, declared at [`include/shulib/mechanism/service.hpp:2174`](../../include/shulib/mechanism/service.hpp#L2174).*

<a id="mechanismservice-enterautonomous"></a>

### `MechanismService::enterAutonomous`

```cpp
int enterAutonomous()
```

AUTONOMOUS BEGINS (the brief's D1): PROS deletes the driver-control task on a mode change (HA-161), so driver control's own exit may never run — a driver's intake left at 12 V then ran through the whole auton. The auton runner (competition::runAuton's mechanism-service overload, MECH-A4) calls this before the guard's run: the re-entrancy flag cleared; a claim ANOTHER owner left released WITHOUT its cancel(), counted, said; every motor mechanism's behaviour stopped (its safe state); a pneumatic KEEPS its level; a running step CANCELLED. Returns how many leftover claims this call released.

*function, declared at [`include/shulib/mechanism/service.hpp:2202`](../../include/shulib/mechanism/service.hpp#L2202).*

<a id="mechanismservice-leavedrivercontrol"></a>

### `MechanismService::leaveDriverControl`

```cpp
void leaveDriverControl()
```

DRIVER CONTROL ENDS (a disable): every MOTOR mechanism's behaviour stops — inert, its declared safe state, this service's claim released; a PNEUMATIC one keeps its level (a claw must not drop what it holds — HA-157); a running step ends CANCELLED. A claim another owner took is left to that owner.

*function, declared at [`include/shulib/mechanism/service.hpp:2222`](../../include/shulib/mechanism/service.hpp#L2222).*

<a id="mechanismservice-size"></a>

### `MechanismService::size`

```cpp
[[nodiscard]] std::size_t size() const noexcept
```

How many mechanisms are bound.

*function, declared at [`include/shulib/mechanism/service.hpp:2237`](../../include/shulib/mechanism/service.hpp#L2237).*

<a id="mechanismservice-slot"></a>

### `MechanismService::slot`

```cpp
[[nodiscard]] Slot& slot(std::size_t i)
```

The slot at `i`. PRECONDITION: `i` below size().

*function, declared at [`include/shulib/mechanism/service.hpp:2239`](../../include/shulib/mechanism/service.hpp#L2239).*

<a id="mechanismservice-slotptr"></a>

### `MechanismService::slotPtr`

```cpp
[[nodiscard]] const Slot* slotPtr(std::size_t i) const noexcept
```

The slot at `i`, or nullptr past the robot's mechanisms.

*function, declared at [`include/shulib/mechanism/service.hpp:2244`](../../include/shulib/mechanism/service.hpp#L2244).*

<a id="mechanismservice-power"></a>

### `MechanismService::power`

```cpp
[[nodiscard]] PowerSlot& power(std::size_t i)
```

The POWER slot at `i`. PRECONDITION: `i` is a power mechanism.

*function, declared at [`include/shulib/mechanism/service.hpp:2248`](../../include/shulib/mechanism/service.hpp#L2248).*

<a id="mechanismservice-position"></a>

### `MechanismService::position`

```cpp
[[nodiscard]] PositionSlot& position(std::size_t i)
```

The POSITION slot at `i` (an Arm or a Lift). PRECONDITION: `i` is a position mechanism.

*function, declared at [`include/shulib/mechanism/service.hpp:2254`](../../include/shulib/mechanism/service.hpp#L2254).*

<a id="mechanismservice-speed"></a>

### `MechanismService::speed`

```cpp
[[nodiscard]] SpeedSlot& speed(std::size_t i)
```

The SPEED slot at `i` (a Flywheel). PRECONDITION: `i` is a speed mechanism.

*function, declared at [`include/shulib/mechanism/service.hpp:2260`](../../include/shulib/mechanism/service.hpp#L2260).*

<a id="mechanismservice-pneumatic"></a>

### `MechanismService::pneumatic`

```cpp
[[nodiscard]] PneumaticSlot& pneumatic(std::size_t i)
```

The PNEUMATIC slot at `i`. PRECONDITION: `i` is a pneumatic mechanism.

*function, declared at [`include/shulib/mechanism/service.hpp:2266`](../../include/shulib/mechanism/service.hpp#L2266).*

<a id="mechanismservice-find"></a>

### `MechanismService::find`

```cpp
[[nodiscard]] PowerSlot* find(const mechanism::Power& d) noexcept
```

A DECLARATION's slot (Q18 (a): the declaration is the handle), found by its KIND (the overload — it cannot mismatch) and its NAME (unique on a robot: the build refuses two). nullptr when this robot's list does not have it — a step on it then ends ABSENT, said. By name, never by address: a declaration written without `inline` has one copy per translation unit (the brief's P12), and an address lookup would end a step in another file ABSENT, silently. (So a robot two auton compiled into another robot's build commands a same-named mechanism of the same kind there — autons are per robot, landmine 6.)

*function, declared at [`include/shulib/mechanism/service.hpp:2278`](../../include/shulib/mechanism/service.hpp#L2278).*

<a id="mechanismservice-find-2"></a>

### `MechanismService::find (overload 2)`

```cpp
[[nodiscard]] PositionSlot* find(const Arm& d) noexcept
```

An ARM declaration's slot (find(const Power&)'s rule) — a position mechanism in DEGREES (a same-named LIFT, in inches, is not an arm: nullptr).

*function, declared at [`include/shulib/mechanism/service.hpp:2281`](../../include/shulib/mechanism/service.hpp#L2281).*

<a id="mechanismservice-find-3"></a>

### `MechanismService::find (overload 3)`

```cpp
[[nodiscard]] PositionSlot* find(const Lift& d) noexcept
```

A LIFT declaration's slot — a position mechanism in INCHES.

*function, declared at [`include/shulib/mechanism/service.hpp:2283`](../../include/shulib/mechanism/service.hpp#L2283).*

<a id="mechanismservice-find-4"></a>

### `MechanismService::find (overload 4)`

```cpp
[[nodiscard]] SpeedSlot* find(const mechanism::Flywheel& d) noexcept
```

A FLYWHEEL declaration's slot.

*function, declared at [`include/shulib/mechanism/service.hpp:2285`](../../include/shulib/mechanism/service.hpp#L2285).*

<a id="mechanismservice-find-5"></a>

### `MechanismService::find (overload 5)`

```cpp
[[nodiscard]] PneumaticSlot* find(const mechanism::Pneumatic& d) noexcept
```

A pneumatic declaration's slot (find(const Power&)'s rule).

*function, declared at [`include/shulib/mechanism/service.hpp:2287`](../../include/shulib/mechanism/service.hpp#L2287).*

<a id="mechanismservice-find-6"></a>

### `MechanismService::find (overload 6)`

```cpp
[[nodiscard]] PositionSlot* find(const PositionGoal& g) noexcept
```

A position GOAL's slot (`arm["score"]`, `lift.to(18_in)`): this robot's position mechanism of that name and of the goal's UNITS (an Arm's goal never finds a same-named lift) — and, for a named preset, only when it has that preset at that value (a goal of another robot's same-named mechanism must not move this one to a stranger's value).

*function, declared at [`include/shulib/mechanism/service.hpp:2294`](../../include/shulib/mechanism/service.hpp#L2294).*

<a id="mechanismservice-find-7"></a>

### `MechanismService::find (overload 7)`

```cpp
[[nodiscard]] SpeedSlot* find(const SpeedGoal& g) noexcept
```

A speed GOAL's slot (`flywheel["shoot"]`): this robot's flywheel of that name — and, for a named preset, only when it has that preset at that speed (find(const PositionGoal&)'s rule).

*function, declared at [`include/shulib/mechanism/service.hpp:2305`](../../include/shulib/mechanism/service.hpp#L2305).*

<a id="mechanismservice-find-8"></a>

### `MechanismService::find (overload 8)`

```cpp
[[nodiscard]] PneumaticSlot* find(const PneumaticGoal& g) noexcept
```

A pneumatic GOAL's slot (`claw["closed"]`): also checks the POSITION — returned only when this robot's mechanism of that name has that position's name at that index.

*function, declared at [`include/shulib/mechanism/service.hpp:2315`](../../include/shulib/mechanism/service.hpp#L2315).*

<a id="mechanismservice-slotfor"></a>

### `MechanismService::slotFor`

```cpp
[[nodiscard]] const Slot* slotFor(const char* name) const noexcept
```

The slot of ANY kind with this NAME (another mechanism's piece sensor, the driver's inputs), or nullptr.

*function, declared at [`include/shulib/mechanism/service.hpp:2329`](../../include/shulib/mechanism/service.hpp#L2329).*

<a id="mechanismservice-mechanisms"></a>

### `MechanismService::mechanisms`

```cpp
[[nodiscard]] std::span<hal::IMechanism* const> mechanisms() const noexcept
```

Every BUILT mechanism, in the list's order — the RunGuard's span (the guard reaches each one's slot through the claim it registers).

*function, declared at [`include/shulib/mechanism/service.hpp:2343`](../../include/shulib/mechanism/service.hpp#L2343).*

<a id="mechanismservice-ticks"></a>

### `MechanismService::ticks`

```cpp
[[nodiscard]] long ticks() const noexcept
```

How many ticks have run (an unbound pace() and a re-entered one are not ticks) — a program's boot proves its pacer chain with it (MECH-A5).

*function, declared at [`include/shulib/mechanism/service.hpp:2348`](../../include/shulib/mechanism/service.hpp#L2348).*

<a id="mechanismservice-leftoverclaimsreleased"></a>

### `MechanismService::leftoverClaimsReleased`

```cpp
[[nodiscard]] int leftoverClaimsReleased() const noexcept
```

How many leftover claims of another owner the mode changes have released, in total.

*function, declared at [`include/shulib/mechanism/service.hpp:2350`](../../include/shulib/mechanism/service.hpp#L2350).*

<a id="mechanismservice-clock"></a>

### `MechanismService::clock`

```cpp
[[nodiscard]] hal::IClock& clock() const
```

The clock it was bound with. PRECONDITION: bound.

*function, declared at [`include/shulib/mechanism/service.hpp:2352`](../../include/shulib/mechanism/service.hpp#L2352).*

<a id="mechanismservice-faults"></a>

### `MechanismService::faults`

```cpp
[[nodiscard]] diag::FaultLatch& faults() const
```

The fault latch it was bound with. PRECONDITION: bound.

*function, declared at [`include/shulib/mechanism/service.hpp:2357`](../../include/shulib/mechanism/service.hpp#L2357).*

<a id="mechanismservice-telemetry"></a>

### `MechanismService::telemetry`

```cpp
[[nodiscard]] hal::ITelemetrySink& telemetry() const
```

The telemetry sink it was bound with. PRECONDITION: bound.

*function, declared at [`include/shulib/mechanism/service.hpp:2362`](../../include/shulib/mechanism/service.hpp#L2362).*

<a id="mechanismservice-rederive"></a>

### `MechanismService::rederive`

```cpp
[[nodiscard]] bool rederive(std::size_t index, const char* what = nullptr)
```

RE-READ mechanism `index`'s spec after live tuning changed it IN PLACE (mechanism/live_tuning.hpp owns the table this service is bound over, so the slot's spec already IS the new one; what it copied at bind is stale until this runs — the brief's landmine 10). The slot re-derives every copy (Slot::rederiveSlot(): the PIDs' gains bumpless, the stall detectors, the limits in degrees, the soft stop's acceleration, the cap re-written when it changed) and ONE Info line says it: 'lift': retuned live -- kP 0.25 -> 0.3125 (`what`: the change in words, printed as given — live tuning gives each number in its shortest form; nullptr says "its numbers re-read"). REFUSED — false, nothing re-derived, nothing said — while a tick runs (from a predicate, a sink, anything a tick calls), while that slot's STEP runs (a change is applied only BETWEEN steps: the brief's D8), and — a position mechanism — when the spec's travel limits leave out the target it HOLDS, change while a held button moves it, or (holding nothing) leave WHERE IT IS further outside them (PositionSlot::takesRange(), read ONCE here, before anything is re-derived: a hold is never moved — the brief's re-check #7). A hold, a spin, a held button go on through it and pick the new numbers up at the next tick. Live tuning checks the same conditions BEFORE it writes the spec (for its sentence) and puts the old one back if this still refuses — a reading that moved between the two can make it refuse here. A caller that wrote the spec itself must put the old one back on false: the slot runs the table it was bound over. PRECONDITIONS: bound; `index` below size(); the slot's spec passes the build's check (specRefusal) — live tuning's own rules are stricter still (mechanism::tuningRefusal).

*function, declared at [`include/shulib/mechanism/service.hpp:2388`](../../include/shulib/mechanism/service.hpp#L2388).*

<a id="mechanismservice-canrederive"></a>

### `MechanismService::canRederive`

```cpp
[[nodiscard]] bool canRederive(std::size_t index) const noexcept
```

Whether rederive(`index`) would run NOW: bound, `index` one of its mechanisms, no tick running, and no step running on that slot (TUNE1-A).

*function, declared at [`include/shulib/mechanism/service.hpp:2408`](../../include/shulib/mechanism/service.hpp#L2408).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 147 lines, click to expand</summary>

```text

 mechanism/service.hpp — THE MECHANISM SERVICE: one object, ticked once per control tick, that runs
 every mechanism's behaviour (the mechanism core, chunk MECH-A2, 2026-10-15).

 ── Why one ticker ───────────────────────────────────────────────────────────────────
 A mechanism step must keep running while the robot does something else: an intake started before
 a drive must stop the moment its sensor sees a piece MID-drive; a claw step must reach its verdict
 while the robot turns. Nothing in a blocking drive verb ticks a mechanism — the drive's wait is the
 scheduler's, and a student's own loop around it would be a second loop owner. So the behaviours
 live HERE, in one object every control tick reaches through exactly one of two seams:
   * THE PACER SEAM (autons): the service IS a motion::ITickPacer decorator. A program constructs it
     around the pacer that advances the world and puts the RunGuard AROUND it — Chassis → RunGuard →
     MechanismService → (the program's watch) → the tick pacer — so every pace of every blocking verb
     first lets the guard check its deadlines, then ticks every mechanism, then advances the world
     (the requirements' order, :116-155).
   * THE DRIVER LOOP'S STATED PLACE: tickFromDriverLoop(), after the driver's inputs, before the
     loop's perTick (MECH-A4 wires it into runDriverControl). A pace() that follows such a tick does
     NOT tick again, so the service ticks EXACTLY ONCE per tick whichever pacer the loop is handed —
     the raw one, this service, or a guard around it.

 ── Each mechanism's SLOT is its claimant ────────────────────────────────────────────
 A slot takes its mechanism's claim (hal/mechanism.hpp) with ITSELF as the registered claimant
 whenever it commands, so the RunGuard's cancel-all — and, since this run, its deadline and abandon
 cut (sequence/run_guard.hpp) — reaches every running behaviour: inert, the declared safe state,
 the claim released, a running step CANCELLED. A mechanism ANOTHER owner has claimed (an F1
 operation a team ticks itself) is never commanded: every step on it is REFUSED and a hold-to-run
 does nothing — "one ticker per operation", never a take-over. A claim RELEASED outside the
 service (no cancel(), nobody took it) ends the slot's behaviour at its next tick, a running step
 CANCELLED: a motor mechanism is put in its safe state, a pneumatic keeps its level. One behaviour
 per mechanism at a time, and the MOST RECENT command wins: a new step, a hold-to-run or a stop
 replaces a running step, which ends CANCELLED, said. `commandSeq()` is bumped by every command —
 how the driver's inputs (MECH-A4) see that something else has spoken since they last did.

 ── Piece sensors are POSITIVE evidence (the brief's N8, amended) ────────────────────
 `Slot::seen()` answers YES only on a fresh reading that sees (a distance closer than its threshold
 with confidence above 0; an optical at or above its proximity; a switch at its pressed level), NO
 only on a fresh reading that does not (a distance at or past the threshold — the adapter's honest
 393.66 in for PROS's 9999 "no object" included; an optical below; a switch released), and UNKNOWN
 otherwise: no sensor built this boot; a distance held NEAR with confidence 0; a reading that is
 not a number (the F4 seams promise finite ones — a NaN must not compare its way to "no piece");
 and ANY sample taken while the sensor's READ-FAULT count moved since the end of the service's
 previous tick (the provider's hal::IReadFaults — a PROS adapter answers a failed read with its
 last good value, and a distance sensor that failed while it read "far" is byte for byte the 9999
 "no object" reading). UNKNOWN satisfies NEITHER `seesPiece` nor `stopsSeeingPiece`. At a step's
 bound: a sensor DECLARED but MISSING this boot → UNCONFIRMED, COUNTED done (the team lead's Q2 (c)
 and N8 (a): a loose cable costs the step's time, not the auton); a sensor that gave no evidence on
 even ONE sample of the step → UNCONFIRMED, NOT counted ("its piece sensor stopped answering": it
 may have missed the very piece the step waited for); otherwise TIMED_OUT. The
 count is compared against the END of the previous tick — never a before/after pair inside one
 read — so two reads of one sensor in one tick agree, and a device that answers again is fresh at
 the next tick (the brief's landmine 22). What it cannot see: an UNPLUGGED 3-wire switch reads a
 level, not PROS_ERR (HA-184).

 ── The four behaviours ──────────────────────────────────────────────────────────────
 MECH-A2 built the slot, the two seams, the claimant, the three mode changes, the lookups and the
 POWER and PNEUMATIC behaviours; MECH-A3 (2026-10-15) the POSITION (Arm, Lift) and SPEED (Flywheel)
 ones, so bind() takes every kind a robot's list can hold. Their rules, each pinned by a test:
   * POSITION. Before it has found its zero a move is NOT_HOMED and a held button jogs it at most
     `.unhomedVolts` (0: nothing moves it — no limits are known, and no power is invented). It finds
     its zero with NO motion at bind (rests at its stop; an absolute angle sensor; a home switch
     already pressed on a FRESH read) or by a homing STEP (push until its switch is pressed, or until
     it stalls against its stop) — and every (re-)home logs where its zero went, a Warn when a
     re-home moves it past the "reached" tolerance (homing into a wedged piece sets a wrong zero
     silently otherwise). A move is a trapezoid profile in OUTPUT degrees (a lift's speed in inches
     turned into degrees by its steepest slope, so the plan never asks more of the motor than the stated
     speed there), tracked by P + D with feedforward (kS, kV, kA and gravity: kG·cos(angle −
     level) for an arm, kG for a lift); when the profile ends a FRESH hold loop (P + I + D, its
     integral clamped to kHoldIntegralLimitVolts) takes over, so the integral never winds up behind
     the profile. REACHED is judged in the mechanism's OWN units (a degree tolerance converted at a
     table's end or a linkage's top is 0), for the settle time: SUCCEEDED, and the HOLD goes on. Out
     of time: TIMED_OUT, and it HOLDS WHERE IT STOPPED — never pushing on toward a target it could not
     reach (a blocked arm would sit at stall current). No hold target is ever past a travel
     limit. A constant target past a limit never compiles; a COMPUTED one is clamped and logged once
     (the soft stop); a held button pushing toward a limit is taken over where it could no longer stop
     before it at the smooth move's own acceleration — brought to the limit by the move loop, then
     HELD AT THE LIMIT by a fresh closed-loop hold for the rest of that push (one soft stop per push),
     never pushed toward the limit harder than the button pushes.
   * SPEED. Feedforward (kS + kV·ω) plus P on the speed error, plus an integral that acts only within
     kSpeedIntegralWindow of the target (from the start it winds up and overshoots) and is clamped to
     kSpeedIntegralLimitVolts. "At speed" — within the band for its time — is SUCCEEDED, and it KEEPS
     spinning until stopped; out of time is TIMED_OUT and it still spins (an unreachable wheel spins;
     a jammed one trips its stall signature).
   * THE STALL WATCH: a stated stall signature watches EVERY behaviour of a motor kind — a
     move, the hold after it, a jog, the release's hold, a homing push by switch, a spin before and
     after "at speed", a power run and a hold-to-run (homing by stall keeps its own signature). A trip
     is STALLED, MECHANISM_STALLED once, the declared safe state, the claim released; one that ends a
     behaviour AFTER its step (a hold, a jog, a spin) also bumps the command number — how the driver's
     inputs (mechanism/driver_inputs.hpp, MECH-A4) see that a held button is superseded until it is released.
 The library's own numbers in all of this are HA-182 — invented, PROVISIONAL — in two groups. The
 starting gains, profile and "reached" (kinds.hpp) are per mechanism: startingValuesLine()
 (wiring.hpp) words them for a boot line (the programs print it since MECH-A5) and a declaration's
 `.tuning` replaces each one. The hold's integral clamp and the speed integral's window and clamp
 (below) are LIBRARY constants: not on the boot line, not `.tuning` fields — only an edit here
 changes them, for every mechanism.

 ── Re-entrancy (requirements :142-152) ──────────────────────────────────────────────
 A tick never calls a blocking verb, a wait or the scheduler's cancel(), and is never re-entered
 (a precondition). A predicate may READ a slot (`seen()`, `running()`), never tick it. A tick that
 never finished — its task deleted mid-tick by a PROS mode change (HA-161), or a throw out of a
 device — leaves the re-entrancy flag set: the next mode change (enterDriverControl /
 enterAutonomous) clears it.

 ── The honest limit ─────────────────────────────────────────────────────────────────
 The service runs ONLY when something ticks it: inside a shulib motion, wait or pause, and (from
 MECH-A4, which wires it into the driver loop) every driver-loop tick. A student's own busy loop (`pros::delay` in an auton, a then() action that
 blocks) STARVES it, and a starved service leaves each mechanism at its LAST command — not held,
 not limited, not safe (the brief's D11, measured: 11.74 V held for 1.5 s). The run guard says the
 same of itself: nothing preempts pure user code.

 ── The CURRENT CAP (chunk MECH-S-A, 2026-11-05; the MECH-S brief's D1 (a), D2 (a), D12 (b)) ─
 A mechanism that STATES `.currentCap` has it written into EVERY one of its motors at bind, through
 each motor's own limit (Devices::currentLimits — hal/motor_current_limit.hpp), and READ BACK; from
 then on the MOTOR enforces it, ticked or not (A4 register HA-219 — how fast, unmeasured). The boot
 says it once, per motor AND in all:
     'lift': current cap 1.20 A per motor (2 motors: 2.40 A in all) -- set and read back
 and WARNS for each motor that would not take it (the port, the asked and the read-back numbers — the
 mechanism still runs), for a provider that gave no limit to set ("this provider cannot cap a
 motor"), and for each stated stall or homing signature within 0.2 A of the cap (a capped motor may
 read a little under its cap, so such a signature may never trip — A4 register HA-223; the build
 already refuses one at or above it). Every one of these comparisons is made in the DEVICE's unit,
 WHOLE MILLIAMPS (a V5 limit is whole mA): the cap as hal::motorCurrentLimitToMilliamps sends it, the
 read-back rounded to the mA — so a typed 2.4995 A (sent as 2500 mA) reads back as itself — and a
 signature as the FIRST whole-mA reading that trips it (a signature is never sent, so never rounded:
 0.9996 A and 0.9994 A both trip at 1000 mA). The 0.2 A margin is 200 mA between those two integers,
 inclusive: under a 1.2 A cap, 1.0 A, 0.9996 A and 0.9994 A all warn; 0.999 A does not. A mechanism that states NO cap has each motor's limit READ and
 NOTHING written (D12 (b), until robot two's 10-motor boot is measured — HA-221): a limit that reads
 other than the motor's 2.5 A default gets one Info line naming both possible causes (a cap another
 program left — HA-220 — or VEXos's own sharing — HA-221) and "nothing written".
 ONCE A SECOND, on the service's clock and INSIDE a tick (never its own task — a starved service does
 not re-check, and the motor keeps its cap meanwhile: that is the point of D1), each capped motor's
 limit is read again: one that reads back the motor's DEFAULT (it lost power and came back — HA-220)
 is written again, with one Warn; one that reads ANY OTHER number is a Warn ONCE and is NOT written
 again (something else set it — the library never fights it every second). No verdict changes and
 nothing is refused at boot: the cap is a limit, never a gate. Choosing the number (four currents in
 ORDER — the heaviest hold < the signature < the cap − 0.2 A; the heaviest move's peak < the cap; the
 cap ≤ the string's safe pull) is the guide's (chapter 18); the cap does not limit a carriage's
 momentum at a stop (HA-222).

 ── What a step says ─────────────────────────────────────────────────────────────────
 Every step's verdict is ONE line, tag MECH — Info when SUCCEEDED, Warn otherwise:
     'intake': SUCCEEDED -- sees a piece in 0.97s
     'claw': UNCONFIRMED -- time in 0.50s (no piece sensor to confirm it; counted done)
 (the format, field by field: docs/guide/11-reading-the-diagnostics.md). STALLED also raises
 MECHANISM_STALLED, once per stall.

 PROS-free, allocation-free: every slot lives in the service's own fixed storage, so a program keeps
 the service in static storage (a deleted task cannot take it with it).
```

</details>
