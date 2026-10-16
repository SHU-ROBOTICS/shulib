<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/service.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `service.hpp`

mechanism/service.hpp — THE MECHANISM SERVICE: one object, ticked once per control tick, that runs every mechanism's behaviour.

This header declares **6** types (69 members) and **1** type alias.

Extracted from [`include/shulib/mechanism/service.hpp`](../../include/shulib/mechanism/service.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`MechanismOutcome`](#mechanismoutcome) — *type alias*
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
  - [`run`](#powerslot-run)
  - [`stop`](#powerslot-stop)
  - [`drive`](#powerslot-drive)
  - [`commandedVolts`](#powerslot-commandedvolts)
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
  - [`pneumatic`](#mechanismservice-pneumatic)
  - [`find`](#mechanismservice-find)
  - [`find (overload 2)`](#mechanismservice-find-2)
  - [`find (overload 3)`](#mechanismservice-find-3)
  - [`slotFor`](#mechanismservice-slotfor)
  - [`mechanisms`](#mechanismservice-mechanisms)
  - [`ticks`](#mechanismservice-ticks)
  - [`leftoverClaimsReleased`](#mechanismservice-leftoverclaimsreleased)
  - [`clock`](#mechanismservice-clock)
  - [`faults`](#mechanismservice-faults)
  - [`telemetry`](#mechanismservice-telemetry)

<a id="mechanismoutcome"></a>

## `MechanismOutcome`

```cpp
using manipulation::MechanismOutcome
```

A step's verdict vocabulary (manipulation/mechanism_outcome.hpp), named here for the service's readers: SUCCEEDED, UNCONFIRMED, TIMED_OUT, CANCELLED, STALLED, ABSENT, NOT_HOMED, REFUSED.

*type alias, declared at [`include/shulib/mechanism/service.hpp:116`](../../include/shulib/mechanism/service.hpp#L116).*

<a id="struct-stepreport"></a>

## `struct StepReport`

```cpp
struct StepReport
```

What the most recent step on a mechanism did — what a step reads and logs.

*struct, declared at [`include/shulib/mechanism/service.hpp:119`](../../include/shulib/mechanism/service.hpp#L119).*

<a id="stepreport-outcome"></a>

### `StepReport::outcome`

```cpp
MechanismOutcome outcome = MechanismOutcome::Running
```

The verdict; `Running` while the step is still working.

*field, declared at [`include/shulib/mechanism/service.hpp:121`](../../include/shulib/mechanism/service.hpp#L121).*

<a id="stepreport-doneby"></a>

### `StepReport::doneBy`

```cpp
DoneBy doneBy = DoneBy::None
```

Which condition ended it (`None`: the bound, a refusal or a cancel).

*field, declared at [`include/shulib/mechanism/service.hpp:123`](../../include/shulib/mechanism/service.hpp#L123).*

<a id="stepreport-counteddone"></a>

### `StepReport::countedDone`

```cpp
bool countedDone = false
```

UNCONFIRMED only because no sensor exists to confirm it — none declared (a sensorless grab, the team lead's Q2 (c)), or the declared one MISSING this boot (N8 (a)): the step is counted DONE and the chain goes on.

*field, declared at [`include/shulib/mechanism/service.hpp:127`](../../include/shulib/mechanism/service.hpp#L127).*

<a id="stepreport-elapsed"></a>

### `StepReport::elapsed`

```cpp
double elapsed = 0.0
```

Seconds from the command to the verdict.

*field, declared at [`include/shulib/mechanism/service.hpp:129`](../../include/shulib/mechanism/service.hpp#L129).*

<a id="class-slot"></a>

## `class Slot`

```cpp
class Slot : public hal::ICancellable
```

ONE mechanism's slot (header): its behaviour, its claim, its last verdict, its sensors' evidence. The per-kind slots below derive from it. Lives in the service's storage; non-copyable and non-movable, because the mechanism's claim registers it BY ADDRESS.

*class, declared at [`include/shulib/mechanism/service.hpp:152`](../../include/shulib/mechanism/service.hpp#L152).*

<a id="slot-slot"></a>

### `Slot::Slot`

```cpp
Slot(const Slot&) = delete
```

Non-copyable: a copy would leave the mechanism's claim registered to the original.

*function, declared at [`include/shulib/mechanism/service.hpp:155`](../../include/shulib/mechanism/service.hpp#L155).*

<a id="slot-operator-eq"></a>

### `Slot::operator=`

```cpp
Slot& operator=(const Slot&) = delete
```

Non-copyable: a copy would leave the mechanism's claim registered to the original. Non-copy-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:157`](../../include/shulib/mechanism/service.hpp#L157).*

<a id="slot-slot-2"></a>

### `Slot::Slot (overload 2)`

```cpp
Slot(Slot&&) = delete
```

Non-copyable: a copy would leave the mechanism's claim registered to the original. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:159`](../../include/shulib/mechanism/service.hpp#L159).*

<a id="slot-operator-eq-2"></a>

### `Slot::operator= (overload 2)`

```cpp
Slot& operator=(Slot&&) = delete
```

Non-copyable: a copy would leave the mechanism's claim registered to the original. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:161`](../../include/shulib/mechanism/service.hpp#L161).*

<a id="slot-destructor-slot"></a>

### `Slot::~Slot`

```cpp
~Slot() override = default
```

Non-copyable: a copy would leave the mechanism's claim registered to the original. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason). Releases nothing itself: the service that owns the slot ends one still holding its claim before the slot is destroyed (~MechanismService), while the slot is whole.

*function, declared at [`include/shulib/mechanism/service.hpp:164`](../../include/shulib/mechanism/service.hpp#L164).*

<a id="slot-kind"></a>

### `Slot::kind`

```cpp
[[nodiscard]] Kind kind() const noexcept
```

Which behaviour runs it.

*function, declared at [`include/shulib/mechanism/service.hpp:167`](../../include/shulib/mechanism/service.hpp#L167).*

<a id="slot-spec"></a>

### `Slot::spec`

```cpp
[[nodiscard]] const MechanismSpec& spec() const noexcept
```

The mechanism as data (the robot's list's entry, borrowed for the service's lifetime).

*function, declared at [`include/shulib/mechanism/service.hpp:169`](../../include/shulib/mechanism/service.hpp#L169).*

<a id="slot-name"></a>

### `Slot::name`

```cpp
[[nodiscard]] const char* name() const noexcept
```

Its name ("claw pivot 6").

*function, declared at [`include/shulib/mechanism/service.hpp:171`](../../include/shulib/mechanism/service.hpp#L171).*

<a id="slot-index"></a>

### `Slot::index`

```cpp
[[nodiscard]] std::uint8_t index() const noexcept
```

Its index in the robot's list.

*function, declared at [`include/shulib/mechanism/service.hpp:173`](../../include/shulib/mechanism/service.hpp#L173).*

<a id="slot-built"></a>

### `Slot::built`

```cpp
[[nodiscard]] bool built() const noexcept
```

True when its actuator was built at boot (false: ABSENT — every step answers ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:175`](../../include/shulib/mechanism/service.hpp#L175).*

<a id="slot-absence"></a>

### `Slot::absence`

```cpp
[[nodiscard]] const char* absence() const noexcept
```

Why it was not built — the boot's sentence naming the port; "" when it was.

*function, declared at [`include/shulib/mechanism/service.hpp:177`](../../include/shulib/mechanism/service.hpp#L177).*

<a id="slot-laststep"></a>

### `Slot::lastStep`

```cpp
[[nodiscard]] const StepReport& lastStep() const noexcept
```

The most recent verdict: a step's, or the STALLED that ended a hold-to-run (drive() — not a step, so stepsBegun() does not count it). Before either, a default report (`Running`, with running() false).

*function, declared at [`include/shulib/mechanism/service.hpp:181`](../../include/shulib/mechanism/service.hpp#L181).*

<a id="slot-running"></a>

### `Slot::running`

```cpp
[[nodiscard]] bool running() const noexcept
```

True while a STEP is still working (a behaviour that goes on after its step is not a step).

*function, declared at [`include/shulib/mechanism/service.hpp:183`](../../include/shulib/mechanism/service.hpp#L183).*

<a id="slot-stepsbegun"></a>

### `Slot::stepsBegun`

```cpp
[[nodiscard]] std::uint32_t stepsBegun() const noexcept
```

How many steps have begun on this slot (refused and absent ones included; a hold-to-run is not a step). 0 with lastStep() still `Running`: no verdict yet — the default report.

*function, declared at [`include/shulib/mechanism/service.hpp:188`](../../include/shulib/mechanism/service.hpp#L188).*

<a id="slot-commandseq"></a>

### `Slot::commandSeq`

```cpp
[[nodiscard]] std::uint32_t commandSeq() const noexcept
```

Bumped by EVERY command (a step, a hold-to-run, a stop) and by a stall that ends a hold-to-run: "the most recent input wins" (Q8) — an input that sees a number it did not write has been superseded.

*function, declared at [`include/shulib/mechanism/service.hpp:192`](../../include/shulib/mechanism/service.hpp#L192).*

<a id="slot-holdsclaim"></a>

### `Slot::holdsClaim`

```cpp
[[nodiscard]] bool holdsClaim() const noexcept
```

True while THIS slot holds its mechanism's claim (it is the registered claimant).

*function, declared at [`include/shulib/mechanism/service.hpp:194`](../../include/shulib/mechanism/service.hpp#L194).*

<a id="slot-seen"></a>

### `Slot::seen`

```cpp
[[nodiscard]] Seen seen() const
```

What the piece sensor says NOW — one sample, read from the device (header: positive evidence, the read-fault rule). UNKNOWN for a mechanism with no piece sensor built, and for a reading that is not a number (the F4 sensor seams promise finite readings; an implementer that broke that promise must not turn "not a number" into "no piece" — a comparison with NaN is false).

*function, declared at [`include/shulib/mechanism/service.hpp:208`](../../include/shulib/mechanism/service.hpp#L208).*

<a id="slot-sees"></a>

### `Slot::sees`

```cpp
[[nodiscard]] bool sees() const
```

True only on positive evidence of a piece: `seen() == Seen::Yes`.

*function, declared at [`include/shulib/mechanism/service.hpp:238`](../../include/shulib/mechanism/service.hpp#L238).*

<a id="slot-declarespiecesensor"></a>

### `Slot::declaresPieceSensor`

```cpp
[[nodiscard]] bool declaresPieceSensor() const noexcept
```

True when the DECLARATION has a piece sensor, whatever this boot found.

*function, declared at [`include/shulib/mechanism/service.hpp:240`](../../include/shulib/mechanism/service.hpp#L240).*

<a id="slot-haspiecesensor"></a>

### `Slot::hasPieceSensor`

```cpp
[[nodiscard]] bool hasPieceSensor() const noexcept
```

True when the piece sensor was built this boot.

*function, declared at [`include/shulib/mechanism/service.hpp:244`](../../include/shulib/mechanism/service.hpp#L244).*

<a id="slot-piecesensormissing"></a>

### `Slot::pieceSensorMissing`

```cpp
[[nodiscard]] bool pieceSensorMissing() const noexcept
```

Declared, but MISSING this boot: its conditions run their bound and end UNCONFIRMED, counted done (N8 (a)).

*function, declared at [`include/shulib/mechanism/service.hpp:250`](../../include/shulib/mechanism/service.hpp#L250).*

<a id="slot-piecesensorplace"></a>

### `Slot::pieceSensorPlace`

```cpp
void pieceSensorPlace(char* out, std::size_t cap) const
```

Where its piece sensor is wired, as a log line says it — "port 9", "3-wire E" — from the robot's list; "its port unknown to the service" when it was bound from bare specs (or declares none). Writes at most `cap` bytes, NUL-terminated.

*function, declared at [`include/shulib/mechanism/service.hpp:254`](../../include/shulib/mechanism/service.hpp#L254).*

<a id="slot-cancel"></a>

### `Slot::cancel`

```cpp
void cancel() override
```

THE GUARD'S REACH (hal::ICancellable): the behaviour goes inert, the declared safe state is applied, the claim is released, a running step ends CANCELLED. Idempotent. A mechanism ANOTHER owner has claimed is left alone (it is not this slot's to make safe — the guard reaches that owner through its own registration).

*function, declared at [`include/shulib/mechanism/service.hpp:268`](../../include/shulib/mechanism/service.hpp#L268).*

<a id="enum-class-slot-seen"></a>

## `enum class Slot::Seen`

```cpp
enum class Seen : std::uint8_t
```

The piece sensor's evidence (header): YES and NO only on a fresh reading, UNKNOWN otherwise.

*enum class, declared at [`include/shulib/mechanism/service.hpp:199`](../../include/shulib/mechanism/service.hpp#L199).*

<a id="slot-seen-yes"></a>

### `Slot::Seen::Yes`

```cpp
Yes
```

a fresh reading that sees a piece

*enumerator, declared at [`include/shulib/mechanism/service.hpp:200`](../../include/shulib/mechanism/service.hpp#L200).*

<a id="slot-seen-no"></a>

### `Slot::Seen::No`

```cpp
No
```

a fresh reading that does not

*enumerator, declared at [`include/shulib/mechanism/service.hpp:201`](../../include/shulib/mechanism/service.hpp#L201).*

<a id="slot-seen-unknown"></a>

### `Slot::Seen::Unknown`

```cpp
Unknown
```

no evidence: no sensor this boot, a near reading with no confidence, a read fault

*enumerator, declared at [`include/shulib/mechanism/service.hpp:202`](../../include/shulib/mechanism/service.hpp#L202).*

<a id="class-powerslot"></a>

## `class PowerSlot`

```cpp
class PowerSlot final : public Slot
```

POWER: run at a voltage until a done-condition, within a bound; the driver's hold-to-run.

*class, declared at [`include/shulib/mechanism/service.hpp:411`](../../include/shulib/mechanism/service.hpp#L411).*

<a id="powerslot-powerslot"></a>

### `PowerSlot::PowerSlot`

```cpp
PowerSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i, const Devices& d)
```

Built by the service's bind(); `d.motors` is the actuator (nullptr when ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:414`](../../include/shulib/mechanism/service.hpp#L414).*

<a id="powerslot-run"></a>

### `PowerSlot::run`

```cpp
void run(units::Voltage volts, Until until, units::Time bound)
```

Start running at `volts` (clamped to the declaration's maxVolts) until `until`, bounded by `bound`. Returns at once; the service ticks it, and EVERY exit applies the declared safe state and releases the claim. PRECONDITIONS (nothing is commanded when one fails — a condition that can never be met is the team's mistake, said where it is made): `volts` finite; `bound` finite and > 0; a time condition finite, > 0 and at most `bound` (shorter than it beside another condition: `orAfter`); `seesPiece` / `stopsSeeingPiece` only on a mechanism whose DECLARATION has a piece sensor, named on this robot; `stalls` only with a stated stall signature; one mechanism's sensor (`!until.conflicting()`, and the step's own sensor beside a named one only when the name is this mechanism's — `until.readsOwnSensorToo()`); a named mechanism has a name.

*function, declared at [`include/shulib/mechanism/service.hpp:432`](../../include/shulib/mechanism/service.hpp#L432).*

<a id="powerslot-stop"></a>

### `PowerSlot::stop`

```cpp
void stop()
```

Stop: a running step CANCELLED, the declared safe state, the claim released. Does nothing to a mechanism another owner holds, or one that is ABSENT.

*function, declared at [`include/shulib/mechanism/service.hpp:485`](../../include/shulib/mechanism/service.hpp#L485).*

<a id="powerslot-drive"></a>

### `PowerSlot::drive`

```cpp
void drive(units::Voltage volts)
```

The DRIVER's hold-to-run: `volts` (clamped) every tick until another command — open-ended; the button's release is the caller's stop(). Call it once or every tick (each call restates the volts). A stated stall signature watches it (N10): a trip is STALLED, MECHANISM_STALLED, the safe state, released, and the command number bumped (a held button is superseded until it is released). With NO signature (allowed on a power mechanism) nothing but the release cuts it — held against its stop, it sits at the motor's current limit (A4 register HA-154). Does nothing on a mechanism another owner holds (the T3 rule), or one that is ABSENT. PRECONDITION: `volts` finite.

*function, declared at [`include/shulib/mechanism/service.hpp:501`](../../include/shulib/mechanism/service.hpp#L501).*

<a id="powerslot-commandedvolts"></a>

### `PowerSlot::commandedVolts`

```cpp
[[nodiscard]] double commandedVolts() const
```

The voltage its motors were last commanded — the DEVICE readback (0 when ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:526`](../../include/shulib/mechanism/service.hpp#L526).*

<a id="class-pneumaticslot"></a>

## `class PneumaticSlot`

```cpp
class PneumaticSlot final : public Slot
```

PNEUMATIC: two or three named positions, each the team's level per line (Q13 (a)).

*class, declared at [`include/shulib/mechanism/service.hpp:645`](../../include/shulib/mechanism/service.hpp#L645).*

<a id="pneumaticslot-knoposition"></a>

### `PneumaticSlot::kNoPosition`

```cpp
static constexpr std::uint8_t kNoPosition = 0xFF
```

`commandedPosition()` when the lines match none of the positions.

*field, declared at [`include/shulib/mechanism/service.hpp:648`](../../include/shulib/mechanism/service.hpp#L648).*

<a id="pneumaticslot-pneumaticslot"></a>

### `PneumaticSlot::PneumaticSlot`

```cpp
PneumaticSlot(detail::SlotHost& host, const MechanismSpec& spec, const MechanismWiring* wiring, std::uint8_t i, const Devices& d)
```

Built by the service's bind(); `d.pneumatic` is the actuator (nullptr when ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:651`](../../include/shulib/mechanism/service.hpp#L651).*

<a id="pneumaticslot-set"></a>

### `PneumaticSlot::set`

```cpp
void set(std::uint8_t p)
```

Go to position `p`: EACH line gets the position's level NOW; the step's verdict comes after the move time — SUCCEEDED when the position confirms nothing; a `grabs` / `releases` position with a piece sensor SUCCEEDS when the sensor agrees within the confirm window, else UNCONFIRMED (not counted); with NO sensor (none declared, or the declared one missing this boot) it is UNCONFIRMED and COUNTED done (Q2 (c)). The claim is held until the verdict; the level persists after it. PRECONDITION: `p` is one of its positions.

*function, declared at [`include/shulib/mechanism/service.hpp:660`](../../include/shulib/mechanism/service.hpp#L660).*

<a id="pneumaticslot-commandedposition"></a>

### `PneumaticSlot::commandedPosition`

```cpp
[[nodiscard]] std::uint8_t commandedPosition() const
```

The position its lines are at NOW — read back from the DEVICES (so the end-of-run guard's safe state shows here too), or kNoPosition when they match none of its positions (or it is ABSENT).

*function, declared at [`include/shulib/mechanism/service.hpp:672`](../../include/shulib/mechanism/service.hpp#L672).*

<a id="pneumaticslot-laststepbound"></a>

### `PneumaticSlot::lastStepBound`

```cpp
[[nodiscard]] double lastStepBound() const noexcept
```

The longest a set() takes to reach its verdict: the move time plus the confirm window, seconds.

*function, declared at [`include/shulib/mechanism/service.hpp:689`](../../include/shulib/mechanism/service.hpp#L689).*

<a id="class-mechanismservice"></a>

## `class MechanismService`

```cpp
class MechanismService final : public motion::ITickPacer
```

THE SERVICE (file header): one slot per mechanism of the robot's list, ticked exactly once per control tick through the pacer seam or the driver loop's stated place.

*class, declared at [`include/shulib/mechanism/service.hpp:777`](../../include/shulib/mechanism/service.hpp#L777).*

<a id="mechanismservice-mechanismservice"></a>

### `MechanismService::MechanismService`

```cpp
explicit MechanismService(motion::ITickPacer& inner) noexcept
```

`inner` advances the world (the program's watch, or the tick pacer) and must outlive the service. Unbound, pace() ticks nothing and forwards — a program builds the pacer chain before the Robot and binds the service after it (landmine 1).

*function, declared at [`include/shulib/mechanism/service.hpp:782`](../../include/shulib/mechanism/service.hpp#L782).*

<a id="mechanismservice-mechanismservice-2"></a>

### `MechanismService::MechanismService (overload 2)`

```cpp
MechanismService(const MechanismService&) = delete
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it.

*function, declared at [`include/shulib/mechanism/service.hpp:785`](../../include/shulib/mechanism/service.hpp#L785).*

<a id="mechanismservice-operator-eq"></a>

### `MechanismService::operator=`

```cpp
MechanismService& operator=(const MechanismService&) = delete
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it. Non-copy-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:787`](../../include/shulib/mechanism/service.hpp#L787).*

<a id="mechanismservice-mechanismservice-3"></a>

### `MechanismService::MechanismService (overload 3)`

```cpp
MechanismService(MechanismService&&) = delete
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:789`](../../include/shulib/mechanism/service.hpp#L789).*

<a id="mechanismservice-operator-eq-2"></a>

### `MechanismService::operator= (overload 2)`

```cpp
MechanismService& operator=(MechanismService&&) = delete
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason).

*function, declared at [`include/shulib/mechanism/service.hpp:791`](../../include/shulib/mechanism/service.hpp#L791).*

<a id="mechanismservice-destructor-mechanismservice"></a>

### `MechanismService::~MechanismService`

```cpp
~MechanismService() override
```

Pinned where it is constructed: the chassis's pacer chain and every slot's claim registration point at it. Non-copy-assignable (the copy constructor's reason). Non-movable (the copy constructor's reason). Non-move-assignable (the copy constructor's reason). A slot still holding its mechanism's claim — a step or a hold-to-run mid-flight — is ended first: CANCELLED ("its service was destroyed"), the declared safe state, the claim released. Otherwise the claim would stay registered to a destroyed slot (any later cancel-all would call into it) and the motor would stay at its last volts — the F1 operations' destructor rule (manipulation/mechanism_op.hpp). A slot holding no claim is untouched (another owner's claim is that owner's; a pneumatic's level persists). This is where bind()'s rule matters: the devices, the clock, the latch and the sink must outlive the service. Nothing escapes: a device that throws is left where it is, its claim is STILL released (no claimant may outlive the slot it points at), and the other slots are still ended.

*function, declared at [`include/shulib/mechanism/service.hpp:801`](../../include/shulib/mechanism/service.hpp#L801).*

<a id="mechanismservice-bind"></a>

### `MechanismService::bind`

```cpp
void bind(std::span<const MechanismSpec> specs, std::span<const Devices> devices, hal::IClock& clock, diag::FaultLatch& faults, hal::ITelemetrySink& telemetry)
```

BOOT: one slot per spec, over its devices — `devices[i]` is `specs[i]`'s, as robot::MechanismParts built them (the SAME devices the guard's span lists, landmine 20). A mechanism that was not built is a slot that answers ABSENT. Nothing is commanded. The specs, the devices' objects, the clock, the latch and the sink must outlive the service. PRECONDITIONS, each checked before anything is stored: bound ONCE; one Devices per spec, at most 8; each spec valid (specRefusal) and its name unique; each a POWER or PNEUMATIC mechanism (position and speed behaviours arrive with MECH-A3); a built mechanism's Devices carries the actuator of its kind, with the declared number of motors or lines, and an ABSENT one none; a piece sensor only as declared, of the declared kind; no angle sensor or home switch.

*function, declared at [`include/shulib/mechanism/service.hpp:827`](../../include/shulib/mechanism/service.hpp#L827).*

<a id="mechanismservice-bind-2"></a>

### `MechanismService::bind (overload 2)`

```cpp
void bind(const MechanismTable& robot, std::span<const Devices> devices, hal::IClock& clock, diag::FaultLatch& faults, hal::ITelemetrySink& telemetry)
```

BOOT from the ROBOT'S LIST (wiring.hpp's mechanisms(...)): the same, and each slot also knows its wiring (a missing sensor's line names its port). `robot` must outlive the service.

*function, declared at [`include/shulib/mechanism/service.hpp:833`](../../include/shulib/mechanism/service.hpp#L833).*

<a id="mechanismservice-bound"></a>

### `MechanismService::bound`

```cpp
[[nodiscard]] bool bound() const noexcept
```

True once bind() has run.

*function, declared at [`include/shulib/mechanism/service.hpp:838`](../../include/shulib/mechanism/service.hpp#L838).*

<a id="mechanismservice-pace"></a>

### `MechanismService::pace`

```cpp
void pace() override
```

THE PACER SEAM (autons): tick every mechanism once, then advance the world. A pace() right after tickFromDriverLoop() does NOT tick again (exactly once per tick). Unbound: forwards only.

*function, declared at [`include/shulib/mechanism/service.hpp:843`](../../include/shulib/mechanism/service.hpp#L843).*

<a id="mechanismservice-tickfromdriverloop"></a>

### `MechanismService::tickFromDriverLoop`

```cpp
void tickFromDriverLoop()
```

THE DRIVER LOOP'S STATED PLACE (after the driver's inputs, before perTick): tick every mechanism once; the next pace() then only advances the world.

*function, declared at [`include/shulib/mechanism/service.hpp:852`](../../include/shulib/mechanism/service.hpp#L852).*

<a id="mechanismservice-enterdrivercontrol"></a>

### `MechanismService::enterDriverControl`

```cpp
void enterDriverControl()
```

DRIVER CONTROL BEGINS: every slot drops its autonomous-era behaviour. A claim this service holds is released with the behaviour; a claim ANOTHER owner left (an operation on a task PROS deleted) is released WITHOUT calling its cancel() — it may have lived on the deleted task's stack — counted, and said (the team lead's ruling of 2026-09-26, T3 Part 2). A running step ends CANCELLED. Motor mechanisms are put in their declared safe state; a pneumatic one is sent its START position on the FIRST entry only (T4's ruling: the claw starts closed — whatever autonomous left it at — and afterwards keeps what it holds). Clears a re-entrancy flag a deleted task left.

*function, declared at [`include/shulib/mechanism/service.hpp:866`](../../include/shulib/mechanism/service.hpp#L866).*

<a id="mechanismservice-enterautonomous"></a>

### `MechanismService::enterAutonomous`

```cpp
int enterAutonomous()
```

AUTONOMOUS BEGINS (the brief's D1): PROS deletes the driver-control task on a mode change (HA-161), so driver control's own exit may never run — a driver's intake left at 12 V then ran through the whole auton. The auton runner (MECH-A4's runAuton overload) calls this before the guard's run: the re-entrancy flag cleared; a claim ANOTHER owner left released WITHOUT its cancel(), counted, said; every motor mechanism's behaviour stopped (its safe state); a pneumatic KEEPS its level; a running step CANCELLED. Returns how many leftover claims this call released.

*function, declared at [`include/shulib/mechanism/service.hpp:893`](../../include/shulib/mechanism/service.hpp#L893).*

<a id="mechanismservice-leavedrivercontrol"></a>

### `MechanismService::leaveDriverControl`

```cpp
void leaveDriverControl()
```

DRIVER CONTROL ENDS (a disable): every MOTOR mechanism's behaviour stops — inert, its declared safe state, this service's claim released; a PNEUMATIC one keeps its level (a claw must not drop what it holds — HA-157); a running step ends CANCELLED. A claim another owner took is left to that owner.

*function, declared at [`include/shulib/mechanism/service.hpp:913`](../../include/shulib/mechanism/service.hpp#L913).*

<a id="mechanismservice-size"></a>

### `MechanismService::size`

```cpp
[[nodiscard]] std::size_t size() const noexcept
```

How many mechanisms are bound.

*function, declared at [`include/shulib/mechanism/service.hpp:928`](../../include/shulib/mechanism/service.hpp#L928).*

<a id="mechanismservice-slot"></a>

### `MechanismService::slot`

```cpp
[[nodiscard]] Slot& slot(std::size_t i)
```

The slot at `i`. PRECONDITION: `i` below size().

*function, declared at [`include/shulib/mechanism/service.hpp:930`](../../include/shulib/mechanism/service.hpp#L930).*

<a id="mechanismservice-slotptr"></a>

### `MechanismService::slotPtr`

```cpp
[[nodiscard]] const Slot* slotPtr(std::size_t i) const noexcept
```

The slot at `i`, or nullptr past the robot's mechanisms.

*function, declared at [`include/shulib/mechanism/service.hpp:935`](../../include/shulib/mechanism/service.hpp#L935).*

<a id="mechanismservice-power"></a>

### `MechanismService::power`

```cpp
[[nodiscard]] PowerSlot& power(std::size_t i)
```

The POWER slot at `i`. PRECONDITION: `i` is a power mechanism.

*function, declared at [`include/shulib/mechanism/service.hpp:939`](../../include/shulib/mechanism/service.hpp#L939).*

<a id="mechanismservice-pneumatic"></a>

### `MechanismService::pneumatic`

```cpp
[[nodiscard]] PneumaticSlot& pneumatic(std::size_t i)
```

The PNEUMATIC slot at `i`. PRECONDITION: `i` is a pneumatic mechanism.

*function, declared at [`include/shulib/mechanism/service.hpp:945`](../../include/shulib/mechanism/service.hpp#L945).*

<a id="mechanismservice-find"></a>

### `MechanismService::find`

```cpp
[[nodiscard]] PowerSlot* find(const mechanism::Power& d) noexcept
```

A DECLARATION's slot (Q18 (a): the declaration is the handle), found by its KIND (the overload — it cannot mismatch) and its NAME (unique on a robot: the build refuses two). nullptr when this robot's list does not have it — a step on it then ends ABSENT, said. By name, never by address: a declaration written without `inline` has one copy per translation unit (the brief's P12), and an address lookup would end a step in another file ABSENT, silently. (So a robot two auton compiled into another robot's build commands a same-named mechanism of the same kind there — autons are per robot, landmine 6.)

*function, declared at [`include/shulib/mechanism/service.hpp:957`](../../include/shulib/mechanism/service.hpp#L957).*

<a id="mechanismservice-find-2"></a>

### `MechanismService::find (overload 2)`

```cpp
[[nodiscard]] PneumaticSlot* find(const mechanism::Pneumatic& d) noexcept
```

A pneumatic declaration's slot (find(const Power&)'s rule).

*function, declared at [`include/shulib/mechanism/service.hpp:959`](../../include/shulib/mechanism/service.hpp#L959).*

<a id="mechanismservice-find-3"></a>

### `MechanismService::find (overload 3)`

```cpp
[[nodiscard]] PneumaticSlot* find(const PneumaticGoal& g) noexcept
```

A pneumatic GOAL's slot (`claw["closed"]`): also checks the POSITION — returned only when this robot's mechanism of that name has that position's name at that index.

*function, declared at [`include/shulib/mechanism/service.hpp:964`](../../include/shulib/mechanism/service.hpp#L964).*

<a id="mechanismservice-slotfor"></a>

### `MechanismService::slotFor`

```cpp
[[nodiscard]] const Slot* slotFor(const char* name) const noexcept
```

The slot of ANY kind with this NAME (another mechanism's piece sensor, the driver's inputs), or nullptr.

*function, declared at [`include/shulib/mechanism/service.hpp:978`](../../include/shulib/mechanism/service.hpp#L978).*

<a id="mechanismservice-mechanisms"></a>

### `MechanismService::mechanisms`

```cpp
[[nodiscard]] std::span<hal::IMechanism* const> mechanisms() const noexcept
```

Every BUILT mechanism, in the list's order — the RunGuard's span (the guard reaches each one's slot through the claim it registers).

*function, declared at [`include/shulib/mechanism/service.hpp:992`](../../include/shulib/mechanism/service.hpp#L992).*

<a id="mechanismservice-ticks"></a>

### `MechanismService::ticks`

```cpp
[[nodiscard]] long ticks() const noexcept
```

How many ticks have run (an unbound pace() and a re-entered one are not ticks) — a program's boot proves its pacer chain with it (MECH-A5).

*function, declared at [`include/shulib/mechanism/service.hpp:997`](../../include/shulib/mechanism/service.hpp#L997).*

<a id="mechanismservice-leftoverclaimsreleased"></a>

### `MechanismService::leftoverClaimsReleased`

```cpp
[[nodiscard]] int leftoverClaimsReleased() const noexcept
```

How many leftover claims of another owner the mode changes have released, in total.

*function, declared at [`include/shulib/mechanism/service.hpp:999`](../../include/shulib/mechanism/service.hpp#L999).*

<a id="mechanismservice-clock"></a>

### `MechanismService::clock`

```cpp
[[nodiscard]] hal::IClock& clock() const
```

The clock it was bound with. PRECONDITION: bound.

*function, declared at [`include/shulib/mechanism/service.hpp:1001`](../../include/shulib/mechanism/service.hpp#L1001).*

<a id="mechanismservice-faults"></a>

### `MechanismService::faults`

```cpp
[[nodiscard]] diag::FaultLatch& faults() const
```

The fault latch it was bound with. PRECONDITION: bound.

*function, declared at [`include/shulib/mechanism/service.hpp:1006`](../../include/shulib/mechanism/service.hpp#L1006).*

<a id="mechanismservice-telemetry"></a>

### `MechanismService::telemetry`

```cpp
[[nodiscard]] hal::ITelemetrySink& telemetry() const
```

The telemetry sink it was bound with. PRECONDITION: bound.

*function, declared at [`include/shulib/mechanism/service.hpp:1011`](../../include/shulib/mechanism/service.hpp#L1011).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 83 lines, click to expand</summary>

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

 ── What this run builds, and what it does not ───────────────────────────────────────
 MECH-A2 builds the slot, the two seams, the claimant, the three mode changes, the lookups and the
 POWER and PNEUMATIC behaviours. The POSITION (Arm, Lift) and SPEED (Flywheel) behaviours are
 MECH-A3's: until it lands, bind() REFUSES a list that has one (a precondition naming it — never a
 slot that silently does nothing). The home switch's half of the read-fault rule arrives with
 MECH-A3's homing, its only reader.

 ── Re-entrancy (requirements :142-152) ──────────────────────────────────────────────
 A tick never calls a blocking verb, a wait or the scheduler's cancel(), and is never re-entered
 (a precondition). A predicate may READ a slot (`seen()`, `running()`), never tick it. A tick that
 never finished — its task deleted mid-tick by a PROS mode change (HA-161), or a throw out of a
 device — leaves the re-entrancy flag set: the next mode change (enterDriverControl /
 enterAutonomous) clears it.

 ── The honest limit ─────────────────────────────────────────────────────────────────
 The service runs ONLY when something ticks it: inside a shulib motion, wait or pause, and every
 driver-loop tick. A student's own busy loop (`pros::delay` in an auton, a then() action that
 blocks) STARVES it, and a starved service leaves each mechanism at its LAST command — not held,
 not limited, not safe (the brief's D11, measured: 11.74 V held for 1.5 s). The run guard says the
 same of itself: nothing preempts pure user code.

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
