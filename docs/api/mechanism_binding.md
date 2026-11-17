<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/mechanism_binding.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `mechanism_binding.hpp`

MechanismBinding — two controller buttons driving one motor mechanism, hold-to-run.

This header declares **3** types (35 members), **4** free functions, and **1** constant.

Extracted from [`include/shulib/teleop/mechanism_binding.hpp`](../../include/shulib/teleop/mechanism_binding.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kControllerButtonCount`](#kcontrollerbuttoncount) — *constant*
- [`controllerButtonWord`](#controllerbuttonword) — *free function*
- [`isControllerButton`](#iscontrollerbutton) — *free function*
- [`class MechanismBinding`](#class-mechanismbinding)
  - [`MechanismBinding`](#mechanismbinding-mechanismbinding)
  - [`MechanismBinding (overload 2)`](#mechanismbinding-mechanismbinding-2)
  - [`MechanismBinding (overload 3)`](#mechanismbinding-mechanismbinding-3)
  - [`operator=`](#mechanismbinding-operator-eq)
  - [`operator= (overload 2)`](#mechanismbinding-operator-eq-2)
  - [`~MechanismBinding`](#mechanismbinding-destructor-mechanismbinding)
  - [`tick`](#mechanismbinding-tick)
  - [`makeSafe`](#mechanismbinding-makesafe)
  - [`reset`](#mechanismbinding-reset)
  - [`releaseLeftoverClaim`](#mechanismbinding-releaseleftoverclaim)
  - [`releasedLeftoverClaim`](#mechanismbinding-releasedleftoverclaim)
  - [`state`](#mechanismbinding-state)
  - [`forwardButton`](#mechanismbinding-forwardbutton)
  - [`reverseButton`](#mechanismbinding-reversebutton)
  - [`volts`](#mechanismbinding-volts)
  - [`mechanism`](#mechanismbinding-mechanism)
  - [`enum class MechanismBinding::State`](#enum-class-mechanismbinding-state)
    - [`Idle`](#mechanismbinding-state-idle)
    - [`Forward`](#mechanismbinding-state-forward)
    - [`Reverse`](#mechanismbinding-state-reverse)
    - [`Stopped`](#mechanismbinding-state-stopped)
    - [`Inert`](#mechanismbinding-state-inert)
- [`class ToggleBinding`](#class-togglebinding)
  - [`ToggleBinding`](#togglebinding-togglebinding)
  - [`ToggleBinding (overload 2)`](#togglebinding-togglebinding-2)
  - [`ToggleBinding (overload 3)`](#togglebinding-togglebinding-3)
  - [`operator=`](#togglebinding-operator-eq)
  - [`operator= (overload 2)`](#togglebinding-operator-eq-2)
  - [`~ToggleBinding`](#togglebinding-destructor-togglebinding)
  - [`enter`](#togglebinding-enter)
  - [`tick`](#togglebinding-tick)
  - [`releaseLeftoverClaim`](#togglebinding-releaseleftoverclaim)
  - [`releasedLeftoverClaim`](#togglebinding-releasedleftoverclaim)
  - [`entered`](#togglebinding-entered)
  - [`button`](#togglebinding-button)
  - [`initialState`](#togglebinding-initialstate)
  - [`mechanism`](#togglebinding-mechanism)
- [`validBindings`](#validbindings) — *free function*
- [`validBindings (overload 2)`](#validbindings-2) — *free function*

<a id="kcontrollerbuttoncount"></a>

## `kControllerButtonCount`

```cpp
inline constexpr int kControllerButtonCount = 12
```

The number of `hal::ControllerButton` values — the twelve driver-usable buttons.

*constant, declared at [`include/shulib/teleop/mechanism_binding.hpp:155`](../../include/shulib/teleop/mechanism_binding.hpp#L155).*

<a id="controllerbuttonword"></a>

## `controllerButtonWord`

```cpp
[[nodiscard]] inline const char* controllerButtonWord(hal::ControllerButton b) noexcept
```

A controller button as its label on the V5 controller ("L1", "R2", "UP", "A", ...), for refusal messages and panels. "?" for a byte outside the enum.

*free function, declared at [`include/shulib/teleop/mechanism_binding.hpp:159`](../../include/shulib/teleop/mechanism_binding.hpp#L159).*

<a id="iscontrollerbutton"></a>

## `isControllerButton`

```cpp
[[nodiscard]] constexpr bool isControllerButton(hal::ControllerButton b) noexcept
```

True when `b` is one of the twelve `hal::ControllerButton` values — a cast byte outside the enum is not a button, and handing it to `IController::pressed()` would index past its table.

*free function, declared at [`include/shulib/teleop/mechanism_binding.hpp:179`](../../include/shulib/teleop/mechanism_binding.hpp#L179).*

<a id="class-mechanismbinding"></a>

## `class MechanismBinding`

```cpp
class MechanismBinding
```

Hold-to-run: a forward button and a reverse button driving one `hal::MotorMechanism` at a stated voltage, with release as the stop. The file header states every rule. SUPERSEDED by the mechanism service's `whileHeld` (file header) — for a program with no mechanism service.  Non-copyable and non-movable: the loop and a panel hold it by pointer, and a copy would be a second state machine driving the same mechanism. Construct it where it lives — for a PROS program, in static storage, because the opcontrol task can be deleted mid-tick.

*class, declared at [`include/shulib/teleop/mechanism_binding.hpp:191`](../../include/shulib/teleop/mechanism_binding.hpp#L191).*

<a id="mechanismbinding-mechanismbinding"></a>

### `MechanismBinding::MechanismBinding`

```cpp
MechanismBinding(hal::MotorMechanism& mechanism, hal::ControllerButton forward, hal::ControllerButton reverse, units::Voltage volts)
```

Bind `forward` and `reverse` to `mechanism` at `volts`. `mechanism` is BORROWED and must outlive the binding. PRECONDITIONS (checked here, at construction, never in the loop): both buttons are real buttons, `forward != reverse`, and `volts` is finite and in (0, 12] V — stated, because there is no voltage that is right for every mechanism. Which buttons, and whether they clash with the program's other buttons, is checked across a whole set by `validBindings()`.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:216`](../../include/shulib/teleop/mechanism_binding.hpp#L216).*

<a id="mechanismbinding-mechanismbinding-2"></a>

### `MechanismBinding::MechanismBinding (overload 2)`

```cpp
MechanismBinding(const MechanismBinding&) = delete
```

Non-copyable: two state machines on one mechanism would fight.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:230`](../../include/shulib/teleop/mechanism_binding.hpp#L230).*

<a id="mechanismbinding-mechanismbinding-3"></a>

### `MechanismBinding::MechanismBinding (overload 3)`

```cpp
MechanismBinding(MechanismBinding&&) = delete
```

Non-copyable: two state machines on one mechanism would fight. Non-movable: the loop and panels hold it by address.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:232`](../../include/shulib/teleop/mechanism_binding.hpp#L232).*

<a id="mechanismbinding-operator-eq"></a>

### `MechanismBinding::operator=`

```cpp
MechanismBinding& operator=(const MechanismBinding&) = delete
```

Non-copyable: two state machines on one mechanism would fight. Non-movable: the loop and panels hold it by address. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:234`](../../include/shulib/teleop/mechanism_binding.hpp#L234).*

<a id="mechanismbinding-operator-eq-2"></a>

### `MechanismBinding::operator= (overload 2)`

```cpp
MechanismBinding& operator=(MechanismBinding&&) = delete
```

Non-copyable: two state machines on one mechanism would fight. Non-movable: the loop and panels hold it by address. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:236`](../../include/shulib/teleop/mechanism_binding.hpp#L236).*

<a id="mechanismbinding-destructor-mechanismbinding"></a>

### `MechanismBinding::~MechanismBinding`

```cpp
~MechanismBinding() = default
```

Non-copyable: two state machines on one mechanism would fight. Non-movable: the loop and panels hold it by address. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Commands nothing: a binding going away is not a stop (the loop's disable path is).

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:238`](../../include/shulib/teleop/mechanism_binding.hpp#L238).*

<a id="mechanismbinding-tick"></a>

### `MechanismBinding::tick`

```cpp
void tick(bool connected, bool forwardDown, bool reverseDown)
```

One driver-control tick. `connected` is the controller's flag for THIS tick; `forwardDown` / `reverseDown` are the two buttons' levels, which the caller reads only while connected — and which are IGNORED here while `connected` is false, so a caller that read a lying disconnected controller still cannot drive the mechanism. Applies the rules in the file header, in this order: an inert binding waits for an observed release; a mechanism someone else claimed is never commanded; one button drives; otherwise the safe state is applied once, on the transition.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:247`](../../include/shulib/teleop/mechanism_binding.hpp#L247).*

<a id="mechanismbinding-makesafe"></a>

### `MechanismBinding::makeSafe`

```cpp
void makeSafe()
```

An OUTSIDE stop: apply the mechanism's declared safe state now (unless someone else has claimed it — then its claimant owns the stop), and go INERT unless the last tick saw the controller connected with both buttons up. Synchronous; idempotent; commands nothing more until the binding re-arms.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:277`](../../include/shulib/teleop/mechanism_binding.hpp#L277).*

<a id="mechanismbinding-reset"></a>

### `MechanismBinding::reset`

```cpp
void reset() noexcept
```

Forget the last run: state Idle, and nothing held as far as this binding knows — so the next makeSafe() leaves it armed and a button held on the next tick drives at once. Commands nothing. The loop calls reset() then makeSafe() before its first tick.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:287`](../../include/shulib/teleop/mechanism_binding.hpp#L287).*

<a id="mechanismbinding-releaseleftoverclaim"></a>

### `MechanismBinding::releaseLeftoverClaim`

```cpp
bool releaseLeftoverClaim() noexcept
```

THE LEFTOVER-CLAIM RULING (the team lead, 2026-09-26; file header): release a claim still on this binding's mechanism — for `runDriverControl` to call at loop ENTRY, and nowhere else (a claim met by tick() or makeSafe() is still deferred to). The claimant's cancel() is NOT called: it may have lived on a deleted task's stack. Returns true when a claim was there, and remembers that answer until the next call, as `releasedLeftoverClaim()`. Commands nothing, and leaves the binding's state alone.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:298`](../../include/shulib/teleop/mechanism_binding.hpp#L298).*

<a id="mechanismbinding-releasedleftoverclaim"></a>

### `MechanismBinding::releasedLeftoverClaim`

```cpp
[[nodiscard]] bool releasedLeftoverClaim() const noexcept
```

True when the last `releaseLeftoverClaim()` — the last loop entry — found a claim on the mechanism and released it: what a panel shows so the driver knows (the loop also logs it).

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:308`](../../include/shulib/teleop/mechanism_binding.hpp#L308).*

<a id="mechanismbinding-state"></a>

### `MechanismBinding::state`

```cpp
[[nodiscard]] State state() const noexcept
```

Where the binding is (see State).

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:311`](../../include/shulib/teleop/mechanism_binding.hpp#L311).*

<a id="mechanismbinding-forwardbutton"></a>

### `MechanismBinding::forwardButton`

```cpp
[[nodiscard]] hal::ControllerButton forwardButton() const noexcept
```

The button that drives +V.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:313`](../../include/shulib/teleop/mechanism_binding.hpp#L313).*

<a id="mechanismbinding-reversebutton"></a>

### `MechanismBinding::reverseButton`

```cpp
[[nodiscard]] hal::ControllerButton reverseButton() const noexcept
```

The button that drives -V.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:315`](../../include/shulib/teleop/mechanism_binding.hpp#L315).*

<a id="mechanismbinding-volts"></a>

### `MechanismBinding::volts`

```cpp
[[nodiscard]] units::Voltage volts() const noexcept
```

The stated drive voltage (positive; reverse commands its negation).

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:317`](../../include/shulib/teleop/mechanism_binding.hpp#L317).*

<a id="mechanismbinding-mechanism"></a>

### `MechanismBinding::mechanism`

```cpp
[[nodiscard]] hal::MotorMechanism& mechanism() const noexcept
```

The mechanism this binding drives.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:319`](../../include/shulib/teleop/mechanism_binding.hpp#L319).*

<a id="enum-class-mechanismbinding-state"></a>

## `enum class MechanismBinding::State`

```cpp
enum class State : std::uint8_t
```

Where the binding is. A panel reads this; nothing branches on it outside this class.

*enum class, declared at [`include/shulib/teleop/mechanism_binding.hpp:194`](../../include/shulib/teleop/mechanism_binding.hpp#L194).*

<a id="mechanismbinding-state-idle"></a>

### `MechanismBinding::State::Idle`

```cpp
Idle
```

Constructed or reset(): nothing commanded yet. The first tick that does not drive applies the safe state (so "not driving" always means "made safe since the last drive").

*enumerator, declared at [`include/shulib/teleop/mechanism_binding.hpp:197`](../../include/shulib/teleop/mechanism_binding.hpp#L197).*

<a id="mechanismbinding-state-forward"></a>

### `MechanismBinding::State::Forward`

```cpp
Forward
```

Forward held alone: +V was commanded this tick.

*enumerator, declared at [`include/shulib/teleop/mechanism_binding.hpp:199`](../../include/shulib/teleop/mechanism_binding.hpp#L199).*

<a id="mechanismbinding-state-reverse"></a>

### `MechanismBinding::State::Reverse`

```cpp
Reverse
```

Reverse held alone: -V was commanded this tick.

*enumerator, declared at [`include/shulib/teleop/mechanism_binding.hpp:201`](../../include/shulib/teleop/mechanism_binding.hpp#L201).*

<a id="mechanismbinding-state-stopped"></a>

### `MechanismBinding::State::Stopped`

```cpp
Stopped
```

Not driving; the safe state was applied on the transition (or by makeSafe(), or the mechanism's claimant owns it). Nothing is commanded until a button drives.

*enumerator, declared at [`include/shulib/teleop/mechanism_binding.hpp:204`](../../include/shulib/teleop/mechanism_binding.hpp#L204).*

<a id="mechanismbinding-state-inert"></a>

### `MechanismBinding::State::Inert`

```cpp
Inert
```

An outside stop (or someone else's claim) landed while a button was held or the controller was gone: nothing is commanded until a connected tick shows both buttons up.

*enumerator, declared at [`include/shulib/teleop/mechanism_binding.hpp:207`](../../include/shulib/teleop/mechanism_binding.hpp#L207).*

<a id="class-togglebinding"></a>

## `class ToggleBinding`

```cpp
class ToggleBinding
```

SUPERSEDED by the mechanism service's `toggle` (file header) — for a program with no mechanism service. Press-to-toggle: one button flipping one `hal::PneumaticMechanism` between its two line levels, once per SEEN press — a pneumatic claw on A. The file header's TOGGLE section states every rule; the short form: a press flips the commanded level exactly once, a level the binding could not see counts as held, the initial level is commanded once in the binding's life, and a disconnect, a reconnect or a disable commands nothing.  Non-copyable and non-movable, for the MechanismBinding reasons: the loop and a panel hold it by address, and a copy would be a second state machine flipping the same valve. For a PROS program, construct it in static storage.

*class, declared at [`include/shulib/teleop/mechanism_binding.hpp:341`](../../include/shulib/teleop/mechanism_binding.hpp#L341).*

<a id="togglebinding-togglebinding"></a>

### `ToggleBinding::ToggleBinding`

```cpp
ToggleBinding(hal::PneumaticMechanism& mechanism, hal::ControllerButton button, bool initialState)
```

Bind `button` to `mechanism`, whose initial level is `initialState` — STATED, with no default, because which level is "closed" depends on how the valve is plumbed. The binding commands it once, at its first loop entry (`enter()`), never at construction. `mechanism` is BORROWED and must outlive the binding. PRECONDITION (checked here, at construction): `button` is one of the twelve buttons. Whether it clashes with the program's other buttons is checked across the whole set by `validBindings()`.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:349`](../../include/shulib/teleop/mechanism_binding.hpp#L349).*

<a id="togglebinding-togglebinding-2"></a>

### `ToggleBinding::ToggleBinding (overload 2)`

```cpp
ToggleBinding(const ToggleBinding&) = delete
```

Non-copyable: two state machines on one valve would each flip it.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:358`](../../include/shulib/teleop/mechanism_binding.hpp#L358).*

<a id="togglebinding-togglebinding-3"></a>

### `ToggleBinding::ToggleBinding (overload 3)`

```cpp
ToggleBinding(ToggleBinding&&) = delete
```

Non-copyable: two state machines on one valve would each flip it. Non-movable: the loop and panels hold it by address.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:360`](../../include/shulib/teleop/mechanism_binding.hpp#L360).*

<a id="togglebinding-operator-eq"></a>

### `ToggleBinding::operator=`

```cpp
ToggleBinding& operator=(const ToggleBinding&) = delete
```

Non-copyable: two state machines on one valve would each flip it. Non-movable: the loop and panels hold it by address. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:362`](../../include/shulib/teleop/mechanism_binding.hpp#L362).*

<a id="togglebinding-operator-eq-2"></a>

### `ToggleBinding::operator= (overload 2)`

```cpp
ToggleBinding& operator=(ToggleBinding&&) = delete
```

Non-copyable: two state machines on one valve would each flip it. Non-movable: the loop and panels hold it by address. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:364`](../../include/shulib/teleop/mechanism_binding.hpp#L364).*

<a id="togglebinding-destructor-togglebinding"></a>

### `ToggleBinding::~ToggleBinding`

```cpp
~ToggleBinding() = default
```

Non-copyable: two state machines on one valve would each flip it. Non-movable: the loop and panels hold it by address. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Commands nothing: a binding going away is not a flip.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:366`](../../include/shulib/teleop/mechanism_binding.hpp#L366).*

<a id="togglebinding-enter"></a>

### `ToggleBinding::enter`

```cpp
void enter()
```

Loop entry — `runDriverControl` calls it before its first tick. The FIRST call in the binding's life commands `initialState()`, once, unless someone else has claimed the mechanism (then it is never sent — header). `runDriverControl` releases a leftover claim just before this call, so there the first entry always sends it. Every call forgets the button's level, so a button already down counts as held, not as a press.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:373`](../../include/shulib/teleop/mechanism_binding.hpp#L373).*

<a id="togglebinding-tick"></a>

### `ToggleBinding::tick`

```cpp
void tick(bool connected, bool down)
```

One driver-control tick. `connected` is the controller's flag for THIS tick; `down` is the button's level, which the caller reads only while connected — and which is IGNORED while `connected` is false (the level is then unknown, and an unknown level counts as held). A level handed over on a CONNECTED tick is taken as live, the first one after a reconnect included (the banner's A PRESS MUST BE SEEN rule says what that rests on). On a press — a connected tick with the button down after a connected tick with it up — the mechanism's commanded level is flipped, unless someone else has claimed it: then the press is dropped. Nothing else here ever commands the mechanism.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:391`](../../include/shulib/teleop/mechanism_binding.hpp#L391).*

<a id="togglebinding-releaseleftoverclaim"></a>

### `ToggleBinding::releaseLeftoverClaim`

```cpp
bool releaseLeftoverClaim() noexcept
```

THE LEFTOVER-CLAIM RULING, for a toggle exactly as for a hold binding (`MechanismBinding::releaseLeftoverClaim()`; file header): release a claim still on this toggle's mechanism — for `runDriverControl` at loop ENTRY only, BEFORE `enter()`, so a first entry then sends the initial level. The claimant's cancel() is NOT called. Returns true when a claim was there, remembered as `releasedLeftoverClaim()` until the next call. Commands nothing.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:410`](../../include/shulib/teleop/mechanism_binding.hpp#L410).*

<a id="togglebinding-releasedleftoverclaim"></a>

### `ToggleBinding::releasedLeftoverClaim`

```cpp
[[nodiscard]] bool releasedLeftoverClaim() const noexcept
```

True when the last `releaseLeftoverClaim()` found a claim and released it (a panel's word).

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:419`](../../include/shulib/teleop/mechanism_binding.hpp#L419).*

<a id="togglebinding-entered"></a>

### `ToggleBinding::entered`

```cpp
[[nodiscard]] bool entered() const noexcept
```

Whether the first loop entry has happened (and with it the one chance to send the initial level). A panel reads it; nothing branches on it outside this class.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:423`](../../include/shulib/teleop/mechanism_binding.hpp#L423).*

<a id="togglebinding-button"></a>

### `ToggleBinding::button`

```cpp
[[nodiscard]] hal::ControllerButton button() const noexcept
```

The button that flips the mechanism.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:425`](../../include/shulib/teleop/mechanism_binding.hpp#L425).*

<a id="togglebinding-initialstate"></a>

### `ToggleBinding::initialState`

```cpp
[[nodiscard]] bool initialState() const noexcept
```

The stated initial level, commanded once at the first loop entry.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:427`](../../include/shulib/teleop/mechanism_binding.hpp#L427).*

<a id="togglebinding-mechanism"></a>

### `ToggleBinding::mechanism`

```cpp
[[nodiscard]] hal::PneumaticMechanism& mechanism() const noexcept
```

The mechanism this binding flips. Its `commanded()` is the level the line was last told.

*function, declared at [`include/shulib/teleop/mechanism_binding.hpp:429`](../../include/shulib/teleop/mechanism_binding.hpp#L429).*

<a id="validbindings"></a>

## `validBindings`

```cpp
[[nodiscard]] inline bool validBindings(std::span<MechanismBinding* const> bindings, std::span<ToggleBinding* const> toggles, hal::ControllerButton rezeroButton, char* why = nullptr, std::size_t cap = 0) noexcept
```

Can this SET of hold bindings and toggle bindings run together under a driver-control loop whose re-zero button is `rezeroButton`? PURE, TOTAL and `noexcept`: `why` (optional; nullptr and 0 to skip) receives the FIRST problem, naming the button or the mechanism. Refused, in this order:  1. a null entry (hold bindings first, then toggles); 2. a binding button equal to the loop's re-zero button (`DriverLoopConfig::rezeroButton` DEFAULTS to L1, so a program that bound L1 and kept the default is caught here) — a hold binding's forward or reverse button, or a toggle's button; 3. any button used twice across the WHOLE set — within one binding, between two hold bindings, between a hold binding and a toggle, or between two toggles; 4. one mechanism bound twice (bindings take no claim, so two of them would fight over it) — two hold bindings on one motor mechanism, or two toggles on one pneumatic mechanism. A hold binding and a toggle can never share one: they take different mechanism types.  A program calls this where a refusal can still be reported — before driver control starts — and `runDriverControl` checks it again, as a precondition, before its first tick.

*free function, declared at [`include/shulib/teleop/mechanism_binding.hpp:465`](../../include/shulib/teleop/mechanism_binding.hpp#L465).*

<a id="validbindings-2"></a>

## `validBindings (overload 2)`

```cpp
[[nodiscard]] inline bool validBindings(std::span<MechanismBinding* const> bindings, hal::ControllerButton rezeroButton, char* why = nullptr, std::size_t cap = 0) noexcept
```

`validBindings` for a set of hold bindings and NO toggles — the chunk-T3 form, forwarding to the overload above with an empty toggle span, so one function owns every rule and every message.

*free function, declared at [`include/shulib/teleop/mechanism_binding.hpp:575`](../../include/shulib/teleop/mechanism_binding.hpp#L575).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 138 lines, click to expand</summary>

```text

 MechanismBinding — two controller buttons driving one motor mechanism, hold-to-run
 (chunk T3, 2026-09-26). ToggleBinding — one button flipping one pneumatic mechanism, once per
 press (T3's addendum B2, 2026-09-26; its own section below).

 ── SUPERSEDED BY THE MECHANISM SERVICE (chunk MECH-A4, 2026-10-22; the MECH brief's N4 (a)) ──
 A robot whose mechanisms are DECLARED (mechanism/kinds.hpp) and run by the mechanism service drives
 them from its driver file's controls — mechanism::whileHeld / toggle / preset (mechanism/
 driver_inputs.hpp) — through teleop::runDriverControl's service overload: the same hold-to-run and
 toggle rules plus presets, homing and "the most recent input wins", and the same service that runs
 them in autons. These two bindings, and the runDriverControl overloads that take them, are KEPT
 for a program with NO mechanism service, unchanged; one program uses one path per mechanism (the
 loop refuses the two together). Removing them later is a small chunk of its own, on the team
 lead's word.

 ── What it is ──────────────────────────────────────────────────────────────────────
 The driver-control half of a mechanism: "while R1 is held the claw pivot drives forward at
 full power, while R2 is held it drives back, and the moment neither is held it goes to its
 declared safe state". A pure state machine over two button LEVELS (plus the controller's
 connected flag) and ONE `hal::MotorMechanism&`, ticked once per driver-control tick by
 `teleop::runDriverControl` (teleop_loop.hpp) after the drive has been commanded.

 It is NOT a `manipulation::IMechanismOp`, and that is the F1 contract, not a shortcut: an
 operation is BOUNDED (it ends with a verdict), and "while the button is held" is open-ended.
 So the binding commands the mechanism directly and owns its own stop, exactly as that header
 tells open-ended behaviour to.

 ── THE RULES — each one pinned by a test (test/mechanism_binding_test.cpp) ─────────
   * FORWARD ONLY held  ⇒ setVoltage(+V) EVERY tick. REVERSE ONLY ⇒ setVoltage(-V) every tick.
     V is STATED at construction — no default — finite and in (0, 12] volts. Re-commanding
     every tick is deliberate: the voltage is the driver's intent, re-asserted, and a motor
     that lost a command (a brief brown-out, a cable blip) gets it back on the next tick.
   * NEITHER held, BOTH held, or the controller DISCONNECTED ⇒ applySafeState() EXACTLY ONCE, on
     the transition, then NO command at all until a button drives again. Both held is a HOLD —
     neither wins (the team lead's ruling R2, 2026-09-26); releasing one lets the other drive
     (level semantics). Once, not every tick: re-applying a hold every tick would starve any
     later operation of the device, and on V5 a repeated hold may re-capture its hold point
     where the load has sagged to — a slow creep nobody commanded.
   * RELEASING THE BUTTON IS THE STOP. For a mechanism the driver uses WHILE driving, the
     chassis-macro rule "a stick input interrupts it" would make the mechanism unusable, so it
     does not apply here (ruling R1, 2026-09-26): the stop is the release, and it lands in the
     same tick the release is read.
   * AN OUTSIDE STOP — makeSafe(), which the loop calls on a field disable, and which any other
     code that HOLDS the binding may call (a program's own panic stop) — applies the safe state
     NOW and, if a button was held (or the controller was not connected, so nobody can say),
     leaves the binding INERT: it commands nothing until a CONNECTED tick shows BOTH its buttons
     up, then re-arms. An outside stop is never undone by the next tick of a finger that never
     left the button. A disconnect is a release for the level rules above — a still-held button
     drives again after a reconnect — but it does NOT clear an outside stop, because a
     disconnected controller cannot show that the button came up.
     Only a stop that goes THROUGH the binding sticks. Code that holds just the mechanism — an
     end-of-run guard walking `IMechanism*` (sequence/run_guard.hpp) — can repaint the safe
     state, but a binding takes no claim, so there is nothing for the guard to cancel, and a
     still-held button drives the mechanism again on the next tick (the half-safe failure
     hal/mechanism.hpp describes). A program that runs driver control under such a guard makes
     the loop's `disabled()` true once the guard expires, so the loop's own disable path stops
     every binding.
   * reset() forgets all of that (the loop calls reset() + makeSafe() at entry), so a button
     already held on the loop's first tick drives at once.
   * NO CLAIM, and a mechanism someone else has claimed() is NEVER commanded — makeSafe()
     included. A claim taken by a binding would outlive a PROS task deletion (a field disable
     deletes the opcontrol task and runs no destructor), and would then break the next
     operation's start() and RunGuard's cancel-all. The claim's owner owns the mechanism; a
     press seen while it does leaves the binding inert, so it never carries over after.
   * A CLAIM STILL THERE WHEN DRIVER CONTROL STARTS IS RELEASED (the team lead's ruling,
     2026-09-26; T3 Part 2). The same task deletion can strand SOMEONE ELSE's claim: an
     operation autonomous was running when the field ended it keeps its mechanism claimed into
     driver control, and nothing can tell that dead claim from a live one — so, deferring to it,
     the binding would do nothing for the whole driver period. A claim left by a deleted task
     must never lock the driver out. So `runDriverControl`, at loop ENTRY, calls every binding's
     releaseLeftoverClaim() — after its set check, before the entry safe state — which releases
     the claim (`releaseClaim()`; never the claimant's cancel(), which may have lived on the
     deleted task's stack) and remembers that it did (releasedLeftoverClaim(), for a panel); the
     loop says so, one Warn line per released claim (tag TEL). The mechanism is then made safe
     at entry like any other and its buttons work from the first tick. What the ruling costs,
     stated: an operation a program starts on a bound mechanism BEFORE entering the loop (to tick
     it from perTick) loses its claim at entry — start such an operation after entry, from
     perTick; and the dead operation object still believes it holds the claim, so a later
     cancel() of THAT object would safe the mechanism and release whatever claim is on it then
     (the exposure an end-of-run guard's anonymous force-release already accepts). A claim taken
     DURING the loop is untouched by the ruling: the binding defers to it, as above.

 ── TOGGLE BINDINGS — one button, one pneumatic mechanism (addendum B2) ──────────────
 "Press A and the claw opens; press A again and it closes." A ToggleBinding is one button over
 one `hal::PneumaticMechanism&`. Its rules — each one pinned by a test (test/toggle_binding_test.cpp):
   * EACH PRESS FLIPS THE COMMANDED LEVEL EXACTLY ONCE. A press is a rising edge of the
     binding's OWN hal::ButtonEdge (a per-consumer edge, HA-104); holding the button does nothing
     more, and a press, a release and a press are two flips. The flip is
     set(!mechanism.commanded()) — the opposite of what the line was LAST COMMANDED, whoever
     commanded it — so an end-of-run guard or an operation that set the claw in between never
     leaves the next press doing the invisible thing.
   * A PRESS MUST BE SEEN. A level the binding could not see counts as HELD: at loop entry and on
     every disconnected tick the edge is re-seeded "down", so a button already down when the loop
     starts, or when the controller comes back, is not a press — it has to be seen up, then down,
     both on connected ticks. That is how "a press made while the controller was gone is not
     replayed" is kept — PROVIDED the controller's connected flag and its button levels change
     TOGETHER, which nobody has measured (A4 register HA-147, invented; HA-103 says only that a
     disconnected controller reads 0). The connected flag is the only thing that tells an unknown
     level from a real one, so the FIRST connected tick's level is taken as live: if that tick
     read a stale "up" while the driver still held the button — or if a level could bounce
     across two 10 ms samples — the binding would see a release, and the next live "down" would
     be a press nobody made (test/toggle_binding_test.cpp T4 pins exactly that boundary). The
     opposite of the hold binding's "a button held on the first tick drives at once", on
     purpose: a hold ends when released; a flip is permanent, and a phantom flip opens the claw
     on what it holds. The cost: a press landing on the very tick of a reconnect (or of loop
     entry) is not counted — the driver presses again.
   * THE INITIAL LEVEL IS STATED (no default) and commanded ONCE IN THE BINDING'S LIFE, at its
     first loop entry (enter()). A later entry — PROS calls opcontrol() again after a field
     disable — commands nothing: re-commanding the initial level there would move the claw on the
     re-enable. (So if autonomous commanded the claw, the FIRST driver-control entry still sets
     the initial level once; a program for which that is wrong states the initial level to match.)
   * A DISCONNECT, A RECONNECT AND A DISABLE COMMAND NOTHING. The claw keeps its level — it must
     not drop what it holds — so a toggle has no makeSafe() at all, and runDriverControl's disable
     path leaves every toggle untouched while it stops every hold binding. Whether the BRAIN keeps
     a 3-wire output's level while it is disabled is not known (A4 register HA-157); nothing here
     can make it, and nothing here pretends to.
   * NO CLAIM, and a mechanism someone else has claimed() is never commanded — the initial level
     included: enter() over a claim spends the first entry WITHOUT sending it (a start level
     landing later, whenever a claim happens to end, would be a movement nobody pressed for).
     The edge keeps following the button during a claim, so a press seen then is DROPPED, never
     carried over. But runDriverControl releases a claim still there at loop ENTRY before it
     calls enter(), exactly as for a hold binding (the ruling above) — so inside the loop the
     first entry always finds the mechanism unclaimed and SENDS the initial level. What that
     means, stated: a claw an autonomous operation left claimed at the OTHER level moves to the
     initial level when driver control first starts — exactly as an unclaimed one does (the
     INITIAL LEVEL rule above); before the ruling, a claimed one stayed where the operation left
     it. Only a program that calls enter() itself, over a claim, gets "never sent".
 The loop reads a toggle's button only while the controller is connected, and validBindings()
 checks toggles and hold bindings as ONE set: one button, one job, none on the re-zero button.

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 Host-tested only, against recording motors, recording lines and the host test plant. Whether a
 V5 motor at 0 V in BrakeMode::Hold actually holds a LOADED mechanism is a hardware belief of
 the mechanism layer (the A4 register) that no test here can settle, and no driver has used a
 binding of either kind. Which LEVEL opens robot two's claw is not known either (the program
 states it, marked unverified).

 PROS-free, sim-free, allocation-free.
```

</details>
