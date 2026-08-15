<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/controller.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `controller.hpp`

IController — the V5 game controller behind the HAL (chunk R1a, absorbing Phase T's T1): the INPUT half of driver control.

This header declares **4** types (26 members).

Extracted from [`include/shulib/hal/controller.hpp`](../../include/shulib/hal/controller.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class ControllerAxis`](#enum-class-controlleraxis)
  - [`LeftX`](#controlleraxis-leftx)
  - [`LeftY`](#controlleraxis-lefty)
  - [`RightX`](#controlleraxis-rightx)
  - [`RightY`](#controlleraxis-righty)
- [`enum class ControllerButton`](#enum-class-controllerbutton)
  - [`L1`](#controllerbutton-l1)
  - [`L2`](#controllerbutton-l2)
  - [`R1`](#controllerbutton-r1)
  - [`R2`](#controllerbutton-r2)
  - [`Up`](#controllerbutton-up)
  - [`Down`](#controllerbutton-down)
  - [`Left`](#controllerbutton-left)
  - [`Right`](#controllerbutton-right)
  - [`X`](#controllerbutton-x)
  - [`B`](#controllerbutton-b)
  - [`Y`](#controllerbutton-y)
  - [`A`](#controllerbutton-a)
- [`class IController`](#class-icontroller)
  - [`~IController`](#icontroller-destructor-icontroller)
  - [`IController`](#icontroller-icontroller)
  - [`IController (overload 2)`](#icontroller-icontroller-2)
  - [`IController (overload 3)`](#icontroller-icontroller-3)
  - [`operator=`](#icontroller-operator-eq)
  - [`operator= (overload 2)`](#icontroller-operator-eq-2)
  - [`axis`](#icontroller-axis)
  - [`pressed`](#icontroller-pressed)
  - [`isConnected`](#icontroller-isconnected)
- [`class ButtonEdge`](#class-buttonedge)
  - [`update`](#buttonedge-update)

<a id="enum-class-controlleraxis"></a>

## `enum class ControllerAxis`

```cpp
enum class ControllerAxis
```

The four analog stick channels, named by physical position.

*enum class, declared at [`include/shulib/hal/controller.hpp:38`](../../include/shulib/hal/controller.hpp#L38).*

<a id="controlleraxis-leftx"></a>

### `ControllerAxis::LeftX`

```cpp
LeftX
```

left stick, horizontal (+ = pushed right)

*enumerator, declared at [`include/shulib/hal/controller.hpp:39`](../../include/shulib/hal/controller.hpp#L39).*

<a id="controlleraxis-lefty"></a>

### `ControllerAxis::LeftY`

```cpp
LeftY
```

left stick, vertical (+ = pushed up)

*enumerator, declared at [`include/shulib/hal/controller.hpp:40`](../../include/shulib/hal/controller.hpp#L40).*

<a id="controlleraxis-rightx"></a>

### `ControllerAxis::RightX`

```cpp
RightX
```

right stick, horizontal (+ = pushed right)

*enumerator, declared at [`include/shulib/hal/controller.hpp:41`](../../include/shulib/hal/controller.hpp#L41).*

<a id="controlleraxis-righty"></a>

### `ControllerAxis::RightY`

```cpp
RightY
```

right stick, vertical (+ = pushed up)

*enumerator, declared at [`include/shulib/hal/controller.hpp:42`](../../include/shulib/hal/controller.hpp#L42).*

<a id="enum-class-controllerbutton"></a>

## `enum class ControllerButton`

```cpp
enum class ControllerButton
```

The twelve driver-usable buttons. The power button is deliberately absent: pressing it turns the controller off, so no routine may ever bind it.

*enum class, declared at [`include/shulib/hal/controller.hpp:47`](../../include/shulib/hal/controller.hpp#L47).*

<a id="controllerbutton-l1"></a>

### `ControllerButton::L1`

```cpp
L1
```

left shoulder, upper trigger (PROS's "first" left trigger)

*enumerator, declared at [`include/shulib/hal/controller.hpp:48`](../../include/shulib/hal/controller.hpp#L48).*

<a id="controllerbutton-l2"></a>

### `ControllerButton::L2`

```cpp
L2
```

left shoulder, lower trigger (PROS's "second" left trigger)

*enumerator, declared at [`include/shulib/hal/controller.hpp:49`](../../include/shulib/hal/controller.hpp#L49).*

<a id="controllerbutton-r1"></a>

### `ControllerButton::R1`

```cpp
R1
```

right shoulder, upper trigger

*enumerator, declared at [`include/shulib/hal/controller.hpp:50`](../../include/shulib/hal/controller.hpp#L50).*

<a id="controllerbutton-r2"></a>

### `ControllerButton::R2`

```cpp
R2
```

right shoulder, lower trigger

*enumerator, declared at [`include/shulib/hal/controller.hpp:51`](../../include/shulib/hal/controller.hpp#L51).*

<a id="controllerbutton-up"></a>

### `ControllerButton::Up`

```cpp
Up
```

left arrow pad, up

*enumerator, declared at [`include/shulib/hal/controller.hpp:52`](../../include/shulib/hal/controller.hpp#L52).*

<a id="controllerbutton-down"></a>

### `ControllerButton::Down`

```cpp
Down
```

left arrow pad, down

*enumerator, declared at [`include/shulib/hal/controller.hpp:53`](../../include/shulib/hal/controller.hpp#L53).*

<a id="controllerbutton-left"></a>

### `ControllerButton::Left`

```cpp
Left
```

left arrow pad, left

*enumerator, declared at [`include/shulib/hal/controller.hpp:54`](../../include/shulib/hal/controller.hpp#L54).*

<a id="controllerbutton-right"></a>

### `ControllerButton::Right`

```cpp
Right
```

left arrow pad, right

*enumerator, declared at [`include/shulib/hal/controller.hpp:55`](../../include/shulib/hal/controller.hpp#L55).*

<a id="controllerbutton-x"></a>

### `ControllerButton::X`

```cpp
X
```

Right button pad, TOP of the diamond. These four are declared X, B, Y, A — PROS's own order, which is neither alphabetical nor a walk around the pad, so take each button's position from these notes and never from the order.

*enumerator, declared at [`include/shulib/hal/controller.hpp:59`](../../include/shulib/hal/controller.hpp#L59).*

<a id="controllerbutton-b"></a>

### `ControllerButton::B`

```cpp
B
```

right button pad, RIGHT of the diamond

*enumerator, declared at [`include/shulib/hal/controller.hpp:60`](../../include/shulib/hal/controller.hpp#L60).*

<a id="controllerbutton-y"></a>

### `ControllerButton::Y`

```cpp
Y
```

right button pad, LEFT of the diamond

*enumerator, declared at [`include/shulib/hal/controller.hpp:61`](../../include/shulib/hal/controller.hpp#L61).*

<a id="controllerbutton-a"></a>

### `ControllerButton::A`

```cpp
A
```

right button pad, BOTTOM of the diamond

*enumerator, declared at [`include/shulib/hal/controller.hpp:62`](../../include/shulib/hal/controller.hpp#L62).*

<a id="class-icontroller"></a>

## `class IController`

```cpp
class IController
```

The INPUT half of driver control behind the HAL: normalized sticks, LEVEL buttons, and a positive connected/not signal. Canonical at the seam — the V5's raw −127…127 is scaled away in the adapter, and edge detection is refused here on purpose (see ButtonEdge below). One instance per PHYSICAL controller: VEX U's two drivers are a CONSTRUCTION fact (the PROS adapter takes a ControllerId), never an argument here.

*class, declared at [`include/shulib/hal/controller.hpp:70`](../../include/shulib/hal/controller.hpp#L70).*

<a id="icontroller-destructor-icontroller"></a>

### `IController::~IController`

```cpp
virtual ~IController() = default
```

Public defaulted special members on a polymorphic base: the virtual destructor makes `delete` through an `IController*` well-defined, and copy/move stay available so an adapter deriving from this is free to be value-like. This base carries no state, so copying one copies nothing — hold implementations by reference or pointer, never by value.

*function, declared at [`include/shulib/hal/controller.hpp:77`](../../include/shulib/hal/controller.hpp#L77).*

<a id="icontroller-icontroller"></a>

### `IController::IController`

```cpp
IController() = default
```

*Covered by the comment on [`~IController`](#icontroller-destructor-icontroller) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/controller.hpp:78`](../../include/shulib/hal/controller.hpp#L78).*

<a id="icontroller-icontroller-2"></a>

### `IController::IController (overload 2)`

```cpp
IController(const IController&) = default
```

*Covered by the comment on [`~IController`](#icontroller-destructor-icontroller) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/controller.hpp:79`](../../include/shulib/hal/controller.hpp#L79).*

<a id="icontroller-icontroller-3"></a>

### `IController::IController (overload 3)`

```cpp
IController(IController&&) = default
```

*Covered by the comment on [`~IController`](#icontroller-destructor-icontroller) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/controller.hpp:80`](../../include/shulib/hal/controller.hpp#L80).*

<a id="icontroller-operator-eq"></a>

### `IController::operator=`

```cpp
IController& operator=(const IController&) = default
```

*Covered by the comment on [`~IController`](#icontroller-destructor-icontroller) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/controller.hpp:81`](../../include/shulib/hal/controller.hpp#L81).*

<a id="icontroller-operator-eq-2"></a>

### `IController::operator= (overload 2)`

```cpp
IController& operator=(IController&&) = default
```

*Covered by the comment on [`~IController`](#icontroller-destructor-icontroller) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/controller.hpp:82`](../../include/shulib/hal/controller.hpp#L82).*

<a id="icontroller-axis"></a>

### `IController::axis`

```cpp
[[nodiscard]] virtual double axis(ControllerAxis axis) const = 0
```

Current deflection of `axis`, normalized to [-1, 1] (full deflection = ±1). Reads 0.0 while disconnected — check isConnected() to tell a drop from centred sticks.

*function, declared at [`include/shulib/hal/controller.hpp:87`](../../include/shulib/hal/controller.hpp#L87).*

<a id="icontroller-pressed"></a>

### `IController::pressed`

```cpp
[[nodiscard]] virtual bool pressed(ControllerButton button) const = 0
```

True while `button` is held down (a LEVEL, not an edge — see header: per-consumer edge detection lives in ButtonEdge, above the seam).

*function, declared at [`include/shulib/hal/controller.hpp:91`](../../include/shulib/hal/controller.hpp#L91).*

<a id="icontroller-isconnected"></a>

### `IController::isConnected`

```cpp
[[nodiscard]] virtual bool isConnected() const = 0
```

True while this controller is connected and its readings are live. POSITIVE polarity by convention (cf. IImu::isReady, IGps::hasFix).

*function, declared at [`include/shulib/hal/controller.hpp:95`](../../include/shulib/hal/controller.hpp#L95).*

<a id="class-buttonedge"></a>

## `class ButtonEdge`

```cpp
class ButtonEdge
```

Per-consumer rising-edge detector over IController::pressed() levels — the PROS-free replacement for get_digital_new_press(), which consumes the event on read so a second consumer silently misses every press (HA-104). Each consumer owns its own ButtonEdge, so N consumers all see the same press.  Stateful by nature (it must remember the previous level); call update() exactly once per tick per consumer.

*class, declared at [`include/shulib/hal/controller.hpp:105`](../../include/shulib/hal/controller.hpp#L105).*

<a id="buttonedge-update"></a>

### `ButtonEdge::update`

```cpp
[[nodiscard]] bool update(bool pressedNow) noexcept
```

Feed this tick's level; returns true exactly on a false→true transition. The first call after construction reports a press only if the button is already down (prev starts false) — a button held across a mode switch registers once, deliberately, rather than being swallowed.

*function, declared at [`include/shulib/hal/controller.hpp:111`](../../include/shulib/hal/controller.hpp#L111).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 32 lines</summary>

```text

 IController — the V5 game controller behind the HAL (chunk R1a, absorbing
 Phase T's T1): the INPUT half of driver control. The frozen F6 facade already
 has the OUTPUT half — drive(ChassisSpeeds, Frame) — so this seam is what
 makes the library a one-stop shop for teleop without a second library.

 Canonical at the seam, like every other HAL edge (§7):
  * axis() is normalized to [-1, 1] — the V5's raw −127…127 never reaches the
    core (controller_conversion.hpp owns the one scale; HA-103).
  * pressed() is a plain bool LEVEL (is it down NOW). Edge detection ("was it
    JUST pressed") deliberately lives ABOVE the seam, in ButtonEdge below —
    PROS's get_digital_new_press() CONSUMES the event on read, so with two
    consumers one silently loses. The adapter binds get_digital() and NEVER
    get_digital_new_press() (HA-104); each consumer owns its own ButtonEdge.
  * isConnected() is a POSITIVE validity signal (true = usable), matching
    IImu::isReady() / IGps::hasFix() polarity (imu.hpp:31-33) — a controller
    really does drop mid-match, and the core must be able to tell that from
    "sticks centred" (PROS reports 0 on every channel when disconnected, so
    without this signal the two are indistinguishable).

 PARTNER SUPPORT is structural, not a retrofit: VEX U runs two drivers, so
 "which controller" is a CONSTRUCTION question (make two instances — the
 hal/pros adapter takes ControllerId::Master or ControllerId::Partner), never
 an interface question. Retrofitting a second driver through a single-
 controller seam is the reshape a seam exists to prevent.

 NOT part of the F4 freeze (that register row locked 2026-06-19 with the ten
 runtime interfaces; this seam is R1a's addition, outside it — the same
 additive-sibling shape F1 used for IDigitalOut, digital_out.hpp:22-24).
 Register row F13 records the NON-freeze out loud (D2's lesson: silence in
 that register reads as "frozen"). Freeze trigger: a second real consumer —
 T2's driver-control layer is the first.
```

</details>
