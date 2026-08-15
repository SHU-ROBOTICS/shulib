<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/controller.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `controller.hpp`

ProsController — IController over pros::Controller (chunk R1a): the driver's hands behind the HAL.

This header declares **2** types (6 members).

Extracted from [`include/shulib/hal/pros/controller.hpp`](../../include/shulib/hal/pros/controller.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class ControllerId`](#enum-class-controllerid)
  - [`Master`](#controllerid-master)
  - [`Partner`](#controllerid-partner)
- [`class ProsController`](#class-proscontroller)
  - [`ProsController`](#proscontroller-proscontroller)
  - [`axis`](#proscontroller-axis)
  - [`pressed`](#proscontroller-pressed)
  - [`isConnected`](#proscontroller-isconnected)

<a id="enum-class-controllerid"></a>

## `enum class ControllerId`

```cpp
enum class ControllerId
```

Which physical controller this adapter reads.

*enum class, declared at [`include/shulib/hal/pros/controller.hpp:46`](../../include/shulib/hal/pros/controller.hpp#L46).*

<a id="controllerid-master"></a>

### `ControllerId::Master`

```cpp
Master
```

The controller paired to the brain — the only one present in a one-driver setup.

*enumerator, declared at [`include/shulib/hal/pros/controller.hpp:48`](../../include/shulib/hal/pros/controller.hpp#L48).*

<a id="controllerid-partner"></a>

### `ControllerId::Partner`

```cpp
Partner
```

The second controller, tethered to the master for VEX U's two-driver case. With no partner attached it is simply a disconnected controller: zeros on every channel and isConnected() == false, which is the case that signal exists to make visible.

*enumerator, declared at [`include/shulib/hal/pros/controller.hpp:52`](../../include/shulib/hal/pros/controller.hpp#L52).*

<a id="class-proscontroller"></a>

## `class ProsController`

```cpp
class ProsController final : public IController
```

IController over pros::Controller — one adapter per physical controller, the id fixed at construction. Reports what the driver's hands are doing, normalized, and nothing else: no deadband, curve, slew or rumble, because driver-feel is policy and belongs above this seam. Buttons are read as LEVELS via get_digital(); this adapter never calls get_digital_new_press(), whose edge state is consumed on read, so edge detection is each consumer's own hal::ButtonEdge and N consumers all see the same press.

*class, declared at [`include/shulib/hal/pros/controller.hpp:61`](../../include/shulib/hal/pros/controller.hpp#L61).*

<a id="proscontroller-proscontroller"></a>

### `ProsController::ProsController`

```cpp
explicit ProsController(ControllerId id)
```

Binds this adapter to one physical controller. `id` is a construction fact, not a per-call argument — two drivers means two instances. Touches no hardware: nothing is queried here, so construction succeeds with the controller switched off or unpaired, and isConnected() is what tells you afterwards.

*function, declared at [`include/shulib/hal/pros/controller.hpp:67`](../../include/shulib/hal/pros/controller.hpp#L67).*

<a id="proscontroller-axis"></a>

### `ProsController::axis`

```cpp
[[nodiscard]] double axis(ControllerAxis axis) const override
```

Normalized [-1, 1] via controllerAxisToCanonical (the ONE ÷127).

*function, declared at [`include/shulib/hal/pros/controller.hpp:72`](../../include/shulib/hal/pros/controller.hpp#L72).*

<a id="proscontroller-pressed"></a>

### `ProsController::pressed`

```cpp
[[nodiscard]] bool pressed(ControllerButton button) const override
```

Level read via get_digital() — NEVER get_digital_new_press() (header).

*function, declared at [`include/shulib/hal/pros/controller.hpp:78`](../../include/shulib/hal/pros/controller.hpp#L78).*

<a id="proscontroller-isconnected"></a>

### `ProsController::isConnected`

```cpp
[[nodiscard]] bool isConnected() const override
```

True while the controller is actually linked. Load-bearing rather than informational: a disconnected controller reads 0.0 on every axis and false on every button, so without this signal "the driver was unplugged" and "sticks centred, nothing pressed" are one number.

*function, declared at [`include/shulib/hal/pros/controller.hpp:85`](../../include/shulib/hal/pros/controller.hpp#L85).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 31 lines</summary>

```text

 ProsController — IController over pros::Controller (chunk R1a): the driver's
 hands behind the HAL.

 BINDS:
  * get_analog()  → axis()      (controllerAxisToCanonical — ÷127, HA-103)
  * get_digital() → pressed()   — a LEVEL, deliberately. This adapter NEVER
    calls get_digital_new_press(): PROS's edge detection is stateful on the
    device object and CONSUMES the press when read, so with two consumers
    one silently loses (HA-104, trap C). Edge detection belongs to each
    consumer, above the seam, via hal::ButtonEdge (controller.hpp) — N
    consumers all see the same press. The fence guard test greps this file
    to keep the forbidden binding structurally absent.
  * is_connected() → isConnected() — POSITIVE validity (imu.hpp:31-33
    polarity). Load-bearing because a disconnected controller reads 0 on
    every channel (HA-103): without this signal, "driver unplugged" and
    "sticks centred" are the same number.

 PARTNER SUPPORT: construct one adapter per physical controller
 (ControllerId::Master / ControllerId::Partner) — "which controller" is a
 construction fact, not an interface question (controller.hpp header).

 SENTINELS: none to screen — the controller API reports disconnection
 through is_connected() and benign zeros, not PROS_ERR (HA-103), so the
 conversion's finiteness backstop is the only guard needed.

 DELIBERATELY NOT here: deadband, curves, slew, rumble — driver-feel policy
 is chunk T2's layer. This seam reports what the hands are doing, verbatim
 (normalized), and nothing else.

 HA register: HA-103, HA-104.
```

</details>
