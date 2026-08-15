<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/digital_out.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `digital_out.hpp`

IDigitalOut — a single digital output line behind the HAL (chunk F1, WS7/M4): a pneumatic solenoid on the ADI ports, or any other two-state actuator.

This header declares **1** type (8 members).

Extracted from [`include/shulib/hal/digital_out.hpp`](../../include/shulib/hal/digital_out.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IDigitalOut`](#class-idigitalout)
  - [`~IDigitalOut`](#idigitalout-destructor-idigitalout)
  - [`IDigitalOut`](#idigitalout-idigitalout)
  - [`IDigitalOut (overload 2)`](#idigitalout-idigitalout-2)
  - [`IDigitalOut (overload 3)`](#idigitalout-idigitalout-3)
  - [`operator=`](#idigitalout-operator-eq)
  - [`operator= (overload 2)`](#idigitalout-operator-eq-2)
  - [`set`](#idigitalout-set)
  - [`commanded`](#idigitalout-commanded)

<a id="class-idigitalout"></a>

## `class IDigitalOut`

```cpp
class IDigitalOut
```

A single two-state output line — a pneumatic solenoid on the ADI ports, or any other on/off actuator. It is the first actuation seam here that is not a motor, because the H-drive's primary mechanism is a pneumatic clamp. There is NO feedback channel and no validity signal, both deliberate: a V5 solenoid cannot report whether the cylinder actually moved (air can be exhausted, a linkage can bind) and a dead ADI port reads exactly like a working one. Confirmation must come from a SEPARATE sensor — current, distance, optical — which is why the manipulation layer carries an Unconfirmed verdict rather than letting this seam pretend to know.

*class, declared at [`include/shulib/hal/digital_out.hpp:35`](../../include/shulib/hal/digital_out.hpp#L35).*

<a id="idigitalout-destructor-idigitalout"></a>

### `IDigitalOut::~IDigitalOut`

```cpp
virtual ~IDigitalOut() = default
```

Polymorphic-base boilerplate: the destructor is virtual so a concrete solenoid held as `IDigitalOut&`/`IDigitalOut*` destroys correctly, and DECLARING it is what suppresses the implicit copy/move, which are re-defaulted below. The base holds no state of its own. Ownership stays with the caller either way: PneumaticMechanism takes a NON-OWNING `span<IDigitalOut* const>` (non-empty, every line checked non-null at construction), so each line must outlive the mechanism that fans commands out to it.

*function, declared at [`include/shulib/hal/digital_out.hpp:43`](../../include/shulib/hal/digital_out.hpp#L43).*

<a id="idigitalout-idigitalout"></a>

### `IDigitalOut::IDigitalOut`

```cpp
IDigitalOut() = default
```

*Covered by the comment on [`~IDigitalOut`](#idigitalout-destructor-idigitalout) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_out.hpp:44`](../../include/shulib/hal/digital_out.hpp#L44).*

<a id="idigitalout-idigitalout-2"></a>

### `IDigitalOut::IDigitalOut (overload 2)`

```cpp
IDigitalOut(const IDigitalOut&) = default
```

*Covered by the comment on [`~IDigitalOut`](#idigitalout-destructor-idigitalout) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_out.hpp:45`](../../include/shulib/hal/digital_out.hpp#L45).*

<a id="idigitalout-idigitalout-3"></a>

### `IDigitalOut::IDigitalOut (overload 3)`

```cpp
IDigitalOut(IDigitalOut&&) = default
```

*Covered by the comment on [`~IDigitalOut`](#idigitalout-destructor-idigitalout) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_out.hpp:46`](../../include/shulib/hal/digital_out.hpp#L46).*

<a id="idigitalout-operator-eq"></a>

### `IDigitalOut::operator=`

```cpp
IDigitalOut& operator=(const IDigitalOut&) = default
```

*Covered by the comment on [`~IDigitalOut`](#idigitalout-destructor-idigitalout) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_out.hpp:47`](../../include/shulib/hal/digital_out.hpp#L47).*

<a id="idigitalout-operator-eq-2"></a>

### `IDigitalOut::operator= (overload 2)`

```cpp
IDigitalOut& operator=(IDigitalOut&&) = default
```

*Covered by the comment on [`~IDigitalOut`](#idigitalout-destructor-idigitalout) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_out.hpp:48`](../../include/shulib/hal/digital_out.hpp#L48).*

<a id="idigitalout-set"></a>

### `IDigitalOut::set`

```cpp
virtual void set(bool value) = 0
```

Command the line high (true) or low (false). What "true" means physically (clamp closed? cylinder extended?) is plumbing-dependent and belongs to the mechanism that owns the line, not to this seam.

*function, declared at [`include/shulib/hal/digital_out.hpp:53`](../../include/shulib/hal/digital_out.hpp#L53).*

<a id="idigitalout-commanded"></a>

### `IDigitalOut::commanded`

```cpp
[[nodiscard]] virtual bool commanded() const = 0
```

The value actually commanded (readback of the command, NOT of the world — see the no-feedback note in the header).

*function, declared at [`include/shulib/hal/digital_out.hpp:57`](../../include/shulib/hal/digital_out.hpp#L57).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 23 lines</summary>

```text

 IDigitalOut — a single digital output line behind the HAL (chunk F1, WS7/M4):
 a pneumatic solenoid on the ADI ports, or any other two-state actuator. The
 first actuation seam in the tree that is not a motor — the H-drive's primary
 mechanism is a pneumatic clamp (master plan §14), so a mechanism layer shaped
 only around motors would have been shaped by half the hardware.

 Contract:
  * set() commands the line; commanded() returns the value actually COMMANDED —
    the same telemetry-reflects-the-command rule as IMotor::commandedVoltage().
  * There is NO feedback channel, deliberately. A V5 solenoid cannot report
    whether the cylinder moved: air can be exhausted, a linkage can bind, and
    the line reads exactly what it was told either way. That physical fact is
    why the manipulation layer's ActuateAndConfirm carries an Unconfirmed
    verdict — confirmation must come from a SEPARATE sensor (current, distance,
    optical), never from this seam pretending to know.
  * Like the always-valid sensors (F4 note, master plan §7): no validity
    signal. A dead ADI port is indistinguishable from a working one here;
    detecting the difference is a confirmation sensor's job.

 NOT part of the F4 freeze (that register row locked 2026-06-19 with the ten
 runtime interfaces; this seam is F1's addition, outside it — register row F11
 records the non-freeze). The hal/pros implementation (ADI digital out) is R1's.
```

</details>
