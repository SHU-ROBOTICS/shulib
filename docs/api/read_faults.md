<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/read_faults.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `read_faults.hpp`

IReadFaults — how many reads a SENSOR ADAPTER has SCREENED.

This header declares **1** type (7 members).

Extracted from [`include/shulib/hal/read_faults.hpp`](../../include/shulib/hal/read_faults.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IReadFaults`](#class-ireadfaults)
  - [`~IReadFaults`](#ireadfaults-destructor-ireadfaults)
  - [`IReadFaults`](#ireadfaults-ireadfaults)
  - [`IReadFaults (overload 2)`](#ireadfaults-ireadfaults-2)
  - [`IReadFaults (overload 3)`](#ireadfaults-ireadfaults-3)
  - [`operator=`](#ireadfaults-operator-eq)
  - [`operator= (overload 2)`](#ireadfaults-operator-eq-2)
  - [`faultedReads`](#ireadfaults-faultedreads)

<a id="class-ireadfaults"></a>

## `class IReadFaults`

```cpp
class IReadFaults
```

How many reads a sensor adapter has SCREENED because the device failed (unplugged, a wrong port, a PROS_ERR) — a scalar sensor's answered with its last good value, a camera's as a frame that says it failed. A consumer that must not act on a held value remembers this count and compares it at its next read: a change means a value it was given in between was held, not measured — for a camera, a read in between was not evidence (header).

*class, declared at [`include/shulib/hal/read_faults.hpp:49`](../../include/shulib/hal/read_faults.hpp#L49).*

<a id="ireadfaults-destructor-ireadfaults"></a>

### `IReadFaults::~IReadFaults`

```cpp
virtual ~IReadFaults() = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IReadFaults*` well-defined. A consumer holds one by pointer and never owns it (the provider does).

*function, declared at [`include/shulib/hal/read_faults.hpp:53`](../../include/shulib/hal/read_faults.hpp#L53).*

<a id="ireadfaults-ireadfaults"></a>

### `IReadFaults::IReadFaults`

```cpp
IReadFaults() = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IReadFaults*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state.

*function, declared at [`include/shulib/hal/read_faults.hpp:55`](../../include/shulib/hal/read_faults.hpp#L55).*

<a id="ireadfaults-ireadfaults-2"></a>

### `IReadFaults::IReadFaults (overload 2)`

```cpp
IReadFaults(const IReadFaults&) = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IReadFaults*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state. Copyable at the base, because the base holds no state; an adapter decides for itself.

*function, declared at [`include/shulib/hal/read_faults.hpp:57`](../../include/shulib/hal/read_faults.hpp#L57).*

<a id="ireadfaults-ireadfaults-3"></a>

### `IReadFaults::IReadFaults (overload 3)`

```cpp
IReadFaults(IReadFaults&&) = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IReadFaults*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state. Copyable at the base, because the base holds no state; an adapter decides for itself. Movable, same reasoning as the copy constructor.

*function, declared at [`include/shulib/hal/read_faults.hpp:59`](../../include/shulib/hal/read_faults.hpp#L59).*

<a id="ireadfaults-operator-eq"></a>

### `IReadFaults::operator=`

```cpp
IReadFaults& operator=(const IReadFaults&) = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IReadFaults*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state. Copyable at the base, because the base holds no state; an adapter decides for itself. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning.

*function, declared at [`include/shulib/hal/read_faults.hpp:61`](../../include/shulib/hal/read_faults.hpp#L61).*

<a id="ireadfaults-operator-eq-2"></a>

### `IReadFaults::operator= (overload 2)`

```cpp
IReadFaults& operator=(IReadFaults&&) = default
```

Polymorphic base: the virtual destructor makes `delete` through an `IReadFaults*` well-defined. A consumer holds one by pointer and never owns it (the provider does). Default-constructible: the base has no state. Copyable at the base, because the base holds no state; an adapter decides for itself. Movable, same reasoning as the copy constructor. Copy-assignable, same reasoning. Move-assignable, same reasoning.

*function, declared at [`include/shulib/hal/read_faults.hpp:63`](../../include/shulib/hal/read_faults.hpp#L63).*

<a id="ireadfaults-faultedreads"></a>

### `IReadFaults::faultedReads`

```cpp
[[nodiscard]] virtual int faultedReads() const noexcept = 0
```

Reads screened since the adapter was constructed. MONOTONIC: it never decreases and is never reset, so "it moved" is the only question a consumer may ask of it.

*function, declared at [`include/shulib/hal/read_faults.hpp:67`](../../include/shulib/hal/read_faults.hpp#L67).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 39 lines</summary>

```text

 IReadFaults — how many reads a SENSOR ADAPTER has SCREENED (chunk MECH-A1, 2026-10-08).

 ── An F4-ADDITIVE SIBLING, not a change to F4 ──────────────────────────────────────
 Exactly as IDigitalOut (F11), IController (F13) and IDigitalIn (F14) were added beside the
 locked F4 seams, this one is added beside them: IDistance, IOptical, IRotation and the rest are
 untouched, byte for byte. An adapter that screens its reads implements this interface AS WELL
 as its seam (hal::pros::ProsDistance, ProsOptical, ProsDigitalIn — and, since VIS1-A1, the AI
 Vision camera's hal::pros::ProsAiVision); a consumer that needs the count asks the device provider
 for it (IDeviceProvider::readFaults / lineReadFaults).

 ── WHY IT EXISTS (the MECH brief's re-check, its probe R) ──────────────────────────
 The locked sensor seams carry NO validity channel, by design: a PROS adapter that gets a
 PROS_ERR answers with its LAST GOOD value (hal/pros/distance.hpp's "T7" note; optical.hpp;
 digital_in.hpp) and counts the screen in its own faultedReads(). At the seam a held value is
 indistinguishable from a fresh one — and for a distance sensor a value held from a FAR reading
 (or from boot: the hold starts at the far no-object distance) is byte for byte the 9999
 "no object" reading. The mechanism core ends a step on "the piece is GONE"; acting on a held
 value there is a false success: the brief's probe R measured a distance sensor whose cable came
 out while the intake was empty, one dead from boot, and an optical that failed while empty —
 each ended an eject step SUCCEEDED "stopped seeing a piece" at its first tick with the motor
 never run. So the core reads this count BESIDE the value: a sample taken while the count moved
 is UNKNOWN, never evidence (the rule is the mechanism service's, MECH-A2).

 ── A camera's count: a frame that SAID it failed, not a held value (VIS1-A1) ─────────
 The camera's seam (hal/vision_frames.hpp) HAS a validity channel: a failed or torn read is a frame
 whose status says so, with no detections — never the last good frame held (a held frame would read
 as a fresh "nothing changed"). Its adapter counts those frames here too, so a consumer written for
 this channel works unchanged; for a camera the count's change means "a read in between was not
 evidence", and nothing was held.

 ── What it cannot see ──────────────────────────────────────────────────────────────
 Only what the adapter can: a read the device REFUSED. An UNPLUGGED 3-wire switch is not a
 refused read — the brain reads a level from an empty 3-wire port like any other (HA-158), and
 which level is HA-184's question. A count that never moves is therefore not proof a switch is
 connected.

 One member, no state of its own, no reset: the count only grows (each adapter's own rule).
 PROS-free, allocation-free.
```

</details>
