<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/digital_out.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `digital_out.hpp`

ProsDigitalOut — IDigitalOut over pros::adi::DigitalOut (chunk R1b): the pneumatic solenoid line behind the HAL (F1's seam, finally on hardware).

This header declares **1** type (6 members).

Extracted from [`include/shulib/hal/pros/digital_out.hpp`](../../include/shulib/hal/pros/digital_out.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsDigitalOut`](#class-prosdigitalout)
  - [`ProsDigitalOut`](#prosdigitalout-prosdigitalout)
  - [`ProsDigitalOut (overload 2)`](#prosdigitalout-prosdigitalout-2)
  - [`ProsDigitalOut (overload 3)`](#prosdigitalout-prosdigitalout-3)
  - [`set`](#prosdigitalout-set)
  - [`commanded`](#prosdigitalout-commanded)
  - [`faultedWrites`](#prosdigitalout-faultedwrites)

<a id="class-prosdigitalout"></a>

## `class ProsDigitalOut`

```cpp
class ProsDigitalOut final : public IDigitalOut
```

IDigitalOut over `pros::adi::DigitalOut` — the pneumatic solenoid line, on real hardware. CONSTRUCTING ONE IS A PHYSICAL ACTION: PROS drives the line from its own constructor, so on a pneumatic clamp the cylinder moves the moment this object is built. That is why `initialState` is a required argument with NO default — the author must state the boot level, and must state one that agrees with the owning PneumaticMechanism's declared safe state, or there is a window at boot where the line is wrong and "wrong" means physically moving. A write the port refuses is COUNTED (faultedWrites), never raised: the seam has no validity channel by design, and commanded() goes on reporting the caller's intent, which is exactly what makes a refused write visible as a divergence instead of a device that silently agrees with itself.

*class, declared at [`include/shulib/hal/pros/digital_out.hpp:68`](../../include/shulib/hal/pros/digital_out.hpp#L68).*

<a id="prosdigitalout-prosdigitalout"></a>

### `ProsDigitalOut::ProsDigitalOut`

```cpp
ProsDigitalOut(std::uint8_t adiPort, bool initialState)
```

Brain ADI port ('a'–'h', 'A'–'H', or 1–8). CONSTRUCTION DRIVES THE LINE to `initialState` — a physical action (header). `initialState` is required, no default: state what the boot level must be, and make it agree with the owning mechanism's declared safe state.

*function, declared at [`include/shulib/hal/pros/digital_out.hpp:74`](../../include/shulib/hal/pros/digital_out.hpp#L74).*

<a id="prosdigitalout-prosdigitalout-2"></a>

### `ProsDigitalOut::ProsDigitalOut (overload 2)`

```cpp
template <typename T, typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, bool>>> ProsDigitalOut(std::uint8_t adiPort, T initialState) = delete
```

POISON OVERLOAD, deliberately deleted: the EXPANDER form written with the boot state forgotten. `ProsDigitalOut d(1, 2);` — a caller meaning {smartPort 1, adiPort 2} — used to compile CLEAN under every one of this project's strict flags, silently selecting the brain-ADI constructor above with adiPort = 1 and initialState = (bool)2 = true. Construction is a PHYSICAL ACTION, so that typo fires the solenoid HIGH at boot, on the wrong port: exactly the failure the required argument exists to prevent, defeating the header's central claim that this safety step "cannot be skipped, only stated". Brace initialisation already rejected it (narrowing int → bool), but parentheses did not. Now neither does.

*function, declared at [`include/shulib/hal/pros/digital_out.hpp:87`](../../include/shulib/hal/pros/digital_out.hpp#L87).*

<a id="prosdigitalout-prosdigitalout-3"></a>

### `ProsDigitalOut::ProsDigitalOut (overload 3)`

```cpp
ProsDigitalOut(std::uint8_t smartPort, std::uint8_t adiPort, bool initialState)
```

Expander form: {smartPort 1–21, adiPort as above}. Same actuating construction, same required initial state (T6: one class — where the wire lands is a construction fact, not a type).

*function, declared at [`include/shulib/hal/pros/digital_out.hpp:92`](../../include/shulib/hal/pros/digital_out.hpp#L92).*

<a id="prosdigitalout-set"></a>

### `ProsDigitalOut::set`

```cpp
void set(bool value) override
```

Command the line (bool → 1/0, HA-119). A refused write is counted in faultedWrites(); commanded() reports the command regardless (header).

*function, declared at [`include/shulib/hal/pros/digital_out.hpp:98`](../../include/shulib/hal/pros/digital_out.hpp#L98).*

<a id="prosdigitalout-commanded"></a>

### `ProsDigitalOut::commanded`

```cpp
[[nodiscard]] bool commanded() const override
```

The value last COMMANDED (the ctor's initial state until set() runs) — a readback of the command, NEVER of the world (digital_out.hpp:44).

*function, declared at [`include/shulib/hal/pros/digital_out.hpp:107`](../../include/shulib/hal/pros/digital_out.hpp#L107).*

<a id="prosdigitalout-faultedwrites"></a>

### `ProsDigitalOut::faultedWrites`

```cpp
[[nodiscard]] int faultedWrites() const noexcept
```

How many set() calls (ctor excluded) the device refused with PROS_ERR — exposure, not policy (raising stays with the loop layer).

*function, declared at [`include/shulib/hal/pros/digital_out.hpp:111`](../../include/shulib/hal/pros/digital_out.hpp#L111).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 41 lines</summary>

```text

 ProsDigitalOut — IDigitalOut over pros::adi::DigitalOut (chunk R1b): the
 pneumatic solenoid line behind the HAL (F1's seam, finally on hardware).

 ── CONSTRUCTION IS A PHYSICAL ACTION (T3) ──────────────────────────────────────────
 pros::adi::DigitalOut drives the line the moment it is constructed — its
 ctor takes `init_state` and PROS defaults it to LOW (vendored
 adi.hpp:546-547,564,596; HA-119). On a pneumatic clamp that means THE
 CYLINDER MOVES WHEN THIS OBJECT IS BUILT. This adapter therefore REFUSES
 the default: `initialState` is a REQUIRED constructor argument, so the
 author must state what the line should be at boot — and that statement
 must AGREE with the owning PneumaticMechanism's declared safe state
 (mechanism.hpp), or there is a window at boot where the line is wrong and
 "wrong" means physically moving. A safety step a caller can forget is a
 safety step that WILL be forgotten (the legacy escapeJSONString lesson);
 this one cannot be skipped, only stated.

 BINDS:
  * ctor(init_state) — the boot actuation, stated explicitly (HA-119)
  * set_value(1/0) ← set(bool) — the one bool→int32 mapping (HA-119)

 ADDRESSING (T6, HA-120): one class, two constructors — the brain's own 8
 ADI ports ('a'–'h', 'A'–'H', or 1–8) or an expander's {smartPort, adiPort}.
 The seam is identical either way; where the wire lands is a construction
 fact, never a type. Whether OUR robot has an expander is UNKNOWN (R1a's
 expander report came from an out-of-range registry index — bench step).

 COMMAND, NOT WORLD: commanded() returns the value this adapter last
 COMMANDED (ctor's initial state until set() is called) — never a read of
 the device (digital_out.hpp:44: there IS no feedback channel; air can be
 exhausted, a linkage can bind, and the line reads what it was told either
 way). Confirmation comes from a SEPARATE sensor, by contract.

 SENTINELS (T7 shape, write-side): set_value() returns PROS_ERR when the
 port refuses (misconfigured/reconfigured — HA-119). The seam has no
 validity channel by design, so the refusal is EXPOSED, not raised:
 faultedWrites() counts refused writes; commanded() still reports the
 command (the caller's intent is the telemetry contract even when the wire
 refused — that is what makes a refused write VISIBLE as a divergence).

 HA register: HA-119, HA-120 (docs/hardware-assumptions.md).
```

</details>
