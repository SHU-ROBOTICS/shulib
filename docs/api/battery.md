<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/battery.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `battery.hpp`

IBattery — the V5 battery (pros::battery) behind the HAL.

This header declares **1** type (9 members).

Extracted from [`include/shulib/hal/battery.hpp`](../../include/shulib/hal/battery.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IBattery`](#class-ibattery)
  - [`~IBattery`](#ibattery-destructor-ibattery)
  - [`IBattery`](#ibattery-ibattery)
  - [`IBattery (overload 2)`](#ibattery-ibattery-2)
  - [`IBattery (overload 3)`](#ibattery-ibattery-3)
  - [`operator=`](#ibattery-operator-eq)
  - [`operator= (overload 2)`](#ibattery-operator-eq-2)
  - [`voltage`](#ibattery-voltage)
  - [`current`](#ibattery-current)
  - [`capacity`](#ibattery-capacity)

<a id="class-ibattery"></a>

## `class IBattery`

```cpp
class IBattery
```

The V5 battery behind the HAL. Its ONLY effect on control is a CEILING, never a scale factor: shulib commands actual volts, so control::compensateForBattery() clamps a desired voltage to ±the measured battery and reports whether it saturated, while the kS/kV/kA gains stay battery-independent by construction. Nothing anywhere multiplies a command by a fraction of pack voltage — the percent-output mental model is the one this design rejects.

*class, declared at [`include/shulib/hal/battery.hpp:28`](../../include/shulib/hal/battery.hpp#L28).*

<a id="ibattery-destructor-ibattery"></a>

### `IBattery::~IBattery`

```cpp
virtual ~IBattery() = default
```

Abstract base, held and destroyed through IBattery*. RobotContext keeps a NON-OWNING pointer (validated non-null at construction) that the caller must keep alive for the context's whole life. Copy/move are defaulted because the interface itself carries no state, but copying THROUGH this base slices an implementation down to nothing — pass implementations by reference or pointer, never by value.

*function, declared at [`include/shulib/hal/battery.hpp:35`](../../include/shulib/hal/battery.hpp#L35).*

<a id="ibattery-ibattery"></a>

### `IBattery::IBattery`

```cpp
IBattery() = default
```

*Covered by the comment on [`~IBattery`](#ibattery-destructor-ibattery) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/battery.hpp:36`](../../include/shulib/hal/battery.hpp#L36).*

<a id="ibattery-ibattery-2"></a>

### `IBattery::IBattery (overload 2)`

```cpp
IBattery(const IBattery&) = default
```

*Covered by the comment on [`~IBattery`](#ibattery-destructor-ibattery) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/battery.hpp:37`](../../include/shulib/hal/battery.hpp#L37).*

<a id="ibattery-ibattery-3"></a>

### `IBattery::IBattery (overload 3)`

```cpp
IBattery(IBattery&&) = default
```

*Covered by the comment on [`~IBattery`](#ibattery-destructor-ibattery) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/battery.hpp:38`](../../include/shulib/hal/battery.hpp#L38).*

<a id="ibattery-operator-eq"></a>

### `IBattery::operator=`

```cpp
IBattery& operator=(const IBattery&) = default
```

*Covered by the comment on [`~IBattery`](#ibattery-destructor-ibattery) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/battery.hpp:39`](../../include/shulib/hal/battery.hpp#L39).*

<a id="ibattery-operator-eq-2"></a>

### `IBattery::operator= (overload 2)`

```cpp
IBattery& operator=(IBattery&&) = default
```

*Covered by the comment on [`~IBattery`](#ibattery-destructor-ibattery) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/battery.hpp:40`](../../include/shulib/hal/battery.hpp#L40).*

<a id="ibattery-voltage"></a>

### `IBattery::voltage`

```cpp
[[nodiscard]] virtual units::Voltage voltage() const = 0
```

Present battery voltage (canonical volts).

*function, declared at [`include/shulib/hal/battery.hpp:43`](../../include/shulib/hal/battery.hpp#L43).*

<a id="ibattery-current"></a>

### `IBattery::current`

```cpp
[[nodiscard]] virtual units::Current current() const = 0
```

Present current draw (canonical amperes) — the I half of the DebugRecord battery V/I (§18.2); voltage × current is dimensionally Power.

*function, declared at [`include/shulib/hal/battery.hpp:47`](../../include/shulib/hal/battery.hpp#L47).*

<a id="ibattery-capacity"></a>

### `IBattery::capacity`

```cpp
[[nodiscard]] virtual double capacity() const = 0
```

Remaining capacity in [0, 1].

*function, declared at [`include/shulib/hal/battery.hpp:50`](../../include/shulib/hal/battery.hpp#L50).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 16 lines</summary>

```text

 IBattery — the V5 battery (pros::battery) behind the HAL. voltage() in canonical
 volts; capacity() in [0, 1]. Feeds voltage / brownout compensation (master plan
 §M2).

 WHAT THAT COMPENSATION IS, EXACTLY — this comment said the wrong thing until DOCS1
 (2026-08-14), and it was the ancestor of the same wrong claim in guide ch. 6. It read
 "the control layer SCALES motor commands by the measured battery voltage so a routine
 behaves the same on a full or a sagging battery." Nothing scales. shulib commands
 ACTUAL VOLTS (IMotor::setVoltage), so the only battery effect is a CEILING:
 control::compensateForBattery() clamps a desired voltage to +/-battery and flags the
 saturation, and the kS/kV/kA gains are battery-independent by construction
 (control/feedforward.hpp:13-17, which has been right all along — the two headers
 disagreed). The percent-output mental model this comment implied is precisely the one
 the design rejects. What survives unchanged: the guaranteed end-of-run park still
 fires as the battery collapses.
```

</details>
