<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/gps.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `gps.hpp`

IGps — the VEX GPS behind the HAL, reporting in shulib's CANONICAL frame (the VEX meters / clockwise-from-North convention is converted away in the hal/pros adapter via gps_conversion.hpp).

This header declares **1** type (9 members).

Extracted from [`include/shulib/hal/gps.hpp`](../../include/shulib/hal/gps.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IGps`](#class-igps)
  - [`~IGps`](#igps-destructor-igps)
  - [`IGps`](#igps-igps)
  - [`IGps (overload 2)`](#igps-igps-2)
  - [`IGps (overload 3)`](#igps-igps-3)
  - [`operator=`](#igps-operator-eq)
  - [`operator= (overload 2)`](#igps-operator-eq-2)
  - [`pose`](#igps-pose)
  - [`rmsError`](#igps-rmserror)
  - [`hasFix`](#igps-hasfix)

<a id="class-igps"></a>

## `class IGps`

```cpp
class IGps
```

The VEX GPS behind the HAL, already converted to shulib's canonical frame and units by the adapter — nothing above this seam ever sees VEX metres or clockwise-from-North. Read it as one three-part answer in a fixed order: hasFix() decides whether this tick's reading exists at all, pose() is the robot-CENTER estimate, and rmsError() is the sensor's confidence in itself, which the fuser turns into a measurement noise R rather than a reason to snap. A permanently false hasFix() is a SUPPORTED mode, not a fault: Driving Skills runs on a field with no GPS strip and the estimator dead-reckons.

*class, declared at [`include/shulib/hal/gps.hpp:24`](../../include/shulib/hal/gps.hpp#L24).*

<a id="igps-destructor-igps"></a>

### `IGps::~IGps`

```cpp
virtual ~IGps() = default
```

Re-declared only because the virtual destructor suppresses the implicit copy/move members; this seam holds no state of its own, so defaulting them is harmless. The virtual destructor is what lets an owner delete an implementation through IGps*.

*function, declared at [`include/shulib/hal/gps.hpp:29`](../../include/shulib/hal/gps.hpp#L29).*

<a id="igps-igps"></a>

### `IGps::IGps`

```cpp
IGps() = default
```

*Covered by the comment on [`~IGps`](#igps-destructor-igps) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/gps.hpp:30`](../../include/shulib/hal/gps.hpp#L30).*

<a id="igps-igps-2"></a>

### `IGps::IGps (overload 2)`

```cpp
IGps(const IGps&) = default
```

*Covered by the comment on [`~IGps`](#igps-destructor-igps) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/gps.hpp:31`](../../include/shulib/hal/gps.hpp#L31).*

<a id="igps-igps-3"></a>

### `IGps::IGps (overload 3)`

```cpp
IGps(IGps&&) = default
```

*Covered by the comment on [`~IGps`](#igps-destructor-igps) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/gps.hpp:32`](../../include/shulib/hal/gps.hpp#L32).*

<a id="igps-operator-eq"></a>

### `IGps::operator=`

```cpp
IGps& operator=(const IGps&) = default
```

*Covered by the comment on [`~IGps`](#igps-destructor-igps) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/gps.hpp:33`](../../include/shulib/hal/gps.hpp#L33).*

<a id="igps-operator-eq-2"></a>

### `IGps::operator= (overload 2)`

```cpp
IGps& operator=(IGps&&) = default
```

*Covered by the comment on [`~IGps`](#igps-destructor-igps) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/gps.hpp:34`](../../include/shulib/hal/gps.hpp#L34).*

<a id="igps-pose"></a>

### `IGps::pose`

```cpp
[[nodiscard]] virtual math::Pose2d pose() const = 0
```

Canonical robot-CENTER pose (position + heading), lever-arm and frame corrected. When hasFix()==false the value is UNSPECIFIED but MUST be finite (no NaN/Inf) and MUST NOT throw — read it only as a stale last-known estimate; the fuser ignores it and dead-reckons (§13 #4). Callers MUST check hasFix() before trusting pose().

*function, declared at [`include/shulib/hal/gps.hpp:40`](../../include/shulib/hal/gps.hpp#L40).*

<a id="igps-rmserror"></a>

### `IGps::rmsError`

```cpp
[[nodiscard]] virtual units::Length rmsError() const = 0
```

RMS position error (canonical Length) — drives the corrector's R; large when off-strip. CONTRACT, stated because pose() states its own and this one was left to convention: the value MUST be finite (no NaN/Inf), MUST NOT be negative, and MUST NOT throw. When hasFix() is false it is UNSPECIFIED but still bound by both rules — holding the last good reading (what the PROS adapter does) and reporting a large "no information" figure are both conforming. Two places had already had to invent this rule independently, and GpsCorrector keeps its own finiteness-and-sign backstop anyway, because a backstop exists for the implementation that gets it wrong.

*function, declared at [`include/shulib/hal/gps.hpp:50`](../../include/shulib/hal/gps.hpp#L50).*

<a id="igps-hasfix"></a>

### `IGps::hasFix`

```cpp
[[nodiscard]] virtual bool hasFix() const = 0
```

True when the GPS currently has a usable fix (on the strip, error bounded, connected). False → the estimator must dead-reckon (ignore the GPS this tick).

*function, declared at [`include/shulib/hal/gps.hpp:54`](../../include/shulib/hal/gps.hpp#L54).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 9 lines</summary>

```text

 IGps — the VEX GPS behind the HAL, reporting in shulib's CANONICAL frame (the VEX
 meters / clockwise-from-North convention is converted away in the hal/pros adapter
 via gps_conversion.hpp). pose() is the robot-CENTER pose (lever-arm corrected).

 rmsError() and hasFix() drive the fusion (§13 #4): the GPS feeds a gated nudge,
 never a snap; and Driving Skills has NO GPS strip, so when the sensor is off the
 strip (or error is high / disconnected) hasFix() is false and the estimator runs
 dead-reckon only. rmsError() sets the corrector's measurement noise R.
```

</details>
