<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/kinematics/desaturate.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `desaturate.hpp`

desaturateUniform — the downstream wheel-speed safety scale.

This header declares **1** free function.

Extracted from [`include/shulib/kinematics/desaturate.hpp`](../../include/shulib/kinematics/desaturate.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`desaturateUniform`](#desaturateuniform) — *free function*

<a id="desaturateuniform"></a>

## `desaturateUniform`

```cpp
[[nodiscard]] inline WheelSpeeds desaturateUniform(const WheelSpeeds& wheels, units::Velocity maxWheelSpeed)
```

Scale EVERY wheel by ONE common factor so the largest just reaches `maxWheelSpeed`. Uniform scaling preserves the ratios between wheels — and therefore the DIRECTION of the commanded motion — trading only speed for feasibility. A set already inside the budget (the all-zero set included) is returned unchanged: this never scales UP, so it cannot be used to reach a speed floor. The result has the same size() as the input, and both the input and the limit are canonical velocity. Precondition: maxWheelSpeed > 0 — a zero or negative limit is a caller bug, not a request to hold still.

*free function, declared at [`include/shulib/kinematics/desaturate.hpp:29`](../../include/shulib/kinematics/desaturate.hpp#L29).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 13 lines</summary>

```text

 desaturateUniform — the downstream wheel-speed safety scale (master plan §5
 data-flow, §13 #5).

 If any wheel exceeds `maxWheelSpeed` in magnitude, scale EVERY wheel by the
 same factor so the largest just reaches the limit. Scaling uniformly preserves
 the ratios between wheels — and therefore the *direction* of the commanded
 motion — trading only speed for feasibility. If the command already fits within
 budget it is returned unchanged (never scaled UP).

 This is deliberately separate from strafeAuthority() clamping (§13 #5): that
 shapes the command upstream; this is the last-line guarantee that no wheel is
 ever asked for more than it can give. toWheels() itself stays clamp-free.
```

</details>
