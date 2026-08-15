<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/controller_conversion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `controller_conversion.hpp`

Controller canonical conversions — the ONE place the V5 controller's raw stick range becomes shulib's canonical [-1, 1] (§7: "convert exactly once, at the edge").

This header declares **1** free function.

Extracted from [`include/shulib/hal/controller_conversion.hpp`](../../include/shulib/hal/controller_conversion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`controllerAxisToCanonical`](#controlleraxistocanonical) — *free function*

<a id="controlleraxistocanonical"></a>

## `controllerAxisToCanonical`

```cpp
[[nodiscard]] inline double controllerAxisToCanonical(double raw)
```

Raw analog reading (get_analog(), [-127, 127], HA-103) → canonical [-1, 1]. ÷127 so full deflection is exactly ±1; clamped so the contract holds even against an out-of-range raw value. Drop the ÷127 and every stick input saturates the speed budget at the slightest touch — full-speed lurch on a 1-count wiggle.

*free function, declared at [`include/shulib/hal/controller_conversion.hpp:31`](../../include/shulib/hal/controller_conversion.hpp#L31).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 16 lines</summary>

```text

 Controller canonical conversions — the ONE place the V5 controller's raw
 stick range becomes shulib's canonical [-1, 1] (§7: "convert exactly once,
 at the edge"). Pure, PROS-free, host- and mutation-testable in isolation;
 the hal/pros IController adapter is thin glue that CALLS this.

 V5 controller convention (vendored pros/misc.hpp:85: "The current reading of
 the analog channel: [-127, 127]. If the controller was not connected, then 0
 is returned" — HA-103). shulib canonical: [-1, 1], full deflection = ±1.

 Note the asymmetry this hides: −127…+127 is symmetric, so ÷127 maps both
 rails exactly to ±1. A hypothetical −128 (int8 min, never documented to
 occur) would map to −1.008 — the clamp below makes the canonical contract
 ("axis() is in [-1, 1]") unconditionally true rather than true-if-the-doc-
 is-right. Clamping is NOT deadband: deadband/curves/slew are driver-feel
 POLICY and belong to the teleop layer (chunk T2), never to a conversion.
```

</details>
