<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/optical.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `optical.hpp`

ProsOptical — IOptical over pros::Optical (chunk R1b): the game-object color/proximity confirmation sensor behind the HAL.

This header declares **1** type (6 members).

Extracted from [`include/shulib/hal/pros/optical.hpp`](../../include/shulib/hal/pros/optical.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsOptical`](#class-prosoptical)
  - [`ProsOptical`](#prosoptical-prosoptical)
  - [`hue`](#prosoptical-hue)
  - [`saturation`](#prosoptical-saturation)
  - [`brightness`](#prosoptical-brightness)
  - [`proximity`](#prosoptical-proximity)
  - [`faultedReads`](#prosoptical-faultedreads)

<a id="class-prosoptical"></a>

## `class ProsOptical`

```cpp
class ProsOptical final : public IOptical
```

IOptical over pros::Optical — the game-object colour/proximity confirm sensor behind the HAL. Every reader SCREENS the device's failure sentinel and returns the last good value instead of the failure: never INFINITY, which would break the F4 finiteness contract downstream, and never a 0 written OVER a good reading, because hue 0.0 IS a colour (red) — a zeroed failure would answer "what colour is this game piece" confidently and wrongly. The cache itself STARTS at 0.0, so a sensor that has never once read successfully (unplugged at boot) does serve hue() == 0.0 on every call; faultedReads(), which counts every screen, is the only channel that separates that from a sensor staring at a steady red object, because this seam has no confidence() to say so (T7). The readers are const but update that held-value cache, so const here does not mean safe to poll from two tasks at once. Raising a fault is the loop layer's job, not this seam's.

*class, declared at [`include/shulib/hal/pros/optical.hpp:60`](../../include/shulib/hal/pros/optical.hpp#L60).*

<a id="prosoptical-prosoptical"></a>

### `ProsOptical::ProsOptical`

```cpp
explicit ProsOptical(std::uint8_t port)
```

`port`: 1..21.

*function, declared at [`include/shulib/hal/pros/optical.hpp:63`](../../include/shulib/hal/pros/optical.hpp#L63).*

<a id="prosoptical-hue"></a>

### `ProsOptical::hue`

```cpp
[[nodiscard]] double hue() const override
```

Color hue [0, 360) — sentinel-screened to last good (T7).

*function, declared at [`include/shulib/hal/pros/optical.hpp:66`](../../include/shulib/hal/pros/optical.hpp#L66).*

<a id="prosoptical-saturation"></a>

### `ProsOptical::saturation`

```cpp
[[nodiscard]] double saturation() const override
```

Colour purity in [0, 1]: how strongly coloured the reading is, independent of how bright. A low saturation means hue() is describing something near grey and is not a colour call worth acting on. Sentinel-screened to last good (T7); 0.0 until the first successful read.

*function, declared at [`include/shulib/hal/pros/optical.hpp:79`](../../include/shulib/hal/pros/optical.hpp#L79).*

<a id="prosoptical-brightness"></a>

### `ProsOptical::brightness`

```cpp
[[nodiscard]] double brightness() const override
```

The light level the sensor sees, in [0, 1] — a brightness, not a distance; proximity() is the distance channel. Sentinel-screened to last good (T7); 0.0 until the first good read.

*function, declared at [`include/shulib/hal/pros/optical.hpp:91`](../../include/shulib/hal/pros/optical.hpp#L91).*

<a id="prosoptical-proximity"></a>

### `ProsOptical::proximity`

```cpp
[[nodiscard]] double proximity() const override
```

[0, 1], ≈1 = close (HA-117's unmeasured polarity — header note).

*function, declared at [`include/shulib/hal/pros/optical.hpp:102`](../../include/shulib/hal/pros/optical.hpp#L102).*

<a id="prosoptical-faultedreads"></a>

### `ProsOptical::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many reads were screened to last-good (T7 observability).

*function, declared at [`include/shulib/hal/pros/optical.hpp:113`](../../include/shulib/hal/pros/optical.hpp#L113).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 31 lines</summary>

```text

 ProsOptical — IOptical over pros::Optical (chunk R1b): the game-object
 color/proximity confirmation sensor behind the HAL.

 BINDS:
  * get_hue()        [double 0–359.999; HA-116] → hue()
    (opticalHueToCanonical — clamp only; a COLOR, never a heading)
  * get_saturation() [double 0–1.0; HA-116] → saturation()
  * get_brightness() [double 0–1.0; HA-116] → brightness()
    (both via opticalUnitIntervalToCanonical)
  * get_proximity()  [int32 0–255; HA-117] → proximity()
    (opticalProximityToCanonical — ÷255; the LARGER-IS-CLOSER polarity is a
    belief the vendored doc does NOT state; HA-117 flags it and the bench
    measures it before any capture threshold trusts proximity())

 SENTINELS (T7): IOptical has NO validity channel — unlike IDistance there
 is no confidence() to absorb a failure. The double channels return
 PROS_ERR_F (INFINITY) and proximity returns PROS_ERR on device failure
 (HA-118): each reader screens its own sentinel, holds its last good value,
 and counts it in faultedReads(). Never propagate (INFINITY through hue()
 breaks the F4 finiteness contract), never zero (hue 0.0 IS a color — red —
 so a zeroed failure would read as a confident wrong answer to "what color
 is this game piece"). Raising a fault stays with the loop layer (hal/ is
 below diag/).

 DELIBERATELY NOT here: set_led_pwm()/get_led_pwm(), gestures, raw RGBC —
 the seam carries color-confirm channels only; lighting policy is a
 mechanism decision (which owns knowing whether ITS sensor needs the LED),
 and gestures have no consumer anywhere in the plan.

 HA register: HA-116, HA-117, HA-118 (docs/hardware-assumptions.md).
```

</details>
