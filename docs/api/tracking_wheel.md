<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/tracking_wheel.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tracking_wheel.hpp`

TrackingWheel — one unpowered odometry wheel: an `IRotation` sensor + the wheel's diameter + its mounting offset from the tracking center.

This header declares **2** types (8 members).

Extracted from [`include/shulib/localization/tracking_wheel.hpp`](../../include/shulib/localization/tracking_wheel.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class TrackingWheel`](#class-trackingwheel)
  - [`forward`](#trackingwheel-forward)
  - [`lateral`](#trackingwheel-lateral)
  - [`travelDelta`](#trackingwheel-traveldelta)
  - [`offset`](#trackingwheel-offset)
  - [`role`](#trackingwheel-role)
  - [`reset`](#trackingwheel-reset)
  - [`enum class TrackingWheel::Role`](#enum-class-trackingwheel-role)
    - [`Forward`](#trackingwheel-role-forward)
    - [`Lateral`](#trackingwheel-role-lateral)

<a id="class-trackingwheel"></a>

## `class TrackingWheel`

```cpp
class TrackingWheel
```

One unpowered odometry wheel: a rotation sensor, the wheel's diameter, and its mounting offset from the tracking center. It turns cumulative SHAFT rotation into LINEAR travel — arc length = Δθ · r, in inches — and hands PilonsOdometry the two things it needs each tick: the travel since the last read, and the wheel's signed offset.  Build one only through forward() or lateral(). The factory stamps the ROLE, and the role is what fixes which axis the offset is measured along and lets PilonsOdometry reject wheels handed over in the wrong order; the constructor is private so that cannot be bypassed.  The sensor is held by NON-OWNING reference and must outlive the wheel. Nothing here screens readings: the HAL finiteness contract is trusted, and PilonsOdometry keeps the last-resort guard so a breach of that contract cannot poison the persistent pose.

*class, declared at [`include/shulib/localization/tracking_wheel.hpp:49`](../../include/shulib/localization/tracking_wheel.hpp#L49).*

<a id="trackingwheel-forward"></a>

### `TrackingWheel::forward`

```cpp
[[nodiscard]] static TrackingWheel forward(hal::IRotation& sensor, units::Length wheelDiameter, units::Length leftOffset)
```

A FORWARD-rolling wheel (+X body). `leftOffset` is its +Y (LEFT) coordinate from center.

*function, declared at [`include/shulib/localization/tracking_wheel.hpp:60`](../../include/shulib/localization/tracking_wheel.hpp#L60).*

<a id="trackingwheel-lateral"></a>

### `TrackingWheel::lateral`

```cpp
[[nodiscard]] static TrackingWheel lateral(hal::IRotation& sensor, units::Length wheelDiameter, units::Length forwardOffset)
```

A LATERAL-rolling wheel (+Y body). `forwardOffset` is its +X (FORWARD) coordinate.

*function, declared at [`include/shulib/localization/tracking_wheel.hpp:66`](../../include/shulib/localization/tracking_wheel.hpp#L66).*

<a id="trackingwheel-traveldelta"></a>

### `TrackingWheel::travelDelta`

```cpp
[[nodiscard]] units::Length travelDelta()
```

Linear travel (inches) since the previous call. STATEFUL: it advances the baseline, so successive calls return successive deltas, never a cumulative total.

*function, declared at [`include/shulib/localization/tracking_wheel.hpp:73`](../../include/shulib/localization/tracking_wheel.hpp#L73).*

<a id="trackingwheel-offset"></a>

### `TrackingWheel::offset`

```cpp
[[nodiscard]] units::Length offset() const noexcept
```

The wheel's signed offset from the tracking center (perpendicular to its rolling axis).

*function, declared at [`include/shulib/localization/tracking_wheel.hpp:81`](../../include/shulib/localization/tracking_wheel.hpp#L81).*

<a id="trackingwheel-role"></a>

### `TrackingWheel::role`

```cpp
[[nodiscard]] Role role() const noexcept
```

Whether this wheel rolls forward (+X body) or laterally (+Y body) — checked by PilonsOdometry.

*function, declared at [`include/shulib/localization/tracking_wheel.hpp:84`](../../include/shulib/localization/tracking_wheel.hpp#L84).*

<a id="trackingwheel-reset"></a>

### `TrackingWheel::reset`

```cpp
void reset()
```

Resync the baseline to the current reading, so the next travelDelta() starts from zero (used when odometry is (re)initialized, so a pre-existing shaft total isn't counted).

*function, declared at [`include/shulib/localization/tracking_wheel.hpp:88`](../../include/shulib/localization/tracking_wheel.hpp#L88).*

<a id="enum-class-trackingwheel-role"></a>

## `enum class TrackingWheel::Role`

```cpp
enum class Role
```

Which body axis a wheel rolls along, and therefore which axis its offset is measured on. Stamped by the factory, never chosen by the caller; PilonsOdometry preconditions on it so the forward and the lateral wheel cannot be passed in the wrong order.

*enum class, declared at [`include/shulib/localization/tracking_wheel.hpp:54`](../../include/shulib/localization/tracking_wheel.hpp#L54).*

<a id="trackingwheel-role-forward"></a>

### `TrackingWheel::Role::Forward`

```cpp
Forward
```

rolls along body +X; its offset is the +Y (LEFT) coordinate of the wheel

*enumerator, declared at [`include/shulib/localization/tracking_wheel.hpp:55`](../../include/shulib/localization/tracking_wheel.hpp#L55).*

<a id="trackingwheel-role-lateral"></a>

### `TrackingWheel::Role::Lateral`

```cpp
Lateral
```

rolls along body +Y; its offset is the +X (FORWARD) coordinate

*enumerator, declared at [`include/shulib/localization/tracking_wheel.hpp:56`](../../include/shulib/localization/tracking_wheel.hpp#L56).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 28 lines</summary>

```text

 TrackingWheel — one unpowered odometry wheel: an `IRotation` sensor + the wheel's diameter
 + its mounting offset from the tracking center. It turns cumulative shaft rotation into
 LINEAR travel and hands `PilonsOdometry` the two things it needs per tick: the travel delta
 since the last read, and the wheel's signed offset. (The PROS-free analogue of the legacy
 `OdomUnit`, driven by the HAL so it is host-testable with `FakeRotation`.)

 Travel = shaft angle (radians) × wheel radius. `IRotation::position()` is CUMULATIVE and
 unwrapped (it is `AngleDim`, not the wrapping `Angle`), so the running difference is the true
 signed distance the wheel has rolled — including direction reversals — with no seam to handle.
 (Binding contract: the hal/pros adapter MUST source this from the cumulative reading, e.g.
 `pros::Rotation::get_position`, NOT a wrapping 0–360 angle — the same get_rotation-vs-get_heading
 distinction the IMU has. The int32 centidegree range is ~6×10⁴ revolutions ≈ miles of travel,
 far beyond a match, so no wrap is seen in practice. See master plan §7. A4 register HA-11;
 the wheel's MEASURED geometry — offset, sign, effective diameter — is HA-12/HA-13.)

 ROLE + OFFSET. The offset's reference axis DIFFERS by role, so a wheel is built through a named
 factory that stamps the role and documents the axis — you cannot set the wrong axis or pass the
 wheels to PilonsOdometry in the wrong order (it checks the role):
   * forward(): a FORWARD-rolling wheel (+X body); its offset is the +Y (LEFT) coordinate.
   * lateral(): a LATERAL-rolling wheel (+Y body); its offset is the +X (FORWARD) coordinate.
 This is the perpendicular-to-rolling distance — the only offset that matters, because rolling
 along the wheel's own axis is what a turn-in-place sweeps. (Derivation + signs live in
 PilonsOdometry; verified by /tmp/odom_oracle.py and the pure-rotation tests.)

 Finiteness: TrackingWheel does no sentinel screening — it trusts the HAL finiteness contract
 (§7: the hal/pros adapter clamps PROS_ERR/NaN at the edge, so the core never sees non-finite).
 PilonsOdometry adds a last-resort guard so a contract breach can't poison the persistent pose.
```

</details>
