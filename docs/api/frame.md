<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/math/frame.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `frame.hpp`

frame.hpp — THE ONE PLACE a frame rotation is allowed.

This header declares **1** type (2 members) and **2** free functions.

Extracted from [`include/shulib/math/frame.hpp`](../../include/shulib/math/frame.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class Frame`](#enum-class-frame)
  - [`Field`](#frame-field)
  - [`Body`](#frame-body)
- [`fieldToRobot`](#fieldtorobot) — *free function*
- [`robotToField`](#robottofield) — *free function*

<a id="enum-class-frame"></a>

## `enum class Frame`

```cpp
enum class Frame : std::uint8_t
```

The two frames a chassis VELOCITY command can be expressed in — F1's vocabulary, surfaced as a type at the API edge (chunk C4, for `Chassis::drive(ChassisSpeeds, Frame)`). The caller must SAY which frame a command is in; the parameter has no default anywhere, so silently assuming the wrong frame — the classic bug class this rebuild exists to prevent — is a compile error, not a heading-dependent drift.  ADDITIVE to this locked file: new vocabulary only. The F1 conventions and the two rotations below are untouched.

*enum class, declared at [`include/shulib/math/frame.hpp:36`](../../include/shulib/math/frame.hpp#L36).*

<a id="frame-field"></a>

### `Frame::Field`

```cpp
Field
```

+X right, +Y away from red, heading-independent (F1's world frame)

*enumerator, declared at [`include/shulib/math/frame.hpp:37`](../../include/shulib/math/frame.hpp#L37).*

<a id="frame-body"></a>

### `Frame::Body`

```cpp
Body
```

+X forward, +Y left — rotates with the robot (F1's robot frame)

*enumerator, declared at [`include/shulib/math/frame.hpp:38`](../../include/shulib/math/frame.hpp#L38).*

<a id="fieldtorobot"></a>

## `fieldToRobot`

```cpp
[[nodiscard]] inline ChassisSpeeds fieldToRobot(const ChassisSpeeds& field, Angle heading) noexcept
```

Express a FIELD-frame chassis velocity in the ROBOT (body) frame. Applies R(-θ).

*free function, declared at [`include/shulib/math/frame.hpp:42`](../../include/shulib/math/frame.hpp#L42).*

<a id="robottofield"></a>

## `robotToField`

```cpp
[[nodiscard]] inline ChassisSpeeds robotToField(const ChassisSpeeds& body, Angle heading) noexcept
```

Express a ROBOT (body) frame chassis velocity back in the FIELD frame. Applies R(+θ).

*free function, declared at [`include/shulib/math/frame.hpp:53`](../../include/shulib/math/frame.hpp#L53).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 15 lines</summary>

```text

 frame.hpp — THE ONE PLACE a frame rotation is allowed (master plan §7 / Freeze F1).

 fieldToRobot / robotToField rotate a planar velocity between the FIELD frame
 and the ROBOT (body) frame by the robot's heading. The rotation rate ω is
 frame-invariant and passes through untouched.

 Convention (LOCKED, +X / CCW):
   FIELD:  +X right, +Y away from the red station, heading 0 along +X, CCW-positive.
   BODY:   +X forward, +Y left (same chirality as the field).
   fieldToRobot applies R(-θ); robotToField applies R(+θ); they are inverses.

 Because `heading` is an Angle (radians internally), a degree value can never
 reach cos/sin here — the old "field-centric overwrite + degrees-into-trig"
 bug class is structurally impossible.
```

</details>
