<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/kinematics/x_drive.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `x_drive.hpp`

xDrive() — the symmetric 45° X-drive, as a MatrixKinematics preset (the hybrid backend §13 #15: a holonomic linear drive is just a coefficient table).

This header declares **1** free function.

Extracted from [`include/shulib/kinematics/x_drive.hpp`](../../include/shulib/kinematics/x_drive.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`xDrive`](#xdrive) — *free function*

<a id="xdrive"></a>

## `xDrive`

```cpp
[[nodiscard]] inline MatrixKinematics xDrive(units::Length driveRadius)
```

The symmetric 45° X-drive as a MatrixKinematics coefficient table: four omnis in the canonical order front-left, back-left, back-right, front-right (body angles 45°, 135°, 225°, 315° CCW from +X forward). `driveRadius` is the centre-to-wheel distance in INCHES and must be > 0; it is the only geometry input, because every row is [±√2/2, ±√2/2, driveRadius]. Using the SAME √2/2 magnitude on all four wheels makes the coefficient columns exactly orthogonal, so forward() is an exact inverse rather than a least-squares fit. Consequences worth knowing at the call site: strafeAuthority is 1.0 (strafe is symmetric with forward), and a forward command at V asks each wheel for only V/√2. Returned BY VALUE — the caller owns it, and it must outlive every IKinematics reference taken to it. Motor→index mapping and per-motor polarity are the HAL/config layer's; this fixes only the canonical order.

*free function, declared at [`include/shulib/kinematics/x_drive.hpp:52`](../../include/shulib/kinematics/x_drive.hpp#L52).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 31 lines</summary>

```text

 xDrive() — the symmetric 45° X-drive, as a MatrixKinematics preset (the hybrid
 backend §13 #15: a holonomic linear drive is just a coefficient table).

 Geometry — body frame per F1 (frame.hpp): +X = forward, +Y = left, ω CCW-positive.
 Canonical wheel order, by body angle φ measured CCW from +X (forward):
   wheel 0: front-left  (position angle  45°)
   wheel 1: back-left   (             135°)
   wheel 2: back-right  (             225°)
   wheel 3: front-right (             315°)

 Each omni's powered-roll direction is tangential, d̂ = (-sinφ, cosφ). The rigorous
 projection  wheel_i = d̂_i·(v_body + ω×r_i)  yields the row  [-sinφ, cosφ, R]  =
 [±c, ±c, R]  with c = √2/2 and R = driveRadius (center-to-wheel distance), where the
 first column weights vx (forward) and the second vy (left). Using the SAME magnitude
 c on every wheel makes the coefficient columns exactly orthogonal, so
 MatrixKinematics::forward() is an exact inverse.

 Pinned-by-test consequences of this geometry:
   * Forward translation at V needs wheel surface speed V/√2 — i.e. the body
     moves √2× faster forward than the wheels spin (the classic X-drive property).
   * Pure rotation at ω drives EVERY wheel at R·ω with the same sign (all-equal =
     spin in place — the X-drive signature).
   * Strafe (+Y) is symmetric with forward (+X), so strafeAuthority() = 1.0.

 The physical motor→index mapping and per-motor polarity live in the HAL/config
 layer; kinematics only defines this canonical order.

 That the BUILT robot matches this idealized geometry (true 45° symmetric mounts,
 equal radii) is an A4-registered assumption until a physical drivetrain exists:
 A4 register HA-17 (docs/hardware-assumptions.md); R3/R5 settle it.
```

</details>
