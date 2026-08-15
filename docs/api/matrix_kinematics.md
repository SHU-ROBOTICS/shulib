<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/kinematics/matrix_kinematics.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `matrix_kinematics.hpp`

MatrixKinematics — the coefficient-matrix engine for FULLY-HOLONOMIC LINEAR drives (the hybrid backend, §13 #15).

This header declares **2** types (9 members).

Extracted from [`include/shulib/kinematics/matrix_kinematics.hpp`](../../include/shulib/kinematics/matrix_kinematics.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class MatrixKinematics`](#class-matrixkinematics)
  - [`MatrixKinematics`](#matrixkinematics-matrixkinematics)
  - [`toWheels`](#matrixkinematics-towheels)
  - [`forward`](#matrixkinematics-forward)
  - [`desaturate`](#matrixkinematics-desaturate)
  - [`strafeAuthority`](#matrixkinematics-strafeauthority)
  - [`wheelCount`](#matrixkinematics-wheelcount)
  - [`struct MatrixKinematics::Wheel`](#struct-matrixkinematics-wheel)
    - [`h`](#matrixkinematics-wheel-h)
    - [`v`](#matrixkinematics-wheel-v)
    - [`turnInches`](#matrixkinematics-wheel-turninches)

<a id="class-matrixkinematics"></a>

## `class MatrixKinematics`

```cpp
class MatrixKinematics final : public IKinematics
```

Every FULLY-HOLONOMIC LINEAR drive — X, H, mecanum — as ONE implementation: the geometry is pure data, one [h, v, turnInches] row per wheel, so a new drivetrain is a table and not a subclass. toWheels() is that table applied row by row; forward() is the full least-squares pseudo-inverse (AᵀA)⁻¹Aᵀ, inverted once at construction so a call costs two small multiplies. Rank-3 is REQUIRED and checked: tank cannot strafe, so one of its columns is all-zero and construction rejects it by design (tank belongs in TankKinematics), as does any table whose columns are near-dependent. Immutable once built — every method is const, and nothing here allocates. Capping is ONE method's job: toWheels() deliberately returns over-budget wheel speeds (§13 #5), which is what keeps forward() its exact inverse, and desaturate() is the only place a commanded speed is reduced.

*class, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:82`](../../include/shulib/kinematics/matrix_kinematics.hpp#L82).*

<a id="matrixkinematics-matrixkinematics"></a>

### `MatrixKinematics::MatrixKinematics`

```cpp
MatrixKinematics(std::initializer_list<Wheel> wheels, double strafeAuthority)
```

Build from a per-wheel coefficient table + the drive's strafe authority. Preconditions (all red-on-failure): 1..kMaxWheels wheels; strafeAuthority ≥ 0; the table is genuinely rank-3 (each column non-degenerate AND the columns jointly well-conditioned — relDet > kMinRelativeDeterminant, header note). Orthogonal columns are NO LONGER required (C3's pseudo-inverse); they remain the well-trodden fast path.

*function, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:98`](../../include/shulib/kinematics/matrix_kinematics.hpp#L98).*

<a id="matrixkinematics-towheels"></a>

### `MatrixKinematics::toWheels`

```cpp
[[nodiscard]] WheelSpeeds toWheels(const math::ChassisSpeeds& body) const override
```

Inverse kinematics, one row at a time: wheel_i = h_i·vx + v_i·vy + turnInches_i·ω, in in/s. `body` is a BODY-frame command — the single field→body rotation belongs to Chassis, never here. The result has wheelCount() entries, in the table's row order. It CLAMPS NOTHING: ask for more than the drive can deliver and you get wheel speeds that say so, which is exactly what keeps forward() an exact inverse of the command. desaturate() is the downstream cap.

*function, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:178`](../../include/shulib/kinematics/matrix_kinematics.hpp#L178).*

<a id="matrixkinematics-forward"></a>

### `MatrixKinematics::forward`

```cpp
[[nodiscard]] math::Twist2d forward(const WheelSpeeds& wheels) const override
```

Forward kinematics for odometry: per-wheel surface speeds (in/s) → BODY-frame twist, as the least-squares solution t = (AᵀA)⁻¹Aᵀw. For a square full-rank table (the 3-wheel H-drive) that is exactly A⁻¹w; for a redundant one it is the unique minimizer of ‖A·t − w‖, so wheels that disagree are averaged rather than one being believed. Orthogonal tables take the historical per-column projection instead, bit for bit, so no previously-accepted drive's numbers moved. Precondition: wheels.size() == wheelCount().

*function, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:196`](../../include/shulib/kinematics/matrix_kinematics.hpp#L196).*

<a id="matrixkinematics-desaturate"></a>

### `MatrixKinematics::desaturate`

```cpp
[[nodiscard]] WheelSpeeds desaturate(const WheelSpeeds& wheels, units::Velocity maxWheelSpeed) const override
```

Scale EVERY wheel by one common factor until the largest magnitude just reaches `maxWheelSpeed`, so the commanded direction survives and only speed is traded away. A command already within budget (all-zero included) is returned unchanged — this never scales UP. Uniform scaling is the right answer for a linear drive precisely because the table is linear; swerve overrides this to preserve module angles. Precondition: maxWheelSpeed > 0.

*function, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:230`](../../include/shulib/kinematics/matrix_kinematics.hpp#L230).*

<a id="matrixkinematics-strafeauthority"></a>

### `MatrixKinematics::strafeAuthority`

```cpp
[[nodiscard]] double strafeAuthority() const override
```

The constructor's `strafeAuthority` argument, returned verbatim: the sustainable |body vy| as a fraction of the linear speed budget, for the MOTION layer to clamp against. This class neither derives it from the coefficient table nor clamps anything with it — it is a read-only query. 1.0 for the symmetric X-drive; ≈0.35 for the H-drive, which measures it.

*function, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:239`](../../include/shulib/kinematics/matrix_kinematics.hpp#L239).*

<a id="matrixkinematics-wheelcount"></a>

### `MatrixKinematics::wheelCount`

```cpp
[[nodiscard]] int wheelCount() const override
```

Rows in the coefficient table: the number of entries every WheelSpeeds this object produces will have, and the number forward() requires. Fixed at construction, in [1, kMaxWheels].

*function, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:242`](../../include/shulib/kinematics/matrix_kinematics.hpp#L242).*

<a id="struct-matrixkinematics-wheel"></a>

## `struct MatrixKinematics::Wheel`

```cpp
struct Wheel
```

One wheel's contribution row. h, v are dimensionless; turnInches is the yaw lever arm in inches (signed). See the header formula.

*struct, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:86`](../../include/shulib/kinematics/matrix_kinematics.hpp#L86).*

<a id="matrixkinematics-wheel-h"></a>

### `MatrixKinematics::Wheel::h`

```cpp
double h
```

multiplies vx (body +X, forward); a dimensionless projection factor

*field, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:87`](../../include/shulib/kinematics/matrix_kinematics.hpp#L87).*

<a id="matrixkinematics-wheel-v"></a>

### `MatrixKinematics::Wheel::v`

```cpp
double v
```

multiplies vy (body +Y, left/strafe); dimensionless, like h

*field, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:88`](../../include/shulib/kinematics/matrix_kinematics.hpp#L88).*

<a id="matrixkinematics-wheel-turninches"></a>

### `MatrixKinematics::Wheel::turnInches`

```cpp
double turnInches
```

yaw lever arm in INCHES, signed; multiplies ω (rad/s → in/s)

*field, declared at [`include/shulib/kinematics/matrix_kinematics.hpp:89`](../../include/shulib/kinematics/matrix_kinematics.hpp#L89).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 55 lines, click to expand</summary>

```text

 MatrixKinematics — the coefficient-matrix engine for FULLY-HOLONOMIC LINEAR
 drives (the hybrid backend, §13 #15). A drivetrain becomes pure data: each
 wheel is a row [h, v, turnInches], and

     wheel_i surface speed = h_i·vx + v_i·vy + turnInches_i·ω

 where the last term is the ONE sanctioned radian-drop (ω[rad/s]·lever[in] →
 in/s). h and v are dimensionless projection factors; turnInches is the wheel's
 yaw lever arm in inches.

 forward() (wheels → body twist, for odometry) is the FULL LEAST-SQUARES
 pseudo-inverse  t = (AᵀA)⁻¹Aᵀ·w  (chunk C3, discharging the M1 deferral so the
 H-drive's OFF-CENTRE strafe wheel — a non-orthogonal column — is supported).
 Because AᵀA is 3×3 symmetric it is inverted once, in closed form, at
 construction; forward() is then two small matrix multiplies per call.

 ── The strict-generalization guarantee (the C3 no-regression contract) ─────────────
 When the columns are mutually orthogonal (X-drive, symmetric mecanum — every
 table this class accepted before C3), AᵀA is diagonal and the pseudo-inverse
 REDUCES to the historical per-column projection

     vx = (Σ h_i w_i)/Σh²,  vy = (Σ v_i w_i)/Σv²,  ω = (Σ turn_i w_i)/Σturn²

 forward() detects that case ONCE at construction — using the EXACT predicate
 the pre-C3 precondition used to accept tables — and runs the historical
 computation VERBATIM for it, so every previously-accepted drive is
 BIT-IDENTICAL to its pre-C3 numbers (pinned by XOR-of-bit-pattern checksums in
 test/matrix_kinematics_test.cpp, captured from the pre-C3 build). The general
 path serves only tables the old code REJECTED: a relaxed precondition, nothing
 more (F5-safe — signatures, toWheels(), desaturate(), strafeAuthority()
 untouched; decision-checked 2026-06-19, discharged 2026-08-06).

 ── Conditioning guard (the silent-garbage defence) ─────────────────────────────────
 Once orthogonality is no longer required, the per-column rank check below is
 NOT sufficient: three individually-nonzero columns can still be linearly
 dependent (e.g. two parallel wheel directions), making AᵀA singular — and a
 NEAR-degenerate table would pass any exact-singularity test yet amplify wheel
 noise by an unbounded factor in forward(). Construction therefore computes the
 RELATIVE GRAM DETERMINANT

     relDet = det(AᵀA) / (Σh²·Σv²·Σturn²)   ∈ [0, 1]   (Hadamard's inequality;
              1 ⟺ orthogonal columns, 0 ⟺ rank-deficient)

 and REJECTS relDet ≤ kMinRelativeDeterminant with a red-on-failure
 precondition rather than silently mis-inverting — the worst possible failure
 mode here is plausible-looking garbage odometry. relDet is scale-free (column
 units cancel), so a huge turn-lever column cannot mask a genuine geometric
 degeneracy. See kMinRelativeDeterminant below for the threshold's derivation.

 Tank is NOT a MatrixKinematics: it is rank-2 (cannot strafe), so a column is
 all-zero and the rank precondition correctly rejects it. Tank lives in its own
 dedicated TankKinematics. (This is unchanged by the pseudo-inverse: rank-3 is
 still required — the generalization admits non-ORTHOGONAL tables, never
 rank-DEFICIENT ones.)
```

</details>
