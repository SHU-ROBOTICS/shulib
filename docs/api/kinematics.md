<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/kinematics/kinematics.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `kinematics.hpp`

IKinematics — the drivetrain math contract.

This header declares **1** type (11 members).

Extracted from [`include/shulib/kinematics/kinematics.hpp`](../../include/shulib/kinematics/kinematics.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IKinematics`](#class-ikinematics)
  - [`~IKinematics`](#ikinematics-destructor-ikinematics)
  - [`IKinematics`](#ikinematics-ikinematics)
  - [`IKinematics (overload 2)`](#ikinematics-ikinematics-2)
  - [`IKinematics (overload 3)`](#ikinematics-ikinematics-3)
  - [`operator=`](#ikinematics-operator-eq)
  - [`operator= (overload 2)`](#ikinematics-operator-eq-2)
  - [`toWheels`](#ikinematics-towheels)
  - [`forward`](#ikinematics-forward)
  - [`desaturate`](#ikinematics-desaturate)
  - [`strafeAuthority`](#ikinematics-strafeauthority)
  - [`wheelCount`](#ikinematics-wheelcount)

<a id="class-ikinematics"></a>

## `class IKinematics`

```cpp
class IKinematics
```

The drivetrain math contract (Freeze F5): body-frame twist ⇄ per-wheel surface speeds, plus the desaturation hook and the strafeAuthority() query. This is the ONLY thing the motion layer knows about a drivetrain's geometry — it is frame-agnostic pure math, because the one FIELD→BODY rotation lives in Chassis (F1), and it limits nothing outside desaturate(). Speeds are in/s and twists in/s + rad/s throughout. The WHEEL ORDER is each implementation's to define and to document; toWheels() and forward() must agree on it. Two implementations ship: MatrixKinematics (the FULLY-HOLONOMIC linear drives — X, H, mecanum — from a per-wheel coefficient matrix) and TankKinematics, which is hand-written precisely BECAUSE tank cannot be a coefficient table: its strafe column is all-zero, so MatrixKinematics's full-rank precondition refuses it at construction. Swerve is the nonlinear case this interface exists to leave room for.

*class, declared at [`include/shulib/kinematics/kinematics.hpp:67`](../../include/shulib/kinematics/kinematics.hpp#L67).*

<a id="ikinematics-destructor-ikinematics"></a>

### `IKinematics::~IKinematics`

```cpp
virtual ~IKinematics() = default
```

Virtual destructor, so deleting a drivetrain through an `IKinematics*` would be well-defined — but nothing in this tree does that. Every consumer BORROWS a concrete implementation the caller keeps alive: motion holds `const IKinematics*`, sim's DrivePlant and SimHarness hold `const IKinematics&`. Declaring the destructor is also what suppresses the implicit copy/move re-defaulted just below.

*function, declared at [`include/shulib/kinematics/kinematics.hpp:74`](../../include/shulib/kinematics/kinematics.hpp#L74).*

<a id="ikinematics-ikinematics"></a>

### `IKinematics::IKinematics`

```cpp
IKinematics() = default
```

Default construction plus copy/move, re-defaulted because declaring the destructor above suppresses the implicit ones. The interface carries no state: an implementation owns its own geometry and is held by non-owning pointer or const reference, never copied into the motion layer.

*function, declared at [`include/shulib/kinematics/kinematics.hpp:80`](../../include/shulib/kinematics/kinematics.hpp#L80).*

<a id="ikinematics-ikinematics-2"></a>

### `IKinematics::IKinematics (overload 2)`

```cpp
IKinematics(const IKinematics&) = default
```

*Covered by the comment on [`IKinematics`](#ikinematics-ikinematics) — one comment documents this run of special members.*

*function, declared at [`include/shulib/kinematics/kinematics.hpp:81`](../../include/shulib/kinematics/kinematics.hpp#L81).*

<a id="ikinematics-ikinematics-3"></a>

### `IKinematics::IKinematics (overload 3)`

```cpp
IKinematics(IKinematics&&) = default
```

*Covered by the comment on [`IKinematics`](#ikinematics-ikinematics) — one comment documents this run of special members.*

*function, declared at [`include/shulib/kinematics/kinematics.hpp:82`](../../include/shulib/kinematics/kinematics.hpp#L82).*

<a id="ikinematics-operator-eq"></a>

### `IKinematics::operator=`

```cpp
IKinematics& operator=(const IKinematics&) = default
```

*Covered by the comment on [`IKinematics`](#ikinematics-ikinematics) — one comment documents this run of special members.*

*function, declared at [`include/shulib/kinematics/kinematics.hpp:83`](../../include/shulib/kinematics/kinematics.hpp#L83).*

<a id="ikinematics-operator-eq-2"></a>

### `IKinematics::operator= (overload 2)`

```cpp
IKinematics& operator=(IKinematics&&) = default
```

*Covered by the comment on [`IKinematics`](#ikinematics-ikinematics) — one comment documents this run of special members.*

*function, declared at [`include/shulib/kinematics/kinematics.hpp:84`](../../include/shulib/kinematics/kinematics.hpp#L84).*

<a id="ikinematics-towheels"></a>

### `IKinematics::toWheels`

```cpp
[[nodiscard]] virtual WheelSpeeds toWheels(const math::ChassisSpeeds& body) const = 0
```

Inverse kinematics: a BODY-frame commanded twist → per-wheel surface speeds. MUST NOT clamp or desaturate (§13 #5). size() of the result == wheelCount().

*function, declared at [`include/shulib/kinematics/kinematics.hpp:88`](../../include/shulib/kinematics/kinematics.hpp#L88).*

<a id="ikinematics-forward"></a>

### `IKinematics::forward`

```cpp
[[nodiscard]] virtual math::Twist2d forward(const WheelSpeeds& wheels) const = 0
```

Forward kinematics: per-wheel surface speeds → BODY-frame twist (for odometry). Precondition: wheels.size() == wheelCount().

*function, declared at [`include/shulib/kinematics/kinematics.hpp:92`](../../include/shulib/kinematics/kinematics.hpp#L92).*

<a id="ikinematics-desaturate"></a>

### `IKinematics::desaturate`

```cpp
[[nodiscard]] virtual WheelSpeeds desaturate(const WheelSpeeds& wheels, units::Velocity maxWheelSpeed) const = 0
```

Direction-preserving scale so every wheel fits within `maxWheelSpeed`. If the command is already within budget it is returned unchanged. Linear drives use a uniform scale (desaturateUniform); swerve overrides to keep module angles. Precondition: maxWheelSpeed > 0.

*function, declared at [`include/shulib/kinematics/kinematics.hpp:98`](../../include/shulib/kinematics/kinematics.hpp#L98).*

<a id="ikinematics-strafeauthority"></a>

### `IKinematics::strafeAuthority`

```cpp
[[nodiscard]] virtual double strafeAuthority() const = 0
```

PURE READ-ONLY query: the sustainable |body vy| as a fraction of the linear speed budget (see contract above — the C1-D11 semantic, confirmed at C3). Clamps nothing.

*function, declared at [`include/shulib/kinematics/kinematics.hpp:104`](../../include/shulib/kinematics/kinematics.hpp#L104).*

<a id="ikinematics-wheelcount"></a>

### `IKinematics::wheelCount`

```cpp
[[nodiscard]] virtual int wheelCount() const = 0
```

Number of kinematic wheels this drivetrain exposes (the size() of toWheels()).

*function, declared at [`include/shulib/kinematics/kinematics.hpp:107`](../../include/shulib/kinematics/kinematics.hpp#L107).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 47 lines, click to expand</summary>

```text

 IKinematics — the drivetrain math contract. **Freeze F5** (frozen at M1).

 Maps a body-frame chassis twist (vx, vy, ω) to per-wheel surface speeds and
 back, plus a desaturation hook and the strafeAuthority() query. This is the
 ONLY thing the motion layer knows about a drivetrain's geometry.

 CONTRACT (master plan §5 data-flow, §13 #5 & #15; do not break without a
 versioned migration — that is the F5 promise):

  * FRAME.  toWheels() takes a BODY-frame ChassisSpeeds. The single FIELD→BODY
    rotation lives in Chassis, never here (F1). Kinematics is frame-agnostic
    pure math.

  * NO CLAMPING in toWheels().  It is pure inverse kinematics. Saturation is
    handled in two SEPARATE places, on purpose (§13 #5):
      1. upstream — the motion layer reads strafeAuthority() and clamps the
         commanded vy *before* calling toWheels();
      2. downstream — desaturate() applies the final wheel-speed safety scale.
    toWheels() itself never limits anything, so odometry's forward() is an exact
    inverse of an *unclamped* command.

  * strafeAuthority() is a PURE READ-ONLY QUERY: the fraction of the drive's
    LINEAR SPEED BUDGET that is sustainable as body-frame lateral speed — the
    motion layer clamps |body vy| ≤ strafeAuthority()·maxLinearSpeed (C1's D11
    reading, CONFIRMED against the real H-drive at C3: the physical limit is
    the strafe wheel's own sustainable surface speed, an ABSOLUTE lateral cap
    independent of vx. The historical "|vy|/|vx| ratio" phrasing was ill-defined
    at vx = 0 — a pure strafe is legal on an H-drive — and would wrongly ADMIT
    MORE strafe at high vx than the wheel can deliver; it is retired.
    Doc-clarification only: no F5 signature or behaviour changed — both
    consumers, C1's clamp and C3's hDrive(), already implement this semantic.)
    XDrive = 1.0 (symmetric), HDrive ≈ 0.35 (sysid-measured default, not a
    hardcoded constant — h_drive.hpp derives it), Tank = 0.0 (cannot strafe).
    It computes and clamps nothing.

  * forward() is the inverse map (wheels → body twist) consumed by odometry. For
    the linear drives it is the closed-form left-inverse of toWheels(); for an
    achievable twist the round-trip forward(toWheels(t)) == t.

 THE HYBRID BACKEND (§13 #15, LOCKED 2026-06-19): this interface is the home for
 the nonlinear case (swerve — module angles a coefficient table can't express)
 and for the queries above. The FULLY-HOLONOMIC *linear* drives (X / H /
 mecanum) are a single implementation, MatrixKinematics, driven by a per-wheel
 [h, v, turn] coefficient matrix. Tank is NOT among them: it is rank-2 (its
 strafe column is all-zero), so MatrixKinematics's full-rank precondition
 rejects it at construction and tank gets its own TankKinematics.
```

</details>
