<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/kinematics/tank.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tank.hpp`

TankKinematics — a 2-wheel differential (skid-steer) drive.

This header declares **1** type (6 members).

Extracted from [`include/shulib/kinematics/tank.hpp`](../../include/shulib/kinematics/tank.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class TankKinematics`](#class-tankkinematics)
  - [`TankKinematics`](#tankkinematics-tankkinematics)
  - [`toWheels`](#tankkinematics-towheels)
  - [`forward`](#tankkinematics-forward)
  - [`desaturate`](#tankkinematics-desaturate)
  - [`strafeAuthority`](#tankkinematics-strafeauthority)
  - [`wheelCount`](#tankkinematics-wheelcount)

<a id="class-tankkinematics"></a>

## `class TankKinematics`

```cpp
class TankKinematics final : public IKinematics
```

A 2-wheel differential (skid-steer) drive. Canonical wheel order is 0 = LEFT, 1 = RIGHT, in F1's body frame (+X forward, +Y left, ω CCW-positive). Deliberately a hand-written IKinematics rather than a MatrixKinematics preset: tank's strafe column is all-zero, so it is rank-2 and the holonomic engine's full-rank precondition correctly refuses it. Pure geometry — no state, no clock, no HAL; the only thing it knows is the track width.

*class, declared at [`include/shulib/kinematics/tank.hpp:32`](../../include/shulib/kinematics/tank.hpp#L32).*

<a id="tankkinematics-tankkinematics"></a>

### `TankKinematics::TankKinematics`

```cpp
explicit TankKinematics(units::Length trackWidth)
```

trackWidth = lateral distance between the left and right wheel contact lines.

*function, declared at [`include/shulib/kinematics/tank.hpp:35`](../../include/shulib/kinematics/tank.hpp#L35).*

<a id="tankkinematics-towheels"></a>

### `TankKinematics::toWheels`

```cpp
[[nodiscard]] WheelSpeeds toWheels(const math::ChassisSpeeds& body) const override
```

Inverse kinematics: left = vx − ω·halfTrack, right = vx + ω·halfTrack, both in in/s, from a BODY-frame twist. The commanded vy is SILENTLY IGNORED — not clamped, not an error — because a tank cannot strafe and strafeAuthority() == 0 is how the motion layer is meant to have zeroed it already. Nothing here limits speed (§13 #5); that is desaturate()'s job.

*function, declared at [`include/shulib/kinematics/tank.hpp:43`](../../include/shulib/kinematics/tank.hpp#L43).*

<a id="tankkinematics-forward"></a>

### `TankKinematics::forward`

```cpp
[[nodiscard]] math::Twist2d forward(const WheelSpeeds& wheels) const override
```

Forward kinematics for odometry: vx = (left + right)/2, ω = (right − left)/(2·halfTrack), and vy ALWAYS exactly 0 — this drivetrain cannot observe lateral motion, so a real skid sideways is reported as no motion at all. Exact left-inverse of toWheels() on the achievable (vx, ω) subspace, so forward(toWheels(t)) returns t only when t's vy was already 0. Precondition: exactly 2 wheels, in the canonical left-then-right order.

*function, declared at [`include/shulib/kinematics/tank.hpp:57`](../../include/shulib/kinematics/tank.hpp#L57).*

<a id="tankkinematics-desaturate"></a>

### `TankKinematics::desaturate`

```cpp
[[nodiscard]] WheelSpeeds desaturate(const WheelSpeeds& wheels, units::Velocity maxWheelSpeed) const override
```

The uniform scale (desaturateUniform): if either wheel is over `maxWheelSpeed`, BOTH are scaled by the same factor, which preserves left:right and therefore the arc the command describes — the robot follows the same curve, more slowly. Returned unchanged when already within budget; never scales up. Precondition: maxWheelSpeed > 0.

*function, declared at [`include/shulib/kinematics/tank.hpp:70`](../../include/shulib/kinematics/tank.hpp#L70).*

<a id="tankkinematics-strafeauthority"></a>

### `TankKinematics::strafeAuthority`

```cpp
[[nodiscard]] double strafeAuthority() const override
```

Always 0: no lateral authority whatever, so the motion layer's |body vy| ≤ strafeAuthority()·maxLinearSpeed clamp reduces to "vy must be 0" here.

*function, declared at [`include/shulib/kinematics/tank.hpp:77`](../../include/shulib/kinematics/tank.hpp#L77).*

<a id="tankkinematics-wheelcount"></a>

### `TankKinematics::wheelCount`

```cpp
[[nodiscard]] int wheelCount() const override
```

Always 2 — the left and right KINEMATIC wheels. How many physical motors sit on each side is the HAL's business; kinematics sees one speed per side.

*function, declared at [`include/shulib/kinematics/tank.hpp:81`](../../include/shulib/kinematics/tank.hpp#L81).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 15 lines</summary>

```text

 TankKinematics — a 2-wheel differential (skid-steer) drive. Body frame per F1
 (frame.hpp): +X = FORWARD, +Y = left, ω CCW-positive. The wheels drive along +X
 (forward); a commanded vy (strafe) is physically unachievable and is ignored —
 strafeAuthority() = 0 (= max sustainable |vy|/|vx|, §13 #5).

 This is a DEDICATED impl, NOT a MatrixKinematics preset: tank is rank-2 (its
 strafe column is all-zero), so the holonomic engine's full-rank precondition
 correctly excludes it. The closed form (halfTrack = trackWidth/2):

   left  = vx − ω·halfTrack          right = vx + ω·halfTrack
   vx = (left + right)/2             ω    = (right − left)/(2·halfTrack)

 ω·halfTrack is the sanctioned radian-drop (rad/s·in → in/s); the inverse
 re-attaches it. Canonical wheel order: 0 = left, 1 = right.
```

</details>
