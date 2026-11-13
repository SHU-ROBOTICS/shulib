<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/drive_modes.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drive_modes.hpp`

Drive modes — the pure mappings from a driver's four stick axes to ONE body-frame DriveRequest.

This header declares **1** type (3 members) and **4** free functions.

Extracted from [`include/shulib/teleop/drive_modes.hpp`](../../include/shulib/teleop/drive_modes.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class DriveMode`](#enum-class-drivemode)
  - [`ArcadeBody`](#drivemode-arcadebody)
  - [`ArcadeField`](#drivemode-arcadefield)
  - [`TankSticks`](#drivemode-tanksticks)
- [`arcadeRequest`](#arcaderequest) — *free function*
- [`tankRequest`](#tankrequest) — *free function*
- [`tankRequest (overload 2)`](#tankrequest-2) — *free function*
- [`modeRequest`](#moderequest) — *free function*

<a id="enum-class-drivemode"></a>

## `enum class DriveMode`

```cpp
enum class DriveMode : std::uint8_t
```

Which way the driver's sticks are wired to the drivetrain. Selected by a DriverProfile (input_shaping.hpp), applied by DriveShaper (driver_control.hpp).  The underlying type is fixed so the value can be logged, sent over a wire or stored in a profile without depending on the compiler's choice.

*enum class, declared at [`include/shulib/teleop/drive_modes.hpp:51`](../../include/shulib/teleop/drive_modes.hpp#L51).*

<a id="drivemode-arcadebody"></a>

### `DriveMode::ArcadeBody`

```cpp
ArcadeBody
```

One stick translates, the other yaws, in the ROBOT's frame — the mapping the library has shipped since R1a and the default. "Forward" means the way the robot is facing.

*enumerator, declared at [`include/shulib/teleop/drive_modes.hpp:54`](../../include/shulib/teleop/drive_modes.hpp#L54).*

<a id="drivemode-arcadefield"></a>

### `DriveMode::ArcadeField`

```cpp
ArcadeField
```

ArcadeBody's mapping with the TRANSLATION rotated out of the driver's frame into the robot's, so "away from me" means away from the DRIVER whatever way the robot faces. Needs a trusted heading and has an honest fallback when it does not have one (field_centric.hpp). The rotation is applied by DriveShaper, not by modeRequest().

*enumerator, declared at [`include/shulib/teleop/drive_modes.hpp:59`](../../include/shulib/teleop/drive_modes.hpp#L59).*

<a id="drivemode-tanksticks"></a>

### `DriveMode::TankSticks`

```cpp
TankSticks
```

Left stick Y drives the LEFT side, right stick Y drives the RIGHT side — the classic tank feel, mapped into the same DriveRequest (see the file header). No strafe.

*enumerator, declared at [`include/shulib/teleop/drive_modes.hpp:62`](../../include/shulib/teleop/drive_modes.hpp#L62).*

<a id="arcaderequest"></a>

## `arcadeRequest`

```cpp
[[nodiscard]] constexpr DriveRequest arcadeRequest(const StickInput& in) noexcept
```

Today's mapping, as a function of the sticks alone: forward = leftY, left = -leftX, yawCcw = -rightX, and the all-zero request when the controller is disconnected (HA-103).  UNSHAPED — no deadband, no curve (the file header says why the order matters). This is the arithmetic `mapSticks()` performs around its deadband, and the two agree exactly wherever the deadband is not engaged.

*free function, declared at [`include/shulib/teleop/drive_modes.hpp:71`](../../include/shulib/teleop/drive_modes.hpp#L71).*

<a id="tankrequest"></a>

## `tankRequest`

```cpp
[[nodiscard]] constexpr DriveRequest tankRequest(double leftSide, double rightSide) noexcept
```

Two side commands in [-1, 1] -> the equivalent DriveRequest: forward = (left + right)/2, yawCcw = (right - left)/2, left(strafe) = 0.  This is the EXACT inverse of tankSideVolts()'s arcade arithmetic — feed the result back through it and the two sides come out unchanged (`tankSideVolts(tankRequest(l, r), V)` == `{V*l, V*r}`). The signs follow from the body frame, not from taste: a CCW (left) turn is a positive yaw, and it is the RIGHT side running faster that turns a tank left.  Exactness is arithmetic, not luck, on inputs that are exact binary fractions (the halving and the recombination are then exact operations); on arbitrary doubles the round trip is correct to the last ulp or two rather than bit-perfect, which is pinned both ways in test/drive_modes_test.cpp.

*free function, declared at [`include/shulib/teleop/drive_modes.hpp:90`](../../include/shulib/teleop/drive_modes.hpp#L90).*

<a id="tankrequest-2"></a>

## `tankRequest (overload 2)`

```cpp
[[nodiscard]] constexpr DriveRequest tankRequest(const StickInput& in) noexcept
```

Tank sticks -> the same DriveRequest: left stick Y is the LEFT side, right stick Y is the RIGHT side, and a disconnected controller yields the all-zero request whatever the axes read (HA-103). `leftX` is ignored — a tank drivetrain cannot strafe, and quietly accepting a strafe request it can never honour would be a lie the kinematics would have to clean up.

*free function, declared at [`include/shulib/teleop/drive_modes.hpp:98`](../../include/shulib/teleop/drive_modes.hpp#L98).*

<a id="moderequest"></a>

## `modeRequest`

```cpp
[[nodiscard]] constexpr DriveRequest modeRequest(DriveMode mode, const StickInput& in) noexcept
```

Dispatch: the mode's mapping, still UNSHAPED and still in the robot's frame.  `ArcadeField` maps identically to `ArcadeBody` here on purpose — field-centric is a ROTATION of this result, applied by DriveShaper after shaping and slew, so that the fallback stays one branch and the rotation happens in the driver's frame rather than the limiter's (driver_control.hpp, ruling S5).

*free function, declared at [`include/shulib/teleop/drive_modes.hpp:111`](../../include/shulib/teleop/drive_modes.hpp#L111).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 37 lines</summary>

```text

 Drive modes — the pure mappings from a driver's four stick axes to ONE
 body-frame DriveRequest (chunk T2).

 THE LOAD-BEARING DECISION IS THAT THERE IS STILL ONE REQUEST TYPE. Tank mode
 is the obvious place to grow a second representation ("left side, right
 side"), and the legacy tree died of exactly that: C6 catalogued THREE data
 representations of one command vocabulary and zero executors. So tank sticks
 are mapped INTO stick_mapping.hpp's DriveRequest — forward = (l + r)/2,
 yawCcw = (r - l)/2, left = 0 — which is the exact algebraic inverse of the
 arcade arithmetic coupled_side_monitor.hpp's tankSideVolts() already performs.
 A tank driver's left stick therefore reaches the left side of the drivetrain
 through the SAME request the library's closed-loop pipeline consumes, and the
 round trip is exact (pinned in test/drive_modes_test.cpp).

 THESE FUNCTIONS DO NOT SHAPE. They are step one of DriveShaper::shape()'s
 documented order (mode -> shaping -> slew -> rotation, driver_control.hpp), so
 the axis signs are applied here and the deadband/curve is applied afterwards.
 That order is what preserves the sign of zero: the raw axis is NEGATED FIRST
 and the deadband applied SECOND, exactly as stick_mapping.hpp pins for the
 R1a mapping. Read alone, `arcadeRequest()` on a centred stick returns -0.0 on
 the negated axes; read in the pipeline, the deadband turns that into +0.0.

 AXIS CONVENTION is stick_mapping.hpp's, unchanged (locked frame F1, body
 frame, CCW-positive): left stick UP = +forward, left stick LEFT = +left
 (hal's LeftX is + when pushed RIGHT, so it is negated), right stick RIGHT =
 NEGATIVE yaw. A disconnected controller yields the all-zero request in EVERY
 mode, whatever the axes read (HA-103).

 FIELD-CENTRIC IS NOT A MAPPING, IT IS A ROTATION of the mapping's output, and
 it needs state (the driver's zero and the fallback latch). It lives in
 field_centric.hpp; `DriveMode::ArcadeField` names it here so a DriverProfile
 can select it, and `modeRequest()` treats it exactly as ArcadeBody — the
 rotation is applied later, by DriveShaper.

 PURE, PROS-free, constexpr, allocation-free, total (no preconditions):
 finiteness becomes a precondition where it always did, at Chassis::drive.
```

</details>
