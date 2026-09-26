<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/stick_mapping.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `stick_mapping.hpp`

Teleop stick mapping — the ONE place a driver's sticks become a body-frame drive request (chunk R3b, Session 2), and the DriveRequest/StickInput vocabulary the whole feel layer is written in.

This header declares **2** types (8 members), **4** free functions, and **1** constant.

Extracted from [`include/shulib/teleop/stick_mapping.hpp`](../../include/shulib/teleop/stick_mapping.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kStickDeadband`](#kstickdeadband) — *constant*
- [`struct StickInput`](#struct-stickinput)
  - [`leftY`](#stickinput-lefty)
  - [`leftX`](#stickinput-leftx)
  - [`rightX`](#stickinput-rightx)
  - [`rightY`](#stickinput-righty)
  - [`connected`](#stickinput-connected)
- [`struct DriveRequest`](#struct-driverequest)
  - [`forward`](#driverequest-forward)
  - [`left`](#driverequest-left)
  - [`yawCcw`](#driverequest-yawccw)
- [`deadbanded`](#deadbanded) — *free function*
- [`mapSticks`](#mapsticks) — *free function*
- [`toChassisSpeeds`](#tochassisspeeds) — *free function*
- [`mapSticksToChassisSpeeds`](#mapstickstochassisspeeds) — *free function*

<a id="kstickdeadband"></a>

## `kStickDeadband`

```cpp
inline constexpr double kStickDeadband = 0.05
```

The stick deadband, as a fraction of full deflection (canonical [-1, 1] axes). INVENTED (A4: HA-112) — it exists only so a centred stick's few counts of noise cannot creep the robot. Applied here as a hard cut (see deadbanded()); T2's continuousDeadband() (input_shaping.hpp) uses the same number as its default and rescales instead of cutting.

*constant, declared at [`include/shulib/teleop/stick_mapping.hpp:69`](../../include/shulib/teleop/stick_mapping.hpp#L69).*

<a id="struct-stickinput"></a>

## `struct StickInput`

```cpp
struct StickInput
```

What the driver's hands are doing, in the form the mapping needs: the three canonical [-1, 1] axes it reads (IController::axis for LeftY / LeftX / RightX) and the controller's POSITIVE validity signal (IController::isConnected). Built by the caller from an IController each tick; a struct rather than an IController& so the mapping stays a pure function of numbers and is testable without a fake controller.

*struct, declared at [`include/shulib/teleop/stick_mapping.hpp:76`](../../include/shulib/teleop/stick_mapping.hpp#L76).*

<a id="stickinput-lefty"></a>

### `StickInput::leftY`

```cpp
double leftY = 0.0
```

Left stick, vertical: + = pushed UP (hal::ControllerAxis::LeftY).

*field, declared at [`include/shulib/teleop/stick_mapping.hpp:77`](../../include/shulib/teleop/stick_mapping.hpp#L77).*

<a id="stickinput-leftx"></a>

### `StickInput::leftX`

```cpp
double leftX = 0.0
```

Left stick, horizontal: + = pushed RIGHT (hal::ControllerAxis::LeftX).

*field, declared at [`include/shulib/teleop/stick_mapping.hpp:78`](../../include/shulib/teleop/stick_mapping.hpp#L78).*

<a id="stickinput-rightx"></a>

### `StickInput::rightX`

```cpp
double rightX = 0.0
```

Right stick, horizontal: + = pushed RIGHT (hal::ControllerAxis::RightX).

*field, declared at [`include/shulib/teleop/stick_mapping.hpp:79`](../../include/shulib/teleop/stick_mapping.hpp#L79).*

<a id="stickinput-righty"></a>

### `StickInput::rightY`

```cpp
double rightY = 0.0
```

Right stick, vertical: + = pushed UP (hal::ControllerAxis::RightY). ADDED at T2 for tank-stick mode (drive_modes.hpp), where it is the RIGHT side's command. `mapSticks()` and every function below IGNORE it, so the R1a bit-identity pin is untouched: a field appended to an aggregate with a default changes no existing caller and no existing value.

*field, declared at [`include/shulib/teleop/stick_mapping.hpp:84`](../../include/shulib/teleop/stick_mapping.hpp#L84).*

<a id="stickinput-connected"></a>

### `StickInput::connected`

```cpp
bool connected = false
```

IController::isConnected() — false forces a zero request.

*field, declared at [`include/shulib/teleop/stick_mapping.hpp:85`](../../include/shulib/teleop/stick_mapping.hpp#L85).*

<a id="struct-driverequest"></a>

## `struct DriveRequest`

```cpp
struct DriveRequest
```

The mapped drive request, DIMENSIONLESS: body-frame forward, left and CCW-yaw fractions in [-1, 1] with the deadband and the axis signs already applied, all exactly 0.0 while the controller is disconnected. Multiply by a speed budget (toChassisSpeeds) for the library teleop loop, or by a voltage ceiling for an open-loop bench drive — the same request feeds both, which is the point of having one.

*struct, declared at [`include/shulib/teleop/stick_mapping.hpp:93`](../../include/shulib/teleop/stick_mapping.hpp#L93).*

<a id="driverequest-forward"></a>

### `DriveRequest::forward`

```cpp
double forward = 0.0
```

+X body fraction: left stick UP is positive.

*field, declared at [`include/shulib/teleop/stick_mapping.hpp:94`](../../include/shulib/teleop/stick_mapping.hpp#L94).*

<a id="driverequest-left"></a>

### `DriveRequest::left`

```cpp
double left = 0.0
```

+Y body fraction: left stick pushed LEFT is positive.

*field, declared at [`include/shulib/teleop/stick_mapping.hpp:95`](../../include/shulib/teleop/stick_mapping.hpp#L95).*

<a id="driverequest-yawccw"></a>

### `DriveRequest::yawCcw`

```cpp
double yawCcw = 0.0
```

CCW-positive yaw fraction: right stick pushed RIGHT is NEGATIVE (clockwise).

*field, declared at [`include/shulib/teleop/stick_mapping.hpp:96`](../../include/shulib/teleop/stick_mapping.hpp#L96).*

<a id="deadbanded"></a>

## `deadbanded`

```cpp
[[nodiscard]] constexpr double deadbanded(double axis) noexcept
```

The R1a deadband, verbatim: an axis strictly inside (−kStickDeadband, +kStickDeadband) becomes exactly 0.0; anything else — the threshold value itself included — passes through untouched. Discontinuous at the threshold by construction.  SUPERSEDED by continuousDeadband() (input_shaping.hpp) and kept deliberately: this body is pinned bit for bit against the original R1a loop, and the drive program and the bench tester still call it. Do not "improve" it — see the file header.

*free function, declared at [`include/shulib/teleop/stick_mapping.hpp:106`](../../include/shulib/teleop/stick_mapping.hpp#L106).*

<a id="mapsticks"></a>

## `mapSticks`

```cpp
[[nodiscard]] constexpr DriveRequest mapSticks(const StickInput& in) noexcept
```

Sticks → dimensionless request. A disconnected controller yields the all-zero request no matter what the axes read (HA-103). Otherwise: forward = deadbanded(leftY), left = deadbanded(−leftX), yawCcw = deadbanded(−rightX) — NEGATE FIRST, then deadband, exactly the original loop's `shaped(-axis)` (header: the order decides the sign of zero).

*free function, declared at [`include/shulib/teleop/stick_mapping.hpp:114`](../../include/shulib/teleop/stick_mapping.hpp#L114).*

<a id="tochassisspeeds"></a>

## `toChassisSpeeds`

```cpp
[[nodiscard]] constexpr math::ChassisSpeeds toChassisSpeeds(const DriveRequest& request, units::Velocity maxLinear, units::AngularVelocity maxAngular) noexcept
```

Request × budgets → the body-frame ChassisSpeeds that Chassis::drive(…, Frame::Body) takes: (forward·maxLinear, left·maxLinear, yawCcw·maxAngular). The budgets are the caller's MotionConfig::maxLinearSpeed / maxAngularSpeed (HA-50, provisional) — passed in rather than read here so this stays a pure function of its arguments.

*free function, declared at [`include/shulib/teleop/stick_mapping.hpp:125`](../../include/shulib/teleop/stick_mapping.hpp#L125).*

<a id="mapstickstochassisspeeds"></a>

## `mapSticksToChassisSpeeds`

```cpp
[[nodiscard]] constexpr math::ChassisSpeeds mapSticksToChassisSpeeds( const StickInput& in, units::Velocity maxLinear, units::AngularVelocity maxAngular) noexcept
```

The whole mapping in one call — what src/main.cpp's R1a loop (`runTeleopLoop`, now called only by the invented X-drive graph) uses each tick. "shulib Teleop" used it until G1 Part 2b moved it onto the library's own loop, `teleop::runDriverControl` (teleop_loop.hpp), which shapes through `DriveShaper` and `toChassisSpeeds` instead; this function stays for the X-drive loop until G1 Part 2c and for its bit-identity pin. Bit-identical to the R1a loop it replaced: a disconnected controller returns a default-constructed ChassisSpeeds (never 0·budget, so the answer does not depend on the budgets' values), and a connected one returns toChassisSpeeds(mapSticks(in), …).

*free function, declared at [`include/shulib/teleop/stick_mapping.hpp:141`](../../include/shulib/teleop/stick_mapping.hpp#L141).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 57 lines, click to expand</summary>

```text

 Teleop stick mapping — the ONE place a driver's sticks become a body-frame
 drive request (chunk R3b, Session 2), and the DriveRequest/StickInput
 vocabulary the whole feel layer is written in.

 T2 HAS LANDED, AND IT DID NOT REPLACE THIS FILE. The feel layer — a
 continuous deadband, monotonic expo, optional slew, drive modes,
 field-centric with its honest fallback — is built ALONGSIDE the two functions
 below, in input_shaping.hpp / slew_limiter.hpp / drive_modes.hpp /
 field_centric.hpp / driver_control.hpp, all speaking the StickInput and
 DriveRequest declared here. The composed entry point is
 teleop::DriveShaper::shape() (driver_control.hpp).

   * `deadbanded()` is SUPERSEDED by `continuousDeadband()`
     (input_shaping.hpp), which rescales the surviving range instead of
     cutting it, so the output leaves zero smoothly. It is kept, unchanged,
     for two reasons that are not sentiment: test/stick_mapping_test.cpp pins
     it BIT FOR BIT against a verbatim copy of the R1a loop, and the drive
     program plus the bench tester's DRIVE station still call `mapSticks()`
     directly. Both callers move to `DriveShaper` when src/ is re-pointed;
     until then, "improving" either body below breaks a pin that exists to
     catch exactly that.
   * `mapSticks()` is superseded by `arcadeRequest()` + `shapeAxis()`
     (drive_modes.hpp, input_shaping.hpp). Same signs, same order of
     operations, same sign of zero — the shaped default profile differs from
     it only by the deadband rescale, and by nothing at all at 0 and at ±1.

 WHAT THE TWO FUNCTIONS BELOW ARE: the R1a loop's mapping, extracted VERBATIM
 from src/main.cpp and pinned bit-for-bit by test/stick_mapping_test.cpp
 against a copy of that original code. Deadband only, at 0.05 (HA-112:
 INVENTED — the deadband exists so a centred stick's ±2-count noise cannot
 creep the robot, and nothing about it is a measurement). That deadband is
 DISCONTINUOUS (output jumps from 0 to 0.05 as the stick crosses the
 threshold), which is what "twitchy, I can't place it" actually means, and it
 is left exactly as it was on purpose.

 NOTHING IN THE FEEL LAYER HAS BEEN DRIVEN. Every default it ships is invented
 and says so; no claim about how any of it feels is made anywhere.

 AXIS CONVENTION (locked frame F1, body frame, CCW-positive):
   * left stick pushed UP     → +forward  (+X body)
   * left stick pushed LEFT   → +left     (+Y body)   — hal's LeftX is + when pushed RIGHT,
                                                         so the raw axis is NEGATED first
   * right stick pushed RIGHT → NEGATIVE yaw (clockwise) — again a negation of RightX
   * controller not connected → all three exactly 0.0 (HA-103: a disconnected controller
                                 reads 0 on every channel, so isConnected() is the ONLY
                                 thing that separates "driver unplugged" from "sticks centred")

 ORDER OF OPERATIONS IS LOAD-BEARING for bit-identity: the raw axis is negated
 FIRST and the deadband applied SECOND, exactly as the R1a loop wrote
 `shaped(-axis)`. Deadband-then-negate gives the same VALUES but a different
 ZERO (−0.0 instead of +0.0 inside the deadband), and the test compares bits.

 PURE and PROS-free: takes numbers, returns numbers, no state, no clock, no
 preconditions — finiteness becomes a precondition where it always did, at
 Chassis::drive (chassis.hpp) and at the motor adapter. Host-tested in
 isolation; the CI guard keeps <pros/…> out of this tree.
```

</details>
