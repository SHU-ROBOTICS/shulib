<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/field_centric.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `field_centric.hpp`

Field-centric driving, and the honest failure mode without which it must not ship.

This header declares **4** types (16 members).

Extracted from [`include/shulib/teleop/field_centric.hpp`](../../include/shulib/teleop/field_centric.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class FieldCentricReason`](#enum-class-fieldcentricreason)
  - [`Active`](#fieldcentricreason-active)
  - [`HeadingNotTrusted`](#fieldcentricreason-headingnottrusted)
  - [`LatchedUntilRezero`](#fieldcentricreason-latcheduntilrezero)
  - [`NotSelected`](#fieldcentricreason-notselected)
- [`struct FieldCentricState`](#struct-fieldcentricstate)
  - [`active`](#fieldcentricstate-active)
  - [`fellBack`](#fieldcentricstate-fellback)
  - [`reason`](#fieldcentricstate-reason)
- [`class FieldCentric`](#class-fieldcentric)
  - [`FieldCentric`](#fieldcentric-fieldcentric)
  - [`rezero`](#fieldcentric-rezero)
  - [`zeroRadians`](#fieldcentric-zeroradians)
  - [`fellBack`](#fieldcentric-fellback)
  - [`reset`](#fieldcentric-reset)
  - [`request`](#fieldcentric-request)
  - [`state`](#fieldcentric-state)
  - [`struct FieldCentric::Result`](#struct-fieldcentric-result)
    - [`request`](#fieldcentric-result-request)
    - [`state`](#fieldcentric-result-state)

<a id="enum-class-fieldcentricreason"></a>

## `enum class FieldCentricReason`

```cpp
enum class FieldCentricReason : std::uint8_t
```

Why field-centric is or is not rotating the driver's sticks right now — an enum rather than a string because this runs on the V5 and a teleop tick allocates nothing.

*enum class, declared at [`include/shulib/teleop/field_centric.hpp:64`](../../include/shulib/teleop/field_centric.hpp#L64).*

<a id="fieldcentricreason-active"></a>

### `FieldCentricReason::Active`

```cpp
Active
```

Rotating normally: the heading is trusted and the driver has a zero.

*enumerator, declared at [`include/shulib/teleop/field_centric.hpp:66`](../../include/shulib/teleop/field_centric.hpp#L66).*

<a id="fieldcentricreason-headingnottrusted"></a>

### `FieldCentricReason::HeadingNotTrusted`

```cpp
HeadingNotTrusted
```

The caller reported the heading as not trustworthy THIS tick (the boot window before the estimate is live, a latched IMU_LOST fault, or a heading that is not even a finite number). The request was passed through unrotated.

*enumerator, declared at [`include/shulib/teleop/field_centric.hpp:70`](../../include/shulib/teleop/field_centric.hpp#L70).*

<a id="fieldcentricreason-latcheduntilrezero"></a>

### `FieldCentricReason::LatchedUntilRezero`

```cpp
LatchedUntilRezero
```

The heading is trustworthy again, but the fallback has LATCHED: the controls stay robot-centric until the driver re-zeroes. This is the state a panel most needs to show, because it is the one where nothing looks wrong.

*enumerator, declared at [`include/shulib/teleop/field_centric.hpp:74`](../../include/shulib/teleop/field_centric.hpp#L74).*

<a id="fieldcentricreason-notselected"></a>

### `FieldCentricReason::NotSelected`

```cpp
NotSelected
```

Field-centric is not selected — the profile's mode is a body-frame one. Reported so a panel can distinguish "off" from "fell back", which look identical at the drivetrain.

*enumerator, declared at [`include/shulib/teleop/field_centric.hpp:77`](../../include/shulib/teleop/field_centric.hpp#L77).*

<a id="struct-fieldcentricstate"></a>

## `struct FieldCentricState`

```cpp
struct FieldCentricState
```

What a panel shows about field-centric — the third of the three things this mode may not ship without (a documented fallback, a driver re-zero, and a VISIBLE INDICATION). Three plain values, cheap to copy every tick.

*struct, declared at [`include/shulib/teleop/field_centric.hpp:83`](../../include/shulib/teleop/field_centric.hpp#L83).*

<a id="fieldcentricstate-active"></a>

### `FieldCentricState::active`

```cpp
bool active = false
```

True only when the translation was actually rotated this tick.

*field, declared at [`include/shulib/teleop/field_centric.hpp:85`](../../include/shulib/teleop/field_centric.hpp#L85).*

<a id="fieldcentricstate-fellback"></a>

### `FieldCentricState::fellBack`

```cpp
bool fellBack = false
```

True while the fallback latch is set — the controls are robot-centric and will stay that way until `rezero()`. Distinct from `!active`, which is also true when the mode simply is not selected.

*field, declared at [`include/shulib/teleop/field_centric.hpp:89`](../../include/shulib/teleop/field_centric.hpp#L89).*

<a id="fieldcentricstate-reason"></a>

### `FieldCentricState::reason`

```cpp
FieldCentricReason reason = FieldCentricReason::NotSelected
```

Which of the four situations above produced this tick's behaviour.

*field, declared at [`include/shulib/teleop/field_centric.hpp:91`](../../include/shulib/teleop/field_centric.hpp#L91).*

<a id="class-fieldcentric"></a>

## `class FieldCentric`

```cpp
class FieldCentric
```

The rotation and the latch: the driver's zero offset, and whether the fallback is engaged.  Two doubles and a bool of state. One instance per driver (VEX U runs two), held by the DriveShaper that owns that driver's feel.

*class, declared at [`include/shulib/teleop/field_centric.hpp:98`](../../include/shulib/teleop/field_centric.hpp#L98).*

<a id="fieldcentric-fieldcentric"></a>

### `FieldCentric::FieldCentric`

```cpp
constexpr FieldCentric() noexcept = default
```

A field-centric mapper with zero at heading 0 and the fallback latch CLEARED. The latch starts clear on purpose: it is a record of something that went wrong, and nothing has gone wrong yet. The first untrusted tick sets it.

*function, declared at [`include/shulib/teleop/field_centric.hpp:103`](../../include/shulib/teleop/field_centric.hpp#L103).*

<a id="fieldcentric-rezero"></a>

### `FieldCentric::rezero`

```cpp
constexpr void rezero(double headingRadians) noexcept
```

"The way I am facing now is away from me" — the driver's button. Records `headingRadians` as the new zero and CLEARS the fallback latch, which is the only thing that clears it.  It does NOT check whether that heading is trustworthy, and it takes no trust argument: the driver is asserting a frame, and the very next `request()` re-latches the fallback if the heading is still untrusted. Refusing the press outright would leave the driver pressing a dead button with no way to tell why; re-latching one tick later says the same thing through the indication that is already on the panel.  Nothing is tared and no estimate moves (HA-05) — the zero lives here.  ONE EXCEPTION (G1 Part 2a, found while wiring this to a button): a heading that is NOT A FINITE NUMBER is not stored. It would become a NaN zero, and every later trusted tick would rotate by `heading − NaN` — a NaN request that `Chassis::drive` rejects by precondition, long after the press, with nothing on the panel to explain it. Instead the previous zero is kept and the fallback latch is SET, so the controls stay robot-centric and the panel says `LatchedUntilRezero` until the driver presses again on a real heading. (`request()` already treats a non-finite heading as untrusted; this closes the one path that let such a heading outlive its tick.)

*function, declared at [`include/shulib/teleop/field_centric.hpp:124`](../../include/shulib/teleop/field_centric.hpp#L124).*

<a id="fieldcentric-zeroradians"></a>

### `FieldCentric::zeroRadians`

```cpp
[[nodiscard]] constexpr double zeroRadians() const noexcept
```

The driver's zero, in radians, as last set by `rezero()` (0 until then).

*function, declared at [`include/shulib/teleop/field_centric.hpp:136`](../../include/shulib/teleop/field_centric.hpp#L136).*

<a id="fieldcentric-fellback"></a>

### `FieldCentric::fellBack`

```cpp
[[nodiscard]] constexpr bool fellBack() const noexcept
```

True while the fallback latch is engaged. Cleared only by `rezero()`.

*function, declared at [`include/shulib/teleop/field_centric.hpp:139`](../../include/shulib/teleop/field_centric.hpp#L139).*

<a id="fieldcentric-reset"></a>

### `FieldCentric::reset`

```cpp
constexpr void reset() noexcept
```

Forget everything: zero back to 0 and the latch cleared. For a program handing control between drivers or restarting teleop, not for anything a match does.

*function, declared at [`include/shulib/teleop/field_centric.hpp:143`](../../include/shulib/teleop/field_centric.hpp#L143).*

<a id="fieldcentric-request"></a>

### `FieldCentric::request`

```cpp
[[nodiscard]] Result request(const DriveRequest& shapedBody, double headingRadians, bool headingTrusted) noexcept
```

Rotate a SHAPED body-frame request out of the driver's frame into the robot's, or fall back honestly.  `shapedBody` is the request as the mode mapping and the shaping produced it: its `forward`/`left` pair is read as the driver's stick vector in the DRIVER's frame, and `yawCcw` is frame-invariant and is copied through untouched.  `headingRadians` is the robot's ESTIMATED heading, CCW-positive, in the same frame the driver's zero was captured in. `headingTrusted` is the caller's verdict on it — the estimate is live (past the boot window) and no IMU_LOST fault is latched.  Rotates by MINUS `(heading - zero)`: a robot turned CCW from the driver's zero must move CW in its own frame to keep going the way the driver pushed. This is the same R(-θ) the frozen `math::fieldToRobot` applies, re-derived here on plain doubles so that this file stays free of Angle's non-finite precondition and of the velocity types.  FALLS BACK when `headingTrusted` is false OR the heading is not a finite number: the request is returned unrotated, `fellBack` is set, and it LATCHES — subsequent trusted ticks keep falling back, reporting `LatchedUntilRezero`, until the driver re-zeroes. A non-finite heading is treated as untrusted rather than propagated because "trusted" is exactly the claim such a number fails; propagating it would turn the whole request into NaN and surface as a precondition abort inside `Chassis::drive` with nothing on the panel to explain it.

*function, declared at [`include/shulib/teleop/field_centric.hpp:180`](../../include/shulib/teleop/field_centric.hpp#L180).*

<a id="fieldcentric-state"></a>

### `FieldCentric::state`

```cpp
[[nodiscard]] constexpr FieldCentricState state() const noexcept
```

The indication with no tick run — what a panel shows when the mode is not selected, or between ticks. `active` is never true here: it is a property of a tick, not of the object.

*function, declared at [`include/shulib/teleop/field_centric.hpp:208`](../../include/shulib/teleop/field_centric.hpp#L208).*

<a id="struct-fieldcentric-result"></a>

## `struct FieldCentric::Result`

```cpp
struct Result
```

What `request()` returns: the body-frame request to command, and what to show about it.

*struct, declared at [`include/shulib/teleop/field_centric.hpp:149`](../../include/shulib/teleop/field_centric.hpp#L149).*

<a id="fieldcentric-result-request"></a>

### `FieldCentric::Result::request`

```cpp
DriveRequest request{}
```

The request to hand to the drivetrain, always in the ROBOT's frame — rotated when field-centric is active, passed through untouched when it is not.

*field, declared at [`include/shulib/teleop/field_centric.hpp:152`](../../include/shulib/teleop/field_centric.hpp#L152).*

<a id="fieldcentric-result-state"></a>

### `FieldCentric::Result::state`

```cpp
FieldCentricState state{}
```

The indication (see FieldCentricState).

*field, declared at [`include/shulib/teleop/field_centric.hpp:154`](../../include/shulib/teleop/field_centric.hpp#L154).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 52 lines, click to expand</summary>

```text

 Field-centric driving, and the honest failure mode without which it must not
 ship (chunk T2).

 WHAT IT IS. In field-centric mode the driver's stick vector is read in the
 DRIVER's frame, not the robot's: "away from me" means away from the driver
 whatever way the robot happens to be facing. That is a rotation of the
 TRANSLATION by minus the robot's heading (measured from the driver's zero);
 yaw is frame-invariant and is never touched.

 WHY IT IS DANGEROUS, AND WHY THE FALLBACK IS THE POINT OF THE FILE. The
 rotation uses the ESTIMATED heading. A3 proved the heading drifts. So when
 heading quality degrades, the driver's controls silently rotate away from
 reality WHILE EVERYTHING STILL APPEARS TO WORK — the robot moves, the sticks
 respond, and the mapping between them is wrong by an angle nobody announced.
 Build-order T2: "do not ship field-centric without" a documented fallback to
 robot-centric, a driver-accessible re-zero, and a visible indication. All
 three are here.

 THE FALLBACK IS A LATCH, NOT A FLICKER. When the caller says the heading is
 not trusted, this returns the BODY request unrotated, reports `fellBack` with
 a reason a panel can show, and STAYS fallen back until the driver re-zeroes —
 even after trust returns. Controls that silently flip between two frames
 mid-match are worse than controls that stay in one: the driver can adapt to a
 robot that is robot-centric, and cannot adapt to a robot that changes its
 mind. Clearing the latch is therefore a DRIVER action, `rezero()`, and never
 an automatic one.

 NOTHING HERE TOUCHES THE IMU OR THE ESTIMATE (HA-05). The driver's zero is
 this object's own offset, subtracted from the heading it is handed. Re-zeroing
 by taring the IMU or resetting the localizer was the obvious alternative and
 is rejected outright: a tare under a live boot-heading offset silently
 corrupts the estimate, and a driver pressing a comfort button must not be able
 to break autonomous localization.

 THE HEADING IS AN ARGUMENT, NOT A READING. This class never sees a Chassis, a
 Localizer or an IImu — it takes a number and a bool. That keeps it testable
 without any of them, keeps the boot window out of its vocabulary (the caller
 already knows whether the estimate is live), and keeps this file out of the
 dependency graph of everything it would otherwise have to include.

 THE RESULT IS STILL A BODY-FRAME REQUEST, so the caller commands
 `Frame::Body`. Routing field-centric through `Chassis::drive(…, Frame::Field)`
 was the alternative and was rejected: the facade's field frame is correct, but
 going through it means the fallback needs the facade's cooperation, and the
 facade's boot-window rule silently commands zero volts. Rotating here and
 always commanding Body makes the fallback one branch and the boot window a
 non-event. (The facade's field verb stays frozen and untouched; autonomous
 callers may still use it.)

 PROS-free, allocation-free, no std::string, no std::function, total (no
 preconditions): a teleop tick must not be able to abort the program.
```

</details>
