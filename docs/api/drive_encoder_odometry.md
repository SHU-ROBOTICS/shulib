<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/drive_encoder_odometry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drive_encoder_odometry.hpp`

DriveEncoderOdometry — dead reckoning from the DRIVE motors' own encoders and the IMU, for a tank drive with no tracking wheels.

This header declares **3** types (14 members).

Extracted from [`include/shulib/localization/drive_encoder_odometry.hpp`](../../include/shulib/localization/drive_encoder_odometry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct DriveEncoderOdometryConfig`](#struct-driveencoderodometryconfig)
  - [`maxTickRotation`](#driveencoderodometryconfig-maxtickrotation)
  - [`maxTickTravel`](#driveencoderodometryconfig-maxticktravel)
- [`struct SideTravel`](#struct-sidetravel)
  - [`left`](#sidetravel-left)
  - [`right`](#sidetravel-right)
- [`class DriveEncoderOdometry`](#class-driveencoderodometry)
  - [`DriveEncoderOdometry`](#driveencoderodometry-driveencoderodometry)
  - [`update`](#driveencoderodometry-update)
  - [`pose`](#driveencoderodometry-pose)
  - [`setPose`](#driveencoderodometry-setpose)
  - [`lastDeltaImplausible`](#driveencoderodometry-lastdeltaimplausible)
  - [`lastHeadingDisagreement`](#driveencoderodometry-lastheadingdisagreement)
  - [`lastSideTravel`](#driveencoderodometry-lastsidetravel)
  - [`leftInchesPerRadian`](#driveencoderodometry-leftinchesperradian)
  - [`rightInchesPerRadian`](#driveencoderodometry-rightinchesperradian)
  - [`trackWidth`](#driveencoderodometry-trackwidth)

<a id="struct-driveencoderodometryconfig"></a>

## `struct DriveEncoderOdometryConfig`

```cpp
struct DriveEncoderOdometryConfig
```

The trust gate's two knobs — the SAME two as PilonsOdometryConfig, same names, same defaults, same reasoning (cited, not restated: read that struct's field comments).

*struct, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:70`](../../include/shulib/localization/drive_encoder_odometry.hpp#L70).*

<a id="driveencoderodometryconfig-maxtickrotation"></a>

### `DriveEncoderOdometryConfig::maxTickRotation`

```cpp
units::AngleDim maxTickRotation{0.5 * math::Angle::kPi}
```

|Δθ| (radians) above which a tick's heading change is implausible. Default π/2 — PilonsOdometryConfig::maxTickRotation, verbatim.

*field, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:73`](../../include/shulib/localization/drive_encoder_odometry.hpp#L73).*

<a id="driveencoderodometryconfig-maxticktravel"></a>

### `DriveEncoderOdometryConfig::maxTickTravel`

```cpp
units::Length maxTickTravel{36.0}
```

Largest believable |Δtravel| of ONE side in one tick (inches) before the delta is implausible. Default 36 in — PilonsOdometryConfig::maxTickTravel, verbatim, incl. its dt-blindness caveat. PROVISIONAL (A4: HA-123).

*field, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:77`](../../include/shulib/localization/drive_encoder_odometry.hpp#L77).*

<a id="struct-sidetravel"></a>

## `struct SideTravel`

```cpp
struct SideTravel
```

The two sides' travel over the last tick (inches), for the panel and the stall-check agreement test.

*struct, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:82`](../../include/shulib/localization/drive_encoder_odometry.hpp#L82).*

<a id="sidetravel-left"></a>

### `SideTravel::left`

```cpp
units::Length left{}
```

left side's wheel travel this tick (+ = forward)

*field, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:83`](../../include/shulib/localization/drive_encoder_odometry.hpp#L83).*

<a id="sidetravel-right"></a>

### `SideTravel::right`

```cpp
units::Length right{}
```

right side's wheel travel this tick (+ = forward)

*field, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:84`](../../include/shulib/localization/drive_encoder_odometry.hpp#L84).*

<a id="class-driveencoderodometry"></a>

## `class DriveEncoderOdometry`

```cpp
class DriveEncoderOdometry final : public IOdometry
```

Dead reckoning for a tank drive without tracking wheels: each side's drive-motor encoder delta (radians × that side's hal::DriveGeometry) gives the side's travel; the centre moves forward by their mean and sideways by exactly zero (tank.hpp's written decision); the IMU owns heading; the same arcStep and the same trust gate as PilonsOdometry. Exposes the encoder-vs-IMU heading cross-check as an observable — never a fault. Holds the IMU and both motors by reference; owns no loop.

*class, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:93`](../../include/shulib/localization/drive_encoder_odometry.hpp#L93).*

<a id="driveencoderodometry-driveencoderodometry"></a>

### `DriveEncoderOdometry::DriveEncoderOdometry`

```cpp
DriveEncoderOdometry(hal::IImu& imu, hal::IMotor& left, hal::IMotor& right, hal::DriveGeometry leftGeometry, hal::DriveGeometry rightGeometry, units::Length trackWidth, const math::Pose2d& initial = {}, const DriveEncoderOdometryConfig& config = {})
```

`left` / `right`: the side's IMotor (a hal::MotorGroup on a real robot) in the kinematics' wheel order; `leftGeometry` / `rightGeometry`: each side's geometry — one object per side, the SAME objects the stall check is configured with; `trackWidth`: contact line to contact line, > 0 (TankKinematics's own value); `initial` seeds the position (heading informational — IMU-owned from the first reading).

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:100`](../../include/shulib/localization/drive_encoder_odometry.hpp#L100).*

<a id="driveencoderodometry-update"></a>

### `DriveEncoderOdometry::update`

```cpp
void update() override
```

One integration tick: read the IMU and both sides, convert, arcStep, accumulate, evaluate the trust gate and the heading cross-check.

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:133`](../../include/shulib/localization/drive_encoder_odometry.hpp#L133).*

<a id="driveencoderodometry-pose"></a>

### `DriveEncoderOdometry::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const noexcept override
```

The accumulated field-frame estimate (heading the IMU's). A pure read.

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:174`](../../include/shulib/localization/drive_encoder_odometry.hpp#L174).*

<a id="driveencoderodometry-setpose"></a>

### `DriveEncoderOdometry::setPose`

```cpp
void setPose(const math::Pose2d& p) override
```

Teleport the POSITION (x, y); heading stays IMU-owned. Re-baselines the heading reference so the teleport injects no phantom rotation; the encoder baselines are left intact (a teleport doesn't change what the wheels have rolled).

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:179`](../../include/shulib/localization/drive_encoder_odometry.hpp#L179).*

<a id="driveencoderodometry-lastdeltaimplausible"></a>

### `DriveEncoderOdometry::lastDeltaImplausible`

```cpp
[[nodiscard]] bool lastDeltaImplausible() const noexcept override
```

True iff the last update() was untrustworthy (oversized Δθ, oversized side travel, or a non-finite integration).

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:186`](../../include/shulib/localization/drive_encoder_odometry.hpp#L186).*

<a id="driveencoderodometry-lastheadingdisagreement"></a>

### `DriveEncoderOdometry::lastHeadingDisagreement`

```cpp
[[nodiscard]] double lastHeadingDisagreement() const noexcept
```

The heading cross-check of the last tick: encoder-implied Δθ − IMU Δθ (radians, signed). ≈ 0 under clean rolling; a slipping or stalled side shows as a bias with the sign of the side that under-travelled the IMU's turn. An observable, not a fault.

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:191`](../../include/shulib/localization/drive_encoder_odometry.hpp#L191).*

<a id="driveencoderodometry-lastsidetravel"></a>

### `DriveEncoderOdometry::lastSideTravel`

```cpp
[[nodiscard]] SideTravel lastSideTravel() const noexcept
```

Each side's wheel travel over the last tick (inches) — the panel's two numbers and the value the stall check must agree with for the same encoder delta.

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:195`](../../include/shulib/localization/drive_encoder_odometry.hpp#L195).*

<a id="driveencoderodometry-leftinchesperradian"></a>

### `DriveEncoderOdometry::leftInchesPerRadian`

```cpp
[[nodiscard]] units::Length leftInchesPerRadian() const noexcept
```

Inches per radian in use for the left / right side (the geometry, resolved).

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:198`](../../include/shulib/localization/drive_encoder_odometry.hpp#L198).*

<a id="driveencoderodometry-rightinchesperradian"></a>

### `DriveEncoderOdometry::rightInchesPerRadian`

```cpp
[[nodiscard]] units::Length rightInchesPerRadian() const noexcept
```

Inches per radian in use for the right side.

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:200`](../../include/shulib/localization/drive_encoder_odometry.hpp#L200).*

<a id="driveencoderodometry-trackwidth"></a>

### `DriveEncoderOdometry::trackWidth`

```cpp
[[nodiscard]] units::Length trackWidth() const noexcept
```

The track width in use (inches).

*function, declared at [`include/shulib/localization/drive_encoder_odometry.hpp:202`](../../include/shulib/localization/drive_encoder_odometry.hpp#L202).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 51 lines, click to expand</summary>

```text

 DriveEncoderOdometry — dead reckoning from the DRIVE motors' own encoders and the IMU,
 for a tank drive with no tracking wheels (chunk R3b Part 2, 2026-09-14). The second
 IOdometry implementation, beside PilonsOdometry.

 ── What it integrates, and the decision it borrows ─────────────────────────────────
 Per tick: ΔL and ΔR are each side's cumulative IMotor::position() delta (output-shaft
 radians, the MEDIAN of a hal::MotorGroup on a real robot) times that side's inches per
 radian (hal::DriveGeometry — the ONE representation of the per-side ratio, shared with
 the stall check). The centre's forward travel is (ΔL + ΔR)/2 and its LATERAL travel is
 EXACTLY 0: kinematics/tank.hpp's forward() already states, in writing, that "this
 drivetrain cannot observe lateral motion, so a real skid sideways is reported as no
 motion at all". This class is the odometry counterpart of that decision, not a new one
 — it inherits the justification rather than inventing a second. Then the SAME arcStep
 as PilonsOdometry (the one constant-curvature integration step; the localizer's
 accuracy-critical line, exhaustively tested on its own).

 HEADING IS IMU-OWNED — the identical policy, in the identical words, as PilonsOdometry's
 decision #3: the pose heading is set EQUAL to the IMU heading every tick (absolute,
 never integrated from the wheels — wheel-difference heading is the cross-check only,
 never the authority), and from construction onward (the seeded pose's heading is
 informational; the IMU is the authority, so there is no construction→first-update
 window where they disagree). Δθ for the arc comes from the two IMU samples via
 Angle::errorTo (shortest signed, wrap-correct).

 ── The trust gate — carried over, not re-derived ───────────────────────────────────
 Both halves of PilonsOdometry's gate, with the same two knobs, the same names, the same
 defaults and the same reasoning (read PilonsOdometryConfig's field comments; they are
 not repeated here): |Δθ| above maxTickRotation, or either side's |Δtravel| above
 maxTickTravel, or a non-finite integration ⇒ lastDeltaImplausible(). An implausible-but-
 finite tick is REPORTED, NOT WITHHELD; only a non-finite tick FREEZES position (heading
 still advances). The travel half matters more here than for tracking wheels: a drive
 port that enumerates late, or whose adapter holds a last-good value and then wakes,
 produces exactly the one-tick phantom the gate exists to flag.

 ── The heading cross-check — NEW, and deliberately an observable, not a fault ──────
 The encoders imply their own heading change, Δθ_enc = (ΔR − ΔL) / trackWidth. Against
 the IMU's Δθ it is the wheel-difference heading the Pilons header calls "the cross-check
 only, never the authority": lastHeadingDisagreement() = Δθ_enc − Δθ_imu (radians,
 signed) is what a SLIPPING side looks like, and it is the measurement R3d's track-width
 calibration is built from. It is NOT promoted to a fault here: a straight-line stall
 (both sides slipping equally) does not show in it, so it cannot stand in for the
 stall check. For THIS drivetrain the stall check has no independent motion source at
 all (the odometry IS the wheels — motion/odo_stall_check.hpp's
 `independentMotionSource`), and a future stuck/slip detector for the chassis will be
 built on this observable; the composition root says so once at boot.

 Baselines both sides at construction (the TrackingWheel::reset() precedent,
 pilons_odometry.hpp:119), so a pre-existing shaft total is not counted as travel on the
 first update(). Holds every reference; the IMU and both motors must outlive this object.
 Owns no loop. PROS-free; host-tested against the A2 plant's ground truth.
```

</details>
