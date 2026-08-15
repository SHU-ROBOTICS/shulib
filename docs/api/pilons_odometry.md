<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/pilons_odometry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `pilons_odometry.hpp`

PilonsOdometry — tracking-wheel dead-reckoning.

This header declares **2** types (7 members).

Extracted from [`include/shulib/localization/pilons_odometry.hpp`](../../include/shulib/localization/pilons_odometry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct PilonsOdometryConfig`](#struct-pilonsodometryconfig)
  - [`maxTickRotation`](#pilonsodometryconfig-maxtickrotation)
  - [`maxTickTravel`](#pilonsodometryconfig-maxticktravel)
- [`class PilonsOdometry`](#class-pilonsodometry)
  - [`PilonsOdometry`](#pilonsodometry-pilonsodometry)
  - [`update`](#pilonsodometry-update)
  - [`pose`](#pilonsodometry-pose)
  - [`setPose`](#pilonsodometry-setpose)
  - [`lastDeltaImplausible`](#pilonsodometry-lastdeltaimplausible)

<a id="struct-pilonsodometryconfig"></a>

## `struct PilonsOdometryConfig`

```cpp
struct PilonsOdometryConfig
```

The trust gate's one tuning knob, and deliberately nothing else: the geometry (wheel diameter, offset, role) rides on the TrackingWheels, and the heading source is the IMU handed to the constructor. Default-constructible, so `PilonsOdometry{imu, fwd, lat}` is the ordinary call.

*struct, declared at [`include/shulib/localization/pilons_odometry.hpp:55`](../../include/shulib/localization/pilons_odometry.hpp#L55).*

<a id="pilonsodometryconfig-maxtickrotation"></a>

### `PilonsOdometryConfig::maxTickRotation`

```cpp
units::AngleDim maxTickRotation{0.5 * math::Angle::kPi}
```

|Δθ| (radians) above which a tick's heading change is treated as implausible. Default π/2 sits far above any real ~100 Hz tick (≪ 1 rad) yet below the π wrap cliff. Tighten it for a known loop rate / max yaw rate. (See the trust-gate note in the header for what it can detect.) TYPED (units::AngleDim, radians). It was a bare `double` — the only untyped physical quantity in a file where travel, offsets and the pose are all typed — so the unit survived only in this comment, and passing 90.0 meaning DEGREES compiled, disabled the trust gate entirely (nothing is ever implausible below 90 radians) and tripped no check, because the constructor only bounded it from below.

*field, declared at [`include/shulib/localization/pilons_odometry.hpp:64`](../../include/shulib/localization/pilons_odometry.hpp#L64).*

<a id="pilonsodometryconfig-maxticktravel"></a>

### `PilonsOdometryConfig::maxTickTravel`

```cpp
units::Length maxTickTravel{36.0}
```

Largest believable |Δtravel| from ONE tracking wheel in one tick, before the delta is called implausible. The rotation half of this gate existed from the start and the translation half did not, which left a real hole: a pod that is dead or not yet enumerated at construction baselines at 0, and on the tick it finally answers, TrackingWheel differences its true cumulative position against that 0 and injects a one-tick phantom translation — measured at 28.4 in for a pod waking at 1000°, and unbounded in general. Nothing gated it: this class checked |Δθ| and never |Δtravel|. PROVISIONAL (A4: HA-123), and deliberately GENEROUS. This class has no clock, so the bound is dt-BLIND: the same 12 in is 100 ft/s on a 10 ms tick and 5 ft/s on a 200 ms one, and a bound tight enough to be interesting on a fast loop would reject real motion on a slow one. 36 in exceeds a full field length per tick at any plausible loop rate, so it cannot fire on a real drivetrain; it catches the CLASS of corruption that is orders of magnitude out, which is what a late-enumerating pod produces. A dt-aware bound needs a measured loop rate and belongs to R3/R4.

*field, declared at [`include/shulib/localization/pilons_odometry.hpp:80`](../../include/shulib/localization/pilons_odometry.hpp#L80).*

<a id="class-pilonsodometry"></a>

## `class PilonsOdometry`

```cpp
class PilonsOdometry
```

Tracking-wheel dead reckoning: every update() folds the two perpendicular wheels and the IMU heading into one constant-curvature arc and accumulates a field-frame Pose2d (canonical inches; F1's FIELD axes — +X right, +Y away from the red driver station, heading 0 along +X, CCW positive). The wheels measure BODY travel (+X forward, +Y left); arcStep rotates it by the tick's average heading, so the two frames coincide only at heading 0. The ~100 Hz prediction backbone the fused Localizer corrects with absolute fixes. Two facts govern everything else: HEADING IS IMU-OWNED — set equal to the IMU reading every tick, never integrated from the wheels — and wheel travel is offset-corrected to the tracking CENTER here, so a turn in place accumulates zero position. Holds the IMU by reference and copies the two wheels; the IMU and the wheels' rotation sensors must outlive this object. Owns no loop: the caller calls update() at its own cadence.

*class, declared at [`include/shulib/localization/pilons_odometry.hpp:96`](../../include/shulib/localization/pilons_odometry.hpp#L96).*

<a id="pilonsodometry-pilonsodometry"></a>

### `PilonsOdometry::PilonsOdometry`

```cpp
PilonsOdometry(hal::IImu& imu, TrackingWheel forward, TrackingWheel lateral, const math::Pose2d& initial = {}, const PilonsOdometryConfig& config = {})
```

`forward` must be a TrackingWheel::forward(), `lateral` a TrackingWheel::lateral() — the roles are checked so a swapped pair throws at construction. `initial` seeds the position; its heading is informational (the IMU owns heading from the first reading — see header).

*function, declared at [`include/shulib/localization/pilons_odometry.hpp:101`](../../include/shulib/localization/pilons_odometry.hpp#L101).*

<a id="pilonsodometry-update"></a>

### `PilonsOdometry::update`

```cpp
void update()
```

One integration tick: read the IMU + wheels, offset-correct, arcStep, accumulate.

*function, declared at [`include/shulib/localization/pilons_odometry.hpp:124`](../../include/shulib/localization/pilons_odometry.hpp#L124).*

<a id="pilonsodometry-pose"></a>

### `PilonsOdometry::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const noexcept
```

The accumulated field-frame estimate: x, y in canonical inches, heading as of the last update() or setPose() (the IMU's, never wheel-derived). A pure read — it advances only when update() runs, so repeated calls between ticks return the same pose. Before the first update() it is the seeded position with the IMU's construction-time heading.

*function, declared at [`include/shulib/localization/pilons_odometry.hpp:172`](../../include/shulib/localization/pilons_odometry.hpp#L172).*

<a id="pilonsodometry-setpose"></a>

### `PilonsOdometry::setPose`

```cpp
void setPose(const math::Pose2d& p)
```

Teleport the POSITION (x, y); heading stays IMU-owned. Re-baselines the heading reference so the teleport itself injects no phantom rotation on the next tick. Wheel baselines are left intact (a teleport doesn't change what the wheels have rolled).

*function, declared at [`include/shulib/localization/pilons_odometry.hpp:177`](../../include/shulib/localization/pilons_odometry.hpp#L177).*

<a id="pilonsodometry-lastdeltaimplausible"></a>

### `PilonsOdometry::lastDeltaImplausible`

```cpp
[[nodiscard]] bool lastDeltaImplausible() const noexcept
```

True iff the last update() was untrustworthy (oversized Δθ OR non-finite integration).

*function, declared at [`include/shulib/localization/pilons_odometry.hpp:183`](../../include/shulib/localization/pilons_odometry.hpp#L183).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 36 lines</summary>

```text

 PilonsOdometry — tracking-wheel dead-reckoning (master plan §8; WS5). Each tick it folds the
 two perpendicular tracking wheels + the IMU heading into one `arcStep` and accumulates the
 field-frame `Pose2d`. This is the high-rate (~100 Hz) prediction backbone the fused
 `Localizer` corrects with absolute fixes (GPS/AprilTag) at M3.

 Two responsibilities live here (and NOT in arc_step, which stays pure geometry):

 1. OFFSET CORRECTION (wheel travel → tracking-CENTER travel). A wheel offset from the center
    reads extra/less travel during a turn. Removing the rotation-induced component (derived
    from the planar rigid-body velocity v_point = (vf − ω·b, vl + ω·a); verified end-to-end by
    /tmp/odom_oracle.py):
        centerForward = forwardWheel.travel + Δθ · forwardWheel.offset   (offset = +LEFT coord)
        centerLateral = lateralWheel.travel − Δθ · lateralWheel.offset   (offset = +FORWARD coord)
    The opposite signs are not arbitrary: a pure in-place rotation (either direction) must yield
    ZERO center travel (else the robot "drifts" while turning — a classic odom bug). The
    pure-rotation tests (CW and CCW) pin both signs.

 2. HEADING is IMU-OWNED. The pose heading is set EQUAL to the IMU heading every tick (absolute,
    never integrated from the wheels — wheel-difference heading is the H-bot's cross-check only,
    decision #3), and from construction onward (the seeded pose's heading is informational; the
    IMU is the authority, so there is no construction→first-update window where they disagree).
    Δθ for the arc comes from the two IMU samples via Angle::errorTo (shortest signed, wrap-correct).

 TRUST GATE. `lastDeltaImplausible()` is true when a tick is NOT trustworthy, for either reason:
   * |Δθ| > maxTickRotation — a heading change too large for one tick. NOTE this is a magnitude
     threshold on the ALREADY-WRAPPED Δθ; it catches an aliased/stalled sample only when the
     wrapped magnitude lands in (maxTickRotation, π]. A real rotation > π that aliases to a SMALL
     Δθ is invisible here — it is excluded by the ~100 Hz loop-rate assumption (arc_step's
     PRECONDITION; A4 register HA-32 — the sustained loop rate on a loaded V5 is unmeasured),
     NOT by this gate. So `false` means "Δθ is within the per-tick bound," not
     "the rotation is certainly real."
   * the integration came out non-finite — a breach of the HAL finiteness contract (§7). A
     non-finite tick FREEZES the position at its last good value (it never writes NaN into the
     persistent pose) rather than poisoning the run; an oversized-but-finite tick is flagged but
     STILL integrated (the motion may be real). Recovery policy beyond this is the fusion layer's.
```

</details>
