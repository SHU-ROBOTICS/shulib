<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/motion_config.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motion_config.hpp`

MotionConfig — the shared knobs of the C1 motion primitives.

This header declares **2** types (17 members).

Extracted from [`include/shulib/motion/motion_config.hpp`](../../include/shulib/motion/motion_config.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct AxisGains`](#struct-axisgains)
  - [`kP`](#axisgains-kp)
  - [`kI`](#axisgains-ki)
  - [`kD`](#axisgains-kd)
  - [`integralLimit`](#axisgains-integrallimit)
- [`struct MotionConfig`](#struct-motionconfig)
  - [`wheelFf`](#motionconfig-wheelff)
  - [`translation`](#motionconfig-translation)
  - [`heading`](#motionconfig-heading)
  - [`maxLinearSpeed`](#motionconfig-maxlinearspeed)
  - [`maxAngularSpeed`](#motionconfig-maxangularspeed)
  - [`maxWheelSpeed`](#motionconfig-maxwheelspeed)
  - [`translationSettle`](#motionconfig-translationsettle)
  - [`headingSettle`](#motionconfig-headingsettle)
  - [`brakeSettle`](#motionconfig-brakesettle)
  - [`defaultTimeout`](#motionconfig-defaulttimeout)
  - [`rotationRadius`](#motionconfig-rotationradius)
  - [`stall`](#motionconfig-stall)
  - [`validate`](#motionconfig-validate)

<a id="struct-axisgains"></a>

## `struct AxisGains`

```cpp
struct AxisGains
```

Per-axis PID gains (units documented at each use site). Output saturation is deliberately NOT here — the motion layer's norm/ω caps own it (header note).

*struct, declared at [`include/shulib/motion/motion_config.hpp:55`](../../include/shulib/motion/motion_config.hpp#L55).*

<a id="axisgains-kp"></a>

### `AxisGains::kP`

```cpp
double kP = 0.0
```

Proportional gain, 1/s on both axes: in→in/s, rad→rad/s.

*field, declared at [`include/shulib/motion/motion_config.hpp:56`](../../include/shulib/motion/motion_config.hpp#L56).*

<a id="axisgains-ki"></a>

### `AxisGains::kI`

```cpp
double kI = 0.0
```

Integral gain, 1/s². 0 (the default) makes the axis pure-P.

*field, declared at [`include/shulib/motion/motion_config.hpp:57`](../../include/shulib/motion/motion_config.hpp#L57).*

<a id="axisgains-kd"></a>

### `AxisGains::kD`

```cpp
double kD = 0.0
```

Derivative gain (dimensionless), on the MEASUREMENT — no setpoint kick.

*field, declared at [`include/shulib/motion/motion_config.hpp:58`](../../include/shulib/motion/motion_config.hpp#L58).*

<a id="axisgains-integrallimit"></a>

### `AxisGains::integralLimit`

```cpp
double integralLimit = std::numeric_limits<double>::infinity()
```

Symmetric ± clamp on the I-TERM (kI·∫e dt) in command units, with the accumulator back-calculated so it cannot wind up past the clamp. Infinity means unclamped, which is only safe while kI is 0 — the default pairing. Must be ≥ 0.

*field, declared at [`include/shulib/motion/motion_config.hpp:62`](../../include/shulib/motion/motion_config.hpp#L62).*

<a id="struct-motionconfig"></a>

## `struct MotionConfig`

```cpp
struct MotionConfig
```

Every knob the C1 motion primitives share. A motion COPIES it at construction and validate()s the copy, so later edits to the object you built from never reach a live motion — build a fresh config, then a fresh motion. Units are canonical throughout (inches, radians, seconds), but only the speed and geometry budgets carry theirs in the TYPE (units::Velocity / AngularVelocity / Length); the gains, defaultTimeout and every SettleConfig / OdoStallCheckConfig field are bare doubles whose units live only in the comment beside them. Nor are the gains dimensionless — kP is 1/s and kI 1/s², kD alone is dimensionless — what the axis they are handed to supplies is WHICH quantity they act on (inches for translation, radians for heading), not their dimension.

*struct, declared at [`include/shulib/motion/motion_config.hpp:74`](../../include/shulib/motion/motion_config.hpp#L74).*

<a id="motionconfig-wheelff"></a>

### `MotionConfig::wheelFf`

```cpp
control::FeedforwardGains wheelFf{.kS = 1.0, .kV = 12.0 / 70.0, .kA = 0.0}
```

Wheel feedforward — MUST match the drivetrain's characterization (R5). Default mirrors the plant's placeholder (≈70 in/s free speed at 12 V). PROVISIONAL (A4: HA-45/HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:78`](../../include/shulib/motion/motion_config.hpp#L78).*

<a id="motionconfig-translation"></a>

### `MotionConfig::translation`

```cpp
AxisGains translation{.kP = 3.0}
```

Translation: inches of field-axis error → in/s of field-axis velocity command. Applied identically to x AND y (header note). PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:82`](../../include/shulib/motion/motion_config.hpp#L82).*

<a id="motionconfig-heading"></a>

### `MotionConfig::heading`

```cpp
AxisGains heading{.kP = 4.0}
```

Heading: radians of shortest-path error → rad/s. PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:84`](../../include/shulib/motion/motion_config.hpp#L84).*

<a id="motionconfig-maxlinearspeed"></a>

### `MotionConfig::maxLinearSpeed`

```cpp
units::Velocity maxLinearSpeed{60.0}
```

Field-frame linear speed budget (in/s) — the norm cap AND the base of the strafe-authority clamp. PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:88`](../../include/shulib/motion/motion_config.hpp#L88).*

<a id="motionconfig-maxangularspeed"></a>

### `MotionConfig::maxAngularSpeed`

```cpp
units::AngularVelocity maxAngularSpeed{6.0}
```

Yaw-rate budget (rad/s). PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:90`](../../include/shulib/motion/motion_config.hpp#L90).*

<a id="motionconfig-maxwheelspeed"></a>

### `MotionConfig::maxWheelSpeed`

```cpp
units::Velocity maxWheelSpeed{60.0}
```

Per-wheel surface-speed budget for desaturate() (in/s). PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:92`](../../include/shulib/motion/motion_config.hpp#L92).*

<a id="motionconfig-translationsettle"></a>

### `MotionConfig::translationSettle`

```cpp
control::SettleConfig translationSettle{.maxError = 0.5, .maxErrorRate = 1.0, .settleTime = 0.1}
```

Translation settle: |pos error| (in), |d error/dt| (in/s), held (s). PROVISIONAL (A4: HA-51).

*field, declared at [`include/shulib/motion/motion_config.hpp:96`](../../include/shulib/motion/motion_config.hpp#L96).*

<a id="motionconfig-headingsettle"></a>

### `MotionConfig::headingSettle`

```cpp
control::SettleConfig headingSettle{.maxError = 0.02, .maxErrorRate = 0.30, .settleTime = 0.1}
```

Heading settle: |shortest error| (rad ≈ 1.15°), rate (rad/s — noise floor note in header), held (s). PROVISIONAL (A4: HA-51).

*field, declared at [`include/shulib/motion/motion_config.hpp:100`](../../include/shulib/motion/motion_config.hpp#L100).*

<a id="motionconfig-brakesettle"></a>

### `MotionConfig::brakeSettle`

```cpp
control::SettleConfig brakeSettle{.maxError = 1.2, .maxErrorRate = 100.0, .settleTime = 0.1}
```

DriveBrake settle on the AVERAGED speed norm |v| + rotationRadius·|ω| (in/s), its rate (in/s²), held (s). The threshold sits deliberately ABOVE the M2 estimator's averaged twist-noise floor (~0.3–0.9 in/s at a physical dead stop under composed hostility — drive_brake.hpp header); tighter would never settle on a hostile field. PROVISIONAL (A4: HA-51).

*field, declared at [`include/shulib/motion/motion_config.hpp:107`](../../include/shulib/motion/motion_config.hpp#L107).*

<a id="motionconfig-defaulttimeout"></a>

### `MotionConfig::defaultTimeout`

```cpp
double defaultTimeout = 5.0
```

Watchdog default when a motion is constructed without an explicit timeout (seconds). PROVISIONAL (A4: HA-51).

*field, declared at [`include/shulib/motion/motion_config.hpp:112`](../../include/shulib/motion/motion_config.hpp#L112).*

<a id="motionconfig-rotationradius"></a>

### `MotionConfig::rotationRadius`

```cpp
units::Length rotationRadius{7.0}
```

Center-to-wheel distance (in) — converts |ω| to an equivalent linear speed in DriveBrake's norm. Stand-in geometry (A4: HA-17/HA-52).

*field, declared at [`include/shulib/motion/motion_config.hpp:116`](../../include/shulib/motion/motion_config.hpp#L116).*

<a id="motionconfig-stall"></a>

### `MotionConfig::stall`

```cpp
OdoStallCheckConfig stall{}
```

The spin-vs-motion cross-check thresholds (A4: HA-52).

*field, declared at [`include/shulib/motion/motion_config.hpp:119`](../../include/shulib/motion/motion_config.hpp#L119).*

<a id="motionconfig-validate"></a>

### `MotionConfig::validate`

```cpp
void validate() const
```

Re-check the invariants the motions rely on and RAISE on the first violation: feedforward and PID gains finite, integral limits non-negative, and all FIVE speed / timeout / geometry scalars strictly positive (maxLinearSpeed, maxAngularSpeed, maxWheelSpeed, defaultTimeout, rotationRadius — 0 is rejected, never read as "unset"). Every C1 motion calls this from its own constructor, so it is a backstop rather than a step you can forget — call it yourself only when validating a config you have not yet handed to a motion. It deliberately does NOT descend into the SettleConfig or OdoStallCheckConfig members: those are checked by SettledUtil and OdoStallCheck when the motion builds them, which is the only place their own invariants are known.

*function, declared at [`include/shulib/motion/motion_config.hpp:131`](../../include/shulib/motion/motion_config.hpp#L131).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 39 lines</summary>

```text

 MotionConfig — the shared knobs of the C1 motion primitives.

 ── EVERY DEFAULT HERE IS PROVISIONAL (Phase C's opening rule) ──────────────────────
 The A2 plant proves control LOGIC, not CONSTANTS. Gains below converge on the
 plant's placeholder dynamics; R5 measures real kS/kV/kA and re-tunes, R6
 back-fits the plant. Register entries: HA-50 (gains + speed budget), HA-51
 (settle tolerances), HA-52 (stall cross-check + stand-in radii). Do not read
 any number here as a measurement.

 ── Unit discipline (constraint: this layer owns unit consistency) ──────────────────
 Pid is bare-double by design. Each axis's gains are documented WITH their
 units and never shared across dimensions:
   * translation: error INCHES → command IN/S      (kP in 1/s)
   * heading:     error RADIANS → command RAD/S    (kP in 1/s)
 One translation gain set serves BOTH field axes deliberately: the x/y
 controllers act on FIELD coordinates, and unequal gains would make the
 closed-loop behaviour depend on which way the FIELD is oriented — breaking
 the rotational equivariance the frame tests pin. (An axis-asymmetric robot is
 a BODY-frame property; it belongs in kinematics/feedforward, not here.)

 ── Saturation policy (who clamps what — F5 choreography) ───────────────────────────
   1. |ω| clamped to maxAngularSpeed (scalar).
   2. (vx, vy) FIELD demand norm-capped to maxLinearSpeed — UNIFORMLY, so the
      commanded direction is preserved (per-axis clamps would curve diagonals).
   3. After fieldToRobot: BODY |vy| clamped to strafeAuthority()·maxLinearSpeed
      (the upstream clamp §13 #5 assigns to the motion layer; authority is a
      READ-ONLY query — kinematics itself never clamps).
   4. IKinematics::desaturate(…, maxWheelSpeed) — the downstream uniform scale.
   5. compensateForBattery() — the final per-wheel hardware ceiling.
 maxWheelSpeed's default keeps Feedforward(maxWheelSpeed) inside the 12 V rail
 with margin, so step 5 engages only under genuine sag (per-wheel clamping
 distorts direction, so routine operation must not rely on it).

 Settle-rate floors vs sensor noise (why headingSettle.maxErrorRate = 0.3):
 hostile IMU heading noise σ ≈ 0.05° (HA-21) differentiates to ≈ 0.12 rad/s of
 MEASURED rate noise at 100 Hz — a tighter rate bound would flap the settle
 window under hostility without the robot moving at all. 0.3 rad/s clears the
 noise floor ~2.5σ while still rejecting real residual rotation.
```

</details>
