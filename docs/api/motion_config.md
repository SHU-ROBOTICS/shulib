<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/motion_config.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motion_config.hpp`

MotionConfig — the shared knobs of the C1 motion primitives.

This header declares **3** types (25 members) and **1** free function.

Extracted from [`include/shulib/motion/motion_config.hpp`](../../include/shulib/motion/motion_config.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct AxisGains`](#struct-axisgains)
  - [`kP`](#axisgains-kp)
  - [`kI`](#axisgains-ki)
  - [`kD`](#axisgains-kd)
  - [`integralLimit`](#axisgains-integrallimit)
- [`struct TankSteerConfig`](#struct-tanksteerconfig)
  - [`crossGain`](#tanksteerconfig-crossgain)
  - [`maxSteer`](#tanksteerconfig-maxsteer)
  - [`fadeDistance`](#tanksteerconfig-fadedistance)
  - [`leadTime`](#tanksteerconfig-leadtime)
  - [`yawObserverTime`](#tanksteerconfig-yawobservertime)
  - [`yawCorrectionLimit`](#tanksteerconfig-yawcorrectionlimit)
  - [`valid`](#tanksteerconfig-valid)
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
  - [`tankSteer`](#motionconfig-tanksteer)
  - [`validate`](#motionconfig-validate)
- [`validatedConfig`](#validatedconfig) — *free function*

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

<a id="struct-tanksteerconfig"></a>

## `struct TankSteerConfig`

```cpp
struct TankSteerConfig
```

How MoveToPose steers a TANK (a drivetrain whose strafeAuthority() is exactly 0) back onto its target's LINE while it drives — chunk TANK1. The line runs through the target point along the target heading (for Routine::driveTo that is the start-to-target line). On any drivetrain with strafe authority this struct is never read, so X-drive and H-drive motion is unchanged bit for bit. Every value is PROVISIONAL (A4: HA-179) — host-test-plant numbers, not a robot's.  What it does, in one sentence per knob: the heading MoveToPose servoes to is the target heading bent by crossGain × (inches off the line, as predicted leadTime seconds ahead), never by more than maxSteer, faded to nothing over the last fadeDistance inches of the leg; and while it steers, it MEASURES the difference between the yaw rate the robot made (the Localizer's reading of the IMU, this tick) and the yaw rate the heading loop asked for the tick before (after the omega clamp, before desaturation), low-passed over yawObserverTime, and commands the opposite, never more than yawCorrectionLimit — which is what cancels the steady turn of a weaker drive side. That difference also contains desaturation at full speed, motor lag and blocked rotation: while the robot cannot follow a yaw command, the correction integrates like a heading integral, up to yawCorrectionLimit, and decays within about yawObserverTime once the robot turns again (HA-179's blast radius). What it never does: plan a turn, or aim at the target POINT. A leg that STARTS more than maxSteer off its target heading is not steered at all (it runs exactly as it did before TANK1, and one Warn line says why) — face() stays your turn.

*struct, declared at [`include/shulib/motion/motion_config.hpp:85`](../../include/shulib/motion/motion_config.hpp#L85).*

<a id="tanksteerconfig-crossgain"></a>

### `TankSteerConfig::crossGain`

```cpp
double crossGain = 0.1
```

Heading bend per inch of cross-track error (rad/in). PROVISIONAL (A4: HA-179).

*field, declared at [`include/shulib/motion/motion_config.hpp:87`](../../include/shulib/motion/motion_config.hpp#L87).*

<a id="tanksteerconfig-maxsteer"></a>

### `TankSteerConfig::maxSteer`

```cpp
double maxSteer = 0.2617993877991494
```

The cap on the bend AND the engage window (rad, 15 degrees): the heading setpoint never leaves target heading ± maxSteer, and a leg whose first live heading is farther than this from its target heading is not steered. The two are ONE number on purpose: a leg that starts within the cap is one whose heading error the bend itself could have commanded; beyond it, steering would mean the heading loop first making the author's turn. So raising it to let the bend remove larger offsets also widens which un-faced legs are steered — on the host test plant un-faced legs arrive up to about 30 degrees off and time out from about 40, a cliff the author cannot see, so keep it well under 30 (HA-179). 0 turns TANK1 off entirely: a tank's moveTo then moves, records and ends exactly as it did before TANK1, bit for bit. Must be < pi/2 (a bend of 90 degrees or more would reverse which way the robot drives along the line). PROVISIONAL (A4: HA-179).

*field, declared at [`include/shulib/motion/motion_config.hpp:99`](../../include/shulib/motion/motion_config.hpp#L99).*

<a id="tanksteerconfig-fadedistance"></a>

### `TankSteerConfig::fadeDistance`

```cpp
double fadeDistance = 6.0
```

Along-track distance over which the bend fades to zero at arrival (in, > 0): the bend is scaled by clamp(along-track distance to go / fadeDistance, -1, 1), which is continuous through arrival and carries the sign that makes a REVERSE approach steer the right way. PROVISIONAL (A4: HA-179).

*field, declared at [`include/shulib/motion/motion_config.hpp:104`](../../include/shulib/motion/motion_config.hpp#L104).*

<a id="tanksteerconfig-leadtime"></a>

### `TankSteerConfig::leadTime`

```cpp
double leadTime = 0.06
```

How far ahead (s) the bend looks: it steers on the cross-track error predicted this far ahead from the robot's heading and the forward speed it was commanded the tick before (tank_steer.hpp, tankLeadCross), not on the error now. It damps the line loop: without it a leg that starts pointed at its line can overshoot the line near the target, where the fade leaves the rest unremovable — a leg that finished before TANK1 then timed out. On the host test plant (2026-09-28, the Part A review's grid of 24 563 legs) no lead from 0.05 to 0.10 s left any leg that settles with TANK1 off timing out with it on, at motor time constants from 0 to about 0.18 s; with no lead 178 did (0.04 s: 4; 0.25 s under lag: 72–102). 0.06 sits inside that band and keeps a 6 in offset over 40 in removable on the lag-free plant. >= 0 (0 = steer on the error now). PROVISIONAL (A4: HA-179).

*field, declared at [`include/shulib/motion/motion_config.hpp:115`](../../include/shulib/motion/motion_config.hpp#L115).*

<a id="tanksteerconfig-yawobservertime"></a>

### `TankSteerConfig::yawObserverTime`

```cpp
double yawObserverTime = 0.1
```

Time constant (s) of the low-pass that turns "measured yaw rate − commanded yaw rate" into the estimated unexplained yaw. It must be long against IMU noise and short against the leg; it assumes the drivetrain follows a yaw command much faster than this (a heavy robot's lag reads as unexplained yaw — HA-179 records the host-test-plant margin). > 0. PROVISIONAL (A4: HA-179).

*field, declared at [`include/shulib/motion/motion_config.hpp:121`](../../include/shulib/motion/motion_config.hpp#L121).*

<a id="tanksteerconfig-yawcorrectionlimit"></a>

### `TankSteerConfig::yawCorrectionLimit`

```cpp
double yawCorrectionLimit = 1.5
```

The most yaw rate (rad/s) the correction may add, either way — bounding how large a side imbalance TANK1 will cancel, how hard it will fight a robot being turned by something else, and how far it integrates while the robot cannot follow (pinned, lagging). 0 disables the correction (the bend alone). >= 0. PROVISIONAL (A4: HA-179).

*field, declared at [`include/shulib/motion/motion_config.hpp:126`](../../include/shulib/motion/motion_config.hpp#L126).*

<a id="tanksteerconfig-valid"></a>

### `TankSteerConfig::valid`

```cpp
[[nodiscard]] bool valid() const noexcept
```

Every field finite; crossGain, leadTime and yawCorrectionLimit >= 0; maxSteer in [0, pi/2); fadeDistance and yawObserverTime > 0. MotionConfig::validate() refuses a config whose tankSteer fails this.

*function, declared at [`include/shulib/motion/motion_config.hpp:131`](../../include/shulib/motion/motion_config.hpp#L131).*

<a id="struct-motionconfig"></a>

## `struct MotionConfig`

```cpp
struct MotionConfig
```

Every knob the C1 motion primitives share. A motion COPIES it at construction and validate()s the copy, so later edits to the object you built from never reach a live motion — build a fresh config, then a fresh motion. Units are canonical throughout (inches, radians, seconds), but only the speed and geometry budgets carry theirs in the TYPE (units::Velocity / AngularVelocity / Length); the gains, defaultTimeout and every SettleConfig / OdoStallCheckConfig field are bare doubles whose units live only in the comment beside them. Nor are the gains dimensionless — kP is 1/s and kI 1/s², kD alone is dimensionless — what the axis they are handed to supplies is WHICH quantity they act on (inches for translation, radians for heading), not their dimension.

*struct, declared at [`include/shulib/motion/motion_config.hpp:149`](../../include/shulib/motion/motion_config.hpp#L149).*

<a id="motionconfig-wheelff"></a>

### `MotionConfig::wheelFf`

```cpp
control::FeedforwardGains wheelFf{.kS = 1.0, .kV = 12.0 / 70.0, .kA = 0.0}
```

Wheel feedforward — MUST match the drivetrain's characterization (R5). Default mirrors the plant's placeholder (≈70 in/s free speed at 12 V). PROVISIONAL (A4: HA-45/HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:153`](../../include/shulib/motion/motion_config.hpp#L153).*

<a id="motionconfig-translation"></a>

### `MotionConfig::translation`

```cpp
AxisGains translation{.kP = 3.0}
```

Translation: inches of field-axis error → in/s of field-axis velocity command. Applied identically to x AND y (header note). PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:157`](../../include/shulib/motion/motion_config.hpp#L157).*

<a id="motionconfig-heading"></a>

### `MotionConfig::heading`

```cpp
AxisGains heading{.kP = 4.0}
```

Heading: radians of shortest-path error → rad/s. PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:159`](../../include/shulib/motion/motion_config.hpp#L159).*

<a id="motionconfig-maxlinearspeed"></a>

### `MotionConfig::maxLinearSpeed`

```cpp
units::Velocity maxLinearSpeed{60.0}
```

Field-frame linear speed budget (in/s) — the norm cap AND the base of the strafe-authority clamp. PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:163`](../../include/shulib/motion/motion_config.hpp#L163).*

<a id="motionconfig-maxangularspeed"></a>

### `MotionConfig::maxAngularSpeed`

```cpp
units::AngularVelocity maxAngularSpeed{6.0}
```

Yaw-rate budget (rad/s). PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:165`](../../include/shulib/motion/motion_config.hpp#L165).*

<a id="motionconfig-maxwheelspeed"></a>

### `MotionConfig::maxWheelSpeed`

```cpp
units::Velocity maxWheelSpeed{60.0}
```

Per-wheel surface-speed budget for desaturate() (in/s). PROVISIONAL (HA-50).

*field, declared at [`include/shulib/motion/motion_config.hpp:167`](../../include/shulib/motion/motion_config.hpp#L167).*

<a id="motionconfig-translationsettle"></a>

### `MotionConfig::translationSettle`

```cpp
control::SettleConfig translationSettle{.maxError = 0.5, .maxErrorRate = 1.0, .settleTime = 0.1}
```

Translation settle: |pos error| (in), |d error/dt| (in/s), held (s). PROVISIONAL (A4: HA-51).

*field, declared at [`include/shulib/motion/motion_config.hpp:171`](../../include/shulib/motion/motion_config.hpp#L171).*

<a id="motionconfig-headingsettle"></a>

### `MotionConfig::headingSettle`

```cpp
control::SettleConfig headingSettle{.maxError = 0.02, .maxErrorRate = 0.30, .settleTime = 0.1}
```

Heading settle: |shortest error| (rad ≈ 1.15°), rate (rad/s — noise floor note in header), held (s). PROVISIONAL (A4: HA-51).

*field, declared at [`include/shulib/motion/motion_config.hpp:175`](../../include/shulib/motion/motion_config.hpp#L175).*

<a id="motionconfig-brakesettle"></a>

### `MotionConfig::brakeSettle`

```cpp
control::SettleConfig brakeSettle{.maxError = 1.2, .maxErrorRate = 100.0, .settleTime = 0.1}
```

DriveBrake settle on the AVERAGED speed norm |v| + rotationRadius·|ω| (in/s), its rate (in/s²), held (s). The threshold sits deliberately ABOVE the M2 estimator's averaged twist-noise floor (~0.3–0.9 in/s at a physical dead stop under composed hostility — drive_brake.hpp header); tighter would never settle on a hostile field. PROVISIONAL (A4: HA-51).

*field, declared at [`include/shulib/motion/motion_config.hpp:182`](../../include/shulib/motion/motion_config.hpp#L182).*

<a id="motionconfig-defaulttimeout"></a>

### `MotionConfig::defaultTimeout`

```cpp
double defaultTimeout = 5.0
```

Watchdog default when a motion is constructed without an explicit timeout (seconds). PROVISIONAL (A4: HA-51).

*field, declared at [`include/shulib/motion/motion_config.hpp:187`](../../include/shulib/motion/motion_config.hpp#L187).*

<a id="motionconfig-rotationradius"></a>

### `MotionConfig::rotationRadius`

```cpp
units::Length rotationRadius{7.0}
```

Center-to-wheel distance (in) — converts |ω| to an equivalent linear speed in DriveBrake's norm. Stand-in geometry (A4: HA-17/HA-52).

*field, declared at [`include/shulib/motion/motion_config.hpp:191`](../../include/shulib/motion/motion_config.hpp#L191).*

<a id="motionconfig-stall"></a>

### `MotionConfig::stall`

```cpp
OdoStallCheckConfig stall{}
```

The spin-vs-motion cross-check thresholds (A4: HA-52) and, since R3b Part 2, its per-wheel drive geometry (`stall.wheels`, hal::DriveGeometry — the same objects the drive-encoder odometry converts with; `stall.setAllWheels()` for a symmetric drive) and `stall.independentMotionSource` (false on a drivetrain whose odometry IS the drive encoders: the check then never reports a verdict — odo_stall_check.hpp's §2 ruling).

*field, declared at [`include/shulib/motion/motion_config.hpp:198`](../../include/shulib/motion/motion_config.hpp#L198).*

<a id="motionconfig-tanksteer"></a>

### `MotionConfig::tankSteer`

```cpp
TankSteerConfig tankSteer{}
```

Tank steering (chunk TANK1): read by MoveToPose only, and only when the drivetrain's strafeAuthority() is exactly 0. `tankSteer.maxSteer = 0` switches it off for every leg built from this config. Appended last so every designated initializer of this struct keeps compiling. PROVISIONAL (A4: HA-179).

*field, declared at [`include/shulib/motion/motion_config.hpp:204`](../../include/shulib/motion/motion_config.hpp#L204).*

<a id="motionconfig-validate"></a>

### `MotionConfig::validate`

```cpp
void validate() const
```

Re-check the invariants the motions rely on and RAISE on the first violation: feedforward and PID gains finite, integral limits non-negative, all FIVE speed / timeout / geometry scalars strictly positive (maxLinearSpeed, maxAngularSpeed, maxWheelSpeed, defaultTimeout, rotationRadius — 0 is rejected, never read as "unset"), and tankSteer.valid(). Every C1 motion calls this from its own constructor, so it is a backstop rather than a step you can forget — call it yourself only when validating a config you have not yet handed to a motion. It deliberately does NOT descend into the SettleConfig or OdoStallCheckConfig members: those are checked by SettledUtil and OdoStallCheck when the motion builds them, which is the only place their own invariants are known. It DOES descend into tankSteer (since TANK1): no other object owns that struct, and MoveToPose reads it on every tank tick, so this is the one place a bad value can be refused before it steers.

*function, declared at [`include/shulib/motion/motion_config.hpp:218`](../../include/shulib/motion/motion_config.hpp#L218).*

<a id="validatedconfig"></a>

## `validatedConfig`

```cpp
[[nodiscard]] inline const MotionConfig& validatedConfig(const MotionConfig& config, double timeout, const char* who)
```

Validate `config` (and a caller-supplied `timeout`) and hand the config straight back, so a motion can write `cfg_{validatedConfig(config, timeout, "TurnTo")}` as the FIRST member in its initializer list and have the check run before any component is built from these fields. The counterpart to MotionDeps::validatedClock(), which exists for exactly the same reason on the pointer half: "a null pointer trips the precondition rather than being dereferenced." Without it the first component constructed from a bad config reports the failure in ITS vocabulary, naming a class the caller never named.

*free function, declared at [`include/shulib/motion/motion_config.hpp:257`](../../include/shulib/motion/motion_config.hpp#L257).*

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
 (settle tolerances), HA-52 (stall cross-check + stand-in radii), HA-179 (TANK1's
 tank steering, TankSteerConfig). Do not read any number here as a measurement.

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
