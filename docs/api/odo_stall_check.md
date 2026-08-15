<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/odo_stall_check.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `odo_stall_check.hpp`

OdoStallCheck — the spin-vs-motion cross-check.

This header declares **2** types (10 members).

Extracted from [`include/shulib/motion/odo_stall_check.hpp`](../../include/shulib/motion/odo_stall_check.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct OdoStallCheckConfig`](#struct-odostallcheckconfig)
  - [`window`](#odostallcheckconfig-window)
  - [`minSpinTravel`](#odostallcheckconfig-minspintravel)
  - [`motionRatio`](#odostallcheckconfig-motionratio)
  - [`wheelRadius`](#odostallcheckconfig-wheelradius)
  - [`rotationRadius`](#odostallcheckconfig-rotationradius)
- [`class OdoStallCheck`](#class-odostallcheck)
  - [`kMaxWheels`](#odostallcheck-kmaxwheels)
  - [`OdoStallCheck`](#odostallcheck-odostallcheck)
  - [`update`](#odostallcheck-update)
  - [`stalled`](#odostallcheck-stalled)
  - [`reset`](#odostallcheck-reset)

<a id="struct-odostallcheckconfig"></a>

## `struct OdoStallCheckConfig`

```cpp
struct OdoStallCheckConfig
```

The five knobs of the spin-vs-motion cross-check, taken BY VALUE at construction (editing the struct afterwards does nothing to a live check) and every one of them validated by that constructor. Every default is PROVISIONAL: the two radii are stand-in geometry and the three thresholds are invented numbers — none has yet been measured against a real drivetrain or a real noise floor, so treat a default as a placeholder that compiles, not as a tuning.

*struct, declared at [`include/shulib/motion/odo_stall_check.hpp:69`](../../include/shulib/motion/odo_stall_check.hpp#L69).*

<a id="odostallcheckconfig-window"></a>

### `OdoStallCheckConfig::window`

```cpp
double window = 0.3
```

Evaluation window (seconds). PROVISIONAL (A4: HA-52).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:71`](../../include/shulib/motion/odo_stall_check.hpp#L71).*

<a id="odostallcheckconfig-minspintravel"></a>

### `OdoStallCheckConfig::minSpinTravel`

```cpp
units::Length minSpinTravel{1.0}
```

Mean wheel-implied travel that counts as "the wheels are spinning" (inches per window). PROVISIONAL (A4: HA-52).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:74`](../../include/shulib/motion/odo_stall_check.hpp#L74).*

<a id="odostallcheckconfig-motionratio"></a>

### `OdoStallCheckConfig::motionRatio`

```cpp
double motionRatio = 0.25
```

observedMotion / spinTravel below this ⇒ stalled. PROVISIONAL (A4: HA-52).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:76`](../../include/shulib/motion/odo_stall_check.hpp#L76).*

<a id="odostallcheckconfig-wheelradius"></a>

### `OdoStallCheckConfig::wheelRadius`

```cpp
units::Length wheelRadius{3.25 / 2.0}
```

Drive wheel RADIUS (inches) — converts shaft radians to surface travel. Stand-in geometry (3.25″ wheel, 1:1 gearing — A4: HA-14).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:79`](../../include/shulib/motion/odo_stall_check.hpp#L79).*

<a id="odostallcheckconfig-rotationradius"></a>

### `OdoStallCheckConfig::rotationRadius`

```cpp
units::Length rotationRadius{7.0}
```

Converts |Δheading| to equivalent wheel travel (≈ center-to-wheel distance). Stand-in geometry (A4: HA-17/HA-52).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:82`](../../include/shulib/motion/odo_stall_check.hpp#L82).*

<a id="class-odostallcheck"></a>

## `class OdoStallCheck`

```cpp
class OdoStallCheck
```

The windowed spin-vs-motion cross-check: the drive encoders say the wheels rolled, the fused estimate says the robot did not move, and sustained disagreement means the odometry is stuck. It is the only defence against a FROZEN tracking encoder, which the estimator itself cannot see — zero travel is a perfectly plausible reading, so no plausibility guard fires while the fused pose walks away from truth at exactly truth's speed. Owned per-motion and reset() at start(), because a window straddling a motion boundary would read a setPose as motion. The verdict HOLDS between window closes, so a consumer sees one sustained episode, not chatter.

*class, declared at [`include/shulib/motion/odo_stall_check.hpp:92`](../../include/shulib/motion/odo_stall_check.hpp#L92).*

<a id="odostallcheck-kmaxwheels"></a>

### `OdoStallCheck::kMaxWheels`

```cpp
static constexpr int kMaxWheels = 8
```

Fixed capacity of the per-wheel shaft baseline, mirroring kinematics::WheelSpeeds so the hot path never allocates. update() rejects a larger span outright rather than truncating.

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:96`](../../include/shulib/motion/odo_stall_check.hpp#L96).*

<a id="odostallcheck-odostallcheck"></a>

### `OdoStallCheck::OdoStallCheck`

```cpp
explicit OdoStallCheck(const OdoStallCheckConfig& config = {})
```

Copies `config` and validates every field: window finite and > 0, minSpinTravel > 0, both radii > 0, and motionRatio strictly inside (0, 1) — at 0 nothing could ever trip, at 1 any slip at all would read as a stall. A violation trips the precondition handler; nothing is clamped. The check starts with no baseline, so the first update() only baselines and no verdict can be true until a full `window` has elapsed.

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:103`](../../include/shulib/motion/odo_stall_check.hpp#L103).*

<a id="odostallcheck-update"></a>

### `OdoStallCheck::update`

```cpp
[[nodiscard]] bool update(units::Time now, std::span<hal::IMotor* const> motors, const math::Pose2d& fusedPose)
```

Feed one tick's observables; returns the current (window-held) verdict. `motors` are the drive motors in kinematic order (size constant per run).

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:118`](../../include/shulib/motion/odo_stall_check.hpp#L118).*

<a id="odostallcheck-stalled"></a>

### `OdoStallCheck::stalled`

```cpp
[[nodiscard]] bool stalled() const noexcept
```

The latest window verdict (held between window closes).

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:150`](../../include/shulib/motion/odo_stall_check.hpp#L150).*

<a id="odostallcheck-reset"></a>

### `OdoStallCheck::reset`

```cpp
void reset() noexcept
```

Forget the window baseline AND the verdict (motion start / after setPose).

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:153`](../../include/shulib/motion/odo_stall_check.hpp#L153).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 49 lines, click to expand</summary>

```text

 OdoStallCheck — the spin-vs-motion cross-check (chunk C1; A3 handoff #2).

 ── Why this exists (the A3 finding, verbatim consequence) ──────────────────────────
 A frozen tracking encoder is INVISIBLE to the M2 estimator: zero travel is a
 perfectly plausible reading, so PilonsOdometry::lastDeltaImplausible() never
 fires and the fused estimate walks away from truth at exactly the truth's
 speed (measured during the hostile-fakes campaign, and asserted by test).
 fault.hpp assigns OdoStuck to "the C/E layers"; the estimator-side detector
 is E-phase work. Until then,
 THIS windowed cross-check — owned by every C1 motion's tick — is the only
 defence against a dead encoder: the drive encoders say the wheels are rolling,
 the fused estimate says the robot is not moving. Sustained disagreement ⇒ the
 odometry is stuck ⇒ HealthMonitor::Observations::odomStalled ⇒ ODO_STUCK.

 ── The verdict (per evaluation window) ─────────────────────────────────────────────
     spinTravel     = mean_i |Δ driveShaft_i| · wheelRadius          (inches)
     observedMotion = hypot(Δx, Δy) + rotationRadius · |Δheading|    (inches)
     stalled        = spinTravel ≥ minSpinTravel
                      AND observedMotion < motionRatio · spinTravel

 Design points, each load-bearing:
   * The ROTATION TERM (rotationRadius·|Δheading|, shortest-path Δ so the ±180°
     seam cannot inflate it) is what makes a pure TurnTo immune to false
     positives: spinning in place the wheels travel ≈ R·Δθ each while the
     position stands still — without the term every turn would fault. It also
     means a frozen tracking encoder does NOT false-fault a pure turn: heading
     is IMU-owned, so a turn genuinely progresses and reports its motion even
     with dead position odometry. That is correct, not a miss — dead position
     odometry cannot hurt a pure turn, and any translation attempt still trips.
   * The RATIO (not an absolute floor) keeps the check speed-independent, with
     margin: A3's slip model still propels ≈70% of spin (HA-40), far above the
     25% default — slip degrades, a stuck estimate STOPS. A physically blocked
     robot with spinning wheels also trips; that is the same fault family
     (fault.hpp: "odometry implausible / WHEEL STUCK"), and on purpose.
   * MEAN |Δshaft| over all drive wheels: a single dead DRIVE encoder halves
     the mean rather than zeroing it (still trips), while an X-drive strafe
     (all four wheels spinning) reads full spin travel.
   * WINDOWED (default 0.3 s), not per-tick: per-tick deltas are quantization-
     noise-dominated; a window integrates real travel. The verdict HOLDS until
     the next window closes, so HealthMonitor's edge-per-episode logic sees one
     sustained episode, not chatter.

 Thresholds are PROVISIONAL (A4: HA-52) — window, minSpinTravel, motionRatio
 and the radii are invented/stand-in numbers until R3 measures geometry and R4
 measures noise floors. The register carries the falsifiable claims.

 Owned per-motion, reset() at start(): the window baseline must not straddle a
 motion boundary (a teleport/setPose between motions would look like motion).
```

</details>
