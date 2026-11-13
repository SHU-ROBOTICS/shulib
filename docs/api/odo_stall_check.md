<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/odo_stall_check.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `odo_stall_check.hpp`

OdoStallCheck — the spin-vs-motion cross-check.

This header declares **2** types (15 members) and **2** constants.

Extracted from [`include/shulib/motion/odo_stall_check.hpp`](../../include/shulib/motion/odo_stall_check.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kOdoStallMaxWheels`](#kodostallmaxwheels) — *constant*
- [`kNoIndependentStallSourceNote`](#knoindependentstallsourcenote) — *constant*
- [`struct OdoStallCheckConfig`](#struct-odostallcheckconfig)
  - [`window`](#odostallcheckconfig-window)
  - [`minSpinTravel`](#odostallcheckconfig-minspintravel)
  - [`motionRatio`](#odostallcheckconfig-motionratio)
  - [`wheels`](#odostallcheckconfig-wheels)
  - [`rotationRadius`](#odostallcheckconfig-rotationradius)
  - [`independentMotionSource`](#odostallcheckconfig-independentmotionsource)
  - [`setAllWheels`](#odostallcheckconfig-setallwheels)
- [`class OdoStallCheck`](#class-odostallcheck)
  - [`kMaxWheels`](#odostallcheck-kmaxwheels)
  - [`OdoStallCheck`](#odostallcheck-odostallcheck)
  - [`update`](#odostallcheck-update)
  - [`stalled`](#odostallcheck-stalled)
  - [`canDetectStall`](#odostallcheck-candetectstall)
  - [`lastSpinTravel`](#odostallcheck-lastspintravel)
  - [`lastObservedMotion`](#odostallcheck-lastobservedmotion)
  - [`reset`](#odostallcheck-reset)

<a id="kodostallmaxwheels"></a>

## `kOdoStallMaxWheels`

```cpp
inline constexpr int kOdoStallMaxWheels = 8
```

Fixed capacity of the per-wheel shaft baseline and geometry table, mirroring kinematics::WheelSpeeds::kMaxWheels so the hot path never allocates. Named at namespace scope so the config's array can be sized before the class is declared.

*constant, declared at [`include/shulib/motion/odo_stall_check.hpp:96`](../../include/shulib/motion/odo_stall_check.hpp#L96).*

<a id="knoindependentstallsourcenote"></a>

## `kNoIndependentStallSourceNote`

```cpp
inline constexpr const char* kNoIndependentStallSourceNote = "ODO_STUCK stall detection has NO INDEPENDENT motion source on this robot: the odometry " "is the drive encoders (no tracking wheels), so the wheels-vs-odometry cross-check is a " "tautology and is NOT wired to report a verdict; a stuck robot cannot be seen by it. The " "encoder-vs-IMU heading cross-check is exposed as an observable instead."
```

The one boot-log line for a drivetrain whose stall check has no independent motion source (the §2 ruling above): the composition root logs it ONCE, the panel shows it, and the hardware register carries the entry. Named here so the log line and the header's reasoning cannot drift apart.

*constant, declared at [`include/shulib/motion/odo_stall_check.hpp:102`](../../include/shulib/motion/odo_stall_check.hpp#L102).*

<a id="struct-odostallcheckconfig"></a>

## `struct OdoStallCheckConfig`

```cpp
struct OdoStallCheckConfig
```

The knobs of the spin-vs-motion cross-check, taken BY VALUE at construction (editing the struct afterwards does nothing to a live check) and every one of them validated by that constructor. Every default is PROVISIONAL: the geometry is stand-in and the three thresholds are invented numbers — none has yet been measured against a real drivetrain or a real noise floor, so treat a default as a placeholder that compiles, not as a tuning.

*struct, declared at [`include/shulib/motion/odo_stall_check.hpp:113`](../../include/shulib/motion/odo_stall_check.hpp#L113).*

<a id="odostallcheckconfig-window"></a>

### `OdoStallCheckConfig::window`

```cpp
double window = 0.3
```

Evaluation window (seconds). PROVISIONAL (A4: HA-52).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:115`](../../include/shulib/motion/odo_stall_check.hpp#L115).*

<a id="odostallcheckconfig-minspintravel"></a>

### `OdoStallCheckConfig::minSpinTravel`

```cpp
units::Length minSpinTravel{1.0}
```

Mean wheel-implied travel that counts as "the wheels are spinning" (inches per window). PROVISIONAL (A4: HA-52).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:118`](../../include/shulib/motion/odo_stall_check.hpp#L118).*

<a id="odostallcheckconfig-motionratio"></a>

### `OdoStallCheckConfig::motionRatio`

```cpp
double motionRatio = 0.25
```

observedMotion / spinTravel below this ⇒ stalled. PROVISIONAL (A4: HA-52).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:120`](../../include/shulib/motion/odo_stall_check.hpp#L120).*

<a id="odostallcheckconfig-wheels"></a>

### `OdoStallCheckConfig::wheels`

```cpp
std::array<hal::DriveGeometry, static_cast<std::size_t>(kOdoStallMaxWheels)> wheels{}
```

Per-WHEEL drive geometry (kinematic wheel order) — converts each wheel's shaft radians to surface travel; the SAME type (and, at a composition root, the same objects) the drive-encoder odometry uses. Every slot defaults to the stand-in geometry (3.25 in wheel, 1:1 — A4: HA-14); setAllWheels() is the symmetric one-liner; an asymmetric drivetrain sets its slots individually.

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:126`](../../include/shulib/motion/odo_stall_check.hpp#L126).*

<a id="odostallcheckconfig-rotationradius"></a>

### `OdoStallCheckConfig::rotationRadius`

```cpp
units::Length rotationRadius{7.0}
```

Converts |Δheading| to equivalent wheel travel (≈ center-to-wheel distance). Stand-in geometry (A4: HA-17/HA-52).

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:129`](../../include/shulib/motion/odo_stall_check.hpp#L129).*

<a id="odostallcheckconfig-independentmotionsource"></a>

### `OdoStallCheckConfig::independentMotionSource`

```cpp
bool independentMotionSource = true
```

True (the default) when the odometry has a motion source INDEPENDENT of the drive wheels (tracking wheels), so wheels-vs-odometry disagreement means something. FALSE for a drivetrain whose odometry IS the drive encoders (DriveEncoderOdometry): the verdict is then never true — the header's §2 ruling — and canDetectStall() reports it.

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:134`](../../include/shulib/motion/odo_stall_check.hpp#L134).*

<a id="odostallcheckconfig-setallwheels"></a>

### `OdoStallCheckConfig::setAllWheels`

```cpp
void setAllWheels(hal::DriveGeometry geometry) noexcept
```

The symmetric drivetrain in one line: every wheel slot gets `geometry`.

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:137`](../../include/shulib/motion/odo_stall_check.hpp#L137).*

<a id="class-odostallcheck"></a>

## `class OdoStallCheck`

```cpp
class OdoStallCheck
```

The windowed spin-vs-motion cross-check: the drive encoders say the wheels rolled, the fused estimate says the robot did not move, and sustained disagreement means the odometry is stuck. It is the only defence against a FROZEN tracking encoder, which the estimator itself cannot see — zero travel is a perfectly plausible reading, so no plausibility guard fires while the fused pose walks away from truth at exactly truth's speed. Owned per-motion and reset() at start(), because a window straddling a motion boundary would read a setPose as motion. The verdict HOLDS between window closes, so a consumer sees one sustained episode, not chatter. On a drivetrain whose odometry IS the wheels the verdict is structurally meaningless and is never reported (independentMotionSource = false; header).

*class, declared at [`include/shulib/motion/odo_stall_check.hpp:149`](../../include/shulib/motion/odo_stall_check.hpp#L149).*

<a id="odostallcheck-kmaxwheels"></a>

### `OdoStallCheck::kMaxWheels`

```cpp
static constexpr int kMaxWheels = kOdoStallMaxWheels
```

Fixed capacity of the per-wheel shaft baseline (kOdoStallMaxWheels). update() rejects a larger span outright rather than truncating.

*field, declared at [`include/shulib/motion/odo_stall_check.hpp:153`](../../include/shulib/motion/odo_stall_check.hpp#L153).*

<a id="odostallcheck-odostallcheck"></a>

### `OdoStallCheck::OdoStallCheck`

```cpp
explicit OdoStallCheck(const OdoStallCheckConfig& config = {})
```

Copies `config` and validates every field: window finite and > 0, minSpinTravel > 0, rotationRadius > 0, EVERY wheel geometry valid (finite, radius > 0, ratio > 0 — an UNSET geometry is refused, never scaled by zero), and motionRatio strictly inside (0, 1) — at 0 nothing could ever trip, at 1 any slip at all would read as a stall. A violation trips the precondition handler; nothing is clamped. The check starts with no baseline, so the first update() only baselines and no verdict can be true until a full `window` has elapsed.

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:162`](../../include/shulib/motion/odo_stall_check.hpp#L162).*

<a id="odostallcheck-update"></a>

### `OdoStallCheck::update`

```cpp
[[nodiscard]] bool update(units::Time now, std::span<hal::IMotor* const> motors, const math::Pose2d& fusedPose)
```

Feed one tick's observables; returns the current (window-held) verdict. `motors` are the drive motors in kinematic order (size constant per run), NON-EMPTY and all non-null — the same discipline every other span-taking fan-out in the tree keeps (MotorMechanism, PneumaticMechanism, RobotContext, Localizer). Both checks were missing, and the empty case was the dangerous one: with no motors the mean shaft delta is 0, so spinTravel is 0, so `spinTravel >= minSpinTravel` is never true and the check reports "healthy" forever. A misconfiguration that silently disables a safety cross-check is exactly what this library's precondition discipline exists to turn into a loud failure. The in-tree path (ctx.driveMotors()) was already safe; a direct caller — which the generated reference invites — was not.

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:188`](../../include/shulib/motion/odo_stall_check.hpp#L188).*

<a id="odostallcheck-stalled"></a>

### `OdoStallCheck::stalled`

```cpp
[[nodiscard]] bool stalled() const noexcept
```

The latest window verdict (held between window closes). Always false on a check configured without an independent motion source.

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:229`](../../include/shulib/motion/odo_stall_check.hpp#L229).*

<a id="odostallcheck-candetectstall"></a>

### `OdoStallCheck::canDetectStall`

```cpp
[[nodiscard]] bool canDetectStall() const noexcept
```

Whether this check CAN ever report a stall: the configured independentMotionSource. A composition root logs the note below once when this is false; a panel shows it.

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:234`](../../include/shulib/motion/odo_stall_check.hpp#L234).*

<a id="odostallcheck-lastspintravel"></a>

### `OdoStallCheck::lastSpinTravel`

```cpp
[[nodiscard]] units::Length lastSpinTravel() const noexcept
```

Mean wheel-implied travel (inches) over the last CLOSED window — the observable the verdict is computed from, exposed so the odometry's implied travel can be checked against it (one geometry, two consumers) and so a panel can show it. 0 until a window has closed.

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:240`](../../include/shulib/motion/odo_stall_check.hpp#L240).*

<a id="odostallcheck-lastobservedmotion"></a>

### `OdoStallCheck::lastObservedMotion`

```cpp
[[nodiscard]] units::Length lastObservedMotion() const noexcept
```

Observed motion (inches: hypot(Δx, Δy) + rotationRadius·|Δheading|) over the last CLOSED window. 0 until a window has closed.

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:246`](../../include/shulib/motion/odo_stall_check.hpp#L246).*

<a id="odostallcheck-reset"></a>

### `OdoStallCheck::reset`

```cpp
void reset() noexcept
```

Forget the window baseline AND the verdict (motion start / after setPose).

*function, declared at [`include/shulib/motion/odo_stall_check.hpp:251`](../../include/shulib/motion/odo_stall_check.hpp#L251).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 77 lines, click to expand</summary>

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
     spinTravel     = mean_i ( |Δ driveShaft_i| · inchesPerRadian_i )    (inches)
     observedMotion = hypot(Δx, Δy) + rotationRadius · |Δheading|        (inches)
     stalled        = independentMotionSource
                      AND spinTravel ≥ minSpinTravel
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
   * MEAN |Δshaft| over all drive wheels: a single dead DRIVE encoder leaves
     (n-1)/n of the mean rather than zeroing it (still trips) — 1/2 on a
     2-wheel tank, 3/4 on the X-drive named below, 2/3 on the C3 H-bot. This
     bullet used to say "halves", which is the n=2 answer stated as if it were
     general, in the header of a check whose flagship consumers have 3 and 4
     wheels. The conclusion ("still trips") holds and is in fact STRONGER than
     the wrong figure implied. An X-drive strafe (all four wheels spinning)
     reads full spin travel.
   * WINDOWED (default 0.3 s), not per-tick: per-tick deltas are quantization-
     noise-dominated; a window integrates real travel. The verdict HOLDS until
     the next window closes, so HealthMonitor's edge-per-episode logic sees one
     sustained episode, not chatter.
   * PER-WHEEL GEOMETRY (R3b Part 2): the shaft-to-inches conversion is one
     hal::DriveGeometry PER WHEEL — the SAME type, and at the composition root the
     SAME objects, the drive-encoder odometry converts with. It used to be one
     scalar `wheelRadius` for every drive motor, which A29 predicted would be
     wrong on a geared robot and which could not express an asymmetric drivetrain
     at all. The symmetric case is still one line: setAllWheels(geometry).

 ── NO INDEPENDENT MOTION SOURCE — the R3b Parts 1–3 §2 ruling ─────────────────────
 This check compares the WHEELS against the ODOMETRY. With DriveEncoderOdometry the
 odometry IS the wheels, so the comparison is a tautology: a robot pushing against a
 wall with its wheels spinning reports wheel travel AND the same "observed" travel,
 and the check would say "not stalled" forever — the health monitor would be lying.
 Do not feed it a fake. `independentMotionSource = false` declares the fact: the
 verdict is then never true, canDetectStall() says so, the observables
 (lastSpinTravel / lastObservedMotion) still compute for the panel, and the
 composition root logs kNoIndependentStallSourceNote ONCE at boot. The heading
 cross-check the drive-encoder odometry exposes (encoder ω vs IMU ω) is what a
 future stuck/slip detector for such a chassis is built on; it is NOT this check.
 (A4 register HA-131: on robot two a stuck robot is invisible to the health monitor.)
 Rejected: leaving the check wired on such a robot (a monitor reporting a verdict it
 cannot make); and refusing to construct (every motion builds this from its config
 and the robot must still drive).

 Thresholds are PROVISIONAL (A4: HA-52) — window, minSpinTravel, motionRatio
 and the radii are invented/stand-in numbers until R3 measures geometry and R4
 measures noise floors. The register carries the falsifiable claims.

 Owned per-motion, reset() at start(): the window baseline must not straddle a
 motion boundary (a teleport/setPose between motions would look like motion).
```

</details>
