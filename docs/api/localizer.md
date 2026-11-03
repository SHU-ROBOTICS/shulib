<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/localizer.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `localizer.hpp`

Localizer — the fused field-frame estimate.

This header declares **3** types (29 members).

Extracted from [`include/shulib/localization/localizer.hpp`](../../include/shulib/localization/localizer.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct LocalizerConfig`](#struct-localizerconfig)
  - [`maxDt`](#localizerconfig-maxdt)
  - [`minDt`](#localizerconfig-mindt)
  - [`driftHorizon`](#localizerconfig-drifthorizon)
  - [`qFloor`](#localizerconfig-qfloor)
  - [`bootSettleTime`](#localizerconfig-bootsettletime)
- [`class Localizer`](#class-localizer)
  - [`kMaxCorrectors`](#localizer-kmaxcorrectors)
  - [`kTurnCounterStepRadians`](#localizer-kturncounterstepradians)
  - [`kTravelCounterStepInches`](#localizer-ktravelcounterstepinches)
  - [`Localizer`](#localizer-localizer)
  - [`update`](#localizer-update)
  - [`pose`](#localizer-pose)
  - [`twist`](#localizer-twist)
  - [`quality`](#localizer-quality)
  - [`isDeadReckoning`](#localizer-isdeadreckoning)
  - [`qualityClass`](#localizer-qualityclass)
  - [`distanceSinceCorrection`](#localizer-distancesincecorrection)
  - [`travelSinceSeat`](#localizer-travelsinceseat)
  - [`turnedSinceHeadingSeed`](#localizer-turnedsinceheadingseed)
  - [`lastCorrection`](#localizer-lastcorrection)
  - [`lastOdomDeltaImplausible`](#localizer-lastodomdeltaimplausible)
  - [`lastOdomTickAudit`](#localizer-lastodomtickaudit)
  - [`headingBias`](#localizer-headingbias)
  - [`setPose`](#localizer-setpose)
  - [`reseatHeading`](#localizer-reseatheading)
  - [`seatAxis`](#localizer-seataxis)
  - [`enum class Localizer::Quality`](#enum-class-localizer-quality)
    - [`Uninitialized`](#localizer-quality-uninitialized)
    - [`DeadReckon`](#localizer-quality-deadreckon)
    - [`Corrected`](#localizer-quality-corrected)
    - [`Degraded`](#localizer-quality-degraded)

<a id="struct-localizerconfig"></a>

## `struct LocalizerConfig`

```cpp
struct LocalizerConfig
```

Tuning for the fused estimate: the dt window the twist finite-difference is trusted over, how fast the quality scalar decays while dead-reckoning, and how long the boot settle window holds the fold closed. The Localizer constructor range-checks maxDt, driftHorizon, qFloor and bootSettleTime (red-on-failure); `minDt` is NOT checked, and nothing checks `minDt <= maxDt`, so the dt BAND is the caller's to keep sane: a floor above the ceiling empties it and every tick then silently reports zero linear velocity and Degraded quality, while a floor <= 0 disables the velocity-spike guard minDt exists to be. The drift-rate numbers are invented guesses until a real drivetrain is measured.

*struct, declared at [`include/shulib/localization/localizer.hpp:152`](../../include/shulib/localization/localizer.hpp#L152).*

<a id="localizerconfig-maxdt"></a>

### `LocalizerConfig::maxDt`

```cpp
double maxDt = 0.1
```

Above this tick dt (s), the linear-velocity finite-difference is not trusted (first tick after construction/teleport, or a loop stall) → zero linear velocity for that tick + a flagged tick.

*field, declared at [`include/shulib/localization/localizer.hpp:155`](../../include/shulib/localization/localizer.hpp#L155).*

<a id="localizerconfig-mindt"></a>

### `LocalizerConfig::minDt`

```cpp
double minDt = 1e-4
```

Below this tick dt (s), the finite-difference is likewise not trusted (a near-zero interval would otherwise blow up into an unphysical velocity spike).

*field, declared at [`include/shulib/localization/localizer.hpp:158`](../../include/shulib/localization/localizer.hpp#L158).*

<a id="localizerconfig-drifthorizon"></a>

### `LocalizerConfig::driftHorizon`

```cpp
units::Length driftHorizon{12.0}
```

distanceSinceCorrection at which the quality scalar decays to qFloor (drift erodes trust as we dead-reckon farther — process noise scales with travel). Default ~ one foot — an INVENTED drift-rate guess until R4 measures real dead-reckon drift (A4 register HA-36).

*field, declared at [`include/shulib/localization/localizer.hpp:162`](../../include/shulib/localization/localizer.hpp#L162).*

<a id="localizerconfig-qfloor"></a>

### `LocalizerConfig::qFloor`

```cpp
double qFloor = 0.2
```

Quality floor while dead-reckoning far from a fix, in [0,1).

*field, declared at [`include/shulib/localization/localizer.hpp:164`](../../include/shulib/localization/localizer.hpp#L164).*

<a id="localizerconfig-bootsettletime"></a>

### `LocalizerConfig::bootSettleTime`

```cpp
double bootSettleTime = 0.1
```

How long after a WITNESSED not-ready→ready transition the fold stays closed while the delayed sensor data path flushes its boot-boundary garbage (the settle window — header note). Applies ONLY when a not-ready phase was observed; a ready-from-construction boot takes no hold. Must cover the worst sensor data-path latency; 0.1 s clears the ~50 ms GPS-class delay with margin (adequacy vs. REAL latencies: A4 register HA-35, R4 measures).

*field, declared at [`include/shulib/localization/localizer.hpp:170`](../../include/shulib/localization/localizer.hpp#L170).*

<a id="class-localizer"></a>

## `class Localizer`

```cpp
class Localizer final : public IPoseSource
```

The fused field-frame estimate, and the IPoseSource every consumer above it reads: a deterministic five-step tick over an injected clock, IMU, IOdometry (PilonsOdometry or DriveEncoderOdometry — the R3b Part 2 seam) and a non-owning list of correctors. Position is a PERSISTENT accumulator advanced by odometry DELTAS and nudged — never snapped — toward corrector proposals (only a CALLER seeds it: setPose(), or seatAxis() for one axis — the wall reset); heading is composed from the IMU as the LAST write of every tick, so nothing below can ASSIGN a heading, only move a bounded, persistent bias. It owns no loop and raises no faults: the caller calls update() once per control tick, and pose()/twist()/quality() then describe THAT tick until the next one.

*class, declared at [`include/shulib/localization/localizer.hpp:182`](../../include/shulib/localization/localizer.hpp#L182).*

<a id="localizer-kmaxcorrectors"></a>

### `Localizer::kMaxCorrectors`

```cpp
static constexpr std::size_t kMaxCorrectors = 4
```

At most this many correctors (GPS + AI-Vision tag + Pi tag + LIDAR today) — the valid-proposal buffer is fixed-capacity so the hot path never heap-allocates.

*field, declared at [`include/shulib/localization/localizer.hpp:212`](../../include/shulib/localization/localizer.hpp#L212).*

<a id="localizer-kturncounterstepradians"></a>

### `Localizer::kTurnCounterStepRadians`

```cpp
static constexpr double kTurnCounterStepRadians = 1.0 * math::Angle::kPi / 180.0
```

The turn counter's step (turnedSinceHeadingSeed(), the wall reset's): 1 degree, in radians. The raw heading must move this far from the counter's anchor reading before the move is counted. Twenty times the register's HA-21 heading-noise sigma (0.05 degrees) — fourteen sigma of the difference of two noisy readings — so a still IMU at HA-21's noise never trips it. Past that the margin is a CLIFF, not a slope. On the host test plant (a still minute, twenty seeds) it counts 0 with noise alone up to three times HA-21's sigma. With HA-20's drift added it counts ~1 degree up to twice (the drift itself, counted as it should be) and up to ~3 at three times. At four times a still minute counts up to ~4 degrees with noise alone and up to ~31 with the drift (a reading whose mean sits mid-step crosses it back and forth); at five times up to ~100; at six times 20-245. An over-count only widens the wall reset's heading tolerance, so it refuses more, never seats wrongly; the register's HA-211 has the numbers and the still-robot capture that settles them. Small enough that a turn's uncounted remainder costs the heading tolerance at most 0.02 degrees a turn. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/localizer.hpp:227`](../../include/shulib/localization/localizer.hpp#L227).*

<a id="localizer-ktravelcounterstepinches"></a>

### `Localizer::kTravelCounterStepInches`

```cpp
static constexpr double kTravelCounterStepInches = 0.5
```

The travel counters' step (travelSinceSeat(), the wall reset's): 0.5 in. The odometry's path must move this far from the counters' anchor point before the move is counted. A still robot's odometry jitter — the register's HA-21 heading noise on a tracking wheel 4.5 in off the turning centre moves it ~0.006 in a tick, back and forth — stays far inside it: on the host test plant a still minute counts 0 up to six times HA-21's noise (twenty seeds); and a leg's uncounted remainder costs the wall reset's keep-out widening at most 0.05 in (10 % of a step). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/localizer.hpp:236`](../../include/shulib/localization/localizer.hpp#L236).*

<a id="localizer-localizer"></a>

### `Localizer::Localizer`

```cpp
Localizer(hal::IClock& clock, hal::IImu& imu, IOdometry& odom, IFusionPolicy& fusion, std::span<ICorrector* const> correctors = {}, const LocalizerConfig& config = {})
```

`correctors` is a NON-OWNING view: the backing array (and the correctors it points to) must outlive the Localizer. Empty at M2 (dead-reckon). All references are validated non-null.

*function, declared at [`include/shulib/localization/localizer.hpp:240`](../../include/shulib/localization/localizer.hpp#L240).*

<a id="localizer-update"></a>

### `Localizer::update`

```cpp
void update()
```

One fused tick (the five steps above).

*function, declared at [`include/shulib/localization/localizer.hpp:272`](../../include/shulib/localization/localizer.hpp#L272).*

<a id="localizer-pose"></a>

### `Localizer::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const noexcept override
```

The fused field-frame pose as of the last update(): x/y in INCHES from the persistent accumulator, heading in RADIANS as `imu.heading() + headingBias()`. While the IMU is still booting or settling the POSITION is frozen at its seed value (the fold is closed) while the heading keeps tracking the raw IMU, calibration garbage included — so check qualityClass() before believing this, rather than reading a plausible-looking pose that does not exist yet.

*function, declared at [`include/shulib/localization/localizer.hpp:516`](../../include/shulib/localization/localizer.hpp#L516).*

<a id="localizer-twist"></a>

### `Localizer::twist`

```cpp
[[nodiscard]] math::Twist2d twist() const noexcept override
```

Field-frame velocity: vx/vy in in/s, finite-differenced from the FUSED pose, and ω in rad/s taken straight from the IMU (0 when the IMU reads non-finite). A tick whose dt lands outside [minDt, maxDt] — a loop stall, or the tick after a teleport — reports ZERO linear velocity rather than a spike; the first tick, and any dt <= 0, keeps the previous linear velocity and refreshes only ω.

*function, declared at [`include/shulib/localization/localizer.hpp:522`](../../include/shulib/localization/localizer.hpp#L522).*

<a id="localizer-quality"></a>

### `Localizer::quality`

```cpp
[[nodiscard]] double quality() const noexcept override
```

Graded trust in [0,1], kept consistent with qualityClass(): EXACTLY 0 whenever the IMU has no heading authority (booting, settling, or lost mid-run), otherwise a drift term decaying linearly to qFloor over driftHorizon of dead-reckoned travel, halved for an unhealthy dt and halved again for an implausible odometry delta. An applied fix clears the drift term in PROPORTION to that fix's confidence, so a microscopic fix cannot spring this to 1.0.

*function, declared at [`include/shulib/localization/localizer.hpp:528`](../../include/shulib/localization/localizer.hpp#L528).*

<a id="localizer-isdeadreckoning"></a>

### `Localizer::isDeadReckoning`

```cpp
[[nodiscard]] bool isDeadReckoning() const noexcept override
```

True when no corrector proposal was applied on the most recent update(). A per-TICK answer, not a summary: it returns to true the moment a source goes quiet, and says nothing about how far the robot has dead-reckoned since (that is distanceSinceCorrection()). True before the first update().

*function, declared at [`include/shulib/localization/localizer.hpp:533`](../../include/shulib/localization/localizer.hpp#L533).*

<a id="localizer-qualityclass"></a>

### `Localizer::qualityClass`

```cpp
[[nodiscard]] Quality qualityClass() const noexcept
```

The categorical health a motion or skills gate branches on, carrying the distinction the [0,1] scalar cannot: Uninitialized means there is no live estimate YET (never on the first update(), nor before the IMU is ready and settled — Quality's note) and is what the motion layer's wait-for-live gate blocks on, while Degraded means an estimate exists and is decaying. Keeping those two apart is deliberate — a robot that had a fix and lost heading authority needs different recovery from one that is still booting.

*function, declared at [`include/shulib/localization/localizer.hpp:542`](../../include/shulib/localization/localizer.hpp#L542).*

<a id="localizer-distancesincecorrection"></a>

### `Localizer::distanceSinceCorrection`

```cpp
[[nodiscard]] units::Length distanceSinceCorrection() const noexcept
```

Inches of odometry travel accumulated since a fix was last applied — the input the quality decay is computed from. An applied fix does not zero it but SCALES it by (1 − the fix's confidence), so a weak fix barely dents it; setPose() clears it outright, and travel made while the boot fold is closed never enters it.

*function, declared at [`include/shulib/localization/localizer.hpp:547`](../../include/shulib/localization/localizer.hpp#L547).*

<a id="localizer-travelsinceseat"></a>

### `Localizer::travelSinceSeat`

```cpp
[[nodiscard]] units::Length travelSinceSeat(Axis axis) const noexcept
```

Inches of odometry path length since `axis` was last SET — seeded by setPose() (both axes) or seated by seatAxis() (that axis only) — added at L6-A1 for the wall reset. Per axis and never touched by a corrector's fix (conservative). Why it exists: the wall reset tests its keep-outs at the estimate's coordinate ALONG the wall — the one axis that reset does not measure — so it widens them by how far that axis may have drifted (10 % of this, by default; A4 register HA-211). The single drift term cannot serve: it is one number for both axes, and a one-axis seat deliberately leaves it alone. Counted in STEPS of kTravelCounterStepInches (0.5 in) along the odometry's own path: the path's straight-line distance from an anchor point is added, and the anchor moved, each time it reaches a step — so a leg counts in full but for at most one step at its end (and the step a seatAxis() lands inside may count toward that axis: over, never under). WHY STEPS, NOT EVERY TICK'S |delta| (the L6-A1 review): on tracking wheels off the turning centre, a still IMU's heading noise (the register's HA-21) moves the odometry a few thousandths of an inch each tick, back and forth about a fixed point — a per-tick sum adds that up to ~25-50 in of "travel" a minute while nothing moves, and the keep-outs widened until a still robot's reset was refused KEEP-OUT. (distanceSinceCorrection() still sums every tick, as it always has.) Bookkeeping only: no estimate reads it.

*function, declared at [`include/shulib/localization/localizer.hpp:565`](../../include/shulib/localization/localizer.hpp#L565).*

<a id="localizer-turnedsinceheadingseed"></a>

### `Localizer::turnedSinceHeadingSeed`

```cpp
[[nodiscard]] units::AngleDim turnedSinceHeadingSeed() const noexcept
```

Radians the robot has TURNED since the heading was last seeded — by setPose() or reseatHeading() — added at L6-A1. Counted from the raw IMU heading in STEPS: its |wrapped| excursion from an anchor reading is added, and the anchor moved, each time the excursion reaches kTurnCounterStepRadians (1 degree) — so a left-right wiggle adds up (each leg counts) and a 350-degree spin reads 350 degrees, never 10, while a robot standing still reads ~0 however long it stands — at the register's believed noise (HA-21; the step's own doc, kTurnCounterStepRadians, says how fast that stops holding as the noise grows past three times it). A turn counts in full but for at most one step at its end (and up to two at each reversal of direction) — under-counting by at most ~1 degree a turn; and a wobble that never strays a full step from the anchor is not counted at all (the step IS the noise floor: under it, a wobble cannot be told from a still IMU's jitter). WHY STEPS, NOT A SUM OF EVERY TICK'S CHANGE (the L6-A1 review): a still IMU's heading jitters — the register's HA-21 believes white noise of about 0.05 degrees — and a per-tick sum adds that jitter up at ~5.6 degrees a SECOND while nothing turns, so a robot that stood still for 45 s had "turned" ~250 degrees and its wall reset was refused imprecise. The wall reset grows its heading tolerance with this counter: the register's HA-84 believes heading error grows about 1 % of the rotation (A4 register HA-211 doubles it). Only ticks with the boot fold open count. Bookkeeping only: no estimate reads it.

*function, declared at [`include/shulib/localization/localizer.hpp:585`](../../include/shulib/localization/localizer.hpp#L585).*

<a id="localizer-lastcorrection"></a>

### `Localizer::lastCorrection`

```cpp
[[nodiscard]] const AppliedCorrection& lastCorrection() const noexcept
```

The last tick's applied correction AND the gate's account of why (`audit`, added at E1) — the values a record producer stamps into the §18.2 gating slots.

*function, declared at [`include/shulib/localization/localizer.hpp:590`](../../include/shulib/localization/localizer.hpp#L590).*

<a id="localizer-lastodomdeltaimplausible"></a>

### `Localizer::lastOdomDeltaImplausible`

```cpp
[[nodiscard]] bool lastOdomDeltaImplausible() const noexcept
```

Forwarding accessor for IOdometry::lastDeltaImplausible() — added at C1 (additive) so the motion loop can feed HealthMonitor's odomImplausible observable without holding the odometry itself. Raising stays POLICY: this only EXPOSES the flag; the Localizer still never raises faults (D3 at A3).

*function, declared at [`include/shulib/localization/localizer.hpp:595`](../../include/shulib/localization/localizer.hpp#L595).*

<a id="localizer-lastodomtickaudit"></a>

### `Localizer::lastOdomTickAudit`

```cpp
[[nodiscard]] diag::OdomTickAudit lastOdomTickAudit() const noexcept
```

Forwarding accessor for IOdometry::lastTickAudit() — added at DIAG1 (additive), beside the flag above: the trust gate's account of the last tick (which check, which values), which motion::tickHealthObservables hands HealthMonitor so the ODO_STUCK line can say why. Exposes only, like the flag; the Localizer still never raises.

*function, declared at [`include/shulib/localization/localizer.hpp:602`](../../include/shulib/localization/localizer.hpp#L602).*

<a id="localizer-headingbias"></a>

### `Localizer::headingBias`

```cpp
[[nodiscard]] units::AngleDim headingBias() const noexcept
```

The learned heading bias, in radians: how far the published heading sits from the raw IMU reading (E3). Exposed so a test can prove the correction ACCUMULATES rather than evaporating each tick — the M2 red team's failure mode. It is a DRIFT measure only while nothing has seeded it: reseatHeading() (COMP-a) SETS it to the caller's start heading minus the raw IMU reading, so after a competition auton starts it carries the start-heading seed (for example +90 deg, or a RELATIVE start's minus-the-pre-match-drift) plus whatever a heading corrector learned since. Zero on any tree with no heading-providing corrector and no reseatHeading() seed, exactly.

*function, declared at [`include/shulib/localization/localizer.hpp:614`](../../include/shulib/localization/localizer.hpp#L614).*

<a id="localizer-setpose"></a>

### `Localizer::setPose`

```cpp
void setPose(const math::Pose2d& p)
```

Teleport the POSITION (x, y); heading stays IMU-owned. Forwards to IOdometry::setPose so the predictor and the fused belief never diverge, and re-baselines twist + dt so the teleport injects no phantom velocity next tick.  E3: the learned heading bias is KEPT, deliberately. A teleport says where the robot IS, not which way the IMU is wrong; discarding a bias that took a second of tag sightings to learn, every time a routine re-seeds its position, would throw away the correction at exactly the moments a routine cares most. `p.heading()` is still ignored, as it always was — the heading's own seed is reseatHeading(), below; a caller seeding both calls the two on the same tick (competition::runAuton does).  L6-A1: a seed of BOTH axes and of the heading's reference, so it also zeroes the wall reset's counters — travelSinceSeat() for x and y, and turnedSinceHeadingSeed() (the IMU is read ONCE here, as before; that one reading is also the turn counter's new reference).

*function, declared at [`include/shulib/localization/localizer.hpp:632`](../../include/shulib/localization/localizer.hpp#L632).*

<a id="localizer-reseatheading"></a>

### `Localizer::reseatHeading`

```cpp
void reseatHeading(math::Angle heading)
```

Seed the HEADING: make the published heading equal `heading` on this tick, by setting the bias so that `imu.heading() + bias == heading` (the shortest signed difference, so the bias stays within half a turn). The heading twin of setPose() — a CALLER seed, never a correction (header: THE HEADING BIAS) — added at COMP-a so an auton's declared start heading (and a "relative" start's zero) replaces whatever the IMU read at boot and drifted to while the robot waited. The IMU stays the sole source of every heading CHANGE after it.  It re-baselines EXACTLY as setPose() does: twist cleared and the next tick's dt made 0. That is load-bearing, not tidiness. A fusion policy that sizes its heading process noise from the change in the handed heading (EkfFusion's step A) would otherwise read this seed as a real rotation of the whole difference, and a 90-degree seed would inflate its heading uncertainty as if the robot had spun; the dt-0 tick instead re-bases the filter onto the new heading. The position, its trust (distanceSinceCorrection) and the odometry are untouched — the wheels' deltas were measured against the RAW IMU heading and are re-aimed by the bias every tick (STEP 2), so position follows the new heading from the next tick on.  Preconditions: the estimate is LIVE (qualityClass() != Uninitialized) — during the IMU's boot the raw heading is calibration garbage that moves, and a bias computed from it would be wrong the moment calibration ends; and, by contract since this class cannot see it, NO MOTION is active — a motion mid-flight would see its heading error jump under it (competition::runAuton checks that before it calls this). Call it on the same tick as a setPose() that seeds the position, with no update() between them. L6-A1: it also restarts the wall reset's turn counter (turnedSinceHeadingSeed()), and leaves the travel counters.

*function, declared at [`include/shulib/localization/localizer.hpp:675`](../../include/shulib/localization/localizer.hpp#L675).*

<a id="localizer-seataxis"></a>

### `Localizer::seatAxis`

```cpp
void seatAxis(Axis axis, units::Length value)
```

SEAT ONE POSITION AXIS (chunk L6-A1, the wall reset): make the estimate's `axis` coordinate equal `value` on this tick, and change NOTHING else — not the other axis, not the heading, not the heading bias, not the drift term (distanceSinceCorrection), not the quality, not the last correction, and not the odometry. A caller act like setPose(), never a correction — the one exception locked decision #4 (never snap) carries, as amended by the team lead on 2026-10-22: a caller-invoked ONE-AXIS wall reset, while no motion is active, inside gates whose largest jump is the author's maxCorrection (chassis::resetFromWall checks every one before it calls this). No sensor seats the heading — here or anywhere.  WHY NOT setPose() WITH THE OTHER AXIS COPIED: setPose() clears the drift term, so from the second tick after it the quality reads DeadReckon (1.000) where a one-axis measurement should leave it Degraded (0.200) — one measured axis passing as two (probed while the wall reset was designed). One axis measured is not a fix, so the drift term, and with it the quality, stay.  WHAT IT DOES: the axis jumps (`fusedX_` or `fusedY_`), the fused-pose history is re-taken (as setPose() does), the published pose is recomposed with the CURRENT published heading (the IMU is not read), the twist is cleared, that axis's travelSinceSeat() restarts — and the next tick's dt is made 0. That last is LOAD-BEARING, not tidiness: EkfFusion keeps its own position state and recovers each tick's odometry increment as (handed prediction − its last answer), so without the dt-0 re-base it reads the seat as one tick's TRAVEL — on the host tests, 1.68 in of a 2-in seat bounced back and the filter's velocity read 167.6 in/s. The dt-0 tick resyncs it onto the seated prediction instead.  WHY THE ODOMETRY IS NOT TOUCHED (setPose() forwards to IOdometry::setPose): nothing reads an odometry's absolute pose but this class, and it reads only per-tick DELTAS (STEP 2), so the seated axis simply advances from its new value; IOdometry::setPose would also re-take the odometry's heading reference (its prevHeading), a side effect a one-axis seat has no reason to cause. setPose()'s "the predictor and the fused belief never diverge" is not an invariant this class keeps elsewhere either: an applied correction already lets them diverge, by design (STEP 2's note).  Preconditions: the estimate is LIVE (qualityClass() != Uninitialized — reseatHeading()'s rule) and `value` is finite; and, by contract since this class cannot see the scheduler, NO MOTION is active — a motion mid-flight would see its position error jump under it (the master plan's risk 5, "corrector pose-jump mid-motion"); the wall reset refuses MotionActive before it samples.

*function, declared at [`include/shulib/localization/localizer.hpp:724`](../../include/shulib/localization/localizer.hpp#L724).*

<a id="enum-class-localizer-quality"></a>

## `enum class Localizer::Quality`

```cpp
enum class Quality
```

Categorical health for motion/skills gating (distinct from the [0,1] scalar). The order below is declaration order, NOT a ranking — `Degraded` is worse than `DeadReckon` despite sorting after it, so compare by enumerator and never by value.

*enum class, declared at [`include/shulib/localization/localizer.hpp:187`](../../include/shulib/localization/localizer.hpp#L187).*

<a id="localizer-quality-uninitialized"></a>

### `Localizer::Quality::Uninitialized`

```cpp
Uninitialized
```

No live estimate yet: no update() has run, or only ONE has, or the IMU has never been ready, or the boot settle window is still open. The very first update() never makes the estimate live — the class is judged BEFORE that call marks the estimate as ever-updated (refreshQuality() runs ahead of the mark), and that call has no earlier tick to take a dt against — so a Localizer whose IMU is ready from the start is live from its SECOND update(). Distinct from Degraded on purpose — a consumer can tell "not started" from "started and lost it".

*enumerator, declared at [`include/shulib/localization/localizer.hpp:195`](../../include/shulib/localization/localizer.hpp#L195).*

<a id="localizer-quality-deadreckon"></a>

### `Localizer::Quality::DeadReckon`

```cpp
DeadReckon
```

Running on odometry alone, within the configured drift horizon. Healthy: no corrector has proposed recently, and the estimate has not yet dead-reckoned far enough for that to matter.

*enumerator, declared at [`include/shulib/localization/localizer.hpp:199`](../../include/shulib/localization/localizer.hpp#L199).*

<a id="localizer-quality-corrected"></a>

### `Localizer::Quality::Corrected`

```cpp
Corrected
```

The best state: a corrector proposal was folded in this tick and every health check passed. This is the only class that means an absolute reference is live.

*enumerator, declared at [`include/shulib/localization/localizer.hpp:202`](../../include/shulib/localization/localizer.hpp#L202).*

<a id="localizer-quality-degraded"></a>

### `Localizer::Quality::Degraded`

```cpp
Degraded
```

Trust the pose less. Reached four different ways, all of which mean the same thing to a caller: the IMU was ready and stopped being ready, the odometry reported an implausible delta, the tick's dt was outside the trusted band, or dead reckoning has run past `driftHorizon`.

*enumerator, declared at [`include/shulib/localization/localizer.hpp:207`](../../include/shulib/localization/localizer.hpp#L207).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 119 lines, click to expand</summary>

```text

 Localizer — the fused field-frame estimate (master plan §5/§6/§8; WS5). It is a thin, deterministic
 orchestrator over three already-tested pieces: an IOdometry (PilonsOdometry over tracking
 wheels, or DriveEncoderOdometry over the drive motors — high-rate prediction; the seam
 landed at R3b Part 2 and this class depends on nothing past its five members), the IMU
 (heading authority), and a list of correctors (absolute fixes) behind an IFusionPolicy. It owns
 the four jobs the pieces below cannot, in a fixed five-step update():
   1. dt — source it from the injected IClock and turn the per-tick position change into a Twist2d.
   2. predict — advance the IOdometry; its pose is the dead-reckon prediction.
   3. gather — ask each corrector for an absolute proposal and keep the VALID ones.
   4. fuse — fold them in as an innovation-bounded, per-tick-clamped GATED NUDGE, via the
      IFusionPolicy.
   5. heading + publish — compose the fused heading from the IMU as the LAST write of the tick, so no
      corrector or policy can ever assign the robot a heading (decision #4). Since E3 the
      composition is `imu.heading() + headingBias_`, where the bias moves by at most a bounded
      nudge per tick — see the heading-bias note below.
      …then recompute the quality scalar + categorical flags from observable inputs.

 The numbering above matches the STEP labels in update() exactly. It used to be off by one
 from step 3 onward — the banner's "fuse" was the code's STEP 4, the banner's "heading" was
 STEP 5, and the banner's separate step 5 had no label at all — so a reader arriving from the
 generated page and jumping to the source landed on the wrong step every time. The banner was
 also inconsistent with itself downstream, where the heading-bias note cites "STEP 4" using
 the old numbering for what the code calls STEP 5.

 ── THE HEADING BIAS (added at E3, the absolute-yaw path M2 reserved) ──
 Until E3 nothing in the tree could tell the estimator its heading was wrong, so STEP 5 stamped
 the raw IMU reading and heading was IMU-owned in the strongest possible sense. That was correct
 while it lasted and is NOT what the accuracy spec needs: the master plan records that a raw V5
 IMU drifts ≈1°/min, so dead-reckoned heading alone will not hold the team's `< 1°` requirement
 across a 60-second run, and makes "IMU-owned heading + absolute yaw correction" load-bearing.
 AprilTagCorrector supplies the absolute heading; this is where it lands.

 The published heading is `imu.heading() + headingBias_`. The IMU remains the SOLE source of
 heading CHANGE — every rotation the robot makes still enters through imu.heading(), tick for
 tick, and PilonsOdometry is untouched — while the bias is a slowly-learned constant offset that
 moves by at most `maxHeadingNudgeRate · dt` per tick (ComplementaryFusion). No CORRECTOR and no
 fusion policy can ASSIGN a heading, here or anywhere below: the policy can only return an
 increment. That is what makes never-snap (§13 #4) structural for yaw rather than a promise.

 The one assignment that exists is a CALLER SEED, and it is not a correction: reseatHeading()
 (COMP-a, 2026-09-26) is the heading twin of setPose(). A caller who KNOWS which way the robot
 faces — the competition program, at the instant an auton starts, from the auton's declared start
 pose — sets the bias so the published heading equals that value on this tick; the IMU stays the
 only source of every change after it. Never-snap is a rule about what a SENSOR may do to a live
 estimate, and it still holds: nothing that reads a sensor can reach reseatHeading().
 THE ONE POSITION EXCEPTION (chunk L6-A1, 2026-10-29 — locked decision #4 as AMENDED by the team
 lead, 2026-10-22): seatAxis() below lets a CALLER — the wall reset (localization/wall_reset.hpp,
 chassis/wall_reset_call.hpp) — write ONE position axis from a side distance sensor's reading, while
 no motion is active and only inside gates whose largest jump is the author's maxCorrection. It is a
 caller act like setPose(), never a corrector's, and no sensor can reach the HEADING through it or
 anywhere else (the wall reset never calls reseatHeading()).
 (Decision-consistency check, logged in the COMP-a progress record. The sentence this paragraph
 replaced — "there is no code path, here or anywhere below, that ASSIGNS a heading" — had three
 echoes, each re-read: apriltag_corrector.hpp's "no code path HERE writes a pose or a heading"
 (about the corrector; still true word for word), complementary_fusion.hpp's "a yaw reset can
 never happen" on maxHeadingNudgeRate (about the fusion path; now says so), and guide chapter 3's
 "no code path anywhere assigns the robot a heading" (reworded with this change). No test
 asserted the stronger reading; the never-snap tests exercise correctors, which still cannot.
 That first check grepped for the ASSIGNMENT sentence only and missed its CONSEQUENCES — the
 claims that the bias is exactly zero without a heading corrector. A review of COMP-a found
 them (five), and a second grep — "heading-providing corrector|no-heading-corrector|EXACTLY
 0.0|bit-identical" over include/ and the public docs — found one more; all six now say "no
 heading-providing corrector AND no reseatHeading() seed": the position note below
 ("BIT-IDENTICAL to before E3"), STEP 2's early-out comment (the sixth), headingBias()'s doc
 (which also called the bias a drift measure), biasedHeading()'s doc, the headingBias_
 member's comment, and ekf_fusion.hpp's "bit-identical to the raw IMU" on the θ gain. Every
 test asserting a zero bias builds a tree with no seed, so each is still true.)

 WHY THE BIAS MUST PERSIST, which is the whole design. The Localizer re-reads the IMU every tick,
 so a heading correction that only decorated THIS tick's published heading would be discarded on
 the next one — individually sane nudges, collectively useless. That is verbatim the M2 red
 team's "corrections not accumulating" failure mode. The accumulator is exactly the structure
 STEP 2 already uses for position (fusedX_ persists and is advanced by odom DELTAS, never reset
 to absolute odom). It is deliberately NOT capped: the nudge is always toward an absolute
 measurement, so the loop is closed and cannot run away, whereas a cap would silently lock out
 recovery from precisely the large real drift that makes the correction worth having — the same
 gate-lockout failure E2's D2 identified for position.

 AND THE CONSEQUENCE FOR POSITION. PilonsOdometry rotates its per-tick field delta by the RAW
 IMU heading. Left alone, a learned bias `b` would fix the reported heading while the position
 prediction kept accumulating a cross-track error of roughly `b × distance` — which is most of
 what heading drift actually costs over a run. So STEP 2 rotates the odometry delta by `b` before
 folding it. At `b == 0` this is skipped by an explicit early-out, so a tree with no
 heading-providing corrector AND no reseatHeading() seed is BIT-IDENTICAL to before E3 by
 construction, not by a floating-point argument. (A seed makes `b` non-zero on purpose — the
 start heading lives in it — and from then on the delta IS rotated, which is the point.)

 Swapping the complementary filter for an EKF later is a one-argument change (IFusionPolicy); the
 IPoseSource read seam and the ICorrector write seam never move. At M2 the corrector list is empty
 or holds only NullCorrector, so the default-tested path is dead-reckoning (odom + IMU).

 ── The IMU-readiness boot guard (added at A3, after the hostile fakes drew blood) ──
 A calibrating IMU emits GARBAGE THAT MOVES, and the odometry offset correction converts garbage
 heading swings into phantom translation (Δθ·offset per tick) — observed at A3: a robot sitting
 STILL through a 2 s calibration window ended 10.8 inches from where it started, permanently
 (no corrector at M2 can heal it), while quality honestly said 0. The flag was honest; the pose
 was not. So update() now distinguishes three IMU states:
   * NEVER been ready (boot): odometry still consumes its wheel deltas tick-by-tick (so the
     transition tick sees one tick's travel, not the whole boot's), but the fused position folds
     NOTHING — no odom deltas, no corrector proposals. Quality: Uninitialized, 0.
   * THE SETTLE WINDOW (the second A3 finding, caught only by the COMPOSED hostile model):
     isReady() is a status flag; the heading STREAM is a data path with its own latency. When
     calibration garbage and sensor latency are both live, ready flips true while the stream is
     still serving delayed garbage — observed: one post-transition fold differenced against a
     delayed-garbage prevHeading leaked 3.65 in. So after a WITNESSED not-ready phase, the fold
     stays closed for `bootSettleTime` past the first ready tick (quality stays Uninitialized —
     the estimate is not live yet). A boot that was ready from the very first update() has no
     boundary in its stream and takes no settle hold, so the normal path is byte-identical to
     before. Consequence, stated as the consumer contract: motion commanded before qualityClass
     leaves Uninitialized is unaccounted — C1's loop waits for a live estimate (real autons wait
     out calibration anyway).
   * ready: normal operation.
   * WAS ready, lost mid-run (dropout): deltas KEEP folding — the encoders are still good and a
     stale-heading estimate beats a frozen one — but quality reports Degraded (NOT Uninitialized:
     a robot that had a fix and lost its heading authority must be distinguishable from one still
     booting, or a skills gate applies the wrong recovery) with the scalar pinned to 0.
 PilonsOdometry itself stays readiness-blind on purpose — its header assigns recovery policy to
 this layer, and a second gate below would hide this one's absence from the tests.
```

</details>
