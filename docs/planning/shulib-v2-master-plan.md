# shulib v2 — Master Plan & Architecture Reference

> **Status:** DESIGN-FIRST (no code yet). Living document — sections marked _PENDING_ are
> still in progress. Last updated 2026-06.
>
> **What this is:** a design & architecture reference for shulib — the plan, not the robot code itself.

---

## Table of Contents
1. Mission & Thesis
2. Context & Constraints
3. Locked Decisions
4. Why shulib Beats LemLib & OkapiLib
5. Architecture (Layered)
6. Module Map
7. Canonical Conventions (frame, units, accuracy targets)
8. Subsystem Designs
9. Roadmap (clean-room build) — full breakdown in roadmap.md
10. The Onboard-Estimation Story & Award Narrative
11. Red-Team Gates & v1 Cut-List
12. Accuracy Risks to Watch
13. Open Decisions / TBD
14. Skills Strategy & Routing
15. The One-Stop-Shop Capability Catalog (past · present · future)
16. VexBuilder Integration Contract
17. Accessibility & Progressive Disclosure (non-coder teams)
18. Diagnostics & Observability
19. Glossary & References

---

## 1. Mission & Thesis

Rework `shulib` (PROS / VEX V5 / C++20) into a motion + autonomy library that is **genuinely
better than LemLib** for this team's **holonomic** robots, delivering **extremely accurate**
autonomous for the **VEX U Autonomous Coding Skills Challenge** (Override, 2026-27).

**Thesis.** LemLib is structurally **tank/differential-only** — every motion ends in
`desaturate(lateral, angular) → left/right wheels`, a model that **cannot strafe** and therefore
cannot drive a 24" X-drive or 15" H-drive — and its odometry is tracking-wheel + IMU
dead-reckoning with **no absolute correction**, so error grows unbounded over a 60s run.
shulib v2 inverts both:

- **One `IKinematics` abstraction** makes the universal motion currency a field-frame twist
  `(vx, vy, ω)`, so the **same** motion code drives X-drive / H-drive / tank, and **translation
  is decoupled from heading** (strafe into a goal while facing an AprilTag — impossible on a
  differential drive).
- **A fused `Localizer`** (Pilons arc odom + **IMU-owned heading** + **V5 GPS** + **AprilTag**,
  Mahalanobis-gated) **bounds drift across the full minute**, exploiting the GPS code strip that
  rule `<RSC4>` guarantees on the Auto-Skills field.
- All atop a **PROS-free, dependency-inverted Hardware Abstraction Layer**, so the core is
  **host-unit-testable** and **VexBuilder-sim-runnable** — neither of which LemLib is — with
  **profiled + kS/kV/kA feedforward** control for speed-independent accuracy.

---

## 2. Context & Constraints

- **Division:** VEX U (VURC). **Two robots per team**, used together (incl. Skills, `<VURS2>`):
  - **24" robot — X-drive** (4 omni @ 45°, full holonomic).
  - **15" robot — H-drive** (tank + transverse strafe wheel; limited strafe authority).
- **Sensors (full stack, both robots):** V5 IMU, V5 Rotation sensors w/ dedicated tracking
  wheels, **V5 GPS**, **AI Vision**, Optical, Distance. VEX U also legalizes **onboard compute**
  (Raspberry Pi / Coral, `<VUR12>`) and **LIDAR/spinning sensors** (`<VUG3>`).
- **Game / Skills scoring (Override):** red/blue Pin = **5**; yellow Pin = **10** _if Owned_
  (quadrant Toggle set to your color, or robot ends in the Midfield for Midfield yellows);
  Robot in the **Midfield** at end = **8**. Objects: **Pins** (two colored halves) and **Cups**
  (transparent + opaque) that **nest** into stacks on **Goals**. **4 Toggles** (one/quadrant).
  Match-loads intaken from **Loaders**. Auto Coding Skills = **1:00** fully autonomous.
- **VEX U skills match loads** (`<VURS1>`): 4 red/yellow Pins, 6 blue/yellow Pins, 10 Cups.
- **Ecosystem:** shulib is the runtime half of a **two-tool** ecosystem. **VexBuilder** (a separate,
  in-progress app) is the single authoring tool — robot design, path planning, and a (planned) sim,
  all saved in one **`.vexbot`** project file. shulib must be **standalone-usable**; we design only
  the **seams** (HAL, the `.vexbot` config+paths it reads, `SHUL/2` telemetry). No authoring/sim
  inside shulib.
- **Build window:** pre-season, **~2–3 months dedicated build time** → **clean-room v2 build**
  (no legacy auton to protect; replace, don't patch).

---

## 3. Locked Decisions

| Decision | Choice |
|---|---|
| Build scope | **Tier it** — full ambition is the north star, proven subset ships first (complementary filter → EKF; trapezoid → S-curve; GPS-only → +AprilTag). |
| Vision backend | **Both** — a VEX-U Raspberry Pi (student CV) **and** the V5 AI Vision Sensor, behind one `ITagSource` (run both, cross-check, fuse). |
| Migration | **Clean-room v2 build** over the pre-season window. No parallel-legacy adapters required. |
| Multi-robot coordination | **Stretch goal** — thin seam only; prove the core on the X-drive first. Running shulib on both bots is just two independent instances, which is core (not "coordination"). |
| Odometry config | Two **perpendicular unpowered tracking wheels + IMU-as-heading + GPS/AprilTag correction** (X-bot); L/R + transverse + IMU on the H-bot (wheel-heading = cross-check only). |

---

## 4. Why shulib Beats LemLib & OkapiLib

We benchmark against **two** libraries, for different reasons. **LemLib** is the *capability*
baseline (modern, popular, tank-only — the bar to clear on motion/localization). **OkapiLib** is
the *style* baseline (the cleaner-architected ancestor — the bar to clear on ergonomics). shulib
must beat both. The table below is vs LemLib; §4a is the OkapiLib pass.

| Dimension | LemLib | shulib v2 |
|---|---|---|
| **Drivetrain** | Tank/differential ONLY; cannot strafe ("not going to change until other drivetrains are competitive"). | `IKinematics` twist `(vx,vy,ω)` → per-wheel cmds + inverse. `XDrive` / `HDrive` (capped strafe) / `Tank` subclasses. New drivetrain = new subclass, zero controller edits. |
| **Localization** | Tracking wheels + IMU dead-reckoning. No GPS/vision/fusion. Drift unbounded over a run. | Fused `Localizer`: Pilons odom + IMU-owned heading + `GpsCorrector` + `AprilTagCorrector` behind `IPoseSource`; complementary→EKF on SE(2) with Mahalanobis gating. Bounded over 60s; degrades gracefully. |
| **Motion** | Boomerang `moveToPose`, pure-pursuit `follow` — heading coupled to travel direction. | Decoupled per-axis (x,y,θ): profiled `vx/vy` to target while independently servoing θ. `MoveToPose` / `TurnTo` / `StrafeTo` / `FollowTrajectory` (independent per-waypoint heading). |
| **Architecture** | PROS objects embedded throughout; global namespaces; not unit-testable or sim-able. | 5-layer dependency-inverted; core depends ONLY on a PROS-free HAL. Host gtests + sim adapters. CI forbids `#include <pros/` in core. |
| **Units** | Migrating mid-library to a heavy type-safe lib; API churn. | Lightweight header-only `Quantity<Dim>` (~6 dims) + `_in`/`_deg`/`_tile` literals + built-in angle wrap. Kills the deg-into-cos/sin bug class at compile time. |
| **Async** | Global `MotionHandler`; single motion. | `MotionScheduler` object (injected clock/HAL): one active `IMotion`, `.async()`/`waitUntilSettled()`/`waitUntil(pred)`/`cancel()`; min-velocity handoff chaining. |
| **Accuracy** | PID-only (derivative-on-error, no FF), degrades with speed. | Bounded error via absolute correction; profiled + kS/kV/kA FF + derivative-on-measurement PID w/ integral clamp; vision-servo docking decouples sub-inch scoring from drift. |
| **Ecosystem** | Monolith; no seams. | Two boundary contracts: the `.vexbot` config+paths sub-schemas shulib reads + `SHUL/2` telemetry (plus the internal HAL sim/real/test seam). Fully standalone — pure-code use needs no file at all. |

### 4a. The OkapiLib pass (style baseline)

OkapiLib is the cleaner-architected ancestor — in several places it is closer to our target
than LemLib is. We treat it as a *positive model to learn from*, not just a target to beat. Two
facts frame it:

- **Archived / frozen.** Read-only since 2023-02-03; last release **v4.8.0 (2022-06)**. No
  Override support, no active development. Building *on* it is a non-starter; beating a stationary
  target is the easy half.
- **Better style, same capability gaps.** Its odometry is **wheel-encoder-only** (no IMU/GPS/vision
  fusion — same core gap as LemLib), and its motion is **turn-then-drive decomposition**:
  `driveToPoint`/`turnToPoint` compute an angle + distance and forward to a tank-style
  `ChassisController`. **Even on a 3-wheel X-drive it does not strafe-to-pose under closed loop** —
  it *tracks* omnidirectionally but *moves* like a tank. Its frame is **+x fwd / +y right** (opposite
  handedness from our §7 +X/CCW — a third independent reason to freeze the frame before any code).

**Adopt from OkapiLib (steal the good ergonomics, with credit):**

- **Dimensional-analysis units** — OkapiLib's `QLength`/`QAngle`/`QTime` proves the type-safe-units
  bet pays off. Validates our `Quantity<Dim>` decision (§7, §13 #3); keep it lighter (~6 dims, no
  template-heavy `RQuantity` verbosity / compile-time blowup).
- **`SettledUtil`-style settling** — exit on (|error| < tol) **and** (|derivative| < tol) **and**
  (held for `time` ms). This is the principled replacement for the old `while(fabs(error)>1)` hang
  bug; bake it into every `IMotion` exit condition + the motion watchdog (§12).
- **Fluent builder** — `ChassisControllerBuilder` ergonomics → our `Chassis` builder (§8) already
  follows this; hold the bar there.
- **Composable filters** — `EmaFilter`/`MedianFilter` as small objects → a `filters/` utility for
  conditioning of **task-sensor / alignment** signals (Distance/Optical/vision-offset for `dockToGoal`)
  — **not** the EKF measurement inputs (GPS/AprilTag/odom), which are conditioned only by the
  gated/adaptive-R fusion math (§13 #4), never by EMA/median.
- **`AsyncController` + `ControllerRunner`** — formalize our `MotionScheduler` (§4 Async row) against
  this proven interface shape.
- **Squiggles** (their path generator, replaced Pathfinder) — reference prior art for the VexBuilder
  path planner, not a shulib runtime dependency.

**Where shulib is strictly better than OkapiLib:** maintained & game-current · true holonomic
motion-to-pose (twist currency) vs turn-then-drive · multi-sensor fusion vs wheel-only ·
host-unit-testable core vs PROS-bound · defined ecosystem seams.

---

## 5. Architecture (Layered)

Dependencies point **down / inward only** — no layer includes a header from a layer above it.

```
L4  APPLICATION (per-robot, student-owned)
    src/main.cpp: initialize / autonomous / opcontrol
    Composition root: builds HAL impls -> RobotContext -> Chassis
      | depends on
L3  FACADE
    Chassis (orchestration)   RobotContext (DI container of HAL handles)
    Verbs: moveTo / strafeTo / turnTo / followTrajectory / drive(ChassisSpeeds, Frame)
      |
L2  CORE  (pure C++20, NO pros, host-testable)
    motion/            localization/            control/
      IMotion            IPoseSource              Pid (d-on-measurement)
      MoveToPose/TurnTo  Localizer (fused)        Feedforward {kS,kV,kA}
      StrafeTo/Follow    PilonsOdometry           MotionProfile (trap/S-curve)
      MotionScheduler    Gps/AprilTagCorrector    ExitCondition / ExitGroup
      |
L1  KINEMATICS + MATH (pure C++20)
    kinematics/ IKinematics, XDriveKinematics, HDriveKinematics, TankKinematics
    math/ Pose2d, Twist2d, ChassisSpeeds, Angle (wrap), units::Quantity<Dim>
      |
L0  HAL INTERFACES (pure abstractions, ZERO pros)
    hal/ IMotor IRotation IImu IGps IVision IDistance IOptical IClock ITelemetrySink
      | implemented by (NOT depended on by core)
    hal/pros/* (real V5 — the ONLY files that include <pros/*>)
    hal/sim/*  (VexBuilder)        hal/fake/* (host gtest doubles)
```

**Data flow (per ~10ms tick):** HAL sensors → `Localizer.update()` → fused `Pose2d/Twist2d` →
`ActiveMotion.update(dt, pose, vel)` (profile FF + per-axis x/y/θ PID + ExitGroup) →
`ChassisSpeeds` (FIELD frame) → Chassis rotates FIELD→BODY (the **one** correct transform) →
`IKinematics.toWheels()` → desaturate → `IMotor.moveVoltage()` → `Telemetry.emit(...)` → SHUL/2.

---

## 6. Module Map

**L0 HAL** — `hal/` pure interfaces (`IMotor`, `IRotation`, `IImu`, `IGps`, `IVision`,
`IDistance`, `IOptical`, `IClock`, `ITelemetrySink`); `hal/pros/` V5 adapters (only files that
include `<pros/*>`); `hal/sim/` (VexBuilder) + `hal/fake/` (deterministic test doubles).

**L1 Math/Kinematics** — `math/` (`Pose2d`, `Twist2d`, `ChassisSpeeds`, `Angle`,
`units::Quantity<Dim>`, literals); `kinematics/` (`IKinematics` + `XDrive`/`HDrive`/`Tank`,
forward + inverse + desaturate + `strafeAuthority()`).

**L2 Core**
- `control/` — `Pid` (derivative-on-measurement, integral + output clamp, injected clock),
  `Feedforward{kS,kV,kA}`, `MotionProfile` (trapezoid; S-curve later), `ExitCondition`/`ExitGroup`.
- `localization/` — `PilonsOdometry` (kept arc math, mid-heading, diagnostics off hot path);
  `Localizer` (fuses odom + IMU heading + correctors; complementary → EKF; quality flag);
  `GpsCorrector` + `AprilTagCorrector` behind `IPoseSource`/`ICorrector`.
- `motion/` — `IMotion`, `MoveToPose`, `TurnTo`, `StrafeTo`, `FollowTrajectory`,
  `MotionScheduler` (async/cancel/chaining).

**L3 Facade** — `chassis/Chassis` + `RobotContext` (composition root; the only object that
differs across hardware/sim/test).

**Skills layer** — `alignment/` (`DockToGoal` visual-servo, Distance fallback),
`manipulation/` (`IntakeFromLoader`, `PlacePin`, `NestCup`, `SetQuadrantToggle`),
`sequence/` (Action engine: Sequence/Parallel/Race/Deadline + match-timer park guard).
_See §14 for the full skills-layer primitive set (synthesized from the strategy digest)._

**IO / Tools** — `io/Trajectory` + `.vexbot` `paths[]` reader (+ legacy `.shupaths` importer); `io/Telemetry` SHUL/2
(ports/absorbs `logger.hpp` into the new core **before** the end-of-M2 legacy deletion, fixing its 3 known bugs — §18.6); `tools/sysid` (offline kS/kV/kA least-squares → emits **constants**,
not code) + on-robot tuner.

---

## 7. Canonical Conventions

> **Coordinate frame is artifact #1 — the #1 silent accuracy-killer. Freeze before any code.**

**RATIFIED 2026-06-08 (LOCKED — +X / CCW):**
- **Origin** (0,0) at **field center**; **+X right, +Y away from the red station** (audience view).
- **Heading θ = 0 along +X, increasing CCW** (standard math convention).
- **Internal units: inches + radians + seconds, always.** Degrees only at the user-API boundary
  (`_deg`); milliseconds only where a clock hands them over, converted to seconds at the HAL edge.
  *(Time unit = seconds, locked 2026-06-19 — matches kS/kV/kA feedforward; ms-into-seconds is its own
  drift bug-class, killed the same way as deg-into-trig.)*
- The V5 **IMU (compass/CW)** and **GPS (own frame)** are converted to canonical **exactly once,
  at the HAL adapter**. One unit-tested `fieldToRobot()` / `robotToField()` pair is the **only**
  place a frame transform is allowed. A single `Rotation2d`/`Angle` type owns wrapping → kills the
  `deg-into-cos/sin` bug class.
- _Rejected alternative:_ "0 = forward/+Y, CW" (the old odom). Valid in isolation, but we go
  **+X/CCW** — standard math convention, matches the kinematics / EKF literature.

**Accuracy targets (RATIFIED 2026-06-08 — heading is a HARD requirement; validate on the field):**
- **Heading: `< 1.0°` error, always.** This is the firm team spec; position targets flex around it.
- **v1 (GPS + wheels + IMU, no vision docking):** end-of-60s pose within **~1.0″ and `< 1.0°`**;
  run-to-run repeatability within **~0.75″**.
- **With vision docking (Phase 3):** final goal alignment within **~0.25″** and **`< 1.0°`** _closed-loop_
  (drift-independent; typically < 0.5° on a tag) — enough to nest a 1.6″ Pin with a funnel for the residual.
- Each becomes a telemetry-based pass/fail acceptance test per phase.
- _Engineering consequence (honest):_ a raw V5 IMU yaw drifts ≈ **1°/min**, so dead-reckoned heading
  alone will **not** reliably hold `< 1°` across a 60s run. The spec therefore makes **IMU-owned heading +
  absolute yaw correction (AprilTag/GPS) load-bearing, not optional** — Phase 3 yaw correction is
  promoted from "nice-to-have" to **required to meet spec**. [[vexu-override-facts]]

---

## 8. Subsystem Designs (summary)

### Sensor utilization map

Every sensor flows through the HAL into **one fused estimate** — none is trusted alone.

| Sensor | Bucket | How shulib uses it | Module |
|---|---|---|---|
| **V5 IMU** | localize | Authoritative heading (yaw) in the fused estimate; yaw-rate gates GPS; pitch/roll → tip detection; cold-calibrated at boot | `localization/`, safety |
| **Rotation sensors (tracking wheels)** | localize | Two perpendicular _unpowered_ wheels → Pilons arc x/y; the predict backbone between absolute fixes | `localization/PilonsOdometry` |
| **V5 GPS** | localize | Absolute (x,y,θ) on the `<RSC4>` strip; gated corrector (adaptive R from `get_error()`, lever-arm + latency, Mahalanobis); dead-reckon-only off-strip | `localization/GpsCorrector` |
| **AI Vision + AprilTags** | localize + task | `relocalize()` feeds a high-confidence absolute fix from a known goal tag **into the gated corrector** (low-R → fast, drift-canceling **nudge**, still innovation-bounded per §13 #4 — *never a snap*); `dockToGoal()` sub-inch visual-servo; pin/cup detect + color | `localization/AprilTagCorrector`, `alignment/`, `manipulation/` |
| **Raspberry Pi / Coral** (VEX U) | localize | Student CV coprocessor (AprilTag/object) → pose fixes to the Brain over serial; backs AI Vision behind `ITagSource` | HAL `ITagSource` |
| **Optical** | task | `setQuadrantToggle` color confirm (no over-rotation); `orientToScoringHalf` (visible-half color); intake color/presence | `manipulation/` |
| **Distance** | task | `dockToGoal` no-tag fallback (v1); capture confirm; wall cross-check; goal-base-occupied → height-adaptive fallback | `alignment/`, `manipulation/` |
| **Motor encoders + current** (built-in) | task | `liftToLevel` height (encoder + linkage map + sag comp); drive velocity for FF/PID; current-spike capture/dock confirm + stall homing; thermal monitor | `control/`, `manipulation/` |
| **Battery voltage** | control | Scale motor-voltage commands by measured V so feedforward holds as the pack sags <11V | `control/` |
| **LIDAR** (VEX U, optional) | localize (stretch) | 2D scan-match as another `ICorrector`; obstacle/robot detect | stretch |

**Two buckets:** _localization sensors_ (IMU + wheels + GPS + AprilTag/Pi [+LIDAR]) fuse into one
gated pose that degrades gracefully as inputs drop out; _task sensors_ (Optical + Distance +
encoders/current + Vision) **confirm every manipulation/alignment action** so a no-driver auton never
advances on a failed grab or place. Redundancy is the point — it's what LemLib (wheels + IMU only)
structurally can't match.

- **Localization / fusion.** High-rate prediction (Pilons arc odom @ ~100 Hz, IMU-owned heading)
  + low-rate absolute correction (GPS via `<RSC4>` strip with adaptive R from `get_error()`,
  lever-arm + latency comp; AprilTag PnP from tags 0–4). Error-state complementary filter first,
  upgraded to a 5-state SE(2) EKF `[px,py,θ,vx,vy]` with Mahalanobis gating, high-yaw-rate
  rejection, consecutive-reject re-init, and a hard **dead-reckon-only** flag (Driving Skills has
  no strip). Process noise scales with travel (slip ∝ distance).
- **Holonomic motion.** Field-frame trapezoidal profile + kS/kV/kA feedforward +
  derivative-on-measurement PID per axis; wheel-command desaturation (uniform scale). `MoveToPose`
  decouples translate + heading; `StrafeTo` is the holonomic-only verb; H-drive caps commanded
  `vy` to `strafeAuthority()` with an automatic turn-then-drive fallback (observable in telemetry).
- **API & units.** Fluent builder over HAL + kinematics + sensors + estimator; chainable async
  motions; type-safe `Quantity<Dim>`. Standalone autons need no planner.
- **Ecosystem seams.** (a) **HAL** sim/real/test seam; (b) the **`.vexbot` `paths[]`** sub-schema
  (typed waypoints w/ independent per-waypoint heading + event markers) consumed by
  `PathRunner`/`FollowTrajectory`; (c) **`SHUL/2`** versioned, sequenced, typed telemetry stream behind a Sink
  (NullSink default = zero cost) for VexBuilder visualize/record/replay. All version-gated from
  day one; shulib fully functional with no VexBuilder present.

---

## 9. Roadmap (clean-room build, ~2–3 months)

> Build in **testable vertical slices**: each phase ships a host-tested, hardware-validated unit.
> **The in-depth, website-facing roadmap is [`roadmap.md`](roadmap.md)** — full task breakdown per
> milestone (M0–M8), a Freeze Register of don't-break contracts, the 12 permanent workstreams, and
> the build/CI/test-harness scaffolding tasks. The phases below are the summary; `roadmap.md` is the
> exhaustive source of truth for *what's left to do*.

- **Phase 0 — Canonical foundation.** Freeze the coordinate frame (unit-tested) + accuracy
  targets; stand up `math/units`, the `Angle` type, and the IMU-owned heading path. Establish that
  the old bug classes (field-centric overwrite, deg-into-trig, nullptr IMU, derivative-on-error)
  are **structurally impossible** by design. _Milestone: heading stable on a 60s strafe test; frame
  transform unit-tested._
- **Phase 1 — HAL + kinematics foundation.** HAL interfaces + `hal/pros` + `hal/fake`;
  `IKinematics` + `XDriveKinematics` (+ Tank) host-tested with pure numbers; CI forbids `<pros/>`
  in core; freeze v1 `.vexbot` `paths[]` schema (the `DebugRecord`/`SHUL/2` schema freezes later — at
  M2 and M6 respectively, §18). _Milestone: kinematics + a trivial motion run
  identically in a host gtest and on the V5 by swapping only `RobotContext`._
- **Phase 2 — Real holonomic motion.** `control/` (trapezoid profile + FF + ExitGroup); `motion/`
  (`MoveToPose`/`TurnTo`/`StrafeTo` + `MotionScheduler`); `Localizer` on odom + IMU (correctors
  stubbed); `HDriveKinematics` w/ capped strafe; `tools/sysid` kV/kA characterization.
  _Milestone: a hand-written X-drive auton chains profiled motions; same code runs the H-bot._
- **Phase 3 — Bound the 60s drift (accuracy edge).** `GpsCorrector` + `AprilTagCorrector`
  (Pi + V5 AI Vision backends) → upgrade `Localizer` to the EKF; `alignment/DockToGoal`
  visual-servo (Distance fallback); documented calibration routines. _Milestone: pose error bounded
  across a full 60s run; docking nests a 1.6″ Pin repeatably._
- **Phase 4 — Sequencer + ecosystem.** `sequence/` Action engine + guaranteed Midfield park;
  `.vexbot` `paths[]` import + `FollowTrajectory`; `SHUL/2` handshake + replay; min-velocity chaining;
  on-robot tuner. _Milestone: VexBuilder replays a real run; a dense goal-to-goal skills run flows
  without stop-and-go and always ends parked._

---

## 10. The Onboard-Estimation Story & Award Narrative

**The onboard "AI" =** a **student-built state estimator** (complementary filter → EKF) fusing wheel
odometry + IMU + GPS + AprilTag, plus **onboard CV** on the Pi / AI-Vision and **offline system-ID /
auto-tuning** that emits **constants, not code**.

**Award narrative (Excellence / Design / Innovate):** "We built our own multi-sensor state estimator
for absolute, drift-free **holonomic** localization — beyond LemLib's tracking-wheel-only
dead-reckoning." Innovate headline: **fused absolute correction + holonomic visual-servo docking that
strafes to align** (impossible on a differential drive), tied to `<RSC4>` (GPS strip) and the goal
AprilTags, mapped to the Skills math (yellow Pins via a sensor-confirmed `SetQuadrantToggle`; the
Midfield park).

_Tiering note: complementary-filter-before-EKF is partly a simplicity choice — a simpler filter is
easier to get right and to explain._

---

## 11. Red-Team Gates & v1 Cut-List

**Gates (must do):**
1. **Freeze ONE coordinate frame**, unit-tested, before any code (§7).
2. **AI Vision AprilTag SDK — RESOLVED (2026-06).** Native AprilTag detection ships in **PROS 4.2.2**
   (`pros::AIVision`: 4 tag families `tag_21H7/16H5/25H9/61H11`, `enable_detection_types(tags)`,
   `set_tag_family`, returns tag ID + corners). This repo is just pinned to the **stale kernel 4.1.0**
   (calypso is on 4.2.2) — fix is a one-time `pros` kernel bump. The team already has a working
   `pros::AIVision` wrapper (`calypso`/`Downloads/ai_vision.hpp`, object-mode); it becomes the
   `hal/pros` `IVision`/`ITagSource` adapter, extended to read tag detections. The **V5 AI Vision Sensor
   is now the easy primary vision backend**; the Pi drops to optional/redundant (stretch). _Action: bump
   shulib's kernel 4.1.0 → 4.2.2 and vendor/extend the existing wrapper in Phase 1._
3. **Tier the estimator** — ship the complementary filter first; EKF is Phase 3, gated on the
   complementary tier demonstrably bounding drift with GPS-only correction.
4. Add **voltage/brownout compensation** (FF is voltage-based; battery sags <11V by 60s), a
   **motion watchdog** (old `while(fabs(error)>1)` loops can hang forever), and a **numeric accuracy
   target** (§7).

**Cut from v1 (defer):** 5-state EKF + ZUPT + latency comp + adaptive-R (→ Phase 3); AprilTag
PnP / visual-servo docking (→ gated on backend; Distance-sensor docking fallback for v1); S-curve
profiles (trapezoid only); offline auto-tuning / learned-friction tables / on-robot live tuner
(hand-tune kS/kV/kA via one ramp routine); min-velocity motion chaining (stop-and-settle first);
full Action-combinator engine (hand-written blocking calls + one async handle + a wall-clock park
guard); Distance/Optical as *fusion* inputs (use for manipulation confirmation only); binary
compiled-path artifact + full schema-hash handshake (versioned JSON `.vexbot` first).

---

## 12. Accuracy Risks to Watch

1. **Frame-convention mismatch** — #1 killer; resolved by §7 + a regression unit test.
2. **IMU drift over 60s** — cold-calibrate at boot, never move during calib; characterize per-boot
   bias; GPS-yaw as a slow heading anchor at low yaw-rate; gate GPS out at high yaw-rate.
3. **X-drive wheel slip** corrupting predict — **dedicated unpowered perpendicular tracking
   wheels are a hard requirement**; don't value-engineer to drive-motor encoders. Scale Q with travel.
4. **GPS availability / latency / occlusion** near the tall center goal — characterize the real
   field envelope; default GPS trust LOW; dead-reckon-only mode for Driving Skills.
5. **Corrector pose-jump mid-motion** — innovation-bounded, covariance-weighted, gated nudge
   (never snap); clamp per-tick nudge; log every gating decision.
6. **Calibration burden** (wheel scale/offset, GPS lever-arm, camera mount, IMU bias) — ship
   documented routines + persistence (SD/config); make it a Phase-3 entry gate.
7. **Terminal nesting** needs <0.25″ → closed-loop docking (Phase 3) + mechanical funnel; don't
   promise sub-inch nesting on pose estimate alone.

---

## 13. Open Decisions / TBD

| # | Decision | Recommendation | Status |
|---|---|---|---|
| 1 | Coordinate frame (+X/CCW vs +Y/CW) | **+X / CCW** (§7) | **LOCKED 2026-06-08** |
| 2 | Accuracy targets (§7) | heading **`< 1.0°`** (hard); ~1.0″ pos; ~0.25″ docked | **LOCKED 2026-06-08** |
| 3 | Units system | Minimal header-only `Quantity<Dim>` over **6 dims** (length, angle, time, velocity, acceleration, voltage; canonical = inch, radian, **second**, in/s, in/s², volt). `Angle`/`Rotation2d` is the wrapping heading type; `_deg` constructs an auto-wrapped `Angle`, not a bare `Quantity`. Literals are convenience-only (roadmap M0 = source list). | **Locked 2026-06-19** |
| 4 | GPS/AprilTag aggressiveness | Innovation-bounded, covariance-weighted, per-tick-clamped **gated nudge (never snap)** + hard dead-reckon-only flag. `relocalize()` feeds this corrector — it does not snap. Clamp/gate/gain are M3-tuned (mechanism locked, numbers open); complementary tier realizes the same contract heuristically. | **Locked 2026-06-19** |
| 5 | H-drive model | `strafeAuthority()` = **pure read-only query** returning max sustainable \|vy\|/\|vx\| (XDrive 1.0, HDrive ~0.35 *sysid-measured default, not a constant*, Tank 0.0); the **motion/Chassis layer** reads it to clamp commanded `vy` + trigger the turn-then-drive fallback — kinematics does **not** clamp inside `toWheels()`. Telemetry-visible (`strafeFallbackActive`). | **Locked 2026-06-19** |
| 6 | Estimator depth | Complementary first → EKF Phase 3 | **Locked (tier-it)** |
| 7 | Vision backend | Pi **and** V5 AI Vision behind `ITagSource` | **Locked (both)** |
| 8 | Skills routing / robot roles | X = tall scorer, H = Toggle-owner + parker (§14) | **Filled; build-team decisions open** |
| 9 | Settling & filtering primitives | OkapiLib-style `SettledUtil` (err+deriv+time) on all motion exits; composable `filters/` (EMA/median) for **task-sensor/alignment** conditioning only — **excluded** from the EKF measurement path (§4a, §13 #4) | **Locked 2026-06-19** |
| 10 | Robot-config seam | `.vexbot` → codegen `robot_config.hpp` (primary) + optional **SD-card runtime loader** (same `robotProfile` schema → same `RobotConfig`); both behind `IRobotConfig` (§16.2). Interface = Freeze F4/M1, schema = Freeze F7/M5. | **Locked 2026-06-19** |
| 11 | Path command model | Command-**id registry** (`runner.on("intake_in",…)`), NOT embedded C++ `code_template`. shulib owns the canonical id **manifest** (VexBuilder reads it); unknown id → WARN+skip; optional typed-args on markers (§16.3) | **Locked 2026-06-19** |
| 12 | Canonical ecosystem formats | shulib **defines** the `robotProfile` + `paths` sub-schemas inside `.vexbot` (+ command-id vocab) and `SHUL/2`; VexBuilder produces them. `.shupaths` + standalone planner **retired** (§16.1) | **Locked** |
| 13 | Sim seam timing | **Define** the `SHUL/2` schema + `hal/sim` seam at design time (so M6 is a plug-in); **implement** the wire + sim adapter at M6 (F9); wire to Rapier at VexBuilder Phase 7. Scope = `hal/sim` + wire sink only (does **not** move `TermSink` M2 / `SdSink` M3); `hal/sim` is exercisable via `hal/fake` even if Rapier never lands. | **Locked 2026-06-19** |
| 14 | VexBuilder must add | Explicit drivetrain fields (kind/trackWidth/wheelDia) in `.vexbot` so config isn't geometry-inferred (§16.2 caveat) | **Cross-team ask** |
| 15 | Kinematics backend | **HYBRID:** `IKinematics` is the interface (swerve is *nonlinear* — a coeff table can't express it — and the interface is the home for `forward()` inverse-kinematics used by odometry + the `strafeAuthority()` query). The **linear** holonomic drives (X/H/tank/mecanum) are one impl, `MatrixKinematics`, driven by a per-wheel `[h, v, turn]` coefficient matrix (salvages `lodge`'s data-table idea). `toWheels()` is clamp-free (§13 #5); `desaturate()` is a virtual with a uniform-scale default. Wheel outputs are dimensioned `Velocity`, not bare doubles (F3). Frozen as **F5** at M1. | **Locked 2026-06-19** |

---

## 14. Skills Strategy & Routing

_Synthesized from 16 Override strategy/design video transcripts (digest run 2026-06)._

### The one insight that dominates: ownership, not volume
An **owned** yellow Pin = 10 pts and double-counts across clear zones (~20 effective); an
**unowned** yellow ≈ 0. So a single **`setQuadrantToggle`** (flip a quadrant's Toggle to our color)
outscores several raw cycles — a **~40-pt swing per quadrant**. **The two highest-value software
primitives are `setQuadrantToggle` and `orientToScoringHalf` (yellow-side-out).** Spend the
accuracy budget there first. Alliance goals are protected and neutral/opposing goals can't be
de-scored → build the tall stack on a **protected alliance goal**; rush the Midfield goal early only
for denial value.

### Recommended two-robot role split (don't contest the same quadrants)
- **24" X-drive = primary tall scorer.** Floor pickup (counter-roller + widest poly funnel) + ~50"
  lift + 180° rotator (yellow-side-out) + **AI Vision AprilTag** for mid-run relocalization. Owns its
  home quadrant; builds one tall owned-yellow nested stack on a protected alliance goal.
- **15" H-drive = Toggle-owner + match-load cycler + parker.** Self-aligning vertical clamp
  (standing/match-load only — sidesteps floor pickup, the season's hardest unsolved problem) +
  poly-cutout passive dock. Owns the other two Toggles and **parks the Midfield (+8)** at the buzzer.

### Two 60s routing concepts (separate per-robot autons)
- **Concept A — X-drive tall-tower (~90–130 pts):** own home Toggle (t0–4) → preload + deny the
  Midfield goal early → ~8–10 cycles of `intakeUntilCapture → relocalize/dockToGoal → liftToLevel →
  orientToScoringHalf → buildStack` (t10–48) → re-verify Toggle (t48–56) → hold/brake on it.
- **Concept B — H-drive Toggle-own + cycle + park (~60–90 pts):** flip its two Toggles, wedge-jam
  (t0–8) → ~6–8 `matchLoadCycle()` (t8–45) → sweep loose pins (t45–55) → `endInMidfield()` +8 (t55–60).
- **Combined ≈ 150–220+ pts**, dominated by owned-yellow stacking — not raw cycle count.

### The four accuracy bottlenecks (each maps to a primitive)
1. **Floor-pickup reliability** → `intakeUntilCapture` (counter-roller, retry, sensor-confirm).
2. **Per-level lift height repeatability** under growing load + cascade back-drive → `liftToLevel`
   (homing + linkage-aware mapping + sag-comp PID hold).
3. **Yellow-half-out orientation** → `orientToScoringHalf` (color-sense → pass-through vs flip).
4. **Odometry drift over 8–10 cycles** in a crowded field → `relocalize` (AprilTag/poly-cutout fix
   before each score). _This is the §8 localization stack earning its keep._

### Skills-layer primitives the library must expose
| Layer | Primitives |
|---|---|
| **motion** | `fieldCentricStrafe` / `strafeTrim` (H), `moveToPoseProfiled` (lift-state-aware accel), `holdPose`/`driveBrake` |
| **alignment** | `dockToGoal` (poly-cutout/AprilTag; current/distance/pose confirm; height-adaptive fallback), `dockToLoader`, `relocalize(featureSource)` |
| **manipulation** | `setQuadrantToggle` (index N clicks on the 3-state Toggle + color confirm), `intakeUntilCapture`, `liftToLevel`, `orientToScoringHalf`, `rotateClampToAngle` (profiled, no fling), `clampActuate`+`clampConfirm`, `deployActuator` (air-budget aware) |
| **sequence** | `buildStack` (align→lift→orient→place pin→cup-over-pin→confirm nest), `matchLoadCycle`, `endInMidfield` (18" height lockout), `strategyMode(tallTower\|fastCycle)`, time-budgeted **Sequencer** (possession-aware: 1 pin + 1 cup; aware of the 2 pre-scored pins; **guaranteed end-of-run action**) |

### Non-negotiable: time-budgeted sequencer with a guaranteed end-of-run action
No driver recovers a stall. The **+8 Midfield park** and the final Toggle re-verify **must fire on a
hard schedule** regardless of where the scoring loop stalled — a scheduled-action sequencer
(extending the existing `RobotCommands`/`Command` queue) is required, not optional.

### Build-team decisions to settle (mostly hardware, not software)
1. **Robot roles** — X = tall floor scorer / H = Toggle-owner + parker (recommended). _Drives the auton split._
2. **Mechanisms** — X: flexwheel + counter-roller + lift + 180° rotator; H: self-aligning vertical clamp.
3. **Lift** — cascade (lowest CG, needs homing) vs DR6B (simpler, tippier). _Recommend cascade on X._
4. **Goal localization** — AI Vision AprilTag on the X-drive scorer; poly-cutout dock on the H-drive.
5. **Toggle defense** — wedge-jam where reachable + a ~t50s re-verify pass (low priority in skills — no opponent).

_VEX U note: no 55W drive cap — we can run heavier X/H drives and dual-motor lifts; treat the HS/MS
55W RPM figures (2.75" @ ~450 rpm) as analog targets, not caps._

---

## 15. The One-Stop-Shop Capability Catalog (past · present · future)

> The north star: shulib is not "a chassis class" — it is the **complete autonomy stack** for a
> VEX U holonomic robot, from the build file to the scored point, usable by a team that cannot
> write C++ and extensible by one that can. This catalog is the *whole* surface area. We **tier**
> it (§3) so the proven subset ships first; nothing here is hand-waved away, but nothing ships
> unproven either.

**Maturity legend:** ● **Core** (proven v1 subset, ships first) · ◐ **Tiered** (Phase 2–3,
clearly reachable) · ○ **Frontier** (stretch / research — the "future" we design seams for now).

| Domain | ● Core (present) | ◐ Tiered (near future) | ○ Frontier (north star) |
|---|---|---|---|
| **Drivetrain & kinematics** | `IKinematics` twist `(vx,vy,ω)`; `XDrive`/`HDrive`/`Tank` | Mecanum, swerve scaffolds; per-wheel slip model | Auto-ID drivetrain from VexBuilder geometry; online wheel-radius calibration |
| **Motion primitives** | `MoveToPose`, `TurnTo`, `StrafeTo`, `driveBrake`, `holdPose` | `FollowTrajectory` (per-waypoint heading), min-vel handoff chaining | Dynamic replan around detected obstacles; time-optimal profiling |
| **Path & trajectory** | `.vexbot` `paths[]` import + profiled execution; markers/commands | In-lib spline/`Squiggles`-style smoothing; velocity-constrained profiles | Live VexBuilder round-trip (edit path → re-sim in seconds) |
| **Localization** | Pilons odom + IMU-owned heading; `IPoseSource` | `GpsCorrector` + `AprilTagCorrector`; complementary filter; Mahalanobis gating | Error-state EKF on SE(2); LIDAR scan-match; Pi-fused absolute pose |
| **Control & FF** | derivative-on-measurement PID + integral clamp; `SettledUtil` exits; kS/kV/kA FF | brownout/voltage comp; motion watchdog; per-axis decoupled servo | adaptive gains; learned feedforward from logged runs |
| **Manipulation** | `Mechanism` HAL, `intakeUntilCapture`, `liftToLevel`, profiled rotators | `buildStack`, `matchLoadCycle`, air-budget-aware actuation | vision-servo grasp; closed-loop stack verification |
| **Autonomy authoring** | data-driven `PathRunner` + command registry (no C++ needed) | fluent recipe API; time-budgeted Sequencer w/ guaranteed end action | GUI sequence builder in VexBuilder writing `paths[]` into `.vexbot` |
| **Simulation & test** | host gtests via `hal/fake`; deterministic clock | `hal/sim` adapter + `SHUL/2` over VexBuilder agent socket | full Rapier physics round-trip; planned-vs-actual overlay |
| **Diagnostics & telemetry** | **`TermSink` readable terminal stream** + `DebugRecord` + fault codes + per-motion result (§18); `NullSink` default | `SdSink` binary blackbox; `SHUL/2` wire; live PID/FF tuner; record/replay | on-brain HUD add-on; cloud run library; auto-tune from replays |
| **Config & hardware** | `IRobotConfig` + `RobotBuilder`; hand-written config | `.vexbot` → `robot_config.hpp` codegen; SD-card runtime profile | zero-touch: build in VexBuilder → working robot, no edits |
| **Docs & onboarding** | per-module notes | "first auton in 10 min" path; recipe cookbook | generated API site + interactive examples (the team website) |

The three columns ARE the roadmap's spine (§9, and the public `roadmap.md`). Anything a future
team asks for should map to a cell here; if it doesn't, the catalog grows — deliberately, tiered.

---

## 16. VexBuilder Integration Contract

VexBuilder is the team's **single authoring tool** — 3D robot designer + path planner + (planned)
physics sim (Tauri / React / Rust+SQLite) — saving everything in **one `.vexbot` project file**.
shulib is the **runtime**: it ingests that file and runs it, today and for robots not yet designed.
**Decision (locked):** the standalone `.shupaths` format and the separate Python path planner are
**retired** — paths now live inside `.vexbot` alongside the robot, so a routine can never fall out
of version-sync with the robot it was drawn for. A one-way importer migrates existing `.shupaths`.

**16.0 — Principle: shulib defines the contracts; VexBuilder implements them.** Because shulib's
core depends only on the PROS-free **HAL** (§5), the *exact same* motion/localization/auton code
runs in **three targets** with zero changes: real robot (`hal/pros`), VexBuilder sim (`hal/sim`),
host tests (`hal/fake`). "Works with the simulation" is therefore a *structural* guarantee, not a
feature we bolt on. At the tool boundary shulib reads just **two** contracts — the `robotProfile` +
`paths` sub-schemas inside `.vexbot`, and the `SHUL/2` telemetry wire. VexBuilder owns the `.vexbot`
*file*; shulib co-owns the *slices it reads* (plus the command-id vocabulary) and ignores the rest
(parts/holes/joints).

**16.1 — The format divergence we must resolve (finding).** VexBuilder's *spec docs* and its
*shipped implementation* have already drifted apart. shulib picks the canonical shape and collapses
the path tooling into the one file:

| Artifact | Spec docs want | Shipped `.vexbot` v2.0.0 has | **shulib canonical decision** |
|---|---|---|---|
| Robot config | `mechanisms[]` + `shulibExportConfig` + codegen `robot_config.hpp` | `.vexbot` `electrical{motors,sensors,pneumatics}` + `joints[]` (electrical UI not built — arrays empty) | **`robotProfile`** block shulib specifies, written into `.vexbot`; codegen to `robot_config.hpp` primary. Source data from the shipped `electrical{}` + explicit drivetrain fields VexBuilder must add. |
| Paths | separate `.vbpath` | separate `.shupaths` (Python planner: `x,y,heading,heading_mode,motion_type,reverse,commands_after[{id,code_template}]`) | **Folded into `.vexbot` as `project.paths[]`** — one file, version-locked to the robot. `.shupaths` + standalone planner **retired**; a one-way importer migrates old `.shupaths`. Drop embedded `code_template` for command **ids** (§16.3). |
| Sim/telemetry | Rapier 60 Hz + IPC events (Phase 7) | Tauri agent server only (`server.json`: `port`,`token`,`pid`) | **`SHUL/2`** wire protocol over that local socket; defined now, wired when Rapier lands. |

**16.2 — Seam 1: one project file in (`.vexbot` → config + paths).** shulib reads a single `.vexbot`
and produces an in-memory `RobotConfig` *and* the robot's `Route`s. The config slice it specifies:

```
.vexbot › project.robotProfile {
  identity{name,team,season};
  drivetrain{ kind: x|h|tank; wheelDiameter; trackWidth|geometry; motors:[{port,reversed,cartridge}] };
  odometry{ wheels:[{port,reversed,diameter,offset}]; imuPort; gpsPort };
  sensors:[{port,type}]; mechanisms:[{name, motors:[...], pneumatics:[port]}]; corrections{x,y,theta} }
.vexbot › project.paths[]   (the routines — see §16.3)
```

Two ingestion paths, **same** in-memory types behind `IRobotConfig` / `IRouteSource`:

1. **Codegen (primary, on-robot).** A host-side generator (a small shulib CLI, or a VexBuilder
   export button) reads `.vexbot` → emits a typed `robot_config.hpp` carrying **both** the profile
   and the routines (`inline constexpr`). **No runtime JSON parse, no SD dependency, compile-checked.**
2. **SD-card runtime (optional).** shulib can also read the `.vexbot` directly off the SD card at
   boot — so a non-coder re-exports a file and re-runs **without recompiling**. Same interface, gated.

`RobotBuilder.from(profile)` returns a fully wired `Chassis` (drivetrain + kinematics + localizer +
sensors); `runner.load(route)` arms a routine. **"VexBuilder file → working robot + auton" is two
lines.** _Standalone preserved:_ a code-fluent team can build `RobotConfig` and `Route` directly in
C++ with **no file at all** — `.vexbot` is the on-ramp, not a dependency. _Honest caveat:_ the shipped
`.vexbot` doesn't yet carry drivetrain type / track width / wheel diameter as first-class fields —
today they'd be *inferred* from part geometry (brittle). **Action: VexBuilder must emit them
explicitly** (Open-Decision #14); shulib ships an `inferDrivetrain()` fallback but treats explicit
fields as the contract.

**16.3 — Seam 2: path execution + command registry.** `project.paths[]` holds routines (waypoints:
`x,y,heading,headingMode,motion,reverse,constraints,markers`); shulib's `PathRunner` executes
profiled motion per segment with marker callbacks. The legacy planner embedded **C++ snippets**
(`code_template:"mech.intakeIn();"`) — a data/code coupling we reject. Instead markers carry command
**ids** (`"intake_in"`) and the student registers handlers **once**:

```cpp
runner.on("intake_in", []{ intake.in(); });   // data-driven: the AUTON is data, not code
```

This is the keystone of the no-code story (§17): a routine is **data a non-coder authors in
VexBuilder**, executed by a library the coders maintain. (A codegen mode may emit the **same
id→handler bindings** as a generated `runner.on(...)` registration table at compile time — never
free-form code, so the data/code split holds.) shulib **owns the canonical command-id vocabulary**,
exported as a manifest VexBuilder reads to populate its command picker; an **unknown id** logs a WARN
fault and is skipped, never crashes (§16.5, §18.4). Markers may carry an optional **typed-args** object
(e.g. `{id:"lift_to_level", args:{level:3}}`) so parametric primitives don't explode into one id per
value. The `.shupaths` importer maps old `code_template` strings to ids best-effort, flagging any it can't.

**16.4 — Seam 3: Simulation & telemetry (`SHUL/2`).** When Rapier lands, shulib runs *unmodified*
in-sim through `hal/sim`, which speaks `SHUL/2` over VexBuilder's agent socket (discovered via
`~/.local/share/com.gonzei.vexbuilder/agent/server.json`). Bidirectional: VexBuilder feeds simulated
sensor readings *in* (so the same estimator runs), shulib streams pose/twist/wheel-cmd/markers *out*
(so VexBuilder renders the ghost robot and overlays planned-vs-actual). Versioned and **defined now**
so Phase 7 is a plug-in, not a redesign.

**16.5 — Versioning & forward-compat (future build files).** Every contract carries `schemaVersion`;
shulib negotiates and **migrates additively** (mirrors VexBuilder's own `migrateProject()` 1.1→2.0).
Unknown newer fields are ignored, not fatal; missing fields get safe defaults. **Bonus of one file:**
a robot and its routines share a version and can't drift apart — the spec's unsolved "re-export when
the robot changes" sync problem disappears. The promise holds: **a `.vexbot` you make next year still
drops into the library.**

---

## 17. Accessibility & Progressive Disclosure (for teams that can't code yet)

**The pillar:** a future team that cannot write C++ must still field a *working, accurate, award-
caliber* autonomous — and a team that *can* code must never hit a ceiling. shulib delivers this with
**progressive disclosure**: four tiers, each a strict superset of the one below, no cliff between them.

| Tier | Who | What they touch | What they get |
|---|---|---|---|
| **0 — Zero-code hardware** | Anyone | Build the robot in VexBuilder | `robot_config.hpp` — *no hand port-mapping, no measuring* (§16.2) |
| **1 — Data-driven auton** | Non-coder | Drag waypoints + pick commands in VexBuilder | saved into `.vexbot` → `PathRunner` runs it. **A full routine with zero C++.** |
| **2 — Recipe API** | Beginner coder | ~10 readable lines | `chassis.moveTo(p).then(intake.in)…` — fluent, hard to misuse |
| **3 — Full API** | Programmer | Everything in this doc | kinematics, estimator, custom motions, the works |

**The bridge is the command registry (§16.3).** A non-coder designs *strategy as data*; one coder
maps command-ids → mechanism actions *once*. After that, the non-coder iterates autons all season
without touching code, and re-running a new path is loading a different file — not a rebuild.

**You stay in command.** The library and its tools carry out the strategy *you* design — a routine is
your decisions expressed as data; the tools never make those decisions for you.

**Onboarding as a first-class feature.** "Your first auton in 10 minutes" (build → export → drag a
path → run), a recipe cookbook, and **generated API docs that publish to the team website** (the
`roadmap.md` and capability catalog are written to be web-portable). Documentation is a deliverable
with milestones in the roadmap, not an afterthought.

---

## 18. Diagnostics & Observability

> **Observability is a first-class, cross-cutting discipline — not a side effect of the telemetry
> wire.** The audit (2026-06) found the old plan scattered observability across other features and
> parked it in the M6 telemetry bundle, while the legacy code reached for debugging power the wrong
> way (a ~250-line hot-loop wheel-health block, prose `[VERT]`/`[VEER]` diagnosis trees, ALL-CAPS
> banners, raw `std::cout`). This section makes it a single owned contract. **Primary debug surface:
> the terminal** (readable, human-friendly). On-brain HUD is an optional add-on; the SD blackbox is
> the no-laptop field counterpart.

**18.1 — One record, many sinks.** A single structured per-tick `DebugRecord` is the source of
truth; every sink just *formats* it, so bench / terminal / field / sim traces are directly
comparable. Sinks sit behind the `ITelemetrySink` HAL seam (M1), `NullSink` default = zero cost:

| Sink | Role | When | Tier |
|---|---|---|---|
| `NullSink` | nothing (zero-cost default) | competition build | core, M1 |
| **`TermSink`** | **human-readable terminal stream — the primary dev/debug surface** | every dev/tune session | **core, M2** |
| `SdSink` | compact **binary blackbox** to `/usd/` — the no-laptop field record | field runs | core, M3 |
| `Shul2Sink` | the `SHUL/2` wire (VexBuilder visualize/record/replay) | ecosystem | tiered, M6 |
| `BrainHud` | optional on-brain HUD/summary screen | add-on | **add-on / Frontier** |

`SHUL/2` v1 **is** the serialized `DebugRecord` (Freeze F9) — define the schema once, every sink
serializes it (text / binary / wire). `TermSink` is not the JSON wire; it's a *pretty-printer* for
humans.

**18.2 — The `DebugRecord` (per-tick snapshot).** Captured each control tick, rate-budgeted:
`t`, `dt`, target/measured `Pose2d`, per-axis error, commanded `(vx,vy,ω)`, per-wheel `V`+`I`,
IMU yaw+yaw-rate, active command id/state, **dead-reckon flag**, **quality flag**, covariance
trace / filter trust weights, gating `(residual, Mahalanobis, reason)`, applied-correction
`(dx,dy,dθ)` + `clampedThisTick` (audits *never-snap*, §13 #4), `strafeFallbackActive` (§13 #5),
**fault code**, battery `V`/`I`. Competition build routes it to `NullSink` (≈free); dev build to
`TermSink` (+`SdSink`).

**18.3 — Readable terminal output (the headline requirement).** Leveled
(`ERROR`/`WARN`/`INFO`/`DEBUG`/`TRACE`, compile-time `TRACE` strip off the hot path), tagged by
subsystem, column-aligned (never prose), throttled on high-rate channels, with a one-screen
**run-summary** at auton end. Target shape:

```text
[t=12.34] [MOT] MoveToPose#7 tgt(24.0,36.0,90°) err(0.4",0.2",0.3°) v(18,4,0.1) ▸run
[t=12.41] [EKF] gps fix ACCEPT  resid(0.8",0.5",0.2°)  mahal 1.9  Q=0.91
[t=12.50] [MOT] MoveToPose#7 ✓SETTLED final(24.1,36.0,90.1°) over 0.2" drift 0.1° 1.16s
[t=12.51] [WARN][SEQ] intakeUntilCapture retry 1/3 (optical=none)
── RUN SUMMARY ───────────────────────────────────────────
 scored 6 pin · 1 cup   failed 1 grab (FLOOR_PICKUP_MISS)
 heading: max 0.7° final 0.3°   gating rejects 4   brownout no
 worst loop dt 11.2ms   first fault none
 build a1b2c3d · routine "redLeftTall" · batt 12.4→11.6V
──────────────────────────────────────────────────────────
```

The per-motion result line (`target vs final · overshoot · drift · time · exit-reason`) and the
run summary are the two genuinely-good ideas salvaged from the legacy "logging extreme" code,
re-expressed as **structured fields**, not essays emitted from motion loops.

**18.4 — Fault discipline.** A stable **numeric fault-code enum** (`ODO_STUCK`, `IMU_LOST`,
`GPS_GATE_REJECT`, `BROWNOUT`, `LOOP_OVERRUN`, `NAN_POSE`, `MOTION_TIMEOUT`, …) + **latched
first-fault** capture (root cause vs cascade); **NaN/Inf + invariant asserts** (heading wrapped,
per-tick pose-delta within physical max) that **log-and-recover** (non-fatal, safe fallback) instead
of propagating a silent NaN; **motion exit-reason codes** (`SETTLED`/`TIMEOUT`/`CANCELLED`/
`FAULT_ABORT`/`SUPERSEDED`) on every `IMotion`; **loop-overrun / tick-timing** detection (a blown
10 ms tick corrupts PID `dt` → degrades the very `< 1°` heading we promise).

**18.5 — Session header / provenance.** First record of every run: **git build hash** + routine id +
alliance/side + port map + battery start — lets us compare/reproduce runs and confirm exactly which
binary produced a given log.

**18.6 — Migration from the current code.** **KEEP** the appetite and the two good ideas
(sensor-health/stuck detection → rebuilt generic, gated, *off* the hot path, feeding fusion
sensor-validity; the per-motion result record → a structured field). **DROP** the bad
implementation (the hot-loop wheel-health block with hardcoded ports 8/9/10, the prose hardware-
diagnosis trees, the `!!!!!` banners, raw `std::cout`). **FIX** the three inherited `logger.hpp`
bugs before anything layers on it: `escapeJSONString` declared but never applied (frame corruption),
a dead `sendDebugMessages` decl, and a manual `update()` racing the background flush task.

**18.7 — Tiering (pulled forward — the key fix).** Observability does **not** wait for M6.
`TermSink` + `DebugRecord` + fault codes + per-motion result + loop-overrun + the logger bug-fixes
land at **M2**, so the M2–M3 localization/control milestones — where we fight for `< 1°` and tune
fusion gates — are field-debuggable *as they're built*. `SdSink` blackbox + estimator-introspection
quantities (residual/Mahalanobis/covariance) land at **M3** (the accuracy milestone's evidence).
`SHUL/2` wire + replay stay **M6**; the on-brain HUD is an **add-on**; cloud/auto-tune stay Frontier.

---

## 19. Glossary & References

- **Override scoring objects:** **Pin** (two colored halves, ~1.6″×6.5″), **Cup** (transparent +
  opaque, ~3.15″×6.5″), **Goal** (9; tall center "Midfield" + 4 short neutral + 4 alliance),
  **Toggle** (4, one/quadrant, 3 states), **Loader** (match-load intake).
- **Pilons odometry:** the 5225A arc-based 3-wheel tracking method (kept as a pure function).
- **Key rules:** `<VUR1>` (two robots 24"/15"),
  `<VUR12>` (onboard processors legal), `<VUG3>` (LIDAR/spinning sensors), `<RSC4>`/`<VURS1>`
  (skills field setup + GPS strip), `<R13c>` (vision wireless off in match).
