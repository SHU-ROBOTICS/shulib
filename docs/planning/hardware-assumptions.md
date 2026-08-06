# Hardware Assumptions Register

> **What this is.** Every claim about physical hardware that shulib v2 currently relies on but
> **cannot check without a robot** — inventoried, falsifiable, and owned. Three chunks of work
> (A1–A3) plus the M1/M2 conversion layers were built against these claims; this document is what
> converts that from a silent risk into a plan. Created by chunk A4
> ([brief](chunks/A4-assumptions-register.md)); it becomes **Phase R's checklist**: R3 walks the
> R3 group top to bottom on hardware day one, R4 the characterization group, R5/R6 the rest.
>
> **How to read an entry.** Each is a claim stated so it can be proven *false* — a number with
> units, or a definite assertion. Fields:
> - **Claim** — the falsifiable statement.
> - **Source** — where the tree currently assumes it (file:line at A4 close; the `HA-nn` tag
>   embedded at that spot is the durable anchor if lines drift).
> - **Confidence** — one of:
>   - **measured elsewhere** — a documented VEX/PROS spec we have not confirmed on our units;
>   - **reasoned** — derived from documentation, community reports, or arithmetic, unverified;
>   - **invented** — a number we made up to have *a* number. Invented is a legitimate and useful
>     value: a guess *labeled* as a guess is exactly what this register exists to protect. A guess
>     dressed as a measurement is the failure mode.
> - **Settle** — the specific bench measurement or test that decides it, and the owning chunk
>   (R3 conventions/geometry · R4 noise/drift/characterization · R5 gains · R6 plant back-fit).
> - **Blast radius if wrong** — the highest-value field: what breaks and how far it propagates.
>   Where the damage is contained *by design* (behind the `hal/pros` seam, or behind a single
>   config constant), the entry says so — that containment is a real, deliberate result of the
>   architecture, worth recording alongside the risk.
>
> **Rules of this document.**
> 1. **Nothing here is resolved on the host.** If an entry could be settled without a robot, its
>    presence here is a bug — settle it and note that instead. (No such entry was found at A4.)
> 2. **Entries are settled only with cited evidence** (the R-phase measurement, logged), then
>    marked `[x]` with the measured value recorded next to the guess it replaces. A corrected
>    value is the system *working* — the register predicted and localized the defect.
> 3. **The list is supposed to be long.** Its length is the honest cost of building without
>    hardware. Do not shrink it by quiet resolution.
> 4. Labels in code: `PROVISIONAL (A4: HA-nn)` on config fields; `A4 register HA-nn` in prose
>    comments. Reconciliation is bidirectional and grep-verified (see §Reconciliation).
>
> **Status: 0 of 52 settled.** No robot exists. Counts: **36 invented · 13 reasoned · 2 measured
> elsewhere · 1 mixed** (HA-44: documented shape, unmeasured onset). HA-50–52 added by chunk C1
> per the Maintenance convention (the first post-A4 contributor).

---

## Index

| ID | Claim (short) | Confidence | Owner |
|---|---|---|---|
| HA-01 | GPS position axes: +X = East, +Y = North | **invented** | R3 |
| HA-02 | IMU as-mounted heading is CW-positive | reasoned | R3 |
| HA-03 | `get_rotation()` is cumulative/unbounded (the required binding) | reasoned | R1/R3 |
| HA-04 | IMU yaw-rate source sign (`get_gyro_rate().z` undocumented) | **invented** | R3 |
| HA-05 | Post-cal tare invalidates bootHeading; offset applied once | reasoned | R1/R3 |
| HA-06 | Firmware GPS offset unset ⇒ `get_position()` = SENSOR pose | reasoned | R1/R3 |
| HA-07 | GPS `get_error()` returns meters | reasoned | R3 |
| HA-08 | Failed reads return PROS_ERR/PROS_ERR_F; adapters screen to stale-finite | reasoned | R1/R3 |
| HA-09 | `northHeadingDeg = 90°` matches our field setup | reasoned | R3 |
| HA-10 | GPS lever arm value (body inches) as configured | **invented** | R3 |
| HA-11 | Rotation `get_position()` cumulative; int32 centidegree never wraps in a match | reasoned | R1/R3 |
| HA-12 | Tracking-wheel offsets/signs as configured (−3.0″/−4.5″ are stand-ins) | **invented** | R3 |
| HA-13 | Tracking-wheel effective diameter = nominal 2.0″ | **invented** | R3 |
| HA-14 | Drive wheel 3.25″, wheel↔shaft gearing 1:1 | **invented** | R3 |
| HA-15 | Drive cartridge GREEN → 900 ticks/rev at output | **invented** | R3 |
| HA-16 | Rotation sensor 36000 ticks/rev (centidegree) | measured elsewhere | R3 |
| HA-17 | Built drivetrain matches the frozen preset geometry | **invented** | R3 |
| HA-18 | F5 numbers match on-V5 (host ≡ robot, swapping `RobotContext`) | reasoned | R3 |
| HA-19 | Brownout kills motors first; CPU survives; the run continues | reasoned | R3 |
| HA-20 | IMU per-boot drift ≤ 1°/min (typical 0.1–0.5) | **invented** | R4 |
| HA-21 | IMU heading noise σ ≈ 0.05° | **invented** | R4 |
| HA-22 | IMU yaw-rate noise σ ≈ 0.5°/s | **invented** | R4 |
| HA-23 | IMU calibration ≈ 2 s, emits garbage-that-moves while not-ready | reasoned | R3/R4 |
| HA-24 | IMU end-to-end heading latency ≈ 10 ms | **invented** | R4 |
| HA-25 | Encoder refresh ≈ 1 tick (10 ms) | reasoned | R4 |
| HA-26 | GPS on-strip position noise σ ≈ 0.7 in/axis | **invented** | R4 |
| HA-27 | GPS heading noise σ ≈ 1° | **invented** | R4 |
| HA-28 | GPS fresh-fix cadence ≈ 50 ms | **invented** | R4 |
| HA-29 | GPS claims rms ≈ 1.0″ healthy / ≈ 99″ no-fix; claim ≠ truth | **invented** | R4 |
| HA-30 | GPS end-to-end latency ≈ 50 ms | **invented** | R4 |
| HA-31 | Real off-strip/no-fix stale-pose behaviour (model: origin) | **invented** | R3/R4 |
| HA-32 | ~100 Hz loop sustained under full stack load | reasoned | R3 |
| HA-33 | PROS sensor reads are non-blocking (µs-class) | **invented** | R4 |
| HA-34 | Loop jitter ±20%, 2% spikes at ×5 | **invented** | R4 |
| HA-35 | `bootSettleTime` 0.1 s covers the worst data-path latency | **invented** | R4 |
| HA-36 | `driftHorizon` 12″ matches real dead-reckon drift decay | **invented** | R4 |
| HA-37 | Traction breaks above ≈ 80 in/s² wheel accel | **invented** | R4 |
| HA-38 | A slipping wheel still propels ≈ 70% of its spin | **invented** | R4 |
| HA-39 | Field foam is traction-uniform (one slip pair suffices) | **invented** | R4 |
| HA-40 | Pack sag ≈ 0.02 V per commanded volt | **invented** | R4 |
| HA-41 | Pack discharge ≈ 0.005 V/s under match load | **invented** | R4 |
| HA-42 | Brownout trips ≈ 10.5 V, recovers ≈ 10.8 V | **invented** | R3/R4 |
| HA-43 | Motor thermal: 0.0023 °C/(V²s) heat, 0.01 /s cool, 25 °C ambient | **invented** | R4 |
| HA-44 | Throttle steps 55/60/65 °C → 50/25/12.5% | mixed | R4 |
| HA-45 | Wheel FF placeholders: kS = 1.0 V, kV = 12/70, kA = 0 | **invented** | R5 |
| HA-46 | Fresh pack nominal ≈ 12.6 V | measured elsewhere | R4 |
| HA-47 | `move_voltage` is true voltage control (sag only bites at the ceiling) | reasoned | R5 |
| HA-48 | FF-inversion + first-order lag is an adequate plant shape | **invented** | R6 |
| HA-49 | Unmodeled 2.5 A current limiting changes no host-phase conclusion | **invented** | R6 |
| HA-50 | C1 motion gains + speed budget (kP 3.0/4.0, 60 in/s, 6 rad/s, 60 in/s wheels) | **invented** | R5 |
| HA-51 | C1 settle tolerances (0.5″/1.15° + rate floors; brake 1.2 in/s; 5 s timeout) | **invented** | R4/R5 |
| HA-52 | Stall cross-check thresholds (0.3 s window, 1.0″ spin, 25% ratio, stand-in radii) | **invented** | R3/R4 |

---

## Group R3 — conventions, bindings, geometry (hardware day one)

Walk top to bottom with the robot on a bench and then on a field tile. Every entry here is either
a **frame/sign convention** (a wrong guess mirrors or rotates the world), a **binding contract on
the R1 adapters** (a wrong binding is a bug the adapter must not have), or a **geometry constant**
(a wrong value is a measured correction). All of them are contained behind the `hal/pros` seam or
a config constant **by design** — a correction here never touches the core.

- [ ] **HA-01 — GPS position axes: VEX +X = East, +Y = North.**
  *Claim:* `pros::Gps::get_position()`'s x axis is compass East and y is compass North. False if
  the axes are swapped, negated, or bound to the field rather than the compass.
  *Source:* `include/shulib/hal/gps_conversion.hpp:15–21` (the flagged ASSUMPTION block);
  skipped oracle `test/gps_conversion_test.cpp:164`.
  *Confidence:* **invented** — PROS does not document the position-axis→compass binding at all.
  The oldest entry in this register (flagged "validate-on-field" since June).
  *Settle (R3):* the field-cal oracle: place the GPS at a known +1 m-East / +1 m-North point at a
  known heading, read raw `get_position()`/`get_heading()`, replace the oracle's expected values
  with the measured mapping, **unskip it**. It stays green forever after.
  *Blast radius if wrong:* a wrong axis label silently **MIRRORS** every GPS pose, and
  `northHeadingDeg` cannot recover it (a rotation knob cannot undo a reflection). Downstream: the
  E2 corrector would propose reflected fixes that fight odometry — Mahalanobis gating would reject
  most (quality decay, GPS effectively dead) and accept the rest near the mirror axis (real pose
  corruption). **Contained by design:** the fix is a few lines in `gpsSensorPose()` — one
  conversion function, zero core impact. The oracle exists precisely so this is found on a bench,
  not mid-match.

- [ ] **HA-02 — the as-mounted IMU heading is CW-positive, per its own doc strings.**
  *Claim:* a physical +90° clockwise spin increases `get_rotation()` by ~+90 (and therefore
  decreases canonical heading by 90°).
  *Source:* `include/shulib/hal/imu_conversion.hpp:9–13, 29–32`.
  *Confidence:* reasoned — both `get_rotation()` and `get_heading()` document "clockwise rotations
  are positive" in the vendored PROS headers; unverified against a physical sensor as mounted.
  *Settle (R3):* bench: rotate the robot +90° CW against a wall/protractor; canonical heading must
  decrease by 90°.
  *Blast radius if wrong:* globally negated heading — odometry arcs mirror left/right, every turn
  goes the wrong way. Caught within seconds by the R3 push/spin test. **Contained:** the sign of
  one subtraction in `imuHeadingToCanonical` is the documented line to flip.

- [ ] **HA-03 — `pros::Imu::get_rotation()` is cumulative and unbounded.**
  *Claim:* `get_rotation()` continues past ±360° without wrapping (unlike `get_heading()`), so the
  adapter's required binding preserves revolution continuity.
  *Source:* `include/shulib/hal/imu_conversion.hpp:16–19` (binding contract).
  *Confidence:* reasoned — documented "theoretically unbounded"; untested across many revolutions
  on a real unit.
  *Settle (R1 review + R3):* adapter code review pins the binding; bench: spin > 360° and confirm
  no discontinuity in canonical heading.
  *Blast radius if wrong (or mis-bound):* a ±360° step once per revolution; `PilonsOdometry`'s
  trust gate flags the tick (|Δθ| > π/2) but heading authority is corrupted from then on.
  **Contained:** adapter-only fix; the conversion function is already cumulative-correct.

- [ ] **HA-04 — the IMU yaw-rate source's sign convention.**
  *Claim:* differentiating `get_rotation()` yields a CW-positive rate (provably, since HA-03);
  `get_gyro_rate().z`'s sign — the cheaper source — is **undocumented** and assumed CW-positive if
  ever used.
  *Source:* `include/shulib/hal/imu_conversion.hpp:52–61` (the adapter caveat).
  *Confidence:* **invented** for `get_gyro_rate().z` (no documentation exists); reasoned for the
  differentiate-`get_rotation()` path.
  *Settle (R3):* bench: spin CW at a steady rate; the canonical yaw rate must be negative. Do it
  for whichever source the R1 adapter picks.
  *Blast radius if wrong:* a negated yaw rate breaks the drift-observability property Phase E
  depends on (heading and rate must move together — `imu_hostility.hpp` models them consistently
  for exactly this reason) and inverts E2's high-yaw-rate rejection. **Contained:** one negate in
  `imuYawRateToCanonical` / adapter source choice.

- [ ] **HA-05 — any post-calibration tare desyncs bootHeading; the additive offset is applied
  exactly once.**
  *Claim:* `tare()`/`set_rotation()`/`set_heading()`-family calls after calibration re-zero the
  sensor independently of shulib's bootHeading offset (silently shifting canonical heading), so
  the adapter must never call them; bootHeading's ONE owner is the robot's canonical start pose.
  *Source:* `include/shulib/hal/imu_conversion.hpp:20–26`.
  *Confidence:* reasoned — follows from PROS's documented tare semantics; the *discipline* is a
  contract on R1 code that does not exist yet.
  *Settle (R1 review + R3):* adapter review (no tare-family calls anywhere); bench: confirm a
  deliberate mid-run `tare_rotation()` on a scratch program shifts the raw stream (documenting the
  hazard is real), and that our adapter path never does.
  *Blast radius if wrong:* a silent constant heading offset from the tare instant onward — eats
  the < 1° budget invisibly, unexplainable from telemetry. **Contained** only by discipline plus
  R3's bench check; this is why it is registered rather than assumed.

- [ ] **HA-06 — with no firmware offset configured, `get_position()` reports the SENSOR, not the
  robot center.**
  *Claim:* constructing `pros::Gps` port-only (never `set_offset()`/`initialize_full()`) leaves
  `get_offset() == (0,0)` and `get_position()` = sensor pose, so shulib's own lever-arm removal
  (`gpsRemoveLeverArm`) is applied exactly once.
  *Source:* `include/shulib/hal/gps_conversion.hpp:27–34` (binding contract).
  *Confidence:* reasoned — from PROS's documented offset semantics; untested against firmware.
  *Settle (R1 review + R3):* boot-check `get_offset() == (0,0)` in the adapter; bench: compare a
  raw fix against tape-measured sensor position.
  *Blast radius if wrong:* the lever arm subtracted twice → a constant bias that *rotates with
  heading* (inches, direction-dependent — nastier than a fixed offset). **Contained:** adapter
  construction path + the boot check that exists to catch exactly this.

- [ ] **HA-07 — `pros::Gps::get_error()` returns METERS.**
  *Claim:* the device's self-reported rms error is in meters and must scale ×39.37 to inches at
  the HAL edge.
  *Source:* `include/shulib/hal/gps_conversion.hpp:39–40`.
  *Confidence:* reasoned — PROS documents meters; unverified against a live device.
  *Settle (R3):* read `get_error()` on-strip; a healthy fix should report ~0.01–0.05 (meters), not
  ~1–2 (inches already).
  *Blast radius if wrong (scale skipped or double-applied):* E2's measurement covariance R is ~39×
  off. Too small → good fixes gated out, GPS silently dead (quality decay — honest but wasteful);
  too large → lies accepted too readily. **Contained:** one multiply in the R1 adapter.

- [ ] **HA-08 — failed/disconnected reads return the documented PROS sentinels, and screening
  them at the edge yields stale-finite values in the core.**
  *Claim:* a dead/absent device read returns `PROS_ERR`/`PROS_ERR_F` (= +∞ for floats) rather
  than garbage or a hang; R1 adapters screen these so the core only ever sees stale-but-finite
  values (the F4 finiteness contract) — for GPS specifically, screening to `hasFix() == false`
  BEFORE conversion.
  *Source:* `include/shulib/hal/gps_conversion.hpp:35–38`; breach modeled on purpose in
  `include/shulib/sim/hostile/encoder_hostility.hpp` (sentinel window).
  *Confidence:* reasoned — sentinels are PROS-documented; the full unplug matrix is unverified.
  *Settle (R3):* the unplug session — disconnect each sensor live (IMU, GPS, Rotation, motor) and
  log the raw returns; confirm every adapter screens to the F4 contract.
  *Blast radius if wrong:* a leaked +∞ reaches the estimators. **Contained twice, and proven at
  A3:** the conversion functions throw on sentinels (fail-loud backstop), and `PilonsOdometry`'s
  last-resort finite guard freezes position rather than integrating — the A3 breach attack
  measured ~1.05″ of bounded damage and zero NaNs, with `HealthMonitor` raising `ODO_STUCK`.
  (`math::Angle` cannot even carry a NaN, by construction.)

- [ ] **HA-09 — `northHeadingDeg = 90°`: VEX-North points toward canonical +Y (away from red) on
  our field setup.**
  *Claim:* the default rotation parameter matches how our field/alliance is physically laid out
  (the alternative canonical value is 270°).
  *Source:* `include/shulib/hal/gps_conversion.hpp:22–26`.
  *Confidence:* reasoned — a setup convention with exactly two sane values, owned by the same
  authority as the robot's start pose; unverifiable until a strip is hung.
  *Settle (R3):* on-field: a fix at a known corner must land at that corner's canonical
  coordinates, not rotated.
  *Blast radius if wrong:* every fix rotated 180° (or 90°) — instantly obvious on the R3 field
  check, gated out wholesale before that. **Contained:** one config value with one owner.

- [ ] **HA-10 — the GPS lever arm equals the configured value.**
  *Claim:* the sensor's body-frame mounting position (forward/left inches) used by
  `gpsRemoveLeverArm` matches the physical mount.
  *Source:* `include/shulib/hal/gps_conversion.hpp:28–34` (ownership contract; the value itself
  cannot exist yet — no robot).
  *Confidence:* **invented** until a robot exists to measure.
  *Settle (R3, re-checked at E5):* tape-measure the mount; verify with a spin test (a wrong lever
  arm makes stationary-spin GPS fixes trace a circle of radius = the error).
  *Blast radius if wrong:* heading-dependent position bias in every fix, worst during rotation —
  degrades E2's gains from GPS exactly when correction is most needed. **Contained:** one config
  constant; E5's calibration routine re-measures it after any rebuild.

- [ ] **HA-11 — Rotation-sensor binding: cumulative `get_position()`, centidegree int32, no wrap
  within a match.**
  *Claim:* `pros::Rotation::get_position()` is cumulative (never wraps at 360°), and its int32
  centidegree range (~6×10⁴ revolutions ≈ miles of travel) is unreachable in a match.
  *Source:* `include/shulib/localization/tracking_wheel.hpp:12–16`.
  *Confidence:* reasoned — documented behavior + arithmetic; unverified on-device.
  *Settle (R1 review + R3):* adapter review; bench: roll a tracking wheel through > 5 revolutions
  both directions, confirm monotone cumulative readings through reversals.
  *Blast radius if wrong (mis-bound to a wrapping angle):* travel deltas spike by a full
  revolution at each wrap → position jumps. **Contained:** adapter-only; `TrackingWheel` is
  already written against the cumulative contract.

- [ ] **HA-12 — tracking-wheel offsets and direction signs match the configured geometry.**
  *Claim:* the physical mounts' perpendicular offsets (forward wheel: +LEFT coordinate; lateral
  wheel: +FORWARD coordinate) and rolling-direction signs equal the robot config. The harness
  defaults (−3.0″, −4.5″) are **stand-ins for a robot that has not been designed**.
  *Source:* `include/shulib/sim/scenario.hpp:99–108`; roles/axes contract in
  `include/shulib/localization/tracking_wheel.hpp:17–24`.
  *Confidence:* **invented** — there is no robot, not even a CAD-final mount.
  *Settle (R3):* tape-measure both offsets; then the in-place spin test — a pure rotation (both
  directions) must yield ~zero center travel (the classic odom-offset bug detector, already pinned
  host-side in the pure-rotation tests); then the push test (shove a measured distance, odometry
  must agree).
  *Blast radius if wrong:* phantom translation of Δθ·(offset error) **every tick of every turn** —
  the same mechanism A3's boot-poison attack exploited (10.8″ from garbage swings), at
  smaller-but-permanent scale; dead-reckoning cannot heal it. **Contained:** two config constants;
  every line of offset-correction *math* is already host-pinned, so R3 measures values, not logic.

- [ ] **HA-13 — tracking-wheel effective diameter = nominal (2.0″ in the harness).**
  *Claim:* the wheel's effective rolling diameter equals its nominal spec within ~1%.
  *Source:* `include/shulib/sim/scenario.hpp:102`; `tracking_wheel.hpp:15–16`.
  *Confidence:* **invented** — wear, compression and manufacturing tolerance are real and unknown.
  *Settle (R3, re-run at E5):* roll-calibration — push the robot a tape-measured distance (≥ 100″
  for 1% resolution); scale = measured/read. A2 proved the harness *detects* exactly this class:
  a deliberate 2% diameter lie read 51.0″ against a true 50.0″.
  *Blast radius if wrong:* a uniform scale error on ALL travel (1% diameter = 1% of every
  distance) — bounded, corrector-healable, and the single most common odometry defect in practice.
  **Contained:** one calibration constant, persisted by E5.

- [ ] **HA-14 — drive wheels are 3.25″ and wheel↔motor-shaft gearing is 1:1.**
  *Claim:* the plant's drive-encoder synthesis (shaft ← ∫spin/r, no external ratio term) matches
  the eventual drivetrain: 3.25″ wheels driven 1:1 from the cartridge output.
  *Source:* `include/shulib/sim/drive_plant.hpp:135–138`.
  *Confidence:* **invented** — the drivetrain is not designed.
  *Settle (R3):* count teeth / read the build; one wheel revolution by hand must read one motor
  revolution × ratio on the encoder.
  *Blast radius if wrong:* drive-encoder-derived speeds/distances scale wrong. **Limited by
  design at M2:** odometry reads *tracking* wheels, not drive encoders, so pose is untouched;
  affects R5 sysid bookkeeping and any future drive-encoder fallback odometry. Config constants.

- [ ] **HA-15 — drive motors carry the GREEN cartridge: 900 ticks/rev at the output shaft.**
  *Claim:* our drive uses green (200 RPM) cartridges — red is 1800, blue 300.
  *Source:* `include/shulib/sim/hostile/encoder_hostility.hpp:63` (`driveTicksPerRev`).
  *Confidence:* **invented** — the cartridge choice belongs to a build team that has not made it.
  *Settle (R3):* read the cartridge color; spin one output revolution, confirm the tick count.
  *Blast radius if wrong:* same family as HA-14 (scale on drive-encoder quantities; M2 pose
  untouched). Also invalidates the ≈70 in/s free-speed figure inside HA-45's kV placeholder —
  the two move together. One constant.

- [ ] **HA-16 — the V5 Rotation Sensor resolves 36000 ticks/rev (centidegree).**
  *Claim:* tracking-encoder quantization is at the centidegree grid.
  *Source:* `include/shulib/sim/hostile/encoder_hostility.hpp:64` (`trackingTicksPerRev`).
  *Confidence:* measured elsewhere — VEX-documented spec, unconfirmed on our units.
  *Settle (R3):* one careful hand revolution against an index mark.
  *Blast radius if wrong:* tracking travel scale error (folds into HA-13's roll-calibration,
  which measures the *product* of diameter and resolution — so R3's push test catches both at
  once). One constant.

- [ ] **HA-17 — the built drivetrain matches the frozen preset's idealized geometry.**
  *Claim:* the physical X-drive has true 45° symmetric mounts at equal radii (making the
  `xDrive()` coefficient columns orthogonal and `forward()` an exact inverse); the eventual H-bot
  likewise matches its C3 kinematics when built.
  *Source:* `include/shulib/kinematics/x_drive.hpp:1–34` (geometry derivation + the new register
  note).
  *Confidence:* **invented** — the robot is not built; the preset encodes the *intended* design.
  *Settle (R3/R5):* R3's push/strafe/spin signature checks (forward at V ⇒ wheels at V/√2;
  in-place spin ⇒ all wheels equal; strafe symmetric with forward); residual asymmetry falls out
  of R5's sysid fit.
  *Blast radius if wrong:* small mount asymmetry → small systematic twist error, absorbed by
  closed-loop control and R5 tuning; gross mismatch → a corrected coefficient table
  (`MatrixKinematics` exists precisely so a drivetrain is *data*, and relaxing orthogonality is
  the already-planned C3 pseudo-inverse — F5-safe by design).

- [ ] **HA-18 — F5's numbers match on the V5.**
  *Claim:* the same twist produces identical wheel commands in a host test and on the robot,
  swapping only `RobotContext` (M1's open on-robot clause; F5 is host-frozen).
  *Source:* roadmap.md F5 row ("on-V5 number-match pending"); build-order R3.
  *Confidence:* reasoned — pure IEEE-double arithmetic with no libm in `toWheels()`; ARM
  cross-compilation of the exact code is CI-verified (this chunk), but execution on a V5 is not.
  *Settle (R3):* run the prepared number-match routine; compare printed wheel commands
  bit-for-bit against the host test's.
  *Blast radius if wrong:* would indicate toolchain-level floating-point divergence — motion
  still *works* (F5's math is host-proven and closed-loop-corrected); the number-match is M1's
  evidence bar, so a mismatch blocks the badge and triggers a toolchain investigation, not a
  redesign.

- [ ] **HA-19 — brownout kills the motors first; the CPU survives and the program keeps
  running.**
  *Claim:* at pack collapse the brain cuts motor power while the CPU rides the supercap — the run
  itself continues.
  *Source:* `include/shulib/sim/hostile/power_hostility.hpp:17–21`; `HealthMonitor`'s latched
  brownout marker assumes the monitor is still alive to latch it.
  *Confidence:* reasoned — VEX-documented behavior and community experience; unverified on our
  hardware/firmware version.
  *Settle (R3):* controlled brownout on a bench pack (or current-limited supply): confirm motors
  die, program continues, telemetry keeps flowing.
  *Blast radius if wrong:* **the largest single blast radius in this register.** The
  guaranteed-park design (F2 chunk — §14 calls it non-negotiable, plausibly the
  highest-expected-value code in the library) *presupposes a CPU that outlives the motors*. If
  the brain instead resets, the park guard cannot exist as designed and F2's scheduling approach
  needs a rethink (e.g., park earlier on a voltage trend, not at collapse). NOT contained by a
  constant — this is why it must be settled at R3, before F2's design is trusted on a field.

---

## Group R4 — noise, drift, latency, timing, power, traction (characterization)

The numbers behind every hostile model. A3's convention: the failure **shapes** are confident and
test-pinned; the **magnitudes** are these entries. R4 replaces each with a measurement and re-runs
the suite (the models take the measured values by config — zero code motion). None of these are
settleable before hardware, and none block any host chunk.

- [ ] **HA-20 — IMU per-boot rate bias is ≤ 1°/min (typical 0.1–0.5°/min).**
  *Claim:* a calibrated V5 IMU's per-boot yaw-rate bias magnitude does not exceed 1°/min.
  *Source:* `include/shulib/sim/hostile/imu_hostility.hpp:71` (`rateBiasMax`); consumed by the
  live M2 acceptance test (`test/accuracy_spec_test.cpp:53`).
  *Confidence:* **invented** — a pessimistic community-folklore bound, not a measurement.
  *Settle (R4):* ≥ 10 boots, 60 s stationary each, log heading vs time; the fitted slope
  distribution replaces both the bound and the "typical" range. Build-order R4 names the 60 s
  drift number as a deliverable.
  *Blast radius if wrong:* **the F2 `< 1°` ceiling itself.** At exactly 1°/min the margin is zero
  by construction (1°/min × 60 s = 1°); A3 measured the stack adding ~nothing on top (worst
  end-of-60s 0.912°, worst instantaneous 1.065°). If the real bound is worse, the M2 acceptance
  fails on hardware **with the stack blameless**, and E3's heading correction stops being margin
  and becomes a requirement for F2. If better (likely), everything gains margin. Either way: a
  config value and re-run, no logic change — the acceptance test's scope note already says
  exactly this.

- [ ] **HA-21 — IMU heading white noise σ ≈ 0.05°.**
  *Source:* `imu_hostility.hpp:73`. *Confidence:* **invented**.
  *Settle (R4):* stationary high-rate log; σ of the detrended heading stream.
  *Blast radius if wrong:* `SettledUtil` deadbands and E4's measurement covariance were sized
  against fiction — loops may chatter at settle (σ larger) or tolerances are needlessly loose
  (σ smaller). Constants only.

- [ ] **HA-22 — IMU yaw-rate white noise σ ≈ 0.5°/s.**
  *Source:* `imu_hostility.hpp:75`. *Confidence:* **invented**.
  *Settle (R4):* same stationary log, rate channel.
  *Blast radius if wrong:* E-phase rate weighting and any rate-based rejection thresholds
  (E2 high-yaw-rate gate) miscalibrated. Constants only.

- [ ] **HA-23 — IMU calibration takes ≈ 2 s and emits moving garbage while not-ready.**
  *Claim:* `isReady()` stays false ~2–3 s from power-on, and readings during the window are
  large-magnitude and *changing* (not frozen zeros).
  *Source:* `imu_hostility.hpp:69` (`calibrationEnd`) and `:79` (`calibrationGarbageRate` — shape
  only).
  *Confidence:* reasoned — VEX documents ~2 s calibration; the garbage-that-MOVES shape is
  community-observed and is the exact pathology that exposed the A3 boot-poison defect.
  *Settle (R3/R4):* log heading + `isReady()` from power-on across boots; characterize window
  length and in-window behavior.
  *Blast radius if wrong:* if real boots are *longer*, autons that don't wait violate the
  documented consumer contract (motion before quality leaves `Uninitialized` is unaccounted);
  if in-window readings are *frozen* rather than moving, the Localizer's boot guard is
  over-defensive (harmless). The guard itself is shape-robust — pinned against both a garbage
  window and a ready-from-construction boot.

- [ ] **HA-24 — IMU end-to-end heading latency ≈ 10 ms.**
  *Source:* `include/shulib/sim/hostile/latency_hostility.hpp:54`. *Confidence:* **invented**
  (smart-port refresh cadence used as a stand-in for an unmeasured pipeline).
  *Settle (R4):* step-response rig — snap the robot through a known angle, cross-correlate
  commanded/physical motion against the reported stream.
  *Blast radius if wrong:* transient heading error scales as ω·L (at 1.5 rad/s, 20 ms is already
  1.7° — transiently past the entire F2 budget), and the stop-drain phantom translation scales as
  ω·L·|offset| (A3 §3.6). E2's latency compensation consumes the real number; a wrong guess here
  mis-sizes it. Also feeds HA-35.

- [ ] **HA-25 — encoder refresh ≈ 1 tick (10 ms).**
  *Source:* `latency_hostility.hpp:56–57` (documented; defaulted 0 in the composed model,
  exercised explicitly by test). *Confidence:* reasoned — the smart-port refresh cadence is
  documented; the effective end-to-end value is not measured.
  *Settle (R4):* same rig as HA-24 on a driven wheel.
  *Blast radius if wrong:* wheel-travel/heading pairing skew inside each odometry tick — bounded,
  small, and precisely the class the settle window + E2 compensation absorb. Constants.

- [ ] **HA-26 — GPS on-strip position noise σ ≈ 0.7 in/axis.**
  *Source:* `include/shulib/sim/hostile/gps_hostility.hpp:73`. *Confidence:* **invented**
  (field-test folklore).
  *Settle (R4):* stationary on-strip logs at several field positions; per-axis σ.
  *Blast radius if wrong:* E2's R and gate widths; fusion weighting quality. Constants only —
  and the decimation/double-count *hazard* the corrector must survive is magnitude-independent
  and already pinned.

- [ ] **HA-27 — GPS heading noise σ ≈ 1°.**
  *Source:* `gps_hostility.hpp:74`. *Confidence:* **invented**.
  *Settle (R4):* same logs, heading channel.
  *Blast radius if wrong:* almost none structurally — heading is IMU-owned by decision #4 and no
  corrector can rotate the robot; the number only informs whether GPS heading is ever worth a
  cross-check. Recorded for honesty, not fear.

- [ ] **HA-28 — GPS fresh-fix cadence ≈ 50 ms.**
  *Source:* `gps_hostility.hpp:75` (`updatePeriod`). *Confidence:* **invented**.
  *Settle (R4):* timestamp distinct fixes in a long log; the inter-fix distribution.
  *Blast radius if wrong:* the double-counting hazard's severity for E2 (a ~100 Hz consumer sees
  each fix N times; N is this entry) and the information rate available for correction. Constants.

- [ ] **HA-29 — GPS claims rms ≈ 1.0″ when healthy / ≈ 99″ at no-fix — and its self-estimate is
  not its real error.**
  *Claim:* `get_error()` reports ~an-inch-class value on-strip and a sentinel-large value at
  no-fix; the gap between claimed and actual error is real and must be survived.
  *Source:* `gps_hostility.hpp:76–77` (`reportedRms`, `noFixRms` — deliberately decoupled from
  the actual noise σ in the model).
  *Confidence:* **invented** (both numbers and the size of the claim-vs-truth gap).
  *Settle (R4):* joint log of `get_error()` vs measured error on- and off-strip.
  *Blast radius if wrong:* E2's *adaptive* R maps claimed→trusted; a wildly different mapping
  changes gating behavior. The design premise (never trust the claim raw) already assumes this
  entry is only approximately right. Constants.

- [ ] **HA-30 — GPS end-to-end latency ≈ 50 ms.**
  *Source:* `latency_hostility.hpp:55`. *Confidence:* **invented** (capture+solve+transport
  guess).
  *Settle (R4):* move at a known velocity across the strip; cross-correlate reported vs true
  position streams.
  *Blast radius if wrong:* E2's lever-arm/latency compensation mis-sized (a v·L position skew per
  fix); **HA-35's adequacy directly depends on this number** — see there. Constants.

- [ ] **HA-31 — what the real GPS serves while it has no fix is unknown (the model serves
  origin-until-first-fix, then stale).**
  *Claim (falsifiable):* the real device's no-fix `get_position()` behavior — whatever it is —
  is finite and is *screened by the adapter* so no code path ever consumes it as a pose.
  *Source:* `gps_hostility.hpp:15–22` (model semantics + the register note).
  *Confidence:* **invented** (the model's origin choice is deliberately adversarial, chosen so
  code that trusts a no-fix pose gets dragged to (0,0) and caught — not a prediction of the
  device).
  *Settle (R3/R4):* observe raw off-strip/covered behavior; write the adapter screen against
  what is actually seen (sentinels → HA-08).
  *Blast radius if wrong:* none while the `hasFix` contract holds — the IGps contract
  (no-fix pose is unspecified-but-finite, callers must check) is exactly what makes the real
  behavior not-matter. This entry exists so R3 *verifies the contract holds on the real device*
  rather than assuming the model was prophetic.

- [ ] **HA-32 — a loaded V5 sustains the ~100 Hz loop.**
  *Claim:* the full stack (odometry + localizer + motion + telemetry) completes a tick in ≲10 ms
  sustained, keeping per-tick Δθ ≪ 1 rad — the premise behind arcStep's wrap-horizon
  precondition and the pilons trust-gate calibration.
  *Source:* `include/shulib/localization/arc_step.hpp:49–56` (precondition);
  `include/shulib/localization/pilons_odometry.hpp:29–34` (gate note);
  `composed.hpp` `JitterScheduleConfig.nominal{0.01}`.
  *Confidence:* reasoned — 10 ms is the standard PROS task cadence and the M2 stack is small;
  unmeasured under our full eventual load (E4 EKF + sinks + vision).
  *Settle (R3, re-checked each phase):* `LoopMonitor` (A1) on the first real runs — it exists
  precisely to measure this; overruns raise `LOOP_OVERRUN` with the observed dt.
  *Blast radius if wrong:* slower loops enlarge per-tick Δθ (aliasing horizon approaches — at
  even 20 ms the margin is still enormous for real yaw rates), coarsen control discretization
  (the derived instability thresholds in the closed-loop tests scale with dt), and inflate all
  latency-relative effects. Structure survives — the suite already passes under ×5 jitter
  stalls — but budgets get recalibrated. Watch, don't fear.

- [ ] **HA-33 — PROS sensor reads are non-blocking (µs-class, no ms-scale stalls).**
  *Claim:* `get_position()`/`get_rotation()`/GPS reads return cached values quickly rather than
  blocking on the bus.
  *Source:* no in-tree code assumes a *number* — the assumption lives in the loop-budget math of
  HA-32 (build-order names "PROS call latency" as register seed content; the R1 adapters, when
  written, inherit this entry).
  *Confidence:* **invented** — PROS documents nothing about call cost.
  *Settle (R4):* microbench each adapter call with `IClock` timestamps over thousands of calls;
  record worst-case.
  *Blast radius if wrong:* HA-32 breaks from below — the loop budget is spent inside reads; the
  fix is batching/caching in adapters (an R1-layer change, core untouched).

- [ ] **HA-34 — loop jitter ±20% with ~2% spikes at ×5 nominal.**
  *Source:* `include/shulib/sim/hostile/composed.hpp:48–50, 205–207` (`JitterScheduleConfig`).
  *Confidence:* **invented** — PROS task-contention statistics are unmeasured.
  *Settle (R4):* dt histograms from real-run telemetry (the A1 `DebugRecord` carries per-tick
  dt; `LOOP_OVERRUN` counts spikes).
  *Blast radius if wrong:* the robustness *envelope* host tests certified is mis-shaped — the
  suite proves survival under THIS schedule (Pid convergence through ×5 stalls is pinned); a
  wilder real distribution means re-running the suite under the measured schedule, which is a
  config swap in `JitterScheduleConfig`.

- [ ] **HA-35 — `bootSettleTime = 0.1 s` covers the worst sensor data-path latency.**
  *Claim:* 100 ms past first-ready is enough for a delayed heading stream to flush its
  boot-boundary garbage (2× the HA-30 guess, the worst modeled latency).
  *Source:* `include/shulib/localization/localizer.hpp:81–87`.
  *Confidence:* **invented** — derived from HA-24/HA-30, which are themselves invented.
  *Settle (R4):* set from the *measured* worst latency (HA-24/HA-30/HA-25) with margin; verify
  with instrumented real boots (the fused pose must not move while stationary through
  ready-transition — the A3 attack, run on hardware).
  *Blast radius if wrong:* too small → the §3.2 leak returns on real hardware (A3 measured
  3.65″ pre-fix; the settle window is what closed it); too large → wasted milliseconds at auton
  start. One constant, and the leak signature (pose jump at exactly ready+ε while stationary) is
  now known and greppable in telemetry.

- [ ] **HA-36 — `driftHorizon = 12″`: dead-reckon trust should decay to floor by about a foot of
  travel.**
  *Claim:* real odometry drift accumulates at a rate that makes ~12″ of uncorrected travel the
  right quality-decay scale.
  *Source:* `include/shulib/localization/localizer.hpp:75–78`.
  *Confidence:* **invented** — a policy constant expressing an unmeasured drift rate.
  *Settle (R4):* measured drift-vs-travel from repeated dead-reckon runs (push/figure-eight
  scripts against tape marks); set the horizon where error crosses the useful-fix threshold.
  *Blast radius if wrong:* quality *reporting* miscalibrated (gates too eager or too trusting);
  no pose math involved. One constant.

- [ ] **HA-37 — traction breaks above ≈ 80 in/s² commanded wheel acceleration.**
  *Source:* `include/shulib/sim/hostile/slip_hostility.hpp:61`. *Confidence:* **invented** —
  our wheels/foam combination has never existed.
  *Settle (R4):* commanded-accel ramps while comparing drive-encoder spin vs tracking-wheel
  truth; the divergence onset is the threshold.
  *Blast radius if wrong:* C-phase profile aggressiveness (max accel choices) and the realism of
  slip-scenario tests. Lower threshold → real launches slip more than modeled → tune profiles
  down (a constants change R5 owns anyway).

- [ ] **HA-38 — a slipping wheel still propels ≈ 70% of its spin.**
  *Source:* `slip_hostility.hpp:62`. *Confidence:* **invented**.
  *Settle (R4):* measured travel deficit during induced slip (the A3 exact-accounting test shape
  — truth 64″ vs 80″ encoder-implied — run on hardware).
  *Blast radius if wrong:* slip-window realism in scenarios; no estimator logic depends on the
  value (tracking odometry rides through drive slip *by architecture*, pinned at A3).

- [ ] **HA-39 — the field surface is traction-uniform: one (threshold, retain) pair suffices.**
  *Claim:* foam-tile traction does not vary enough across the field to need a per-region model.
  *Source:* `slip_hostility.hpp:28–31` (register note).
  *Confidence:* **invented** — build-order's "field surface variation" seed, given claim form.
  *Settle (R4):* repeat HA-37's ramp at several field locations/orientations; compare onsets.
  *Blast radius if wrong:* location-dependent slip the model can't express without extension
  (slip windows already provide the escape hatch: declared per-region scenarios); affects skills
  route planning more than the library.

- [ ] **HA-40 — pack sag ≈ 0.02 V per commanded volt (≈1 V at four motors × 12 V).**
  *Source:* `include/shulib/sim/hostile/power_hostility.hpp:71`. *Confidence:* **invented**.
  *Settle (R4):* log battery voltage vs commanded load steps.
  *Blast radius if wrong:* brownout-margin planning and `Feedforward`'s battery compensation
  realism; constants. (The *shape* — sag follows load and recovers — is the pinned part.)

- [ ] **HA-41 — pack discharge ≈ 0.005 V/s under match load (≈0.3 V per 60 s run).**
  *Source:* `power_hostility.hpp:72`. *Confidence:* **invented**.
  *Settle (R4):* full-match logs, fresh vs tired packs.
  *Blast radius if wrong:* end-of-match brownout probability estimates; constants.

- [ ] **HA-42 — the brain cuts motor power at ≈ 10.5 V and recovers at ≈ 10.8 V.**
  *Source:* `power_hostility.hpp:73` (`cutoffVolts`) **and**
  `include/shulib/diag/health_monitor.hpp:66–69` (`brownoutVolts`/`brownoutRecoverVolts`) — two
  deliberately-aligned copies; they must move together when measured.
  *Confidence:* **invented** (community-folklore threshold; the hysteresis width is pure guess).
  *Settle (R3/R4):* bench brownout (HA-19's session): note the actual cutoff and recovery
  voltages under load.
  *Blast radius if wrong:* `HealthMonitor` brownout episodes fire early/late/chatter, and the
  F2 park guard's collapse-margin math is off. Two constants (cross-referenced in both headers).

- [ ] **HA-43 — motor thermal: heating ≈ 0.0023 °C/(V²·s), cooling ≈ 0.01 /s toward 25 °C
  ambient.**
  *Source:* `power_hostility.hpp:74–76`. *Confidence:* **invented** (chosen so full-drive
  reaches ~55 °C in ~90 s — a plausible story, not data).
  *Settle (R4):* `IMotor::temperature()` logs across sustained-load runs; fit both rates.
  *Blast radius if wrong:* when-does-droop-arrive predictions for skills route planning;
  constants.

- [ ] **HA-44 — V5 thermal throttling steps at 55/60/65 °C to 50/25/12.5% — and the onset on
  OUR motors is at the documented values.**
  *Source:* `power_hostility.hpp:77` (`throttleTempC`), model steps in the same header;
  `health_monitor.hpp:70–71` (`maxMotorTempC = 55` — deliberately equal to the first step).
  *Confidence:* **mixed** — the step *shape* is measured elsewhere (VEX-documented current
  limiting); the onset temperatures on our units are unmeasured (invented until R4).
  *Settle (R4):* drive to droop; correlate `temperature()` with observed speed steps.
  *Blast radius if wrong:* `MOTOR_OVER_TEMP` fires at the wrong time relative to the actual
  droop (warning without droop, or droop without warning); constants in two aligned places.

---

## Group R5 — gains and actuation constants

- [ ] **HA-45 — the plant's wheel feedforward placeholders: kS = 1.0 V, kV = 12/70 V·s/in
  (≈ 70 in/s free speed at 12 V), kA = 0.**
  *Claim:* the real drive's sysid constants are within order-of-magnitude of these.
  *Source:* `include/shulib/sim/drive_plant.hpp:130–135` (declared PLACEHOLDERS since A2).
  *Confidence:* **invented** — right order of magnitude for a V5 drive by construction, nothing
  more. (Free speed co-depends on HA-14/HA-15: 200 RPM green × 3.25″ ≈ 34 in/s at the wheel for
  1:1 — the 70 in/s figure is deliberately generous; R5 measures reality.)
  *Settle (R5):* `tools/sysid` least-squares over one on-robot ramp routine → committed
  constants; R6 feeds them back into this config and re-runs the whole suite.
  *Blast radius if wrong:* every sim-tuned gain and every open-loop distance in sim is
  quantitatively off — **which is already the declared position**: the plant proves logic, not
  constants; C-phase gains are provisional until R5 *by explicit rule* (build-order Phase C
  preamble). No conclusion currently drawn from the suite depends on these values being right.

- [ ] **HA-46 — a fresh pack reads ≈ 12.6 V.**
  *Source:* `include/shulib/sim/drive_plant.hpp:141` (`batteryVoltage` default).
  *Confidence:* measured elsewhere — documented Li-ion pack behavior.
  *Settle (R4):* trivially, with the first battery log.
  *Blast radius if wrong:* negligible — a sim default; the F4 `IBattery` path reads reality.

- [ ] **HA-47 — `move_voltage` is true voltage control: a 6 V cruise on a sagged 11.6 V pack is
  untouched (sag only bites at the ceiling).**
  *Claim:* V5 firmware applies commanded voltage without a hidden internal compensation loop, so
  partial-throttle behavior is pack-independent until demand exceeds the sagged ceiling (A3's
  §3.7 modeling insight, promoted to a testable claim).
  *Source:* `power_hostility.hpp` shape 2 (pack ceiling); A3-COMPLETED §3.7.
  *Confidence:* reasoned — matches PROS documentation of `move_voltage` semantics; PWM-level
  firmware behavior unverified.
  *Settle (R5):* measure velocity at fixed partial commands across pack states during sysid.
  *Blast radius if wrong:* gains tuned at partial throttle would be pack-dependent (R5 would
  discover this in its own data); the plant's power model semantics get adjusted at R6.

- [ ] **HA-50 — the C1 motion defaults: translation kP = 3.0 (1/s), heading kP = 4.0 (1/s),
  kI = kD = 0; maxLinearSpeed = 60 in/s, maxAngularSpeed = 6 rad/s, maxWheelSpeed = 60 in/s.**
  *Claim:* gains of this magnitude converge without oscillation on the real drive, and the speed
  budget keeps `Feedforward(maxWheelSpeed)` inside the 12 V rail with margin.
  *Source:* `include/shulib/motion/motion_config.hpp` (every field labeled PROVISIONAL (A4:
  HA-50)); the values converge on the A2 plant across the C1 sweep/routine suites — which by
  Phase C's own rule proves logic, not constants.
  *Confidence:* **invented** — tuned against placeholder plant dynamics (HA-45 co-dependency:
  wrong kV shifts what "60 in/s" costs in volts).
  *Settle (R5):* re-tune against measured kS/kV/kA; commit real values; R6 re-runs the suite.
  *Blast radius if wrong:* convergence speed and overshoot on hardware — contained by design:
  every C1 exit is watchdog-bounded and settle-verified, so a bad gain degrades to slow/TimedOut,
  never to divergence-with-a-green-light. No frozen contract encodes these numbers.

- [ ] **HA-51 — the C1 settle tolerances: translation 0.5 in / 1.0 in/s / 0.1 s; heading
  0.02 rad / 0.30 rad/s / 0.1 s; brake 1.2 in/s / 100 in/s² / 0.1 s (on a 5-tick averaged
  twist); default watchdog 5 s.**
  *Claim:* these thresholds sit ABOVE the real sensors' noise floors (else motions never settle
  on hardware) and BELOW competition-useful accuracy (else settling is meaningless). The
  heading-rate floor (0.30 rad/s) clears the HA-21-noise-differentiation floor (~0.12 rad/s at
  100 Hz) by ~2.5×; the brake threshold clears the measured M2 fused-twist noise floor at a
  physical dead stop under composed hostility (0.5–1.5 in/s raw, ~0.3–0.9 averaged — measured
  at C1, `drive_brake.hpp` header).
  *Source:* `include/shulib/motion/motion_config.hpp` (PROVISIONAL (A4: HA-51));
  `drive_brake.hpp` (the averaging + floor note).
  *Confidence:* **invented** — the floors they must clear are themselves provisional (HA-21/22).
  *Settle (R4 noise floors, R5 tolerances):* measure real estimator twist noise at a dead stop
  and real settle behavior; tighten or loosen with data.
  *Blast radius if wrong:* too tight ⇒ motions time out on a healthy robot (visible, bounded);
  too loose ⇒ arrival accuracy degrades toward the tolerance (bounded by it). Either failure is
  loud; neither corrupts state.

- [ ] **HA-52 — the spin-vs-motion cross-check thresholds: 0.3 s window, 1.0 in minimum spin
  travel, 25% motion ratio, wheel radius 1.625 in, rotation radius 7.0 in.**
  *Claim:* on real hardware the window/thresholds separate the three regimes they must —
  honest driving (ratio ≈ 1), A3-class slip (ratio ≈ 0.7, no fault), and a dead encoder or
  blocked drivetrain (ratio ≈ 0, ODO_STUCK) — with the stand-in radii close enough that the
  ratio comparison is not distorted.
  *Source:* `include/shulib/motion/odo_stall_check.hpp` (PROVISIONAL (A4: HA-52)); radii
  co-depend on HA-14 (wheel/gearing) and HA-17 (built geometry).
  *Confidence:* **invented** — the margins are provisional-vs-provisional (HA-40's 70% slip
  propulsion is itself a guess).
  *Settle (R3 geometry; R4 slip/noise):* measure real slip ratios and encoder noise; verify a
  hand-held wheel-stall raises ODO_STUCK within one window on the bench.
  *Blast radius if wrong:* too eager ⇒ spurious ODO_STUCK during aggressive maneuvers (a log
  smell, not a crash — the fault does not abort at C1); too lax ⇒ a dead encoder is caught by
  the watchdog instead (slower, still bounded). The E-phase estimator-side detector supersedes
  this check's load-bearing role.

---

## Group R6 — model-shape adequacy (settled by back-fit)

- [ ] **HA-48 — the FF-inversion + first-order-lag plant shape is adequate.**
  *Claim:* with R5's measured constants installed, the A2 plant reproduces a recorded real run
  within R6's documented tolerance — no torque curve, stiction model, or inertia tensor needed
  for control-relevant fidelity.
  *Source:* `include/shulib/sim/motor_model.hpp:6–17` (the honesty boundary + register note).
  *Confidence:* **invented** (a structural bet, deliberately conservative: the plant refuses to
  model what it cannot measure).
  *Settle (R6):* the back-fit — feed measured parameters in, replay a recorded run, compare
  trajectories; every newly-failing suite test is a real defect the invented parameters had been
  hiding (build-order R6's DoD verbatim).
  *Blast radius if wrong:* the model grows exactly where the residual says (that is R6's job) and
  the suite re-runs against the improved plant. Host *logic* conclusions stand — they were
  constructed not to depend on plant constants; what would change is sim-vs-real *quantitative*
  agreement.

- [ ] **HA-49 — leaving the 2.5 A stall current limit unmodeled changes no host-phase
  conclusion.**
  *Claim:* no result certified by the host suite (M2 acceptance, survival matrix, closed-loop
  convergence) would change if stall-current limiting were modeled.
  *Source:* `power_hostility.hpp:28–31` (the honest gap note).
  *Confidence:* **invented** — the gap is deliberate (a current model needs the load model the
  honesty boundary forbids), the no-consequence claim is the assumption.
  *Settle (R6):* record stall-adjacent events (wall pushes, jams) on hardware; back-fit; check
  which host results move.
  *Blast radius if wrong:* stall-adjacent behavior in sim is optimistic — affects F′-phase
  mechanism sequencing realism most (jam handling), where F1's hostile mechanism fakes model
  jams *independently* of the drive plant, by design.

---

## Considered and excluded (with reasons)

Recorded so "exhaustive" is checkable — these were examined and deliberately NOT registered:

| Item | Why excluded |
|---|---|
| F2 accuracy targets (< 1° heading, ~1″ pose, ~0.25″ docked) | **Requirements, not assumptions** — frozen at M0 (F2). The register tracks claims about hardware, not goals for it. HA-20 tracks whether the sensor can *meet* one. |
| The F4 ±12 V motor clamp / mV command units | Documented V5 API contract, encoded and tested as the F4 contract; nothing to falsify beyond HA-47's semantics claim. |
| GPS strip present in Autonomous Skills, absent in Driving Skills | Competition rules, not hardware; already designed for (off-strip mode, HA-31 covers the device side). |
| `Localizer` `maxDt`/`minDt` (0.1 s / 0.1 ms) | Self-referential policy bounds on *our own* loop, not claims about hardware; the hardware-facing part is HA-32/HA-34. |
| Cross-libm bit-identity of sim runs | A documented A2 determinism caveat about *host toolchains*, not about the robot; no hardware measurement settles it. |
| Stochastic truth-side hostility (random per-tick traction) | A3 D10/§3.8: a possible model extension, not an assumption — nothing at M2 needs it; revisit only if R4's data shows slip is noise-dominated (HA-37..39 will say). |
| Pneumatics/air budget, mechanism-specific constants | No mechanism exists even on paper (F′-phase, build-team-gated); F1's seam will register its own assumptions when authored. |
| AprilTag camera intrinsics/mount | R2/E3 territory with no code in-tree yet assuming values; register entries get added when E3 authors the PnP path (this register is living). |

---

## Reconciliation (bidirectional, grep-verified)

Every provisional label in the tree names its register entry, and every register entry that came
from a header points back at a label carrying its ID. Verified at A4 close by:

```sh
# Direction 1 — no label without a register ID (must print nothing):
grep -rn "PROVISIONAL (A4" include/ test/ | grep -v "HA-[0-9]"

# Direction 2 — every register ID cited from this file exists in the tree where claimed
# (script: extract HA-nn per source file above, grep each file for the ID; must find all).
```

Both directions were run clean at A4 close (see `chunks/A4-COMPLETED.md` for the captured
output). Entries with no in-tree source (HA-18, HA-33, HA-47 — sourced from the roadmap,
build-order seeds, and the A3 completion record respectively) are exempt from direction 2 and
say so in their Source field.

---

## Maintenance

- **Adding an assumption** (any later chunk that invents a magnitude): label it
  `PROVISIONAL (A4: HA-nn)` in-header and add the entry here — the A3→A4 pipeline is now the
  standing convention (C1 followed it: HA-50–52). Phase E (EKF noise priors) and F1 (mechanism
  fakes) are the known next contributors.
- **Settling an entry** (R3/R4/R5/R6): check the box, record the measured value beside the
  guess, update the in-header constant, cite the measurement log. If the measured value breaks
  a test, the test was resting on the guess — fix forward per the R6 rule (a new failure is a
  real defect that had been hiding).
- **This register is the R3 runbook.** Build-order R3 already commits to walking it top to
  bottom; keep the R3 group in bench-practical order.

---

*Created by chunk A4, 2026-08-06 — the chunk that closes Phase A. Companion documents:
[build-order.md](build-order.md) (Phase R owns settling), [roadmap.md](roadmap.md) (status),
[A3-COMPLETED.md](chunks/A3-COMPLETED.md) §8 (the 25-claim seed inventory this register
subsumes and extends to 49).*
