# Hardware Assumptions Register

> **What this is.** Every claim about physical hardware that shulib v2 currently relies on but
> **cannot check without a robot** — inventoried, falsifiable, and owned. Three chunks of work
> (A1–A3) plus the M1/M2 conversion layers were built against these claims; this document is what
> converts that from a silent risk into a plan. Created by chunk A4
> (2026-08-06); it becomes **Phase R's checklist**: R3 walks the
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
> **Status: 7 of 112 settled** (HA-94/95/96/97/99/100/101, all measured on the old competition bot
> 2026-08-13 — one robot, once; not proof of portability). HA-98 partially settled. No robot exists (a brain has booted the code; nothing has
> driven). Counts: **77 invented · 32 reasoned · 2 measured elsewhere · 1 mixed** (HA-44:
> documented shape, unmeasured onset). HA-50–52 added by chunk C1,
> HA-53 by chunk C2 (the cancel safe state), HA-54–55 by chunk C3 (the H-drive's strafe derate
> and stand-in geometry), HA-56–57 by chunk C5 (the D-5 plausibility envelope and the D-4
> controller-screen grid), HA-58–60 by chunk E1 (the blackbox's flight-recorder depth, RAM
> budget and assumed SD write cost — every magnitude in that chunk is a guess until R4),
> HA-61–67 by chunk E2 (the GPS corrector's tuning set), **HA-68–82 by chunk E3** (the
> AprilTag path — of which three, HA-68/69/70, are about the physical world rather than tuning
> and are the most dangerous entries in this document: each produces a *confidently* wrong fix
> rather than a degraded one), HA-83–91 by chunk E4 (the EKF's noise model — every number
> invented), HA-92–93 by chunk F1 (the mechanism layer's physics), and **HA-94–112 by chunk
> R1a** (the beliefs about PROS itself that the `hal/pros` adapters and their host shim are
> built on — the shim tests the adapters against these beliefs; ONLY a bench tests the
> beliefs), per the Maintenance convention.
> *(This status line was found stale at R1a — it read "0 of 82" while the register held 93
> entries: E4's and F1's additions never updated it. Corrected here; the per-chunk narrative
> above is the part a tool cannot regenerate, so it is the part that must be tended.)*

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
| HA-53 | Cancel safe state: 0 V + BrakeMode::Brake stops the drive promptly from speed | reasoned | R3/R5 |
| HA-54 | H-drive strafe traction derate ≈ 0.35 (⇒ authority 0.35 at 1:1 gearing) | **invented** | R5 |
| HA-55 | 15″ H-bot stand-in geometry (11″ track, strafe wheel 4″ aft, 1:1 strafe gearing) | **invented** | R3 |
| HA-56 | D-5 plausibility envelope defaults (maxSpeed 150 in/s, maxYawRate 20 rad/s, ×1.5 margin) | reasoned | R3/R5 |
| HA-57 | V5 controller text grid is 3 rows × 19 columns (`ILineDisplay` geometry) | reasoned | R1 |
| HA-58 | Flight-recorder depth: 200 ticks (~2 s) reaches back past a fault's cause | **invented** | R4 |
| HA-59 | 64 KiB of RAM is spendable on the blackbox staging buffer | **invented** | R4 |
| HA-60 | An SD flush of tens of KB costs single-digit ms (fine at a boundary, not in a tick) | **invented** | R4 |
| HA-61 | GPS `rmsError()` must be inflated ×2 to be used as sigma (claim ≠ truth) | **invented** | R4 |
| HA-62 | 0.5″ floor on the GPS measurement sigma | **invented** | R4 |
| HA-63 | A fix claiming > 6″ of error is not worth folding | **invented** | R4 |
| HA-64 | 3 rad/s (≈172°/s) is where a GPS fix stops being trustworthy | **invented** | R4 |
| HA-65 | 4σ normalized-innovation gate width | **invented** | R4 |
| HA-66 | The estimate is ≈1″ uncertain immediately after a fix is folded | **invented** | R4/E4 |
| HA-67 | Dead-reckon sigma grows ≈0.02″ per inch travelled (the anti-lockout term) | **invented** | R4 |
| HA-68 | The field's AprilTag layout is knowable and a team's map is right to ≈0.5″ | **invented** | R3 |
| HA-69 | The tag detector reports its four corners in a consistent, knowable WINDING | reasoned | R2/R3 |
| HA-70 | The tag camera is mounted level (no pitch/roll) to within ≈1° | **invented** | R3 |
| HA-71 | Tag pipeline latency ≈ 80 ms (exposure + detect + PnP + transport) | **invented** | R4 |
| HA-72 | A tag frame older than 0.25 s means the vision task has stalled | **invented** | R4 |
| HA-73 | A tag fix is trustworthy only between 6″ and 72″ of range | **invented** | R4 |
| HA-74 | A tag detection below 0.35 confidence is not worth folding | **invented** | R4 |
| HA-75 | 2 rad/s (≈115°/s) is where motion blur kills a tag fix | **invented** | R4 |
| HA-76 | Tag position 1σ ≈ 1.0″ + 0.02″ per inch of range | **invented** | R4 |
| HA-77 | 4σ normalized-innovation gate width for tag fixes | **invented** | R4 |
| HA-78 | The estimate is ≈1″ uncertain immediately after a tag fix | **invented** | R4/E4 |
| HA-79 | Dead-reckon sigma grows ≈0.02″ per inch since this source's last fix | **invented** | R4 |
| HA-80 | A heading innovation above 15° is a fault, not drift | **invented** | R4/E4 |
| HA-81 | 0.15 of the heading innovation per tick at confidence 1 | **invented** | R4/E4 |
| HA-82 | 10°/s is a safe upper bound on estimator heading-bias change | **invented** | R4/E4 |
| HA-83 | EKF position process noise: σ grows 2% of distance travelled, + a 0.5 in/s floor | **invented** | R4 |
| HA-84 | EKF heading process noise: σ grows 1% of rotation, + HA-20's 1°/min drift | **invented** | R4 |
| HA-85 | The drive can change body velocity by 200 in/s² (the EKF's velocity process noise) | **invented** | R4/R5 |
| HA-86 | One tick's odometry displacement is 1σ ≈ 0.01 in + 2% of the tick's travel | **invented** | R4 |
| HA-87 | 3σ is the right Mahalanobis gate width for the EKF tier | **invented** | R4 |
| HA-88 | An absolute heading measurement is 1σ ≈ 2°, flat | **invented** | R4 |
| HA-89 | The EKF's prior when it knows nothing: 24 in, 30°, 24 in/s | **invented** | R4 |
| HA-90 | 50 consecutive gate rejections with a mean innovation over 6 in means "lost" | **invented** | R4 |
| HA-91 | 5 s is long enough between re-init declarations to tell recovery from a storm | **invented** | R4 |
| HA-92 | `BrakeMode::Hold` at 0 V holds a LOADED cascade lift against back-drive | **invented** | R3/R5 |
| HA-93 | A jammed 11 W mechanism motor reads ≈ 2.5 A at a full 12 V command, with ≈ 0.05 rad/s residual creep | **invented** | R4 |
| HA-94 | Motor `move_voltage()` takes millivolts, ±12000 | **settled** measured-on-comp-bot 2026-08-13: 2.0 V commanded → device reported 1823–1990 mV under load | R3 |
| HA-95 | Motor `get_position()` with `MotorUnits::degrees` reports output-shaft degrees | **settled** measured 2026-08-13: deg/rad = 57.296 on all 8 motors (57.2958 exact) | R3 |
| HA-96 | Motor `get_actual_velocity()` returns output-shaft RPM | **settled** measured 2026-08-13: rpm/(rad/s) = 9.549 (9.5493 exact) | R3 |
| HA-97 | Motor `get_current_draw()` returns mA | **settled** measured 2026-08-13: mA/A = 1000.0 exactly, 8 motors | R3 |
| HA-98 | Motor units/gearing live in the DEVICE and persist across programs; explicit set + read-back holds | **partial** 2026-08-13: set+read-back accepted on 8 motors (no precondition fired); PERSISTENCE across programs NOT tested, so this stays open | R3 |
| HA-99 | Battery voltage/current are mV/mA — units NOT in the vendored source (website only) | **settled** measured 2026-08-13: raw 13039 → 13.04 V, raw 919 → 0.92 A. R1a's weakest belief, now observed | R3 |
| HA-100 | `battery_get_capacity()` returns percent 0–100 | **settled** measured 2026-08-13: raw 91.0 → 0.91 | R3 |
| HA-101 | `micros()` is µs since program start, monotonic, uint64 | **settled** measured 2026-08-13: 999784 µs over a nominal 1000 ms delay (0.02% low) | R3 |
| HA-102 | `Task::delay_until(prev, 10)` yields an anchored 100 Hz cadence | reasoned | R3 |
| HA-103 | Controller axes are [-127, 127]; disconnected reads 0, not a sentinel | reasoned | R3 |
| HA-104 | `get_digital()` is a level; `get_digital_new_press()` CONSUMES the press | reasoned | R3 |
| HA-105 | Rotation `get_velocity()` returns centidegrees/second | reasoned | R3 |
| HA-106 | `gps_status_s_t.yaw` is the CW-from-North heading in degrees | reasoned | R3 |
| HA-107 | Controller LCD columns: 19 (HA-57) vs the vendored doc's [0-14] ⇒ 15 — CONFLICT | **invented** | R3 |
| HA-108 | `Imu::reset(false)` starts calibration; one boot-time call is not a post-cal tare | reasoned | R3 |
| HA-109 | `imu_gyro_s_t` is deg/s and z is the yaw axis (sign is HA-04's entry) | **invented** | R3 |
| HA-110 | IMU as-mounted pitch/roll sign conventions | **invented** | R3 |
| HA-111 | main.cpp's port map and motor direction signs match the robot | **invented — and MEASURED WRONG for the comp bot** 2026-08-13: real map is drive LEFT 15/16/17/18, RIGHT 11/12/14, IMU 4, no rotation sensors, no GPS. main.cpp still carries the invented map | R3 |
| HA-112 | Teleop stick mapping signs + 0.05 deadband are drivable | **invented** | T2/R3 |

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
  *Source:* `include/shulib/hal/gps_conversion.hpp` — `gpsRmsErrorToCanonical()`.
  **Changed at E2:** from A4 until E2 this obligation existed only as PROSE in that header, with
  no function performing it and no test pinning it — an instruction addressed to a future adapter
  author, guarding the most dangerous silent factor in the GPS path. E2 gave it a function and an
  independent test. The test asserts against the DEFINITION of the inch (0.0254 m ≡ 1 inch)
  rather than against `kMetersToInches`, because the pre-E2 conversion tests imported that
  constant from the header under test — so a wrong constant would have satisfied both sides of
  the comparison. That hole is closed; the ASSUMPTION (that PROS really returns metres) is not,
  and still needs R3.
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
  *Source:* roadmap.md F5 row ("on-V5 number-match pending"); owned by R3, hardware day one.
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

### R1a's additions — the beliefs about PROS the adapters are built on

Every row of the `hal/pros` unit table and every semantic in the host shim
(`test` tree, PROS stand-ins) is one of these. The shim can only prove the adapters agree
with these beliefs — it can never prove the beliefs. The R1a bench runbook walks them in
dependency order (battery scale before anything drives, signs before any closed loop).

- [ ] **HA-94 — `move_voltage()` takes MILLIVOLTS in [-12000, 12000].**
  *Claim:* the argument is mV; 6 V of command is `move_voltage(6000)`.
  *Source:* vendored `include/pros/motors.hpp:234` ("from -12000 to 12000 in millivolts");
  `include/shulib/hal/motor_conversion.hpp` (`motorVoltageToMillivolts`, the one ×1000).
  *Confidence:* reasoned — documented in the vendored source; unverified against firmware.
  *Settle (R3, runbook step 4):* open-loop: a small commanded voltage produces proportional,
  plausible wheel speed.
  *Blast radius if wrong:* every command 1000× off — the robot either hums in place or
  saturates instantly. **Contained:** one conversion function; caught in the first minute of
  the bench.

- [ ] **HA-95 — `get_position()` with `MotorUnits::degrees` reports OUTPUT-SHAFT degrees.**
  *Claim:* after the adapter sets degrees, position is the output shaft's cumulative degrees
  (gearset already accounted for by firmware).
  *Source:* vendored `motors.hpp:628` ("absolute position in its encoder units");
  `hal/pros/motor.hpp` (the degree→radian binding).
  *Confidence:* reasoned — "degrees" is documented; *output-shaft* (vs internal rotor) is the
  weaker half of the belief.
  *Settle (R3, runbook step 4):* rotate a wheel one marked revolution by hand → position moves
  2π rad.
  *Blast radius if wrong:* odometry scaled by the cartridge ratio (6:1/18:1/36:1) — silently,
  nothing crashes. **Contained:** adapter-only.

- [ ] **HA-96 — `get_actual_velocity()` returns output-shaft RPM.**
  *Claim:* the double is RPM at the output shaft (gearset-corrected by firmware).
  *Source:* vendored `motors.hpp:404`; `motor_conversion.hpp` (`motorRpmToCanonical`, ×2π/60).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 4):* full-stick free spin ≈ the cartridge's rated RPM (green ≈ 200
  → ≈ 20.9 rad/s canonical).
  *Blast radius if wrong:* feedforward and the stall cross-check read speeds ~9.5× off.
  **Contained:** one conversion.

- [ ] **HA-97 — `get_current_draw()` returns milliamps.**
  *Claim:* the int32 is mA; a stalled 11 W motor reads ~2500.
  *Source:* vendored `motors.hpp:426` ("in mA"); `motor_conversion.hpp`
  (`motorMilliampsToCanonical`).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 5):* pinch a wheel at ≤3 V; current climbs into the amp range.
  *Blast radius if wrong:* stall/capture detection fires always or never — every
  sensor-confirm operation lies. **Contained:** one conversion.

- [ ] **HA-98 — motor units/gearing are DEVICE state, persist across programs, and an explicit
  set + read-back holds.**
  *Claim:* the ctor-default `invalid` means "leave as is"; a previous program's configuration
  survives; setting degrees+gearset then reading both back proves the device accepted them.
  *Source:* vendored `motors.hpp:74-75`; `hal/pros/motor.hpp` (ctor read-back preconditions);
  the shim's adversarial rotations/red defaults model exactly this hazard.
  *Confidence:* reasoned — the leave-as-is semantics are documented; persistence-across-boots
  is community knowledge.
  *Settle (R3, runbook steps 0/1/4):* boot with a correct map (read-backs pass); boot with one
  wrong port (read-back faults loudly).
  *Blast radius if wrong (adapter skips the discipline):* a motor left in rotations reads 1/360
  — odometry wrong by 360× with nothing crashing. **Contained:** ctor discipline + mutation-
  proven test (campaign M5).

- [ ] **HA-99 — battery voltage/current are mV/mA. THE WEAKEST UNIT BELIEF IN THIS REGISTER.**
  *Claim:* `battery_get_voltage()` ≈ 12600 on a fresh pack (mV); `battery_get_current()` is mA.
  *Source:* vendored `misc.h:718-750` — which documents **NO UNIT AT ALL** ("the current
  voltage of the battery"); the mV/mA belief is from PROS's website documentation only.
  `hal/pros/battery.hpp` carries the warning in its header.
  *Confidence:* reasoned, and flagged: the only load-bearing unit in R1a that the vendored
  source does not state.
  *Settle (R3, runbook step 2 — BEFORE anything drives):* the session header's battery line
  reads ~12.x V against the brain's own battery screen.
  *Blast radius if wrong:* a 1000× error silently destroys brownout compensation, which scales
  EVERY motor command. **Contained:** one adapter, but only if checked before driving — hence
  the runbook ordering.

- [ ] **HA-100 — `battery_get_capacity()` returns percent 0–100.**
  *Claim:* the double is a percentage; the adapter's ÷100 lands `IBattery::capacity()` in [0,1].
  *Source:* vendored `misc.h:771-787` (unit again undocumented; "Battery Level" example);
  `hal/pros/battery.hpp`.
  *Confidence:* reasoned.
  *Settle (R3, runbook step 2):* compare against the brain's battery percent.
  *Blast radius if wrong:* capacity() pegged at 1.0 by the clamp (a fraction read as percent)
  — misleading telemetry, no control impact today. **Contained.**

- [ ] **HA-101 — `micros()` is microseconds since program start, monotonic, uint64.**
  *Claim:* `IClock::now()` = micros()×1e-6 is a monotonic seconds stream; uint64 µs cannot wrap
  in a robot's lifetime (~584 000 years).
  *Source:* vendored `rtos.h:241`; `hal/pros/clock.hpp` (which also records WHY not millis():
  1 ms quantization is 10% of the 10 ms tick — a 10% error in every PID derivative).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 3):* the telemetry time column tracks a stopwatch 1:1 for 60 s.
  *Blast radius if wrong:* every dt in every controller — global, but impossible to miss on
  the bench. **Contained:** one adapter.

- [ ] **HA-102 — `Task::delay_until(prev, 10)` yields an anchored 100 Hz cadence.**
  *Claim:* wake at `*prev + delta` with `*prev` updated to the wake instant, so tick processing
  time does not stretch the period (unlike `delay(10)`, which drifts by the work done).
  *Source:* vendored `rtos.hpp:742-747`; `hal/pros/tick_pacer.hpp`.
  *Confidence:* reasoned — documented semantics; unmeasured under our stack's load (that half
  is HA-32/HA-34's).
  *Settle (R3, runbook step 12):* LoopMonitor dt stats over 60 s idle ≈ 10 ms mean.
  *Blast radius if wrong:* the loop runs slow by the tick body's cost and every profile
  integrates the error. **Contained:** one adapter.

- [ ] **HA-103 — controller axes are [-127, 127]; a DISCONNECTED controller reads 0, not a
  sentinel.**
  *Claim:* full deflection is ±127 (÷127 → exactly ±1), and disconnection is reported ONLY by
  `is_connected()` — the channels just read 0.
  *Source:* vendored `misc.hpp:85-86` (both halves documented); `controller_conversion.hpp`;
  `hal/pros/controller.hpp`.
  *Confidence:* reasoned.
  *Settle (R3, runbook step 10):* full stick = ±1.00 in telemetry; pull the controller
  mid-stick — the robot must stop via isConnected(), not keep driving on a stale value.
  *Blast radius if wrong:* the range half is cosmetic (clamped); the zero-on-disconnect half is
  the dangerous one — without the validity signal a dropped controller looks like centred
  sticks. **Contained:** the seam carries isConnected() precisely for this.

- [ ] **HA-104 — `get_digital()` is a level; `get_digital_new_press()` CONSUMES the press.**
  *Claim:* the new-press read clears per-device edge state, so with two consumers one silently
  loses; binding levels + per-consumer `ButtonEdge` gives every consumer the press.
  *Source:* vendored `misc.hpp:173-212`; `hal/controller.hpp` (ButtonEdge);
  `hal/pros/controller.hpp` (the ban, guard-tested).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 10):* hold a button — pressed() stays true (a level).
  *Blast radius if wrong (adapter binds new_press):* the second consumer of any button misses
  every press — an auton selector and a mechanism toggle fighting invisibly. **Contained:**
  mutation-proven (campaign M10) + a textual guard.

- [ ] **HA-105 — rotation `get_velocity()` returns centidegrees per second.**
  *Claim:* same centidegree scale as position; π/18000 converts both.
  *Source:* vendored `rotation.hpp:219-242`; `rotation_conversion.hpp`.
  *Confidence:* reasoned.
  *Settle (R3, runbook step 6):* hand-roll at a counted rate; velocity matches.
  *Blast radius if wrong:* the odometry stall cross-check compares spin to travel with a wrong
  scale on one side. **Contained:** one conversion.

- [ ] **HA-106 — `gps_status_s_t.yaw` is the CW-from-North heading in degrees.**
  *Claim:* the atomic status read's yaw field is the same quantity `get_heading()` documents
  ([0,360) CW from North), so one status read gives position AND heading from one sample.
  *Source:* vendored `gps.h:53-64` (field named "yaw", convention undocumented) +
  `gps.hpp:598-608` (get_heading's convention); `hal/pros/gps.hpp` binds the status read.
  *Confidence:* reasoned — the WEAKER half is that yaw ≡ heading's convention; the wrap range
  is immaterial (the conversion wraps).
  *Settle (R3, runbook step 13, strip required):* compare status.yaw against get_heading() and
  against a physically known orientation.
  *Blast radius if wrong:* every GPS pose's heading component wrong the same way — the E2
  corrector's heading channel poisoned. **Contained:** one binding choice in one adapter.

- [ ] **HA-107 — the controller LCD's real column count: HA-57 says 19, the vendored doc
  implies 15. A CONFLICT, found at R1a, that only a screen can settle.**
  *Claim (as shipped):* 19 columns (ILineDisplay::kCols, HA-57) is right and the vendored
  `misc.hpp:322` col-parameter range "[0-14]" is a stale doc comment.
  *Source:* `hal/line_display.hpp:18-21` (HA-57); vendored `misc.hpp:322`;
  `hal/pros/line_display.hpp` (which truncates at kCols and records the conflict).
  *Confidence:* **invented** — two written sources disagree and neither is a measurement.
  *Settle (R3, runbook step 11):* display the 19-char ruler `0123456789ABCDEFGHI`; count
  what shows. 15 visible → shrink kCols (one constant; the content layer already truncates
  through the seam) and mark HA-57 settled-false.
  *Blast radius if wrong:* the last 4 characters of every status row silently never display —
  degraded, not dangerous (the same rows ride the serial log in full). **Contained:** one
  constant.

- [ ] **HA-108 — `Imu::reset(false)` starts calibration; a single boot-time call is the
  calibration itself, not a post-cal tare.**
  *Claim:* reset() begins the ~2 s calibration (HA-23), `is_calibrating()` covers it, and one
  boot-time reset does not fall under HA-05's tare ban (which is about POST-calibration
  re-zeroing under a live bootHeading).
  *Source:* vendored `imu.hpp:141-145`; `hal/pros/imu.hpp` (calibrate(), single-call
  precondition).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 7):* time isReady() false→true from boot.
  *Blast radius if wrong (e.g., reset() is itself a tare-after-autocal):* a constant heading
  offset from boot — caught by step 7's protractor check. **Contained:** one call site.

- [ ] **HA-109 — `imu_gyro_s_t` is in deg/s and z is the yaw axis.**
  *Claim:* the raw gyro struct's units are deg/s and its z component is rotation about
  vertical. (The SIGN is HA-04's entry — undocumented, assumed CW-positive.)
  *Source:* vendored `imu.hpp:438-468` + `imu.h:84-87` (neither documents units);
  `hal/pros/imu.hpp` (the GyroRateZ branch).
  *Confidence:* **invented** — units asserted from community usage, not from the source.
  *Settle (R3, runbook step 8):* steady ~90°/s CW spin → |z| ≈ 90 raw and canonical ≈
  −1.57 rad/s.
  *Blast radius if wrong:* the alternate yaw-rate branch is garbage — but it is OFF by default
  (differentiation is the default source) and the bench measures before anyone flips it.
  **Contained by the default.**

- [ ] **HA-110 — the IMU's as-mounted pitch/roll sign conventions.**
  *Claim:* none yet, honestly: the adapter passes `get_pitch()`/`get_roll()` through unnegated
  BECAUSE the convention is unmeasured; whatever the bench records becomes the convention the
  tip detector is written against.
  *Source:* `hal/pros/imu.hpp` (pitch()/roll(), the unnegated pass-through note).
  *Confidence:* **invented** (a deliberate placeholder convention).
  *Settle (R3, runbook step 9):* nose-up 10° on a book → record pitch's sign; left side up →
  record roll's.
  *Blast radius if wrong:* tip detection reads a climb as a dive — but no tip detector ships
  yet, so today the radius is zero. Settle it before one does.

- [ ] **HA-111 — `src/main.cpp`'s port map and motor direction signs match the robot.**
  *Claim:* FL1 BL2 BR−3 FR−4 ROTF5 ROTL6 GPS9 IMU10, green cartridges — ALL INVENTED.
  *Source:* `src/main.cpp` (the `k*Port` block, labelled).
  *Confidence:* **invented — and MEASURED WRONG on 2026-08-13.** The real map on the old
  competition bot is: drive **LEFT 15/16/17/18, RIGHT 11/12/14** (green cartridges), **IMU on
  port 4**, and **no rotation sensors and no GPS at all**. Only "there are motors on low ports"
  survived. `src/main.cpp` still carries the invented map, deliberately — it also assumes an
  X-drive with two tracking wheels, which this robot is not and does not have, so fixing the
  ports alone would not make it correct. The wiring belongs to whichever chunk runs the shipped
  stack on this robot.
  *Also measured:* port 13 spins FREE (18 mA vs its neighbours' 500–950 mA, and it does not move
  when its wheels are turned by hand) — a mechanical disconnect, not a software matter. Port 16
  under-reports its side-mates by ~20% at an inconsistent ratio — probable slip, undiagnosed.
  *Settle (R3, runbook steps 0 and 4):* device inventory, then per-wheel spin direction —
  inventory and side-split DONE 2026-08-13; per-wheel DIRECTION SIGNS still open (the 2 V bursts
  moved several motors too little to read a sign).
  *Blast radius if wrong:* ports: loud at boot (by design). Signs: a mirrored wheel fights the
  other three — caught in the first open-loop spin, BEFORE any closed loop (runbook order).
  **Contained:** constants + boot checks.

- [ ] **HA-112 — the teleop mapping (stick signs, 0.05 deadband) is drivable.**
  *Claim:* left-stick-up = +X forward, left-stick-left = +Y left, right-stick-right = CW
  (−ω), and a 0.05 deadband kills centred-stick creep without killing fine control.
  *Source:* `src/main.cpp` (opcontrol(), labelled; chunk T2 owns the real driver-feel layer).
  *Confidence:* **invented**.
  *Settle (T2/R3, runbook step 10):* a human drives it; signs that feel backwards get flipped
  at the mapping, deadband tuned by feel.
  *Blast radius if wrong:* annoying, visible, and shallow — exactly the kind of wrong that is
  fine to ship provisionally. **Contained:** one function in main.cpp.

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
  HA-32 ("PROS call latency" was named as seed content when this register was planned; the R1
  adapters, when written, inherit this entry).
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
  *Confidence:* **invented** — the planning-time "field surface variation" seed, given claim form.
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

- [ ] **HA-58 — 200 ticks (~2 s at 100 Hz) of flight recorder reaches back past the CAUSE of a
  fault.**
  *Claim:* when a fault fires, whatever caused it is visible within the preceding ~2 seconds of
  per-tick records, so a 200-tick ring is deep enough to diagnose from.
  *Source:* `include/shulib/diag/sd_sink.hpp` `kDefaultFlightRingTicks` (PROVISIONAL
  (A4: HA-58)); consumed by every caller that sizes `SdSinkBuffers`.
  *Confidence:* **invented** — it is diagnostics-plan D-6's own figure, chosen before any real
  failure had been recorded. Nobody has yet measured how far before a fault its cause sits.
  *Settle (R4):* once real runs exist, take the dumps that actually diagnosed something and
  measure how far back the useful evidence started. If the answer is "further", the ring grows
  (a template/argument change and more RAM); if "much less", the ring shrinks and the dump gets
  cheaper.
  *Blast radius if wrong (too small):* a dump that starts after the cause — the fault is
  recorded, its origin is not, and the run has to be reproduced to learn anything. *(Too
  large):* RAM spent and a slower dump, both harmless.

- [ ] **HA-59 — 64 KiB of RAM is spendable on the blackbox staging buffer.**
  *Claim:* a V5 program running the full stack can permanently give 64 KiB to a diagnostics
  buffer (plus ~88 KB for a 200-tick ring of records) without pressuring anything else.
  *Source:* `include/shulib/diag/sd_sink.hpp` `kRecommendedBufferBytes` (PROVISIONAL
  (A4: HA-59)); the storage is caller-owned, so this is a recommendation, not a hard size.
  *Confidence:* **invented** — no memory budget for the real program exists yet.
  *Settle (R4):* measure free heap/static space with the competition build loaded, then set the
  recommendation to something the real program can actually afford.
  *Blast radius if wrong:* the number is a caller-chosen span, so being wrong costs a
  one-line change — and being wrong SMALL is visible rather than silent (frames are dropped and
  counted, and the count is reported in the run summary and written into the file).

- [ ] **HA-60 — an SD flush of tens of kilobytes costs single-digit milliseconds.**
  *Claim:* writing ~64 KiB to `/usd/` on the V5 brain takes a few ms — affordable at a motion
  boundary or at auton end, and NOT affordable inside a 10 ms control tick.
  *Source:* `include/shulib/diag/sd_sink.hpp`, on `SdSink::flush()` (PROVISIONAL (A4: HA-60));
  it is the reason writes are caller-paced at all, and the reason the fault dump is the one
  exception that may write immediately.
  *Confidence:* **invented** — the V5's SD write path has never been timed by this team. The
  legacy code's experience with V5 SERIAL backpressure (~900-byte chunks with delays) is
  suggestive of the same class of problem, but it is a different device.
  *Settle (R4):* time `fwrite` + `fflush` of 4/16/64/256 KiB on a real brain, cold and warm,
  and put the numbers here.
  *Blast radius if wrong (much slower):* a flush at a motion boundary steals loop time; the
  LoopMonitor would report it as an overrun, so it is at least VISIBLE. The fix is a policy
  change — flush less often, or only at auton end — not a format change. R1 should re-check
  this the first time the adapter runs on hardware.

### The E2 `GpsCorrector` tuning set (HA-61 … HA-67)

Seven constants, added when the first real corrector landed. They share one property worth
stating once instead of seven times: **E2 proved the corrector's LOGIC, and every one of these
numbers is a guess about a device nobody has plugged in.** Each is tested by an assertion about
*shape* — "a worse claim widens sigma", "the gate widens with dead-reckoned travel", "a spin
rejects the fix" — never by an assertion that the constant is right. Contained the same way as
the rest of Phase E's tuning: one config struct, one owner, no core impact.

They also share one settle: **HA-26 (the real on-strip noise) and HA-20 (the real dead-reckon
drift) decide whether folding GPS is worth doing at all.** E2's own measurements found the two
within the same order of magnitude in simulation, which is why the accuracy gain there is real
but modest (see `test/gps_corrector_accuracy_test.cpp`'s header). R4 settles both, and the
answer changes how these seven should be set.

- [ ] **HA-61 — the GPS's self-reported rms must be inflated ×2 before it is used as sigma.**
  *Claim:* the device's `get_error()` understates its true 1σ by roughly a factor of two, so
  trusting the claim raw makes the gate too tight and the pull too strong.
  *Source:* `include/shulib/localization/gps_corrector.hpp`, `GpsCorrectorConfig::rmsTrustFactor`
  (PROVISIONAL (A4: HA-61)). Follows from HA-29, which records that the claim and the truth are
  different numbers without saying by how much.
  *Confidence:* **invented** — the factor 2 is a placeholder for "more than one".
  *Settle (R4):* the same joint log HA-29 needs — `get_error()` against measured error, on
  strip, at several positions. The factor is the ratio of their standard deviations.
  *Blast radius if wrong (too small):* gate too tight, good fixes rejected, GPS quietly useless
  — visible as `RejectedNormalizedInnovation` filling the blackbox. (Too large): weak pull and
  a slack gate, so lies do more damage before the innovation bound catches them. Constant only.

- [ ] **HA-62 — 0.5 inch is a sane floor on the measurement sigma.**
  *Claim:* no GPS fix should ever be treated as better than half an inch, whatever the device
  claims.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::minPositionStdDev` (PROVISIONAL (A4: HA-62)).
  *Confidence:* **invented**.
  *Settle (R4):* set to the best measured per-axis sigma observed on strip.
  *Blast radius if wrong:* the floor exists to stop a device reporting ~0 from producing an
  arbitrarily tight gate that rejects every fix including truthful ones — that failure mode is
  pinned by test regardless of the value. Constant only.

- [ ] **HA-63 — a fix claiming more than 6 inches of error is not worth folding.**
  *Claim:* a device asserting `hasFix()` while reporting a large self-error should be declined
  rather than folded weakly.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::maxReportedRms` (PROVISIONAL (A4: HA-63)).
  *Confidence:* **invented**.
  *Settle (R4):* from the on-strip `get_error()` distribution — set above the healthy tail and
  below whatever the device reports when it is struggling.
  *Blast radius if wrong (too high):* a barely-usable fix is folded, contributing almost nothing
  while making the Localizer report quality class "Corrected" — a run that looks anchored and is
  not. That is the specific failure the ceiling exists for, and it is pinned by test. (Too low):
  usable fixes declined in poor conditions; visible as `RejectedSensorQuality`.

- [ ] **HA-64 — 3 rad/s (≈172°/s) is where a GPS fix stops being trustworthy.**
  *Claim:* above this yaw rate the fix's lever-arm reduction and its heading/position sampling
  skew make it worse than dead-reckoning for the tick.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::maxYawRate` (PROVISIONAL (A4: HA-64)).
  *Confidence:* **invented** — chosen as "clearly a fast spin, not a normal arc"; it is not
  derived from any measurement of how the camera behaves under rotation.
  *Settle (R4):* spin in place on the strip at increasing rates and log reported vs. true
  position; the knee is the threshold. Re-check at E5 whenever the lever arm is re-measured
  (HA-10).
  *Blast radius if wrong (too high):* fixes folded during spins carry a heading-dependent bias —
  exactly the error HA-10 warns about, at its worst. (Too low): the corrector goes quiet during
  every turn, which is a loss of information, not a corruption. The asymmetry is why the default
  errs low.

- [ ] **HA-65 — 4 sigma is the right normalized-innovation gate width.**
  *Claim:* a residual beyond 4× the fix's own 1σ is more likely a lie than a truth.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::gateSigma` (PROVISIONAL (A4: HA-65)).
  *Confidence:* **invented** — 4σ is the conventional engineering choice, not a measured one,
  and it is only as meaningful as the sigma it multiplies (HA-61, HA-66, HA-67).
  *Settle (R4):* from the measured residual distribution on a run with known truth.
  *Blast radius if wrong:* the classic gate trade — too tight rejects truthful corrections
  (stubbornness), too loose accepts lies (corruption). Both are VISIBLE in the blackbox as
  `RejectedNormalizedInnovation` counts, which is why E2 put the residual and the sigma on the
  wire. **Note the ceiling:** `ComplementaryFusion::innovationGate` (12 inches, fixed) applies
  after this gate, so widening `gateSigma` past ~5 with these sigmas changes nothing.

- [ ] **HA-66 — the estimate is ~1 inch uncertain immediately after a fix is folded.**
  *Claim:* the floor of the dead-reckoning sigma, which sets how hard a fresh fix pulls.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::postFixStdDev` (PROVISIONAL (A4: HA-66)).
  *Confidence:* **invented**. It is the closest thing the complementary tier has to a prior,
  and the honest description is "a gain knob wearing the clothes of a covariance".
  *Settle (R4):* properly, this is E4's job — an EKF estimates it instead of asserting it. Until
  then, set from measured post-correction error.
  *Blast radius if wrong:* it and HA-61 together set the confidence
  `σ_dr²/(σ_dr² + σ_meas²)`, i.e. how fast the estimate chases the GPS and therefore how much
  sensor noise it inherits. Must stay > 0: a zero would make confidence zero, the Localizer
  screens a zero-confidence proposal out, and the corrector would look absent rather than weak.
  Guarded by precondition.

- [ ] **HA-67 — dead-reckoning uncertainty grows ≈0.02 inch of sigma per inch travelled.**
  *Claim:* 2% of distance travelled, as a 1σ position uncertainty, since this source's last fix.
  *Source:* `gps_corrector.hpp`, `GpsCorrectorConfig::driftStdDevPerInch` (PROVISIONAL (A4: HA-67)).
  *Confidence:* **invented**, and related to but not the same as HA-36 (the Localizer's
  `driftHorizon`, which decays a quality SCALAR rather than widening a gate).
  *Settle (R4):* the same measurement as HA-36 — drive a known path with the GPS covered and
  measure how error grows with distance.
  *Blast radius if wrong (too small):* GATE LOCKOUT — after a long blind stretch, a truthful fix
  looks outrageous and is rejected, and since nothing else can repair the estimate, so is every
  fix after it. The GPS dies exactly when it is worth the most. This is the failure the term
  exists to prevent and it is pinned by a dedicated test. (Too large): the gate stays wide open
  after one long dead-reckon and a lie is accepted because the corrector still believes it is
  lost — pinned by the companion test that the widening RESETS on an accepted fix.

### The E3 AprilTag set (HA-68 … HA-82)

Fifteen entries, added when absolute yaw correction landed. They divide into **three that are
about the physical world and are the dangerous ones** (HA-68, HA-69, HA-70) and **twelve tuning
constants** that behave exactly like E2's set — proved by shape, never by value, contained in one
config struct each.

The three structural ones deserve their own warning, because unlike a tuning constant **they do
not degrade gracefully**. A wrong tag map, a reversed corner winding or a pitched camera each
produce a fix that is *confidently* wrong with a small residual and a high confidence — i.e. it
looks exactly like a healthy fix, and no gate, no filter and no amount of averaging can reveal it.
E2's tuning constants make the corrector work better or worse; these three make it lie.

- [ ] **HA-68 — the field's AprilTag layout is knowable, and a team's map is accurate to ≈0.5 inch.**
  *Claim:* the tags' field poses can be obtained (from a published specification or by measuring
  the actual competition field) to within about half an inch and half a degree, and the team will
  in fact do so.
  *Source:* `include/shulib/localization/tag_map.hpp` (A4 register HA-68). **shulib deliberately
  ships NO built-in map** — the header explains why at length — so this is an assumption about the
  *user's input*, not about a number in the tree, which is unusual for this register and is the
  point: there is no citable table of AprilTag field poses in this project's sources today.
  *Confidence:* **invented** — nobody here has measured or cited anything.
  *Settle (R3):* obtain the published field layout if one exists; otherwise measure every tag's
  position and facing on the competition field with a tape and a square, and record the method in
  each `TagPlacement::source`.
  *Blast radius if wrong:* **the worst in this register.** A map entry two inches off yields a
  corrector that is confidently two inches wrong every time it sees that tag, with a small
  residual and a high confidence. Sensor noise averages out; **a wrong map does not**. It will
  also fight the GPS corrector, and the complementary tier has no way to tell which is lying.
  Partly contained: `TagMap::add()` refuses an entry with no provenance, and `anyInvented()` lets
  telemetry say the estimate is anchored to guessed geometry.

- [ ] **HA-69 — the tag detector reports its four corners in a consistent, knowable winding.**
  *Claim:* whatever order the V5 AI Vision sensor (or a Pi detector) reports corners in, it is
  stable, and R2 can map it onto the order `hal/vision_conversion.hpp` documents.
  *Source:* `include/shulib/hal/vision_conversion.hpp`, the corner-order contract (A4: HA-69).
  *Confidence:* **reasoned** — every AprilTag implementation documents an order; ours is not
  verified against a physical detector.
  *Settle (R2/R3):* point the real detector at a real tag at a known pose and check the recovered
  pose, ONCE. That is the only check that exists (see the blast radius).
  *Blast radius if wrong:* **measured at E3 and worse than expected.** A CYCLIC ROTATION of the
  order is harmless — the planar reduction discards that degree of freedom. A **REVERSAL** is
  catastrophic and completely silent: it mirrors the tag plane, the recovered heading is 180°
  wrong, and **the reprojection error stays at machine zero**, because a mirrored pose reprojects
  onto the very same four pixels. No self-check anywhere in the pipeline can detect it; only a
  physical tag can. Pinned by three subcases in `test/vision_conversion_test.cpp`.

- [ ] **HA-70 — the tag camera is mounted level, to within about a degree.**
  *Claim:* the camera's optical axis is horizontal (no pitch, no roll), so a tag's height above
  the camera can be discarded without affecting its horizontal position.
  *Source:* `hal/vision_conversion.hpp`, the planar-reduction note (A4: HA-70). `CameraMount`
  carries position and yaw only, which encodes the assumption in the type.
  *Confidence:* **invented** — no camera is mounted on anything.
  *Settle (R3):* measure the mounted camera's pitch and roll; if either is significant, the
  reduction needs the full 3-D transform and `CameraMount` grows two fields.
  *Blast radius if wrong:* a pitched camera turns a tag's HEIGHT into a RANGE error — a tag
  mounted 15 inches above a camera pitched 5° reads several inches nearer or further than it is —
  and nothing downstream can see that it happened. Contained to one function and one struct.

- [ ] **HA-71 — the tag pipeline delivers a fix describing a moment ≈80 ms earlier.**
  *Claim:* exposure + detection + PnP + transport totals about 80 ms, longer than the GPS's ~50 ms
  (HA-30) because a tag pipeline does more work per frame.
  *Source:* `apriltag_corrector.hpp`, `AprilTagCorrectorConfig::latency` (PROVISIONAL (A4: HA-71)).
  *Confidence:* **invented**.
  *Settle (R4):* the same measurement HA-30 needs — flash a known event and timestamp when the
  reduced detection becomes readable.
  *Blast radius if wrong:* an uncompensated 80 ms drags position backwards along the direction of
  travel AND, new at E3, drags HEADING backwards along the direction of rotation — at 180°/s that
  is 14°, fourteen times the entire heading budget. Both compensations are pinned by their own
  exact-case tests; only the magnitude is at risk.

- [ ] **HA-72 — a tag frame older than 0.25 s means the vision task has stalled.**
  *Claim:* a healthy vision task at ~20 Hz never leaves a gap this long, so a gap means failure.
  *Source:* `apriltag_corrector.hpp`, `maxObservationAge` (PROVISIONAL (A4: HA-72)).
  *Confidence:* **invented**.
  *Settle (R4):* log inter-frame intervals on a real coprocessor under load; set to a few times
  the observed 99th percentile.
  *Blast radius if wrong (too small):* healthy frames declined as `RejectedObservationAge` —
  visible, and the estimator falls back to dead-reckoning rather than doing anything wrong.
  (Too large): a dead vision task keeps folding the last frame it saw for a quarter of a second
  longer than it should. Constant only.

- [ ] **HA-73 — a tag fix is trustworthy only between 6 and 72 inches of range.**
  *Claim:* nearer than 6 inches the tag overfills the frame and is likely clipped; further than
  72 inches planar PnP's near-ambiguity makes the ORIENTATION untrustworthy well before the
  position is.
  *Source:* `apriltag_corrector.hpp`, `minRange` / `maxRange` (PROVISIONAL (A4: HA-73)).
  *Confidence:* **invented** — the *shape* of the claim (heading degrades with range faster than
  position) is real geometry; both numbers are made up.
  *Settle (R4):* park the robot at a known pose and sweep range; plot recovered position error and
  recovered heading error against distance. The band is where heading error crosses the budget.
  *Blast radius if wrong:* this band is E3's substitute for a separate heading noise model, so a
  band that is too wide folds heading fixes that are worse than the IMU — the one way this chunk
  could make heading accuracy WORSE. Too narrow and the corrector is simply idle more often.
  E4's EKF replaces the band with a real R_heading, which is the principled fix.

- [ ] **HA-74 — a tag detection below 0.35 confidence is not worth folding.**
  *Claim:* the detector's own score below this indicates a detection too poor to trust.
  *Source:* `apriltag_corrector.hpp`, `minConfidence` (PROVISIONAL (A4: HA-74)).
  *Confidence:* **invented** — and it assumes the V5 sensor's score is even comparable across
  detections, which is itself unverified.
  *Settle (R4):* log the score against measured pose error across a range of lighting and angles.
  *Blast radius if wrong:* the same shape as E2's D7 — without a floor a 0.05-confidence detection
  is still folded with a microscopic pull, and the Localizer reports quality class **Corrected** on
  a run with no usable anchor. That failure mode is pinned by test regardless of the value.

- [ ] **HA-75 — 2 rad/s (≈115°/s) is where motion blur kills a tag fix.**
  *Claim:* above this yaw rate the tag smears across the frame and a rolling shutter skews it into
  a different quadrilateral, which PnP solves happily into a confidently wrong pose.
  *Source:* `apriltag_corrector.hpp`, `maxYawRate` (PROVISIONAL (A4: HA-75)). Tighter than the
  GPS's HA-64 (3 rad/s) because a camera integrating over an exposure is more motion-sensitive
  than a strip reader.
  *Confidence:* **invented**.
  *Settle (R4):* spin at increasing rates in front of a fixed tag and find where the recovered
  pose degrades.
  *Blast radius if wrong (too high):* blurred fixes folded — and unlike a noisy fix, a skewed one
  is biased, so it does not average out. (Too low): the corrector goes quiet during every turn,
  which is when the estimate needs it most.

- [ ] **HA-76 — tag position 1σ is about 1.0 inch, growing 0.02 inch per inch of range.**
  *Claim:* the linear-in-range noise model, at confidence 1.
  *Source:* `apriltag_corrector.hpp`, `baseStdDev` / `stdDevPerInch` (PROVISIONAL (A4: HA-76)).
  *Confidence:* **invented** — the linear form is a modelling choice as much as the coefficients.
  *Settle (R4):* the range sweep from HA-73 gives both.
  *Blast radius if wrong:* sets both the gate width and the pull strength. Tested by shape only —
  "farther is worse", "less certain is worse" — never by value.

- [ ] **HA-77 — 4 sigma is the right normalized-innovation gate width for tags.**
  *Source:* `apriltag_corrector.hpp`, `gateSigma` (PROVISIONAL (A4: HA-77)). Same value and same
  reasoning as E2's HA-65; kept separate because the sigma it multiplies is a different model.
  *Confidence:* **invented**.
  *Settle (R4):* choose from the measured innovation distribution once HA-76 is real.
  *Blast radius if wrong:* as HA-65 — too tight rejects truthful fixes, too slack admits lies.

- [ ] **HA-78 — the estimate is ≈1 inch uncertain immediately after a tag fix is folded.**
  *Source:* `apriltag_corrector.hpp`, `postFixStdDev` (PROVISIONAL (A4: HA-78)).
  *Confidence:* **invented**. Like HA-66 this is **a gain knob wearing the clothes of a
  covariance**; E4's EKF estimates it instead of asserting it.
  *Settle (R4/E4):* superseded by the filter's own covariance.
  *Blast radius if wrong:* sets the floor of σ_dr and therefore the confidence of every fix.

- [ ] **HA-79 — dead-reckon sigma grows ≈0.02 inch per inch since this source's last fix.**
  *Source:* `apriltag_corrector.hpp`, `driftStdDevPerInch` (PROVISIONAL (A4: HA-79)). The tag
  corrector's own copy of HA-67, per-source by design: each corrector tracks how long *it* has
  been blind.
  *Confidence:* **invented**.
  *Settle (R4):* as HA-67.
  *Blast radius if wrong:* **gate lockout**, exactly as HA-67 describes, and pinned by the same
  pair of tests (a fix a zero-growth build rejects is accepted after a long dead-reckon; the
  widening RESETS on an accepted fix).

- [ ] **HA-80 — a heading innovation above 15 degrees is a fault, not drift.**
  *Claim:* a raw V5 IMU drifts ≈1°/min (HA-20), so over a 60-second match the expected heading
  error is about a degree. An innovation fifteen times that is far more likely to be a mirrored
  corner winding (HA-69), a wrong map entry (HA-68) or a misidentified tag id than real drift.
  *Source:* `complementary_fusion.hpp`, `ComplementaryFusionConfig::headingGate`
  (PROVISIONAL (A4: HA-80)).
  *Confidence:* **invented**, and it inherits HA-20's uncertainty: if real IMU drift is much worse
  than 1°/min, this gate becomes the thing that prevents recovery.
  *Settle (R4/E4):* set from the measured IMU drift distribution (HA-20) once that exists; E4's
  EKF replaces it with a Mahalanobis test.
  *Blast radius if wrong (too tight):* a genuinely drifted IMU can never be corrected — the
  correction locks out exactly when it is needed, the heading analogue of HA-67's failure.
  (Too slack): a mirrored tag rotates the robot's entire idea of the field, and every
  field-relative command afterwards inherits it.

- [ ] **HA-81 — 0.15 of the heading innovation is pulled per tick at confidence 1.**
  *Source:* `complementary_fusion.hpp`, `maxHeadingGain` (PROVISIONAL (A4: HA-81)). Same value as
  the position gain, which is a convenience, not a derivation.
  *Confidence:* **invented**.
  *Settle (R4/E4):* superseded by the EKF's Kalman gain.
  *Blast radius if wrong:* sets the settling time. Measured in simulation: a 4° error closes to
  0.5° in about 3 seconds. Too high and the correction fights the turn controller; too low and a
  60-second match ends before the bias is learned.

- [ ] **HA-82 — 10°/s is a safe upper bound on how fast the estimator's heading bias may change.**
  *Claim:* the robot's idea of which way it is pointing may be revised at up to 10°/s without
  disrupting a motion in progress.
  *Source:* `complementary_fusion.hpp`, `maxHeadingNudgeRate` (PROVISIONAL (A4: HA-82)).
  *Confidence:* **invented**.
  *Settle (R4/E4):* drive a heading-holding motion while injecting a bias correction and find the
  rate at which the controller visibly reacts.
  *Blast radius if wrong (too high):* a large correction arrives fast enough to fight an active
  turn — this is the number that stands between "nudge" and "snap" for yaw, and a yaw snap
  poisons every field-relative command after it. (Too low): a real drift takes longer than a match
  to correct. **This bound is what makes never-snap structural for heading**, and it is audited
  on every tick from the blackbox (`correctionDTheta`), so a violation is visible after the fact.

- [ ] **HA-83 — the EKF's position process noise: 1σ grows by 2% of the distance travelled since
  the last accepted fix, plus a 0.5 in/s floor while standing still.**
  *Claim:* dead-reckoning error is dominated by a SYSTEMATIC scale and alignment error, so σ grows
  LINEARLY with distance rather than as its square root, and something small but non-zero
  accumulates even at rest.
  *Source:* `ekf_fusion.hpp`, `posNoisePerInch` / `posNoiseRate` (PROVISIONAL (A4: HA-83)).
  *Confidence:* **invented**. The 2% figure is E2's `driftStdDevPerInch` (HA-67) reused so the two
  layers model the same physics with the same number; the floor is a numerical-health term.
  *Settle (R4):* drive a known distance with no correction, repeatedly, and plot the error
  distribution against distance — the SHAPE of that plot (linear vs √d) settles the model, and its
  slope settles the constant.
  *Blast radius if wrong (too small):* **gate lockout.** Measured during E4 with a random-walk
  form in place: after 360 inches of dead-reckoning the filter believed it was within half an inch
  and rejected a truthful fix 20 inches away. (Too large): the gate never closes and a lie walks
  in.

- [ ] **HA-84 — the EKF's heading process noise: 1σ grows by 1% of the rotation actually
  performed, plus HA-20's ≈1°/min of raw IMU drift.**
  *Source:* `ekf_fusion.hpp`, `headingNoisePerRad` / `headingDriftRate` (PROVISIONAL (A4: HA-84)).
  *Confidence:* **invented**, and it INHERITS HA-20 entirely — the drift term is HA-20 restated as
  a variance, so if HA-20 is wrong this is wrong by the same factor.
  *Settle (R4):* the same boot-drift experiment HA-20 needs; the scale-factor term needs a
  known-rotation test (spin exactly ten turns, compare).
  *Blast radius if wrong (too small):* a genuinely drifted heading is never corrected, because the
  filter is certain about a heading that is wrong. (Too large): a mirrored tag moves the bias fast.

- [ ] **HA-85 — the drivetrain can change its body velocity by about 200 in/s².**
  *Claim:* this is roughly a hard V5 drive launch, and it is the EKF's process noise on the
  velocity states — i.e. how far the constant-velocity model is allowed to be wrong in one tick.
  *Source:* `ekf_fusion.hpp`, `velNoise` (PROVISIONAL (A4: HA-85)).
  *Confidence:* **invented**; co-depends on HA-45's feedforward constants.
  *Settle (R5):* falls straight out of the sysid ramp — the measured peak acceleration IS this
  number.
  *Blast radius if wrong (too small):* the filter lags the wheels through every acceleration, and
  the position estimate lags with it. (Too large): encoder noise passes straight through, which is
  mostly harmless.

- [ ] **HA-86 — one tick's odometry displacement is 1σ ≈ 0.01 inch plus 2% of that tick's travel.**
  *Claim:* the constant term is encoder quantization and tracking-wheel jitter; the proportional
  term is slip.
  *Source:* `ekf_fusion.hpp`, `odomStdDev` / `odomStdDevPerInch` (PROVISIONAL (A4: HA-86)).
  *Confidence:* **invented**. The quantization half is checkable on paper from HA-16 and HA-13 and
  is the more trustworthy of the two.
  *Settle (R4):* push the robot a measured distance by hand with the motors off and compare
  odometry to a tape measure, repeatedly.
  *Blast radius if wrong:* sets how much the filter smooths the wheels. Too small and it follows
  encoder noise; too large and it lags. Measured at the default: the filter's dead-reckoning
  differs from raw odometry by under 0.7 inch over two minutes of stop-start driving, and does not
  accumulate.

- [ ] **HA-87 — 3σ is the right Mahalanobis gate width for the EKF tier.**
  *Claim:* on a 2-degree-of-freedom position innovation, ν > 3 means a fix is an outlier rather
  than noise — about a 1.1% false-reject rate IF the noise model is right, which is the caveat
  that matters.
  *Source:* `ekf_fusion.hpp`, `gateSigma` (PROVISIONAL (A4: HA-87)).
  *Confidence:* **invented**, and note it is a DIFFERENT number from E2's `gateSigma` = 4 (HA-65)
  and E3's = 4 (HA-77) on purpose: those normalise by an assumed constant and must be slacker to
  survive being wrong; this one normalises by an estimated covariance and can afford to be tighter.
  *Settle (R4):* plot the measured innovation distribution once HA-83…HA-86 are real, and choose
  the width from it rather than from a table.
  *Blast radius if wrong (too tight):* honest fixes are refused and the filter re-inits more often
  than it should. (Too slack): the outlier protection stops working, which is the whole reason the
  gate exists.

- [ ] **HA-88 — an absolute heading measurement is 1σ ≈ 2°, flat.**
  *Claim:* a tag-derived heading is about two degrees uncertain, and using one number for every
  such measurement is better than inventing a per-proposal relationship.
  *Source:* `ekf_fusion.hpp`, `headingStdDev` (PROVISIONAL (A4: HA-88)).
  *Confidence:* **invented**, and it is the WEAKEST entry in this group: `CorrectionProposal`
  carries no heading σ, so this one constant stands in for every heading-providing source at every
  range. E3 handled the same gap with a trusted-range BAND (HA-73) instead.
  *Settle (R4):* the range sweep HA-73 already needs — park at a known pose, sweep range, and plot
  recovered heading error separately from position error. That sweep produces σ(range), and a
  corrector that can state it makes this constant unnecessary.
  *Blast radius if wrong (too small):* heading fixes are trusted more than they deserve, and a bad
  tag moves the bias faster. (Too large): the heading correction converges too slowly to matter
  inside a match.

- [ ] **HA-89 — when the EKF knows nothing, it is 24 inches, 30° and 24 in/s uncertain.**
  *Claim:* "somewhere within a tile, facing roughly the right way" is the right ignorance to start
  from, and to return to after a discontinuity or a re-init.
  *Source:* `ekf_fusion.hpp`, `initialPosStdDev` / `initialHeadingStdDev` / `initialVelStdDev`
  (PROVISIONAL (A4: HA-89)).
  *Confidence:* **invented**; the 24 inches is one VEX tile, which is a convenient scale and not a
  measurement.
  *Settle (R4):* it is a prior, so it is not measurable in the way the others are — what R4 can
  settle is whether it is large enough that a first fix is always accepted, and small enough that
  the first few ticks after a re-init are not dominated by it.
  *Blast radius if wrong (too small):* the first fix after a re-init is rejected and the estimator
  cannot recover at all. (Too large): the tick after a re-init accepts almost anything — bounded
  in practice by the per-tick nudge budget, which is why this errs large.

- [ ] **HA-90 — 50 consecutive gate rejections with a mean innovation above 6 inches means the
  estimator is lost.**
  *Claim:* at the ~20 Hz cadence a real corrector achieves, that is about 2.5 seconds of a sensor
  insisting the estimate is wrong — long enough that noise cannot produce it and short enough to
  recover inside a match.
  *Source:* `ekf_fusion.hpp`, `reinitRejectCount` / `reinitInnovation` (PROVISIONAL (A4: HA-90)).
  *Confidence:* **invented**.
  *Settle (R4):* count how often a healthy run produces consecutive rejections at all, and set the
  bar above the observed maximum.
  *Blast radius if wrong (too low):* the filter throws away good confidence on ordinary noise —
  measured as zero occurrences across 8 seeds of a 60-second hostile run at this setting.
  (Too high): a shoved robot takes longer to recover, bounded below by the 12 in/s nudge rate
  anyway.

- [ ] **HA-91 — 5 seconds is long enough between re-init declarations to tell recovery from a
  storm.**
  *Source:* `ekf_fusion.hpp`, `reinitCooldown` (PROVISIONAL (A4: HA-91)).
  *Confidence:* **invented**. It is bounded below by the time a rate-limited recovery actually
  takes: 30 inches at 12 in/s is 2.5 seconds, so a cooldown shorter than that could re-declare
  while the previous recovery is still in progress.
  *Settle (R4):* measure the real recovery time once the nudge rate is settled, and set this to a
  comfortable multiple of it.
  *Blast radius if wrong (too short):* a re-init storm, which in the record is indistinguishable
  from a filter that is working. (Too long): a second genuine displacement inside the window is
  not recovered from until the window expires.

- [ ] **HA-93 — a jammed 11 W mechanism motor reads ≈ 2.5 A at a full 12 V command (scaling
  linearly with commanded voltage), with ≈ 0.05 rad/s residual reported creep.**
  *Claim:* the F1 hostile jam model's device signature is the right order of magnitude for a real
  V5 motor stalled by a wedged ring or a hard stop — stall current ∝ V/R, a spec-ballpark 2.5 A
  at full command, and a not-exactly-zero encoder reading from belt chatter and backlash (included
  deliberately so a detector keyed on EXACT zero is caught by the fake).
  *Source:* `include/shulib/sim/hostile/mechanism_hostility.hpp` (`JammedMotorConfig` defaults,
  PROVISIONAL (A4: HA-93)).
  *Confidence:* **invented** — V5 11 W stall current is commonly cited near this figure; no
  measurement of ours, and 5.5 W motors differ.
  *Settle (R4):* jam a real intake against a block at several commanded voltages; log `current()`
  and `velocity()` through the F4 seam; set per-mechanism `StallConfig` thresholds from the
  measured separation between spin-up and jam (the library ships NO default thresholds for
  exactly this reason).
  *Blast radius if wrong:* hostile-suite realism only — the operation layer's thresholds are
  required per-mechanism parameters, so no library behavior rests on these numbers; a wrong
  magnitude here mis-calibrates the *worst day* the suite rehearses, not the robot.

---

## Group R5 — gains and actuation constants

- [ ] **HA-92 — `BrakeMode::Hold` at 0 V holds a LOADED cascade lift against back-drive.**
  *Claim:* the V5 firmware's active position hold, commanded through the F1 mechanism seam's
  declared safe state, keeps a lift carrying game pieces at its height when its operation exits
  or is cancelled — i.e. "declare Hold and the stack does not come down" is physically true, not
  just commanded. The whole T4 design (per-mechanism declared safe states) makes the *declaration*
  reach the motor provably; whether the *motor* then wins against gravity plus a loaded cascade's
  back-drive torque is this claim.
  *Source:* `include/shulib/hal/mechanism.hpp` (banner + `MotorMechanism::applySafeState`,
  PROVISIONAL (A4: HA-92)).
  *Confidence:* **invented** — no lift exists; V5 Hold is documented as an active position hold,
  its holding torque under a real cascade's load and friction is not.
  *Settle (R3/R5):* build-team lift on a stand: raise under load, cancel the operation, watch the
  stack. Measure sag over 30 s at several loads; if Hold alone is insufficient, F3's `liftToLevel`
  needs its sag-comp PID hold (already in its spec) and the declared safe state stays Hold as the
  best-available floor.
  *Blast radius if wrong:* a cancelled/parked lift descends at the buzzer — scored stack lost and
  a possible entanglement; F2's park guard inherits the same exposure. The failure is visible the
  first time hardware exists, which is why this is registered rather than assumed silently.

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
  constants; C-phase gains are provisional until R5 *by explicit rule* (the Phase C rule: the
  sim proves control logic; real gain values are established on hardware). No conclusion currently drawn from the suite depends on these values being right.

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
  *Blast radius if wrong:* too eager ⇒ spurious ODO_STUCK during aggressive maneuvers — and
  since C2 that is no longer only a log smell: the scheduler's default fault policy ABORTS the
  active motion on a new ODO_STUCK (brakes, records the cause, run continues), so a false
  positive costs one motion, loudly and safely, never a crash; too lax ⇒ a dead encoder is
  caught by the watchdog instead (slower, still bounded). The E-phase estimator-side detector
  supersedes this check's load-bearing role.

- [ ] **HA-53 — the C2 cancel safe state: 0 V + `BrakeMode::Brake` brings the drive from full
  speed to rest promptly (visibly faster than coast), with no adverse firmware interaction from
  re-commanding it repeatedly.**
  *Claim:* commanding zero volts under Brake mode on all drive motors is the correct and
  sufficient "safe state" for every cancel path (user cancel, pre-emption, fault abort, panic
  stop): the real V5 drivetrain decelerates at least as fast as the passive/back-EMF coast the
  host plant models, and short-circuit braking does not misbehave when the very next motion
  immediately commands voltage again.
  *Source:* `include/shulib/motion/motion.hpp` `applyCancelSafeState()` (PROVISIONAL (A4:
  HA-53)); consumed by every `IMotion::cancel()` and by `MotionScheduler`'s pre-empt / fault
  abort / panic paths. Host evidence is deliberately conservative: the A2 plant does not model
  brake modes, so tests prove the 0 V dynamics reach rest (9.7 in coast from 57 in/s on the
  lagged plant) and pin the Brake command by state inspection only.
  *Confidence:* reasoned — Brake mode's short-circuit behaviour is VEX-documented; its stopping
  distance from competition speeds on our robot's mass/wheels is unmeasured.
  *Settle (R3 bench, R5 at speed):* command a cancel from cruise on hardware; measure
  stop distance vs the coast prediction; verify motion-after-cancel resumes cleanly.
  *Blast radius if wrong:* the robot rolls further than expected after any cancel/abort —
  bounded by the coast physics either way (never re-energized: the 0 V command is proven by
  test), so the failure mode is "stops like coast", not "keeps driving". If Brake proves
  harmful (e.g. brownout interaction), the safe state is ONE function in ONE place.

- [ ] **HA-54 — the H-drive's strafe traction derate ≈ 0.35: a single, lightly-loaded strafe
  omni sustains ≈ 35% of its free surface speed pushing the whole robot laterally on foam.**
  *Claim:* `strafeAuthority = strafeSpeedRatio × strafeTractionDerate` with derate 0.35 is
  achievable AND not grossly conservative on the built 15″ H-bot — the robot really can crab at
  ≈ 0.35 × the linear speed budget sustained, and cannot sustain much more. The RATIO half is
  derivable geometry (gearing × wheel radius, 1.0 for same hardware — not registered); the
  DERATE is the part no geometry can supply: normal force on one omni, foam scrub, single-motor
  torque. C3's motion suites prove the CONTRACT at 0.35 (clamp, fallback, visibility, routine
  accuracy); nothing on the host can prove the NUMBER.
  *Source:* `include/shulib/kinematics/h_drive.hpp` `HDriveConfig::strafeTractionDerate`
  (PROVISIONAL (A4: HA-54)); the master plan §13 #5's locked "HDrive ≈ 0.35 sysid-measured
  default, not a hardcoded constant" is this entry's ancestor.
  *Confidence:* **invented** — the 0.35 is the master plan's placeholder; no strafe wheel
  exists to load.
  *Settle (R5):* sysid the built H-bot's sustained lateral speed at full strafe command on
  field foam (loaded, battery-nominal); set derate = measured/ceiling; re-run the C3 suites.
  *Blast radius if wrong:* too HIGH ⇒ the motion layer commands lateral speed the wheel cannot
  deliver — the robot crabs slower than commanded, closed-loop still converges (slower, may
  TimedOut on tight budgets); the fallback flag under-fires. Too LOW ⇒ lateral legs are
  needlessly slow. Either way ONE config field on ONE preset; no frozen contract carries the
  number (F5 carries the QUERY, not the value — by design).

- [ ] **HA-55 — the 15″ H-bot stand-in geometry: 11″ track width, strafe wheel 4″ aft of
  centre, strafe wheel same cartridge/diameter as the drive (ratio 1.0), 3 kinematic wheels
  (left gang / right gang / strafe).**
  *Claim:* the built H-bot's geometry is close enough to these stand-ins that the C3 host
  results (routine accuracy, fallback behaviour, conditioning margin relDet ≈ 0.94) carry over
  qualitatively. The values are INVENTED — chosen to be plausible for a 15″ chassis and
  deliberately OFF-CENTRE so the non-orthogonal pseudo-inverse path is what the suites
  exercise.
  *Source:* `test/motion_test_rig.hpp` `hBotKinematics()` (PROVISIONAL (A4: HA-55));
  co-depends on HA-14 (drive wheel/gearing) and HA-17 (built-vs-preset geometry, which now
  covers the H-bot too).
  *Confidence:* **invented** — no 15″ robot exists, even on paper.
  *Settle (R3):* measure the built chassis (track width, strafe wheel position/diameter/
  gearing); replace the stand-ins; re-run the C3 suites — any newly-failing bound was resting
  on the guess.
  *Blast radius if wrong:* geometry constants only — `hDrive()` makes the drivetrain DATA, so a
  corrected measurement is a config edit, not a redesign. The one structural sensitivity is the
  strafe wheel drifting NEAR centre (relDet → 1, orthogonal fast path takes over — benign) or
  absurdly far aft (relDet falls; the conditioning guard would reject long before accuracy
  degrades silently).

- [ ] **HA-56 — the D-5 plausibility envelope defaults: maxSpeed 150 in/s, maxYawRate
  20 rad/s, margin ×1.5 (per-tick pose-delta invariant).**
  *Claim:* no build of our robots can move its ESTIMATE faster than these bounds by physics —
  a 600 rpm 4″ drive tops out near 125 in/s and ~3 rev/s is past any real chassis — so a delta
  beyond envelope × margin × dt means the estimate is lying, never that the robot is fast.
  The ×1.5 margin absorbs legitimate non-physical estimate motion (never-snap-clamped fusion
  nudges, discretization) and is a logic constant, not part of this claim.
  *Source:* `include/shulib/diag/plausibility_guard.hpp` `PlausibilityConfig` (PROVISIONAL
  (A4: HA-56)); consumed by the scheduler's per-tick check (`MotionSchedulerConfig.plausibility`).
  *Confidence:* reasoned — derived from motor/wheel physics with deliberate headroom; the
  actual drivetrains' top speeds are unmeasured.
  *Settle (R3/R5):* measure top linear speed and yaw rate during sysid; tighten the envelope
  toward measured × margin so the invariant gains sensitivity (today it only catches gross
  lies; a tight envelope catches subtle ones).
  *Blast radius if wrong (too low):* false IMPLAUSIBLE faults — advisory only (never aborts,
  never rewrites the pose), and episode-gated, so the damage is log noise; the C5 suites prove
  zero false positives across every clean AND hostile run at the defaults. *(Too high):* subtle
  estimate lies pass — exactly today's honest state, which R3/R5 tightening fixes.

- [ ] **HA-57 — the V5 controller text grid is 3 rows × 19 columns.**
  *Claim:* `pros::Controller::set_text` addresses 3 text lines and ~19 visible columns; a row
  written longer than 19 columns must be truncated by our adapter (never wrapped into the next
  row).
  *Source:* `include/shulib/hal/line_display.hpp` `ILineDisplay::kRows/kCols` (PROVISIONAL
  (A4: HA-57)); consumed by `diag::ControllerFaultDisplay`, whose row-1 content ("flt " + the
  longest FaultCode spelling) fits 19 columns exactly, pinned by test.
  *Confidence:* reasoned — matches PROS documentation of the controller LCD; the visible
  column count on real hardware/firmware is unverified.
  *Settle (R1):* display a 25-char test row through the real adapter; count what shows; set
  kCols to the measured value (and re-check the row-1 exact-fit pin — if kCols measures
  smaller, the longest fault names truncate, which the seam contract already handles).
  *Blast radius if wrong:* display cosmetics only — content truncates at the seam by contract,
  so a smaller real grid clips characters, never corrupts rows or logic.

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
  hiding (R6's definition of done, verbatim).
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

## Legacy-measured reference points (salvaged at C6, before the C7 deletion)

**Scope, stated carefully:** these numbers were measured or configured on the **previous robot**
("pookster", a 10-motor tank from the pre-v2 era) and are recorded here as **evidence of realistic
magnitudes and as R-phase measurement provenance — NOT as values for any HA entry.** The Override
robots are different machines; every HA stand-in stays exactly as it is until R3–R5 measure *those*
robots. What this table changes is confidence: where a stand-in and the legacy measurement agree in
order of magnitude, the stand-in is corroborated; where they diverge, R-phase should look first.

| Legacy fact | Value (source) | Bears on | Note |
|---|---|---|---|
| Tracking-wheel offsets, as built | L −6.5″ / R +6.5″ / B +2.5″ (`main.cpp:34-36`, corroborated by the stuck-detection log text in `odometry.cpp`) | HA-12 | HA-12's −3.0″/−4.5″ stand-ins are for a *different* (two-wheel + IMU) layout; the legacy numbers show real offsets run ~2–7″ — same order, no change needed |
| Tracking-wheel diameter, as built | 2.75″ (`main.cpp:34-36`) | HA-13 | The harness's nominal 2.0″ is a stand-in; the last real robot used 2.75″ wheels. R3 measures whichever is actually built |
| Rotation sensor ticks/rev | 36000 (centidegree) — `odomUnit.cpp:17` computes travel as `position·πd/36000` | HA-16 | **This is the "measured elsewhere" provenance:** the conversion ran on a real robot all season. HA-16 confidence is effectively settled-by-prior-use; R3 still confirms on current sensors |
| Minimum drive command to move | 20/127 linear, 25/127 rotational (`main.cpp:838-839`, measured by `test_min_output()`'s +1-per-500 ms ramp) | HA-45 (kS) | ≈ 1.9 V equivalent linear kS on that robot vs the 1.0 V placeholder — **R5 should expect kS nearer 2 V** on a comparable drivetrain; the placeholder is likely low, not high |
| Drive geometry, as built | trackWidth 15″, wheel 3.25″, 400 rpm cartridge-equivalent (`main.cpp:38`) | HA-14/HA-15/HA-17 | Corroborates 3.25″ as this team's habitual wheel size (HA-14's guess); rpm differs from HA-15's GREEN/900-ticks stand-in — cartridge is an R3 read-off either way |
| Port map, last season | Drive L {11,−12,13,−14,−15} / R {16,−17,18,−19,20}; Rotation L(−8)/R(10)/B(9); IMU 6; ADI arm 'B', lever 'C'; intake {2,−3}, conveyor {4,−5}, releaser {−6,7} (`main.cpp:23-62`) | R1/R3 runbook | Historical only — the C5 session-header port map and G1's RobotBuilder replace hand-kept port lists. Recorded so the *shape* of a real port map (5-motor sides, reversed-port minus convention) informs R1's adapter tests |
| Calibration-routine specimens | `test_min_output()` (kS ramp), `rotation_calibration()` (iterative IMU-vs-wheel θ-scale comparison, ×3 passes) (`main.cpp:101-217`) | M3 calibration routines, R5 | The *procedures* are the salvage: both are legitimate bench routines to re-derive cleanly at M3/R5. The θ-scale factor itself is obsolete (v2 heading is IMU-owned) |

Diagnostic-culture evidence from the same files (stuck-wheel detection, L/R travel-imbalance and
motor veer/temp diagnosis) is recorded in [`diagnostics-plan.md`](diagnostics-plan.md)'s C6 note
rather than here — it is backlog input, not a hardware assumption.

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
| ~~AprilTag camera intrinsics/mount~~ | **No longer excluded — E3 authored the PnP path and registered HA-68…HA-82.** Camera INTRINSICS themselves remain out: `CameraIntrinsics` has no defaults to be wrong about, and R2 obtains them by calibration. The MOUNT is now HA-70. |

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

Both directions were run clean at A4 close (2026-08-06; the captured output is preserved in
the development log on the `shulib-v2` branch). Entries with no in-tree source (HA-18, HA-33,
HA-47 — sourced from the roadmap, planning-time seeds, and A3's hostile-fake findings
respectively) are exempt from direction 2 and say so in their Source field.

---

## Maintenance

- **Adding an assumption** (any later chunk that invents a magnitude): label it
  `PROVISIONAL (A4: HA-nn)` in-header and add the entry here — the A3→A4 pipeline is now the
  standing convention (C1 followed it: HA-50–52; E1 followed it: HA-58–60). Phase E's remaining
  chunks (EKF noise priors) and F1 (mechanism fakes) are the known next contributors.
- **Settling an entry** (R3/R4/R5/R6): check the box, record the measured value beside the
  guess, update the in-header constant, cite the measurement log. If the measured value breaks
  a test, the test was resting on the guess — fix forward per the R6 rule (a new failure is a
  real defect that had been hiding).
- **This register is the R3 runbook.** R3 — hardware day one — is committed to walking it
  top to bottom; keep the R3 group in bench-practical order.

---

*Created by chunk A4, 2026-08-06 — the chunk that closed Phase A. Companion document:
[roadmap.md](roadmap.md) (status; Phase R owns settling every entry here). The 25-claim seed
inventory this register subsumes and extends to 49 — and the full A4 working record — live in
the development log on the `shulib-v2` branch.*
