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
> **Status: 7 of 158 settled** (HA-94/95/96/97/99/100/101, all measured on the old competition bot
> 2026-08-13 — one robot, once; not proof of portability). HA-98 partially settled. *(The total read
> "122" while HA-123 existed — a one-entry drift, corrected 2026-09-14 when robot two's section,
> HA-124–HA-133, was added: of those ten, four are MEASURED on robot two (HA-127/128/129/130),
> one is reported (HA-124), two are UNSET and refuse at boot (HA-125/126), one reasoned (HA-131),
> two invented (HA-132/133). R3d Part 1 added FOUR more on 2026-09-17 — HA-134/135 reasoned,
> HA-136/137 invented — and not one of the four is measured on anything: each is a belief the host
> plant cannot settle, which is exactly why it is an entry. **R3d Part 2 added THREE more on
> 2026-09-18 — HA-138/139/140, all reasoned** — and the same is true of them: the registry's
> zero-indexing and fopen's `"rb"` are beliefs about PROS that only a brain can settle, and the CRC
> note is a design decision recorded so the next person does not re-litigate it. **G1 Part 2a added
> ONE more on 2026-09-24 — HA-141, reasoned:** that the registry's KIND answer is right for an IMU, a
> rotation sensor and a GPS, which the on-robot device provider now refuses ports on. The
> per-confidence counts below predate every entry from HA-124 on. **R3d Part 2F-1 (2026-09-24) added no entry and
> REWROTE three:** HA-134 now describes a RAMPED pulse and what happens when no level turns the
> robot, HA-136 lists the calibrator's new and removed numbers and the screen model's estimates, and
> HA-137 carries the plant's two new rules — coast-follow and the slipping member — because the first
> night on robot two showed the plant could not fail the way the robot did. **R3d Part 2F-2** (the
> same day) added no entry; HA-136 records how the screen's character budgets were checked against
> earlier screens instead of measured. **DEFECTS2 added FIVE on 2026-09-26 — HA-142..HA-146** — the
> beliefs about how a brake mode reaches a V5 motor, found when the library's Brake and Hold stops
> turned out never to have reached one (HA-142 and HA-146 reasoned from the vendored text, HA-144
> invented, HA-143 and HA-145 UNKNOWN and labelled invented), and AMENDED three: HA-53 (the drive's
> "0 V + Brake" coasted on a V5 until then), HA-92 (its premise that the declaration reaches the
> motor was false at the wire) and HA-98 (brake mode is device state too). **T3 (2026-09-26) added
> FOUR — HA-153 to HA-156**, robot two's claw pivots, which no capture has seen yet: HA-153
> reported, HA-154 and HA-155 reasoned (HA-154's timing invented), HA-156 invented. **T3's addendum
> (E2, the same day) added ONE — HA-159**, robot two's pneumatic claw on 3-wire port A: the port
> reported, its polarity and whether its valve is single or double unknown. **T3's addendum B2 (the library half, the same day) added TWO — HA-157, UNKNOWN, and HA-158,
> reasoned** — about the brain's 3-wire ports that the pneumatic claw depends on — **and the
> merge added ONE — HA-147, invented:** the controller belief the claw's toggle rests on (B2's
> review). HA-148 to HA-152 were reserved for T3 and are UNUSED: the numbers are not reissued, so
> the register has 154 entries up to HA-159 (156 with DIAG1's HA-160 and COMP-a's HA-161, below). **DIAG1 added ONE on 2026-09-26 — HA-160** — that the V5 IMU's heading is continuous tick to
> tick while ready, an assumption the library always made and never wrote down, now QUESTIONED by
> the 2026-09-25 robot evidence — and AMENDED HA-123: extended to drive-encoder sides, and its
> blast radius corrected (a spurious trip ends the active motion). **COMP-a added ONE on 2026-09-26 — HA-161, reasoned** (PROS deletes its competition tasks on
> a mode change and not the program's own — the belief the competition program's own auton task
> rests on), and AMENDED HA-05 (the start heading's owner is the chosen auton's start pose). **T3 Part 2 (the Teleop wiring, the same day) added no
> entry** — HA-148..HA-152 stay unused — and brought HA-153, HA-154, HA-156 and HA-159 up to the
> tree: "shulib Teleop" now builds and drives the claws (not on a robot yet), so their lines saying
> no program drives them, or that the claw row does not exist, were corrected. **TANK1 (2026-09-27)
> added TWO — HA-179, invented, and HA-180, reasoned** — the constants `moveTo`'s new tank steering
> runs on (a bend toward the target's line and a cancelled weak-side yaw, tuned on the host test
> plant's placeholder dynamics), and the belief it rests on that robot two's sideways drift is its
> HEADING being pulled, not the chassis sliding sideways; the register then holds 158 entries.
> HA-162..HA-178 are not in it: numbers other chunks hold (briefed, not landed) — never reissued. The
> per-confidence counts below predate every entry from HA-124 on (2026-09-26): they still sum to
> 122, have no bucket for UNKNOWN (HA-143, HA-145 and HA-157's confidence), and a recount is due.)* **No *v2* robot exists**, and
> the platform layer has now been validated on the team's old competition bot — real adapters
> commanded real motors and read real sensors on 2026-08-13 — but **no control loop has ever
> closed and nothing has driven.** Counts: **78 invented · 41 reasoned · 2 measured elsewhere · 1 mixed** (HA-44:
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
> beliefs), and **HA-113–122 by chunk R1b** (the same class of belief for the mechanism-sensor
> adapters: distance, optical, ADI digital lines, and the SD card — including two flagged-weak
> halves the vendored source does not state: proximity's polarity, HA-117, and fopen's `/usd/`
> prefix, HA-122), per the Maintenance convention.
> *(This status line was found stale at R1a — it read "0 of 82" while the register held 93
> entries: E4's and F1's additions never updated it. Corrected here; the per-chunk narrative
> above is the part a tool cannot regenerate, so it is the part that must be tended.)*

---

## Index

| ID | Claim (short) | Confidence | Owner |
|---|---|---|---|
| HA-01 | GPS position axes: +X = East, +Y = North | **invented** | R3 |
| HA-02 | IMU as-mounted heading is CW-positive | **CONFIRMED** 2026-08-19 on the bench bot: operator turned the robot LEFT/CCW and RAW `get_rotation()` went NEGATIVE (-0.073 -> -87.241 deg) while CANONICAL rose (+0.073 -> +87.241 deg). Raw decreasing on a CCW turn IS raw being CW-positive, exactly as claimed; and the adapter's negation therefore yields the CCW-positive canonical F1 requires. Evidence: `evidence/r3a-session-2026-08-19.log` run 2 | R3 |
| HA-03 | `get_rotation()` is cumulative/unbounded (the required binding) | reasoned | R1/R3 |
| HA-04 | IMU yaw-rate source sign (`get_gyro_rate().z` undocumented) | **invented** | R3 |
| HA-05 | Post-cal tare invalidates bootHeading; offset applied once | reasoned — **AMENDED** 2026-09-26 (COMP-a): the start heading's one owner is the chosen auton's start pose, applied at autonomous start through the Localizer's bias (`reseatHeading`) — bootHeading stays 0 and the IMU is never re-zeroed | R1/R3 |
| HA-06 | Firmware GPS offset unset ⇒ `get_position()` = SENSOR pose | reasoned | R1/R3 |
| HA-07 | GPS `get_error()` returns meters | reasoned | R3 |
| HA-08 | Failed reads return PROS_ERR/PROS_ERR_F; adapters screen to stale-finite | reasoned | R1/R3 |
| HA-09 | `northHeadingDeg = 90°` matches our field setup | reasoned | R3 |
| HA-10 | GPS lever arm value (body inches) as configured | **invented** | R3 |
| HA-11 | Rotation `get_position()` cumulative; int32 centidegree never wraps in a match | reasoned | R1/R3 |
| HA-12 | Tracking-wheel offsets/signs as configured (−3.0″/−4.5″ are stand-ins) | **invented** | R3 |
| HA-13 | Tracking-wheel effective diameter = nominal 2.0″ | **invented** | R3 |
| HA-14 | Drive wheel 3.25″, wheel↔shaft gearing 1:1 | **CORRECTED** 2026-08-19: wheels measured **2.75″**, not 3.25″ — build team, read off the robot. The 3.25″ stand-in is WRONG and every distance derived from it is short by 15.4%. Gearing half still open (B1.2, per side) | R3 |
| HA-15 | Drive cartridge GREEN → 900 ticks/rev at output | **CORRECTED** 2026-08-19: cartridges are **BLUE** (6:1, 600 rpm), read visually off the motors — the only way this is knowable, since a V5 motor cannot sense its own cartridge. GREEN was invented and is WRONG. **The brain is currently CONFIGURED green while the hardware is blue** (`motor_get_gearing()` returned GRN 200 on all eight), so position and velocity are mis-scaled by the 18:1-vs-6:1 ratio until something sets it correctly | R3 |
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
| HA-53 | Cancel safe state: 0 V + BrakeMode::Brake stops the drive promptly from speed | reasoned — **AMENDED** 2026-09-26 (DEFECTS2): on a V5 this stop COASTED until DEFECTS2 (the adapter sent 0 V, which ignores the mode — HA-142); it brakes only since | R3/R5 |
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
| HA-92 | `BrakeMode::Hold` at 0 V holds a LOADED cascade lift against back-drive | **invented** — **AMENDED** 2026-09-26 (DEFECTS2): the premise that the declaration reaches the motor was FALSE at the wire until DEFECTS2 (HA-142) | R3/R5 |
| HA-93 | A jammed 11 W mechanism motor reads ≈ 2.5 A at a full 12 V command, with ≈ 0.05 rad/s residual creep | **invented** | R4 |
| HA-94 | Motor `move_voltage()` takes millivolts, ±12000 | **settled** measured-on-comp-bot 2026-08-13: 2.0 V commanded → device reported 1823–1990 mV under load | R3 |
| HA-95 | Motor `get_position()` with `MotorUnits::degrees` reports output-shaft degrees | **settled** measured 2026-08-13: deg/rad = 57.296 on all 8 motors (57.2958 exact) | R3 |
| HA-96 | Motor `get_actual_velocity()` returns output-shaft RPM | **settled** measured 2026-08-13: rpm/(rad/s) = 9.549 (9.5493 exact) | R3 |
| HA-97 | Motor `get_current_draw()` returns mA | **settled** measured 2026-08-13: mA/A = 1000.0 exactly, 8 motors | R3 |
| HA-98 | Motor units/gearing — and the BRAKE MODE (added to this row by DEFECTS2) — live in the DEVICE and persist across programs; explicit set + read-back holds | **partial** 2026-08-13: set+read-back accepted on 8 motors (no precondition fired); PERSISTENCE across programs NOT tested, so this stays open | R3 |
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
| HA-111 | The invented X-drive's port map and motor direction signs match the robot (`src/robots/xdrive_config.hpp` since G1 Part 2c; `src/main.cpp` before) | **invented — and MEASURED WRONG for the comp bot** 2026-08-13: real map is drive LEFT 15/16/17/18, RIGHT 11/12/14, IMU 4, no rotation sensors, no GPS. The xdrive build still carries the invented map, now as a config constant that refuses at boot on any robot that exists | R3 |
| HA-112 | Teleop stick mapping signs + the feel layer's defaults are drivable (0.05 deadband, applied as a continuous rescale since T2; expo 0; slew off; the field-centric fallback latch) | **invented** | T2/R3 |
| HA-113 | Distance `get_distance()` returns millimeters | reasoned | R3 |
| HA-114 | Distance "no object" is the IN-BAND integer 9999, not PROS_ERR | reasoned | R3 |
| HA-115 | Distance `get_confidence()` is 0–63, only available above 200 mm; its value at/below 200 mm is UNKNOWN | reasoned | R3 |
| HA-116 | Optical hue is 0–359.999°, saturation/brightness are 0–1.0 | reasoned | R3 |
| HA-117 | Optical `get_proximity()` 0–255, LARGER = CLOSER — polarity NOT in the vendored doc | **invented** | R3 |
| HA-118 | Optical sentinels: PROS_ERR_F on the double channels, PROS_ERR on proximity | reasoned | R3 |
| HA-119 | ADI `DigitalOut` DRIVES THE LINE AT CONSTRUCTION (init_state, PROS default LOW); `set_value` is 1/0, PROS_ERR on refusal | reasoned | R3 |
| HA-120 | ADI addressing: 1–8 ≡ 'a'–'h' ≡ 'A'–'H'; expander via {smart, adi} pairs; whether OUR robot has an expander is UNKNOWN | reasoned | R3 |
| HA-121 | ADI `DigitalIn::get_value()` is a level (PROS_ERR on refusal); `get_new_press()` CONSUMES the press | reasoned | R3 |
| HA-122 | SD: `usd_is_installed()` returns 1/0; fopen NEEDS the /usd/ prefix (list_files FORBIDS it); fflush is the strongest persist | **partial** 2026-08-18: beliefs (1) and (2) CONFIRMED on hardware — `usd_is_installed()` returned 1 with a card and 0 without, and `fopen("/usd/probe.txt","wb")` succeeded WITH the prefix; a 17-byte fwrite + `fflush`==0 landed. Belief (3)'s DURABILITY half (yank the card after a flush, count what survived) NOT tested, so this stays open | R3 |
| HA-123 | A per-tick travel above 36 in — from one tracking pod, or one DRIVE SIDE of a drive-encoder odometry — is corruption, not motion | **invented** — **AMENDED** 2026-09-26 (DIAG1): extended to drive-encoder sides (the bound has applied to them since 2026-09-14; this entry had not said so); blast radius corrected — a spurious trip ENDS the active motion | R3 |
| HA-124 | Robot two's drive wheels are 2.75 in in diameter | **reported** (team lead, 2026-09-14; "standard wheels") — not yet rulered on this robot | R3b/R3d |
| HA-125 | Robot two's track width (contact line to contact line) | **UNSET** — no value exists in the tree; the library program refuses to boot until a tape measurement is typed in | R3b/R3d |
| HA-126 | Robot two's motor→wheel external ratio | **UNSET** — "600 rpm" was reported, which is a cartridge, not a tooth count; the library program refuses until direct drive or the counts are stated | R3b/R3d |
| HA-127 | Robot two's brain runs VEXos 1.1.5 (b18) under PROS kernel 4.2.2 | **measured** 2026-09-10 (the kernel banner over USB) | — |
| HA-128 | Starting a program from the controller's own menu arms a pretend field-control MATCH on the brain (status 0x0f) that persists until the controller leaves that mode; a linked controller on its home screen does not | **measured** 2026-09-10 (two sessions, the decisive test with hands off the controller) | — |
| HA-129 | The brain's USB CDC ports renumber between plug-ins (ttyACM0/1 alone, ttyACM1/2 beside the controller) and must be re-resolved from `pros lsusb` for every read | **measured** 2026-09-10 | — |
| HA-130 | Robot two's port 18 travels ~20 % short of its side-mates on a hand push, reproducibly (137 vs ~175 deg, then 171 vs ~210 deg) — a member SLOW, not a fight | **measured** 2026-09-10 (twice); cause UNKNOWN — watch it under power | R3b/R3d |
| HA-131 | On robot two, ODO_STUCK cannot be detected by the wheels-vs-odometry cross-check: the odometry IS the drive encoders, so the check is a tautology and is deliberately not wired to report a verdict | reasoned (structural) | R3d/R4 |
| HA-132 | The coupled-side agreement thresholds — evaluate above 1 V and 1 rad/s; opposite sign above 0.5 rad/s; under 25 % of the side's fastest; 25 consecutive 10 ms ticks — separate a real fight from a transient | **invented** | R3d/R4 |
| HA-133 | A coupled member under 25 % of its side's fastest is disagreeing, and one at 80 % (port 18's signature) is NOT — the disagreement FLOOR tolerates a slow member and catches a frozen one | **invented** | R3d/R4 |
| HA-134 | On a tank, driving ONE side forward alone turns the robot toward the other: LEFT forward → CLOCKWISE → NEGATIVE canonical yaw; RIGHT forward → positive. The calibrator reads a drive port's SIDE off that sign — from a pulse that RAMPS 3 → 6 → 9 V until the turn clears the floor, classifying a port no level turns (since R3d Part 2F-1) | reasoned (rigid-body) | R3d Part 3 |
| HA-135 | The EFFECTIVE track width a powered spin measures is LARGER than the tape's contact-line-to-contact-line width, because a turning tank scrubs; the effective one is what the kinematics and the heading cross-check need | reasoned | R3d Part 3 |
| HA-136 | Every number in `calibration::CalibratorConfig` — the push threshold, the rest floors (motor velocity and, since 2F-1, IMU yaw rate)/settle/timeout, the pulse levels/time/yaw floor, the decoupled minimum, the push distance (a CHOICE), the straightness bound, the minimum push, the spin levels/start check/wrong-way bound/turns/timeout/agreement, the IMU-ready and missed-tick timeouts — plus the two reproducibility tolerances the host suite pins, and the calibration screen model's text budgets, line heights, hit padding and arming delay | **invented** | R3d Part 3 (the screen's: Bench Tests' SCREEN RULER) |
| HA-137 | A coupled train whose members are commanded DIFFERENT volts turns at the MEAN of their effective volts (the A2 plant's model; torque-free); since R3d Part 2F-1 also: an all-coasting wheel may FOLLOW the driven one by a fraction κ (default 0 = pinned), and a slipping (`Decoupled`) member's volts never reach its train | **invented** (sim shape) | R6 |
| HA-138 | `registry_get_plugged_type(i)` is ZERO-indexed: registry index i is PHYSICAL PORT i+1, and indices run 0–20 | reasoned (vendored text) — and the ONE conversion lives in `hal/pros/device_registry.hpp` | R3d Part 3 |
| HA-139 | `fopen(path, "rb")` under `/usd/` reads back the exact bytes `"wb"` wrote, byte for byte, on the brain's FatFS | reasoned, flagged — the host CANNOT test the mode (POSIX does not translate) | R3d Part 3 |
| HA-140 | A CRC-32 over a 180-byte record is worth its 4 bytes and its microseconds on a V5, where the blackbox's own format deliberately carries none | reasoned (design note under HA-122) | R3d Part 3 |
| HA-141 | The registry's KIND answer is right for an IMU, a rotation sensor and a GPS (not only for a motor): a port carrying one reads that kind, and an empty one reads NONE | reasoned (vendored enum) — the on-robot provider refuses a port on this answer ALONE for the three kinds whose adapters cannot fail loudly | IMU half: G1 Part 2b's first boot; the provider's kind refusal: the first program that asks it for a GPS or rotation port / R3 |
| HA-142 | `move_voltage()` (and `move()`) IGNORE the brake mode; only `brake()` applies it — so a stop is "mode + `brake()`", never "mode + 0 V" | reasoned (vendored text, stated twice) — the DEFECTS2 fix rests on it | R3 (an ENABLED Brake stop vs a Coast release — a disable is confounded by HA-145) / T3's claw station |
| HA-143 | At `move_voltage(0)` a V5 motor free-wheels — the "coast" `IMotor` promises, which HA-137's coast-follow and Calibrate's pulses rest on | **invented** (UNKNOWN — no PROS text says what 0 V does; robot observations are qualitative) | R3 / R4 |
| HA-144 | A REPEATED `brake()` under Hold re-captures the held position (so one per tick under load lets a shaft creep); the adapter issues one per transition into zero. Second half: whether a same-mode `set_brake_mode()` written to a motor already held does the same (still sent every tick by a caller that re-states it) | **invented** | T3's claw station |
| HA-145 | What a motor stopped under Hold (or Brake) does while VEXos has the robot DISABLED, and after re-enable | **invented** (UNKNOWN) — the adapter's stop record cannot see a disable | T3's claw station / COMP |
| HA-146 | After `brake()`, the next `move_voltage(v ≠ 0)` drives the motor at once (the stop is replaced, not latched); a `set_brake_mode()` repeated every tick changes nothing but the register of a RUNNING motor (a held one: HA-144) | reasoned (vendored text) | R3 / T3's claw station |
| HA-147 | A V5 controller's connected flag and its button levels change together: on the first tick `is_connected()` is true after a disconnect, `get_digital()` already returns the LIVE level (never a stale one from before the drop), and a button's level does not bounce across two 10 ms samples | **invented** (unmeasured; HA-103 claims only that a disconnected controller reads 0) — the pneumatic claw's toggle takes a connected tick's level as live | Station J.9.5 (robot two's claw) |
| HA-153 | Robot two's two claw pivots are separate V5 motors with RED (100 rpm) cartridges on smart ports 6 (R1 forward / R2 back) and 7 (L1 / L2), and a POSITIVE command moves each the way the team lead calls "forward" | **reported** 2026-09-26 (team lead); NO capture has seen a motor on 6 or 7, and the way each turns is not even reported | T3 / worksheet Station J |
| HA-154 | A claw pivot held at 12 V against its mechanical stop draws up to the motor's DEFAULT current limit (2500 mA — nothing in shulib changes it) for as long as the button is held, and heats; shulib watches only the DRIVE motors' temperatures | reasoned (vendored `motors.h`; a stalled motor heats) — every TIMING **invented** (none is stated) | T3 / worksheet Station J |
| HA-155 | A cable swapped between a claw pivot and a drive motor is invisible to the brain's registry (a MOTOR on both ports either way), so no check before power can see it | reasoned (the registry reports a device's KIND, never which motor) | T3 / worksheet Station J (a person, not the registry) |
| HA-156 | A claw pivot released in Hold (0 V, after DEFECTS2 makes a Hold reach the motor) keeps its angle under its load — its own weight and a game piece — with a sag over 30 s too small to matter | **invented** — no load, sag or hold current measured; "holds" is claimed nowhere until Station J logs it | T3 / worksheet Station J |
| HA-157 | While the brain is DISABLED (between autonomous and driver control, on a field or competition-switch disable) a 3-wire digital output KEEPS the level the program last commanded, and still has it after the re-enable; and a level the program commands WHILE disabled (a Robot built in `initialize()` on a field, or a program started from the controller's menu, HA-128) reaches the valve — at once, or at the latest at the enable | **UNKNOWN** — nothing vendored says; the pneumatic claw's driver-control toggle commands NOTHING on a disable and relies on the first half; the claw's start level on a field relies on the second | Station J (robot two's claw) |
| HA-158 | Nothing the program can read sees a pneumatic solenoid on a brain 3-wire port: the registry is indexed by SMART port, and a valve on A..H is a passive load — a connected line and an empty one are indistinguishable | reasoned (vendored registry + ADI shape) — the on-robot provider refuses a 3-wire line only for being outside 1..8 | Station J (robot two's claw) |
| HA-159 | Robot two's pneumatic claw is driven by ONE solenoid line on the brain's 3-wire port A (line 1), and that line at the level `kClawOpenCommand` (`true`, HIGH) OPENS the claw | **reported** 2026-09-26 (team lead: "port A") — single or double solenoid and the polarity **UNKNOWN** (the team lead said so); `true` is a starting guess | T3 addendum (E2) / worksheet Station J.9 |
| HA-160 | The V5 IMU's heading is CONTINUOUS tick to tick while it reports ready: two reads ~10 ms apart differ by no more than the robot turned in between — never by tens of degrees | **invented** (assumed since M2, unregistered until DIAG1) — **QUESTIONED** by 2026-09-25: in one start, during a hard reverse, the reconstructed heading swung up to 37° per 10 ms tick — 8 of the last 9 ticks at 16.9° or more, alternating in sign; cause UNKNOWN | R3 / R4 |
| HA-161 | PROS runs `competition_initialize()` / `disabled()` / `autonomous()` / `opcontrol()` each on a task it DELETES on a mode change (a deleted task runs no destructor) — and does NOT delete a task the program created itself | reasoned (the belief code comments have carried since R3b Part 0b; no vendored PROS text says either half, and neither has been observed directly) — the competition program's own auton task rests on the second half | COMP-b's practice-match station (K) |
| HA-179 | TANK1's tank-steering constants (bend 0.1 rad/in capped at 15°, fade 6 in, lead 0.06 s; yaw observer 0.1 s, limit 1.5 rad/s) remove robot two's weak-side drift on a tank `moveTo` within the unchanged 0.5 in / 0.02 rad rule, without heading oscillation — its drivetrain follows a yaw command much faster than 0.1 s | **invented** (tuned on the host test plant's placeholder dynamics; with unmodelled motor lag the plant's margin shrinks — measured) | COMP-t Station M.6 / R5 |
| HA-180 | Robot two's sideways error on a straight tank `moveTo` comes from its HEADING being pulled (the IMU sees it), not from the chassis sliding sideways (which no sensor on robot two sees) | reasoned (the 2026-09-25 veer was a heading change read on the IMU; a skid was never measured) | COMP-t Station M.6 |

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
  for exactly this reason) and inverts E2's high-yaw-rate rejection. Since TANK1 (2026-09-27) it would
  also turn a tank `moveTo`'s yaw observer into positive feedback, bounded at its 1.5 rad/s limit
  (HA-179's blast radius has the measurement). **Contained:** one negate in
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
  *Amended (COMP-a, 2026-09-26) — who owns the START heading.* The claim above stands (never tare
  after calibration). What changed is the owner of the start heading: it is the CHOSEN AUTON's
  declared start pose — `StartPose::field(x, y, heading)`, or `StartPose::relative()` (the robot's
  own placement is heading 0) — applied at the instant the auton starts by
  `competition::runAuton`, through `Localizer::reseatHeading`: the published heading is
  `imu.heading() + bias`, and the seed sets the bias. The IMU's boot heading stays 0
  (`hal/pros/device_provider.hpp`), the sensor is never re-zeroed, and the IMU remains the only
  source of heading CHANGE. Two reasons it cannot be the boot heading: the auton is chosen on the
  brain screen AFTER the robot boots (the boot cannot know which start pose will run), and the
  seed also discards whatever the IMU drifted while the robot waited for the match. The seed is
  refused while the IMU is still calibrating (the estimate must be live first), and it re-baselines
  the estimator's next tick exactly as `setPose` does. `Routine::startAt` (frozen, F10) still seeds
  position only. *Settle (COMP-b's practice match):* place the robot at a known field heading,
  wait a minute on the selector, run the acceptance auton; it must drive along its PLACED heading,
  not the heading the IMU read at boot.

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
  *Amended 2026-09-14 (R3b Parts 1–3):* that containment no longer holds for ROBOT TWO, whose
  odometry IS the drive encoders (`localization/drive_encoder_odometry.hpp`). There the scale
  comes from `hal::DriveGeometry`, instantiated from the chassis table's measured values
  (HA-124/125/126), never from this stand-in; the plant still bakes 1:1 (the team lead has
  reported no other ratio), so a host test and the V5 model the same gearing only while that
  stays true.

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

- [x] **HA-94 — `move_voltage()` takes MILLIVOLTS in [-12000, 12000]. SETTLED.**
  *Claim:* the argument is mV; 6 V of command is `move_voltage(6000)`.
  *Source:* vendored `include/pros/motors.hpp:234` ("from -12000 to 12000 in millivolts");
  `include/shulib/hal/motor_conversion.hpp` (`motorVoltageToMillivolts`, the one ×1000).
  *Confidence:* **settled** — was: reasoned, documented in the vendored source, unverified.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* 2.0 V commanded through the real `ProsMotor`; the
  device reported 1823–1990 mV under load across eight motors. The shortfall is not an error —
  `commandedVoltage()` reports what was *applied*, and a motor under load on a sagging battery
  applies less.
  *(Settled by R3 runbook step 4.)*
  *Blast radius if wrong:* every command 1000× off — the robot either hums in place or
  saturates instantly. **Contained:** one conversion function; caught in the first minute of
  the bench.

- [x] **HA-95 — `get_position()` with `MotorUnits::degrees` reports OUTPUT-SHAFT degrees. SETTLED.**
  *Claim:* after the adapter sets degrees, position is the output shaft's cumulative degrees
  (gearset already accounted for by firmware).
  *Source:* vendored `motors.hpp:628` ("absolute position in its encoder units");
  `hal/pros/motor.hpp` (the degree→radian binding).
  *Confidence:* **settled** — was: reasoned, with *output-shaft* the weaker half.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* deg/rad = **57.296** on all eight motors (57.2958 exact).
  The output-shaft half is confirmed: the ratio carries no cartridge factor.
  *(Settled by R3 runbook step 4.)*
  *Blast radius if wrong:* odometry scaled by the cartridge ratio (6:1/18:1/36:1) — silently,
  nothing crashes. **Contained:** adapter-only.

- [x] **HA-96 — `get_actual_velocity()` returns output-shaft RPM. SETTLED.**
  *Claim:* the double is RPM at the output shaft (gearset-corrected by firmware).
  *Source:* vendored `motors.hpp:404`; `motor_conversion.hpp` (`motorRpmToCanonical`, ×2π/60).
  *Confidence:* **settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* rpm/(rad/s) = **9.549** (9.5493 exact).
  *(Settled by R3 runbook step 4.)*
  *Blast radius if wrong:* feedforward and the stall cross-check read speeds ~9.5× off.
  **Contained:** one conversion.

- [x] **HA-97 — `get_current_draw()` returns milliamps. SETTLED.**
  *Claim:* the int32 is mA; a stalled 11 W motor reads ~2500.
  *Source:* vendored `motors.hpp:426` ("in mA"); `motor_conversion.hpp`
  (`motorMilliampsToCanonical`).
  *Confidence:* **settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* mA/A = **1000.0** exactly, across eight motors.
  *(Settled by R3 runbook step 5.)*
  *Blast radius if wrong:* stall/capture detection fires always or never — every
  sensor-confirm operation lies. **Contained:** one conversion.

- [ ] **HA-98 — motor units/gearing are DEVICE state, persist across programs, and an explicit
  set + read-back holds. PARTIALLY SETTLED — the persistence half is still open.**
  *Claim:* the ctor-default `invalid` means "leave as is"; a previous program's configuration
  survives; setting degrees+gearset then reading both back proves the device accepted them.
  *Source:* vendored `motors.hpp:74-75`; `hal/pros/motor.hpp` (ctor read-back preconditions);
  the shim's adversarial rotations/red defaults model exactly this hazard.
  *Confidence:* **partially settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* the explicit set + read-back was accepted on all
  eight motors with no precondition firing, so that half holds. **Persistence across programs
  was NOT tested** and stays open — which is the half the adapter's whole ctor discipline
  exists for, so it is the half that matters. Do not read this row as closed.
  *Settle (R3, runbook steps 0/1/4):* boot with a correct map (read-backs pass); boot with one
  wrong port (read-back faults loudly).
  *Blast radius if wrong (adapter skips the discipline):* a motor left in rotations reads 1/360
  — odometry wrong by 360× with nothing crashing. **Contained:** ctor discipline + mutation-
  proven test (campaign M5).
  *Amended DEFECTS2 (2026-09-26) — the BRAKE MODE is device state too:* `get_brake_mode()` reads
  back whatever mode the port was last given, by any program, and the ctor deliberately commands
  none (choosing a mode for a mechanism it knows nothing about is not the adapter's call). Since
  DEFECTS2 that inherited mode would matter at the motor if anything keyed on it, because a zero
  under Brake/Hold is now sent as `brake()` (HA-142). So `ProsMotor` keys the stop on the mode THIS
  PROGRAM commanded through `setBrakeMode()` — never on the device's read-back, which only seeds
  the T7 fallback (DEFECTS1 E2) — and a motor this program never gave a mode stays on
  `move_voltage(0)` at zero (`pros_motor_adapter_test.cpp` DEFECTS2 a7, the seeded-mode trap,
  mutation-proven). The persistence of the brake mode across programs is untested, like the rest
  of this row's persistence half.

- [x] **HA-99 — battery voltage/current are mV/mA. SETTLED — and it was R1a's weakest belief.**
  *Claim:* `battery_get_voltage()` ≈ 12600 on a fresh pack (mV); `battery_get_current()` is mA.
  *Source:* vendored `misc.h:718-750` — which documents **NO UNIT AT ALL** ("the current
  voltage of the battery"); the mV/mA belief is from PROS's website documentation only.
  `hal/pros/battery.hpp` carries the warning in its header.
  *Confidence:* **settled** — was: reasoned and flagged as the only load-bearing unit in R1a
  that the vendored source does not state at all.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* raw **13039 → 13.04 V**, and raw 919 → 0.92 A.
  The website-only belief was right. *(Settled by R3 runbook step 2, before anything drove.)*
  *Blast radius if wrong:* a 1000× error silently destroys brownout compensation, which scales
  EVERY motor command. **Contained:** one adapter, but only if checked before driving — hence
  the runbook ordering.

- [x] **HA-100 — `battery_get_capacity()` returns percent 0–100. SETTLED.**
  *Claim:* the double is a percentage; the adapter's ÷100 lands `IBattery::capacity()` in [0,1].
  *Source:* vendored `misc.h:771-787` (unit again undocumented; "Battery Level" example);
  `hal/pros/battery.hpp`.
  *Confidence:* **settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* raw **91.0 → 0.91**.
  *(Settled by R3 runbook step 2.)*
  *Blast radius if wrong:* capacity() pegged at 1.0 by the clamp (a fraction read as percent)
  — misleading telemetry, no control impact today. **Contained.**

- [x] **HA-101 — `micros()` is microseconds since program start, monotonic, uint64. SETTLED.**
  *Claim:* `IClock::now()` = micros()×1e-6 is a monotonic seconds stream; uint64 µs cannot wrap
  in a robot's lifetime (~584 000 years).
  *Source:* vendored `rtos.h:241`; `hal/pros/clock.hpp` (which also records WHY not millis():
  1 ms quantization is 10% of the 10 ms tick — a 10% error in every PID derivative).
  *Confidence:* **settled** — was: reasoned.
  *Measured (bench, old competition bot, 2026-08-13 — ONE robot, ONCE; not proof of portability):* **999 784 µs** over a nominal 1000 ms delay — 0.02% low.
  Monotonicity and the uint64 width were not separately exercised.
  *(Settled by R3 runbook step 3.)*
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

- [ ] **HA-111 — the invented X-drive's port map and motor direction signs match the robot.**
  *Claim:* FL1 BL2 BR−3 FR−4 ROTF5 ROTL6 GPS9 IMU10, green cartridges — ALL INVENTED.
  *Source:* `src/robots/xdrive_config.hpp` (`kInventedXDrive`, a `constexpr config::RobotConfig`, its
  name carrying this entry's number) since G1 Part 2c, 2026-09-26 — the SAME numbers, pinned
  against a transcription of the old constants by `test/xdrive_config_test.cpp`; until then
  `src/main.cpp` (the `k*Port` block, labelled), which is now a dispatcher and carries no port.
  *Confidence:* **invented — and MEASURED WRONG on 2026-08-13.** The real map on the old
  competition bot is: drive **LEFT 15/16/17/18, RIGHT 11/12/14** (green cartridges), **IMU on
  port 4**, and **no rotation sensors and no GPS at all**. Only "there are motors on low ports"
  survived. The xdrive build still carries the invented map, deliberately — it also assumes an
  X-drive with two tracking wheels, which this robot is not and does not have, so fixing the
  ports alone would not make it correct. Since G1 Part 2c it is built by `robot::Robot` over the
  on-robot device provider, which asks the brain's registry before opening a port: on this robot,
  and on robot two, it refuses at boot naming port 1 and powers nothing (host-proven through the
  PROS shim only — `test/xdrive_config_test.cpp`; no X-drive build has been uploaded since). The wiring belongs to whichever chunk runs the shipped
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

- [ ] **HA-112 — the teleop mapping AND the feel layer's defaults are drivable.**
  *Claim:* (a) the axis signs — left-stick-up = +X forward, left-stick-left = +Y left,
  right-stick-right = CW (−ω); (b) a **0.05 deadband** kills centred-stick creep without
  killing fine control; and, since T2 built the feel layer, (c) **the shipped
  `DriverProfile` is drivable as it stands** — deadband 0.05 applied as a CONTINUOUS
  rescale rather than a hard cut, **expo 0** (no response curve at all), **slew OFF**
  (no rate limit, so no added lag), mode **ArcadeBody**, and a field-centric fallback that
  **latches** until the driver re-zeroes rather than flickering between frames.
  *Source:* `include/shulib/teleop/input_shaping.hpp` (`kDefaultDeadband`, `kDefaultExpo`,
  `DriverProfile`), `slew_limiter.hpp` (`SlewConfig`), `drive_modes.hpp` (`DriveMode`),
  `field_centric.hpp` (the latch), `driver_control.hpp` (`DriveShaper`, the order); the
  original signs and the hard-cut deadband remain in `teleop/stick_mapping.hpp`, whose
  `mapSticks` the drive program ("shulib Drive") and the bench tester's DRIVE station still call.
  Since G1 Part 2b "shulib Teleop" runs the feel layer instead (`src/programs/teleop_program.cpp`, through
  `teleop::runDriverControl`, `teleop/teleop_loop.hpp`), with this default profile; since G1 Part
  2c the invented X-drive does too (`src/programs/xdrive_program.cpp` — it replaced `src/main.cpp`'s R1a
  loop, the last caller of `mapSticksToChassisSpeeds`), and it refuses at boot on every robot that
  exists, so no driver has felt it there either.
  *Confidence:* **invented — every number of it.** Widened at T2 from "the mapping" to "the
  feel layer's defaults"; no new register number was minted, because nothing new was
  measured. **No claim about how any of it feels is made anywhere**, and none can be: the
  defaults were chosen to reproduce the shipped mapping as closely as a continuous deadband
  allows (identical at centre and at both rails; inside that range the rescale asks for at
  most 0.05 less), which is a compatibility argument, not a feel argument. The feel layer has
  never been driven by a person: the previous "shulib Teleop" drove robot two under a driver on
  2026-09-25 (eight starts, open-loop), but through the OLD raw mapping; the feel layer's first
  driver is the G1 Part 2b version's first run, which has not happened.
  *Settle (T2/R3, runbook step 10):* a human drives it. Signs that feel backwards get flipped
  at the mapping; the deadband, the expo and any slew rate get tuned by feel and recorded as
  a named profile rather than edited into a constant.
  *Blast radius if wrong:* annoying, visible, and shallow — exactly the kind of wrong that is
  fine to ship provisionally. **Contained:** the `teleop/` headers and one `DriverProfile`
  value; the mode functions, the curve and the rate limit are each off or identity by
  default, so a wrong default is a wrong *number*, never a wrong *shape*.

### R1b's additions — the beliefs about PROS the mechanism-sensor adapters are built on

Same rule as R1a's block: every semantic in the extended host shim (distance, optical, ADI,
usd) is one of these; the shim proves the adapters agree with the beliefs, never the beliefs.
Bench runbook steps 16–20 walk them. None of these sensors is known to be ON the available
robot — the steps say what can be done with a sensor on a loose cable at the table.

- [ ] **HA-113 — `Distance::get_distance()` returns MILLIMETERS.**
  *Claim:* the int32 is mm; a game piece 10 in away reads 254.
  *Source:* vendored `include/pros/distance.hpp:63,89` ("measured distance from the sensor in
  mm"); `include/shulib/hal/distance_conversion.hpp` (`distanceMmToCanonical`, the one ÷25.4).
  *Confidence:* reasoned — documented in the vendored source; unverified against firmware.
  *Settle (R3, runbook step 18):* an object at a tape-measured 500 mm reads ~19.7 in canonical.
  *Blast radius if wrong:* every capture/dock-confirm threshold off by 25.4× — confirm never
  fires (or always does). **Contained:** one conversion function.

- [ ] **HA-114 — the distance sensor's "no object" is the IN-BAND integer 9999, not a sentinel.**
  *Claim:* with nothing in range, `get_distance()` returns exactly 9999 (mm) — a plain value
  that converts to a plausible-looking 393.66 in — and NOT `PROS_ERR`, and not an errno state.
  *Source:* vendored `distance.hpp:71,98` ("Will return 9999 if the sensor can not detect an
  object"); `hal/pros/distance.hpp` (the T4 rule: raw 9999 → `confidence() == 0`).
  *Confidence:* reasoned — documented twice in the vendored source; the EXACTNESS of the value
  (is it always precisely 9999, never 9998 or a range?) is the unverified half.
  *Settle (R3, runbook step 18):* empty intake, raw value logged over ~30 s — confirm it is
  constant 9999; through the adapter, confirm confidence() reads 0.0 the whole time.
  *Blast radius if wrong (value differs on firmware):* the screen misses and dock-confirm gets
  a wall 33 feet away that thresholds as "no object anyway" (far), so the practical damage is
  a confidence that flickers non-zero on an empty intake — visible in step 18's log.
  **Contained:** one named constant + the adapter rule, mutation-proven (campaign M2).

- [ ] **HA-115 — `get_confidence()` is 0–63, "only available when distance is > 200mm"; its
  value at or below 200 mm is UNKNOWN.**
  *Claim:* above 200 mm the int32 is 0–63 (63 = high); at or below 200 mm the vendored doc
  says the channel does not exist, and we have NO belief about what it returns there — the
  adapter refuses to consult it in that range (a valid close reading reports confidence 1.0:
  the returned distance IS the detection).
  *Source:* vendored `distance.hpp:133-135`; `hal/pros/distance.hpp` (the close-range rule);
  `distance_conversion.hpp` (`distanceConfidenceToCanonical`, the ÷63).
  *Confidence:* reasoned — both documented halves; the below-200 behaviour is a stated unknown,
  not a guess.
  *Settle (R3, runbook step 18):* log raw `get_confidence()` with an object walked from 400 mm
  to 50 mm — record what the channel actually does below 200 (settles the unknown either way),
  and confirm the adapter's confidence() stays 1.0 through the close range.
  *Blast radius if wrong (0–63 wrong):* thresholds shift but stay monotone. If the adapter had
  instead passed the below-200 raw through and it reads 0 there: capture-confirm REFUSES the
  grab it most needs — the exact failure the close-range rule exists to prevent.
  **Contained:** adapter-only, mutation-proven (campaign M3).

- [ ] **HA-116 — optical hue is 0–359.999°, saturation/brightness are 0–1.0 doubles.**
  *Claim:* the three double channels are already in shulib's canonical ranges — the adapter
  applies NO scale, only the defensive clamp.
  *Source:* vendored `optical.hpp:79-80,103-104,127-128`; `optical_conversion.hpp`.
  *Confidence:* reasoned.
  *Settle (R3, runbook step 19):* a known-red object reads hue ≈ 0–20°, a known-blue one
  ≈ 210–230°, saturation well above 0.5 under room light.
  *Blast radius if wrong:* color windows match nothing (or everything) — loud at the first
  bench read. **Contained:** one conversion header.

- [ ] **HA-117 — optical `get_proximity()` is 0–255 and LARGER means CLOSER. THE POLARITY IS
  NOT IN THE VENDORED SOURCE.**
  *Claim:* the int32 range is 0–255 (documented) and 255 is "object at the lens" (NOT
  documented — the vendored comment states only the range; larger-is-closer is what community
  usage assumes).
  *Source:* vendored `optical.hpp:151-152` (range only); `optical_conversion.hpp`
  (`opticalProximityToCanonical`); `hal/optical.hpp` ("proximity ≈ 1 means an object is
  close").
  *Confidence:* **invented** — the polarity half has no written source; flagged the way HA-99
  was, and like HA-99 it must be measured before anything thresholds on it.
  *Settle (R3, runbook step 19 — BEFORE any capture threshold uses proximity()):* raw value
  with the sensor covered vs. facing open air. Covered ≫ open confirms the polarity; the
  reverse means the adapter needs a documented 1−x flip IN THE CONVERSION with this entry
  corrected.
  *Blast radius if wrong:* proximity-based capture-confirm fires on an EMPTY intake and never
  on a full one — inverted, silently plausible in any single reading. **Contained:** one
  conversion function; caught by one covered/uncovered pair at the bench.

- [ ] **HA-118 — optical failure sentinels: PROS_ERR_F on the double channels, PROS_ERR on
  proximity.**
  *Claim:* a dead/misconfigured optical port returns INFINITY from hue/saturation/brightness
  and INT32_MAX from proximity — screenable, and IN-BAND for nothing physical.
  *Source:* vendored `optical.hpp:87-88,111-112,135-136,159-160`; `hal/pros/optical.hpp`
  (per-channel screen → hold-last-good + faultedReads()).
  *Confidence:* reasoned.
  *Settle (R3, runbook step 19 + step 14's unplug matrix):* unplug the optical mid-read —
  values hold, faultedReads climbs, nothing propagates an infinity.
  *Blast radius if wrong:* an unscreened INFINITY in hue() breaks the F4 finiteness contract
  the estimator layer trusts; unscreened proximity clamps to 1.0 = "object present" from a
  DEAD sensor. **Contained:** adapter screens, tested against the shim's sentinel model.

- [ ] **HA-119 — ADI `DigitalOut` DRIVES THE LINE AT CONSTRUCTION; `set_value` is 1/0 with
  PROS_ERR on refusal.**
  *Claim:* constructing `pros::adi::DigitalOut(port, init_state)` physically drives the line
  to `init_state` immediately (PROS defaults it to LOW — on a pneumatic, the cylinder MOVES at
  object construction); `set_value(1/0)` drives it thereafter and returns PROS_ERR if the port
  is not configured as a digital output.
  *Source:* vendored `adi.hpp:546-547,564,596` ("The initial state for the port"),
  `adi.hpp:598-610`; `hal/pros/digital_out.hpp` (initial state REQUIRED, no default — the T3
  ruling).
  *Confidence:* reasoned — the ctor parameter is documented; "drives immediately at
  construction" (vs. at the first scheduler tick) is the unverified half, and it is exactly
  the half that decides whether a mismatched boot state physically moves a cylinder.
  *Settle (R3, runbook step 20 — AIR DISCONNECTED first):* construct with each initial state
  and watch the solenoid LED / listen for the click; then with air, confirm a matched
  declared-safe construction produces NO motion at boot.
  *Blast radius if wrong:* a clamp that flings its game piece (or opens on the goal) at every
  boot — physical, repeated, and blamed on "flaky pneumatics". **Contained:** the required
  ctor argument + the PneumaticMechanism agreement test.

- [ ] **HA-120 — ADI addressing: 1–8 ≡ 'a'–'h' ≡ 'A'–'H' on the brain; {smart, adi} pairs on
  an expander. Whether OUR robot HAS an expander is UNKNOWN.**
  *Claim:* the three spellings name the same 8 built-in ports; the pair form addresses an
  expander's bank identically; and — separately — the ADI expander R1a's session reported is
  UNVERIFIED (it was read from registry index 21, outside the documented 0–20 range, and never
  re-checked).
  *Source:* vendored `adi.hpp:59,62,91-93`; `hal/pros/digital_out.hpp` / `digital_in.hpp`
  (T6: one class, two ctors). The expander question: the R1a bench session record.
  *Confidence:* reasoned (addressing); the expander's existence is an open QUESTION, not a
  belief — recorded here so it cannot silently become one.
  *Settle (R3, runbook step 17):* the brain's Devices screen + a corrected 0–20 registry scan.
  A switch on port 'a' vs 1 vs 'A' reads identically (addressing); the expander question gets
  a yes/no with a photo.
  *Blast radius if wrong:* a mechanism wired to a bank that does not exist — loud at the first
  actuation test, not on the field. **Contained:** construction-time facts.

- [ ] **HA-121 — ADI `DigitalIn::get_value()` is a plain level (PROS_ERR on refusal);
  `get_new_press()` CONSUMES the press.**
  *Claim:* `get_value()` returns 1/0 for the line's level and PROS_ERR when the port is not a
  digital input — and PROS_ERR ≠ 0, so an unscreened read maps a DEAD port to "pressed";
  `get_new_press()` clears per-line edge state on read, so with two consumers one silently
  starves (the ADI sibling of HA-104).
  *Source:* vendored `adi.hpp:697-712,732-757`; `hal/pros/digital_in.hpp` (binds get_value
  ONLY — guard-pinned; PROS_ERR screened to last-good).
  *Confidence:* reasoned — the consume semantics are documented; the exact refusal value of
  get_value (PROS_ERR vs 0) is the weaker half.
  *Settle (R3, runbook step 20):* hold a switch — state() stays true (a level); unplug the
  wire mid-run and log the RAW get_value — settles the refusal value either way.
  *Blast radius if wrong (refusal is 0, not PROS_ERR):* the screen never fires and a dead port
  reads "released" instead of holding — safer than the pressed direction the screen exists
  for, but the faultedReads() observability goes blind. **Contained:** adapter-only,
  mutation-proven (campaign M6, M9).

- [ ] **HA-122 — SD card: `usd_is_installed()` returns 1/0; `fopen` NEEDS the `/usd/` prefix
  while `usd_list_files` FORBIDS it; `fflush` is the strongest persist available.**
  *Claim:* three beliefs about one subsystem: (1) `pros::usd::is_installed()` is a reliable
  1/0 card probe; (2) newlib file IO on the brain reaches the card only through paths
  prefixed `/usd/` — while `usd_list_files()` documents "DO NOT PREPEND YOUR PATHS WITH
  /usd/" (two conventions in ONE API, the registry-indexing shape R1a met); (3) there is no
  fsync in PROS's exposed surface, so a flushed write is only as durable as FatFS makes it.
  *Source:* vendored `misc.h:790-844` / `misc.hpp:555-612` (the is_installed contract and the
  list_files warning are vendored text; the fopen-needs-/usd/ half is PROS's file-IO
  documentation, NOT in the vendored headers — the weaker half, flagged);
  `hal/pros/block_sink.hpp` (owns the prefix in one place; bare-name precondition).
  *Confidence:* reasoned, flagged — belief (2)'s fopen half and belief (3) are not in the
  vendored source.
  *Settle (R3, runbook step 16):* boot with a card, write a blackbox, pull the card and read
  the file on a laptop (settles the prefix and the write path); boot WITHOUT a card and
  confirm the robot drives with isOpen()==false reported once (settles the probe); yank the
  card after a flushed write and count surviving bytes (bounds belief 3).
  *Blast radius if wrong:* a blackbox that never exists while everything reports healthy —
  the exact silent failure E1's bool-returning seam was built to surface. **Contained:** one
  adapter; the no-card path is mutation-proven (campaign M11).

---

## Robot 2 — the 2026 tank chassis (added 2026-09-14, R3b Parts 1–3)

The register was written for one robot. Robot two — the season's tank chassis, five coupled
motors per side driving four wheels per side, an IMU on port 2 and nothing else — is described
by `src/robots/chassis_table.hpp` (ports, signs, cartridge, IMU port, geometry) and the entries below
carry the facts about it as they are MEASURED, REPORTED or UNSET, each with its source and date.
Nothing here is invented for the robot itself: the invented entries (HA-132/133, and HA-136/137
added by R3d Part 1 on 2026-09-17) are the LIBRARY's thresholds and one SIM model shape, not the
robot's dimensions. Eighteen entries now — four measured on robot two (HA-127/128/129/130), one
reported (HA-124), two UNSET and refusing at boot (HA-125/126), seven reasoned
(HA-131/134/135/138/139/140/141), four invented (HA-132/133/136/137). The bench bot's rows above
are not rewritten.

**R3d Part 2 (2026-09-18) added HA-138, HA-139 and HA-140.** All three are about the PLATFORM
rather than the robot — what PROS's registry means by a port, what newlib does with a binary file
on the card, and what a record on that card is worth protecting with. They sit here rather than in
the R1a/R1b groups because they were found and are settled by robot two's session, and because two
of them are the direct inputs to "shulib Calibrate" and "shulib Teleop".

- [ ] **HA-124 — robot two's drive wheels are 2.75 in in diameter.**
  *Claim:* a ruler across the tread of any of robot two's eight drive wheels reads 2.75 in.
  *Source:* `src/robots/chassis_table.hpp` (`.wheelDiameterIn = 2.75`, tagged `A4 register HA-124`);
  consumed through `tableDriveGeometry()` into `tableToDescription()`, i.e. only when the library
  program falls back to the TYPED TABLE. *(Amended at R3d Part 2, 2026-09-18: "shulib Teleop" now
  builds from a `hal::DrivetrainDescription`, and when that comes from the SD card's calibration
  record the wheel diameter is not read at all — the record carries a MEASURED inches-per-radian
  per side instead, which is the radius × ratio product this entry and HA-126 together stand in
  for. The table remains the fallback, so the entry stays live until a record exists on the
  robot.)*
  *Confidence:* **reported** — the team lead, 2026-09-14: "standard wheels", the bench bot's
  measured size. Not yet rulered on THIS robot (worksheet Station F.0.3).
  *Settle (R3b / **R3d Part 3**):* the ruler; then R3d's one-push-along-a-tape scale calibration
  replaces the typed number with a measured inches-per-radian. That calibration EXISTS since R3d
  Part 1 (2026-09-17) and recovers the A2 plant's own radius to 1e-9; it has never been run on a
  robot, so this entry moves only on Part 3's bench log.
  *Blast radius if wrong:* every odometry distance scales by the error (a 2 % diameter error is
  a 2 % distance error, pinned quantitatively by the drive-encoder odometry's test 10); the
  stall check's spin travel scales identically, so the two never disagree with each other, only
  with the world. **Contained:** one table field.

- [ ] **HA-125 — robot two's track width, contact line to contact line.**
  *Claim:* NONE YET. There is no value in the tree. `src/robots/chassis_table.hpp` carries
  `.trackWidthIn = 0.0` (UNSET, tagged `A4 register HA-125`). *(Amended at R3d Part 2: the
  library program now refuses only when NEITHER source can describe a drivetrain — a calibration
  record on the SD card supplies an EFFECTIVE track width measured by a powered spin, HA-135, and
  a robot with one never reads this field. Without a record the typed table is still the second
  source, still UNSET, and the program still refuses at boot naming it.)*
  *Source:* `src/robots/chassis_table.hpp`; `src/common/boot_source.hpp` (the boot-source decision,
  `decideBootSourceFrom()` over `calibration::chooseBootSource`) and `src/common/boot_source.cpp`
  (`describeMissingForDescription`'s words, printed in the `typed table: cannot describe a
  drivetrain -- UNSET:` boot line) — in `src/main.cpp` until G1 Part 2b moved them out, unchanged.
  *Confidence:* **UNSET** — deliberately. A guessed track width turns every commanded yaw into
  the wrong wheel-speed difference (`kinematics/tank.hpp`) and every encoder-implied heading
  into the wrong number, silently; the drivetrain kind was stated, the width was not.
  *Settle (R3b / **R3d Part 3**):* tape measure, centre of the left contact line to centre of the
  right, front and back pairs (Station F.0.1); typed with provenance. R3d then measures it from the
  IMU (spin in place: Δθ_enc vs Δθ_imu — the cross-check `lastHeadingDisagreement()` exposes). That
  spin EXISTS since R3d Part 1 and recovers the plant's true width both ways to 1e-6 — but note
  that what it will produce on a floor is the EFFECTIVE width (HA-135), which is NOT the tape's
  number and is not meant to be.
  *Blast radius if wrong:* turns under-/over-rotate by the ratio of the error; the heading
  cross-check reads a steady bias on every turn (that is how a wrong width will be SEEN).
  **Contained:** one table field.

- [ ] **HA-126 — robot two's motor→wheel external ratio.**
  *Claim:* NONE YET. `.externalRatio = 0.0` (UNSET, tagged `A4 register HA-126`); the library
  program refuses while it is 0 AND no calibration record exists (amended at R3d Part 2 — see
  HA-125). "600 rpm" was reported, which names the blue cartridge and
  says nothing about the gears between the output shaft and the wheel.
  *Source:* `src/robots/chassis_table.hpp`; `hal/drive_geometry.hpp` (the ratio's one home).
  *Confidence:* **UNSET** — deliberately.
  *Settle (R3b / **R3d Part 3**):* "direct drive" (→ 1.0) or the tooth counts on BOTH sides (the
  R3a B1.2 amendment: ask per side — an asymmetric drivetrain is representable, two
  `DriveGeometry` objects); then R3d's push-along-a-tape calibration, which measures the radius ×
  ratio PRODUCT and cannot separate the factors — so once it has run, this entry is answered by
  being made unnecessary rather than by being filled in
  (`hal::DriveGeometry::fromInchesPerRadian`, R3d Part 1).
  *Blast radius if wrong:* the whole odometry scale (a 5:3 reduction typed as direct drive is a
  67 % distance error). **Contained:** one table field; the plant stays at 1:1 until a ratio
  other than 1:1 is reported (R3b Parts 1–3 D2-7).

- [x] **HA-127 — robot two's brain runs VEXos 1.1.5 (b18) under PROS kernel 4.2.2.**
  *Claim:* the kernel banner printed over USB at boot reads PROS 4.2.2, VEXos 1.1.5 (b18).
  *Source:* no in-tree source (exempt from direction 2) — measured from the serial capture,
  recorded in the development log on the `shulib-v2` branch (R3b Session 2 §9.1).
  *Confidence:* **measured** 2026-09-10.
  *Settle:* settled. Re-read the banner after any VEXos update.
  *Blast radius if wrong:* n/a — a fact about the brain, recorded so a future "it worked on
  1.1.5" has a number beside it. `arm-none-eabi-g++ 13.2.1` builds for it.

- [x] **HA-128 — starting a program from the CONTROLLER's menu arms a pretend field-control
  match on the brain.**
  *Claim:* launching (Run / Timed Run / Match) from the controller's own program menu puts
  VEXos into a competition-system mode (status `0x0f`: CONNECTED | DISABLED | AUTONOMOUS |
  SYSTEM) that persists across `pros v5 stop` / `run` until the controller leaves that mode;
  a controller linked and left on its home screen does not, and the program then runs in
  driver control normally.
  *Source:* `src/common/state_screens.{hpp,cpp}` (the competition-state screens, moved out of
  `src/main.cpp` at G1 Part 2c, output unchanged) and each program's boot-time status print (the
  bits `src/main.cpp` formats and passes, `A4 register HA-128`); the worksheet's Station A rule.
  *Confidence:* **measured** 2026-09-10 — the program restarted and went black when the
  controller linked; turning the controller OFF cleared it within seconds; the decisive test
  (controller linked, hands off, program started from the laptop) ran driver control for
  minutes.
  *Settle:* settled; the rule is in the worksheet and Daniel's note.
  *Blast radius if wrong:* n/a — the screen now names the state, so the diagnosis cannot again
  be "the touchscreen does not work".

- [x] **HA-129 — the brain's USB CDC ports renumber between plug-ins.**
  *Claim:* the brain enumerates as `ttyACM0/1` alone and `ttyACM1/2` beside the controller;
  a serial read must re-resolve the port from `pros lsusb` every time. `pros terminal` cannot
  run from a non-interactive shell (`termios` on stdin); raw reads of the USER CDC port work,
  COBS-framed with a `sout` channel tag (`strings` recovers the text lines whole).
  *Source:* no in-tree source (exempt) — the development log, R3b Session 2 §9.1.
  *Confidence:* **measured** 2026-09-10.
  *Settle:* settled. *Blast radius if wrong:* a capture from the wrong device, nothing more.

- [x] **HA-130 — robot two's port 18 travels ~20 % short of its side-mates on a push.**
  *Claim:* on a whole-robot hand push, port 18's encoder advances about 20 % less than the
  other four motors on its side (137 vs ~175 deg on the first push, 171 vs ~210 deg on the
  second), reproducibly — a member that is SLOW, not one that opposes. Cause UNKNOWN.
  *Source:* `src/robots/chassis_table.hpp` (the table comment, `A4 register HA-130`);
  `hal/motor_group.hpp` (the header cites it as the reason the disagreement floor must
  tolerate a slow member); `test/motor_group_test.cpp` test 16 (a member at 80 % of its mates
  is NOT flagged at rest or under command; the same member opposite in sign IS — pinned, with
  the threshold mutation observed red); and, since R3d Part 1,
  `include/shulib/calibration/drivetrain_calibrator.hpp`, which cites it as the reason each side's
  distance scale is taken through `hal::medianInPlace` — the SAME function the group will report
  `position()` with at run time, so a member that travels short cancels EXACTLY between the
  calibration and the run rather than approximately.
  *Confidence:* **measured** 2026-09-10, twice. What it means mechanically is not.
  *Settle (R3b / R3d Part 3):* watch it under power (the panel's `disagree` counts and the group's
  median); if it persists, swap the motor and re-measure. R3d's calibration report gives a second,
  sharper reading for free: it prints each side's member SPREAD over its push (the one 48 in push
  since R3d Part 2F-1 — there is no separate tape push any more), so one run says how short port 18 is
  against its four mates without anyone watching a live panel; and since 2F-1 its per-pulse serial
  record says whether port 18's own pulse moves the robot at all (a slipping gear is LEFT OUT by name,
  HA-134).
  *Blast radius if wrong:* if it turned out to be a fight rather than a shortfall, the group's
  median would still ignore it (one of five) and the monitor would count it; if the shortfall
  grew past 75 %, the floor (HA-133) would report it as frozen. Either way it is visible.

- [ ] **HA-131 — on robot two, ODO_STUCK cannot be detected by the wheels-vs-odometry
  cross-check.**
  *Claim:* with the drive encoders as the only odometry source (no tracking wheels), the
  `OdoStallCheck` comparison "wheels rolled but the estimate did not move" is a tautology — the
  estimate IS the wheels — and would report "not stalled" forever; so the check is configured
  with `independentMotionSource = false` on this robot, never reports a verdict, and a stuck
  robot is INVISIBLE to the health monitor until a detector built on the encoder-vs-IMU heading
  cross-check exists.
  *Source:* `include/shulib/motion/odo_stall_check.hpp` (the §2 ruling; `A4 register HA-131`);
  `src/programs/teleop_program.cpp` (`kNoIndependentStallSourceNote` logged once at boot, in two pieces
  so the 200-byte log line does not cut it; `src/main.cpp` until G1 Part 2b); `localization/
  drive_encoder_odometry.hpp` (`lastHeadingDisagreement()`, the future detector's input).
  *Confidence:* reasoned (structural — a fact about the sources, not a measurement).
  *Settle (**R4** — RE-HOMED from "R3d/R4" on 2026-09-17 by R3d Part 1):* a stuck/slip detector on
  the heading cross-check plus current, measured against real pushes; or tracking wheels, which
  restore the independent source. **Why it moved:** R3d builds a DESCRIPTION of a drivetrain —
  which ports, which signs, which sides, what scale, what width. A stuck/slip detector is not a
  description; it is a run-time detector, and it cannot be tuned without real pushes on a real
  floor (the host plant's slip is a declared scenario fact, not an emergent one), which is R4's
  whole subject. R3d Part 1 does hand that detector its input, ready and host-proven: the effective
  track width the cross-check has to read zero against.
  *Blast radius if wrong:* n/a in the honest direction — the alternative was a monitor that
  lies. The cost is the missing detection itself: the driver is the supervisor (the panel says
  so), and an auton that pushes a wall will not fault; its watchdog bounds the damage.

- [ ] **HA-132 — the coupled-side agreement thresholds separate a fight from a transient.**
  *Claim:* evaluating only while a side is commanded above 1 V and its fastest member turns
  above 1 rad/s; flagging a member opposite the command above 0.5 rad/s or under 25 % of the
  side's fastest; and requiring 25 consecutive 10 ms ticks (250 ms) — together catch a real
  fight (a wrong sign, a stripped gear, a frozen encoder) and ignore a stick reversal's coast.
  *Source:* `include/shulib/teleop/coupled_side_monitor.hpp` (`SideMonitorConfig`,
  `PROVISIONAL (A4: HA-132)`); consumed by "shulib Drive", the tester's DRIVE station, and
  `hal::MotorGroup` — one evaluator, one set of numbers.
  *Confidence:* **invented** for a first run (R3b Session 2's own words). No fight has yet been
  observed under power; the 250 ms window equals the over-current window by choice.
  *Settle (R3d/R4):* the drive logs' cut lines and the library's MOTOR_GROUP_DISAGREE episodes
  over a season of practice; a real fight (one motor deliberately reversed, wheels up, 3 V)
  must persist; a stick reversal at full speed must not.
  *Blast radius if wrong:* too tight = a healthy drive cut every second (the drive program) or
  a fault line every run (the library); too loose = a fight stalls five motors against four
  until something strips. **Contained:** one config struct, five fields.

- [ ] **HA-133 — the disagreement FLOOR: under 25 % of the side's fastest is disagreeing; 80 %
  is not.**
  *Claim:* robot two's port 18 at ~80 % of its mates (HA-130) must NOT be reported as a fight,
  while a member at 0 rad/s (a frozen encoder) or turning backwards must; 25 % of the fastest
  is the line, and it holds on every real drivetrain.
  *Source:* `include/shulib/hal/motor_group.hpp` (the header's policy note, `A4 register
  HA-133`); `test/motor_group_test.cpp` test 16 (the 20 % case tolerated, the opposite case
  flagged; the threshold mutation 0.25 → 0.85 observed red).
  *Confidence:* **invented**; the 80 %/25 % gap is wide by design.
  *Settle (R3d/R4):* the distribution of member velocities on a healthy side under power (the
  panel shows every member); if a healthy member ever sits below 50 % of its mates, the floor
  moves.
  *Blast radius if wrong:* a slow-but-healthy member reported as a fight on every run (and a
  fault line per run), or a nearly-frozen member (25–50 %) missed. **Contained:** one field
  of HA-132's struct.

### R3d Part 1's additions — the beliefs the self-calibration is built on (2026-09-17)

Four entries, added by the chunk that made the robot able to describe its own drivetrain. **None
of the four has been measured on anything**: two are reasoned from mechanism, one is a set of
invented thresholds, and one is a belief about the host PLANT rather than about a robot. The
calibrator they belong to is proven against the A2 plant and has never moved a physical drivetrain.

- [ ] **HA-134 — on a tank, one side forward alone turns the robot toward the other, and the IMU's
  yaw SIGN is therefore what names a drive port's side.**
  *Claim:* driving ONE side of a tank forward while the other is unpowered turns the robot: the LEFT
  side forward turns it CLOCKWISE, which is NEGATIVE yaw in shulib's CCW-positive canonical frame
  (`hal/imu.hpp`); the RIGHT side forward is positive. The drivetrain calibrator pulses one member at
  a time and reads that member's side off that sign alone. **Amended at R3d Part 2F-1 (2026-09-24):**
  the claim is only about the SIGN; how MUCH one member turns the robot is not claimed at all. The
  pulse therefore RAMPS — 3, then 6, then 9 V, each for 0.4 s with the robot at rest between (motor
  velocities AND the IMU's yaw rate under their floors) — and stops at the first level whose turn
  reaches the 2° floor. A port that no level turns is CLASSIFIED rather than refused blind: its own
  shaft turned while nothing else moved → a slipping gear, LEFT OUT of the description with a
  warning; other drive ports moved but the robot did not turn → it ROLLED (or is off the floor),
  refused naming the port; nothing moved → refused naming the port.
  *Source:* `include/shulib/calibration/drivetrain_calibrator.hpp` (the header's phase table and
  `finishPulse()`, `A4 register HA-134`); `test/drivetrain_calibrator_test.cpp` case 6 — every LEFT
  pulse's Δθ negative, every RIGHT pulse's positive, the resulting sides equal to the plant's wheel
  membership exactly, with the rule-swapping mutation observed red (since 2F-1 it is caught ON THE
  ROBOT too: a flipped rule makes the spin turn the wrong way and the new wrong-way guard stops it at
  15°); cases 22–26 for the ramp and the classification, on a plant that can now roll (HA-137).
  *Confidence:* reasoned — rigid-body kinematics. Note honestly what the host suite does and does
  not prove: it AGREES with the frozen F5 contract (`kinematics/tank.hpp`'s `forward()` computes
  ω = (right − left)/track, so left alone gives ω < 0), and the plant turns the robot USING that
  same contract — so the plant cannot be an independent check of the claim. What it does check is
  that the calibrator reads the sign the same way round as the kinematics writes it, which is the
  half that a typo would break.
  *What the first bench session showed (2026-09-24, R3d Part 3):* five starts of the Part 1 version
  (one 3 V pulse): three stopped at port 11 (0.14°, 0.17°) or before any pulse, and one reached port
  18, so ports 11–17 each cleared the 2° floor in that start — but the program printed no per-pulse
  numbers, so neither the SIGN of those turns nor whether anyone touched the robot is on record, and
  the port-11 stops carry a POSITIVE yaw where this entry predicts negative (noise at that size, or a
  push made back-first while FRONT was answered — open). **Not settled.**
  *Settle (R3d Part 3):* the first floor run of the 2F version with its serial record: one line per
  pulse (port, level, yaw, own travel, others' travel, verdict). Every LEFT member's deciding yaw
  negative and every RIGHT member's positive settles the sign; the levels used say how strong a
  single-member pulse must be on robot two.
  *Blast radius if wrong:* the two sides swap, so every commanded turn goes the wrong way and the
  odometry's heading cross-check reads double instead of zero. **Contained:** one comparison in
  `finishPulse()`. Loud rather than silent — and since 2F-1 caught before any record is written: the
  spin's wrong-way guard refuses at 15°.

- [ ] **HA-135 — the EFFECTIVE track width from a powered spin is LARGER than the taped one.**
  *Claim:* on foam, a tank turning in place SCRUBS — the wheels slide sideways as well as rolling —
  so the drive encoders over-count the turn relative to the yaw the robot actually achieves, and
  `W = (ΔR − ΔL) / Δθ_imu` comes out LARGER than a tape measure's contact-line-to-contact-line
  width. The effective number is the one `TankKinematics` needs to command a turn that lands and
  the one the heading cross-check must read as zero; the taped number is the fallback's provenance
  and is a different quantity, not a better or worse estimate of the same one.
  *Source:* `include/shulib/calibration/drivetrain_calibrator.hpp` (T8, `finishSpin()`,
  `A4 register HA-135`); `include/shulib/hal/drivetrain_description.hpp`'s `trackWidth` field
  comment; `test/drivetrain_calibrator_test.cpp` test 13, which adds a turn-scrub model the plant
  does not itself have: 5 % scrub yields 13.157 in against a true 12.5 — larger, in the predicted
  direction, and within 8 %.
  *Confidence:* reasoned. The A2 plant scrubs NOTHING, so the host recovers the true width exactly
  (1e-6 in both directions) and can say nothing whatever about the size of the gap. The DIRECTION
  is reasoned from the mechanism; the RATIO is unmeasured and no number in the tree claims it.
  *Settle (R3d Part 3):* tape the width, then calibrate, and record both numbers and their ratio,
  on the same robot on the same floor. That ratio is the first real datum about this claim.
  *Blast radius if wrong:* if the effective width came out SMALLER than the tape's, the scrub model
  is backwards and the reason for preferring a measured width to a taped one is wrong — visible
  immediately, because Part 3 records the two numbers side by side. Either way the calibrated
  number remains the one the kinematics wants, because it is defined by the measurement it came
  from.

- [ ] **HA-136 — every number in `CalibratorConfig` is invented for a first run (and so is every
  number in the calibration screen model).**
  *Claim:* the calibration's thresholds — `movedThreshold` 5° (carried unchanged from the bench
  tester's MOTOR WATCH, where it was invented too), `restFloor` 0.05 rad/s, `restYawRateFloor` 2°/s,
  `restSettleTime` 0.5 s, `restTimeout` 3 s, `pulseLevels` 3 / 6 / 9 V (`pulseLevelCount` 3),
  `pulseTime` 0.4 s, `pulseYawFloor` 2°, `decoupledOwnMin` 20° of shaft, `pushDistance` 48 in,
  `straightBound` 3°, `minPushRad` 2π (one wheel turn), `spinLevels` 3 / 6 V (`spinLevelCount` 2),
  `spinStartTime` 2 s, `spinStartYaw` 5°, `spinWrongWay` 15°, `spinTurns` 2, `spinTimeout` 20 s,
  `spinAgreement` 5 %, `imuReadyTimeout` 5 s, `commandTimeout` 0.1 s — plus the two reproducibility
  tolerances the class CLAIMS (`kReproScaleTolerance` 2 %, `kReproWidthTolerance` 3 %) — together
  let a real drivetrain be described without either refusing a good run or accepting a bad one.
  `pushDistance` is a CHOICE, not a belief: two field tiles, stated on the screen, the only distance
  there is (there is no distance question any more, so a wrong one cannot be answered).
  **Changed at R3d Part 2F-1 (2026-09-24):** ADDED `restYawRateFloor`, `pulseLevels` +
  `pulseLevelCount`, `decoupledOwnMin`, `pushDistance`, `spinLevels` + `spinLevelCount`,
  `spinStartTime`, `spinStartYaw`, `spinWrongWay`; REMOVED `pulseVolts` (3 V — now the ramp's first
  level) and `spinVolts` (3 V — now the spin's first level); CHANGED `straightBound` (it now bounds the
  ONE push, which also captures the signs — a twist could otherwise roll one side backward and invert
  its signs) and `minPushRad` (checked at DONE over the moved ports AND per side once the sides are
  known). One relation is now a precondition, not a hope: `restYawRateFloor × pulseTime <
  pulseYawFloor` (0.8° < 2°), so a drift slow enough to pass the rest check can never, on its own,
  cross the floor during a pulse — and a side can only come out wrong through a leak at least as big
  as the floor. **The screen model's estimates** (`calibration/calibrate_panel.hpp`): characters per
  472-pixel line LARGE 18 / MEDIUM 36 / SMALL 50 (and two sub-budgets, the header word 24 and the note
  beside RUN AGAIN 18), line heights 34 / 20 / 16 px, touch padding ±4 px sideways and ±10 px up and
  down, and a 0.5 s arming delay after every layout change — none measured; every literal is checked
  against the budgets at compile time, so a measured number is a one-constant change. *(R3d Part
  2F-2, 2026-09-24: the three character budgets were checked against what earlier programs showed
  legibly on a V5 — the SCREEN RULER could not be run — and kept: SMALL 50 sits under the bench
  tester's 54-character lines, MEDIUM 36 equals the widest MEDIUM line read whole in a photograph of
  the panel (the tester's DRIVE row), and LARGE 18 lies between the 12 characters read whole (the IMU
  test's live number) and a 23-character state screen reported shown. They are DERIVED lower-bound
  evidence, not a measurement; the SCREEN RULER still settles them.)*
  *Source:* `include/shulib/calibration/drivetrain_calibrator.hpp` (`CalibratorConfig`, every field
  tagged `PROVISIONAL (A4: HA-136)`); `include/shulib/calibration/calibrate_panel.hpp` (each budget
  tagged ESTIMATE); `test/drivetrain_calibrator_test.cpp`, where the push threshold, the pulse yaw
  floor, the missed-tick bound and the spin agreement are each attacked from BOTH sides — a value
  just under the line and the same value just over it — and the ramp, the rest's yaw-rate floor, the
  decoupled minimum and the spin's start check and wrong-way bound each have a case that fails
  without them (each mutation run and observed red at R3d Part 2F-1).
  *Confidence:* **invented**, every one, for a first run. The host plant can only show that each
  threshold FIRES where it is set, never that it is set in the right place. The ramp's first level
  and the spin's inherit the DRIVE station's first ceiling (3 V), whose only history is "nothing
  broke at 3 V" — and the first bench session showed 3 V on one member of five is intermittent on
  robot two (HA-134).
  *Settle (R3d Part 3):* one calibration on the floor with the serial record printed: the push table,
  one line per pulse (the level each port needed), one line per spin direction (the level it
  finished at). `restFloor`, `restYawRateFloor` and `pulseYawFloor` are the most likely to move; all
  three are bounded below by sensor noise nobody has measured. The screen's numbers settle at Bench
  Tests' SCREEN RULER (the last digit visible on each size's row).
  *Blast radius if wrong:* too tight and a good drivetrain cannot be described — a refusal, loud,
  with the offending number in the message; too loose and a bad one is described confidently, which
  would bake a wrong sign or a wrong side into a record the competition program then trusts. The
  asymmetry is deliberate: every threshold is placed so that its failure mode is a REFUSAL — and the
  one new relaxation, a slipping member LEFT OUT instead of refused, says so on the screen and in the
  record's absence of that port. **Contained:** one config struct and one screen header.

- [ ] **HA-137 — a coupled train commanded unequal volts turns at the MEAN of its members'
  effective volts; an all-coasting wheel may follow the driven one; a slipping member drives nothing
  (beliefs about the SIM, not about a robot).**
  *Claim:* in the A2 plant, a wheel whose coupled members carry DIFFERENT commands turns as though
  it were commanded the mean of them (a `Reversed` member's volts negated first); when every member
  carries the SAME command — which is what a `hal::MotorGroup` always produces — the wheel takes
  that value, bit for bit. **Added at R3d Part 2F-1 (2026-09-24), both OFF by default:**
  (a) **coast-follow** — `DrivePlantConfig::coastFollow`, κ ∈ [0, 1], tank only: a wheel whose every
  coupled member is at exactly 0 V AND in `BrakeMode::Coast` turns at κ × the driven wheel's spin
  instead of being pinned (a Brake/Hold side stays pinned). One side driven at v then gives
  ω = (1 − κ)·v/W and vx = (1 + κ)·v/2: κ = 0 is the old pinned pivot, κ = 1 pure translation.
  (b) **`MemberFault::Decoupled`** — a slipping gear: the member's command is left out of its
  train's command, and its encoder reports its own free spin plus `decoupledDrag` (default 0) of the
  train's travel.
  **Why (a) and (b) exist — and why the calibrator no longer rests on the pinned wheel:** until 2F-1
  every unpowered wheel was pinned (an implicit perfect brake) and the calibrator's own test rig ran
  with no static friction, so a one-sided pulse could only PIVOT and always passed; at the tree's
  usual kS = 1.2 the same single 3 V pulse turns this plant by EXACTLY nothing
  (`test/drive_plant_coupled_test.cpp` case 25 keeps that as a regression), and on robot two it
  turned the robot 0.14–0.17°. The calibrator's cases now run at kS = 1.2 with κ stated in every one,
  including κ = 0.9 and 1 (the robot rolls and the pulse is refused ROLLED, naming the port) and a
  slipping member (left out, and the run still saves).
  *Source:* `include/shulib/sim/drive_plant.hpp` (`wheelCommand()`, `coupledOnlyCommand()`,
  `stepWheelsCoastFollow()`, `wheelCoasting()`, `MemberFault::Decoupled`, `A4 register HA-137`);
  `test/drive_plant_coupled_test.cpp` cases 18–26. Every other suite is bit-identical with both
  extensions at their defaults (assertion counts and every printed value, compared case by case
  before and after at R3d Part 2F-1).
  *Confidence:* **invented**, and deliberately SIM shapes rather than robot facts. Everything is
  torque-free like the rest of the plant (`motor_model.hpp`'s honesty boundary). The mean is
  OPTIMISTIC — a real motor back-driving four gearboxes reaches less than 1/N of its free speed. κ is
  a KINEMATIC stand-in, not derived from a wheel type, and the direction a real robot takes is not
  obvious: lateral scrub (traction wheels) pushes a one-sided push toward ROLLING (κ → 1), while a
  rigid body whose idle side rolls freely with no lateral force — the ideal of an all-omni robot like
  robot two — turns MORE than a pinned pivot (for a uniform square robot its idle side runs backward
  at about a fifth of the driven side's speed, a negative κ this field does not allow). So κ ∈ [0, 1]
  spans from the pass case to the failing one; which end robot two is nearer is exactly what the 2F
  serial record will show.
  *Settle (R6 — back-fit, beside HA-48's model-shape question):* compare one member's pulse on a
  real robot (the 2F record: the level it needed, the yaw it gave, how far the OTHER side's encoders
  moved) against the plant's prediction for the same command. If the robot needs materially more
  than 1/N, the mean becomes a weighted model; the other side's travel over a pulse is κ's first
  measurement.
  *Blast radius if wrong:* the host suite predicts pulses that turn a real robot more (or less) than
  it does — which the ramp and the classification now absorb: a pulse too weak steps up, and one that
  never turns the robot is refused naming the port and saying whether it rolled or nothing moved.
  It cannot make a calibration WRONG, only make one fail to finish. **Contained:** four functions in
  the plant, all behind switches that default off.

- [ ] **HA-138 — `registry_get_plugged_type(i)` is ZERO-indexed: registry index i is PHYSICAL
  PORT i + 1, and the valid indices are 0–20.**
  *Claim:* PROS's smart-port registry is the ONE API in the tree that is not numbered the way
  humans, motors, sensors and every other PROS call number ports. Index 0 is physical port 1;
  index 20 is physical port 21; index 21 does not exist.
  *Source:* vendored `include/pros/apix.h:640-660` — "Returns the type of the device plugged into
  the **zero-indexed** port … The V5 port number from 0-20", and `ENXIO` outside that range. The
  device type values are vendored `pros/device.h:48-63` and are NOT consecutive (NONE 0, MOTOR 2,
  ROTATION 4, IMU 6, DISTANCE 7, RADIO 8, VISION 11, ADI 12, OPTICAL 16, GPS 20, AIVISION 29,
  SERIAL 129, UNDEFINED 255). Bound in `include/shulib/hal/pros/device_registry.hpp`, where the
  conversion is ONE marked line; shimmed in `test/pros_shim/pros/apix.h`, which is indexed the
  PROS way so the adapter's conversion is the thing under test rather than a convention the shim
  shares with it; exercised by `test/pros_device_registry_adapter_test.cpp` from BOTH ends
  (physical 1 → index 0, physical 21 → index 20, asserted directly through the shim's recorded
  `lastIndex`), and out-of-range ports are asserted never to reach PROS at all.
  *Confidence:* **reasoned** — the vendored text states it plainly, and it already cost a session:
  `src/programs/bench_r3a.cpp`'s registry comment (just above `kMaxRegistryIndex`; lines 135-145 since
  R2a, 112-122 before it) records 2026-08-18, when the tester scanned 1..21 as if the index
  were physical, shifted every reading by one port and invented a device at index 21. HA-120 had
  PREDICTED exactly that (the 2026-08-13 "ADI expander" sighting came from index 21) and the
  prediction was not acted on. This entry exists so the mapping is a NAMED belief with a settle
  step rather than a comment in a tester.
  *Settle (R3d Part 3, Station G's census):* run "shulib Calibrate" beside the brain's own Devices
  screen. The program's `STAGE 0` census prints one line per non-empty PHYSICAL port; every line
  must name the same port and the same device as the brain does. Ten motors on 11–20, an IMU on 2
  and a radio on 1 is the expected answer for robot two.
  *Blast radius if wrong:* a calibration that pulses the wrong motors, or a boot check that
  compares a record against a port map shifted by one and refuses a healthy robot — confidently,
  and with a message naming the wrong ports. **Contained:** one line, one adapter, and the test
  that pins it from both ends.

- [ ] **HA-139 — `fopen(path, "rb")` under `/usd/` reads back exactly the bytes `"wb"` wrote.**
  *Claim:* the read half of HA-122. A calibration record written through `ProsBlockSink`
  (`fopen`+`fwrite`+`fflush`) and read back through `ProsBlockSource` (`fopen("rb")`+`fread`) is
  byte-identical on the V5's FatFS — no newline translation, no 0x1A end-of-file interpretation,
  no short read on a file that is entirely present.
  *Source:* `include/shulib/hal/pros/block_source.hpp`;
  `test/pros_block_source_adapter_test.cpp`, which writes NUL, LF, CR, CR+LF, 0x1A, DEL and 0xFF
  through the sink and reads them back through the source against a REAL temp directory.
  *Confidence:* **reasoned, and FLAGGED.** The host test proves the two adapters agree on Linux —
  where `"rb"` and `"r"` are the same thing, because POSIX does not translate. **No host test can
  fail if the mode is wrong**, and newlib on the brain is a different library. The mode is
  therefore a belief that only a brain can settle, and it is written down as one instead of being
  claimed by a test that could not see it.
  *Settle (R3d Part 3, Station G):* run "shulib Calibrate" to `SAVED` — which is itself the
  measurement, because the program writes, flushes, CLOSES, re-opens through `ProsBlockSource`,
  reads back and compares the bytes before it says the word (ruling P5). Then pull the card and
  compare the 180 bytes on a laptop (`cmp`, or `xxd`), which settles it independently of the
  program's own opinion.
  *Blast radius if wrong:* `NOT SAVED` on the brain, in red, with the reason — the read-back is
  precisely what makes this failure loud instead of silent. The worse variant, a file that reads
  back subtly changed and still passes a byte compare, cannot happen: the compare IS byte-exact,
  and the CRC would refuse the difference at the next boot. **Contained:** one adapter and the
  Done path of one program.

- [ ] **HA-140 — a CRC-32 over the calibration record is worth its four bytes, where the
  blackbox's format deliberately carries none (a DESIGN note under HA-122).**
  *Claim:* the two on-card formats this tree writes take OPPOSITE decisions about integrity, and
  both are right. `diag/blackbox_format.hpp` has no checksum: it is read by a HUMAN after the
  fact, frame by frame, a truncated tail is expected and decodable, and the cost of a corrupt
  field is a misleading number in an investigation. `calibration/calibration_record.hpp` has a
  CRC-32: it is read by the ROBOT, once, at boot, and every field is immediately load-bearing — a
  flipped bit in a SIGN reverses a motor against its side-mates, one in a SCALE makes every
  odometry inch a lie, one in the track width's exponent makes every commanded turn wrong, and
  none of those is visible from outside.
  *Source:* `include/shulib/calibration/calibration_record.hpp` (`crc32Ieee`, computed BITWISE with
  no lookup table — 1 KiB of flash for 180 bytes of input is the wrong trade in a header a
  composition root includes); `test/calibration_record_test.cpp`, which pins the arithmetic against
  the published CRC-32/IEEE check values (`"123456789"` → 0xCBF43926) and flips one bit in each
  field class.
  *Confidence:* **reasoned.** The polynomial and its properties are standard; what is a JUDGEMENT
  is the trade — four bytes and a few microseconds per boot against the class of silent failure it
  removes. It is recorded here because it is exactly the kind of decision a later chunk would
  otherwise re-open without knowing why the blackbox differs.
  *Settle (R3d Part 3):* not a measurement so much as a confirmation — the record decodes on the
  brain and the boot log prints the description it spells. If the bitwise CRC ever shows up in a
  loop-rate measurement (it will not: 180 bytes × 8 iterations, once per boot), the table version
  is a two-line change.
  *Blast radius if wrong (the CRC is not worth it):* four wasted bytes. **If it were ABSENT:** a
  corrupt record drives a robot wrong, confidently, and the first symptom is a robot that will not
  drive straight. The asymmetry is the whole argument. **Contained:** one function, one format.

**G1 Part 2a (2026-09-24) added HA-141.** Like HA-138 it is about the PLATFORM, and it is the
belief the new on-robot device provider rests on: HA-138 says WHICH registry index is which port;
this says the KIND the registry reports on that port can be trusted for the three sensor kinds.

- [ ] **HA-141 — the registry's KIND answer is right for an IMU, a rotation sensor and a GPS, and
  an empty port reads NONE.**
  *Claim:* `registry_get_plugged_type()` on the index of a port carrying a V5 inertial sensor
  returns `E_DEVICE_IMU` (6), on a rotation sensor `E_DEVICE_ROTATION` (4), on a GPS
  `E_DEVICE_GPS` (20) — the same way it has been relied on to return `E_DEVICE_MOTOR` (2) for a motor
  since R3d Part 2 — and on a port with nothing plugged in, `E_DEVICE_NONE` (0). False if any of the
  three reports a different value, `E_DEVICE_UNDEFINED`, or NONE while plugged in (for example
  while the sensor is still enumerating after power-up).
  *Source:* `include/shulib/hal/pros/device_provider.hpp` (`ProsDeviceProvider::expectKind()`,
  tagged `A4 register HA-141`), which asks before constructing ANY port device and refuses on any
  other answer; the kind mapping is `hal/pros/device_registry.hpp`'s `kindOf()` over the vendored
  `pros/device.h:48-63` values; host-tested through `test/pros_shim/pros/apix.h` by
  `test/pros_device_provider_test.cpp` (cases 1b, 1c, 1k), which — like every shim test — tests the
  provider against this belief and cannot test the belief.
  *Confidence:* **reasoned** — the vendored enum names each kind, and for ONE of the three a census
  has already agreed with the robot: robot two's census read the IMU on port 2 (2026-09-10, and again
  2026-09-24), alongside MOTOR on its drive ports and the radio on port 1. **What is new is the
  DEPENDENCE:** until this part only the
  boot check read the registry, and only for motors. `ProsImu` and `ProsRotation` cannot throw and
  `ProsGps` defers on an unreadable device, so for those three kinds the registry's answer is now the
  ONLY thing that can refuse an empty or wrong port — which is why the belief gets its own entry.
  No rotation sensor or GPS has ever been plugged into either robot.
  *Settle (G1 Part 2b's first boot of "shulib Teleop" / R3) — reworded at G1 Part 2b:* on
  "shulib Teleop" the boot check reaches every IMU and drive-port fault FIRST, through the same
  registry call (`calibration/boot_check.hpp`'s `verifyAgainstRegistry`, which refuses a record
  whose IMU port no longer reads IMU), so a pulled IMU cable is refused by the BOOT CHECK and never
  reaches the provider — that boot cannot show "nothing plugged in (expected an IMU)". So the belief
  settles in two halves: the **IMU-present half** on the G1 Part 2b program's first boot (the
  provider constructs the IMU on the registry's word and starts its calibration); the **provider's
  kind refusal** on the first program that asks the provider for a port the boot check does NOT
  verify — a GPS or a rotation sensor, the day one first exists on a robot — with a pair of boots
  (plugged, then unplugged) for each. No worksheet step for it until then. The census beside the
  brain's own Devices screen (HA-138's settle step) reads the kinds at the same time.
  *Blast radius if wrong:* **refuses rather than lies.** A sensor the registry mis-kinds is REFUSED
  at boot, by port, with what was found — a robot that will not start, on a bench, with the reason on
  the screen; it cannot become a live adapter over an empty port (the failure this check exists to
  prevent). The one silent variant would be the registry reporting the right kind for an EMPTY port,
  which would put us back where the provider started — an adapter over nothing — and the IMU's own
  `isReady()` never going true is then the only tell. **Contained:** one check, in one provider.

**DEFECTS2 (2026-09-26) added HA-142–HA-146.** Found by the read-only research for T3 (claw pivots
that must HOLD their angle on release): every Brake and Hold stop in the library reached a V5 as a
0 V command, which PROS documents as ignoring the brake mode — so a declared Hold went limp and the
drive's "0 V + Brake" cancel coasted, while every host test passed (the fakes echo the mode, the A2
plant pins a wheel at 0 V, the shim had no `brake()`). The fix is one function in the PROS adapter
(`hal/pros/motor.hpp`, "THE STOP"); these five entries are what it — and everything that stops a
motor — now rests on. **None of them has been observed on a robot.**

- [ ] **HA-142 — `move_voltage()` and `move()` ignore the brake mode; only `brake()` applies it.**
  *Claim:* a V5 motor commanded `move_voltage(0)` is under VOLTAGE control at 0 V whatever its
  brake-mode register says; the mode acts only after `brake()`, which "sets motor velocity to zero,
  which will cause it to act according to the set brake mode". So a stop that must brake or hold is
  "the mode + `brake()`", never "the mode + 0 V". False if a motor set to Hold and commanded
  `move_voltage(0)` holds its position, or if `brake()` under Hold does not.
  *Source:* vendored `include/pros/motors.h:52` (`motor_move`) and `:253-254`
  (`motor_move_voltage`): *"This function will not respect brake modes, and simply sets the voltage
  to the desired value"*; `motors.h:82-86` and `motors.hpp:260-284` (`brake()`); the mode enum's
  own words, every one about a motor that is STOPPED (`motors.h:860-868`). In the tree:
  `include/shulib/hal/pros/motor.hpp` (THE STOP; `stopAtZero()`), which sends a zero under a
  COMMANDED Brake/Hold as `brake()`; the shim's control state (`test/pros_shim/pros/motors.hpp`);
  the comments that now say so in `hal/motor.hpp`, `hal/mechanism.hpp`,
  `manipulation/mechanism_op.hpp`, `motion/motion.hpp`, `sim/drive_plant.hpp` (each tagged
  `A4 register HA-142`); host-tested by `test/pros_motor_adapter_test.cpp` (DEFECTS2 a1–a11, a9b
  included) and `test/brake_stop_stack_test.cpp` (s1–s8) — which test the adapter against this
  belief and cannot test the belief. The same belief is why the adapter re-sends a mode write the
  port refused before its next `brake()`: `brake()` stops under the REGISTER, not under what the
  program meant (a9b).
  *Confidence:* **reasoned** — the vendored text is explicit, and says it twice. Nothing else in
  PROS's headers contradicts it.
  *Settle (R3, robot two — the first look; T3's claw station — the loaded one):* compare two stops
  from the same speed WHILE ENABLED — a stick release under the ruled Coast, and a library Brake
  stop (COMP's first autonomous `chassis.brake()`, or a driver-loop build with
  `DriverLoopConfig::releaseBrakeMode = Brake`): the Brake stop should be visibly shorter. Before
  DEFECTS2 the two were the same 0 V. **A DISABLE is a confounded test:** the loop's disable path
  issues its Brake stop only once the brain already reports disabled, and whether VEXos is by then
  cutting motor output is HA-145's open question — so what a disable shows is that entry as much
  as this one. For the hold: T3's claw
  pivot declared Hold, released with a load — it stays; turned by hand, it pushes back.
  *Blast radius if wrong:* if `move_voltage(0)` DID honour the mode, DEFECTS2 changed the wire call
  and not the behaviour — harmless. If `brake()` does NOT apply the mode, every Brake/Hold stop stays
  limp: a claw drops, a cancel coasts — the pre-DEFECTS2 robot, no worse. **Contained:** one
  function (`ProsMotor::stopAtZero()`), below every wrapper.

- [ ] **HA-143 — what a V5 motor PHYSICALLY does at `move_voltage(0)`: believed to free-wheel.**
  *Claim:* at `move_voltage(0)` the shaft is undriven and turns freely except for friction — the
  "coast" `hal::BrakeMode::Coast` promises and `IMotor`'s contract describes. False if a 0 V command
  short-circuits the windings (a dynamic brake), or holds.
  *Source:* `include/shulib/hal/motor.hpp` (the `BrakeMode::Coast` doc, tagged `A4 register
  HA-143`); `include/shulib/hal/pros/motor.hpp` (THE STOP: Coast, and every zero under no commanded
  mode, stays `move_voltage(0)` — exactly the pre-DEFECTS2 wire traffic, so Calibrate, shulib Drive
  and the tester's DRIVE station are unchanged); the plant's coast-follow (HA-137) and the
  calibrator's one-sided pulses assume it.
  *Confidence:* **invented — UNKNOWN.** PROS says only that the voltage "simply" is set; it does not
  say what an H-bridge at 0 V does. Two QUALITATIVE observations point toward a coast and neither is
  a measurement: the R3d Part 2F investigation of Calibrate's first night concluded robot two's
  idle side coasts during a one-sided pulse (the development log; HA-137 carries the model it led
  to), and the team lead describes the 2026-09-25 Teleop drive as coasting on stick release (the
  ruling that keeps it).
  *Settle (R3/R4):* robot two at a known speed, WHILE ENABLED, sticks released (0 V under Coast)
  versus an enabled Brake stop (`brake()` under Brake, HA-142 — COMP's first autonomous
  `chassis.brake()`, or a driver-loop build with `DriverLoopConfig::releaseBrakeMode = Brake`): the
  roll distances should differ clearly. Not against a DISABLE — HA-142's settle step says why that
  comparison is confounded (HA-145). And by hand, a motor at 0 V turns as easily with its register
  at hold as at coast (which also checks HA-142's first half).
  *Blast radius if wrong (0 V brakes):* the "coast" in Calibrate's pulses and the plant's κ is
  really a brake — the calibration's side test is modelled with the wrong sign of error — and the
  ruled Teleop release feel is a braking one. Nothing DEFECTS2 did makes it worse: the Coast path is
  byte-for-byte the wire traffic it was. **Contained:** it is a fact about the motor; the only code
  it steers is the plant's coast-follow model.

- [ ] **HA-144 — a REPEATED `brake()` under Hold re-captures the held position — and so, perhaps,
  does a `set_brake_mode()` of the SAME mode written to a motor already held.**
  *Claim:* each `brake()` under Hold re-targets the hold at the shaft's CURRENT position, so one
  issued every 10 ms tick under a load that sags between ticks lets the shaft creep, never springing
  back. False if a repeated `brake()` keeps the first target. *Second half (added in DEFECTS2's
  review):* whether a `set_brake_mode(hold)` written to a motor ALREADY stopped under Hold does the
  same. The adapter keeps sending that write every tick wherever a caller re-states an unchanged
  mode over a stop: `DriveBrake` (Brake, every tick since C1 — Brake holds no position, so only
  Hold is at stake) and the driver loop's release re-assert when `releaseBrakeMode` is Brake or
  Hold (no program does; the ruled Coast never stops). False if a same-mode write leaves the stop
  alone — which is what the shim assumes and cannot test.
  *Source:* `include/shulib/hal/pros/motor.hpp` (THE STOP, and `stopAtZero()`: the adapter issues
  `brake()` only on the TRANSITION into a stop — a zero repeated under the same mode sends nothing
  more; tagged `A4 register HA-144`); `include/shulib/hal/motor.hpp` (`BrakeMode::Hold`); pinned by
  `pros_motor_adapter_test.cpp` a8 and `brake_stop_stack_test.cpp` s7 (one `brake()` across ten
  ticks of the DriveBrake pattern / five released driver-loop ticks — mutation-proven). Both of those
  also send the same-mode `set_brake_mode()` every tick (a8 counts ten); the shim's header lists it
  among what it cannot model (`test/pros_shim/pros/motors.hpp`).
  *Confidence:* **invented.** PROS says only that Hold "may behave differently than calling
  motor_move_absolute(port, 0)", and nothing about a mode write to a stopped motor. The adapter is
  built so that the first half being TRUE costs nothing; for the second half it is NOT — the write
  is still sent. Not suppressed in the adapter (skip a same-mode write while stopped under it):
  that drops the per-tick re-write which heals a register changed underneath (a second adapter on
  the port, a replugged motor — HA-98), changes `DriveBrake`'s wire traffic from what it has been
  since C1, and today guards nothing a program does — the ruled release is Coast, T3's claws take
  their safe state once, and `DriveBrake` stops under Brake.
  *Settle (T3's claw station):* a loaded claw held by one `brake()` versus one re-issued every tick
  (a test build) — the sag over 30 s; then the same claw held by one `brake()` with a
  `set_brake_mode(hold)` written every tick (a test build) — the same sag test. **The opposite risk
  is the real one to look for:** if a hold LAPSES without a fresh command (a firmware timeout no PROS
  text mentions), issuing once is wrong — the claw would hold, then let go. Watch a held claw for a
  full minute.
  *Blast radius if wrong (it lapses):* a held claw lets go after some time with no command; the fix
  would be a periodic re-issue, in the same one function. *If the same-mode write re-targets:* only
  a Hold that a caller re-states every tick creeps — none today; the fix is the suppression above,
  in `setBrakeMode()`. **Contained:** `stopAtZero()` / `setBrakeMode()`.

- [ ] **HA-145 — Hold (or Brake) while VEXos has the robot DISABLED, and after re-enable.**
  *Claim:* UNKNOWN in both halves — whether a motor stopped under Hold keeps holding while the
  brain is disabled (between autonomous and driver control, after the match), and what state it is
  in when re-enabled. The belief this register can state: VEXos cuts motor output while disabled,
  so a hold does NOT survive it.
  *Source:* `include/shulib/hal/pros/motor.hpp` (THE STOP: the adapter's stop record is per object
  and cannot see a disable — after re-enable, a zero repeated under the same mode sends nothing new
  until the command leaves zero or the mode changes; tagged `A4 register HA-145`). The driver loop
  (`include/shulib/teleop/teleop_loop.hpp`) cancels into Brake on a disable and re-asserts its
  release mode every tick; neither re-issues a `brake()` for an unchanged mode. The loop header's
  step 1, which says the disable stop lands before VEXos cuts the motors, points here (tagged
  `A4 register HA-145`).
  *Confidence:* **invented — UNKNOWN.** No PROS or VEX text in the tree covers it.
  *Settle (T3's claw station, then COMP):* stop a loaded claw under Hold, disable (the controller's
  Timed Run ending), feel the shaft; re-enable WITHOUT commanding it and feel again. The same
  question covers the driver loop's disable stop: its Brake `brake()` is issued once the brain
  already reports disabled (the loop's header records, from R3b Part 0b, that this lands before
  VEXos cuts the motors itself — how long that window is, and whether a `brake()` issued in it
  outlasts the cut, is this entry).
  *Blast radius if wrong:* a claw holding a game piece at the end of autonomous drops it at the
  disable; a re-enabled claw stays limp until its first command. The re-arm would be one explicit
  "re-issue the stop" at the start of each competition phase — not built, because nothing yet says
  it is needed. **Contained:** the adapter plus one call site per phase.

- [ ] **HA-146 — leaving the stop: after `brake()`, the next `move_voltage(v ≠ 0)` drives the motor
  at once; a `set_brake_mode()` repeated every tick changes nothing but a RUNNING motor's register.**
  *Claim:* the velocity-zero stop `brake()` puts the motor in is a control mode the next
  `move_voltage()` REPLACES — not a latch — so a held claw moves the tick it is commanded, and a
  cancelled drive responds to the first stick after it. And a `set_brake_mode()` sent while the
  mode is unchanged (the driver loop's release re-assert, every tick since DEFECTS2; `DriveBrake`'s,
  every tick since C1) neither jolts nor resets a RUNNING motor. (The same write to a motor STOPPED
  under Hold is HA-144's second half, not this entry.)
  *Source:* vendored `motors.h:253-254` (`move_voltage` "simply sets the voltage") and
  `motors.hpp:260-284` (`brake()` is a velocity command); `include/shulib/hal/pros/motor.hpp`
  (`setVoltage()` clears the stop record on any non-zero command; tagged `A4 register HA-146`);
  `include/shulib/teleop/teleop_loop.hpp` (step 6b); pinned on the host by
  `pros_motor_adapter_test.cpp` a4 — against the shim, which leaves a stop at once.
  *Confidence:* **reasoned** — PROS's movement calls each set a new target; nothing says `brake()`
  is sticky.
  *Settle (R3 / T3's claw station):* hold a claw, then command it open — it moves on the first
  tick; on robot two, disable, re-enable, and drive — the first stick moves the robot.
  *Blast radius if wrong:* a held mechanism that ignores its next command (a claw that will not
  open); a drive sluggish after every library stop. **Contained:** `ProsMotor::setVoltage()`.

### T3's additions — robot two's claw pivots and its pneumatic claw (2026-09-26)

**T3 (the claw pivots, 2026-09-26) added HA-153 to HA-156**, its addendum HA-157 to HA-159 (the
pneumatic claw, below), and its merge HA-147 (the controller belief the claw's toggle rests on,
first below). HA-142 to HA-146 are DEFECTS2's (above); HA-148 to HA-152 were reserved for T3 and
are unused. HA-153 to HA-156 are beliefs about motors NOBODY HAS SEEN YET: the team lead reported
the two claw pivots on 2026-09-26, no census or capture has shown a motor on port 6 or 7, and
nothing has commanded either on a robot. `src/robots/mechanism_table.hpp` declares them;
`src/programs/calibrate_program.cpp` acts on the declaration (it never makes 6 or 7 a drive candidate);
"shulib Teleop" builds and drives them since T3's Teleop wiring (its Part 2, the same day) —
host- and build-gated, never run. Worksheet Station J is their first run.

- [ ] **HA-147 — a V5 controller's connected flag and its button levels change together.**
  *Claim:* on the first tick `is_connected()` reads true after a disconnect, `get_digital()`
  already returns the button's LIVE level — never a stale one from before the drop — and a
  button's level does not bounce across two 10 ms samples on a press or a release. False if the
  flag can come back a tick (or more) before the button data does, or if a press or release reads
  down-up-down across consecutive ticks.
  *Source:* `include/shulib/teleop/mechanism_binding.hpp` (ToggleBinding's A PRESS MUST BE SEEN
  rule, tagged `A4 register HA-147`: the FIRST connected tick's level is taken as live);
  `test/toggle_binding_test.cpp` T4 ("WHAT THE GUARANTEE RESTS ON") pins the boundary in the
  direction of the hazard — a stale "up" on the first reconnected tick, then the live "down", FLIPS
  the claw. HA-103 claims only that a disconnected controller reads 0 and that `is_connected()` is
  the only validity signal; nothing claimed the two change together. Found by T3's B2 review and
  registered at the merge.
  *Confidence:* **invented** — unmeasured; nothing vendored says either way.
  *Settle (worksheet Station J.9.5):* open the claw, hold A down, pull the controller's cable,
  plug it back in STILL holding A, keep holding for two seconds, then let go. Nothing may move
  until A is released and pressed again; then one press closes it. Record whether the claw moved
  while A was held across the reconnect.
  *Blast radius if wrong:* a phantom flip of the pneumatic claw when the driver holds A across a
  controller reconnect (or on a bouncing release) — an open flip drops what the claw holds. Until
  it is measured the driver's rule is: after a controller drop, let go of A before pressing it
  again (the driver-control cookbook says so). **Contained:** `ToggleBinding::tick()`; the
  mitigation (a reconnect settle window) is flagged for the team lead's ruling after J.9.5, not
  built.

- [ ] **HA-153 — robot two's claw pivots are RED motors on ports 6 and 7, and a positive command
  moves each the way the team lead calls "forward".**
  *Claim:* robot two carries two SEPARATE claw-pivot motors, each a V5 smart motor with a RED
  (100 rpm) cartridge: one on smart port **6** (forward while R1 is held, back while R2 is), one on
  port **7** (L1 / L2). A POSITIVE voltage — the table's `+6` / `+7`, which PROS does not reverse —
  moves each pivot the way R1 (port 6) and L1 (port 7) are meant to move it. False if a census shows
  no motor, or another kind of device, on 6 or 7; if either cartridge insert is not red; or if R1 /
  L1 moves its claw the "back" way.
  *Source:* `src/robots/mechanism_table.hpp` (`kRobotTwoMechanisms`, tagged `A4 register HA-153`): the
  ports, the cartridge and the `+` signs; `src/programs/calibrate_program.cpp` drops 6 and 7 from its drive
  candidates on this belief, and since T3 Part 2 "shulib Teleop" builds both pivots from it
  (`src/common/boot_source.hpp`), names them as claw pivots in its port check, and binds R1/R2 and L1/L2
  to them (`src/programs/teleop_claws.hpp`). `test/mechanism_table_test.cpp` pins the table's contents — which
  tests the TYPING against the report, never the report.
  *Confidence:* **reported** by the team lead, 2026-09-26 — not measured. No capture has seen a
  motor on 6 or 7: the 2026-09-24 Calibrate census lists motors on 11–18 and 20 only, and the
  2026-09-25 Teleop capture's port check names only port 14 as a motor outside its record. The
  DIRECTION is not even reported: "forward" is the name of a button, not a measured rotation, and
  the `+` signs are the table's unmeasured starting point.
  *Settle (worksheet Station J):* Bench Tests' DEVICE CENSUS shows MOTOR on 6 and 7; the insert
  colour read off each motor; Calibrate prints `port 6: claw pivot (declared) -- not a candidate,
  not touched` and the same for 7; J's direction check — hold R1, which way does the port-6 claw
  go; hold L1, the port-7 claw.
  *Blast radius if wrong:* a wrong PORT leaves that claw ABSENT in Teleop (the provider refuses a
  port with no motor on it, by name, and the drive still runs) — or, if a DRIVE motor is on 6 or
  7, Calibrate drops that drive motor (the side calibrates one short) and R1 spins a wheel. (A
  calibration RECORD that lists 6 or 7 as a drive port — one taken before Calibrate skipped the
  claws — makes Teleop refuse at boot, "recalibrate", since T3 Part 2.) A wrong CARTRIDGE writes the
  wrong gearset: the volts still reach the motor and the claw still moves; only the motor's speed
  and position scaling — unused by a hold-to-run button, needed by a future angle preset — would be
  off. A wrong SIGN moves the claw opposite to what the driver expects, and is fixed by flipping
  that entry's sign in the table (never by swapping its buttons). **Contained:** one table.

- [ ] **HA-154 — a claw pivot held at 12 V against its stop heats, at the motor's default current
  limit, and shulib does not watch it.**
  *Claim:* a claw pivot commanded 12 V while it is pressed against its mechanical stop (the driver
  keeps holding R1 after the claw has closed) draws current up to the V5 motor's current limit —
  PROS's documented DEFAULT, 2500 mA, which nothing in shulib changes — for as long as the button
  is held, and heats; as it heats, the motor's own firmware derates it (HA-44's shape) until the
  claw weakens. A pivot in Hold against a load also draws current, and heats more slowly. **How
  long any of this takes is not known.** False if a stalled pivot barely warms in a minute at 12 V
  (then nothing is needed), or if it reaches the derating temperature within seconds (then a stall
  cut-off is not optional).
  *Source:* `src/robots/mechanism_table.hpp` (`volts = 12.0`, tagged `A4 register HA-154`) — the ordered
  full power passes straight through while a button is held; a stall cut-off is OUT of T3 by the
  brief. The vendored `include/pros/motors.h` (`motor_get_current_limit`: "The default value is
  2500 mA."). `include/shulib/diag/health_monitor.hpp` raises `MOTOR_OVER_TEMP` from the hottest
  motor "over the drive" — a claw motor is not in it.
  *Confidence:* reasoned (the limit is the vendored default; a stalled DC motor heats) — every
  TIMING is **invented**, and none is written here on purpose: nothing may use one until Station J
  measures it.
  *Settle (worksheet Station J):* close one claw against its stop and hold the button for a timed
  interval, reading the motor's temperature before and after (Teleop's claw row, `CLAW 6 HOLD
  34C | ...`, since T3 Part 2); stop well below HA-44's 55 °C first step. The row shows the
  motor's LAST GOOD reading — it freezes, unmarked, if the motor stops answering, and reads 20 °C
  if it never answered (`hal/pros/motor.hpp`'s screen; the motor seam has no validity channel) —
  so a reading that does not move while the pivot is held means a cable check first.
  *Blast radius if wrong:* a claw that goes weak mid-match as its motor derates, or a motor that
  cuts itself out hot — and shulib would not say why: `MOTOR_OVER_TEMP` watches the drive only.
  The driver sees a claw that stopped gripping; Teleop's claw row SHOWS each pivot's temperature
  (T3 Part 2; its last good reading — see the settle text), but nothing acts on it. **Contained:** one mechanism; the drive is not
  affected.

- [ ] **HA-155 — a cable swapped between a claw pivot and a drive motor is invisible to the
  registry.**
  *Claim:* the brain's device registry reports a port's device KIND — MOTOR — never WHICH motor,
  so a claw-pivot cable and a drive-motor cable swapped with each other at the brain look identical
  to every check shulib runs before power: the census, Calibrate's declared-port drop, Teleop's
  port check against the record, the provider's kind check. False only if the registry could tell
  one motor from another, which its API (HA-138, HA-141) does not offer.
  *Source:* `src/programs/calibrate_program.cpp` (the declared-port drop, tagged `A4 register HA-155`): it
  drops PORTS, trusting that the cable in port 6 is the claw's; `src/robots/mechanism_table.hpp`. Teleop's
  boot check already says the same of two DRIVE cables swapped with each other ("NOT CHECKED: two
  drive cables swapped WITH EACH OTHER look identical here").
  *Confidence:* reasoned (what the registry reports).
  *Settle:* not by the registry, ever. By a person: label both ends of both claw cables; Station
  J's direction check is the first run that would show a swap (R1 must move the CLAW, not a wheel).
  *Blast radius if a swap happens:* Calibrate skips the drive motor now on 6 or 7 (that side
  calibrates one short) and takes the claw motor now on a drive port as a candidate — geared BLUE
  and set to Coast, so a loaded claw drops, and if the push swings it far enough to count as moved,
  it is taken for a drive motor and PULSED. On a record made before the swap, Teleop drives the claw
  motor as a member of a drive side — the coupled-side monitor catches that half
  (`MOTOR_GROUP_DISAGREE`: a claw at its stop cannot follow its side) — while R1 drives a wheel at
  12 V, which only a person sees. **Contained:** by labels and Station J; no software can contain
  it.

- [ ] **HA-156 — a claw pivot released in Hold keeps its angle under its load.**
  *Claim:* once DEFECTS2 has made a program-commanded Hold at 0 V reach the motor (its own
  register entries carry that part), a claw pivot released at an angle HOLDS that angle under its
  own weight and a game piece: the sag over 30 s is small enough that the claw still does its job.
  "Holds" is claimed nowhere — in the code, the worksheet or a commit — until Station J logs it.
  False if the claw droops visibly within 30 s, drops its piece, or the motor heats holding it
  (HA-154).
  *Source:* `src/robots/mechanism_table.hpp` (`safeMode = Hold`, tagged `A4 register HA-156`). The T3
  brief specifies that the library's hold-to-run binding applies the safe state ONCE on release
  (never re-applied per tick, which could creep the hold point); that binding is
  `teleop::MechanismBinding` (T3 Part 1, `include/shulib/teleop/mechanism_binding.hpp`), and
  "shulib Teleop" binds both pivots with it since T3 Part 2 — a release is ONE `brake()` under Hold
  at the PROS shim (`test/teleop_claws_test.cpp` test 1), which says nothing about the load.
  *Confidence:* **invented** — the load, the hold's stiffness and the hold current are all
  unmeasured; the V5's Hold is a position hold whose strength under a claw's load nobody here has
  measured.
  *Settle (worksheet Station J):* move each pivot to a mid angle, loaded with a game piece if the
  claw carries one, release, and measure the sag at 0, 10 and 30 s against a mark; read the
  temperature after.
  *Blast radius if wrong:* a claw that sags or drops its piece after release, so the driver has to
  keep holding the button — and HA-154's heating then applies. **Contained:** one mechanism; the
  fix is a MEASURED one (a holding voltage or a hold stiffness), never a guess.

**T3's addendum (E2, the pneumatic claw, 2026-09-26) added HA-159** (its library half, B2, owns
HA-157 and HA-158). The team lead reported a PNEUMATIC claw the same night: its solenoid on the
brain's 3-wire port A, toggled by the controller's A button, closed at program start, keeping its
state when the robot is disabled. `src/robots/mechanism_table.hpp` declares it; "shulib Teleop" drives
its line since T3 Part 2 (made at the START level when the Robot is built, then A's toggle) — not
yet on the robot — and shulib's census cannot show it (it reads the 21 smart ports). Worksheet
Station J.9 is its first run.

**T3's addendum B2 (2026-09-26) added HA-157 and HA-158.** Both are about the brain's own eight
3-wire ports, which the library first USES here — for robot two's pneumatic claw on port A, toggled
by the driver's A button. Neither is about the claw itself (its port, its valve and which level
opens it are the program half's entry, from the team lead's report).

- [ ] **HA-157 — a 3-wire output keeps its level while the brain is disabled, and takes a level
  commanded while it is disabled.**
  *Claim:* two halves. (1) While the V5 brain is DISABLED — the gap between autonomous and driver
  control, a field disable, a competition-switch disable — a 3-wire digital output stays at the
  level the program last commanded, and is still at that level when the brain is enabled again.
  (2) A level the program commands WHILE the brain is disabled reaches the valve — at once, or at
  the latest when the brain is enabled. The second half is not a corner case: `robot::Robot` makes
  each pneumatic line at the config's `pneumaticStart` the moment it is built, and a program builds
  its Robot in `initialize()` ("shulib Teleop" does, since G1 Part 2b: its `teleopInitialize()`,
  commit `7f9b9a1`), which on a field — and whenever the program is started from the
  controller's own menu (HA-128, measured: the brain then starts DISABLED) — runs while the brain
  is disabled. False if VEXos drives the brain's 3-wire outputs to any level of its own while
  disabled (as it cuts the motors) or on the way back to enabled, or drops (or holds back past
  the enable) a 3-wire write made while disabled.
  *Source:* `include/shulib/teleop/mechanism_binding.hpp` (ToggleBinding, the DISCONNECT/DISABLE
  rule, tagged `A4 register HA-157`) and `include/shulib/teleop/teleop_loop.hpp` (the disable path
  leaves every toggle untouched): the library commands NOTHING to a pneumatic claw on a disable, on
  the team lead's rule that the claw keeps its state. `include/shulib/robot/robot.hpp` (the
  MECHANISMS section and `buildPneumatic()`, tagged `A4 register HA-157`): the start level is
  written when the Robot is built, whatever the brain's state. `test/toggle_binding_test.cpp` TL3
  pins that the LIBRARY commands nothing on a disable, and `test/pros_device_provider_test.cpp` 1m
  that construction writes the start level once; no host test can pin what the brain does with
  either.
  *Confidence:* **UNKNOWN.** The vendored PROS headers document the 3-wire output's constructor
  and `set_value()` only (vendored `adi.hpp` ~:530-615) and say nothing about a disabled brain.
  VEXos is known to stop the MOTORS while disabled; whether it treats a 3-wire output the same
  way — its level, or a write to it — is exactly the open question. No solenoid has ever been on
  either robot.
  *Settle:* (1) at Station J (robot two's claw — J.9.6): close the claw, disable (the field
  control / Timed Run disable, or a competition switch), and watch it; then open it, disable, and
  watch again; re-enable and press A once. Record whether the claw moved on the disable, whether it
  moved on the re-enable, and whether that first press did the visible thing. (2) at COMP's
  station (the competition program's bench run), NOT Station J — Station J never starts a program
  from the controller's menu (its DO NOT list; HA-128), and COMP's station does so on purpose,
  through the controller's Timed Run, because the team has no competition switch (the coordinator's
  placement at T3's merge, 2026-09-26): leave the claw at the OTHER level from the start level (the
  run before ends with it there), start the program that way — the brain then comes up DISABLED,
  so `initialize()` builds the Robot and writes the start level while disabled — and watch: does
  the claw move at the start (the write reached the valve while disabled), only when the robot is
  enabled, only when driver control begins (the toggle's own initial level), or never? A bench
  start from the laptop runs enabled and cannot answer (2).
  *Blast radius if wrong:* (1) the claw moves by ITSELF on every disable — if that is open, it
  drops what it holds — and because the adapter still reports the level it last commanded, the
  first press after the re-enable commands the level the line already has: the driver sees a
  press that does nothing and has to press again. (2) The claw's FIELD-START state rests on it:
  if a write made while disabled is dropped, the claw sits wherever its valve rests from program
  start through autonomous — a preload it should grip is not gripped — until driver control
  starts and the toggle sends its initial level (which the program states equal to the start
  level); if the write is held back to the enable, the claw moves at the enable, on the field.
  **Contained:** one rule in the toggle and one line in the loop's disable path for (1) — the fix
  (re-assert the commanded level when driver control re-enters) is a small, testable change
  there; for (2), the fix is to command the start level again at the first ENABLED moment (the
  top of `autonomous()` and of `opcontrol()`), a program-side change.

- [ ] **HA-158 — nothing the program can read sees a solenoid on a brain 3-wire port.**
  *Claim:* the brain's device registry (`registry_get_plugged_type()`) is indexed by SMART port —
  indices 0..20 (HA-138) — and the brain's own eight 3-wire ports are served by its internal ADI
  (PROS's `INTERNAL_ADI_PORT`, 22, vendored `adi.h:136`), outside that range; a solenoid valve on
  one of them is a passive load with no digital identity, and a digital output has no read-back
  (hal/digital_out.hpp: no feedback, no validity). So a connected valve and an empty port are
  indistinguishable to the program. False if some API — a registry kind, an ADI read, a current
  sense — reports what is plugged into a 3-wire port.
  *Source:* `include/shulib/hal/device_provider.hpp` (the header's "A 3-WIRE LINE IS THE
  EXCEPTION"), `include/shulib/hal/pros/device_provider.hpp` (`digitalOut()` asks no registry,
  tagged `A4 register HA-158`), `include/shulib/hal/fake/fake_device_provider.hpp`;
  `test/pros_device_provider_test.cpp` 1l (the line is made with the registry reporting NONE on
  every port, and the registry is never asked).
  *Confidence:* **reasoned** — from the registry's documented index range, the internal-ADI port
  number, and the 3-wire output's documented API, which has no read of anything but the level the
  program set.
  *Settle (Station J — a step the station does not have yet; the coordinator adds it to J.9, whose
  J.9.0 only LOOKS at the valve's cable and whose J.9.5 pulls the CONTROLLER's):* with no hand in
  the claw, pull the valve's 3-wire cable out of port A and start slot 2 — the program must build
  the claw exactly as it does with it plugged in (the same boot line for the claw, no refusal, no
  difference on the screen) — and look at the brain's own Devices screen for any entry for the
  valve, plugged and unplugged. Confirming the belief means seeing NO difference. Plug the cable
  back in before anything else in J.9.
  *Blast radius if wrong:* in the safe direction — if something CAN see a solenoid, the provider
  could refuse an empty 3-wire port and does not, which is a missed check, not a wrong action. If
  RIGHT (the expected case), an unplugged or mis-plugged valve is invisible to every check in the
  library: the claw simply does not move, and the driver's eyes are the only tell — which is why
  the station tests it by pulling the cable. **Contained:** one provider member.

- [ ] **HA-159 — robot two's claw solenoid is on 3-wire port A; single or double, and which level
  opens the claw, are NOT KNOWN.**
  *Claim:* robot two's pneumatic claw is moved by a solenoid valve wired to the brain's 3-wire
  port **A** (line 1) through ONE cable, and commanding that line to the level `kClawOpenCommand`
  — `true` (HIGH), the addendum's starting guess, "ON = open" — OPENS the claw, while the other
  level CLOSES it. False if the valve's cable is in another lettered port; if the valve has TWO
  cables into the brain (a double solenoid, whose second line nobody has declared); or if HIGH
  closes the claw.
  *Source:* `src/robots/mechanism_table.hpp` (`kRobotTwoMechanisms.pneumatics[0]` and
  `kClawOpenCommand`, tagged `A4 register HA-159`): line 1, the polarity as ONE named constant
  marked UNVERIFIED, the start and safe positions CLOSED. `test/mechanism_table_test.cpp` pins the
  entry against the report AND the polarity's value (`true`), as it pins the pivots' signs: the
  flip Station J.9 may call for is ONE line of code (the constant; every level follows it), and it
  is a measurement — so the constant, that pin, this entry (its claim and its index row) and
  Station J.9's quote change together, in one change.
  *Confidence:* **reported** by the team lead, 2026-09-26 — the PORT only ("port A"). Whether the
  valve is single or double, and the polarity, are **unknown**: the team lead said so, and `true`
  is a guess recorded as one. Nothing has seen the claw move under a program: the one program that
  drives its line ("shulib Teleop", since T3 Part 2) has not run on the robot, shulib's census
  reads the 21 smart ports only, and nothing in shulib
  can ask the brain what is on a 3-wire line (the library half's register entries carry what the
  brain can and cannot report there).
  *Settle (worksheet Station J.9):* J.9.0 — look at the valve: one cable or two into the brain,
  and which letters (one cable NOT in A stops the station: the port is wrong, not the polarity);
  J.9.2 — where the claw is after slot 2 starts; J.9.3 — the FIRST press of A decides, with one
  cable in A: CLOSED after the start and the press OPENS it ⇒ the polarity is right; OPEN after
  the start and the press CLOSES it ⇒ it is backwards (flip `kClawOpenCommand`, the only code
  edit, and record it — the test's pin, this entry, J.9); the press moves NOTHING ⇒ no answer
  about the polarity (the air, the cable, the port, the valve). Then A opens it, A again closes
  it.
  *Blast radius if wrong:* a wrong POLARITY opens the claw at program start — constructing the
  3-wire output drives the line at once (HA-119) — so a preloaded piece drops as the program
  starts; every press still flips the claw, so the driver sees the button "backwards"; and a
  library-forced safe state (COMP's end-of-run guard, later) would OPEN the claw instead of keeping
  what it holds. Fixed by flipping ONE constant. A wrong PORT leaves the claw unmoved by A while
  the program drives a line with nothing (or something else) on it — and no software can see
  that: a 3-wire output has no feedback channel (`hal/digital_out.hpp`). A valve with TWO cables
  gets only the one in port A driven, never the other; what the claw does then is not known, and
  J.9.0's look is what catches it before power. **Contained:** one mechanism; the drive is not
  affected.

**COMP-a (2026-09-26) added HA-161.** The competition program (COMP-b) runs its autonomous routine on
a task it creates itself, and ends that routine itself — through `RunGuard::requestAbandon()` — when
the field disables the robot or leaves autonomous (the team lead's ruling of 2026-09-24: "our own
auton task"). That design, and code already in the tree, rest on a belief about PROS that no
vendored header states. It was registered at COMP-a because COMP-a built the library half of the
design; COMP-b's practice match settles it. (The number was COMP-a's reservation, taken while T3 and DIAG1
were built in parallel; both have since landed, with the numbers below it.)

- [ ] **HA-161 — PROS deletes its competition tasks on a mode change, and leaves the program's own
  tasks alone.**
  *Claim:* (a) PROS runs each of `competition_initialize()`, `disabled()`, `autonomous()` and
  `opcontrol()` on a task of its own and DELETES that task when the competition mode changes (the
  field disables the robot, or autonomous ends and driver control begins) — mid-statement, with no
  unwinding: a deleted task runs no destructor, releases nothing and finishes no loop. (b) A task
  the PROGRAM created (`pros::Task`) is not deleted on a mode change — it keeps running, and must
  end its own work.
  *Source:* half (a) is written, as a belief, in code comments only:
  `include/shulib/teleop/teleop_loop.hpp` (step 1: "PROS deletes the opcontrol task on a field
  disable, and a deleted task runs no destructor", so the driver loop polls the disable and
  returns, braking the drive before the task is torn down), `src/main.cpp` (the X-drive loop:
  "PROS deletes the task on disable and a deleted task runs no destructor"),
  `src/programs/drive_program.cpp` ("A FIELD DISABLE is NOT a C++ exit — PROS deletes the opcontrol task,
  and a deleted task runs no destructors … Stated so nobody believes the guard covers a kill")
  and `src/programs/calibrate_program.cpp` (the same belief, nearly word for word, for its all-stop guard), and
  `include/shulib/robot/robot.hpp` ("PROS calls opcontrol() again after every disable"). Two
  rules rest on half (a) as well: the claw bindings take no claim (a claim would survive a
  deleted task), and T3's static-state rule — the teleop program keeps its bindings, its
  mechanisms and their `IMotor*` arrays in its static state, never on the driver task's stack,
  which a deleted task would leave dangling. Half (b) is what COMP's own-task design rests on
  (`competition/match_runner.hpp` runs on whichever task calls it; the program's task and its
  handshake are COMP-b's). The vendored `include/pros/misc.h` (`competition_get_status`,
  `competition_is_disabled`, `competition_is_autonomous`, `competition_is_field`,
  `competition_is_switch`) says nothing about tasks.
  *Confidence:* **reasoned** — both halves are how PROS's competition template is generally
  understood to work, and half (a) is the belief this tree's code comments have carried since R3b
  Part 0b; neither half is stated in the vendored PROS text, and neither has been observed directly
  on this project's brain (R3b Part 0b's "finding" was a correction to a brief's REASONING — a
  deleted FreeRTOS task does not unwind its stack — not an observation of what PROS does).
  *Settle (COMP-b's practice-match station, K):* a program whose `autonomous()` holds an object whose
  destructor logs a line and loops logging "alive N" every 50 ms, and whose own task logs "task
  alive N" likewise; end autonomous (the controller's Timed Run, or a competition switch). (a) is
  confirmed if "alive" stops at the mode change and the destructor's line never appears; (b) if
  "task alive" continues through it.
  *Blast radius if wrong:* if (a) is wrong (PROS does not delete its tasks), an `autonomous()` that
  kept running would command the drive alongside driver control — COMP-b's design does not let it
  (the auton runs on the program's own task, which `opcontrol()` waits for), so the damage is
  contained to the loops that assume the kill. If (b) is wrong (PROS deletes the program's tasks
  too), a routine could be killed mid-motion, leaving the scheduler's active motion on a freed
  stack — the failure the own-task design exists to avoid; the recovery would be a disable-aware
  pacer that ends the routine before the mode change lands, which the abandon latch already is.
  **Contained:** the program's task body and its watch pacer (COMP-b).

**TANK1 (2026-09-27) added TWO — HA-179 and HA-180.** A tank `moveTo` now steers back onto its line
while it drives (the changelog's API 2.3 entry; `motion/tank_steer.hpp`, `motion/move_to_pose.hpp`): the
heading it servoes to is bent toward the target's line, and the yaw the robot makes that the heading loop
did not ask for — a weaker side's pull — is measured and cancelled. Every constant in it was tuned on the
host test plant, whose motors respond instantly and whose tank never skids; these two entries are what
that leaves unproven. The numbers below are host-test-plant measurements of 2026-09-27, re-measured on
2026-09-28 after a review added the bend's lead; **none of it has run on a robot.**

- [ ] **HA-179 — TANK1's steering constants hold on robot two.**
  *Claim:* with a bend of 0.1 rad per inch off the line capped at 15°, steering on where the robot will
  be 0.06 s ahead, a 6 in fade, a 0.1 s yaw observer and a 1.5 rad/s correction limit, robot two's tank
  `moveTo` removes the drift of a side up to about 10 % weaker than the other before arrival, inside the
  unchanged 0.5 in / 0.02 rad arrival rule, without heading oscillation, and without making a leg that
  arrives with the steering off time out with it on — which requires the drivetrain to follow a yaw
  command much faster than 0.1 s (its motor time constant, kA/kV, well under about 0.2 s).
  *Source:* `include/shulib/motion/motion_config.hpp` (`TankSteerConfig`, every field PROVISIONAL (A4:
  HA-179)); `include/shulib/motion/move_to_pose.hpp` (the tank path, labelled HA-179). Host test plant,
  instant motors: the 40 in leg settles with one side up to 25 % weak (1.83–2.57 s, within 0.1 in of its
  line), the acceptance run finishes with either side 9 % weak, and a faced leg removes an offset from its
  line of up to 6 in over 40 in, 8 in over 50 in, about 1 in over 20 in. **The lead** (steering on the
  error 0.06 s ahead): without it the line loop was under-damped — a leg starting 10–15° off its target
  heading, turned toward its line, crossed the line near its target and timed out half an inch beside it
  (178 of the 1 861 legs of a 24 563-leg grid that arrive with steering off), and the lateral reach had
  holes (over 50 in, 4–6 in timed out while 7–8 in arrived). With it: none of them, at plant lags from 0
  to about 0.18 s; every lead from 0.05 to 0.10 s measured did the same, 0.04 s left 4, 0.25 s left 72–102
  under lag. **With a motor lag the controller does not model** (the plant's kA raised, the controller's
  left at 0, as R5a will leave it): at a time constant of about 0.12 s and about 0.18 s the 40 in leg still
  finishes with one side up to 25 % weak (1.37–1.82 s and 2.09–2.42 s; 30 % times out at every lag, as
  without lag), and a 6 in offset over 40 in is still removed (1.45 / 2.39 s); a 10 in one is not (1.6 in
  off the line at 0.12 s). **At about 0.24 s** 24 of the 1 867 grid legs that arrive with steering off
  time out with it on (all 48 in long, starting 10–14° off their heading) — the heading loop's own kP 4 is
  the limit there. Near-target yaw-command sign changes stay small: 0 on every drift case without lag;
  at most 4 per arriving leg at 0.12–0.18 s (7 and 11 on the 12 in offset that times out).
  *Confidence:* **invented** — tuned on the host test plant's placeholder dynamics (the HA-45/HA-50
  co-dependency: wrong kV or kA changes what every one of these numbers costs).
  *Settle:* COMP-t's floor run of the 40 in auton (its Station M.6: the RESULT, END y, how many ticks show
  TSTR, and the tape's sideways offset), and R5, whose characterization records kA (R5a) and so the motor
  time constant. **When that is known, re-run the host probe grids with the measured kA** — the review's
  24 563-leg grid for "no leg that arrived before times out", and the design sweep: the lead's band
  (0.05–0.10 s measured regression-free at 0–0.18 s) was chosen for an unknown lag, and the right
  `leadTime` grows with the robot's real heading lag; that choice is R5's. The same grids re-derive how
  often TSTR lights if R5a moves `maxAngularSpeed` (the flag's floor is 2 % of it).
  *Blast radius if wrong:* too weak — the drift is not removed, and the leg ends as it did before TANK1: an
  honest MOTION_TIMEOUT (bounded, visible, and the tick lines show TSTR to the end). Too strong, or a
  drivetrain too slow for it — heading wobble on steered tank legs, bounded by the 15° cap and the
  1.5 rad/s limit and visible as TSTR; at a stop a weak side's static friction can leave a small commanded
  yaw it does not follow. **While the robot cannot follow a yaw command** (pinned by a defender, wedged on a
  wall, motor lag, both sides saturated) the correction integrates the heading error, up to 1.5 rad/s, and
  unwinds within about 0.1 s once the robot turns again: measured, a 0.5 s pin of the 9 % weak 40 in leg
  still arrives (2.41 s, peak correction 0.62 rad/s) and so does a 1.0 s pin (3.11 s, the correction at its
  1.5 rad/s limit), and a 6 in offset leg pinned during its bend swings to 16.5° off its heading on release
  (19° with 0.18 s of lag) and still arrives (before the lead it timed out). **The cap is also the engage window:** raising
  `maxSteer` to remove larger offsets also steers un-faced legs further off — with no window at all an
  un-faced 40 in leg arrives up to 30° off its nose and times out from 40°, a cliff an author cannot see —
  so keep it well under 30°. **The IMU read:** the observer uses the rate the Localizer read that tick
  (`loc.twist()`), the tick's FIRST `ProsImu::yawRate()` on robot two; on a robot with a GPS or tag
  corrector, until IMU1 lands, the corrector reads first on the ticks it reaches its yaw-rate gate and the
  Localizer's rate is about 0 (or a spike) there — modelled on every other tick: with zeros 15 of 15 probe
  cases still arrive, with zeros and a 3 % ×50 spike 8 of 15. (The Localizer also reads the rate a second
  time on the first tick of every chained motion — IMU1's too; the observer never runs on a motion's first
  tick, so it does not reach TANK1.) **The rate's SIGN:** TANK1's observer is the first control law in the
  library that closes a loop on the yaw rate's sign (before it, the rate fed records, rest checks and
  |ω| gates), so a rate whose sign disagrees with the heading's (HA-04 — `get_gyro_rate().z`'s sign is
  undocumented) makes the observer push the WRONG way, up to its 1.5 rad/s limit: measured with a negated
  rate on the 9 % weak 40 in leg, heading wobble to about 16° and a TIMEOUT, visible as TSTR lit and the
  correction pinned at its limit. Robot two's default rate (differentiated `get_rotation()`) has the
  heading's sign by construction. **Contained:** `maxSteer = 0` restores the pre-TANK1 tank's motion,
  records and log exactly; every number is config.

- [ ] **HA-180 — robot two's sideways error on a straight `moveTo` comes from its heading, not a skid.**
  *Claim:* the drift of a tank leg on robot two is the HEADING being pulled — which the IMU sees and the
  drive encoders integrate into the pose — not the chassis sliding sideways, which
  `TankKinematics::forward()` reports as zero and no sensor on robot two sees.
  *Source:* TANK1 steers on the ESTIMATE's distance off the line; robot two's odometry is its drive
  encoders and IMU (HA-131), with no tracking wheel and no GPS on the bench. On the host test plant the
  drift is all heading by construction (its tank never skids), and a drive-encoder rig wired the way the
  builder wires robot two settles the 5 % and 9 % weak 40 in legs exactly like the tracking-wheel rig.
  *Confidence:* **reasoned** — the 2026-09-25 veer (about 11° over 48 in, open loop) was a heading change,
  read on the IMU; a sideways skid component was never measured.
  *Settle:* COMP-t's Station M.6 — END y by the estimate against the tape's sideways offset from the line,
  three runs; agreement within 0.5 in settles it.
  *Blast radius if wrong:* the screen says FINISHED while the robot is off its line on the tape by the
  skid, and every later leg starts from that error. Not contained by config — only a GPS, a tracking wheel
  or a wall reset sees a skid.

---

## Group R4 — noise, drift, latency, timing, power, traction (characterization)

The numbers behind every hostile model. A3's convention: the failure **shapes** are confident and
test-pinned; the **magnitudes** are these entries. R4 replaces each with a measurement and re-runs
the suite (the models take the measured values by config — zero code motion). None of these are
settleable before hardware, and none block any host chunk.


- [ ] **HA-123 — a single tracking-pod or drive-side travel delta above 36 in in one tick is
  corruption, not motion.**
  *Claim:* `PilonsOdometry` treats |Δtravel| > `maxTickTravel` (default 36 in) from either pod
  in one `update()` as implausible, and `DriveEncoderOdometry` applies the same bound, with the
  same default, to each DRIVE SIDE's travel (since R3b Part 2, 2026-09-14 — its config field
  carried this entry's label from the start, but the entry itself spoke only of tracking wheels
  until DIAG1 extended it, 2026-09-26). The number is a SANITY bound, not a measurement: it is
  chosen to be unreachable by a real drivetrain at any plausible loop rate, so it can only
  catch corruption that is orders of magnitude out.
  *Source:* introduced at DEFECTS1 (item N1) — a tracking pod that is dead or not yet
  enumerated at construction reads 0, so `TrackingWheel` baselines at 0 and, on the tick the
  pod finally answers, differences its TRUE cumulative position against that 0. Measured at
  **28.4 in** of phantom translation for a pod waking at 1000°, and unbounded in general.
  Before this, `PilonsOdometry` gated |Δθ| and never |Δtravel|, so the jump passed in silence.
  The drive-encoder side has the same failure shape where a side is ONE motor: a drive port
  that enumerates late, or whose adapter holds a last-good value and then wakes
  (`localization/drive_encoder_odometry.hpp`). Where a side is a motor GROUP (robot two: four
  motors a side) the odometry reads the members' MEDIAN, so one such member is absorbed and the
  bound never sees it — a drive-side trip there means enough members jumped together to move
  the median (two of four). The group-level detector for one bad member is the coupled-side
  monitor (`MOTOR_GROUP_DISAGREE`), which samples member velocity; whether it catches a member
  whose velocity is held too is not established.
  *Confidence:* **invented.** Nobody has measured a real loop rate under load, and neither class
  holds a clock, so the bound is **dt-BLIND** — the same 36 in is 300 ft/s on a 10 ms tick and
  15 ft/s on a 200 ms one. That is why it is generous rather than tight, and why the gate
  REPORTS rather than withholds: a dt-blind bound that could stop odometry accumulating on a
  slow loop would be a worse failure than the jump it prevents.
  *Settle (R3/R4):* measure the real control-loop period under load, then re-express this as a
  velocity bound (`maxTickTravel = vMax × dt`) using the same clock the loop reads. R4 owns the
  loop-rate measurement; until then this stays a magnitude check. Since DIAG1 a trip names
  itself on the fault line — `implausible odometry tick why=TRAVEL … L=… R=…` (`fwd=`/`lat=` for
  pods) — so any trip seen on a robot says which side or pod, and by how much.
  *Blast radius if wrong (too tight):* legitimate fast motion on a slow loop is flagged
  implausible, which HealthMonitor turns into `ODO_STUCK` — and `ODO_STUCK` is the motion
  scheduler's DEFAULT ABORT fault, so a spurious trip **ENDS the active motion** (an autonomous
  stops where it is). **Corrected 2026-09-26 (DIAG1):** this entry said "noisy, not dangerous,
  because the delta is still integrated". The integration half is true; the "not dangerous"
  half was not, and was never true while `ODO_STUCK` aborted by default. **If too loose:** a
  smaller phantom jump stays invisible, which is the pre-DEFECTS1 behaviour. **Contained:** one
  config field and one comparison per odometry; the POSE is unaffected either way (the delta is
  integrated), the active MOTION is not.
- [ ] **HA-160 — the V5 IMU's heading is continuous tick to tick while it reports ready.**
  *Claim:* while `isReady()` is true, two heading reads taken one control tick (~10 ms) apart
  differ by no more than the robot actually turned in between — a few degrees at most, never
  tens. The library has always rested on this without saying so: heading is IMU-OWNED (the
  pose's heading is set equal to the reading every tick), and each tick's Δθ — the arc
  `arcStep` integrates along, and the drive-encoder heading cross-check — is the difference of
  two consecutive reads.
  *Source:* implicit since M2; registered by DIAG1 (2026-09-26). Consumed by
  `localization/drive_encoder_odometry.hpp` and `localization/pilons_odometry.hpp`. Policed only
  coarsely: the odometry trust gate trips at |Δθ| > π/2 per tick (`maxTickRotation`), and during
  a SCHEDULED motion the scheduler's pose-delta guard raises `IMPLAUSIBLE` above its yaw-rate
  envelope (that guard does not run under driver control) — so under driver control a swing
  smaller than 90° in one tick passes in silence.
  *Confidence:* **invented, and QUESTIONED by the 2026-09-25 robot evidence** (robot two, the
  library's graph under open-loop driver control). In one start, during ~0.35 s of reversing at
  up to ~34 in/s, the heading RECONSTRUCTED from the logged commands swung by up to 37° between two
  consecutive 10 ms records — of the last nine such steps, eight were 16.9° or more, alternating
  in sign, and one was 1.4° — and 37° in 10 ms would be a robot spinning at ~3700°/s; an
  `implausible odometry tick` fault fired on the first tick the command returned to zero. That
  is an INFERENCE, not a reading: the log of that day carried no heading field, so the heading
  was recovered from the field-frame command (assuming the tank's body command had no lateral
  part), and the fault line then named no check. The cause is UNKNOWN — an IMU data glitch, a
  mounting or vibration effect under a hard reverse, or something between the sensor and the
  read are all open.
  *Settle (R3/R4):* record the raw IMU heading every tick (not reconstructed) through hard
  reverses and stops, wheels up and on the floor; any consecutive-read step larger than the
  robot can turn in 10 ms falsifies the claim. Since DIAG1 a step over 90° also names itself on
  the fault line (`why=ROT dth=… h0=… h1=…`).
  *Blast radius if wrong:* the pose's heading jumps with the reading, and every tick's arc is
  integrated along the wrong average heading, so position is corrupted too; a step over 90°
  trips `ODO_STUCK`, which ENDS the active motion under the default fault policy (HA-123's
  corrected blast radius). **Contained:** detection only — the trust gate, the pose-delta guard
  and now the fault line's values. Nothing filters the IMU heading, deliberately: a filter would
  lag every real turn to hide a fault whose cause is not yet known.
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
  `include/shulib/localization/pilons_odometry.hpp:33–39` (gate note; re-pointed at DIAG1, whose
  header lines moved it);
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
  `include/shulib/diag/health_monitor.hpp:96–99` (`brownoutVolts`/`brownoutRecoverVolts`; line numbers
  re-pointed 2026-09-26 at DIAG1, whose header note moved them — they already read 66–69 against 71–74) — two
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
  `health_monitor.hpp:101` (`maxMotorTempC = 55` — deliberately equal to the first step; re-pointed
  at DIAG1 from a stale `:70–71`).
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
  *Confidence:* **invented** — robot two has carried an AI Vision camera on port 3 since
  2026-09-24, but nobody has measured how it is mounted, and no library adapter reads it yet (R2).
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
  *Amended DEFECTS2 (2026-09-26) — the premise was false at the wire:* "the declaration reach[es]
  the motor provably" was true only down to `IMotor::setBrakeMode()` and the device's mode
  REGISTER. The stop itself went out as `move_voltage(0)`, which PROS documents as ignoring the
  mode (HA-142), so on a V5 a declared Hold went LIMP — the question this entry asks (does Hold win
  against the load?) could not even have been posed on hardware before DEFECTS2. Since then
  `ProsMotor` sends that zero as `brake()`, pinned at the bottom of `MotorMechanism` and a
  mechanism operation over real `ProsMotor`s (`test/brake_stop_stack_test.cpp` s1, mutation-proven).
  Nothing about the holding TORQUE changed; that is still this entry, and still invented. The first
  loaded hold will be T3's claw pivots (ports 6 and 7), and two neighbours ride along: HA-144
  (whether a repeated stop re-captures the held position) and HA-145 (a hold across a disable).
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
  *Source:* `power_hostility.hpp` shape 2 (pack ceiling); A3's completion record §3.7, in the
  development log.
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
  brake TORQUE (it pins a wheel at 0 V whatever its mode, and reads the mode only for R3d Part
  2F-1's coast-follow, off by default), so tests prove the 0 V dynamics reach rest (9.7 in coast
  from 57 in/s on the lagged plant) and pin the Brake command by state inspection only.
  *Confidence:* reasoned — Brake mode's short-circuit behaviour is VEX-documented; its stopping
  distance from competition speeds on our robot's mass/wheels is unmeasured.
  *Settle (R3 bench, R5 at speed):* command a cancel from cruise on hardware; measure
  stop distance vs the coast prediction; verify motion-after-cancel resumes cleanly.
  *Blast radius if wrong:* the robot rolls further than expected after any cancel/abort —
  bounded by the coast physics either way (never re-energized: the 0 V command is proven by
  test), so the failure mode is "stops like coast", not "keeps driving". If Brake proves
  harmful (e.g. brownout interaction), the safe state is ONE function in ONE place.
  *Amended DEFECTS2 (2026-09-26) — until then this stop COASTED on a V5:* the PROS adapter sent
  the zero as `move_voltage(0)`, which ignores the brake mode (HA-142), so every cancel path —
  a user cancel, a pre-emption, a fault abort, the panic stop, and so the driver loops' disable —
  was the "stops like coast" failure mode above, silently, while the host tests pinned "Brake" by
  state inspection. Since DEFECTS2 the adapter sends that zero as `brake()`
  (pinned through a whole `robot::Robot` over the shim: `test/brake_stop_stack_test.cpp` s2, s4,
  s5), so the claim can now be tested at all. Three consequences for the settle step: a library
  stop while ENABLED now brakes while a stick release still COASTS (the team lead's ruling —
  `teleop::DriverLoopConfig::releaseBrakeMode`, default Coast, re-asserted every tick because a
  `drive()` pre-emption cancels into Brake), so the two should look different on robot two; a
  DISABLE is not a clean test of this entry, because the loop's Brake stop goes out once the brain
  already reports disabled, and what VEXos does to motor output by then is HA-145; and "re-commanding it repeatedly" is now one `brake()` per
  transition into the stop plus a `set_brake_mode()` per tick, not a `brake()` per tick (HA-144,
  HA-146).

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
| Port map, last season | Drive L {11,−12,13,−14,−15} / R {16,−17,18,−19,20}; Rotation L(−8)/R(10)/B(9); IMU 6; ADI arm 'B', lever 'C'; intake {2,−3}, conveyor {4,−5}, releaser {−6,7} (`main.cpp:23-62`) | R1/R3 runbook | Historical only — since G1 the session header's port map is generated from a robot's config (`robot::Robot::portMap()`), and since G1 Part 2c every program built on `robot::Robot` prints it that way ("shulib Teleop" and the invented X-drive, whose map is now a config constant); the typed table the tester and "shulib Drive" read (`src/robots/chassis_table.hpp`) is still typed by hand. Recorded so the *shape* of a real port map (5-motor sides, reversed-port minus convention) informs R1's adapter tests |
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
| Pneumatics/air budget, mechanism-specific constants | When written, no mechanism existed even on paper (F′-phase, build-team-gated). *(Since T3, 2026-09-26, robot two's claws exist on paper and in "shulib Teleop" — their beliefs ARE registered, HA-153..HA-159 and HA-147; what stays excluded is the AIR budget (how many actuations a tank holds — nobody has stated the tank or the cylinders) and the constants of mechanisms not yet declared, which F3's primitives register when authored.)* |
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
