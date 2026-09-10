# R3b — Session 2: the new tank chassis, and the teleop-first order

> **Addendum to [`R3b-first-closed-loop.md`](R3b-first-closed-loop.md).** That brief's §4 and §5
> (pieces 1 and 2) still hold as specifications. This file changes three things about how they are
> executed: **what robot they are built for, in what order, and what is done first so the build
> team can drive the chassis this week.** Where this file and the original brief disagree, this
> file wins; where it is silent, the original brief stands (§7–§11 in particular).
>
> Written 2026-09-10 on a clean tree at `7d3e7ce`. Live log: **append to
> [`R3b-PROGRESS.md`](R3b-PROGRESS.md) as "Session 2"** — create that section FIRST.

---

## 0. What changed since the brief was written

1. **A second robot exists.** The build team has built the tank drivetrain of one of this
   season's two competition robots. It is NOT the bench bot. It has **five motors per side driving
   four wheels per side**, which means the motors on a side are mechanically coupled through a gear
   train. It has **no sensors of any kind** yet: no IMU, no tracking wheels, no GPS, no camera.
   Cartridges are *believed* blue. Wheels are "standard, like before" — the bench bot's measured
   2.75 in, to be confirmed with a ruler. Track width unmeasured. Ports unmeasured.
2. **The build team needs driver control with the controller as soon as possible** to test the
   drivetrain mechanically. That is a legitimate build-iteration need and it is on the R1a runbook's
   own path (steps 4 and 10: open-loop stick drive with wheels off the ground, then the controller
   beliefs) — it was always going to happen; it is happening now.
3. **A design argument was made and accepted** (2026-09-10, team lead): most of what the library
   asks a human to type about a drivetrain is discoverable on the robot — ports from the registry,
   polarity from a hand push, side assignment and track width from the IMU, the distance scale from
   one push along a tape. Only the cartridge (physically unobservable — the rotor speed is the same
   for every cartridge and the firmware divides by whatever it was TOLD, so a reading always agrees
   with the setting; measured on the bench bot, R3a-PROGRESS §17.2/§19.2), the drivetrain kind, and
   which end is the front must be stated by a person. **The consequence is a new chunk, R3d
   (drivetrain self-calibration + persistence + boot verification), pulled forward from E5's
   scope to run immediately after this one.** It is NOT built here; this chunk builds the things
   R3d's outputs feed, and the tester's MOTOR WATCH is R3d's seed.

The governing constraint is unchanged and this session does not close it by itself: **the library
has never driven a robot.** Part 0 below drives the robot through the library's *adapters*, not
its motion stack, and is labelled that way everywhere it appears.

---

## 1. What is known about the new chassis, and how well

| Fact | Value | Status | Settles |
|---|---|---|---|
| Drivetrain kind | tank | stated by the build team | which kinematics |
| Motors per side | 5, coupled, driving 4 wheels | stated | group size N=5; the fighting-motor hazard |
| Motor ports, by side | **unknown** | to be reported | the chassis table (§3.1) |
| Per-motor polarity | **unknown** | MOTOR WATCH hand push (§3.2) | signed ports |
| Cartridge | blue | **believed, not read** — must be read off a motor | `MotorGearset` at every `ProsMotor` |
| Wheel diameter | 2.75 in ("like before") | to be confirmed with a ruler | odometry scale |
| External gearing motor→wheel | unknown; build team can state tooth counts | to be reported | odometry scale; whether the group carries a ratio |
| Track width | **unmeasured** | tape measure, contact line to contact line | `TankKinematics`; the heading cross-check |
| Which end is the front | **undecided** | build team decides | sign of "forward" |
| IMU | **none mounted** | build team mounts one, any smart port | required — see §2.3 |
| Tracking wheels / GPS / camera | none | — | `DriveEncoderOdometry`, `AbsentGps`, `AbsentTagSource`, `AbsentVision` |

**Rule for every "unknown" above: it is NEVER invented.** A variant built with an unset chassis
table refuses loudly at boot and says what is missing. HA-111's invented port map is the exact
defect class, and it is not repeated for robot two.

---

## 2. Scope and order

### 2.1 In, in this order

- **Part 0 — the bench tester learns this chassis and gains a DRIVE station.** Small. Unblocks the
  build team. **STOP after Part 0, run the verification battery, append the log, and report.** The
  coordinator verifies and commits Part 0 on its own so it can be uploaded, then says "continue".
- **Part 1 — `hal::MotorGroup`** (the original brief's piece 1, §4).
- **Part 2 — `localization::IOdometry` + `DriveEncoderOdometry`** (piece 2, §5), and ONE
  representation of per-side drive geometry consumed by odometry and the stall check.
- **Part 3 — the composition root for the new chassis**, and the teleop loop shared by every
  variant, so the existing R1a loop drives this chassis through the frozen `Chassis::drive()`.

### 2.2 Out, with the owner

- Side/polarity/scale/track-width **discovery routines and their persistence** → **R3d** (new; §9
  records it). Part 0's MOTOR WATCH capture is in-memory for one power cycle and that is all.
- Driver feel (curves, slew, per-driver tuning) → T2. The shared mapping stays deadband-only
  (HA-112 stays invented).
- Measured noise → R4; real gains → R5; the plant back-fit → R6.
- Anything needing a GPS, tracking wheels, a camera → R3c.

### 2.3 Explicitly rejected

- **An `AbsentImu`.** The Localizer's heading is IMU-owned by locked design
  (`pilons_odometry.hpp` header, decision #3), the `< 1°` heading spec depends on it, and R3d's
  discovery routines need it. Absence is the honest word for a device a robot legitimately lacks
  (GPS on a bench bot); it is a lie for a device the design requires. **The build team mounts an
  IMU. Until it is mounted, the library variant does not boot its graph, and says so.**
- **A voltage fan-out loop anywhere but the tester.** Part 0 lives in `src/bench_r3a.cpp` as a
  numbered, hands-on station, and the tester's banner says it is not the motion stack. Nothing of
  it is copied into `main.cpp`'s library path; the library path is `Chassis::drive()` or nothing.
- **Per-member signs inside `MotorGroup`.** Reversal is the adapter's business — a negative port,
  applied by PROS once (`hal/pros/motor.hpp:78-92`). One place for a sign, or the sign gets
  applied twice.
- **Relaxing the `RobotContext` preconditions** or touching F4/F5/F6/F10. `MotorGroup` *implements*
  `IMotor`; it does not change it.
- **Mean aggregation for position/velocity.** See §4.2.

---

## 3. Part 0 — the tester, for this chassis

Read `src/bench_r3a.cpp` end to end first (1,188 lines): the menu table at `:966`, the census, the
`kLeftPorts`/`kRightPorts` hypothesis at `:81-84`, `motorWatch()` at `:781-875`, `reportPlatform()`
at `:878` (it already constructs a `ProsController`), and the `handsOn` flag every dangerous
station must carry.

### 3.1 The chassis table — the ONE typed input

Replace the two hard-coded port arrays with a per-variant chassis descriptor: ports per side, the
gearset, and a `measured` flag/date. `ROBOT=bench` keeps the bench bot's measured table (15–18 L,
11–14 R, blue — R3a-PROGRESS §15–§19). **`ROBOT=tank` ships with the table UNSET** until the build
team reports; the census still runs (it needs no table), MOTOR WATCH runs over every motor the
registry finds with side `?`, and DRIVE refuses with "chassis table unset — see §3.1" on screen.
When the ports arrive, the coordinator fills the table; that edit is the only thing between the
build team and a driving robot, so keep it to one obvious place.

### 3.2 MOTOR WATCH, generalized

- Iterate **every motor the census found**, not a hypothesis list (up to 21). Sides come from the
  table when set, `?` otherwise. The panel layout assumed two columns of four; make it lay out N.
- **Capture the verdict.** After the touch, store per port: `+1` (UP), `−1` (DOWN), `0` (did not
  move), in a process-global that DRIVE consumes. A whole-robot push is the only capture that
  yields signs; a single-wheel spin yields the port→wheel map and must NOT be mistaken for signs —
  distinguish by the moved-count exactly as the existing verdict text does.
- The convention: the front the build team chose is "forward"; the operator pushes toward it. A port
  that reads DOWN on a forward push is wired reversed and will be constructed with a **negative
  port** by DRIVE.

### 3.3 The DRIVE station — the one station that powers motors

Everything else in this binary is read-only, and that has been a safety guarantee. Keep it one:
this station is the **only** code path that calls `setVoltage`, and the banner, the menu label and
the `handsOn` stripe all say so.

Construction: one `hal::pros::ProsMotor{signedPort, gearset}` per table entry (the adapter's
constructor WRITES the gearset and reads it back — `hal/pros/motor.hpp:96-110` — so this is also
where a wrong cartridge would be entrenched; print the configured gearset loudly as a belief), one
`hal::pros::ProsController` master. Through the adapters, not raw PROS: this is the first time
`ProsMotor` and `ProsController` run on hardware, and the run measures HA-94 onward.

Safety gates, all of them, in this order — a station that skips one is the defect:

1. **Refuse unless MOTOR WATCH captured signs this power cycle** for every table port, and the
   capture was a whole-robot push (moved count == table count).
2. **Confirm wheels-off-the-ground on screen** before the first volt. The DONE-IF text says when
   to put it down.
3. **Dead-man:** motors are driven only while a named controller button is held (choose one that
   is not near the sticks; say which on screen and the controller LCD). Release → 0 V, coast.
4. **Voltage ceiling starts at 3 V.** A button steps it 3 → 6 → 9 → 12 V, shown on both screens.
   It never starts above 3 V, and it resets to 3 V when the station is re-entered.
5. **The fighting-motor cut-out.** While commanded above a small threshold, every member of a side
   must report a velocity of the SAME sign as its side-mates (the sign the adapter already applied).
   One member disagreeing, or one member near zero while its mates move, cuts the whole drive to
   0 V, names the port on screen, and stays cut until the station is re-entered. This is the
   coupled-drivetrain hazard: a wrong sign on one of five coupled motors stalls it against the other
   four. Also cut on any member above 2.4 A for more than 250 ms, and show current and temperature
   per port live.
6. Exit (touch) → every motor 0 V, coast.

Mapping: arcade — left stick Y forward, right stick X yaw, the SAME pure mapping function the
library teleop loop uses (§6.2), so the driver feels one robot in both programs. Per-side command
= ceiling × (fwd ∓ yaw), clamped to the ceiling, same volts to every member of the side.

### 3.4 The variant

Add **`ROBOT=tank`** to the `Makefile`'s validated enumeration (keep `bench` and `xdrive` exactly
as they are; the `$(error)` lists all three), with its own define, and teach
`tools/src_build_gate.py` the third variant: the structural check (its define lands on every
`src/` TU and no other variant's does), the hash assertion, and the behavioral detector. For the
behavioral detector, `tank` behaves like `bench` (the invented X-drive `Robot` struct is dead code
in it → ≥ 1 `-Wunused-function` from `src/main.cpp`); say so in the tool header next to GATE1's
coupling note. Extend the self-test so a variant whose define does not land is caught for `tank`
too. CI's `arm-compile-gate` job runs the same two commands and needs no edit unless the tool's
interface changes.

In Part 0, `ROBOT=tank`'s `opcontrol()` runs the tester (as `bench` does) and `initialize()` builds
no library graph. Part 3 changes that.

### 3.5 The bench procedure

Append a **Station D — DRIVE** to `R3a-BENCH-WORKSHEET.md` in that file's voice (DO / DONE IF, the
amber-stripe convention): mount the IMU, pair the controller, read the cartridge, ruler the wheel,
tape the track width, report ports per side and the chosen front; then census → MOTOR WATCH push
→ DRIVE at 3 V wheels up → raise → ground. Results append to `R3a-PROGRESS.md` as a new phase,
by the coordinator, from the SD log.

**Then STOP** (§2.1).

---

## 4. Part 1 — `hal::MotorGroup`

`hal/motor_group.hpp`, `final`, implements `IMotor` over a NON-OWNING
`std::span<IMotor* const>` of N ≥ 1 members, all non-null (precondition), members outlive the
group — the tree's ownership convention (`hal/motor.hpp:57-65`).

### 4.1 Command path — fan-out, nothing else

`setVoltage(v)` and `setBrakeMode(m)` go to every member, same value. Clamping is each member's.
`commandedVoltage()` is the group's last applied value (a local mirror, like `ProsMotor`).

### 4.2 Read path — median, and a disagreement observable

**RULING: `position()` and `velocity()` are the MEDIAN across members, not the mean.** Members are
coupled, so they agree to within noise when healthy, and the question is what happens when one is
not: a member whose port stopped answering reports its LAST GOOD position forever (`ProsMotor`'s
contract, `hal/motor.hpp:47-54`). A mean drifts by 1/N of all further travel — silently wrong
odometry, exactly the failure the odometry cross-checks cannot see because the number keeps moving.
A median is untouched until a majority fails. Rejected: mean (above); "first member" (one dead port
kills the side with no signal at all).

`current()` = mean per member, so the tree's per-motor stall/capture thresholds keep their
meaning (a member at 2.4 A reads as a member at 2.4 A); the sum is exposed as a separate,
non-`IMotor` accessor. `temperature()` = MAX (the hottest member throttles first; the thermal
monitor must see the worst one). `brakeMode()` = the first member's read-back, with a disagreeing
member counted below.

**The disagreement observable** — `disagreeingMembers()` (count) and which — evaluated on each
read: when the group's median |velocity| is above a floor, a member whose velocity sign differs
from the median's, or whose |velocity| is below a fraction of the median's, is disagreeing. This is
both the fighting-motor detector and the frozen-member detector. **The group raises no fault** —
raising is the loop layer's policy (`hal/pros/motor.hpp` header says the same of the adapter) — but
`HealthMonitor` must turn a persistent disagreement into a fault: append `FaultCode::MotorGroupDisagree`
via the documented additive path (`diag/fault.hpp:45-64`, values are pinned, append only), and
route it through the existing observable-ticking helper (`motion/motion.hpp` `tickHealthObservables`)
so it fires in `drive()` and in every motion. Policy in the scheduler's mask: **continue-degraded**
by default (the drive still works with one bad member; the driver must be told, not stopped),
configurable like the others. Document the choice and its alternative (abort) in the header.

### 4.3 The pipeline needs no change, and the guard tightens

With `driveMotors = {&leftGroup, &rightGroup}`, `command_pipeline.hpp:147-152`'s 1:1 wheel→motor
mapping is CORRECT by construction — that is the point of the group. What remains wrong is that
`MotionDeps::validate()` (`motion/motion.hpp:201-203`) accepts MORE motors than wheels, which is
how eight raw motors on a two-wheel kinematics leaves six silent. **Change `>=` to `==`**, with the
message naming `MotorGroup` as the fix. Update `robot_context.hpp:34-37`'s comment, which promises
the count is never checked. If an existing test relied on `>=`, it was exercising the defect; fix
the test and say so in the log.

### 4.4 Plant support — coupled members are physical truth

`sim::DrivePlant` takes exactly one `FakeMotor*` per wheel (`drive_plant.hpp:153-171`) and writes
the synthesized state into it. To test a group against the plant, the plant must be able to write
the SAME wheel state into every member of a coupled set, and to degrade ONE member (frozen position;
sign-flipped velocity) for the detector tests. Add that as an optional per-wheel member list on the
plant — one member per wheel remains the default and must be bit-identical to today (the whole
existing suite is the pin). Read `hal/fake/fake_motor.hpp` and the plant's `synthesizeSensors()`
before deciding the shape; keep the plant derivable from rigid-body kinematics (its header note).

---

## 5. Part 2 — `IOdometry` and drive-encoder odometry

### 5.1 The seam

`localization/odometry.hpp`: `IOdometry` with exactly the four members `Localizer` uses —
`update()`, `pose()`, `setPose(const Pose2d&)`, `lastDeltaImplausible()` (`localizer.hpp:204-502`,
all `odom_.` uses). `PilonsOdometry` derives from it with NO behavioural change (the existing suite
is the bit-identity pin). `Localizer` takes `IOdometry&`; every existing caller still compiles.
Neither `Localizer` nor `PilonsOdometry` is in the Freeze Register — check the register anyway and
say so in the log.

### 5.2 `DriveEncoderOdometry`

`localization/drive_encoder_odometry.hpp`: constructed from the IMU, the left and right `IMotor`
(the groups), the per-side geometry (§5.3), the track width, an initial pose and a config with the
same two trust-gate knobs as `PilonsOdometryConfig` (same names, same defaults, same reasoning —
cite it, do not re-derive it). Per tick:

- ΔL, ΔR = each side's cumulative `position()` delta (radians) × that side's inches-per-radian.
  Baseline both at construction (the `TrackingWheel::reset()` precedent, `pilons_odometry.hpp:119`).
- Δθ from the IMU exactly as Pilons does (`Angle::errorTo`, wrap-correct); **heading is IMU-owned**,
  identical policy, identical wording in the header.
- Center travel: forward = (ΔL + ΔR)/2, lateral = **0**, citing `kinematics/tank.hpp:53-56`'s
  written decision that this drivetrain cannot observe lateral motion. Then the SAME `arcStep`.
- Trust gate: both halves, same semantics as Pilons (report, never withhold; freeze position only on
  a non-finite tick).
- **A heading cross-check, new:** encoder-implied Δθ_enc = (ΔR − ΔL)/trackWidth against the IMU's
  Δθ; expose `lastHeadingDisagreement()` (radians, signed). This is the wheel-difference heading the
  Pilons header calls "the cross-check only, never the authority" — it is what a slipping side
  looks like, and it is the measurement R3d's track-width calibration is built from. Not a fault
  here; an observable.

### 5.3 One drive geometry, two consumers

The original brief §4.1 ruled the per-side ratio lives with the motors, never in F5 kinematics, and
that piece 2 must not solve it twice. Concretely: **one value type describing a drive side's
geometry** — wheel radius and motor→wheel ratio, yielding inches-per-radian — instantiated ONCE per
side at the composition root, and consumed by BOTH `DriveEncoderOdometry` and `OdoStallCheck`.
`OdoStallCheckConfig::wheelRadius` (`motion/odo_stall_check.hpp:82-84`) is today one scalar for
every drive motor with a name and comment A29 predicted would be wrong on a geared robot; it must
become per-wheel (a span sized to `wheelCount`, or the geometry type itself), with the symmetric
case still one line to write. A test must prove the two consumers agree on implied travel for the
same encoder delta from the same geometry object. Asymmetric sides (the calypso "right side geared
down a touch", R3a-PROGRESS §8.4) must be representable even though this chassis is believed
symmetric.

`A29`'s "the A2 plant bakes 1:1 in too": `DrivePlantConfig::driveWheelDiameter` (`drive_plant.hpp:136-138`)
is the plant's half. If the build team reports external gearing other than 1:1, teach the plant the
ratio in this chunk (the original brief's ruling: M1's comparison is not checkable otherwise). If
direct drive, leave it and say so.

---

## 6. Part 3 — the composition root, and one teleop loop

### 6.1 The tank robot graph

In `src/main.cpp` under `ROBOT=tank`: `TankKinematics{trackWidth}`, ten `ProsMotor`s from the
chassis table (signed ports, the read cartridge), two `MotorGroup`s, `ProsImu` on the reported port,
`AbsentGps` / `AbsentTagSource` / `AbsentVision` (the R3b §6 wiring rule: never polled),
`ProsBattery`, the diagnostics stack as the X-drive graph has it, `DriveEncoderOdometry`,
`ComplementaryFusion`, `Localizer`, `RobotContext`, `MotionDeps`, `ProsTickPacer`, `Chassis`. The
port-map string is built from the same constants (the `portMapString()` precedent). **If the chassis
table is unset or the IMU port is unset, `initialize()` logs exactly what is missing and boots into
the tester instead** — never a fault-abort, never an invented value.

### 6.2 One mapping, one loop, three variants

Extract the stick → `ChassisSpeeds` mapping (deadband, axis assignment, speed budgets) into a
PROS-free, host-tested pure function in `include/shulib/` (a small header; T2 will replace its body
with real driver feel, so name and document it as the seam T2 owns). Both `main.cpp` variants call
it, and the tester's DRIVE station calls it for its axes (§3.3). Extract the R1a loop body into
one function both graphs use. Nothing about the mapping changes (HA-112 stays invented).

### 6.3 Choosing tester vs teleop on the tank variant

At `opcontrol()` start on `ROBOT=tank`: a two-button brain-screen chooser, **TELEOP** (default
after 3 s) / **BENCH TESTS**. A build-team member should never need the CLI to reach either.

---

## 7. Tests — every one names the bug it catches; mutations are run, not described

| # | Test | Bug it catches | Mutation that must go RED |
|---|---|---|---|
| 1 | Group fan-out: every member receives every `setVoltage`/`setBrakeMode` | a silent surplus motor | skip member N in the loop |
| 2 | N=1 group is bit-identical to the bare motor through a full plant routine | the group changes numbers | any arithmetic in the read path |
| 3 | Median aggregation with one FROZEN member: odometry unaffected, member counted | the 1/N drift | median → mean |
| 4 | Fighting member (sign-flipped velocity via the plant degradation): counted; `HealthMonitor` raises `MotorGroupDisagree`; scheduler continues degraded | undetected coupled fight | detector disabled; fault not routed |
| 5 | `MotionDeps` with more motors than wheels THROWS naming `MotorGroup` | six silent motors | `==` → `>=` |
| 6 | `temperature()` = max, `current()` = mean-per-member, sum exposed | thermal blind spot; threshold semantics | max → mean |
| 7 | 5-member groups per side on tank, ideal coupling, through the plant: bit-identical to the 1-motor baseline (clean AND hostile sweeps from `test/motion_sweep_test.cpp`) | the group perturbs motion | — (equivalence pin) |
| 8 | `PilonsOdometry` through `IOdometry`: the ENTIRE existing suite unchanged | the seam changed behaviour | — (suite is the pin) |
| 9 | `DriveEncoderOdometry` vs the A2 truth integrator: straight, arc, in-place spin (zero translation), reverse | integration/sign errors; the arcStep independence property | swap ΔL/ΔR; drop the /2; lateral ≠ 0 |
| 10 | Asymmetric per-side scale: a 2 % right-side error produces the predicted drift; correct scales produce none | one-scalar geometry | use the left scale for both sides |
| 11 | Heading cross-check fires under injected slip, silent otherwise | a slipping side goes unnoticed | cross-check computed from the wrong sign |
| 12 | Trust gate parity with Pilons (implausible Δθ; non-finite tick freezes position, heading advances) | the gate was not carried over | remove either half |
| 13 | Stall check and odometry agree on implied travel from ONE geometry object, symmetric and asymmetric | two sources of truth for the ratio | change one consumer's conversion |
| 14 | The pure stick mapping: deadband, axis signs (left-pushed = +Y, right-pushed = −ω), zero twist when disconnected | the driver's stick drives the wrong way | flip any sign |
| 15 | `ROBOT=tank` variant: the src build gate asserts its define lands and the behavioral detector holds; self-test catches a non-landing define for `tank` | §4.1's silent no-op, third variant | drop the Makefile append for tank |

The D5 standard from Session 1 applies to every mutation: check the mutated binary actually BUILT
(the stale-binary near-miss, R3b-PROGRESS §4.2), and a red for any reason other than the one the
test is meant to catch is a fake red — say so.

---

## 8. Definition of Done

**Part 0 (stop and report here):**
- [ ] `ROBOT=tank` builds, links, and the src build gate passes for all THREE variants; self-test OK
- [ ] The chassis table exists, `bench` is measured, `tank` is UNSET and refuses DRIVE loudly
- [ ] MOTOR WATCH iterates all registry motors and captures signs for DRIVE
- [ ] DRIVE carries all six safety gates (§3.3), through `ProsMotor`/`ProsController`, and the pure
      mapping function of §6.2 (host-tested, test 14)
- [ ] Worksheet Station D written; the tester banner says which station powers motors
- [ ] Verification battery green; log appended; **nothing committed**

**Parts 1–3:**
- [ ] `MotorGroup`, `IOdometry`, `DriveEncoderOdometry`, the single drive geometry, the tightened
      `==` guard, the plant's coupled members, `MotorGroupDisagree` routed — tests 1–13 green, every
      mutation observed red then restored
- [ ] `ROBOT=tank` boots the library graph when the table and IMU are set, the chooser works, the
      teleop loop is one function shared by both graphs
- [ ] All eight doc gates pass; new headers documented and generated; roadmap/build-order/register
      updated per §9
- [ ] **Not claimed here:** M1's badge, "the library drove a robot", HA-18/52/112 — those flip only
      on the bench, from the SD log, by the coordinator. `docs/guide/14` stays as it is until then.

---

## 9. Documentation contract

The eight gates and the obligations of new public headers are exactly the original brief's §9.1–§9.2
(nav is GENERATED — Session 1 §2 — never hand-edited). In addition:

- **`R3b-PROGRESS.md`** — "Session 2" section created first; scope declaration first (Session 1's
  precedent), so an interrupted log cannot over-claim. Part 0 ends with a line the coordinator can
  find: `PART 0 READY FOR VERIFICATION`.
- **`roadmap.md`** — the "What R3b must BUILD" checkboxes (`:1166-1180`) with file/test/count
  evidence; the "you are here" paragraph gains the second-robot fact and the R3d decision. Under-claim.
- **`build-order.md`** — R3b's status line; **three new deviations rows**: (1) teleop-first order
  inside R3b, with the ASAP reason; (2) **R3d pulled forward from E5**, with the runtime-discovery
  argument and what stays in E5 (GPS lever arm, camera mount, IMU bias); (3) the third build
  variant. Add R3d to Phase R after R3c's entry and to the Stage 1 path
  (`R3a → R3b → R3d → R4 …`). **Also fix the stale last sentence at `:700`** ("main is current
  through Phase D and is missing E1–E4, F1, F2, R1a and R1b") — it predates the 2026-08-15 release
  the paragraph above it describes; correct it by measurement (`git cat-file -e main:<path>`).
- **`hardware-assumptions.md`** — the register was written for one robot. Add a **"Robot 2 — the
  2026 tank chassis"** section for the facts in §1 as they are MEASURED (not as reported), each with
  source and date; do not rewrite the bench bot's rows. New entries for the beliefs this chunk
  introduces (coupled-member agreement thresholds; the disagreement floor) as invented, owned by R3d/R4.
- **`R3a-BENCH-WORKSHEET.md`** — Station D (§3.5). **`R3a-PROGRESS.md`** — one cross-reference
  line pointing at Session 2 for the tester changes (append, never rewrite).
- `docs/README.md` and `docs/internal/docs-publishing.md` entity/page counts (ungated, §9.3 of the
  original brief). `PROJECT-BRIEFING.md` regenerated with the CLEAN-tree suite number (Session 1 §6).

---

## 10. Landmines

1. **Never invent a port, a sign, or a track width for robot two.** Unset means unset, refused
   loudly. The first robot's port map was invented and every adapter constructor threw at boot.
2. **The cartridge is a write.** `ProsMotor`'s constructor sets the gearset and reads back whatever
   it set; a wrong belief passes the read-back. Print the belief loudly in the tester and in the
   session header, and do not let "believed blue" become "blue" in any register row.
3. **Coupled motors fight.** Gate 5 in §3.3 is the difference between a test and a repair.
4. **Sign in one place.** Negative port in the adapter. A group that also negates applies it twice.
5. **Median, not mean** (§4.2), and the reason is a dead port's frozen last-good reading.
6. **`>=` → `==` may break a test that was exercising the defect.** Fix the test, log it.
7. **The plant change must be bit-identical for one member per wheel.** The whole suite is the pin;
   an assertion-count change alone proves nothing (the `-dirty` trap, R3a-PROGRESS §12.2).
8. **Do not resurrect the ABI blocker** — `make` links; GATE1 measured it; `RESUMING.md` boxes it.
9. **Generated nav, generated briefing, generated api pages** — regenerate, never hand-edit.
10. **STOP after Part 0.** The build team is waiting on it; Parts 1–3 are not.
11. Run every verification command from the repo root (the anchored `sed`).

---

## 11. Verification baseline — measured today, 2026-09-10, clean tree at `7d3e7ce`

| Check | State |
|---|---|
| Host suite | **1157 cases / 1,538,101 assertions / 0 failed / 3 skipped** |
| `include/shulib/*.hpp` | **151 headers** (ARM header gate clean, `-Werror`) |
| `docs/api/` | **120 pages**; 1,645 entities / 118 headers, all documented |
| `src_build_gate.py check` | **PASS**, bench + xdrive link, hash `v0.1.1-273-g7d3e7ce` |
| Toolchain | `arm-none-eabi-g++ 13.2.1` |

Re-establish on a clean tree before trusting any number.

---

*Companion to [`R3b-first-closed-loop.md`](R3b-first-closed-loop.md) and
[`build-order.md`](../build-order.md) §R3b. Created 2026-09-10.*
