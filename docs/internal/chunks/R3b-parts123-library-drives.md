# R3b Parts 1–3 — the library drives robot two

> Execution brief, 2026-09-14. **The specification is already written and is not restated here:**
> [`R3b-first-closed-loop.md`](R3b-first-closed-loop.md) §4 (piece 1) and §5 (piece 2), and
> [`R3b-session2-tank-chassis.md`](R3b-session2-tank-chassis.md) §4 (`MotorGroup`), §5
> (`IOdometry`, `DriveEncoderOdometry`, the single drive geometry), §6 (the composition root), §7
> (tests 1–13 with their mutations), §8 (Parts 1–3 DoD) and §9 (documentation). This file carries
> what has changed since those were written, the rulings that fill their open slots, and the order.
> Live log: append to [`R3b-PROGRESS.md`](R3b-PROGRESS.md) as **"Parts 1–3"**, created FIRST.
> Tree at start: `881b342`. **Deadline that shaped this: coders arrive Thursday 2026-09-17 to
> program the drivetrain with the library; the target is the library's own teleop loop driving
> robot two through `Chassis::drive()`, verified on the bench before they arrive.**

## 0. What changed since the specs were written

| Then | Now |
|---|---|
| ports UNSET, signs unknown | `src/chassis_table.hpp`: ten SIGNED ports (`LEFT -11 +12 -13 +14 -15 \| RIGHT +20 -19 +18 -17 +16`), measured twice, `signsMeasured = true` |
| no IMU | IMU on **port 2** (`imuPort = 2`) |
| the tester alone | three programs on one tree: `PROGRAM=tester` ("Bench Tests", slot 3), `PROGRAM=drive` ("shulib Drive", slot 1, adapters only), and — this chunk — **`PROGRAM=library` ("shulib Teleop", slot 2)** |
| the group's fight detector to be written | `include/shulib/teleop/coupled_side_monitor.hpp` exists, pure and tested — **the group REUSES it** as its disagreement observable (one evaluator, not two) |
| geometry a guess | wheel diameter **2.75 in reported** by the team lead ("standard wheels"); track width **UNSET**; motor→wheel ratio **UNSET** ("600 rpm" reported, which reads as 1:1 but is not a count) |
| port 18 | travels ~20 % short of its side-mates on a push, twice — the group's median and the disagreement observable must both be shown, in a test, to tolerate one such member without flagging it as a fight at rest and WITH flagging it if it opposes |

## 1. RULING — geometry is table data, UNSET refuses, and it is measured before Thursday

`ChassisTable` gains `wheelDiameterIn`, `trackWidthIn`, `externalRatio` (motor→wheel, wheel
revolutions per motor revolution) and `geometryProvenance`. Robot two ships `wheelDiameterIn =
2.75` (reported, provenance says by whom) and the other two **UNSET (0)** until the team lead
reports a tape-measured track width and tooth counts (or "direct drive"). `TankKinematics` needs
the track width and the odometry needs the scale, so **the library graph refuses to construct
while either is UNSET**, painting the state screen with exactly what is missing (the existing
`describeMissing` pattern, extended). Nothing is guessed. The team lead has been asked for both
numbers today; the coordinator types them in on arrival, one edit, with provenance.

## 2. RULING — with drive-encoder odometry, `ODO_STUCK` has no independent source; say so

`OdoStallCheck` compares wheel-implied travel against odometry travel. With `DriveEncoderOdometry`
the odometry IS the wheels, so the check is a tautology and cannot see a stuck robot. **Do not
feed it a fake.** For this drivetrain: `odomStalled` is not computed from the stall check; the
composition root logs ONCE at boot that stuck-detection has no independent motion source on this
robot (no tracking wheels), the panel shows it, and the register carries the entry. The heading
cross-check (encoder ω vs IMU ω, `DriveEncoderOdometry::lastHeadingDisagreement()`) is exposed as
an observable and is what a future stuck/slip detector for this chassis will be built on; it is
NOT promoted to a fault here (a straight-line stall does not show in it). Rejected: leaving the
stall check wired — it would report "not stalled" forever and the health monitor would be lying.

## 3. RULING — order, and what "done for Thursday" means

Part 1 → Part 2 → Part 3, as specified. `Chassis` cannot be constructed without a `Localizer`,
which needs the `IOdometry` seam, so Part 3 cannot precede Part 2. Done for Thursday = **`make
ROBOT=tank PROGRAM=library` builds "shulib Teleop": the R1a teleop loop, unchanged in shape,
driving robot two through `Chassis::drive(speeds, Frame::Body)`**, with the object graph built
from the table, the absent devices explicit, the IMU calibrating at boot, and the state screens
for every non-driver competition state. The first hardware run is the coordinator's and the team
lead's, wheels-up then ground, and is NOT claimed by this chunk. M1's badge does not flip here.

**Expect the library teleop to FEEL different from "shulib Drive", and say so on its panel:** the
pipeline commands wheel *speeds* through feedforward with PROVISIONAL gains (HA-45: kV = 12/70 V
per in/s, a placeholder), capped by `MotionConfig::maxLinearSpeed` and `maxWheelSpeed`, plus
battery compensation. If it is slow or saturates, that is R5's measurement showing up, not a bug;
the panel prints the budgets and the gains in use.

## 4. Part 3 specifics that the session-2 brief left open

- **Program:** `PROGRAM=library` → `-DSHULIB_PROGRAM_LIBRARY`, valid only with `ROBOT=tank`;
  upload `pros upload --slot 2 --name "shulib Teleop"`. Fifth beacon `shulib-robot-variant=tank-library`;
  the `src/` build gate learns the fifth build (define sets, beacon, behavioural expectation:
  like tank — the invented X-drive `Robot` stays dead code; if the tank graph makes those helpers
  live, update the expectation and say so). `xdrive` still emits 0 `-Wunused-function`.
- **The tank `Robot` graph** (the session-2 brief §6.1) built from the table: `TankKinematics{trackWidth}`,
  ten `ProsMotor`s from the signed ports with the table's cartridge, two `MotorGroup`s in the
  kinematics' wheel order (left, right), `ProsImu{imuPort}`, `AbsentGps` / `AbsentTagSource` /
  `AbsentVision` with the never-poll rule, `ProsBattery`, the diagnostics stack exactly as the
  X-drive graph has it, `DriveEncoderOdometry` from the two groups + IMU + geometry,
  `ComplementaryFusion`, `Localizer`, `RobotContext`, `MotionDeps`, `ProsTickPacer`, `Chassis`.
  Function-local static, constructed in `initialize()` after the precondition handler, exactly
  like `robot()`. Ports/signs/geometry printed in the §18.5 session header (port-map string built
  from the table, never retyped).
- **Boot refusal, not fault-abort:** if the table's geometry is UNSET, or the IMU port is 0, or a
  `ProsMotor` constructor throws, `initialize()` catches at the graph boundary, paints the state
  screen with the reason, and `opcontrol()` idles with 0 V. Dead-port tolerance for the LIBRARY
  graph is NOT in scope (the group requires N ≥ 1 present members at construction; a refused
  adapter refuses the graph) — state this on the panel and in the log; "shulib Drive" is the
  tolerant program until R3d.
- **The teleop loop** is the one function both graphs share (session-2 §6.2, already extracted
  for the X-drive graph — reuse it for the tank graph; do not fork it).
- **Panel:** the library teleop's brain screen shows the session header essentials, the estimate
  (x, y, heading from the localizer, its quality class), the two group commands, each group's
  member count / disagreeing count, the heading cross-check, battery, and any latched fault by
  name. Controller LCD: `LIB 12V Bxx.xV`, `hdg ±ddd.d deg`, last fault or `ok`.

## 5. Tests — session-2 §7 tests 1–13 and 15 stand, plus

| # | Test | Mutation → RED |
|---|---|---|
| 16 | `MotorGroup` over the coupled-side monitor: a member ~20 % slow (port 18's signature) at rest and under command is NOT a disagreement; the same member opposite in sign IS | threshold moved so 20 % flags; sign test dropped |
| 17 | The stall-check ruling: a tank graph with `DriveEncoderOdometry` never reports `odomStalled` true from the wheel-vs-odometry check, and the boot log carries the "no independent source" line | the check re-wired |
| 18 | `ChassisTable` geometry: UNSET refuses; set values reach `TankKinematics` and the odometry scale unchanged (bit-identical to a hand-built rig) | swap wheel diameter and track width |
| 19 | Fifth build in the gate: define lands, beacon asserted, `PROGRAM=library` without tank is an `$(error)` | drop the append |

Every mutation run (build exit + md5 checked), red observed verbatim, restored, green re-run.

## 6. Documentation, in the same commit

Session-2 §9 in full, and: worksheet **Station F — LIBRARY TELEOP** (first run wheels-up, then
ground; what the estimate on screen means; what "feels slower than shulib Drive" means);
`hardware-assumptions.md` gains **robot two's section** — the geometry entries (with UNSET rows
where UNSET), VEXos 1.1.5 on this brain, the controller-launch match trap, USB port renumbering,
port 18's short travel, and the no-independent-stall-source entry — each cited from the tree;
`roadmap.md`'s "What R3b must BUILD" checkboxes 1 and 2 flip with file/test/count evidence, the
"you are here" says the library graph builds for robot two and has NOT run, and the stale "the
upload is next" phrase is corrected (the upload happened 2026-09-10); `build-order.md` Next block;
Daniel's note mentions the third program; `docs/guide/14` untouched (still true). `PART 1-3 READY
FOR VERIFICATION` closes the log.

## 7. Landmines

1. Never invent the track width or the ratio. UNSET refuses; the coordinator types measured values.
2. Signs in ONE place (the table → `ProsMotor`); the group never negates.
3. Median, not mean; the disagreement observable comes from the shared monitor.
4. `>=` → `==` in `MotionDeps::validate()`; fix any test that relied on `>=` and say so.
5. The plant's coupled members must be bit-identical for one member per wheel (the whole suite is
   the pin; a count change alone proves nothing).
6. Do not wire `OdoStallCheck` to lie (§2).
7. Frozen surfaces untouched: F4 seams, F5 kinematics, F6 `Chassis`, F10 `Routine`. `Localizer`'s
   constructor taking `IOdometry&` is source-compatible for every existing caller.
8. Baseline `881b342`: suite 1188 / 1,540,944 clean, 154 headers, gate PASS ×4, 3 `src/` TUs.

*Created 2026-09-14.*
