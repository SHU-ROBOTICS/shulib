# R3 — live progress log

> Appended continuously, in real time, by the session running the chunk. **Raw device readings are
> logged here BEFORE they are interpreted** — a bench session's value is in the numbers that cannot
> be reconstructed later. If this chunk is interrupted, this file is the recovery point.
>
> Started 2026-08-17. Tree clean at `14128c0`, branch `shulib-v2`, synced with `origin/shulib-v2`.
>
> **Renamed from `R3-PROGRESS.md` → `R3a-PROGRESS.md`** once the split was ruled (§5).
> `tools/briefing_status.py:85-91` pairs a `-PROGRESS.md` with a `-COMPLETED.md` by **exact stem**, so
> leaving it as `R3-PROGRESS.md` would have demanded an `R3-COMPLETED.md` for a chunk that no longer
> exists — a gate failure at the very end of the chunk, naming the wrong problem. Renamed early on
> purpose. *(The sentence in §2 below still says `R3-PROGRESS.md` because that is what it was called
> when the event it describes happened.)*

---

## Phase 0 — reading and orientation (no edits)

- Read `RESUMING.md`, `PROJECT-BRIEFING.md` (full), `build-order.md` (Current position + Phase R),
  `chunks/R1a-BENCH-SESSION.md`, `chunks/R1a-BENCH-RUNBOOK.md`, `chunks/DEFECTS1-COMPLETED.md`,
  `docs/hardware-assumptions.md` (index + R3 group).
- `git log --oneline -5` → HEAD `14128c0`. `git status` → clean. `origin/main` → `c778c11`.

### Register census, measured off the file rather than quoted

```
$ awk -F'|' '/^\| HA-/{...owner...}' docs/hardware-assumptions.md | sort | uniq -c
     48 R4        47 R3        5 R4/E4     5 R1/R3     4 R5
      4 R3/R4      3 R3/R5     2 R6        2 R4/R5     1 T2/R3   1 R2/R3   1 R1
```

**62 entries name R3 as an owner** (47 R3 + 5 R1/R3 + 4 R3/R4 + 3 R3/R5 + 1 T2/R3 + 1 R2/R3, plus
HA-57 which is owned "R1" but whose settling measurement is runbook step 11 and was never run).
7 settled + 1 partial (HA-94…101) leaves **54 R3-owned entries open**.

---

## Phase 1 — reading the code R3 must actually run (before writing a line of the brief)

These are source facts, each re-checked in the tree, not recalled. They are what the
split-or-not ruling rests on.

### 1.1 The shipped binary cannot boot on this robot — and not only because of the port map

`src/main.cpp` is wired for an **X-drive with a GPS and two rotation sensors**:

| `main.cpp` wiring | This robot (measured 2026-08-13) | Result at boot |
|---|---|---|
| `ProsMotor` on 1, 2, −3, −4 | motors on 1, 2, 3, 5; **port 4 is the IMU** | read-back precondition throws |
| `ProsRotation` on 5, 6 | port 5 is a motor; port 6 empty | throws |
| `ProsGps` on 9 | empty | throws |
| `ProsImu` on 10 | empty (IMU is on 4) | throws |
| `xDrive(7.0")` kinematics | tank, LEFT 15/16/17/18 RIGHT 11/12/14 | wrong model |

HA-111 already records the port map as measured-wrong. What HA-111 does **not** record is that
fixing the numbers is not sufficient — see 1.2 and 1.3.

### 1.2 FINDING (structural): shulib cannot command more than one motor per kinematic wheel

`motion/command_pipeline.hpp:146-152` maps **kinematic wheel index → motor span index, 1:1**:

```cpp
const auto motors = deps.ctx->driveMotors();
for (int i = 0; i < wheels.size(); ++i) { ... motors[i]->setVoltage(...); }
```

`kinematics/tank.hpp:80-81` says the opposite is someone else's job —
*"How many physical motors sit on each side is the HAL's business; kinematics sees one speed per
side"* — and **no such HAL facility exists.** `grep -rn "MotorGroup\|motorGroup\|MotorSide"
include/ test/ src/` returns only `include/pros/motor_group.hpp` (vendored PROS, unused by shulib).

`motion/motion.hpp:201-203` checks `driveMotors().size() >= kinematics->wheelCount()` — **`>=`**, so
handing 7 motors to `TankKinematics` (2 wheels) is *accepted* and silently commands **motors[0] and
motors[1] only**. The other five are never given a voltage and never given a brake mode: they are
dead weight the two commanded motors must drag, and nothing says so.

Consequence for R3: this robot has **7 drive motors on 2 sides**. It cannot be driven correctly by
the shipped pipeline. This is a library gap found at the seam, not a bench measurement.

### 1.3 FINDING (structural): no odometry is constructible on this robot

- `localization/tracking_wheel.hpp:60-69` — a `TrackingWheel` takes `hal::IRotation&`. Nothing else.
- `localization/pilons_odometry.hpp:101-113` — `PilonsOdometry` **requires both** a
  `Role::Forward` and a `Role::Lateral` wheel, precondition-checked.
- `localization/localizer.hpp:177` — `Localizer` takes `PilonsOdometry&`, a **concrete type**.
  `grep -rn "IOdometry" include/ test/ docs/` → **no such seam exists.**
- `localization/i_pose_source.hpp` — `Localizer` is the only `IPoseSource` implementor in the tree
  (`grep` confirms: one class, plus one string in `api_reference_fidelity_test.cpp`).

So the chain **motion → IPoseSource → Localizer → PilonsOdometry → 2 × IRotation** is hard, and
this robot has **zero rotation sensors**. There is no motor-encoder odometry path: an `IRotation`
adapter over `IMotor` does not exist, and a tank chassis has no lateral wheel to put one on.

Consequence for R3: **no closed loop can be closed on this robot** without new library code.
"A v2 auton runs on the robot" is not gated on a measurement; it is gated on two capabilities that
do not exist.

### 1.4 A29 confirmed in source, and it is bench-measurable here

`motion/odo_stall_check.hpp:82-84`:

```cpp
/// Drive wheel RADIUS (inches) — converts shaft radians to surface travel.
/// Stand-in geometry (3.25" wheel, 1:1 gearing — A4: HA-14).
units::Length wheelRadius{3.25 / 2.0};
```

and `:153` — `spinTravel = meanShaftDelta * cfg_.wheelRadius.value()`, where `meanShaftDelta` comes
from `IMotor::position()`, i.e. the **cartridge output shaft**. Any external gear ratio between that
shaft and the wheel is unrepresented, and `wheelRadius` is the only field it can hide in — which
makes the field's name and its own comment wrong on any geared drivetrain. Exactly as DEFECTS1
handed it over (A29).

**Bench-measurable in minutes:** count the teeth on the motor gear and the wheel gear on this robot,
and measure the wheel diameter with a caliper/ruler. That produces a real ratio and a real diameter.

---

## Phase 2 — baseline gates, before touching anything

Run 2026-08-17 on a clean `14128c0` tree. **The build failed first**, and correctly: creating
`R3-PROGRESS.md` made R3 an interrupted chunk, so `briefing_status.py check` (a build gate) went
red and blocked the compile. The documented escape worked exactly as written —
`python3 tools/briefing_status.py generate`, then build. Logged because it is the deadlock the
briefing warns about, met on the very first action of the chunk.

```
[doctest] test cases:    1151 |    1151 passed | 0 failed | 3 skipped
[doctest] assertions: 1523871 | 1523871 passed | 0 failed |
[doctest] Status: SUCCESS!
```

| Gate | Result |
|---|---|
| `api_doc_tool self-test / check-coverage / check-fresh / check-examples / check-removability` | PASS ×5 |
| `briefing_status.py check` | PASS |
| `doc_staleness_audit.py self-test` + audit | PASS ×2 |
| `prepare_site.py /tmp/site_src` | PASS |
| GUARD1 (PROS-free, path-anchored) · GUARD2 (sim-free) | PASS · PASS |
| ARM gate, 149 headers as one TU | PASS |
| `release.py check` | 2 expected FAILs: dirty tree, interrupted chunk R3 — the gate working |

Baseline matches the briefing's generated block exactly (1,151 / 1,523,871 / 3 skipped).

### The three skipped tests, by name

```
test/accuracy_spec_test.cpp:121   [acceptance][M3] end-of-60s fused pose within row-F2 targets
test/accuracy_spec_test.cpp:127   [acceptance][M3] vision docking nests a 1.6in pin
test/gps_conversion_test.cpp:186  gpsSensorPose: FIELD-CAL axis oracle
```

---

## Phase 3 — the two loose ends from last session, resolved with evidence

### 3.1 `stash@{0}` — **season content. Do not drop without the team lead's say.**

```
$ git stash list
stash@{0}: On lodge: stash
$ git stash show --stat stash@{0}
 src/seasons/pushback_2026/auton.cpp | 54 ++++++++----------------------------
 1 file changed, 11 insertions(+), 43 deletions(-)
```

One file, and it is **`src/seasons/pushback_2026/auton.cpp`** — a season auton. By the §16 guardrail
that is *students' strategy content*, which is exactly the class of thing this process must not
delete on its own judgement. It is fully preserved in the stash. (It is also very likely the source
of the six stray `include/shulib/seasons/` headers that moved DEFECTS1's assertion count by 6.)
**Verdict: keep, and ask.**

### 3.2 The `robot-bringup` worktree — **safe to remove; holds nothing unique.**

```
$ git worktree list
/home/gonzei/projects/shulib                     14128c0 [shulib-v2]
.../663015e7-.../scratchpad/robot-bringup        6d5dd35 [main]
.../bbbecc27-.../scratchpad/mainchk              0fe7d71 (detached HEAD)
```

Measured rather than assumed:

- `git merge-base --is-ancestor 6d5dd35 origin/main` → **YES**, so its HEAD is superseded.
- Its 157 changes are **all staged** (102 `D`, 55 `M`, zero unstaged, zero untracked).
- Its staged tree `d829082…` vs `origin/main^{tree}` `9693265…` → **different, and different in the
  direction that matters**: `diff-tree staged→main` is a long list of `A docs/api/*.md`. The
  released tree is a strict superset; the worktree is an *earlier, incomplete* hand-rolled attempt.

**Verdict: it is an abandoned manual release attempt, fully superseded by `c778c11` and by
`tools/release.py`. Nothing in it is unique. Removal is safe — but it is outside the repo and it is
the team lead's, so it is reported, not removed.** (A third worktree, `mainchk`, is detached at
`0fe7d71` with only an untracked `site_out/` — scratch, also harmless.)

---

## Phase 4 — pre-brief host measurements (raw output, uninterpreted first)

`scratchpad/probe_r3.cpp`, built standalone against the headers:
`g++ -std=gnu++20 -I include -I test -I test/vendor probe_r3.cpp`.

### 4.0 An instrument error of my own, logged because it nearly became a finding

The probe's **first run reported "0 of 7 motors commanded"** and I nearly wrote that down. The cause
was mine: `FakeBattery`'s default voltage is **0 V**, and `compensateForBattery` correctly zeroes
every volt against a dead pack. The finding was an artifact of my fake, not of the library. Fixed by
setting 12.6 V, and a **negative control** added (the same command through a context holding exactly
2 motors) so the 7-motor number cannot be a broken probe. Trap 1, self-inflicted, caught by looking.

### 4.1 RAW — 7 drive motors + `TankKinematics`, one `applyCommandPipeline` call

```
=== A. 7 physical drive motors + TankKinematics (2 kinematic wheels) ===
  battery = 12.600 V, command = vx 20.0 in/s, vy 0, omega 0
  motor[0]  volts =  +4.4286   brake = Coast(untouched)
  motor[1]  volts =  +4.4286   brake = Coast(untouched)
  motor[2]  volts =  +0.0000   brake = Coast(untouched)
  motor[3]  volts =  +0.0000   brake = Coast(untouched)
  motor[4]  volts =  +0.0000   brake = Coast(untouched)
  motor[5]  volts =  +0.0000   brake = Coast(untouched)
  motor[6]  volts =  +0.0000   brake = Coast(untouched)
  => 2 of 7 motors received a nonzero voltage.
  => faults raised by the pipeline: NONE (silent)

  -- negative control: same command, exactly 2 motors --
  motor[0]  volts =  +4.4286
  motor[1]  volts =  +4.4286
```

**Interpretation.** 1.2 is confirmed by measurement, with a working instrument: with the robot's real
7-motor drivetrain, **five of seven motors are never commanded and never braked, and nothing
complains.** The negative control commands both of two motors at the identical 4.4286 V, so the
instrument distinguishes the two cases. This is not a wrong number to correct on a bench — it is a
missing capability, and it blocks driving this robot at all.

### 4.2 RAW — the F5 tank reference table (for the on-robot number match, HA-18)

`TankKinematics{12.0 in}`; hand-check is `left = vx − ω·6.0`, `right = vx + ω·6.0`, computed
independently in the same line.

```
  twist (vx, vy, omega)             left in/s     right in/s   hand-check
  ( 20.00,  0.00,  0.00000)        20.000000000   20.000000000   L=20.000000000 R=20.000000000
  (  0.00,  0.00,  1.00000)        -6.000000000    6.000000000   L=-6.000000000 R=6.000000000
  ( 20.00,  0.00,  1.00000)        14.000000000   26.000000000   L=14.000000000 R=26.000000000
  ( 20.00,  7.50,  0.00000)        20.000000000   20.000000000   L=20.000000000 R=20.000000000
  (-15.00,  0.00, -2.00000)        -3.000000000  -27.000000000   L=-3.000000000 R=-27.000000000
  ( 10.00,  0.00,  3.14159)        -8.849555922   28.849555922   L=-8.849555922 R=28.849555922
  strafeAuthority() = 0.0000
```

All six agree with the hand-derivation to 9 decimals. Row 4 confirms **vy is silently ignored** as
`tank.hpp:49` documents. These become the host half of HA-18's number match — the robot must print
the same digits. *(Caveat, stated so it is not over-read: host and robot compile the same header, so
this checks that ARM `-Os` and host `-O0/-O2` agree on the arithmetic. It does not validate the
kinematics — trap 1. The geometry is validated by a ruler, separately.)*

### 4.3 RAW — HA-123's bound expressed as an implied speed

`PilonsOdometryConfig::maxTickTravel = 36.0 in` (`pilons_odometry.hpp:81`), and the class holds no
clock, so:

```
  dt =  0.005 s  ->  36 in/tick =     7200.0 in/s =    600.0 ft/s
  dt =  0.010 s  ->  36 in/tick =     3600.0 in/s =    300.0 ft/s
  dt =  0.020 s  ->  36 in/tick =     1800.0 in/s =    150.0 ft/s
  dt =  0.050 s  ->  36 in/tick =      720.0 in/s =     60.0 ft/s
  dt =  0.100 s  ->  36 in/tick =      360.0 in/s =     30.0 ft/s
  dt =  0.200 s  ->  36 in/tick =      180.0 in/s =     15.0 ft/s
```

A VEX drive tops out near 70 in/s. At the nominal 10 ms tick the bound admits **51× the physical
maximum**; it only becomes interesting near dt ≈ 0.5 s. So a measured loop rate converts HA-123 from
`36 in` to `vMax × dt`, which is what DEFECTS1 said it should have been.

### 4.4 A29 confirmed: no gear-ratio parameter exists at the seam either

`hal/pros/motor.hpp:96` — `ProsMotor(std::int8_t port, MotorGearset gearset)`. The cartridge is
selectable; an **external** reduction has nowhere to go. Combined with 1.4, the whole library's only
home for a gear ratio is `OdoStallCheckConfig::wheelRadius`.

---

## Phase 5 — the ruling, and the team lead's two calls

### 5.1 THE RULING: R3 splits three ways

Full reasoning and the rejected alternative are in the brief
([`R3a-tank-bench-validation.md`](R3a-tank-bench-validation.md) §3). One-line version:

**The register is not what blocks R3. The DoD is.** 49 of the 54 open R3-owned entries are reachable
on this robot in some degree; the clause *"a v2 auton runs on the robot"* is blocked on **two missing
library capabilities**, not on missing hardware.

The obvious split — *this bot vs a competition bot* — was **REJECTED**: it would file M2's on-robot
clause behind a robot that does not exist, when it is reachable on **this** bench with two additive
pieces of code. So the axis is *what is missing*:

| Chunk | What is missing | Gate |
|---|---|---|
| **R3a** | nothing but the measurements | the robot in the room |
| **R3b** | a **library capability** (Findings 1–3) | the robot in the room + R3a's numbers |
| **R3c** | **hardware** — GPS, pods, camera, H-bot | a competition robot |

Chunks 44 → 46. Nothing renumbered (R3a/R3b/R3c follows the R1a/R1b precedent).

**What the ruling costs, stated:** `build-order.md:1349` promised *"R1–R3 close M1's Definition of
Done"*. M1's DoD is *"identical numbers in a host test and on the V5, swapping only `RobotContext`"*,
and Finding 3 makes that unbuildable here. **M1's badge does not flip at R3a; it flips at R3b.** A
real slip, recorded in the deviations table rather than absorbed.

### 5.2 Register classification — all 62 R3-owned entries accounted for

| Group | Count | Meaning |
|---|---:|---|
| already settled | 7 | HA-94/95/96/97/99/100/101 (2026-08-13) |
| **(a) settleable tonight** | 20 | brain + IMU + controller + 11 live motors + a ruler |
| **(b) settleable in part** | 9 | the unsettled half must be named, not omitted |
| **(c) opportunistic** | 13 | needs a loose sensor / SD card / an identified mechanism motor |
| **(d) impossible here** | 13 | needs GPS, mounted pods, a camera, or the H-bot → **R3c** |
| | **62** | ✓ reconciles with the census in Phase 0 |

### 5.3 Team lead's ruling — port 13: EXCLUDE, and carry the asymmetry

Port 13 stays mechanically dead and is **dropped from the drive map entirely**. The measured
drivetrain is therefore **LEFT 15/16/17/18 (4 motors) vs RIGHT 11/12/14 (3 motors)**.

**This is not cosmetic and it must ride on every number tonight:** at equal commanded voltage the left
side produces roughly a third more force, so **this robot will not drive straight under an open-loop
symmetric command.** That is expected, not a defect to chase — and it means R3a's numbers are not
evidence about straight-line behaviour at all.

### 5.4 Team lead's answer — bench kit

Confirmed on hand: **brain, IMU, controller.** Sensors "can get some" — uncertain. So group (a) and
(b) are the plan; group (c) is opportunistic and each entry stays open naming the sensor it needs.

**The controller is the win here.** No controller was paired at any point on 2026-08-13
(`master=0 partner=0`), so HA-103, HA-104, HA-107 and HA-57 have never been reachable — including the
**15-vs-19 LCD column conflict**, where two documents disagree and neither is a measurement.

---

## Phase 6 — the upload path, verified before asking for the robot

Checked so that a bench session does not begin by debugging the build (the 2026-08-13 session lost a
long stretch to exactly that, and to a dropped USB cable):

| Known blocker (briefing L8) | State | Evidence |
|---|---|---|
| `CXX_STANDARD=gnu++26` | **already fixed** | `Makefile:35` pins `CXX_STANDARD:=gnu++20`, overriding `common.mk:24`'s `?=gnu++26` |
| stale soft-float firmware / `liblvgl.a` | **already fixed** | `firmware/` holds `libc.a`, `libm.a`, `libpros.a` and the linker scripts — **no `liblvgl.a`** |
| PROS CLI + toolchain | present | `pros 3.5.6`, `/usr/bin/arm-none-eabi-g++`, user in `dialout` |

`make` run from a clean tree:

```
Linking hot project with ./bin/cold.package.elf and libc,libm,libpros [OK]
Section sizes:
   text	   data	    bss	  total	    hex	filename
27.40KB   4.00B  46.01MB  46.03MB 2e07a2c bin/hot.package.elf
Creating bin/hot.package.bin for VEX EDR V5 [DONE]
```

**Observation, not investigated, recorded so it is not lost: `bss` is 46.01 MB.** The 2026-08-12 run
booted this same path so it evidently fits, but no document anywhere records the binary's RAM
footprint, and HA-59 asserts "64 KiB of RAM is spendable on the blackbox staging buffer" as though
the budget were tight and known. **46 MB of BSS is three orders of magnitude above that entry's frame
of reference.** Not chased in this chunk; flagged as a candidate register entry for whoever needs a
RAM budget (R4 owns HA-58/59/60).

**The brain is not plugged in yet** — `ls /dev/ttyACM*` → no such file. That is the first bench step.

---

## Phase 7 — bench session, batch 1: the measurements that need no code

Asked of the team lead while the validation binary is being written, because these are the highest
value-per-minute items in the chunk and **none of them requires a program on the brain.**

*(Raw answers to be pasted in below as they come back, before any interpretation.)*

| # | Measurement | Settles | What a wrong answer looks like |
|---|---|---|---|
| B1.1 | Drive wheel outside diameter, tread to tread | `HA-14` (first half) | anything not near 2.75 / 3.25 / 4.0 in — the three VEX sizes. A number between them means the tread is worn or it was measured across a hub |
| B1.2 | **External gear ratio** — teeth on the motor's gear : teeth on the wheel's gear (or "direct drive") | `HA-14` (second half), **and DEFECTS1's `A29`** | 1:1 *would* mean the library's stand-in is right; anything else means `OdoStallCheckConfig::wheelRadius`'s name and comment are wrong on this robot, exactly as A29 predicted |
| B1.3 | Track width — left wheel contact line to right, centre to centre | `HA-17` (tank half), `HA-52` (`rotationRadius`) | a number far from the frame width; a 15–18 in chassis should read close to its frame |
| B1.4 | Photograph the brain screen → **Devices** | `HA-120`'s open expander question, and re-confirms `HA-111` | an ADI expander appearing here would confirm the 2026-08-13 report; its **absence** corrects that report, which was read from registry index 21 — outside the documented 0–20 range |
| B1.5 | What do the motors on ports **1, 2, 3, 5** drive? | scopes `HA-92`, and tells R3b what mechanisms exist | — |

---

## Phase 8 — `origin/calypso`: a prior robot's MEASURED drivetrain, found on an old branch

Team lead's suggestion, and it was a good one. `origin/calypso` — 2 commits (`dea557e` "wip ah",
`6306651` "Initial commit for calypso robot") — is a **LemLib** project for a robot called Calypso,
with drivetrain constants **measured on hardware 2026-04-21/22**. This is the closest thing to a prior
measurement of a real team robot that exists anywhere in this repository's history.

**Provenance warning applied throughout: none of this is adopted. It is a set of falsifiable
PREDICTIONS for tonight's bench session.** Every number below was measured on *a* robot in April with
a tape measure and a phone compass, by a different codebase, with no IMU and no encoders.

### 8.1 RAW — `src/hardware.cpp` (the port map and geometry)

```cpp
// Left side: ports 11, 12, 13, 14 (12 and 14 reversed via the negative sign).
// Right side: ports 15, 16, 17, 18 (16 and 18 reversed). Port 15 was added
// after the 19 -> 15 shuffle done by the builder.
// All motors use the BLUE cartridge [...] but the right side is mechanically
// geared down a touch for traction (intentional; see RIGHT_DRIVE_BIAS).
pros::MotorGroup left({11, -12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup right({-16, 17, -18, 15}, pros::MotorGearset::blue);

// Track width 15", wheel diameter 3", placeholder RPM 450, drift radius 2.
lemlib::Drivetrain tachyon(&left, &right, 15, 3, 450 /*placeholder*/, 2);

pros::MotorGroup intake({1, -2});
pros::MotorGroup conveyor({3, -4});

// NOTE (2026-04-21): smart port scan confirmed NO sensors are actually wired.
pros::Imu imu(6);                // NOT WIRED
pros::Rotation horizontal(21);   // NOT WIRED
pros::Rotation vertical(-20);    // NOT WIRED

pros::adi::Pneumatics column('A', false);    // the CRANE
pros::adi::Pneumatics releaser('B', false);  // flap at the end of the crane
pros::adi::Pneumatics unloader('C', false);  // the "tongue"
pros::adi::Pneumatics descore('D', false);   // defensive stick

lemlib::TrackingWheel horizontalTracking(&horizontal, 1.5, -4);
lemlib::TrackingWheel verticalTracking(&vertical, 1.5, 0);
lemlib::ControllerSettings translational(6, 0, 3, 0, 1, 100, 3, 500, 0);  // never tuned
lemlib::ControllerSettings rotational  (1.25, 0, 2, 0, 1, 100, 3, 500, 0);  // never tuned
```

### 8.2 RAW — `include/calibration.hpp` (measured 2026-04-21/22, Phase 0 + Phase C)

| Constant | Value | Note as written |
|---|---|---|
| `TURN_VOLTAGE` | 60 | of PROS's ±127 scale. *"Measured: 253°/s at 60V (Phase 0.4)"* |
| `CRUISE_SPEED_FORWARD` | 0.0584 in/ms | = **58.4 in/s** at cruise |
| `CRUISE_SPEED_REVERSE` | 0.0572 in/ms | forward × 0.9795 — **not independently re-measured** |
| `STARTUP_LOSS_FORWARD` | 0.8 in | net of ramp-up loss ~3.58″ minus ramp-down gain ~2.78″ |
| `CRUISE_RATE_TURN` | 0.410 deg/ms | = **410 °/s** cruise (consistent with the 253 °/s average once the 53° ramp loss is applied) |
| `STARTUP_LOSS_TURN` | 53.0 deg | |
| **`RIGHT_DRIVE_BIAS`** | **1.08** | *"Right-side drivetrain has intentionally different (slightly larger) gearing for traction, so at identical voltage the right wheels travel slightly less distance than the left."* Tuned to <0.25″ spread over a 48″ drive |

**How the turn numbers were obtained, and it caps their worth:** *"we have no IMU to count them"* and
*"We hit a phone-compass measurement noise floor around ±3°."* The **distance** numbers were
tape-measured and are worth more than the **angle** numbers, which were phone-compass measured.

### 8.3 The conflicts with the 2026-08-13 bench session — four of them, all checkable in minutes

| | Calypso (April, from code) | Bench bot (August, measured) |
|---|---|---|
| drive port **sets** | {11,12,13,14} and {15,16,17,18} | {15,16,17,18} and {11,12,14} — **same two sets** |
| which set is **LEFT** | **11,12,13,14** | **15,16,17,18** — **OPPOSITE** |
| cartridges | **BLUE** (600 rpm) | **GREEN** (200 rpm) — **3× disagreement** |
| port 4 | conveyor motor | **IMU**, alive and calibrating |
| port 5 | AI Vision sensor | a motor |
| IMU | port 6, **NOT WIRED** | port 4, **working** |
| sensors wired | **none** (port scan 4/21) | IMU working |

**Two readings of this, and I am not choosing between them without a measurement:**

- **(a) Same robot, rewired between April and August**, with one of the two side labels wrong. The
  drive port sets matching exactly is a strong coincidence otherwise, and "the team's old competition
  bot" fits Calypso's description (Push Back was the 2025-26 game).
- **(b) Two different robots** that happen to group drive motors as 11–14 and 15–18, which is a
  natural grouping.

**On the side-label conflict specifically:** August's determination was a *direct physical
observation* — one side spun by hand while a read-only monitor showed the other side reading exactly
zero — which is much stronger than a code comment. **But "left" depends on which end of the robot the
observer called the front**, and nothing in the August record says how that was fixed. A mirrored
left/right is exactly the class of error that produces a robot that turns the wrong way, and it is
free to re-check.

### 8.4 THE FINDING — A29 is worse than "no gear-ratio concept". It needs a PER-SIDE ratio.

*"the right side is mechanically geared down a touch for traction"* + `RIGHT_DRIVE_BIAS = 1.08`
means, if this drivetrain is the bench bot's, that **the two sides have different gear ratios.**

shulib cannot represent that at all:

- `motion/odo_stall_check.hpp:84` — `wheelRadius` is **one scalar for every drive motor**.
- `kinematics/tank.hpp:47-51` — `toWheels()` is `left = vx − ω·halfTrack`, `right = vx + ω·halfTrack`:
  **the same scale on both sides, by construction.** An asymmetric drivetrain is outside the model,
  not merely unparameterized.
- `hal/pros/motor.hpp:96` — `ProsMotor(port, gearset)`: no ratio parameter of any kind.

So DEFECTS1's `A29` ("no gear-ratio concept exists anywhere") understates it. The fix R3b needs is a
**per-side** ratio, not a single library-wide one — and that is a design change to how a drivetrain is
described, not a new constant. **Checkable tonight by counting teeth on BOTH sides**, which is now the
single highest-value measurement in the chunk.

### 8.5 Two opportunities this opens

1. **Two rotation sensors were specified** — `pros::Rotation horizontal(21)` and `vertical(-20)`, with
   1.5″ tracking wheels at offsets −4 and 0. Declared, never wired. **If those sensors physically
   exist in a parts bin, R3b's odometry problem largely evaporates and a POSITION-based auton becomes
   reachable** rather than being gated on a purchase. Worth asking before anything else.
2. **Sanity check on shulib's invented motion budget.** Calypso measured ~58.4 in/s forward and
   ~410 °/s (7.15 rad/s) turn. shulib's `HA-50` stand-ins are **60 in/s** and **6 rad/s** — the same
   order, and closer than an invented number has any right to be. Not a confirmation (different
   robot, different cartridges, unrelated derivation), but the guess is not absurd, and that is worth
   knowing.

### 8.6 What must NOT be carried across, and why

- **`RIGHT_DRIVE_BIAS = 1.08` must not be ported.** It is a voltage bias on PROS's ±127 `move()`
  scale, tuned against a specific brake mode, battery and floor, for a *time-based open-loop* auton.
  shulib is voltage-based and closed-loop-heading; the bias's whole job is done by heading feedback
  instead — Calypso's own `config.hpp` says exactly this: *"The static RIGHT_DRIVE_BIAS used by the
  time-based driveStraight() is NOT applied here — heading feedback subsumes it."*
- **The LemLib PID gains must not be ported.** Calypso's own comment: *"Numbers are placeholders
  pending real tuning."* Briefing trap: never carry gains across chassis.
- **The turn constants are phone-compass grade (±3°).** shulib's hard target is **< 1.0°**. These
  cannot inform it.
- **`calibration.hpp`'s own re-calibration rule applies to us too:** *"Re-calibrate after any of
  these change: battery model swap, drivetrain mechanical change (gears, wheels, weight, tire wear),
  floor / mat surface change, brake mode setting change."* Between April and August at least the
  cartridges and the sensor wiring changed, and a drive motor's chain came off.

### 8.7 Also on that branch, not pursued here

`include/ai_vision.hpp` — the working `pros::AIVision` wrapper the master plan §462-469 already
names as R2's starting point ("the team already has a working `pros::AIVision` wrapper"). Confirmed
present. **R2's input exists**; it is object-mode only and needs tag-mode extension, exactly as the
master plan says. Also present: `docs/00_PROJECT_CONTEXT.md` … `05_STATUS.md`, a `skills_auton_24hr_plan.md`,
and `legacy_auton.cpp` (the time-based routes). The doc set is internally stale — `05_STATUS.md` lists
the config module as "Not Started" while `config.hpp` exists — so **the code on that branch outranks
its own documentation.**

*(Other branches noted, not read: `origin/test-bot` — "Wasnt able to get a working config without
changing x drive motor group coefficients" — is an **X-drive with motor groups**, which is a third
independent sighting of the capability shulib lacks. `origin/lodge` is the earlier v2 attempt.)*

---

## Phase 9 — PORT 13 IS FIXED (team lead, 2026-08-17), and it strengthens the Calypso match to near-certainty

> *"the builder found the problem with the one motor so he fixed it you can expect now all similar
> readings"*

### 9.1 §6.4's ruling is SUPERSEDED — port 13 is back in the drive map

The earlier ruling (exclude port 13, carry a 4-left-vs-3-right asymmetry caveat on every number) is
**withdrawn, because its premise no longer holds.** Recorded rather than quietly overwritten: the
ruling was correct when it was made, and mechanical repair is the outcome it was hedging against.

**Consequences, all of them improvements:**

- The drivetrain is **8 motors, 4 per side** — symmetric.
- **The "will not drive straight under an open-loop symmetric command" caveat is retracted.** It was
  a consequence of the 4-vs-3 force imbalance and that imbalance is gone. *(A residual asymmetry may
  still exist from the per-side gearing in §8.4 — a different cause, and one this repair does not
  address.)*
- The multi-motor gap (Finding 1) gets **worse, not better**: with 8 drive motors on 2 kinematic
  wheels, the shipped pipeline would command **2 and leave 6 dead**, not 5. The measured probe number
  in §4.1 was taken at 7 motors and should be re-stated at 8.

### 9.2 The port inventory now matches Calypso almost exactly

With port 13 live, the bench bot's twelve motors line up against Calypso's twelve like this:

| Group | Calypso (April, code) | Bench bot (August, measured) | Match |
|---|---|---|---|
| drive group A | `left {11, -12, 13, -14}` | 11, 12, 13, 14 — **13 now live** | ✅ **exact set** |
| drive group B | `right {-16, 17, -18, 15}` | 15, 16, 17, 18 | ✅ **exact set** |
| intake | `{1, -2}` | motors on 1, 2 | ✅ |
| conveyor | `{3, -4}` | motors on 3, **5** | ⚠️ second motor on **5, not 4** |
| IMU | port 6, **not wired** | port **4**, working | ⚠️ moved |
| AI Vision | port 5 | — | ⚠️ port 5 now holds a motor |
| **total motors** | **12** | **12** | ✅ |

**Every remaining discrepancy is explained by one natural rewire:** the IMU was wired to **port 4**,
which the conveyor's second motor had been using, so that motor moved to **port 5** — which is where
the AI Vision sensor had been declared. One sensor arriving displaces one motor by one port, and the
inventory reconciles completely.

**Ruling: Calypso and the bench bot are the same physical robot, rewired between April and August.**
This is now the working hypothesis rather than a possibility — 12 motors, two exact drive sets of four,
matching manipulator pairs, and a single self-consistent explanation for the only three differences.
**It is still a hypothesis, and two of tonight's checks falsify it if wrong** (§9.3).

### 9.3 Three conflicts remain, and each is a cheap check with a large blast radius

1. **BLUE vs GREEN cartridges — 3× disagreement, ~10 seconds to settle.** Calypso's code says
   `pros::MotorGearset::blue` (600 rpm) for all eight; the August session recorded "all green (18:1,
   200 RPM)". These cannot both be true of one robot at one time. **A 3× error in the gearset makes
   every velocity, every feedforward and every distance-per-tick wrong by 3×.** Look at a cap.
2. **LEFT and RIGHT are labelled OPPOSITE.** Calypso: 11–14 = left. August: 15–18 = left, established
   by a hand-spin with a read-only monitor — a direct physical observation, and the stronger evidence.
   **But "left" is defined relative to whichever end the observer called the front**, and nothing in
   the August record fixes that. A mirrored left/right makes the robot turn the wrong way. Re-check
   with the front end named explicitly.
3. **Port 16's ~20% under-report is NOT addressed by this repair.** The team lead fixed *one* motor
   (13). Port 16 was compared against its own side-mates, so a per-side gearing difference cannot
   explain it, and 13 and 16 are on opposite sides under either mapping — so **fixing 13 cannot fix
   16.** With the drivetrain now otherwise symmetric, port 16 is the sole remaining anomaly and it is
   the one that biases odometry quietly. **Do not assume "all similar readings" includes it — measure
   it.** The discriminator is unchanged: spin one side by hand and compare all four encoders
   individually, never averaged.

---

*(Appended below as the session proceeds. Raw readings first, interpretation after.)*
