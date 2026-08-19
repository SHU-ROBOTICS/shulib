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

## Phase 10 — bench batch 1, first two answers (team lead, 2026-08-17)

### 10.1 RAW answers

> **1.** *"we didnt add odom aka rotation sensors to calypso but we have some somewhere"*
> **6.** *"the left side is ports 15-18"*

### 10.2 SETTLED — the drive side labels. **LEFT = 15/16/17/18. RIGHT = 11/12/13/14.**

Confirmed by a **second independent physical determination**, five days after the first
(2026-08-13 hand-spin with a read-only monitor; 2026-08-17 direct re-check with the front end named).
Two observations, two methods, same answer. **This is the strongest-evidenced fact about this robot.**

### 10.3 FINDING — Calypso's code has LEFT and RIGHT **SWAPPED**, and its one measured
### asymmetry constant is therefore attached to the wrong side

```cpp
// origin/calypso, src/hardware.cpp — the labels, against measured reality:
pros::MotorGroup left ({11, -12, 13, -14}, ...);   // ← these are physically the RIGHT side
pros::MotorGroup right({-16, 17, -18, 15}, ...);   // ← these are physically the LEFT side
```

**Consequences, and the third one is the reason this matters to us:**

1. **`RIGHT_DRIVE_BIAS = 1.08` was applied to the physically-LEFT side.** Whatever it compensated for,
   it did not compensate for it on the side its name claims.
2. **The comment *"the right side is mechanically geared down a touch for traction"* is now ambiguous**
   — it may mean the code's `right` (physically LEFT) or the author's mental "right" (physically
   RIGHT). **Nothing in that branch can resolve it**, which promotes batch-1 step 3 (tooth counts on
   BOTH sides, labelled by physical side) from "high value" to **the measurement that resolves an
   otherwise unresolvable ambiguity.**
3. **A swapped pair survives empirical tuning invisibly.** Calypso's auton was tuned by driving it and
   adjusting until it did the right thing — so the tuning silently absorbed the swap and the robot
   behaved. Nothing would ever have surfaced it. **This is trap 1 in its purest form: the calibration
   and the error cancelled, and the result was a robot that worked and a codebase that was wrong.**

**Discipline this imposes on R3a and R3b:** shulib's `TankKinematics` fixes canonical wheel order as
**0 = LEFT, 1 = RIGHT** (`kinematics/tank.hpp:16`). Wiring the port groups from Calypso's *names*
would mirror every turn. **The port map is taken from the physical measurement, never from that
branch**, and the bench binary prints the side label beside the port list so a mirror is visible on
screen rather than inferred from a robot turning the wrong way.

### 10.4 The rotation sensors EXIST — position-based odometry is not gated on a purchase

*"we didnt add odom … but we have some somewhere."* They were never fitted to Calypso (its
`Rotation horizontal(21)` / `vertical(-20)` are declarations against unwired ports, exactly as its own
comment says), **but the team owns some.**

**This materially improves R3b's ceiling.** The earlier reading — that a position-based auton is gated
on buying parts — is **withdrawn**. It is gated on *finding and mounting* parts the team already has.
Two consequences for R3b's design, both to be ruled when the sensors are in hand:

- The drive-encoder odometry path (`EncoderRotation`) stays worth building regardless — it is what
  makes shulib usable by teams *without* pods, which is a capability LemLib has and shulib does not,
  and it is the only honest home for a gear ratio. It does **not** become dead work.
- But if two real pods get mounted, `PilonsOdometry` runs **as shipped and as designed**, with no
  lateral-observability compromise at all — which is a strictly better first closed loop than a
  forward-only one, and it makes the R3c "tracking-wheel geometry" and "push test" entries reachable
  on *this* robot rather than on a competition robot.

**Open, and now worth asking:** how many, and is there room to mount two perpendicular unpowered
wheels (one forward-rolling, one lateral-rolling) near the tracking centre?

### 10.5 Still pending from batch 1

Steps **2** (cartridge colour — the 3× BLUE/GREEN conflict), **3** (tooth counts, per side),
**4** (wheel diameter), **5** (track width), **7** (port 16 play), **8** (Devices photo + what
ports 1/2/3/5 drive).

---

## Phase 11 — the v1 autonomous code review, mapped against v2 finding by finding

Team lead supplied a two-part code review titled *"Forward Movement Inconsistency Investigation —
shulib Chassis Library + Autonomous Routines"*. **It is a review of shulib v1**, not v2: it names
`main.cpp`, `odometry.cpp`, `pid.cpp`, `odomUnit.cpp`, `chassis.cpp`, `drivetrain.cpp`,
`tankdrive.hpp`, `logger.cpp` (v2 is header-only and has none of these) and calls
`move_vertical` / `rotate_to` / `move_to_pose` over `pooksterLeft` / `pooksterRight` — the
`origin/pookster*` branch family. This is the code C6 audited and C7 deleted.

**It is the most valuable document handed to this project in weeks**, because it is an *independent*
account of how a real robot actually failed, written without reference to v2's design — and it is
therefore a fair test of whether v2's structural choices were the right ones.

### 11.1 Every finding, checked in v2's source. 10 of 10 are structurally absent.

| v1 finding | v2 status | Verified at |
|---|---|---|
| **5.1 accumulated distance** — each tick adds the magnitude of the step, so noise only ever *adds* | **absent by design.** Error is recomputed absolutely every tick: `errX = target_.x() − pose.x()` | `move_to_pose.hpp:171-173` |
| **5.2 the `kC` term** — constant power 25 at any error, and **NaN when error is exactly 0** (it divides by the error) | **absent by design.** kS keys off the *velocity setpoint*, not error, and the zero case is an explicit branch — no division, no NaN: `(v>0)? kS : (v<0)? −kS : 0.0` | `feedforward.hpp` `calculate()` |
| **5.3 heading PID all zeros, rotation hardcoded 0** | **absent by design.** `MoveToPose` runs **three decoupled per-axis loops**; heading is one of them, mutation-proven at C1 | `move_to_pose.hpp:15,197-198` |
| **5.4 odometry uses FINAL heading, not average** | **absent — and this is the exact legacy bug re-deriving `arcStep` caught.** `thAvg = headingStart + 0.5·dTheta` | `arc_step.hpp:93,99` |
| **5.5 PID `time` passed as a literal** (0.001 or 5 against a 5 ms loop) | **absent by design.** dt comes from an **injected clock**, measured; `dt ≤ 0` applies P only | `pid.hpp:10-11,85-88` |
| **5.6 no mutex on `odomPose`** | **not applicable.** v2 has no concurrent writer — one loop, no shared mutable pose | see 5.7 |
| **5.7 a new `pros::Task` every 20 ms in opcontrol** | **not applicable.** `grep -rn "pros::Task" include/ src/` finds **one** hit, `Task::delay_until` — a static sleep, not a task creation. **v2 creates zero tasks** | `hal/pros/tick_pacer.hpp:71` |
| **5.8 `new` without `delete`** | **not applicable.** No owning `new` in the library | — |
| **5.9 NO TIMEOUT on any movement loop** — *"a single stuck move kills the entire autonomous"* | **absent by design, and it is one of v2's headline guarantees.** Every motion is `Watchdog`-bounded, takes a `timeout`, and reports an `ExitReason`; `defaultTimeout = 5.0 s` | `move_to_pose.hpp:88-92`, `motion_config.hpp:112` |
| **5.10 a background task commands the drive motors while a movement is running** | **absent by design.** `MotionScheduler` enforces **exactly one active motion** structurally — a new one PRE-EMPTS, cancelling the old inert before the new exists. F1's **claim token** does the same for mechanisms | `motion_scheduler.hpp:6,29`, `hal/mechanism.hpp:41-50` |

### 11.2 The honest caveat, and it is the whole point of §3

**Every "absent by design" above is TIER-1 evidence** (host simulation) — see `ORIENTATION.md`'s three
tiers. v1's failures were observed **on a robot that actually drove**. v2's fixes have **never met
one.** So the correct claim is *"v2's design does not contain these ten failure modes"*, **not**
*"v2 would not have these problems"* — and the difference is exactly the gap R3a and R3b exist to
close. A structural absence is a strong claim about logic and says nothing about constants.

**And v2 fixes none of the review's own §1.3 "not ruled out" list**, which is entirely physical:
tracking-wheel diameter wrong, pods bouncing or under-sprung, mount play, surface friction, battery
sag between runs, motor thermal droop. Those are **precisely the hardware-assumptions register's
territory** — HA-13, HA-25, HA-37/38/39, HA-40/41, HA-43/44 — and every one of them is still
`invented`. **v1's review and v2's register are describing the same unknowns from opposite ends.**

One narrower honesty note: v2's *stopping* behaviour is structurally right (profiled decel, settle
tolerance + rate floors, watchdog) but its **constants are guesses** — `SettleConfig`'s tolerances and
rate floors are **HA-51, invented**. v2 does not have v1's `kC` failure *mechanism*; whether v2 stops
cleanly on a real robot is unmeasured.

### 11.3 NEW information this document gives R3 — three things, all load-bearing

1. **The team has previously run a robot with THREE tracking wheels, mounted and working.**
   *"Left wheel Port 10, 1.5in diameter, offset −6.5in · Right wheel Port −8, 1.5in, offset +6.5in ·
   Back wheel Port 9, 1.5in, offset −4.0in."* This is much stronger than §10.4's *"we have some
   somewhere"*: pods **exist, have been mounted, and have been driven.** R3b's odometry blocker is
   not a purchase and not a novel fabrication problem — it is a re-mount of a configuration this team
   has already built once.
2. **`Theta correction = 1.275` — raw tracking-wheel heading scaled by 27.5% to match reality.**
   A 27.5% fudge factor on heading is not a calibration, it is a **symptom of badly wrong geometry**
   (wrong offsets, wrong effective diameter, or slip). It is also the single best argument for v2's
   *heading is IMU-owned* rule, which exists precisely so wheel geometry can never corrupt heading.
   **This factor must NOT be carried across under any circumstances** — v2 does not derive heading
   from wheels at all, so there is nowhere for it to go, and re-introducing it would mean re-creating
   the bug it was papering over. Note also that v1 **had an IMU on port 6 and passed `nullptr` to
   `OdomSensors`** — the sensor that would have fixed heading was plugged in and unused.
3. **1.5″ tracking-wheel diameter appears in BOTH this review AND the Calypso branch** — two
   independent sources agree. shulib's `HA-13` stand-in is **2.0″**. If the team's pods really are
   1.5″, **HA-13 is wrong by 33%**, and that is a direct multiplier on every odometry distance. It
   goes on the caliper list beside the drive wheels.

### 11.4 What to do with the review itself

Its §8 testing plan is sound and **its §8.3 physical checks are R3a's worklist almost verbatim**
(caliper the tracking wheels, check ground contact, check mount play, check sensor seating). Its §8.5
fix order is for v1 and is moot — C7 deleted that code, and v2 already implements fixes (1), (3), (4),
(5) and (6) structurally. **The one item worth acting on regardless is §8.4** (the opcontrol task
spam), and only if any v1 branch is still being driven by students; v2 cannot inherit it.

*(Filed for the record rather than actioned: this review reached findings C6's audit did not,
because C6 classified files for salvage and this traced a live failure. Both were right about the
port list being empty — nothing here is code worth carrying, and the value is entirely in the
failure modes and the three physical facts in §11.3.)*

---

*(Appended below as the session proceeds. Raw readings first, interpretation after.)*

---

## Phase 12 — VexBuilder handover, and an assertion-count trap measured to ground

### 12.1 Deliverable

`docs/vexbuilder-integration.md` (366 lines) — the complete contract, written FOR the VP who is
finishing VexBuilder rather than for us. Three seams, every field, both sides' ordered work lists,
the versioning promise, and a "what can still move under you" risk table.

`roadmap.md`'s Cross-team asks section rewritten: it listed only **VexBuilder's** four obligations,
which is how a two-sided dependency becomes a stall with each side believing it is blocked. Now
carries both lists plus three asks that were missing entirely — the `robotProfile` block (implied by
#2, never stated, so it had no owner), **the electrical UI** (VexBuilder's true critical path:
`.vexbot` v2.0.0 ships `electrical{}` as EMPTY ARRAYS because the UI was never built, and that is the
source data for `robotProfile`), and a **per-motor `gearRatio`** (shulib has no gear-ratio concept at
all, and this robot may be geared differently left vs right — §8.4).

Added to `mkdocs.yml`'s nav, not just to `docs/` — DOCS2 measured that a page absent from the nav
publishes **unreachable with exit code 0**. Verified present in `prepare_site`'s output.

### 12.2 MEASURED — the assertion count is a function of a DIRTY WORKING TREE

The count moved 1,523,871 → **1,523,877** with **no C++ change**. Chased to ground rather than waved
at, because the count is a build-gate input:

| Step | Observation |
|---|---|
| ran the **same binary** with and without the doc edits | **1,523,877 both times** → the docs are not the cause |
| checked `include/shulib/` | **148 headers, unchanged** → not the DEFECTS1 stray-header trigger |
| read the baked-in hash | `v0.1.1-245-g3b1266d-**dirty**` |
| reconfigured on a genuinely clean tree | hash `v0.1.1-246-g0b86ee7`, count **1,523,871 exactly** |

**Cause:** `test/CMakeLists.txt` injects `git describe --always --dirty` as `SHULIB_BUILD_HASH`; a test
asserts through that string; **`-dirty` is exactly 6 characters.**

**This is a second trigger for the trap DEFECTS1 recorded with a different one** (a stray untracked
header moving the count by 6 — coincidentally the same delta, which is its own small trap). Two
further facts that make it hard to diagnose: the hash is captured at CMake **configure** time, so a
plain `cmake --build` can carry a stale one indefinitely; and **committing changes the answer**, so
the gate looks broken at exactly the moment you are trying to commit. Note also that **untracked files
do not make `git describe` report dirty** — only modified *tracked* files do, which is why this
session's earlier baselines read clean while an untracked progress log existed.

**The committed briefing must carry the CLEAN-tree number**, because that is what a fresh clone
reproduces. Recorded in the briefing's process-failures list with the escape sequence.

*(No hardware measurement in this phase. Batch 1 steps 2, 3, 4, 5, 7, 8 remain open.)*

---

## Phase 13 — the validation binary exists, and one scope item was narrowed on purpose

### 13.1 Deliverable

`src/bench_r3a.{hpp,cpp}` + a compile-time robot selector in `src/main.cpp`. Default build is now
the **tank bench bot**; the invented X-drive wiring is **preserved verbatim** behind
`-DSHULIB_ROBOT_XDRIVE_INVENTED` and **was re-compiled to prove it still builds** — it is the only
artifact of the 2026-08-12 whole-object-graph boot and losing it silently was the risk.

### 13.2 RULING — the binary is READ-ONLY. §4.2's "commands open-loop voltages" is NOT built.

§4.2 item 1 specifies an entry point that "commands open-loop voltages on request." **This build
commands no motion at all**, and that is a decision rather than an omission:

- The only thing open-loop voltage buys is **which port drives which wheel, and in which
  direction**. That is obtainable at **zero risk** by turning a wheel BY HAND and watching the
  encoder — worksheet Station 2, and stage 3 of the binary prints exactly the numbers to watch.
- Powering eight motors whose **signs are unmeasured**, on a robot whose **port map is the thing
  under test**, can lurch a robot off a bench and buys nothing the hand method does not.
- If open-loop commands are wanted later they belong behind an explicit opt-in **with the wheels off
  the ground**, as a separate change.

### 13.3 Design: it probes BEFORE it constructs

Every `hal/pros` adapter ctor does a device read-back and raises a precondition on disagreement —
correct for a competition binary, **wrong for a discovery binary**, because one wrong port would kill
the session before it printed anything. So stage 1 is a **raw registry census of all 21 ports**
constructing nothing and unable to fail; stages 2+ construct adapters only for what stage 1 found,
each inside a `try/catch`, so **an adapter refusing a device is captured as a measurement** rather
than ending the run.

### 13.4 §10.3's mirror check is built in

The port groups are labelled **`LEFT(hyp)` / `RIGHT(hyp)`** and the banner states in three lines that
they are a **hypothesis to be falsified, not a configuration** — because a prior robot's code had the
sides swapped and its tuning absorbed the swap invisibly. Ports 1–21 are all scanned, **21
deliberately included**: that is the index the 2026-08-13 expander report was read from.

### 13.5 Output goes two places — and NOT through the blackbox

USB serial plus, when a card is present, a plain-text file at **`/usd/r3a_bench.txt`** written
through `ProsBlockSink`.

**Recorded because it corrects an assumption made earlier this session:** the E1 **blackbox cannot
carry this**. `blackbox_format.hpp` v1 deliberately omits the `log()` message channel — it carries
per-tick `DebugRecord`s, a run summary and a fault triage block, and counts text lines without
storing them. A text census is therefore written as **raw bytes through the device seam**, which is
what that seam is for. The blackbox becomes useful at R3b, when a control loop produces real
per-tick records.

### 13.6 Verification (tree dirty at time of writing; hash carried from the last clean configure)

| Check | Result |
|---|---|
| `make` (tank, default) | **PASS** — `bin/hot.package.bin`, 12,208 bytes, text 11.92 KB |
| `make -DSHULIB_ROBOT_XDRIVE_INVENTED` | **PASS** — the preserved path still builds |
| Non-`liblvgl` compiler warnings | **0** |
| Host suite | **1151 cases / 1,523,871 assertions, 0 failed** |
| ARM header gate | **PASS, 148 headers** |
| Doc gates (6) | **ALL PASS** |
| Brain detected | `pros lsusb` → `/dev/ttyACM0` + `/dev/ttyACM1`, brain **2F007C00** |

*(The assertion count reads the CLEAN-tree number because the hash is captured at CMake configure
time and no reconfigure happened — §12.2's trap, observed working as documented.)*

### 13.7 What it does NOT settle

`bss` is still **46.01 MB** and still unexplained (§6). No hardware number is measured until the
binary is actually run — this phase delivers the instrument, not the readings.

### 13.8 The brain-screen menu — the bencher runs this unattended

Team lead's ask, and it changes what the binary is for: the bencher operates it **alone, with no
laptop**, so the serial stream is invisible to them and the panel is the only live output.

- A **touch menu** (`pros/screen.h`, no LVGL) with six large targets — census / IMU+rotate / motors /
  batt+ctrl+SD / loop rate / run-all. Buttons are 232x62 deliberately: this is operated by somebody
  crouched over a robot.
- **Every emitted line now mirrors to the panel**, truncated to 54 columns, with a
  *touch-for-more* pause when it fills. The SD file keeps full width.
- The **IMU rotate test suspends the scrolling log** and becomes a large fixed readout with a running
  delta, because that number is watched while turning the robot with both hands — a scrolling log is
  unreadable in that posture. Window widened 8 s → 20 s for the same reason.
- Each test returns to the menu on touch, so **re-running is free**. That matters most for the motor
  test: turn a wheel by hand, re-run, read which port moved and which way.

**Known limitation, stated rather than discovered later:** `ProsBlockSink` owns its `FILE*` for the
whole boot (one file per boot, by its own header contract), so a power cycle starts a **fresh**
`/usd/r3a_bench.txt` rather than appending. The worksheet says to copy the card off before rebooting.

**Uploaded to slot 1, brain `2F007C00`.** Verification re-run after the menu landed: host suite
1151 / 1,523,871 green, ARM gate PASS, six doc gates PASS, zero non-`liblvgl` warnings, and the
preserved X-drive path still compiles.

### 13.9 Program renamed and re-slotted (team lead's direction)

`project.pros`'s `project_name` was still **`QueensRevenge`** — inherited from the pre-rebuild
project and meaningless to anyone reading the brain's Programs list. Renamed to **`Bench Tests`**,
which is what a helper at the robot needs to see, and uploaded to **slot 3** rather than slot 1 so
the low slots stay free for competition programs.

**Open hazard, recorded because nothing in the toolchain closes it:** slot 1 still holds the earlier
`QueensRevenge` upload, and **the PROS CLI has no remove-program command** — a slot can only be
cleared from the brain's own Programs menu. Until somebody deletes it, a bencher can pick a build
whose port map is invented and watch it fault at boot. The worksheet now warns about it in Station A.

### 13.10 SD logging state is now ON THE MENU, not buried in a test

Team lead inserted the card **after** the program had already started, which exposed a real silent
failure: `ProsBlockSink` opens its `FILE*` **at construction**, and the sink is a function-local
static built on the first `runR3a()` call. A card inserted mid-run is therefore **never picked up** —
`write()` returns false from the first call and keeps returning false, exactly as
`hal/pros/block_sink.hpp`'s T5 ruling says it should. Nothing was wrong with the sink; the problem
was that **an unattended bencher could run an entire session believing it was being logged.**

Fix: the menu now carries a permanent SD status line — **green "SD LOGGING ON"** or **red "SD LOGGING
OFF — insert card, then POWER-CYCLE and restart"** — drawn every time the menu is painted, and the
same fact goes into the serial/SD banner. Buttons moved down 16 px so the two-line warning cannot
collide with them.

This is E1's principle 5 applied at the operator level rather than the API level: the sink already
degraded honestly and reported through `isOpen()`; what was missing was **somebody being told**.

*(Also recorded for the runbook: the log is one file per boot and is OVERWRITTEN on each power
cycle. Copy the card off before rebooting if a session's output matters.)*

### 13.11 FIRST RUN: data abort in `strlen` — `std::string_view` through varargs

**The first real execution crashed**, and it found a bug that four green gates could not:

```
data abort exception   PC: 0x38006ea   Current task: User Operator Control (PROS)
```

`arm-none-eabi-addr2line -e bin/cold.package.elf 0x38006ea` → **`strlen`**. (0x38006ea is
`cold_addr` 0x3800000 + 0x6ea, i.e. inside the cold package's libc, reached from hot code.)

**Cause:**

```cpp
emitf("build hash : %s", diag::compiledBuildHash());   // WRONG
```

`compiledBuildHash()` returns **`std::string_view`**, not `const char*`. Passed through varargs,
`vsnprintf` reads the view's raw bytes (pointer + size) as a `char*`, gets a garbage pointer, and
faults inside `strlen`.

**Why nothing caught it — the part worth keeping:**

- **The compiler was silent.** `std::string_view` *is* trivially copyable, so the usual
  "cannot pass non-trivially-copyable type through `...`" error does not fire, and
  `__attribute__((format(printf, …)))` did not flag the class type either. Compiled clean at
  `-Wall -Wextra`.
- **Every gate stayed green** — host suite (1,523,871 assertions), ARM header gate, six doc gates,
  zero non-`liblvgl` warnings. **None of them execute this translation unit.** `src/` is not in the
  host build and the ARM gate compiles *headers only*. This binary's first execution WAS its first
  test, exactly as flagged when it was handed over.

**Fix** — `%.*s` with the explicit size, and it now also honours `build_info.hpp`'s **loudness
contract**, which the original line silently broke: empty means MISSING and must render as an error,
never as a plausible-looking placeholder.

```cpp
const std::string_view hash = diag::compiledBuildHash();
if (hash.empty()) emit("build hash : [ERROR] MISSING ...");
else              emitf("build hash : %.*s", static_cast<int>(hash.size()), hash.data());
```

**Generalisable landmine, added as a comment at the site:** never pass a `std::string_view` (or any
class type) to a `printf`-family `%s`. In this tree that is a live hazard because
`compiledBuildHash()` is the natural thing to print in any session header, and it returns a view.

**Also observed:** after the abort the brain dropped off USB entirely (`no /dev/ttyACM*`), so the
fixed build could not be pushed until it was power-cycled. Worth knowing before a bench session —
a crash costs a reboot, not just a re-run.

### 13.12 Pre-upload audit — a second crash-class bug found BEFORE it reached the robot

Team lead's call: re-check everything before recompiling. It paid.

**FOUND: the touch menu was unusable, and it would have looked like a hang.**
`screen.h` states the touch status "will be **released by default if no action was taken**" — so
`touch_status == E_TOUCH_RELEASED` is *also the at-rest value*. Consequences of building on it:

- `waitForTouch()` returned **immediately, with nothing touched**. "TOUCH TO RETURN TO MENU" and the
  screen-full pause would both flash past, so results scrolled away unreadably.
- The menu polled the same value and hit-tested the **stale x/y that ride with it**, so after the
  first tap it would re-launch that same test forever.

Fixed by edge-detecting on **`release_count`**, a monotonic counter — an unambiguous "a new tap
happened" regardless of how the enum reads. *(Worth recording: the vendored docs name the values
`E_TOUCH_EVENT_RELEASE/PRESS` while the enum spells them `E_TOUCH_RELEASED/PRESSED`, and their
doc-comments are transposed. Not a value to build behaviour on.)*

**Verified on the host rather than reasoned about.** Button geometry was compiled and asserted
off-robot: all six boxes inside 480×272, no overlaps, every box centre hit-tests to its own button,
and `(0,0)` and the title strip hit nothing — the last two mattering because a stale coordinate must
not land on a button. **PASS, 0 problems.**

**Four more, each small, none of which the compiler would say:**

1. `kScreenLines` 19 → **14**, and both prompts moved to explicit **pixel** positions. A line index
   past the font's last visible row draws off-screen and the prompt vanishes silently.
2. Button labels are drawn pen-on-eraser, so the eraser is now set to the button fill — otherwise
   every label carries a black box.
3. The IMU catch path restores `g_screenActive`; the rotate test suspends the scrolling log, so a
   refusal would have reached serial and the SD card but **never the panel**.
4. Re-running test 2 constructs a fresh `ProsImu`, so `calibrateStarted_` is false and HA-05's
   second-calibrate precondition does not fire — the physical IMU **is** re-zeroed every run. That is
   right for a repeatable rotate test and is the exact opposite of what a competition binary must do.
   Now announced on screen instead of happening silently.

**Audited every `%s` in the file.** All arguments are `const char*`, `char[]`, or literals; the one
`string_view` goes through `%.*s`. Clean under
`-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wformat=2`.

### 13.13 ROOT CAUSE OF BOTH: `src/` had no warning flags at all

`common.mk` sets only `-Wno-psabi`, and `Makefile:15` was a bare `WARNFLAGS+=`. So while every
library header is compiled `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror` by CI's
ARM gate, **`src/main.cpp` and `src/bench_r3a.cpp` were compiled with essentially nothing** — and
`src/` is not in the host test build either. Two gates, neither covering the one file that runs.

Now `WARNFLAGS+=-Wall -Wextra -Wformat=2`, deliberately **without** `-Werror`: in the bench build the
X-drive helpers are legitimately unused, and those three `-Wunused-function` warnings are correct and
useful — they are how you can *see* that the invented wiring is genuinely dead code.

**Stated so nobody trusts it further than it goes: `-Wformat=2` did NOT catch the `string_view` bug.**
It is trivially copyable, so it passes through varargs silently. That hazard is held by a comment at
the call site, not by the compiler. **The real structural gap remains open: no gate executes
`src/`.** The button geometry is now host-checkable in principle; wiring it into `test/` is
unclaimed work.

### 13.14 Two usability findings from the first real session, both from the team lead

**(a) "Make it clear that it updated."** A silent `pros upload` failure (the brain had dropped off
USB) left the OLD binary in slot 3, so the same crash reproduced after a "fix" that was never
actually on the robot. Nothing on screen distinguished the two builds. Added `kBuildStamp` =
`__DATE__ " " __TIME__`, evaluated when this TU compiles, shown **three** places: a 2-second boot
splash, a permanent yellow line on the menu, and the serial/SD banner. The git hash identifies the
COMMIT; this identifies the BUILD, and only the second answers *"did my upload land?"*

**(b) "SD logging says off but the card is in."** The message was **true but useless**, and that is
the real defect: `ProsBlockSink` calls `fopen` ONLY when `usd_is_installed()` is nonzero, and
`isOpen()` reads false in **both** cases — card-not-detected and file-open-failed — which are
different problems with different fixes.

Added `probeSdCard()`, staged so each step names its own failure:

1. `usd_is_installed()` — if 0, VEXos cannot see the card at all: **FAT32 (not exFAT)**, reseat, and
   **power-cycle**, since VEXos mounts at boot and a card inserted mid-run is never picked up.
2. `fopen("/usd/probe.txt","wb")` — reports **`errno`** on failure. Card visible but not writable.
3. `fwrite` + `fflush` — proves an actual byte reached the medium.

It runs at startup **before the census**, so a card problem is the first thing seen, and it is also
menu item 5 for re-running after a fix.

Log filename shortened `r3a_bench.txt` → **`r3a_log.txt`** (8.3-safe). Defensive only: the vendored
headers document FAT32 as a requirement but say nothing about name length, and ruling it out is free.

**Menu grew to 7 items, so the button grid was re-verified on the host** — 4 rows at 48 px ends at
y=269 inside a 272 px panel; all seven boxes in bounds, no overlaps, every centre hit-tests to its
own button. **PASS, 0 problems.** Catching that off-screen row on the host rather than on the robot
is the second time the host geometry check has paid for itself.

---

## Phase 14 — FIRST REAL READINGS. The instrument ran, and the register is wrong.

Captured over USB serial, 2026-08-18, build stamp `Aug 18 2026 18:49:31`, hash
`v0.1.1-254-g78673bd-dirty`. **RAW, uninterpreted, pasted before analysis** — per the Phase 4 rule.

```
1. usd_is_installed()        : 1
   >> card IS detected.
2. fopen(/usd/probe.txt,wb) : FAILED
   >> errno=6.

port  type            our hypothesis
   1  MOTOR             (code 2)
   2  MOTOR             (code 2)
   3  IMU               (code 6)
   4  MOTOR           IMU(hyp)  (code 2)
   9  RADIO             (code 8)
  10  MOTOR             (code 2)
  11  MOTOR           RIGHT(hyp)  (code 2)
  12  MOTOR           RIGHT(hyp)  (code 2)
  13  MOTOR           RIGHT(hyp)  (code 2)
  14  MOTOR           RIGHT(hyp)  (code 2)
  15  MOTOR           LEFT(hyp)  (code 2)
  16  MOTOR           LEFT(hyp)  (code 2)
  17  MOTOR           LEFT(hyp)  (code 2)
  21  ADI-EXPANDER      (code 12)
motors found: 11   (hypothesis expects 8: 4 left + 4 right)
```

### 14.1 FINDING — the IMU is on port **3**, not port 4. `HA-111` is WRONG.

The register records `IMU 4` from 2026-08-13 and **that is not what is on this robot**: port 3 reports
`E_DEVICE_IMU`, port 4 reports `E_DEVICE_MOTOR`. Every IMU entry (HA-02/03/04/05/23/108/109/110) was
therefore unreachable by the shipped hypothesis, and `reportImu()` would have refused port 4 as
not-an-IMU. **The staged design paid for itself**: the census constructs nothing, so a wrong port map
produced a readable table instead of a boot fault.

### 14.2 FINDING — port **18 is EMPTY**. The 8-motor symmetric drivetrain does not exist.

`§9.1` recorded port 13 as mechanically repaired and concluded **"8 motors, 4 per side — symmetric"**,
retracting §5.3's asymmetry caveat. The measurement says **13 IS live and 18 IS NOT**:

| Group | §9.1 expected | MEASURED |
|---|---|---|
| 11/12/13/14 | 4 motors | **4 motors ✓** |
| 15/16/17/18 | 4 motors | **3 motors — 18 absent** |

So **the 4-vs-3 asymmetry never went away; it moved sides.** §9.1's symmetric conclusion and its
retraction of the "will not drive straight under an open-loop symmetric command" caveat are both
**withdrawn** — recorded rather than overwritten, exactly as §9.1 itself withdrew §6.4.

**This lands directly on R3b piece 1:** the motor group must handle **unequal counts per side**, not
merely N-per-side. And §8.4's per-side ratio question now has a per-side *count* question beside it.

### 14.3 FINDING — the ADI expander on port **21 is CONFIRMED**. `HA-120` closes.

The 2026-08-13 report claimed an expander read from registry index 21 — outside PROS's documented
0–20 range — and §5 flagged that its **absence** would correct that report. It is **present**:
`E_DEVICE_ADI (code 12)`. The earlier report stands, and scanning port 21 deliberately is what
proved it.

### 14.4 FINDING — 11 motors, not 8. Four are unaccounted for.

Motors on **1, 2, 4, 10** are outside both drive groups. These are the mechanism motors batch-1 item
**B1.5** asks about, and they now have measured ports rather than a question. Port 9 is `RADIO`,
which is expected.

### 14.5 SD CARD — detected, but **not a FAT32 filesystem**. `errno 6 = ENXIO`.

Step 1 passed (`usd_is_installed() = 1`) and step 2 failed with **`errno=6`**. PROS's own header
documents that value for this subsystem as **`ENXIO — drive number is invalid or not a FAT32 drive`**
(`include/pros/misc.h:818`). So the hardware is fine and the **filesystem is not** — almost certainly
**exFAT**, which is what modern OSes choose by default for cards ≥64 GB.

**Fix: reformat the card as FAT32.** HA-122's first belief (`usd_is_installed()` is a reliable 1/0
probe) is **CONFIRMED**; its second and third (the `/usd/` prefix, `fflush` durability) remain
untested because no file has opened yet.

*(The staged probe is the reason this took one run. `isOpen()` alone said "off" and would have sent
somebody to check the card was seated — which it was.)*

### 14.6 The panel layout was wrong, and the host check passed against the wrong constant

The bottom menu row rendered off-screen. The panel is 480x272, but **VEXos reserves the top strip for
its status bar, so usable height is ~240** — the host geometry check asserted against 272 and
returned PASS. **The check was right; its constant was wrong.** Re-verified at `kUsableH = 236`: all
seven buttons span y=46..226, no overlaps, every centre hit-tests to its own button. Both touch
prompts were also at y≥246 and therefore invisible; they are now positioned relative to `kUsableH`.

**Lesson worth keeping: a host check of a hardware-facing constant is only as good as the constant,
and it will report PASS with total confidence either way.**

---

## Phase 15 — PHASE 14 IS RETRACTED. It was an off-by-one, and HA-120 had predicted it.

Team lead: *"no there definitely is a number 18."* Correct — and chasing that falsified **every
device finding in Phase 14**.

### 15.1 The error

`apix.h` on `registry_get_plugged_type`:

> *"Returns the type of the device plugged into the **ZERO-INDEXED** port … \param port The V5 port
> number from **0-20**"*

The Phase 13 census looped `1..21` as though those were physical ports. **Registry index i is
physical port i+1**, so every reading was shifted one port, physical port 1 was never scanned at all,
and index 21 — outside the documented range — returned a garbage device.

**HA-120 had already called for exactly the scan that was not written:** *"Settle (R3, runbook step
17): the brain's Devices screen **and a corrected 0–20 registry scan**"* — the entry exists BECAUSE
the 2026-08-13 expander sighting came from index 21. The register named the trap and the trap was
walked into anyway.

### 15.2 Corrected reading (build `Aug 18 2026 19:01:03`)

```
PORT  idx  type            our hypothesis
   1    0  MOTOR             (code 2)
   2    1  MOTOR             (code 2)
   3    2  MOTOR             (code 2)
   4    3  IMU             IMU(hyp)  (code 6)
   5    4  MOTOR             (code 2)
  10    9  RADIO             (code 8)
  11   10  MOTOR           RIGHT(hyp)  (code 2)
  12   11  MOTOR           RIGHT(hyp)  (code 2)
  13   12  MOTOR           RIGHT(hyp)  (code 2)
  14   13  MOTOR           RIGHT(hyp)  (code 2)
  15   14  MOTOR           LEFT(hyp)  (code 2)
  16   15  MOTOR           LEFT(hyp)  (code 2)
  17   16  MOTOR           LEFT(hyp)  (code 2)
  18   17  MOTOR           LEFT(hyp)  (code 2)
motors found: 12
```

### 15.3 Every Phase 14 device finding is withdrawn

| Phase 14 claim | Truth | Status |
|---|---|---|
| 14.1 IMU on port 3; `HA-111` wrong | **IMU is on port 4.** `HA-111` was RIGHT | **WITHDRAWN** |
| 14.2 Port 18 empty; drivetrain is 4-vs-3 | **Port 18 has a motor. 8 motors, 4 per side** | **WITHDRAWN** |
| 14.3 ADI expander on 21 CONFIRMED | **No expander anywhere in 1–21.** Index 21 is out of range | **WITHDRAWN** |
| 14.4 11 motors; mechanisms on 1/2/4/10 | **12 motors; mechanisms on 1, 2, 3, 5** | **WITHDRAWN** |
| 14.5 SD `errno 6` | unaffected — different API | **STANDS** |
| 14.6 panel usable height ~240 | unaffected — observed directly | **STANDS** |

**Consequences, all of them restorations:**

- **`HA-111`'s IMU port is CONFIRMED, not corrected.** The IMU is on 4, as recorded.
- **`§9.1` STANDS.** Port 13 is live, port 18 is live, the drivetrain is **8 motors, 4 per side,
  symmetric**, and §14.2's withdrawal of §9.1 is itself withdrawn. R3b piece 1 needs **N-per-side**,
  not unequal counts — §14.2's extra requirement is dropped.
- **`HA-120` resolves to NO EXPANDER.** §5's stated criterion — *its absence corrects the 2026-08-13
  report* — is met. **The R1a expander sighting was an out-of-range read, and this is the second
  time that index has produced a phantom.**
- **`B1.5` is answered by measurement.** It asks what the motors on *"ports 1, 2, 3, 5"* drive, and
  the corrected census finds motors on exactly **1, 2, 3, 5**. The question was well-posed; only what
  they *drive* is still open, and that needs eyes on the robot.

### 15.4 The lesson, which is not "read the docs more carefully"

Phase 14 was **confident, specific, internally consistent, and wrong** — it produced a clean table,
three named register corrections, and a downstream design requirement for R3b. Nothing in the output
looked suspicious. It was caught by **one person who knew the robot saying "there is definitely an
18."**

Two things that would have caught it without that:

1. **The hypothesis was a falsification tool and its failure was not treated as one.** Phase 14
   showed `IMU(hyp)` beside a MOTOR and `LEFT(hyp)` against a missing port — the hypothesis
   disagreeing with reality on *four* counts at once. That pattern is far more likely to mean the
   *reader* is wrong than that four independent facts changed. **A wholesale disagreement should
   raise suspicion of the instrument first.**
2. **The register already held the answer.** HA-120 named the exact failure and prescribed the exact
   fix. It was read while writing the brief and not applied while writing the code.

The census now prints **PORT and idx side by side** so the mapping is visible in the output itself,
and stops at index 20 with a comment naming this incident.

---

## Phase 16 — the SD card works. `HA-122` is two-thirds settled, and a worry is retired.

Card reformatted **exFAT → FAT32** and re-run. Build `Aug 18 2026 19:01:03`, hash
`v0.1.1-256-g7a35529-dirty`:

```
sd logging : ON -> /usd/r3a_log.txt (overwritten each boot)

== SD CARD PROBE (HA-122) ==
1. usd_is_installed()        : 1
   >> card IS detected.
2. fopen(/usd/probe.txt,wb) : OK
3. fwrite/fflush            : 17 bytes, fflush=0
   >> CARD IS FULLY WORKING.
```

### 16.1 Root cause was the FILESYSTEM, and the laptop confirmed it independently

`lsblk` reported the card as **`exfat`** before formatting — which is what modern OSes choose by
default at this capacity — and PROS documents `errno 6 / ENXIO` for this subsystem as *"not a FAT32
drive"*. Two independent sources, same answer. `mkfs.vfat -F 32` fixed it.

### 16.2 `HA-122`: beliefs (1) and (2) CONFIRMED

| Belief | Status |
|---|---|
| (1) `usd_is_installed()` is a reliable 1/0 card probe | **CONFIRMED** — 1 with a card, 0 without, both observed today |
| (2) newlib file IO reaches the card only via the `/usd/` prefix (while `usd_list_files` FORBIDS it) | **CONFIRMED** — `fopen("/usd/probe.txt","wb")` succeeded WITH the prefix |
| (3) `fflush` is the strongest persist available (no fsync) | **PARTIAL** — `fflush` returned 0, but the DURABILITY half (yank the card after a flush, count what survived) is untested |

Register updated. Belief (3)'s durability test stays open and still belongs on the bench.

### 16.3 RETRACTED: the "V5 tops out at 32 GB" worry

It was flagged twice that a **116.5 GB** card might be rejected regardless of filesystem, and that a
smaller card was the higher-percentage move. **That was wrong, and measured wrong today:** a 116.5 GB
microSD formatted FAT32 is detected, opened, written and flushed without complaint.

Recorded because the advice was given twice and would otherwise have sent somebody to buy a card they
do not need. **The capacity was never the problem; the filesystem always was.**

### 16.4 Where R3a now stands

Settled by measurement: the port map (`HA-111` CONFIRMED — IMU on 4, drive 11–18, mechanisms on
1/2/3/5, 12 motors), the expander question (`HA-120` → **NO expander**), and `HA-122` (1) and (2).

Still open and needing hands on the robot, not code: the IMU sign convention (test 2 — nobody has
rotated the robot yet), per-side tooth counts (`B1.2`, still the highest-value measurement in the
chunk), wheel diameter and track width, what the mechanism motors on 1/2/3/5 actually drive
(`B1.5`), controller pairing, and the loop rate under load.

### 16.5 Two more layout bugs, and a silent-edit failure worth recording

Team lead asked to preview the UI before uploading, and specifically about **what happens after a
button is tapped** — *"that stuff was breaking before."* Building the preview found two real bugs and
exposed a process failure.

**(a) The rotate screen still used LINE INDICES.** `reportImu()`'s big-readout block drew with
`screen_print(font, line, …)` at indices 0, 1, 4, 5 and 8 — **while mixing MEDIUM and SMALL**. Line
height is per-font and unmeasured, so "line 4" had no defined meaning after two MEDIUM rows, and
index 8 could land on top of the `screenBig()` readout at y=150. Same class as the off-screen menu
row, **on the one screen whose wrong answer mirrors every turn the library will ever command.** Now
explicit pixels throughout, with the delta coloured green while rising and red if it falls, so the
HA-02 verdict is legible without reading a number.

**(b) `drawMenu()` was never actually restyled — the edit failed SILENTLY.** The header bar, build
stamp and palette described in the previous commit message were applied with `str.replace()` calls
that **did not match and did not assert**, so they no-oped. The committed binary still had the old
title drawn with a line index, hardcoded colours, and no build stamp on the menu — and the published
preview showed a design that **was not in the code**. Fixed, and every replacement in that pass now
carries an `assert`.

> **The lesson is the same one as Phase 15, in a different costume:** a confident, clean-looking
> result that nothing verified. There `PASS` came from a check with a wrong constant; here a commit
> message described edits that never landed. **Both were caught by a human looking at the actual
> thing, not by any gate.** `grep -c "screen_print(pros::E_TEXT"` is now 0 and is worth keeping as a
> spot check — no drawing in this file may use a line index.

**Preview published** as an interactive artifact: the panel at true 480×240 with real button
geometry, real colours, real captured content, pagination, and the rotate readout. It faithfully
reproduces everything except **font metrics**, which remain unmeasured — and building it surfaced
that `kScreenCols = 54` and `kLineH = 13` may be **mutually inconsistent**: a font wide enough that
only 54 columns fit would be ~14 px tall and overlap a 13 px pitch. Test 8 settles it.
