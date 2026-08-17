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

*(Appended below as the session proceeds. Raw readings first, interpretation after.)*
