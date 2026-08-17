# R3a — platform validation on the tank bench bot

> **The chunk that walks the assumptions register against the robot that exists, and refuses to
> claim the clause it cannot close.**
>
> Predecessor: DEFECTS1 (and R1a/R1b for the adapters this validates).
> Live log: [`R3a-PROGRESS.md`](R3a-PROGRESS.md) *(opened as `R3-PROGRESS.md` before the split was
> ruled, then renamed — `briefing_status.py` pairs a `-PROGRESS.md` with a `-COMPLETED.md` by
> **exact stem**, so an `R3-PROGRESS.md` would have demanded an `R3-COMPLETED.md` and failed the gate
> at completion time, naming the wrong problem. Renamed early and deliberately rather than
> discovered late.)*
> Successors: **R3b** (first closed loop), **R3c** (holonomic + absolute reference).

---

## 1. Why this chunk is here, and why it is not the chunk `build-order.md` describes

`build-order.md`'s R3 entry — *"Day-one validation ⟵ closes M1 and M2's on-robot clause"* — was
written in June, when **no robot existed at all** and the planning documents believed the available
hardware would be *"an old tank/differential practice bot"* with *"not much on it"*. Its scope list
names four things this robot cannot do and one DoD clause that is not blocked on hardware at all.

R1 met the same problem on 2026-08-13 and split into **R1a + R1b, by consumer**. This chunk makes
the same kind of ruling, on a different axis, and §3 defends it.

**The reason R3 comes now rather than later is unchanged and still right:** every hour spent on
Phases T/G/H is an hour of code written against 54 unsettled beliefs about physical hardware, and
the register exists precisely so that debt is paid at a seam rather than discovered mid-season.

---

## 2. What exists to build on — read in the tree, not recalled

### 2.1 The adapters, and their honest limit

Fourteen `hal/pros` adapters exist (`include/shulib/hal/pros/`, R1a nine + R1b five). They are
host-proven against `test/pros_shim/`, which tests them **against our beliefs about PROS**
(HA-94…122) and can never test the beliefs. Seven beliefs were settled at the 2026-08-13 bench
session; **HA-98 is partial** and 54 R3-owned entries are open.

### 2.2 What the 2026-08-13 session actually left undone

`R1a-BENCH-SESSION.md` §5 is the honest inventory: of fifteen runbook steps, **step 0, 2, 3, 4, 5
were done and nine were not.** Two were impossible (no pods, no GPS); **the controller was never
paired at any point (`master=0 partner=0`)**, which alone blocked steps 10 and 11; and step 1 —
booting the shipped `main.cpp` — was never attempted because its port map does not match this
robot. Every heading-related assumption is still open, and heading is the term the `< 1.0°` target
lives or dies on.

**The controller is available tonight.** That is four register entries (HA-103, HA-104, HA-107,
HA-57) plus half of HA-112, none of which were reachable last time.

### 2.3 Three structural findings, measured before this brief was written

Each was found by reading the code R3 has to run, and each changes what this chunk can promise.
The raw measurements are in [`R3a-PROGRESS.md`](R3a-PROGRESS.md) §4; the sources are here.

#### FINDING 1 — shulib cannot command more than one motor per kinematic wheel

`motion/command_pipeline.hpp:146-152` maps kinematic-wheel index → motor-span index, **1:1**:

```cpp
const auto motors = deps.ctx->driveMotors();
for (int i = 0; i < wheels.size(); ++i) { ... motors[i]->setVoltage(...); }
```

`motion/motion.hpp:201-203` guards it with **`>=`**, so more motors than wheels is *accepted*:

```cpp
SHULIB_PRECONDITION(
    ctx->driveMotors().size() >= static_cast<std::size_t>(kinematics->wheelCount()),
    "MotionDeps: fewer drive motors than the kinematics has wheels");
```

`kinematics/tank.hpp:80` says the aggregation is someone else's job — *"How many physical motors sit
on each side is the HAL's business; kinematics sees one speed per side"* — and **no such HAL
facility exists**: `grep -rn "MotorGroup\|motorGroup\|MotorSide" include/ test/ src/` returns only
the vendored, unused `include/pros/motor_group.hpp`.

**Measured, with a negative control** (`R3a-PROGRESS.md` §4.1): this robot's real 7-motor drivetrain
plus `TankKinematics`, one `applyCommandPipeline` call at vx = 20 in/s and a 12.6 V pack —
**motors[0] and motors[1] get +4.4286 V; the other five get 0 V, keep whatever brake mode a previous
program left, and no fault is raised.** The negative control (the same call through a context holding
exactly two motors) commands both, so the instrument distinguishes the two cases.

This is not a constant to correct on a bench. It is a missing capability, and **every real VEX
drivetrain has 2–4 motors per side.** It is R3b's first deliverable.

#### FINDING 2 — no odometry is constructible on this robot, and the chain is hard, not soft

| Link | Source | What it demands |
|---|---|---|
| motion → pose | `localization/i_pose_source.hpp` | an `IPoseSource`; `Localizer` is the **only** implementor in the tree |
| `Localizer` | `localization/localizer.hpp:177` | `PilonsOdometry&` — a **concrete type**, not a seam |
| `PilonsOdometry` | `pilons_odometry.hpp:110-113` | **both** a `Role::Forward` and a `Role::Lateral` wheel, precondition-checked |
| `TrackingWheel` | `tracking_wheel.hpp:60-69` | an `hal::IRotation&`, and nothing else |

`grep -rn "IOdometry" include/ test/ docs/` → **no such seam exists.** There is no `IRotation`
adapter over `IMotor`, and a tank chassis has no lateral wheel to mount one on.

**So no control loop can close on this robot** — not because a number is unmeasured, but because the
object graph cannot be built. That is also the honest reading of a competitive gap: **LemLib does
drive-encoder odometry and shulib currently cannot**, and the thesis in briefing §1 is that we beat
LemLib. Worth knowing now rather than in September.

#### FINDING 3 — `RobotContext` requires a GPS, a tag source and a vision that most robots lack

`chassis/robot_context.hpp:62-73` precondition-checks **every** handle non-null, including
`gps`, `tags` and `vision`. `src/main.cpp:166-167` already ships **`FakeTagSource` and `FakeVision`
as stubs** with a comment saying so, so the precedent for stubbing an absent device exists — but a
*test fake* and an *absent-device null object* are different things, and only one of them belongs in
a competition binary. The library has fakes and no null objects.

Consequence for R3a: it **cannot construct a `RobotContext`** on this robot without shipping a fake
GPS, so it validates below the facade. Consequence for R3b: this needs a ruling, and it is a
one-line-per-seam ruling, not a redesign.

### 2.4 A29 confirmed in source, and measurable with a ruler tonight

`motion/odo_stall_check.hpp:82-84,153`:

```cpp
/// Drive wheel RADIUS (inches) — converts shaft radians to surface travel.
/// Stand-in geometry (3.25" wheel, 1:1 gearing — A4: HA-14).
units::Length wheelRadius{3.25 / 2.0};
...
const double spinTravel = meanShaftDelta * cfg_.wheelRadius.value();
```

`meanShaftDelta` comes from `IMotor::position()` — the **cartridge output shaft**. `ProsMotor`'s
constructor is `ProsMotor(std::int8_t port, MotorGearset gearset)` (`hal/pros/motor.hpp:96`): the
cartridge is selectable, an **external** reduction has nowhere to live. So `wheelRadius` is the only
field an external ratio can hide in, which makes the field's name and its own comment wrong on any
geared drivetrain — exactly DEFECTS1's `A29`.

**This is a tooth count and a caliper reading.** It is the cheapest high-value measurement in the
chunk and it needs no code at all.

---

## 3. THE RULING — R3 splits three ways, and the third way is the surprise

### 3.1 What R3 asks for, item by item, against the robot in the room

| R3 scope item (`build-order.md:1421-1427`) | On this robot |
|---|---|
| GPS field-cal axis oracle, **unskip it** | **impossible** — no GPS |
| The F5 on-V5 number match | **possible** for the kinematics half; the "swapping only `RobotContext`" half is blocked by Finding 3 |
| IMU conversion truth (heading, sign, wrap) | **possible** — IMU on port 4 |
| Tracking-wheel geometry vs `PilonsOdometry` | **impossible** — no rotation sensors |
| A push test, odometry vs a measured distance | **impossible** — no odometry is constructible (Finding 2) |
| Every remaining register entry, confirmed or corrected | **mostly possible** — see §4 |
| **DoD:** *a v2 auton runs on the robot* | **blocked on missing library code, not on missing hardware** |

### 3.2 The ruling

**R3 splits into R3a + R3b + R3c.** Three, not two, and the reason the third exists is the finding
this brief adds: **the register is not what blocks R3. The DoD is.**

| | Chunk | Gate | Owns |
|---|---|---|---|
| **R3a** | *this chunk* — platform validation on the tank bench bot | **the robot in the room** | the register walk, the validation entry point, the measured port map, IMU convention truth, real loop rate, PROS call latency, the F5 kinematics number match, the A29 gear ratio |
| **R3b** | first closed loop on a tank drive | **the robot in the room, plus R3a's numbers** | the multi-motor-per-side capability (Finding 1), an odometry path that does not require two dedicated pods (Finding 2), the absent-device ruling (Finding 3) — then **M1's DoD and M2's on-robot clause** |
| **R3c** | holonomic + absolute-reference validation | **a competition robot with pods, a GPS and a camera** | the GPS field-cal oracle, tracking-wheel geometry, the push test against `PilonsOdometry`, the H-bot geometry, the AprilTag physical set |

**Chunk count 44 → 46.** Nothing is renumbered: R3a/R3b/R3c follow the R1a/R1b precedent exactly.

### 3.3 Why three and not two — the argument, with the alternative I rejected

The obvious split, and the one the chunk request proposed, is **two: what this tank bot can settle
versus what needs a competition robot.** I am rejecting it, and the reason is that it puts the wrong
work behind the wrong gate.

Findings 1–3 are **library gaps, not hardware gaps.** Once they are closed, *this* robot — 7 drive
motors, an IMU, no pods, no GPS — can close a control loop, run a tank auton, and satisfy M1's
"identical numbers host and robot, swapping only `RobotContext`". A two-way split by hardware would
file "a v2 auton runs on the robot" under *needs a competition robot*, which is **false and
pessimistic**: it would leave M2's on-robot clause open until a robot that does not yet exist is
built, when in fact it is reachable on the bench with two additive pieces of code.

So the axis that carries information is not *which robot* but *what is missing*:

- **R3a — nothing is missing but the measurements.** Pure validation. No new capability. Every
  deliverable is a number or a corrected constant.
- **R3b — a capability is missing.** Two additive pieces of library code, both host-testable against
  the A2 plant and the existing fakes long before they meet the robot, both then validated on the
  robot in the room.
- **R3c — hardware is missing.** No amount of code helps; a GPS and two rotation sensors have to
  exist.

That is the same shape of reasoning R1 used ("split by consumer, not by convenience"), applied to a
different question, and it is auditable in the deviations table.

### 3.4 Why R3a must not simply do R3b's work as well

Three reasons, in order of how much they cost if ignored:

1. **A measurement taken through new, never-run code is not a measurement of the hardware.** If R3a
   built the motor group and then measured the drivetrain through it, a wrong aggregation and a
   wrong belief about PROS would be indistinguishable. R3a's numbers must come through code that has
   already been host-proven for a year (the adapters) or through no code at all (a ruler).
2. **R3b's design depends on R3a's answers.** The gear ratio decides whether the motor group needs a
   ratio parameter. The measured loop rate decides HA-123's re-expression and the stall window. The
   IMU sign decides whether the odometry path integrates the right direction. Building first is
   guessing first.
3. **The bench session is the scarce resource.** Hardware time should be spent on things only
   hardware can answer. Writing a `MotorGroup` at a bench with a robot on it is a waste of the
   robot.

### 3.5 What this ruling costs, stated plainly

**M1's badge does not flip at R3a, and M2's on-robot clause stays open.** `build-order.md` line 1349
says *"R1–R3 close M1's Definition of Done"*; measured against Finding 3, R3a cannot, because M1's DoD
is specifically *"identical numbers in a host test and on the V5, swapping only `RobotContext`"* and
this robot cannot build a `RobotContext`. **M1 closes at R3b.** That is a real slip and it is written
here rather than absorbed.

---

## 4. Scope

### 4.1 IN — the register worklist, classified entry by entry

62 register entries name R3 as an owner (47 `R3` + 5 `R1/R3` + 4 `R3/R4` + 3 `R3/R5` + 1 `T2/R3` +
1 `R2/R3`, plus HA-57 whose owner reads `R1` but whose settling measurement is runbook step 11 and
was never run). 7 are settled and 1 is partial, leaving **54 open**. Classified against the hardware
actually on the bench:

**(a) Settleable tonight — brain + IMU + controller + 11 live motors + a ruler. 20 entries.**

`HA-02` `HA-03` `HA-04` `HA-05` (IMU heading sign, unboundedness, gyro-z sign, tare discipline) ·
`HA-14` `HA-15` `HA-17`(tank half) (wheel diameter, cartridge ticks, real track width — **the A29
gear ratio lives here**) · `HA-18`(kinematics half) · `HA-23` (calibration window) · `HA-57` `HA-107`
(the 15-vs-19 LCD column conflict) · `HA-98` (**closes the partial** — units persisting *across
programs* was never tested) · `HA-102` (anchored 100 Hz) · `HA-103` `HA-104` (controller axes and
level-vs-consume) · `HA-108` `HA-109` `HA-110` (calibration call, gyro units, pitch/roll signs) ·
`HA-111` (**the port map — the keystone; nothing else runs until this is right**) · `HA-120` (the ADI
expander question, answerable off the brain's own Devices screen).

**(b) Settleable in part, with the unsettled half named. 9 entries.**

`HA-08` (unplug matrix: the IMU and motor halves yes, the GPS and rotation halves no) · `HA-19`
`HA-42` (brownout — attemptable on a low pack, and if it cannot be induced safely that is recorded as
*not measured*, never as *confirmed*) · `HA-32` (loop rate under **the load R3a can build**, which is
not the full stack — no localizer, no motion; the number is real and its scope is stated) · `HA-52`
(the geometry half from a ruler; the window and ratio thresholds need motion → R3b) · `HA-53` (cancel
safe state, measurable **open-loop**: command volts directly, then 0 V + Brake, and measure the coast
distance) · `HA-56` (the envelope's plausibility against this robot's measured top speed) · `HA-112`
(mapping *signs* yes with a controller; *"drivable"* needs R3b) · `HA-123` (**re-expressed as
`vMax × dt`** once the loop rate is measured).

**(c) Opportunistic — needs a loose sensor, an SD card, or a mechanism motor identified. 13 entries.**

`HA-11` `HA-16` `HA-105` (a rotation sensor on a cable settles all three, unmounted) · `HA-113`
`HA-114` `HA-115` (Distance: mm scale, the in-band 9999, confidence below 200 mm) · `HA-116` `HA-117`
`HA-118` (Optical: hue ranges, **proximity's unmeasured polarity**, sentinels) · `HA-119` `HA-121`
(ADI line semantics) · `HA-122` (SD, all three halves) · `HA-92` (only if one of the four non-drive
motors on ports 1/2/3/5 turns out to be a loaded lift).

Runbook steps 16–20 already cover these. **Each is opportunistic: if the sensor is not there, the
entry stays open and this chunk says which sensor it needs.** No entry is closed by reasoning.

**(d) Impossible on this robot — 13 entries, each named with the hardware it needs. → R3c.**

| Entries | Needs |
|---|---|
| `HA-01` `HA-06` `HA-07` `HA-09` `HA-10` `HA-31` `HA-106` | a **GPS** (and, for HA-01/HA-09, the field strip) |
| `HA-11`† `HA-12` `HA-13` | **rotation sensors mounted as tracking wheels** |
| `HA-68` `HA-69` `HA-70` | an **AI Vision camera** and a field with tags |
| `HA-55` | the **15″ H-bot** |

† `HA-11` appears twice on purpose: the *scale and non-wrapping* half is settleable on a loose sensor
tonight (c); the *"never wraps in a match"* half needs a pod actually mounted and driven.

### 4.2 IN — the code R3a must write

1. **A bench validation entry point for the measured tank bot.** The shipped `src/main.cpp` cannot
   boot on this robot: it constructs motors on 1/2/−3/−4 (**port 4 is the IMU**), rotation sensors on
   5/6 (5 is a motor, 6 is empty), a GPS on 9 and an IMU on 10 (both empty), and installs X-drive
   kinematics on a tank chassis. Every one of those throws a read-back precondition at boot.
   R3a delivers a **compile-time-selected** second wiring — the measured tank bot, `TankKinematics`,
   7 drive motors, IMU on 4, **no** GPS, **no** pods, **port 13 excluded** — that reads every device,
   prints **raw PROS value beside canonical value** on one line, commands open-loop voltages on
   request, and streams `TermSink`. The invented X-drive wiring is **preserved verbatim** and
   relabelled, because it is the only evidence of the 2026-08-12 whole-object-graph boot.
   **Which robot the binary is for must appear in the §18.5 session header** — a binary that cannot
   say which robot it believes in is how a wrong port map becomes a mystery.
2. **The loop-rate and call-latency measurement**, under the load this build can actually carry, with
   the scope of "load" printed next to the number.
3. **The F5 number-match print**: the six twists in `R3a-PROGRESS.md` §4.2, computed on the robot,
   compared against the host table to full precision.
4. **The register updates**, in the same commit as the measurement that justifies each (process rule 5).
5. **The runbook extension** — steps 21+ in `R1a-BENCH-RUNBOOK.md`, for the measurements R1a's
   fifteen steps do not cover: the gear ratio, the loop rate under load, PROS call latency, the
   cross-program unit persistence that leaves HA-98 partial, the port-16 diagnosis, and the F5 match.

### 4.3 OUT, with the chunk that owns it

| Out of scope | Owner |
|---|---|
| Multi-motor-per-side aggregation | **R3b** |
| Any odometry path that does not need two pods; the `IOdometry`/`IPoseSource` question | **R3b** |
| The absent-device null-object ruling (Finding 3) | **R3b** |
| Any closed loop, any motion under the library's own steering, any auton | **R3b** |
| M1's badge, M2's on-robot clause | **R3b** |
| The GPS field-cal oracle; tracking-wheel geometry; the push test; the H-bot; the AprilTag set | **R3c** |
| Noise, drift, latency distributions; the EKF re-verification | **R4** |
| kS/kV/kA and any PID gain | **R5** — and see landmine L2 |
| Fixing port 13 mechanically | **the team lead** — ruled: excluded, validated around (§6.4) |

### 4.4 Explicitly rejected

- **Rejected: rewriting `src/main.cpp` in place for the tank bot.** It would destroy the only
  artifact of the 2026-08-12 boot and would make the X-drive path unbuildable, which is the path a
  competition robot will use. Compile-time selection keeps both honest.
- **Rejected: stubbing an absent GPS with `FakeGps` to get a `RobotContext` built.** It would let
  R3a claim the M1 "swapping only `RobotContext`" clause on a graph containing a test double. The
  clause stays open and Finding 3 stays a finding. *(That `main.cpp` already ships `FakeTagSource`
  and `FakeVision` is the precedent I am declining to extend — it is a thing to rule at R3b, not a
  thing to lean on tonight.)*
- **Rejected: closing HA-52's threshold half by reasoning from the geometry.** The window and ratio
  are motion properties. A ruler gives the radii and nothing else.
- **Rejected: unskipping `gps_conversion_test.cpp`'s field-cal oracle with a "documented" mapping.**
  The oracle exists to be unskipped by a *measurement*. Unskipping it against PROS's documentation
  would convert an honest gap into a green test that proves nothing — the single worst outcome
  available in this chunk.
- **Rejected: any per-side averaging of encoder readings.** Port 16 under-reports its side-mates by
  an inconsistent ~20%; an average hides exactly the fault we are trying to characterize. **Every
  motor is logged individually, always.**

---

## 5. Load-bearing constraints, each with its reasoning

- **C1 — Do not validate a conversion using the same conversion** (briefing trap 1, six chunks and
  counting). Every canonical value printed must sit **beside its raw PROS value on the same line**,
  and the expected value must be hand-computed from a ruler, a protractor or a tooth count. The
  2026-08-13 session got this right and it is why its seven settled entries mean something.
- **C2 — One robot, once, is an observation.** Every entry settled tonight is stamped
  `measured-on-comp-bot 2026-08-13/17`. Not proof of portability, and the register must not read as
  though it were.
- **C3 — Never carry gains across chassis.** kS/kV/kA and every PID gain are mass-, friction- and
  geometry-dependent. **R3a measures no gain.** A measured-looking wrong number is worse than an
  honest placeholder.
- **C4 — `registry_get_plugged_type()` is ZERO-indexed (0–20); every device API is ONE-indexed
  (1–21).** Mixing them produced a confident report of two dead devices that were both healthy. No
  shipped adapter uses the registry. **Keep it that way** — including in any throwaway probe, because
  the throwaway probe is what got it wrong last time.
- **C5 — Frozen means frozen.** F1–F6 and F10 are LOCKED. Findings 1–3 all touch code near frozen
  surfaces; none of them is licence to change a frozen signature. The doc freshness gate fires
  **before** the signature pins, so it will name the wrong problem first.
- **C6 — A correction is the system working.** Expect entries to come back *corrected*, not
  confirmed. Each one is a defect the register predicted and localized to the HAL seam.
- **C7 — Under-claim.** `[~]` for partial, naming the owner. A pass reporting only wins has not
  looked hard enough.

---

## 6. Rulings this chunk makes

### 6.1 The bench build is selected at compile time, not at runtime

A runtime switch would still have to *construct* both object graphs, and the tank graph cannot be
constructed with a GPS while the X-drive graph cannot be constructed without one. Compile-time
selection is the only spelling where each graph is legal.

### 6.2 The validation entry point works below the facade, and says so

Finding 3 means no `RobotContext`, so no `Chassis`. R3a therefore validates the HAL, the four
conversion headers, the tick pacer and `TankKinematics` — which is exactly where the beliefs it is
settling live. **The banner must state that the facade is not in this binary**, so nobody reads a
successful bench run as a facade result.

### 6.3 HA-18 splits, and the split is recorded

The *kinematics* half (the same twist produces the same wheel numbers on ARM and on the host) is
settleable and gets settled. The *"swapping only `RobotContext`"* half is M1's DoD wording and is
blocked by Finding 3. HA-18 goes to **`[~]` partial with R3b named**, not to settled.

### 6.4 Port 13 — ~~excluded~~ **REPAIRED 2026-08-17; this ruling is SUPERSEDED**

> **The original ruling is struck, not edited away, because it was right when it was made.** It read:
> *"Port 13 stays mechanically dead. It is dropped from the drive map entirely, so the drivetrain R3a
> measures is LEFT 15/16/17/18 (4 motors) against RIGHT 11/12/14 (3 motors) […] this robot will not
> drive straight under an open-loop symmetric command."* The team lead's builder found and fixed the
> fault the same day, so its premise is gone.

**The drivetrain is now 8 motors, 4 per side, symmetric.** Consequences:

- **The "will not drive straight open-loop" caveat is RETRACTED** — it followed from the 4-vs-3 force
  imbalance and that imbalance no longer exists. *(A residual asymmetry may remain from the per-side
  **gearing** in the Calypso record — a different cause, which this repair does not address, and which
  §6.6 makes tonight's highest-value measurement.)*
- **Finding 1 gets worse:** 8 drive motors on 2 kinematic wheels means the shipped pipeline commands
  **2 and leaves 6 dead**. The §2.3 probe number was taken at 7 motors and understates it by one.
- **Port 16 is NOT covered by this repair** and is now the sole remaining drivetrain anomaly. 13 and
  16 sit on opposite sides under either mapping, and 16's ~20% under-report was measured against its
  *own* side-mates, so neither the repair nor a per-side gearing difference can explain it. §6.5's
  discriminators stand unchanged, and "expect all similar readings" must be **verified, not assumed** —
  a drive motor that under-reports travel biases odometry quietly and silently.

### 6.6 The Calypso branch supplies PREDICTIONS, not measurements (added 2026-08-17)

`origin/calypso` holds a LemLib project for a robot called Calypso with drivetrain constants
**measured on hardware 2026-04-21/22** — the only prior measurement of a team robot anywhere in this
repository's history. Full raw extract in [`R3a-PROGRESS.md`](R3a-PROGRESS.md) §8, and the port-inventory
reconciliation in §9.2 concludes **Calypso and the bench bot are the same physical robot, rewired**
(12 motors, two exact drive sets of four, matching manipulator pairs, and one self-consistent rewire —
the IMU taking port 4 and displacing the conveyor's second motor to port 5 — explaining every
difference).

**Nothing from it is adopted.** It was measured by a different codebase, open-loop, with **no IMU and
no encoders**, using a tape measure for distance and a **phone compass (±3° noise floor)** for angle.
It is used here in exactly one way: to turn tonight's blank ruler measurements into **falsifiable
predictions**, which is a strictly better instrument — a prediction that fails is informative, and a
blank measurement cannot disagree with anything.

| Prediction from Calypso | shulib's current stand-in | If Calypso is right |
|---|---|---|
| wheel diameter **3.0″** | 3.25″ (HA-14) | **HA-14 is wrong**, and so is `OdoStallCheckConfig::wheelRadius` |
| track width **15″** | 7.0″ X-drive radius (HA-17) | tank geometry replaces it wholesale |
| **BLUE** cartridges (600 rpm) | GREEN per the August session | **the two records contradict by 3×** — see §6.4's sibling check |
| **per-side gearing difference**, right geared taller, `RIGHT_DRIVE_BIAS = 1.08` | no gear ratio exists anywhere | **A29 needs a PER-SIDE ratio**, which is a change to how a drivetrain is *described*, not a new constant |
| ~58.4 in/s forward, ~410 °/s turn | 60 in/s, 6 rad/s (HA-50) | the invented budget is the right order — worth knowing, not a confirmation |

**Three things must not cross over, and the reasons are in `R3a-PROGRESS.md` §8.6:**
`RIGHT_DRIVE_BIAS` (a ±127-scale voltage bias whose job heading feedback subsumes — Calypso's own
`config.hpp` says so), the LemLib PID gains (its own comment calls them placeholders), and the turn
constants (phone-compass grade against a **< 1.0°** hard target).

**One opportunity worth chasing before anything else:** Calypso declares
`pros::Rotation horizontal(21)` and `vertical(-20)` with 1.5″ tracking wheels — specified, never
wired. **If those two sensors physically exist, R3b's odometry blocker largely evaporates and a
position-based auton becomes reachable without a purchase.**

### 6.5 Port 16 gets a diagnosis attempt, not a repair

Its ~20% under-report at an inconsistent ratio (~85% and ~73% across two spins) is not a gear
difference. Two cheap discriminators, both in the runbook extension: **(a)** roll the robot in a
straight line on the floor by a measured distance and compare all four left encoders — a slip shows
up under rolling load, a failing encoder shows up always; **(b)** command port 16 alone at low
voltage off the ground — if its wheel turns while its side-mates stay still, it is mechanically
decoupled like port 13. Whatever it reports, **the finding is recorded and the entry stays open if
undiagnosed.**

---

## 7. Test requirements

R3a's evidence is mostly *measurements*, not tests — but every code change it makes is host-tested,
and the register updates are gated:

- **The bench wiring must compile under the ARM gate.** It is a new TU in `src/`, so it is not
  swept by the header glob; it must be compiled explicitly and the command recorded.
- **The X-drive wiring must still compile**, proving the compile-time selection did not silently
  delete a path.
- **Every corrected constant that has a host test gets that test updated to the measured value**, and
  the update must **fail against the old value** — otherwise the test does not depend on the number.
- **Mutations, line-count-neutral** (`docs/api/` records declaration line numbers; a line-shifting
  edit fails `check-fresh`, the binary is never relinked, and running it reports the PREVIOUS
  result). Required:
  - **M1** — flip a measured IMU sign in the *conversion*; the conversion tests must go red.
  - **M2** — set the measured gear ratio / wheel radius wrong by the measured factor; whatever test
    pins the geometry must go red. **If nothing goes red, that is a hole and it is the chunk's most
    valuable output** — geometry errors are trap 1's favourite shape (C3's mutation M4 exists because
    they cancel end-to-end through a shared plant).
  - **M3** — break the raw-beside-canonical print so the two columns come from the same source. If no
    check notices, the print is decoration and C1 is unenforced.
- **A mutation that stays GREEN is a hole in the suite and the most valuable thing to find.** Do not
  quietly re-spell it until it goes red.

---

## 8. Definition of done

- [ ] The split ruling made here, with reasoning, and reflected in `build-order.md` **including the
      deviations table**; nothing renumbered
- [ ] A validation binary that **boots on this robot** and prints raw-beside-canonical for every
      device, with the robot identity in the session header
- [ ] `HA-111` settled with the measured port map, and `src/` carrying it
- [ ] Every group-(a) entry settled with its measured value **or corrected with what it actually is**
- [ ] Every group-(b) entry settled in part with **the unsettled half named**
- [ ] Every group-(c) entry either settled or left open **naming the sensor it needs**
- [ ] Every group-(d) entry left open **naming the hardware it needs**, and counted
- [ ] The A29 gear ratio measured, and `OdoStallCheckConfig::wheelRadius`'s name/comment either
      corrected or the defect re-registered with the measured ratio in it
- [ ] Real loop rate measured under the load this build carries, with the scope of "load" stated, and
      **HA-123 re-expressed as `vMax × dt`** if the number supports it
- [ ] `HA-98`'s persistence half tested across two programs — the partial closed or corrected
- [ ] Port 16 diagnosed or explicitly recorded as undiagnosed with what was tried
- [ ] Port 13's exclusion recorded, and the 4-vs-3 asymmetry carried as a caveat on every number
- [ ] The GPS field-cal oracle **still skipped**, with "needs a GPS and the field strip" written at
      the skip
- [ ] Findings 1–3 registered so they cannot be lost, with R3b named
- [ ] All gates green; both guards; ARM over all headers **and** the new TU; the release gate
- [ ] Milestone badges moved only with cited evidence — **M1 does NOT flip; M2's clause stays open**
- [ ] The honest partial stated with a count
- [ ] Nothing pushed without asking

---

## 9. Documentation contract

- `docs/hardware-assumptions.md` — every measured entry updated **in the same commit as its
  measurement**; the status line's settled count and the per-chunk narrative both updated.
- `docs/internal/chunks/R3a-BENCH-SESSION.md` — the session record, in the shape of
  `R1a-BENCH-SESSION.md`: firmware versions, transcript, and per step the measured value, the HA-nn,
  and settled-true / settled-false / still-open.
- `docs/internal/chunks/R1a-BENCH-RUNBOOK.md` — steps 21+ appended (§4.2 item 5).
- `docs/internal/build-order.md` — the split, the three new entries, the deviations row, `Next:`.
- `docs/roadmap.md` — "you are here"; badges only with evidence.
- `docs/internal/PROJECT-BRIEFING.md` — §13 retargeted, §15's "what this robot CAN settle" list
  corrected against what was actually settled, and **the §3 governing constraint left standing**.
- The generated briefing block regenerated at the **end**, after the last build.

---

## 10. Landmines

- **L1 — "It ran on the bench" must not drift into "it works on a robot."** Defended in six places.
  R3a makes it *more* tempting, because a successful bench run feels like a working robot. Nothing
  R3a produces licenses editing that sentence anywhere in the tree.
- **L2 — Do not tune anything.** No gain, no threshold, no tolerance. R3a measures. Tuning against a
  single robot's numbers is R5's, and gains do not transfer.
- **L3 — The doc-gate deadlock is once per BUILD, not once per chunk.** `shulib_tests` depends on
  `shulib_doc_gates`, and `briefing_status.py check` derives suite state from the binary that already
  exists, so a red or stale binary blocks the rebuild that would fix it. Escape:
  `python3 tools/briefing_status.py generate`, then build. **This fired on this chunk's very first
  action** (creating the progress log made R3 an interrupted chunk) — see `R3a-PROGRESS.md` §2.
- **L4 — Mutations must be line-count-neutral**, and one that changes a *rendered declaration* also
  needs `api_doc_tool.py generate`.
- **L5 — The suite's assertion count is not a function of the committed tree.**
  `test/pros_adapter_fence_test.cpp` walks `include/shulib/` **on disk**. Check `git status` for stray
  untracked headers before suspecting code. *(Relevant here: `stash@{0}` holds season content that
  previously dropped six headers into `include/shulib/seasons/` and moved the count by 6.)*
- **L6 — `pros terminal` needs a real TTY.** Under automation wrap it:
  `script -qec "pros terminal" /dev/null`. A raw read of `/dev/ttyACM1` yields nothing — the user port
  is framed, not plain serial. `ttyACM0` is system, `ttyACM1` is user.
- **L7 — When a program uploads and produces no output, check `ls /dev/ttyACM*` before suspecting the
  code.** A dropped USB cable cost the last session hours and produced the false theory "linking
  shulib breaks the binary", which only a control test disproved.
- **L8 — Two known on-robot blockers are config, not toolchain:** stale soft-float firmware /
  `liblvgl.a`, and `CXX_STANDARD=gnu++26`. Both small; host tests unaffected.
- **L9 — PROS resolves adapter includes with `-iquote` only.** Quoted includes are mandatory in
  anything under `hal/pros/`; angle brackets compile everywhere except the build that matters.
- **L10 — `cp -a` preserves mtime**, so restoring a mutated file leaves `make` believing it is up to
  date and the next run reads a stale binary. Bump mtime on restore.
- **L11 — Do not push.** R3a changes what the library claims about hardware, and that claim goes
  public on the next release. Ask.

---

*Brief written 2026-08-17, before execution, from a clean tree at `14128c0`. Every source citation in
§2 was re-read in the tree; every number in §2.3 and §2.4 was measured by
`scratchpad/probe_r3.cpp` and is logged raw in [`R3a-PROGRESS.md`](R3a-PROGRESS.md) §4 — including one
instrument error of my own that nearly became a finding (§4.0).*
