# R3d — Drivetrain self-calibration + persistence + boot verification

> **Chunk spec for all of R3d, and the execution brief for PART 1** (the calibrator, host-only).
> Parts 2 and 3 get their own briefs when their turn comes (§10). Written 2026-09-17 against tree
> `6ceb244`, with the robot **apart** (robot two is not built this week — the team lead, 2026-09-17)
> and R3b **open** (its DoD is a v2 auton under the library's steering; nothing of Parts 1–3 has
> run on hardware). The build-order entry is the spec; this file carries what the code says now,
> the rulings the entry left open, and the order.
> Live log: [`R3d-PROGRESS.md`](R3d-PROGRESS.md), created FIRST, appended as work proceeds.
> **Executor model: `opus`** (RESUMING §2). **Do not commit.**

---

## 0. Why this chunk, why now, and why it does not wait for R3b's bench run

**The argument that made R3d** (team lead, 2026-09-10; build-order deviations table): almost every
number a human is asked to type about a drivetrain is discoverable on the robot. Ports come from the
device registry; polarity from a hand push; sides and the front from short pulses with the IMU
watching; the distance scale from one push along a tape; the track width from a spin in place
comparing IMU yaw to encoder yaw. Only the **cartridge** (physically unobservable — the rotor turns
the same for every insert and the firmware divides by what it was told), the **drivetrain kind**
and the **front** must be stated. LemLib makes a team type all of it; a robot that describes itself
serves the accessibility pillar and removes the invented-constant defect class (HA-111) at its
source.

**Why now, with R3b still open.** R3b's DoD waits on a robot. R3d's *gate* is R3b's motor group and
odometry seam (build-order R3d entry: "Gated on: R3b pieces 1–2 … and an IMU on the robot") — and
both are in the tree since `3cf03d0`. The first bench day therefore reads, in order: calibrate →
"shulib Teleop" wheels-up → the floor → a short auton → R3b and M1 close on the log. The two
numbers the table is waiting for (HA-125, HA-126) never get typed: the robot measures them. The
typed path **stays as the fallback** (§3 T9) so bench day is not blocked if the calibration meets a
hardware surprise. This ordering — R3d's host half before R3b's bench half — is a scheduling
decision, recorded in the deviations table, not a change to any dependency.

**The risk, stated.** The odometry seam has never run on hardware, and R3d is built on it. If bench
day finds a unit or a sign defect in `DriveEncoderOdometry` or `MotorGroup`, R3d inherits it. R3d's
own push and spin are the same seam exercised a second way — so they would *find* such a defect —
but debugging two chunks in one bench session is harder than one. The mitigation is the fallback
above and Part 3's order (Teleop's wheels-up run is the first thing powered on bench day, before the
calibrator moves anything on the floor).

---

## 1. What exists to build on — read these files, they decide the design

| Piece | Where | What matters for R3d |
|---|---|---|
| The sign-capture logic (**R3d's seed**) | `src/bench_r3a.cpp:913-1140` MOTOR WATCH | Baseline every census motor, watch deltas live, a port with `|Δ| > 5°` MOVED; a whole-robot push in which EVERY table port moved is the only run that yields SIGNS; the panel ASKS "front first or back first?" and inverts on back-first (`:1077-1097`); a single-wheel spin maps a port to a wheel and yields no signs. In-memory for one power cycle "by design — persistence across boots is chunk R3d's" (`:914-915`). PROS calls directly; not library code |
| The group | `include/shulib/hal/motor_group.hpp` | Reads are the MEDIAN across members (`:147-156`, header: why not the mean — a dead port's frozen reading); THE GROUP NEVER NEGATES — a reversed member is a NEGATIVE PORT applied once in PROS (`:21-24`); every member PRESENT by contract, "no dead-port tolerance (that is shulib Drive's job until R3d)" (`:72-74`); `member(i)` exposes members (`:214`) |
| The odometry | `include/shulib/localization/drive_encoder_odometry.hpp` | Per side: `Δshaft_rad × inchesPerRadian` (`:145-151`); `lastHeadingDisagreement() = (dR − dL)/trackWidth − Δθ_imu` — "the measurement R3d's track-width calibration is built from" (`:37-47`, `:158-160`); constructor refuses invalid geometry (`:114-121`) |
| The one geometry type | `include/shulib/hal/drive_geometry.hpp` | `{wheelRadius, motorToWheelRatio}`, consumers use only `inchesPerRadian()` (`:52-54`); `fromDiameter(diameter, ratio)` is the human-measured constructor (`:58-61`); one object per side, both consumers |
| The typed input, today | `src/chassis_table.hpp` | Signed ports, cartridge, IMU port, `wheelDiameterIn`/`trackWidthIn`/`externalRatio` (0 = UNSET, refuses), `geometryProvenance`; helpers `tableDriveGeometry()` / `tableTrackWidth()` / `describeMissingForLibrary()`; PROS-free on purpose so `test/chassis_table_test.cpp` can break it. "R3d makes the signs discovered and persisted" (`:26`) |
| The composition root | `src/main.cpp:442-577` (`tanklib::TankRobot`) | Ten `ProsMotor`s from the SIGNED table entries with the table's cartridge; two groups; ONE `DriveGeometry` handed to the odometry AND the stall check; `TankKinematics{tableTrackWidth}`; refusal path `:825-847` (consistent → signed → complete → construct inside a try) |
| The IMU adapter | `include/shulib/hal/pros/imu.hpp` | `calibrate()` once, never blocks; `isReady()` only after calibration ends; heading CCW-positive radians, cumulative; never tares |
| The plant | `include/shulib/sim/drive_plant.hpp` | Velocity-driven, torque-free (`motor_model.hpp`'s honesty boundary); `attachCoupledMembers(wheel, extras)` writes ONE shaft into every member (`:296-313`); the plant reads its command from the PRIMARY member only (`:290-291`); `MemberFault::{None, Frozen, SignFlipped}` (`:150-156`) — SignFlipped negates READINGS only; `DrivePlantConfig::driveWheelDiameter` with 1:1 gearing baked (`:136-138`); the IMU reports TRUE heading; the tank kinematics carries the TRUE track width |
| The coupled-side monitor | `include/shulib/teleop/coupled_side_monitor.hpp` | Pure fight detector, `SideMonitorConfig` thresholds (HA-132/133), persistence counted inside; the group evaluates it once per tick |
| The dead-port policy | `include/shulib/teleop/drivetrain_degradation.hpp` | "≥ 3 answering per side and ≤ 2 dead in total" — pure; the drive program's. Part 2 reuses it |
| Persistence, write half | `hal/block_sink.hpp`, `hal/pros/block_sink.hpp`, `hal/fake/fake_block_sink.hpp` | Bytes verbatim, bool-valued, no throw; `/usd/` prefix owned by the adapter with an injectable mount root (host tests run the real fopen path against a temp dir). **There is no READ seam** — E1 built the blackbox to be read on a laptop. Part 2 adds the sibling |
| The record precedent | `include/shulib/diag/blackbox_format.hpp` | Magic + version + fixed width + little-endian, written byte by byte; refuse-don't-misread; **deliberately no per-frame checksum** (E1's trade; §3 T10 rules the opposite for a calibration record and says why) |
| The registry | `src/bench_r3a.cpp:114-121, :573-620` | `pros::c::registry_get_plugged_type(idx)` is ZERO-indexed: index i is PORT i+1 (the R1a-shape trap, recorded). No `hal/` seam and no shim entry exist; Part 2 adds both |

**Promises the tree has already made to R3d**, each of which this chunk must keep or explicitly
re-home: signs discovered and persisted (`chassis_table.hpp:26`); dead-port tolerance in the library
graph (`motor_group.hpp:73`, `main.cpp:461`, `:889`); the track width from the heading cross-check
(`drive_encoder_odometry.hpp:41`, HA-125); the per-side distance scale from a push along a tape
(master plan §Robots (2), HA-124, HA-126); the thresholds HA-132/133 "R3d/R4 measure"; the stuck/slip
detector on the cross-check (HA-131, "R3d/R4"). The last is **re-homed to R4** (§2, out) — it is a
detector, not a description of the drivetrain, and it needs real pushes to tune.

---

## 2. Scope

### Part 1 — IN (this brief)

1. **`hal::DrivetrainDescription`** — `include/shulib/hal/drivetrain_description.hpp`. The ONE
   representation of a drivetrain's discovered facts, produced by the calibrator (Part 1), by the
   table (Part 1, `tableToDescription()`), later read back from the SD card (Part 2) and — the build
   order's stated intent — by G1's `RobotBuilder` from a `.vexbot` profile. Fixed-width, allocation-
   free, PROS-free, designed to be serialized byte by byte in Part 2 (no pointers, no `std::string`,
   bounded arrays). Contents in §4.1.
2. **`hal::DriveGeometry::fromInchesPerRadian(units::Length)`** — the calibrated constructor (§3 T5).
3. **`calibration::DrivetrainCalibrator`** — `include/shulib/calibration/drivetrain_calibrator.hpp`,
   a new library directory. A tick-driven, allocation-free state machine over `std::span<IMotor*
   const>` candidates (every motor the registry found), their port numbers, one `IImu&`, one
   `IClock&`; the operator's answers arrive through a method, never a blocking read. Phases in §4.2.
   Owns no loop; raises no faults; PROS-free; host-proven against the A2 plant.
4. **Two additive plant extensions** (`sim/`, §3 T6), each bit-identical for every existing use:
   `MemberFault::Reversed` (a member wired backwards: command AND reading negated), and the coupled-
   train command model (a wheel whose members are commanded DIFFERENT volts spins at their mean; the
   same volts → the primary's value, exactly as today).
5. **`tableToDescription(const ChassisTable&)`** in `src/chassis_table.hpp` (PROS-free, tested in
   `test/chassis_table_test.cpp`): the second source of the one representation. Nothing else in
   `src/` changes in Part 1 — no program consumes the description until Part 2.
6. Tests with the required mutations (§5), the DoD (§6), the documentation contract (§7).

### Part 1 — OUT (with the owner)

- **Persistence** (the record format, the SD read seam, the pros/fake adapters) → **Part 2.**
- **Boot verification against the registry**, the registry seam and its shim entry → **Part 2.**
- **Dead-port tolerance in the library graph** (an absent `MotorGroup` member; the degradation
  policy in the library program) → **Part 2** (it is a boot-time question and belongs with the boot
  check).
- **The composition roots** — `PROGRAM=calibrate` ("shulib Calibrate") and "shulib Teleop" reading
  the description from the card with the table as fallback → **Part 2.**
- **Anything powered on a robot** → **Part 3** (the bench session; the worksheet's Station G; the
  register's settle lines; `docs/guide/` chapter on calibrating a robot, written when the procedure
  has been followed once).
- **A stuck/slip detector on the heading cross-check** (HA-131) → **R4.** Re-homed here.
- **Holonomic calibration** (X-drive strafe scale, the pseudo-inverse's per-wheel geometry) → spring,
  with R3c. `DrivetrainKind` carries `Tank` only and says so.
- **The thresholds' measurement** (HA-132/133) → **R4**, from a season of logs; Part 3 observes them.

### Explicitly REJECTED

- **Calibrating at boot, automatically.** The routines move the robot; a robot must not calibrate
  itself before a match (build-order R3d entry). The calibrator runs only in its own program, started
  by a person who stands clear. The competition program VERIFIES (Part 2) and never discovers.
- **Discovering the cartridge from the free speed.** Rejected on physics: the rotor speed is the same
  for every insert; `get_actual_velocity()` is already divided by the configured gearset, so a wrong
  belief is invisible from inside. Human-stated, as the entry says.
- **Skipping the hand push and finding polarity from a low-voltage pulse alone.** Rejected: a pulse
  on a coupled train with an UNKNOWN sign is the fight the coupled-side monitor exists to prevent (one
  motor stalled against four, `coupled_side_monitor.hpp:8-12`). The push is powerless and finds the
  sign first; only signed members are ever pulsed.
- **A separate "wheel diameter" and "ratio" in the calibrated description.** The push along a tape
  measures their PRODUCT and nothing can separate the factors; storing two invented factors for one
  measured number is HA-111's defect class in a new coat (§3 T5).

---

## 3. Rulings — each with the alternative and the reason

**T1 — During calibration the CALIBRATOR applies the captured sign; after calibration only the
adapter does.** The rule "one place for a sign, or it is applied twice" (`motor_group.hpp:21-24`)
holds, with the place moving once: in "shulib Calibrate" the adapters are constructed from the
registry with POSITIVE ports because no sign is known yet, so the calibrator commands
`sign[i] × V` to a member and reads `sign[i] × position()`. The description it produces carries the
sign INSIDE the port number (`-11`), and every later program constructs the adapter with that
signed port and never negates (the drive program's convention, `chassis_table.hpp:19-27`). Rejected:
reconstructing adapters mid-calibration once signs are known (a `ProsMotor` re-configures the device
at construction, `pros/motor.hpp:96-118` — a second construction on a live port is a new hardware
belief nobody has measured, and the host cannot model it).

**T2 — The front is stated by the push, exactly as MOTOR WATCH states it.** The operator pushes the
robot and then answers "FRONT first" or "BACK first" (`bench_r3a.cpp:1077-1097`); back-first inverts
every sign. Nothing here decides which end is the front — the build team's decision is expressed by
the direction they push. Rejected: asking for the front as a port ("15/16 end") — a port is a fact
about the table, not about the robot, and the team lead's rule is that only cartridge, kind and front
are human-stated, with the front stated physically.

**T3 — Side assignment: pulse ONE signed member forward; the IMU's yaw sign names the side.** On a
tank, one side driving forward alone pivots the robot about the other side: the LEFT side forward
turns the robot CLOCKWISE, which is NEGATIVE yaw in shulib's CCW-positive canonical frame
(`hal/imu.hpp:16-19`); the right side forward is positive. The pulse also CONFIRMS the sign from the
push: the pulsed member's own encoder must advance positive under `sign × V`; a negative advance is a
contradiction and the run is refused with the port named, never averaged away. Every drive port is
pulsed, one at a time, with rest (yaw rate and every velocity under a floor) between pulses; the
result must be exactly two non-empty sides or the run is refused. On a coupled train the four unpowered
mates are back-driven by the one pulsed member — slow, but the yaw sign is unambiguous; the plant's
coupled-train command model (T6) is what lets the host prove this. Rejected: pulsing a whole
"assumed side" (there is no side to assume before this step); reading the side from the port's
position on the brain (a wiring convention, not a measurement).

**T4 — The per-side distance scale is `tapeDistance / median(Δshaft of that side's members)`.** The
median, not the mean, for the same reason the group reads the median (`motor_group.hpp:26-37`): robot
two's port 18 travels ~20 % short on a push (HA-130) and must neither poison the scale nor be
silently accepted as the truth of the side. The median is ALSO what `MotorGroup::position()` will
report to the odometry at run time — the scale is measured through the same statistic it will be
applied to, so a slow member cancels exactly. The straightness of the push is checked: |Δθ_imu| over
the push above a bound (config, default 3°) refuses the run ("not straight — push again"). Rejected:
per-member scales (five numbers per side that the group cannot apply — it reads one median).

**T5 — The calibrated geometry is ONE number per side, inches per radian, and `DriveGeometry` gains a
constructor for it.** `fromInchesPerRadian(ipr)` builds `{wheelRadius = ipr, ratio = 1.0}`:
`inchesPerRadian()` is exact, both consumers (`drive_encoder_odometry.hpp:107-108`,
`odo_stall_check.hpp`'s per-wheel geometry) use only that product, and the header must say in words
that a calibrated geometry's `wheelRadius` field carries the PRODUCT (the panel prints "ipr", never
"radius", for a calibrated description). Rejected: a third field or a tagged variant on a type two
consumers already hold by value (a shape change for no consumer benefit); storing a guessed diameter
and a derived ratio (§2, rejected #4).

**T6 — Two additive plant extensions, both bit-identical for every existing scenario.**
(a) `MemberFault::Reversed`: the member reports negated position AND velocity (like `SignFlipped`),
and if it is the PRIMARY its commanded volts are negated before the motor model — the behaviour of a
real motor wired backwards through a POSITIVE port. `SignFlipped` stays exactly as it is (its tests
pin a fight, where the command still goes the right way). (b) The coupled-train command model: today
the plant reads a wheel's command from its primary only (`drive_plant.hpp:290-291`); a calibrator
that pulses ONE extra member would move nothing on the plant. Rule: if every member of a wheel is
commanded the SAME volts (exact compare), the wheel's command is the primary's value — the existing
path, bit for bit, which is why `motor_group_plant_test.cpp` test 7's 5-member equivalence pin
survives; otherwise it is the MEAN of the members' commanded volts (a velocity-driven model of a
train pulled by some of its motors; torque-free, like everything in the plant). Both extensions are
pinned by the WHOLE existing suite (unchanged output) and by their own tests. Rejected: a hand-rolled
"push world" outside the plant (the A2 rule — closed-loop truth lives in one place).

**T7 — "No number typed" means no DRIVETRAIN CONSTANT typed; the tape distance is a procedure
input.** The push along a tape needs the distance pushed. It is not a fact about the robot (a
different push, a different number), so the DoD's "no number typed except the three" is kept in
its meaning: the operator CHOOSES the distance from the screen's short list (24 / 48 / 72 in, the
tile multiples a tape on a field naturally hits; 48 default) and the calibrator computes. Rejected:
the robot driving itself a fixed encoder distance and asking the operator to read the tape (the
same human reads the same tape, with the robot under power and the scale unknown); a free-text number
on a resistive screen.

**T8 — The track width is the EFFECTIVE width from a powered spin, and the register says how it
differs from a tape.** `W = (ΔR − ΔL) / Δθ_imu` over a spin of at least two full turns each way
(config), sides' travel from the T4 scales, both directions averaged. On foam a turning tank scrubs,
so the encoders over-count the turn and the measured `W` comes out LARGER than the tape's contact-
line-to-contact-line width; that effective number is the one `TankKinematics` needs to command a
turn that lands and the one the heading cross-check must read as zero. Rejected: taping it (Station
F.0.1 stays as the fallback's provenance); a single direction (asymmetric scrub biases it; the
average cancels the first-order term). The plant scrubs nothing, so the host recovers the true width
exactly (to floating point); the slip hostility is what proves the estimate degrades gracefully.

**T9 — The table stays, as the fallback, and it becomes a SOURCE of the one representation.**
`tableToDescription()` maps signed ports, cartridge, IMU port and `fromDiameter(wheel, ratio)` ×
side into a `DrivetrainDescription` with `source = TypedTable` and the table's provenance string.
Part 2's composition root then has exactly one input type with two sources — the sentence the build-
order entry ends on. The table's UNSET fields still refuse; nothing here loosens that.

**T10 — Part 2's record WILL carry a checksum, and this brief says so now because the description's
shape is decided here.** E1 chose no per-frame CRC for the blackbox (bytes and cycles on every
tick; a wrong value is a wrong log line). A calibration record is read ONCE per boot and a bit flip
in a sign or a scale DRIVES THE ROBOT WRONG SILENTLY. The description is therefore laid out as fixed-
width fields (no padding-dependent structs, no pointers) so Part 2 can serialize it byte by byte
under a CRC-32 and refuse on mismatch. Part 1 does not write bytes.

---

## 4. Design

### 4.1 `hal::DrivetrainDescription` (Part 1 defines; Part 2 serializes)

```
enum class DrivetrainKind : uint8_t { Tank = 1 };            // append-only; holonomic in spring
enum class MotorCartridge : uint8_t { Red = 1, Green = 2, Blue = 3 };   // human-stated
enum class DescriptionSource : uint8_t { TypedTable = 1, RobotMeasured = 2 };

struct DriveSideDescription {
    std::int8_t signedPorts[kMaxSmartPorts];   // |p| in 1..21, sign = PROS reversal; fixed order
    std::uint8_t count;                        // 0 = no side (invalid)
    units::Length inchesPerRadian;             // > 0; the T4 scale (or radius × ratio from a table)
};

struct DrivetrainDescription {
    DrivetrainKind kind;
    MotorCartridge cartridge;
    std::uint8_t imuPort;                      // 1..21
    DriveSideDescription left, right;          // kinematics' wheel order: 0 = left, 1 = right
    units::Length trackWidth;                  // > 0; effective (T8) or taped (T9)
    DescriptionSource source;
    char provenance[96];                       // who/how/when — printed in every banner
    // helpers (free functions, PROS-free, tested): valid(desc, why, cap) — every check a typo
    // would produce (the chassis table's six, restated for the description, plus ipr/width > 0
    // and kind == Tank); leftGeometry()/rightGeometry() → DriveGeometry::fromInchesPerRadian;
    // describe(desc, buf, cap) → the one-line panel text; sideOf(desc, port); signedPortOf(desc, port).
};
```

`kMaxSmartPorts = 21` lives here (the table's `kMaxPort` and the monitor's `kMaxSideMembers` say the
same number; the description is the library's home for it and the other two cite it or keep their
own — the executor decides, in the log, with the reason).

### 4.2 `calibration::DrivetrainCalibrator` — the phases

Construction: `candidates` (span of `IMotor*`, every motor the registry found or, on the host, every
fake), `ports` (parallel `std::int8_t`, positive), `IImu&`, `IClock&`, `CalibratorConfig`
(§4.3). `start(HumanFacts{kind, cartridge, imuPort})` from `Idle`. `tick()` once per control tick
(the caller paces; 10 ms on the robot); `status()` returns `{phase, prompt, progress line}`;
`answer(Answer)` supplies the operator's touch (`Done`, `FrontFirst`, `BackFirst`, `Distance24/48/72`,
`Abort`); `result()` is valid only in `Done`; `abort()` from anywhere → every candidate 0 V + Coast
and `Aborted` with the reason. No phase blocks; no phase allocates.

| Phase | Powered? | What happens | Refuses when |
|---|---|---|---|
| `WaitImu` | no | waits for `imu.isReady()` (bounded by `imuReadyTimeout`) | timeout → `Aborted("IMU never ready")` |
| `PushCapture` | **no** | baseline every candidate's `position()`; each tick records `Δ`; on `Done`, ports with `|Δ| ≥ movedThreshold` (5° = 0.0873 rad, MOTOR WATCH's number) are DRIVE ports, sign = direction; then the front prompt (T2): back-first inverts. Ports that did not move are NOT drive motors and are never touched again | < 2 drive ports; the operator answers before anything moved |
| `SideAssign` | yes, one member at a time | for each drive port i: wait for rest, command `sign[i] × pulseVolts` for `pulseTime`, then 0 V + Coast, wait for rest; record Δθ_imu over the pulse and the member's own Δshaft. Side = sign of Δθ (T3). The member's Δshaft must be > 0 | |Δθ| < `pulseYawFloor` ("nothing turned — floor, not blocks?"); Δshaft ≤ 0 (sign contradiction, port named); not exactly two non-empty sides; any candidate outside the pulsed one moving above `restFloor` while it should be at rest |
| `ScalePush` | **no** | baseline; the operator pushes along the tape and answers `Done` then the distance; per side `ipr = D / median(Δshaft)` (T4); straightness check | |Δθ_imu| > `straightBound`; any side's median Δshaft ≤ 0 or below `minPushRad` (too short to measure) |
| `SpinTest` | yes, both sides | left = −spinVolts, right = +spinVolts until Δθ_imu ≥ +`spinTurns × 2π` (bounded by `spinTimeout`), brake, rest; then the mirror image; `W` per direction = (ΔR − ΔL)/Δθ_imu with side travel = median Δshaft × ipr; result = mean of the two (T8). The coupled-side monitor runs over each side's members during the spin with the group's thresholds; a persisted disagreement aborts | W ≤ 0 or the two directions disagree by more than `spinAgreement` (5 %); timeout; a fight |
| `Done` | no | `result()` = the description with `source = RobotMeasured` and a provenance line the caller supplies (the program's build stamp), plus a `CalibrationReport` (per-port Δ of the push, per-port pulse Δθ, per-side scale and its member spread, W per direction) for the panel and the log | — |

Every powered phase is bounded by a time and a voltage ceiling (`pulseVolts` / `spinVolts` default
3 V, the tester DRIVE station's first ceiling), and `tick()` in any powered phase re-commands the
same volts every tick (a caller that stops ticking leaves the motors at the last command — so the
config's `commandTimeout` makes a missing tick a stop: if `clock.now()` jumps more than it since the
last tick, the calibrator commands 0 V and aborts. The stalled-pacer lesson of C2, applied here).

### 4.3 `CalibratorConfig` — every number a field, every field a register entry

`movedThreshold` (0.0873 rad — carried from MOTOR WATCH, invented there), `restFloor` (0.05 rad/s),
`restSettleTime` (0.5 s), `pulseVolts` (3 V), `pulseTime` (0.4 s), `pulseYawFloor` (2°),
`straightBound` (3°), `minPushRad` (2π — at least one wheel turn), `spinVolts` (3 V), `spinTurns`
(2), `spinTimeout` (20 s), `spinAgreement` (0.05), `imuReadyTimeout` (5 s), `commandTimeout`
(0.1 s), `monitor` (a `SideMonitorConfig`, defaults = the group's). All INVENTED for a first run and
said so; the register carries them (§7); Part 3 measures which need to move.

### 4.4 Reproducibility is a property of the calibrator, tested on the host

Two complete runs on the same plant under different seeds of the encoder and IMU hostility must
agree: each side's `ipr` within `reproScale` (2 %) and `W` within `reproWidth` (3 %) — the DoD's
"reproduced by a second run within a stated tolerance", stated here and pinned before hardware.

---

## 5. Tests — `test/drivetrain_calibrator_test.cpp`, `test/drivetrain_description_test.cpp`, plus additions

Every test names the property it would fail without; every mutation is RUN, observed red, restored,
logged with the observed message. Bit-identity claims are `memcmp`/exact, never `Approx`.

| # | Test | Load-bearing property | Mutation (must go red) |
|---|---|---|---|
| 1 | Description validity: every check a typo would produce (a port twice, both sides, |p| > 21, IMU port a drive port, an empty side, ipr ≤ 0, width ≤ 0, kind ≠ Tank, NaN anywhere) — each the SOLE cause of refusal in its case | a contradictory description never reaches a constructor | remove any one check |
| 2 | `tableToDescription()` on robot two's re-typed table: signed ports preserved bit for bit, `ipr = 1.375 × ratio`, width, cartridge, IMU 2, `source = TypedTable`, provenance carried; an UNSET table refuses | the second source is faithful | drop the sign; use the diameter as the radius |
| 3 | `fromInchesPerRadian(k).inchesPerRadian() == k` exactly, and equals `fromDiameter(d, r)` for `k = d/2 × r` bit for bit | the calibrated constructor is exact | multiply by 2 |
| 4 | **Plant, push capture:** a 2-wheel tank plant with 5 coupled members per wheel, one primary `Reversed`, one extra `Reversed` on the other side, plus TWO non-drive fakes that never move; the harness "pushes" (commands the primaries at 4 V for 1 s with the calibrator observing); `Done` + `FrontFirst` → exactly the ten drive ports, the two reversed ones with sign −1, the two idle fakes excluded | MOTOR WATCH's rule holds in the library | threshold → 0 (idle fakes captured); drop the inversion for `BackFirst` (test 5) |
| 5 | Same push answered `BackFirst` → every sign inverted vs test 4 | the front is what the operator says | — (mutation of 4) |
| 6 | **Plant, side assignment:** after 4, pulses run; each member's pulse yaw sign is negative on the left wheel and positive on the right; sides = the plant's wheel 0 / wheel 1 membership exactly; a `Reversed` member pulsed with its captured sign advances POSITIVE on its own encoder | T3 | swap the yaw-sign → side rule (all sides flip; the truth comparison fails); drop the own-encoder check while a wrong sign is injected (test 7) |
| 7 | A member whose captured sign is deliberately corrupted before `SideAssign` (test hook: the harness answers the push, then flips one recorded sign) → refused, the PORT named in the reason | a contradiction is never averaged | drop the check |
| 8 | One idle candidate creeping (a non-drive fake set moving during a pulse) → refused | "everything else at rest" is enforced | drop the rest check |
| 9 | Wheels-up pulse (plant with the IMU heading frozen — a `FakeImu` the harness holds still) → `pulseYawFloor` refusal with the "floor, not blocks?" prompt | the calibrator knows when it is on blocks | floor → 0 |
| 10 | **Plant, scale:** push 48 in of TRUE travel (the harness integrates the plant's true pose), answer `Distance48` → each side's `ipr` equals the plant's `driveWheelDiameter / 2` to 1e-9; with one member at 80 % (a `Reversed`-independent plant knob is NOT added — the harness instead freezes one extra member mid-push: a frozen member's Δ is short) the median is untouched and the report shows the spread | T4 | median → mean (frozen member drags the scale; red) |
| 11 | A push that curves (the harness commands the sides unevenly) → `straightBound` refusal | T4's straightness | bound → ∞ |
| 12 | **Plant, spin:** true width 12.5 in (`TankKinematics{12.5}`); the spin runs both ways; `W` per direction within 1e-6 of 12.5 and their mean exact to 1e-9; `Done`; `result()` valid; `result().leftGeometry().inchesPerRadian()` feeds a `DriveEncoderOdometry` that then tracks the plant's truth within the existing odometry test's bound over a figure-eight | T8, and the round trip into the consumer | W formula sign; use ΔL + ΔR; drop the second direction (with an asymmetric slip hostility on: red) |
| 13 | Spin with `sim/hostile/slip_hostility` on both sides: `W` within 8 % of truth; reproducibility over 4 seeds within `reproWidth`; scale reproducibility within `reproScale` under encoder hostility | §4.4 | tighten the tolerance below the observed spread (documents the spread), then restore |
| 14 | A fight during the spin (a `SignFlipped` extra member — the group's fight signature) → `Aborted` naming the member, every candidate at 0 V + Coast within one tick | the monitor is wired and the safe state is reached | detach the monitor; skip the safe state |
| 15 | Timeouts: IMU never ready; spin that never reaches the turn count (the harness stops advancing the plant) → aborts at the configured time, motors safe | every powered phase is bounded | remove either bound |
| 16 | `commandTimeout`: the harness skips ticks for 0.2 s while spinning → 0 V + abort | a missing tick is a stop | remove the check |
| 17 | `abort()` from every phase → motors safe, `Aborted`, `result()` a precondition violation | the safe state is unconditional | — |
| 18 | **Plant pins:** the ENTIRE existing suite is bit-identical after the two `sim/` extensions (the clean-tree assertion count and every golden unchanged — the log records the count before and after); `motor_group_plant_test` test 7 (5 members, equal command) unchanged | T6 bit identity | make the equal-command shortcut unconditional-mean (test 7 red) |
| 19 | `MemberFault::Reversed` on a primary: the plant drives BACKWARD under +V and the member reads negative; on an extra: reading negated, plant unaffected | T6(a) | — |
| 20 | Coupled-train command model: one extra member at 3 V, primary at 0 → the wheel spins at the 0.6 V-equivalent mean | T6(b) | — |

**Tests must try to break it** (RESUMING, standards): the push threshold, the yaw floor and the
agreement bound are each attacked from BOTH sides (a value just under / just over the line).

---

## 6. Definition of Done — Part 1

- [ ] `hal/drivetrain_description.hpp` with `valid()`, `describe()`, the geometry accessors; every
      public member documented (the D3 coverage gate); ARM gate green with the new headers
- [ ] `hal/drive_geometry.hpp` gains `fromInchesPerRadian`; header says what `wheelRadius` then carries
- [ ] `calibration/drivetrain_calibrator.hpp`: all six phases as §4.2, every refusal naming its cause,
      allocation-free, PROS-free, sim-free (the CI layering guard passes for the new directory)
- [ ] `sim/drive_plant.hpp`: the two T6 extensions; the whole pre-existing suite bit-identical
      (assertion count and every golden; recorded in the log before and after)
- [ ] `src/chassis_table.hpp`: `tableToDescription()`; `test/chassis_table_test.cpp` extended; the src
      build gate's five builds green (`python3 tools/src_build_gate.py self-test && check`)
- [ ] Tests 1–20 green; every mutation in §5 RUN, observed red with the message logged, restored
- [ ] Host suite green; PROS-free guard, layering guard, ARM header gate, the doc gates green
- [ ] `R3d-PROGRESS.md` complete: the order of work, every ruling the executor had to make beyond
      §3 with its alternative, the prose-review log (§7), the clean-tree suite count
- [ ] **Nothing here claims a hardware result.** The description has never been produced on a robot;
      the calibrator has never moved one. The PROGRESS log and every header say so.

---

## 7. Documentation, in the same commit — the contract

Touched (and what the executor writes there):

1. **`docs/api/`** — regenerated (`python3 tools/api_doc_tool.py`; the freshness gate fails otherwise).
2. **`docs/hardware-assumptions.md`** — new entries in the Robot 2 section (numbering continues from
   HA-133): the T3 yaw-sign belief (reasoned from rigid-body kinematics; settled by Part 3's first
   pulse); the T8 effective-vs-taped width (reasoned; Part 3 measures the ratio); the `CalibratorConfig`
   numbers as ONE invented entry naming every field (settle: Part 3); the T6 coupled-train command
   model as a sim belief (settle: R6's back-fit). HA-124/125/126's *Settle* lines name Part 3 rather
   than "R3d". HA-131's owner becomes R4 (the re-homing, with the sentence that says why).
3. **`docs/roadmap.md`** — the "You are here" pointer gains one sentence (R3d Part 1 landed, host-only,
   nothing on hardware; R3b still open on its bench run).
4. **`docs/internal/build-order.md`** — the R3d entry's status line (Part 1 landed, Parts 2–3 next);
   the first bold `**Next:**` block; the deviations row already added with this brief is left as is.
5. **`docs/internal/PROJECT-BRIEFING.md`** — regenerated (`python3 tools/briefing_status.py generate`;
   the gate).

Deliberately NOT touched, with why: **`docs/guide/14-what-it-cannot-do-yet.md`** (still true in every
sentence; the calibration chapter is Part 3's, written from a followed procedure); **`README.md`** (no
user-facing behaviour exists until Part 2's programs); **`docs/shulib-v2-master-plan.md`** (its
catalog row "online wheel-radius calibration" moves when it has run — Part 3); **the worksheet**
(Station G is Part 3's); **`hal/motor_group.hpp:73` and `src/main.cpp:461/:889`** ("until R3d" — the
dead-port promise is Part 2's, still open); **`docs/faq.md`** (no question exists yet).

**Prose review, logged (rule 4):** for every changed source file, grep `docs/` and `README.md` for
its name and its concepts (`DriveGeometry`, `fromDiameter`, `MotorWatch`/`MOTOR WATCH`, `chassis
table`, `track width`, `external ratio`, `MemberFault`, `attachCoupledMembers`), read every hit,
fix what is stale, and record in the PROGRESS log what was checked and what changed — including
"checked, nothing stale" per file.

**C7 removability** re-run before the log's last entry: `grep -rn "internal/\|chunks/\|RESUMING\|build-order" docs/*.md docs/guide docs/cookbook docs/api README.md` must return only the pre-existing prose mentions (C8-COMPLETED §3 names them), no links.

---

## 8. Landmines

1. **The sign is applied in the calibrator ONLY while calibrating (T1).** A calibrator that negates a
   member whose adapter was ALSO constructed with a negative port applies the sign twice and captures
   every sign backwards, consistently — invisible on the host, a fight on the robot. The candidates'
   ports are positive by contract; precondition it.
2. **`IMotor::position()` is cumulative and NOT wrapped** (`hal/motor.hpp:12-14`); a `math::Angle`
   anywhere in the calibrator's Δshaft arithmetic wraps a two-turn spin to nothing. `AngleDim` only.
3. **The IMU's Δθ over a spin must be accumulated from `heading()` via `errorTo` per tick** (the
   odometry's own line, `drive_encoder_odometry.hpp:134-136`); a single end-minus-start on a wrapped
   heading cannot see two turns.
4. **The plant reads commands from the PRIMARY member** until T6(b) lands; a calibrator test written
   before that extension passes for the wrong reason (pulsing an extra member moves nothing and the
   test may "pass" by pulsing primaries only). Land T6 first, with its own tests, then the phases.
5. **`FakeImu` reports whatever the plant last synthesized**; "wheels-up" in test 9 means the harness
   stops the plant's heading, not that the plant models blocks. Say so in the test.
6. **Median of an EVEN member count** is the mean of the middle two — for the bench bot (4 per side)
   a frozen member still cannot reach it; the group's header says so and the calibrator's median must
   be the same function (share it or pin equality; do not write a third median).
7. **A refusal must leave every candidate at 0 V + Coast** — Coast, not Brake: a calibration abort
   with a person nearby must let the robot be pushed clear.
8. **Do not touch `SignFlipped`'s semantics** — `motor_group_plant_test` tests 3–4 and the monitor
   tests pin a fight through it. `Reversed` is a NEW member, appended.
9. **The doc coverage gate accepts nothing undocumented and the generator once leaked an internal
   path from a header comment** (D3): no `docs/internal/` path in any new header.
10. **No hardware claim, anywhere.** Not "the robot measures", but "the calibrator is designed to
    measure … proven on the plant; not yet run on a robot".

---

## 9. Verification the coordinator runs before committing (RESUMING §3)

```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6
python3 tools/src_build_gate.py self-test && python3 tools/src_build_gate.py check
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo "int main(){return 0;}" >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
# the CI guards, exactly as .github/workflows/ci.yml spells them (PROS-free; layering minus sim/)
python3 tools/briefing_status.py check
```

Then: read `drivetrain_calibrator.hpp`'s `SideAssign` and `SpinTest` in full (T1, T3, T8 are the
load-bearing lines), re-run mutations 6 and 12 by hand, confirm the plant pin (test 18) against the
recorded pre-change count, and confirm nothing was committed.

---

## 10. Parts 2 and 3 — scope only; each gets its own brief

**Part 2 — persistence, the registry, boot verification, the programs (host-only, build-gated).**
`hal/block_source.hpp` (`IBlockSource`, the READ sibling of `IBlockSink`; whole-file into a caller's
buffer; bool + byte count; no throw) with `hal/fake/` and `hal/pros/` (fopen `rb` under the same
`/usd/` rule, host-tested against a temp dir like `pros_block_sink_adapter_test.cpp`).
`hal/device_registry.hpp` (`IDeviceRegistry`: `pluggedKind(port)`), its fake, its pros adapter over
`registry_get_plugged_type` with the zero-index conversion in exactly one place, and the shim entry.
`calibration/calibration_record.hpp`: magic, version, fixed width, little-endian, byte by byte,
CRC-32 (T10), refuse-don't-misread; encode/decode with a byte-exact golden. `calibration/boot_check.
hpp`: the record vs the registry — a recorded drive port with NO device is MISSING, with a NON-motor
is SWAPPED, an unrecorded motor is NEW; SWAPPED refuses always; MISSING goes through
`teleop::DrivetrainDegradation` (≥ 3 per side, ≤ 2 total) in the competition program and refuses in
the calibrate program; NEW is a warning. **Honest limit, stated in the header:** two drive cables
swapped WITH EACH OTHER are invisible to the registry (both ports still carry motors) and are caught
at the first drive by the coupled-side monitor as a fight — so the DoD's "a swapped port is refused at
boot naming the port" is met for a port that lost its motor, and the cable swap is met one step later
and named there. `MotorGroup` gains an ABSENT member (a null pointer skipped in fan-out and reads,
`present = false` to the monitor; the median over present members) — the promise at
`motor_group.hpp:73`. "shulib Teleop" builds its graph from the card's record when present and
verified, else from the table when set, else refuses naming both. `PROGRAM=calibrate` → "shulib
Calibrate" (slot 4): the calibrator with a touch panel for its prompts, the record written and read
back and verified before "DONE" is painted; the src build gate grows to six builds. Docs: HA entries
(registry, fopen rb, CRC), the worksheet's Station G (written, unrun), `docs/guide/` "Calibrating a
robot" (marked unrun), README's "what it is not yet".

**Part 3 — the bench session.** In order: "shulib Teleop" wheels-up (with the typed fallback if the
card is empty — the table's two numbers taped that morning, or Teleop waits for the record);
"shulib Calibrate" wheels-up refused as designed (test 9's prompt on a real screen), then on the
floor: push, pulses, tape, spin; the record on the card; a second run reproduces within §4.4; reboot
into "shulib Teleop" from the record; wheels-up, floor, a short auton; then R3b's close and M1's badge
on THAT log, and R3d's close on the reproduction. Every register entry Parts 1–2 added gets its
settle line answered or its number moved; `docs/guide/14` loses the sentence the log falsifies.

---

*Written by the coordinator 2026-09-17 from the tree at `6ceb244`, after reading the files in §1.
Nothing in this brief has run on a robot.*
