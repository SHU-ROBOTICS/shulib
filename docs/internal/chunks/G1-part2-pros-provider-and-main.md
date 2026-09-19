# G1 Part 2 (+ T2's wiring) — the PROS device provider, the library teleop loop, and a `main.cpp` a person can read

> **Execution brief.** Written 2026-09-18 (evening) by the coordinator, tree `225d0ac`. Parent spec:
> [`G1-robot-config-and-builder.md`](G1-robot-config-and-builder.md) §2 "Part 2"; T2's integration
> contract: [`T2-input-shaping-drive-modes.md`](T2-input-shaping-drive-modes.md) §7. The team lead's
> ruling that opens it (2026-09-18): *"we haven't tested teleop before so we might as well rewrite"*
> — "shulib Teleop" has never run on hardware, so nothing driven is lost by rebuilding it on the
> library. Live log: append a **"Part 2"** section to [`G1-PROGRESS.md`](G1-PROGRESS.md), FIRST.
> **Executor model: `opus`. Do not commit.**

---

## 0. Why, and what it is measured against

The team lead: *"I want the main to be a LOT cleaner and human looking through our library … extremely
user friendly."* Today `src/main.cpp` is 1,338 lines: two hand-wired object graphs (`Robot` `:335-424`,
invented X-drive; `tanklib::TankRobot` `:604-670`), a port-map string builder, a refusal painter, a
47-line teleop loop, a 100-line panel, a 130-line boot-source block inside `initialize()`, and an
`#if` lattice over six builds. G1 Part 1 built the replacement for the graphs (`robot::Robot` from a
`config::RobotConfig`); T2 built the replacement for the raw stick mapping (`teleop::DriveShaper`).
Neither is used by any program. This chunk uses them.

**The measure:** after this chunk `src/main.cpp` is a DISPATCHER — the variant checks, the beacon,
the precondition handler, and the five PROS entry points each forwarding to the selected program —
and "shulib Teleop" lives in its own file whose core a student can read top to bottom:

```text
ProsDeviceProvider devices;                       // the adapters, made on demand, by port
config::RobotConfig config = bootConfig(devices); // the card's record, else the typed table
robot::Robot robot{config, devices};              // the whole library graph, wired
teleop::runDriverControl(robot.chassis(), master, devices.pacer(), shaper, fieldDisabled, panel);
```

Target sizes, stated so "cleaner" is a number: `main.cpp` ≤ 300 lines; the teleop program's
composition (everything except its panel/refusal painting) ≤ 120 lines. The executor reports the
before/after line counts per file.

---

## 1. What exists to build on (read these)

| Piece | Where | What matters |
|---|---|---|
| The builder | `include/shulib/robot/robot.hpp` | `validate(cfg, devices) → BuildVerdict`, `Robot{cfg, devices}` (non-movable, in place), accessors `chassis()/localizer()/faults()/health()/context()/config()/describe()`. **A refused drive motor makes `validate()` fail — there is NO dead-port tolerance in it (§3 ruling M1)** |
| The provider seam | `include/shulib/hal/device_provider.hpp`, `hal/fake/fake_device_provider.hpp` | `motor(signedPort, cartridge)`, `imu(port)`, `gps(port, leverF, leverL)`, `rotation(port)`, `clock()`, `battery()`, `telemetry()`, `pacer()`, `lastRefusal()`; devices are provider-OWNED; `nullptr` on refusal |
| The config | `include/shulib/config/robot_config.hpp` | `RobotConfig` embedding `hal::DrivetrainDescription` for a tank; `XDrive{…}`; odometry choice; GPS; tuning |
| The boot source (R3d Part 2) | `src/main.cpp:965-1060` (inside `initialize()`), `calibration/boot_check.hpp` (`verifyAgainstRegistry`, `chooseBootSource`), `calibration/calibration_record.hpp`, `src/chassis_table.hpp` (`tableToDescription`, `describeMissingForDescription`) | record → verify → table → verify → refuse naming both; `Degraded` ⇒ the missing ports become ABSENT group members (`TankMotors`, `:554-596`) |
| The adapters | `include/shulib/hal/pros/*.hpp` | constructors and their throwing read-backs; `ProsBlockSource`, `ProsDeviceRegistry`; `ProsImu::calibrate()` once |
| The tolerant-construction pattern | `src/drive_program.cpp:140-200, :320-380` | `std::optional<ProsMotor>` per port, one `try` each, the refusal reason kept |
| The teleop loop | `src/main.cpp:452-474` `runTeleopLoop` | disabled poll → `chassis.cancel()`; sticks read only while connected; `mapSticksToChassisSpeeds`; `drive(Body)`; per-tick callback; `pace()` |
| The driver-feel layer (T2) | `include/shulib/teleop/driver_control.hpp` (`DriveShaper::shape(StickInput, dt, heading, headingTrusted)`, `rezero`, `state()`), `input_shaping.hpp` (`DriverProfile`), `drive_modes.hpp`, `field_centric.hpp`, `stick_mapping.hpp` (`toChassisSpeeds`) | the default profile reproduces yesterday's feel except for the continuous deadband; field-centric falls back on an untrusted heading and latches |
| The other consumers of the raw mapping | `src/drive_program.cpp` (`mapSticks` → `tankSideVolts`), `src/bench_r3a.cpp` station 10 DRIVE | **"shulib Drive" HAS been driven (2026-09-17) and the tester has a hardware history** — §3 ruling M4 |
| The panel and the refusal screen | `src/main.cpp:723-762` (refusal), `:783-898` (`TankPanel`) | PROS screen calls; stays in `src/`, moves file |
| The build axis and its gate | `Makefile:78-125`, `tools/src_build_gate.py` | six builds; the beacons; `-Wformat-nonliteral`, `-Wformat-truncation` |
| The docs that describe the wiring | `README.md` (the "Wiring a `Chassis`…" paragraph and "what it is not yet"), `docs/guide/08-your-first-routine.md` (§"the wiring"), `docs/guide/10-the-api.md` (the teleop-shaped loop), `docs/guide/14` | what Part 1 listed for this part |

---

## 2. Scope

### IN

1. **`include/shulib/hal/pros/device_provider.hpp` — `ProsDeviceProvider`**: `IDeviceProvider` over
   the real adapters. Fixed storage, no heap: `std::optional<ProsMotor>` × 21 by port,
   `std::optional<ProsImu>`, `std::optional<ProsGps>`, `std::optional<ProsRotation>` × 21, and the
   port-less `ProsClock`, `ProsBattery`, `ProsCharSink` + `diag::TermSink` (the telemetry),
   `ProsTickPacer`. Each device is constructed ON FIRST REQUEST inside its own `try`; a throwing
   adapter ⇒ `nullptr` and `lastRefusal()` = the port and the adapter's message; a second request
   for the same port returns the SAME device (a request with a different sign or cartridge for an
   already-made motor is a precondition violation — one port, one configuration). `imu()` starts
   calibration exactly once. Host-tested through the PROS shim like every adapter
   (`test/pros_device_provider_test.cpp`): same-port identity, refusal naming the port, the
   sign/cartridge conflict, calibrate-once.
2. **Dead-port tolerance in the builder (ruling M1):** `robot::validate()` gains an optional
   `const teleop::DegradationPolicy*`-style input — concretely `BuildOptions{bool tolerateDeadPorts
   = false}`: when true, drive motors the provider refuses are counted per side, passed through
   `teleop::evaluateDegradation`, and on `Drive`/`DriveDegraded` become ABSENT slots in the side's
   `MotorGroup` (R3d Part 2's `nullptr` member); on `Refuse`, and ALWAYS for a tank side left with
   zero present members, the verdict refuses naming the ports. `BuildVerdict` reports
   `degraded` + the dead ports; `Robot::describe()` says so. Default false keeps every Part 1 test
   bit-identical. Plant-tested (a fake provider refusing one of five ports: the robot builds, the
   group reports 4 present, the banner names the port; three refused ⇒ refusal).
3. **`include/shulib/teleop/teleop_loop.hpp` — `teleop::runDriverControl(...)`**: the loop as
   LIBRARY code, PROS-free: `(chassis::Chassis&, const hal::IController&, motion::ITickPacer&,
   DriveShaper&, const motion::MotionConfig& budgets via the chassis, IsDisabled&& disabled,
   PerTick&& perTick)`. Per tick: `disabled()` ⇒ `chassis.cancel()` and return; read the sticks only
   while connected (HA-103); `shape(sticks, dt, heading, trusted)` with `dt` = the pacer's tick
   period, `heading` = the chassis estimate's heading, `trusted` = estimate live AND no
   `FaultCode::ImuLost` raised (the two inputs T2's brief names); `toChassisSpeeds`;
   `drive(Body)`; a re-zero `ButtonEdge` on a caller-chosen button calls `shaper.rezero(heading)`;
   `perTick(command, connected, shaper.state())`; `pace()`. Host-tested on the plant with
   `FakeController`: the one-tick latency pin THROUGH THE LOOP, disabled ⇒ cancel observed at the
   motors, disconnected ⇒ zero, the re-zero edge, field-centric fallback visible in the state.
4. **`src/teleop_program.cpp/.hpp` — "shulib Teleop" rebuilt on the library**: the boot source
   (record → table → refuse, moved out of `initialize()` into `bootConfig()` and returning a
   `config::RobotConfig` whose tank slice is the chosen `DrivetrainDescription`), `ProsDeviceProvider`,
   `robot::validate(config, devices, {.tolerateDeadPorts = true})` ⇒ refusal screen naming the
   port/field or `Robot` constructed in place, the session header from `robot.describe()`,
   `runDriverControl` with the default `DriverProfile`, the panel (moved, plus ONE new row: the
   field-centric state word and the profile's mode), the refusal painter. Every line the current
   program says at boot that is still true is still said (UNRUN; provisional gains; no independent
   stall source; the source word MEASURED/TYPED).
5. **The X-drive variant as data**: the invented graph becomes a `constexpr config::RobotConfig`
   (`XDrive`, tracking wheels, GPS) carrying the HA-111 label in its identity/provenance and built
   through the same `ProsDeviceProvider` + `Robot`. It still cannot boot on a real robot (its ports
   are invented) and still says so.
6. **`src/main.cpp` becomes the dispatcher** (§0's measure): the variant/program `#if` checks, the
   beacon, `robotPreconditionHandler`, the competition-state screens, and `initialize()`/`disabled()`
   /`competition_initialize()`/`autonomous()`/`opcontrol()` forwarding to `bench::`, `drive::`,
   `calibrate::`, `teleopprog::` functions. No object graph, no panel, no boot logic in it.
7. **The src build gate stays at six builds, all green**, beacons unchanged, zero disallowed warnings.
8. Tests, the DoD, the documentation contract below.

### OUT

- **"shulib Drive" and the tester's DRIVE station keep `mapSticks`** (ruling M4). Their re-pointing
  at `DriveShaper` happens AFTER a driver has felt the shaper in Teleop on hardware.
- **"shulib Calibrate"** is untouched (it does not build a `Robot`; it builds a calibrator).
- T3 (bindings), G3 (codegen), anything on hardware, any hardware claim.

### REJECTED

- **Deleting the longhand construction from the docs.** The standalone promise stays (chassis.hpp's
  header; `chassis_facade_test`'s file-free case): the builder is the on-ramp, the longhand is Tier 3.
- **A `Robot` that owns PROS adapters.** The provider owns hardware; the builder stays PROS-free.
- **Silently stricter Teleop.** Without M1 the rewrite would refuse on one dead port where the
  committed program drives degraded — a behaviour change nobody chose.

---

## 3. Rulings

**M1 — Dead-port tolerance moves INTO the builder, opt-in.** The policy is already one pure function
(`teleop::evaluateDegradation`); R3d Part 2 applied it in `main.cpp`'s `TankMotors`. With the graph
moving into `robot::Robot`, the tolerance must move with it or be lost. Opt-in because a test rig and
a calibration want strictness; the competition program opts in. Rejected: the provider faking a
motor (a fake in a competition binary — R3b §6's rule).

**M2 — The boot check and the builder's device pass are TWO checks and both stay.** The registry
check (`verifyAgainstRegistry`) runs BEFORE any adapter exists and catches a SWAPPED port (a
non-motor on a drive port), which the provider's `try` would report only as "refused". The builder's
pass catches what the registry cannot (an adapter whose read-back fails). `bootConfig()` runs the
first; `validate()` runs the second; a port MISSING in the first is expected-refused in the second
and must not be reported twice as two different problems — the refusal/degraded line names it once.

**M3 — The teleop loop is library code with the disabled predicate injected.** `pros::c::
competition_is_disabled` is the only PROS call in the loop; injecting it keeps `teleop/` PROS-free
and makes the loop host-testable end to end. Rejected: leaving the loop in `src/` (the 47 lines a
student would otherwise copy, and the one place the one-tick latency property can silently break).

**M4 — Only "shulib Teleop" changes feel now.** It has never been driven, so there is no feel to
preserve. "shulib Drive" HAS been driven and the tester's DRIVE station has a hardware history; both
keep the pinned `mapSticks` until a driver has compared the two on the robot. The roadmap's T2 rows
stay `[~]` and say exactly this.

**M5 — The default profile in Teleop is `ArcadeBody`.** Field-centric ships selectable (a constant
in the program, documented) but OFF: the first hardware run must not also be the first run of a
frame rotation. The re-zero button is wired regardless (harmless in a body mode).

---

## 4. Tests

| # | Test | Property | Mutation |
|---|---|---|---|
| 1 | `ProsDeviceProvider` via the shim: same port ⇒ same object; a throwing adapter ⇒ `nullptr` + `lastRefusal()` naming the port; conflicting sign/cartridge ⇒ precondition; `imu()` calibrates once across repeated requests | the provider's contract | return a fresh device per request; drop the conflict check |
| 2 | Builder tolerance (fake provider): 1 of 5 refused + opt-in ⇒ builds, 4 present, `degraded`, banner names the port; opt-in OFF ⇒ refuses naming it (Part 1's behaviour, unchanged); 3 refused ⇒ refuses via the policy; a side with 0 present ⇒ refuses | M1 | ignore the policy (always tolerate) |
| 3 | Degraded robot on the plant: drives and reports; the healthy-case bit-identity tests of Part 1 unchanged (assertion counts recorded before/after) | tolerance changes nothing when nothing is dead | — |
| 4 | `runDriverControl` on the plant with `FakeController`: a stick step at tick k is in the commanded volts of tick k; `disabled()` true ⇒ `cancel()` (0 V + Brake at the motors) and return; disconnected ⇒ zero command whatever the axes; the re-zero edge calls `rezero` once per press; an untrusted heading ⇒ `state().fieldCentric.fellBack` | the loop's five properties | add a one-tick buffer; skip the cancel; poll axes while disconnected |
| 5 | Default-profile loop vs the OLD loop's mapping on a sweep: identical outside the deadband rescale (the documented bound), identical at 0 and ±1 | Teleop's feel change is exactly the continuous deadband | — |
| 6 | `bootConfig()`'s pure core (the description → `RobotConfig` step) host-tested: record description ⇒ config valid, `source` preserved, drive-encoder odometry, GPS absent | the boot source feeds the builder faithfully | drop the sign copy (caught by Part 1's signed-port assertion through the builder) |
| 7 | The six-build gate green; `main.cpp` ≤ 300 lines and contains no `MotorGroup`, `Localizer`, `RobotContext`, `MotionDeps`, `screen_print` (a grep in the log) | §0's measure is mechanical | — |

Every mutation RUN on a fresh binary (edit → touch EVERY test that includes the edited header →
`--target shulib_tests/fast` → require the link line), observed red, logged, restored; the briefing
is regenerated only after the last restore is relinked green.

---

## 5. DoD

- [ ] Items 1–7 of §2; every public member documented; ARM gate green; PROS-free and sim-free guards
      (the provider lives under `hal/pros/`; `teleop_loop.hpp` includes no `pros/`)
- [ ] Tests 1–7; mutations logged red; the pre-existing suite unchanged except stated additions
- [ ] Six builds link, zero disallowed warnings, beacons unchanged; line counts before/after reported
- [ ] "shulib Teleop" still says UNRUN everywhere it did; no hardware claim
- [ ] The documentation contract; the prose review; the log complete

---

## 6. Documentation, in the same commit

Touch: `docs/api/` + `mkdocs.yml`; `README.md` — the "Wiring a `Chassis`" paragraph (the builder is
the on-ramp, `src/teleop_program.cpp` the example, the longhand kept as Tier 3) and "what it is not
yet"; `docs/guide/08-your-first-routine.md` — "the wiring" section gains the two-line builder path
with its example compiled in `test/guide_examples_test.cpp` and quoted verbatim (the guide's
anti-staleness rule), the longhand kept; `docs/guide/10-the-api.md` — the teleop-shaped loop points
at `runDriverControl`; `docs/guide/14` — the driver-control bullet (the shaper now drives Teleop;
unfelt); `docs/roadmap.md` — the M5 config bullet, the two Phase T rows (still `[~]`: Teleop wired,
Drive and the tester not, no driver has felt it), "You are here"; `docs/internal/build-order.md` —
G1's and T2's status lines, the first `Next:` block; `docs/hardware-assumptions.md` — HA-112's
source lines (Teleop now consumes the feel layer), HA-111's note that the invented map is now a
config constant; the worksheet's Station F/G — ONLY the panel-row table if a row changed (the new
field-centric row), nothing else; `PROJECT-BRIEFING.md`. NOT touched, with why: the master plan,
`docs/faq.md`, `src/drive_program.cpp`, `src/bench_r3a.cpp`, `src/calibrate_program.cpp`.

Prose review logged per changed file: grep `docs/` and `README.md` for `main.cpp`, `TankRobot`,
`runTeleopLoop`, `composition root`, `wiring`, `hand-wired`, `object graph`, `mapSticksToChassisSpeeds`,
`deadband-only`; read every hit; fix what is stale.

---

## 7. Landmines

1. **`Robot` and the provider are non-movable statics**; construction order across translation units
   is undefined — build them inside a function (`static` locals, the `tankRobot()` pattern), never
   as namespace-scope objects.
2. **`-Wformat-nonliteral` and `-Wformat-truncation`** are on in `src/` (both have bitten).
3. **LVGL fonts are ASCII-only**: no em dashes or arrows in any runtime string.
4. **The precondition handler must be installed before the first adapter is constructed**, and the
   graph boundary `try` stays: a throwing precondition inside `Robot`'s constructor is a refusal
   screen, never a crash loop.
5. **One sign, one place**: the description's signed port reaches `ProsMotor` as stored.
6. **The session header's port map comes from `robot.describe()`/the description**, never retyped.
7. **Do not change `stick_mapping.hpp`'s bodies or `stick_mapping_test.cpp`**; the drive program's
   pin depends on them (M4).
8. **No hardware claim.** Teleop remains UNRUN; say so where the old program did.

---

## 8. Verification (the coordinator)

`verify.sh`; read `teleop_program.cpp`'s composition and `main.cpp` in full (it must now be
readable in one sitting); re-run mutation 2 (tolerance policy ignored) and mutation 4 (the cancel
skipped) by hand; diff the boot banner's strings against the committed program's; rebuild the four
hardware packages from the result and confirm the beacons.
