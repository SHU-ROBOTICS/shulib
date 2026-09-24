# G1 Part 2 (+ T2's wiring) — the PROS device provider, the library teleop loop, and a `main.cpp` a person can read

> **Execution brief.** First written 2026-09-18 (evening) at tree `225d0ac`; **REVISED 2026-09-24**
> at tree `a2c9fff` after an audit of the brief against the tree (below). Parent spec:
> [`G1-robot-config-and-builder.md`](G1-robot-config-and-builder.md) §2 "Part 2"; T2's integration
> contract: [`T2-input-shaping-drive-modes.md`](T2-input-shaping-drive-modes.md) §7. The team lead's
> ruling that opens it (2026-09-18): *"we haven't tested teleop before so we might as well rewrite"*
> — "shulib Teleop" has never run on hardware, so nothing driven is lost by rebuilding it on the
> library. **Executor model: `opus`. Do not commit.** Live log: append a section per part to
> [`G1-PROGRESS.md`](G1-PROGRESS.md), FIRST ("Part 2a", "Part 2b", "Part 2c").
>
> **This brief is THREE PARTS, each its own executor run, verification and commit, in order.** Each
> part waits for the team lead's go. The split exists because the one-part version's documentation
> list ran to ten documents (rule 7: about five) and because Part 2a is library-only while 2b and 2c
> rewrite `src/`.

---

## Revision note (2026-09-24) — what the audit found, and what changed

The first version would have failed its executor in four places, each confirmed in the source:

1. **`robot::Robot`'s constructor re-runs `validate(cfg, devices)` with no options**
   (`robot.hpp` `prechecked()`, the first member initializer), and `wireMotors()` requires every
   drive motor non-null. A robot that passed a *tolerant* `validate()` would throw inside
   `Robot{…}`. → Options now reach the constructor (§2a.3), and a test constructs the degraded robot.
2. **Only `ProsMotor` fails loudly on a dead port** (its read-back, `motor.hpp`). `ProsImu`'s and
   `ProsRotation`'s constructors cannot throw; `ProsGps` defers on an empty port. A provider that
   refuses by "catching the throw" would hand back a live device for an EMPTY IMU, GPS or rotation
   port. → The provider consults the device registry's `pluggedKind(port)` before constructing
   anything (§2a.2). `device_provider.hpp`'s comment ("every adapter's constructor throws on a dead
   port") is corrected in the same part.
3. **The `src` build gate's variant differential** (`tools/src_build_gate.py`,
   `XDRIVE_WIRING_DEAD_IN`) requires every tank build to emit a `-Wunused-function` from
   `src/main.cpp` — the dead X-drive helpers `robot()`/`portMapString()` — and `Makefile:15-23`
   documents those warnings as the evidence. Moving the X-drive graph out of `main.cpp` fails all
   five tank builds. → Part 2c redesigns the differential deliberately (§2c.4).
4. **Test 7 forbade `screen_print` in a `main.cpp` that §2.6 said keeps the state screens**
   (`benchStateScreen` calls `screen_print_at` six times). → The state screens move out (§2c.1).

And the ruling that shaped the first version — "dead-port tolerance moves into the builder,
through `teleop::evaluateDegradation`" — was replaced after reading how the committed program
actually does it. **Tolerance is decided ONCE, by the boot check over the brain's port registry,
before any adapter exists** (`calibration::verifyAgainstRegistry`, which applies the policy); the
committed `TankMotors` only turns that verdict's MISSING set into `nullptr` slots, and lets any
other adapter failure — including a present motor that rejects its gearset — refuse the program.
Re-running the policy inside the builder over provider refusals would (a) report every dead port
twice, (b) make Teleop silently MORE tolerant (a motor that answers the registry but rejects its
configuration would be driven around), and (c) refuse every robot with fewer than three motors a
side (the policy's stated domain is 4–5 per side). → The builder instead takes the boot check's
**list of absent ports** (§2a.3, ruling M1).

Smaller corrections folded in: the loop's tick period had no source (`motion::ITickPacer` has only
`pace()`); the tank-sticks mode reads the RIGHT stick's Y, which today's loop never reads; the panel
reads drive-encoder odometry that `Robot` keeps private; the precondition hook (`g_faults`) lives in
`main.cpp`'s anonymous namespace and cannot reach a robot built in another file; "estimate live"
has a real spelling (`Localizer::qualityClass() != Quality::Uninitialized` — T2's brief named an
`estimateState()` that does not exist); a function-local static whose constructor throws is
re-attempted on the next call; the session header's port-map field is 96 bytes and
`Robot::describe()` writes a longer banner; `main.cpp`'s entry functions are all in
`shulib::bench`, not per-program namespaces; the Station F panel table is already stale against the
code; and M4's reason for keeping the tester on `mapSticks` was wrong (its DRIVE station has
constructed motors on hardware but never commanded a non-zero voltage — only shulib Drive has a
driven history). Line counts in §0 are re-measured.

---

## 0. Why, and what it is measured against

The team lead: *"I want the main to be a LOT cleaner and human looking through our library …
extremely user friendly."* Today `src/main.cpp` is 1,338 lines: two hand-wired object graphs
(`Robot` `:335-424`, invented X-drive; `tanklib::TankRobot` `:604-670`), a port-map string builder,
a refusal painter, a 22-line teleop loop (`runTeleopLoop`, `:452-473`), a ~95-line panel
(`TankPanel`, `:803-898`), a 77-line boot-source block inside `initialize()` (`:982-1058`), and an
`#if` lattice over six builds. G1 Part 1 built the replacement for the graphs (`robot::Robot` from
a `config::RobotConfig`); T2 built the replacement for the raw stick mapping (`teleop::DriveShaper`).
Neither is used by any program. This chunk uses them.

**The measure, at the end of Part 2c:** `src/main.cpp` is a DISPATCHER — the variant checks, the
beacon, the precondition handler, and the five PROS entry points each forwarding to the selected
program — and "shulib Teleop" lives in its own file whose core a student can read top to bottom:

```text
ProsDeviceProvider devices;                         // the adapters, made on demand, by port
BootChoice boot = bootConfig();                     // the card's record, else the typed table; the absent ports
robot::Robot robot{boot.config, devices, {boot.absentPorts}};   // the whole library graph, wired
teleop::runDriverControl(robot.chassis(), master, devices.pacer(), shaper, loopCfg, disabled, heading, panel);
```

Target sizes, stated so "cleaner" is a number: `main.cpp` ≤ 300 lines; the teleop program's
composition (everything except its panel/refusal painting) ≤ 120 lines. Each part's executor
reports before/after line counts per file.

---

## 1. What exists to build on (read these)

| Piece | Where | What matters |
|---|---|---|
| The builder | `include/shulib/robot/robot.hpp` | `validate(cfg, devices) → BuildVerdict{ok, why[192]}`, three passes; `Robot{cfg, devices}` (non-movable; **its first member initializer re-runs `validate()`**; `wireMotors()` preconditions every motor non-null); accessors `chassis()/localizer()/faults()/health()/context()/config()/describe()`; `effectiveTuning()` tells the tank stall check it has no independent source — the same truth `main.cpp`'s `tankChassisConfig()` tells |
| The provider seam | `include/shulib/hal/device_provider.hpp`, `hal/fake/fake_device_provider.hpp` | `motor(signedPort, cartridge)`, `imu(port)`, `gps(port, leverF, leverL)`, `rotation(port)`, `clock()`, `battery()`, `telemetry()`, `pacer()`, `lastRefusal()`; provider-OWNED devices; `nullptr` on refusal. **Line 26's claim that every adapter's constructor throws on a dead port is false** (revision note 2) |
| The device registry | `hal/device_registry.hpp` (`IDeviceRegistry::pluggedKind(port)`, `DeviceKind`), `hal/pros/device_registry.hpp`, `hal/fake/fake_device_registry.hpp`; the shim's `apix.h` | what is physically on each port, before anything is constructed — HA-138 carries its port-indexing belief; that its KIND answer is right for an IMU, a rotation sensor and a GPS is a belief the provider newly rests on (register it in 2a) |
| The boot source (R3d Part 2) | `src/main.cpp:982-1058` (inside `initialize()`), `calibration/boot_check.hpp` (`verifyAgainstRegistry` → `BootVerdict` with `decision`, `reason`, `isMissing(port)`; `chooseBootSource`), `calibration/calibration_record.hpp`, `src/chassis_table.hpp` (`tableToDescription`, `describeMissingForDescription`) | record → verify → table → verify → refuse naming both; **the policy (`teleop::evaluateDegradation`) is applied INSIDE `verifyAgainstRegistry`**; `Degraded` ⇒ the verdict's missing ports become ABSENT group members (`TankMotors`, `:554-596`); any adapter that throws ⇒ refuse (`:1060-1066`). **The typed table cannot describe robot two** (track width and ratio 0.0), so today Teleop boots only from a SAVED record |
| The adapters | `include/shulib/hal/pros/*.hpp` | `ProsMotor` reads back and throws; `ProsImu`/`ProsRotation` cannot throw; `ProsGps` defers on an empty port; `ProsTickPacer::kTickMs = 10`; `ProsImu::calibrate()` once |
| The teleop loop | `src/main.cpp:452-473` `runTeleopLoop` | disabled poll → `chassis.cancel()`; sticks read only while connected (LeftY, LeftX, RightX — **not RightY**); `mapSticksToChassisSpeeds(sticks, cfg.maxLinearSpeed, cfg.maxAngularSpeed)` with `cfg = chassis.motionConfig()`; `drive(Body)`; per-tick callback; `pace()` |
| The driver-feel layer (T2) | `teleop/driver_control.hpp` (`DriveShaper::shape(StickInput, double dtSeconds, double headingRadians, bool headingTrusted)`, `rezero(double)`, `state()`), `input_shaping.hpp` (`DriverProfile`), `drive_modes.hpp` (tank sticks reads `rightY`), `field_centric.hpp`, `stick_mapping.hpp` (`toChassisSpeeds(request, maxLinear, maxAngular)`) | the default profile reproduces yesterday's feel except for the continuous deadband; field-centric falls back on an untrusted heading and latches |
| Heading trust, spelled | `localization::Localizer::qualityClass()` (`Uninitialized` / `DeadReckon` / `Corrected` / `Degraded`), `hal::IImu::isReady()` via `robot.context().imu()` | "live" = not `Uninitialized` (motion.hpp's wait-for-live contract); `Degraded` has four causes, so the IMU's own readiness is the heading-specific signal |
| Buttons | `hal::ControllerButton`, `hal::ButtonEdge` (per consumer, HA-104) | the re-zero edge is the loop's own `ButtonEdge` |
| The other consumers of the raw mapping | `src/drive_program.cpp` (`mapSticks` → `tankSideVolts`), `src/bench_r3a.cpp` station 10 DRIVE | "shulib Drive" HAS been driven (2026-09-17, no log); the tester's DRIVE station has constructed motors on hardware but never commanded a non-zero voltage (R3b-PROGRESS) — ruling M4 |
| The panel, refusal screen, state screens | `src/main.cpp:723-762` (refusal), `:783-898` (`drawHeader`, `TankPanel`), `:257-285` (`competitionBits`, `benchStateScreen`) | PROS screen calls; they stay in `src/`, and move files |
| The precondition hook | `src/main.cpp:322-328` (`g_faults`, `robotPreconditionHandler`), `:913` (installed), `:1068` (`g_faults = &r.faults`) | anonymous namespace — a graph built in another file cannot reach it |
| The session header's port map | `diag/session_info.hpp` (`kMaxPortMapBytes`, "G1 generates" note at `:47`), `src/main.cpp:678-722` (`tankPortMapString`, bounded by hand for `-Wformat-truncation`) | the library owes the generator |
| Program entry functions | `src/bench_r3a.hpp` (`runR3a`), `src/drive_program.hpp` (`runDrive`), `src/calibrate_program.hpp` (`runCalibrate`) | **all in `namespace shulib::bench`** |
| The build axis and its gate | `Makefile:15-23` (WARNFLAGS rationale naming `robot()`/`portMapString()`), `Makefile:78-129`, `tools/src_build_gate.py` (six builds, beacons, `-Wformat-nonliteral`, `-Wformat-truncation`, the variant differential at `:88-111`, `:198-200`, `:441-456`, its self-test) | revision note 3 |
| The docs that describe the wiring | `README.md` (the "Wiring a `Chassis`…" paragraph), `docs/guide/08-your-first-routine.md` (§"the wiring"), `docs/guide/10-the-api.md` (the teleop-shaped loop), `docs/guide/07-getting-set-up.md` (`src/`), `docs/guide/14` | what each part updates |

---

## 2a. PART 2a — the library half (no `src/` change; host-proven)

### IN

1. **`include/shulib/hal/pros/device_provider.hpp` — `hal::pros::ProsDeviceProvider`**, an
   `IDeviceProvider` over the real adapters. Fixed storage, no heap: `std::optional<ProsMotor>` × 21
   by port, `std::optional<ProsImu>`, `std::optional<ProsGps>`, `std::optional<ProsRotation>` × 21,
   and the port-less `ProsClock`, `ProsBattery`, `ProsCharSink` + `diag::TermSink` (the telemetry),
   `ProsTickPacer`. It owns (or takes a reference to) an `IDeviceRegistry` — the real one on the
   brain, the fake in tests.
   - **Before constructing any port device, it asks the registry what is on that port.** A port
     whose `pluggedKind` is not the expected kind (including `None`) ⇒ `nullptr`, and
     `lastRefusal()` = `"port 13: nothing plugged in (expected a motor)"` / `"port 2: a Rotation
     sensor (expected an IMU)"`. A motor whose kind is right but whose adapter throws (the
     read-back) ⇒ `nullptr` and `"port 13: the motor rejected its configuration: <adapter's
     message>"`.
   - Construction ON FIRST REQUEST, each inside its own `try`; a second request for the same port
     returns the SAME device; a request with a different sign or cartridge for an already-made
     motor (or a different lever arm for the GPS) is a precondition violation — one port, one
     configuration.
   - `imu()` starts calibration exactly once, on first construction (so the program's explicit
     `r.imu.calibrate()` goes away in 2b).
   - `device_provider.hpp:26`'s false sentence is corrected to say what refuses a device now.
2. **Tests (`test/pros_device_provider_test.cpp`, through the PROS shim):** same port ⇒ same object;
   an EMPTY IMU port, an EMPTY rotation port, an EMPTY GPS port and a wrong-kind port each ⇒
   `nullptr` + a refusal naming the port and what was found; a motor whose read-back fails ⇒
   `nullptr` + the adapter's message; a sign/cartridge conflict ⇒ precondition; `imu()` calibrates
   once across repeated requests.
3. **Absent drive ports in the builder (ruling M1).** `robot::BuildOptions{ std::span<const int>
   absentDrivePorts = {} }` (port MAGNITUDES — exactly what `BootVerdict::isMissing` answers), and
   `validate(cfg, devices, options = {})` / `Robot{cfg, devices, options = {}}` — **the options reach
   `prechecked()` and `wireMotors()`**. An absent port is never requested from the provider and
   becomes a `nullptr` member of its side's `MotorGroup` (R3d Part 2's ABSENT member). New
   refusals, each naming the port: an absent port that is not one of the description's drive ports;
   a non-empty list on a non-tank drivetrain; a side with ZERO present members. Everything else is
   unchanged — any requested device the provider refuses still refuses the build. `describe()`
   names the absent ports. The builder applies NO policy of its own: the boot check's verdict is the
   policy's one application.
4. **`Robot::portMap(char* out, std::size_t cap)`** — the session header's port-map text, generated
   from the config (the "G1 generates" promise, `session_info.hpp:47`): a tank's two sides' signed
   ports as stored and the IMU (the format `tankPortMapString()` prints today), an X-drive's wheel
   ports + IMU + GPS/tracking wheels. Bounded by hand (no moving `snprintf` —
   `-Wformat-truncation`), always NUL-terminated; a 21-port side truncates rather than overflows.
   And **`Robot::driveOdometry()`** → `const localization::DriveEncoderOdometry*` (`nullptr` on a
   tracking-wheel robot) so the panel's cross-check row survives the move.
5. **`include/shulib/teleop/teleop_loop.hpp` — `teleop::runDriverControl(...)`**, PROS-free:
   `(chassis::Chassis&, const hal::IController&, motion::ITickPacer&, DriveShaper&,
   const DriverLoopConfig&, IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick)`,
   where `DriverLoopConfig{ units::Time tickPeriod; hal::ControllerButton rezeroButton; }` and
   `HeadingSource` returns `{double radians; bool trusted;}`. Per tick: `disabled()` ⇒
   `chassis.cancel()` and return; read ALL FOUR axes only while connected (HA-103; `rightY` is what
   tank-sticks mode reads); `shape(sticks, tickPeriod, heading.radians, heading.trusted)`;
   `toChassisSpeeds(request, cfg.maxLinearSpeed, cfg.maxAngularSpeed)` with
   `cfg = chassis.motionConfig()`; `drive(Body)`; the loop's OWN `ButtonEdge` on `rezeroButton` calls
   `shaper.rezero(heading.radians)` once per press; `perTick(command, connected, shaper.state())`;
   `pace()`. The program supplies `tickPeriod` (from `ProsTickPacer::kTickMs`) and the heading
   source (`trusted = localizer.qualityClass() != Uninitialized && context().imu().isReady()`).
6. **Tests:** see §4, tests 1–5.

### Documentation, in Part 2a's commit
`docs/api/` + `mkdocs.yml` (generated); `docs/guide/08-your-first-routine.md` — "the wiring" gains
the two-line builder path (`robot::validate` + `robot::Robot` over a provider), its example COMPILED
in `test/guide_examples_test.cpp` and quoted verbatim, the longhand kept as Tier 3;
`docs/guide/10-the-api.md` — the teleop-shaped loop points at `runDriverControl`;
`docs/changelog.md` — the additions (no frozen surface grows, so no bump; `IDeviceProvider` and
`Robot` are unfrozen); `docs/roadmap.md` — the G1 row and "You are here"; `docs/internal/build-order.md`
— G1's status and the first `Next:`; `docs/hardware-assumptions.md` — the new belief the provider
rests on (the registry's kind answer is right for an IMU, a rotation sensor and a GPS; HA-138 states
only the indexing); the briefing
(regenerated). **Not touched, with why:** `src/` and everything that describes it (2b/2c); the master
plan (the WHY document; nothing here changes a decision in it); guide ch. 14 (Teleop is not re-pointed
until 2b).

---

## 2b. PART 2b — "shulib Teleop" rebuilt on the library (`src/`; `main.cpp` shrinks, still has the X-drive)

### IN

1. **`src/teleop_program.cpp` / `.hpp`** (in `namespace shulib::bench`, like the other programs;
   entry functions `teleopInitialize()` and `runTeleop()`):
   - `bootConfig()` — the existing record → verify → table → refuse block, MOVED (not rewritten)
     **into a shared `src/boot_source.cpp`/`.hpp`** (amended 2026-09-24, proposed with the COMP brief: the
     competition program boots exactly as Teleop does and must call the same function, not a copy),
     returning `{config::RobotConfig config; std::array<int, …> absentPorts; std::size_t
     absentCount; …the words the banner prints}`. Its pure core (description + verdict →
     `RobotConfig`: the tank slice is the chosen `DrivetrainDescription`, drive-encoder odometry,
     GPS absent, the provisional tuning) is a library-free function host-tested in test 6.
   - `ProsDeviceProvider` → `robot::validate(config, devices, {absentPorts})` ⇒ the refusal screen
     naming the port/field, or the `Robot` constructed **once, from `initialize()`, into
     `std::optional<robot::Robot>` storage** — never a function-local static that a later call would
     re-attempt after a throw (landmine 1). The graph-boundary `try` stays: a throwing precondition
     inside construction is a refusal screen, never a crash loop.
   - The precondition hook: `main.cpp`'s handler gains `attachFaultLatch(diag::FaultLatch*)`,
     declared in a small `src/` header and called once the `Robot` exists, so a precondition after
     boot still latches `FaultCode::Precondition` exactly as `:1068` does today.
   - The session header's port map from `robot.portMap()`; the banner from `robot.describe()`.
   - `runDriverControl` with the default `DriverProfile` (`ArcadeBody`, ruling M5), the re-zero
     button a named constant, field-centric a constant that is OFF.
   - The panel, MOVED: its group rows via `robot.chassis().deps().motorGroups` (the Tier-3 path is
     fine in `src/`), its cross-check row via `robot.driveOdometry()`, its estimate row via
     `robot.localizer()`; plus ONE new row: the shaper's mode word and the field-centric state
     (`fellBack`/latched), from `DriveShaperState`. The refusal painter, MOVED.
   - **Every line the current program says at boot that is still true is still said**: UNRUN;
     provisional gains; no independent stall source; the source word MEASURED/TYPED; the
     competition status; the build stamp. The old and new banners are diffed in the log.
2. **`src/main.cpp`**: the tank-library branches of `initialize()` and `opcontrol()` forward to
   `teleopInitialize()`/`runTeleop()`; `TankMotors`, `TankRobot`, `tankPortMapString`,
   `runTeleopLoop`, the panel and the boot block leave `main.cpp`. **The X-drive graph and its
   helpers stay in Part 2b**, so the gate's variant differential still holds unchanged.
3. The `src` build gate stays at six builds, all green, beacons unchanged, zero disallowed warnings.

### Documentation, in Part 2b's commit
`README.md` — the "Wiring a `Chassis`" paragraph (the builder is the on-ramp;
`src/teleop_program.cpp` the worked example; the longhand kept as Tier 3); `docs/guide/14` — the
driver-control bullet (the shaper now drives Teleop; no driver has felt it);
`docs/hardware-assumptions.md` — HA-112's source lines (Teleop consumes the feel layer);
`docs/internal/chunks/R3a-BENCH-WORKSHEET.md` — **Station F.1's panel table rewritten from the new
code** (it is already stale against the committed panel: "5 members" vs "%u/%u present", the
removed "dead ports: NOT tolerated" row, the geometry row) and Station G's Teleop steps checked
string-for-string against the new banner and panel; `docs/roadmap.md` — the two Phase T rows (still
`[~]`: Teleop wired, Drive and the tester not, no driver has felt it) and "You are here";
`docs/internal/build-order.md` — G1's and T2's status and the first `Next:`; the briefing. **If
Part 2b lands before the bench session, the S1 runbook's Teleop IS this program** — the worksheet
edit above is what keeps Station G true.

---

## 2c. PART 2c — `main.cpp` becomes a dispatcher; the X-drive becomes data; the gate follows

### IN

1. **The state screens move** — `competitionBits` and `benchStateScreen` → `src/state_screens.cpp` /
   `.hpp` — so `main.cpp` has no screen code at all.
2. **The X-drive variant as data:** the invented graph becomes a `config::RobotConfig` constant
   (`XDrive{…}`, tracking wheels, GPS) in its own `src/` file, carrying the HA-111 label in
   `identity.name` (`XDriveDrivetrain` has no provenance field) and built through the same
   `ProsDeviceProvider` + `Robot`. It still cannot boot on a real robot (its ports are invented)
   and still says so. Make it `constexpr` if `RobotConfig` allows it — prove that with a
   `static_assert` in a test — else `static const`, and say which in the log (no constexpr
   `RobotConfig` instance exists anywhere in the tree today).
3. **`src/main.cpp` is the dispatcher** (§0's measure): the variant/program `#if` checks, the beacon,
   `robotPreconditionHandler` + `attachFaultLatch`, and `initialize()`/`disabled()`/
   `competition_initialize()`/`autonomous()`/`opcontrol()` forwarding to `shulib::bench::` functions.
   No object graph, no panel, no boot logic, no screen code.
4. **The `src` build gate's variant differential is redesigned, deliberately.** With the X-drive
   helpers gone from `main.cpp`, "every tank build emits ≥ 1 `-Wunused-function` from
   `src/main.cpp`" can no longer hold. The detector it provided — a variant define passed on the
   command line but an `#ifdef` in `main.cpp` renamed or broken — is already covered more directly by
   the per-variant BEACON in the linked package, which the tool's own header calls the only detector
   that can tell the tank builds apart. So: remove the warning-count differential
   (`XDRIVE_WIRING_DEAD_IN`, the check at `:441-456`), rewrite the tool header's VARIANT DIFFERENTIAL
   section to say what replaced it and why, update the self-test, and rewrite `Makefile:15-23`'s
   WARNFLAGS rationale (`-Wunused-function` stays allowed by category). **Mutation:** break one
   `#ifdef` in `main.cpp`'s variant lattice and show the gate FAIL on the beacon; restore; green.
   Update `.github/workflows/ci.yml`'s comment if its wording changes.
5. `main.cpp` ≤ 300 lines; a grep in the log shows no `MotorGroup`, `Localizer`, `RobotContext`,
   `MotionDeps`, `screen_print`.

### Documentation, in Part 2c's commit
`README.md` — the tree layout's `src/` line and "src/main.cpp is the complete wiring example";
`docs/guide/07-getting-set-up.md` — its `src/` description; `docs/hardware-assumptions.md` —
HA-111's note (the invented map is now a config constant) and the "Port map, last season" row
(`:2299`), whose tail says a builder will replace hand-kept port lists; `docs/roadmap.md` — the M5 config bullet, the
port-map sentence ("G1's RobotBuilder generates it"), "You are here"; `docs/internal/build-order.md` —
G1 CLOSED, the first `Next:`; `docs/vexbuilder-integration.md` — the `IRobotConfig` / builder status
row (`:64`, "G1 Part 1 landed … Part 2 …"); **`G1-COMPLETED.md`**; the briefing. **Deferred, stated in the
commit:** the guide chapter on describing a robot as data (Part 1 handed it to Part 2) waits for G3,
when a robot can come from a FILE — today it would teach typing a `RobotConfig` in C++, the audience
the feature exists to avoid (G2 deferred its routine chapter for the same reason).

---

## 3. Rulings

**M1 — The builder takes the boot check's absent ports; it applies no policy of its own.** The
dead-port policy is one pure function (`teleop::evaluateDegradation`), applied once, inside
`calibration::verifyAgainstRegistry`, over the registry — before any adapter exists. The builder
honours that verdict's MISSING set as absent members and refuses on anything else the provider will
not make. Rejected: a `tolerateDeadPorts` flag that re-runs the policy over provider refusals (the
first version of this brief) — it double-reports, makes Teleop more tolerant of a present-but-wrong
motor, and refuses every robot with fewer than three motors a side. Rejected: the provider faking a
motor (a fake in a competition binary — R3b §6's rule).

**M2 — The registry check and the builder's device pass are TWO checks and both stay.** The boot
check runs BEFORE any adapter exists and catches a SWAPPED port and a MISSING one; the provider now
also refuses a wrong-kind port by the registry, and catches what the registry cannot (a motor that
rejects its configuration). A port the boot check reported missing is never requested, so it is
named once.

**M3 — The teleop loop is library code with its PROS dependencies injected.** The disabled
predicate, the tick period and the heading source are parameters, so `teleop/` stays PROS-free and
the loop is host-testable end to end. Rejected: leaving the loop in `src/`; adding a period to
`motion::ITickPacer` (it is part of the frozen `Chassis` constructor's signature — a new pure
virtual would break every implementer).

**M4 — Only "shulib Teleop" changes feel now.** It has never been driven, so there is no feel to
preserve. "shulib Drive" HAS been driven (2026-09-17); the tester's DRIVE station has never
commanded a non-zero voltage but is the measuring tool whose gates are written around the raw
mapping. Both keep the pinned `mapSticks` until a driver has compared the two on the robot (the T2
finish). The roadmap's T2 rows stay `[~]` and say exactly this.

**M5 — The default profile in Teleop is `ArcadeBody`.** Field-centric ships selectable (a constant in
the program, documented) but OFF: the first hardware run must not also be the first run of a frame
rotation. The re-zero button is wired regardless (harmless in a body mode).

---

## 4. Tests

| # | Part | Test | Property | Mutation |
|---|---|---|---|---|
| 1 | 2a | `ProsDeviceProvider` via the shim: same port ⇒ same object; empty IMU / rotation / GPS port and a wrong-kind port ⇒ `nullptr` + refusal naming port and kind; a motor read-back failure ⇒ `nullptr` + the adapter's message; sign/cartridge conflict ⇒ precondition; `imu()` calibrates once | the provider's contract | skip the registry check (an empty IMU port comes back live); return a fresh device per request; drop the conflict check |
| 2 | 2a | Builder with absent ports (fake provider): one port absent ⇒ builds, the group reports 4 of 5 present, `describe()` names the port, the absent port was NEVER requested from the provider (the fake counts requests); **`Robot{cfg, devices, opts}` constructs** (not only validates); an absent port not in the description ⇒ refusal; every port of a side absent ⇒ refusal; a list on an X-drive ⇒ refusal; default options ⇒ Part 1's behaviour exactly | M1 | drop the options in `prechecked()` (the constructor throws); request absent ports anyway |
| 3 | 2a | The degraded robot on the plant drives and reports; Part 1's bit-identity tests unchanged (assertion counts recorded before/after) | absence changes nothing when nothing is absent | — |
| 4 | 2a | `runDriverControl` on the plant with `FakeController`: a stick step at tick k is in the commanded volts of tick k; `disabled()` ⇒ `cancel()` (0 V + Brake at the motors) and return; disconnected ⇒ zero command whatever the axes; the re-zero edge calls `rezero` once per press; an untrusted heading ⇒ `state().fieldCentric.fellBack`; tank-sticks mode moves the RIGHT side from `RightY` | the loop's six properties | add a one-tick buffer; skip the cancel; poll axes while disconnected; never read `RightY` |
| 5 | 2a | `portMap()` for a tank and an X-drive, and a 21-port side truncated not overflowed; the default-profile loop vs the old mapping on a sweep: identical outside the deadband rescale (the documented bound), identical at 0 and ±1 | the header field is generated, bounded; Teleop's feel change is exactly the continuous deadband | drop the bound |
| 6 | 2b | `bootConfig()`'s pure core: a record description ⇒ config valid, `source` preserved, drive-encoder odometry, GPS absent; the verdict's missing ports carried through | the boot source feeds the builder faithfully | drop the sign copy (caught through the builder's signed-port assertion) |
| 7 | 2b/2c | Six builds green; beacons unchanged; old and new boot banners diffed (2b); `main.cpp` ≤ 300 lines with the grep above (2c); the gate's beacon catches a broken `#ifdef` (2c's mutation) | §0's measure is mechanical | 2c's `#ifdef` mutation |

Every mutation RUN on a fresh binary (edit → touch EVERY test that includes the edited header →
`--target shulib_tests/fast` → require the link line), observed red, logged, restored; the briefing
is regenerated only after the last restore is relinked green.

---

## 5. DoD (per part)

- [ ] **2a:** §2a items 1–6; every public member documented; ARM gate green; PROS-free and sim-free
      guards (the provider lives under `hal/pros/`; `teleop_loop.hpp` includes no `pros/`); tests 1–5
      with their mutations logged red; the pre-existing suite unchanged except stated additions;
      `src/` byte-identical; §2a's documentation; the prose review (below); the log.
- [ ] **2b:** §2b items 1–3; test 6; the banner diff; six builds, beacons unchanged; "shulib Teleop"
      still says UNRUN everywhere it did, no hardware claim; §2b's documentation; the prose review; the
      log.
- [ ] **2c:** §2c items 1–5; the gate redesign with its mutation; line counts before/after; §2c's
      documentation; `G1-COMPLETED.md`; the prose review; the log.

**Prose review, logged per part:** grep `docs/`, `README.md`, **`include/` and `src/` comments**
(the audit found stale "Part 2" sentences in `robot.hpp:13-16, :55-56` and `device_provider.hpp:26`)
for `main.cpp`, `TankRobot`, `runTeleopLoop`, `composition root`, `wiring`, `hand-wired`,
`object graph`, `mapSticksToChassisSpeeds`, `deadband-only`, `Part 2`; read every hit; fix what is
stale. Run the grep BEFORE writing that it was run.

---

## 6. Landmines

1. **`Robot` and the provider are non-movable**, construction order across translation units is
   undefined, and **a function-local static whose constructor throws is re-attempted on the next
   call** — build them ONCE, from `initialize()`, into `std::optional` storage that lives for the
   program; never at namespace scope and never lazily from `opcontrol()`.
2. **`-Wformat-nonliteral` and `-Wformat-truncation`** are on in `src/` (both have bitten); bound
   string building by hand where the compiler cannot bound it.
3. **LVGL fonts are ASCII-only**: no em dashes or arrows in any runtime string.
4. **The precondition handler must be installed before the first adapter is constructed**, and the
   graph boundary `try` stays.
5. **One sign, one place**: the description's signed port reaches `ProsMotor` as stored.
6. **The session header's port map comes from `robot.portMap()`**, never retyped and never from
   `describe()` (whose banner is longer than the 96-byte field).
7. **Do not change `stick_mapping.hpp`'s bodies or `stick_mapping_test.cpp`**; the drive program's pin
   depends on them (M4).
8. **No hardware claim.** Teleop remains UNRUN; say so where the old program did.
9. **Parts 2b and 2c change the packages the first drive uses.** If the brain enumerates while one is
   in flight, S1 builds its packages from the last COMMITTED tree in a separate worktree
   (`R3a-BENCH-WORKSHEET.md` G.1), never from the executor's working tree.

---

## 7. Verification (the coordinator, per part)

The command set in `HANDOFF-remaining-work.md` §3 (there is no tracked `verify.sh`); read the part's
load-bearing code in full (2a: the provider's registry check and `prechecked()`/`wireMotors()` with
options; 2b: `teleop_program.cpp`'s composition and the moved boot block against the old one; 2c:
`main.cpp` whole — it must be readable in one sitting — and the gate's new header); re-run by hand
2a's "skip the registry check" and "drop the options in `prechecked()`" mutations, 2b's banner diff,
2c's `#ifdef` mutation; after 2b and 2c, rebuild the four hardware packages from the committed tree
and confirm the beacons.
